-- Tomasulo's algorithm with a reorder buffer (ROB), Chapter 3.
--
--   issue (in order) -> reservation station -> execute (out of order) -> CDB broadcast -> commit (in order)
--
-- ISA: three-register ops on NREG registers:  0 ADD  rd = rs + rt    1 SUB  rd = rs - rt    2 MUL  rd = rs * rt
-- Resources: NADD add/sub reservation stations sharing ONE non-pipelined add unit (ADD_LAT cycles),
--            NMUL mul reservation stations sharing ONE non-pipelined multiplier (MUL_LAT cycles),
--            a ROBN-entry reorder buffer, and ONE common data bus (CDB): one result broadcast per cycle
--            (add unit wins a tie; the multiplier holds its result until the CDB is free).
--
-- Exact timing, counting clock edges (edge 1 = the edge at which instruction 0 is accepted):
--   issue edge I     : ROB entry + reservation station allocated, sources read or tagged, rd renamed to the ROB tag
--   edge I+1 (earliest): execution starts (needs both operands in the station as of the previous edge)
--   edge S+LAT       : result computed ("done")                  -> unit holds it until it wins the CDB
--   next edge        : CDB broadcast: ROB entry done, waiting stations capture the value, station + unit freed
--   next edge        : commit (head of ROB only): architectural register written, ROB entry freed
-- So an independent ADD issued at edge I commits at I+5; an independent MUL at I+7.
-- A freed station/ROB entry is reusable one cycle later (issue_ready looks at the state before the edge).
-- No branches or memory: speculation past branches is left as an exercise (the ROB is what would make it possible).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tomasulo_rob is
  generic (
    NREG    : positive := 8;
    ROBN    : positive := 8;
    NADD    : positive := 3;
    NMUL    : positive := 2;
    ADD_LAT : positive := 2;
    MUL_LAT : positive := 4;
    DW      : positive := 32
  );
  port (
    clk, rst_n   : in  std_logic;
    -- initialise architectural registers (use only while the machine is empty)
    init_we      : in  std_logic;
    init_reg     : in  natural range 0 to NREG-1;
    init_data    : in  std_logic_vector(DW-1 downto 0);
    -- instruction issue: accepted at a clock edge when issue_valid and issue_ready
    issue_valid  : in  std_logic;
    issue_op     : in  natural range 0 to 3;
    issue_rd, issue_rs, issue_rt : in natural range 0 to NREG-1;
    issue_ready  : out std_logic;
    -- observation
    cdb_valid    : out std_logic;                       -- a broadcast will happen at the next edge
    cdb_tag      : out natural range 0 to ROBN-1;
    commit_valid : out std_logic;                       -- registered: an instruction committed at the previous edge
    commit_reg   : out natural range 0 to NREG-1;
    commit_data  : out std_logic_vector(DW-1 downto 0);
    empty        : out std_logic;                       -- nothing in flight
    dbg_raddr    : in  natural range 0 to NREG-1;
    dbg_rdata    : out std_logic_vector(DW-1 downto 0)
  );
end entity;

architecture rtl of tomasulo_rob is
  constant NRS : positive := NADD + NMUL;
  type data_t is array (natural range <>) of std_logic_vector(DW-1 downto 0);
  type int_t  is array (natural range <>) of natural;

  -- architectural state / renaming
  signal regfile    : data_t(0 to NREG-1) := (others => (others => '0'));
  signal regst_busy : std_logic_vector(0 to NREG-1) := (others => '0');
  signal regst_tag  : int_t(0 to NREG-1) := (others => 0);
  -- reorder buffer
  signal rob_done : std_logic_vector(0 to ROBN-1) := (others => '0');
  signal rob_reg  : int_t(0 to ROBN-1) := (others => 0);
  signal rob_val  : data_t(0 to ROBN-1) := (others => (others => '0'));
  signal head, tail : natural range 0 to ROBN-1 := 0;
  signal count : natural range 0 to ROBN := 0;
  -- reservation stations (0..NADD-1 add type, NADD..NRS-1 mul type)
  signal rs_busy, rs_exec, rs_qjw, rs_qkw : std_logic_vector(0 to NRS-1) := (others => '0');
  signal rs_op, rs_qjt, rs_qkt, rs_dest : int_t(0 to NRS-1) := (others => 0);
  signal rs_vj, rs_vk : data_t(0 to NRS-1) := (others => (others => '0'));
  -- functional units: 0 = add/sub, 1 = multiplier
  signal u_busy, u_done : std_logic_vector(0 to 1) := (others => '0');
  signal u_rs, u_rem : int_t(0 to 1) := (others => 0);
  signal u_res : data_t(0 to 1) := (others => (others => '0'));

  signal cdb_v : std_logic;
  signal cdb_t, cdb_u : natural range 0 to ROBN;
  signal cdb_val : std_logic_vector(DW-1 downto 0);
  signal free_rs : integer range -1 to NRS-1;
  signal src_val  : data_t(0 to 1);
  signal src_wait : std_logic_vector(0 to 1);
  signal src_tag  : int_t(0 to 1);
  signal ready_i  : std_logic;
  signal commit_valid_i : std_logic := '0';

  function alu(op : natural; a, b : std_logic_vector) return std_logic_vector is
  begin
    case op is
      when 0      => return std_logic_vector(unsigned(a) + unsigned(b));
      when 1      => return std_logic_vector(unsigned(a) - unsigned(b));
      when others => return std_logic_vector(resize(unsigned(a) * unsigned(b), DW));
    end case;
  end function;
begin
  dbg_rdata    <= regfile(dbg_raddr);
  empty        <= '1' when count = 0 else '0';
  issue_ready  <= ready_i;
  cdb_valid    <= cdb_v;
  cdb_tag      <= cdb_t when cdb_t < ROBN else 0;
  commit_valid <= commit_valid_i;

  -- ---- common data bus arbitration -------------------------------------------------------------
  cdb_arb : process (all)
    variable usel : natural range 0 to 1;
  begin
    cdb_v <= '0';  cdb_u <= 0;  cdb_t <= 0;  cdb_val <= (others => '0');
    usel := 0;
    if u_done(0) = '1' then cdb_v <= '1'; usel := 0;
    elsif u_done(1) = '1' then cdb_v <= '1'; usel := 1;
    end if;
    cdb_u <= usel;
    if u_done(0) = '1' or u_done(1) = '1' then
      cdb_val <= u_res(usel);
      cdb_t   <= rs_dest(u_rs(usel));
    end if;
  end process;

  -- ---- issue: free station, operand lookup --------------------------------------------------------
  issue_lookup : process (all)
    variable fr : integer range -1 to NRS-1;
    variable r  : natural range 0 to NREG-1;
    variable tg : natural range 0 to ROBN-1;
  begin
    fr := -1;
    for i in 0 to NRS-1 loop
      if rs_busy(i) = '0' and fr < 0 and ((i >= NADD) = (issue_op = 2)) then fr := i; end if;
    end loop;
    free_rs <= fr;
    if count < ROBN and fr >= 0 then ready_i <= '1'; else ready_i <= '0'; end if;

    for s in 0 to 1 loop
      if s = 0 then r := issue_rs; else r := issue_rt; end if;
      src_val(s) <= regfile(r);  src_wait(s) <= '0';  src_tag(s) <= 0;
      if regst_busy(r) = '1' then
        tg := regst_tag(r);
        src_tag(s) <= tg;
        if cdb_v = '1' and cdb_t = tg then src_val(s) <= cdb_val;           -- caught off the CDB
        elsif rob_done(tg) = '1' then src_val(s) <= rob_val(tg);            -- already in the ROB
        else src_wait(s) <= '1';                                            -- wait for the broadcast
        end if;
      end if;
    end loop;
  end process;

  -- ---- the clocked machine ----------------------------------------------------------------------
  seq : process (clk, rst_n)
    variable pick : integer;
    variable lo, hi : natural;
    variable do_issue, do_commit : boolean;
    variable nxt_count : integer;
    variable cu : natural range 0 to 1;
  begin
    if rst_n = '0' then
      head <= 0;  tail <= 0;  count <= 0;  commit_valid_i <= '0';  commit_reg <= 0;  commit_data <= (others => '0');
      regfile <= (others => (others => '0'));  regst_busy <= (others => '0');  regst_tag <= (others => 0);
      rob_done <= (others => '0');  rob_reg <= (others => 0);  rob_val <= (others => (others => '0'));
      rs_busy <= (others => '0');  rs_exec <= (others => '0');  rs_qjw <= (others => '0');  rs_qkw <= (others => '0');
      rs_op <= (others => 0);  rs_qjt <= (others => 0);  rs_qkt <= (others => 0);  rs_dest <= (others => 0);
      rs_vj <= (others => (others => '0'));  rs_vk <= (others => (others => '0'));
      u_busy <= (others => '0');  u_done <= (others => '0');  u_rs <= (others => 0);  u_rem <= (others => 0);
      u_res <= (others => (others => '0'));
    elsif rising_edge(clk) then
      do_issue  := issue_valid = '1' and ready_i = '1';
      do_commit := count /= 0 and rob_done(head) = '1';
      cu := cdb_u;
      if init_we = '1' and count = 0 then regfile(init_reg) <= init_data; end if;

      -- 1. CDB broadcast: ROB entry completes, waiting stations capture the value
      if cdb_v = '1' then
        rob_val(cdb_t)  <= cdb_val;
        rob_done(cdb_t) <= '1';
        for i in 0 to NRS-1 loop
          if rs_busy(i) = '1' and rs_qjw(i) = '1' and rs_qjt(i) = cdb_t then rs_vj(i) <= cdb_val; rs_qjw(i) <= '0'; end if;
          if rs_busy(i) = '1' and rs_qkw(i) = '1' and rs_qkt(i) = cdb_t then rs_vk(i) <= cdb_val; rs_qkw(i) <= '0'; end if;
        end loop;
      end if;

      -- 2. functional units
      for u in 0 to 1 loop
        if u = 0 then lo := 0; hi := NADD - 1; else lo := NADD; hi := NRS - 1; end if;
        if u_busy(u) = '0' then
          pick := -1;
          for i in lo to hi loop
            if pick < 0 and rs_busy(i) = '1' and rs_exec(i) = '0' and rs_qjw(i) = '0' and rs_qkw(i) = '0' then pick := i; end if;
          end loop;
          if pick >= 0 then
            u_busy(u) <= '1';  u_done(u) <= '0';  u_rs(u) <= pick;
            if u = 0 then u_rem(u) <= ADD_LAT; else u_rem(u) <= MUL_LAT; end if;
            rs_exec(pick) <= '1';
          end if;
        elsif u_done(u) = '0' then
          if u_rem(u) = 1 then
            u_done(u) <= '1';  u_rem(u) <= 0;
            u_res(u)  <= alu(rs_op(u_rs(u)), rs_vj(u_rs(u)), rs_vk(u_rs(u)));
          else
            u_rem(u) <= u_rem(u) - 1;
          end if;
        elsif cdb_v = '1' and cu = u then                 -- result broadcast this edge: free everything
          u_busy(u) <= '0';  u_done(u) <= '0';
          rs_busy(u_rs(u)) <= '0';  rs_exec(u_rs(u)) <= '0';
        end if;
      end loop;

      -- 3. commit the head of the ROB, in program order
      if do_commit then commit_valid_i <= '1'; else commit_valid_i <= '0'; end if;
      if do_commit then
        regfile(rob_reg(head)) <= rob_val(head);
        commit_reg  <= rob_reg(head);
        commit_data <= rob_val(head);
        if regst_busy(rob_reg(head)) = '1' and regst_tag(rob_reg(head)) = head then regst_busy(rob_reg(head)) <= '0'; end if;
        rob_done(head) <= '0';
        if head = ROBN - 1 then head <= 0; else head <= head + 1; end if;
      end if;

      -- 4. issue (last, so a new rename overrides a commit-time clear of the same register)
      if do_issue then
        rob_done(tail) <= '0';
        rob_reg(tail)  <= issue_rd;
        if tail = ROBN - 1 then tail <= 0; else tail <= tail + 1; end if;
        regst_busy(issue_rd) <= '1';
        regst_tag(issue_rd)  <= tail;
        rs_busy(free_rs) <= '1';  rs_exec(free_rs) <= '0';  rs_op(free_rs) <= issue_op;  rs_dest(free_rs) <= tail;
        rs_vj(free_rs) <= src_val(0);  rs_qjw(free_rs) <= src_wait(0);  rs_qjt(free_rs) <= src_tag(0);
        rs_vk(free_rs) <= src_val(1);  rs_qkw(free_rs) <= src_wait(1);  rs_qkt(free_rs) <= src_tag(1);
      end if;
      nxt_count := count;
      if do_issue  then nxt_count := nxt_count + 1; end if;
      if do_commit then nxt_count := nxt_count - 1; end if;
      count <= nxt_count;
    end if;
  end process;
end architecture;
