-- Tomasulo with hardware speculation past branches (Chapter 3): reservation stations + CDB + reorder buffer, now with a fetch
-- unit and a branch instruction.  See ../tomasulo_rob for the non-speculative core and its exact timing rules; they are unchanged.
--
-- ISA (32-bit words, 8 registers):   op(31:28) | rd(27:24) | rs(23:20) | rt(19:16) | target(15:0)
--   0 ADD rd=rs+rt   1 SUB rd=rs-rt   2 MUL rd=rs*rt   3 BNEZ rs,target (branch if rs /= 0)   4 HALT
-- Speculation:
--   * Every BNEZ is predicted when it issues: PREDICT_BTFN = false -> always not taken; true -> backward (target < pc) taken, forward not taken.
--     Fetch continues down the predicted path, so younger instructions issue, execute and even broadcast speculatively.
--   * A branch occupies a ROB entry and an ADD-type reservation station (the add unit forwards its source: nonzero = taken).
--   * Nothing speculative ever reaches the architectural registers: results are written only at commit, in order.
--   * Recovery happens at COMMIT: when the branch is at the head of the ROB and its outcome disagrees with its prediction, the whole
--     ROB, all stations, both units and the rename table are cleared (everything left in the machine is younger and wrong) and fetch
--     restarts at the correct address.  Nothing issues on the flush edge, so the next instruction issues one edge later.
--   * HALT stops the front end; it commits like any other instruction and then sets `halted`.  A wrong-path HALT is simply flushed.
--   * Each functional unit starts the OLDEST ready station (smallest distance from the ROB head): with speculation, younger wrong-path
--     work must not steal a unit from the older instructions (often the branch) that decide whether it was needed.
-- Statistics: stat_cycles (edges while running), stat_committed, stat_branches (committed BNEZ), stat_flushes (mispredictions),
--             stat_squashed (instructions thrown away by flushes).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tomasulo_spec is
  generic (
    NREG : positive := 8;  ROBN : positive := 8;  NADD : positive := 3;  NMUL : positive := 2;
    ADD_LAT : positive := 2;  MUL_LAT : positive := 4;  PREDICT_BTFN : boolean := false;  DW : positive := 32
  );
  port (
    clk, rst_n     : in  std_logic;
    init_we        : in  std_logic;                       -- initialise registers while stopped and empty
    init_reg       : in  natural range 0 to NREG-1;
    init_data      : in  std_logic_vector(DW-1 downto 0);
    start          : in  std_logic;                       -- begin fetching at pc = 0 (first issue at the next edge)
    imem_addr      : out std_logic_vector(7 downto 0);
    imem_rdata     : in  std_logic_vector(31 downto 0);
    halted         : out std_logic;
    stat_cycles    : out unsigned(31 downto 0);
    stat_committed : out unsigned(31 downto 0);
    stat_branches  : out unsigned(31 downto 0);
    stat_flushes   : out unsigned(31 downto 0);
    stat_squashed  : out unsigned(31 downto 0);
    dbg_raddr      : in  natural range 0 to NREG-1;
    dbg_rdata      : out std_logic_vector(DW-1 downto 0)
  );
end entity;

architecture rtl of tomasulo_spec is
  constant NRS : positive := NADD + NMUL;
  type data_t is array (natural range <>) of std_logic_vector(DW-1 downto 0);
  type int_t  is array (natural range <>) of natural;

  signal regfile    : data_t(0 to NREG-1) := (others => (others => '0'));
  signal regst_busy : std_logic_vector(0 to NREG-1) := (others => '0');
  signal regst_tag  : int_t(0 to NREG-1) := (others => 0);
  signal rob_done, rob_wr, rob_br, rob_halt, rob_pred : std_logic_vector(0 to ROBN-1) := (others => '0');
  signal rob_tgt, rob_ft, rob_reg : int_t(0 to ROBN-1) := (others => 0);
  signal rob_val  : data_t(0 to ROBN-1) := (others => (others => '0'));
  signal head, tail : natural range 0 to ROBN-1 := 0;
  signal count : natural range 0 to ROBN := 0;
  signal rs_busy, rs_exec, rs_qjw, rs_qkw : std_logic_vector(0 to NRS-1) := (others => '0');
  signal rs_op, rs_qjt, rs_qkt, rs_dest : int_t(0 to NRS-1) := (others => 0);
  signal rs_vj, rs_vk : data_t(0 to NRS-1) := (others => (others => '0'));
  signal u_busy, u_done : std_logic_vector(0 to 1) := (others => '0');
  signal u_rs, u_rem : int_t(0 to 1) := (others => 0);
  signal u_res : data_t(0 to 1) := (others => (others => '0'));
  signal pc : natural range 0 to 255 := 0;
  signal running, fetch_stop, halted_i : std_logic := '0';
  signal cycles, committed, branches, flushes, squashed : unsigned(31 downto 0) := (others => '0');

  signal cdb_v : std_logic;
  signal cdb_t, cdb_u : natural range 0 to ROBN;
  signal cdb_val : std_logic_vector(DW-1 downto 0);
  signal free_rs : integer range -1 to NRS-1;
  signal src_val  : data_t(0 to 1);
  signal src_wait : std_logic_vector(0 to 1);
  signal src_tag  : int_t(0 to 1);
  signal issue_ok : std_logic;
  signal i_op, i_rd, i_rs, i_rt, i_tgt : natural;
  signal do_commit, flush_now, do_issue, br_actual, pred_taken : boolean;

  function alu(op : natural; a, b : std_logic_vector) return std_logic_vector is
  begin
    case op is
      when 0      => return std_logic_vector(unsigned(a) + unsigned(b));
      when 1      => return std_logic_vector(unsigned(a) - unsigned(b));
      when 2      => return std_logic_vector(resize(unsigned(a) * unsigned(b), DW));
      when others => return a;                                  -- branch: the condition value
    end case;
  end function;
begin
  dbg_rdata <= regfile(dbg_raddr);
  imem_addr <= std_logic_vector(to_unsigned(pc, 8));
  halted <= halted_i;
  stat_cycles <= cycles;  stat_committed <= committed;  stat_branches <= branches;  stat_flushes <= flushes;  stat_squashed <= squashed;

  -- ---- common data bus ---------------------------------------------------------------------------
  cdb_arb : process (all)
    variable usel : natural range 0 to 1;
  begin
    cdb_v <= '0';  cdb_u <= 0;  cdb_t <= 0;  cdb_val <= (others => '0');
    usel := 0;
    if u_done(0) = '1' then cdb_v <= '1'; usel := 0;
    elsif u_done(1) = '1' then cdb_v <= '1'; usel := 1;
    end if;
    cdb_u <= usel;
    if u_done(0) = '1' or u_done(1) = '1' then cdb_val <= u_res(usel); cdb_t <= rs_dest(u_rs(usel)); end if;
  end process;

  -- ---- fetch/decode of the instruction at pc, free station, operand lookup ------------------------------
  i_op  <= to_integer(unsigned(imem_rdata(31 downto 28)));
  i_rd  <= to_integer(unsigned(imem_rdata(26 downto 24)));
  i_rs  <= to_integer(unsigned(imem_rdata(22 downto 20)));
  i_rt  <= to_integer(unsigned(imem_rdata(18 downto 16)));
  i_tgt <= to_integer(unsigned(imem_rdata(7 downto 0)));

  issue_lookup : process (all)
    variable fr : integer range -1 to NRS-1;
    variable r  : natural range 0 to NREG-1;
    variable tg : natural range 0 to ROBN-1;
    variable alu_i, br_i, halt_i, mul_i, ok : boolean;
  begin
    alu_i := i_op <= 2;  br_i := i_op = 3;  halt_i := i_op = 4;  mul_i := i_op = 2;
    fr := -1;
    for i in 0 to NRS-1 loop
      if rs_busy(i) = '0' and fr < 0 and ((i >= NADD) = mul_i) then fr := i; end if;
    end loop;
    free_rs <= fr;
    ok := running = '1' and halted_i = '0' and fetch_stop = '0' and count < ROBN and (halt_i or ((alu_i or br_i) and fr >= 0));
    if ok then issue_ok <= '1'; else issue_ok <= '0'; end if;

    for s in 0 to 1 loop
      if s = 0 then r := i_rs; else r := i_rt; end if;
      src_val(s) <= regfile(r);  src_wait(s) <= '0';  src_tag(s) <= 0;
      if regst_busy(r) = '1' then
        tg := regst_tag(r);
        src_tag(s) <= tg;
        if cdb_v = '1' and cdb_t = tg then src_val(s) <= cdb_val;
        elsif rob_done(tg) = '1' then src_val(s) <= rob_val(tg);
        else src_wait(s) <= '1';
        end if;
      end if;
      if s = 1 and br_i then src_val(1) <= (others => '0'); src_wait(1) <= '0'; end if;   -- a branch has only one source
    end loop;
  end process;

  -- ---- commit / misprediction detection ----------------------------------------------------------------
  do_commit  <= count /= 0 and rob_done(head) = '1';
  br_actual  <= unsigned(rob_val(head)) /= 0;
  flush_now  <= do_commit and rob_br(head) = '1' and (br_actual /= (rob_pred(head) = '1'));
  do_issue   <= issue_ok = '1' and not flush_now;
  pred_taken <= PREDICT_BTFN and i_op = 3 and i_tgt < pc;

  seq : process (clk, rst_n)
    variable pick : integer;
    variable lo, hi : natural;
    variable cu : natural range 0 to 1;
    variable nxt : integer;
    variable alu_i, br_i, halt_i : boolean;
    variable pbit : std_logic;
    variable age_i, age_p : natural;
  begin
    if rst_n = '0' then
      head <= 0;  tail <= 0;  count <= 0;  pc <= 0;  running <= '0';  fetch_stop <= '0';  halted_i <= '0';
      cycles <= (others => '0');  committed <= (others => '0');  branches <= (others => '0');
      flushes <= (others => '0');  squashed <= (others => '0');
      regfile <= (others => (others => '0'));  regst_busy <= (others => '0');  regst_tag <= (others => 0);
      rob_done <= (others => '0');  rob_wr <= (others => '0');  rob_br <= (others => '0');  rob_halt <= (others => '0');
      rob_pred <= (others => '0');  rob_tgt <= (others => 0);  rob_ft <= (others => 0);  rob_reg <= (others => 0);
      rob_val <= (others => (others => '0'));
      rs_busy <= (others => '0');  rs_exec <= (others => '0');  rs_qjw <= (others => '0');  rs_qkw <= (others => '0');
      rs_op <= (others => 0);  rs_qjt <= (others => 0);  rs_qkt <= (others => 0);  rs_dest <= (others => 0);
      rs_vj <= (others => (others => '0'));  rs_vk <= (others => (others => '0'));
      u_busy <= (others => '0');  u_done <= (others => '0');  u_rs <= (others => 0);  u_rem <= (others => 0);
      u_res <= (others => (others => '0'));
    elsif rising_edge(clk) then
      cu := cdb_u;
      alu_i := i_op <= 2;  br_i := i_op = 3;  halt_i := i_op = 4;
      if init_we = '1' and running = '0' and count = 0 then regfile(init_reg) <= init_data; end if;
      if start = '1' and running = '0' then running <= '1'; end if;
      if running = '1' and halted_i = '0' then cycles <= cycles + 1; end if;

      -- 1. CDB broadcast
      if cdb_v = '1' then
        rob_val(cdb_t)  <= cdb_val;
        rob_done(cdb_t) <= '1';
        for i in 0 to NRS-1 loop
          if rs_busy(i) = '1' and rs_qjw(i) = '1' and rs_qjt(i) = cdb_t then rs_vj(i) <= cdb_val; rs_qjw(i) <= '0'; end if;
          if rs_busy(i) = '1' and rs_qkw(i) = '1' and rs_qkt(i) = cdb_t then rs_vk(i) <= cdb_val; rs_qkw(i) <= '0'; end if;
        end loop;
      end if;

      -- 2. functional units (oldest ready station first)
      for u in 0 to 1 loop
        if u = 0 then lo := 0; hi := NADD - 1; else lo := NADD; hi := NRS - 1; end if;
        if u_busy(u) = '0' then
          pick := -1;
          for i in lo to hi loop
            if rs_busy(i) = '1' and rs_exec(i) = '0' and rs_qjw(i) = '0' and rs_qkw(i) = '0' then
              if pick < 0 then pick := i;
              else
                age_i := (rs_dest(i) + ROBN - head) mod ROBN;
                age_p := (rs_dest(pick) + ROBN - head) mod ROBN;
                if age_i < age_p then pick := i; end if;
              end if;
            end if;
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
        elsif cdb_v = '1' and cu = u then
          u_busy(u) <= '0';  u_done(u) <= '0';
          rs_busy(u_rs(u)) <= '0';  rs_exec(u_rs(u)) <= '0';
        end if;
      end loop;

      -- 3. commit the head of the ROB, in program order
      if do_commit then
        committed <= committed + 1;
        if rob_wr(head) = '1' then
          regfile(rob_reg(head)) <= rob_val(head);
          if regst_busy(rob_reg(head)) = '1' and regst_tag(rob_reg(head)) = head then regst_busy(rob_reg(head)) <= '0'; end if;
        end if;
        if rob_br(head) = '1' then branches <= branches + 1; end if;
        if rob_halt(head) = '1' then halted_i <= '1'; end if;
        rob_done(head) <= '0';
        if head = ROBN - 1 then head <= 0; else head <= head + 1; end if;
      end if;

      -- 4. issue: fetch the instruction at pc and follow the predicted path
      if do_issue then
        if halt_i then rob_done(tail) <= '1'; else rob_done(tail) <= '0'; end if;                 -- HALT needs no execution
        if alu_i then rob_wr(tail) <= '1'; else rob_wr(tail) <= '0'; end if;
        if br_i then rob_br(tail) <= '1'; else rob_br(tail) <= '0'; end if;
        if halt_i then rob_halt(tail) <= '1'; else rob_halt(tail) <= '0'; end if;
        if pred_taken then pbit := '1'; else pbit := '0'; end if;
        rob_pred(tail) <= pbit;  rob_tgt(tail) <= i_tgt;  rob_ft(tail) <= pc + 1;
        rob_reg(tail) <= i_rd;
        if tail = ROBN - 1 then tail <= 0; else tail <= tail + 1; end if;
        if alu_i then regst_busy(i_rd) <= '1'; regst_tag(i_rd) <= tail; end if;
        if alu_i or br_i then
          rs_busy(free_rs) <= '1';  rs_exec(free_rs) <= '0';  rs_dest(free_rs) <= tail;
          if br_i then rs_op(free_rs) <= 3; else rs_op(free_rs) <= i_op; end if;
          rs_vj(free_rs) <= src_val(0);  rs_qjw(free_rs) <= src_wait(0);  rs_qjt(free_rs) <= src_tag(0);
          rs_vk(free_rs) <= src_val(1);  rs_qkw(free_rs) <= src_wait(1);  rs_qkt(free_rs) <= src_tag(1);
        end if;
        if pred_taken then pc <= i_tgt; else pc <= pc + 1; end if;
        if halt_i then fetch_stop <= '1'; end if;
      end if;
      nxt := count;
      if do_issue  then nxt := nxt + 1; end if;
      if do_commit then nxt := nxt - 1; end if;
      count <= nxt;

      -- 5. misprediction: throw away everything younger than the branch (it is at the head, so that is everything else)
      if flush_now then
        flushes  <= flushes + 1;
        squashed <= squashed + (count - 1);
        if br_actual then pc <= rob_tgt(head); else pc <= rob_ft(head); end if;
        fetch_stop <= '0';
        head <= 0;  tail <= 0;  count <= 0;
        rob_done <= (others => '0');
        rs_busy <= (others => '0');  rs_exec <= (others => '0');  rs_qjw <= (others => '0');  rs_qkw <= (others => '0');
        u_busy <= (others => '0');  u_done <= (others => '0');
        regst_busy <= (others => '0');
      end if;
    end if;
  end process;
end architecture;
