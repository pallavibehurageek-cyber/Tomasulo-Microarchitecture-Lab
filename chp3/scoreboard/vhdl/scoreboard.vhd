-- The CDC 6600 scoreboard (Chapter 3 in the early editions; Appendix C of the later ones).
--
-- In-order issue, out-of-order execution, NO renaming and NO forwarding bus.  Every instruction goes through four steps:
--   Issue          : needs a free functional unit (structural) and no pending write to the same destination (WAW).
--   Read operands  : waits until BOTH source registers hold their final values (RAW); values are read from the register file.
--   Execute        : ADD_LAT (add/sub) or MUL_LAT (mul) cycles.
--   Write result   : waits until no earlier instruction still has to READ the old value of the destination (WAR).
-- Units: FU0 = add/sub, FU1 and FU2 = multipliers.  ISA: 0 ADD  1 SUB  2 MUL, three registers, rd = rs op rt.
--
-- Timing, counting clock edges (the edge at which an instruction is accepted is its issue edge I):
--   read  edge R >= I+1 and >= (the edge a producer wrote) + 1
--   execute occupies edges R+1 .. R+LAT, the result is ready after edge R+LAT
--   write edge W >= R+LAT+1, and W >= (the edge every older reader of rd read it) + 1
--   a unit and a destination register become reusable by the NEXT edge after W (issue looks at the state before the edge).
-- Observation outputs (combinational, describing what happens at the NEXT edge): read_go(f), write_go(f), fu_seq
-- (16 bits per unit: a sequence number identifying the instruction in unit f), so a testbench can reconstruct every timeline.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity scoreboard is
  generic (NREG : positive := 8; DW : positive := 32; ADD_LAT : positive := 2; MUL_LAT : positive := 4);
  port (
    clk, rst_n  : in  std_logic;
    init_we     : in  std_logic;                       -- initialise registers while the machine is empty
    init_reg    : in  natural range 0 to NREG-1;
    init_data   : in  std_logic_vector(DW-1 downto 0);
    issue_valid : in  std_logic;
    issue_op    : in  natural range 0 to 3;
    issue_rd, issue_rs, issue_rt : in natural range 0 to NREG-1;
    issue_ready : out std_logic;
    read_go     : out std_logic_vector(2 downto 0);
    write_go    : out std_logic_vector(2 downto 0);
    fu_seq      : out std_logic_vector(47 downto 0);
    empty       : out std_logic;
    dbg_raddr   : in  natural range 0 to NREG-1;
    dbg_rdata   : out std_logic_vector(DW-1 downto 0)
  );
end entity;

architecture rtl of scoreboard is
  constant NFU : positive := 3;
  type phase_t is (PH_WAIT, PH_EXEC, PH_DONE);
  type data_t is array (natural range <>) of std_logic_vector(DW-1 downto 0);
  type int_t  is array (natural range <>) of natural;
  type phase_arr is array (0 to NFU-1) of phase_t;

  signal regfile  : data_t(0 to NREG-1) := (others => (others => '0'));
  signal res_busy : std_logic_vector(0 to NREG-1) := (others => '0');          -- "result register status"
  signal res_fu   : int_t(0 to NREG-1) := (others => 0);

  signal busy, qjw, qkw, rj, rk : std_logic_vector(0 to NFU-1) := (others => '0');
  signal f_op, f_fi, f_fj, f_fk, qj, qk, cnt, seq : int_t(0 to NFU-1) := (others => 0);
  signal phase : phase_arr := (others => PH_WAIT);
  signal vj, vk, res : data_t(0 to NFU-1) := (others => (others => '0'));
  signal seq_ctr : natural range 0 to 65535 := 0;

  signal rgo, wgo : std_logic_vector(0 to NFU-1);
  signal pick : natural range 0 to NFU-1;
  signal ready_i : std_logic;
  signal s_wait : std_logic_vector(0 to 1);
  signal s_fu : int_t(0 to 1);

  function alu(op : natural; a, b : std_logic_vector) return std_logic_vector is
  begin
    case op is
      when 0      => return std_logic_vector(unsigned(a) + unsigned(b));
      when 1      => return std_logic_vector(unsigned(a) - unsigned(b));
      when others => return std_logic_vector(resize(unsigned(a) * unsigned(b), DW));
    end case;
  end function;
begin
  dbg_rdata   <= regfile(dbg_raddr);
  issue_ready <= ready_i;

  -- ---- per-unit "go" conditions ----------------------------------------------------------------
  go : process (all)
    variable war : boolean;
    variable e : std_logic;
  begin
    e := '1';
    for f in 0 to NFU-1 loop
      if busy(f) = '1' then e := '0'; end if;
      war := false;
      for g in 0 to NFU-1 loop
        if g /= f and busy(g) = '1' and ((f_fj(g) = f_fi(f) and rj(g) = '1') or (f_fk(g) = f_fi(f) and rk(g) = '1')) then war := true; end if;
      end loop;
      if busy(f) = '1' and phase(f) = PH_WAIT and rj(f) = '1' and rk(f) = '1' then rgo(f) <= '1'; else rgo(f) <= '0'; end if;
      if busy(f) = '1' and phase(f) = PH_DONE and not war then wgo(f) <= '1'; else wgo(f) <= '0'; end if;
      fu_seq(16*f + 15 downto 16*f) <= std_logic_vector(to_unsigned(seq(f), 16));
    end loop;
    empty <= e;
  end process;
  read_go  <= rgo(2) & rgo(1) & rgo(0);
  write_go <= wgo(2) & wgo(1) & wgo(0);

  -- ---- issue ------------------------------------------------------------------------------------
  issue_logic : process (all)
    variable have_fu : boolean;
    variable pk : natural range 0 to NFU-1;
    variable r : natural range 0 to NREG-1;
  begin
    have_fu := false;  pk := 0;
    if issue_op = 2 then
      if busy(1) = '0' then have_fu := true; pk := 1;
      elsif busy(2) = '0' then have_fu := true; pk := 2;
      end if;
    elsif busy(0) = '0' then have_fu := true; pk := 0;
    end if;
    pick <= pk;
    if have_fu and res_busy(issue_rd) = '0' then ready_i <= '1'; else ready_i <= '0'; end if;   -- structural and WAW
    for s in 0 to 1 loop
      if s = 0 then r := issue_rs; else r := issue_rt; end if;
      s_wait(s) <= res_busy(r);  s_fu(s) <= res_fu(r);
      if res_busy(r) = '1' and wgo(res_fu(r)) = '1' then s_wait(s) <= '0'; end if;   -- producer writes at this very edge
    end loop;
  end process;

  -- ---- the clocked machine ----------------------------------------------------------------------
  seq_p : process (clk, rst_n)
    variable do_issue : boolean;
  begin
    if rst_n = '0' then
      seq_ctr <= 0;
      regfile <= (others => (others => '0'));  res_busy <= (others => '0');  res_fu <= (others => 0);
      busy <= (others => '0');  qjw <= (others => '0');  qkw <= (others => '0');  rj <= (others => '0');  rk <= (others => '0');
      f_op <= (others => 0);  f_fi <= (others => 0);  f_fj <= (others => 0);  f_fk <= (others => 0);
      qj <= (others => 0);  qk <= (others => 0);  cnt <= (others => 0);  seq <= (others => 0);
      phase <= (others => PH_WAIT);
      vj <= (others => (others => '0'));  vk <= (others => (others => '0'));  res <= (others => (others => '0'));
    elsif rising_edge(clk) then
      do_issue := issue_valid = '1' and ready_i = '1';
      if init_we = '1' and (busy = "000") then regfile(init_reg) <= init_data; end if;
      for f in 0 to NFU-1 loop
        -- read operands: both ready -> latch values and start executing
        if rgo(f) = '1' then
          vj(f) <= regfile(f_fj(f));  vk(f) <= regfile(f_fk(f));
          rj(f) <= '0';  rk(f) <= '0';
          phase(f) <= PH_EXEC;
          if f = 0 then cnt(f) <= ADD_LAT; else cnt(f) <= MUL_LAT; end if;
        end if;
        -- execute
        if busy(f) = '1' and phase(f) = PH_EXEC then
          if cnt(f) = 1 then res(f) <= alu(f_op(f), vj(f), vk(f)); phase(f) <= PH_DONE; end if;
          cnt(f) <= cnt(f) - 1;
        end if;
        -- write result: update the register, free the unit, wake the units waiting for this result
        if wgo(f) = '1' then
          regfile(f_fi(f)) <= res(f);
          res_busy(f_fi(f)) <= '0';
          busy(f) <= '0';
          for g in 0 to NFU-1 loop
            if busy(g) = '1' and qjw(g) = '1' and qj(g) = f then qjw(g) <= '0'; rj(g) <= '1'; end if;
            if busy(g) = '1' and qkw(g) = '1' and qk(g) = f then qkw(g) <= '0'; rk(g) <= '1'; end if;
          end loop;
        end if;
      end loop;
      -- issue (the structural and WAW checks keep it disjoint from everything above)
      if do_issue then
        busy(pick) <= '1';  f_op(pick) <= issue_op;  f_fi(pick) <= issue_rd;  f_fj(pick) <= issue_rs;  f_fk(pick) <= issue_rt;
        qjw(pick) <= s_wait(0);  qj(pick) <= s_fu(0);  rj(pick) <= not s_wait(0);
        qkw(pick) <= s_wait(1);  qk(pick) <= s_fu(1);  rk(pick) <= not s_wait(1);
        phase(pick) <= PH_WAIT;  seq(pick) <= seq_ctr;
        if seq_ctr = 65535 then seq_ctr <= 0; else seq_ctr <= seq_ctr + 1; end if;
        res_busy(issue_rd) <= '1';  res_fu(issue_rd) <= pick;
      end if;
    end if;
  end process;
end architecture;
