library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

entity tb_tomasulo_rob is end entity;

architecture sim of tb_tomasulo_rob is
  constant NREG : positive := 8;
  constant ROBN : positive := 8;
  constant DW   : positive := 32;
  type data_t is array (natural range <>) of std_logic_vector(DW-1 downto 0);
  type int_t  is array (natural range <>) of integer;

  signal clk : std_logic := '0';
  signal rst_n, init_we, issue_valid : std_logic := '0';
  signal init_reg, issue_rd, issue_rs, issue_rt, dbg_raddr : natural range 0 to NREG-1 := 0;
  signal init_data : std_logic_vector(DW-1 downto 0) := (others => '0');
  signal issue_op : natural range 0 to 3 := 0;
  signal issue_ready, cdb_valid, commit_valid, empty : std_logic;
  signal cdb_tag : natural range 0 to ROBN-1;
  signal commit_reg : natural range 0 to NREG-1;
  signal commit_data, dbg_rdata : std_logic_vector(DW-1 downto 0);

  -- observation (owned by the logger process)
  signal edge_no : integer := 0;
  signal base    : integer := 0;                       -- driven by the stimulus process
  signal ngot    : integer := 0;
  signal got_reg, got_edge : int_t(0 to 8191) := (others => 0);
  signal got_val : data_t(0 to 8191) := (others => (others => '0'));
  signal bcast_edge : int_t(0 to ROBN-1) := (others => 0);
begin
  clk <= not clk after 5 ns;

  dut : entity work.tomasulo_rob
    generic map (NREG, ROBN, 3, 2, 2, 4, DW)
    port map (clk, rst_n, init_we, init_reg, init_data, issue_valid, issue_op, issue_rd, issue_rs, issue_rt,
              issue_ready, cdb_valid, cdb_tag, commit_valid, commit_reg, commit_data, empty, dbg_raddr, dbg_rdata);

  process (clk) begin
    if rising_edge(clk) and rst_n = '1' then edge_no <= edge_no + 1; end if;
  end process;

  logger : process (clk)
    variable n : integer := 0;
  begin
    if falling_edge(clk) and rst_n = '1' then
      if commit_valid = '1' then
        got_reg(n) <= commit_reg;  got_val(n) <= commit_data;  got_edge(n) <= edge_no - base;
        n := n + 1;  ngot <= n;
      end if;
      if cdb_valid = '1' then bcast_edge(cdb_tag) <= edge_no - base + 1; end if;   -- the broadcast happens at the next edge
    end if;
  end process;

  stim : process
    variable errors, stall_cycles : natural := 0;
    variable g : data_t(0 to NREG-1) := (others => (others => '0'));          -- sequential golden register file
    variable seed : unsigned(31 downto 0) := to_unsigned(1990, 32);
    variable start_n : integer;

    impure function rnd32 return std_logic_vector is
    begin
      seed := resize(seed * to_unsigned(1664525, 32) + to_unsigned(1013904223, 32), 32);
      return std_logic_vector(seed);
    end function;
    impure function rnd(nn : positive) return natural is
    begin
      seed := resize(seed * to_unsigned(1664525, 32) + to_unsigned(1013904223, 32), 32);
      return to_integer(seed(30 downto 16)) mod nn;
    end function;

    procedure set_reg(r : natural; v : std_logic_vector(DW-1 downto 0)) is
    begin
      wait until falling_edge(clk);  init_we <= '1';  init_reg <= r;  init_data <= v;
      wait until falling_edge(clk);  init_we <= '0';
      g(r) := v;
    end procedure;
    procedure issue(op, rd, rs, rt : natural) is
    begin
      wait until falling_edge(clk);
      issue_valid <= '1';  issue_op <= op;  issue_rd <= rd;  issue_rs <= rs;  issue_rt <= rt;
      wait for 1 ns;
      while issue_ready = '0' loop
        stall_cycles := stall_cycles + 1;
        wait until falling_edge(clk);  wait for 1 ns;
      end loop;
    end procedure;
    procedure finish_issue is
    begin wait until falling_edge(clk); issue_valid <= '0'; end procedure;
    procedure drain is
    begin
      while empty = '0' loop wait until falling_edge(clk); end loop;
      for k in 1 to 3 loop wait until falling_edge(clk); end loop;
    end procedure;
    procedure expect_int(what : string; got, exp_v : integer) is
    begin
      if got /= exp_v then
        errors := errors + 1;
        report "FAIL " & what & ": got " & integer'image(got) & " expected " & integer'image(exp_v) severity error;
      end if;
    end procedure;
    function model(op : natural; a, b : std_logic_vector) return std_logic_vector is
    begin
      case op is
        when 0 => return std_logic_vector(unsigned(a) + unsigned(b));
        when 1 => return std_logic_vector(unsigned(a) - unsigned(b));
        when others => return std_logic_vector(resize(unsigned(a) * unsigned(b), DW));
      end case;
    end function;
    procedure begin_program is
    begin
      start_n := ngot;
      base <= edge_no + 1;                 -- issue() first waits one more negedge: accepted at the 2nd posedge from here = "edge 1"
    end procedure;

    type prog_i is array (0 to 127) of natural;
    variable ops, rds, rss, rts : prog_i;
    variable ev : data_t(0 to 127);
    variable n : natural;
  begin
    wait until falling_edge(clk);  wait until falling_edge(clk);
    rst_n <= '1';

    -- 1. hand-derived schedule -------------------------------------------------------------
    --   I0: MUL r1 = r2*r3   I1: ADD r4 = r1+r5   I2: ADD r6 = r2+r3        (ADD_LAT=2, MUL_LAT=4)
    -- I2 (independent) finishes first, wins the CDB at edge 7; the multiplier waits and broadcasts at 8;
    -- I1 captures r1 off the bus at 8, starts at 9, broadcasts at 12.  Commits are in program order: 9, 13, 14.
    set_reg(2, x"00000003");  set_reg(3, x"00000004");  set_reg(5, x"0000000A");
    begin_program;  wait for 1 ns;
    issue(2, 1, 2, 3);  issue(0, 4, 1, 5);  issue(0, 6, 2, 3);  finish_issue;
    drain;
    expect_int("commits", ngot - start_n, 3);
    expect_int("bcast I2", bcast_edge(2), 7);  expect_int("bcast I0", bcast_edge(0), 8);  expect_int("bcast I1", bcast_edge(1), 12);
    expect_int("commit I0 edge", got_edge(start_n), 9);  expect_int("commit I1 edge", got_edge(start_n + 1), 13);  expect_int("commit I2 edge", got_edge(start_n + 2), 14);
    expect_int("commit order 0", got_reg(start_n), 1);  expect_int("commit order 1", got_reg(start_n + 1), 4);  expect_int("commit order 2", got_reg(start_n + 2), 6);
    expect_int("r1 value", to_integer(unsigned(got_val(start_n))), 12);
    expect_int("r4 value", to_integer(unsigned(got_val(start_n + 1))), 22);
    expect_int("r6 value", to_integer(unsigned(got_val(start_n + 2))), 7);
    report "directed: completion order I2,I0,I1 (CDB edges 7,8,12) but commit order I0,I1,I2 (edges 9,13,14)";

    -- 2. an isolated ADD commits at issue+5, an isolated MUL at issue+7 ---------------------
    begin_program;  wait for 1 ns;
    issue(0, 7, 2, 3);  finish_issue;  drain;
    expect_int("lone ADD commit edge", got_edge(start_n), 1 + 5);
    begin_program;  wait for 1 ns;
    issue(2, 7, 2, 3);  finish_issue;  drain;
    expect_int("lone MUL commit edge", got_edge(start_n), 1 + 7);

    -- 3. renaming: WAW and WAR on r1 do not serialize; architectural order is preserved -------
    --   I0: MUL r1=r2*r3 (12)  I1: ADD r4=r1+r5 (reads OLD r1 value 12)  I2: ADD r1=r5+r5 (20, completes first)
    set_reg(1, x"00000000");  set_reg(4, x"00000000");  set_reg(6, x"00000000");  set_reg(7, x"00000000");
    set_reg(2, x"00000003");  set_reg(3, x"00000004");  set_reg(5, x"0000000A");
    begin_program;  wait for 1 ns;
    issue(2, 1, 2, 3);  issue(0, 4, 1, 5);  issue(0, 1, 5, 5);  finish_issue;  drain;
    expect_int("rename commits", ngot - start_n, 3);
    expect_int("I1 used old r1", to_integer(unsigned(got_val(start_n + 1))), 22);
    expect_int("final r1 is the later write", to_integer(unsigned(got_val(start_n + 2))), 20);
    dbg_raddr <= 1;  wait for 1 ns;  expect_int("arch r1", to_integer(unsigned(dbg_rdata)), 20);
    dbg_raddr <= 4;  wait for 1 ns;  expect_int("arch r4", to_integer(unsigned(dbg_rdata)), 22);

    -- 4. random programs vs. a sequential golden model ----------------------------------------
    n := 120;
    for p in 0 to 39 loop
      drain;
      for r in 0 to NREG-1 loop set_reg(r, rnd32); end loop;
      for i in 0 to n-1 loop
        ops(i) := rnd(3);  rds(i) := rnd(NREG);  rss(i) := rnd(NREG);  rts(i) := rnd(NREG);
        ev(i) := model(ops(i), g(rss(i)), g(rts(i)));
        g(rds(i)) := ev(i);
      end loop;
      begin_program;  wait for 1 ns;
      for i in 0 to n-1 loop issue(ops(i), rds(i), rss(i), rts(i)); end loop;
      finish_issue;  drain;
      if ngot - start_n /= n then
        errors := errors + 1;  report "FAIL program " & integer'image(p) & ": wrong number of commits" severity error;
      else
        for i in 0 to n-1 loop
          if got_reg(start_n + i) /= rds(i) or got_val(start_n + i) /= ev(i) then
            errors := errors + 1;
            report "FAIL program " & integer'image(p) & " instr " & integer'image(i) severity error;
          end if;
        end loop;
      end if;
      for r in 0 to NREG-1 loop
        dbg_raddr <= r;  wait for 1 ns;
        if dbg_rdata /= g(r) then errors := errors + 1; report "FAIL program " & integer'image(p) & " final register r" & integer'image(r) severity error; end if;
      end loop;
    end loop;
    if stall_cycles = 0 then errors := errors + 1; report "FAIL: random phase never exercised issue back-pressure" severity error; end if;
    report "random phase: 40 programs x 120 instructions, " & integer'image(stall_cycles) & " issue-stall cycles";
    if errors = 0 then report "TEST PASSED";
    else report "TEST FAILED: " & integer'image(errors) & " errors" severity error;
    end if;
    finish;
  end process;
end architecture;
