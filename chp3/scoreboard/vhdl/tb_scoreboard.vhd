-- 1. A hand-derived schedule that shows RAW, structural, WAR and WAW hazards in one 5-instruction program.
-- 2. Random programs, checked against (a) a sequential golden register file and (b) exact timing rules derived from the spec:
--      read  = max(issue+1, 1 + write of the producer of each source)
--      write = max(read+LAT+1, 1 + read of every OLDER reader of the destination)
--      issue after the previous issue; after the write of an older writer of the same register (WAW);
--      at most 0 older adds / 1 older multiply still in flight when an add / multiply issues (structural).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

entity tb_scoreboard is end entity;

architecture sim of tb_scoreboard is
  constant NREG : positive := 8;
  constant DW   : positive := 32;
  constant NMAX : positive := 4096;
  type data_t is array (natural range <>) of std_logic_vector(DW-1 downto 0);
  type int_t  is array (natural range <>) of integer;

  signal clk : std_logic := '0';
  signal rst_n, init_we, issue_valid : std_logic := '0';
  signal init_reg, issue_rd, issue_rs, issue_rt, dbg_raddr : natural range 0 to NREG-1 := 0;
  signal init_data : std_logic_vector(DW-1 downto 0) := (others => '0');
  signal issue_op : natural range 0 to 3 := 0;
  signal issue_ready, empty : std_logic;
  signal read_go, write_go : std_logic_vector(2 downto 0);
  signal fu_seq : std_logic_vector(47 downto 0);
  signal dbg_rdata : std_logic_vector(DW-1 downto 0);

  signal edge_no : integer := 0;
  signal ev_issue : int_t(0 to NMAX-1) := (others => 0);     -- written by the stimulus process
  signal ev_read, ev_write : int_t(0 to NMAX-1) := (others => 0);   -- written by the logger process
begin
  clk <= not clk after 5 ns;
  dut : entity work.scoreboard generic map (NREG, DW, 2, 4)
    port map (clk, rst_n, init_we, init_reg, init_data, issue_valid, issue_op, issue_rd, issue_rs, issue_rt,
              issue_ready, read_go, write_go, fu_seq, empty, dbg_raddr, dbg_rdata);

  process (clk) begin
    if rising_edge(clk) and rst_n = '1' then edge_no <= edge_no + 1; end if;
  end process;

  logger : process (clk)
    variable sq : natural;
  begin
    if falling_edge(clk) and rst_n = '1' then
      for f in 0 to 2 loop
        sq := to_integer(unsigned(fu_seq(16*f + 15 downto 16*f)));
        if read_go(f)  = '1' then ev_read(sq)  <= edge_no + 1; end if;       -- happens at the next edge
        if write_go(f) = '1' then ev_write(sq) <= edge_no + 1; end if;
      end loop;
    end if;
  end process;

  stim : process
    variable errors, stall_cycles, n_issued : natural := 0;
    variable g : data_t(0 to NREG-1) := (others => (others => '0'));
    variable seed : unsigned(31 downto 0) := to_unsigned(6600, 32);
    variable s0, base : integer;

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
      ev_issue(n_issued) <= edge_no + 1;  n_issued := n_issued + 1;           -- accepted at the next edge
    end procedure;
    procedure finish_issue is
    begin wait until falling_edge(clk); issue_valid <= '0'; end procedure;
    procedure drain is
    begin
      while empty = '0' loop wait until falling_edge(clk); end loop;
      for k in 1 to 3 loop wait until falling_edge(clk); end loop;
    end procedure;
    function model(op : natural; a, b : std_logic_vector) return std_logic_vector is
    begin
      case op is
        when 0 => return std_logic_vector(unsigned(a) + unsigned(b));
        when 1 => return std_logic_vector(unsigned(a) - unsigned(b));
        when others => return std_logic_vector(resize(unsigned(a) * unsigned(b), DW));
      end case;
    end function;
    function lat(op : natural) return natural is begin if op = 2 then return 4; else return 2; end if; end function;
    function imax(a, b : integer) return integer is begin if a > b then return a; else return b; end if; end function;
    procedure expect_int(what : string; got, exp_v : integer) is
    begin
      if got /= exp_v then
        errors := errors + 1;
        report "FAIL " & what & ": got " & integer'image(got) & " expected " & integer'image(exp_v) severity error;
      end if;
    end procedure;

    type prog_i is array (0 to 127) of natural;
    variable ops, rds, rss, rts : prog_i;
    variable n, s, iss, rd_t, wr_t, rmin, wmin, in_flight, src, prod : integer;
    type int5 is array (0 to 4) of integer;
    variable exp_i, exp_r, exp_w : int5;
  begin
    wait until falling_edge(clk);  wait until falling_edge(clk);
    rst_n <= '1';

    -- 1. hand-derived schedule -----------------------------------------------------------------
    --   J0: MUL r1 = r2*r3    J1: MUL r4 = r1*r5    J2: ADD r5 = r2+r3    J3: ADD r6 = r5+r4    J4: MUL r4 = r2*r3
    --   issue/read/write edges:  J0 1/2/7   J1 2/8/13 (RAW on r1)   J2 3/4/9 (WAR: waits for J1 to read old r5)
    --                            J3 10/14/17 (structural: add unit busy until J2 writes; RAW on r4)
    --                            J4 14/15/20 (WAW: r4 pending from J1 until edge 13)
    set_reg(2, x"00000003");  set_reg(3, x"00000004");  set_reg(5, x"0000000A");
    base := edge_no;  s0 := n_issued;
    issue(2, 1, 2, 3);  issue(2, 4, 1, 5);  issue(0, 5, 2, 3);  issue(0, 6, 5, 4);  issue(2, 4, 2, 3);  finish_issue;
    drain;
    exp_i := (1, 2, 3, 10, 14);  exp_r := (2, 8, 4, 14, 15);  exp_w := (7, 13, 9, 17, 20);
    -- the first accepted instruction is "edge 1" only after issue()'s extra negedge, so shift by base+1
    for j in 0 to 4 loop
      expect_int("J" & integer'image(j) & " issue", ev_issue(s0 + j) - (base + 1), exp_i(j));
      expect_int("J" & integer'image(j) & " read",  ev_read (s0 + j) - (base + 1), exp_r(j));
      expect_int("J" & integer'image(j) & " write", ev_write(s0 + j) - (base + 1), exp_w(j));
    end loop;
    dbg_raddr <= 1;  wait for 1 ns;  expect_int("r1", to_integer(unsigned(dbg_rdata)), 12);
    dbg_raddr <= 4;  wait for 1 ns;  expect_int("r4", to_integer(unsigned(dbg_rdata)), 12);     -- J4 wrote last
    dbg_raddr <= 5;  wait for 1 ns;  expect_int("r5", to_integer(unsigned(dbg_rdata)), 7);
    dbg_raddr <= 6;  wait for 1 ns;  expect_int("r6", to_integer(unsigned(dbg_rdata)), 127);    -- J3 read J2's r5 (7) and J1's r4 (120)
    report "directed: RAW, structural, WAR and WAW stalls reproduced exactly (20 cycles)";

    -- 2. random programs -------------------------------------------------------------------------
    n := 100;
    for p in 0 to 29 loop
      drain;
      for r in 0 to NREG-1 loop set_reg(r, rnd32); end loop;
      for i in 0 to n-1 loop
        ops(i) := rnd(3);  rds(i) := rnd(NREG);  rss(i) := rnd(NREG);  rts(i) := rnd(NREG);
        g(rds(i)) := model(ops(i), g(rss(i)), g(rts(i)));
      end loop;
      s := n_issued;
      for i in 0 to n-1 loop issue(ops(i), rds(i), rss(i), rts(i)); end loop;
      finish_issue;  drain;
      -- (a) architectural state
      for r in 0 to NREG-1 loop
        dbg_raddr <= r;  wait for 1 ns;
        if dbg_rdata /= g(r) then errors := errors + 1; report "FAIL program " & integer'image(p) & " final r" & integer'image(r) severity error; end if;
      end loop;
      -- (b) timing rules, from the log alone
      for i in 0 to n-1 loop
        iss := ev_issue(s + i);  rd_t := ev_read(s + i);  wr_t := ev_write(s + i);
        if i > 0 and iss <= ev_issue(s + i - 1) then errors := errors + 1; report "FAIL p" & integer'image(p) & " i" & integer'image(i) & ": issue not in order" severity error; end if;
        rmin := iss + 1;
        for k in 0 to 1 loop                                                -- RAW: latest older writer of each source
          if k = 0 then src := rss(i); else src := rts(i); end if;
          prod := -1;
          for j in 0 to i-1 loop if rds(j) = src then prod := j; end if; end loop;
          if prod >= 0 then rmin := imax(rmin, ev_write(s + prod) + 1); end if;
        end loop;
        if rd_t /= rmin then errors := errors + 1; report "FAIL p" & integer'image(p) & " i" & integer'image(i) & ": read at " & integer'image(rd_t) & ", rule says " & integer'image(rmin) severity error; end if;
        wmin := rd_t + lat(ops(i)) + 1;
        for j in 0 to i-1 loop                                              -- WAR: every older reader of my destination
          if rss(j) = rds(i) or rts(j) = rds(i) then wmin := imax(wmin, ev_read(s + j) + 1); end if;
          if rds(j) = rds(i) and iss <= ev_write(s + j) then errors := errors + 1; report "FAIL p" & integer'image(p) & " i" & integer'image(i) & ": WAW, issued before older writer finished" severity error; end if;
        end loop;
        if wr_t /= wmin then errors := errors + 1; report "FAIL p" & integer'image(p) & " i" & integer'image(i) & ": write at " & integer'image(wr_t) & ", rule says " & integer'image(wmin) severity error; end if;
        in_flight := 0;                                                     -- structural: add unit x1, multipliers x2
        for j in 0 to i-1 loop
          if (ops(j) = 2) = (ops(i) = 2) and ev_write(s + j) >= iss then in_flight := in_flight + 1; end if;
        end loop;
        if (ops(i) = 2 and in_flight > 1) or (ops(i) /= 2 and in_flight > 0) then
          errors := errors + 1; report "FAIL p" & integer'image(p) & " i" & integer'image(i) & ": structural hazard violated" severity error;
        end if;
      end loop;
    end loop;
    if stall_cycles = 0 then errors := errors + 1; report "FAIL: issue stalls never exercised" severity error; end if;
    report "random phase: 30 programs x 100 instructions, " & integer'image(stall_cycles) & " issue-stall cycles";
    if errors = 0 then report "TEST PASSED";
    else report "TEST FAILED: " & integer'image(errors) & " errors" severity error;
    end if;
    finish;
  end process;
end architecture;
