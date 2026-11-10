-- Directed programs (hand-derived), a counted loop under both predictors, and 40 random forward-branch programs checked against a
-- sequential interpreter.  Policy false = always not taken, policy true = backward taken / forward not taken.
--
--  M (mispredicted forward branch):   0 ADD r3=r1+r2 | 1 BNEZ r3,4 | 2 ADD r4=r4+r5 | 3 ADD r4=r4+r5 | 4 ADD r6=r1+r1 | 5 HALT     (r1=1 r2=2 r5=10)
--     I0 issues at edge 1 and commits at 6.  The branch (issue 2) waits for r3 (CDB edge 5), executes 6..8, broadcasts at 9, reaches the head of
--     the ROB and commits at edge 10 -> mispredicted (predicted not taken, r3 = 3): flush, pc <- 4.  Nothing issues on edge 10; ADD r6 issues at 11
--     (execute 12-13, done 14, CDB 15, commit 16) and HALT issues at 12 and commits at 17.  So: 17 cycles, 4 commits, 1 flush, and 2 squashed
--     instructions (the two wrong-path ADDs I2, I3: the 3-entry add-station pool is full, so I4/I5 of the first pass have not issued yet).
--     The wrong-path ADDs never reach r4 -> r4 stays 0.
--  C: the same program with r1 = r2 = 0: r3 = 0, the branch is NOT taken, the prediction was right -> 6 commits, no flush, r4 = 20.
--  L (loop, N = 6): 0 ADD r3=r3+r1 | 1 SUB r1=r1-r5 | 2 BNEZ r1,0 | 3 HALT   (r1=6, r5=1) -> r3 = 21, 19 commits, 6 branches.
--     policy false mispredicts the 5 taken iterations -> 5 flushes; policy true predicts the backward branch taken and mispredicts only the exit -> 1 flush.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity spec_harness is
  generic (PREDICT : boolean := false);
  port (clk, rst_n : in std_logic; done : out std_logic; errors : out integer; loop_cycles : out integer);
end entity;

architecture sim of spec_harness is
  subtype word is std_logic_vector(31 downto 0);
  type imem_t is array (0 to 63) of word;
  type regs_t is array (0 to 7) of word;
  signal imem : imem_t := (others => (others => '0'));

  signal local_rst, init_we, start : std_logic := '0';
  signal dut_rst_n : std_logic;
  signal init_reg, dbg_raddr : natural range 0 to 7 := 0;
  signal init_data : word := (others => '0');
  signal imem_addr : std_logic_vector(7 downto 0);
  signal imem_rdata, dbg_rdata : word;
  signal halted : std_logic;
  signal cycles, committed, branches, flushes, squashed : unsigned(31 downto 0);

  function ins(o, rd, rs, rt : natural; tgt : natural) return word is
  begin
    return std_logic_vector(to_unsigned(o, 4)) & std_logic_vector(to_unsigned(rd, 4)) & std_logic_vector(to_unsigned(rs, 4)) &
           std_logic_vector(to_unsigned(rt, 4)) & std_logic_vector(to_unsigned(tgt, 16));
  end function;
  constant ADD : natural := 0;  constant SUB : natural := 1;  constant MUL : natural := 2;  constant BNEZ : natural := 3;  constant HALT : natural := 4;
begin
  dut_rst_n <= rst_n and not local_rst;
  dut : entity work.tomasulo_spec generic map (PREDICT_BTFN => PREDICT)
    port map (clk, dut_rst_n, init_we, init_reg, init_data, start, imem_addr, imem_rdata, halted,
              cycles, committed, branches, flushes, squashed, dbg_raddr, dbg_rdata);
  imem_rdata <= imem(to_integer(unsigned(imem_addr(5 downto 0))));

  stim : process
    variable errs : integer := 0;
    variable seed : unsigned(31 downto 0) := to_unsigned(1967, 32);
    variable ri, g : regs_t;
    type prog_i is array (0 to 31) of natural;
    variable p_op, p_rd, p_rs, p_rt, p_tg : prog_i;
    variable n, pcg, steps, dyn, dbr, taken : integer;

    impure function rnd(nn : positive) return natural is
    begin
      seed := resize(seed * to_unsigned(1664525, 32) + to_unsigned(1013904223, 32), 32);
      return to_integer(seed(30 downto 16)) mod nn;
    end function;

    procedure put(i : natural; w : word) is begin imem(i) <= w; end procedure;
    procedure run_program is
      variable t : natural := 0;
    begin
      wait until falling_edge(clk);  local_rst <= '1';  wait until falling_edge(clk);  wait until falling_edge(clk);  local_rst <= '0';
      for r in 0 to 7 loop
        wait until falling_edge(clk);  init_we <= '1';  init_reg <= r;  init_data <= ri(r);
        wait until falling_edge(clk);  init_we <= '0';
      end loop;
      wait until falling_edge(clk);  start <= '1';  wait until falling_edge(clk);  start <= '0';
      while halted /= '1' and t < 3000 loop wait until rising_edge(clk); t := t + 1; end loop;
      if halted /= '1' then errs := errs + 1; report "FAIL: never halted" severity error; end if;
      wait until falling_edge(clk);
    end procedure;
    procedure check_reg(r : natural; v : word; what : string) is
    begin
      dbg_raddr <= r;  wait for 1 ns;
      if dbg_rdata /= v then errs := errs + 1; report "FAIL " & what & ": r" & integer'image(r) & " = " & integer'image(to_integer(unsigned(dbg_rdata))) severity error; end if;
    end procedure;
    procedure check_stat(what : string; got : unsigned; exp_v : integer) is
    begin
      if to_integer(got) /= exp_v then
        errs := errs + 1;  report "FAIL " & what & " = " & integer'image(to_integer(got)) & ", expected " & integer'image(exp_v) severity error;
      end if;
    end procedure;
    function w32(v : natural) return word is begin return std_logic_vector(to_unsigned(v, 32)); end function;
  begin
    done <= '0';  errors <= 0;  loop_cycles <= 0;
    wait until rst_n = '1';

    -- M: mispredicted forward branch
    put(0, ins(ADD, 3, 1, 2, 0));  put(1, ins(BNEZ, 0, 3, 0, 4));  put(2, ins(ADD, 4, 4, 5, 0));
    put(3, ins(ADD, 4, 4, 5, 0));  put(4, ins(ADD, 6, 1, 1, 0));   put(5, ins(HALT, 0, 0, 0, 0));
    ri := (others => (others => '0'));  ri(1) := w32(1);  ri(2) := w32(2);  ri(5) := w32(10);
    run_program;
    check_stat("M cycles", cycles, 17);  check_stat("M committed", committed, 4);  check_stat("M branches", branches, 1);
    check_stat("M flushes", flushes, 1); check_stat("M squashed", squashed, 2);
    check_reg(3, w32(3), "M");  check_reg(4, w32(0), "M (wrong path must not commit)");  check_reg(6, w32(2), "M");

    -- C: same program, branch not taken (prediction correct)
    ri(1) := w32(0);  ri(2) := w32(0);  ri(5) := w32(10);
    run_program;
    check_stat("C committed", committed, 6);  check_stat("C flushes", flushes, 0);  check_stat("C squashed", squashed, 0);
    check_reg(3, w32(0), "C");  check_reg(4, w32(20), "C");  check_reg(6, w32(0), "C");

    -- L: counted loop
    put(0, ins(ADD, 3, 3, 1, 0));  put(1, ins(SUB, 1, 1, 5, 0));  put(2, ins(BNEZ, 0, 1, 0, 0));  put(3, ins(HALT, 0, 0, 0, 0));
    ri := (others => (others => '0'));  ri(1) := w32(6);  ri(5) := w32(1);
    run_program;
    check_stat("L committed", committed, 19);  check_stat("L branches", branches, 6);
    if PREDICT then check_stat("L flushes", flushes, 1); else check_stat("L flushes", flushes, 5); end if;
    check_reg(3, w32(21), "L");  check_reg(1, w32(0), "L");
    loop_cycles <= to_integer(cycles);
    report "loop, policy " & boolean'image(PREDICT) & ": " & integer'image(to_integer(cycles)) & " cycles, " &
           integer'image(to_integer(flushes)) & " flushes, " & integer'image(to_integer(squashed)) & " squashed";

    -- random forward-branch programs vs a sequential interpreter
    for p in 0 to 39 loop
      n := 24;
      for i in 0 to n-1 loop
        if rnd(5) = 0 and i < n - 2 then                        -- forward BNEZ to a later instruction (possibly the HALT)
          p_op(i) := 3;  p_rd(i) := 0;  p_rs(i) := rnd(8);  p_rt(i) := 0;  p_tg(i) := i + 2 + rnd(n - i - 1);
        else
          p_op(i) := rnd(3);  p_rd(i) := rnd(8);  p_rs(i) := rnd(8);  p_rt(i) := rnd(8);  p_tg(i) := 0;
        end if;
        put(i, ins(p_op(i), p_rd(i), p_rs(i), p_rt(i), p_tg(i)));
      end loop;
      put(n, ins(HALT, 0, 0, 0, 0));
      for r in 0 to 7 loop ri(r) := w32(rnd(4)); g(r) := ri(r); end loop;
      pcg := 0;  dyn := 0;  dbr := 0;  taken := 0;  steps := 0;       -- sequential interpreter
      while pcg < n and steps < 1000 loop
        steps := steps + 1;  dyn := dyn + 1;
        case p_op(pcg) is
          when 0 => g(p_rd(pcg)) := std_logic_vector(unsigned(g(p_rs(pcg))) + unsigned(g(p_rt(pcg))));  pcg := pcg + 1;
          when 1 => g(p_rd(pcg)) := std_logic_vector(unsigned(g(p_rs(pcg))) - unsigned(g(p_rt(pcg))));  pcg := pcg + 1;
          when 2 => g(p_rd(pcg)) := std_logic_vector(resize(unsigned(g(p_rs(pcg))) * unsigned(g(p_rt(pcg))), 32));  pcg := pcg + 1;
          when others =>
            dbr := dbr + 1;
            if unsigned(g(p_rs(pcg))) /= 0 then taken := taken + 1;  pcg := p_tg(pcg); else pcg := pcg + 1; end if;
        end case;
      end loop;
      dyn := dyn + 1;                                                 -- the HALT
      run_program;
      for r in 0 to 7 loop check_reg(r, g(r), "random " & integer'image(p)); end loop;
      check_stat("random committed", committed, dyn);
      check_stat("random branches", branches, dbr);
      check_stat("random flushes", flushes, taken);                   -- every forward branch is predicted not taken, under both policies
    end loop;
    errors <= errs;  done <= '1';
    wait;
  end process;
end architecture;

library ieee;
use ieee.std_logic_1164.all;
use std.env.all;

entity tb_tomasulo_spec is end entity;

architecture sim of tb_tomasulo_spec is
  signal clk : std_logic := '0';
  signal rst_n : std_logic := '0';
  signal d0, d1 : std_logic;
  signal er0, er1, lc0, lc1 : integer;
begin
  clk <= not clk after 5 ns;
  nt   : entity work.spec_harness generic map (false) port map (clk, rst_n, d0, er0, lc0);
  btfn : entity work.spec_harness generic map (true)  port map (clk, rst_n, d1, er1, lc1);

  process
    variable extra : integer := 0;
  begin
    wait until falling_edge(clk);  wait until falling_edge(clk);
    rst_n <= '1';
    wait until d0 = '1' and d1 = '1';
    wait for 1 ns;
    if lc1 >= lc0 then
      extra := extra + 1;
      report "FAIL: backward-taken prediction (" & integer'image(lc1) & " cycles) is not faster than always-not-taken (" & integer'image(lc0) & ")" severity error;
    else
      report "loop of 6: always-not-taken " & integer'image(lc0) & " cycles, backward-taken/forward-not-taken " & integer'image(lc1) &
             " cycles (" & integer'image(lc0 - lc1) & " saved)";
    end if;
    if er0 + er1 + extra = 0 then report "TEST PASSED";
    else report "TEST FAILED: " & integer'image(er0 + er1 + extra) & " errors" severity error;
    end if;
    finish;
  end process;
end architecture;
