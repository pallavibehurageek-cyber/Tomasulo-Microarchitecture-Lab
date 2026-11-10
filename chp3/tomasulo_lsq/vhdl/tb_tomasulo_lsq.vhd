-- Hand-derived programs (first instruction issues at edge 1; LD_LAT = 3, FWD_LAT = 1, MUL_LAT = 4, ADD_LAT = 2; "cycles" = edge at which HALT commits).
-- Timing rules (see the RTL header): load  address i+1, start i+2, done i+2+L, CDB i+3+L, commit i+4+L;   store  address i+1, done i+2, commit (write) i+3.
--   A  LD r1,[r0+4] ; HALT                  LD issues 1, address 2, start 3, done 6, CDB 7, commit 8; HALT commits 9                               -> 9 cycles
--   B  ST r2,[r0+5] ; LD r1,[r0+5] ; HALT   ST: address 2, done 3, commit 4.  LD: address 3 (one address unit), start 4 -- the store is still in the queue -> FORWARD:
--        done 5, CDB 6, commit 7, HALT 8 -> 8 cycles.   Without forwarding the load must wait until the store has committed (edge 4): start 5, done 8, CDB 9, commit 10 -> 11 cycles
--   B2 MUL r3=r1*r2 ; ST r3,[r0+5] ; LD r4,[r0+5] ; HALT    MUL CDB at 7, commit 8.  ST data arrives at 7, done 8, commit 9.  LD address 4; with forwarding it starts when the data
--        is there (edge 8): done 9, CDB 10, commit 11, HALT 12 -> 12 cycles.   Without: starts after the store commit (edge 10): done 13, CDB 14, commit 15, HALT 16 -> 16
--   C  MUL r3=r1*r2 ; ST r5,[r3+0] ; LD r4,[r0+9] ; HALT    the store's address waits for the MUL (CDB 7, address at 8): the LD (address known at 4, different address!) may not start
--        until the older store's address is known -- edge 9: done 12, CDB 13, commit 14, HALT 15 -> 15 cycles   (conservative memory disambiguation)
--   C0 ST r5,[r0+6] ; LD r4,[r0+9] ; HALT   no wait: LD address 3, start 4 (store address known, different), done 7, CDB 8, commit 9, HALT 10 -> 10 cycles
--   S  ST r1,[r0+5] ; ST r2,[r0+5] ; LD r3,[r0+5] ; HALT    the load takes the value of the YOUNGEST older store: r3 = r2.  ST1 commits 4, ST2 commits 5, LD address 4, start 5:
--        forward (ST2 still queued): done 6, CDB 7, commit 8, HALT 9 -> 9 cycles.   Without forwarding: start 6, done 9, CDB 10, commit 11, HALT 12 -> 12 cycles
--   S2 MUL r6=r1*r2 ; ST r1,[r0+5] ; ST r2,[r0+5] ; LD r3,[r0+5] ; HALT    the slow multiply keeps both stores in the queue (MUL commit 8, ST1 9, ST2 10).  LD: address 5, start 6; with
--        forwarding the YOUNGEST older store (ST2) supplies r2: done 7, CDB 8, commit 11, HALT 12 -> 12 cycles.   Without: start after ST2 committed (11): done 14, CDB 15, commit 16, HALT 17
--   D  wrong path: ADD r3=r1+r2 ; BNEZ r3,6 ; ST r4,[r0+7] ; LD r6,[r0+7] ; ADD r5=r6+r6 ; ADD r5=r5+r5 ; LD r7,[r0+8] ; HALT    (r1=1 r2=2 r4=99, mem[7]=5, mem[8]=31)
--        the branch (taken, predicted not taken) commits at 10 and flushes as in tomasulo_spec; LD r7 issues at 11, commit 18, HALT 19.  The wrong-path store NEVER reaches memory
--        (mem[7] stays 5), the wrong-path load and adds never reach r6 / r5, r7 = 31 -> 19 cycles, 1 flush, 4 commits (ADD, BNEZ, LD, HALT), 1 load, 0 stores
-- Then 40 random programs (ADD/SUB/MUL, LD, ST, forward BNEZ; load/store bases are the constants r0 = 0 / r1 = 1 and the offsets are 0..7, so addresses alias heavily) against a sequential interpreter:
-- all registers, all 64 memory words, committed instructions, loads, stores, branches and flushes (= taken branches).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity lsq_harness is
  generic (FORWARD : boolean := true);
  port (clk, rst_n : in std_logic; done : out std_logic; errors : out integer; rand_cycles : out integer; rand_fwd : out integer);
end entity;

architecture sim of lsq_harness is
  subtype word is std_logic_vector(31 downto 0);
  type imem_t is array (0 to 63) of word;
  type regs_t is array (0 to 7) of word;
  type mem_t is array (0 to 63) of word;
  signal imem : imem_t := (others => (others => '0'));

  signal local_rst, init_we, dinit_we, start : std_logic := '0';
  signal dut_rst_n : std_logic;
  signal init_reg, dbg_raddr : natural range 0 to 7 := 0;
  signal dinit_addr, dbg_maddr : natural range 0 to 63 := 0;
  signal init_data, dinit_data : word := (others => '0');
  signal imem_addr : std_logic_vector(7 downto 0);
  signal imem_rdata, dbg_rdata, dbg_mdata : word;
  signal halted : std_logic;
  signal cycles, committed, branches, flushes, squashed, loads, stores, fwds : unsigned(31 downto 0);

  function ins(o, rd, rs, rt : natural; imm : natural) return word is
  begin
    return std_logic_vector(to_unsigned(o, 4)) & std_logic_vector(to_unsigned(rd, 4)) & std_logic_vector(to_unsigned(rs, 4)) &
           std_logic_vector(to_unsigned(rt, 4)) & std_logic_vector(to_unsigned(imm, 16));
  end function;
  constant ADD : natural := 0;  constant SUB : natural := 1;  constant MUL : natural := 2;  constant BNEZ : natural := 3;  constant HALT : natural := 4;
  constant LD : natural := 5;  constant ST : natural := 6;
begin
  dut_rst_n <= rst_n and not local_rst;
  dut : entity work.tomasulo_lsq generic map (FORWARD => FORWARD)
    port map (clk, dut_rst_n, init_we, init_reg, init_data, dinit_we, dinit_addr, dinit_data, start, imem_addr, imem_rdata, halted,
              cycles, committed, branches, flushes, squashed, loads, stores, fwds, dbg_raddr, dbg_rdata, dbg_maddr, dbg_mdata);
  imem_rdata <= imem(to_integer(unsigned(imem_addr(5 downto 0))));

  stim : process
    variable errs : integer := 0;
    variable seed : unsigned(31 downto 0) := to_unsigned(1967, 32);
    variable ri, g : regs_t;
    variable mi, gm : mem_t;
    type prog_i is array (0 to 31) of natural;
    variable p_op, p_rd, p_rs, p_rt, p_tg : prog_i;
    variable n, pcg, steps, dyn, dbr, taken, nld, nst, ty, f_rd, f_rs, f_rt, f_im, f_tg, f_op, ad, sumc, sumf : integer;

    impure function rnd(nn : positive) return natural is
    begin
      seed := resize(seed * to_unsigned(1664525, 32) + to_unsigned(1013904223, 32), 32);
      return to_integer(seed(30 downto 16)) mod nn;
    end function;

    procedure put(i : natural; w : word) is begin imem(i) <= w; end procedure;
    function w32(v : natural) return word is begin return std_logic_vector(to_unsigned(v, 32)); end function;
    procedure clear_state is
    begin
      ri := (others => (others => '0'));
      for a in 0 to 63 loop mi(a) := w32(1000 + a); end loop;
    end procedure;
    procedure run_program is
      variable t : natural := 0;
    begin
      wait until falling_edge(clk);  local_rst <= '1';  wait until falling_edge(clk);  wait until falling_edge(clk);  local_rst <= '0';
      for r in 0 to 7 loop
        wait until falling_edge(clk);  init_we <= '1';  init_reg <= r;  init_data <= ri(r);
        wait until falling_edge(clk);  init_we <= '0';
      end loop;
      for a in 0 to 63 loop
        wait until falling_edge(clk);  dinit_we <= '1';  dinit_addr <= a;  dinit_data <= mi(a);
        wait until falling_edge(clk);  dinit_we <= '0';
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
    procedure check_mem(a : natural; v : word; what : string) is
    begin
      dbg_maddr <= a;  wait for 1 ns;
      if dbg_mdata /= v then errs := errs + 1; report "FAIL " & what & ": mem[" & integer'image(a) & "] = " & integer'image(to_integer(unsigned(dbg_mdata))) severity error; end if;
    end procedure;
    procedure check_stat(what : string; got : unsigned; exp_v : integer) is
    begin
      if to_integer(got) /= exp_v then
        errs := errs + 1;  report "FAIL " & what & " = " & integer'image(to_integer(got)) & ", expected " & integer'image(exp_v) severity error;
      end if;
    end procedure;
    function fsel(a, b : integer) return integer is begin if FORWARD then return a; else return b; end if; end function;
  begin
    done <= '0';  errors <= 0;  rand_cycles <= 0;  rand_fwd <= 0;
    wait until rst_n = '1';

    -- A: one load
    put(0, ins(LD, 1, 0, 0, 4));  put(1, ins(HALT, 0, 0, 0, 0));
    clear_state;  mi(4) := w32(123);
    run_program;
    check_stat("A cycles", cycles, 9);  check_stat("A loads", loads, 1);  check_stat("A forwards", fwds, 0);  check_reg(1, w32(123), "A");

    -- B: store then load of the same address
    put(0, ins(ST, 0, 0, 2, 5));  put(1, ins(LD, 1, 0, 0, 5));  put(2, ins(HALT, 0, 0, 0, 0));
    clear_state;  ri(2) := w32(77);
    run_program;
    check_stat("B cycles", cycles, fsel(8, 11));  check_stat("B forwards", fwds, fsel(1, 0));  check_stat("B stores", stores, 1);  check_stat("B loads", loads, 1);
    check_reg(1, w32(77), "B");  check_mem(5, w32(77), "B");

    -- B2: the store's data comes from a multiply
    put(0, ins(MUL, 3, 1, 2, 0));  put(1, ins(ST, 0, 0, 3, 5));  put(2, ins(LD, 4, 0, 0, 5));  put(3, ins(HALT, 0, 0, 0, 0));
    clear_state;  ri(1) := w32(2);  ri(2) := w32(3);
    run_program;
    check_stat("B2 cycles", cycles, fsel(12, 16));  check_stat("B2 forwards", fwds, fsel(1, 0));
    check_reg(4, w32(6), "B2");  check_mem(5, w32(6), "B2");

    -- C: an older store with an unknown address blocks a younger load of ANOTHER address
    put(0, ins(MUL, 3, 1, 2, 0));  put(1, ins(ST, 0, 3, 5, 0));  put(2, ins(LD, 4, 0, 0, 9));  put(3, ins(HALT, 0, 0, 0, 0));
    clear_state;  ri(1) := w32(2);  ri(2) := w32(3);  ri(5) := w32(55);
    run_program;
    check_stat("C cycles", cycles, 15);  check_stat("C forwards", fwds, 0);  check_reg(4, mi(9), "C");  check_mem(6, w32(55), "C");
    -- C0: the same without the late address
    put(0, ins(ST, 0, 0, 5, 6));  put(1, ins(LD, 4, 0, 0, 9));  put(2, ins(HALT, 0, 0, 0, 0));
    clear_state;  ri(5) := w32(55);
    run_program;
    check_stat("C0 cycles", cycles, 10);  check_reg(4, mi(9), "C0");  check_mem(6, w32(55), "C0");

    -- S: two stores to one address, then a load
    put(0, ins(ST, 0, 0, 1, 5));  put(1, ins(ST, 0, 0, 2, 5));  put(2, ins(LD, 3, 0, 0, 5));  put(3, ins(HALT, 0, 0, 0, 0));
    clear_state;  ri(1) := w32(11);  ri(2) := w32(22);
    run_program;
    check_stat("S cycles", cycles, fsel(9, 12));  check_stat("S forwards", fwds, fsel(1, 0));  check_stat("S stores", stores, 2);
    check_reg(3, w32(22), "S (youngest older store)");  check_mem(5, w32(22), "S (commit order)");

    -- S2: both stores are still queued behind a slow multiply when the load starts
    put(0, ins(MUL, 6, 1, 2, 0));  put(1, ins(ST, 0, 0, 1, 5));  put(2, ins(ST, 0, 0, 2, 5));  put(3, ins(LD, 3, 0, 0, 5));  put(4, ins(HALT, 0, 0, 0, 0));
    clear_state;  ri(1) := w32(11);  ri(2) := w32(22);
    run_program;
    check_stat("S2 cycles", cycles, fsel(12, 17));  check_stat("S2 forwards", fwds, fsel(1, 0));
    check_reg(3, w32(22), "S2 (youngest older store)");  check_reg(6, w32(242), "S2");  check_mem(5, w32(22), "S2");

    -- D: a store, a load and adds on the wrong path of a mispredicted branch
    put(0, ins(ADD, 3, 1, 2, 0));  put(1, ins(BNEZ, 0, 3, 0, 6));  put(2, ins(ST, 0, 0, 4, 7));  put(3, ins(LD, 6, 0, 0, 7));
    put(4, ins(ADD, 5, 6, 6, 0));  put(5, ins(ADD, 5, 5, 5, 0));  put(6, ins(LD, 7, 0, 0, 8));  put(7, ins(HALT, 0, 0, 0, 0));
    clear_state;  ri(1) := w32(1);  ri(2) := w32(2);  ri(4) := w32(99);  mi(7) := w32(5);  mi(8) := w32(31);
    run_program;
    check_stat("D cycles", cycles, 19);  check_stat("D flushes", flushes, 1);  check_stat("D committed", committed, 4);  check_stat("D stores", stores, 0);  check_stat("D loads", loads, 1);
    check_mem(7, w32(5), "D (the wrong-path store must not reach memory)");  check_reg(6, w32(0), "D");  check_reg(5, w32(0), "D");  check_reg(7, w32(31), "D");  check_reg(4, w32(99), "D");

    -- random programs vs a sequential interpreter
    sumc := 0;  sumf := 0;
    for p in 0 to 39 loop
      n := 24;
      for i in 0 to n-1 loop
        ty := rnd(10);  f_rd := rnd(8);  f_rs := rnd(8);  f_rt := rnd(8);  f_im := rnd(8);  f_tg := rnd(n);  f_op := rnd(3);     -- fixed draw order (same stream as the SystemVerilog testbench)
        if ty < 2 or ty = 9 then
          p_op(i) := f_op;  p_rd(i) := 2 + f_rd mod 6;  p_rs(i) := f_rs;  p_rt(i) := f_rt;  p_tg(i) := 0;
        elsif ty < 5 then
          p_op(i) := 5;  p_rd(i) := 2 + f_rd mod 6;  p_rs(i) := f_rs mod 2;  p_rt(i) := 0;  p_tg(i) := f_im;
        elsif ty < 8 then
          p_op(i) := 6;  p_rd(i) := 0;  p_rs(i) := f_rs mod 2;  p_rt(i) := f_rt;  p_tg(i) := f_im;
        elsif ty = 8 and i < n - 2 then
          p_op(i) := 3;  p_rd(i) := 0;  p_rs(i) := f_rs;  p_rt(i) := 0;  p_tg(i) := i + 2 + (f_tg mod (n - i - 1));
        else
          p_op(i) := f_op;  p_rd(i) := 2 + f_rd mod 6;  p_rs(i) := f_rs;  p_rt(i) := f_rt;  p_tg(i) := 0;
        end if;
        put(i, ins(p_op(i), p_rd(i), p_rs(i), p_rt(i), p_tg(i)));
      end loop;
      put(n, ins(HALT, 0, 0, 0, 0));
      for r in 0 to 7 loop
        ri(r) := w32(rnd(4));
        if r = 0 then ri(r) := w32(0); end if;                       -- r0 = 0 and r1 = 1 are never written: constant bases, heavy aliasing
        if r = 1 then ri(r) := w32(1); end if;
        g(r) := ri(r);
      end loop;
      for a in 0 to 63 loop mi(a) := w32(1000 + a);  gm(a) := mi(a); end loop;
      pcg := 0;  dyn := 0;  dbr := 0;  taken := 0;  steps := 0;  nld := 0;  nst := 0;       -- sequential interpreter
      while pcg < n and steps < 1000 loop
        steps := steps + 1;  dyn := dyn + 1;
        case p_op(pcg) is
          when 0 => g(p_rd(pcg)) := std_logic_vector(unsigned(g(p_rs(pcg))) + unsigned(g(p_rt(pcg))));  pcg := pcg + 1;
          when 1 => g(p_rd(pcg)) := std_logic_vector(unsigned(g(p_rs(pcg))) - unsigned(g(p_rt(pcg))));  pcg := pcg + 1;
          when 2 => g(p_rd(pcg)) := std_logic_vector(resize(unsigned(g(p_rs(pcg))) * unsigned(g(p_rt(pcg))), 32));  pcg := pcg + 1;
          when 5 => ad := (to_integer(unsigned(g(p_rs(pcg))(5 downto 0))) + p_tg(pcg)) mod 64;  g(p_rd(pcg)) := gm(ad);  nld := nld + 1;  pcg := pcg + 1;
          when 6 => ad := (to_integer(unsigned(g(p_rs(pcg))(5 downto 0))) + p_tg(pcg)) mod 64;  gm(ad) := g(p_rt(pcg));  nst := nst + 1;  pcg := pcg + 1;
          when others =>
            dbr := dbr + 1;
            if unsigned(g(p_rs(pcg))) /= 0 then taken := taken + 1;  pcg := p_tg(pcg); else pcg := pcg + 1; end if;
        end case;
      end loop;
      dyn := dyn + 1;                                                 -- the HALT
      run_program;
      for r in 0 to 7 loop check_reg(r, g(r), "random " & integer'image(p)); end loop;
      for a in 0 to 63 loop check_mem(a, gm(a), "random " & integer'image(p)); end loop;
      check_stat("random committed", committed, dyn);
      check_stat("random loads", loads, nld);
      check_stat("random stores", stores, nst);
      check_stat("random branches", branches, dbr);
      check_stat("random flushes", flushes, taken);
      sumc := sumc + to_integer(cycles);  sumf := sumf + to_integer(fwds);
    end loop;
    rand_cycles <= sumc;  rand_fwd <= sumf;
    errors <= errs;  done <= '1';
    wait;
  end process;
end architecture;

library ieee;
use ieee.std_logic_1164.all;
use std.env.all;

entity tb_tomasulo_lsq is end entity;

architecture sim of tb_tomasulo_lsq is
  signal clk : std_logic := '0';
  signal rst_n : std_logic := '0';
  signal d0, d1 : std_logic;
  signal er0, er1, rc0, rc1, rf0, rf1 : integer;
begin
  clk <= not clk after 5 ns;
  fw   : entity work.lsq_harness generic map (true)  port map (clk, rst_n, d0, er0, rc0, rf0);
  nofw : entity work.lsq_harness generic map (false) port map (clk, rst_n, d1, er1, rc1, rf1);

  process
    variable extra : integer := 0;
  begin
    wait until falling_edge(clk);  wait until falling_edge(clk);
    rst_n <= '1';
    wait until d0 = '1' and d1 = '1';
    wait for 1 ns;
    if rf1 /= 0 then extra := extra + 1; report "FAIL: forwarding disabled but " & integer'image(rf1) & " loads were forwarded" severity error; end if;
    if not (rc0 < rc1) then
      extra := extra + 1;
      report "FAIL: store-to-load forwarding (" & integer'image(rc0) & " cycles) is not faster than waiting for the commit (" & integer'image(rc1) & ")" severity error;
    else
      report "40 random programs: " & integer'image(rc0) & " cycles with store-to-load forwarding (" & integer'image(rf0) & " loads forwarded), " & integer'image(rc1) & " cycles without";
    end if;
    if er0 + er1 + extra = 0 then report "TEST PASSED";
    else report "TEST FAILED: " & integer'image(er0 + er1 + extra) & " errors" severity error;
    end if;
    finish;
  end process;
end architecture;
