-- Three predictors see the same 60-branch trace (inner loop T,T,T,N; an alternating branch;
-- an outer-loop branch).  Expected misprediction counts come from an independent Python model.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.all;

entity tb_branch_predictor is end entity;

architecture sim of tb_branch_predictor is
  signal clk : std_logic := '0';
  signal rst_n : std_logic := '0';
  signal pc, upd_pc : std_logic_vector(31 downto 0) := (others => '0');
  signal upd_valid, upd_taken : std_logic := '0';
  signal p1, p2, pg : std_logic;
begin
  clk <= not clk after 5 ns;

  one_bit : entity work.branch_predictor generic map (8, 1, 0)
    port map (clk, rst_n, pc, p1, upd_valid, upd_pc, upd_taken);
  two_bit : entity work.branch_predictor generic map (8, 2, 0)
    port map (clk, rst_n, pc, p2, upd_valid, upd_pc, upd_taken);
  gshare  : entity work.branch_predictor generic map (8, 2, 8)
    port map (clk, rst_n, pc, pg, upd_valid, upd_pc, upd_taken);

  process
    variable mis1, mis2, misg, n, errors : natural := 0;

    procedure branch(a : natural; taken : boolean) is
      variable t : std_logic;
    begin
      if taken then t := '1'; else t := '0'; end if;
      wait until falling_edge(clk);
      pc <= std_logic_vector(to_unsigned(a, 32));
      wait for 1 ns;
      if p1 /= t then mis1 := mis1 + 1; end if;
      if p2 /= t then mis2 := mis2 + 1; end if;
      if pg /= t then misg := misg + 1; end if;
      n := n + 1;
      upd_pc <= std_logic_vector(to_unsigned(a, 32));
      upd_taken <= t;  upd_valid <= '1';
      wait until falling_edge(clk);
      upd_valid <= '0';
    end procedure;
  begin
    wait until falling_edge(clk);  wait until falling_edge(clk);
    rst_n <= '1';
    for i in 0 to 9 loop
      for k in 0 to 3 loop branch(16#100#, k < 3); end loop;   -- inner loop branch
      branch(16#400#, i mod 2 = 0);                              -- alternating branch
      branch(16#200#, i < 9);                                    -- outer loop branch
    end loop;
    report "branches=" & integer'image(n) & " mispredicts: 1-bit=" & integer'image(mis1) &
           " 2-bit=" & integer'image(mis2) & " gshare=" & integer'image(misg);
    if n    /= 60 then errors := errors + 1; report "FAIL trace length" severity error; end if;
    if mis1 /= 32 then errors := errors + 1; report "FAIL 1-bit" severity error; end if;
    if mis2 /= 23 then errors := errors + 1; report "FAIL 2-bit" severity error; end if;
    if misg /= 17 then errors := errors + 1; report "FAIL gshare" severity error; end if;
    if errors = 0 then report "TEST PASSED";
    else report "TEST FAILED: " & integer'image(errors) & " errors" severity error;
    end if;
    finish;
  end process;
end architecture;
