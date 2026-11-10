-- Table of saturating counters indexed by PC bits, optionally XORed with global history.
--   HIST_W = 0, CTR_BITS = 1 : 1-bit predictor        (1st/2nd edition branch history table)
--   HIST_W = 0, CTR_BITS = 2 : 2-bit bimodal predictor
--   HIST_W > 0, CTR_BITS = 2 : gshare (global-history correlating predictor, 3rd+ editions)
-- predict_taken is combinational from `pc`.  Table/history are updated when upd_valid is
-- high at a clock edge, i.e. at branch resolution (non-speculative history).
-- HIST_W must be <= IDX_W.  Counters start weakly not-taken.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity branch_predictor is
  generic (IDX_W : positive := 8; CTR_BITS : positive := 2; HIST_W : natural := 0);
  port (
    clk, rst_n    : in  std_logic;
    pc            : in  std_logic_vector(31 downto 0);
    predict_taken : out std_logic;
    upd_valid     : in  std_logic;
    upd_pc        : in  std_logic_vector(31 downto 0);
    upd_taken     : in  std_logic
  );
end entity;

architecture rtl of branch_predictor is
  constant ENTRIES  : natural := 2 ** IDX_W;
  constant CTR_MAX  : natural := 2 ** CTR_BITS - 1;
  constant CTR_INIT : natural := 2 ** (CTR_BITS - 1) - 1;
  constant HW       : positive := maximum(HIST_W, 1);
  type ctr_t is array (0 to ENTRIES-1) of natural range 0 to CTR_MAX;
  signal ctr  : ctr_t := (others => CTR_INIT);
  signal hist : std_logic_vector(HW-1 downto 0) := (others => '0');

  function index(a : std_logic_vector(31 downto 0); h : std_logic_vector) return natural is
    variable x : unsigned(IDX_W-1 downto 0);
  begin
    x := unsigned(a(IDX_W+1 downto 2));
    if HIST_W > 0 then x := x xor resize(unsigned(h), IDX_W); end if;
    return to_integer(x);
  end function;
begin
  predict_taken <= '1' when ctr(index(pc, hist)) >= 2 ** (CTR_BITS - 1) else '0';

  process (clk, rst_n)
    variable i : natural;
  begin
    if rst_n = '0' then
      ctr  <= (others => CTR_INIT);
      hist <= (others => '0');
    elsif rising_edge(clk) then
      if upd_valid = '1' then
        i := index(upd_pc, hist);
        if upd_taken = '1' then
          if ctr(i) /= CTR_MAX then ctr(i) <= ctr(i) + 1; end if;
        else
          if ctr(i) /= 0 then ctr(i) <= ctr(i) - 1; end if;
        end if;
        if HIST_W > 0 then hist <= hist(HW-2 downto 0) & upd_taken; end if;
      end if;
    end if;
  end process;
end architecture;
