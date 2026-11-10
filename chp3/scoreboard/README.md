# The CDC 6600 Scoreboard

**Topic:** Chapter 3 (dynamic scheduling) · **Editions:** a core topic of the early editions' pipelining/ILP chapters, kept as a stepping stone to
Tomasulo in the later ones · **Files:** [`sv/scoreboard.sv`](sv/scoreboard.sv), [`vhdl/scoreboard.vhd`](vhdl/scoreboard.vhd)

## Background

The first dynamically scheduled machine, the CDC 6600 (1964), let instructions execute **out of order** whenever their operands were ready, using a central
bookkeeping structure: the **scoreboard**. Every instruction passes through four steps:

| Step | Waits until | Hazard handled |
|------|-------------|----------------|
| **Issue** (in order) | a functional unit is free *and* no pending write targets the same destination | structural, **WAW** |
| **Read operands** | both source registers hold their final values | **RAW** |
| **Execute** | `LAT` cycles | |
| **Write result** | no *older* instruction still has to read the old value of the destination | **WAR** |

Per functional unit the scoreboard keeps the textbook's fields -- `Busy, Op, Fi, Fj, Fk, Qj, Qk, Rj, Rk` -- and one **result register status** table
saying which unit will write each register. There is **no renaming** (so WAW and WAR *stall* instead of disappearing) and **no forwarding bus** (a consumer
cannot start until the producer has written the register file and one more cycle has passed).

## Design

* Units: **FU0** = add/sub (2-cycle), **FU1, FU2** = multipliers (4-cycle); `ISA: ADD, SUB, MUL`, `rd = rs op rt`, 8 registers, in-order issue 1/cycle.
* `issue_ready` is low on a structural hazard (no free unit of the right kind) or on WAW (`res_busy[rd]`).
* If an instruction issues on the very edge its producer writes, the source is treated as ready -- otherwise the producer would never wake it (it is not in the
  table yet). The timing is identical to being woken one edge later.
* Observation outputs `read_go`, `write_go`, `fu_seq` expose, one cycle ahead, which instruction reads or writes at the next edge, so a testbench can rebuild each
  instruction's timeline.

**Exact timing** (clock edges; `I` = issue edge):

```
read  R >= I+1,  R >= (edge a producer wrote) + 1
execute edges R+1 .. R+LAT       result ready after edge R+LAT
write W >= R+LAT+1,  W >= (edge every older reader of rd read it) + 1
```

## Verification

**A hand-derived schedule with all four hazards** (initial `r2 = 3, r3 = 4, r5 = 10`):

```
J0: MUL r1 = r2*r3    J1: MUL r4 = r1*r5    J2: ADD r5 = r2+r3    J3: ADD r6 = r5+r4    J4: MUL r4 = r2*r3
```

| Instr | Issue | Read | Write | Why |
|-------|------:|-----:|------:|-----|
| J0 | 1 | 2 | 7 | plain 4-cycle multiply |
| J1 | 2 | **8** | 13 | **RAW**: waits for J0's `r1` (written at 7, readable at 8) |
| J2 | 3 | 4 | **9** | **WAR**: it would write `r5` at 7, but J1 has not yet read the old `r5` (it reads at 8), so it must wait until 9 |
| J3 | **10** | 14 | 17 | **structural**: the only adder is busy with J2 until edge 9; then **RAW** on `r4` from J1 (written 13) |
| J4 | **14** | 15 | 20 | **WAW**: `r4` is still owed to J1 until edge 13 |

Final registers: `r1 = 12`, `r4 = 12` (J4 last), `r5 = 7`, `r6 = 127` (J3 saw J2's `r5 = 7` and J1's `r4 = 120`). The testbench asserts every one of the 15
numbers in the table, and the RTL reproduces them exactly -- an independent agreement between the hand derivation and the design.

**Random programs.** 30 programs x 100 instructions over 8 registers (8 844 issue-stall cycles in each language). Besides matching a sequential golden register
file, the testbench logs each instruction's issue/read/write edges and checks **exact** rules derived from the spec -- not just inequalities:

* `read = max(issue + 1, 1 + write of the producer of each source)`
* `write = max(read + LAT + 1, 1 + read of every older reader of the destination)`
* issue in order; never before an older writer of the same register has written (WAW); at most 0 older adds / 1 older multiply in flight (structural).

Removing the WAR check or the WAW check makes both the directed and the random tests fail.

## Scoreboard vs. Tomasulo

| | Scoreboard (this example) | [Tomasulo + ROB](../tomasulo_rob) |
|---|---|---|
| WAW / WAR | **stall** (issue / write result) | **eliminated** by renaming |
| operand delivery | read from the register file after the producer writes | broadcast on the CDB the cycle the value exists |
| dependent instruction executes from | producer write + 2 (read operands at +1, then execute) | broadcast + 1 |
| waiting instructions occupy | the functional unit (structural hazards bite) | a reservation station |
| completion order seen by software | out of order (imprecise exceptions) | in order via the ROB (precise) |
| cost | one small table | tags, comparators on a bus, a reorder buffer |

On the sample program above the scoreboard loses cycles to a WAR stall (J2), a structural stall (J3) and a WAW stall (J4). Renaming removes the WAR and WAW
stalls outright; reservation stations let instructions issue while a unit is busy, so the structural stall no longer blocks issue (up to the number of stations).

## Run it

From this module directory, run the SystemVerilog testbench:

```bash
iverilog -g2012 -s tb_scoreboard -o /tmp/tb_scoreboard.vvp sv/scoreboard.sv sv/tb_scoreboard.sv
vvp /tmp/tb_scoreboard.vvp
```

For VHDL-2008:

```bash
mkdir -p /tmp/ghdl-scoreboard
ghdl -a --std=08 --workdir=/tmp/ghdl-scoreboard vhdl/scoreboard.vhd vhdl/tb_scoreboard.vhd
ghdl -e --std=08 --workdir=/tmp/ghdl-scoreboard tb_scoreboard
ghdl -r --std=08 --workdir=/tmp/ghdl-scoreboard tb_scoreboard
```
## Exercises

1. Add a **divider** with a long latency and watch the structural and WAR stalls grow.
2. Add more adders (`FU0, FU3`) and recompute the sample schedule: which hazards disappear and which do not?
3. Run the same 5-instruction program on [`tomasulo_rob`](../tomasulo_rob) and compare the total cycles.
4. Add **forwarding**: let a consumer read the operand on the same edge the producer writes, and measure the saved cycles.

## Textbook reference

John L. Hennessy and David A. Patterson, *Computer Architecture: A Quantitative Approach*, Chapter 3, “Instruction-Level Parallelism and Its Exploitation.”
