# Tomasulo with Hardware Speculation

**Topic:** Chapter 3 (hardware-based speculation) · **Editions:** the speculative, reorder-buffer form of Tomasulo is the one the 3rd edition and later present ·
**Files:** [`sv/tomasulo_spec.sv`](sv/tomasulo_spec.sv), [`vhdl/tomasulo_spec.vhd`](vhdl/tomasulo_spec.vhd)

## Background

[`tomasulo_rob`](../tomasulo_rob) finds parallelism *within a straight line* of code. But roughly one instruction in five is a branch, and an instruction after a
branch cannot start until the branch is resolved -- unless the machine **speculates**: it guesses the branch outcome, keeps fetching and executing down the
guessed path, and undoes the damage if the guess was wrong. Three pieces make that safe:

1. **Prediction** at fetch time (see [`branch_predictor`](../branch_predictor)).
2. **Results held back.** Speculative instructions may execute and broadcast on the CDB, but their results stay in the **reorder buffer**; nothing reaches the
   architectural registers until the instruction **commits**, in order.
3. **Recovery.** If a branch commits and its outcome disagrees with the prediction, everything younger is wrong. Because only committed state is architectural, recovery
   is simply "throw away the ROB and the rename table and restart fetch at the right address". This is why the ROB gives *precise* state.

```
cost of a misprediction  ~  (cycles from the branch's fetch to its commit) + refill,  paid again for every wrong guess
```

## Design

The execution core is that of `tomasulo_rob` (3 add stations + 2 multiply stations, one add unit of 2 cycles, one multiplier of 4, one CDB, 8-entry ROB), with a fetch
unit and a conditional branch added:

| ISA (32-bit words, 8 registers) | |
|---|---|
| `ADD / SUB / MUL rd, rs, rt` | as before |
| `BNEZ rs, target` | branch if `rs != 0`; occupies a ROB entry and an add-type station (the add unit passes `rs` through: nonzero = taken) |
| `HALT` | needs no execution; stops the front end; commits, then `halted` goes high |

* **Prediction (generic `PREDICT_BTFN`).** `false`: every branch predicted *not taken*. `true`: **backward taken / forward not taken** (loops predicted to repeat). The prediction and both
  possible next PCs are stored in the ROB entry.
* **Issue** now follows the predicted path: `pc <= predicted_taken ? target : pc + 1`.
* **Commit-time recovery.** When the branch at the ROB head is done and `(value != 0) != predicted`, the ROB, all stations, both units and the rename table are cleared, fetch restarts at the
  correct address, and `stat_squashed` is increased by the number of discarded instructions. Nothing issues on the flush edge, so the correct-path instruction issues one edge later.
* **Wrong-path safety.** A wrong-path instruction can execute, even broadcast, and wrong-path instructions can even include a `HALT`, but none of it can touch a register: only commit writes registers.
* **Oldest-first selection.** Each functional unit starts the *oldest* ready station (smallest distance from the ROB head), not simply the lowest-numbered one. See the note below.
* Statistics: `stat_cycles`, `stat_committed`, `stat_branches`, `stat_flushes` (mispredictions), `stat_squashed`.

### Why oldest-first matters once you speculate

My first version picked the lowest-numbered ready station, as `tomasulo_rob` does. On a six-iteration loop the *better* predictor ran **slower** (106 cycles against 85) even though it
mispredicted 1 branch instead of 5. The cause: the loop is bound by the single non-pipelined adder, and after the final (mispredicted) loop branch the machine had speculatively issued a whole extra
iteration (7 instructions, all squashed). Those younger, doomed instructions sat in lower-numbered stations and **took the adder away from the older branch that decides whether they are needed**,
delaying its resolution. Picking the oldest ready instruction fixes it (squashed instructions fell from 7 to 2, cycles from 106 to 76). Speculation spends resources on guesses; the scheduler must make
sure the guesses never delay the instructions that settle them.

## Verification

**A hand-derived mispredicted branch (program M, `r1 = 1, r2 = 2, r5 = 10`):**

```
0 ADD r3 = r1 + r2     1 BNEZ r3, 4     2 ADD r4 = r4 + r5     3 ADD r4 = r4 + r5     4 ADD r6 = r1 + r1     5 HALT
```

| Edge | Event |
|-----:|-------|
| 1 / 2 | `ADD r3` and `BNEZ` issue; the branch is predicted *not taken* and fetch runs on (instructions 2, 3 issue speculatively) |
| 5 | CDB broadcasts `r3`; the branch captures it |
| 6 | `ADD r3` commits; the branch starts on the adder |
| 9 | CDB broadcasts the branch's condition (`r3 = 3`, nonzero: taken) |
| **10** | the branch reaches the ROB head: outcome *taken* ≠ prediction *not taken* → **flush**, `pc <- 4` |
| 11 | `ADD r6` issues on the correct path |
| 12 | `HALT` issues |
| 16 / 17 | `ADD r6` commits; `HALT` commits |

Asserted exactly: **17 cycles, 4 commits, 1 branch, 1 flush, 2 squashed** (the two wrong-path `ADD`s; the add-station pool is full, so the rest of the fall-through stream had not issued yet), and the
wrong-path `ADD`s **never reach `r4`** (still 0). The same program with `r1 = r2 = 0` makes the branch *not taken*: prediction correct, 6 commits, 0 flushes, `r4 = 20`.

**A counted loop** (`ADD r3 += r1; SUB r1 -= r5; BNEZ r1, loop`, `r1 = 6`): `r3 = 21`, 19 commits, 6 branches. Always-not-taken mispredicts 5 times (**5 flushes, 85 cycles**);
backward-taken mispredicts only the exit (**1 flush, 76 cycles**). The testbench requires the better predictor to be strictly faster.

**40 random programs** of 24 instructions (ADD/SUB/MUL and forward `BNEZ`s over 8 registers with small values so branches go both ways), each checked against a **sequential interpreter** in the testbench:
all eight final registers, the number of committed instructions, the number of committed branches, and -- exactly -- `flushes = number of taken branches` (every forward branch is predicted not taken under
both policies). Mutations that leave stale rename entries after a flush, or let a branch write a register, fail the random tests.

## Run it

From this module directory, run the SystemVerilog testbench:

```bash
iverilog -g2012 -s tb_tomasulo_spec -o /tmp/tb_tomasulo_spec.vvp sv/tomasulo_spec.sv sv/tb_tomasulo_spec.sv
vvp /tmp/tb_tomasulo_spec.vvp
```

For VHDL-2008:

```bash
mkdir -p /tmp/ghdl-tomasulo_spec
ghdl -a --std=08 --workdir=/tmp/ghdl-tomasulo_spec vhdl/tomasulo_spec.vhd vhdl/tb_tomasulo_spec.vhd
ghdl -e --std=08 --workdir=/tmp/ghdl-tomasulo_spec tb_tomasulo_spec
ghdl -r --std=08 --workdir=/tmp/ghdl-tomasulo_spec tb_tomasulo_spec
```
## Exercises

1. Add a **1-bit or 2-bit branch history table** from [`branch_predictor`](../branch_predictor) instead of the static policies, and rerun the loop.
2. Resolve mispredictions **when the branch executes** instead of at commit (flush only the entries *younger* than the branch using ROB tags) and measure the cycles saved.
3. Add `LOAD`/`STORE` with a store buffer so memory too is only updated at commit.
4. Make the adder pipelined (initiation interval 1) and see how much the loop speeds up -- the single non-pipelined adder is the bottleneck here.
5. Plot cycles against the misprediction rate for random programs under both policies.

## Textbook reference

John L. Hennessy and David A. Patterson, *Computer Architecture: A Quantitative Approach*, Chapter 3, “Instruction-Level Parallelism and Its Exploitation.”
