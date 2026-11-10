# Speculative Tomasulo with a Load/Store Queue

**Topic:** Chapter 3 (dynamic scheduling and hardware speculation with memory: memory dependences, store-to-load forwarding) · **Editions:** the 3rd edition onward (ROB form of Tomasulo with a load/store buffer) ·
**Files:** [`sv/tomasulo_lsq.sv`](sv/tomasulo_lsq.sv), [`vhdl/tomasulo_lsq.vhd`](vhdl/tomasulo_lsq.vhd)

## Background

[`tomasulo_spec`](../tomasulo_spec) schedules register-to-register code and speculates past branches. Real programs also use **memory**, and memory brings dependences the register
rename table cannot see: a load and an earlier store may touch the *same address*, and the address is only known once the base register has been computed. Two rules make speculative
out-of-order memory safe:

1. **Stores update memory only at commit.** A store on a mispredicted path must never change memory, so the store waits in a **store buffer** (here: the load/store queue) until it reaches the head of the ROB.
2. **Loads must respect older stores.** A load may bypass older stores only if it is sure they are to *other* addresses. If the youngest older store to the same address has its data, the load takes
   that value directly (**store-to-load forwarding**) without waiting for memory; if the store's address is not yet known the load must (conservatively) wait.

## Design

The core is that of `tomasulo_spec` (3 add + 2 multiply stations, 8-entry ROB, one CDB, branches predicted not taken, commit-time recovery) plus data memory (64 words) and two instructions:

| Instruction | |
|---|---|
| `LD rd, [rs + imm]` | `rd = mem[(rs + imm) mod 64]` |
| `ST rt, [rs + imm]` | `mem[(rs + imm) mod 64] = rt` at commit |

Both go to an 8-entry **load/store queue** (LSQ) in program order, freed at commit, with the same operand-capture mechanism as the reservation stations (base register, and for stores the data).

* **Address unit:** one per cycle, oldest queue entry whose base register is available; the address is visible one edge later.
* **Stores:** "done" in the ROB one edge after both address and data are known; the memory write happens at the store's commit edge, which also frees its queue entry.
* **Loads:** one load unit. The oldest load whose address is known starts if no older store has an unknown address; if the youngest older store to the same address has its data it supplies the
  value (`FWD_LAT` = 1 cycle); if the data is missing the load waits; otherwise it reads memory (`LD_LAT` = 3 cycles). Loads broadcast on the CDB (priority add, multiply, load).
  Generic `FORWARD = false` disables forwarding: the load then waits for the older store to commit.
* **Misprediction** flushes the LSQ together with the ROB, stations and rename table.

Timing (edge numbers, first instruction issues at edge 1, `i` = issue edge, base register ready, `L` = 3 or 1): load: address `i+1`, start `i+2`, done `i+2+L`, CDB `i+3+L`, commit `i+4+L`;
store: address `i+1`, done `i+2`, commit `i+3`.

## Verification

Cycle counts derived by hand from the rules above (cycles = edge at which `HALT` commits):

| Program | with forwarding | without |
|---------|----------------:|--------:|
| `A`: `LD r1,[4]; HALT` | 9 | 9 |
| `B`: `ST r2,[5]; LD r1,[5]` - the store is still queued when the load starts | **8** | 11 |
| `B2`: `MUL r3; ST r3,[5]; LD r4,[5]` - the load waits for the store's data from the multiplier | **12** | 16 |
| `S`: two stores to `[5]`, then `LD [5]` - must read the **younger** store's value | **9** | 12 |
| `S2`: like `S`, but a slow `MUL` keeps both stores queued - the load must pick the **younger** one among two in the queue | **12** | 17 |
| `C`: `MUL r3; ST r5,[r3]; LD r4,[9]` - the store's address is late, the load of a *different* address must wait | 15 | 15 |
| `C0`: `ST r5,[6]; LD r4,[9]` - same without the late address | 10 | 10 |

Program `C` shows the cost of conservative disambiguation: without the late store address `C0` takes 10 cycles, with it 15.

**Wrong path (`D`):** `ADD r3=r1+r2; BNEZ r3,6; ST r4,[7]; LD r6,[7]; ADD; ADD; LD r7,[8]; HALT` - the branch is taken but predicted not taken, so a store, a load and two adds execute speculatively.
After the flush at edge 10 the correct-path `LD r7` issues at 11 and commits at 18, `HALT` at 19: asserted **19 cycles, 1 flush, 4 commits, 0 stores, 1 load**, `mem[7]` still holds its original value,
`r6` and `r5` are untouched.

**40 random programs** (ADD/SUB/MUL, loads, stores, forward branches; load/store bases are the constants `r0 = 0` and `r1 = 1` with offsets 0-7 so that addresses alias heavily) are compared
with a sequential interpreter: all registers, all 64 memory words, committed instructions, loads, stores, branches and flushes (= taken branches). With forwarding the 40 programs take **1752 cycles
(15 loads forwarded)**, without **1798**; both languages produce identical numbers.

Mutation checks (each fails the testbench in both languages): ignoring older stores with unknown addresses, forwarding from the *oldest* instead of the youngest matching store, forwarding before the
store's data has arrived, letting a store write memory before it commits; (SystemVerilog also: not flushing the LSQ on a misprediction).

## Run it

From this module directory, run the SystemVerilog testbench:

```bash
iverilog -g2012 -s tb_tomasulo_lsq -o /tmp/tb_tomasulo_lsq.vvp sv/tomasulo_lsq.sv sv/tb_tomasulo_lsq.sv
vvp /tmp/tb_tomasulo_lsq.vvp
```

For VHDL-2008:

```bash
mkdir -p /tmp/ghdl-tomasulo_lsq
ghdl -a --std=08 --workdir=/tmp/ghdl-tomasulo_lsq vhdl/tomasulo_lsq.vhd vhdl/tb_tomasulo_lsq.vhd
ghdl -e --std=08 --workdir=/tmp/ghdl-tomasulo_lsq tb_tomasulo_lsq
ghdl -r --std=08 --workdir=/tmp/ghdl-tomasulo_lsq tb_tomasulo_lsq
```
## Exercises

1. Add **memory dependence speculation**: let a load bypass older stores with unknown addresses, and detect and repair the violation when the store's address arrives (a *memory order violation* flushes from the load).
2. Add a **second load unit** or make the load unit pipelined, and measure the random programs again.
3. Let a load **forward partially** (byte stores / word loads) and see which cases force the load to wait.
4. Replace the 3-cycle memory by a direct-mapped cache with a miss penalty (see a cache with a miss penalty) so that loads complete out of order.
5. Resolve branch mispredictions when the branch *executes* (see exercise 2 of [`tomasulo_spec`](../tomasulo_spec)) and squash only the younger LSQ entries.

## Textbook reference

John L. Hennessy and David A. Patterson, *Computer Architecture: A Quantitative Approach*, Chapter 3, “Instruction-Level Parallelism and Its Exploitation.”
