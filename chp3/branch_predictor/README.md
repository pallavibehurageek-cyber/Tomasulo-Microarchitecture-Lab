# Dynamic Branch Prediction

**Topic:** Chapter 3 (Instruction-Level Parallelism) · **Editions:** 1-7 (2-bit prediction from the first edition;
the book's coverage of correlating and tournament predictors grows in the later ones) ·
**Files:** [`sv/branch_predictor.sv`](sv/branch_predictor.sv), [`vhdl/branch_predictor.vhd`](vhdl/branch_predictor.vhd)

## Background

A pipeline that resolves branches late pays a
penalty for every taken branch. The deeper and wider the machine, the more that costs, so processors
**predict** the outcome at fetch time and recover only on a misprediction.

Dynamic predictors learn from history:

* **1-bit predictor** -- remember the last outcome. A loop branch that is taken *n-1* times and then falls
  through is mispredicted *twice* per loop execution (once at the exit, once on re-entry).
* **2-bit saturating counter** -- needs two wrong guesses in a row to change its mind, so the loop is
  mispredicted only once.

  ```
  00 strongly NT <-> 01 weakly NT <-> 10 weakly T <-> 11 strongly T      predict taken when the MSB is 1
  ```
* **Correlating / global-history predictors** -- a branch's outcome often depends on *other* recent
  branches. **gshare** indexes the counter table with `PC xor global_history`, letting one static branch use
  different counters in different contexts, which learns patterns (e.g. strict alternation) that
  per-branch counters cannot.

## Design

One parameterized module covers all three:

| `CTR_BITS` | `HIST_W` | Predictor |
|-----------:|---------:|-----------|
| 1 | 0 | 1-bit |
| 2 | 0 | 2-bit bimodal |
| 2 | = `IDX_W` | gshare |

* The table has `2^IDX_W` counters indexed by `pc[IDX_W+1:2]` (xor-ed with the history for gshare).
* `predict_taken` is combinational from `pc`. The table and history are updated by `upd_valid/upd_pc/upd_taken`
  at **branch resolution**, so the history holds committed outcomes only.
* Counters reset to *weakly not-taken*. `HIST_W` must be `<= IDX_W`.

## Verification

All three variants (8-bit index) see the same 60-branch trace:

| Branch | PC | Pattern per outer iteration (10 iterations) |
|--------|----|----------------------------------------------|
| inner loop  | `0x100` | T, T, T, N  (4 executions) |
| alternating | `0x400` | T on even iterations, N on odd |
| outer loop  | `0x200` | T for 9 iterations, N on the last |

Expected mispredictions come from an **independent Python model** of the same specification:

| Predictor | Mispredicts / 60 | Accuracy |
|-----------|-----------------:|---------:|
| 1-bit   | 32 | 47 % |
| 2-bit   | 23 | 62 % |
| gshare  | 17 | 72 % |

The ordering is the lesson: hysteresis helps the loop branches, and global history is what finally starts to
learn the alternating branch.

## Run it

From this module directory, run the System## Exercises

1. Sweep `IDX_W` from 2 to 10 and plot mispredictions against table size to study aliasing.
2. Connect the predictor to a pipeline's branch-resolution signals and measure the effect on CPI.

## Textbook reference

John L. Hennessy and David A. Patterson, *Computer Architecture: A Quantitative Approach*, Chapter 3, “Instruction-Level Parallelism and Its Exploitation.”
