# Chapter 3 — Instruction-Level Parallelism and Its Exploitation

This chapter collection studies techniques that expose instruction-level parallelism (ILP) in a processor. The examples focus on hardware techniques that discover and schedule work at run time: branch prediction, dynamic scheduling, register renaming, reorder buffers, speculation, and memory ordering.

**Textbook reference:** *Computer Architecture: A Quantitative Approach* by **John L. Hennessy and David A. Patterson**, Chapter 3, “Instruction-Level Parallelism and Its Exploitation.”

## Concepts and examples

| Module | What it demonstrates |
|---|---|
| [branch_predictor](branch_predictor/) | 1-bit, 2-bit, and gshare dynamic branch prediction using a common branch trace |
| [scoreboard](scoreboard/) | CDC 6600 scoreboard scheduling, including RAW, WAR, WAW, and structural hazards without register renaming |
| [tomasulo_rob](tomasulo_rob/) | Reservation stations, common data bus, register renaming, reorder buffer, out-of-order completion, and in-order commit |
| [tomasulo_spec](tomasulo_spec/) | Tomasulo scheduling with branch prediction, speculative execution, commit-time recovery, and wrong-path protection |
| [tomasulo_lsq](tomasulo_lsq/) | Speculative loads and stores, memory disambiguation, store-to-load forwarding, and commit-time memory updates |

## Languages and verification

Each example is provided in **SystemVerilog** and **VHDL**, with a testbench for each language. The module-level READMEs describe the design, timing model, directed verification cases, random-program checks where applicable, and simulator commands.

## Suggested reading order

1. Start with the scoreboard to understand dynamic scheduling without register renaming.
2. Study Tomasulo with a reorder buffer to see reservation stations, tags, result broadcast, and precise in-order commit.
3. Read the branch predictor example, then the speculative Tomasulo design to connect prediction with recovery.
4. Finish with the load/store queue, which extends speculative execution to memory operations.
