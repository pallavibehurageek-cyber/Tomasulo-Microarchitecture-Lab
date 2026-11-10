# Tomasulo Microarchitecture Lab

A dual-language RTL study lab focused on instruction-level parallelism (ILP), dynamic scheduling, and speculative execution.

The examples cover branch prediction, the CDC 6600 scoreboard, Tomasulo scheduling with a reorder buffer, branch speculation, and load/store ordering. Each design is provided in SystemVerilog and VHDL with a corresponding self-checking testbench.

## Included designs

| Design | Main concepts |
|---|---|
| [Dynamic Branch Prediction](chp3/branch_predictor/) | 1-bit, 2-bit saturating-counter, and gshare predictors |
| [CDC 6600 Scoreboard](chp3/scoreboard/) | In-order issue, out-of-order execution, RAW/WAR/WAW and structural hazards |
| [Tomasulo + ROB](chp3/tomasulo_rob/) | Reservation stations, register renaming, common data bus, reorder buffer, in-order commit |
| [Speculative Tomasulo](chp3/tomasulo_spec/) | Branch prediction, speculative execution, commit-time recovery, wrong-path safety |
| [Tomasulo + LSQ](chp3/tomasulo_lsq/) | Load/store queue, memory disambiguation, store-to-load forwarding, commit-time stores |

## Repository layout

- README.md
- chp3/README.md
- chp3/branch_predictor/ — SystemVerilog, VHDL, testbenches, and module documentation
- chp3/scoreboard/ — SystemVerilog, VHDL, testbenches, and module documentation
- chp3/tomasulo_rob/ — SystemVerilog, VHDL, testbenches, and module documentation
- chp3/tomasulo_spec/ — SystemVerilog, VHDL, testbenches, and module documentation
- chp3/tomasulo_lsq/ — SystemVerilog, VHDL, testbenches, and module documentation

Each module directory contains its own README, RTL source files, and testbench files. See [Chapter 3](chp3/README.md) for the topic overview and module relationships.

## Tools

- SystemVerilog: Icarus Verilog (iverilog and vvp)
- VHDL: GHDL with VHDL-2008 support

The module READMEs include commands to compile and run each language's testbench from that module's directory.

## Textbook reference

The concepts are based on Chapter 3, “Instruction-Level Parallelism and Its Exploitation,” in *Computer Architecture: A Quantitative Approach* by **John L. Hennessy and David A. Patterson**.

## Scope

This repository is organized as a focused study and experimentation lab. The documented cycle counts and expected testbench outcomes are design-specific; consult each module's verification section and run its testbench when evaluating the implementation.
