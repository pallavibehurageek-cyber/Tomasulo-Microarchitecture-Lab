`timescale 1ns/1ps
// Directed programs (hand-derived), a counted loop under both predictors, and 40 random forward-branch programs checked against a
// sequential interpreter.  Policy 0 = always not taken, policy 1 = backward taken / forward not taken.
//
//  M (mispredicted forward branch):   0 ADD r3=r1+r2 | 1 BNEZ r3,4 | 2 ADD r4=r4+r5 | 3 ADD r4=r4+r5 | 4 ADD r6=r1+r1 | 5 HALT     (r1=1 r2=2 r5=10)
//     I0 issues at edge 1 and commits at 6.  The branch (issue 2) waits for r3 (CDB edge 5), executes 6..8, broadcasts at 9, reaches the head of
//     the ROB and commits at edge 10 -> mispredicted (predicted not taken, r3 = 3): flush, pc <- 4.  Nothing issues on edge 10; ADD r6 issues at 11
//     (execute 12-13, done 14, CDB 15, commit 16) and HALT issues at 12 and commits at 17.  So: 17 cycles, 4 commits, 1 flush, and 2 squashed
//     instructions (the two wrong-path ADDs I2, I3: the 3-entry add-station pool is full, so I4/I5 of the first pass have not issued yet).
//     The wrong-path ADDs never reach r4 -> r4 stays 0.
//  C: the same program with r1 = r2 = 0: r3 = 0, the branch is NOT taken, the prediction was right -> 6 commits, no flush, r4 = 20.
//  L (loop, N = 6): 0 ADD r3=r3+r1 | 1 SUB r1=r1-r5 | 2 BNEZ r1,0 | 3 HALT   (r1=6, r5=1) -> r3 = 21, 19 commits, 6 branches.
//     policy 0 mispredicts the 5 taken iterations -> 5 flushes; policy 1 predicts the backward branch taken and mispredicts only the exit -> 1 flush.
module spec_harness #(parameter bit PREDICT = 1'b0) (input logic clk, input logic rst_n, output logic done, output integer errors, output integer loop_cycles);
  logic local_rst = 0;
  wire  dut_rst_n = rst_n && !local_rst;
  logic init_we = 0, start = 0;  logic [2:0] init_reg = 0;  logic [31:0] init_data = 0;
  logic [7:0] imem_addr;  logic [31:0] imem_rdata, dbg_rdata;  logic halted;  logic [2:0] dbg_raddr = 0;
  logic [31:0] cycles, committed, branches, flushes, squashed;

  tomasulo_spec #(.PREDICT_BTFN(PREDICT)) dut (.clk, .rst_n(dut_rst_n), .init_we, .init_reg, .init_data, .start, .imem_addr, .imem_rdata,
                                              .halted, .stat_cycles(cycles), .stat_committed(committed), .stat_branches(branches),
                                              .stat_flushes(flushes), .stat_squashed(squashed), .dbg_raddr, .dbg_rdata);

  function automatic [31:0] ins(input [3:0] o, input [3:0] rd, rs, rt, input int tgt); ins = {o, rd, rs, rt, tgt[15:0]}; endfunction
  localparam [3:0] ADD = 0, SUB = 1, MUL = 2, BNEZ = 3, HALT = 4;
  logic [31:0] imem [0:63];
  assign imem_rdata = imem[imem_addr[5:0]];

  integer e;
  logic [31:0] g [0:7];
  logic [31:0] seed = 32'd1967;
  function automatic int rnd(input int n); seed = seed * 32'd1664525 + 32'd1013904223; return int'(seed[30:16]) % n; endfunction

  logic [31:0] ri [0:7];                                  // initial registers of the next program (Icarus: no array task ports)
  task automatic run_program();
    int t;
    @(negedge clk); local_rst = 1;  @(negedge clk); @(negedge clk); local_rst = 0;
    for (int r = 0; r < 8; r++) begin
      @(negedge clk); init_we = 1; init_reg = r[2:0]; init_data = ri[r];
      @(negedge clk); init_we = 0;
    end
    @(negedge clk); start = 1;  @(negedge clk); start = 0;
    t = 0;
    while (!halted && t < 3000) begin @(posedge clk); t++; end
    if (!halted) begin e++; $display("FAIL [P=%0b] never halted", PREDICT); end
    @(negedge clk);
  endtask
  task automatic check_reg(input int r, input logic [31:0] v, input string what);
    dbg_raddr = r[2:0]; #1;
    if (dbg_rdata !== v) begin e++; $display("FAIL [P=%0b] %s: r%0d = %0d, expected %0d", PREDICT, what, r, dbg_rdata, v); end
  endtask
  task automatic check_stat(input string what, input [31:0] got, input int exp_v);
    if (got !== exp_v) begin e++; $display("FAIL [P=%0b] %s = %0d, expected %0d", PREDICT, what, got, exp_v); end
  endtask

  initial begin
    int n, pcg, steps, dyn, dbr, taken, k;
    int prog_op [0:31], prog_rd [0:31], prog_rs [0:31], prog_rt [0:31], prog_tg [0:31];
    e = 0; done = 0; errors = 0; loop_cycles = 0;
    @(posedge rst_n);

    // ---- M: mispredicted forward branch ---------------------------------------------------------
    imem[0] = ins(ADD, 3, 1, 2, 0);  imem[1] = ins(BNEZ, 0, 3, 0, 4);  imem[2] = ins(ADD, 4, 4, 5, 0);
    imem[3] = ins(ADD, 4, 4, 5, 0);  imem[4] = ins(ADD, 6, 1, 1, 0);   imem[5] = ins(HALT, 0, 0, 0, 0);
    for (int r = 0; r < 8; r++) ri[r] = 0;
    ri[1] = 1; ri[2] = 2; ri[5] = 10;
    run_program();
    check_stat("M cycles", cycles, 17);  check_stat("M committed", committed, 4);  check_stat("M branches", branches, 1);
    check_stat("M flushes", flushes, 1); check_stat("M squashed", squashed, 2);
    check_reg(3, 3, "M");  check_reg(4, 0, "M (wrong path must not commit)");  check_reg(6, 2, "M");

    // ---- C: same program, branch not taken (prediction correct) ---------------------------------------
    ri[1] = 0; ri[2] = 0; ri[5] = 10;
    run_program();
    check_stat("C committed", committed, 6);  check_stat("C flushes", flushes, 0);  check_stat("C squashed", squashed, 0);
    check_reg(3, 0, "C");  check_reg(4, 20, "C");  check_reg(6, 0, "C");

    // ---- L: counted loop ------------------------------------------------------------------------------
    imem[0] = ins(ADD, 3, 3, 1, 0);  imem[1] = ins(SUB, 1, 1, 5, 0);  imem[2] = ins(BNEZ, 0, 1, 0, 0);  imem[3] = ins(HALT, 0, 0, 0, 0);
    for (int r = 0; r < 8; r++) ri[r] = 0;
    ri[1] = 6; ri[5] = 1;
    run_program();
    check_stat("L committed", committed, 19);  check_stat("L branches", branches, 6);
    check_stat("L flushes", flushes, PREDICT ? 1 : 5);
    check_reg(3, 21, "L");  check_reg(1, 0, "L");
    loop_cycles = int'(cycles);
    $display("loop, policy %0d: %0d cycles, %0d flushes, %0d squashed", PREDICT, cycles, flushes, squashed);

    // ---- random forward-branch programs vs a sequential interpreter ------------------------------------
    for (int p = 0; p < 40; p++) begin
      n = 24;
      for (int i = 0; i < n; i++) begin
        if (rnd(5) == 0 && i < n - 2) begin                   // forward BNEZ to a later instruction (possibly the HALT)
          prog_op[i] = 3; prog_rd[i] = 0; prog_rs[i] = rnd(8); prog_rt[i] = 0; prog_tg[i] = i + 2 + rnd(n - i - 1);
        end else begin
          prog_op[i] = rnd(3); prog_rd[i] = rnd(8); prog_rs[i] = rnd(8); prog_rt[i] = rnd(8); prog_tg[i] = 0;
        end
        imem[i] = ins(prog_op[i][3:0], prog_rd[i][3:0], prog_rs[i][3:0], prog_rt[i][3:0], prog_tg[i]);
      end
      imem[n] = ins(HALT, 0, 0, 0, 0);
      for (int r = 0; r < 8; r++) begin ri[r] = rnd(4); g[r] = ri[r]; end
      pcg = 0; dyn = 0; dbr = 0; taken = 0; steps = 0;       // sequential interpreter
      while (pcg < n && steps < 1000) begin
        steps++; dyn++;
        case (prog_op[pcg])
          0: begin g[prog_rd[pcg]] = g[prog_rs[pcg]] + g[prog_rt[pcg]]; pcg++; end
          1: begin g[prog_rd[pcg]] = g[prog_rs[pcg]] - g[prog_rt[pcg]]; pcg++; end
          2: begin g[prog_rd[pcg]] = g[prog_rs[pcg]] * g[prog_rt[pcg]]; pcg++; end
          default: begin dbr++; if (g[prog_rs[pcg]] != 0) begin taken++; pcg = prog_tg[pcg]; end else pcg++; end
        endcase
      end
      dyn++;                                                  // the HALT
      run_program();
      for (int r = 0; r < 8; r++) check_reg(r, g[r], $sformatf("random %0d", p));
      check_stat("random committed", committed, dyn);
      check_stat("random branches", branches, dbr);
      check_stat("random flushes", flushes, taken);           // every forward branch is predicted not taken, under both policies
    end
    errors = e;  done = 1;
  end
endmodule

module tb_tomasulo_spec;
  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;
  logic d0, d1;  integer er0, er1, lc0, lc1;  integer extra;
  spec_harness #(.PREDICT(1'b0)) nt   (.clk, .rst_n, .done(d0), .errors(er0), .loop_cycles(lc0));
  spec_harness #(.PREDICT(1'b1)) btfn (.clk, .rst_n, .done(d1), .errors(er1), .loop_cycles(lc1));
  initial begin
    extra = 0;
    repeat (2) @(negedge clk); rst_n = 1;
    wait (d0 && d1);
    #1;
    if (lc1 >= lc0) begin extra++; $display("FAIL: backward-taken prediction (%0d cycles) is not faster than always-not-taken (%0d)", lc1, lc0); end
    else $display("loop of 6: always-not-taken %0d cycles, backward-taken/forward-not-taken %0d cycles (%0d saved)", lc0, lc1, lc0 - lc1);
    if (er0 + er1 + extra == 0) $display("TEST PASSED"); else $display("TEST FAILED: %0d errors", er0 + er1 + extra);
    $finish;
  end
endmodule
