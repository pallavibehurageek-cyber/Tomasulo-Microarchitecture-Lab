`timescale 1ns/1ps
// Three predictors see the same 60-branch trace (inner loop T,T,T,N; an alternating branch;
// an outer-loop branch).  Expected misprediction counts come from an independent Python model.
module tb_branch_predictor;
  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;

  logic [31:0] pc = 0, upd_pc = 0;
  logic upd_valid = 0, upd_taken = 0;
  logic p1, p2, pg;

  branch_predictor #(.IDX_W(8), .CTR_BITS(1), .HIST_W(0)) one_bit (.clk, .rst_n, .pc, .predict_taken(p1), .upd_valid, .upd_pc, .upd_taken);
  branch_predictor #(.IDX_W(8), .CTR_BITS(2), .HIST_W(0)) two_bit (.clk, .rst_n, .pc, .predict_taken(p2), .upd_valid, .upd_pc, .upd_taken);
  branch_predictor #(.IDX_W(8), .CTR_BITS(2), .HIST_W(8)) gshare  (.clk, .rst_n, .pc, .predict_taken(pg), .upd_valid, .upd_pc, .upd_taken);

  int mis1 = 0, mis2 = 0, misg = 0, n = 0, errors = 0;

  task automatic branch(input [31:0] a, input bit taken);
    @(negedge clk);
    pc = a; #1;
    if (p1 !== taken) mis1++;
    if (p2 !== taken) mis2++;
    if (pg !== taken) misg++;
    n++;
    upd_pc = a; upd_taken = taken; upd_valid = 1;
    @(negedge clk);
    upd_valid = 0;
  endtask

  initial begin
    repeat (2) @(negedge clk); rst_n = 1;
    for (int i = 0; i < 10; i++) begin
      for (int k = 0; k < 4; k++) branch(32'h100, k < 3);   // inner loop branch
      branch(32'h400, i % 2 == 0);                           // alternating branch
      branch(32'h200, i < 9);                                // outer loop branch
    end
    $display("branches=%0d  mispredicts: 1-bit=%0d  2-bit=%0d  gshare=%0d", n, mis1, mis2, misg);
    if (n    != 60) begin errors++; $display("FAIL trace length"); end
    if (mis1 != 32) begin errors++; $display("FAIL 1-bit: %0d != 32", mis1); end
    if (mis2 != 23) begin errors++; $display("FAIL 2-bit: %0d != 23", mis2); end
    if (misg != 17) begin errors++; $display("FAIL gshare: %0d != 17", misg); end
    if (errors == 0) $display("TEST PASSED"); else $display("TEST FAILED: %0d errors", errors);
    $finish;
  end
endmodule
