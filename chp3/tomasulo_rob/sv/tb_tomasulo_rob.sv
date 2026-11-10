`timescale 1ns/1ps
module tb_tomasulo_rob;
  localparam int NREG = 8, ROBN = 8, DW = 32;
  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;

  logic init_we = 0;  logic [2:0] init_reg = 0;  logic [DW-1:0] init_data = 0;
  logic issue_valid = 0;  logic [1:0] issue_op = 0;  logic [2:0] issue_rd = 0, issue_rs = 0, issue_rt = 0;
  logic issue_ready, cdb_valid, commit_valid, empty;
  logic [2:0] cdb_tag, commit_reg, dbg_raddr = 0;
  logic [DW-1:0] commit_data, dbg_rdata;

  tomasulo_rob #(.NREG(NREG), .ROBN(ROBN)) dut (.*);

  // ---- observation ----------------------------------------------------------------------------
  int edge_no = 0, base = 0;
  always @(posedge clk) if (rst_n) edge_no <= edge_no + 1;

  int ngot = 0;
  int got_reg [0:255];  logic [DW-1:0] got_val [0:255];  int got_edge [0:255];
  int bcast_edge [0:ROBN-1];
  always @(negedge clk) if (rst_n) begin
    if (commit_valid) begin got_reg[ngot] = commit_reg; got_val[ngot] = commit_data; got_edge[ngot] = edge_no - base; ngot++; end
    if (cdb_valid) bcast_edge[cdb_tag] = edge_no - base + 1;      // the broadcast happens at the next edge
  end

  // ---- helpers --------------------------------------------------------------------------------
  int errors = 0, stall_cycles = 0;
  logic [DW-1:0] g [0:NREG-1];                  // sequential golden register file

  task automatic set_reg(input int r, input logic [DW-1:0] v);
    @(negedge clk); init_we = 1; init_reg = r[2:0]; init_data = v;
    @(negedge clk); init_we = 0;
    g[r] = v;
  endtask
  task automatic issue(input int op, rd, rs, rt);
    @(negedge clk);
    issue_valid = 1; issue_op = op[1:0]; issue_rd = rd[2:0]; issue_rs = rs[2:0]; issue_rt = rt[2:0];
    #1;
    while (!issue_ready) begin stall_cycles++; @(negedge clk); #1; end
  endtask
  task automatic finish_issue(); @(negedge clk); issue_valid = 0; endtask
  task automatic drain();
    while (!empty) @(negedge clk);
    repeat (3) @(negedge clk);
  endtask
  function automatic logic [DW-1:0] model(input int op, input logic [DW-1:0] a, b);
    case (op) 0: model = a + b; 1: model = a - b; default: model = a * b; endcase
  endfunction
  task automatic expect_int(input string what, input int got, input int exp_v);
    if (got != exp_v) begin errors++; $display("FAIL %s: got %0d expected %0d", what, got, exp_v); end
  endtask

  logic [31:0] seed = 32'd1990;
  function automatic logic [31:0] rnd32(); seed = seed * 32'd1664525 + 32'd1013904223; return seed; endfunction
  function automatic int rnd(input int n); seed = seed * 32'd1664525 + 32'd1013904223; return int'(seed[30:16]) % n; endfunction

  initial begin
    repeat (2) @(negedge clk); rst_n = 1;

    // ---- 1. hand-derived schedule ------------------------------------------------------------
    //   I0: MUL r1 = r2*r3   I1: ADD r4 = r1+r5   I2: ADD r6 = r2+r3        (ADD_LAT=2, MUL_LAT=4)
    // I2 (independent) finishes first, wins the CDB at edge 7; the multiplier waits and broadcasts at 8;
    // I1 captures r1 off the bus at 8, starts at 9, broadcasts at 12.  Commits are in program order: 9, 13, 14.
    for (int r = 0; r < NREG; r++) g[r] = 0;
    set_reg(2, 3); set_reg(3, 4); set_reg(5, 10);
    ngot = 0;  base = edge_no + 1;       // issue() first waits one more negedge, so its instruction is accepted at the 2nd posedge from here = "edge 1"
    issue(2, 1, 2, 3);  issue(0, 4, 1, 5);  issue(0, 6, 2, 3);  finish_issue();
    drain();
    expect_int("commits", ngot, 3);
    expect_int("bcast I2", bcast_edge[2], 7);  expect_int("bcast I0", bcast_edge[0], 8);  expect_int("bcast I1", bcast_edge[1], 12);
    expect_int("commit I0 edge", got_edge[0], 9);  expect_int("commit I1 edge", got_edge[1], 13);  expect_int("commit I2 edge", got_edge[2], 14);
    expect_int("commit order 0", got_reg[0], 1);  expect_int("commit order 1", got_reg[1], 4);  expect_int("commit order 2", got_reg[2], 6);
    expect_int("r1 value", int'(got_val[0]), 12);  expect_int("r4 value", int'(got_val[1]), 22);  expect_int("r6 value", int'(got_val[2]), 7);
    $display("directed: completion order I2,I0,I1 (CDB edges 7,8,12) but commit order I0,I1,I2 (edges 9,13,14)");

    // ---- 2. an isolated ADD commits at issue+5, an isolated MUL at issue+7 -------------------------
    ngot = 0;  base = edge_no + 1;
    issue(0, 7, 2, 3);  finish_issue();  drain();
    expect_int("lone ADD commit edge", got_edge[0], 1 + 5);
    ngot = 0;  base = edge_no + 1;
    issue(2, 7, 2, 3);  finish_issue();  drain();
    expect_int("lone MUL commit edge", got_edge[0], 1 + 7);

    // ---- 3. renaming: WAW and WAR on r1 do not serialize; architectural order is preserved ---------
    //   I0: MUL r1=r2*r3 (12)  I1: ADD r4=r1+r5 (reads OLD r1 value 12)  I2: ADD r1=r5+r5 (20, completes first)
    for (int r = 0; r < NREG; r++) g[r] = 0;
    // registers already hold r2=3,r3=4,r5=10 and r1=12,r4=22,r6=7,r7=7 from the earlier programs; re-seed
    set_reg(1, 0); set_reg(4, 0); set_reg(6, 0); set_reg(7, 0); set_reg(2, 3); set_reg(3, 4); set_reg(5, 10);
    ngot = 0;  base = edge_no + 1;
    issue(2, 1, 2, 3);  issue(0, 4, 1, 5);  issue(0, 1, 5, 5);  finish_issue();  drain();
    expect_int("rename commits", ngot, 3);
    expect_int("I1 used old r1", int'(got_val[1]), 22);
    expect_int("final r1 is the later write", int'(got_val[2]), 20);
    dbg_raddr = 1; #1; expect_int("arch r1", int'(dbg_rdata), 20);
    dbg_raddr = 4; #1; expect_int("arch r4", int'(dbg_rdata), 22);

    // ---- 4. random programs vs. a sequential golden model ------------------------------------------
    for (int p = 0; p < 40; p++) begin
      int n;  int ops [0:127], rds [0:127], rss [0:127], rts [0:127];
      logic [DW-1:0] ev [0:127];
      n = 120;
      drain();
      for (int r = 0; r < NREG; r++) set_reg(r, rnd32());
      for (int i = 0; i < n; i++) begin
        ops[i] = rnd(3);  rds[i] = rnd(NREG);  rss[i] = rnd(NREG);  rts[i] = rnd(NREG);
        ev[i] = model(ops[i], g[rss[i]], g[rts[i]]);
        g[rds[i]] = ev[i];
      end
      ngot = 0;  base = edge_no + 1;
      for (int i = 0; i < n; i++) issue(ops[i], rds[i], rss[i], rts[i]);
      finish_issue();  drain();
      if (ngot != n) begin errors++; $display("FAIL program %0d: %0d commits, expected %0d", p, ngot, n); end
      else for (int i = 0; i < n; i++)
        if (got_reg[i] != rds[i] || got_val[i] !== ev[i]) begin
          errors++; $display("FAIL program %0d instr %0d: committed r%0d=%h, expected r%0d=%h", p, i, got_reg[i], got_val[i], rds[i], ev[i]);
        end
      for (int r = 0; r < NREG; r++) begin
        dbg_raddr = r[2:0]; #1;
        if (dbg_rdata !== g[r]) begin errors++; $display("FAIL program %0d final r%0d = %h expected %h", p, r, dbg_rdata, g[r]); end
      end
    end
    if (stall_cycles == 0) begin errors++; $display("FAIL: random phase never exercised issue back-pressure"); end
    $display("random phase: 40 programs x 120 instructions, %0d issue-stall cycles", stall_cycles);
    if (errors == 0) $display("TEST PASSED"); else $display("TEST FAILED: %0d errors", errors);
    $finish;
  end
endmodule
