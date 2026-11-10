`timescale 1ns/1ps
// 1. A hand-derived schedule that shows RAW, structural, WAR and WAW hazards in one 5-instruction program.
// 2. Random programs, checked against (a) a sequential golden register file and (b) exact timing rules derived from the spec:
//      read  = max(issue+1, 1 + write of the producer of each source)
//      write = max(read+LAT+1, 1 + read of every OLDER reader of the destination)
//      issue after the previous issue; after the write of an older writer of the same register (WAW);
//      at most 0 older adds / 1 older multiply still in flight when an add / multiply issues (structural).
module tb_scoreboard;
  localparam int NREG = 8, DW = 32, NMAX = 4096;
  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;

  logic init_we = 0;  logic [2:0] init_reg = 0;  logic [DW-1:0] init_data = 0;
  logic issue_valid = 0;  logic [1:0] issue_op = 0;  logic [2:0] issue_rd = 0, issue_rs = 0, issue_rt = 0;
  logic issue_ready, empty;  logic [2:0] read_go, write_go;  logic [47:0] fu_seq;
  logic [2:0] dbg_raddr = 0;  logic [DW-1:0] dbg_rdata;

  scoreboard dut (.*);

  int edge_no = 0;
  always @(posedge clk) if (rst_n) edge_no <= edge_no + 1;

  // absolute event log, indexed by the DUT's sequence number
  int ev_issue [0:NMAX-1], ev_read [0:NMAX-1], ev_write [0:NMAX-1];
  always @(negedge clk) if (rst_n)
    for (int f = 0; f < 3; f++) begin
      if (read_go[f])  ev_read[fu_seq[16*f +: 16]]  = edge_no + 1;     // happens at the next edge
      if (write_go[f]) ev_write[fu_seq[16*f +: 16]] = edge_no + 1;
    end

  int errors = 0, stall_cycles = 0, n_issued = 0;
  logic [DW-1:0] g [0:NREG-1];
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
    ev_issue[n_issued] = edge_no + 1;  n_issued++;                   // accepted at the next edge
  endtask
  task automatic finish_issue(); @(negedge clk); issue_valid = 0; endtask
  task automatic drain();
    while (!empty) @(negedge clk);
    repeat (3) @(negedge clk);
  endtask
  function automatic logic [DW-1:0] model(input int op, input logic [DW-1:0] a, b);
    case (op) 0: model = a + b; 1: model = a - b; default: model = a * b; endcase
  endfunction
  function automatic int lat(input int op); lat = (op == 2) ? 4 : 2; endfunction
  task automatic expect_int(input string what, input int got, input int exp_v);
    if (got != exp_v) begin errors++; $display("FAIL %s: got %0d expected %0d", what, got, exp_v); end
  endtask
  function automatic int imax(input int a, b); imax = (a > b) ? a : b; endfunction

  logic [31:0] seed = 32'd6600;
  function automatic logic [31:0] rnd32(); seed = seed * 32'd1664525 + 32'd1013904223; return seed; endfunction
  function automatic int rnd(input int n); seed = seed * 32'd1664525 + 32'd1013904223; return int'(seed[30:16]) % n; endfunction

  initial begin
    int s0, base;
    repeat (2) @(negedge clk); rst_n = 1;

    // ---- 1. hand-derived schedule ------------------------------------------------------------
    //   J0: MUL r1 = r2*r3    J1: MUL r4 = r1*r5    J2: ADD r5 = r2+r3    J3: ADD r6 = r5+r4    J4: MUL r4 = r2*r3
    //   issue/read/write edges:  J0 1/2/7   J1 2/8/13 (RAW on r1)   J2 3/4/9 (WAR: waits for J1 to read old r5)
    //                            J3 10/14/17 (structural: add unit busy until J2 writes; RAW on r4)
    //                            J4 14/15/20 (WAW: r4 pending from J1 until edge 13)
    for (int r = 0; r < NREG; r++) g[r] = 0;
    set_reg(2, 3); set_reg(3, 4); set_reg(5, 10);
    base = edge_no;  s0 = n_issued;
    issue(2, 1, 2, 3);  issue(2, 4, 1, 5);  issue(0, 5, 2, 3);  issue(0, 6, 5, 4);  issue(2, 4, 2, 3);  finish_issue();
    drain();
    begin
      int exp_i [5], exp_r [5], exp_w [5];
      exp_i[0] = 1;  exp_i[1] = 2;  exp_i[2] = 3;  exp_i[3] = 10; exp_i[4] = 14;      // (Icarus cannot assign array literals)
      exp_r[0] = 2;  exp_r[1] = 8;  exp_r[2] = 4;  exp_r[3] = 14; exp_r[4] = 15;
      exp_w[0] = 7;  exp_w[1] = 13; exp_w[2] = 9;  exp_w[3] = 17; exp_w[4] = 20;
      // our edge numbering: the first accepted instruction is "edge 1" only after issue()'s extra negedge, so shift by base+1
      for (int j = 0; j < 5; j++) begin
        expect_int($sformatf("J%0d issue", j), ev_issue[s0 + j] - (base + 1), exp_i[j]);
        expect_int($sformatf("J%0d read ", j), ev_read [s0 + j] - (base + 1), exp_r[j]);
        expect_int($sformatf("J%0d write", j), ev_write[s0 + j] - (base + 1), exp_w[j]);
      end
    end
    dbg_raddr = 1; #1; expect_int("r1", int'(dbg_rdata), 12);
    dbg_raddr = 4; #1; expect_int("r4", int'(dbg_rdata), 12);     // J4 wrote last
    dbg_raddr = 5; #1; expect_int("r5", int'(dbg_rdata), 7);
    dbg_raddr = 6; #1; expect_int("r6", int'(dbg_rdata), 127);    // J3 read J2's r5 (7) and J1's r4 (120)
    $display("directed: RAW, structural, WAR and WAW stalls reproduced exactly (20 cycles)");

    // ---- 2. random programs ------------------------------------------------------------------------
    for (int p = 0; p < 30; p++) begin
      int n;  int ops [0:127], rds [0:127], rss [0:127], rts [0:127];  int s;
      n = 100;
      drain();
      for (int r = 0; r < NREG; r++) set_reg(r, rnd32());
      for (int i = 0; i < n; i++) begin
        ops[i] = rnd(3);  rds[i] = rnd(NREG);  rss[i] = rnd(NREG);  rts[i] = rnd(NREG);
        g[rds[i]] = model(ops[i], g[rss[i]], g[rts[i]]);
      end
      s = n_issued;
      for (int i = 0; i < n; i++) issue(ops[i], rds[i], rss[i], rts[i]);
      finish_issue();  drain();
      // (a) architectural state
      for (int r = 0; r < NREG; r++) begin
        dbg_raddr = r[2:0]; #1;
        if (dbg_rdata !== g[r]) begin errors++; $display("FAIL program %0d final r%0d = %h expected %h", p, r, dbg_rdata, g[r]); end
      end
      // (b) timing rules, from the log alone
      for (int i = 0; i < n; i++) begin
        int I, R, W, rmin, wmin, in_flight;
        I = ev_issue[s+i];  R = ev_read[s+i];  W = ev_write[s+i];
        if (i > 0 && I <= ev_issue[s+i-1]) begin errors++; $display("FAIL p%0d i%0d: issue not in order", p, i); end
        rmin = I + 1;
        for (int k = 0; k < 2; k++) begin                                  // RAW: latest older writer of each source
          int src, prod;  src = (k == 0) ? rss[i] : rts[i];  prod = -1;
          for (int j = 0; j < i; j++) if (rds[j] == src) prod = j;
          if (prod >= 0) rmin = imax(rmin, ev_write[s+prod] + 1);
        end
        if (R != rmin) begin errors++; $display("FAIL p%0d i%0d: read at %0d, rule says %0d", p, i, R, rmin); end
        wmin = R + lat(ops[i]) + 1;
        for (int j = 0; j < i; j++) begin                                  // WAR: every older reader of my destination
          if (rss[j] == rds[i] || rts[j] == rds[i]) wmin = imax(wmin, ev_read[s+j] + 1);
          if (rds[j] == rds[i] && I <= ev_write[s+j]) begin errors++; $display("FAIL p%0d i%0d: WAW, issued before older writer finished", p, i); end
        end
        if (W != wmin) begin errors++; $display("FAIL p%0d i%0d: write at %0d, rule says %0d", p, i, W, wmin); end
        in_flight = 0;                                                     // structural: add unit x1, multipliers x2
        for (int j = 0; j < i; j++)
          if ((ops[j] == 2) == (ops[i] == 2) && ev_write[s+j] >= I) in_flight++;
        if (in_flight > ((ops[i] == 2) ? 1 : 0)) begin errors++; $display("FAIL p%0d i%0d: structural hazard violated (%0d in flight)", p, i, in_flight); end
      end
    end
    if (stall_cycles == 0) begin errors++; $display("FAIL: issue stalls never exercised"); end
    $display("random phase: 30 programs x 100 instructions, %0d issue-stall cycles", stall_cycles);
    if (errors == 0) $display("TEST PASSED"); else $display("TEST FAILED: %0d errors", errors);
    $finish;
  end
endmodule
