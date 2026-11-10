`timescale 1ns/1ps
// Hand-derived programs (first instruction issues at edge 1; LD_LAT = 3, FWD_LAT = 1, MUL_LAT = 4, ADD_LAT = 2; "cycles" = edge at which HALT commits).
// Timing rules (see the RTL header): load  address i+1, start i+2, done i+2+L, CDB i+3+L, commit i+4+L;   store  address i+1, done i+2, commit (write) i+3.
//   A  LD r1,[r0+4] ; HALT                  LD issues 1, address 2, start 3, done 6, CDB 7, commit 8; HALT commits 9                               -> 9 cycles
//   B  ST r2,[r0+5] ; LD r1,[r0+5] ; HALT   ST: address 2, done 3, commit 4.  LD: address 3 (one address unit), start 4 -- the store is still in the queue -> FORWARD:
//        done 5, CDB 6, commit 7, HALT 8 -> 8 cycles.   Without forwarding the load must wait until the store has committed (edge 4): start 5, done 8, CDB 9, commit 10 -> 11 cycles
//   B2 MUL r3=r1*r2 ; ST r3,[r0+5] ; LD r4,[r0+5] ; HALT    MUL CDB at 7, commit 8.  ST data arrives at 7, done 8, commit 9.  LD address 4; with forwarding it starts when the data
//        is there (edge 8): done 9, CDB 10, commit 11, HALT 12 -> 12 cycles.   Without: starts after the store commit (edge 10): done 13, CDB 14, commit 15, HALT 16 -> 16
//   C  MUL r3=r1*r2 ; ST r5,[r3+0] ; LD r4,[r0+9] ; HALT    the store's address waits for the MUL (CDB 7, address at 8): the LD (address known at 4, different address!) may not start
//        until the older store's address is known -- edge 9: done 12, CDB 13, commit 14, HALT 15 -> 15 cycles   (conservative memory disambiguation)
//   C0 ST r5,[r0+6] ; LD r4,[r0+9] ; HALT   no wait: LD address 3, start 4 (store address known, different), done 7, CDB 8, commit 9, HALT 10 -> 10 cycles
//   S  ST r1,[r0+5] ; ST r2,[r0+5] ; LD r3,[r0+5] ; HALT    the load takes the value of the YOUNGEST older store: r3 = r2.  ST1 commits 4, ST2 commits 5, LD address 4, start 5:
//        forward (ST2 still queued): done 6, CDB 7, commit 8, HALT 9 -> 9 cycles.   Without forwarding: start 6, done 9, CDB 10, commit 11, HALT 12 -> 12 cycles
//   S2 MUL r6=r1*r2 ; ST r1,[r0+5] ; ST r2,[r0+5] ; LD r3,[r0+5] ; HALT    the slow multiply keeps both stores in the queue (MUL commit 8, ST1 9, ST2 10).  LD: address 5, start 6; with
//        forwarding the YOUNGEST older store (ST2) supplies r2: done 7, CDB 8, commit 11, HALT 12 -> 12 cycles.   Without: start after ST2 committed (11): done 14, CDB 15, commit 16, HALT 17
//   D  wrong path: ADD r3=r1+r2 ; BNEZ r3,6 ; ST r4,[r0+7] ; LD r6,[r0+7] ; ADD r5=r6+r6 ; ADD r5=r5+r5 ; LD r7,[r0+8] ; HALT    (r1=1 r2=2 r4=99, mem[7]=5, mem[8]=31)
//        the branch (taken, predicted not taken) commits at 10 and flushes as in tomasulo_spec; LD r7 issues at 11, commit 18, HALT 19.  The wrong-path store NEVER reaches memory
//        (mem[7] stays 5), the wrong-path load and adds never reach r6 / r5, r7 = 31 -> 19 cycles, 1 flush, 4 commits (ADD, BNEZ, LD, HALT), 1 load, 0 stores
// Then 40 random programs (ADD/SUB/MUL, LD, ST, forward BNEZ; load/store bases are the constants r0 = 0 / r1 = 1 and the offsets are 0..7, so addresses alias heavily) against a sequential interpreter:
// all registers, all 64 memory words, committed instructions, loads, stores, branches and flushes (= taken branches).
module lsq_harness #(parameter bit FORWARD = 1'b1) (input logic clk, input logic rst_n, output logic done, output integer errors, output integer rand_cycles, output integer rand_fwd);
  logic local_rst = 0;
  wire  dut_rst_n = rst_n && !local_rst;
  logic init_we = 0, dinit_we = 0, start = 0;  logic [2:0] init_reg = 0;  logic [31:0] init_data = 0;  logic [5:0] dinit_addr = 0;  logic [31:0] dinit_data = 0;
  logic [7:0] imem_addr;  logic [31:0] imem_rdata, dbg_rdata, dbg_mdata;  logic halted;  logic [2:0] dbg_raddr = 0;  logic [5:0] dbg_maddr = 0;
  logic [31:0] cycles, committed, branches, flushes, squashed, loads, stores, fwds;

  tomasulo_lsq #(.FORWARD(FORWARD)) dut (.clk, .rst_n(dut_rst_n), .init_we, .init_reg, .init_data, .dinit_we, .dinit_addr, .dinit_data, .start, .imem_addr, .imem_rdata,
                                         .halted, .stat_cycles(cycles), .stat_committed(committed), .stat_branches(branches), .stat_flushes(flushes), .stat_squashed(squashed),
                                         .stat_loads(loads), .stat_stores(stores), .stat_fwd(fwds), .dbg_raddr, .dbg_rdata, .dbg_maddr, .dbg_mdata);

  function automatic [31:0] ins(input [3:0] o, input [3:0] rd, rs, rt, input int imm); ins = {o, rd, rs, rt, imm[15:0]}; endfunction
  localparam [3:0] ADD = 0, SUB = 1, MUL = 2, BNEZ = 3, HALT = 4, LD = 5, ST = 6;
  logic [31:0] imem [0:63];
  assign imem_rdata = imem[imem_addr[5:0]];

  integer e;
  logic [31:0] g [0:7], gm [0:63];
  logic [31:0] seed = 32'd1967;
  function automatic int rnd(input int n); seed = seed * 32'd1664525 + 32'd1013904223; return int'(seed[30:16]) % n; endfunction

  logic [31:0] ri [0:7], mi [0:63];                         // initial registers / memory of the next program
  task automatic run_program();
    int t;
    @(negedge clk); local_rst = 1;  @(negedge clk); @(negedge clk); local_rst = 0;
    for (int r = 0; r < 8; r++) begin
      @(negedge clk); init_we = 1; init_reg = r[2:0]; init_data = ri[r];
      @(negedge clk); init_we = 0;
    end
    for (int a = 0; a < 64; a++) begin
      @(negedge clk); dinit_we = 1; dinit_addr = a[5:0]; dinit_data = mi[a];
      @(negedge clk); dinit_we = 0;
    end
    @(negedge clk); start = 1;  @(negedge clk); start = 0;
    t = 0;
    while (!halted && t < 3000) begin @(posedge clk); t++; end
    if (!halted) begin e++; $display("FAIL [F=%0b] never halted", FORWARD); end
    @(negedge clk);
  endtask
  task automatic check_reg(input int r, input logic [31:0] v, input string what);
    dbg_raddr = r[2:0]; #1;
    if (dbg_rdata !== v) begin e++; $display("FAIL [F=%0b] %s: r%0d = %0d, expected %0d", FORWARD, what, r, dbg_rdata, v); end
  endtask
  task automatic check_mem(input int a, input logic [31:0] v, input string what);
    dbg_maddr = a[5:0]; #1;
    if (dbg_mdata !== v) begin e++; $display("FAIL [F=%0b] %s: mem[%0d] = %0d, expected %0d", FORWARD, what, a, dbg_mdata, v); end
  endtask
  task automatic check_stat(input string what, input [31:0] got, input int exp_v);
    if (got !== exp_v) begin e++; $display("FAIL [F=%0b] %s = %0d, expected %0d", FORWARD, what, got, exp_v); end
  endtask
  task automatic clear_state();
    for (int r = 0; r < 8; r++) ri[r] = 0;
    for (int a = 0; a < 64; a++) mi[a] = 32'd1000 + a;
  endtask

  initial begin
    int n, pcg, steps, dyn, dbr, taken, nld, nst, k, ty, f_rd, f_rs, f_rt, f_im, f_tg, f_op, ad, sumc, sumf;
    int prog_op [0:31], prog_rd [0:31], prog_rs [0:31], prog_rt [0:31], prog_tg [0:31];
    e = 0; done = 0; errors = 0; rand_cycles = 0; rand_fwd = 0;
    @(posedge rst_n);

    // ---- A: one load ----
    imem[0] = ins(LD, 1, 0, 0, 4);  imem[1] = ins(HALT, 0, 0, 0, 0);
    clear_state();  mi[4] = 123;
    run_program();
    check_stat("A cycles", cycles, 9);  check_stat("A loads", loads, 1);  check_stat("A forwards", fwds, 0);  check_reg(1, 123, "A");

    // ---- B: store then load of the same address ----
    imem[0] = ins(ST, 0, 0, 2, 5);  imem[1] = ins(LD, 1, 0, 0, 5);  imem[2] = ins(HALT, 0, 0, 0, 0);
    clear_state();  ri[2] = 77;
    run_program();
    check_stat("B cycles", cycles, FORWARD ? 8 : 11);  check_stat("B forwards", fwds, FORWARD ? 1 : 0);  check_stat("B stores", stores, 1);  check_stat("B loads", loads, 1);
    check_reg(1, 77, "B");  check_mem(5, 77, "B");

    // ---- B2: the store's data comes from a multiply ----
    imem[0] = ins(MUL, 3, 1, 2, 0);  imem[1] = ins(ST, 0, 0, 3, 5);  imem[2] = ins(LD, 4, 0, 0, 5);  imem[3] = ins(HALT, 0, 0, 0, 0);
    clear_state();  ri[1] = 2; ri[2] = 3;
    run_program();
    check_stat("B2 cycles", cycles, FORWARD ? 12 : 16);  check_stat("B2 forwards", fwds, FORWARD ? 1 : 0);
    check_reg(4, 6, "B2");  check_mem(5, 6, "B2");

    // ---- C: an older store with an unknown address blocks a younger load of ANOTHER address ----
    imem[0] = ins(MUL, 3, 1, 2, 0);  imem[1] = ins(ST, 0, 3, 5, 0);  imem[2] = ins(LD, 4, 0, 0, 9);  imem[3] = ins(HALT, 0, 0, 0, 0);
    clear_state();  ri[1] = 2; ri[2] = 3; ri[5] = 55;
    run_program();
    check_stat("C cycles", cycles, 15);  check_stat("C forwards", fwds, 0);  check_reg(4, mi[9], "C");  check_mem(6, 55, "C");
    // ---- C0: the same without the late address ----
    imem[0] = ins(ST, 0, 0, 5, 6);  imem[1] = ins(LD, 4, 0, 0, 9);  imem[2] = ins(HALT, 0, 0, 0, 0);
    clear_state();  ri[5] = 55;
    run_program();
    check_stat("C0 cycles", cycles, 10);  check_reg(4, mi[9], "C0");  check_mem(6, 55, "C0");

    // ---- S: two stores to one address, then a load ----
    imem[0] = ins(ST, 0, 0, 1, 5);  imem[1] = ins(ST, 0, 0, 2, 5);  imem[2] = ins(LD, 3, 0, 0, 5);  imem[3] = ins(HALT, 0, 0, 0, 0);
    clear_state();  ri[1] = 11; ri[2] = 22;
    run_program();
    check_stat("S cycles", cycles, FORWARD ? 9 : 12);  check_stat("S forwards", fwds, FORWARD ? 1 : 0);  check_stat("S stores", stores, 2);
    check_reg(3, 22, "S (youngest older store)");  check_mem(5, 22, "S (commit order)");

    // ---- S2: both stores are still queued behind a slow multiply when the load starts: the YOUNGEST older store must supply the value ----
    imem[0] = ins(MUL, 6, 1, 2, 0);  imem[1] = ins(ST, 0, 0, 1, 5);  imem[2] = ins(ST, 0, 0, 2, 5);  imem[3] = ins(LD, 3, 0, 0, 5);  imem[4] = ins(HALT, 0, 0, 0, 0);
    clear_state();  ri[1] = 11; ri[2] = 22;
    run_program();
    check_stat("S2 cycles", cycles, FORWARD ? 12 : 17);  check_stat("S2 forwards", fwds, FORWARD ? 1 : 0);
    check_reg(3, 22, "S2 (youngest older store)");  check_reg(6, 242, "S2");  check_mem(5, 22, "S2");

    // ---- D: a store, a load and adds on the wrong path of a mispredicted branch ----
    imem[0] = ins(ADD, 3, 1, 2, 0);  imem[1] = ins(BNEZ, 0, 3, 0, 6);  imem[2] = ins(ST, 0, 0, 4, 7);  imem[3] = ins(LD, 6, 0, 0, 7);
    imem[4] = ins(ADD, 5, 6, 6, 0);  imem[5] = ins(ADD, 5, 5, 5, 0);  imem[6] = ins(LD, 7, 0, 0, 8);  imem[7] = ins(HALT, 0, 0, 0, 0);
    clear_state();  ri[1] = 1; ri[2] = 2; ri[4] = 99;  mi[7] = 5;  mi[8] = 31;
    run_program();
    check_stat("D cycles", cycles, 19);  check_stat("D flushes", flushes, 1);  check_stat("D committed", committed, 4);  check_stat("D stores", stores, 0);  check_stat("D loads", loads, 1);
    check_mem(7, 5, "D (the wrong-path store must not reach memory)");  check_reg(6, 0, "D");  check_reg(5, 0, "D");  check_reg(7, 31, "D");  check_reg(4, 99, "D");

    // ---- random programs vs a sequential interpreter ----
    sumc = 0;  sumf = 0;
    for (int p = 0; p < 40; p++) begin
      n = 24;
      for (int i = 0; i < n; i++) begin
        ty = rnd(10);  f_rd = rnd(8);  f_rs = rnd(8);  f_rt = rnd(8);  f_im = rnd(8);  f_tg = rnd(n);  f_op = rnd(3);   // fixed draw order: the VHDL testbench draws the same stream
        if (ty < 2 || (ty == 9)) begin prog_op[i] = f_op; prog_rd[i] = 2 + f_rd % 6; prog_rs[i] = f_rs; prog_rt[i] = f_rt; prog_tg[i] = 0; end
        else if (ty < 5)  begin prog_op[i] = 5; prog_rd[i] = 2 + f_rd % 6; prog_rs[i] = f_rs % 2; prog_rt[i] = 0; prog_tg[i] = f_im; end
        else if (ty < 8)  begin prog_op[i] = 6; prog_rd[i] = 0; prog_rs[i] = f_rs % 2; prog_rt[i] = f_rt; prog_tg[i] = f_im; end
        else if (ty == 8 && i < n - 2) begin prog_op[i] = 3; prog_rd[i] = 0; prog_rs[i] = f_rs; prog_rt[i] = 0; prog_tg[i] = i + 2 + (f_tg % (n - i - 1)); end
        else begin prog_op[i] = f_op; prog_rd[i] = 2 + f_rd % 6; prog_rs[i] = f_rs; prog_rt[i] = f_rt; prog_tg[i] = 0; end
        imem[i] = ins(prog_op[i][3:0], prog_rd[i][3:0], prog_rs[i][3:0], prog_rt[i][3:0], prog_tg[i]);
      end
      imem[n] = ins(HALT, 0, 0, 0, 0);
      for (int r = 0; r < 8; r++) begin ri[r] = rnd(4); if (r == 0) ri[r] = 0; if (r == 1) ri[r] = 1; g[r] = ri[r]; end        // r0 = 0 and r1 = 1 are never written: constant bases, heavy aliasing
      for (int a = 0; a < 64; a++) begin mi[a] = 32'd1000 + a; gm[a] = mi[a]; end
      pcg = 0; dyn = 0; dbr = 0; taken = 0; steps = 0; nld = 0; nst = 0;       // sequential interpreter
      while (pcg < n && steps < 1000) begin
        steps++; dyn++;
        case (prog_op[pcg])
          0: begin g[prog_rd[pcg]] = g[prog_rs[pcg]] + g[prog_rt[pcg]]; pcg++; end
          1: begin g[prog_rd[pcg]] = g[prog_rs[pcg]] - g[prog_rt[pcg]]; pcg++; end
          2: begin g[prog_rd[pcg]] = g[prog_rs[pcg]] * g[prog_rt[pcg]]; pcg++; end
          5: begin ad = (g[prog_rs[pcg]] + prog_tg[pcg]) & 63; g[prog_rd[pcg]] = gm[ad]; nld++; pcg++; end
          6: begin ad = (g[prog_rs[pcg]] + prog_tg[pcg]) & 63; gm[ad] = g[prog_rt[pcg]]; nst++; pcg++; end
          default: begin dbr++; if (g[prog_rs[pcg]] != 0) begin taken++; pcg = prog_tg[pcg]; end else pcg++; end
        endcase
      end
      dyn++;                                                  // the HALT
      run_program();
      for (int r = 0; r < 8; r++) check_reg(r, g[r], $sformatf("random %0d", p));
      for (int a = 0; a < 64; a++) check_mem(a, gm[a], $sformatf("random %0d", p));
      check_stat("random committed", committed, dyn);
      check_stat("random loads", loads, nld);
      check_stat("random stores", stores, nst);
      check_stat("random branches", branches, dbr);
      check_stat("random flushes", flushes, taken);
      sumc += int'(cycles);  sumf += int'(fwds);
    end
    rand_cycles = sumc;  rand_fwd = sumf;
    errors = e;  done = 1;
  end
endmodule

module tb_tomasulo_lsq;
  logic clk = 0, rst_n = 0;
  always #5 clk = ~clk;
  logic d0, d1;  integer er0, er1, rc0, rc1, rf0, rf1, extra;
  lsq_harness #(.FORWARD(1'b1)) fw   (.clk, .rst_n, .done(d0), .errors(er0), .rand_cycles(rc0), .rand_fwd(rf0));
  lsq_harness #(.FORWARD(1'b0)) nofw (.clk, .rst_n, .done(d1), .errors(er1), .rand_cycles(rc1), .rand_fwd(rf1));
  initial begin
    extra = 0;
    repeat (2) @(negedge clk); rst_n = 1;
    wait (d0 && d1);
    #1;
    if (rf1 != 0) begin extra++; $display("FAIL: forwarding disabled but %0d loads were forwarded", rf1); end
    if (!(rc0 < rc1)) begin extra++; $display("FAIL: store-to-load forwarding (%0d cycles) is not faster than waiting for the commit (%0d)", rc0, rc1); end
    else $display("40 random programs: %0d cycles with store-to-load forwarding (%0d loads forwarded), %0d cycles without", rc0, rf0, rc1);
    if (er0 + er1 + extra == 0) $display("TEST PASSED"); else $display("TEST FAILED: %0d errors", er0 + er1 + extra);
    $finish;
  end
endmodule
