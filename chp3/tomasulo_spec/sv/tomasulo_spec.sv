// Tomasulo with hardware speculation past branches (Chapter 3): reservation stations + CDB + reorder buffer, now with a fetch
// unit and a branch instruction.  See ../tomasulo_rob for the non-speculative core and its exact timing rules; they are unchanged.
//
// ISA (32-bit words, 8 registers):   op[31:28] | rd[27:24] | rs[23:20] | rt[19:16] | target[15:0]
//   0 ADD rd=rs+rt   1 SUB rd=rs-rt   2 MUL rd=rs*rt   3 BNEZ rs,target (branch if rs != 0)   4 HALT
// Speculation:
//   * Every BNEZ is predicted when it issues: PREDICT_BTFN = 0 -> always not taken; 1 -> backward (target < pc) taken, forward not taken.
//     Fetch continues down the predicted path, so younger instructions issue, execute and even broadcast speculatively.
//   * A branch occupies a ROB entry and an ADD-type reservation station (the add unit forwards its source: nonzero = taken).
//   * Nothing speculative ever reaches the architectural registers: results are written only at commit, in order.
//   * Recovery happens at COMMIT: when the branch is at the head of the ROB and its outcome disagrees with its prediction, the whole
//     ROB, all stations, both units and the rename table are cleared (everything left in the machine is younger and wrong) and fetch
//     restarts at the correct address.  Nothing issues on the flush edge, so the next instruction issues one edge later.
//   * HALT stops the front end; it commits like any other instruction and then sets `halted`.  A wrong-path HALT is simply flushed.
// Statistics: stat_cycles (edges while running), stat_committed, stat_branches (committed BNEZ), stat_flushes (mispredictions),
//             stat_squashed (instructions thrown away by flushes).
module tomasulo_spec #(
    parameter int NREG        = 8,
    parameter int ROBN        = 8,
    parameter int NADD        = 3,
    parameter int NMUL        = 2,
    parameter int ADD_LAT     = 2,
    parameter int MUL_LAT     = 4,
    parameter bit PREDICT_BTFN = 1'b0,
    parameter int DW          = 32
) (
    input  logic                  clk,
    input  logic                  rst_n,
    input  logic                  init_we,            // initialise registers while stopped and empty
    input  logic [$clog2(NREG)-1:0] init_reg,
    input  logic [DW-1:0]         init_data,
    input  logic                  start,              // begin fetching at pc = 0 (first issue at the next edge)
    output logic [7:0]            imem_addr,
    input  logic [31:0]           imem_rdata,
    output logic                  halted,
    output logic [31:0]           stat_cycles,
    output logic [31:0]           stat_committed,
    output logic [31:0]           stat_branches,
    output logic [31:0]           stat_flushes,
    output logic [31:0]           stat_squashed,
    input  logic [$clog2(NREG)-1:0] dbg_raddr,
    output logic [DW-1:0]         dbg_rdata
);
  localparam int RW  = $clog2(NREG);
  localparam int TW  = $clog2(ROBN);
  localparam int NRS = NADD + NMUL;

  logic [DW-1:0] regfile    [0:NREG-1];
  logic          regst_busy [0:NREG-1];
  logic [TW-1:0] regst_tag  [0:NREG-1];
  logic          rob_done [0:ROBN-1];
  logic          rob_wr   [0:ROBN-1];                 // writes a register at commit (ADD/SUB/MUL)
  logic          rob_br   [0:ROBN-1];
  logic          rob_halt [0:ROBN-1];
  logic          rob_pred [0:ROBN-1];                 // predicted taken
  logic [7:0]    rob_tgt  [0:ROBN-1];                 // pc if taken
  logic [7:0]    rob_ft   [0:ROBN-1];                 // pc if not taken
  logic [RW-1:0] rob_reg  [0:ROBN-1];
  logic [DW-1:0] rob_val  [0:ROBN-1];
  logic [TW-1:0] head, tail;
  logic [TW:0]   count;
  logic          rs_busy [0:NRS-1];
  logic          rs_exec [0:NRS-1];
  logic [1:0]    rs_op   [0:NRS-1];                   // 0 add, 1 sub, 2 mul, 3 branch (passes rs through)
  logic [DW-1:0] rs_vj   [0:NRS-1];
  logic [DW-1:0] rs_vk   [0:NRS-1];
  logic          rs_qjw  [0:NRS-1];
  logic          rs_qkw  [0:NRS-1];
  logic [TW-1:0] rs_qjt  [0:NRS-1];
  logic [TW-1:0] rs_qkt  [0:NRS-1];
  logic [TW-1:0] rs_dest [0:NRS-1];
  logic          u_busy [0:1];
  logic          u_done [0:1];
  logic [7:0]    u_rs   [0:1];
  logic [7:0]    u_rem  [0:1];
  logic [DW-1:0] u_res  [0:1];
  logic [7:0]    pc;
  logic          running, fetch_stop;

  assign dbg_rdata = regfile[dbg_raddr];
  assign imem_addr = pc;

  function automatic logic [DW-1:0] alu(input logic [1:0] op, input logic [DW-1:0] a, b);
    case (op)
      2'd0:    alu = a + b;
      2'd1:    alu = a - b;
      2'd2:    alu = a * b;
      default: alu = a;                                 // branch: the condition value
    endcase
  endfunction

  // ---- common data bus -------------------------------------------------------------------------
  logic          cdb_valid, cdb_unit;
  logic [TW-1:0] cdb_tag;
  logic [DW-1:0] cdb_val;
  always_comb begin
    cdb_valid = 1'b0; cdb_unit = 1'b0; cdb_tag = '0; cdb_val = '0;
    if (u_done[0])      begin cdb_valid = 1'b1; cdb_unit = 1'b0; end
    else if (u_done[1]) begin cdb_valid = 1'b1; cdb_unit = 1'b1; end
    if (cdb_valid) begin cdb_val = u_res[cdb_unit]; cdb_tag = rs_dest[int'(u_rs[cdb_unit])]; end
  end

  // ---- fetch/decode of the instruction at pc, free station, operand lookup ---------------------------
  wire [3:0]  i_op  = imem_rdata[31:28];
  wire [RW-1:0] i_rd = imem_rdata[24 +: RW];
  wire [RW-1:0] i_rs = imem_rdata[20 +: RW];
  wire [RW-1:0] i_rt = imem_rdata[16 +: RW];
  wire [7:0]  i_tgt = imem_rdata[7:0];
  wire is_alu = (i_op <= 4'd2);
  wire is_br  = (i_op == 4'd3);
  wire is_halt = (i_op == 4'd4);
  wire is_mul = (i_op == 4'd2);

  int free_rs;
  logic [RW-1:0] src_reg  [0:1];
  logic [DW-1:0] src_val  [0:1];
  logic          src_wait [0:1];
  logic [TW-1:0] src_tag  [0:1];
  logic          issue_ok;
  always_comb begin
    free_rs = -1;
    for (int i = 0; i < NRS; i++)
      if (!rs_busy[i] && free_rs < 0 && ((i >= NADD) == is_mul)) free_rs = i;
    issue_ok = running && !halted && !fetch_stop && (count < ROBN) && (is_halt || ((is_alu || is_br) && free_rs >= 0));

    src_reg[0] = i_rs;  src_reg[1] = i_rt;
    for (int s = 0; s < 2; s++) begin
      src_val[s] = regfile[src_reg[s]];  src_wait[s] = 1'b0;  src_tag[s] = '0;
      if (regst_busy[src_reg[s]]) begin
        src_tag[s] = regst_tag[src_reg[s]];
        if (cdb_valid && cdb_tag == src_tag[s])  src_val[s] = cdb_val;
        else if (rob_done[src_tag[s]])           src_val[s] = rob_val[src_tag[s]];
        else                                     src_wait[s] = 1'b1;
      end
    end
    if (is_br) begin src_val[1] = '0; src_wait[1] = 1'b0; end              // a branch has only one source
  end

  // ---- commit / misprediction detection --------------------------------------------------------------
  wire do_commit  = (count != 0) && rob_done[head];
  wire br_actual  = (rob_val[head] != '0);
  wire flush_now  = do_commit && rob_br[head] && (br_actual != rob_pred[head]);
  wire do_issue   = issue_ok && !flush_now;
  wire pred_taken = PREDICT_BTFN && is_br && (i_tgt < pc);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      head <= '0; tail <= '0; count <= '0; pc <= '0; running <= 1'b0; fetch_stop <= 1'b0; halted <= 1'b0;
      stat_cycles <= '0; stat_committed <= '0; stat_branches <= '0; stat_flushes <= '0; stat_squashed <= '0;
      for (int r = 0; r < NREG; r++) begin regfile[r] <= '0; regst_busy[r] <= 1'b0; regst_tag[r] <= '0; end
      for (int t = 0; t < ROBN; t++) begin
        rob_done[t] <= 1'b0; rob_wr[t] <= 1'b0; rob_br[t] <= 1'b0; rob_halt[t] <= 1'b0; rob_pred[t] <= 1'b0;
        rob_tgt[t] <= '0; rob_ft[t] <= '0; rob_reg[t] <= '0; rob_val[t] <= '0;
      end
      for (int i = 0; i < NRS; i++) begin
        rs_busy[i] <= 1'b0; rs_exec[i] <= 1'b0; rs_qjw[i] <= 1'b0; rs_qkw[i] <= 1'b0;
        rs_op[i] <= '0; rs_vj[i] <= '0; rs_vk[i] <= '0; rs_qjt[i] <= '0; rs_qkt[i] <= '0; rs_dest[i] <= '0;
      end
      for (int u = 0; u < 2; u++) begin u_busy[u] <= 1'b0; u_done[u] <= 1'b0; u_rs[u] <= '0; u_rem[u] <= '0; u_res[u] <= '0; end
    end else begin
      if (init_we && !running && count == 0) regfile[init_reg] <= init_data;
      if (start && !running) running <= 1'b1;
      if (running && !halted) stat_cycles <= stat_cycles + 1;

      // 1. CDB broadcast
      if (cdb_valid) begin
        rob_val[cdb_tag]  <= cdb_val;
        rob_done[cdb_tag] <= 1'b1;
        for (int i = 0; i < NRS; i++) begin
          if (rs_busy[i] && rs_qjw[i] && rs_qjt[i] == cdb_tag) begin rs_vj[i] <= cdb_val; rs_qjw[i] <= 1'b0; end
          if (rs_busy[i] && rs_qkw[i] && rs_qkt[i] == cdb_tag) begin rs_vk[i] <= cdb_val; rs_qkw[i] <= 1'b0; end
        end
      end

      // 2. functional units
      for (int u = 0; u < 2; u++) begin
        if (!u_busy[u]) begin : try_start
          int pick;  pick = -1;                               // oldest ready instruction first (smallest distance from the ROB head)
          for (int i = (u == 0 ? 0 : NADD); i < (u == 0 ? NADD : NRS); i++)
            if (rs_busy[i] && !rs_exec[i] && !rs_qjw[i] && !rs_qkw[i])
              if (pick < 0 || ((rs_dest[i] - head) & TW'(ROBN - 1)) < ((rs_dest[pick] - head) & TW'(ROBN - 1))) pick = i;
          if (pick >= 0) begin
            u_busy[u] <= 1'b1; u_done[u] <= 1'b0; u_rs[u] <= 8'(pick);
            u_rem[u] <= (u == 0) ? 8'(ADD_LAT) : 8'(MUL_LAT);
            rs_exec[pick] <= 1'b1;
          end
        end else if (!u_done[u]) begin
          if (u_rem[u] == 8'd1) begin
            u_done[u] <= 1'b1; u_rem[u] <= '0;
            u_res[u]  <= alu(rs_op[int'(u_rs[u])], rs_vj[int'(u_rs[u])], rs_vk[int'(u_rs[u])]);
          end else u_rem[u] <= u_rem[u] - 8'd1;
        end else if (cdb_valid && int'(cdb_unit) == u) begin
          u_busy[u] <= 1'b0; u_done[u] <= 1'b0;
          rs_busy[int'(u_rs[u])] <= 1'b0; rs_exec[int'(u_rs[u])] <= 1'b0;
        end
      end

      // 3. commit the head of the ROB, in program order
      if (do_commit) begin
        stat_committed <= stat_committed + 1;
        if (rob_wr[head]) begin
          regfile[rob_reg[head]] <= rob_val[head];
          if (regst_busy[rob_reg[head]] && regst_tag[rob_reg[head]] == head) regst_busy[rob_reg[head]] <= 1'b0;
        end
        if (rob_br[head]) stat_branches <= stat_branches + 1;
        if (rob_halt[head]) halted <= 1'b1;
        rob_done[head] <= 1'b0;
        head <= (head == TW'(ROBN - 1)) ? '0 : head + 1'b1;
      end

      // 4. issue: fetch the instruction at pc and follow the predicted path
      if (do_issue) begin
        rob_done[tail] <= is_halt;                         // HALT needs no execution
        rob_wr[tail]   <= is_alu;  rob_br[tail] <= is_br;  rob_halt[tail] <= is_halt;
        rob_pred[tail] <= pred_taken;  rob_tgt[tail] <= i_tgt;  rob_ft[tail] <= pc + 8'd1;
        rob_reg[tail]  <= i_rd;
        tail <= (tail == TW'(ROBN - 1)) ? '0 : tail + 1'b1;
        if (is_alu) begin regst_busy[i_rd] <= 1'b1; regst_tag[i_rd] <= tail; end
        if (is_alu || is_br) begin
          rs_busy[free_rs] <= 1'b1;  rs_exec[free_rs] <= 1'b0;  rs_op[free_rs] <= is_br ? 2'd3 : i_op[1:0];  rs_dest[free_rs] <= tail;
          rs_vj[free_rs] <= src_val[0];  rs_qjw[free_rs] <= src_wait[0];  rs_qjt[free_rs] <= src_tag[0];
          rs_vk[free_rs] <= src_val[1];  rs_qkw[free_rs] <= src_wait[1];  rs_qkt[free_rs] <= src_tag[1];
        end
        pc <= pred_taken ? i_tgt : pc + 8'd1;
        if (is_halt) fetch_stop <= 1'b1;
      end
      count <= count + (do_issue ? 1 : 0) - (do_commit ? 1 : 0);

      // 5. misprediction: throw away everything younger than the branch (it is at the head, so that is everything else)
      if (flush_now) begin
        stat_flushes  <= stat_flushes + 1;
        stat_squashed <= stat_squashed + (32'(count) - 32'd1);
        pc <= br_actual ? rob_tgt[head] : rob_ft[head];
        fetch_stop <= 1'b0;
        head <= '0; tail <= '0; count <= '0;
        for (int t = 0; t < ROBN; t++) rob_done[t] <= 1'b0;
        for (int i = 0; i < NRS; i++) begin rs_busy[i] <= 1'b0; rs_exec[i] <= 1'b0; rs_qjw[i] <= 1'b0; rs_qkw[i] <= 1'b0; end
        for (int u = 0; u < 2; u++) begin u_busy[u] <= 1'b0; u_done[u] <= 1'b0; end
        for (int r = 0; r < NREG; r++) regst_busy[r] <= 1'b0;
      end
    end
  end
endmodule
