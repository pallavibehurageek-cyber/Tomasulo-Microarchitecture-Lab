// Tomasulo with speculation AND a load/store queue (Chapter 3: hardware speculation, memory dependences).  See ../tomasulo_spec for the speculative core (reservation
// stations, CDB, reorder buffer, commit-time recovery); its timing rules are unchanged.  This version adds data memory, LOAD and STORE.
//
// ISA (32-bit words, 8 registers, 64 words of data memory, address = (rs + imm) mod 64):  op[31:28] | rd[27:24] | rs[23:20] | rt[19:16] | imm[15:0]
//   0 ADD rd=rs+rt   1 SUB   2 MUL   3 BNEZ rs,imm (branch target)   4 HALT   5 LD rd,[rs+imm]   6 ST rt,[rs+imm]
// Memory instructions go through a LOAD/STORE QUEUE (LSQ, NLSQ entries, allocated in program order, freed at commit) instead of the reservation stations:
//   * ADDRESS UNIT: one per cycle; it takes the oldest entry whose base register is available and computes the address (visible one edge later).
//   * STORES do not touch memory until they COMMIT (so a store on a wrong path can never corrupt memory): a store is "done" in the ROB one edge after both its address and
//     its data are known, and writes data memory at its commit edge.
//   * LOADS run on one load unit.  A load may start only when its address is known and the addresses of ALL OLDER STORES are known (conservative memory disambiguation).
//     If the youngest older store to the same address has its data, the load takes the value from it (STORE-TO-LOAD FORWARDING, FWD_LAT cycles; FORWARD = 0 disables it:
//     the load then waits until that store has committed and reads memory in LD_LAT cycles); if the data is not there yet the load waits; with no older store to the
//     same address the load reads memory (LD_LAT cycles).  Loads broadcast on the common data bus like ADD/MUL (priority: add unit, multiplier, load unit).
//   * A misprediction flushes the LSQ together with the ROB, stations and rename table.
// Timing, edge numbers with the first instruction issuing at edge 1 and a base register that is ready at issue (i = issue edge):
//   load: address at edge i+1, start at i+2, done at i+2+L, CDB at i+3+L, commit at i+4+L at the earliest        (L = LD_LAT, or FWD_LAT when forwarded)
//   store: address at edge i+1, marked done at i+2, commit (memory write) at i+3 at the earliest
// Statistics: as tomasulo_spec, plus stat_loads / stat_stores (committed) and stat_fwd (loads served by forwarding).
module tomasulo_lsq #(
    parameter int NREG        = 8,
    parameter int ROBN        = 8,
    parameter int NADD        = 3,
    parameter int NMUL        = 2,
    parameter int NLSQ        = 8,
    parameter int ADD_LAT     = 2,
    parameter int MUL_LAT     = 4,
    parameter int LD_LAT      = 3,
    parameter int FWD_LAT     = 1,
    parameter bit PREDICT_BTFN = 1'b0,
    parameter bit FORWARD     = 1'b1,
    parameter int DW          = 32
) (
    input  logic                  clk,
    input  logic                  rst_n,
    input  logic                  init_we,            // initialise registers while stopped and empty
    input  logic [$clog2(NREG)-1:0] init_reg,
    input  logic [DW-1:0]         init_data,
    input  logic                  dinit_we,           // initialise data memory while stopped and empty
    input  logic [5:0]            dinit_addr,
    input  logic [DW-1:0]         dinit_data,
    input  logic                  start,              // begin fetching at pc = 0 (first issue at the next edge)
    output logic [7:0]            imem_addr,
    input  logic [31:0]           imem_rdata,
    output logic                  halted,
    output logic [31:0]           stat_cycles,
    output logic [31:0]           stat_committed,
    output logic [31:0]           stat_branches,
    output logic [31:0]           stat_flushes,
    output logic [31:0]           stat_squashed,
    output logic [31:0]           stat_loads,
    output logic [31:0]           stat_stores,
    output logic [31:0]           stat_fwd,
    input  logic [$clog2(NREG)-1:0] dbg_raddr,
    output logic [DW-1:0]         dbg_rdata,
    input  logic [5:0]            dbg_maddr,
    output logic [DW-1:0]         dbg_mdata
);
  localparam int RW  = $clog2(NREG);
  localparam int TW  = $clog2(ROBN);
  localparam int LW  = $clog2(NLSQ);
  localparam int NRS = NADD + NMUL;

  logic [DW-1:0] regfile    [0:NREG-1];
  logic          regst_busy [0:NREG-1];
  logic [TW-1:0] regst_tag  [0:NREG-1];
  logic [DW-1:0] dmem [0:63];
  logic          rob_done [0:ROBN-1];
  logic          rob_wr   [0:ROBN-1];                 // writes a register at commit (ADD/SUB/MUL/LD)
  logic          rob_br   [0:ROBN-1];
  logic          rob_halt [0:ROBN-1];
  logic          rob_mem  [0:ROBN-1];                 // LD or ST (owns the oldest LSQ entry at commit)
  logic          rob_st   [0:ROBN-1];
  logic          rob_pred [0:ROBN-1];
  logic [7:0]    rob_tgt  [0:ROBN-1];
  logic [7:0]    rob_ft   [0:ROBN-1];
  logic [RW-1:0] rob_reg  [0:ROBN-1];
  logic [DW-1:0] rob_val  [0:ROBN-1];
  logic [TW-1:0] head, tail;
  logic [TW:0]   count;
  logic          rs_busy [0:NRS-1];
  logic          rs_exec [0:NRS-1];
  logic [1:0]    rs_op   [0:NRS-1];
  logic [DW-1:0] rs_vj   [0:NRS-1];
  logic [DW-1:0] rs_vk   [0:NRS-1];
  logic          rs_qjw  [0:NRS-1];
  logic          rs_qkw  [0:NRS-1];
  logic [TW-1:0] rs_qjt  [0:NRS-1];
  logic [TW-1:0] rs_qkt  [0:NRS-1];
  logic [TW-1:0] rs_dest [0:NRS-1];
  // load/store queue
  logic          lq_v     [0:NLSQ-1];
  logic          lq_st    [0:NLSQ-1];
  logic          lq_addr_v [0:NLSQ-1];
  logic          lq_exec  [0:NLSQ-1];                 // a load that has started
  logic [5:0]    lq_addr  [0:NLSQ-1];
  logic [DW-1:0] lq_vj    [0:NLSQ-1];                 // base register
  logic          lq_qjw   [0:NLSQ-1];
  logic [TW-1:0] lq_qjt   [0:NLSQ-1];
  logic [DW-1:0] lq_vk    [0:NLSQ-1];                 // store data
  logic          lq_qkw   [0:NLSQ-1];
  logic [TW-1:0] lq_qkt   [0:NLSQ-1];
  logic [15:0]   lq_imm   [0:NLSQ-1];
  logic [TW-1:0] lq_rob   [0:NLSQ-1];
  logic [LW-1:0] lsq_head, lsq_tail;
  logic [LW:0]   lsq_cnt;
  // units: 0 add, 1 multiply, 2 load
  logic          u_busy [0:2];
  logic          u_done [0:2];
  logic [7:0]    u_rs   [0:2];
  logic [7:0]    u_rem  [0:2];
  logic [DW-1:0] u_res  [0:2];
  logic          u_fwd;                                 // the load unit is serving a forwarded value
  logic [DW-1:0] u_fv;
  logic [7:0]    pc;
  logic          running, fetch_stop;

  assign dbg_rdata = regfile[dbg_raddr];
  assign dbg_mdata = dmem[dbg_maddr];
  assign imem_addr = pc;

  function automatic logic [DW-1:0] alu(input logic [1:0] op, input logic [DW-1:0] a, b);
    case (op)
      2'd0:    alu = a + b;
      2'd1:    alu = a - b;
      2'd2:    alu = a * b;
      default: alu = a;                                 // branch: the condition value
    endcase
  endfunction

  // ---- common data bus ------------------------------------------------------------------------------
  logic          cdb_valid;
  logic [1:0]    cdb_unit;
  logic [TW-1:0] cdb_tag;
  logic [DW-1:0] cdb_val;
  always_comb begin
    cdb_valid = 1'b0; cdb_unit = 2'd0; cdb_tag = '0; cdb_val = '0;
    if (u_done[0])      begin cdb_valid = 1'b1; cdb_unit = 2'd0; end
    else if (u_done[1]) begin cdb_valid = 1'b1; cdb_unit = 2'd1; end
    else if (u_done[2]) begin cdb_valid = 1'b1; cdb_unit = 2'd2; end
    if (cdb_valid) begin
      cdb_val = u_res[cdb_unit];
      cdb_tag = (cdb_unit == 2'd2) ? lq_rob[int'(u_rs[2])] : rs_dest[int'(u_rs[cdb_unit])];
    end
  end

  // ---- fetch/decode of the instruction at pc, free station, operand lookup ---------------------------------
  wire [3:0]  i_op  = imem_rdata[31:28];
  wire [RW-1:0] i_rd = imem_rdata[24 +: RW];
  wire [RW-1:0] i_rs = imem_rdata[20 +: RW];
  wire [RW-1:0] i_rt = imem_rdata[16 +: RW];
  wire [15:0] i_imm = imem_rdata[15:0];
  wire [7:0]  i_tgt = imem_rdata[7:0];
  wire is_alu = (i_op <= 4'd2);
  wire is_br  = (i_op == 4'd3);
  wire is_halt = (i_op == 4'd4);
  wire is_ld  = (i_op == 4'd5);
  wire is_st  = (i_op == 4'd6);
  wire is_mem = is_ld || is_st;
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
    issue_ok = running && !halted && !fetch_stop && (count < ROBN) &&
               (is_halt || ((is_alu || is_br) && free_rs >= 0) || (is_mem && lsq_cnt < NLSQ));

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
    if (is_br || is_ld) begin src_val[1] = '0; src_wait[1] = 1'b0; end      // one source only
  end

  // ---- commit / misprediction detection -------------------------------------------------------------------
  wire do_commit  = (count != 0) && rob_done[head];
  wire br_actual  = (rob_val[head] != '0);
  wire flush_now  = do_commit && rob_br[head] && (br_actual != rob_pred[head]);
  wire do_issue   = issue_ok && !flush_now;
  wire pred_taken = PREDICT_BTFN && is_br && (i_tgt < pc);
  wire commit_mem = do_commit && rob_mem[head];
  wire issue_mem  = do_issue && is_mem;

  // oldest-first helper: distance of a queue entry from the head of the LSQ
  function automatic int ldist(input int j, input int h);
    return (j + NLSQ - h) % NLSQ;
  endfunction

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      head <= '0; tail <= '0; count <= '0; pc <= '0; running <= 1'b0; fetch_stop <= 1'b0; halted <= 1'b0;
      lsq_head <= '0; lsq_tail <= '0; lsq_cnt <= '0;
      stat_cycles <= '0; stat_committed <= '0; stat_branches <= '0; stat_flushes <= '0; stat_squashed <= '0;
      stat_loads <= '0; stat_stores <= '0; stat_fwd <= '0;
      for (int r = 0; r < NREG; r++) begin regfile[r] <= '0; regst_busy[r] <= 1'b0; regst_tag[r] <= '0; end
      for (int a = 0; a < 64; a++) dmem[a] <= '0;
      for (int t = 0; t < ROBN; t++) begin
        rob_done[t] <= 1'b0; rob_wr[t] <= 1'b0; rob_br[t] <= 1'b0; rob_halt[t] <= 1'b0; rob_pred[t] <= 1'b0; rob_mem[t] <= 1'b0; rob_st[t] <= 1'b0;
        rob_tgt[t] <= '0; rob_ft[t] <= '0; rob_reg[t] <= '0; rob_val[t] <= '0;
      end
      for (int i = 0; i < NRS; i++) begin
        rs_busy[i] <= 1'b0; rs_exec[i] <= 1'b0; rs_qjw[i] <= 1'b0; rs_qkw[i] <= 1'b0;
        rs_op[i] <= '0; rs_vj[i] <= '0; rs_vk[i] <= '0; rs_qjt[i] <= '0; rs_qkt[i] <= '0; rs_dest[i] <= '0;
      end
      for (int j = 0; j < NLSQ; j++) begin
        lq_v[j] <= 1'b0; lq_st[j] <= 1'b0; lq_addr_v[j] <= 1'b0; lq_exec[j] <= 1'b0; lq_addr[j] <= '0; lq_vj[j] <= '0; lq_qjw[j] <= 1'b0; lq_qjt[j] <= '0;
        lq_vk[j] <= '0; lq_qkw[j] <= 1'b0; lq_qkt[j] <= '0; lq_imm[j] <= '0; lq_rob[j] <= '0;
      end
      for (int u = 0; u < 3; u++) begin u_busy[u] <= 1'b0; u_done[u] <= 1'b0; u_rs[u] <= '0; u_rem[u] <= '0; u_res[u] <= '0; end
      u_fwd <= 1'b0;  u_fv <= '0;
    end else begin
      if (init_we && !running && count == 0) regfile[init_reg] <= init_data;
      if (dinit_we && !running && count == 0) dmem[dinit_addr] <= dinit_data;
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
        for (int j = 0; j < NLSQ; j++) begin
          if (lq_v[j] && lq_qjw[j] && lq_qjt[j] == cdb_tag) begin lq_vj[j] <= cdb_val; lq_qjw[j] <= 1'b0; end
          if (lq_v[j] && lq_qkw[j] && lq_qkt[j] == cdb_tag) begin lq_vk[j] <= cdb_val; lq_qkw[j] <= 1'b0; end
        end
      end

      // 2. functional units (add, multiply, load)
      for (int u = 0; u < 3; u++) begin
        if (!u_busy[u]) begin : try_start
          int pick, ms;  logic ok, blocked, fwd;
          pick = -1;  ms = -1;  fwd = 1'b0;
          if (u < 2) begin
            for (int i = (u == 0 ? 0 : NADD); i < (u == 0 ? NADD : NRS); i++)
              if (rs_busy[i] && !rs_exec[i] && !rs_qjw[i] && !rs_qkw[i])
                if (pick < 0 || ((rs_dest[i] - head) & TW'(ROBN - 1)) < ((rs_dest[pick] - head) & TW'(ROBN - 1))) pick = i;
            if (pick >= 0) begin
              u_busy[u] <= 1'b1; u_done[u] <= 1'b0; u_rs[u] <= 8'(pick);
              u_rem[u] <= (u == 0) ? 8'(ADD_LAT) : 8'(MUL_LAT);
              rs_exec[pick] <= 1'b1;
            end
          end else begin
            // the oldest load whose address is known and which no older store can still change
            for (int j = 0; j < NLSQ; j++)
              if (lq_v[j] && !lq_st[j] && !lq_exec[j] && lq_addr_v[j]) begin
                ok = 1'b1;  blocked = 1'b0;  ms = -1;
                for (int k = 0; k < NLSQ; k++)
                  if (lq_v[k] && lq_st[k] && ldist(k, int'(lsq_head)) < ldist(j, int'(lsq_head))) begin
                    if (!lq_addr_v[k]) blocked = 1'b1;                                          // an older store with an unknown address
                    else if (lq_addr[k] == lq_addr[j]) begin
                      if (ms < 0 || ldist(k, int'(lsq_head)) > ldist(ms, int'(lsq_head))) ms = k;   // the youngest older store to the same address
                    end
                  end
                if (blocked) ok = 1'b0;
                if (ok && ms >= 0 && (!FORWARD || lq_qkw[ms])) ok = 1'b0;                      // must wait for the store (data or commit)
                if (ok && (pick < 0 || ldist(j, int'(lsq_head)) < ldist(pick, int'(lsq_head)))) begin pick = j; fwd = (ms >= 0); u_fv <= (ms >= 0) ? lq_vk[ms] : '0; end
              end
            if (pick >= 0) begin
              u_busy[2] <= 1'b1; u_done[2] <= 1'b0; u_rs[2] <= 8'(pick);
              u_rem[2] <= fwd ? 8'(FWD_LAT) : 8'(LD_LAT);
              u_fwd <= fwd;
              lq_exec[pick] <= 1'b1;
              if (fwd) stat_fwd <= stat_fwd + 1;
            end
          end
        end else if (!u_done[u]) begin
          if (u_rem[u] == 8'd1) begin
            u_done[u] <= 1'b1; u_rem[u] <= '0;
            if (u < 2) u_res[u] <= alu(rs_op[int'(u_rs[u])], rs_vj[int'(u_rs[u])], rs_vk[int'(u_rs[u])]);
            else       u_res[u] <= u_fwd ? u_fv : dmem[lq_addr[int'(u_rs[2])]];
          end else u_rem[u] <= u_rem[u] - 8'd1;
        end else if (cdb_valid && int'(cdb_unit) == u) begin
          u_busy[u] <= 1'b0; u_done[u] <= 1'b0;
          if (u < 2) begin rs_busy[int'(u_rs[u])] <= 1'b0; rs_exec[int'(u_rs[u])] <= 1'b0; end
        end
      end

      // 3. address unit: the oldest queue entry with an available base register gets its address
      begin : agu
        int pa;  pa = -1;
        for (int j = 0; j < NLSQ; j++)
          if (lq_v[j] && !lq_addr_v[j] && !lq_qjw[j])
            if (pa < 0 || ldist(j, int'(lsq_head)) < ldist(pa, int'(lsq_head))) pa = j;
        if (pa >= 0) begin
          lq_addr[pa] <= 6'(lq_vj[pa] + {16'b0, lq_imm[pa]});
          lq_addr_v[pa] <= 1'b1;
        end
      end
      // a store is done (ready to commit) one edge after its address and data are both known
      for (int j = 0; j < NLSQ; j++)
        if (lq_v[j] && lq_st[j] && lq_addr_v[j] && !lq_qkw[j] && !rob_done[lq_rob[j]]) rob_done[lq_rob[j]] <= 1'b1;

      // 4. commit the head of the ROB, in program order
      if (do_commit) begin
        stat_committed <= stat_committed + 1;
        if (rob_wr[head]) begin
          regfile[rob_reg[head]] <= rob_val[head];
          if (regst_busy[rob_reg[head]] && regst_tag[rob_reg[head]] == head) regst_busy[rob_reg[head]] <= 1'b0;
        end
        if (rob_br[head]) stat_branches <= stat_branches + 1;
        if (rob_halt[head]) halted <= 1'b1;
        if (rob_mem[head]) begin
          if (rob_st[head]) begin dmem[lq_addr[lsq_head]] <= lq_vk[lsq_head]; stat_stores <= stat_stores + 1; end
          else stat_loads <= stat_loads + 1;
          lq_v[lsq_head] <= 1'b0;  lq_addr_v[lsq_head] <= 1'b0;
          lsq_head <= (int'(lsq_head) == NLSQ - 1) ? '0 : lsq_head + 1'b1;
        end
        rob_done[head] <= 1'b0;
        head <= (head == TW'(ROBN - 1)) ? '0 : head + 1'b1;
      end

      // 5. issue: fetch the instruction at pc and follow the predicted path
      if (do_issue) begin
        rob_done[tail] <= is_halt;                         // HALT needs no execution
        rob_wr[tail]   <= is_alu || is_ld;  rob_br[tail] <= is_br;  rob_halt[tail] <= is_halt;  rob_mem[tail] <= is_mem;  rob_st[tail] <= is_st;
        rob_pred[tail] <= pred_taken;  rob_tgt[tail] <= i_tgt;  rob_ft[tail] <= pc + 8'd1;
        rob_reg[tail]  <= i_rd;
        tail <= (tail == TW'(ROBN - 1)) ? '0 : tail + 1'b1;
        if (is_alu || is_ld) begin regst_busy[i_rd] <= 1'b1; regst_tag[i_rd] <= tail; end
        if (is_alu || is_br) begin
          rs_busy[free_rs] <= 1'b1;  rs_exec[free_rs] <= 1'b0;  rs_op[free_rs] <= is_br ? 2'd3 : i_op[1:0];  rs_dest[free_rs] <= tail;
          rs_vj[free_rs] <= src_val[0];  rs_qjw[free_rs] <= src_wait[0];  rs_qjt[free_rs] <= src_tag[0];
          rs_vk[free_rs] <= src_val[1];  rs_qkw[free_rs] <= src_wait[1];  rs_qkt[free_rs] <= src_tag[1];
        end
        if (is_mem) begin
          lq_v[lsq_tail] <= 1'b1;  lq_st[lsq_tail] <= is_st;  lq_addr_v[lsq_tail] <= 1'b0;  lq_exec[lsq_tail] <= 1'b0;  lq_imm[lsq_tail] <= i_imm;  lq_rob[lsq_tail] <= tail;
          lq_vj[lsq_tail] <= src_val[0];  lq_qjw[lsq_tail] <= src_wait[0];  lq_qjt[lsq_tail] <= src_tag[0];
          lq_vk[lsq_tail] <= src_val[1];  lq_qkw[lsq_tail] <= src_wait[1];  lq_qkt[lsq_tail] <= src_tag[1];
          lsq_tail <= (int'(lsq_tail) == NLSQ - 1) ? '0 : lsq_tail + 1'b1;
        end
        pc <= pred_taken ? i_tgt : pc + 8'd1;
        if (is_halt) fetch_stop <= 1'b1;
      end
      count <= count + (do_issue ? 1 : 0) - (do_commit ? 1 : 0);
      lsq_cnt <= lsq_cnt + (issue_mem ? 1 : 0) - (commit_mem ? 1 : 0);

      // 6. misprediction: throw away everything younger than the branch (it is at the head, so that is everything else)
      if (flush_now) begin
        stat_flushes  <= stat_flushes + 1;
        stat_squashed <= stat_squashed + (32'(count) - 32'd1);
        pc <= br_actual ? rob_tgt[head] : rob_ft[head];
        fetch_stop <= 1'b0;
        head <= '0; tail <= '0; count <= '0;
        lsq_head <= '0; lsq_tail <= '0; lsq_cnt <= '0;
        for (int t = 0; t < ROBN; t++) rob_done[t] <= 1'b0;
        for (int i = 0; i < NRS; i++) begin rs_busy[i] <= 1'b0; rs_exec[i] <= 1'b0; rs_qjw[i] <= 1'b0; rs_qkw[i] <= 1'b0; end
        for (int j = 0; j < NLSQ; j++) begin lq_v[j] <= 1'b0; lq_addr_v[j] <= 1'b0; lq_exec[j] <= 1'b0; lq_qjw[j] <= 1'b0; lq_qkw[j] <= 1'b0; end
        for (int u = 0; u < 3; u++) begin u_busy[u] <= 1'b0; u_done[u] <= 1'b0; end
        for (int r = 0; r < NREG; r++) regst_busy[r] <= 1'b0;
      end
    end
  end
endmodule
