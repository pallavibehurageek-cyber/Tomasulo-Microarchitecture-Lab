// Tomasulo's algorithm with a reorder buffer (ROB), Chapter 3.
//
//   issue (in order) -> reservation station -> execute (out of order) -> CDB broadcast -> commit (in order)
//
// ISA: three-register ops on NREG registers:  0 ADD  rd = rs + rt    1 SUB  rd = rs - rt    2 MUL  rd = rs * rt
// Resources: NADD add/sub reservation stations sharing ONE non-pipelined add unit (ADD_LAT cycles),
//            NMUL mul reservation stations sharing ONE non-pipelined multiplier (MUL_LAT cycles),
//            a ROBN-entry reorder buffer, and ONE common data bus (CDB): one result broadcast per cycle
//            (add unit wins a tie; the multiplier holds its result until the CDB is free).
//
// Exact timing, counting clock edges (edge 1 = the edge at which instruction 0 is accepted):
//   issue edge I     : ROB entry + reservation station allocated, sources read or tagged, rd renamed to the ROB tag
//   edge I+1 (earliest): execution starts (needs both operands in the station as of the previous edge)
//   edge S+LAT       : result computed ("done")                  -> unit holds it until it wins the CDB
//   next edge        : CDB broadcast: ROB entry done, waiting stations capture the value, station + unit freed
//   next edge        : commit (head of ROB only): architectural register written, ROB entry freed
// So an independent ADD issued at edge I commits at I+5; an independent MUL at I+7.
// A freed station/ROB entry is reusable one cycle later (issue_ready looks at the state before the edge).
// No branches or memory: speculation past branches is left as an exercise (the ROB is what would make it possible).
module tomasulo_rob #(
    parameter int NREG    = 8,
    parameter int ROBN    = 8,
    parameter int NADD    = 3,
    parameter int NMUL    = 2,
    parameter int ADD_LAT = 2,
    parameter int MUL_LAT = 4,
    parameter int DW      = 32
) (
    input  logic                  clk,
    input  logic                  rst_n,
    // initialise architectural registers (use only while the machine is empty)
    input  logic                  init_we,
    input  logic [$clog2(NREG)-1:0] init_reg,
    input  logic [DW-1:0]         init_data,
    // instruction issue: accepted at a clock edge when issue_valid && issue_ready
    input  logic                  issue_valid,
    input  logic [1:0]            issue_op,
    input  logic [$clog2(NREG)-1:0] issue_rd, issue_rs, issue_rt,
    output logic                  issue_ready,
    // observation
    output logic                  cdb_valid,        // a broadcast will happen at the next edge
    output logic [$clog2(ROBN)-1:0] cdb_tag,
    output logic                  commit_valid,     // registered: an instruction committed at the previous edge
    output logic [$clog2(NREG)-1:0] commit_reg,
    output logic [DW-1:0]         commit_data,
    output logic                  empty,            // nothing in flight
    input  logic [$clog2(NREG)-1:0] dbg_raddr,
    output logic [DW-1:0]         dbg_rdata
);
  localparam int RW  = $clog2(NREG);
  localparam int TW  = $clog2(ROBN);
  localparam int NRS = NADD + NMUL;

  // architectural state
  logic [DW-1:0] regfile    [0:NREG-1];
  logic          regst_busy [0:NREG-1];            // register-status table (renaming)
  logic [TW-1:0] regst_tag  [0:NREG-1];
  // reorder buffer
  logic          rob_done [0:ROBN-1];
  logic [RW-1:0] rob_reg  [0:ROBN-1];
  logic [DW-1:0] rob_val  [0:ROBN-1];
  logic [TW-1:0] head, tail;
  logic [TW:0]   count;
  // reservation stations (0..NADD-1 add type, NADD..NRS-1 mul type)
  logic          rs_busy [0:NRS-1];
  logic          rs_exec [0:NRS-1];                 // already sent to a functional unit
  logic [1:0]    rs_op   [0:NRS-1];
  logic [DW-1:0] rs_vj   [0:NRS-1];
  logic [DW-1:0] rs_vk   [0:NRS-1];
  logic          rs_qjw  [0:NRS-1];                 // still waiting for operand j
  logic          rs_qkw  [0:NRS-1];
  logic [TW-1:0] rs_qjt  [0:NRS-1];
  logic [TW-1:0] rs_qkt  [0:NRS-1];
  logic [TW-1:0] rs_dest [0:NRS-1];
  // functional units: 0 = add/sub, 1 = multiplier
  logic          u_busy [0:1];
  logic          u_done [0:1];
  logic [7:0]    u_rs   [0:1];
  logic [7:0]    u_rem  [0:1];
  logic [DW-1:0] u_res  [0:1];

  assign dbg_rdata = regfile[dbg_raddr];
  assign empty     = (count == 0);

  function automatic logic [DW-1:0] alu(input logic [1:0] op, input logic [DW-1:0] a, b);
    case (op)
      2'd0:    alu = a + b;
      2'd1:    alu = a - b;
      default: alu = a * b;
    endcase
  endfunction

  // ---- common data bus arbitration ----------------------------------------------------------
  logic          cdb_unit;
  logic [DW-1:0] cdb_val;
  always_comb begin
    cdb_valid = 1'b0; cdb_unit = 1'b0; cdb_tag = '0; cdb_val = '0;
    if (u_done[0])      begin cdb_valid = 1'b1; cdb_unit = 1'b0; end
    else if (u_done[1]) begin cdb_valid = 1'b1; cdb_unit = 1'b1; end
    if (cdb_valid) begin cdb_val = u_res[cdb_unit]; cdb_tag = rs_dest[int'(u_rs[cdb_unit])]; end
  end

  // ---- issue: free station, operand lookup --------------------------------------------------
  int free_rs;
  logic [RW-1:0] src_reg  [0:1];
  logic [DW-1:0] src_val  [0:1];
  logic          src_wait [0:1];
  logic [TW-1:0] src_tag  [0:1];
  always_comb begin
    free_rs = -1;
    for (int i = 0; i < NRS; i++)
      if (!rs_busy[i] && free_rs < 0 && ((i >= NADD) == (issue_op == 2'd2))) free_rs = i;
    issue_ready = (count < ROBN) && (free_rs >= 0);

    src_reg[0] = issue_rs;  src_reg[1] = issue_rt;
    for (int s = 0; s < 2; s++) begin
      src_val[s] = regfile[src_reg[s]];  src_wait[s] = 1'b0;  src_tag[s] = '0;
      if (regst_busy[src_reg[s]]) begin
        src_tag[s] = regst_tag[src_reg[s]];
        if (cdb_valid && cdb_tag == src_tag[s])  src_val[s] = cdb_val;               // caught off the CDB
        else if (rob_done[src_tag[s]])           src_val[s] = rob_val[src_tag[s]];   // already in the ROB
        else                                     src_wait[s] = 1'b1;                 // wait for the broadcast
      end
    end
  end

  wire do_issue  = issue_valid && issue_ready;
  wire do_commit = (count != 0) && rob_done[head];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      head <= '0; tail <= '0; count <= '0; commit_valid <= 1'b0; commit_reg <= '0; commit_data <= '0;
      for (int r = 0; r < NREG; r++) begin regfile[r] <= '0; regst_busy[r] <= 1'b0; regst_tag[r] <= '0; end
      for (int t = 0; t < ROBN; t++) begin rob_done[t] <= 1'b0; rob_reg[t] <= '0; rob_val[t] <= '0; end
      for (int i = 0; i < NRS; i++) begin
        rs_busy[i] <= 1'b0; rs_exec[i] <= 1'b0; rs_qjw[i] <= 1'b0; rs_qkw[i] <= 1'b0;
        rs_op[i] <= '0; rs_vj[i] <= '0; rs_vk[i] <= '0; rs_qjt[i] <= '0; rs_qkt[i] <= '0; rs_dest[i] <= '0;
      end
      for (int u = 0; u < 2; u++) begin u_busy[u] <= 1'b0; u_done[u] <= 1'b0; u_rs[u] <= '0; u_rem[u] <= '0; u_res[u] <= '0; end
    end else begin
      if (init_we && count == 0) regfile[init_reg] <= init_data;

      // 1. CDB broadcast: ROB entry completes, waiting stations capture the value
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
          int pick;  pick = -1;
          for (int i = (u == 0 ? 0 : NADD); i < (u == 0 ? NADD : NRS); i++)
            if (pick < 0 && rs_busy[i] && !rs_exec[i] && !rs_qjw[i] && !rs_qkw[i]) pick = i;
          if (pick >= 0) begin
            u_busy[u] <= 1'b1; u_done[u] <= 1'b0; u_rs[u] <= 8'(pick);
            u_rem[u] <= (u == 0) ? 8'(ADD_LAT) : 8'(MUL_LAT);
            rs_exec[pick] <= 1'b1;
          end
        end else if (!u_done[u]) begin
          if (u_rem[u] == 1) begin
            u_done[u] <= 1'b1; u_rem[u] <= '0;
            u_res[u]  <= alu(rs_op[int'(u_rs[u])], rs_vj[int'(u_rs[u])], rs_vk[int'(u_rs[u])]);
          end else u_rem[u] <= u_rem[u] - 8'd1;
        end else if (cdb_valid && int'(cdb_unit) == u) begin       // result broadcast this edge: free everything
          u_busy[u] <= 1'b0; u_done[u] <= 1'b0;
          rs_busy[int'(u_rs[u])] <= 1'b0; rs_exec[int'(u_rs[u])] <= 1'b0;
        end
      end

      // 3. commit the head of the ROB, in program order
      commit_valid <= do_commit;
      if (do_commit) begin
        regfile[rob_reg[head]] <= rob_val[head];
        commit_reg  <= rob_reg[head];
        commit_data <= rob_val[head];
        if (regst_busy[rob_reg[head]] && regst_tag[rob_reg[head]] == head) regst_busy[rob_reg[head]] <= 1'b0;
        rob_done[head] <= 1'b0;
        head <= (head == TW'(ROBN - 1)) ? '0 : head + 1'b1;
      end

      // 4. issue (last, so a new rename overrides a commit-time clear of the same register)
      if (do_issue) begin
        rob_done[tail] <= 1'b0;
        rob_reg[tail]  <= issue_rd;
        tail <= (tail == TW'(ROBN - 1)) ? '0 : tail + 1'b1;
        regst_busy[issue_rd] <= 1'b1;
        regst_tag[issue_rd]  <= tail;
        rs_busy[free_rs] <= 1'b1;  rs_exec[free_rs] <= 1'b0;  rs_op[free_rs] <= issue_op;  rs_dest[free_rs] <= tail;
        rs_vj[free_rs] <= src_val[0];  rs_qjw[free_rs] <= src_wait[0];  rs_qjt[free_rs] <= src_tag[0];
        rs_vk[free_rs] <= src_val[1];  rs_qkw[free_rs] <= src_wait[1];  rs_qkt[free_rs] <= src_tag[1];
      end
      count <= count + (do_issue ? 1 : 0) - (do_commit ? 1 : 0);
    end
  end
endmodule
