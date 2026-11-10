// The CDC 6600 scoreboard (Chapter 3 in the early editions; Appendix C of the later ones).
//
// In-order issue, out-of-order execution, NO renaming and NO forwarding bus.  Every instruction goes through four steps:
//   Issue          : needs a free functional unit (structural) and no pending write to the same destination (WAW).
//   Read operands  : waits until BOTH source registers hold their final values (RAW); values are read from the register file.
//   Execute        : ADD_LAT (add/sub) or MUL_LAT (mul) cycles.
//   Write result   : waits until no earlier instruction still has to READ the old value of the destination (WAR).
// Units: FU0 = add/sub, FU1 and FU2 = multipliers.  ISA: 0 ADD  1 SUB  2 MUL, three registers, rd = rs op rt.
//
// Timing, counting clock edges (the edge at which an instruction is accepted is its issue edge I):
//   read  edge R >= I+1 and >= (the edge a producer wrote) + 1
//   execute occupies edges R+1 .. R+LAT, the result is ready after edge R+LAT
//   write edge W >= R+LAT+1, and W >= (the edge every older reader of rd read it) + 1
//   a unit and a destination register become reusable by the NEXT edge after W (issue looks at the state before the edge).
// Observation outputs (combinational, describing what happens at the NEXT edge): read_go[f], write_go[f], fu_seq[f]
// (a sequence number identifying the instruction in unit f), so a testbench can reconstruct every instruction's timeline.
module scoreboard #(
    parameter int NREG    = 8,
    parameter int DW      = 32,
    parameter int ADD_LAT = 2,
    parameter int MUL_LAT = 4
) (
    input  logic                    clk,
    input  logic                    rst_n,
    input  logic                    init_we,                 // initialise registers while the machine is empty
    input  logic [$clog2(NREG)-1:0] init_reg,
    input  logic [DW-1:0]           init_data,
    input  logic                    issue_valid,
    input  logic [1:0]              issue_op,
    input  logic [$clog2(NREG)-1:0] issue_rd, issue_rs, issue_rt,
    output logic                    issue_ready,
    output logic [2:0]              read_go,
    output logic [2:0]              write_go,
    output logic [47:0]             fu_seq,                  // 16 bits per unit
    output logic                    empty,
    input  logic [$clog2(NREG)-1:0] dbg_raddr,
    output logic [DW-1:0]           dbg_rdata
);
  localparam int RW  = $clog2(NREG);
  localparam int NFU = 3;
  localparam logic [1:0] PH_WAIT = 2'd0, PH_EXEC = 2'd1, PH_DONE = 2'd2;

  logic [DW-1:0] regfile  [0:NREG-1];
  logic          res_busy [0:NREG-1];                 // "result register status": which unit will write each register
  logic [1:0]    res_fu   [0:NREG-1];

  logic          busy [0:NFU-1];
  logic [1:0]    f_op [0:NFU-1];
  logic [RW-1:0] f_fi [0:NFU-1];
  logic [RW-1:0] f_fj [0:NFU-1];
  logic [RW-1:0] f_fk [0:NFU-1];
  logic          qjw  [0:NFU-1];                      // operand j still being produced by unit qj
  logic          qkw  [0:NFU-1];
  logic [1:0]    qj   [0:NFU-1];
  logic [1:0]    qk   [0:NFU-1];
  logic          rj   [0:NFU-1];                      // operand j is ready and not yet read   (Rj in the textbook)
  logic          rk   [0:NFU-1];
  logic [1:0]    phase [0:NFU-1];
  logic [7:0]    rem  [0:NFU-1];
  logic [DW-1:0] vj   [0:NFU-1];
  logic [DW-1:0] vk   [0:NFU-1];
  logic [DW-1:0] res  [0:NFU-1];
  logic [15:0]   seq  [0:NFU-1];
  logic [15:0]   seq_ctr;

  assign dbg_rdata = regfile[dbg_raddr];

  function automatic logic [DW-1:0] alu(input logic [1:0] op, input logic [DW-1:0] a, b);
    case (op)
      2'd0:    alu = a + b;
      2'd1:    alu = a - b;
      default: alu = a * b;
    endcase
  endfunction

  // ---- per-unit "go" conditions ---------------------------------------------------------------
  logic war_block [0:NFU-1];
  always_comb begin
    empty = 1'b1;
    for (int f = 0; f < NFU; f++) begin
      if (busy[f]) empty = 1'b0;
      war_block[f] = 1'b0;
      for (int g = 0; g < NFU; g++)
        if (g != f && busy[g] && ((f_fj[g] == f_fi[f] && rj[g]) || (f_fk[g] == f_fi[f] && rk[g]))) war_block[f] = 1'b1;
      read_go[f]  = busy[f] && phase[f] == PH_WAIT && rj[f] && rk[f];
      write_go[f] = busy[f] && phase[f] == PH_DONE && !war_block[f];
      fu_seq[16*f +: 16] = seq[f];
    end
  end

  // ---- issue ----------------------------------------------------------------------------------
  logic [1:0] pick;  logic have_fu;
  logic       s_wait [0:1];  logic [1:0] s_fu [0:1];  logic [RW-1:0] s_reg [0:1];
  always_comb begin
    have_fu = 1'b0;  pick = 2'd0;
    if (issue_op == 2'd2) begin
      if (!busy[1])      begin have_fu = 1'b1; pick = 2'd1; end
      else if (!busy[2]) begin have_fu = 1'b1; pick = 2'd2; end
    end else if (!busy[0]) begin have_fu = 1'b1; pick = 2'd0; end
    issue_ready = have_fu && !res_busy[issue_rd];                       // structural and WAW
    s_reg[0] = issue_rs;  s_reg[1] = issue_rt;
    for (int s = 0; s < 2; s++) begin
      s_wait[s] = res_busy[s_reg[s]];  s_fu[s] = res_fu[s_reg[s]];
      if (s_wait[s] && write_go[s_fu[s]]) s_wait[s] = 1'b0;             // producer writes at this very edge: value is there next cycle
    end
  end

  wire do_issue = issue_valid && issue_ready;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      seq_ctr <= '0;
      for (int r = 0; r < NREG; r++) begin regfile[r] <= '0; res_busy[r] <= 1'b0; res_fu[r] <= '0; end
      for (int f = 0; f < NFU; f++) begin
        busy[f] <= 1'b0; f_op[f] <= '0; f_fi[f] <= '0; f_fj[f] <= '0; f_fk[f] <= '0; qjw[f] <= 1'b0; qkw[f] <= 1'b0;
        qj[f] <= '0; qk[f] <= '0; rj[f] <= 1'b0; rk[f] <= 1'b0; phase[f] <= PH_WAIT; rem[f] <= '0;
        vj[f] <= '0; vk[f] <= '0; res[f] <= '0; seq[f] <= '0;
      end
    end else begin
      if (init_we && empty) regfile[init_reg] <= init_data;
      for (int f = 0; f < NFU; f++) begin
        // read operands: both ready -> latch values and start executing
        if (read_go[f]) begin
          vj[f] <= regfile[f_fj[f]];  vk[f] <= regfile[f_fk[f]];
          rj[f] <= 1'b0;  rk[f] <= 1'b0;
          phase[f] <= PH_EXEC;  rem[f] <= (f == 0) ? 8'(ADD_LAT) : 8'(MUL_LAT);
        end
        // execute
        if (busy[f] && phase[f] == PH_EXEC) begin
          if (rem[f] == 8'd1) begin res[f] <= alu(f_op[f], vj[f], vk[f]); phase[f] <= PH_DONE; end
          rem[f] <= rem[f] - 8'd1;
        end
        // write result: update the register, free the unit, wake the units waiting for this result
        if (write_go[f]) begin
          regfile[f_fi[f]] <= res[f];
          res_busy[f_fi[f]] <= 1'b0;
          busy[f] <= 1'b0;
          for (int g = 0; g < NFU; g++) begin
            if (busy[g] && qjw[g] && qj[g] == 2'(f)) begin qjw[g] <= 1'b0; rj[g] <= 1'b1; end
            if (busy[g] && qkw[g] && qk[g] == 2'(f)) begin qkw[g] <= 1'b0; rk[g] <= 1'b1; end
          end
        end
      end
      // issue (last, so it wins over nothing it should not collide with: WAW and structural checks keep it disjoint)
      if (do_issue) begin
        busy[pick] <= 1'b1;  f_op[pick] <= issue_op;  f_fi[pick] <= issue_rd;  f_fj[pick] <= issue_rs;  f_fk[pick] <= issue_rt;
        qjw[pick] <= s_wait[0];  qj[pick] <= s_fu[0];  rj[pick] <= !s_wait[0];
        qkw[pick] <= s_wait[1];  qk[pick] <= s_fu[1];  rk[pick] <= !s_wait[1];
        phase[pick] <= PH_WAIT;  seq[pick] <= seq_ctr;  seq_ctr <= seq_ctr + 16'd1;
        res_busy[issue_rd] <= 1'b1;  res_fu[issue_rd] <= pick;
      end
    end
  end
endmodule
