
// execute_stage.v
// Combined Execute stage, branch/jump unit, ALU family and forwarding unit
// Minor fixes only — no ISA/datapath changes.


// -------------------------------
// execute_stage
// -------------------------------
module execute_stage (
    input wire [31:0] pc_ex,
    input wire [31:0] rs1_data_ex,
    input wire [31:0] rs2_data_ex,
    input wire [31:0] imm_ex,
    input wire [4:0] rs1_ex,
    input wire [4:0] rs2_ex,
    input wire [4:0] rd_ex,

    input wire ex_alu_src_ex,
    input wire mem_write_ex,
    input wire mem_read_ex,
    input wire [2:0] mem_load_type_ex,
    input wire [1:0] mem_store_type_ex,
    input wire wb_reg_file_ex,
    input wire memtoreg_ex,
    input wire [3:0] alu_ctrl_ex,

    input wire [1:0] operand_a_forward_cntl,
    input wire [1:0] operand_b_forward_cntl,
    input wire [31:0] data_forward_mem,
    input wire [31:0] data_forward_wb,

    output wire [31:0] alu_result_ex,
    output wire        zero_flag_ex,
    output wire        negative_flag_ex,
    output wire        carry_flag_ex,
    output wire        overflow_flag_ex,

    output wire [31:0] rs2_data_for_mem_ex,
    output wire [4:0]  rd_ex_out,
    output wire        mem_write_ex_out,
    output wire        mem_read_ex_out,
    output wire [2:0]  mem_load_type_ex_out,
    output wire [1:0]  mem_store_type_ex_out,
    output wire        wb_reg_file_ex_out,
    output wire        memtoreg_ex_out,

    output wire [31:0] op1_selected_ex,
    output wire [31:0] op2_selected_ex,
    output wire [31:0] op2_after_alu_src_ex
);

    //------------------------------
    // Forwarding Logic
    //------------------------------
    reg [31:0] op1_sel;
    reg [31:0] op2_sel;

    always @(*) begin
        op1_sel = rs1_data_ex;
        op2_sel = rs2_data_ex;

        case (operand_a_forward_cntl)
            2'b01: op1_sel = data_forward_mem;
            2'b10: op1_sel = data_forward_wb;
        endcase

        case (operand_b_forward_cntl)
            2'b01: op2_sel = data_forward_mem;
            2'b10: op2_sel = data_forward_wb;
        endcase
    end

    assign op1_selected_ex = op1_sel;
    assign op2_selected_ex = op2_sel;

    reg [31:0] op2_final;
    always @(*) begin
        op2_final = ex_alu_src_ex ? imm_ex : op2_sel;
    end

    assign op2_after_alu_src_ex = op2_final;

    //--------------------------------
    // ALU
    //--------------------------------
    wire [31:0] alu_result_w;
    wire zf_w, nf_w, cf_w, of_w;

    alu_top32 u_alu (
        .rs1(op1_sel),
        .rs2(op2_final),
        .alu_ctrl(alu_ctrl_ex),
        .alu_result(alu_result_w),
        .zero_flag(zf_w),
        .negative_flag(nf_w),
        .carry_flag(cf_w),
        .overflow_flag(of_w)
    );

    assign alu_result_ex    = alu_result_w;
    assign zero_flag_ex     = zf_w;
    assign negative_flag_ex = nf_w;
    assign carry_flag_ex    = cf_w;
    assign overflow_flag_ex = of_w;

    assign rs2_data_for_mem_ex = op2_sel;

    assign rd_ex_out             = rd_ex;
    assign mem_write_ex_out      = mem_write_ex;
    assign mem_read_ex_out       = mem_read_ex;
    assign mem_load_type_ex_out  = mem_load_type_ex;
    assign mem_store_type_ex_out = mem_store_type_ex[1:0];
    assign wb_reg_file_ex_out    = wb_reg_file_ex;
    assign memtoreg_ex_out       = memtoreg_ex;

endmodule


// -------------------------------
// branch_jump_unit
// -------------------------------
// branch_jump_unit.v
// Evaluates branch/jump conditions in EX stage and produces control-flow outputs
// - Corrected unsigned compare semantics for BLTU / BGEU (borrow interpretation)
// - JAL/JALR target calculation included (JALR result aligned by clearing LSB)

module branch_jump_unit (
    input  wire branch_ex,
    input  wire jal_ex,
    input  wire jalr_ex,
    input  wire [2:0] func3_ex,
    input  wire [31:0] pc_ex,
    input  wire [31:0] imm_ex,
    input  wire predictedTaken_ex,

    input  wire zero_flag,
    input  wire negative_flag,
    input  wire carry_flag,
    input  wire overflow_flag,
    input  wire [31:0] op1_forwarded,

    output wire ex_branch_resolved,
    output wire ex_branch_taken,
    output wire ex_predicted_taken,
    output wire modify_pc_ex,
    output wire [31:0] update_pc_ex,
    output wire [31:0] jump_addr_ex,
    output wire update_btb_ex
);

    wire is_branch = branch_ex;
    wire is_jal    = jal_ex;
    wire is_jalr   = jalr_ex;

    wire any_ctrl = is_branch | is_jal | is_jalr;
    assign ex_branch_resolved = any_ctrl;
    assign ex_predicted_taken = predictedTaken_ex;

    reg branch_cond;
    always @(*) begin
        branch_cond = 1'b0;
        if (is_branch) begin
            case (func3_ex)
                3'b000: branch_cond = zero_flag;
                3'b001: branch_cond = ~zero_flag;
                3'b100: branch_cond = (negative_flag ^ overflow_flag);
                3'b101: branch_cond = ~(negative_flag ^ overflow_flag);
                3'b110: branch_cond = ~carry_flag;
                3'b111: branch_cond = carry_flag;
            endcase
        end
    end

    wire actual_taken = is_branch ? branch_cond : (is_jal | is_jalr);

    assign ex_branch_taken = actual_taken;

    wire [31:0] target_branch = pc_ex + imm_ex;
    wire [31:0] target_jalr   = (op1_forwarded + imm_ex) & 32'hFFFFFFFE;

    assign jump_addr_ex = is_jalr ? target_jalr : target_branch;

    wire [31:0] pc_plus_4 = pc_ex + 32'd4;

    wire mispredict = (actual_taken ^ predictedTaken_ex);

    assign modify_pc_ex =
        mispredict;

    assign update_pc_ex =
        mispredict ? (actual_taken ? jump_addr_ex : pc_plus_4) : pc_plus_4;

    assign update_btb_ex = any_ctrl;

endmodule


// -------------------------------
// ALU Top and subunits
// -------------------------------
module alu_top32 (
    input  [31:0] rs1,
    input  [31:0] rs2,
    input  [3:0]  alu_ctrl,
    output [31:0] alu_result,
    output        zero_flag,
    output        negative_flag,
    output        carry_flag,
    output        overflow_flag
);
    wire [31:0] result_arith;
    wire [31:0] result_logic;
    wire [31:0] result_shift;
    wire [31:0] result_cmp;
    wire zf, nf, cf, of;

    arithmetic_unit32 u_arith (
        .rs1(rs1), .rs2(rs2), .alu_ctrl(alu_ctrl),
        .result_alu(result_arith), .zero_flag(zf),
        .carry_flag(cf), .negative_flag(nf), .overflow_flag(of)
    );

    logical_unit32 u_logic (
        .rs1(rs1), .rs2(rs2), .alu_ctrl(alu_ctrl),
        .result_alu(result_logic)
    );

    shift_unit32 u_shift (
        .rs1(rs1), .rs2(rs2), .alu_ctrl(alu_ctrl),
        .result_shift(result_shift)
    );

    compare_unit32 u_cmp (
        .rs_1(rs1), .rs_2(rs2), .alu_ctrl(alu_ctrl),
        .result_cmp(result_cmp)
    );

    reg [31:0] result_final;
    always @(*) begin
        case (alu_ctrl)
            4'b0000, 4'b0001, 4'b1010, 4'b1011: result_final = result_arith;
            4'b0010, 4'b0011, 4'b0100:          result_final = result_logic;
            4'b0101, 4'b0110, 4'b0111:          result_final = result_shift;
            4'b1000, 4'b1001:                   result_final = result_cmp;
            default: result_final = 32'b0;
        endcase
    end

    assign alu_result = result_final;
    assign zero_flag = zf;
    assign carry_flag = cf;
    assign negative_flag = nf;
    assign overflow_flag = of;
endmodule


module logical_unit32 (
    input  [31:0] rs1,
    input  [31:0] rs2,
    input  [3:0]  alu_ctrl,
    output reg [31:0] result_alu
);
    always @(*) begin
        case (alu_ctrl)
            4'b0010: result_alu = rs1 & rs2;
            4'b0011: result_alu = rs1 | rs2;
            4'b0100: result_alu = rs1 ^ rs2;
            default: result_alu = 32'b0;
        endcase
    end
endmodule


module shift_unit32 (
    input  [31:0] rs1,
    input  [31:0] rs2,
    input  [3:0]  alu_ctrl,
    output reg [31:0] result_shift
);
    wire [4:0] shamt = rs2[4:0];
    always @(*) begin
        case (alu_ctrl)
            4'b0101: result_shift = rs1 << shamt;
            4'b0110: result_shift = rs1 >> shamt;
            4'b0111: result_shift = $signed(rs1) >>> shamt;
            default: result_shift = 32'b0;
        endcase
    end
endmodule


module arithmetic_unit32 (
    input  [31:0] rs1,
    input  [31:0] rs2,
    input  [3:0]  alu_ctrl,
    output reg [31:0] result_alu,
    output       zero_flag,
    output reg   carry_flag,
    output reg   negative_flag,
    output reg   overflow_flag
);
    wire [32:0] add_ext = {1'b0, rs1} + {1'b0, rs2};
    wire [32:0] sub_ext = {1'b0, rs1} - {1'b0, rs2};

    always @(*) begin
        result_alu = 32'b0;
        carry_flag = 1'b0;
        negative_flag = 1'b0;
        overflow_flag = 1'b0;

        case (alu_ctrl)
            4'b0000: begin
                result_alu = add_ext[31:0];
                carry_flag = add_ext[32];
            end
            4'b0001: begin
                result_alu = sub_ext[31:0];
                carry_flag = sub_ext[32]; // borrow indicator style
            end
            4'b1010: begin
                result_alu = rs2; // LUI expects prepared imm
                carry_flag = 1'b0;
            end
            4'b1011: begin
                result_alu = add_ext[31:0]; // AUIPC: PC+imm expected as inputs
                carry_flag = add_ext[32];
            end
            default: begin
                result_alu = 32'b0;
                carry_flag = 1'b0;
            end
        endcase

        negative_flag = result_alu[31];

        case (alu_ctrl)
            4'b0000: begin
                overflow_flag = (~rs1[31] & ~rs2[31] & result_alu[31]) |
                                ( rs1[31] & rs2[31] & ~result_alu[31]);
            end
            4'b0001: begin
                overflow_flag = ( rs1[31] & ~rs2[31] & ~result_alu[31]) |
                                (~rs1[31] &  rs2[31] &  result_alu[31]);
            end
            default: overflow_flag = 1'b0;
        endcase
    end

    assign zero_flag = (result_alu == 32'b0);
endmodule


module compare_unit32 (
    input  [31:0] rs_1,
    input  [31:0] rs_2,
    input  [3:0]  alu_ctrl,
    output reg [31:0] result_cmp
);
    always @(*) begin
        case (alu_ctrl)
            4'b1000: result_cmp = ($signed(rs_1) < $signed(rs_2)) ? 32'b1 : 32'b0;
            4'b1001: result_cmp = (rs_1 < rs_2) ? 32'b1 : 32'b0;
            default: result_cmp = 32'b0;
        endcase
    end
endmodule


// -------------------------------
// forwarding_unit
// -------------------------------
module forwarding_unit (
    input  wire [4:0] rs1_ex,
    input  wire [4:0] rs2_ex,
    input  wire       exmem_regwrite,
    input  wire [4:0] exmem_rd,
    input  wire       memwb_regwrite,
    input  wire [4:0] memwb_rd,
    output reg  [1:0] operand_a_forward_cntl,
    output reg  [1:0] operand_b_forward_cntl
);
    always @(*) begin
        operand_a_forward_cntl = 2'b00;
        operand_b_forward_cntl = 2'b00;

        // Operand A (rs1) - EX/MEM priority
        if (exmem_regwrite && (exmem_rd != 5'd0) && (exmem_rd == rs1_ex)) begin
            operand_a_forward_cntl = 2'b01;
        end else if (memwb_regwrite && (memwb_rd != 5'd0) && (memwb_rd == rs1_ex)) begin
            operand_a_forward_cntl = 2'b10;
        end

        // Operand B (rs2)
        if (exmem_regwrite && (exmem_rd != 5'd0) && (exmem_rd == rs2_ex)) begin
            operand_b_forward_cntl = 2'b01;
        end else if (memwb_regwrite && (memwb_rd != 5'd0) && (memwb_rd == rs2_ex)) begin
            operand_b_forward_cntl = 2'b10;
        end
    end
endmodule

