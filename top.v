// rv32i_core_with_simple_btb.v
// Top-level integration for RISC-V RV32I 5-stage pipeline
// Uses if_stage_simple_btb (OPTION-A) and btb_2way instantiated inside IF stage.
//
// Notes:
//  - This file assumes all submodules you posted earlier exist with the same
//    interface names (if_id_reg, top_decode, hazard_unit, id_ex_reg,
//    forwarding_unit, execute_stage, branch_jump_unit, ex_mem_reg, mem_stage,
//    mem_wb, wb_stage).
//  - if_stage_simple_btb must match the ports used below (see Option-A).
module rv32i_core (
    input  wire clk,
    input  wire rst

    );

    // -------------------------
    // IF Stage <-> IF/ID wires
    // -------------------------
    wire [31:0] pc_if;
    wire [31:0] instr_if;
    wire predictedTaken_if;
    wire [31:0] predictedTarget_if;

    // -------------------------
    // Hazard wires
    // -------------------------
    wire hazard_pc_en;
    wire hazard_if_id_en;
    wire hazard_if_id_flush;
    wire hazard_id_ex_en;
    wire hazard_id_ex_flush;
    wire hazard_load_stall;

    // -------------------------
    // IF/ID pipeline regs
    // -------------------------
    wire [31:0] pc_id;
    wire [31:0] instr_id;
    wire predictedTaken_id;
    wire [31:0] predictedTarget_id;

    // -------------------------
    // Decode outputs (top_decode)
    // -------------------------
    wire [6:0] opcode_id;
    wire [2:0] func3_id;
    wire [6:0] func7_id;
    wire [4:0] rd_id;
    wire [4:0] rs1_id;
    wire [4:0] rs2_id;
    wire [31:0] imm_id;
    wire [31:0] rs1_data_id;
    wire [31:0] rs2_data_id;

    // Control outputs from top_decode
    wire ex_alu_src_id; 
    wire mem_write_id; 
    wire mem_read_id; 
    wire [2:0] mem_load_type_id;
    wire [1:0] mem_store_type_id;
    wire wb_reg_file_id;
    wire memtoreg_id;
    wire branch_id;
    wire jal_id;
    wire jalr_id;
    wire auipc_id;
    wire lui_id;
    wire [3:0] alu_ctrl_id;

    // -------------------------
    // ID/EX pipeline regs
    // -------------------------
    wire [31:0] pc_ex;
    wire [31:0] instr_ex;
    wire predictedTaken_ex; 
    wire [31:0] predictedTarget_ex;

    wire [6:0] opcode_ex;
    wire [2:0] func3_ex;
    wire [6:0] func7_ex;
    wire [4:0] rd_ex;
    wire [4:0] rs1_ex;
    wire [4:0] rs2_ex;
    wire [31:0] imm_ex;
    wire [31:0] rs1_data_ex;
    wire [31:0] rs2_data_ex;

    wire ex_alu_src_ex;
    wire mem_write_ex;
    wire mem_read_ex;
    wire [2:0] mem_load_type_ex;
    wire [1:0] mem_store_type_ex;
    wire wb_reg_file_ex;
    wire memtoreg_ex;
    wire branch_ex_wires;
    wire jal_ex;
    wire jalr_ex;
    wire auipc_ex;
    wire lui_ex;
    wire [3:0] alu_ctrl_ex;

    // -------------------------
    // Forwarding control
    // -------------------------
    wire [1:0] operand_a_forward_cntl;
    wire [1:0] operand_b_forward_cntl;

    // -------------------------
    // EX outputs and ALU flags
    // -------------------------
    wire [31:0] alu_result_ex;
    wire zero_flag_ex;
    wire negative_flag_ex;
    wire carry_flag_ex;
    wire overflow_flag_ex;
    wire [31:0] rs2_data_for_mem_ex;
    wire [4:0] rd_ex_out;
    wire mem_write_ex_out;
    wire mem_read_ex_out;
    wire [2:0] mem_load_type_ex_out;
    wire [1:0] mem_store_type_ex_out;
    wire wb_reg_file_ex_out;
    wire memtoreg_ex_out;

    wire [31:0] op1_selected_ex;
    wire [31:0] op2_selected_ex;
    wire [31:0] op2_after_alu_src_ex;

    // -------------------------
    // Branch unit outputs (direct from EX)
    // -------------------------
    wire ex_branch_resolved;
    wire ex_branch_taken;
    wire ex_predicted_taken;
    wire ex_modify_pc;
    wire [31:0] ex_update_pc;
    wire [31:0] ex_jump_addr;
    wire ex_update_btb;

    // -------------------------
    // EX/MEM pipeline regs
    // -------------------------
    wire [31:0] alu_result_mem;
    wire [31:0] rs2_data_mem;
    wire [4:0]  rd_mem;
    wire mem_write_mem;
    wire mem_read_mem;
    wire [2:0] mem_load_type_mem;
    wire [1:0] mem_store_type_mem;
    wire wb_reg_file_mem;
    wire memtoreg_mem;

    // -------------------------
    // MEM stage outputs
    // -------------------------
    wire [31:0] alu_result_for_wb;
    wire [31:0] load_wb_data;
    wire [4:0]  rd_for_wb;
    wire wb_reg_file_out;
    wire memtoreg_out;

    // pass-through branch outputs (for observation or WB logging) - optional
    wire modify_pc_out_mem;
    wire [31:0] update_pc_out_mem;
    wire [31:0] jump_addr_out_mem;
    wire update_btb_out_mem;

    // -------------------------
    // MEM/WB pipeline regs
    // -------------------------
    wire [31:0] alu_result_wb;
    wire [31:0] load_data_wb;
    wire [4:0]  rd_wb;
    wire wb_reg_file_wb;
    wire memtoreg_wb;

    // -------------------------
    // Forwarding sources
    // -------------------------
    wire [31:0] data_forward_mem; // EX/MEM ALU result
    wire [31:0] data_forward_wb;  // MEM/WB forwarded write data

    // -------------------------
    // WB stage outputs to regfile
    // -------------------------
    wire [31:0] wb_write_data;
    wire [4:0]  wb_write_addr;
    wire        wb_write_en;

    // =====================================================
    // 1) IF stage (Simple BTB integrated inside IF stage)
    //    Using OPTION-A: if_stage_simple_btb
    // =====================================================
    if_stage_simple_btb u_if (
        .clk(clk),
        .rst(rst),
        .pc_en(hazard_pc_en),
	.flush(im_flush),
        // EX direct signals for redirect / BTB update (connected from branch unit)
        .modify_pc_ex(ex_modify_pc),
        .update_pc_ex(ex_update_pc),
        .pc_ex(pc_ex),
        .jump_addr_ex(ex_jump_addr),
        .update_btb_ex(ex_update_btb),
        .ex_branch_taken(ex_branch_taken),
        // IF outputs
        .pc_if(pc_if),
        .instr_if(instr_if),
        .predictedTaken_if(predictedTaken_if),
        .predictedTarget_if(predictedTarget_if)
    );

    // Expose PC (test hook)
    assign pc = pc_if;

    // Next PC (computed similarly to pc_update priority)
    // priority: EX override (modify_pc) -> update_pc_ex
    // else predictedTaken -> predictedTarget_if
    // else pc+4
    wire [31:0] pc_plus4 = pc_if + 32'd4;
    assign next_pc = ex_modify_pc ? ex_update_pc :
                     (predictedTaken_if ? predictedTarget_if : pc_plus4);

    // =====================================================
    // 2) IF/ID pipeline register
    // =====================================================
    if_id_reg u_if_id (
        .clk(clk), .rst(rst),
        .en(hazard_if_id_en),
        .flush(hazard_if_id_flush),
        .pc_in(pc_if),
        .instr_in(instr_if),
        .predictedTaken_in(predictedTaken_if),
        .predictedTarget_in(predictedTarget_if),
        .pc_id(pc_id),
        .instr_id(instr_id),
        .predictedTaken_id(predictedTaken_id),
        .predictedTarget_id(predictedTarget_id)
    );

    assign if_id_instr = instr_id;

    // =====================================================
    // 3) Decode stage (top_decode) -- register file is inside
    //    Feed top_decode WB write port from WB stage outputs
    // =====================================================
    top_decode u_decode (
        .clk(clk),
        .rst(rst),
        .instruction_in(instr_id),
        .id_flush(hazard_if_id_flush),    // treat IF/ID flush as decode flush
        // WB writeback inputs (from WB stage)
        .wb_wr_en(wb_write_en),
        .wb_wr_addr(wb_write_addr),
        .wb_wr_data(wb_write_data),
        // instruction fields & outputs
        .opcode(opcode_id),
        .func3(func3_id),
        .func7(func7_id),
        .rd(rd_id),
        .rs1(rs1_id),
        .rs2(rs2_id),
        .imm_out(imm_id),
        .rs1_data(rs1_data_id),
        .rs2_data(rs2_data_id),
        // control signals
        .ex_alu_src(ex_alu_src_id),
        .mem_write(mem_write_id),
        .mem_read(mem_read_id),
        .mem_load_type(mem_load_type_id),
        .mem_store_type(mem_store_type_id),
        .wb_reg_file(wb_reg_file_id),
        .memtoreg(memtoreg_id),
        .branch(branch_id),
        .jal(jal_id),
        .jalr(jalr_id),
        .auipc(auipc_id),
        .lui(lui_id),
        .alu_ctrl(alu_ctrl_id)
    );

    // =====================================================
    // 4) Hazard unit
    // =====================================================
    hazard_unit u_hazard (
        .id_rs1(rs1_id),
        .id_rs2(rs2_id),
        .opcode_id(opcode_id),
        .ex_rd(rd_ex),                 // rd in EX (ID/EX)
        .ex_load_inst(mem_read_ex),    // mem_read_ex from ID/EX
        .modify_pc_ex(ex_modify_pc),   // direct from EX branch unit
        .pc_en(hazard_pc_en),
        .if_id_en(hazard_if_id_en),
        .if_id_flush(hazard_if_id_flush),
	 .im_flush(im_flush),
        .id_ex_en(hazard_id_ex_en),
        .id_ex_flush(hazard_id_ex_flush),
        .load_stall(hazard_load_stall)
    );

    // expose stall/flush hooks
    assign pc_en = hazard_pc_en;
    assign if_id_en_out = hazard_if_id_en;
    assign if_id_flush_out = hazard_if_id_flush;
    assign id_ex_en_out = hazard_id_ex_en;
    assign id_ex_flush_out = hazard_id_ex_flush;
    assign load_stall_out = hazard_load_stall;

    // =====================================================
    // 5) ID/EX pipeline register
    // =====================================================
    id_ex_reg u_id_ex (
        .clk(clk), .rst(rst),
        .en(hazard_id_ex_en),
        .flush(hazard_id_ex_flush),
        .pc_id(pc_id),
        .instr_id(instr_id),
        .predictedTaken_id(predictedTaken_id),
        .predictedTarget_id(predictedTarget_id),
        // instruction fields
        .opcode(opcode_id),
        .func3(func3_id),
        .func7(func7_id),
        .rd(rd_id),
        .rs1(rs1_id),
        .rs2(rs2_id),
        .imm_out(imm_id),
        .rs1_data(rs1_data_id),
        .rs2_data(rs2_data_id),
        // control
        .ex_alu_src(ex_alu_src_id),
        .mem_write(mem_write_id),
        .mem_read(mem_read_id),
        .mem_load_type(mem_load_type_id),
        .mem_store_type(mem_store_type_id),
        .wb_reg_file(wb_reg_file_id),
        .memtoreg(memtoreg_id),
        .branch(branch_id),
        .jal(jal_id),
        .jalr(jalr_id),
        .auipc(auipc_id),
        .lui(lui_id),
        .alu_ctrl(alu_ctrl_id),
        // outputs
        .pc_ex(pc_ex),
        .instr_ex(instr_ex),
        .predictedTaken_ex(predictedTaken_ex),
        .predictedTarget_ex(predictedTarget_ex),
        .opcode_ex(opcode_ex),
        .func3_ex(func3_ex),
        .func7_ex(func7_ex),
        .rd_ex(rd_ex),
        .rs1_ex(rs1_ex),
        .rs2_ex(rs2_ex),
        .imm_ex(imm_ex),
        .rs1_data_ex(rs1_data_ex),
        .rs2_data_ex(rs2_data_ex),
        .ex_alu_src_ex(ex_alu_src_ex),
        .mem_write_ex(mem_write_ex),
        .mem_read_ex(mem_read_ex),
        .mem_load_type_ex(mem_load_type_ex),
        .mem_store_type_ex(mem_store_type_ex),
        .wb_reg_file_ex(wb_reg_file_ex),
        .memtoreg_ex(memtoreg_ex),
        .branch_ex(branch_ex_wires),
        .jal_ex(jal_ex),
        .jalr_ex(jalr_ex),
        .auipc_ex(auipc_ex),
        .lui_ex(lui_ex),
        .alu_ctrl_ex(alu_ctrl_ex)
    );

    assign id_ex_instr = instr_ex;

    // =====================================================
    // 6) Forwarding unit (produces selection controls for EX)
    // =====================================================
    forwarding_unit u_fwd (
        .rs1_ex(rs1_ex),
        .rs2_ex(rs2_ex),
        .exmem_regwrite(wb_reg_file_mem),
        .exmem_rd(rd_mem),
        .memwb_regwrite(wb_reg_file_wb),
        .memwb_rd(rd_wb),
        .operand_a_forward_cntl(operand_a_forward_cntl),
        .operand_b_forward_cntl(operand_b_forward_cntl)
    );

    // =====================================================
    // 7) Execute stage (ALU + forwarding selects)
    // =====================================================
    execute_stage u_exe (
        .pc_ex(pc_ex),
        .rs1_data_ex(rs1_data_ex),
        .rs2_data_ex(rs2_data_ex),
        .imm_ex(imm_ex),
        .rs1_ex(rs1_ex),
        .rs2_ex(rs2_ex),
        .rd_ex(rd_ex),
        .ex_alu_src_ex(ex_alu_src_ex),
        .mem_write_ex(mem_write_ex),
        .mem_read_ex(mem_read_ex),
        .mem_load_type_ex(mem_load_type_ex),
        .mem_store_type_ex(mem_store_type_ex),
        .wb_reg_file_ex(wb_reg_file_ex),
        .memtoreg_ex(memtoreg_ex),
        .alu_ctrl_ex(alu_ctrl_ex),
        .operand_a_forward_cntl(operand_a_forward_cntl),
        .operand_b_forward_cntl(operand_b_forward_cntl),
        .data_forward_mem(data_forward_mem),
        .data_forward_wb(data_forward_wb),
        .alu_result_ex(alu_result_ex),
        .zero_flag_ex(zero_flag_ex),
        .negative_flag_ex(negative_flag_ex),
        .carry_flag_ex(carry_flag_ex),
        .overflow_flag_ex(overflow_flag_ex),
        .rs2_data_for_mem_ex(rs2_data_for_mem_ex),
        .rd_ex_out(rd_ex_out),
        .mem_write_ex_out(mem_write_ex_out),
        .mem_read_ex_out(mem_read_ex_out),
        .mem_load_type_ex_out(mem_load_type_ex_out),
        .mem_store_type_ex_out(mem_store_type_ex_out),
        .wb_reg_file_ex_out(wb_reg_file_ex_out),
        .memtoreg_ex_out(memtoreg_ex_out),
        .op1_selected_ex(op1_selected_ex),
        .op2_selected_ex(op2_selected_ex),
        .op2_after_alu_src_ex(op2_after_alu_src_ex)
    );

    // =====================================================
    // 8) Branch / Jump unit (uses ALU flags + forwarded rs1)
    //    Outputs: ex_modify_pc, ex_update_pc, ex_jump_addr, ex_update_btb, ex_branch_taken
    // =====================================================
    branch_jump_unit u_branch (
        .branch_ex(branch_ex_wires),
        .jal_ex(jal_ex),
        .jalr_ex(jalr_ex),
        .func3_ex(func3_ex),
        .pc_ex(pc_ex),
        .imm_ex(imm_ex),
        .predictedTaken_ex(predictedTaken_ex),
        .zero_flag(zero_flag_ex),
        .negative_flag(negative_flag_ex),
        .carry_flag(carry_flag_ex),
        .overflow_flag(overflow_flag_ex),
        .op1_forwarded(op1_selected_ex),
        .ex_branch_resolved(ex_branch_resolved),
        .ex_branch_taken(ex_branch_taken),
        .ex_predicted_taken(ex_predicted_taken),
        .modify_pc_ex(ex_modify_pc),
        .update_pc_ex(ex_update_pc),
        .jump_addr_ex(ex_jump_addr),
        .update_btb_ex(ex_update_btb)
    );

    // expose branch observability hooks
    assign branch_ex = ex_branch_resolved;
    assign branch_taken_ex = ex_branch_taken;
    assign branch_target = ex_jump_addr;

    // =====================================================
    // 9) EX/MEM pipeline register
    //    NOTE: EX->IF branch outputs NOT passed here; remain direct EX->IF
    // =====================================================
    ex_mem_reg u_ex_mem (
        .clk(clk), .rst(rst),
        .en(1'b1),
        .flush(1'b0),
        .alu_result_ex(alu_result_ex),
        .rs2_data_ex(rs2_data_for_mem_ex),
        .rd_ex(rd_ex_out),
        .mem_write_ex(mem_write_ex_out),
        .mem_read_ex(mem_read_ex_out),
        .mem_load_type_ex(mem_load_type_ex_out),
        .mem_store_type_ex(mem_store_type_ex_out),
        .wb_reg_file_ex(wb_reg_file_ex_out),
        .memtoreg_ex(memtoreg_ex_out),
        // Branch signals removed from EX/MEM per design choice (EX driven direct to IF)
        .alu_result_mem(alu_result_mem),
        .rs2_data_mem(rs2_data_mem),
        .rd_mem(rd_mem),
        .mem_write_mem(mem_write_mem),
        .mem_read_mem(mem_read_mem),
        .mem_load_type_mem(mem_load_type_mem),
        .mem_store_type_mem(mem_store_type_mem),
        .wb_reg_file_mem(wb_reg_file_mem),
        .memtoreg_mem(memtoreg_mem)
    );

    assign data_forward_mem = alu_result_mem; // EX/MEM ALU result forwarding source

    assign ex_mem_instr = pc_ex; // rough probe: using pc_ex to indicate EX instruction location

    // =====================================================
    // 10) MEM stage
    //     Connect branch signals directly from EX (ex_modify_pc, ex_update_pc, ex_jump_addr, ex_update_btb)
    // =====================================================
    mem_stage u_mem (
        .clk(clk), .rst(rst),
        .en(1'b1),
        .flush(1'b0),
        .alu_result_mem(alu_result_mem),
        .rs2_data_mem(rs2_data_mem),
        .rd_mem(rd_mem),
        .mem_write_mem(mem_write_mem),
        .mem_read_mem(mem_read_mem),
        .mem_load_type_mem(mem_load_type_mem),
        .mem_store_type_mem(mem_store_type_mem),
        .wb_reg_file_mem(wb_reg_file_mem),
        .memtoreg_mem(memtoreg_mem),
        // Branch signals connected directly from EX outputs (not from EX/MEM)
        .modify_pc_mem(ex_modify_pc),
        .update_pc_mem(ex_update_pc),
        .jump_addr_mem(ex_jump_addr),
        .update_btb_mem(ex_update_btb),
        // MEM -> MEM/WB outputs
        .alu_result_for_wb(alu_result_for_wb),
        .load_wb_data(load_wb_data),
        .rd_for_wb(rd_for_wb),
        .wb_reg_file_out(wb_reg_file_out),
        .memtoreg_out(memtoreg_out),
        // pass-through branch observability
        .modify_pc_out(modify_pc_out_mem),
        .update_pc_out(update_pc_out_mem),
        .jump_addr_out(jump_addr_out_mem),
        .update_btb_out(update_btb_out_mem)
    );

    // =====================================================
    // 11) MEM/WB pipeline register
    // =====================================================
    mem_wb u_mem_wb (
        .clk(clk), .rst(rst),
        .en(1'b1),
        .flush(1'b0),
        .alu_result_in(alu_result_for_wb),
        .load_data_in(load_wb_data),
        .rd_in(rd_for_wb),
        .wb_reg_file_in(wb_reg_file_out),
        .memtoreg_in(memtoreg_out),
        // optional branch pass-through
        .modify_pc_in(modify_pc_out_mem),
        .update_pc_in(update_pc_out_mem),
        .jump_addr_in(jump_addr_out_mem),
        .update_btb_in(update_btb_out_mem),
        .alu_result_out(alu_result_wb),
        .load_data_out(load_data_wb),
        .rd_out(rd_wb),
        .wb_reg_file_out(wb_reg_file_wb),
        .memtoreg_out(memtoreg_wb),
        .data_forward_wb(data_forward_wb),
        .modify_pc_out(modify_pc_out_mem), // forwarded for visibility
        .update_pc_out(update_pc_out_mem),
        .jump_addr_out(jump_addr_out_mem),
        .update_btb_out(update_btb_out_mem)
    );

    // =====================================================
    // 12) WB stage
    // =====================================================
    wb_stage u_wb (
        .clk(clk),
        .rst(rst),
        .alu_result_wb(alu_result_wb),
        .load_data_wb(load_data_wb),
        .rd_wb(rd_wb),
        .wb_reg_file_wb(wb_reg_file_wb),
        .memtoreg_wb(memtoreg_wb),
        .wb_write_data(wb_write_data),
        .wb_write_addr(wb_write_addr),
        .wb_write_en(wb_write_en)
    );

    // expose some chosen observability signals
    assign pc_en = hazard_pc_en;

endmodule
