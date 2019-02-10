module decode_unit (
    input  wire [31:0] instruction_in,
    input  wire        id_flush,        // when asserted, treat instruction as NOP (all zeros)
    output wire [6:0]  opcode,
    output wire [2:0]  func3,
    output wire [6:0]  func7,
    output wire [4:0]  rd,
    output wire [4:0]  rs1,
    output wire [4:0]  rs2,
    output reg  [31:0] imm_out
);
    // Select either real instruction or zero when flushed
    wire [31:0] instr = id_flush ? 32'h00000000 : instruction_in;

    // Field extraction (combinational)
    assign opcode = instr[6:0];
    assign rd     = instr[11:7];
    assign func3  = instr[14:12];
    assign rs1    = instr[19:15];
    assign rs2    = instr[24:20];
    assign func7  = instr[31:25];

    // Immediate generation
    always @(*) begin
        case (opcode)
            // I-type (includes arithmetic immediates) and loads (I-format) and JALR
            7'b0010011, 7'b0000011, 7'b1100111: begin
                imm_out = {{20{instr[31]}}, instr[31:20]};
            end

            // S-type (store): imm[11:5]=instr[31:25], imm[4:0]=instr[11:7]
            7'b0100011: begin
                imm_out = {{20{instr[31]}}, instr[31:25], instr[11:7]};
            end

            // B-type (branch): imm = {instr[31], instr[7], instr[30:25], instr[11:8], 1'b0}
            7'b1100011: begin
                imm_out = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
            end

            // J-type (JAL): imm = {instr[31], instr[19:12], instr[20], instr[30:21], 1'b0}
            7'b1101111: begin
                imm_out = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
            end

            // U-type (LUI/AUIPC): imm = instr[31:12] << 12
            7'b0110111, 7'b0010111: begin
                imm_out = {instr[31:12], 12'b0};
            end

            // Default (R-type or unknown)
            default: begin
                imm_out = 32'h00000000;
            end
        endcase
    end
endmodule


// Small control unit (decoding high-level control signals)
module control_unit(
    input wire [6:0] opcode,
    input wire [2:0] func3,
    input wire [6:0] func7,

    output reg ex_alu_src,
    output reg mem_write,
    output reg mem_read,
    output reg [2:0] mem_load_type,
    output reg [1:0] mem_store_type,
    output reg wb_reg_file,
    output reg memtoreg,
    output reg branch,
    output reg jal,
    output reg jalr,
    output reg auipc,
    output reg lui,
    output reg [3:0] alu_ctrl
);
    always @(*) begin
        // defaults
        ex_alu_src = 1'b0; mem_write = 1'b0; mem_read = 1'b0; mem_load_type = 3'b010; mem_store_type = 2'b10;
        wb_reg_file = 1'b0; memtoreg = 1'b0; branch = 1'b0; jal = 1'b0; jalr = 1'b0; auipc = 1'b0; lui = 1'b0;
        alu_ctrl = 4'b0000; // ADD default

        case (opcode)
            7'b0110011: begin // R-type
                wb_reg_file = 1'b1;
                case (func3)
                    3'b000: alu_ctrl = func7[5] ? 4'b0001 : 4'b0000; // SUB/ADD
                    3'b111: alu_ctrl = 4'b0010; // AND
                    3'b110: alu_ctrl = 4'b0011; // OR
                    3'b100: alu_ctrl = 4'b0100; // XOR
                    3'b001: alu_ctrl = 4'b0101; // SLL
                    3'b101: alu_ctrl = func7[5] ? 4'b0111 : 4'b0110; // SRA/SRL
                    3'b010: alu_ctrl = 4'b1000; // SLT
                    3'b011: alu_ctrl = 4'b1001; // SLTU
                    default: alu_ctrl = 4'b0000;
                endcase
            end
            7'b0010011: begin // I-type ALU
                ex_alu_src = 1'b1; wb_reg_file = 1'b1;
                case (func3)
                    3'b000: alu_ctrl = 4'b0000; // ADDI
                    3'b111: alu_ctrl = 4'b0010; // ANDI
                    3'b110: alu_ctrl = 4'b0011; // ORI
                    3'b100: alu_ctrl = 4'b0100; // XORI
                    3'b001: alu_ctrl = 4'b0101; // SLLI
                    3'b101: alu_ctrl = func7[5] ? 4'b0111 : 4'b0110; // SRAI/SRLI
                    3'b010: alu_ctrl = 4'b1000; // SLTI
                    3'b011: alu_ctrl = 4'b1001; // SLTIU
                    default: alu_ctrl = 4'b0000;
                endcase
            end
            7'b0000011: begin // LOAD
                ex_alu_src = 1'b1; mem_read = 1'b1; wb_reg_file = 1'b1; memtoreg = 1'b1;
                case (func3)
                    3'b000: mem_load_type = 3'b000; // LB
                    3'b001: mem_load_type = 3'b001; // LH
                    3'b010: mem_load_type = 3'b010; // LW
                    3'b100: mem_load_type = 3'b011; // LBU
                    3'b101: mem_load_type = 3'b100; // LHU
                    default: mem_load_type = 3'b010;
                endcase
            end
            7'b0100011: begin // STORE
                ex_alu_src = 1'b1; mem_write = 1'b1;
                case (func3)
                    3'b000: mem_store_type = 2'b00; // SB
                    3'b001: mem_store_type = 2'b01; // SH
                    3'b010: mem_store_type = 2'b10; // SW
                    default: mem_store_type = 2'b10;
                endcase
            end
            7'b1100011: begin // BRANCH
                branch = 1'b1; alu_ctrl = 4'b0001; // SUB for compare
            end
            7'b1101111: begin // JAL
                jal = 1'b1; wb_reg_file = 1'b1;
            end
            7'b1100111: begin // JALR
                jalr = 1'b1; ex_alu_src = 1'b1; wb_reg_file = 1'b1;
            end
            7'b0110111: begin // LUI
                wb_reg_file = 1'b1; alu_ctrl = 4'b1010;
            end
            7'b0010111: begin // AUIPC
                wb_reg_file = 1'b1; alu_ctrl = 4'b1011;
            end
            default: begin end
        endcase
    end
endmodule


// top_decode: connects decode_unit, control_unit and register file
module top_decode (
    input  wire        clk,
    input  wire        rst,

    // Instruction fetch / ID inputs
    input  wire [31:0] instruction_in,
    input  wire        id_flush,

    // Writeback port (from WB stage)
    input  wire        wb_wr_en,
    input  wire [4:0]  wb_wr_addr,
    input  wire [31:0] wb_wr_data,

    // Instruction fields (outputs)
    output wire [6:0]  opcode,
    output wire [2:0]  func3,
    output wire [6:0]  func7,
    output wire [4:0]  rd,
    output wire [4:0]  rs1,
    output wire [4:0]  rs2,
    output wire [31:0] imm_out,

    // Register file outputs
    output wire [31:0] rs1_data,
    output wire [31:0] rs2_data,

    // Control signals
    output wire        ex_alu_src,
    output wire        mem_write,
    output wire        mem_read,
    output wire [2:0]  mem_load_type,
    output wire [1:0]  mem_store_type,
    output wire        wb_reg_file,
    output wire        memtoreg,
    output wire        branch,
    output wire        jal,
    output wire        jalr,
    output wire        auipc,
    output wire        lui,
    output wire [3:0]  alu_ctrl
);
    // Internal wires
    wire [6:0] opcode_w; wire [2:0] func3_w; wire [6:0] func7_w;
    wire [4:0] rd_w; wire [4:0] rs1_w; wire [4:0] rs2_w; wire [31:0] imm_w;
    wire ex_alu_src_w; wire mem_write_w; wire mem_read_w; wire [2:0] mem_load_type_w; wire [1:0] mem_store_type_w;
    wire wb_reg_file_w; wire memtoreg_w; wire branch_w; wire jal_w; wire jalr_w; wire auipc_w; wire lui_w; wire [3:0] alu_ctrl_w;

    // Decode unit
    decode_unit u_decode_unit (
        .instruction_in(instruction_in),
        .id_flush(id_flush),
        .opcode(opcode_w),
        .func3(func3_w),
        .func7(func7_w),
        .rd(rd_w),
        .rs1(rs1_w),
        .rs2(rs2_w),
        .imm_out(imm_w)
    );

    // Control unit
    control_unit u_ctrl (
        .opcode(opcode_w), .func3(func3_w), .func7(func7_w),
        .ex_alu_src(ex_alu_src_w), .mem_write(mem_write_w), .mem_read(mem_read_w),
        .mem_load_type(mem_load_type_w), .mem_store_type(mem_store_type_w),
        .wb_reg_file(wb_reg_file_w), .memtoreg(memtoreg_w),
        .branch(branch_w), .jal(jal_w), .jalr(jalr_w), .auipc(auipc_w), .lui(lui_w),
        .alu_ctrl(alu_ctrl_w)
    );

    // Register file (external file regfile.v)
    register_file u_regfile (
        .clk(clk),
        .wr_en(wb_wr_en), .wr_addr(wb_wr_addr), .wr_data(wb_wr_data),
        .rs1_addr(rs1_w), .rs2_addr(rs2_w),
        .rs1_data(rs1_data), .rs2_data(rs2_data)
    );

    // expose outputs
    assign opcode = opcode_w;
    assign func3 = func3_w;
    assign func7 = func7_w;
    assign rd = rd_w;
    assign rs1 = rs1_w;
    assign rs2 = rs2_w;
    assign imm_out = imm_w;

    assign ex_alu_src = ex_alu_src_w;
    assign mem_write = mem_write_w;
    assign mem_read = mem_read_w;
    assign mem_load_type = mem_load_type_w;
    assign mem_store_type = mem_store_type_w;
    assign wb_reg_file = wb_reg_file_w;
    assign memtoreg = memtoreg_w;
    assign branch = branch_w;
    assign jal = jal_w;
    assign jalr = jalr_w;
    assign auipc = auipc_w;
    assign lui = lui_w;
    assign alu_ctrl = alu_ctrl_w;
endmodule



module register_file (
    input  wire        clk,

    // write port (from WB stage)
    input  wire        wr_en,
    input  wire [4:0]  wr_addr,
    input  wire [31:0] wr_data,

    // read addresses (from ID stage)
    input  wire [4:0]  rs1_addr,
    input  wire [4:0]  rs2_addr,

    // read data outputs
    output wire [31:0] rs1_data,
    output wire [31:0] rs2_data
);
    reg [31:0] reg_file [0:31];
      //  integer i;
    
    initial begin
        // Optional: initialize registers from a file if present
        $readmemh("reg_mem.hex", reg_file);
       // for (i = 0; i < 32; i = i + 1) reg_file[i] = 32'h00000000;
    end

    // Forwarding behavior: if a write happens to the same reg in the same cycle,
    // provide the write data to reads (combinational forwarding).
    // This simple scheme assumes write happens on posedge and reads are combinational.
    wire [31:0] rs1_comb = reg_file[rs1_addr];
    wire [31:0] rs2_comb = reg_file[rs2_addr];

    assign rs1_data = (rs1_addr == 5'd0) ? 32'h0 :
                      ((wr_en && (wr_addr == rs1_addr)) ? wr_data : rs1_comb);

    assign rs2_data = (rs2_addr == 5'd0) ? 32'h0 :
                      ((wr_en && (wr_addr == rs2_addr)) ? wr_data : rs2_comb);

    // Write operation (synchronous)
    always @(posedge clk) begin
        if (wr_en && (wr_addr != 5'd0)) begin
            reg_file[wr_addr] <= wr_data;
        end
    end

endmodule




// OPCODES
`define OPCODE_RTYPE 7'b0110011
`define OPCODE_ITYPE 7'b0010011
`define OPCODE_ILOAD 7'b0000011
`define OPCODE_IJALR 7'b1100111
`define OPCODE_BTYPE 7'b1100011
`define OPCODE_STYPE 7'b0100011
`define OPCODE_JTYPE 7'b1101111
`define OPCODE_AUIPC 7'b0010111
`define OPCODE_UTYPE 7'b0110111

// FUNC7 - ADD
`define FUNC7_ADD 7'b0000000
`define FUNC7_SUB 7'b0100000

// ALU Codes
`define ALU_ADD  4'b0000
`define ALU_SUB  4'b0001
`define ALU_AND  4'b0010
`define ALU_OR   4'b0011
`define ALU_XOR  4'b0100
`define ALU_SLL  4'b0101
`define ALU_SRL  4'b0110
`define ALU_SRA  4'b0111
`define ALU_SLT  4'b1000
`define ALU_SLTU 4'b1001

// B Type Codes
`define BTYPE_BEQ  3'b000
`define BTYPE_BNE  3'b001
`define BTYPE_BLT  3'b100
`define BTYPE_BGE  3'b101
`define BTYPE_BLTU 3'b110
`define BTYPE_BGEU 3'b111

// Forwarding Unit
`define FORWARD_ORG 2'b00
`define FORWARD_MEM 2'b01
`define FORWARD_WB  2'b10

// Store Types
`define STORE_SB  2'b00
`define STORE_SH  2'b01
`define STORE_SW  2'b10
`define STORE_DEF 2'b11

// Load Types
`define LOAD_LB  3'b000
`define LOAD_LH  3'b001   // FIXED NAME
`define LOAD_LW  3'b010
`define LOAD_LBU 3'b011
`define LOAD_LHU 3'b100
`define LOAD_DEF 3'b111

// Constants
`define ZERO_32BIT  32'h00000000
`define ZERO_12BIT  12'h000

// BTB State
`define STRONG_NOT_TAKEN 2'b00
`define WEAK_NOT_TAKEN   2'b01
`define STRONG_TAKEN     2'b10
`define WEAK_TAKEN       2'b11

