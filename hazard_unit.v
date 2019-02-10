
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
// hazard_unit.v (fixed)
module hazard_unit (
    input wire [4:0] id_rs1,
    input wire [4:0] id_rs2,
    input wire [6:0] opcode_id,          // from ID (for rs usage detection)
    input wire [4:0] ex_rd,              // rd in EX (ID/EX pipeline reg)
    input wire       ex_load_inst,       // mem_read_ex
    input wire       modify_pc_ex,       // mispredict or taken branch/jump resolved in EX

    output reg pc_en,
    output reg if_id_en,
    output reg if_id_flush,
    output reg im_flush,
    output reg id_ex_en,
    output reg id_ex_flush,
    output reg load_stall
);
    // --------------------------------------------------------------
    // 1. Detect which source registers are actually used in ID
    // --------------------------------------------------------------
    wire rs1_used = (opcode_id == `OPCODE_RTYPE)  ||
                    (opcode_id == `OPCODE_ITYPE)  ||
                    (opcode_id == `OPCODE_ILOAD)  ||
                    (opcode_id == `OPCODE_STYPE)  ||
                    (opcode_id == `OPCODE_BTYPE)  ||
                    (opcode_id == `OPCODE_IJALR);

    wire rs2_used = (opcode_id == `OPCODE_RTYPE)  ||
                    (opcode_id == `OPCODE_STYPE)  ||
                    (opcode_id == `OPCODE_BTYPE);

    // --------------------------------------------------------------
    // 2. Load-use hazard detection
    // --------------------------------------------------------------
    wire load_use_hazard = ex_load_inst &&
                           (ex_rd != 5'd0) &&
                           ((rs1_used && (ex_rd == id_rs1)) ||
                            (rs2_used && (ex_rd == id_rs2)));

    // --------------------------------------------------------------
    // 3. Combinational hazard/stall/flush logic
    //    NOTE: modify_pc_ex (branch resolved in EX) must be able to
    //    flush the pipeline even when a stall is active. Therefore
    //    we give modify_pc_ex higher priority than a load-use stall.
    // --------------------------------------------------------------
    always @(*) begin
        // Default: normal forward progress, no flush
        pc_en          = 1'b1;
        if_id_en       = 1'b1;
        if_id_flush    = 1'b0;
	im_flush       = 1'b0;
        id_ex_en       = 1'b1;
        id_ex_flush    = 1'b0;
        load_stall     = 1'b0;

        // Highest priority: Control hazard resolved in EX (mispredict or taken branch/jump)
        if (modify_pc_ex) begin
            pc_en       = 1'b1;   // allow PC to take corrected value (IF-stage pc_update handles override)
            if_id_en    = 1'b1;
	im_flush       = 1'b1;
		
            if_id_flush = 1'b1;   // kill wrong-path instruction in IF/ID
            id_ex_flush = 1'b1;   // kill wrong-path instruction in ID/EX
        end
        // Second priority: Load-use stall (1 cycle)
        else if (load_use_hazard) begin
            pc_en       = 1'b0;   // stall PC
            if_id_en    = 1'b0;   // stall IF/ID
            id_ex_flush = 1'b1;   // insert bubble into EX stage
            load_stall  = 1'b1;
        end
    end
endmodule
