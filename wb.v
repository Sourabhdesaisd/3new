// wb_stage.v
// Writeback stage: select ALU or load data and present write port to register file.
module wb_stage (
    input  wire        clk,
    input  wire        rst,

    // ---------------- MEM/WB register inputs ----------------
    input  wire [31:0] alu_result_wb,
    input  wire [31:0] load_data_wb,
    input  wire [4:0]  rd_wb,
    input  wire        wb_reg_file_wb,   // RegWrite
    input  wire        memtoreg_wb,      // 1 = load, 0 = ALU

    // ---------------- Register File Interface ----------------
    output wire [31:0] wb_write_data,    // Writeback data
    output wire [4:0]  wb_write_addr,
    output wire        wb_write_en
);

    // Writeback data selection:
    // If memtoreg=1, choose memory load value, else choose ALU result.
    assign wb_write_data = memtoreg_wb ? load_data_wb : alu_result_wb;

    // Destination register number
    assign wb_write_addr = rd_wb;

    // Write enable:
    // Only write when wb_reg_file_wb is true AND rd != x0
    assign wb_write_en = (wb_reg_file_wb && (rd_wb != 5'd0));

endmodule

