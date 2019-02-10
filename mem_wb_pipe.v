// mem_wb.v
// MEM -> WB pipeline register
// - captures ALU result and load result and forwards control to WB
// - provides a forwarding data output (data_forward_wb) for EX-stage forwarding
// - explicit flush-over-stall priority and deterministic hold behavior


module mem_wb (
    input  wire        clk,
    input  wire        rst,
    input  wire        en,
    input  wire        flush,

    // From MEM stage
    input  wire [31:0] alu_result_in,
    input  wire [31:0] load_data_in,
    input  wire [4:0]  rd_in,
    input  wire        wb_reg_file_in,
    input  wire        memtoreg_in,

    // Optional BTB / branch pass-through
    input  wire        modify_pc_in,
    input  wire [31:0] update_pc_in,
    input  wire [31:0] jump_addr_in,
    input  wire        update_btb_in,

    // Outputs to WB stage
    output reg  [31:0] alu_result_out,
    output reg  [31:0] load_data_out,
    output reg  [4:0]  rd_out,
    output reg         wb_reg_file_out,
    output reg         memtoreg_out,

    // Forwarding value for EX stage
    output wire [31:0] data_forward_wb,

    // Optional branch output
    output reg         modify_pc_out,
    output reg [31:0]  update_pc_out,
    output reg [31:0]  jump_addr_out,
    output reg         update_btb_out
);

    localparam ZERO32 = 32'h00000000;
    localparam ZERO5  = 5'd0;

    // Forwarding chooses data already latched in MEM/WB
    assign data_forward_wb = memtoreg_out ? load_data_out : alu_result_out;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            alu_result_out   <= ZERO32;
            load_data_out    <= ZERO32;
            rd_out           <= ZERO5;
            wb_reg_file_out  <= 1'b0;
            memtoreg_out     <= 1'b0;
            modify_pc_out    <= 1'b0;
            update_pc_out    <= ZERO32;
            jump_addr_out    <= ZERO32;
            update_btb_out   <= 1'b0;
        end

        // ---------------------
        // FLUSH (NOP insertion)
        // ---------------------
        else if (flush) begin
            // DO NOT clear ALU result or load data
            alu_result_out   <= alu_result_out;  
            load_data_out    <= load_data_out;

            // Invalidate control signals only
            rd_out           <= ZERO5;
            wb_reg_file_out  <= 1'b0;
            memtoreg_out     <= 1'b0;

            // Branch info invalidated
            modify_pc_out    <= 1'b0;
            update_pc_out    <= ZERO32;
            jump_addr_out    <= ZERO32;
            update_btb_out   <= 1'b0;
        end

        // ---------------------
        // STALL (HOLD VALUES)
        // ---------------------
        else if (!en) begin
            alu_result_out   <= alu_result_out;
            load_data_out    <= load_data_out;
            rd_out           <= rd_out;
            wb_reg_file_out  <= wb_reg_file_out;
            memtoreg_out     <= memtoreg_out;
            modify_pc_out    <= modify_pc_out;
            update_pc_out    <= update_pc_out;
            jump_addr_out    <= jump_addr_out;
            update_btb_out   <= update_btb_out;
        end

        // ---------------------
        // NORMAL PIPELINE ADVANCE
        // ---------------------
        else begin
            alu_result_out   <= alu_result_in;
            load_data_out    <= load_data_in;
            rd_out           <= rd_in;
            wb_reg_file_out  <= wb_reg_file_in;
            memtoreg_out     <= memtoreg_in;
            modify_pc_out    <= modify_pc_in;
            update_pc_out    <= update_pc_in;
            jump_addr_out    <= jump_addr_in;
            update_btb_out   <= update_btb_in;
        end
    end

endmodule

