module ex_mem_reg (
    input wire clk,
    input wire rst,
    input wire en,
    input wire flush,

    input wire [31:0] alu_result_ex,
    input wire [31:0] rs2_data_ex,
    input wire [4:0]  rd_ex,
    input wire        mem_write_ex,
    input wire        mem_read_ex,
    input wire [2:0]  mem_load_type_ex,
    input wire [1:0]  mem_store_type_ex,
    input wire        wb_reg_file_ex,
    input wire        memtoreg_ex,

    output reg [31:0] alu_result_mem,
    output reg [31:0] rs2_data_mem,
    output reg [4:0]  rd_mem,
    output reg        mem_write_mem,
    output reg        mem_read_mem,
    output reg [2:0]  mem_load_type_mem,
    output reg [1:0]  mem_store_type_mem,
    output reg        wb_reg_file_mem,
    output reg        memtoreg_mem
);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            alu_result_mem     <= 32'h0;
            rs2_data_mem       <= 32'h0;
            rd_mem             <= 5'd0;
            mem_write_mem      <= 1'b0;
            mem_read_mem       <= 1'b0;
            mem_load_type_mem  <= 3'b111;
            mem_store_type_mem <= 2'b11;
            wb_reg_file_mem    <= 1'b0;
            memtoreg_mem       <= 1'b0;
        end

        // -----------------------
        // FLUSH (BUBBLE INSERTION)
        // -----------------------
        else if (flush) begin
            // ? Control signals converted to bubble
            rd_mem             <= 5'd0;
            mem_write_mem      <= 1'b0;
            mem_read_mem       <= 1'b0;
            mem_load_type_mem  <= 3'b111;
            mem_store_type_mem <= 2'b11;
            wb_reg_file_mem    <= 1'b0;
            memtoreg_mem       <= 1'b0;

            // ? Data signals NOT overwritten!
            // alu_result_mem   <= alu_result_mem;
            // rs2_data_mem     <= rs2_data_mem;
        end

        // -----------------------
        // STALL (HOLD)
        // -----------------------
        else if (!en) begin
            alu_result_mem     <= alu_result_mem;
            rs2_data_mem       <= rs2_data_mem;
            rd_mem             <= rd_mem;
            mem_write_mem      <= mem_write_mem;
            mem_read_mem       <= mem_read_mem;
            mem_load_type_mem  <= mem_load_type_mem;
            mem_store_type_mem <= mem_store_type_mem;
            wb_reg_file_mem    <= wb_reg_file_mem;
            memtoreg_mem       <= memtoreg_mem;
        end

        // -----------------------
        // NORMAL PIPELINE ADVANCE
        // -----------------------
        else begin
            alu_result_mem     <= alu_result_ex;
            rs2_data_mem       <= rs2_data_ex;
            rd_mem             <= rd_ex;
            mem_write_mem      <= mem_write_ex;
            mem_read_mem       <= mem_read_ex;
            mem_load_type_mem  <= mem_load_type_ex;
            mem_store_type_mem <= mem_store_type_ex;
            wb_reg_file_mem    <= wb_reg_file_ex;
            memtoreg_mem       <= memtoreg_ex;
        end
    end
endmodule

