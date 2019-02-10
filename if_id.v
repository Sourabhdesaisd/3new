// if_id_reg.v
// IF/ID pipeline register with proper flush vs stall priority
module if_id_reg (
    input wire clk,
    input wire rst,
    input wire en,         // enable (1 = advance, 0 = stall/hold)
    input wire flush,      // flush the stage and insert a NOP

    // IF inputs
    input wire [31:0] pc_in,
    input wire [31:0] instr_in,
    input wire        predictedTaken_in,
    input wire [31:0] predictedTarget_in,

    // ID outputs
    output reg [31:0] pc_id,
    output reg [31:0] instr_id,
    output reg        predictedTaken_id,
    output reg [31:0] predictedTarget_id
);
    localparam NOP = 32'h00000013;  // ADDI x0, x0, 0

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pc_id               <= 32'h0;
            instr_id            <= NOP;
            predictedTaken_id   <= 1'b0;
            predictedTarget_id  <= 32'h0;
        end
        // FLUSH must have priority over stall/en so a mispredict or control-flow
        // correction can clear inflight instruction even during a stall cycle.
        else if (flush) begin
            // Insert bubble (NOP)
            pc_id               <= 32'h0;
            instr_id            <= NOP;
            predictedTaken_id   <= 1'b0;
            predictedTarget_id  <= 32'h0;
        end
        else if (!en) begin
            // Stall: hold current values (no update)
            pc_id               <= pc_id;
            instr_id            <= instr_id;
            predictedTaken_id   <= predictedTaken_id;
            predictedTarget_id  <= predictedTarget_id;
        end
        else begin
            // Normal advance
            pc_id               <= pc_in;
            instr_id            <= instr_in;
            predictedTaken_id   <= predictedTaken_in;
            predictedTarget_id  <= predictedTarget_in;
        end
    end
endmodule
