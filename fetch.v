// inst_mem.v
// Simple 32-bit word-addressed instruction memory

module inst_mem (
    input  wire [31:0] pc, 
	input  rst,clk,flush,read_en,     // byte address
    output reg [31:0] instruction
);
    reg [31:0] mem [0:255];

    initial begin
        $readmemh("instructions.hex", mem);  // optional
    end


always @(posedge clk) 
begin

        if (rst) 

            instruction <= 32'h00000000; // Reset instruction to NOP

        
	 else if (flush) 

            instruction <= 32'h00000000; // Flush instruction to NOP

         
	else if (read_en) 

            instruction <= mem[pc[11:2]]; // Fetch instruction based on PC

        

    end

endmodule




// pc_reg.v
module pc_reg (
    input  wire clk,
    input  wire rst,
    input  wire pc_en,
    input  wire [31:0] next_pc,
    output reg  [31:0] pc
);
    always @(posedge clk or posedge rst) begin
        if (rst)
            pc <= 32'h0000_0000;
        else if (pc_en)
            pc <= next_pc;
    end
endmodule



module if_stage_simple_btb (
    input  wire clk,
    input  wire rst,

    // Hazard stall from hazard unit
    input  wire pc_en,
    input  wire flush,

    // Signals from EX (branch/jump resolution)
    input  wire        modify_pc_ex,
    input  wire [31:0] update_pc_ex,
    input  wire [31:0] pc_ex,
    input  wire [31:0] jump_addr_ex,
    input  wire        update_btb_ex,
    input  wire        ex_branch_taken,

    // Outputs to IF/ID
    output wire [31:0] pc_if,
    output wire [31:0] instr_if,
    output wire        predictedTaken_if,
    output wire [31:0] predictedTarget_if
);

    // ------------------------------------------------------
    // PC Register using pc_reg module
    // ------------------------------------------------------
    wire [31:0] pc_current;
    reg  [31:0] pc_next;

    pc_reg u_pc_reg (
        .clk(clk),
        .rst(rst),
        .pc_en(pc_en),
        .next_pc(pc_next),
        .pc(pc_current)
    );

    assign pc_if = pc_current;

    // ------------------------------------------------------
    // BTB Prediction Lookup
    // ------------------------------------------------------
    wire        btb_valid;
    wire        btb_taken;
    wire [31:0] btb_target;

    btb u_btb (
        .clk(clk),
        .rst(rst),

        // FETCH
        .pc(pc_current),
        .predict_valid(btb_valid),
        .predict_taken(btb_taken),
        .predict_target(btb_target),

        // UPDATE (from EX)
        .update_en(update_btb_ex),
        .update_pc(pc_ex),
        .actual_taken(ex_branch_taken),
        .update_target(jump_addr_ex)
    );

    assign predictedTaken_if  = btb_valid && btb_taken;
    assign predictedTarget_if = predictedTaken_if ? btb_target
                                                  : (pc_current + 32'd4);

    // ------------------------------------------------------
    // NEXT PC selection
    // Priority:
    // 1) modify_pc_ex (redirect)
    // 2) BTB prediction
    // 3) default sequential PC + 4
    // ------------------------------------------------------
    always @(*) begin
        if (modify_pc_ex)
            pc_next = update_pc_ex;
        else if (btb_valid && btb_taken)
            pc_next = btb_target;
        else
            pc_next = pc_current + 32'd4;
    end

    // ------------------------------------------------------
    // INSTRUCTION MEMORY
    // ------------------------------------------------------
    inst_mem u_imem (
        .pc(pc_current),
        .rst(rst),
        .clk(clk),
        .flush(flush),
        .read_en(pc_en),
        .instruction(instr_if)
    );

endmodule


module btb_file #(
    parameter SETS = 8,
    parameter WAYS = 2,
    parameter TAGW = 27
)(
    input  wire                  clk,
    input  wire                  rst,

    // -------- READ PORT --------
    input  wire [2:0]            rd_set,
    input  wire [0:0]            rd_way0,   // dummy, to align structure
    output wire                  rd_valid0,
    output wire [TAGW-1:0]       rd_tag0,
    output wire [31:0]           rd_target0,
    output wire [1:0]            rd_state0,

    input  wire [0:0]            rd_way1,
    output wire                  rd_valid1,
    output wire [TAGW-1:0]       rd_tag1,
    output wire [31:0]           rd_target1,
    output wire [1:0]            rd_state1,

    // -------- WRITE PORT --------
    input  wire                  wr_en,
    input  wire [2:0]            wr_set,
    input  wire                  wr_way,     // 0 or 1
    input  wire                  wr_valid,
    input  wire [TAGW-1:0]       wr_tag,
    input  wire [31:0]           wr_target,
    input  wire [1:0]            wr_state,

    // LRU
    output wire                  rd_lru,
    input  wire                  wr_lru_en,
    input  wire                  wr_lru_val
);

    // ================= Arrays =================
    reg                valid_arr  [0:SETS-1][0:WAYS-1];
    reg [TAGW-1:0]     tag_arr    [0:SETS-1][0:WAYS-1];
    reg [31:0]         target_arr [0:SETS-1][0:WAYS-1];
    reg [1:0]          state_arr  [0:SETS-1][0:WAYS-1];
    reg                lru        [0:SETS-1];

    // ============= READ ACCESS =============
    assign rd_valid0  = valid_arr[rd_set][0];
    assign rd_tag0    = tag_arr[rd_set][0];
    assign rd_target0 = target_arr[rd_set][0];
    assign rd_state0  = state_arr[rd_set][0];

    assign rd_valid1  = valid_arr[rd_set][1];
    assign rd_tag1    = tag_arr[rd_set][1];
    assign rd_target1 = target_arr[rd_set][1];
    assign rd_state1  = state_arr[rd_set][1];

    assign rd_lru     = lru[rd_set];

    // ============= WRITE ACCESS =============
    integer i,j;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i=0; i<SETS; i=i+1) begin
                lru[i] <= 0;
                for (j=0; j<WAYS; j=j+1) begin
                    valid_arr[i][j]  <= 0;
                    tag_arr[i][j]    <= 0;
                    target_arr[i][j] <= 0;
                    state_arr[i][j]  <= 2'b01;   // weakly not taken
                end
            end
        end
        else begin
            if (wr_en) begin
                valid_arr [wr_set][wr_way] <= wr_valid;
                tag_arr   [wr_set][wr_way] <= wr_tag;
                target_arr[wr_set][wr_way] <= wr_target;
                state_arr [wr_set][wr_way] <= wr_state;
            end

            if (wr_lru_en)
                lru[wr_set] <= wr_lru_val;
        end
    end

endmodule


module dynamic_branch_predictor(
    input  wire [1:0] curr_state,
    input  wire       actual_taken,
    output reg  [1:0] next_state
);
    always @(*) begin
        if (actual_taken) begin
            case (curr_state)
                2'b00: next_state = 2'b01;
                2'b01: next_state = 2'b10;
                2'b10: next_state = 2'b11;
                2'b11: next_state = 2'b11;
            endcase
        end
        else begin
            case (curr_state)
                2'b00: next_state = 2'b00;
                2'b01: next_state = 2'b00;
                2'b10: next_state = 2'b01;
                2'b11: next_state = 2'b10;
            endcase
        end
    end
endmodule


module lru_next(
    input  wire hit_way,     // 0 or 1
    output wire next_lru     // 0 => way0 LRU, 1 => way1 LRU
);
    assign next_lru = hit_way ? 1'b0 : 1'b1;
endmodule



module lru_reg(
    input  wire clk,
    input  wire rst,
    input  wire en,
    input  wire next_lru,
    output reg  lru_out
);
    always @(posedge clk or posedge rst) begin
        if (rst)
            lru_out <= 1'b0;
        else if (en)
            lru_out <= next_lru;
    end
endmodule


module btb_read #(
    parameter TAGW = 27
)(
    input  wire [31:0] pc,

    // data from file
    input  wire rd_valid0,
    input  wire [TAGW-1:0] rd_tag0,
    input  wire rd_valid1,
    input  wire [TAGW-1:0] rd_tag1,

    output wire [2:0]  set_index,
    output wire [TAGW-1:0] tag,
    output wire hit0,
    output wire hit1
);
    assign set_index = pc[4:2];
    assign tag       = pc[31:5];

    assign hit0 = rd_valid0 && (rd_tag0 == tag);
    assign hit1 = rd_valid1 && (rd_tag1 == tag);
endmodule


module btb_write #(
    parameter TAGW = 27
)(
    input  wire clk,
    input  wire rst,
    input  wire update_en,
    input  wire [31:0] update_pc,
    input  wire actual_taken,
    input  wire [31:0] update_target,

    // read info for hit detection
    input  wire rd_valid0_upd,
    input  wire [TAGW-1:0] rd_tag0_upd,
    input  wire rd_valid1_upd,
    input  wire [TAGW-1:0] rd_tag1_upd,
    input  wire rd_lru_upd,

    // write commands to btb_file
    output reg         wr_en,
    output reg  [2:0]  wr_set,
    output reg         wr_way,
    output reg         wr_valid,
    output reg  [TAGW-1:0] wr_tag,
    output reg  [31:0] wr_target,
    output reg  [1:0]  wr_state,

    // LRU
    output reg         wr_lru_en,
    output reg         wr_lru_val,

    // predictor input/output
    input  wire [1:0]  state0_in,
    input  wire [1:0]  state1_in,
    output wire [1:0]  next_state0,
    output wire [1:0]  next_state1
);

    wire [2:0] upd_set = update_pc[4:2];
    wire [TAGW-1:0] upd_tag = update_pc[31:5];

    // predictor logic
    dynamic_branch_predictor dp0(state0_in, actual_taken, next_state0);
    dynamic_branch_predictor dp1(state1_in, actual_taken, next_state1);

    wire hit0 = rd_valid0_upd && (rd_tag0_upd == upd_tag);
    wire hit1 = rd_valid1_upd && (rd_tag1_upd == upd_tag);

    always @(*) begin
        wr_en      = 0;
        wr_lru_en  = 0;

        if (!update_en) begin
            wr_valid = 0;
            wr_set   = 0;
            wr_way   = 0;
            wr_tag   = 0;
            wr_target= 0;
            wr_state = 0;
            wr_lru_val = 0;
        end
        else begin
            wr_set = upd_set;

            if (hit0) begin
                wr_way   = 0;
                wr_en    = 1;
                wr_valid = 1;
                wr_tag   = upd_tag;
                wr_state = next_state0;
                wr_target= actual_taken ? update_target : update_target;
                wr_lru_en = 1;
                wr_lru_val = 1;

            end else if (hit1) begin
                wr_way   = 1;
                wr_en    = 1;
                wr_valid = 1;
                wr_tag   = upd_tag;
                wr_state = next_state1;
                wr_target= actual_taken ? update_target : update_target;
                wr_lru_en = 1;
                wr_lru_val = 0;

            end else begin
                wr_en    = 1;
                wr_valid = 1;
                wr_tag   = upd_tag;
                wr_state = actual_taken ? 2'b10 : 2'b01;
                wr_target= update_target;
                wr_lru_en = 1;

                if (!rd_valid0_upd) begin
                    wr_way = 0;
                    wr_lru_val = 1;
                end else if (!rd_valid1_upd) begin
                    wr_way = 1;
                    wr_lru_val = 0;
                end else if (rd_lru_upd == 0) begin
                    wr_way = 0;
                    wr_lru_val = 1;
                end else begin
                    wr_way = 1;
                    wr_lru_val = 0;
                end
            end
        end
    end
endmodule



// =====================================================================
// btb.v  (Top module for 2-way Set-Associative BTB with:
//         - BTB Storage Arrays (btb_file)
//         - Fetch Read Path (btb_read)
//         - Update Logic (btb_write)
//         - 2-bit Predictor per Way
//         - LRU Maintenance)
// =====================================================================

module btb #(
    parameter SETS = 8,
    parameter WAYS = 2,
    parameter TAGW = 27
)(
    input              clk,
    input              rst,

    // ================= FETCH =================
    input      [31:0]  pc,
    output reg         predict_valid,
    output reg         predict_taken,
    output reg [31:0]  predict_target,

    // ================= UPDATE =================
    input              update_en,
    input      [31:0]  update_pc,
    input              actual_taken,
    input      [31:0]  update_target
);

    // ============================================================
    // FETCH STAGE
    // ============================================================
    wire [2:0] fetch_set;
    wire [TAGW-1:0] fetch_tag;
    wire hit0, hit1;

    // Data returned from btb_file for fetch read
    wire rd_valid0, rd_valid1;
    wire [TAGW-1:0] rd_tag0, rd_tag1;
    wire [31:0] rd_target0, rd_target1;
    wire [1:0]  rd_state0, rd_state1;
    wire rd_lru;

    // ----------------- FETCH READ MODULE -----------------
    btb_read #(.TAGW(TAGW)) U_READ (
        .pc(pc),

        .rd_valid0(rd_valid0),
        .rd_tag0(rd_tag0),

        .rd_valid1(rd_valid1),
        .rd_tag1(rd_tag1),

        .set_index(fetch_set),
        .tag(fetch_tag),
        .hit0(hit0),
        .hit1(hit1)
    );

    // ----------------- FETCH OUTPUT LOGIC -----------------
    always @(*) begin
        predict_valid = hit0 | hit1;

        if (hit0) begin
            predict_taken  = rd_state0[1];
            predict_target = rd_target0;
        end
        else if (hit1) begin
            predict_taken  = rd_state1[1];
            predict_target = rd_target1;
        end
        else begin
            predict_taken  = 1'b0;
            predict_target = pc + 32'd4;
        end
    end

    // ============================================================
    // UPDATE STAGE (WRITE PATH)
    // ============================================================
    wire [2:0] upd_set = update_pc[4:2];
    wire [TAGW-1:0] upd_tag = update_pc[31:5];

    // Signals from BTB file for update side read
    wire upd_valid0 = rd_valid0;
    wire upd_valid1 = rd_valid1;
    wire [TAGW-1:0] upd_tag0 = rd_tag0;
    wire [TAGW-1:0] upd_tag1 = rd_tag1;

    // Predictor states for write logic
    wire [1:0] next_state0, next_state1;

    // ----------------- WRITE CONTROL -----------------
    wire         wr_en;
    wire [2:0]   wr_set;
    wire         wr_way;
    wire         wr_valid;
    wire [TAGW-1:0] wr_tag;
    wire [31:0] wr_target;
    wire [1:0]  wr_state;

    wire wr_lru_en;
    wire wr_lru_val;

    // ----------------- UPDATE MODULE -----------------
    btb_write #(.TAGW(TAGW)) U_WRITE (
        .clk(clk),
        .rst(rst),
        .update_en(update_en),
        .update_pc(update_pc),
        .actual_taken(actual_taken),
        .update_target(update_target),

        .rd_valid0_upd(upd_valid0),
        .rd_tag0_upd(upd_tag0),
        .rd_valid1_upd(upd_valid1),
        .rd_tag1_upd(upd_tag1),
        .rd_lru_upd(rd_lru),

        .wr_en(wr_en),
        .wr_set(wr_set),
        .wr_way(wr_way),
        .wr_valid(wr_valid),
        .wr_tag(wr_tag),
        .wr_target(wr_target),
        .wr_state(wr_state),

        .wr_lru_en(wr_lru_en),
        .wr_lru_val(wr_lru_val),

        .state0_in(rd_state0),
        .state1_in(rd_state1),
        .next_state0(next_state0),
        .next_state1(next_state1)
    );

    // ============================================================
    // BTB FILE (Storage Arrays)
    // ============================================================

    btb_file #(.SETS(SETS), .WAYS(WAYS), .TAGW(TAGW)) U_FILE (
        .clk(clk),
        .rst(rst),

        // READ side
        .rd_set(fetch_set),
        .rd_way0(1'b0),
        .rd_valid0(rd_valid0),
        .rd_tag0(rd_tag0),
        .rd_target0(rd_target0),
        .rd_state0(rd_state0),

        .rd_way1(1'b0),
        .rd_valid1(rd_valid1),
        .rd_tag1(rd_tag1),
        .rd_target1(rd_target1),
        .rd_state1(rd_state1),

        .rd_lru(rd_lru),

        // WRITE side
        .wr_en(wr_en),
        .wr_set(wr_set),
        .wr_way(wr_way),
        .wr_valid(wr_valid),
        .wr_tag(wr_tag),
        .wr_target(wr_target),
        .wr_state(wr_state),

        .wr_lru_en(wr_lru_en),
        .wr_lru_val(wr_lru_val)
    );

endmodule

