// mem_stage_and_data_path_fixed.v
// Corrected load/store datapath + data memory + mem_stage wiring.
// Small, safe fix: correct halfword extraction for little-endian layout.


module load_datapath (
    input  wire [2:0]  load_type,
    input  wire [31:0] mem_data_in,
    input  wire [31:0] addr,
    output reg  [31:0] read_data
);
    // byte lanes (little-endian)
    wire [7:0]  byte0 = mem_data_in[7:0];
    wire [7:0]  byte1 = mem_data_in[15:8];
    wire [7:0]  byte2 = mem_data_in[23:16];
    wire [7:0]  byte3 = mem_data_in[31:24];

    // halfwords for little-endian: low half (bytes[1:0]), high half (bytes[3:2])
    wire [15:0] half0 = {byte1, byte0}; // addr[1] == 0 -> bytes [1:0]
    wire [15:0] half1 = {byte3, byte2}; // addr[1] == 1 -> bytes [3:2]

    // selected byte depending on addr[1:0]
    wire [7:0] selected_byte = (addr[1:0] == 2'b00) ? byte0 :
                               (addr[1:0] == 2'b01) ? byte1 :
                               (addr[1:0] == 2'b10) ? byte2 :
                                                      byte3;

    // select halfword by addr[1]
    wire [15:0] selected_half = (addr[1] == 1'b0) ? half0 : half1;

    always @(*) begin
        case (load_type)
            3'b000: begin // LB - sign-extend byte
                read_data = {{24{selected_byte[7]}}, selected_byte};
            end
            3'b011: begin // LBU - zero-extend byte
                read_data = {24'b0, selected_byte};
            end
            3'b001: begin // LH - sign-extend halfword
                read_data = {{16{selected_half[15]}}, selected_half};
            end
            3'b100: begin // LHU - zero-extend halfword
                read_data = {16'b0, selected_half};
            end
            3'b010: begin // LW - full word
                read_data = mem_data_in;
            end
            default: begin
                read_data = 32'h00000000;
            end
        endcase
    end
endmodule


module store_datapath (
    input  wire [1:0]  store_type, // 00=SB, 01=SH, 10=SW
    input  wire [31:0] write_data, // rs2 data
    input  wire [31:0] addr,       // ALU result (byte address)
    output reg  [31:0] mem_write_data,
    output reg  [3:0]  byte_enable
);
    always @(*) begin
        mem_write_data = 32'b0;
        byte_enable    = 4'b0000;

        case(store_type)
            2'b00: begin // SB
                mem_write_data = {4{write_data[7:0]}}; // replicate byte across word
                case(addr[1:0])
                    2'b00: byte_enable = 4'b0001;
                    2'b01: byte_enable = 4'b0010;
                    2'b10: byte_enable = 4'b0100;
                    2'b11: byte_enable = 4'b1000;
                endcase
            end
            2'b01: begin // SH
                mem_write_data = {2{write_data[15:0]}};
                byte_enable = addr[1] ? 4'b1100 : 4'b0011;
            end
            2'b10: begin // SW
                mem_write_data = write_data;
                byte_enable    = 4'b1111;
            end
            default: begin
                mem_write_data = 32'b0;
                byte_enable = 4'b0000;
            end
        endcase
    end
endmodule


module data_mem_top (
    input  wire        clk,
    input  wire        mem_read,
    input  wire        mem_write,
    input  wire [2:0]  load_type,    // LB/LH/LW/LBU/LHU
    input  wire [1:0]  store_type,   // SB/SH/SW
    input  wire [31:0] addr,         // byte address
    input  wire [31:0] rs2_data,     // store data
    output wire [31:0] read_data
);

    wire [31:0] mem_write_data;
    wire [3:0]  byte_enable;
    wire [31:0] mem_data_out;

    // STORE DATAPATH
    store_datapath u_store (
        .store_type(store_type),
        .write_data(rs2_data),
        .addr(addr),
        .mem_write_data(mem_write_data),
        .byte_enable(byte_enable)
    );

    // MEMORY
    data_memory u_mem (
        .clk(clk),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .addr(addr),
        .write_data(mem_write_data),
        .byte_enable(byte_enable),
        .mem_data_out(mem_data_out)
    );

    // LOAD DATAPATH
    load_datapath u_load (
        .load_type(load_type),
        .mem_data_in(mem_data_out),
        .addr(addr),
        .read_data(read_data)
    );

endmodule


module mem_stage (
    input  wire        clk,
    input  wire        rst,
    input  wire        en,
    input  wire        flush,

    // From EX/MEM Register
    input  wire [31:0] alu_result_mem,
    input  wire [31:0] rs2_data_mem,
    input  wire [4:0]  rd_mem,
    input  wire        mem_write_mem,
    input  wire        mem_read_mem,
    input  wire [2:0]  mem_load_type_mem,
    input  wire [1:0]  mem_store_type_mem,
    input  wire        wb_reg_file_mem,
    input  wire        memtoreg_mem,

    // From EX stage (direct path)
    input  wire        modify_pc_mem,
    input  wire [31:0] update_pc_mem,
    input  wire [31:0] jump_addr_mem,
    input  wire        update_btb_mem,

    // To MEM/WB stage
    output wire [31:0] alu_result_for_wb,
    output wire [31:0] load_wb_data,
    output wire [4:0]  rd_for_wb,
    output wire        wb_reg_file_out,
    output wire        memtoreg_out,

    // Pass through control flow
    output wire        modify_pc_out,
    output wire [31:0] update_pc_out,
    output wire [31:0] jump_addr_out,
    output wire        update_btb_out
);

    wire [31:0] mem_read_data;

    data_mem_top u_datamem (
        .clk(clk),
        .mem_read(mem_read_mem),
        .mem_write(mem_write_mem),
        .load_type(mem_load_type_mem),
        .store_type(mem_store_type_mem),
        .addr(alu_result_mem),
        .rs2_data(rs2_data_mem),
        .read_data(mem_read_data)
    );

    // Outputs to MEM/WB stage
    assign alu_result_for_wb = alu_result_mem;
    assign load_wb_data      = mem_read_data;
    assign rd_for_wb         = rd_mem;
    assign wb_reg_file_out   = wb_reg_file_mem;
    assign memtoreg_out      = memtoreg_mem;

    // Pass-through branch/jump information
    assign modify_pc_out     = modify_pc_mem;
    assign update_pc_out     = update_pc_mem;
    assign jump_addr_out     = jump_addr_mem;
    assign update_btb_out    = update_btb_mem;

endmodule


module data_memory (
    input  wire        clk,
    input  wire        mem_read,
    input  wire        mem_write,
    input  wire [31:0] addr,          // byte address
    input  wire [31:0] write_data,    // from store datapath
    input  wire [3:0]  byte_enable,   // from store datapath
    output reg  [31:0] mem_data_out   // to load datapath
);
    reg [7:0] mem [0:1023]; // 1KB byte-addressable

    integer i;
    initial begin
        for(i=0;i<1024;i=i+1)
            mem[i] = 8'b0; // avoid X in simulation
    end

    // WRITE — Byte controlled
    always @(posedge clk) begin
        if (mem_write) begin
            if (byte_enable[0]) mem[addr]     <= write_data[7:0];
            if (byte_enable[1]) mem[addr+1]   <= write_data[15:8];
            if (byte_enable[2]) mem[addr+2]   <= write_data[23:16];
            if (byte_enable[3]) mem[addr+3]   <= write_data[31:24];
        end
    end

    // READ — Form 32-bit word from 4 bytes (combinational)
    always @(*) begin
        if (mem_read) begin
            mem_data_out = { mem[addr+3], mem[addr+2], mem[addr+1], mem[addr] };
        end else begin
            mem_data_out = 32'b0;
        end
    end
endmodule
