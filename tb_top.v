module main_tb;

    reg clk;
    reg rst;

    // Instantiate the core/top (change name if your top is different)
    rv32i_core dut (
        .clk(clk),
        .rst(rst)
    );

    initial begin
        // waveform / shared memory probe (as in your environment)
        $shm_open("wave.shm");
        $shm_probe("ACTMF");
    end

    // Clock generation: 10ns period
    initial begin
        clk = 1;
        forever #5 clk = ~clk;
    end

    // Test stimulus
    initial begin
        // Apply reset
        rst = 1;
        #20;       // Hold reset for 20ns
        rst = 0;

        // Run simulation for N ns then finish (adjust as needed)
        #1000;
        $display("SIMULATION DONE");
        $finish;
    end

endmodule

