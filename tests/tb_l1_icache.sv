`timescale 1ns/1ns

module tb_l1_icache;
    reg clk = 0;
    always #5 clk = ~clk;

    reg reset = 1;
    reg [31:0] cpu_addr = 32'h0;
    wire [127:0] cpu_line;
    reg cpu_valid = 1'b0;
    wire cpu_ready;
    wire cpu_resp_valid;

    wire [31:0] mem_addr;
    reg [31:0] mem_dout = 32'h0;
    wire [3:0] mem_be;
    wire [7:0] mem_burstcount;
    reg mem_ready = 1'b0;
    wire mem_valid;
    reg mem_resp_valid = 1'b0;

    reg [31:0] snoop_addr = 32'h0;
    reg [31:0] snoop_data = 32'h0;
    reg [3:0] snoop_be = 4'h0;
    reg snoop_patch = 1'b0;
    reg snoop_valid = 1'b0;

    l1_icache #(.SET_BITS(3)) dut (
        .clk(clk),
        .reset(reset),
        .cpu_addr(cpu_addr),
        .cpu_line(cpu_line),
        .cpu_valid(cpu_valid),
        .cpu_ready(cpu_ready),
        .cpu_resp_valid(cpu_resp_valid),
        .mem_addr(mem_addr),
        .mem_dout(mem_dout),
        .mem_be(mem_be),
        .mem_burstcount(mem_burstcount),
        .mem_busy(1'b0),
        .mem_valid(mem_valid),
        .mem_ready(mem_ready),
        .mem_resp_valid(mem_resp_valid),
        .snoop_addr(snoop_addr),
        .snoop_data(snoop_data),
        .snoop_be(snoop_be),
        .snoop_patch(snoop_patch),
        .snoop_valid(snoop_valid),
        .cache_enable(1'b1)
    );

    reg [7:0] mem [0:4095];
    reg [31:0] rd_addr = 32'h0;
    reg [7:0] rd_left = 8'd0;

    task automatic mem_put32(input [31:0] addr, input [31:0] data);
    begin
        mem[addr + 0] = data[7:0];
        mem[addr + 1] = data[15:8];
        mem[addr + 2] = data[23:16];
        mem[addr + 3] = data[31:24];
    end
    endtask

    function automatic [31:0] mem_get32(input [31:0] addr);
        mem_get32 = {mem[addr + 3], mem[addr + 2], mem[addr + 1], mem[addr + 0]};
    endfunction

    always_ff @(posedge clk) begin
        mem_ready <= 1'b0;
        mem_resp_valid <= 1'b0;

        if (rd_left != 8'd0) begin
            mem_resp_valid <= 1'b1;
            mem_dout <= mem_get32(rd_addr);
            rd_addr <= rd_addr + 32'd4;
            rd_left <= rd_left - 8'd1;
        end

        if (mem_valid && !mem_ready && rd_left == 8'd0) begin
            mem_ready <= 1'b1;
            rd_addr <= mem_addr;
            rd_left <= mem_burstcount == 8'd0 ? 8'd1 : mem_burstcount;
        end
    end

    task automatic cache_read(input [31:0] addr, input [127:0] expected);
    begin
        do @(negedge clk); while (!cpu_ready);
        cpu_addr = addr;
        cpu_valid = 1'b1;
        @(negedge clk);
        cpu_valid = 1'b0;
        if (!cpu_resp_valid)
            do @(negedge clk); while (!cpu_resp_valid);
        if (cpu_line !== expected) begin
            $display("L1 ICACHE READ FAIL addr=%08x got=%032x expected=%032x",
                     addr, cpu_line, expected);
            $fatal(1);
        end
    end
    endtask

    initial begin
        fork
            begin
                repeat (2000) @(posedge clk);
                $display("L1 ICACHE TIMEOUT state=%0d ready=%0b resp=%0b",
                         dut.state, cpu_ready, cpu_resp_valid);
                $fatal(1);
            end
        join_none

        for (integer n = 0; n < 4096; n = n + 1)
            mem[n] = 8'h0;
        mem_put32(32'h40, 32'h4433_2211);
        mem_put32(32'h44, 32'h8877_6655);
        mem_put32(32'h48, 32'hCCBB_AA99);
        mem_put32(32'h4C, 32'h00FF_EEDD);

        repeat (5) @(posedge clk);
        reset <= 1'b0;
        repeat (20) @(posedge clk);

        cache_read(32'h40, 128'h00FF_EEDD_CCBB_AA99_8877_6655_4433_2211);
        cache_read(32'h40, 128'h00FF_EEDD_CCBB_AA99_8877_6655_4433_2211);

        // Accept a hit in the same cycle that a store snoops the cached line.
        // The old line must not escape before the registered invalidation.
        do @(negedge clk); while (!cpu_ready);
        mem_put32(32'h40, 32'hDEAD_BEEF);
        cpu_addr = 32'h40;
        cpu_valid = 1'b1;
        snoop_addr = 32'h40;
        snoop_data = 32'hDEAD_BEEF;
        snoop_be = 4'hF;
        snoop_patch = 1'b1;
        snoop_valid = 1'b1;
        @(negedge clk);
        cpu_valid = 1'b0;
        snoop_valid = 1'b0;
        snoop_patch = 1'b0;
        if (cpu_resp_valid) begin
            $display("L1 ICACHE SNOOP RACE exposed stale hit %032x", cpu_line);
            $fatal(1);
        end
        do @(negedge clk); while (!cpu_resp_valid);
        if (cpu_line !== 128'h00FF_EEDD_CCBB_AA99_8877_6655_DEAD_BEEF) begin
            $display("L1 ICACHE SNOOP RACE FAIL got=%032x", cpu_line);
            $fatal(1);
        end

        $display("L1 PIPT instruction cache unit test PASS");
        $finish;
    end
endmodule
