`timescale 1ns/1ns

module tb_l1_cache;
    reg clk = 0;
    always #5 clk = ~clk;

    reg reset = 1;

    reg  [31:0] cpu_addr = 32'h0;
    reg  [31:0] cpu_din = 32'h0;
    wire [31:0] cpu_dout;
    reg   [3:0] cpu_be = 4'hF;
    reg         cpu_valid = 1'b0;
    reg         cpu_write = 1'b0;
    wire        cpu_ready;
    wire        cpu_resp_valid;

    wire [31:0] mem_addr;
    wire [31:0] mem_din;
    reg  [31:0] mem_dout = 32'h0;
    wire  [3:0] mem_be;
    wire  [7:0] mem_burstcount;
    reg         mem_ready = 1'b0;
    wire        mem_valid;
    wire        mem_write;
    reg         mem_resp_valid = 1'b0;

    reg  [31:0] snoop_addr = 32'h0;
    reg         snoop_valid = 1'b0;

    l1_cache #(
        .SET_BITS(3)
    ) dut (
        .clk(clk),
        .reset(reset),

        .cpu_addr(cpu_addr),
        .cpu_din(cpu_din),
        .cpu_dout(cpu_dout),
        .cpu_be(cpu_be),
        .cpu_valid(cpu_valid),
        .cpu_write(cpu_write),
        .cpu_ready(cpu_ready),
        .cpu_resp_valid(cpu_resp_valid),

        .mem_addr(mem_addr),
        .mem_din(mem_din),
        .mem_dout(mem_dout),
        .mem_be(mem_be),
        .mem_burstcount(mem_burstcount),
        .mem_busy(1'b0),
        .mem_valid(mem_valid),
        .mem_write(mem_write),
        .mem_ready(mem_ready),
        .mem_resp_valid(mem_resp_valid),

        .snoop_addr(snoop_addr),
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
        mem_dout <= 32'h0;

        if (rd_left != 8'd0) begin
            mem_resp_valid <= 1'b1;
            mem_dout <= mem_get32(rd_addr);
            rd_addr <= rd_addr + 32'd4;
            rd_left <= rd_left - 8'd1;
        end

        if (mem_valid && !mem_ready && rd_left == 8'd0) begin
            mem_ready <= 1'b1;
            if (mem_write) begin
                if (mem_be[0]) mem[mem_addr + 0] <= mem_din[7:0];
                if (mem_be[1]) mem[mem_addr + 1] <= mem_din[15:8];
                if (mem_be[2]) mem[mem_addr + 2] <= mem_din[23:16];
                if (mem_be[3]) mem[mem_addr + 3] <= mem_din[31:24];
            end else begin
                rd_addr <= mem_addr;
                rd_left <= mem_burstcount == 8'd0 ? 8'd1 : mem_burstcount;
            end
        end
    end

    task automatic cache_read(input [31:0] addr, input [3:0] be, input [31:0] expected);
    begin
        do @(negedge clk); while (!cpu_ready);
        cpu_addr = addr;
        cpu_be = be;
        cpu_din = 32'h0;
        cpu_write = 1'b0;
        cpu_valid = 1'b1;
        @(negedge clk);
        cpu_valid = 1'b0;
        if (!cpu_resp_valid)
            do @(negedge clk); while (!cpu_resp_valid);
        if (cpu_dout !== expected) begin
            $display("L1 READ FAIL addr=%08x got=%08x expected=%08x", addr, cpu_dout, expected);
            $fatal(1);
        end
    end
    endtask

    task automatic cache_write(input [31:0] addr, input [3:0] be, input [31:0] data);
    begin
        do @(negedge clk); while (!cpu_ready);
        cpu_addr = addr;
        cpu_be = be;
        cpu_din = data;
        cpu_write = 1'b1;
        cpu_valid = 1'b1;
        @(negedge clk);
        cpu_valid = 1'b0;
        cpu_write = 1'b0;
    end
    endtask

    initial begin
        fork
            begin
                repeat (5000) @(posedge clk);
                $display("L1 TIMEOUT state=%0d ready=%0b valid=%0b resp=%0b mem_valid=%0b mem_ready=%0b storeq_count=%0d",
                         dut.state, cpu_ready, cpu_valid, cpu_resp_valid,
                         mem_valid, mem_ready, dut.storeq_count);
                $fatal(1);
            end
        join_none

        for (integer i = 0; i < 4096; i = i + 1)
            mem[i] = 8'h00;

        mem_put32(32'h40, 32'h4433_2211);
        mem_put32(32'h44, 32'h8877_6655);
        mem_put32(32'h48, 32'hCCBB_AA99);
        mem_put32(32'h4C, 32'h00FF_EEDD);
        mem_put32(32'h80, 32'h0102_0304);

        repeat (5) @(posedge clk);
        reset <= 1'b0;
        repeat (20) @(posedge clk);

        cache_read(32'h40, 4'hF, 32'h4433_2211);       // miss + fill
        cache_read(32'h40, 4'hF, 32'h4433_2211);       // hit
        cache_write(32'h40, 4'hC, 32'hAAAA_5555);      // write-hit patch
        cache_read(32'h40, 4'hF, 32'hAAAA_2211);

        cache_write(32'h80, 4'hF, 32'hDEAD_BEEF);      // write miss, no allocate
        cache_read(32'h80, 4'hF, 32'hDEAD_BEEF);       // fill patched from store queue

        repeat (20) @(posedge clk);
        mem_put32(32'h40, 32'hCAFE_BABE);
        @(posedge clk);
        snoop_addr <= 32'h40;
        snoop_valid <= 1'b1;
        @(posedge clk);
        snoop_valid <= 1'b0;
        cache_read(32'h40, 4'hF, 32'hCAFE_BABE);       // snoop invalidated line

        $display("L1 PIPT cache unit test PASS");
        $finish;
    end
endmodule
