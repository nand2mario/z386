`timescale 1ns/1ns

//
// Testbench for z386 - test386.asm Program Test Runner
// Runs the comprehensive test386 CPU test program
//

module tb_test386;
    // Clock and reset
    reg clk = 0;
    always #5 clk <= ~clk;  // 100 MHz clock

    reg reset_n = 0;

    // Test control
    int max_cycles = 100_000_000;  // 100M cycles max (plenty for test386)
    int cycle = 0;

    // CPU bus interface (32-bit, ready/valid)
    wire [31:2] addr;       // 4-byte aligned address
    wire [3:0]  be;         // Byte enables
    wire [7:0]  burstcount;
    wire [31:0] dout;       // Data output from CPU
    wire        valid, write, io;
    reg  [31:0] din;        // Data input to CPU
    reg         ready;
    reg         resp_valid;
    reg         intr = 0;
    reg         nmi = 0;
    wire        inta;

    // Instantiate the z386 CPU
    z386 dut (
        .clk(clk),
        .reset_n(reset_n),
        .addr(addr),
        .be(be),
        .burstcount(burstcount),
        .din(din),
        .dout(dout),
        .valid(valid),
        .write(write),
        .io(io),
        .ready(ready),
        .resp_valid(resp_valid),
        .intr(intr),
        .nmi(nmi),
        .inta(inta),
        .snoop_addr(32'h0),
        .snoop_valid(1'b0),
        .a20_enable(1'b1),
        .single_step(1'b0), // Continuous execution for test386.asm
        .dbg_CS(),
        .dbg_EIP(),
        .dbg_CS_base(),
        .dbg_pe(),
        .dbg_vm(),
        .triple_fault_reset()
    );

    // Memory: 1MB for protected mode testing
    // test386 uses memory up to 0x9FFFF (640KB) for code/data,
    // plus ring 0 stack at ~0x1FFFB, code at 0xF0000
    localparam MEM_SIZE = 1 << 20;  // 1MB
    reg [7:0] mem [0:MEM_SIZE-1];

    // POST tracking
    reg [7:0] current_post = 8'hFF;
    reg [7:0] last_post = 8'hFF;
    int post_count = 0;

    // Instruction counting
    wire instruction_boundary = dut.uc_is_rni && dut.uc_active;
    reg prev_instruction_boundary = 0;
    longint instruction_count = 0;

    // Character output line buffer
    reg [7:0] char_buf [0:255];
    int char_pos = 0;
    reg [7:0] rd_remaining = 8'd0;
    reg [7:0] rd_index = 8'd0;
    reg [31:0] rd_byte_addr = 32'h0;
    reg rd_io_pending = 1'b0;

    // Memory behavior with 1-cycle ready/valid latency
    always @(posedge clk) begin
        ready <= 1'b0;
        resp_valid <= 1'b0;
        din <= 32'h00000000;

        if (rd_remaining != 8'd0) begin
            reg [31:0] byte_addr;
            byte_addr = rd_byte_addr + {22'd0, rd_index, 2'b00};
            if (byte_addr >= MEM_SIZE)
                byte_addr = byte_addr & (MEM_SIZE - 1);
            resp_valid <= 1'b1;
            din <= rd_io_pending ? 32'hFFFFFFFF :
                   {mem[byte_addr+3], mem[byte_addr+2],
                    mem[byte_addr+1], mem[byte_addr+0]};
            rd_index <= rd_index + 8'd1;
            rd_remaining <= rd_remaining - 8'd1;
        end

        if (valid && !ready && rd_remaining == 8'd0) begin
            ready <= 1'b1;

            if (!write) begin
                reg [31:0] byte_addr;
                byte_addr = {addr, 2'b00};
                rd_byte_addr <= byte_addr;
                rd_index <= 8'd0;
                rd_remaining <= io ? 8'd1 : ((burstcount == 8'd0) ? 8'd1 : burstcount);
                rd_io_pending <= io;

                if (!io) begin
                    if (byte_addr >= MEM_SIZE)
                        byte_addr = byte_addr & (MEM_SIZE - 1);

                    if ($test$plusargs("trace_mem"))
                        $display("MEM RD @%08x = %08x", byte_addr,
                                 {mem[byte_addr+3], mem[byte_addr+2],
                                  mem[byte_addr+1], mem[byte_addr+0]});
                end
            end else begin
                // Write
                if (io) begin
                // I/O writes
                reg [15:0] port;
                port = {addr[15:2], 2'b00};

                // POST port (0x190)
                if (port == 16'h0190) begin
                    current_post <= dout[7:0];
                    if (dout[7:0] != last_post) begin
                        $display("POST: 0x%02X at cycle %0d, instr %0d",
                                 dout[7:0], cycle, instruction_count);
                        last_post <= dout[7:0];
                        post_count <= post_count + 1;

                        // Success!
                        if (dout[7:0] == 8'hFF) begin
                            $display("");
                            $display("========================================");
                            $display("  TEST386 PASSED!");
                            $display("  Total cycles: %0d", cycle);
                            $display("  Total instructions: %0d", instruction_count);
                            $display("========================================");
                            $finish;
                        end
                    end
                end

                // Text output ports (LPT, COM, custom OUT)
                // LPT1=3BC, LPT2=378, LPT3=278
                // COM1=3F8, COM2=2F8
                if (port == 16'h03BC || port == 16'h0378 || port == 16'h0278 ||
                    port == 16'h03F8 || port == 16'h02F8 ||
                    port == 16'h00E8) begin  // 0xE9 -> aligned to 0xE8
                    if (dout[7:0] == 8'h0A) begin
                        // Newline: emit buffered line
                        $write("CHAROUT: ");
                        for (int ci = 0; ci < char_pos; ci++)
                            $write("%c", char_buf[ci]);
                        $write("\n");
                        char_pos = 0;
                    end else if (dout[7:0] >= 8'h20 && dout[7:0] < 8'h7F) begin
                        if (char_pos < 255)
                            char_buf[char_pos++] = dout[7:0];
                    end
                    // ignore CR (0x0D) and other control chars
                end

                if ($test$plusargs("trace_io"))
                    $display("IO WR port=%04x data=%08x", port, dout);
            end else begin
                // Memory writes
                reg [31:0] byte_addr;
                byte_addr = {addr, 2'b00};

                if (byte_addr < MEM_SIZE) begin
                    if (be[0]) mem[byte_addr+0] <= dout[7:0];
                    if (be[1]) mem[byte_addr+1] <= dout[15:8];
                    if (be[2]) mem[byte_addr+2] <= dout[23:16];
                    if (be[3]) mem[byte_addr+3] <= dout[31:24];
                end

                if ($test$plusargs("trace_mem"))
                    $display("MEM WR @%08x be=%b data=%08x", byte_addr, be, dout);

                // Watchpoint: detect writes to code area (0xF0000+)
                if (byte_addr >= 32'hF0000 && byte_addr < 32'h100000)
                    $display("WARNING: WRITE TO CODE AREA @%08x be=%b data=%08x cycle=%0d instr=%0d CS:EIP=%04X:%08X",
                             byte_addr, be, dout, cycle, instruction_count, dut.CS, dut.EIP);
            end
        end
    end
    end

    // Load test386.bin and initialize
    string binfile;
    int fd, bytes_read;
    reg [7:0] byte_val;

    initial begin
        // Initialize memory to 0
        for (int i = 0; i < MEM_SIZE; i++)
            mem[i] = 8'h00;

        // Get binary file path
        if (!$value$plusargs("bin=%s", binfile))
            binfile = "test386.asm/test386.bin";

        // Load binary at 0xF0000 (64KB BIOS area)
        fd = $fopen(binfile, "rb");
        if (fd == 0) begin
            $display("ERROR: Cannot open %s", binfile);
            $finish;
        end

        bytes_read = 0;
        while (!$feof(fd) && bytes_read < 65536) begin
            byte_val = $fgetc(fd);
            if (!$feof(fd)) begin
                mem[32'hF0000 + bytes_read] = byte_val;
                bytes_read = bytes_read + 1;
            end
        end
        $fclose(fd);
        $display("Loaded %0d bytes from %s at 0xF0000", bytes_read, binfile);

        // Get max cycles
        if ($value$plusargs("cycles=%d", max_cycles))
            $display("Max cycles: %0d", max_cycles);

        // Reset sequence - CPU starts at F000:FFF0
        // After reset: CS=F000, IP=FFF0, physical = 0xFFFF0
        #20;
        reset_n = 0;
        #10;

        // Force initial state for reset
        // 386 reset state: CS=F000, base=FFFF0000, IP=FFF0
        // But we're in real mode so base should be F0000
        force dut.CS = 16'hF000;
        force dut.EIP = 32'h0000FFF0;
        force dut.prefetch_inst.pf_fetch_addr = 32'h000FFFF0;

        // All other segments = 0
        force dut.DS = 16'h0000;
        force dut.ES = 16'h0000;
        force dut.SS = 16'h0000;
        force dut.FS = 16'h0000;
        force dut.GS = 16'h0000;

        // General registers
        force dut.EAX = 32'h0;
        force dut.ECX = 32'h0;
        force dut.EDX = 32'h0;
        force dut.EBX = 32'h0;
        force dut.ESP = 32'h0;
        force dut.EBP = 32'h0;
        force dut.ESI = 32'h0;
        force dut.EDI = 32'h0;

        // Flags and control
        force dut.EFLAGS = 32'h00000002;
        force dut.CR0 = 32'h0;  // Real mode

        // Force continuous mode - prevent single-step halt
        force dut.halted = 1'b0;
        force dut.dbg_first_done = 1'b0;

        #50;
        reset_n = 1;

        // Release all forces (but keep halted forced to 0 for continuous execution)
        release dut.CS;
        release dut.EIP;
        release dut.prefetch_inst.pf_fetch_addr;
        release dut.DS;
        release dut.ES;
        release dut.SS;
        release dut.FS;
        release dut.GS;
        release dut.EAX;
        release dut.ECX;
        release dut.EDX;
        release dut.EBX;
        release dut.ESP;
        release dut.EBP;
        release dut.ESI;
        release dut.EDI;
        release dut.EFLAGS;
        release dut.CR0;

        $display("CPU reset complete, starting at F000:FFF0");
        $display("");
    end

    // Main test loop
    always @(posedge clk) begin
        if (reset_n) begin
            cycle <= cycle + 1;

            // Count instructions
            prev_instruction_boundary <= instruction_boundary;
            if (instruction_boundary && !prev_instruction_boundary)
                instruction_count <= instruction_count + 1;

            // Timeout
            if (cycle >= max_cycles) begin
                $display("");
                $display("========================================");
                $display("  TIMEOUT after %0d cycles", max_cycles);
                $display("  Last POST: 0x%02X", current_post);
                $display("  Instructions: %0d", instruction_count);
                $display("========================================");
                $finish;
            end

            // Check for HLT instruction execution
            // HLT enters HLTS loop at uc_addr 0x329-0x32B (RPT WIO).
            // Don't use opcode sniffing (IR=F4) because HLT from ring 3
            // faults via #GP and IR still holds F4 during fault processing.
            if (dut.uc_addr == 12'h329) begin
                if (current_post != 8'hFF) begin
                    $display("");
                    $display("========================================");
                    $display("  HLT EXECUTED - TEST FAILED");
                    $display("  Failed at POST: 0x%02X", current_post);
                    $display("  Cycle: %0d", cycle);
                    $display("  Instructions: %0d", instruction_count);
                    $display("  CS:EIP: %04X:%08X", dut.CS, dut.EIP);
                    $display("  EAX=%08X EBX=%08X ECX=%08X EDX=%08X",
                             dut.EAX, dut.EBX, dut.ECX, dut.EDX);
                    $display("  ESP=%08X EBP=%08X ESI=%08X EDI=%08X",
                             dut.ESP, dut.EBP, dut.ESI, dut.EDI);
                    $display("  DS=%04X ES=%04X SS=%04X", dut.DS, dut.ES, dut.SS);
                    $display("  EFLAGS=%08X", dut.EFLAGS);
                    $display("========================================");
                    #100;
                    $finish;
                end
            end

            // Optional: periodic status
            if ($test$plusargs("progress") && (cycle % 500000 == 0))
                $display("Progress: cycle=%0d instr=%0d POST=0x%02X EIP=%08X ESI=%08X",
                         cycle, instruction_count, current_post, dut.EIP, dut.ESI);

            // Trace each instruction
            if ($test$plusargs("trace_instr") && instruction_boundary && !prev_instruction_boundary)
                $display("INSTR[%0d]: CS:EIP=%04X:%08X IR=%02X ECX=%08X",
                         instruction_count, dut.CS, dut.EIP, dut.i.opcode, dut.ECX);

            // POST-specific debug traces removed (were for POST 0x0A, 0x08, 0x11 debugging)
            // Use VCD + vcd-server for future debugging instead
        end
    end

    // Waveform dump handled by C++ sim_main_test386.cpp (writes test386.vcd)

endmodule
