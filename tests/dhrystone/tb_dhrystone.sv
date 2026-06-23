`timescale 1ns/1ns

/* verilator lint_off SYNCASYNCNET */

`ifdef Z386S_CORE
`define DHRY_INTERNAL_CPU_CACHE
`endif
`ifdef Z386_INTERNAL_CACHE
`define DHRY_INTERNAL_CPU_CACHE
`endif

module tb_dhrystone;
    localparam SEG_ES = 0, SEG_CS = 1, SEG_SS = 2, SEG_DS = 3;
    localparam SEG_FS = 4, SEG_GS = 5;

    localparam CODE_PHYS_BASE = 32'h0001_0000;
    localparam LINEAR_BASE    = 32'h0001_0000;
    localparam SEG_LIMIT      = 20'hF_FFFF;
    localparam DEFAULT_DATA_FLAGS = 16'h21E0;
    localparam DEFAULT_CODE_FLAGS = 16'hA1E0;

    reg clk = 0;
    always #5 clk <= ~clk;

    reg reset_n = 0;
    int max_cycles = 2_000_000;
    int cycle = 0;

    wire [31:2] read_addr;
    wire [3:0]  read_be;
`ifdef DHRY_INTERNAL_CPU_CACHE
    wire [7:0]  cpu_burstcount;
`else
    wire [7:0]  cpu_burstcount = 8'd1;
    wire        cache_lookup;
    wire [31:0] cache_lookup_addr;
    wire        cache_lookup_write;
    wire        cache_lookup_cancel;
    wire        cache_lookup_ready;
`endif
    wire [31:0] read_data;
    wire        read_valid, cpu_write, cpu_io;
    wire [31:0] cpu_din;
    wire        read_ready;
    wire        cpu_resp_valid;
    reg  [31:0] bus_din;
    reg         bus_ready = 1'b1;
    reg         bus_resp_valid = 1'b0;
    reg         intr = 0;
    reg         nmi = 0;
    wire        inta;

    z386 dut (
        .clk(clk),
        .reset_n(reset_n),
        .addr(read_addr),
        .be(read_be),
`ifdef DHRY_INTERNAL_CPU_CACHE
        .burstcount(cpu_burstcount),
`endif
        .din(cpu_din),
        .dout(read_data),
        .valid(read_valid),
        .write(cpu_write),
        .io(cpu_io),
        .ready(read_ready),
        .resp_valid(cpu_resp_valid),
`ifdef DHRY_INTERNAL_CPU_CACHE
        .snoop_addr(32'h0),
        .snoop_valid(1'b0),
`endif
        .intr(intr),
        .nmi(nmi),
        .inta(inta),
`ifndef DHRY_INTERNAL_CPU_CACHE
        .cache_lookup(cache_lookup),
        .cache_lookup_addr(cache_lookup_addr),
        .cache_lookup_write(cache_lookup_write),
        .cache_lookup_cancel(cache_lookup_cancel),
        .cache_lookup_ready(cache_lookup_ready),
`endif
        .single_step(1'b0),
        .dbg_CS(),
`ifdef Z386S_CORE
        .dbg_EIP_cur(),
`else
        .dbg_EIP(),
`endif
        .dbg_CS_base(),
        .dbg_pe(),
        .dbg_vm()
    );

    localparam MEM_SIZE = 1 << 19;
    reg [7:0] mem [0:MEM_SIZE-1];

`ifdef Z386S_CORE
    wire instruction_retire = dut.wb_instr_boundary;
`else
    wire instruction_retire = dut.i_pop;
`endif
    longint instruction_count = 0;
    reg [31:0] prev_trace_eip = 32'hFFFF_FFFF;

`ifdef DHRY_INTERNAL_CPU_CACHE
    assign cpu_din = bus_din;
    assign read_ready = bus_ready;
    assign cpu_resp_valid = bus_resp_valid;

    wire [31:2] bus_addr = read_addr;
    wire [3:0]  bus_be = read_be;
    wire [7:0]  bus_burstcount = cpu_burstcount;
    wire [31:0] bus_dout = read_data;
    wire        bus_valid = read_valid;
    wire        bus_write = cpu_write;
    wire        bus_io = cpu_io;
    wire        bus_inta = inta;
`else
    wire        cpu_mem_valid = read_valid && !cpu_io && !inta;
    wire [31:0] cache_cpu_dout;
    wire        cache_cpu_ready;
    wire        cache_cpu_resp_valid;
    wire [31:0] cache_mem_addr;
    wire [31:0] cache_mem_din;
    wire  [3:0] cache_mem_be;
    wire  [7:0] cache_mem_burstcount;
    wire        cache_mem_valid;
    wire        cache_mem_write;

    assign cpu_din = cpu_io ? bus_din : cache_cpu_dout;
    assign read_ready = cpu_mem_valid ? cache_cpu_ready : bus_ready;
    assign cpu_resp_valid = cpu_io ? bus_resp_valid : cache_cpu_resp_valid;

    l1_cache l1_cache_inst (
        .clk           (clk),
        .reset         (!reset_n),

        .cpu_addr      ({read_addr, 2'b00}),
        .cpu_din       (read_data),
        .cpu_dout      (cache_cpu_dout),
        .cpu_be        (read_be),
        .cpu_valid     (cpu_mem_valid),
        .cpu_write     (cpu_write),
        .cpu_ready     (cache_cpu_ready),
        .cpu_resp_valid(cache_cpu_resp_valid),

        .lookup_addr   (cache_lookup_addr),
        .lookup        (cache_lookup),
        .lookup_cancel (cache_lookup_cancel),
        .lookup_ready  (cache_lookup_ready),

        .mem_addr      (cache_mem_addr),
        .mem_din       (cache_mem_din),
        .mem_dout      (bus_din),
        .mem_be        (cache_mem_be),
        .mem_burstcount(cache_mem_burstcount),
        .mem_busy      (1'b0),
        .mem_valid     (cache_mem_valid),
        .mem_write     (cache_mem_write),
        .mem_ready     (bus_ready),
        .mem_resp_valid(bus_resp_valid),

        .snoop_addr    (32'h0),
        .snoop_valid   (1'b0),
        .cache_enable  (1'b1)
    );

    wire [31:2] bus_addr = cache_mem_valid ? cache_mem_addr[31:2] : read_addr;
    wire [3:0]  bus_be = cache_mem_valid ? cache_mem_be : read_be;
    wire [7:0]  bus_burstcount = cache_mem_valid ? cache_mem_burstcount : 8'd1;
    wire [31:0] bus_dout = cache_mem_valid ? cache_mem_din : read_data;
    wire        bus_valid = cache_mem_valid || (read_valid && (cpu_io || inta));
    wire        bus_write = cache_mem_valid ? cache_mem_write : cpu_write;
    wire        bus_io = !cache_mem_valid && cpu_io;
    wire        bus_inta = !cache_mem_valid && inta;
`endif

    reg [7:0]  test_status = 8'h00;
    reg [31:0] test_data = 32'h0;
    reg        test_done = 0;

    reg [7:0]  intr_vector = 8'h20;
    reg        inta_first = 0;
    reg        inta_resp_pending = 0;
    reg [31:0] inta_resp_data = 32'h0;

    int mem_latency = 1;
    int sdram_first_latency = 5;
    int sdram_gap_cycles = 1;
    int sdram_write_busy = 3;
    bit simple_mem = 0;
    int rd_wait_count = 0;
    int rd_gap_count = 0;
    int wr_busy_count = 0;
    reg [31:0] rd_byte_addr = 32'h0;
    reg [7:0] rd_remaining = 8'd0;
    reg [7:0] rd_index = 8'd0;
    reg rd_io_pending = 1'b0;
    wire bus_busy = (rd_wait_count != 0) || (rd_gap_count != 0) ||
                    (rd_remaining != 0) || (wr_busy_count != 0) ||
                    inta_resp_pending;

    always @(posedge clk) begin
        bus_ready <= !bus_busy;
        bus_resp_valid <= 1'b0;

        if (inta_resp_pending) begin
            bus_resp_valid <= 1'b1;
            bus_din <= inta_resp_data;
            inta_resp_pending <= 1'b0;
        end else if (wr_busy_count != 0) begin
            wr_busy_count <= wr_busy_count - 1;
        end else if (rd_wait_count != 0) begin
            rd_wait_count <= rd_wait_count - 1;
        end else if (rd_gap_count != 0) begin
            rd_gap_count <= rd_gap_count - 1;
        end else if (rd_remaining != 8'd0) begin
            reg [31:0] byte_addr;
            byte_addr = rd_byte_addr + {22'd0, rd_index, 2'b00};
            if (byte_addr >= MEM_SIZE)
                byte_addr = byte_addr & (MEM_SIZE - 1);

            bus_resp_valid <= 1'b1;
            if (rd_io_pending) begin
                bus_din <= 32'hFFFF_FFFF;
            end else begin
                bus_din <= {mem[byte_addr+3], mem[byte_addr+2],
                            mem[byte_addr+1], mem[byte_addr+0]};
                if ($test$plusargs("trace_mem"))
                    $display("MEM RESP @%08x = %08x",
                             byte_addr,
                             {mem[byte_addr+3], mem[byte_addr+2],
                              mem[byte_addr+1], mem[byte_addr+0]});
            end

            rd_index <= rd_index + 8'd1;
            rd_remaining <= rd_remaining - 8'd1;
            if (!simple_mem && rd_remaining > 8'd1)
                rd_gap_count <= sdram_gap_cycles;
        end

        if (bus_valid && bus_ready && !bus_busy) begin
            if (bus_inta) begin
                bus_ready <= 1'b0;
                inta_resp_pending <= 1'b1;
                if (!inta_first) begin
                    inta_first <= 1'b1;
                    inta_resp_data <= 32'h0;
                end else begin
                    inta_first <= 1'b0;
                    inta_resp_data <= {24'h0, intr_vector};
                    intr <= 1'b0;
                end
            end else if (!bus_write) begin
                reg [7:0] burst_len;
                reg [31:0] byte_addr;
                burst_len = bus_io ? 8'd1 : bus_burstcount;
                byte_addr = {bus_addr, 2'b00};
                rd_byte_addr <= {bus_addr, 2'b00};
                rd_index <= (simple_mem || bus_io) ? 8'd1 : 8'd0;
                if (!simple_mem && !bus_io)
                    rd_wait_count <= (sdram_first_latency <= 1) ? 0 : (sdram_first_latency - 1);
                else
                    rd_wait_count <= (mem_latency <= 1) ? 0 : (mem_latency - 1);
                rd_io_pending <= bus_io;
                if (simple_mem || bus_io) begin
                    bus_ready <= (burst_len <= 8'd1);
                    bus_resp_valid <= 1'b1;
                    if (bus_io) begin
                        bus_din <= 32'hFFFF_FFFF;
                    end else begin
                        if (byte_addr >= MEM_SIZE)
                            byte_addr = byte_addr & (MEM_SIZE - 1);
                        bus_din <= {mem[byte_addr+3], mem[byte_addr+2],
                                    mem[byte_addr+1], mem[byte_addr+0]};
                    end
                    rd_remaining <= (burst_len > 8'd1) ? (burst_len - 8'd1) : 8'd0;
                end else begin
                    bus_ready <= 1'b0;
                    rd_remaining <= burst_len;
                end
            end else begin
                bus_ready <= simple_mem || bus_io;
                if (bus_io) begin
                    reg [15:0] port;
                    port = {bus_addr[15:2], 2'b00};

                    if (port == 16'h00E0) begin
                        test_status <= bus_dout[7:0];
                        if (bus_dout[7:0] == 8'h01) begin
                            $display("");
                            $display("========================================");
                            $display("  TEST PASSED!");
                            $display("  Total cycles: %0d", cycle);
                            $display("  Total instructions: %0d", instruction_count);
                            $display("========================================");
                            test_done <= 1;
                        end else if (bus_dout[7:0] == 8'hFF) begin
                            $display("");
                            $display("========================================");
                            $display("  TEST FAILED!");
                            $display("  Failure data: 0x%08X", test_data);
                            $display("  Total cycles: %0d", cycle);
                            $display("  CS:EIP: %04X:%08X", dut.CS, dut.EIP);
                            $display("========================================");
                            test_done <= 1;
                        end
                    end

                    if (port == 16'h00E4) begin
                        test_data <= bus_dout;
                        if ($test$plusargs("trace_io"))
                            $display("TEST DATA: 0x%08X", bus_dout);
                    end

                    if ($test$plusargs("trace_io"))
                        $display("IO WR port=%04x data=%08x", port, bus_dout);
                end else begin
                    reg [31:0] byte_addr;
                    byte_addr = {bus_addr, 2'b00};

                    if (byte_addr < MEM_SIZE) begin
                        if (bus_be[0]) mem[byte_addr+0] <= bus_dout[7:0];
                        if (bus_be[1]) mem[byte_addr+1] <= bus_dout[15:8];
                        if (bus_be[2]) mem[byte_addr+2] <= bus_dout[23:16];
                        if (bus_be[3]) mem[byte_addr+3] <= bus_dout[31:24];
                    end

                    if ($test$plusargs("trace_mem"))
                        $display("MEM WR @%08x be=%b data=%08x", byte_addr, bus_be, bus_dout);

                    if (!simple_mem)
                        wr_busy_count <= sdram_write_busy;
                end
            end
        end
    end

    string memfile;

    function automatic z386_pkg::seg_desc_t build_seg_desc(
        input [31:0] base, input [19:0] limit, input [15:0] flags
    );
        z386_pkg::seg_desc_t desc;
        desc.base       = base;
        desc.limit      = limit;
        desc.seg_type   = flags[15:12];
        desc.S          = flags[11];
        desc.DPL        = flags[10:9];
        desc.P          = flags[8];
        desc.D_B        = flags[7];
        desc.G          = flags[6];
        desc.A          = flags[5];
        desc.executable = flags[15];
        desc.expand_down= ~flags[15] & flags[14];
        desc.conforming = flags[15] & flags[14];
        desc.writable   = ~flags[15] & flags[13];
        desc.readable   = flags[15] & flags[13];
        return desc;
    endfunction

    initial begin
        for (int i = 0; i < MEM_SIZE; i++)
            mem[i] = 8'h00;

        if ($value$plusargs("mem=%s", memfile)) begin
            $readmemh(memfile, mem);
            $display("[TB] Loaded memory from %s", memfile);
        end else begin
            $display("[TB] ERROR: No memory file specified (+mem=file.hex)");
            $finish;
        end

        if ($value$plusargs("cycles=%d", max_cycles))
            $display("[TB] Max cycles: %0d", max_cycles);
        if ($value$plusargs("mem_latency=%d", mem_latency))
            $display("[TB] Memory latency: %0d cycles", mem_latency);
        if ($test$plusargs("simple_mem"))
            simple_mem = 1;
        if ($value$plusargs("sdram_first_latency=%d", sdram_first_latency))
            simple_mem = 0;
        if ($value$plusargs("sdram_gap_cycles=%d", sdram_gap_cycles))
            simple_mem = 0;
        if ($value$plusargs("sdram_write_busy=%d", sdram_write_busy))
            simple_mem = 0;
        if (!simple_mem)
            $display("[TB] Memory model: sdram-ish first=%0d gap=%0d write_busy=%0d",
                     sdram_first_latency, sdram_gap_cycles, sdram_write_busy);
        else
            $display("[TB] Memory model: simple");

        #20;
        reset_n = 0;
        #10;

        force dut.CR0 = 32'h8000_0001;
        force dut.CR3 = 32'h0000_0000;
        force dut.EAX = 32'h0;
        force dut.ECX = 32'h0;
        force dut.EDX = 32'h0;
        force dut.EBX = 32'h0;
        force dut.ESP = 32'h0000_FF00;
        force dut.EBP = 32'h0;
        force dut.ESI = 32'h0;
        force dut.EDI = 32'h0;
        force dut.EFLAGS = 32'h0000_0002;
        force dut.CS = 16'h0008;
        force dut.DS = 16'h0010;
        force dut.SS = 16'h0018;
        force dut.ES = 16'h0020;
        force dut.FS = 16'h0028;
        force dut.GS = 16'h0030;
        force dut.EIP = 32'h0;

        begin
            z386_pkg::seg_desc_t cs_desc;
            cs_desc = build_seg_desc(LINEAR_BASE, SEG_LIMIT, DEFAULT_CODE_FLAGS);
            cs_desc.D_B = 1'b1;
            force dut.seg_unit.seg_init_cs = cs_desc;
        end
        force dut.seg_unit.seg_init_ds = build_seg_desc(LINEAR_BASE, SEG_LIMIT, DEFAULT_DATA_FLAGS);
        force dut.seg_unit.seg_init_ss = build_seg_desc(LINEAR_BASE, SEG_LIMIT, DEFAULT_DATA_FLAGS);
        force dut.seg_unit.seg_init_es = build_seg_desc(LINEAR_BASE, SEG_LIMIT, DEFAULT_DATA_FLAGS);
        force dut.seg_unit.seg_init_fs = build_seg_desc(LINEAR_BASE, SEG_LIMIT, DEFAULT_DATA_FLAGS);
        force dut.seg_unit.seg_init_gs = build_seg_desc(LINEAR_BASE, SEG_LIMIT, DEFAULT_DATA_FLAGS);
        force dut.seg_unit.seg_init_idt = build_seg_desc(32'h0, 20'h0FFF, 16'h0);
        force dut.seg_unit.seg_init_gdt = build_seg_desc(32'h0, 20'h0FFF, 16'h0);

`ifdef Z386S_CORE
        begin
            reg [31:0] pf_start_addr;
            reg [5:0] pf_start_ptr;
            pf_start_addr = CODE_PHYS_BASE;
            pf_start_ptr = {2'b0, pf_start_addr[3:0]};
            force dut.prefetch_inst.pf_fetch_addr = pf_start_addr;
            force dut.prefetch_inst.d1_ptr = pf_start_ptr;
            force dut.prefetch_inst.d2_ptr = pf_start_ptr;
            force dut.prefetch_inst.fill_ptr = pf_start_ptr;
        end
`else
        force dut.prefetch_inst.pf_fetch_addr = CODE_PHYS_BASE;
`endif

        #50;
        reset_n = 1;
        release dut.CR0;
        release dut.CR3;
        release dut.EAX;
        release dut.ECX;
        release dut.EDX;
        release dut.EBX;
        release dut.ESP;
        release dut.EBP;
        release dut.ESI;
        release dut.EDI;
        release dut.EFLAGS;
        release dut.CS;
        release dut.DS;
        release dut.SS;
        release dut.ES;
        release dut.FS;
        release dut.GS;
        release dut.EIP;
        release dut.seg_unit.seg_init_cs;
        release dut.seg_unit.seg_init_ds;
        release dut.seg_unit.seg_init_ss;
        release dut.seg_unit.seg_init_es;
        release dut.seg_unit.seg_init_fs;
        release dut.seg_unit.seg_init_gs;
        release dut.seg_unit.seg_init_idt;
        release dut.seg_unit.seg_init_gdt;
`ifdef Z386S_CORE
        release dut.prefetch_inst.pf_fetch_addr;
        release dut.prefetch_inst.d1_ptr;
        release dut.prefetch_inst.d2_ptr;
        release dut.prefetch_inst.fill_ptr;
`else
        release dut.prefetch_inst.pf_fetch_addr;
`endif

        $display("[TB] CPU reset complete, starting execution");
        $display("");
    end

    always @(posedge clk) begin
        if (reset_n) begin
            cycle <= cycle + 1;
            if (instruction_retire)
                instruction_count <= instruction_count + 1;

            if (test_done) begin
                #50;
                $finish;
            end

            if (cycle >= max_cycles) begin
                $display("");
                $display("========================================");
                $display("  TIMEOUT after %0d cycles", max_cycles);
                $display("  Test status: 0x%02X", test_status);
                $display("  Test data: 0x%08X", test_data);
                $display("  Instructions: %0d", instruction_count);
                $display("  CS:EIP: %04X:%08X", dut.CS, dut.EIP);
                $display("========================================");
                $finish;
            end

            if ($test$plusargs("progress") && (cycle % 1000000 == 0))
                $display("Progress: cycle=%0d instr=%0d", cycle, instruction_count);

            if ($test$plusargs("trace_instr") && dut.EIP != prev_trace_eip) begin
`ifdef Z386S_CORE
                $display("INSTR[%0d]: cycle=%0d CS:EIP=%04X:%08X bytes=%06X EAX=%08X EBX=%08X ECX=%08X EDX=%08X ESI=%08X EDI=%08X ESP=%08X EFLAGS=%08X uc=%03X",
                         instruction_count, cycle, dut.CS, dut.EIP, dut.pf_d1_bytes,
                         dut.EAX, dut.EBX, dut.ECX, dut.EDX, dut.ESI, dut.EDI,
                         dut.ESP, dut.EFLAGS, dut.uc_addr);
`else
                $display("INSTR[%0d]: cycle=%0d CS:EIP=%04X:%08X EAX=%08X EBX=%08X ECX=%08X EDX=%08X ESI=%08X EDI=%08X ESP=%08X EFLAGS=%08X",
                         instruction_count, cycle, dut.CS, dut.EIP,
                         dut.EAX, dut.EBX, dut.ECX, dut.EDX, dut.ESI, dut.EDI,
                         dut.ESP, dut.EFLAGS);
`endif
                prev_trace_eip <= dut.EIP;
            end
        end
    end
endmodule
