`timescale 1ns/1ns

//
// Testbench for z386 - Protected Mode Test Runner
// Generic testbench for protected mode tests with configurable segment descriptors
// and paging.
//

/* verilator lint_off SYNCASYNCNET */

module tb_protected_mode;
    // Segment cache array indices (from z386_pkg)
    localparam SEG_ES = 0, SEG_CS = 1, SEG_SS = 2, SEG_DS = 3;
    localparam SEG_FS = 4, SEG_GS = 5, SEG_IDT = 6, SEG_TR = 8, SEG_GDT = 10;
    // Clock and reset
    reg clk = 0;
    always #5 clk <= ~clk;  // 100 MHz clock

    reg reset_n = 0;

    // Test control
    int max_cycles = 10_000_000;
    int cycle = 0;
    bit stop_on_hlt = 1'b1;

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
        .single_step(1'b0), // Continuous execution
        .dbg_CS(),
        .dbg_EIP(),
        .dbg_pe(),
        .dbg_vm()
    );

    // Memory: 512KB for protected mode testing
    localparam MEM_SIZE = 1 << 19;  // 512KB
    reg [7:0] mem [0:MEM_SIZE-1];

    // Instruction counting
    wire instruction_boundary = dut.uc_is_rni && dut.uc_active;
    reg prev_instruction_boundary = 0;
    longint instruction_count = 0;

    // Test result tracking
    reg [7:0] test_status = 8'h00;  // 0x00=running, 0x01=pass, 0xFF=fail
    reg [31:0] test_data = 32'h0;
    reg test_done = 0;

    // Hardware interrupt emulation
    reg [7:0] intr_vector = 8'h20;     // Vector to return on INTA
    reg       intr_request = 0;        // Pending INTR request from signal port
    reg       nmi_request = 0;         // Pending NMI request from signal port
    int       intr_delay = 0;          // Cycles to delay before asserting intr/nmi
    int       signal_delay_cycles = 50;// Configurable cycle delay for signal trigger
    int       signal_delay_instr = 0;  // Optional retired-instruction delay for trigger
    int       intr_instr_remaining = 0;
    int       nmi_instr_remaining = 0;
    int       nmi_pulse_cycles = 1;    // NMI high width in cycles (for deterministic edge delivery)
    int       nmi_hold_count = 0;
    reg       inta_first = 0;          // Track first vs second INTA pair
    reg       inta_resp_pending = 0;
    reg [31:0] inta_resp_data = 32'h0;

    // Configurable memory latency (default 1 cycle, +mem_latency=N for more)
    int mem_latency = 1;
    int rd_wait_count = 0;
    reg [31:0] rd_byte_addr = 32'h0;
    reg [7:0] rd_remaining = 8'd0;
    reg [7:0] rd_index = 8'd0;
    reg rd_io_pending = 1'b0;
    wire rd_busy = (rd_wait_count != 0) || (rd_remaining != 0) || inta_resp_pending;

    // Memory behavior with configurable latency (ready/valid protocol)
    // Note: din is held stable (not cleared) to allow paging unit to sample it
    // when pg_mem_ready is asserted (which has 1-cycle delay from bus ready)
    always @(posedge clk) begin
        ready <= !rd_busy;
        resp_valid <= 1'b0;
        // Don't clear din - hold it stable for page walker timing

        // Handle pending read with latency countdown, then return one DWORD per cycle.
        if (inta_resp_pending) begin
            resp_valid <= 1'b1;
            din <= inta_resp_data;
            inta_resp_pending <= 1'b0;
        end else if (rd_wait_count != 0) begin
            rd_wait_count <= rd_wait_count - 1;
        end else if (rd_remaining != 8'd0) begin
            reg [31:0] byte_addr;
            byte_addr = rd_byte_addr + {22'd0, rd_index, 2'b00};
            if (byte_addr >= MEM_SIZE)
                byte_addr = byte_addr & (MEM_SIZE - 1);

            resp_valid <= 1'b1;
            if (rd_io_pending) begin
                din <= 32'hFFFFFFFF;
            end else begin
                din <= {mem[byte_addr+3], mem[byte_addr+2],
                        mem[byte_addr+1], mem[byte_addr+0]};

                if ($test$plusargs("trace_mem"))
                    $display("MEM RESP @%08x = %08x",
                             byte_addr,
                             {mem[byte_addr+3], mem[byte_addr+2],
                              mem[byte_addr+1], mem[byte_addr+0]});
            end

            rd_index <= rd_index + 8'd1;
            rd_remaining <= rd_remaining - 8'd1;
        end

        if (valid && ready && !rd_busy) begin
            if (inta) begin
                // INTA bus cycle handling
                ready <= 1'b0;
                inta_resp_pending <= 1'b1;
                if (!inta_first) begin
                    inta_first <= 1'b1;
                    inta_resp_data <= 32'h0;
                    if ($test$plusargs("trace_io"))
                        $display("INTA cycle 1 (dummy)");
                end else begin
                    inta_first <= 1'b0;
                    inta_resp_data <= {24'h0, intr_vector};
                    intr <= 1'b0;
                    if ($test$plusargs("trace_io"))
                        $display("INTA cycle 2: vector=0x%02X", intr_vector);
                end
            end else if (!write) begin
                // Read
                reg [7:0] burst_len;
                burst_len = io ? 8'd1 : burstcount;
                rd_byte_addr <= {addr, 2'b00};
                rd_index <= (mem_latency <= 1) ? 8'd1 : 8'd0;
                rd_wait_count <= (mem_latency <= 1) ? 0 : (mem_latency - 1);
                rd_io_pending <= io;

                if (!io) begin
                    reg [31:0] byte_addr;
                    byte_addr = {addr, 2'b00};

                    if (byte_addr >= MEM_SIZE)
                        byte_addr = byte_addr & (MEM_SIZE - 1);

                    if ($test$plusargs("trace_mem"))
                        $display("MEM RD @%08x count=%0d first=%08x", byte_addr, burstcount,
                                 {mem[byte_addr+3], mem[byte_addr+2],
                                  mem[byte_addr+1], mem[byte_addr+0]});
                    if (mem_latency <= 1) begin
                        resp_valid <= 1'b1;
                        din <= {mem[byte_addr+3], mem[byte_addr+2],
                                mem[byte_addr+1], mem[byte_addr+0]};
                    end
                end else if (mem_latency <= 1) begin
                    resp_valid <= 1'b1;
                    din <= 32'hFFFFFFFF;
                end
                rd_remaining <= (mem_latency <= 1 && burst_len > 8'd1) ?
                                (burst_len - 8'd1) : burst_len;
                ready <= (mem_latency <= 1 && burst_len <= 8'd1);
            end else begin
                // Write
                ready <= 1'b1;
                if (io) begin
                // I/O writes - check result ports
                reg [15:0] port;
                port = {addr[15:2], 2'b00};

                // Status port (0xE0) - test result
                if (port == 16'h00E0) begin
                    test_status <= dout[7:0];
                    if (dout[7:0] == 8'h01) begin
                        $display("");
                        $display("========================================");
                        $display("  TEST PASSED!");
                        $display("  Total cycles: %0d", cycle);
                        $display("  Total instructions: %0d", instruction_count);
                        $display("========================================");
                        test_done <= 1;
                    end else if (dout[7:0] == 8'hFF) begin
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

                // Data port (0xE4) - debug/verification data
                if (port == 16'h00E4) begin
                    test_data <= dout;
                    if ($test$plusargs("trace_io"))
                        $display("TEST DATA: 0x%08X", dout);
                end

                // Signal port (0xE8) - trigger hardware interrupts
                // 1 = assert INTR after short delay
                // 2 = pulse NMI after short delay
                // 3 = assert INTR while IF=0 (masked test)
                if (port == 16'h00E8) begin
                    if (dout[7:0] == 8'h01 || dout[7:0] == 8'h03) begin
                        if (signal_delay_instr > 0) begin
                            intr_instr_remaining <= signal_delay_instr;
                            intr_request <= 1'b1;
                        end else begin
                            intr_request <= 1'b1;
                            intr_delay <= signal_delay_cycles;
                        end
                        if ($test$plusargs("trace_io"))
                            $display("SIGNAL: INTR requested (mode=%0d, cyc=%0d, instr=%0d)",
                                     dout[7:0], signal_delay_cycles, signal_delay_instr);
                    end
                    if (dout[7:0] == 8'h02) begin
                        if (signal_delay_instr > 0) begin
                            nmi_instr_remaining <= signal_delay_instr;
                            nmi_request <= 1'b1;
                        end else begin
                            nmi_request <= 1'b1;
                            intr_delay <= signal_delay_cycles;
                        end
                        if ($test$plusargs("trace_io"))
                            $display("SIGNAL: NMI requested (cyc=%0d, instr=%0d)",
                                     signal_delay_cycles, signal_delay_instr);
                    end
                end

                // Signal delay (0xEC) - low 16 bits are cycle delay
                if (port == 16'h00EC) begin
                    signal_delay_cycles <= dout[15:0];
                    if ($test$plusargs("trace_io"))
                        $display("SIGNAL CFG: cycle delay=%0d", dout[15:0]);
                end

                // Signal instruction delay (0xF0) - low 16 bits are retired instruction delay
                // 0 means use cycle delay mode.
                if (port == 16'h00F0) begin
                    signal_delay_instr <= dout[15:0];
                    if ($test$plusargs("trace_io"))
                        $display("SIGNAL CFG: instruction delay=%0d", dout[15:0]);
                end

                // Signal vector (0xF4) - low 8 bits are INTR vector
                if (port == 16'h00F4) begin
                    intr_vector <= dout[7:0];
                    if ($test$plusargs("trace_io"))
                        $display("SIGNAL CFG: INTR vector=0x%02X", dout[7:0]);
                end

                // NMI pulse width (0xF8) - low 16 bits = cycles to keep nmi asserted
                if (port == 16'h00F8) begin
                    if (dout[15:0] == 0)
                        nmi_pulse_cycles <= 1;
                    else
                        nmi_pulse_cycles <= dout[15:0];
                    if ($test$plusargs("trace_io"))
                        $display("SIGNAL CFG: NMI pulse cycles=%0d",
                                 (dout[15:0] == 0) ? 1 : dout[15:0]);
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
                    $display("MEM WR @%08x be=%b data=%08x uc=%03h mem_wdata=%08h src=%02h",
                             byte_addr, be, dout, dut.uc_addr, dut.mem_wdata, dut.uc_source);
            end
        end
    end
    end

    // Configuration from plusargs
    string memfile;
    int eip_arg;
    int cr0_arg, cr3_arg;
    int code_phys_base;  // Physical address where code is loaded (for prefetch)
    int start_protected; // 1: force protected-mode entry state, 0: start in real mode
    int d_init;          // Initial default operand size flag

    // Initial visible segment selectors
    int init_cs, init_ds, init_ss, init_es, init_fs, init_gs;

    // Segment descriptor cache values from plusargs
    int cs_base, cs_limit, cs_flags;
    int ds_base, ds_limit, ds_flags;
    int ss_base, ss_limit, ss_flags;
    int es_base, es_limit, es_flags;
    int fs_base, fs_limit, fs_flags;
    int gs_base, gs_limit, gs_flags;

    // Build seg_desc_t from flags
    // flags[15:12] = type, flags[11] = S, flags[10:9] = DPL, flags[8] = P,
    // flags[7] = D_B, flags[6] = G, flags[5] = A
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

    // Default protected mode descriptor flags:
    // type=0010 (data RW), S=1, DPL=0, P=1, D_B=1 (32-bit), G=1 (4K), A=1
    localparam DEFAULT_DATA_FLAGS = 16'h21E0;  // Data RW, S=1, DPL=0, P=1, D_B=1, G=1, A=1
    localparam DEFAULT_CODE_FLAGS = 16'hA1E0;  // Code RX, S=1, DPL=0, P=1, D_B=1, G=1, A=1
    // Real-mode: D_B=0 (16-bit), G=0 (byte granularity), P=1, A=0
    // flags: type[15:12] S[11] DPL[10:9] P[8] D_B[7] G[6] A[5]
    localparam RM_DATA_FLAGS = 16'h2900;       // type=0010(RW), S=1, DPL=0, P=1, D_B=0, G=0
    localparam RM_CODE_FLAGS = 16'hA900;       // type=1010(RX), S=1, DPL=0, P=1, D_B=0, G=0

    initial begin
        // Initialize memory to 0
        for (int i = 0; i < MEM_SIZE; i++)
            mem[i] = 8'h00;

        // Get memory file (hex format)
        if ($value$plusargs("mem=%s", memfile)) begin
            $readmemh(memfile, mem);
            $display("[TB] Loaded memory from %s", memfile);
        end else begin
            $display("[TB] ERROR: No memory file specified (+mem=file.hex)");
            $finish;
        end

        // Get max cycles
        if ($value$plusargs("cycles=%d", max_cycles))
            $display("[TB] Max cycles: %0d", max_cycles);
        if ($value$plusargs("mem_latency=%d", mem_latency))
            $display("[TB] Memory latency: %0d cycles", mem_latency);
        if ($test$plusargs("continue_on_hlt"))
            stop_on_hlt = 1'b0;

        // Get initial EIP (default 0)
        if (!$value$plusargs("eip=%d", eip_arg)) eip_arg = 0;

        // Get CR0/CR3
        if (!$value$plusargs("cr0=%d", cr0_arg)) cr0_arg = 32'h80000001;  // PE=1, PG=1
        if (!$value$plusargs("cr3=%d", cr3_arg)) cr3_arg = 32'h00000000;  // Page dir at 0

        // Get physical address where code is loaded (prefetch bypasses paging)
        if (!$value$plusargs("code_phys_base=%d", code_phys_base)) code_phys_base = 32'h00010000;

        // Start mode (default protected for backward compatibility)
        if (!$value$plusargs("start_protected=%d", start_protected)) start_protected = 1;

        // Get segment descriptor parameters
        // CS
        if (!$value$plusargs("cs_base=%d", cs_base)) cs_base = 32'h10000000;
        if (!$value$plusargs("cs_limit=%d", cs_limit)) cs_limit = 20'hFFFFF;
        if (!$value$plusargs("cs_flags=%d", cs_flags)) cs_flags = DEFAULT_CODE_FLAGS;
        // DS
        if (!$value$plusargs("ds_base=%d", ds_base)) ds_base = 32'h20000000;
        if (!$value$plusargs("ds_limit=%d", ds_limit)) ds_limit = 20'hFFFFF;
        if (!$value$plusargs("ds_flags=%d", ds_flags)) ds_flags = DEFAULT_DATA_FLAGS;
        // SS
        if (!$value$plusargs("ss_base=%d", ss_base)) ss_base = 32'h30000000;
        if (!$value$plusargs("ss_limit=%d", ss_limit)) ss_limit = 20'hFFFFF;
        if (!$value$plusargs("ss_flags=%d", ss_flags)) ss_flags = DEFAULT_DATA_FLAGS;
        // ES
        if (!$value$plusargs("es_base=%d", es_base)) es_base = 32'h00000000;
        if (!$value$plusargs("es_limit=%d", es_limit)) es_limit = 20'hFFFFF;
        if (!$value$plusargs("es_flags=%d", es_flags)) es_flags = DEFAULT_DATA_FLAGS;
        // FS
        if (!$value$plusargs("fs_base=%d", fs_base)) fs_base = 32'h00000000;
        if (!$value$plusargs("fs_limit=%d", fs_limit)) fs_limit = 20'hFFFFF;
        if (!$value$plusargs("fs_flags=%d", fs_flags)) fs_flags = DEFAULT_DATA_FLAGS;
        // GS
        if (!$value$plusargs("gs_base=%d", gs_base)) gs_base = 32'h00000000;
        if (!$value$plusargs("gs_limit=%d", gs_limit)) gs_limit = 20'hFFFFF;
        if (!$value$plusargs("gs_flags=%d", gs_flags)) gs_flags = DEFAULT_DATA_FLAGS;

        // Real-mode: override segment descriptors to 16-bit mode
        if (!start_protected) begin
            if (!$value$plusargs("cs_base=%d", cs_base)) cs_base = code_phys_base;
            if (!$value$plusargs("ds_base=%d", ds_base)) ds_base = 32'h00000000;
            if (!$value$plusargs("ss_base=%d", ss_base)) ss_base = 32'h00000000;
            if (!$value$plusargs("cs_limit=%d", cs_limit)) cs_limit = 20'h0FFFF;
            if (!$value$plusargs("ds_limit=%d", ds_limit)) ds_limit = 20'h0FFFF;
            if (!$value$plusargs("ss_limit=%d", ss_limit)) ss_limit = 20'h0FFFF;
            if (!$value$plusargs("es_limit=%d", es_limit)) es_limit = 20'h0FFFF;
            if (!$value$plusargs("fs_limit=%d", fs_limit)) fs_limit = 20'h0FFFF;
            if (!$value$plusargs("gs_limit=%d", gs_limit)) gs_limit = 20'h0FFFF;
            if (!$value$plusargs("cs_flags=%d", cs_flags)) cs_flags = RM_CODE_FLAGS;
            if (!$value$plusargs("ds_flags=%d", ds_flags)) ds_flags = RM_DATA_FLAGS;
            if (!$value$plusargs("ss_flags=%d", ss_flags)) ss_flags = RM_DATA_FLAGS;
            if (!$value$plusargs("es_flags=%d", es_flags)) es_flags = RM_DATA_FLAGS;
            if (!$value$plusargs("fs_flags=%d", fs_flags)) fs_flags = RM_DATA_FLAGS;
            if (!$value$plusargs("gs_flags=%d", gs_flags)) gs_flags = RM_DATA_FLAGS;
        end

        // Initial segment selectors:
        // - protected start defaults to canonical GDT selectors
        // - real-mode start defaults to CS derived from code physical address
        if (!$value$plusargs("init_cs=%d", init_cs))
            init_cs = start_protected ? 16'h0008 : ((code_phys_base >> 4) & 16'hFFFF);
        if (!$value$plusargs("init_ds=%d", init_ds))
            init_ds = start_protected ? 16'h0010 : 16'h0000;
        if (!$value$plusargs("init_ss=%d", init_ss))
            init_ss = start_protected ? 16'h0018 : 16'h0000;
        if (!$value$plusargs("init_es=%d", init_es))
            init_es = start_protected ? 16'h0020 : 16'h0000;
        if (!$value$plusargs("init_fs=%d", init_fs))
            init_fs = start_protected ? 16'h0028 : 16'h0000;
        if (!$value$plusargs("init_gs=%d", init_gs))
            init_gs = start_protected ? 16'h0030 : 16'h0000;

        // Initial default operand-size mode
        if (!$value$plusargs("d_init=%d", d_init))
            d_init = start_protected ? 1 : 0;

        $display("[TB] Configuration:");
        $display("[TB]   mode=%s", start_protected ? "protected-start" : "real-start");
        $display("[TB]   EIP=0x%08X CR0=0x%08X CR3=0x%08X", eip_arg, cr0_arg, cr3_arg);
        $display("[TB]   selectors: CS=%04X DS=%04X SS=%04X ES=%04X FS=%04X GS=%04X",
                 init_cs[15:0], init_ds[15:0], init_ss[15:0], init_es[15:0], init_fs[15:0], init_gs[15:0]);
        $display("[TB]   CS: base=0x%08X limit=0x%05X flags=0x%04X", cs_base, cs_limit, cs_flags);
        $display("[TB]   DS: base=0x%08X limit=0x%05X flags=0x%04X", ds_base, ds_limit, ds_flags);
        $display("[TB]   SS: base=0x%08X limit=0x%05X flags=0x%04X", ss_base, ss_limit, ss_flags);

        // Reset sequence
        #20;
        reset_n = 0;
        #10;

        #50;
        reset_n = 1;
        #1;

        // Initialize CPU state immediately after reset release, before the next
        // clock edge.  The 17.z386 seg_cache is an unpacked struct array, and
        // the simulator currently trips an internal force-pass bug if this
        // testbench uses hierarchical force.
        dut.CS = init_cs[15:0];
        dut.DS = init_ds[15:0];
        dut.SS = init_ss[15:0];
        dut.ES = init_es[15:0];
        dut.FS = init_fs[15:0];
        dut.GS = init_gs[15:0];

        dut.EIP = eip_arg;

        dut.seg_cache[SEG_CS] = build_seg_desc(cs_base, cs_limit[19:0], cs_flags);
        dut.seg_cache[SEG_DS] = build_seg_desc(ds_base, ds_limit[19:0], ds_flags);
        dut.seg_cache[SEG_SS] = build_seg_desc(ss_base, ss_limit[19:0], ss_flags);
        dut.seg_cache[SEG_ES] = build_seg_desc(es_base, es_limit[19:0], es_flags);
        dut.seg_cache[SEG_FS] = build_seg_desc(fs_base, fs_limit[19:0], fs_flags);
        dut.seg_cache[SEG_GS] = build_seg_desc(gs_base, gs_limit[19:0], gs_flags);
        dut.seg_cache[SEG_CS].D_B = d_init[0];

        // Set prefetch to physical address where code resides
        // (prefetch currently bypasses paging, so we use physical address directly)
        dut.prefetch_inst.pf_fetch_addr = code_phys_base + eip_arg;

        // General registers = 0
        dut.EAX = 32'h0;
        dut.ECX = 32'h0;
        dut.EDX = 32'h0;
        dut.EBX = 32'h0;
        dut.ESP = 32'h0000FF00;  // Stack pointer in SS
        dut.EBP = 32'h0;
        dut.ESI = 32'h0;
        dut.EDI = 32'h0;

        // Flags and control
        dut.EFLAGS = 32'h00000002;
        dut.CR0 = cr0_arg;
        dut.CR3 = cr3_arg;

        $display("[TB] CPU reset complete, starting execution");
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

            // Test completed
            if (test_done) begin
                #50;
                $finish;
            end

            // Timeout
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

            // Hardware interrupt signal generation
            if (intr_delay > 0) begin
                intr_delay <= intr_delay - 1;
                if (intr_delay == 1) begin
                    if (intr_request) begin
                        intr <= 1'b1;
                        intr_request <= 1'b0;
                        if ($test$plusargs("trace_io"))
                            $display("[TB] INTR asserted at cycle %0d", cycle);
                    end
                    if (nmi_request) begin
                        nmi <= 1'b1;
                        nmi_hold_count <= nmi_pulse_cycles;
                        nmi_request <= 1'b0;
                        if ($test$plusargs("trace_io"))
                            $display("[TB] NMI asserted at cycle %0d", cycle);
                    end
                end
            end

            // Deterministic trigger mode: count retired instructions.
            // This is useful for shadow-window tests (STI/MOV SS).
            if (instruction_boundary && !prev_instruction_boundary) begin
                if (intr_request && intr_instr_remaining > 0) begin
                    intr_instr_remaining <= intr_instr_remaining - 1;
                    if (intr_instr_remaining == 1) begin
                        intr <= 1'b1;
                        intr_request <= 1'b0;
                        if ($test$plusargs("trace_io"))
                            $display("[TB] INTR asserted at retired-instr=%0d (cycle %0d)",
                                     instruction_count + 1, cycle);
                    end
                end
                if (nmi_request && nmi_instr_remaining > 0) begin
                    nmi_instr_remaining <= nmi_instr_remaining - 1;
                    if (nmi_instr_remaining == 1) begin
                        nmi <= 1'b1;
                        nmi_hold_count <= nmi_pulse_cycles;
                        nmi_request <= 1'b0;
                        if ($test$plusargs("trace_io"))
                            $display("[TB] NMI asserted at retired-instr=%0d (cycle %0d)",
                                     instruction_count + 1, cycle);
                    end
                end
            end
            if (nmi_hold_count > 0) begin
                nmi_hold_count <= nmi_hold_count - 1;
                if (nmi_hold_count == 1)
                    nmi <= 1'b0;
            end

            // Check for HLT (can be disabled for HLT-wakeup interrupt tests)
            if (stop_on_hlt && dut.i.opcode == 8'hF4 && instruction_boundary && !prev_instruction_boundary) begin
                $display("");
                $display("========================================");
                $display("  HLT EXECUTED");
                $display("  Test status: 0x%02X", test_status);
                $display("  Test data: 0x%08X", test_data);
                $display("  Cycle: %0d", cycle);
                $display("  CS:EIP: %04X:%08X", dut.CS, dut.EIP);
                $display("========================================");
                #100;
                $finish;
            end

            // Optional progress
            if ($test$plusargs("progress") && (cycle % 1000000 == 0))
                $display("Progress: cycle=%0d instr=%0d", cycle, instruction_count);

            // Trace instructions
            if ($test$plusargs("trace_instr") && instruction_boundary && !prev_instruction_boundary)
                $display("INSTR[%0d]: CS:EIP=%04X:%08X IR=%02X EAX=%08X",
                         instruction_count, dut.CS, dut.EIP, dut.i.opcode, dut.EAX);

            // Trace microcode execution
            if ($test$plusargs("trace_ucode") && !dut.stall)
                $display("UCODE pc=%03h dest=%02h src=%02h bus=%02h aluop=%02h SIGMA=%08h ESP=%08h IND=%08h CS=%04h wdata=%08h",
                         dut.uc_addr, dut.uc_dest, dut.uc_source, dut.uc_buscode, dut.uc_aluop,
                         dut.SIGMA, dut.ESP, dut.IND, dut.CS, dut.mem_wdata);

            // Trace paging
            if ($test$plusargs("trace_paging") && dut.mem_req_upcoming)
                $display("PAGING: linear=%08X servicing=%0d", dut.mem_linear_addr, dut.mem_servicing);
        end
    end

    // Waveform dump
    string tracefile;
    initial begin
        if ($test$plusargs("trace")) begin
            if (!$value$plusargs("tracefile=%s", tracefile))
                tracefile = "trace.vcd";
            $display("[TB] Waveform trace: %s", tracefile);
            $dumpfile(tracefile);
            $dumpvars(0, tb_protected_mode);
        end
    end

endmodule
