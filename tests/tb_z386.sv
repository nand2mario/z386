`timescale 1ns/1ns

//
// Testbench for z386 - Single Instruction Test Runner
// Based on z8086/tests/tb_z8086.sv but adapted for 32-bit
//

module tb_z386;
    // Segment cache array indices (from z386_pkg)
    localparam SEG_ES = 0, SEG_CS = 1, SEG_SS = 2, SEG_DS = 3;
    localparam SEG_FS = 4, SEG_GS = 5, SEG_IDT = 6, SEG_TR = 8, SEG_GDT = 10;

    // Clock and reset
    reg clk = 0;
    longint start_time;
    always #5 clk <= ~clk;  // 100 MHz clock

    reg reset_n = 0;

    // Test control
    int max_cycles;
    bit stop_after_first = 1'b1;  // Stop when first instruction completes
    bit stop_on_halt = 1'b0;      // Stop when CPU halts (for single-step tests with HLT)

    // CPU bus interface (32-bit, ready/valid)
    wire [31:2] addr;       // 4-byte aligned address
    wire [3:0]  be;         // Byte enables
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
        .cache_lookup(),
        .cache_lookup_addr(),
        .cache_lookup_write(),
        .cache_lookup_cancel(),
        .cache_lookup_ready(1'b1),
        .single_step(1'b1), // Halt after each instruction for single-step tests
        .dbg_CS(),
        .dbg_EIP(),
        .dbg_pe(),
        .dbg_vm()
    );

    // Access debug signals
    wire dbg_first_done = dut.dbg_first_done;
    wire cpu_halted = dut.halted;
    wire instruction_boundary = dut.uc_is_rni && dut.uc_active;
    integer instruction_count = 0;

    // Fault detection: at uc_addr=875, ALU computes SIGMA+9 = fault vector
    reg fault_detected = 0;
    integer fault_vector = -1;
    always @(posedge clk) begin
        if (reset_n && !dut.stall && dut.uc_addr == 12'h875 && !fault_detected) begin
            fault_vector = dut.alu_result;  // SIGMA + 9 = vector number
            fault_detected = 1;
            $display("RESULT FAULT: vector=%0d", fault_vector);
        end
    end

    // 4GB memory model (sparse - only allocate what's used)
    localparam MEM_SIZE = 1 << 24;  // 16MB for testing (can expand)
    reg [7:0] mem [0:MEM_SIZE-1];

    // Statistics and control
    integer fetch_count = 0;
    reg hlt_fetched = 0;
    reg pass = 0;
    integer stop_reads = 0;

    // Optional checks (set via plusargs)
    int ram_addrs [0:1023];
    int ram_cnt = 0;

    // Memory behavior with 1-cycle ready/valid latency
    always @(posedge clk) begin
        ready <= 1'b0;
        resp_valid <= 1'b0;
        din <= 32'h00000000;

        if (valid && !ready) begin
            ready <= 1'b1;  // Accept the request

            if (!write) begin
                // Read
                resp_valid <= 1'b1;
                if (io) begin
                    din <= 32'hFFFFFFFF;  // I/O reads return 0xFF
                end else begin
                    reg [31:0] byte_addr;
                    byte_addr = {addr, 2'b00};

                    din <= {mem[byte_addr+3], mem[byte_addr+2],
                            mem[byte_addr+1], mem[byte_addr+0]};

                    $display("TB RESP addr=%08x be=%b din=%08x [%02x %02x %02x %02x]",
                             byte_addr, be,
                             {mem[byte_addr+3], mem[byte_addr+2], mem[byte_addr+1], mem[byte_addr+0]},
                             mem[byte_addr+3], mem[byte_addr+2], mem[byte_addr+1], mem[byte_addr+0]);
                end
                fetch_count <= fetch_count + 1;
            end else begin
                // Write
                if (!io) begin
                    reg [31:0] byte_addr;
                    byte_addr = {addr, 2'b00};

                    if (be[0]) mem[byte_addr+0] <= dout[7:0];
                    if (be[1]) mem[byte_addr+1] <= dout[15:8];
                    if (be[2]) mem[byte_addr+2] <= dout[23:16];
                    if (be[3]) mem[byte_addr+3] <= dout[31:24];

                    $display("TB WRITE addr=%08x be=%b dout=%08x", byte_addr, be, dout);
                end else begin
                    if (addr == 30'h0 && dout == 32'h12345678)
                        pass <= 1'b1;
                end
            end
        end
    end

    // Build seg_desc_t from flags (same as tb_protected_mode.sv)
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

    // Extract flags back from seg_desc_t (for readout)
    function automatic [15:0] extract_seg_flags(input z386_pkg::seg_desc_t desc);
        extract_seg_flags = {desc.seg_type, desc.S, desc.DPL, desc.P, desc.D_B, desc.G, desc.A, 5'b0};
    endfunction

    // Load memory from file and initialize registers via plusargs
    string memfile;
    longint cs_arg, ds_arg, es_arg, ss_arg, fs_arg, gs_arg;
    longint ip_arg, d_arg;
    longint eax_arg, ecx_arg, edx_arg, ebx_arg;
    longint esp_arg, ebp_arg, esi_arg, edi_arg;
    longint eflags_arg;
    longint cr0_arg, cr3_arg;

    // Segment cache plusargs (for protected mode tests)
    longint cs_base, cs_limit, cs_flags;
    longint ds_base, ds_limit, ds_flags;
    longint es_base, es_limit, es_flags;
    longint ss_base, ss_limit, ss_flags;
    longint fs_base, fs_limit, fs_flags;
    longint gs_base, gs_limit, gs_flags;

    // Descriptor table register plusargs
    longint gdt_base_arg, gdt_limit_arg;
    longint idt_base_arg, idt_limit_arg;

    // Task register plusargs
    longint tr_sel_arg, tr_base_arg, tr_limit_arg, tr_flags_arg;

    initial begin
        // Get memory file
        if ($value$plusargs("mem=%s", memfile)) begin
            $readmemh(memfile, mem);
            $display("Loaded memory from %s", memfile);
        end

        // Get initial register values
        if (!$value$plusargs("cr0=%d", cr0_arg)) cr0_arg = 32'h0000_0000;
        if (!$value$plusargs("cr3=%d", cr3_arg)) cr3_arg = 32'h0000_0000;
        if (!$value$plusargs("eax=%d", eax_arg)) eax_arg = 0;
        if (!$value$plusargs("ecx=%d", ecx_arg)) ecx_arg = 0;
        if (!$value$plusargs("edx=%d", edx_arg)) edx_arg = 0;
        if (!$value$plusargs("ebx=%d", ebx_arg)) ebx_arg = 0;
        if (!$value$plusargs("esp=%d", esp_arg)) esp_arg = 0;
        if (!$value$plusargs("ebp=%d", ebp_arg)) ebp_arg = 0;
        if (!$value$plusargs("esi=%d", esi_arg)) esi_arg = 0;
        if (!$value$plusargs("edi=%d", edi_arg)) edi_arg = 0;
        if (!$value$plusargs("eflags=%d", eflags_arg)) eflags_arg = 32'h00000002;

        if (!$value$plusargs("cs=%d", cs_arg)) cs_arg = 32'h0000_F000;
        if (!$value$plusargs("ds=%d", ds_arg)) ds_arg = 0;
        if (!$value$plusargs("es=%d", es_arg)) es_arg = 0;
        if (!$value$plusargs("ss=%d", ss_arg)) ss_arg = 0;
        if (!$value$plusargs("fs=%d", fs_arg)) fs_arg = 0;
        if (!$value$plusargs("gs=%d", gs_arg)) gs_arg = 0;
        if (!$value$plusargs("ip=%d", ip_arg)) ip_arg = 0;
        if (!$value$plusargs("d=%d", d_arg)) d_arg = 1;

        if (!$value$plusargs("cycles=%d", max_cycles)) max_cycles = 30000;
        if (!$value$plusargs("reads=%d", stop_reads)) stop_reads = 0;

        // Segment cache plusargs (only used when CR0.PE=1)
        if (!$value$plusargs("cs_base=%d", cs_base)) cs_base = 0;
        if (!$value$plusargs("cs_limit=%d", cs_limit)) cs_limit = 20'hFFFFF;
        if (!$value$plusargs("cs_flags=%d", cs_flags)) cs_flags = 16'hA1E0;
        if (!$value$plusargs("ds_base=%d", ds_base)) ds_base = 0;
        if (!$value$plusargs("ds_limit=%d", ds_limit)) ds_limit = 20'hFFFFF;
        if (!$value$plusargs("ds_flags=%d", ds_flags)) ds_flags = 16'h21E0;
        if (!$value$plusargs("es_base=%d", es_base)) es_base = 0;
        if (!$value$plusargs("es_limit=%d", es_limit)) es_limit = 20'hFFFFF;
        if (!$value$plusargs("es_flags=%d", es_flags)) es_flags = 16'h21E0;
        if (!$value$plusargs("ss_base=%d", ss_base)) ss_base = 0;
        if (!$value$plusargs("ss_limit=%d", ss_limit)) ss_limit = 20'hFFFFF;
        if (!$value$plusargs("ss_flags=%d", ss_flags)) ss_flags = 16'h21E0;
        if (!$value$plusargs("fs_base=%d", fs_base)) fs_base = 0;
        if (!$value$plusargs("fs_limit=%d", fs_limit)) fs_limit = 20'hFFFFF;
        if (!$value$plusargs("fs_flags=%d", fs_flags)) fs_flags = 16'h21E0;
        if (!$value$plusargs("gs_base=%d", gs_base)) gs_base = 0;
        if (!$value$plusargs("gs_limit=%d", gs_limit)) gs_limit = 20'hFFFFF;
        if (!$value$plusargs("gs_flags=%d", gs_flags)) gs_flags = 16'h21E0;

        // Descriptor table registers
        if (!$value$plusargs("gdt_base=%d", gdt_base_arg)) gdt_base_arg = 0;
        if (!$value$plusargs("gdt_limit=%d", gdt_limit_arg)) gdt_limit_arg = 16'h03FF;
        if (!$value$plusargs("idt_base=%d", idt_base_arg)) idt_base_arg = 0;
        if (!$value$plusargs("idt_limit=%d", idt_limit_arg)) idt_limit_arg = 16'h03FF;

        // Task register
        if (!$value$plusargs("tr_sel=%d", tr_sel_arg)) tr_sel_arg = 0;
        if (!$value$plusargs("tr_base=%d", tr_base_arg)) tr_base_arg = 0;
        if (!$value$plusargs("tr_limit=%d", tr_limit_arg)) tr_limit_arg = 0;
        if (!$value$plusargs("tr_flags=%d", tr_flags_arg)) tr_flags_arg = 0;

        // Check for stop_on_halt mode
        if ($test$plusargs("stop_on_halt")) begin
            stop_on_halt = 1'b1;
            stop_after_first = 1'b0;
        end

        // Collect RAM check addresses
        ram_cnt = 0;
        for (int i = 0; i < 1024; i++) begin
            string argname;
            int addr_val;
            argname = $sformatf("ram%0d", i);
            if ($value$plusargs({argname, "=%d"}, addr_val)) begin
                ram_addrs[ram_cnt] = addr_val;
                ram_cnt++;
            end else
                break;
        end

        // Reset sequence
        #20;
        reset_n = 0;
        #10;

        // Force initial register values
        force dut.CR0 = cr0_arg;
        force dut.CR3 = cr3_arg;
        force dut.EAX = eax_arg;
        force dut.ECX = ecx_arg;
        force dut.EDX = edx_arg;
        force dut.EBX = ebx_arg;
        force dut.ESP = esp_arg;
        force dut.EBP = ebp_arg;
        force dut.ESI = esi_arg;
        force dut.EDI = edi_arg;
        force dut.EFLAGS = eflags_arg;
        force dut.CS = cs_arg[15:0];
        force dut.DS = ds_arg[15:0];
        force dut.ES = es_arg[15:0];
        force dut.SS = ss_arg[15:0];
        force dut.FS = fs_arg[15:0];
        force dut.GS = gs_arg[15:0];
        force dut.EIP = ip_arg;

        // Protected mode: force segment caches, GDTR, IDTR, D from CS.D_B
        if (cr0_arg[0]) begin
            force dut.seg_unit.seg_init_cs = build_seg_desc(cs_base, cs_limit[19:0], cs_flags[15:0]);
            force dut.seg_unit.seg_init_ds = build_seg_desc(ds_base, ds_limit[19:0], ds_flags[15:0]);
            force dut.seg_unit.seg_init_es = build_seg_desc(es_base, es_limit[19:0], es_flags[15:0]);
            force dut.seg_unit.seg_init_ss = build_seg_desc(ss_base, ss_limit[19:0], ss_flags[15:0]);
            force dut.seg_unit.seg_init_fs = build_seg_desc(fs_base, fs_limit[19:0], fs_flags[15:0]);
            force dut.seg_unit.seg_init_gs = build_seg_desc(gs_base, gs_limit[19:0], gs_flags[15:0]);
            force dut.seg_unit.seg_init_gdt = build_seg_desc(gdt_base_arg, gdt_limit_arg[19:0], 16'h0);
            force dut.seg_unit.seg_init_idt = build_seg_desc(idt_base_arg, idt_limit_arg[19:0], 16'h0);
            if (tr_sel_arg != 0) begin
                force dut.TR = tr_sel_arg[15:0];
                force dut.seg_unit.seg_init_tr = build_seg_desc(tr_base_arg, tr_limit_arg[19:0], tr_flags_arg[15:0]);
            end
            force dut.prefetch_inst.pf_fetch_addr = cs_base + ip_arg;
        end else begin
            // Real mode: initialize segment caches from selector values
            begin
                z386_pkg::seg_desc_t cs_desc;
                cs_desc = z386_pkg::seg_desc_real_mode_code(cs_arg[15:0]);
                cs_desc.D_B = d_arg[0];
                force dut.seg_unit.seg_init_cs = cs_desc;
            end
            force dut.seg_unit.seg_init_ds = z386_pkg::seg_desc_real_mode(ds_arg[15:0]);
            force dut.seg_unit.seg_init_es = z386_pkg::seg_desc_real_mode(es_arg[15:0]);
            force dut.seg_unit.seg_init_ss = z386_pkg::seg_desc_real_mode(ss_arg[15:0]);
            force dut.seg_unit.seg_init_fs = z386_pkg::seg_desc_real_mode(fs_arg[15:0]);
            force dut.seg_unit.seg_init_gs = z386_pkg::seg_desc_real_mode(gs_arg[15:0]);
            // In real mode: physical address = (CS << 4) + IP
            force dut.prefetch_inst.pf_fetch_addr = (cs_arg << 4) + ip_arg;
        end

        // Release after a few cycles
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
        release dut.ES;
        release dut.SS;
        release dut.FS;
        release dut.GS;
        release dut.EIP;
        release dut.prefetch_inst.pf_fetch_addr;
        // Release init value forces — reset block has already loaded them into seg_cache
        release dut.seg_unit.seg_init_cs;
        release dut.seg_unit.seg_init_ds;
        release dut.seg_unit.seg_init_es;
        release dut.seg_unit.seg_init_ss;
        release dut.seg_unit.seg_init_fs;
        release dut.seg_unit.seg_init_gs;
        release dut.seg_unit.seg_init_idt;
        release dut.seg_unit.seg_init_gdt;
        release dut.seg_unit.seg_init_tr;
        if (tr_sel_arg != 0)
            release dut.TR;

        start_time = $time;
    end

    // Main test loop
    integer cycle = 0;
    reg prev_instruction_boundary = 0;
    always @(posedge clk) begin
        if (reset_n) begin
            cycle <= cycle + 1;

            // Count instructions
            prev_instruction_boundary <= instruction_boundary;
            if (instruction_boundary && !prev_instruction_boundary) begin
                instruction_count <= instruction_count + 1;
            end

            if ($test$plusargs("trace_ucode") && !dut.stall) begin
                $display("UCODE uc_addr=%0h op=%0h src=%0h alu_src=%0h dest=%0h opcode=%0h bus=%02h alu_res=%08x SIGMA=%08x shifter=%0d",
                         dut.uc_addr, dut.uc_aluop, dut.uc_source, dut.uc_alu_src,
                         dut.uc_dest, dut.uc_opcode, dut.uc_buscode, dut.alu_result, dut.SIGMA, dut.use_shifter_result);
            end
            if ($test$plusargs("trace_decode") && dut.decoder_inst.decq_count != 0) begin
                $display("DECODE opcode=%02x entry=%03x modrm=%02x has_modrm=%0d ctl_first=%03x entry=%04x 0f=%0d rep=%0d",
                         dut.i_bus.opcode, dut.i_bus.entry_point, dut.i_bus.modrm, dut.i_bus.has_modrm,
                         dut.decoder_inst.dec_ctl_first,
                         dut.decoder_inst.entry_2nd_pass ? dut.decoder_inst.modrm_entry_result : dut.decoder_inst.idle_entry_result,
                         dut.i_bus.has_0f, dut.i_bus.has_rep);
            end

            // Check termination conditions
            if (cycle >= max_cycles) begin
                $display("TIMEOUT after %0d cycles", max_cycles);
                $finish;
            end

            if (stop_on_halt && instruction_count >= 2) begin
                $display("Completed 2 instructions at cycle %0d", cycle);
                #50;  // Let things settle

                // Print final register state
                $display("RESULT REG: eax=0x%08x ecx=0x%08x edx=0x%08x ebx=0x%08x",
                         dut.EAX, dut.ECX, dut.EDX, dut.EBX);
                $display("RESULT REG: esp=0x%08x ebp=0x%08x esi=0x%08x edi=0x%08x",
                         dut.ESP, dut.EBP, dut.ESI, dut.EDI);
                $display("RESULT REG: eflags=0x%08x cs=0x%04x ds=0x%04x es=0x%04x ss=0x%04x",
                         dut.EFLAGS, dut.CS, dut.DS, dut.ES, dut.SS);
                $display("RESULT REG: fs=0x%04x gs=0x%04x ip=0x%08x",
                         dut.FS, dut.GS, dut.EIP);

                // Print segment caches (for protected mode tests)
                $display("RESULT SEG: cs base=0x%08x limit=0x%05x flags=0x%04x",
                         dut.seg_unit.seg_cache[SEG_CS].base, dut.seg_unit.seg_cache[SEG_CS].limit,
                         extract_seg_flags(dut.seg_unit.seg_cache[SEG_CS]));
                $display("RESULT SEG: ds base=0x%08x limit=0x%05x flags=0x%04x",
                         dut.seg_unit.seg_cache[SEG_DS].base, dut.seg_unit.seg_cache[SEG_DS].limit,
                         extract_seg_flags(dut.seg_unit.seg_cache[SEG_DS]));
                $display("RESULT SEG: es base=0x%08x limit=0x%05x flags=0x%04x",
                         dut.seg_unit.seg_cache[SEG_ES].base, dut.seg_unit.seg_cache[SEG_ES].limit,
                         extract_seg_flags(dut.seg_unit.seg_cache[SEG_ES]));
                $display("RESULT SEG: ss base=0x%08x limit=0x%05x flags=0x%04x",
                         dut.seg_unit.seg_cache[SEG_SS].base, dut.seg_unit.seg_cache[SEG_SS].limit,
                         extract_seg_flags(dut.seg_unit.seg_cache[SEG_SS]));
                $display("RESULT SEG: fs base=0x%08x limit=0x%05x flags=0x%04x",
                         dut.seg_unit.seg_cache[SEG_FS].base, dut.seg_unit.seg_cache[SEG_FS].limit,
                         extract_seg_flags(dut.seg_unit.seg_cache[SEG_FS]));
                $display("RESULT SEG: gs base=0x%08x limit=0x%05x flags=0x%04x",
                         dut.seg_unit.seg_cache[SEG_GS].base, dut.seg_unit.seg_cache[SEG_GS].limit,
                         extract_seg_flags(dut.seg_unit.seg_cache[SEG_GS]));
                $display("RESULT SEG: tr sel=0x%04x base=0x%08x limit=0x%05x flags=0x%04x",
                         dut.TR, dut.seg_unit.seg_cache[SEG_TR].base, dut.seg_unit.seg_cache[SEG_TR].limit,
                         extract_seg_flags(dut.seg_unit.seg_cache[SEG_TR]));

                // Print requested RAM locations
                for (int i = 0; i < ram_cnt; i++) begin
                    $display("RESULT MEM: @%0d=%d", ram_addrs[i], mem[ram_addrs[i]]);
                end

                // Print queue info
                $display("RESULT REG: q_len=0 q_consumed=0");  // Placeholder

                $finish;
            end

            if (stop_after_first && dbg_first_done) begin
                $display("First instruction completed at cycle %0d", cycle);
                #50;  // Let things settle

                // Print final register state
                $display("RESULT REG: eax=0x%08x ecx=0x%08x edx=0x%08x ebx=0x%08x",
                         dut.EAX, dut.ECX, dut.EDX, dut.EBX);
                $display("RESULT REG: esp=0x%08x ebp=0x%08x esi=0x%08x edi=0x%08x",
                         dut.ESP, dut.EBP, dut.ESI, dut.EDI);
                $display("RESULT REG: eflags=0x%08x cs=0x%04x ds=0x%04x es=0x%04x ss=0x%04x",
                         dut.EFLAGS, dut.CS, dut.DS, dut.ES, dut.SS);
                $display("RESULT REG: fs=0x%04x gs=0x%04x ip=0x%08x",
                         dut.FS, dut.GS, dut.debug_ip);

                // Print segment caches (for protected mode tests)
                $display("RESULT SEG: cs base=0x%08x limit=0x%05x flags=0x%04x",
                         dut.seg_unit.seg_cache[SEG_CS].base, dut.seg_unit.seg_cache[SEG_CS].limit,
                         extract_seg_flags(dut.seg_unit.seg_cache[SEG_CS]));
                $display("RESULT SEG: ds base=0x%08x limit=0x%05x flags=0x%04x",
                         dut.seg_unit.seg_cache[SEG_DS].base, dut.seg_unit.seg_cache[SEG_DS].limit,
                         extract_seg_flags(dut.seg_unit.seg_cache[SEG_DS]));
                $display("RESULT SEG: es base=0x%08x limit=0x%05x flags=0x%04x",
                         dut.seg_unit.seg_cache[SEG_ES].base, dut.seg_unit.seg_cache[SEG_ES].limit,
                         extract_seg_flags(dut.seg_unit.seg_cache[SEG_ES]));
                $display("RESULT SEG: ss base=0x%08x limit=0x%05x flags=0x%04x",
                         dut.seg_unit.seg_cache[SEG_SS].base, dut.seg_unit.seg_cache[SEG_SS].limit,
                         extract_seg_flags(dut.seg_unit.seg_cache[SEG_SS]));
                $display("RESULT SEG: fs base=0x%08x limit=0x%05x flags=0x%04x",
                         dut.seg_unit.seg_cache[SEG_FS].base, dut.seg_unit.seg_cache[SEG_FS].limit,
                         extract_seg_flags(dut.seg_unit.seg_cache[SEG_FS]));
                $display("RESULT SEG: gs base=0x%08x limit=0x%05x flags=0x%04x",
                         dut.seg_unit.seg_cache[SEG_GS].base, dut.seg_unit.seg_cache[SEG_GS].limit,
                         extract_seg_flags(dut.seg_unit.seg_cache[SEG_GS]));
                $display("RESULT SEG: tr sel=0x%04x base=0x%08x limit=0x%05x flags=0x%04x",
                         dut.TR, dut.seg_unit.seg_cache[SEG_TR].base, dut.seg_unit.seg_cache[SEG_TR].limit,
                         extract_seg_flags(dut.seg_unit.seg_cache[SEG_TR]));

                // Print requested RAM locations
                for (int i = 0; i < ram_cnt; i++) begin
                    $display("RESULT MEM: @%0d=%d", ram_addrs[i], mem[ram_addrs[i]]);
                end

                // Print queue info
                $display("RESULT REG: q_len=0 q_consumed=0");  // Placeholder

                $finish;
            end

            if (!stop_after_first && stop_reads > 0 && fetch_count >= stop_reads) begin
                $display("Reached %0d memory reads at cycle %0d", stop_reads, cycle);
                #50;
                $finish;
            end
        end
    end

    // Waveform dump
    // initial begin
    //     if ($test$plusargs("trace")) begin
    //         $dumpfile("trace.vcd");
    //         $dumpvars(0, tb_z386);
    //     end
    // end

endmodule
