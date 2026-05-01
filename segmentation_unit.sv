//
// Segmentation Unit for z386
//
// This contains:
//   - Descriptor Cache array (ES/CS/SS/DS/FS/GS/IDT/TR/LDT/GDT)
//   - Segment selection and address generation (seg_sel, mem_seg_base, mem_linear_addr)
//   - Limit Checker (GP/SS fault detection)
//
//
module segmentation_unit
    import z386_pkg::*;
(
    input              clk,
    input              reset_n,

    // Command interface — descriptor cache manipulation
    input              seg_cmd_valid,       // 1 when command should actually execute
    input      [3:0]   seg_cmd,            // SEG_CMD_* command (computed stall-independently)
    input      [3:0]   seg_target,         // Target segment (SEG_ES..SEG_GDT, SEG_NONE)
    input      [31:0]  seg_data,           // Command data (dest_value or alu_src_data)
    input      [31:0]  desc_lo,            // Descriptor low DWORD (from TMPC)
    input      [31:0]  desc_hi,            // Descriptor high DWORD (saved at PTOVRR)
    input      [15:0]  slctr,              // SLCTR register (null selector check for SDEL)
    input              copy_stack_dpl_s2,  // Write descriptor DPL into SS cache
    input      [1:0]   copy_dpl_s2,        // DPL value to write
    input              conform_dpl_s2,     // Update CS.DPL to match CPL

    output seg_desc_t  seg_cache [0:10],   // Descriptor caches (SEG_ES=0..SEG_GDT=10)
    output     [31:0]  lar_result,         // LAR combinational readback (keyed by seg_target)
    output     [31:0]  llim_result,        // LLIM combinational readback (keyed by seg_target)
    output     [31:0]  lbas_result,        // LBAS combinational readback (keyed by seg_target)

    // Segment state (set by commands, used by address translation and z386)
    output reg [3:0]   seg_sel,            // Active segment for memory ops
    output reg         is_dtable,          // Accessing GDT/IDT
    output reg         descsw_mode,        // Cross-privilege stack switch active
    output reg         stack_push_mode,    // Stack push direction active
    output reg         tss_access_flag,    // TSS access flag (for JTSSAF)

    // Address translation — offset → linear address + fault check
    input              pe,                 // CR0.PE - protected mode enable
    input              vm,                 // EFLAGS.VM - V86 mode
    input      [1:0]   cpl,                // Current privilege level
    input      [31:0]  offset,             // Segment offset (EA or IND)
    input      [1:0]   access_size,        // 0=byte, 1=word, 3=dword
    input              check_en,           // Limit check enabled this cycle
    input              is_mem_op,          // Memory operation (needs limit check)
    input              is_write,           // Write operation (needs writable check)

    output     [31:0]  linear_addr,        // Linear address = base + offset
    output             seg_fault,          // Segment limit/protection fault
    output             is_stack_fault      // Fault is on SS (→ #SS not #GP)
);

reg [3:0]   desc_write_seg;     // Tracks target segment for SDES/SDEL
reg         dt_target_idt;      // Tracks GDTR vs IDTR for SBAS/SLIM_TABLE
reg         addr_size;          // 1=32-bit, 0=16-bit effective address
reg [31:0]  seg_base_r;         // Registered segment base
reg [31:0]  seg_limit_r;        // Registered segment limit

// Instruction state latched from SEG_CMD_INIT_SEG
reg         i_addr32_r;
reg         i_stack_op_r;

wire [31:0] ES_base = seg_cache[SEG_ES].base;
wire [31:0] CS_base = seg_cache[SEG_CS].base;
wire [31:0] SS_base = seg_cache[SEG_SS].base;
wire [31:0] DS_base = seg_cache[SEG_DS].base;
wire [31:0] FS_base = seg_cache[SEG_FS].base;
wire [31:0] GS_base = seg_cache[SEG_GS].base;

wire [31:0] seg_base = (seg_sel == SEG_ES) ? ES_base :
                       (seg_sel == SEG_CS) ? CS_base :
                       (seg_sel == SEG_SS) ? (descsw_mode ? CS_base : SS_base) :
                       (seg_sel == SEG_DS) ? DS_base :
                       (seg_sel == SEG_FS) ? FS_base :
                       (seg_sel == SEG_GS) ? GS_base :
                       (seg_sel == SEG_TR) ? seg_cache[SEG_TR].base :
                       32'h0;  // IO/IDT/GDT: no base offset

wire [31:0] eff_offset = (addr_size || is_dtable) ? offset : {16'h0, offset[15:0]};
assign linear_addr = seg_base_r + eff_offset;

assign is_stack_fault = (seg_sel == SEG_SS);

// Split into subtractor + size comparison to keep access_size off the 32-bit critical path
wire [32:0] base_diff = {1'b0, seg_limit_r} - {1'b0, eff_offset};
wire start_out_of_bounds = base_diff[32];
// access_size encoding (0=byte,1=word,3=dword) == bytes-1, so check remaining < access_size
wire [31:0] diff_res = base_diff[31:0];
wire size_fault = (diff_res[31:3] == 29'd0) && (diff_res[2:0] < access_size);
wire limit_violated = start_out_of_bounds | size_fault;

// Real mode: stack ops with 16-bit addressing wrap instead of faulting
wire rm_limit_fault = !pe && limit_violated &&
                      !(is_stack_fault && !addr_size);

wire pm_limit_fault = pe && limit_violated && !is_dtable;

wire seg_writable = (seg_sel == SEG_ES) ? seg_cache[SEG_ES].writable :
                    (seg_sel == SEG_CS) ? vm :
                    (seg_sel == SEG_SS) ? 1'b1 :
                    (seg_sel == SEG_DS) ? seg_cache[SEG_DS].writable :
                    (seg_sel == SEG_FS) ? seg_cache[SEG_FS].writable :
                    (seg_sel == SEG_GS) ? seg_cache[SEG_GS].writable :
                    1'b1;  // TR/IDT/GDT/IO: no write check
wire write_fault = pe && is_write && !seg_writable && !is_dtable;

assign seg_fault = check_en && is_mem_op &&
                   (seg_sel != SEG_IO) &&
                   (rm_limit_fault || pm_limit_fault || write_fault);

function automatic [31:0] seg_base_for(input [3:0] sel, input dsw);
    case (sel)
        SEG_ES: seg_base_for = ES_base;
        SEG_CS: seg_base_for = CS_base;
        SEG_SS: seg_base_for = dsw ? CS_base : SS_base;
        SEG_DS: seg_base_for = DS_base;
        SEG_FS: seg_base_for = FS_base;
        SEG_GS: seg_base_for = GS_base;
        SEG_TR: seg_base_for = seg_cache[SEG_TR].base;
        default: seg_base_for = 32'h0;
    endcase
endfunction

function automatic [31:0] seg_limit_for(input [3:0] sel, input dsw);
    case (sel)
        SEG_ES: seg_limit_for = seg_effective_limit(seg_cache[SEG_ES]);
        SEG_CS: seg_limit_for = seg_effective_limit(seg_cache[SEG_CS]);
        SEG_SS: seg_limit_for = seg_effective_limit(dsw ? seg_cache[SEG_CS] : seg_cache[SEG_SS]);
        SEG_DS: seg_limit_for = seg_effective_limit(seg_cache[SEG_DS]);
        SEG_FS: seg_limit_for = seg_effective_limit(seg_cache[SEG_FS]);
        SEG_GS: seg_limit_for = seg_effective_limit(seg_cache[SEG_GS]);
        SEG_TR: seg_limit_for = seg_effective_limit(seg_cache[SEG_TR]);
        default: seg_limit_for = 32'hFFFFFFFF;
    endcase
endfunction

function automatic [3:0] effective_target(input [3:0] target);
    effective_target = (target == SEG_NONE) ? desc_write_seg : target;
endfunction

// LAR/LLIM/LBAS: z386 routes these to IND in same cycle
seg_desc_t lar_desc;
wire [7:0] lar_ar_byte = {lar_desc.P, lar_desc.DPL, lar_desc.S, lar_desc.seg_type};
assign lar_desc = (seg_target <= SEG_GDT) ? seg_cache[seg_target] : '0;
assign lar_result = {16'h0, lar_ar_byte, 8'h0};
assign llim_result = seg_effective_limit(lar_desc);
assign lbas_result = lar_desc.base;

// Testbench can't force unpacked array struct elements, so use individual regs
`ifdef VERILATOR
seg_desc_t seg_init_es, seg_init_cs, seg_init_ss, seg_init_ds;
seg_desc_t seg_init_fs, seg_init_gs, seg_init_idt, seg_init_tr;
seg_desc_t seg_init_ldt, seg_init_gdt;
initial begin
    seg_init_es  = seg_desc_real_mode(16'h0000);
    seg_init_cs  = seg_desc_reset_cs();
    seg_init_ss  = seg_desc_real_mode(16'h0000);
    seg_init_ds  = seg_desc_real_mode(16'h0000);
    seg_init_fs  = seg_desc_real_mode(16'h0000);
    seg_init_gs  = seg_desc_real_mode(16'h0000);
    seg_init_idt = seg_desc_idt_real_mode();
    seg_init_tr  = seg_desc_real_mode(16'h0000);
    seg_init_ldt = seg_desc_real_mode(16'h0000);
    seg_init_gdt = seg_desc_idt_real_mode();
end
`endif

//=============================================================================
// Segment descriptor cache
//=============================================================================
always_ff @(posedge clk) begin
    if (!reset_n) begin
`ifdef VERILATOR
        seg_cache[SEG_ES]  <= seg_init_es;
        seg_cache[SEG_CS]  <= seg_init_cs;
        seg_cache[SEG_SS]  <= seg_init_ss;
        seg_cache[SEG_DS]  <= seg_init_ds;
        seg_cache[SEG_FS]  <= seg_init_fs;
        seg_cache[SEG_GS]  <= seg_init_gs;
        seg_cache[SEG_IDT] <= seg_init_idt;
        seg_cache[SEG_TR]  <= seg_init_tr;
        seg_cache[SEG_LDT] <= seg_init_ldt;
        seg_cache[SEG_GDT] <= seg_init_gdt;
`else
        seg_cache[SEG_ES]  <= seg_desc_real_mode(16'h0000);
        seg_cache[SEG_CS]  <= seg_desc_reset_cs();
        seg_cache[SEG_SS]  <= seg_desc_real_mode(16'h0000);
        seg_cache[SEG_DS]  <= seg_desc_real_mode(16'h0000);
        seg_cache[SEG_FS]  <= seg_desc_real_mode(16'h0000);
        seg_cache[SEG_GS]  <= seg_desc_real_mode(16'h0000);
        seg_cache[SEG_IDT] <= seg_desc_idt_real_mode();
        seg_cache[SEG_TR]  <= seg_desc_real_mode(16'h0000);
        seg_cache[SEG_LDT] <= seg_desc_real_mode(16'h0000);
        seg_cache[SEG_GDT] <= seg_desc_idt_real_mode();  // GDTR: base=0, limit=0x3FF
`endif
    end else if (seg_cmd_valid) begin
        case (seg_cmd)
            SEG_CMD_SBRM: begin
                // Real-mode segment register load: set base = segment × 16.
                // Limit and granularity are NOT reset, thus preserving "unreal mode"
                case (seg_target[2:0])
                    SEG_ES:  seg_cache[SEG_ES].base <= {12'h0, seg_data[15:0], 4'h0};
                    SEG_CS: begin
                        seg_cache[SEG_CS].base <= {12'h0, seg_data[15:0], 4'h0};
                        seg_cache[SEG_CS].D_B <= 1'b0;
                    end
                    SEG_SS:  seg_cache[SEG_SS].base <= {12'h0, seg_data[15:0], 4'h0};
                    SEG_DS:  seg_cache[SEG_DS].base <= {12'h0, seg_data[15:0], 4'h0};
                    SEG_FS:  seg_cache[SEG_FS].base <= {12'h0, seg_data[15:0], 4'h0};
                    SEG_GS:  seg_cache[SEG_GS].base <= {12'h0, seg_data[15:0], 4'h0};
                    SEG_IDT: seg_cache[SEG_IDT].base <= {12'h0, seg_data[15:0], 4'h0};
                    default: ;
                endcase
            end

            SEG_CMD_SAR: begin
                automatic seg_desc_t sar_merged;
                automatic logic [3:0] sar_seg;
                sar_seg = effective_target(seg_target);
                sar_merged = merge_sar(seg_cache[sar_seg], seg_data);
                seg_cache[sar_seg] <= sar_merged;
            end

            SEG_CMD_SLIM: begin
                seg_cache[effective_target(seg_target)].limit <= seg_data[19:0];
            end

            SEG_CMD_SDEH: begin
                automatic seg_desc_t sdeh_merged;
                automatic logic [3:0] sdeh_seg;
                sdeh_seg = effective_target(seg_target);
                sdeh_merged = merge_sdeh(seg_cache[sdeh_seg], seg_data);
                seg_cache[sdeh_seg] <= sdeh_merged;
            end

            SEG_CMD_SDES: begin
                automatic seg_desc_t sdes_merged;
                sdes_merged = merge_sdes(seg_cache[desc_write_seg], seg_data);
                seg_cache[desc_write_seg] <= sdes_merged;
            end

            SEG_CMD_SDEL: begin
                if (slctr[15:2] == 14'b0) begin
                    automatic seg_desc_t null_desc;
                    null_desc = '0;
                    null_desc.base = 32'hFFFFFFFF;
                    null_desc.limit = 20'hFFFFF;
                    null_desc.P = 1'b1;
                    seg_cache[desc_write_seg] <= null_desc;
                end else begin
                    automatic seg_desc_t sdel_merged;
                    sdel_merged = merge_sdel(seg_cache[desc_write_seg], seg_data);
                    seg_cache[desc_write_seg] <= sdel_merged;
                    if (desc_write_seg == SEG_TR)
                        seg_cache[SEG_TR].seg_type[1] <= 1'b1;
                end
            end

            SEG_CMD_DESC: begin
                seg_cache[effective_target(seg_target)] <= decode_descriptor(desc_lo, desc_hi);
            end

            SEG_CMD_SBAS: begin
                if (dt_target_idt)
                    seg_cache[SEG_IDT].base <= seg_data;
                else
                    seg_cache[SEG_GDT].base <= seg_data;
            end

            SEG_CMD_SLIM_TABLE: begin
                if (dt_target_idt)
                    seg_cache[SEG_IDT].limit <= {4'h0, seg_data[15:0]};
                else
                    seg_cache[SEG_GDT].limit <= {4'h0, seg_data[15:0]};
            end

            default: ;
        endcase

        if (copy_stack_dpl_s2)
            seg_cache[SEG_SS].DPL <= copy_dpl_s2;

        if (conform_dpl_s2)
            seg_cache[SEG_CS].DPL <= cpl;
    end
end

always_ff @(posedge clk) begin
    if (!reset_n) begin
        dt_target_idt <= 1'b0;
        desc_write_seg <= 4'd0;
    end else if (seg_cmd_valid) begin
        if (seg_target == SEG_IDT)
            dt_target_idt <= 1'b1;
        else if (seg_target == SEG_GDT)
            dt_target_idt <= 1'b0;

        if (seg_target != SEG_NONE &&
            seg_target != SEG_IDT && seg_target != SEG_GDT && seg_target != SEG_IO)
            desc_write_seg <= seg_target;
    end
end

//=============================================================================
// Segment selection, address size, and mode flags
//=============================================================================
always_ff @(posedge clk) begin
    if (!reset_n) begin
        seg_sel <= SEG_DS;
        is_dtable <= 1'b0;
        addr_size <= 1'b0;
        seg_base_r <= 32'h0;  // DS_base at reset
        seg_limit_r <= 32'hFFFF;
        stack_push_mode <= 1'b0;
        descsw_mode <= 1'b0;
        tss_access_flag <= 1'b0;
        i_addr32_r <= 1'b0;
        i_stack_op_r <= 1'b0;
    end else if (seg_cmd_valid) begin
        case (seg_cmd)
            SEG_CMD_INIT_SEG: begin
                seg_sel <= seg_target;
                is_dtable <= 1'b0;
                seg_base_r <= seg_base_for(seg_target, 1'b0);
                seg_limit_r <= seg_limit_for(seg_target, 1'b0);
                addr_size <= (seg_data[1] && pe) ? seg_cache[SEG_SS].D_B : seg_data[0];
                i_addr32_r <= seg_data[0];
                i_stack_op_r <= seg_data[1];
                stack_push_mode <= 1'b0;
                descsw_mode <= 1'b0;
            end

            SEG_CMD_UPDATE_SEG: begin
                seg_sel <= seg_target;
                is_dtable <= (seg_target == SEG_IDT || seg_target == SEG_GDT);
                if (seg_data[0]) begin
                    descsw_mode <= 1'b0;
                    addr_size <= pe ? seg_cache[SEG_SS].D_B : i_addr32_r;
                    seg_base_r <= seg_base_for(seg_target, 1'b0);
                    seg_limit_r <= seg_limit_for(seg_target, 1'b0);
                end else begin
                    if ((i_stack_op_r || stack_push_mode) && pe && seg_target == SEG_SS)
                        addr_size <= descsw_mode ? seg_cache[SEG_CS].D_B : seg_cache[SEG_SS].D_B;
                    else
                        addr_size <= i_addr32_r;
                    seg_base_r <= seg_base_for(seg_target, descsw_mode);
                    seg_limit_r <= seg_limit_for(seg_target, descsw_mode);
                end
            end

            SEG_CMD_SPCR: begin
                stack_push_mode <= 1'b1;
            end

            SEG_CMD_STSSAF: begin
                stack_push_mode <= 1'b0;
                descsw_mode <= 1'b0;
                tss_access_flag <= 1'b1;
                if (seg_sel == SEG_SS) begin
                    seg_base_r <= SS_base;
                    seg_limit_r <= seg_effective_limit(seg_cache[SEG_SS]);
                end
            end

            SEG_CMD_CTSSAF: begin
                tss_access_flag <= 1'b0;
            end

            SEG_CMD_DESCSW: begin
                stack_push_mode <= 1'b1;
                seg_sel <= SEG_SS;
                is_dtable <= 1'b0;
                addr_size <= pe ? seg_cache[SEG_CS].D_B : i_addr32_r;
                descsw_mode <= 1'b1;
                seg_base_r <= CS_base;
                seg_limit_r <= seg_effective_limit(seg_cache[SEG_CS]);
            end

            default: ;
        endcase
    end
end

endmodule
