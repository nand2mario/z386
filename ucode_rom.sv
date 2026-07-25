// Microcode ROM with predecode. The physical ROM stores a 37-bit native microcode word plus a 3-bit v52 D2 early kind. Execution still...
// Details: doc/z386x/implementation_notes.md#src-24-z386x-ucode-rom-sv-1
module ucode_rom
    import z386_pkg::*;
#(
    parameter INIT_HEX = "ucode.hex"
) (
    input              clk,
    input              addr_ce,
    input              q_ce,
    input       [11:0] addr,
    output      [50:0] q_early,
    output      [50:0] q,
    output      [2:0]  q_kind_early,
    output      [5:0]  q_shift_source,
    output      [1:0]  q_shift2_source,
    output      [5:0]  q_shift_alu_src,
    output      [6:0]  q_shift_aluop,
    output      [2:0]  q_dly_source,
    output      [8:0]  q_mem_ctrl
);

(* preserve *) reg [5:0] q_shift_source_r;
(* preserve *) reg [1:0] q_shift2_source_r;
(* preserve *) reg [5:0] q_shift_alu_src_r;
(* preserve *) reg [6:0] q_shift_aluop_r;
reg [2:0] q_dly_source_r;
(* preserve *) reg [8:0] q_mem_ctrl_r;

// Source class for architectural GPR writes in an RNI delay slot. The Intel
// ROM uses only these four sources; zero also covers non-forwarding uops.
function automatic [2:0] dly_source_predecode(input [5:0] s);
    case (s)
        SRC_SIGMA:  dly_source_predecode = 3'd1;
        SRC_OPR_R:  dly_source_predecode = 3'd2;
        SRC_COUNTR: dly_source_predecode = 3'd3;
        SRC_NEG1:   dly_source_predecode = 3'd4;
        default:    dly_source_predecode = 3'd0;
    endcase
endfunction

// SHIFT2 uses only four source fields in the immutable Intel ROM. Register a
// compact selector beside q so the flag-critical barrel path avoids the
// high-fanout general source decoder.
function automatic [1:0] shift2_source_predecode(input [5:0] s);
    case (s)
        SRC_TMPC:   shift2_source_predecode = 2'd0;
        SRC_TMPE:   shift2_source_predecode = 2'd1;
        SRC_SIGMA:  shift2_source_predecode = 2'd2;
        SRC_SRCREG: shift2_source_predecode = 2'd3;
        default:    shift2_source_predecode = 2'd0;
    endcase
endfunction

// Predecoded control bits 50:37, computed from the raw 37-bit word in the
// ROM output register stage (one LUT level between RAM output and q_r).
function automatic [13:0] ucode_predecode(input [36:0] w);
    logic [6:0] aluop;
    logic [5:0] buscode;
    logic [1:0] subcode;
    logic [5:0] alusrc;
    logic       mem_busop;
    aluop   = w[17:11];
    buscode = w[5:0];
    subcode = w[7:6];
    alusrc  = w[36:31];
    // Note: BUSOP_WR_D (SGDT/SIDT base store) was historically missing here
    // (and in the retired gen_ucode45.py), so the base store never issued a
    // memory request — EMM386/VCPI context saves silently lost the IDT/GDT
    // base.  Found via TC3-under-EMM386 whole-system debugging, 2026-06-12.
    mem_busop = (buscode == BUSOP_RD_BW) || (buscode == BUSOP_RD_D) ||
                (buscode == BUSOP_RD) || (buscode == BUSOP_RD_WORD) ||
                (buscode == BUSOP_RD_IND) || (buscode == BUSOP_WR) ||
                (buscode == BUSOP_WR_OPR) || (buscode == BUSOP_WR_WORD) ||
                (buscode == BUSOP_WR_D) || (buscode == BUSOP_CW);
    // bit37: ALU group op (arithmetic flags path)
    ucode_predecode[0] = (aluop == ALUJMP_ALU) || (aluop == ALUJMP_INCDEC) ||
                         (aluop == ALUJMP_CMPTST) || (aluop == ALUJMP_ADC) ||
                         (aluop == ALUJMP_CMP) || (aluop == ALUJMP_AAAAAS) ||
                         (aluop == ALUJMP_DAADAS);
    // bit38: bus op or DLY (uc_bus_or_dly)
    ucode_predecode[1] = mem_busop || (buscode == BUSOP_IACK) || (subcode == 2'b00);
    // bit39: memory bus op (uc_is_mem_busop)
    ucode_predecode[2] = mem_busop;
    // bit40: memory write
    ucode_predecode[3] = (buscode == BUSOP_WR) || (buscode == BUSOP_WR_OPR) ||
                         (buscode == BUSOP_WR_WORD) || (buscode == BUSOP_WR_D);
    // bit41: check write (CW)
    ucode_predecode[4] = (buscode == BUSOP_CW);
    // bit42: word-sized access
    ucode_predecode[5] = (buscode == BUSOP_RD_WORD) || (buscode == BUSOP_WR_WORD);
    // bit43: descriptor/auxiliary dword access (always 32-bit regardless of
    // operand size); includes the SGDT/SIDT base store
    ucode_predecode[6] = (buscode == BUSOP_RD_D) || (buscode == BUSOP_RD_IND) ||
                         (buscode == BUSOP_WR_D);
    // bit44: JPEREQ with jump offset (coprocessor jump, always taken)
    ucode_predecode[7] = (aluop == ALUJMP_JPEREQ) && (alusrc != 6'h3F);
    // bit45: IO-capable read buscode (issues an IO read when seg_sel == SEG_IO)
    ucode_predecode[8] = (buscode == BUSOP_RD_BW) || (buscode == BUSOP_RD);
    // bit46: IO-capable write buscode (issues an IO write when seg_sel == SEG_IO)
    ucode_predecode[9] = (buscode == BUSOP_WR) || (buscode == BUSOP_WR_OPR);
    // bit47: IACK bus cycle
    ucode_predecode[10] = (buscode == BUSOP_IACK);
    // bit48: pure DLY — waits for the bus but issues no request itself
    //        (optimistic-read grace may release this uop one cycle early)
    ucode_predecode[11] = (subcode == 2'b00) && !mem_busop && (buscode != BUSOP_IACK);
    // bit49: RPT (repeat) opcode
    ucode_predecode[12] = (w[10:8] == 3'b110);
    // bit50: WIO — wait for interrupt/IO (RPT opcode with WIO subcode)
    ucode_predecode[13] = (w[10:8] == 3'b110) && (subcode == 2'b10);
endfunction

// Dedicated copy of bits 47:39 for the paging request cone. Keeping these
// controls beside q avoids routing the wide, high-fanout ucode bus into TLB
// finalization and request selection.
function automatic [8:0] mem_ctrl_predecode(input [36:0] w);
    logic [13:0] p;
    begin
        p = ucode_predecode(w);
        mem_ctrl_predecode = p[10:2];
    end
endfunction

`ifdef Z386_QUARTUS_M10K_UCODE
wire [39:0] q_mem;
reg  [50:0] q_r;

altsyncram #(
    .operation_mode("ROM"),
	    .width_a(40),
	    .widthad_a(12),
	    .numwords_a(2560),
	    .outdata_reg_a("UNREGISTERED"),
	    .address_aclr_a("NONE"),
	    .outdata_aclr_a("NONE"),
    .init_file("ucode.mif"),
    .ram_block_type("M10K"),
    .intended_device_family("Cyclone V"),
    .lpm_type("altsyncram")
) microcode_rom_altsyncram (
    .address_a(addr),
    .clock0(clk),
    .clocken0(addr_ce),
    .q_a(q_mem),
    .aclr0(1'b0),
    .addressstall_a(1'b0),
    .clocken1(1'b1),
    .clocken2(1'b1),
    .clocken3(1'b1),
    .rden_a(1'b1),
    .eccstatus()
);

always_ff @(posedge clk) begin
    if (q_ce) begin
        q_r <= {ucode_predecode(q_mem[36:0]), q_mem[36:0]};
        q_shift_source_r <= q_mem[23:18];
        q_shift2_source_r <= shift2_source_predecode(q_mem[23:18]);
        q_shift_alu_src_r <= q_mem[36:31];
        q_shift_aluop_r <= q_mem[17:11];
        q_dly_source_r <= dly_source_predecode(q_mem[23:18]);
        q_mem_ctrl_r <= mem_ctrl_predecode(q_mem[36:0]);
    end
end

assign q = q_r;
assign q_early = {ucode_predecode(q_mem[36:0]), q_mem[36:0]};
assign q_kind_early = q_mem[39:37];
`else
`ifdef Z386_QUARTUS_LOGIC_UCODE
(* ramstyle = "logic" *) reg [39:0] microcode_rom [0:2559];
`else
(* ram_style = "block" *) reg [39:0] microcode_rom [0:2559] /* synthesis syn_ramstyle = "block_ram" */;
`endif
	reg [39:0] q_mem;
	reg [50:0] q_r;

initial begin
    $readmemh(INIT_HEX, microcode_rom);
end

	always_ff @(posedge clk) begin
	    if (addr_ce)
	        q_mem <= microcode_rom[addr];
	    if (q_ce) begin
	        q_r <= {ucode_predecode(q_mem[36:0]), q_mem[36:0]};
	        q_shift_source_r <= q_mem[23:18];
	        q_shift2_source_r <= shift2_source_predecode(q_mem[23:18]);
	        q_shift_alu_src_r <= q_mem[36:31];
	        q_shift_aluop_r <= q_mem[17:11];
	        q_dly_source_r <= dly_source_predecode(q_mem[23:18]);
	        q_mem_ctrl_r <= mem_ctrl_predecode(q_mem[36:0]);
	    end
	end

assign q = q_r;
assign q_early = {ucode_predecode(q_mem[36:0]), q_mem[36:0]};
assign q_kind_early = q_mem[39:37];
`endif

assign q_shift_source = q_shift_source_r;
assign q_shift2_source = q_shift2_source_r;
assign q_shift_alu_src = q_shift_alu_src_r;
assign q_shift_aluop = q_shift_aluop_r;
assign q_dly_source = q_dly_source_r;
assign q_mem_ctrl = q_mem_ctrl_r;

endmodule
