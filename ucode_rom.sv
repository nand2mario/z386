// Microcode ROM with predecode.
//
// The ROM stores the original 37-bit microcode word (ucode.hex / ucode.mif).
// Takes two cycles as 386 allows for better timing; bits 50:37 are predecoded
// combinationally in the spare ROM output register stage, so no expanded
// ROM image is needed.
module ucode_rom
    import z386_pkg::*;
#(
    parameter INIT_HEX = "ucode.hex"
) (
    input              clk,
    input              ce,
    input       [11:0] addr,
    output      [50:0] q_early,
    output      [50:0] q,
    output      [5:0]  q_shift_source,
    output      [5:0]  q_shift_alu_src,
    output      [6:0]  q_shift_aluop
);

(* preserve *) reg [5:0] q_shift_source_r;
(* preserve *) reg [5:0] q_shift_alu_src_r;
(* preserve *) reg [6:0] q_shift_aluop_r;

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

`ifdef Z386_QUARTUS_M10K_UCODE
wire [36:0] q_mem;
reg  [50:0] q_r;

altsyncram #(
    .operation_mode("ROM"),
	    .width_a(37),
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
    .clocken0(ce),
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
    if (ce) begin
        q_r <= {ucode_predecode(q_mem), q_mem};
        q_shift_source_r <= q_mem[23:18];
        q_shift_alu_src_r <= q_mem[36:31];
        q_shift_aluop_r <= q_mem[17:11];
    end
end

assign q = q_r;
assign q_early = {ucode_predecode(q_mem), q_mem};
`else
`ifdef Z386_QUARTUS_LOGIC_UCODE
(* ramstyle = "logic" *) reg [36:0] microcode_rom [0:2559];
`else
(* ram_style = "block" *) reg [36:0] microcode_rom [0:2559] /* synthesis syn_ramstyle = "block_ram" */;
`endif
	reg [36:0] q_mem;
	reg [50:0] q_r;

initial begin
    $readmemh(INIT_HEX, microcode_rom);
end

	always_ff @(posedge clk) begin
	    if (ce) begin
	        q_mem <= microcode_rom[addr];
	        q_r <= {ucode_predecode(q_mem), q_mem};
	        q_shift_source_r <= q_mem[23:18];
	        q_shift_alu_src_r <= q_mem[36:31];
	        q_shift_aluop_r <= q_mem[17:11];
	    end
	end

assign q = q_r;
assign q_early = {ucode_predecode(q_mem), q_mem};
`endif

assign q_shift_source = q_shift_source_r;
assign q_shift_alu_src = q_shift_alu_src_r;
assign q_shift_aluop = q_shift_aluop_r;

endmodule
