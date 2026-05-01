// 45-bit expanded microcode ROM.
//
// takes two cycles as 386 allows for better timing
module ucode_rom #(
    parameter INIT_HEX = "ucode45.hex"
) (
    input              clk,
    input              ce,
    input       [11:0] addr,
    output      [44:0] q,
    output      [5:0]  q_shift_source,
    output      [5:0]  q_shift_alu_src,
    output      [6:0]  q_shift_aluop
);

(* preserve *) reg [5:0] q_shift_source_r;
(* preserve *) reg [5:0] q_shift_alu_src_r;
(* preserve *) reg [6:0] q_shift_aluop_r;

`ifdef Z386_QUARTUS_M10K_UCODE
wire [44:0] q_mem;
reg  [44:0] q_r;

altsyncram #(
    .operation_mode("ROM"),
	    .width_a(45),
	    .widthad_a(12),
	    .numwords_a(2560),
	    .outdata_reg_a("UNREGISTERED"),
	    .address_aclr_a("NONE"),
	    .outdata_aclr_a("NONE"),
    .init_file("ucode45.mif"),
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
        q_r <= q_mem;
        q_shift_source_r <= q_mem[23:18];
        q_shift_alu_src_r <= q_mem[36:31];
        q_shift_aluop_r <= q_mem[17:11];
    end
end

assign q = q_r;
`else
`ifdef Z386_QUARTUS_LOGIC_UCODE
(* ramstyle = "logic" *) reg [44:0] microcode_rom [0:2559];
`else
(* ram_style = "block" *) reg [44:0] microcode_rom [0:2559] /* synthesis syn_ramstyle = "block_ram" */;
`endif
	reg [44:0] q_mem;
	reg [44:0] q_r;

initial begin
    $readmemh(INIT_HEX, microcode_rom);
end

	always_ff @(posedge clk) begin
	    if (ce) begin
	        q_mem <= microcode_rom[addr];
	        q_r <= q_mem;
	        q_shift_source_r <= q_mem[23:18];
	        q_shift_alu_src_r <= q_mem[36:31];
	        q_shift_aluop_r <= q_mem[17:11];
	    end
	end

assign q = q_r;
`endif

assign q_shift_source = q_shift_source_r;
assign q_shift_alu_src = q_shift_alu_src_r;
assign q_shift_aluop = q_shift_aluop_r;

endmodule
