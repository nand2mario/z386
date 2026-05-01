//
// 80386 ALU variant written to encourage Intel/Altera carry-chain inference.
//
module alu_alt
    import z386_pkg::*;
(
    input  [4:0]  op,           // ALUOPC
    input  [31:0] src,
    input  [31:0] dst,
    input  [1:0]  op_size,      // 0=byte, 1=word, 2=dword
    input  [31:0] flags,
    input         update_carry, // gate CF updates

    output [31:0] result,
    output [31:0] flags_out
);

wire is_byte  = (op_size == 2'd0);
wire is_word  = (op_size == 2'd1);
wire is_dword = (op_size == 2'd2);

wire is_logic  = (op == ALU_AND) || (op == ALU_OR) || (op == ALU_XOR) || (op == ALU_ANDN);
wire is_extend = (op == ALU_ZEXT) || (op == ALU_SEXT);
wire is_not    = (op == ALU_NOT);
wire is_addfam = (op == ALU_ADD) || (op == ALU_ADC) || (op == ALU_INC) || (op == ALU_INC2);
wire is_subfam = (op == ALU_SUBT) || (op == ALU_SBB) || (op == ALU_CMP) ||
                 (op == ALU_DEC) || (op == ALU_DEC2) || (op == ALU_NEG);
wire is_adjust = (op == ALU_DAA) || (op == ALU_DAS) || (op == ALU_AAA) || (op == ALU_AAS);

// Decimal/ASCII adjust helper signals.
wire daa_low_adj    = (dst[3:0] > 4'd9) || flags[4];
wire daa_high_adj   = (dst[7:0] > 8'h99) || flags[0];
wire daa_low_carry  = daa_low_adj && (dst[7:0] > 8'hF9);
wire das_low_borrow = daa_low_adj && (dst[7:0] < 8'h06);
wire [7:0] daa_adj8 = ({8{daa_low_adj}}  & 8'h06) |
                      ({8{daa_high_adj}} & 8'h60);
wire aaa_cond = (dst[3:0] > 4'd9) || flags[4];
wire [31:0] aaa_adj = aaa_cond ? 32'h0000_0106 : 32'h0;

reg [31:0] add_a;
reg [31:0] add_b;
reg        add_cin;
reg [31:0] logic_result;
reg [31:0] override_result;
reg [31:0] af_operand;
reg [31:0] of_operand;
reg        of_is_sub;
reg        use_override;

always_comb begin
    add_a           = src;
    add_b           = dst;
    add_cin         = 1'b0;
    logic_result    = dst;
    override_result = dst;
    af_operand      = src;
    of_operand      = src;
    of_is_sub       = 1'b0;
    use_override    = 1'b0;

    case (op)
        ALU_ADD: begin
            add_a = src;
            add_b = dst;
            af_operand = dst;
            of_operand = dst;
        end
        ALU_ADC: begin
            add_a   = src;
            add_b   = dst;
            add_cin = flags[0];
            af_operand = dst;
            of_operand = dst;
        end
        ALU_SUBT, ALU_CMP: begin
            add_a   = dst;
            add_b   = ~src;
            add_cin = 1'b1;
            of_is_sub = 1'b1;
        end
        ALU_SBB: begin
            add_a   = dst;
            add_b   = ~src;
            add_cin = ~flags[0];
            of_is_sub = 1'b1;
        end
        ALU_INC: begin
            add_a   = dst;
            add_b   = 32'h0;
            add_cin = 1'b1;
            af_operand = 32'h1;
            of_operand = 32'h1;
        end
        ALU_DEC: begin
            add_a   = dst;
            add_b   = ~32'h1;
            add_cin = 1'b1;
            af_operand = 32'h1;
            of_operand = 32'h1;
            of_is_sub = 1'b1;
        end
        ALU_INC2: begin
            add_a = dst;
            add_b = 32'h2;
            af_operand = 32'h2;
            of_operand = 32'h2;
        end
        ALU_DEC2: begin
            add_a   = dst;
            add_b   = ~32'h2;
            add_cin = 1'b1;
            af_operand = 32'h2;
            of_operand = 32'h2;
            of_is_sub = 1'b1;
        end
        ALU_NEG: begin
            add_a   = 32'h0;
            add_b   = ~dst;
            add_cin = 1'b1;
            af_operand = dst;
            of_operand = dst;
            of_is_sub = 1'b1;
        end
        ALU_DAA: begin
            add_a = dst;
            add_b = {24'h0, daa_adj8};
            af_operand = {24'h0, daa_adj8};
            of_operand = {24'h0, daa_adj8};
        end
        ALU_DAS: begin
            add_a   = dst;
            add_b   = ~{24'h0, daa_adj8};
            add_cin = 1'b1;
            af_operand = {24'h0, daa_adj8};
            of_operand = {24'h0, daa_adj8};
            of_is_sub = 1'b1;
        end
        ALU_AAA: begin
            add_a = dst;
            add_b = aaa_adj;
            af_operand = aaa_adj;
            of_operand = aaa_adj;
        end
        ALU_AAS: begin
            add_a   = dst;
            add_b   = ~aaa_adj;
            add_cin = 1'b1;
            af_operand = aaa_adj;
            of_operand = aaa_adj;
            of_is_sub = 1'b1;
        end

        ALU_AND:  logic_result = src & dst;
        ALU_OR:   logic_result = src | dst;
        ALU_XOR:  logic_result = src ^ dst;
        ALU_ANDN: logic_result = dst & ~src;
        ALU_PASS: logic_result = dst;
        default:  logic_result = src; // ALU_PASS2 and default slice behavior
    endcase

    case (op)
        ALU_NOT: begin
            use_override    = 1'b1;
            override_result = ~dst;
        end
        ALU_ZEXT: begin
            use_override    = 1'b1;
            override_result = is_word ? {24'h0, dst[7:0]} :
                              is_dword ? {16'h0, dst[15:0]} : dst;
        end
        ALU_SEXT: begin
            use_override    = 1'b1;
            override_result = is_word ? {{24{dst[7]}}, dst[7:0]} :
                              is_dword ? {{16{dst[15]}}, dst[15:0]} : dst;
        end
        ALU_ZEXT_B: begin
            use_override    = 1'b1;
            override_result = {24'h0, dst[7:0]};
        end
        ALU_SEXT_B: begin
            use_override    = 1'b1;
            override_result = {{24{dst[7]}}, dst[7:0]};
        end
        ALU_SEXTD: begin
            use_override    = 1'b1;
            override_result = is_dword ? (dst[31] ? 32'hFFFF_FFFF : 32'h0) :
                                         (dst[15] ? 32'h0000_FFFF : 32'h0);
        end
        ALU_SIGN: begin
            use_override    = 1'b1;
            override_result = is_byte ? {32{dst[7]}} :
                              is_word ? {32{dst[15]}} :
                                        {32{dst[31]}};
        end
        default: ;
    endcase
end

wire [32:0] add_sum33 = {1'b0, add_a} + {1'b0, add_b} + {32'd0, add_cin};
wire [31:0] arith_result = add_sum33[31:0];
wire arith_op = is_addfam || is_subfam || is_adjust;

wire [31:0] raw_result = arith_op ? arith_result : logic_result;
wire [31:0] alu_raw = use_override ? override_result : raw_result;
wire [31:0] alu_post = (op == ALU_AAA || op == ALU_AAS) ? {alu_raw[31:8], 4'h0, alu_raw[3:0]} :
                       (op == ALU_DAA || op == ALU_DAS) ? {dst[31:8], alu_raw[7:0]} :
                       alu_raw;

assign result = alu_post;

wire [8:0]  add_sum8  = {1'b0, add_a[7:0]}   + {1'b0, add_b[7:0]}   + {8'd0, add_cin};
wire [16:0] add_sum16 = {1'b0, add_a[15:0]}  + {1'b0, add_b[15:0]}  + {16'd0, add_cin};

wire carry7  = arith_op ? add_sum8[8]   : 1'b0;
wire carry15 = arith_op ? add_sum16[16] : 1'b0;
wire carry31 = arith_op ? add_sum33[32] : 1'b0;

wire flag_byte_mode = is_byte || is_adjust;
wire r_msb = flag_byte_mode ? raw_result[7] : (is_word ? raw_result[15] : raw_result[31]);
wire zf_byte  = raw_result[7:0] == 8'h00;
wire zf_word  = raw_result[15:0] == 16'h0000;
wire zf_dword = raw_result == 32'h0000_0000;
wire zf_result = is_dword ? zf_dword : (is_word ? zf_word : zf_byte);
wire pf_result = ~^raw_result[7:0];

wire add_a_sign = flag_byte_mode ? add_a[7] : (is_word ? add_a[15] : add_a[31]);
wire of_b_sign = flag_byte_mode ? of_operand[7] : (is_word ? of_operand[15] : of_operand[31]);
wire result_sign = r_msb;
wire of_arith = arith_op && (of_is_sub ? ((add_a_sign ^ of_b_sign) & (add_a_sign ^ result_sign)) :
                                        (~(add_a_sign ^ of_b_sign) & (add_a_sign ^ result_sign)));
wire af_arith = arith_op && (add_a[4] ^ af_operand[4] ^ raw_result[4]);

wire cf_byte  = carry7;
wire cf_word  = carry15;
wire cf_dword = carry31;
wire cout_msb = flag_byte_mode ? cf_byte : (is_word ? cf_word : cf_dword);

reg [31:0] f2;
always_comb begin
    logic cf_cand;

    f2 = flags;
    cf_cand = flags[0];

    if (is_extend || is_not) begin
        f2 = flags;
    end else begin
        if (!(op == ALU_AAA || op == ALU_AAS)) begin
            f2[7] = r_msb;
            f2[6] = zf_result;
            f2[2] = pf_result;
        end

        f2[11] = of_arith;

        if (is_addfam) begin
            f2[4] = af_arith;
        end else if (is_subfam) begin
            f2[4] = af_arith;
        end else if (op == ALU_NOT || op == ALU_PASS || op == ALU_PASS2) begin
            f2[4] = flags[4];
        end else begin
            f2[4] = 1'b0;
        end

        if (is_logic) begin
            cf_cand = 1'b0;
        end else if (is_addfam) begin
            cf_cand = cout_msb;
        end else if (is_subfam) begin
            if (op == ALU_NEG) cf_cand = (dst != 32'h0000_0000);
            else               cf_cand = ~cout_msb;
        end
        f2[0] = update_carry ? cf_cand : flags[0];

        if (op == ALU_DAA) begin
            f2[4]  = daa_low_adj;
            f2[0]  = daa_high_adj || daa_low_carry;
            f2[11] = 1'b0;
        end else if (op == ALU_DAS) begin
            f2[4]  = daa_low_adj;
            f2[0]  = daa_high_adj || das_low_borrow;
            f2[11] = 1'b0;
        end else if (op == ALU_AAA || op == ALU_AAS) begin
            f2[4] = aaa_cond;
            f2[0] = aaa_cond;
        end
    end
end

assign flags_out = f2;

endmodule
