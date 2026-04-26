//
// 80386 ALU and flags logic
//
module alu
    import z386_pkg::*;
(
    input  [4:0]  op,           // ALUOPC
    input  [31:0] src,
    input  [31:0] dst,
    input  [1:0]  op_size,       // 0=byte, 1=word, 2=dword
    input  [31:0] flags,
    input         update_carry,  // gate CF updates

    output [31:0] result,
    output [31:0] flags_out
);

// -----------------------------------------------------------------------------
// Per-bit ALU slice
// -----------------------------------------------------------------------------
function [1:0] alu_slice_fn;
    input arg1;
    input arg2;
    input carry_in;
    input ctrl_c0;
    input ctrl_c1;
    input ctrl_00;
    input ctrl_01;
    input ctrl_10;
    input ctrl_11;

    reg carry_gen;
    reg carry_prop;
begin
    // Select terms based on {arg1,arg2}
    case ({arg1, arg2})
        2'b00: begin
            carry_gen  = 1'b0;
            carry_prop = ctrl_00;
        end
        2'b01: begin
            carry_gen  = 1'b0;
            carry_prop = ctrl_01;
        end
        2'b10: begin
            carry_gen  = ctrl_c0;
            carry_prop = ctrl_10;
        end
        2'b11: begin
            carry_gen  = ctrl_c1;
            carry_prop = ctrl_11;
        end
    endcase

    // {carry_out, result}
    alu_slice_fn[1] = carry_gen | (carry_prop & carry_in);
    alu_slice_fn[0] = carry_prop ^ carry_in;
end
endfunction

// ----------------------------------------------------------------
// Control signals common to all slices for this op
// ----------------------------------------------------------------
reg        ctrl_c0, ctrl_c1;
reg        ctrl_00, ctrl_01, ctrl_10, ctrl_11;
reg [31:0] arg1_bus;
reg [31:0] arg2_bus;
reg        carry_in0;

// use_override: when 1, bypass slices and use result_override
reg [31:0] result_override;
reg        use_override;

`define CTRL(x) {ctrl_c0, ctrl_c1, ctrl_00, ctrl_01, ctrl_10, ctrl_11} = x;

// ----------------------------------------------------------------
// Configure control/bus signals from op/src/dst/flags
// ----------------------------------------------------------------
wire is_byte = (op_size == 2'd0);
wire is_word = (op_size == 2'd1);
wire is_dword = (op_size == 2'd2);

// Operation family classification (reduces control decode logic depth)
wire is_add_family = (op == ALU_ADD) || (op == ALU_ADC) || (op == ALU_INC) || (op == ALU_INC2);
wire is_sub_family = (op == ALU_SUBT) || (op == ALU_SBB) || (op == ALU_CMP) ||
                      (op == ALU_DEC) || (op == ALU_DEC2) || (op == ALU_NEG);
wire is_logic_family = (op == ALU_AND) || (op == ALU_OR) || (op == ALU_XOR) || (op == ALU_ANDN);

// Decimal/ASCII adjust helper signals (operate on AL from dst, which contains SIGMA)
wire daa_low_adj  = (dst[3:0] > 4'd9) || flags[4];
wire daa_high_adj = (dst[7:0] > 8'h99) || flags[0];
// For DAA: carry from low nibble adjustment when AL + 6 > 0xFF
wire daa_low_carry = daa_low_adj && (dst[7:0] > 8'hF9);
// For DAS: borrow from low nibble adjustment when AL < 6 and we're subtracting 6
wire das_low_borrow = daa_low_adj && (dst[7:0] < 8'h06);
wire [7:0] daa_adj8 = ({8{daa_low_adj}}  & 8'h06) |
                        ({8{daa_high_adj}} & 8'h60);
wire aaa_cond = (dst[3:0] > 4'd9) || flags[4];
wire [31:0] aaa_adj = aaa_cond ? 32'h0000_0106 : 32'h0;

always @* begin
    // Flat single-level case: sets CTRL, arg buses, carry, and override in one decode
    // Defaults: PASS2 (pass dst), no override
    `CTRL(6'b000011);
    arg1_bus        = src;
    arg2_bus        = dst;
    carry_in0       = 1'b0;
    result_override = dst;
    use_override    = 1'b0;

    case (op)
        // --- Arithmetic (adder CTRL: c0=0,c1=1,00=0,01=1,10=1,11=0) ---
        ALU_ADD: begin
            `CTRL(6'b010110);
        end
        ALU_ADC: begin
            `CTRL(6'b010110);
            carry_in0 = flags[0];
        end
        ALU_SUBT, ALU_CMP: begin
            `CTRL(6'b010110);
            arg1_bus  = dst;
            arg2_bus  = ~src;
            carry_in0 = 1'b1;
        end
        ALU_SBB: begin
            `CTRL(6'b010110);
            arg1_bus  = dst;
            arg2_bus  = ~src;
            carry_in0 = ~flags[0];
        end
        ALU_INC: begin
            `CTRL(6'b010110);
            arg1_bus  = dst;
            arg2_bus  = 32'h0;
            carry_in0 = 1'b1;
        end
        ALU_DEC: begin
            `CTRL(6'b010110);
            arg1_bus  = dst;
            arg2_bus  = ~32'h1;
            carry_in0 = 1'b1;
        end
        ALU_INC2: begin
            `CTRL(6'b010110);
            arg1_bus  = dst;
            arg2_bus  = 32'h2;
        end
        ALU_DEC2: begin
            `CTRL(6'b010110);
            arg1_bus  = dst;
            arg2_bus  = ~32'h2;
            carry_in0 = 1'b1;
        end
        ALU_NEG: begin
            `CTRL(6'b010110);
            arg1_bus  = 32'h0;
            arg2_bus  = ~dst;
            carry_in0 = 1'b1;
        end

        // --- Decimal adjust ---
        ALU_DAA: begin
            `CTRL(6'b010110);
            arg1_bus  = dst;
            arg2_bus  = {24'h0, daa_adj8};
        end
        ALU_DAS: begin
            `CTRL(6'b010110);
            arg1_bus  = dst;
            arg2_bus  = ~{24'h0, daa_adj8};
            carry_in0 = 1'b1;
        end
        ALU_AAA: begin
            `CTRL(6'b010110);
            arg1_bus  = dst;
            arg2_bus  = aaa_adj;
        end
        ALU_AAS: begin
            `CTRL(6'b010110);
            arg1_bus  = dst;
            arg2_bus  = ~aaa_adj;
            carry_in0 = 1'b1;
        end

        // --- Logic ---
        ALU_AND:  begin `CTRL(6'b000001); end
        ALU_OR:   begin `CTRL(6'b000111); end
        ALU_XOR:  begin `CTRL(6'b000110); end
        ALU_ANDN: begin `CTRL(6'b000100); end

        // --- Pass-through ---
        ALU_PASS:  begin `CTRL(6'b000101); end
        ALU_PASS2: begin /* default CTRL 000011 */ end

        // --- NOT (override path) ---
        ALU_NOT: begin
            result_override = ~dst;
            use_override    = 1'b1;
        end

        // --- Extensions (override path) ---
        ALU_ZEXT: begin
            use_override = 1'b1;
            result_override = is_word ? {24'h0, dst[7:0]} :
                              is_dword ? {16'h0, dst[15:0]} : dst;
        end
        ALU_SEXT: begin
            use_override = 1'b1;
            result_override = is_word ? {{24{dst[7]}}, dst[7:0]} :
                              is_dword ? {{16{dst[15]}}, dst[15:0]} : dst;
        end
        ALU_ZEXT_B: begin
            use_override    = 1'b1;
            result_override = {24'h0, dst[7:0]};
        end
        ALU_SEXT_B: begin
            use_override    = 1'b1;
            result_override = {{24{dst[7]}}, dst[7:0]};
        end
        ALU_SEXTD: begin
            use_override = 1'b1;
            result_override = is_dword ? (dst[31] ? 32'hFFFF_FFFF : 32'h0) :
                                         (dst[15] ? 32'h0000_FFFF : 32'h0);
        end
        ALU_SIGN: begin
            use_override = 1'b1;
            result_override = is_byte  ? {32{dst[7]}} :
                              is_word  ? {32{dst[15]}} :
                                         {32{dst[31]}};
        end

        default: ;
    endcase
end

// ----------------------------------------------------------------
// 32 slices with ripple carry (FPGA maps to dedicated carry chain)
// ----------------------------------------------------------------
wire [31:0] slice_result;
wire [31:0] slice_carry;

genvar i;
generate
    for (i = 0; i < 32; i = i + 1) begin : GEN_ALU_SLICES
        wire cin_i = (i == 0) ? carry_in0 : slice_carry[i-1];
        wire [1:0] fo = alu_slice_fn(
            arg1_bus[i],
            arg2_bus[i],
            cin_i,
            ctrl_c0,
            ctrl_c1,
            ctrl_00,
            ctrl_01,
            ctrl_10,
            ctrl_11
        );
        assign slice_result[i] = fo[0];
        assign slice_carry[i]  = fo[1];
    end
endgenerate

wire [31:0] alu_raw = use_override ? result_override : slice_result;
wire [31:0] alu_post = (op == ALU_AAA || op == ALU_AAS) ? {alu_raw[31:8], 4'h0, alu_raw[3:0]} :
                        (op == ALU_DAA || op == ALU_DAS) ? {dst[31:8], alu_raw[7:0]} :
                        alu_raw;

assign result = alu_post;  // CMP computes dst-src; microcode controls dest write

// ----------------------------------------------------------------
// 80386 flags (subset)
// ----------------------------------------------------------------
wire is_logic  = (op == ALU_AND) || (op == ALU_OR) || (op == ALU_XOR) || (op == ALU_ANDN);
wire is_extend = (op == ALU_ZEXT) || (op == ALU_SEXT);  // MOVZX/MOVSX don't modify flags
wire is_not    = (op == ALU_NOT);  // NOT doesn't modify any flags
// NOTE: Shifts are handled by the dedicated shifter module, not the ALU
wire is_inc    = (op == ALU_INC);
wire is_dec    = (op == ALU_DEC);
wire is_addfam = (op == ALU_ADD) || (op == ALU_ADC) || (op == ALU_INC) || (op == ALU_INC2);
wire is_subfam = (op == ALU_SUBT) || (op == ALU_SBB) || (op == ALU_CMP) || (op == ALU_DEC) || (op == ALU_DEC2) || (op == ALU_NEG);
wire is_adjust = (op == ALU_DAA) || (op == ALU_DAS) || (op == ALU_AAA) || (op == ALU_AAS);

wire [31:0] R = slice_result;
wire flag_byte_mode = is_byte || is_adjust;

// Per-size flag signals — fixed indices instead of dynamic R[msb_idx] (avoids 32:1 mux)
wire r_msb = flag_byte_mode ? R[7] : (is_word ? R[15] : R[31]);
wire cout_3 = slice_carry[3];

// OF: cin_msb ^ cout_msb per size (XOR of adjacent carries, then 3:1 mux)
wire of_byte  = slice_carry[6]  ^ slice_carry[7];
wire of_word  = slice_carry[14] ^ slice_carry[15];
wire of_dword = slice_carry[30] ^ slice_carry[31];
wire of_arith = flag_byte_mode ? of_byte : (is_word ? of_word : of_dword);

// CF: carry out of MSB per size
wire cf_byte  = slice_carry[7];
wire cf_word  = slice_carry[15];
wire cf_dword = slice_carry[31];
wire cout_msb = flag_byte_mode ? cf_byte : (is_word ? cf_word : cf_dword);

reg [31:0] f2;
always @* begin
    reg cf_cand;

    f2 = flags;
    cf_cand = flags[0];

    if (is_extend || is_not) begin
        // No flag updates for extension operations or NOT
        f2 = flags;
    end else begin
        // SF, ZF, PF - standard flag updates (not for AAA/AAS)
        if (!(op == ALU_AAA || op == ALU_AAS)) begin
            f2[7] = r_msb;
            // Use hierarchical zero flag computation
            if (is_dword)
                f2[6] = |R == 0;
            else if (is_word)
                f2[6] = |R[15:0] == 0;
            else
                f2[6] = |R[7:0] == 0;
            // PF is always computed on low 8 bits regardless of operand size
            f2[2] = ~^R[7:0];
        end

        // OF - overflow flag (pre-computed per-size, no dynamic indexing)
        f2[11] = of_arith;

        // AF - auxiliary carry flag
        if (is_addfam) begin
            f2[4] = cout_3;
        end else if (is_subfam) begin
            f2[4] = ~cout_3;
        end else if (op == ALU_NOT || op == ALU_PASS || op == ALU_PASS2) begin
            f2[4] = flags[4];
        end else begin
            f2[4] = 1'b0;
        end

        // CF - carry flag
        cf_cand = flags[0];
        if (is_logic) begin
            cf_cand = 1'b0;
        end else if (is_addfam) begin
            cf_cand = cout_msb;
        end else if (is_subfam) begin
            if (op == ALU_NEG) cf_cand = (dst[31:0] != 32'h0000_0000);  // NEG: CF=1 if original != 0
            else               cf_cand = ~cout_msb;
        end
        f2[0] = update_carry ? cf_cand : flags[0];

        // Special handling for decimal adjust instructions
        if (op == ALU_DAA) begin
            f2[4]  = daa_low_adj;
            f2[0]  = daa_high_adj || daa_low_carry;  // CF includes carry from +6
            f2[11] = 1'b0;
        end else if (op == ALU_DAS) begin
            f2[4]  = daa_low_adj;
            f2[0]  = daa_high_adj || das_low_borrow;  // CF includes borrow from -6
            f2[11] = 1'b0;
        end else if (op == ALU_AAA || op == ALU_AAS) begin
            f2[4]  = aaa_cond;
            f2[0]  = aaa_cond;
        end
    end
end

assign flags_out = f2;

endmodule
