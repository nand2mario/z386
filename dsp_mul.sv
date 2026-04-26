//=============================================================================
// DSP-Based 32x32 Multiplier
//=============================================================================
// Uses four 17x17 signed multipliers (fits in FPGA DSP blocks like DSP48).
// Computes 32x32->64 bit multiplication in 4 cycles instead of 32.
//
module dsp_mul (
    input  logic        clk,
    input  logic        reset_n,

    // Control
    input  logic        start,          // Start multiplication
    input  logic [1:0]  op_size,        // 0=byte, 1=word, 2=dword
    input  logic        is_signed,      // IMUL vs MUL

    // Operands
    input  logic [31:0] multiplicand,   // MULTMP
    input  logic [31:0] multiplier,     // TMPB

    // Result - positioned for z386 compatibility
    output logic [63:0] product,        // Full 64-bit result
    output logic        done,           // Result valid for this cycle
    output logic        active          // Multiplication in progress (only for 32-bit)
);

    wire [15:0] Alo = multiplicand[15:0];
    wire [15:0] Ahi = multiplicand[31:16];
    wire [15:0] Blo = multiplier[15:0];
    wire [15:0] Bhi = multiplier[31:16];

    // Lower halves are always zero-extended (they represent positive values)
    // Upper halves are sign-extended only for signed multiplication
    wire signed [16:0] Alo_s = {1'b0, Alo};  // Always positive
    wire signed [16:0] Blo_s = {1'b0, Blo};  // Always positive
    wire signed [16:0] Ahi_s = is_signed ? {Ahi[15], Ahi} : {1'b0, Ahi};
    wire signed [16:0] Bhi_s = is_signed ? {Bhi[15], Bhi} : {1'b0, Bhi};

    // DSP multiplication
    wire signed [33:0] p0 = Alo_s * Blo_s;  // Always positive (both inputs positive)
    wire signed [33:0] p1 = Alo_s * Bhi_s;  // Sign depends on Bhi
    wire signed [33:0] p2 = Ahi_s * Blo_s;  // Sign depends on Ahi
    wire signed [33:0] p3 = Ahi_s * Bhi_s;  // Sign depends on both

    // 8-bit: sign-extend to 17 bits
    wire signed [16:0] a8_s = is_signed ? {{9{multiplicand[7]}}, multiplicand[7:0]}
                                        : {9'b0, multiplicand[7:0]};
    wire signed [16:0] b8_s = is_signed ? {{9{multiplier[7]}}, multiplier[7:0]}
                                        : {9'b0, multiplier[7:0]};
    wire signed [33:0] p8 = a8_s * b8_s;

    // 16-bit: sign-extend to 17 bits
    wire signed [16:0] a16_s = is_signed ? {multiplicand[15], multiplicand[15:0]}
                                         : {1'b0, multiplicand[15:0]};
    wire signed [16:0] b16_s = is_signed ? {multiplier[15], multiplier[15:0]}
                                         : {1'b0, multiplier[15:0]};
    wire signed [33:0] p16 = a16_s * b16_s;

    reg [1:0] cycle_count;
    reg [63:0] acc;

    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            cycle_count <= 2'd0;
            acc <= 64'd0;
            active <= 1'b0;
        end else if (start && cycle_count == 2'd0 && op_size == 2'd2) begin
            // Cycle 0: Start 32-bit multiply, load p0
            acc <= {32'd0, p0[31:0]};  // p0 is always positive, take lower 32 bits
            cycle_count <= 2'd1;
            active <= 1'b1;
        end else if (cycle_count == 2'd1) begin
            // Cycle 1: Add p1 << 16 (sign-extend p1 to 48 bits, then shift)
            acc <= acc + {{16{p1[33]}}, p1, 16'b0};
            cycle_count <= 2'd2;
        end else if (cycle_count == 2'd2) begin
            // Cycle 2: Add p2 << 16
            acc <= acc + {{16{p2[33]}}, p2, 16'b0};
            cycle_count <= 2'd3;
        end else if (cycle_count == 2'd3) begin
            // Cycle 3: Add p3 << 32
            acc <= acc + {p3[31:0], 32'b0};
            cycle_count <= 2'd0;
            active <= 1'b0;
        end
    end

    reg done_r;
    always_ff @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            done_r <= 1'b0;
        end else begin
            // Done pulses high when:
            // - 8/16-bit: one cycle after start
            // - 32-bit: when cycle_count transitions from 3 to 0
            done_r <= (start && op_size != 2'd2) || (cycle_count == 2'd3);
        end
    end

    assign done = done_r;

    // The original shift-and-add positions results as:
    //   8-bit:  bits [39:24] contain the 16-bit product (upper in [39:32], lower in [31:24])
    //   16-bit: bits [47:16] contain the 32-bit product (upper in [47:32], lower in [31:16])
    //   32-bit: bits [63:0] contain the 64-bit product (upper in [63:32], lower in [31:0])
    always_comb begin
        case (op_size)
            2'd0: begin  // 8-bit
                // p8[15:0] contains the 16-bit product
                product = {24'b0, p8[15:8], p8[7:0], 24'b0};
            end
            2'd1: begin  // 16-bit
                // p16[31:0] contains the 32-bit product
                product = {16'b0, p16[31:16], p16[15:0], 16'b0};
            end
            default: begin  // 32-bit
                // acc contains the 64-bit product (valid when done)
                product = acc;
            end
        endcase
    end

endmodule
