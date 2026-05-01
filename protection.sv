//
// 80386 Protection Unit, including the Test PLA
//
// Implements hardware-accelerated privilege checking and protection validation
// for protected mode operations. The tests performed here directly corresponds
// to PLA4 rom on the 80386 die.
//
// The protection unit is invoked by specific microcode ALU operations in the
// range 0x60 to 0x6F.
module protection_unit
    import z386_pkg::*;
(
    // Clock and reset (for 2-stage pipeline)
    input               clk,
    input               reset_n,
    input               pipe_en,          // Pipeline advance enable (matches ROM latch condition)

    // Test Control (alu_src field of microcode)
    input        [5:0]  test_const,       // Test constant (ABCDEF field), selects protection test type
    input        [3:0]  aluop_type,       // Lower 4 bits of aluop (TUVWXYZ[3:0]), controls Tiny PLA mux

    // Narrowed descriptor attribute bundle used by the protection datapath.
    // This avoids forwarding an entire 32-bit descriptor word when only a
    // small subset of bits participates in the critical path.
    input               descriptor_g,
    input               descriptor_p,
    input        [1:0]  descriptor_dpl,
    input               descriptor_s,
    input        [3:0]  descriptor_type,
    input        [1:0]  descriptor_rpl,
    input               descriptor_low16_nonzero,

    // Selector fields (from segment register or temp register)
    input        [1:0]  selector_rpl,     // Requested Privilege Level (bits[1:0])
    input               selector_ti,      // Table Indicator (0=GDT, 1=LDT)
    input               selector_null,    // Null selector (TI=0 AND Index=0)
    input               selector_oob,     // Selector exceeds GDT/LDT limit

    // Processor State
    input        [1:0]  cpl,              // Current Privilege Level (CS[1:0])
    input               pe_mode,          // Protected mode enabled (CR0.PE)

    // CR0 flags for FPU tests (multiplexed into state vector)
    input               cr0_et,           // CR0.ET (1=287, 0=387)
    input               cr0_ts,           // CR0.TS (Task Switched)
    input               cr0_em,           // CR0.EM (Emulation)
    input               cr0_mp,           // CR0.MP (Monitor Coprocessor)

    // ARPL support: latched source RPL from READ_RPL
    input        [1:0]  arpl_rpl,         // Source selector RPL (latched by READ_RPL at uc=6B6)


    input               test_en,          // Enable protection test

    // Test mode for verification (bypasses Tiny PLA preprocessing)
    input               test_mode,        // 1 = Direct state vector mode (for testbench)
                                          // 0 = Normal mode (full descriptor processing)
    input        [9:0]  test_state_vector,// Direct state vector input (test mode only)
                                          // Format: [9:0] = {p1, p2, b13, b12, p, u, x, ce, rw, a}

    //--------------------------------------------------------------------------
    // Outputs (to microcode sequencer and control logic)
    //--------------------------------------------------------------------------
    output       [11:0] jump_addr,        // Microcode jump address
    output              jump_valid,       // jump_addr is a redirect (vs 0x000 = CONTINUE)

    output              stack_op,         // K flag: Stack operation / update CPL
    output              limit_check,      // L flag: Perform limit/type validation
    output              validation_ok,    // M flag: Descriptor validated, safe to commit
    output              set_accessed,     // N flag: Set Accessed bit in descriptor

    output              result_valid,     // Pipelined result is valid (fires 2 cycles after test)
    output              is_checking_test  // Result is from a "checking" test (not PTGEN action)
);

//==============================================================================
// Internal Signals
//==============================================================================
// Protection test constants are now defined in z386_pkg.sv

// Combinational Tiny PLA outputs (computed same cycle as test)
logic [9:0] state_vector_comb;

// Combinational privilege flags (inverted to match query.py: p1=1 in query = no violation)
logic       p1_comb;     // bit 15: ~Privilege violation
logic       p2_comb;     // bit 14: ~Privilege levels differ

// Combinational descriptor attributes (multiplexed: segment type bits OR CR0 flags for FPU)
// Values match query.py convention: 1 = attribute IS set (e.g., p=1 means present)
logic       b13_comb;      // bit 13: unknown (always 0)
logic       b12_comb;      // bit 12: unused  (always 0)
logic       p_comb;        // bit 11: Present (or Granularity for TST_DES_GRANUL) / unused for FPU
logic       u_comb;        // bit 10: S bit (system/user segment) / CR0.ET for FPU
logic       x_comb;        // bit  9: Type[3] Executable / CR0.TS for FPU
logic       ce_comb;       // bit  8: Type[2] Conforming/Expand-down / CR0.EM for FPU
logic       rw_comb;       // bit  7: Type[1] Readable/Writable / CR0.MP for FPU
logic       a_comb;        // bit  6: Type[0] Accessed / unused for FPU

localparam bit TRACE_PROT_EN = 1'b0;

//==============================================================================
// Stage 1 Pipeline Registers (Tiny PLA outputs → PLA4 inputs)
//==============================================================================
// Registered versions of Tiny PLA outputs, used by Main PLA4
logic       p1;
logic       p2;
logic       b13;
logic       b12;
logic       p;
logic       u;
logic       x;
logic       ce;
logic       rw;
logic       a;

logic [5:0] s1_test_const;      // Pipelined test constant
logic       s1_valid;            // Stage 1 has valid data
logic       s1_is_checking_test; // Stage 1 is a "checking" test (not PTGEN)
logic [1:0] s1_arpl_rpl;        // Pipelined ARPL source RPL
logic [1:0] s1_desc_rpl;        // Pipelined descriptor_hi[1:0] (dest RPL for ARPL)
logic [1:0] s1_cpl;             // Pipelined CPL for CPL guards
logic [1:0] s1_desc_dpl;        // Pipelined descriptor DPL for CPL guards

//==============================================================================
// Stage 2 Pipeline Registers (PLA4 outputs)
//==============================================================================
logic [11:0] s2_jump_addr;
logic        s2_jump_valid;
logic [3:0]  s2_flags;
logic        s2_valid;
logic        s2_is_checking_test;
logic [5:0]  s2_test_const;     // For debug display

// PLA4 input (16 bits)
logic [15:0] pla_test_input;

// PLA4 output (18 bits)
logic [17:0] pla_test_output;

// Descriptor field extraction
wire       desc_g    = descriptor_g;
wire       desc_p    = descriptor_p;
wire [1:0] desc_dpl  = descriptor_dpl;
wire [3:0] desc_type = descriptor_type;
wire       desc_s    = descriptor_s;   // 1=code/data, 0=system
wire       desc_x    = desc_type[3];        // Executable (1=code, 0=data)
wire       desc_ce   = desc_type[2];        // Conforming (code) / Expand-down (data)
wire       desc_rw   = desc_type[1];        // Readable (code)   / Writable (data)
wire       desc_a    = desc_type[0];        // Accessed

//==============================================================================
// Tiny PLA Logic (Pre-processor)
//==============================================================================

// State bit computation (combinational — Tiny PLA)
always_comb begin
    if (test_mode) begin
        // Test mode: Use state vector directly (bypass Tiny PLA)
        p1_comb     = test_state_vector[9];
        p2_comb = test_state_vector[8];
        b13_comb  = test_state_vector[7];
        b12_comb  = test_state_vector[6];
        p_comb   = test_state_vector[5];
        u_comb    = test_state_vector[4];
        x_comb    = test_state_vector[3];
        ce_comb        = test_state_vector[2];
        rw_comb   = test_state_vector[1];
        a_comb   = test_state_vector[0];
    end else begin
        // Normal mode: Compute state vector from descriptor (Tiny PLA)
        //
        // The lower 4 bits of alujmp_op control muxes that select which
        // pre-computed privilege signals feed into PLA4 input bits [15:14].
        //
        // The following logic is probably incomplete. We'll get the complete
        // logic by debugging each protection test.
        //
        // Different aluops route different signals:
        //   TSTDES (0x8): inside LD_DESCRIPTOR — RPL-only check
        //     bit15 = (RPL > DPL)
        //     bit14 = (RPL != CPL) | (DPL != CPL)
        //
        //   TSTGT (0x7), TSTGT2 (0x9), TSTPRV (0xA), TSTPM (0x3), TSTINT (0xF):
        //     bit15 = (RPL > DPL) | (CPL > DPL)
        //     bit14 = (RPL != CPL) | (DPL != CPL)
        //
        //   TSTLD (0xE):
        //     bit15 = (RPL <= DPL) & (CPL <= DPL)  (inverted)
        //     bit14 = (RPL != CPL)

        if (aluop_type == 4'hE) begin  // PTSELE
            // PTSELE (0x6E): selector-only test — p1/p2 use RPL vs CPL only, not DPL.
            // SPTR sets the descriptor cache pointer but doesn't pre-read the GDT entry,
            // so no descriptor DPL is available. The actual descriptor privilege check
            // happens later in PTOVRR inside LD_DESCRIPTOR.
            p1_comb = (selector_rpl < cpl);     // violation when RPL < CPL
            p2_comb = ~(selector_rpl != cpl);   // 1=match, 0=mismatch
        end else if (aluop_type == 4'hF) begin   // PTF
            // PTF (0x6F): direct privilege comparison using CPL vs DPL.
            // Used by TST_DES_RTOLOS (null segment if CPL > DPL),
            // TST_DES_INT_SW (!p1 = CPL <= DPL for software INT gate validation),
            // and others (INT_HW, GRANUL, READ_RPL, COPY_STACK_DPL) that don't check p1/p2.
            p1_comb = (cpl > desc_dpl);          // privilege violation
            p2_comb = (desc_dpl == cpl);          // same privilege level
        end else if (aluop_type == 4'hA) begin   // PTSELA
            // PTSELA (0x6A): selector-only test — no descriptor available yet.
            // Used by TST_SEL_MOREPR (uc=607), TST_SEL_GDT (uc=667), etc.
            // Compare selector RPL against CPL only (no DPL).
            p1_comb = (selector_rpl > cpl);     // Not used by current PLA4 terms for PTSELA
            p2_comb = ~(selector_rpl != cpl);   // 1=match (RPL == CPL), 0=mismatch
        end else if (aluop_type == 4'h8) begin   // PTOVRR
            // TSTDES (0x68): inside LD_DESCRIPTOR — descriptor privilege check
            // p1: RPL vs DPL violation (0=ok, 1=violation)
            // p2: DPL vs CPL match (1=same privilege, 0=different privilege)
            // Note: p2 uses only DPL vs CPL, not selector_rpl. Per Intel manual,
            // "the RPL of the new code segment selector is not used in the privilege
            // check" for INT/CALL gates. SET_RPL_TO_CPL later forces RPL=CPL.
            p1_comb = (selector_rpl > desc_dpl);
            p2_comb = ~(desc_dpl != cpl);
        end else begin
            // TSTGT, TSTGT2, TSTPM, TSTPRV, TSTINT, TSTJ, etc.
            p1_comb = (selector_rpl > desc_dpl) | (cpl > desc_dpl);
            p2_comb = ~((selector_rpl != cpl) | (desc_dpl != cpl));
        end

        // Check if this is an FPU test (test_const 0x34-0x3F)
        if (test_const[5:4] == 2'b11 && test_const[3:2] != 2'b00) begin
            // FPU test: CR0 flags mapped to state vector (direct, matches query.py)
            b13_comb  = 1'b0;         // bit13: unused for FPU
            b12_comb  = 1'b0;         // bit12: unused for FPU
            p_comb    = 1'b0;         // bit11: unused for FPU
            u_comb    = cr0_et;       // bit10: CR0.ET (287 vs 387)
            x_comb    = cr0_ts;       // bit 9: CR0.TS (Task Switched)
            ce_comb   = cr0_em;       // bit 8: CR0.EM (Emulation)
            rw_comb   = cr0_mp;       // bit 7: CR0.MP (Monitor Coprocessor)
            a_comb    = 1'b0;         // bit 6: unused for FPU
        end else begin
            // Segment/Gate test: Direct descriptor type bits (matches query.py)
            b13_comb  = 1'b0;         // bit13: unknown/unused
            b12_comb  = 1'b0;         // bit12: unused
            // bit11 (p): For PTSELE/PTSELA (aluop 0xE/0xA), p = selector_null.
            // For GRANUL test, p = G bit. Otherwise p = Present bit.
            if (aluop_type == 4'hE || aluop_type == 4'hA)
                p_comb = selector_null;
            else if (test_const == 6'h20)
                p_comb = desc_g;
            else
                p_comb = desc_p;
            // For PTSELA (aluop 0xA): selector test — remap ce to TI bit,
            // clear other descriptor bits (no descriptor read yet)
            if (aluop_type == 4'hA) begin
                // if (test_const == TST_PORTIO_BIT) begin
                //     // IO permission bitmap check: PROTUN holds (bitmap & mask).
                //     // Pass (no fault) when PROTUN == 0 (all tested port bits clear).
                    // p1_comb = 1'b1;
                    // p2_comb = (descriptor_hi == 32'h0);
                    // p_comb  = (descriptor_hi == 32'h0);
                // end
                u_comb  = 1'b0;
                x_comb  = 1'b0;
                ce_comb = selector_ti;
                rw_comb = 1'b0;
                a_comb  = 1'b0;
            end else begin
                u_comb    = desc_s;       // bit10: S bit (0=system, 1=code/data)
                x_comb    = desc_x;       // bit 9: Type[3] Executable
                ce_comb   = desc_ce;      // bit 8: Type[2] Conforming/Expand-down
                rw_comb   = desc_rw;      // bit 7: Type[1] Readable/Writable
                a_comb    = desc_a;       // bit 6: Type[0] Accessed
            end
        end
    end  // End of normal mode (else begin)
end

// Pack combinational state vector
assign state_vector_comb = {p1_comb, p2_comb, b13_comb, b12_comb, p_comb,
                            u_comb, x_comb, ce_comb, rw_comb, a_comb};

//==============================================================================
// Stage 1: Register Tiny PLA outputs (posedge clk when pipe_en)
//==============================================================================
// is_checking_test: true for protection tests that control LJUMP suppression
// PTGEN (0xB) is an "action" test (SET_RPL_TO_CPL, SET_FAULT, etc.) — NOT a checking test
wire s0_is_checking = test_en && (aluop_type != 4'hB);

always_ff @(posedge clk) begin
    if (!reset_n) begin
        s1_valid <= 1'b0;
        s1_is_checking_test <= 1'b0;
        s1_test_const <= 6'h0;
        s1_arpl_rpl <= 2'b0;
        s1_desc_rpl <= 2'b0;
        s1_cpl <= 2'b0;
        s1_desc_dpl <= 2'b0;
        p1 <= 1'b0;
        p2 <= 1'b0;
        b13 <= 1'b0;
        b12 <= 1'b0;
        p <= 1'b0;
        u <= 1'b0;
        x <= 1'b0;
        ce <= 1'b0;
        rw <= 1'b0;
        a <= 1'b0;
    end else if (pipe_en) begin
        s1_valid <= test_en;
        s1_is_checking_test <= s0_is_checking;
        s1_test_const <= test_const;
        s1_arpl_rpl <= arpl_rpl;
        s1_desc_rpl <= descriptor_rpl;
        s1_cpl <= cpl;
        s1_desc_dpl <= desc_dpl;
        p1 <= p1_comb;
        p2 <= p2_comb;
        b13 <= b13_comb;
        b12 <= b12_comb;
        p <= p_comb;
        u <= u_comb;
        x <= x_comb;
        ce <= ce_comb;
        rw <= rw_comb;
        a <= a_comb;
    end
end

// PLA4 input uses registered state vector + registered test_const
assign pla_test_input = {p1, p2, b13, b12, p,
                     u, x, ce, rw, a, s1_test_const};

//==============================================================================
// Main PLA4 Logic (Protection Decision)
//==============================================================================
always_comb begin
    // Default: test passes (continue execution)
    pla_test_addr  = 12'h000;
    pla_test_flags = 4'b0000;

    case (s1_test_const)
        //----------------------------------------------------------------------
        // TST_SEL_NONSS (0x00) - Test Non-Stack Segment Load (DS/ES/FS/GS)
        //----------------------------------------------------------------------
        // Validates data segment loads, checking privilege and type
        TST_SEL_NONSS: begin
            if (p) begin
                // p (present) → fault handler
                pla_test_addr = 12'h592;
            end
            if (!p)
                pla_test_flags = 4'b0100;
            // else: 0x000 (pass)
        end

        //----------------------------------------------------------------------
        // TST_SEL_CS (0x01) - Code Segment Selector
        //----------------------------------------------------------------------
        TST_SEL_CS: begin
            // Term 1: !p1 p !u x ce rw !a → 0x009
            if (!p1 && p && !u && x && ce && rw && !a)
                pla_test_addr = pla_test_addr | 12'h009;
            // Term 65: !p → L flag
            if (!p)
                pla_test_flags = 4'b0100;
            // Term 143: p → 0x85D
            if (p)
                pla_test_addr = pla_test_addr | 12'h85D;
        end

        //----------------------------------------------------------------------
        // TST_SEL_RET (0x02) - Test Far Return (privilege transition)
        //----------------------------------------------------------------------
        TST_SEL_RET: begin
            // Term 75: !p1 !p2 !p → 0x686 (cross-privilege return)
            if (!p1 && !p2 && !p)
                pla_test_addr = pla_test_addr | 12'h686;
            // Term 118: p1 !p2 → 0x85D
            // Terms 144-146: p → 0x85D
            if (p1 && !p2 || p)
                pla_test_addr = pla_test_addr | 12'h85D;
            // Term 39: !p1 !p → L flag
            if (!p1 && !p)
                pla_test_flags = 4'b0100;
        end

        //----------------------------------------------------------------------
        // TST_SEL_RET_OL (0x03) - Far Return Outer Level Selector
        //----------------------------------------------------------------------
        TST_SEL_RET_OL: begin
            // Term 118: p1 !p2 → 0x85D
            if (p1 && !p2)
                pla_test_addr = pla_test_addr | 12'h85D;
            // Term 128: !p2 → 0x85D
            if (!p2)
                pla_test_addr = pla_test_addr | 12'h85D;
            // Term 146: p → 0x85D
            if (p)
                pla_test_addr = pla_test_addr | 12'h85D;
        end

        //----------------------------------------------------------------------
        // TST_PORTIO_BIT (0x04) - Port I/O Bitmap Check
        //----------------------------------------------------------------------
        TST_PORTIO_BIT: begin
            // this is a hack
            pla_test_addr = descriptor_low16_nonzero ? 12'h85B : 12'h000;
            // // Term 136: !p2 → 0x85B
            // if (!p2)
            //     pla_test_addr = pla_test_addr | 12'h85B;
            // // Term 137: !p → 0x85B
            // if (!p)
            //     pla_test_addr = pla_test_addr | 12'h85B;
        end

        //----------------------------------------------------------------------
        // TST_SEL_ARPL (0x05) - ARPL Check
        // Compares dest RPL (from descriptor_hi[1:0] at test time) against
        // latched source RPL (from READ_RPL). Adjusts if dest < source.
        //----------------------------------------------------------------------
        TST_SEL_ARPL: begin
            if (s1_desc_rpl >= s1_arpl_rpl) begin
                pla_test_addr = 12'h6B3;  // ARPL_FAILED: dest RPL >= source RPL, no adjustment
            end else begin
                pla_test_flags = 4'b0010;  // M flag: dest RPL < source RPL, adjustment needed
            end
            // else: 0x000 (pass)
        end

        //----------------------------------------------------------------------
        // TST_SEL_GDT (0x06) - GDT Selector Check
        //----------------------------------------------------------------------
        TST_SEL_GDT: begin
            // Term 142: ce → 0x85D
            if (ce)
                pla_test_addr = pla_test_addr | 12'h85D;
            // Term 145: p → 0x85D
            if (p)
                pla_test_addr = pla_test_addr | 12'h85D;
        end

        //----------------------------------------------------------------------
        // TST_SEL_LLDT (0x07) - Test Load LDT
        //----------------------------------------------------------------------
        TST_SEL_LLDT: begin
            // Term 139: p → 0x6DD
            // Term 142: ce → 0x85D
            // Both can match: 0x6DD | 0x85D = 0xEDD
            if (p) pla_test_addr = pla_test_addr | 12'h6DD;
            if (ce)      pla_test_addr = pla_test_addr | 12'h85D;
        end

        //----------------------------------------------------------------------
        // TST_SEL_LLVV (0x08) - Test LAR/LSL (Load Access Rights/Segment Limit)
        //----------------------------------------------------------------------
        TST_SEL_LLVV: begin
            if (p || selector_oob) begin
                // p (null selector) or out-of-bounds → clear ZF, end instruction
                pla_test_addr = 12'h86E;  // LAR_LSL_VERRW_NULL_SELECTOR (CLZF RNI)
            end
            if (!p && !selector_oob)
                pla_test_flags = 4'b0100;
            // else: 0x000 (pass)
        end

        //----------------------------------------------------------------------
        // TST_SEL_MOREPR (0x09) - More Privilege Selector Check
        //----------------------------------------------------------------------
        TST_SEL_MOREPR: begin
            // Term 123: !p2 → 0x85D
            if (!p2)
                pla_test_addr = pla_test_addr | 12'h85D;
            // Term 143: p → 0x85D
            if (p)
                pla_test_addr = pla_test_addr | 12'h85D;
        end

        //----------------------------------------------------------------------
        // TST_SEL_TASKGT (0x0A) - Task Gate Selector
        //----------------------------------------------------------------------
        TST_SEL_TASKGT: begin
            // Term 21: !p !ce → L flag
            if (!p && !ce)
                pla_test_flags = 4'b0100;
            // Term 134: ce → 0x85D
            if (ce)
                pla_test_addr = pla_test_addr | 12'h85D;
            // Term 144: p → 0x85D
            if (p)
                pla_test_addr = pla_test_addr | 12'h85D;
        end

        //----------------------------------------------------------------------
        // TST_SEL_TASKFI (0x0B) - Task Gate Selector (TSS read)
        //----------------------------------------------------------------------
        TST_SEL_TASKFI: begin
            // Term 21: !p !ce → L flag
            if (!p && !ce)
                pla_test_flags = 4'b0100;
            // Term 38: !p → L flag
            if (!p)
                pla_test_flags = 4'b0100;
            // Term 132: p → 0x7E5
            if (p)
                pla_test_addr = 12'h7E5;
        end

        //----------------------------------------------------------------------
        // TST_SEL_SS (0x0C) - Stack Segment Selector
        //----------------------------------------------------------------------
        TST_SEL_SS: begin
            // Term 51: !p → L flag
            if (!p)
                pla_test_flags = 4'b0100;
            // Term 122: !p2 → 0x85D
            if (!p2)
                pla_test_addr = pla_test_addr | 12'h85D;
            // Term 135: p → 0x85D
            if (p)
                pla_test_addr = pla_test_addr | 12'h85D;
        end

        //----------------------------------------------------------------------
        // TST_SEL_TR_TSF (0x0D) - Task Register Selector (TSS read)
        //----------------------------------------------------------------------
        TST_SEL_TR_TSF: begin
            // Term 124: ce → 0x85D
            if (ce)
                pla_test_addr = pla_test_addr | 12'h85D;
            // Term 129: p → 0x7E5
            if (p)
                pla_test_addr = pla_test_addr | 12'h7E5;
        end

        //----------------------------------------------------------------------
        // TST_DES_SIMPLE (0x10) - Test Non-Stack Segment Load (variant 2)
        //----------------------------------------------------------------------
        TST_DES_SIMPLE: begin
            // CPL privilege guard: on the real 386, the SPTR bus operation at the
            // segment load entry point (e.g., uc=580) reads the GDT descriptor and
            // pre-validates CPL vs DPL before LD_DESCRIPTOR runs. Since SPTR is not
            // implemented, we add the CPL check here: when CPL > DPL, suppress the
            // PLA4 "pass" redirect so execution falls through to 5D1 (GP fault).
            if (s1_cpl > s1_desc_dpl && !(x && ce)) begin
                // CPL exceeds DPL for non-conforming segment: fall through to 5D1 (#GP)
                // Conforming code segments (x=1, ce=1) allow DPL <= CPL (term 90 handles)
            end else if (!p && u && x && ce && rw) begin
                // Term 57: !p u x ce rw → SEGMENT_NOT_P1
                pla_test_addr = 12'h870;
            end else if (!p1 && !p && u && rw) begin
                // Term 81: !p1 !p u rw → SEGMENT_NOT_P1
                pla_test_addr = 12'h870;
            end else if (!p1 && !p && u && !x) begin
                // Term 82: !p1 !p u !x → SEGMENT_NOT_P1
                pla_test_addr = 12'h870;
            end else if (p && u && x && ce && rw) begin
                // Term 90: p u x ce rw → PROT_TESTS_PASSED
                pla_test_addr = 12'h5D5;
                pla_test_flags = 4'b0001;
            end else if (!p1 && p && u && rw) begin
                // Term 106: !p1 p u rw → PROT_TESTS_PASSED
                pla_test_addr = 12'h5D5;
                pla_test_flags = 4'b0001;
            end else if (!p1 && p && u && !x) begin
                // Term 107: !p1 p u !x → PROT_TESTS_PASSED
                pla_test_addr = 12'h5D5;
                pla_test_flags = 4'b0001;
            end
            // else: 0x000 (no match → fall through to 5D1 #GP)
        end

        //----------------------------------------------------------------------
        // TST_DES_SS (0x11) - Test Stack Segment Load
        //----------------------------------------------------------------------
        TST_DES_SS: begin
            if (!p1 && p2 && u && !x && rw) begin
                if (p) begin
                    // Terms 0 & 74: !p1 p2 p u !x rw → PROT_TESTS_PASSED
                    pla_test_addr = 12'h5D5;
                    pla_test_flags = 4'b0001;
                end else begin
                    // Term 40: pv, !pd, !p u !x rw → 0x86A
                    pla_test_addr = 12'h86A;
                end
            end
            if (p1 && !p2 && p && u && !x && rw)
                pla_test_flags = 4'b1000;
            // else: 0x000 (pass)
        end

        //----------------------------------------------------------------------
        // TST_DES_JMP (0x12) - Test Far Jump in Protected Mode
        //----------------------------------------------------------------------
        // This is the key gate detection test
        TST_DES_JMP: begin
            if (!u) begin
                // Term 138: !u → gate/system descriptor detected
                pla_test_addr = 12'h5B3;
            end else if (!p1 && p2 && !b13 && u && x) begin
                // Terms 18/61: !p1, p2, !?, u, x
                if (!p) begin
                    // Term 18 → 0x870
                    pla_test_addr = 12'h870;
                end else begin
                    // Term 61 → 0x5D5
                    pla_test_addr = 12'h5D5;
                    pla_test_flags = 4'b0001;
                end
            end else if (!p1 && u && x && ce) begin
                // Terms 62/92: !p1, u, x, ce
                if (!p) begin
                    // Term 62 → 0x870
                    pla_test_addr = 12'h870;
                end else begin
                    // Term 92 → 0x5D5
                    pla_test_addr = 12'h5D5;
                    pla_test_flags = 4'b0001;
                end
            end
            // else: 0x000 (pass)
        end

        //----------------------------------------------------------------------
        // TST_DES_JGATE (0x13) / TST_DES_CGATE (0x16) - Gate type dispatch
        // Same PLA4 entries for both JMP and CALL through gates
        //----------------------------------------------------------------------
        TST_DES_JGATE,
        TST_DES_CGATE: begin
            if (!p1 && !u && !rw) begin
                if (p && !x && ce && a) begin
                    // Term 13: !p1 p !u !x ce !rw a → TASKGATE
                    pla_test_addr = 12'h71C;
                end else if (!p && ce && !a) begin
                    // Term 22: !p1 !p !u ce !rw !a → Segment not present
                    pla_test_addr = 12'h870;
                end else if (!p && !x && ce) begin
                    // Term 25: !p1 !p !u !x ce !rw → not present
                    pla_test_addr = 12'h870;
                end else if (p && !ce && a) begin
                    // Term 28: !p1 p !u !ce !rw a → TSS available
                    pla_test_addr = 12'h743;
                end else if (p && x && ce && !a) begin
                    // Term 33: !p1 p !u x ce !rw !a → CALLGATE386
                    pla_test_addr = 12'h5BE;
                end else if (p && !x && ce && !a) begin
                    // Term 36: !p1 p !u !x ce !rw !a → CALLGATE286
                    pla_test_addr = 12'h5BD;
                end else if (!p && !ce && a) begin
                    // Term 50: !p1 !p !u !ce !rw a
                    pla_test_addr = 12'h7E8;
                end
            end
            // else: 0x000 (pass)
        end

        //----------------------------------------------------------------------
        // TST_DES_JGDEST (0x14) - Jump gate destination code segment
        //----------------------------------------------------------------------
        TST_DES_JGDEST: begin
            if (!p1 && p2 && u && x) begin
                // Terms 31, 49, 87: !p1, p2, u, x
                if (!p) begin
                    // Terms 31, 49 → 0x870
                    pla_test_addr = 12'h870;
                end else begin
                    // Term 87 → 0x5DA
                    pla_test_addr = 12'h5DA;
                    pla_test_flags = 4'b0001;
                end
            end else if (!p1 && u && x && ce) begin
                // Terms 73, 91: !p1, u, x, ce
                if (!p) begin
                    // Term 73 → 0x870
                    pla_test_addr = 12'h870;
                end else begin
                    // Term 91 → 0x5DA
                    pla_test_addr = 12'h5DA;
                    pla_test_flags = 4'b0001;
                end
            end
            // else: 0x000 (pass)
        end

        //----------------------------------------------------------------------
        // TST_DES_CALL (0x15) - Far Call Descriptor (gate detection)
        //----------------------------------------------------------------------
        TST_DES_CALL: begin
            // Term 125: !u → 0x5B8 (gate/system descriptor)
            if (!u)
                pla_test_addr = 12'h5B8;
            // Term 31: !p1 p2 !b13 !p u x → 0x870
            if (!p1 && p2 && !b13 && !p && u && x)
                pla_test_addr = pla_test_addr | 12'h870;
            // Term 73: !p1 !p u x ce → 0x870
            if (!p1 && !p && u && x && ce)
                pla_test_addr = pla_test_addr | 12'h870;
            // Term 54: !p1 p2 !b13 p u x → 0x5D5, K=0001
            if (!p1 && p2 && !b13 && p && u && x) begin
                pla_test_addr = pla_test_addr | 12'h5D5;
                pla_test_flags = pla_test_flags | 4'b0001;
            end
            // Term 88: !p1 p u x ce → 0x5D5, K=0001
            if (!p1 && p && u && x && ce) begin
                pla_test_addr = pla_test_addr | 12'h5D5;
                pla_test_flags = pla_test_flags | 4'b0001;
            end
        end

        //----------------------------------------------------------------------
        // TST_DES_CGDEST (0x17) - Call Gate Destination Code Segment
        //----------------------------------------------------------------------
        TST_DES_CGDEST: begin
            // CPL privilege guard: on the real 386, SPTR at uc=8bc pre-validates
            // CPL vs target DPL before LD_DESCRIPTOR runs. Since SPTR is not
            // implemented, we add the check here: when target DPL > CPL, suppress
            // PLA4 match so execution falls through to 5D1 (#GP fault).
            // INT/CALL gates forbid outward transitions (to less-privileged code).
            if (s1_desc_dpl > s1_cpl) begin
                // Target DPL exceeds CPL: illegal outward transition → #GP
            end else begin
                // Term 2: !p1 !p2 p u x !ce → 0x021, KLMN=1100
                if (!p1 && !p2 && p && u && x && !ce) begin
                    pla_test_addr = 12'h021;
                    pla_test_flags = 4'b1100;
                end
                // Term 72: !p1 !p u x → 0x870
                if (!p1 && !p && u && x)
                    pla_test_addr = pla_test_addr | 12'h870;
                // Term 102: !p1 p u x → 0x5DA, K=0001
                if (!p1 && p && u && x) begin
                    pla_test_addr = pla_test_addr | 12'h5DA;
                    pla_test_flags = pla_test_flags | 4'b0001;
                end
            end
        end

        //----------------------------------------------------------------------
        // TST_DES_RETF (0x18) - Test Far Return
        //----------------------------------------------------------------------
        TST_DES_RETF: begin
            if (!p1 && p2 && u && x) begin
                // Terms 43, 79: !p1, p2, u, x
                if (!p) begin
                    // Term 43 → 0x870  SEGMENT_NOT_P1
                    pla_test_addr = 12'h870;
                end else begin
                    // Term 79 → 0x5D5  PROT_TESTS_PASSED
                    pla_test_addr = 12'h5D5;
                    pla_test_flags = 4'b0001;
                end
            end else if (!p1 && u && x && ce) begin
                // Terms 55 & 89: !p1, u, x, ce
                if (!p) begin
                    // Term 55 → 0x870
                    pla_test_addr = 12'h870;
                end else begin
                    // Term 89 → 0x5D5
                    pla_test_addr = 12'h5D5;
                    pla_test_flags = 4'b0001;
                end
            end
            // else: 0x000 (pass)
        end

        //----------------------------------------------------------------------
        // TST_DES_RTOLSS (0x19) - Return to Outer Level SS Descriptor
        //----------------------------------------------------------------------
        TST_DES_RTOLSS: begin
            // Term 40: !p1 p2 !p u !x rw → 0x86A
            if (!p1 && p2 && !p && u && !x && rw)
                pla_test_addr = 12'h86A;
            // Term 74: !p1 p2 p u !x rw → 0x5D5, K=0001
            if (!p1 && p2 && p && u && !x && rw) begin
                pla_test_addr = pla_test_addr | 12'h5D5;
                pla_test_flags = pla_test_flags | 4'b0001;
            end
        end

        //----------------------------------------------------------------------
        // TST_DES_LDTTSK (0x1A) - LDT Descriptor for Task Switch
        //----------------------------------------------------------------------
        TST_DES_LDTTSK: begin
            // Term 35: !p !u !x !ce rw !a → 0x85D
            if (!p && !u && !x && !ce && rw && !a)
                pla_test_addr = 12'h85D;
            // Term 66: p !u !x !ce rw !a → 0x5D5
            if (p && !u && !x && !ce && rw && !a)
                pla_test_addr = pla_test_addr | 12'h5D5;
        end

        //----------------------------------------------------------------------
        // TST_DES_LDT (0x1B) - LDT Descriptor
        //----------------------------------------------------------------------
        TST_DES_LDT: begin
            // Term 10: !p !u !x !ce rw !a → 0x870
            if (!p && !u && !x && !ce && rw && !a)
                pla_test_addr = 12'h870;
            // Term 66: p !u !x !ce rw !a → 0x5D5
            if (p && !u && !x && !ce && rw && !a)
                pla_test_addr = pla_test_addr | 12'h5D5;
        end

        //----------------------------------------------------------------------
        // TST_DES_MOREPR (0x1C) - More Privilege Descriptor Check
        //----------------------------------------------------------------------
        TST_DES_MOREPR: begin
            // Term 20: !p1 p2 !p u !x rw → 0x86A
            if (!p1 && p2 && !p && u && !x && rw)
                pla_test_addr = 12'h86A;
            // Term 56: !p1 p2 p u !x rw → 0x5DA, K=0001
            if (!p1 && p2 && p && u && !x && rw) begin
                pla_test_addr = pla_test_addr | 12'h5DA;
                pla_test_flags = pla_test_flags | 4'b0001;
            end
        end

        //----------------------------------------------------------------------
        // TST_DES_TSSTSG (0x1D) - TSS Descriptor for Task Switch Gate
        //----------------------------------------------------------------------
        TST_DES_TSSTSG: begin
            // Term 42: !p !u !ce !rw a → 0x870
            if (!p && !u && !ce && !rw && a)
                pla_test_addr = 12'h870;
            // Term 71: p !u !ce !rw a → 0x71F
            if (p && !u && !ce && !rw && a)
                pla_test_addr = pla_test_addr | 12'h71F;
        end

        //----------------------------------------------------------------------
        // TST_DES_TSSTSR (0x1E) - Test TSS Busy Bit
        //----------------------------------------------------------------------
        TST_DES_TSSTSR: begin
            if (!u && !ce && rw && a) begin
                if (!p) begin
                    pla_test_addr = 12'h870;  // TSS busy
                end else begin
                    pla_test_addr = 12'h5D3;  // TSS available
                end
            end
            // else: 0x000 (pass)
        end

        //----------------------------------------------------------------------
        // TST_DES_TSSLTR (0x1F) - Test TSS Available
        //----------------------------------------------------------------------
        TST_DES_TSSLTR: begin
            if (!u && !ce && !rw && a) begin
                if (!p) begin
                    pla_test_addr = 12'h870;  // Error (TSS busy when should be available)
                end else begin
                    pla_test_addr = 12'h5D3;  // TSS available (correct)
                end
            end
            // else: 0x000 (pass)
        end

        //----------------------------------------------------------------------
        // TST_DES_INT_SW (0x21) - Software Interrupt Gate Type
        //----------------------------------------------------------------------
        TST_DES_INT_SW: begin
            // Term 1: !p1 p !u x ce rw !a → 0x009
            if (!p1 && p && !u && x && ce && rw && !a)
                pla_test_addr = pla_test_addr | 12'h009;
            // Term 6: !p1 p !u !x ce rw a → 0x8C3
            if (!p1 && p && !u && !x && ce && rw && a)
                pla_test_addr = pla_test_addr | 12'h8C3;
            // Term 8: !p1 p !u x ce rw a → 0x8CC
            if (!p1 && p && !u && x && ce && rw && a)
                pla_test_addr = pla_test_addr | 12'h8CC;
            // Term 9: !p1 p !u ce rw !a → 0x8C2
            if (!p1 && p && !u && ce && rw && !a)
                pla_test_addr = pla_test_addr | 12'h8C2;
            // Term 11: !p1 p !u !x ce !rw a → 0x71C
            if (!p1 && p && !u && !x && ce && !rw && a)
                pla_test_addr = pla_test_addr | 12'h71C;
            // Term 32: !p1 !p !u !x ce a → 0x871
            if (!p1 && !p && !u && !x && ce && a)
                pla_test_addr = pla_test_addr | 12'h871;
            // Term 67: !p1 !p !u ce rw → 0x871
            if (!p1 && !p && !u && ce && rw)
                pla_test_addr = pla_test_addr | 12'h871;
        end

        //----------------------------------------------------------------------
        // TST_DES_INT_HW (0x22) - Test Interrupt Gate Type
        //----------------------------------------------------------------------
        TST_DES_INT_HW: begin
            if (p && !u && !x && ce && !rw && a) begin
                // Term 27: p !u !x ce !rw a → TASKGATE
                pla_test_addr = 12'h71C;
            end else if (!p && !u && ce && rw) begin
                // Term 76: !p !u ce rw → not present
                pla_test_addr = 12'h871;
            end else if (!p && !u && !x && ce && a) begin
                // Term 59: !p !u !x ce a → not present
                pla_test_addr = 12'h871;
            end else if (p && !u && ce && rw && !a) begin
                // Terms 24 & 3 can overlap
                // Term 24: p !u ce rw !a → 0x8C2
                // Term 3:  p !u x ce rw !a → 0x009
                // When both match (x=1): 0x8C2 | 0x009 = 0x8CB (INTGATE386)
                if (x) begin
                    pla_test_addr = 12'h8CB;  // OR of Term 3 and Term 24
                end else begin
                    pla_test_addr = 12'h8C2;  // Only Term 24
                end
            end else if (p && !u && !x && ce && rw && a) begin
                // Term 15: p !u !x ce rw a → INTGATE286
                pla_test_addr = 12'h8C3;
            end else if (p && !u && x && ce && rw && a) begin
                // Term 16: p !u x ce rw a → INTGATE386
                pla_test_addr = 12'h8CC;
            end
            // else: 0x000 (pass)
        end

        //----------------------------------------------------------------------
        // TST_SEL_LES (0x26) - LES Selector Check
        //----------------------------------------------------------------------
        TST_SEL_LES: begin
            // Term 46: !p → L flag
            if (!p)
                pla_test_flags = 4'b0100;
            // Term 120: p → 0x592
            if (p)
                pla_test_addr = 12'h592;
        end

        //----------------------------------------------------------------------
        // TST_SEL_LDS (0x27) - LDS Selector Check
        //----------------------------------------------------------------------
        TST_SEL_LDS: begin
            if (p) begin
                // p → 0x592
                pla_test_addr = 12'h592;
            end else begin
                pla_test_flags = 4'b0100;
            end
            // else: 0x000 (pass)
        end

        //----------------------------------------------------------------------
        // READ_RPL (0x2B)/ WRITE_RPL (0x2C) / SET_RPL_TO_CPL (0x2D) / COPY_STACK_DPL(0x2E)
        // Flag Setting Operations
        //----------------------------------------------------------------------
        READ_RPL, WRITE_RPL, SET_RPL_TO_CPL, COPY_STACK_DPL: begin
            // Always pass with appropriate flags set
            pla_test_addr = 12'h000;
            case (s1_test_const)
                READ_RPL:  pla_test_flags = 4'b0100;  // READ_RPL: set M flag (validation_ok)
                WRITE_RPL:  pla_test_flags = 4'b0010;  // WRITE_RPL: set L flag (limit_check)
                SET_RPL_TO_CPL: pla_test_flags = 4'b1010;  // SET_RPL_TO_CPL: set N and L flags
                COPY_STACK_DPL: pla_test_flags = 4'b1001;  // COPY_STACK_DPL: set N and K flags
                default: pla_test_flags = 4'b0000;
            endcase
        end

        //----------------------------------------------------------------------
        // JMP_GFAULT_INT (0x2A) - Unconditional redirect to GP fault handler
        //----------------------------------------------------------------------
        // PLA4 Entry #127: unconditional redirect (all state bits don't-care)
        // Used by INT/call gate handler to exit the 8BA-8C0 loop after
        // descriptor validation passes. The redirect target 0x865 continues
        // the normal privilege-switching flow.
        6'h2A: begin
            pla_test_addr = 12'h865;
        end

        // TODO: SET_P (0x2F), this turns on output[0]

        //----------------------------------------------------------------------
        // TST_DES_LAR (0x30) - Test LAR/LSL VERR/VERW
        //----------------------------------------------------------------------
        TST_DES_LAR: begin
            if (!p1 && u) begin
                // Term 115: !p1, u → 0x71A (most general)
                pla_test_addr = 12'h71A;
            end else if (u && x && ce) begin
                // Term 104: u x ce → 0x71A
                pla_test_addr = 12'h71A;
            end else if (!p1 && ce && !rw && !a) begin
                // Term 77: !p1, ce !rw !a → 0x71A
                pla_test_addr = 12'h71A;
            end else if (!p1 && !x && !rw && a) begin
                // Term 80: !p1, !x !rw a → 0x71A
                pla_test_addr = 12'h71A;
            end else if (!p1 && !x && !ce && rw) begin
                // Term 85: !p1, !x !ce rw → 0x71A
                pla_test_addr = 12'h71A;
            end else if (!p1 && !ce && a) begin
                // Term 103: !p1, !ce a → 0x71A
                pla_test_addr = 12'h71A;
            end
        end

        //----------------------------------------------------------------------
        // TST_DES_RTOLOS (0x25) - Return to Outer Level Other Segment
        //----------------------------------------------------------------------
        TST_DES_RTOLOS: begin
            // Term 60: p1 !p2 !ce → 0x6A8
            if (p1 && !p2 && !ce)
                pla_test_addr = pla_test_addr | 12'h6A8;
            // Term 63: p1 !p2 !x → 0x6A8
            if (p1 && !p2 && !x)
                pla_test_addr = pla_test_addr | 12'h6A8;
        end

        //----------------------------------------------------------------------
        // TST_ACCESS_VIO (0x24) - Access Rights Violation Check
        //----------------------------------------------------------------------
        TST_ACCESS_VIO: begin
            // All terms OR into the address. Multiple terms can match simultaneously.
            // Term 4: u x !ce !rw → |0x001
            if (u && x && !ce && !rw)
                pla_test_addr = pla_test_addr | 12'h001;
            // Term 5: !u x !ce !rw → |0x004
            if (!u && x && !ce && !rw)
                pla_test_addr = pla_test_addr | 12'h004;
            // Term 17: x !ce !rw !a → |0x862
            if (x && !ce && !rw && !a)
                pla_test_addr = pla_test_addr | 12'h862;
            // Term 19: !u !x !ce rw !a → |0x862
            if (!u && !x && !ce && rw && !a)
                pla_test_addr = pla_test_addr | 12'h862;
            // Term 41: u x !ce !a → |0x862
            if (u && x && !ce && !a)
                pla_test_addr = pla_test_addr | 12'h862;
            // Term 68: x !ce !rw a → |0x85A
            if (x && !ce && !rw && a)
                pla_test_addr = pla_test_addr | 12'h85A;
            // Term 78: !u !x !ce a → |0x85B
            if (!u && !x && !ce && a)
                pla_test_addr = pla_test_addr | 12'h85B;
            // Term 84: !u !x ce rw → |0x85E
            if (!u && !x && ce && rw)
                pla_test_addr = pla_test_addr | 12'h85E;
            // Term 100: !u !x !rw → |0x85B
            if (!u && !x && !rw)
                pla_test_addr = pla_test_addr | 12'h85B;
        end

        //----------------------------------------------------------------------
        // TST_DES_GRANUL (0x20) - LSL Granularity Check
        //----------------------------------------------------------------------
        TST_DES_GRANUL: begin  // TST_DES_GRANUL
            // Term 147: (any) → 0x6F5, Term 58: !p → |0x008
            if (!p) begin
                pla_test_addr = 12'h6FD;  // 0x6F5 | 0x008 = 0x6FD
            end else begin
                pla_test_addr = 12'h6F5;  // p → 0x6F5 only
            end
            if (x && !ce && !rw) begin
                if (u)
                    pla_test_addr[0] = 1'b1;   // Term 4: u x !ce !rw → |0x001
                else
                    pla_test_addr[2] = 1'b1;   // Term 5: !u x !ce !rw → |0x004
            end
        end

        //----------------------------------------------------------------------
        // TST_SEL_LFSLGS (0x28) - Protection Mode Load Flags
        //----------------------------------------------------------------------
        TST_SEL_LFSLGS: begin  // TST_SEL_LFSLGS
            if (!p) begin
                pla_test_addr = 12'h000;  // !p → CONTINUE
                pla_test_flags = 4'b0100;
            end else begin
                pla_test_addr = 12'h592;  // p → redirect
            end
        end

        //----------------------------------------------------------------------
        // TST_DES_LSL (0x31) - Load Segment Limit
        //----------------------------------------------------------------------
        TST_DES_LSL: begin  // TST_DES_LSL
            if (!p1 && u) begin
                // Term 126: !p1, u → 0x6EE (most general)
                pla_test_addr = 12'h6EE;
            end else if (!p1 && !ce && a) begin
                // Term 109: !p1, !ce a → 0x6EE
                pla_test_addr = 12'h6EE;
            end else if (!p1 && !x && !ce && rw) begin
                // Term 97: !p1, !x !ce rw → 0x6EE
                pla_test_addr = 12'h6EE;
            end else if (u && x && ce) begin
                // Term 111: u x ce → 0x6EE
                pla_test_addr = 12'h6EE;
            end
        end

        //----------------------------------------------------------------------
        // TST_DES_VERR (0x32) - Verify Segment for Reading
        //----------------------------------------------------------------------
        TST_DES_VERR: begin
            // CPL guard: SPTR would pre-validate CPL vs DPL. Since SPTR is not
            // implemented, suppress "pass" when CPL > DPL for non-conforming segments.
            if (s1_cpl > s1_desc_dpl && !(x && ce)) begin
                // Fall through to 718 (CLZF — clear ZF, VERR fails)
            end else if (u && x && ce && rw) begin
                // Term 94: u x ce rw → 0x71A (conforming readable code — no priv check)
                pla_test_addr = 12'h71A;
            end else if (!p1 && u && !x && rw) begin
                // Term 96: !p1, u !x rw → 0x71A
                pla_test_addr = 12'h71A;
            end else if (!p1 && u && rw) begin
                // Term 110: !p1, u rw → 0x71A
                pla_test_addr = 12'h71A;
            end else if (!p1 && u && !x) begin
                // Term 112: !p1, u !x → 0x71A
                pla_test_addr = 12'h71A;
            end
        end

        //----------------------------------------------------------------------
        // TST_DES_VERW (0x33) - Verify Segment for Writing
        //----------------------------------------------------------------------
        TST_DES_VERW: begin
            // CPL guard: same as TST_DES_VERR — suppress pass when CPL > DPL
            if (s1_cpl > s1_desc_dpl && !(x && ce)) begin
                // Fall through to 718 (CLZF — clear ZF, VERW fails)
            end else if (!p1 && u && !x && rw) begin
                // Term 96: !p1, u !x rw → 0x71A
                pla_test_addr = 12'h71A;  // LAR_VERRW_SUCCEEDED
            end
            // else: 0x000 (DEFAULT - fall through to CLZF)
        end

        //----------------------------------------------------------------------
        // FPU_WAIT (0x34) - Test FPU Wait
        //----------------------------------------------------------------------
        FPU_WAIT: begin
            if (x && rw) begin
                // Term 116: x rw → Coprocessor not available
                pla_test_addr = 12'h536;  // COPROCESSOR_NOT_AVAILABLE_EXCEPTION
            end
            // else: 0x000 (pass)
        end

        //----------------------------------------------------------------------
        // FPU_OTHER (0x38) - FPU Other Operations
        //----------------------------------------------------------------------
        FPU_OTHER: begin
            if (!x && !ce) begin
                // Terms 140,141: x or ce → fault; pass when both clear
                pla_test_addr = 12'h000;  // CONTINUE
            end else begin
                pla_test_addr = 12'h536;  // COPROCESSOR_NOT_AVAILABLE_EXCEPTION
            end
        end

        //----------------------------------------------------------------------
        // FPU_LOAD_3264 (0x39) - FPU Load 32/64-bit
        //----------------------------------------------------------------------
        FPU_LOAD_3264: begin
            if (!u && !x && !ce) begin
                // Term 83: !u !x !ce → 287 path (ET=0, TS=0, EM=0)
                pla_test_addr = 12'h4E8;  // FPU_LD3264_287
            end else if (u && !x && !ce) begin
                // Term 86: u !x !ce → 387 path (ET=1, TS=0, EM=0)
                pla_test_addr = 12'h51C;  // FPU_LD3264_387
            end else begin
                pla_test_addr = 12'h536;  // COPROCESSOR_NOT_AVAILABLE_EXCEPTION
            end
        end

        //----------------------------------------------------------------------
        // FPU_LOAD_80 (0x3B) - FPU Load 80-bit
        //----------------------------------------------------------------------
        FPU_LOAD_80: begin
            if (!u && !x && !ce) begin
                // Term 98: !u !x !ce → 287 path
                pla_test_addr = 12'h4E7;  // FPU_LD80_287
            end else if (u && !x && !ce) begin
                // Term 99: u !x !ce → 387 path
                pla_test_addr = 12'h4EE;  // FPU_LD80_387
            end else begin
                pla_test_addr = 12'h536;  // COPROCESSOR_NOT_AVAILABLE_EXCEPTION
            end
        end

        //----------------------------------------------------------------------
        // FPU_STORE_3264 (0x3C) - FPU Store 32/64-bit
        //----------------------------------------------------------------------
        FPU_STORE_3264: begin
            if (!u && !x && !ce) begin
                // Term 95: !u !x !ce → 287 path
                pla_test_addr = 12'h54E;  // FPU_ST3264_287
            end else if (u && !x && !ce) begin
                // Term 101: u !x !ce → 387 path
                pla_test_addr = 12'h555;  // FPU_ST3264_387
            end else begin
                pla_test_addr = 12'h536;  // COPROCESSOR_NOT_AVAILABLE_EXCEPTION
            end
        end

        //----------------------------------------------------------------------
        // FPU_STORE_80 (0x3D) - FPU Store 80-bit
        //----------------------------------------------------------------------
        FPU_STORE_80: begin
            if (!u && !x && !ce) begin
                // Term 93: !u !x !ce → 287 path
                pla_test_addr = 12'h54D;  // FPU_ST80_287
            end else if (u && !x && !ce) begin
                // Term 101: u !x !ce → 387 path
                pla_test_addr = 12'h554;  // FPU_ST80_387
            end else begin
                pla_test_addr = 12'h536;  // COPROCESSOR_NOT_AVAILABLE_EXCEPTION
            end
        end

        //----------------------------------------------------------------------
        // FPU_FSAVE (0x3E) - FPU Save State
        //----------------------------------------------------------------------
        FPU_FSAVE: begin
            if (u) begin
                // Term 105: u (CR0.ET=1) → 387 mode
                pla_test_addr = 12'h449;  // FSAVE_387
            end else begin
                // Term 108: !u (CR0.ET=0) → 287 mode
                pla_test_addr = 12'h451;  // FSAVE_287
            end
        end

        //----------------------------------------------------------------------
        // FPU_FRSTOR (0x3F) - FPU Restore State
        //----------------------------------------------------------------------
        FPU_FRSTOR: begin
            if (u) begin
                // Term 114: u (CR0.ET=1) → 387 mode
                pla_test_addr = 12'h4B5;  // FRSTOR_387
            end else begin
                // Term 121: !u (CR0.ET=0) → 287 mode
                pla_test_addr = 12'h4BB;  // FRSTOR_287
            end
        end

        //----------------------------------------------------------------------
        // Default: Unknown test type - pass
        //----------------------------------------------------------------------
        default: begin
            pla_test_addr  = 12'h000;
            pla_test_flags = 4'b0000;
        end
    endcase
end

assign pla_test_output = {pla_test_flags, pla_test_addr, 2'b00};

//==============================================================================
// Output Extraction
//==============================================================================
//
// PLA4 output format (18 bits):
//   bits[17:14]: N,M,L,K control flags
//   bits[13:2]:  Microcode address (12 bits, bit-reversed in PLA dump)
//   bits[1:0]:   Valid/type bits
//
//==============================================================================

logic [11:0] pla_test_addr;      // Computed by always_comb block
logic [3:0]  pla_test_flags;     // Computed by always_comb block

//==============================================================================
// Stage 2: Register PLA4 outputs (posedge clk when pipe_en)
//==============================================================================
always_ff @(posedge clk) begin
    if (!reset_n) begin
        s2_jump_addr <= 12'h000;
        s2_jump_valid <= 1'b0;
        s2_flags <= 4'b0000;
        s2_valid <= 1'b0;
        s2_is_checking_test <= 1'b0;
        s2_test_const <= 6'h0;
    end else if (pipe_en) begin
        s2_jump_addr <= pla_test_addr;
        s2_jump_valid <= (pla_test_addr != 12'h000);
        s2_flags <= pla_test_flags;
        s2_valid <= s1_valid;
        s2_is_checking_test <= s1_is_checking_test;
        s2_test_const <= s1_test_const;
    end
end

// Registered outputs (from Stage 2)
assign jump_addr      = s2_jump_addr;
assign jump_valid     = s2_jump_valid;
assign stack_op       = s2_flags[3];  // K flag (bit 17) — pla_test_flags[3]=K (MSB=output bit 17)
assign limit_check    = s2_flags[2];  // L flag (bit 16)
assign validation_ok  = s2_flags[1];  // M flag (bit 15)
assign set_accessed   = s2_flags[0];  // N flag (bit 14) — pla_test_flags[0]=N (LSB=output bit 14)
assign result_valid   = s2_valid;
assign is_checking_test = s2_is_checking_test;

//==============================================================================
// Assertions and Debug
//==============================================================================

`ifdef SIMULATION
    // Test constant name lookup for debug
    function automatic string test_name(input [5:0] tc);
        case (tc)
            TST_LD_NONSS:   return "TST_LD_NONSS";
            TST_SEL_RET:    return "TST_SEL_RET";
            TST_SEL_ARPL:   return "TST_SEL_ARPL";
            TST_SEL_LLDT:   return "TST_SEL_LLDT";
            TST_SEL_LLVV:   return "TST_SEL_LLVV";
            TST_DES_SIMPLE: return "TST_DES_SIMPLE";
            TST_DES_SS:     return "TST_DES_SS";
            TST_DES_JMP:    return "TST_DES_JMP";
            TST_DES_JGATE:  return "TST_DES_JGATE";
            TST_DES_JGDEST: return "TST_DES_JGDEST";
            TST_DES_CGATE:  return "TST_DES_CGATE";
            TST_DES_CGDEST: return "TST_DES_CGDEST";
            TST_DES_RETF:   return "TST_DES_RETF";
            TST_DES_TSSTSR: return "TST_DES_TSSTSR";
            TST_DES_TSSLTR: return "TST_DES_TSSLTR";
            TST_DES_INT_HW: return "TST_DES_INT_HW";
            TST_SEL_LDS:    return "TST_SEL_LDS";
            READ_RPL:       return "READ_RPL";
            WRITE_RPL:      return "WRITE_RPL";
            SET_RPL_TO_CPL: return "SET_RPL_TO_CPL";
            COPY_STACK_DPL: return "COPY_STACK_DPL";
            SET_FAULT:      return "SET_FAULT";
            TST_DES_LAR:    return "TST_DES_LAR";
            TST_DES_VERR:   return "TST_DES_VERR";
            TST_DES_VERW:   return "TST_DES_VERW";
            FPU_WAIT:       return "FPU_WAIT";
            FPU_OTHER:      return "FPU_OTHER";
            FPU_LOAD_3264:  return "FPU_LOAD_3264";
            FPU_LOAD_80:    return "FPU_LOAD_80";
            FPU_STORE_3264: return "FPU_STORE_3264";
            FPU_STORE_80:   return "FPU_STORE_80";
            FPU_FSAVE:      return "FPU_FSAVE";
            FPU_FRSTOR:     return "FPU_FRSTOR";
            default:        return "UNKNOWN";
        endcase
    endfunction

    // Check that test is only enabled in protected mode (FPU tests excepted)
    always_comb begin
        if (test_en && !pe_mode && !(test_const[5] && test_const[4] && (test_const[3] || test_const[2]))) begin
            $warning("PROT: Protection test in real mode: %s (0x%02x)",
                     test_name(test_const), test_const);
        end
    end

    // Display pipelined protection test results (triggers on result_valid)
    // synthesis translate_off
    always_comb begin
        if (result_valid && TRACE_PROT_EN) begin
            if (jump_addr == 12'h000) begin
                $display("PROT: %s (0x%02x) PASSED",
                         test_name(s2_test_const), s2_test_const);
            end else begin
                $display("PROT: %s (0x%02x) FAILED -> 0x%03x",
                         test_name(s2_test_const), s2_test_const, jump_addr);
                $display("      Flags: N(Accessed)=%b M(Valid)=%b L(Limit)=%b K(Stack)=%b",
                         set_accessed, validation_ok, limit_check, stack_op);
            end
        end
    end
    // synthesis translate_on
`endif

endmodule
