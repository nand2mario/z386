// 80386 Package - shared types and constants
package z386_pkg;

//=============================================================================
// Decoded Instruction Entry Type
//=============================================================================

// z386x FAST-path class descriptor. V52 derives it from the registered
// microcode entry point instead of carrying these 16 bits in every decoded
// instruction queue entry.
typedef struct packed {
    logic fast;        // completes at its RNI word (+ optional commit sideband)
    logic multi_word;  // RNI word is not the entry word (shifts, loads,
                       // ALU/CMP r,m, MOVZX); the chain fires one cycle before
                       // the RNI word, detected via uc_next
    logic [2:0] commit_sel; // 0 none; ALU (alu_result->DSTREG), SHIFT
                       // (shift_result->DSTREG) and SIGSRC (SIGMA->SRCREG,
                       // width from the executing BITS16/32) commit at the
                       // RNI-word cycle; MEM (OPR_R) commits one cycle later
                       // (deferred), only when the slot was chained away
    logic keep_slot;   // memory classes: the slot word (OPR_R writeback / WR
                       // DLY) is real work — execute it normally when
                       // unchained; never turn it into a dead slot
    logic reads_flags; // consumes CF (ADC/SBB/RCL/RCR): never chain INTO — the
                       // previous instruction's flags are still in two-cycle retirement
    logic uses_ea;     // consumes EA/moffs/IND at i_pop (LEA, loads, stores):
                       // never chain INTO — base/index GPRs may be written by
                       // the still-executing predecessor
    logic reads_dst;   // EX-cycle GPR sources, for the load-use chain-out gate
    logic reads_src;   //   (a MEM-commit load's value lands one cycle after
    logic reads_ecx;   //    the chained successor reads its operands)
    logic writes_srcreg; // LEA: the entry word writes SRCREG (modrm reg field)
                       //   via uc_dest - the EA chain-into gate must see it
    logic op_byte;     // operand size is byte (precomputed: byte selectors
                       //   encode {high,reg[1:0]}, needed by overlap checks)
    logic jcc;         // Jcc rel: chains only when NOT taken (fast_issueJ at
                       //   i_first, gated on the settled-EFLAGS condition);
                       //   taken keeps the 21.z386 early redirect + microcode
    logic writes_flags; // produces arithmetic flags: a reads_flags successor
                       //   may chain only after a non-flag-writing predecessor
    logic br_rel;      // relative branch (Jcc/JMP rel/CALL rel): its target is
                       //   a decode-time constant - request a speculative
                       //   target-line fetch at i_pop (independent of .fast)
} fast_class_t;

localparam [2:0] FAST_COMMIT_NONE   = 3'd0;
localparam [2:0] FAST_COMMIT_ALU    = 3'd1;
localparam [2:0] FAST_COMMIT_SHIFT  = 3'd2;
localparam [2:0] FAST_COMMIT_MEM    = 3'd3;
localparam [2:0] FAST_COMMIT_SIGSRC = 3'd4;
localparam [2:0] FAST_COMMIT_ESP    = 3'd5;  // PUSH: SIGMA (post-push ESP,
                                             // precomputed at i_pop) -> ESP

typedef struct packed {
    logic [7:0]  opcode;
    logic [7:0]  modrm;
    logic [7:0]  sib;                // SIB byte for 32-bit addressing with r/m=100
    logic        has_modrm;
    logic        has_sib;            // SIB byte present
    logic        has_0f;
    logic        has_rep;
    logic [3:0]  prefix_count;       // Count of prefix bytes (4 bits for up to 15 prefixes)
    logic [11:0] entry_point;
    logic [31:0] immediate;
    logic [31:0] displacement;
    logic [2:0]  imm_size;
    logic        data32;
    logic        addr32;
    logic [1:0]  rep_lock;
    logic [2:0]  seg;
    logic [4:0]  length;            // Instruction length in bytes

    // ROM1-derived signals
    logic        has_d_bit;
    logic        has_embedded_register;
    logic        has_w_bit;
    logic        is_pushpop_seg;
    logic        update_flags;

    // Special flags
    logic        has_moffs;            // A0-A3: MOV AL/eAX,moffs - immediate field contains direct address

    // Stack operation flags from pla_entry[13:12]
    logic        stack_op;             // pla_entry[13]: ESP will be modified
    logic        stack_dir;            // pla_entry[12]: 0=push/down, 1=pop/up

    // Register selection (computed from modrm/opcode)
    logic [2:0]  src_reg_sel;          // SRCREG selection
    logic [2:0]  dst_reg_sel;          // DSTREG selection

    // EA base/index onehot selectors are computed at decode time (D1), so
    // chain gates and the early-start latch do not re-derive them from ModR/M.
    logic [7:0]  ea_base_onehot;
    logic [7:0]  ea_index_onehot;
} dec_entry_t;

// Optimizer-generated recipe lookup and entry-point-derived FAST classifier.
`include "ucode_recipes.svh"

// Prefix enums
typedef enum logic [1:0] {
    PREFIX_NOREPLOCK = 2'b00,
    PREFIX_LOCK      = 2'b01,  // F0
    PREFIX_REPNE     = 2'b10,  // F2
    PREFIX_REP       = 2'b11   // F3
} prefix_replock_t;

typedef enum logic [2:0] {
    PREFIX_NOSEG = 3'b000,
    PREFIX_CS    = 3'b001,
    PREFIX_SS    = 3'b010,
    PREFIX_DS    = 3'b011,
    PREFIX_ES    = 3'b100,
    PREFIX_FS    = 3'b101,
    PREFIX_GS    = 3'b110
} prefix_seg_t;

// z386x FAST-path classification (doc/z386x/design.md)
// Details: doc/z386x/implementation_notes.md#src-24-z386x-z386-pkg-sv-118


function automatic fast_class_t dec_fast_class(input dec_entry_t e);
    logic mod11;
    logic [2:0] grp;
    fast_class_t r;
    r = '0;
    mod11 = e.has_modrm && (e.modrm[7:6] == 2'b11);
    // Operand byte-ness: 40-4F and 50-5F are always wide, B0-BF use
    // opcode[3], everything else in the FAST set is a w-bit form.
    if (e.opcode[7:4] == 4'h4 || e.opcode[7:4] == 4'h5)
        r.op_byte = 1'b0;
    else if (e.opcode[7:4] == 4'hB && !e.has_0f)
        r.op_byte = !e.opcode[3];
    else
        r.op_byte = !e.opcode[0];
    if (e.has_0f && e.rep_lock == PREFIX_NOREPLOCK) begin
        // 0F B6/B7/BE/BF MOVZX/MOVSX r,r/m: SZ_EXT then BITS16/32+RNI; the
        // slot (SIGMA->SRCREG) is replaced by the SIGSRC sideband whose width
        // comes from the executing BITS op. DSTREG (the r/m source) is read
        // at the entry word, one cycle after i_pop - settled, so no gates.
        if (e.opcode[7:4] == 4'h8) begin
            // 0F 80-8F Jcc rel16/32: same shape as 70-7F
            r.fast = 1'b1; r.jcc = 1'b1; r.multi_word = 1'b1;
            r.reads_flags = 1'b1; r.br_rel = 1'b1;
        end else if ((e.opcode[7:4] == 4'hB) && (e.opcode[2:1] == 2'b11) && e.has_modrm) begin
            r.fast = 1'b1; r.multi_word = 1'b1; r.commit_sel = FAST_COMMIT_SIGSRC;
            r.writes_srcreg = 1'b1;
            if (!mod11) r.uses_ea = 1'b1;
            else r.reads_dst = 1'b1;   // SZ_EXT reads DSTREG at the entry word =
                                       // the deferred-load-commit cycle when
                                       // chained out of a load: gate needed
        end else if (((e.opcode == 8'hA4) || (e.opcode == 8'hA5) ||
                      (e.opcode == 8'hAC) || (e.opcode == 8'hAD)) && mod11) begin
            // SHLD/SHRD r,r,imm/CL: SHIFT1 then SHIFT2+RNI; replace the
            // SIGMA->DSTREG slot with the existing deferred SHIFT commit.
            r.fast = 1'b1; r.multi_word = 1'b1;
            r.commit_sel = FAST_COMMIT_SHIFT;
            r.reads_dst = 1'b1; r.reads_src = 1'b1;
            r.reads_ecx = e.opcode[0];
            r.writes_flags = 1'b1;
        end
    end else if (!e.has_0f && e.rep_lock == PREFIX_NOREPLOCK) begin
        if (e.opcode[7:6] == 2'b00) begin
            // 00-3B ALU/CMP r,r (mod=11) and 04..3D ALU/CMP A,imm forms
            grp = e.opcode[5:3];
            if ((!e.opcode[2] && mod11) || (e.opcode[2:1] == 2'b10)) begin
                r.fast        = 1'b1;
                r.commit_sel  = (grp != 3'b111) ? FAST_COMMIT_ALU : FAST_COMMIT_NONE; // CMP: flags only
                r.reads_flags = (grp == 3'b010) || (grp == 3'b011);  // ADC/SBB
                r.writes_flags = 1'b1;
                r.reads_dst   = 1'b1;
                r.reads_src   = !e.opcode[2];        // r,r forms read both
            end else if (e.opcode[2:0] == 3'b110) begin
                // 06/0E/16/1E PUSH seg (07/17/1F POP seg load segments - SEQ).
                // 09B: SEGREG->OPR_W + WR + RNI; slot = SIGMA->eSP + DLY,
                // replaced by the ESP commit when chained away.
                r.fast = 1'b1; r.commit_sel = FAST_COMMIT_ESP;
                r.keep_slot = 1'b1; r.uses_ea = 1'b1;
            end else if (!e.opcode[2] && e.has_modrm) begin
                // Memory forms: only the read-only shapes are FAST. ALU r,m (d=1, groups 0-6): 027 RD/DLY/OPR_R->TMPB/ALU+RNI CMP r,m / CMP m,r (group...
                // Details: doc/z386x/implementation_notes.md#src-24-z386x-z386-pkg-sv-182
                if (e.opcode[1]) begin
                    r.fast = 1'b1; r.multi_word = 1'b1; r.uses_ea = 1'b1;
                    r.commit_sel = (grp != 3'b111) ? FAST_COMMIT_ALU : FAST_COMMIT_NONE;
                    r.writes_flags = 1'b1;
                end else if (grp == 3'b111) begin
                    r.fast = 1'b1; r.multi_word = 1'b1; r.uses_ea = 1'b1;   // CMP m,r
                    r.writes_flags = 1'b1;
                end else begin
                    // ALU m,r RMW (d=0, groups 0-6): M5 F-RMW retimed ucode 04A RD / 04B DLY+JMP / 04C ALU(OPR_R,SRCREG) / 046 SIGMA->OPR_W+WR+RNI / 047 DLY....
                    // Details: doc/z386x/implementation_notes.md#src-24-z386x-z386-pkg-sv-196
                    r.fast = 1'b1; r.multi_word = 1'b1; r.uses_ea = 1'b1;
                    r.keep_slot = 1'b1; r.writes_flags = 1'b1;
                end
            end
        end else if (e.opcode[7:4] == 4'h4) begin
            // 40-4F INC/DEC r
            r.fast = 1'b1; r.commit_sel = FAST_COMMIT_ALU; r.reads_dst = 1'b1;
            r.writes_flags = 1'b1;
        end else if (e.opcode[7:4] == 4'h7) begin
            // 70-7F Jcc rel8: not-taken flow is 065 JNcond -> 066 RNi(+PREF suppressed) - chainJ fires at 065 on the settled condition. Taken flow...
            // Details: doc/z386x/implementation_notes.md#src-24-z386x-z386-pkg-sv-213
            r.fast = 1'b1; r.jcc = 1'b1; r.multi_word = 1'b1;
            r.reads_flags = 1'b1; r.br_rel = 1'b1;
        end else if (e.opcode[7:3] == 5'b01010) begin
            // 50-57 PUSH r: 086 DSTREG->OPR_W + WR + RNI; slot SIGMA->eSP+DLY.
            // reads_dst: the pushed register is read at the entry word (the
            // deferred-load-commit cycle when chained out of a load).
            r.fast = 1'b1; r.commit_sel = FAST_COMMIT_ESP;
            r.keep_slot = 1'b1; r.uses_ea = 1'b1; r.reads_dst = 1'b1;
        end else if (e.opcode[7:3] == 5'b01011) begin
            // 58-5F POP r: 09F SIGMA->eSP + RD (ESP written at the entry word),
            // 0A0 DLY+RNI, slot OPR_R->DSTREG - the load shape: deferred MEM
            // commit when chained away.
            r.fast = 1'b1; r.multi_word = 1'b1; r.commit_sel = FAST_COMMIT_MEM;
            r.keep_slot = 1'b1; r.uses_ea = 1'b1;
        end else if (e.opcode == 8'h68 || e.opcode == 8'h6A) begin
            // PUSH imm: 09D IMM->OPR_W + WR + RNI; slot SIGMA->eSP+DLY
            r.fast = 1'b1; r.commit_sel = FAST_COMMIT_ESP;
            r.keep_slot = 1'b1; r.uses_ea = 1'b1;
        end else if (e.opcode == 8'h80 || e.opcode == 8'h81 || e.opcode == 8'h83) begin
            grp = e.modrm[5:3];
            if (mod11) begin
                // ALU/CMP r,imm
                r.fast        = 1'b1;
                r.commit_sel  = (grp != 3'b111) ? FAST_COMMIT_ALU : FAST_COMMIT_NONE;
                r.reads_flags = (grp == 3'b010) || (grp == 3'b011);
                r.writes_flags = 1'b1;
                r.reads_dst   = 1'b1;
            end else if (grp == 3'b111) begin
                // CMP m,imm: read-only mem CMPTST shape
                r.fast = 1'b1; r.multi_word = 1'b1; r.uses_ea = 1'b1;
                r.writes_flags = 1'b1;
            end else begin
                // ALU m,imm RMW (039 retimed like 04A): store-shaped FAST,
                // same rationale as the ALU m,r class above.
                r.fast = 1'b1; r.multi_word = 1'b1; r.uses_ea = 1'b1;
                r.keep_slot = 1'b1; r.writes_flags = 1'b1;
            end
        end else if (e.opcode == 8'h84 || e.opcode == 8'h85) begin
            if (mod11) begin
                // TEST r,r
                r.fast = 1'b1; r.reads_dst = 1'b1; r.reads_src = 1'b1;
            end else begin
                // TEST m,r: read-only mem CMPTST shape
                r.fast = 1'b1; r.multi_word = 1'b1; r.uses_ea = 1'b1;
            end
            r.writes_flags = 1'b1;
        end else if (e.opcode[7:2] == 6'b100010) begin
            if (mod11) begin
                // 88-8B MOV r,r
                r.fast = 1'b1; r.commit_sel = FAST_COMMIT_ALU; r.reads_src = 1'b1;
            end else if (e.opcode[1]) begin
                // 8A/8B MOV r,m: 019 RD, 01A DLY+RNI (v43 fold); slot writes
                // OPR_R->DSTREG and stays live when unchained; when chained
                // away, the deferred MEM commit replaces it.
                r.fast = 1'b1; r.multi_word = 1'b1; r.commit_sel = FAST_COMMIT_MEM;
                r.keep_slot = 1'b1; r.uses_ea = 1'b1;
            end else begin
                // 88/89 MOV m,r: 013 OPR_W write + WR + RNI; slot is the DLY.
                // Retire-on-accept is enforced at the WR issue (mem_block_idle).
                r.fast = 1'b1; r.keep_slot = 1'b1; r.uses_ea = 1'b1;
                r.reads_src = 1'b1;
            end
        end else if (e.opcode == 8'h8D && e.has_modrm && (e.modrm[7:6] != 2'b11)) begin
            // LEA r,m: entry word 0B9 writes SRCREG (modrm reg field) from IND
            // itself (no sideband); mod=11 is #UD and stays on the SEQ path
            r.fast = 1'b1; r.uses_ea = 1'b1; r.writes_srcreg = 1'b1;
        end else if (e.opcode[7:2] == 6'b101000) begin
            // A0/A1 MOV A,moffs (load) — same 019 routine as MOV r,m;
            // A2/A3 MOV moffs,A (store) — same 013 routine as MOV m,r
            r.fast = 1'b1; r.keep_slot = 1'b1; r.uses_ea = 1'b1;
            if (!e.opcode[1]) begin
                r.multi_word = 1'b1; r.commit_sel = FAST_COMMIT_MEM;
            end else begin
                r.reads_src = 1'b1;   // 013 reads SRCREG (the accumulator)
            end
        end else if (e.opcode == 8'hEB || e.opcode == 8'hE9 || e.opcode == 8'hE8) begin
            // JMP rel8/rel32, CALL rel: speculative target-line fetch, and the
            // routines end at 068 SIGMA->eIP+RNI - chainN reclaims the final
            // slot with the composed EIP write. CALL's i_pop does the stack
            // SIGMA precompute (uses_ea via the stack gate).
            r.br_rel = 1'b1;
            r.fast = 1'b1; r.multi_word = 1'b1;
            if (e.opcode == 8'hE8) begin
                r.uses_ea = 1'b1;
                if (e.data32)
                    r.commit_sel = FAST_COMMIT_ESP;
            end
        end else if (e.opcode == 8'hC3 || e.opcode == 8'hC2) begin
            // RET / RET iw (near): 072 RD + new-ESP, 073 eSP + jump, 074
            // IND=OPR_R, JMP_PREF..., 068 eIP+RNI - chainN reclaims the final
            // slot when the PREF refill decodes the return target in time.
            r.fast = 1'b1; r.multi_word = 1'b1; r.uses_ea = 1'b1;
        end else if (e.opcode == 8'h98) begin
            // CBW/CWDE: same SZ_EXT + BITS16/32 + RNI shape as MOVZX r,r.
            // reads_dst: SZ_EXT reads AL/AX at the entry word, which is the
            // deferred-commit cycle when chained out of a load (mov al,[m];
            // cbw read stale AL - broke the DOS 7.1 boot loader).
            r.fast = 1'b1; r.multi_word = 1'b1; r.commit_sel = FAST_COMMIT_SIGSRC;
            r.writes_srcreg = 1'b1; r.reads_dst = 1'b1;
        end else if (e.opcode[7:1] == 7'b1010100) begin
            // A8/A9 TEST A,imm
            r.fast = 1'b1; r.reads_dst = 1'b1; r.writes_flags = 1'b1;
        end else if (e.opcode[7:4] == 4'hB) begin
            // B0-BF MOV r,imm
            r.fast = 1'b1; r.commit_sel = FAST_COMMIT_ALU;
        end else if ((e.opcode[7:1] == 7'b1100011) && (e.modrm[5:3] == 3'b000)) begin
            if (mod11) begin
                // C6/C7 MOV r,imm register form (other reg fields are #UD -> SEQ)
                r.fast = 1'b1; r.commit_sel = FAST_COMMIT_ALU;
            end else begin
                // C6/C7 MOV m,imm: 015 IMM->OPR_W + WR + RNI; slot is the DLY
                r.fast = 1'b1; r.keep_slot = 1'b1; r.uses_ea = 1'b1;
            end
        end else if (e.opcode[7:1] == 7'b1111011) begin
            // F6/F7 group 3: /0 TEST r/m,imm; register /2 NOT, /3 NEG
            if (e.modrm[5:3] == 3'b000) begin
                if (mod11) begin
                    r.fast = 1'b1; r.reads_dst = 1'b1;        // TEST r,imm
                end else begin
                    r.fast = 1'b1; r.multi_word = 1'b1; r.uses_ea = 1'b1; // TEST m,imm
                end
                r.writes_flags = 1'b1;
            end else if (mod11 && (e.modrm[5:3] == 3'b010 || e.modrm[5:3] == 3'b011)) begin
                // NOT preserves flags but NEG writes them - be conservative
                r.fast = 1'b1; r.commit_sel = FAST_COMMIT_ALU; r.reads_dst = 1'b1;
                r.writes_flags = 1'b1;
            end
        end else if ((e.opcode[7:1] == 7'b1111111) && mod11 && (e.modrm[5:4] == 2'b00)) begin
            // FE/FF INC/DEC r (register forms, /0 /1)
            r.fast = 1'b1; r.commit_sel = FAST_COMMIT_ALU; r.reads_dst = 1'b1;
            r.writes_flags = 1'b1;
        end else if (((e.opcode[7:1] == 7'b1100000) || (e.opcode[7:2] == 6'b110100)) && mod11
                     && (e.modrm[5:3] != 3'b010) && (e.modrm[5:3] != 3'b011)
                     && (e.modrm[5:3] != 3'b110)) begin
            // C0/C1 shift r,imm; D0/D1 shift r,1; D2/D3 shift r,CL - two-word routines (0F9/0FF): SHIFT1 count capture, then SHIFT2 with RNI; the...
            // Details: doc/z386x/implementation_notes.md#src-24-z386x-z386-pkg-sv-349
            r.fast = 1'b1; r.multi_word = 1'b1; r.commit_sel = FAST_COMMIT_SHIFT;
            r.reads_dst = 1'b1; r.writes_flags = 1'b1;
            r.reads_ecx = e.opcode[1] && (e.opcode[7:2] == 6'b110100); // D2/D3 r,CL
        end
    end
    dec_fast_class = r;
endfunction

// EA register decode helpers (moved from z386.sv; shared with the decoder).

function automatic [7:0] decode_base_register_32(input [7:0] modrm_byte, input [7:0] sib_byte, input has_sib);
    reg [1:0] mod;
    reg [2:0] rm;

    mod = modrm_byte[7:6];
    rm = modrm_byte[2:0];

    if (has_sib && rm == 3'b100) begin
        // SIB byte addressing - decode base from i.sib[2:0]
        case (sib_byte[2:0])
            3'b000: decode_base_register_32 = 8'b0000_0001;  // EAX
            3'b001: decode_base_register_32 = 8'b0000_0010;  // ECX
            3'b010: decode_base_register_32 = 8'b0000_0100;  // EDX
            3'b011: decode_base_register_32 = 8'b0000_1000;  // EBX
            3'b100: decode_base_register_32 = 8'b0001_0000;  // ESP
            3'b101: decode_base_register_32 = (mod == 2'b00) ? 8'b0000_0000 : 8'b0010_0000;  // None or EBP
            3'b110: decode_base_register_32 = 8'b0100_0000;  // ESI
            3'b111: decode_base_register_32 = 8'b1000_0000;  // EDI
        endcase
    end else begin
        // Non-SIB addressing - decode base from rm
        case (rm)
            3'b000: decode_base_register_32 = 8'b0000_0001;  // EAX
            3'b001: decode_base_register_32 = 8'b0000_0010;  // ECX
            3'b010: decode_base_register_32 = 8'b0000_0100;  // EDX
            3'b011: decode_base_register_32 = 8'b0000_1000;  // EBX
            3'b100: decode_base_register_32 = 8'b0001_0000;  // ESP
            3'b101: decode_base_register_32 = (mod == 2'b00) ? 8'b0000_0000 : 8'b0010_0000;  // None or EBP
            3'b110: decode_base_register_32 = 8'b0100_0000;  // ESI
            3'b111: decode_base_register_32 = 8'b1000_0000;  // EDI
        endcase
    end
endfunction


function automatic [7:0] decode_index_register_32(input [7:0] sib_byte, input has_sib);
    if (!has_sib) begin
        decode_index_register_32 = 8'b0000_0000;  // No index
    end else begin
        case (sib_byte[5:3])
            3'b000: decode_index_register_32 = 8'b0000_0001;  // EAX
            3'b001: decode_index_register_32 = 8'b0000_0010;  // ECX
            3'b010: decode_index_register_32 = 8'b0000_0100;  // EDX
            3'b011: decode_index_register_32 = 8'b0000_1000;  // EBX
            3'b100: decode_index_register_32 = 8'b0000_0000;  // No index
            3'b101: decode_index_register_32 = 8'b0010_0000;  // EBP
            3'b110: decode_index_register_32 = 8'b0100_0000;  // ESI
            3'b111: decode_index_register_32 = 8'b1000_0000;  // EDI
        endcase
    end
endfunction


function automatic [15:0] decode_base_register_16(input [7:0] modrm_byte);
    reg [1:0] mod;
    reg [2:0] rm;

    mod = modrm_byte[7:6];
    rm = modrm_byte[2:0];

    case (rm)
        3'b000: decode_base_register_16 = {8'b0100_0000, 8'b0000_1000};  // BX + SI
        3'b001: decode_base_register_16 = {8'b1000_0000, 8'b0000_1000};  // BX + DI
        3'b010: decode_base_register_16 = {8'b0100_0000, 8'b0010_0000};  // BP + SI
        3'b011: decode_base_register_16 = {8'b1000_0000, 8'b0010_0000};  // BP + DI
        3'b100: decode_base_register_16 = {8'b0000_0000, 8'b0100_0000};  // SI only
        3'b101: decode_base_register_16 = {8'b0000_0000, 8'b1000_0000};  // DI only
        3'b110: decode_base_register_16 = (mod == 2'b00) ? 16'h0000 : {8'b0000_0000, 8'b0010_0000};  // None or BP
        3'b111: decode_base_register_16 = {8'b0000_0000, 8'b0000_1000};  // BX only
    endcase
endfunction

// EA base/index onehot selectors as stored in dec_entry_t (zero when the
// instruction has no register-based EA: no modrm, moffs, or reg form).
function automatic [15:0] dec_ea_onehots(input dec_entry_t e);
    logic [7:0] base_sel, index_sel;
    logic [15:0] regs16;
    base_sel = 8'h00; index_sel = 8'h00;
    regs16 = decode_base_register_16(e.modrm);
    if (e.has_modrm && !e.has_moffs) begin
        if (e.addr32) begin
            base_sel  = decode_base_register_32(e.modrm, e.sib, e.has_sib);
            index_sel = decode_index_register_32(e.sib, e.has_sib);
        end else begin
            base_sel  = regs16[7:0];
            index_sel = regs16[15:8];
        end
    end
    dec_ea_onehots = {index_sel, base_sel};
endfunction

// Segment Descriptor Cache
// Details: doc/z386x/implementation_notes.md#src-24-z386x-z386-pkg-sv-457

// Segment register indices
localparam [3:0] SEG_ES  = 4'd0;
localparam [3:0] SEG_CS  = 4'd1;
localparam [3:0] SEG_SS  = 4'd2;
localparam [3:0] SEG_DS  = 4'd3;
localparam [3:0] SEG_FS  = 4'd4;
localparam [3:0] SEG_GS  = 4'd5;
localparam [3:0] SEG_IDT = 4'd6;  // IDT/IVT pseudo-segment (base=0, IND has full linear addr)
localparam [3:0] SEG_IO  = 4'd7;  // IO pseudo-segment for I/O operations
localparam [3:0] SEG_TR  = 4'd8;  // Task Register segment for TSS access
localparam [3:0] SEG_LDT = 4'd9;  // LDT Register descriptor cache
localparam [3:0] SEG_GDT = 4'd10; // GDT/LDT/SDT descriptor table access (base=0, IND has full linear addr)
localparam [3:0] SEG_NONE = 4'd15; // No segment target (use desc_write_seg fallback)

// Segmentation unit command encoding
localparam [3:0] SEG_CMD_NONE        = 4'd0;  // No operation
localparam [3:0] SEG_CMD_UPDATE_SEG  = 4'd1;  // Update segment selection (from IND_PLUS_ALU/IND_SRC)
localparam [3:0] SEG_CMD_SBRM        = 4'd2;  // Real-mode segment load
localparam [3:0] SEG_CMD_SAR         = 4'd3;  // Write access rights to descriptor cache
localparam [3:0] SEG_CMD_SLIM        = 4'd4;  // Write limit to segment descriptor cache
localparam [3:0] SEG_CMD_SLIM_TABLE  = 4'd5;  // Write GDTR/IDTR limit
localparam [3:0] SEG_CMD_SBAS        = 4'd6;  // Write GDTR/IDTR base
localparam [3:0] SEG_CMD_SDEH        = 4'd7;  // Descriptor high extension write (PM)
localparam [3:0] SEG_CMD_SDES        = 4'd8;  // Descriptor mid-field write (PM, uses desc_write_seg)
localparam [3:0] SEG_CMD_SDEL        = 4'd9;  // Descriptor low limit write (PM, uses desc_write_seg)
localparam [3:0] SEG_CMD_SPCR        = 4'd10; // Set stack push mode
localparam [3:0] SEG_CMD_DESCSW      = 4'd11; // Switch to CS-based push mode (cross-privilege)
localparam [3:0] SEG_CMD_STSSAF      = 4'd12; // Set TSS access flag, clear push/descsw mode
localparam [3:0] SEG_CMD_CTSSAF      = 4'd13; // Clear TSS access flag
localparam [3:0] SEG_CMD_DESC        = 4'd14; // Full descriptor write (BUSOP_LLIM in PM)
localparam [3:0] SEG_CMD_INIT_SEG    = 4'd15; // Initialize segment for new instruction (at i_pop)

// Segment descriptor cache entry
// Matches the 386 hidden descriptor cache loaded when segment register changes
typedef struct packed {
    logic [31:0] base;          // Segment base address (24-bit in real mode, 32-bit in protected)
    logic [19:0] limit;         // Segment limit (20-bit, scaled by G bit)
    logic [3:0]  seg_type;      // Type field (interpretation depends on S bit)
    logic        S;             // Descriptor type: 0=system, 1=code/data
    logic [1:0]  DPL;           // Descriptor Privilege Level (0-3)
    logic        P;             // Present bit
    logic        D_B;           // Default size: D for code (0=16-bit, 1=32-bit), B for stack
    logic        G;             // Granularity: 0=byte, 1=4KB pages (limit *= 4096)
    logic        A;             // Accessed bit (set by CPU on access)
} seg_desc_t;

// Default descriptor for real mode initialization
// In real mode: base = selector << 4, limit = 0xFFFF, 16-bit, present, DPL=0
function automatic seg_desc_t seg_desc_real_mode(input [15:0] selector);
    seg_desc_t desc;
    desc.base       = {12'h000, selector, 4'h0};  // selector << 4
    desc.limit      = 20'h0FFFF;                  // 64KB limit (0xFFFF)
    desc.seg_type   = 4'b0010;                    // Data, writable
    desc.S          = 1'b1;                       // Code/data segment
    desc.DPL        = 2'b00;                      // Ring 0
    desc.P          = 1'b1;                       // Present
    desc.D_B        = 1'b0;                       // 16-bit default
    desc.G          = 1'b0;                       // Byte granularity
    desc.A          = 1'b1;                       // Accessed
    return desc;
endfunction

// Default descriptor for code segment in real mode
function automatic seg_desc_t seg_desc_real_mode_code(input [15:0] selector);
    seg_desc_t desc;
    desc.base       = {12'h000, selector, 4'h0};  // selector << 4
    desc.limit      = 20'h0FFFF;                  // 64KB limit
    desc.seg_type   = 4'b1010;                    // Code, readable
    desc.S          = 1'b1;
    desc.DPL        = 2'b00;
    desc.P          = 1'b1;
    desc.D_B        = 1'b0;                       // 16-bit default
    desc.G          = 1'b0;
    desc.A          = 1'b1;
    return desc;
endfunction

// CS descriptor at 386 reset - base is hardwired to FFFF0000h (not F0000h!)
function automatic seg_desc_t seg_desc_reset_cs();
    seg_desc_t desc;
    desc.base       = 32'hFFFF0000;                   // Reset: base is FFFF0000h
    desc.limit      = 20'hFFFF;                       // Reset: limit is 64KB
    desc.seg_type   = 4'b1010;                        // Code, readable
    desc.S          = 1'b1;
    desc.DPL        = 2'b00;
    desc.P          = 1'b1;
    desc.D_B        = 1'b0;                           // 16-bit default
    desc.G          = 1'b0;
    desc.A          = 1'b1;
    return desc;
endfunction

// IDT/IVT descriptor for real mode - base=0, limit=0x3FF (256 vectors * 4 bytes)
function automatic seg_desc_t seg_desc_idt_real_mode();
    seg_desc_t desc;
    desc.base       = 32'h00000000;               // IVT at linear address 0
    desc.limit      = 20'h003FF;                  // 1KB limit (256 * 4 bytes)
    desc.seg_type   = 4'b0010;                    // Data, readable
    desc.S          = 1'b1;
    desc.DPL        = 2'b00;
    desc.P          = 1'b1;
    desc.D_B        = 1'b0;                       // 16-bit
    desc.G          = 1'b0;                       // Byte granularity
    desc.A          = 1'b1;
    return desc;
endfunction

// Calculate effective limit considering G bit (granularity)
// When G=1, limit is in 4KB pages, so effective_limit = (limit << 12) | 0xFFF
function automatic [31:0] seg_effective_limit(input seg_desc_t desc);
    if (desc.G)
        seg_effective_limit = {desc.limit, 12'hFFF};  // Page granularity
    else
        seg_effective_limit = {12'h000, desc.limit};  // Byte granularity
endfunction

// Decode raw descriptor hi/lo DWORDs into seg_desc_t.
function automatic seg_desc_t decode_descriptor(input [31:0] desc_lo, input [31:0] desc_hi);
    seg_desc_t d;
    d.base[7:0]    = desc_lo[23:16];
    d.base[15:8]   = desc_lo[31:24];
    d.base[23:16]  = desc_hi[7:0];
    d.base[31:24]  = desc_hi[31:24];
    d.limit[15:0]  = desc_lo[15:0];
    d.limit[19:16] = desc_hi[19:16];
    d.seg_type     = desc_hi[11:8];
    d.S            = desc_hi[12];
    d.DPL          = desc_hi[14:13];
    d.P            = desc_hi[15];
    d.D_B          = desc_hi[22];
    d.G            = desc_hi[23];
    d.A            = desc_hi[8];
    return d;
endfunction

// SDEH: merge hi DWORD fields (descriptor bytes 6-7) into existing cache.
// data = raw hi DWORD: [31:24]=base[31:24], [23]=G, [22]=D_B, [19:16]=limit[19:16]
function automatic seg_desc_t merge_sdeh(seg_desc_t existing, input [31:0] data);
    merge_sdeh = existing;
    merge_sdeh.base[31:24]  = data[31:24];
    merge_sdeh.G            = data[23];
    merge_sdeh.D_B          = data[22];
    merge_sdeh.limit[19:16] = data[19:16];
endfunction

// SDES: merge shifted DWORD fields (descriptor bytes 2-5) into existing cache.
// data = barrel-shifted middle 32 bits: [31]=P, [30:29]=DPL, [28]=S, [27:24]=Type, [23:0]=base[23:0]
function automatic seg_desc_t merge_sdes(seg_desc_t existing, input [31:0] data);
    merge_sdes = existing;
    merge_sdes.P           = data[31];
    merge_sdes.DPL         = data[30:29];
    merge_sdes.S           = data[28];
    merge_sdes.seg_type    = data[27:24];
    merge_sdes.base[23:0]  = data[23:0];
    merge_sdes.A           = data[24];  // Type[0] = Accessed
endfunction

// SAR: merge access rights into existing descriptor cache.
// Data format: [15]=P, [14:13]=DPL, [12]=S, [11:8]=Type, [7]=G, [6]=D_B
// (matches 386 descriptor bytes 5-6: access byte in [15:8], ar_high in [7:0])
function automatic seg_desc_t merge_sar(seg_desc_t existing, input [31:0] data);
    merge_sar = existing;
    merge_sar.P           = data[15];
    merge_sar.DPL         = data[14:13];
    merge_sar.S           = data[12];
    merge_sar.seg_type    = data[11:8];
    merge_sar.G           = data[7];
    merge_sar.D_B         = data[6];
    merge_sar.A           = data[8];   // Type[0] = Accessed
endfunction

// SDEL: merge lo DWORD fields (descriptor bytes 0-1) into existing cache.
// data[15:0] = limit[15:0]
function automatic seg_desc_t merge_sdel(seg_desc_t existing, input [31:0] data);
    merge_sdel = existing;
    merge_sdel.limit[15:0] = data[15:0];
endfunction

// Microcode fields
localparam ALUJMP_ALU = 7'h00;      // opcode[5:3] specifies ADD, OR, ADC, SBB, AND, SUB, XOR, CMP
localparam ALUJMP_INCDEC = 7'h01;   // ++--~- INC/DEC/NOT/NEG operations
localparam ALUJMP_SHIFT1 = 7'h02;   // <<>>? First pass shift: stores count and value
localparam ALUJMP_CMPTST = 7'h03;   // CMP/TEST - uses SUB or AND based on opcode
localparam ALUJMP_SZ_EXT = 7'h06;   // Sign/zero extension (MOVZX/MOVSX/CBW/CWDE) - uses opcode bit to select
localparam ALUJMP_SZ_EX2 = 7'h07;   // MUL/IMUL fixup: set CF/OF flags, fix signed multiplication result
localparam ALUJMP_AND = 7'h08;
localparam ALUJMP_OR = 7'h09;
localparam ALUJMP_XOR = 7'h0A;
localparam ALUJMP_SIGN = 7'h0B;   // Sign extension (CWD/CDQ)
localparam ALUJMP_ADD = 7'h0C;
localparam ALUJMP_ADC = 7'h0D;
localparam ALUJMP_SUB = 7'h0E;
localparam ALUJMP_CMP = 7'h0F;
localparam ALUJMP_SHIFT = 7'h10;    // opcode[5:3] specifies ROL, ROR, LRCY, RRCY, SHL, SHR, SAR
localparam ALUJMP_BITTST = 7'h11;   // BT/BTS/BTR/BTC: copy bit 0 of result to CF
localparam ALUJMP_SHIFT2 = 7'h12;   // >><<? Second pass shift: executes with stored parameters
localparam ALUJMP_FLGOPS = 7'h13;   // Flag operations: CLC/STC/CLD/STD/CMC
localparam ALUJMP_PASS = 7'h14;
localparam ALUJMP_PASS2 = 7'h15;
localparam ALUJMP_AAAAAS = 7'h16;  // AAA/AAS BCD operations
localparam ALUJMP_DAADAS = 7'h17;  // DAA/DAS BCD operations
localparam ALUJMP_IMCS = 7'h04;      // IMUL Correct Sign - sets CF/OF for IMUL overflow
localparam ALUJMP_SERECO = 7'h05;    // Set/Reset/Complement for BTS/BTR/BTC (IR[5:4]: 00=BT/PASS, 01=BTS/SET, 10=BTR/RESET, 11=BTC/COMPLEMENT)
localparam ALUJMP_IMUL3 = 7'h1B;     // Start multiply, double-width result (MUL/IMUL r/m)
localparam ALUJMP_IMUL4 = 7'h1C;     // Start multiply, single-width result (IMUL r,r,i)
localparam ALUJMP_PREDIV = 7'h18;    // IDIV: compute absolute values, save signs
localparam ALUJMP_IDIV1 = 7'h19;     // IDIV: correct remainder sign
localparam ALUJMP_IDIV2 = 7'h1a;     // IDIV: correct quotient sign
localparam ALUJMP_DIV7 = 7'h1f;      // Division main loop (non-restoring algorithm)
localparam ALUJMP_DIV5 = 7'h1d;      // Division final correction
localparam ALUJMP_LDCNTR = 7'h3E;    // Load COUNTR with iteration count
localparam ALUJMP_DECNTR = 7'h3F;    // Decrement COUNTR (REP string instructions)
localparam ALUJMP_BITS8  = 7'h24;  // Set CPU to 8-bit mode for remainder of instruction (AAD/AAM/BCD)
localparam ALUJMP_BITS16 = 7'h25;  // Set CPU to 16-bit mode for remainder of instruction
localparam ALUJMP_BITS32 = 7'h27;  // Set CPU to 32-bit mode for remainder of instruction
localparam ALUJMP_CLZF = 7'h28;    // Clear Zero Flag (BSR/BSF): also copies TMPC to SRCREG
localparam ALUJMP_JNcond = 7'h41;
localparam ALUJMP_LOOPnE = 7'h43;    // LOOPE/LOOPNE: jump if COUNTR != 0 AND ZF matches REP prefix
localparam ALUJMP_JCNTZ = 7'h44;    // Jump if COUNTR == 0
localparam ALUJMP_JCNTNZ = 7'h45;   // Jump if COUNTR != 0
localparam ALUJMP_JCT4N1 = 7'h46;   // Jump if low 4 bits of COUNTR != 1 (POPA/POPAD)
localparam ALUJMP_JCNZNI = 7'h47;   // Jump if COUNTR != 0 and no interrupt (REP MOVS)
localparam ALUJMP_JCNT1 = 7'h48;    // Jump if COUNTR == 1 (LOOP exit)
localparam ALUJMP_JCNTN1 = 7'h49;   // Jump if COUNTR != 1 (ENTER inner loop)
localparam ALUJMP_JIO_OK = 7'h4B;   // Jump if CPL <= IOPL
localparam ALUJMP_JMP = 7'h5A;
localparam ALUJMP_JNT = 7'h5B;       // Jump if NT (Nested Task) bit is set (EFLAGS[14])
localparam ALUJMP_FLGSBA = 7'h38;    // FLGSBA: Set "flags backup active" flag (used before potentially faulting ops)
localparam ALUJMP_CLRNMI = 7'h23;  // Clear NMI flag (IRETD)
localparam ALUJMP_SETNMI = 7'h26;  // Set NMI blocking flag (NMI handler entry)
localparam ALUJMP_CINTLA = 7'h2D;  // Clear interrupt latch (HARDWARE_IRQ at 830)
localparam ALUJMP_SEZF = 7'h29;    // Set Zero Flag (ARPL/LSL)
localparam ALUJMP_BITSDE = 7'h2A;  // Restore instruction's original bit size
localparam ALUJMP_SCNTFF = 7'h2C;  // Set contributory fault flag
localparam ALUJMP_CLI = 7'h2E;     // Clear IF (prime clear_if_pending flag)
localparam ALUJMP_CLT = 7'h2F;    // Clear TF always, clear IF if primed by CLI
localparam ALUJMP_SINTHW = 7'h32;  // SINTHW: Set interrupt_hw flag (HW IRQ/NMI/exception, not INT n)
localparam ALUJMP_SMISC1 = 7'h33;  // Set MISC1 flag (INT handler sets this; JMISC1 tests it)
localparam ALUJMP_SMISC2 = 7'h35;  // Set MISC2 microcode flag
localparam ALUJMP_CMISC2 = 7'h3C;  // Clear MISC2 microcode flag
localparam ALUJMP_SERRCF = 7'h36;  // Set error code flag (ERROR_CODE_FLAG = true)
localparam ALUJMP_J16BIT = 7'h40;   // Jump if 286-format TSS in TR (16-bit stack switch / task save-load / IO map)
localparam ALUJMP_JNBUSY = 7'h42;   // Jump if BUSY# inactive (always taken: no FPU)
localparam ALUJMP_JTSSAF  = 7'h50;   // JTSSAF: Jump if TSS access flag is set (never taken: no task switching)
localparam ALUJMP_JG = 7'h51;        // JG: Jump if Greater (ZF=0 AND SF=OF)
localparam ALUJMP_JINTSW = 7'h52;    // JINTSW: Jump if software interrupt (!interrupt_hw)
localparam ALUJMP_JMISC1 = 7'h53;   // JMISC1: Jump if MISC1 flag set (INT vs call gate)
localparam ALUJMP_JMISC2 = 7'h55;   // JMISC2: Jump if MISC2 flag set
localparam ALUJMP_JNERRC = 7'h56;    // JNERRC: Jump if no error code (!error_code_flag)
localparam ALUJMP_JPEREQ = 7'h4E;    // JPEREQ: Jump if PEREQ (coprocessor request) — always taken (no FPU)
localparam ALUJMP_JNFLGB = 7'h58;    // JNFLGB: Jump if flags backup NOT active
localparam ALUJMP_JNO = 7'h5C;       // JNO: Jump if Not Overflow (OF=0) - used by INTO
localparam ALUJMP_JNC = 7'h5D;       // JNC: Jump if Not Carry (CF=0)
localparam ALUJMP_JNOINT = 7'h5F;    // JNOINT: Jump if no interrupt pending (always taken in our impl)
localparam ALUJMP_STSSAF = 7'h30;  // Set TSS access flag; also clears stack push mode
localparam ALUJMP_CTSSAF = 7'h3A;  // Clear TSS access flag
localparam ALUJMP_PTSAV1 = 7'h61;    // Protection test: save state 1
localparam ALUJMP_PTSAV3 = 7'h63;    // Protection test: save state 3
localparam ALUJMP_PTSAV7 = 7'h67;    // Protection test: save state 7
localparam ALUJMP_PTOVRR = 7'h68;    // Protection test: override/descriptor test
localparam ALUJMP_PTGATE = 7'h69;    // Protection test: gate pre-check
localparam ALUJMP_PTSELA = 7'h6A;    // Protection test: selector validation
localparam ALUJMP_PTGEN  = 7'h6B;    // Protection test: general (RPL/privilege ops)
localparam ALUJMP_PTSELE = 7'h6E;    // Protection test: selector with side effects
localparam ALUJMP_PTF    = 7'h6F;    // Protection test: fault/interrupt gate
localparam ALUJMP_LCALL = 7'h70;
localparam ALUJMP_LJUMP = 7'h71;
localparam ALUJMP_LJMPNP = 7'h72;   // Long jump if not privileged/protected
localparam ALUJMP_LJMP86 = 7'h73;   // Long jump if V86 mode
localparam ALUJMP_LJMPP = 7'h74;    // Long jump if protected mode enabled
localparam ALUJMP_LDBSRM = 7'h78;   // Load barrel shifter right count: SHRCNT = alu_src & BITS_V
localparam ALUJMP_LDBSLM = 7'h79;   // Load barrel shifter left count: SHLCNT = alu_src & BITS_V
localparam ALUJMP_LDBSRU = 7'h7a;   // Load barrel shifter right count: SHRCNT = alu_src & 0x1F
localparam ALUJMP_LDBSLU = 7'h7b;   // Load barrel shifter left count: SHLCNT = alu_src & 0x1F
localparam ALUJMP_RETURN = 7'h7C;
localparam ALUJMP_NOPMOVE = 7'h7F;

localparam DEST_EAX = 7'h00;
localparam DEST_ECX = 7'h01;
localparam DEST_EDX = 7'h02;
localparam DEST_EBX = 7'h03;
localparam DEST_ESP = 7'h04;
localparam DEST_DESCSW = 7'h58;   // Descriptor: CS base/limit but writable (cross-priv stack)
localparam DEST_eSP = 7'h5C;  // Size-aware ESP write (word/dword based on data32)
localparam DEST_EBP = 7'h05;
localparam DEST_ESI = 7'h06;
localparam DEST_EDI = 7'h07;
localparam DEST_EIP = 7'h08;
localparam DEST_eIP = 7'h5F;      // Size-aware EIP write
localparam DEST_EFLAGS = 7'h09;
localparam DEST_CR0 = 7'h0A;
localparam DEST_CR2 = 7'h2B;      // Page fault linear address
localparam DEST_TMPB = 7'h0B;
localparam DEST_TMPC = 7'h0C;
localparam DEST_TMPD = 7'h0D;
localparam DEST_TMPE = 7'h0E;
localparam DEST_TMPF = 7'h0F;
localparam DEST_FLAGSB = 7'h10;  // FLAGS backup register for INT
localparam DEST_TMPH = 7'h11;  // TMPH: microcode encoding 0x11 (see fields.txt)
localparam DEST_TMPG = 7'h12;  // Used by far CALL/JMP to store IP
localparam DEST_TMP_TR = 7'h13;  // TMP_TR: separate encoding 0x13, aliased to TMPH register in z386
localparam DEST_PROTUN = 7'h15;  // Protection unit register
localparam DEST_TMPeIP = 7'h16;  // Saved EIP for fault handling
localparam DEST_TMPeSP = 7'h17;  // Saved ESP for fault handling
localparam DEST_DR6 = 7'h18;     // Debug register 6
localparam DEST_DR7 = 7'h19;     // Debug register 7
localparam DEST_CSOPCD = 7'h1A;  // CS opcode temp (used by FSAVE and IRETD)
localparam DEST_FSVeIP = 7'h1B;  // Saved EIP for FPU/segment operations
localparam DEST_OPROFF = 7'h1C;  // Operand offset
localparam DEST_MDTMP = 7'h1D;       // Multiplier/divider temp register
localparam DEST_MDTMP4 = 7'h14;     // Multiplier/divider temp register (alternate)

localparam DEST_ES = 7'h20;
localparam DEST_CS = 7'h21;
localparam DEST_SS = 7'h22;
localparam DEST_DS = 7'h23;
localparam DEST_FS = 7'h24;
localparam DEST_GS = 7'h25;
localparam DEST_LDTR = 7'h26;
localparam DEST_eAX_AL = 7'h28;
localparam DEST_TR = 7'h29;
localparam DEST_eDX_AH = 7'h2A;
localparam DEST_OPR_W = 7'h2D;

localparam DEST_eCX = 7'h31;
localparam DEST_COUNTR = 7'h32;
localparam DEST_SLCTR = 7'h35;  // Selector temp for protected mode descriptor load
localparam DEST_eSI = 7'h36;
localparam DEST_eDI = 7'h37;
localparam DEST_FLAGSL = 7'h38;
localparam DEST_IRF = 7'h39;     // Indirect Register File access (POPA/POPAD)
localparam DEST_COUNT5 = 7'h3A;  // COUNTR with only low 5 bits (RCL/RCR/ENTER/call gate)
localparam DEST_SEGREG = 7'h3C;
localparam DEST_DSTREG = 7'h3D;
localparam DEST_SRCREG = 7'h3E;

localparam DEST_AX = 7'h40;
localparam DEST_CX = 7'h41;
localparam DEST_DX = 7'h42;
localparam DEST_BX = 7'h43;
localparam DEST_SP = 7'h44;
localparam DEST_BP = 7'h45;
localparam DEST_SI = 7'h46;
localparam DEST_DI = 7'h47;
localparam DEST_IP = 7'h48;
localparam DEST_FLAGS = 7'h49;
localparam DEST_DESPTR = 7'h4D;  // Descriptor mentioned in dest of last SPTR
localparam DEST_DESSDT = 7'h4E;   // Descriptor select for GDT/LDT based on selector TI bit

localparam DEST_AL = 7'h50;
localparam DEST_CL = 7'h51;
localparam DEST_DL = 7'h52;
localparam DEST_BL = 7'h53;
localparam DEST_AH = 7'h54;
localparam DEST_CH = 7'h55;
localparam DEST_DH = 7'h56;
localparam DEST_BH = 7'h57;

localparam DEST_DESCOD = 7'h59;   // Code descriptor for EIP changes (CS segment select)
localparam DEST_DESSTK = 7'h5A;  // Stack descriptor for stack reads/writes (SS segment select)
localparam DEST_DES_OS = 7'h5D;  // DES_OS - Use EA's default segment with override
localparam DEST_DES_SR = 7'h5E;  // DES_SR - Segment register selected by instruction (for SBRM)
localparam DEST_DES_ES = 7'h60;  // DES_ES - ES segment (string destination)
localparam DEST_DES_CS = 7'h61;  // DES_CS - CS descriptor cache write
localparam DEST_DES_SS = 7'h62;  // DES_SS - SS descriptor cache write
localparam DEST_DES_DS = 7'h63;  // DES_DS - DS descriptor cache write
localparam DEST_DES_FS = 7'h64;  // DES_FS - FS descriptor cache write
localparam DEST_DES_GS = 7'h65;  // DES_GS - GS descriptor cache write
localparam DEST_DESLDT = 7'h66;  // DESLDT - LDTR descriptor cache write
localparam DEST_DESGDT = 7'h67;  // DESGDT - GDTR pseudo-descriptor write
localparam DEST_DESIDT = 7'h68;  // DESIDT - IDT segment for interrupt handling
localparam DEST_DES_TR = 7'h69;  // DES_TR - TR descriptor cache write
localparam DEST_DESABS = 7'h6A;  // DESABS - No paging translation (also CR3/TRn writes)
localparam DEST_DES_IO = 7'h6B;  // IO port address destination
localparam DEST_DESERR = 7'h6C;  // Descriptor that caused fault
localparam DEST_LATTTF = 7'h78;  // Faulting linear address (page fault)
localparam DEST_PFERRC = 7'h7A;  // Page fault error code
localparam DEST_PDBR = 7'h7B;    // Page directory base register (CR3) for LPCR reads
localparam DEST_PAGER5 = 7'h7D;  // Page cache register (paging-related, NOP for now)
localparam DEST_USTEP_ALU = 7'h7E; // Optimized FAST word: ALU result -> DSTREG

// ALU Source field (ABCDEF) constants
localparam ALUSRC_EAX = 6'h00;
localparam ALUSRC_ECX = 6'h01;
localparam ALUSRC_EDX = 6'h02;
localparam ALUSRC_EBX = 6'h03;
localparam ALUSRC_ESP = 6'h04;
localparam ALUSRC_EBP = 6'h05;
localparam ALUSRC_ESI = 6'h06;
localparam ALUSRC_EDI = 6'h07;
localparam ALUSRC_IMM8 = 6'h08;       // Sign-extended 8-bit immediate
localparam ALUSRC_IMM = 6'h09;        // Full immediate
localparam ALUSRC_TMPB = 6'h0B;
localparam ALUSRC_TMPC = 6'h0C;
localparam ALUSRC_TMPD = 6'h0D;
// z386x extension (M5 F-ALUM): OPR_R as an ALU source. 0x0F is unused as a consumed ALU source in the base CROM (only 880/88E carry it,...
// Details: doc/z386x/implementation_notes.md#src-24-z386x-z386-pkg-sv-857
localparam ALUSRC_OPR_R = 6'h0F;
localparam ALUSRC_ALLONES = 6'h10;    // 0xFFFFFFFF mask
localparam ALUSRC_TMPG = 6'h12;
localparam ALUSRC_TMPH = 6'h13;
localparam ALUSRC_PROTUN = 6'h15;
localparam ALUSRC_FLAGS_MASK = 6'h16;  // 0x37fd7 FLAGS mask for INT stack push
localparam ALUSRC_CONST_4000 = 6'h17;  // 0x4000
localparam ALUSRC_CONST_N200 = 6'h18;  // ~0x200 = 0xFFFFFDFF
localparam ALUSRC_CONST_8 = 6'h19;     // 8
localparam ALUSRC_CONST_40 = 6'h1A;    // 0x40
localparam ALUSRC_CONST_F0000 = 6'h1B; // 0xF0000
localparam ALUSRC_CONST_0D = 6'h1C;    // 0x0D (13)
localparam ALUSRC_CONST_5D = 6'h1D;    // 0x5D (93)
localparam ALUSRC_SIGMA = 6'h1E;       // NOTE: fields.txt says 0x800000f8, but we use SIGMA
localparam ALUSRC_CONST_FC = 6'h1F;    // 0x800000fc (FPU port address)
localparam ALUSRC_CONST_70 = 6'h20;    // 0x70
localparam ALUSRC_CONST_73 = 6'h21;    // 0x73
localparam ALUSRC_CONST_1FF = 6'h22;   // 0x1FF descriptor-base mask helper (LGDT/LIDT)
localparam ALUSRC_CONST_8200 = 6'h23;  // 0x8200
localparam ALUSRC_CONST_71 = 6'h24;    // 0x47 (71) for POPA LDCNTR
localparam ALUSRC_CONST_3 = 6'h25;
localparam ALUSRC_CONST_6 = 6'h26;
localparam ALUSRC_CONST_FFFF0000 = 6'h27; // 0xFFFF0000
localparam ALUSRC_CONST_60 = 6'h28;    // 0x60
localparam ALUSRC_CONST_7FF = 6'h29;   // 0x7FF
localparam ALUSRC_CONST_9 = 6'h2A;
localparam ALUSRC_CONST_29 = 6'h2B;    // 0x29 (41)
localparam ALUSRC_CONST_7 = 6'h2C;     // For AAD/AAM 8-bit multiply
localparam ALUSRC_CONST_0F = 6'h2D;    // 0x0F (15)
localparam ALUSRC_CONST_65 = 6'h2E;    // 0x65 (101)
localparam ALUSRC_CONST_1F = 6'h2F;    // 0x1F (31)
localparam ALUSRC_CONST_1 = 6'h30;
localparam ALUSRC_CONST_2 = 6'h31;
localparam ALUSRC_CONST_16 = 6'h32;   // 0x10 (16) for INT shift right by 16
localparam ALUSRC_CONST_4 = 6'h33;
localparam ALUSRC_CONST_NEG1 = 6'h34; // -1 (0xFFFFFFFF)
localparam ALUSRC_CONST_NEG2 = 6'h35; // -2
localparam ALUSRC_MASK16 = 6'h36;     // 0x0000FFFF
localparam ALUSRC_CONST_NEG4 = 6'h37; // -4
localparam ALUSRC_CONST_0 = 6'h38;    // Zero
localparam ALUSRC_WORDSZ = 6'h39;     // 2 or 4 based on data32
localparam ALUSRC_NEGWSZ = 6'h3A;     // -2 or -4 based on data32
localparam ALUSRC_INCREM = 6'h3B;     // String increment (±1/±2/±4)
localparam ALUSRC_BITS_V = 6'h3C;     // Width minus one (7/15/31)
localparam ALUSRC_DSTREG = 6'h3D;
localparam ALUSRC_SRCREG = 6'h3E;
localparam ALUSRC_ZERO = 6'h3F;       // Zero (alternate)

// Source field (NOPQRS) constants
localparam SRC_EAX = 6'h00;
localparam SRC_ECX = 6'h01;
localparam SRC_EDX = 6'h02;
localparam SRC_EBX = 6'h03;
localparam SRC_ESP = 6'h04;
localparam SRC_EBP = 6'h05;
localparam SRC_ESI = 6'h06;
localparam SRC_EDI = 6'h07;
localparam SRC_EIP = 6'h08;
localparam SRC_EFLAGS = 6'h09;
localparam SRC_CR0 = 6'h0A;           // Control register 0
localparam SRC_CR2 = 6'h2B;           // Page fault linear address
localparam SRC_TMPB = 6'h0B;
localparam SRC_TMPC = 6'h0C;
localparam SRC_TMPD = 6'h0D;
localparam SRC_TMPE = 6'h0E;
localparam SRC_TMPF = 6'h0F;
localparam SRC_FLAGSB = 6'h10;    // FLAGS backup register for INT
localparam SRC_TMPH = 6'h11;         // TMPH: microcode encoding 0x11 (see fields.txt)
localparam SRC_TMPG = 6'h12;
localparam SRC_TMP_TR = 6'h13;       // TMP_TR/SLCTR2: encoding 0x13, aliased to TMPH register in z386
localparam SRC_SLCTR2 = 6'h13;       // Alias: SLCTR2 and TMP_TR share encoding 0x13
localparam SRC_COUNTR = 6'h14;
localparam SRC_PROTUN = 6'h15;   // Protection unit register
localparam SRC_TMPeIP = 6'h16;
localparam SRC_TMPeSP = 6'h17;        // Saved ESP for fault handling
localparam SRC_DR6 = 6'h18;           // Debug register 6
localparam SRC_DR7 = 6'h19;           // Debug register 7
localparam SRC_CSOPCD = 6'h1A;        // CS opcode temp (also called TMPM)
localparam SRC_OPROFF = 6'h1C;        // Operand offset
localparam SRC_MDTMP = 6'h1D;         // Multiplier/divider temp (quotient/product)
localparam SRC_SIGMA = 6'h1E;
localparam SRC_IMM = 6'h1F;
localparam SRC_ES = 6'h20;
localparam SRC_CS = 6'h21;
localparam SRC_SS = 6'h22;
localparam SRC_DS = 6'h23;
localparam SRC_FS = 6'h24;
localparam SRC_GS = 6'h25;
localparam SRC_LDTR = 6'h26;
localparam SRC_TR = 6'h29;
localparam SRC_eAX_AL = 6'h28;        // Size-aware accumulator (AL/AX/EAX)
localparam SRC_eDX_AH = 6'h2A;        // Size-aware upper (AH/DX/EDX)
localparam SRC_OPR_R = 6'h2D;
localparam SRC_IRF2 = 6'h2E;          // IRF2 / effective address
localparam SRC_eCX = 6'h31;           // Address-size-aware CX/ECX
localparam SRC_COUNTR2 = 6'h32;       // COUNTR (alternate)
localparam SRC_EA = 6'h36;            // Effective address
localparam SRC_SLCTR = 6'h35;
localparam SRC_ZERO = 6'h38;
localparam SRC_IRF = 6'h39;           // Indirect register file (PUSHA/PUSHAD)
localparam SRC_SEGREG = 6'h3C;
localparam SRC_DSTREG = 6'h3D;
localparam SRC_SRCREG = 6'h3E;
localparam SRC_NEG1 = 6'h3F;          // -1 (all ones)

// Bus operation codes (6-bit field from microcode)
// Read operations
localparam BUSOP_RD_BW = 6'h06;       // RD b/w - Memory read (byte/word based on operand size)
localparam BUSOP_RD_D = 6'h07;        // RD_D - Descriptor/auxiliary dword read
localparam BUSOP_RD_WORD = 6'h15;     // RD w - Word read (POP seg, MOV Sreg,[mem], LDS/LES)
localparam BUSOP_RD = 6'h16;          // RD - Memory read

// Write operations
localparam BUSOP_WR_WORD = 6'h11;     // WR w - Word write (PUSH seg, MOV [mem],Sreg)
localparam BUSOP_WR = 6'h12;          // WR - Memory write
localparam BUSOP_WR_D = 6'h13;        // WR D - Dword write (SGDT/SIDT base store)
localparam BUSOP_WR_OPR = 6'h1A;      // wr - Write-back using OPR_R (string ops, PUSH [mem])

// Check write (probe)
localparam BUSOP_CW = 6'h0A;          // CW - Check write permission without actual write (ENTER)

// IND register operations
localparam BUSOP_IND_PLUS = 6'h1F;    // IN+= - IND = IND + alu_src
localparam BUSOP_IND_ALU2 = 6'h24;    // IN=2 - Set IND from ALU2 (alu_src)
localparam BUSOP_IND_SRC = 6'h25;     // IND= - Set IND from source register
// Bus code 0x26 has dual use:
// - Stack write (PUSH) when used with normal destinations
// - IN=+ (IND = src + alu_op) when dest=IND_IP (Jcc fall-through calculation)
localparam BUSOP_IND_PLUS_ALU = 6'h26; // IN=+ - IND = alu_dst + alu_src

// Prefetch control
localparam BUSOP_PREF = 6'h1C;        // PREF - Flush queue and restart prefetch at IND
localparam BUSOP_IN_PLUS_D = 6'h1D;   // IN+D - IND += IND_DELTA (PUSHA/PUSHAD loop)

// Segment operations
localparam BUSOP_SBRM = 6'h2C;        // Set Base Real Mode - update segment descriptor cache base

// Interrupt/descriptor operations
localparam BUSOP_RD_IND = 6'h17;      // {17} - Read 32 bits, also copy lower 16 bits to IND (for INT IVT read)
localparam BUSOP_SDEH = 6'h2D;        // Descriptor cache base write helper
localparam BUSOP_SDES = 6'h2E;        // Descriptor cache finalize helper
localparam BUSOP_SDEL = 6'h2F;        // Descriptor cache attribute write helper
localparam BUSOP_SPCR = 6'h30;        // SPCR - Store Page Cache Register / set stack decrement mode
localparam BUSOP_LPCR = 6'h34;        // LPCR - Load Page Cache Register into IRF2 (page fault error code / faulting address)
localparam BUSOP_LLIM = 6'h37;        // LLIM - Load limit from descriptor cache into IRF2
localparam BUSOP_LAR = 6'h35;         // LAR - Load access rights from descriptor cache into IRF2
localparam BUSOP_LBAS = 6'h36;        // LBAS - Load base from descriptor cache into IRF2
localparam BUSOP_SPTR = 6'h39;        // SPTR - Makes DESPTR correspond to dest (tracks target descriptor)
localparam BUSOP_LDSG = 6'h39;        // Alias: LDSG and SPTR share encoding 0x39
localparam BUSOP_SAR = 6'h31;         // Select AR field of descriptor cache
localparam BUSOP_SBAS = 6'h32;        // Select BASE field of descriptor cache
localparam BUSOP_SLIM = 6'h33;        // Select LIMIT field of descriptor cache
localparam BUSOP_36 = 6'h36;          // SIDT/SGDT helper
localparam BUSOP_HLTS = 6'h3C;        // HLTS - Enter halt state
localparam BUSOP_IACK = 6'h28;        // IACK - Interrupt acknowledge bus cycle

// Microcode fault entry points
// Fault entry points - these set SIGMA to vector-9, then at FAULT_ERR_CODE adds 9 to get vector
// Vector calculation: SIGMA + 9 = vector (GP: 4+9=13, SS: 3+9=12)
localparam [11:0] UADDR_GENERAL_FAULT1 = 12'h85B;  // #GP(0) - sets SIGMA=4, error code=0
localparam [11:0] UADDR_STACK_FAULT    = 12'h863;  // #SS(0) - sets SIGMA=3, error code=0
localparam [11:0] UADDR_DIVIDE_ERROR   = 12'h824;  // #DE(0) - divide error (vector 0)
localparam [11:0] UADDR_DOUBLE_FAULT   = 12'h83F;  // #DF - vector 8, zero error code
localparam [11:0] UADDR_HARDWARE_IRQ   = 12'h82D;  // INTR handler entry point
localparam [11:0] UADDR_NMI            = 12'h836;  // NMI handler entry point
localparam [11:0] UADDR_PRIV_INT_DONE  = 12'h639;  // Cross-privilege handler CS/SS committed
localparam [11:0] UADDR_TRAP_INT_DONE  = 12'h8E3;  // Handler CS committed; delivery complete
localparam [11:0] UADDR_TSS_PROBLEM    = 12'h85D;  // #TS path used by protected-mode descriptor checks

// Group 2 instructions
localparam ROL = 3'b000;
localparam ROR = 3'b001;
localparam RCL = 3'b010;
localparam RCR = 3'b011;
localparam SHL = 3'b100;
localparam SHR = 3'b101;
localparam SAL = 3'b110;  // Undocumented encoding, behaves like SHL
localparam SAR = 3'b111;

// ALU operations
localparam [4:0] ALU_ADD  = 5'b00000;
localparam [4:0] ALU_OR   = 5'b00001;
localparam [4:0] ALU_ADC  = 5'b00010;
localparam [4:0] ALU_SBB  = 5'b00011;
localparam [4:0] ALU_AND  = 5'b00100;
localparam [4:0] ALU_SUBT = 5'b00101;
localparam [4:0] ALU_XOR  = 5'b00110;
localparam [4:0] ALU_CMP  = 5'b00111;

// NOTE: Shift operations (ROL, ROR, RCL, RCR, SHL, SHR, SAR) are handled
// by the dedicated shifter module, not the ALU.
localparam [4:0] ALU_ANDN = 5'b01011;    // AND-NOT: dst & ~src (for BTR instruction)
localparam [4:0] ALU_AAS  = 5'b01110;  // ASCII adjust after subtraction
localparam [4:0] ALU_SIGN = 5'b01111; // Get sign of alu_dst

localparam [4:0] ALU_PASS = 5'b10000;
localparam [4:0] ALU_PASS2= 5'b10001;  // PASS with swapped src/dst operands
localparam [4:0] ALU_ZEXT = 5'b10010;  // Zero extension: op_size=dest, extends (dest-1)
localparam [4:0] ALU_SEXT = 5'b10011;  // Sign extension: op_size=dest, extends (dest-1)
localparam [4:0] ALU_ZEXT_B= 5'b10100;  // Zero extension: always extends byte
localparam [4:0] ALU_SEXT_B= 5'b10101;  // Sign extension: always extends byte
localparam [4:0] ALU_DAA  = 5'b10110;
localparam [4:0] ALU_DAS  = 5'b10111;

localparam [4:0] ALU_INC  = 5'b11000;
localparam [4:0] ALU_DEC  = 5'b11001;
localparam [4:0] ALU_NOT  = 5'b11010;
localparam [4:0] ALU_NEG  = 5'b11011;
localparam [4:0] ALU_INC2 = 5'b11100;
localparam [4:0] ALU_DEC2 = 5'b11101;
localparam [4:0] ALU_SEXTD= 5'b11110;  // Sign extension for CWD/CDQ (produces DX/EDX value)
localparam [4:0] ALU_AAA  = 5'b11111;

//=============================================================================
// Utility Functions
//=============================================================================

// Byte enable calculation based on operand size and address
function automatic [3:0] calc_be(input [1:0] op_size, input [1:0] addr_low);
    case (op_size)
        2'd0: begin  // Byte
            case (addr_low)
                2'b00: calc_be = 4'b0001;
                2'b01: calc_be = 4'b0010;
                2'b10: calc_be = 4'b0100;
                2'b11: calc_be = 4'b1000;
            endcase
        end
        2'd1: begin  // Word (16-bit)
            case (addr_low)
                2'b00: calc_be = 4'b0011;  // bytes 0-1
                2'b01: calc_be = 4'b0110;  // bytes 1-2
                2'b10: calc_be = 4'b1100;  // bytes 2-3
                2'b11: calc_be = 4'b1000;  // byte 3 only (unaligned, needs second access)
            endcase
        end
        2'd2: calc_be = 4'b1111;  // Dword (32-bit)
        default: calc_be = 4'b1111;
    endcase
endfunction

// Shift write data to correct byte position based on operand size and address alignment
function automatic [31:0] shift_write_data(input [31:0] data, input [1:0] op_size, input [1:0] addr_low);
    case (op_size)
        2'd0: begin  // Byte - replicate to all positions for flexibility
            case (addr_low)
                2'b00: shift_write_data = {24'h0, data[7:0]};
                2'b01: shift_write_data = {16'h0, data[7:0], 8'h0};
                2'b10: shift_write_data = {8'h0, data[7:0], 16'h0};
                2'b11: shift_write_data = {data[7:0], 24'h0};
            endcase
        end
        2'd1: begin  // Word (16-bit)
            case (addr_low)
                2'b00: shift_write_data = {16'h0, data[15:0]};         // bytes 0-1
                2'b01: shift_write_data = {8'h0, data[15:0], 8'h0};    // bytes 1-2
                2'b10: shift_write_data = {data[15:0], 16'h0};         // bytes 2-3
                2'b11: shift_write_data = {data[7:0], 24'h0};          // byte 3 only (incomplete)
            endcase
        end
        default: shift_write_data = data;  // Dword - no shift needed
    endcase
endfunction

// Check if memory access crosses dword boundary (needs 2 bus cycles)
function automatic logic access_crosses_dword(input [1:0] op_size, input [1:0] addr_low);
    case (op_size)
        2'd0: access_crosses_dword = 1'b0;  // Byte never crosses
        2'd1: access_crosses_dword = (addr_low == 2'b11);  // Word at offset 3 crosses
        2'd2: access_crosses_dword = (addr_low != 2'b00);  // Dword at offset 1,2,3 crosses
        default: access_crosses_dword = 1'b0;
    endcase
endfunction

// Calculate byte enables for first access of a crossing operation
function automatic [3:0] calc_be_first(input [1:0] op_size, input [1:0] addr_low);
    case (op_size)
        2'd1: calc_be_first = 4'b1000;  // Word at offset 3: only byte 3 in first dword
        2'd2: begin  // Dword
            case (addr_low)
                2'b01: calc_be_first = 4'b1110;  // bytes 1,2,3
                2'b10: calc_be_first = 4'b1100;  // bytes 2,3
                2'b11: calc_be_first = 4'b1000;  // byte 3 only
                default: calc_be_first = 4'b1111;
            endcase
        end
        default: calc_be_first = 4'b1111;
    endcase
endfunction

// Calculate byte enables for second access of a crossing operation
function automatic [3:0] calc_be_second(input [1:0] op_size, input [1:0] addr_low);
    case (op_size)
        2'd1: calc_be_second = 4'b0001;  // Word at offset 3: only byte 0 in second dword
        2'd2: begin  // Dword
            case (addr_low)
                2'b01: calc_be_second = 4'b0001;  // byte 0 only
                2'b10: calc_be_second = 4'b0011;  // bytes 0,1
                2'b11: calc_be_second = 4'b0111;  // bytes 0,1,2
                default: calc_be_second = 4'b0000;
            endcase
        end
        default: calc_be_second = 4'b0000;
    endcase
endfunction

// Combine data from two dword reads for unaligned access
function automatic [31:0] combine_unaligned_read(
    input [31:0] din1, input [31:0] din2,
    input [1:0] addr_low, input [1:0] op_size
);
    case (op_size)
        2'd1: begin  // Word at offset 3
            combine_unaligned_read = {16'h0, din2[7:0], din1[31:24]};
        end
        2'd2: begin  // Dword
            case (addr_low)
                2'b01: combine_unaligned_read = {din2[7:0], din1[31:8]};
                2'b10: combine_unaligned_read = {din2[15:0], din1[31:16]};
                2'b11: combine_unaligned_read = {din2[23:0], din1[31:24]};
                default: combine_unaligned_read = din1;
            endcase
        end
        default: combine_unaligned_read = din1;
    endcase
endfunction

// Split write data for first access of unaligned write
function automatic [31:0] split_write_first(input [31:0] data, input [1:0] addr_low, input [1:0] op_size);
    case (op_size)
        2'd1: split_write_first = {data[7:0], 24'h0};  // Word at offset 3: low byte to byte 3
        2'd2: begin  // Dword
            case (addr_low)
                2'b01: split_write_first = {data[23:0], 8'h0};  // bytes 0-2 to positions 1-3
                2'b10: split_write_first = {data[15:0], 16'h0}; // bytes 0-1 to positions 2-3
                2'b11: split_write_first = {data[7:0], 24'h0};  // byte 0 to position 3
                default: split_write_first = data;
            endcase
        end
        default: split_write_first = data;
    endcase
endfunction

// Split write data for second access of unaligned write
function automatic [31:0] split_write_second(input [31:0] data, input [1:0] addr_low, input [1:0] op_size);
    case (op_size)
        2'd1: split_write_second = {24'h0, data[15:8]};  // Word at offset 3: high byte to byte 0
        2'd2: begin  // Dword
            case (addr_low)
                2'b01: split_write_second = {24'h0, data[31:24]};  // byte 3 to position 0
                2'b10: split_write_second = {16'h0, data[31:16]};  // bytes 2-3 to positions 0-1
                2'b11: split_write_second = {8'h0, data[31:8]};    // bytes 1-3 to positions 0-2
                default: split_write_second = 32'h0;
            endcase
        end
        default: split_write_second = 32'h0;
    endcase
endfunction

// Even parity calculation
function automatic logic even_parity(input [31:0] value, input use32);
    if (use32)
        even_parity = ~^value;
    else
        even_parity = ~^value[15:0];
endfunction

// LOCK prefix validation
// LOCK is valid if and only if: instruction performs read-modify-write on memory operand
// Returns 1 if LOCK prefix is INVALID (should trigger #UD)
function automatic logic check_lock_invalid(
    input [1:0] rep_lock,
    input       has_0f,
    input [7:0] opcode,
    input       has_modrm,
    input [7:0] modrm
);
    // All variable declarations must be at the top
    logic lock_prefix;
    logic has_mem_operand;
    logic [2:0] modrm_reg;
    logic lock_valid_alu, lock_valid_grp1, lock_valid_grp3, lock_valid_grp45, lock_valid_xchg;
    logic lock_valid_0f_bts, lock_valid_0f_ba, lock_valid_0f_cmpxchg;
    logic lock_valid;

    lock_prefix = (rep_lock == PREFIX_LOCK);
    has_mem_operand = has_modrm && (modrm[7:6] != 2'b11);  // mod != 11
    modrm_reg = modrm[5:3];

    // Valid LOCK cases for non-0F opcodes:
    // 1. ALU ops 00-31 (ADD/OR/ADC/SBB/AND/SUB/XOR) x0/x1 forms - NOT CMP (38-3D)
    lock_valid_alu = !has_0f &&
                     (opcode[7:6] == 2'b00) &&          // 00-3F range
                     (opcode[2:0] <= 3'b001) &&         // x0, x1 forms only
                     (opcode[5:3] != 3'b111) &&         // Not CMP
                     has_mem_operand;

    // 2. Group 1 (80-83): ALU r/m, imm - NOT CMP (reg=7)
    lock_valid_grp1 = !has_0f &&
                      (opcode[7:2] == 6'b100000) &&     // 80-83
                      (modrm_reg != 3'b111) &&          // Not CMP
                      has_mem_operand;

    // 3. Group 3 (F6/F7): NOT (reg=2), NEG (reg=3)
    lock_valid_grp3 = !has_0f &&
                      (opcode[7:1] == 7'b1111011) &&    // F6/F7
                      (modrm_reg == 3'b010 || modrm_reg == 3'b011) && // NOT or NEG
                      has_mem_operand;

    // 4. Group 4/5 (FE/FF): INC (reg=0), DEC (reg=1)
    lock_valid_grp45 = !has_0f &&
                       (opcode[7:1] == 7'b1111111) &&   // FE/FF
                       (modrm_reg == 3'b000 || modrm_reg == 3'b001) && // INC or DEC
                       has_mem_operand;

    // 5. XCHG (86/87) - always read-modify-write when memory operand
    lock_valid_xchg = !has_0f &&
                      (opcode[7:1] == 7'b1000011) &&    // 86/87
                      has_mem_operand;

    // Valid LOCK cases for 0F-prefixed opcodes:
    // 6. BTS (AB), BTR (B3), BTC (BB)
    lock_valid_0f_bts = has_0f &&
                        (opcode == 8'hAB || opcode == 8'hB3 || opcode == 8'hBB) &&
                        has_mem_operand;

    // 7. 0F BA: BTS/BTR/BTC imm (reg=5,6,7) - NOT BT (reg=4, read-only)
    lock_valid_0f_ba = has_0f &&
                       (opcode == 8'hBA) &&
                       (modrm_reg >= 3'b101) &&         // /5, /6, /7 (not /4 BT)
                       has_mem_operand;

    // 8. CMPXCHG (0F B0/B1), XADD (0F C0/C1)
    lock_valid_0f_cmpxchg = has_0f &&
                            (opcode[7:1] == 7'b1011000 ||  // B0/B1 CMPXCHG
                             opcode[7:1] == 7'b1100000) && // C0/C1 XADD
                            has_mem_operand;

    lock_valid = lock_valid_alu || lock_valid_grp1 || lock_valid_grp3 ||
                 lock_valid_grp45 || lock_valid_xchg ||
                 lock_valid_0f_bts || lock_valid_0f_ba || lock_valid_0f_cmpxchg;

    check_lock_invalid = lock_prefix && !lock_valid;
endfunction

//=============================================================================
// Paging Unit Types and Constants
//=============================================================================

// Page fault microcode entry point
localparam [11:0] UADDR_PAGE_FAULT = 12'h8E9;

// TLB entry structure - 4 entries fully-associative
typedef struct packed {
    logic        valid;         // Entry is valid
    logic [19:0] vpn;           // Virtual Page Number (tag) - linear[31:12]
    logic [19:0] pfn;           // Physical Frame Number
    logic        writable;      // R/W permission (combined PDE & PTE)
    logic        user;          // U/S permission (combined PDE & PTE)
    logic        dirty;         // D bit from PTE
    logic        accessed;      // A bit from PTE
} tlb_entry_t;

// Page table entry format (PDE and PTE have same format)
// Bits: [31:12] = frame address, [11:9] = avail, [8] = G, [7] = PS/0,
//       [6] = D, [5] = A, [4] = PCD, [3] = PWT, [2] = U/S, [1] = R/W, [0] = P
localparam PTE_P   = 0;   // Present
localparam PTE_RW  = 1;   // Read/Write
localparam PTE_US  = 2;   // User/Supervisor
localparam PTE_PWT = 3;   // Page Write-Through
localparam PTE_PCD = 4;   // Page Cache Disable
localparam PTE_A   = 5;   // Accessed
localparam PTE_D   = 6;   // Dirty

// Page fault error code bits
localparam PF_P = 0;      // 0=not present, 1=protection violation
localparam PF_W = 1;      // 0=read, 1=write
localparam PF_U = 2;      // 0=supervisor, 1=user

// Protection Unit (PLA4) Test Constants
// Details: doc/z386x/implementation_notes.md#src-24-z386x-z386-pkg-sv-1339

// Selector validation tests (PLA4 test constants 0x00-0x0F)
localparam logic [5:0] TST_SEL_NONSS    = 6'h00;  // Non-stack segment selector (DS/ES/FS/GS)
localparam logic [5:0] TST_SEL_CS       = 6'h01;  // Code segment selector
localparam logic [5:0] TST_SEL_RET      = 6'h02;  // Far return selector (privilege transition)
localparam logic [5:0] TST_SEL_RET_OL   = 6'h03;
localparam logic [5:0] TST_PORTIO_BIT   = 6'h04;
localparam logic [5:0] TST_SEL_ARPL     = 6'h05;  // ARPL selector check
localparam logic [5:0] TST_SEL_GDT      = 6'h06;  // GDT selector check
localparam logic [5:0] TST_SEL_LLDT     = 6'h07;  // Load LDT selector
localparam logic [5:0] TST_SEL_LLVV     = 6'h08;  // LAR/LSL selector validation
localparam logic [5:0] TST_SEL_MOREPR   = 6'h09;
localparam logic [5:0] TST_SEL_TASKGT   = 6'h0A;  // Task gate selector
localparam logic [5:0] TST_SEL_TASKFI   = 6'h0B;  // Task gate selector for task switch (TSS read)
localparam logic [5:0] TST_SEL_SS       = 6'h0C;       // Stack segment selector
localparam logic [5:0] TST_SEL_TR_TSF   = 6'h0D;    // Task Register selector for task switch (TSS read)

// Descriptor validation tests (PLA4 test constants 0x10-0x2F)
localparam logic [5:0] TST_DES_SIMPLE   = 6'h10;  // Simple descriptor test (non-stack)
localparam logic [5:0] TST_DES_SS       = 6'h11;  // Stack segment descriptor (CPL==DPL==RPL)
localparam logic [5:0] TST_DES_JMP      = 6'h12;  // Far jump descriptor (gate detection)
localparam logic [5:0] TST_DES_JGATE    = 6'h13;  // Jump gate type dispatch (386/286 call gate, task gate, TSS)
localparam logic [5:0] TST_DES_JGDEST   = 6'h14;  // Jump gate destination code segment
localparam logic [5:0] TST_DES_CALL     = 6'h15;  // Call descriptor (gate detection)
localparam logic [5:0] TST_DES_CGATE    = 6'h16;  // Call gate descriptor (gate type discrimination)
localparam logic [5:0] TST_DES_CGDEST   = 6'h17;  // Call gate destination (conforming/other)
localparam logic [5:0] TST_DES_RETF     = 6'h18;  // Far return descriptor
localparam logic [5:0] TST_DES_RTOLSS   = 6'h19;
localparam logic [5:0] TST_DES_LDTTSK   = 6'h1A;
localparam logic [5:0] TST_DES_LDT      = 6'h1B;
localparam logic [5:0] TST_DES_MOREPR   = 6'h1C;
localparam logic [5:0] TST_DES_TSSTSG   = 6'h1D;
localparam logic [5:0] TST_DES_TSSTSR   = 6'h1E;  // TSS descriptor for task switch read
localparam logic [5:0] TST_DES_TSSLTR   = 6'h1F;  // TSS descriptor for LTR
localparam logic [5:0] TST_DES_GRANUL   = 6'h20;  // LSL granularity check
localparam logic [5:0] TST_DES_INT_SW   = 6'h21;  // Software interrupt gate type (INT3/CALLF/IRET)
localparam logic [5:0] TST_DES_INT_HW   = 6'h22;  // Interrupt gate type (INT/TRAP/TASK)
localparam logic [5:0] TST_ACCESS_VIO   = 6'h24;
localparam logic [5:0] TST_DES_RTOLOS   = 6'h25;
localparam logic [5:0] TST_SEL_LES      = 6'h26;
localparam logic [5:0] TST_SEL_LDS      = 6'h27;  // LDS/LES selector check
localparam logic [5:0] TST_SEL_LFSLGS   = 6'h28;  // LFS/LGS/LSS selector check
localparam logic [5:0] JMP_GFAULT_INT   = 6'h2A;

// RPL/privilege operations (always pass, set control flags)
localparam logic [5:0] READ_RPL         = 6'h2B;  // Read RPL from selector
localparam logic [5:0] WRITE_RPL        = 6'h2C;  // Write RPL to selector
localparam logic [5:0] SET_RPL_TO_CPL   = 6'h2D;  // Set RPL to CPL (cross-privilege detection)
localparam logic [5:0] COPY_STACK_DPL   = 6'h2E;  // Copy stack DPL
localparam logic [5:0] SET_FAULT        = 6'h2F;  // Set fault flag

// Verification/access tests (VERR/VERW/LAR/LSL instructions)
localparam logic [5:0] TST_DES_LAR      = 6'h30;  // LAR descriptor test
localparam logic [5:0] TST_DES_LSL      = 6'h31;  // LSL descriptor test
localparam logic [5:0] TST_DES_VERR     = 6'h32;  // Verify segment readable
localparam logic [5:0] TST_DES_VERW     = 6'h33;  // Verify segment writable

// FPU tests (CR0 flag checks and 287/387 discrimination via CR0.ET)
localparam logic [5:0] FPU_WAIT         = 6'h34;  // FPU Wait (check CR0.TS/MP/EM)
localparam logic [5:0] FPU_OTHER        = 6'h38;  // FPU Other Operations
localparam logic [5:0] FPU_LOAD_3264    = 6'h39;  // FPU Load 32/64-bit (287 vs 387 path)
localparam logic [5:0] FPU_LOAD_80      = 6'h3B;  // FPU Load 80-bit (287 vs 387 path)
localparam logic [5:0] FPU_STORE_3264   = 6'h3C;  // FPU Store 32/64-bit (287 vs 387 path)
localparam logic [5:0] FPU_STORE_80     = 6'h3D;  // FPU Store 80-bit (287 vs 387 path)
localparam logic [5:0] FPU_FSAVE        = 6'h3E;  // FPU Save State (287 vs 387 via CR0.ET)
localparam logic [5:0] FPU_FRSTOR       = 6'h3F;  // FPU Restore State (287 vs 387 via CR0.ET)

// Common microcode addresses returned by protection unit
// (0x000 = test passed, non-zero = exception handler or special routine)
localparam logic [11:0] PROT_CONTINUE           = 12'h000;  // Test passed, continue
localparam logic [11:0] PROT_GP_FAULT           = 12'h801;  // General Protection Fault (#GP)
localparam logic [11:0] PROT_STACK_FAULT        = 12'h863;  // Stack Fault (#SS)
localparam logic [11:0] PROT_SEGMENT_NOT_PRESENT= 12'h871;  // Segment Not Present (#NP)
localparam logic [11:0] PROT_GATE_HANDLER       = 12'h5B3;  // Gate handler (not a fault)

// Segment selection helpers (used by both z386.sv encoder and segmentation_unit)

// Determine default segment based on ModR/M and SIB addressing mode
function automatic [3:0] calc_default_seg_type(
    input [7:0] modrm_byte, input [7:0] sib_byte, input has_sib, input use_addr32);
    reg [1:0] mod;
    reg [2:0] rm;
    mod = modrm_byte[7:6];
    rm = modrm_byte[2:0];
    if (!use_addr32) begin
        if (rm == 3'b010 || rm == 3'b011 || (rm == 3'b110 && mod != 2'b00))
            calc_default_seg_type = SEG_SS;
        else
            calc_default_seg_type = SEG_DS;
    end else begin
        if (has_sib && rm == 3'b100) begin
            if (sib_byte[2:0] == 3'b100 || (sib_byte[2:0] == 3'b101 && mod != 2'b00))
                calc_default_seg_type = SEG_SS;
            else
                calc_default_seg_type = SEG_DS;
        end else if (rm == 3'b101 && mod != 2'b00)
            calc_default_seg_type = SEG_SS;
        else
            calc_default_seg_type = SEG_DS;
    end
endfunction

// Apply segment override prefix to default segment selection
function automatic [3:0] apply_seg_override_type(input [3:0] default_type, input [2:0] override);
    case (override)
        3'd1: apply_seg_override_type = SEG_CS;
        3'd2: apply_seg_override_type = SEG_SS;
        3'd3: apply_seg_override_type = SEG_DS;
        3'd4: apply_seg_override_type = SEG_ES;
        3'd5: apply_seg_override_type = SEG_FS;
        3'd6: apply_seg_override_type = SEG_GS;
        default: apply_seg_override_type = default_type;
    endcase
endfunction

// Resolve uc_dest field to segment target for command encoding
// Returns SEG_NONE for dest values that don't map to segments
function automatic [3:0] resolve_seg_target(input [6:0] dest, input [2:0] seg_sel, input [5:0] countr);
    case (dest)
        DEST_DES_ES, DEST_ES:   resolve_seg_target = SEG_ES;
        DEST_DES_CS, DEST_CS:   resolve_seg_target = SEG_CS;
        DEST_DES_SS, DEST_SS:   resolve_seg_target = SEG_SS;
        DEST_DES_DS, DEST_DS:   resolve_seg_target = SEG_DS;
        DEST_DES_FS, DEST_FS:   resolve_seg_target = SEG_FS;
        DEST_DES_GS, DEST_GS:   resolve_seg_target = SEG_GS;
        DEST_DES_TR, DEST_TR:   resolve_seg_target = SEG_TR;
        DEST_DESLDT, DEST_LDTR: resolve_seg_target = SEG_LDT;
        DEST_DESIDT:            resolve_seg_target = SEG_IDT;
        DEST_DESGDT:            resolve_seg_target = SEG_GDT;
        DEST_DESSDT:            resolve_seg_target = SEG_GDT;
        DEST_DESCOD:            resolve_seg_target = SEG_CS;
        DEST_DESSTK:            resolve_seg_target = SEG_SS;
        DEST_DES_IO:            resolve_seg_target = SEG_IO;
        DEST_SEGREG, DEST_DES_SR: resolve_seg_target = {1'b0, seg_sel};
        DEST_IRF: begin
            case (countr)
                6'h20: resolve_seg_target = SEG_ES;
                6'h22: resolve_seg_target = SEG_SS;
                6'h23: resolve_seg_target = SEG_DS;
                6'h24: resolve_seg_target = SEG_FS;
                6'h25: resolve_seg_target = SEG_GS;
                default: resolve_seg_target = SEG_NONE;
            endcase
        end
        default: resolve_seg_target = SEG_NONE;
    endcase
endfunction

endpackage
