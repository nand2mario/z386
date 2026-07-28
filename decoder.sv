// Z386 Instruction Decoder

module decoder
    import z386_pkg::*;
(
    input               clk,
    input               reset_n,

    // Prefetch queue interface (two-cursor protocol)
    input        [63:0] win_d1,         // registered raw window at the D1 cursor
    input        [5:0]  d1_avail,       // bytes fetched beyond the D1 cursor
    output       [3:0]  d1_adv,         // D1 cursor advance (0-11 bytes: a prefix,
                                        //   or the whole rest of the instruction
                                        //   at handoff - struct AND literal bytes)
    input        [31:0] win_lit,        // 4 bytes at pop_cursor + lit_off
    input        [5:0]  lit_avail,      // bytes fetched beyond that point
    output       [4:0]  lit_off,        // literal offset from the pop cursor
    output              pop_now,        // instruction completed D2: pop its bytes
    output       [4:0]  pop_len,        //   (registered length from the skeleton)

    // Mode signals
    input               D,              // Default operand/address size (CS.D bit)
    input               pe_enable,      // Protected mode enable (CR0.PE)

    // Control signals
    input               q_flush,        // Flush decoder on branch
    input               i_pop,          // D2 fires into EX

    // Decoded instruction output
    output dec_entry_t  i_bus,          // Decoded instruction
    output              decq_empty,     // Legacy name: unified D2 has no skeleton
    output dec_entry_t  i_bus2,         // Registered D1 skid successor
    output              decq_has2,      // skid successor can complete D2 in one cycle
    output              decq_has_jmp_call, // JMP/CALL rel in flight (halt speculative prefetch)

    // Unified D2 payload and empty-pipe D1 launch
    output dec_entry_t  d2_entry,       // instruction completing D2
    output              d2_push,        // d2_entry is complete this cycle
    output              d1_issue_direct,// handoff has no older D2 skeleton
    output       [11:0] d1_issue_entry_point,
    output dec_entry_t  d1_issue_entry  // live structural entry for empty-pipe launch
);

typedef enum logic [1:0] {
    LIT_NONE = 2'd0,
    LIT_IMM  = 2'd1,
    LIT_DISP = 2'd2
} lit_kind_t;

// The skeleton: a structurally-complete entry plus the literal plan.
// Literal fields (immediate/displacement) and the entry point are pending.
typedef struct packed {
    dec_entry_t entry;
    lit_kind_t  lit1_kind;
    lit_kind_t  lit2_kind;
    logic [2:0] lit1_size;
    logic [2:0] lit2_size;
    logic       lit1_sign_extend;
    logic       lit2_sign_extend;
    logic       lit1_mirror_disp;
    logic       lit2_mirror_disp;
    logic       need_sib;
    logic [2:0] pending_imm_size;
    logic       pending_imm_sign_extend;
} decoder_work_t;

`include "pla_control.svh"
`include "pla_entry.svh"

//=============================================================================
// D1 - structural decode
//=============================================================================

// Prefix state is not part of dec_entry_t until a non-prefix opcode is decoded.
logic        prefix_66;
logic        prefix_67;
logic        prefix_0f;
logic        prefix_rep;
logic [3:0]  prefix_count;
logic [1:0]  prefix_rep_lock;
logic [2:0]  prefix_seg;
reg         code32_r;     // Local CS.D copy; frontend flush hides its one-cycle update latency.

reg            d1_sib;      // SIB sub-cycle pending (cursor held)
decoder_work_t pend_work;   // struct_work parked across the SIB sub-cycle
decoder_work_t skid;        // one registered D1 successor ahead of D2
reg            skid_v;
reg [4:0]      skid_lit_off;
reg            skid_one_d2;

// Byte aliases at the D1 cursor.  The cursor does not advance until the
// skeleton handoff, so during the SIB sub-cycle the sib byte is byte 2.
wire [7:0] opcode = win_d1[7:0];
wire [7:0] modrm  = win_d1[15:8];
wire [7:0] sib_b  = win_d1[23:16];
wire       data32 = code32_r ^ prefix_66;
wire       addr32 = code32_r ^ prefix_67;

wire consume_prefix = !d1_sib && !prefix_0f && is_prefix(opcode) &&
                      (d1_avail >= 6'd1);
wire consume_0f     = !d1_sib && !prefix_0f && (opcode == 8'h0f) &&
                      (d1_avail >= 6'd1);

decoder_work_t struct_work;
logic [2:0]    struct_len;
always_comb build_struct_work(struct_work, struct_len);

wire struct_bytes_ok = !d1_sib && (d1_avail >= {3'b000, struct_len});
wire sib_bytes_ok    = d1_sib && (d1_avail >= 6'd3);   // opcode+modrm+sib in view

// skel_free: d2_done's terms are lit_avail / out_full / phase - all
// register-derived, never i_pop (L1) - so the same-edge free is legal.
wire d2_done;
wire d1_slot_ready = !skid_v || i_pop;

wire d1_to_sib  = !consume_prefix && !consume_0f &&
                  struct_bytes_ok && struct_work.need_sib;
wire d1_handoff = !consume_prefix && !consume_0f && d1_slot_ready &&
                  ((struct_bytes_ok && !struct_work.need_sib) || sib_bytes_ok);

decoder_work_t handoff_work;
always_comb begin
    handoff_work = d1_sib ? capture_sib(pend_work, sib_b) : struct_work;
end
decoder_work_t handoff_d2;
always_comb begin
    handoff_d2 = handoff_work;
    {handoff_d2.entry.ea_index_onehot, handoff_d2.entry.ea_base_onehot} =
        dec_ea_onehots(handoff_work.entry);
end
// First literal byte, as an offset from the POP cursor (prefix bytes are
// not popped until D2 completes): total length minus the literal bytes.
wire [4:0] handoff_lit_off = handoff_work.entry.length -
                             ({2'b00, handoff_work.lit1_size} +
                              {2'b00, handoff_work.lit2_size});
// D1 cursor advance at handoff: the whole REST of the instruction (struct
// bytes AND the literal bytes D2 will capture), so the cursor lands on the
// next instruction's first byte.  It may pass not-yet-fetched literal
// bytes; d1_avail clamps to 0 in that case and D1 waits for the fill.
wire [4:0] handoff_adv = handoff_work.entry.length -
                         {1'b0, prefix_count} - (prefix_0f ? 5'd1 : 5'd0);
// Prefix and 0F bytes have already advanced the D1 cursor. Relative to the
// registered raw D1 window, the first literal follows only opcode/ModR/M/SIB.
wire [4:0] handoff_raw_lit_off_w = handoff_lit_off -
                                    {1'b0, prefix_count} -
                                    (prefix_0f ? 5'd1 : 5'd0);
wire [1:0] handoff_raw_lit_off = handoff_raw_lit_off_w[1:0];

// The entry PLA result is valid during D1 handoff, one cycle before the
// registered skeleton completes D2. It directly launches an empty D2 pipe.
wire [3:0] handoff_lit_size = {1'b0, handoff_work.lit1_size} +
                              {1'b0, handoff_work.lit2_size};
wire handoff_one_d2 = (handoff_work.lit2_kind == LIT_NONE ||
                       handoff_lit_size <= 4'd4) &&
                      (d1_avail >= {1'b0, handoff_adv});
wire [3:0] handoff_raw_need = (handoff_work.lit2_kind == LIT_NONE ||
                               handoff_lit_size <= 4'd4)
                            ? handoff_lit_size
                            : {1'b0, handoff_work.lit1_size};
wire [5:0] handoff_raw_end = {1'b0, handoff_raw_lit_off_w} +
                             {2'b00, handoff_raw_need};
wire handoff_raw_valid = (d1_avail >= handoff_raw_end);
// Launch the synchronous ROM as soon as structural decode owns an empty D2.
// D2 holds the returned word while late or second literals are captured.
assign d1_issue_direct = d1_handoff && !skel_v;
assign d1_issue_entry_point = handoff_work.entry.entry_point;
always_comb begin
    d1_issue_entry = handoff_d2.entry;
end

// TIMING: no !q_flush in the advance/pop legs. q_flush carries the deep uc_exec/mem-block cone
assign d1_adv = (consume_prefix || consume_0f) ? 4'd1 :
                d1_handoff ? handoff_adv[3:0] :
                4'd0;

//=============================================================================
// The skeleton register (D1 -> D2 pipeline boundary; Step 2's lookahead)
//=============================================================================

decoder_work_t skel;
reg            skel_v;
reg [4:0]      skel_lit_off;
reg [31:0]     skel_raw_hi_r;    // upper half of the opcode-relative D1 window
reg [1:0]      skel_raw_lit_off_r;
reg [31:0]     skel_lit_r;       // shared low raw half or pop-relative literal
reg            skel_window_valid_r;
reg            skel_window_raw_r; // skel_lit_r is the low half of the D1 window
wire           head_v = skel_v;  // temporary trace compatibility alias

// Register the raw literal window on the edge that installs an instruction in
// D2. On replacement, the old pop cursor is still active, so add the retiring
// instruction length to reach the successor. Literal interpretation remains a
// D2 operation and therefore does not lengthen the D1 structural path.
wire promote_skid_capture = i_pop && skid_v;
wire promote_handoff_capture = d1_handoff &&
                                ((!skel_v && !i_pop) || (i_pop && !skid_v));
wire incoming_capture = promote_skid_capture || promote_handoff_capture;
// Only skid promotion reads a successor through the pop-relative literal
// port. A direct D1 handoff freezes its opcode-relative win_d1 bytes instead;
// do not feed live structural length decode back through the queue aligner.
wire [5:0] skid_lit_abs = {1'b0, skel.entry.length} +
                          {1'b0, skid_lit_off};
wire [2:0] incoming_lit1_size = promote_skid_capture ? skid.lit1_size :
                                                        handoff_d2.lit1_size;
wire [2:0] incoming_lit2_size = promote_skid_capture ? skid.lit2_size :
                                                        handoff_d2.lit2_size;
wire [1:0] incoming_lit2_kind = promote_skid_capture ? skid.lit2_kind :
                                                        handoff_d2.lit2_kind;
wire [3:0] incoming_lit_total = {1'b0, incoming_lit1_size} +
                                {1'b0, incoming_lit2_size};
wire incoming_fields_fit = (incoming_lit2_kind == LIT_NONE) ||
                           (incoming_lit_total <= 4'd4);
wire [3:0] incoming_lit_size = incoming_fields_fit ? incoming_lit_total :
                                                     {1'b0, incoming_lit1_size};
wire incoming_bytes_ok = lit_avail >= {2'b00, incoming_lit_size};

always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        code32_r <= 1'b0;
        skel_v <= 1'b0;
        skid_v <= 1'b0;
        d1_sib <= 1'b0;
        prefix_66 <= 1'b0;
        prefix_67 <= 1'b0;
        prefix_0f <= 1'b0;
        prefix_rep <= 1'b0;
        prefix_count <= 4'd0;
        prefix_rep_lock <= PREFIX_NOREPLOCK;
        prefix_seg <= PREFIX_NOSEG;
    end else if (q_flush) begin
        code32_r <= D;
        skel_v <= 1'b0;
        skid_v <= 1'b0;
        d1_sib <= 1'b0;
        prefix_66 <= 1'b0;
        prefix_67 <= 1'b0;
        prefix_0f <= 1'b0;
        prefix_rep <= 1'b0;
        prefix_count <= 4'd0;
        prefix_rep_lock <= PREFIX_NOREPLOCK;
        prefix_seg <= PREFIX_NOSEG;
    end else begin
        code32_r <= D;
        if (consume_prefix || consume_0f) begin
            if (consume_0f) begin
                prefix_0f <= 1'b1;
            end else begin
                prefix_count <= prefix_count + 4'd1;
                unique case (opcode)
                    8'h66: prefix_66 <= 1'b1;
                    8'h67: prefix_67 <= 1'b1;
                    8'hf0: prefix_rep_lock <= PREFIX_LOCK;
                    8'hf2: begin
                        prefix_rep_lock <= PREFIX_REPNE;
                        prefix_rep <= 1'b1;
                    end
                    8'hf3: begin
                        prefix_rep_lock <= PREFIX_REP;
                        prefix_rep <= 1'b1;
                    end
                    8'h26, 8'h2e, 8'h36, 8'h3e, 8'h64, 8'h65:
                        prefix_seg <= prefix_seg_code(opcode);
                    default: ;
                endcase
            end
        end

        if (d1_to_sib) begin
            pend_work <= struct_work;
            d1_sib <= 1'b1;
        end

        if (d1_handoff) begin
            d1_sib <= 1'b0;
            // The skeleton carries the prefix state; clear for the next one.
            prefix_66 <= 1'b0;
            prefix_67 <= 1'b0;
            prefix_0f <= 1'b0;
            prefix_rep <= 1'b0;
            prefix_count <= 4'd0;
            prefix_rep_lock <= PREFIX_NOREPLOCK;
            prefix_seg <= PREFIX_NOSEG;
        end

        // The skid holds one structurally decoded successor while D2 owns the
        // current instruction. On d2_fire it promotes from registered state,
        // keeping the full D1 decoder out of the ROM-address path.
        if (i_pop) begin
            if (skid_v) begin
                skel <= skid;
                skel_v <= 1'b1;
                skel_lit_off <= skid_lit_off;
                if (d1_handoff) begin
                    skid <= handoff_d2;
                    skid_v <= 1'b1;
                    skid_lit_off <= handoff_lit_off;
                    skid_one_d2 <= handoff_one_d2;
                end else begin
                    skid_v <= 1'b0;
                end
            end else if (d1_handoff) begin
                skel <= handoff_d2;
                skel_v <= 1'b1;
                skel_lit_off <= handoff_lit_off;
            end else begin
                skel_v <= 1'b0;
            end
        end else if (d1_handoff) begin
            if (!skel_v) begin
                skel <= handoff_d2;
                skel_v <= 1'b1;
                skel_lit_off <= handoff_lit_off;
            end else begin
                skid <= handoff_d2;
                skid_v <= 1'b1;
                skid_lit_off <= handoff_lit_off;
                skid_one_d2 <= handoff_one_d2;
            end
        end
    end
end

//=============================================================================
// D2 - literal capture + entry resolution
//=============================================================================

// Phase A = fields as handed off; phase B = field A captured (workB holds
// the merged entry), literal window advanced by lit1_size.
reg            d2_phaseB;
decoder_work_t workB;

wire [2:0] sizeA    = skel.lit1_size;
wire [2:0] sizeB    = skel.lit2_size;
wire       has_litA = (skel.lit1_kind != LIT_NONE);
wire       has_litB = (skel.lit2_kind != LIT_NONE);
wire [3:0] sizeAB   = {1'b0, sizeA} + {1'b0, sizeB};
wire       ab_fit   = (sizeAB <= 4'd4);   // both fields inside one window

// Once field A is in the registered window, the live literal port is free to
// preread field B while D2 interprets A.
wire d2_window_valid = skel_window_valid_r;
wire prefetch_fieldB = skel_v && d2_window_valid && !d2_phaseB &&
                       has_litB && !ab_fit;
wire [4:0] resident_lit_off = skel_lit_off +
                              ((d2_phaseB || prefetch_fieldB)
                                  ? {2'b00, sizeA} : 5'd0);
assign lit_off = promote_skid_capture ? skid_lit_abs[4:0] : resident_lit_off;

// D1 carries raw bytes, not parsed literals. D2 selects the literal start from
// the registered window; the second field uses the live literal port only when
// both fields do not fit in this 32-bit view.
decoder_work_t capA;
decoder_work_t capAB;
decoder_work_t capB;
wire [31:0] skel_raw_lit_win =
    skel_raw_lit_off_r == 2'd0 ? skel_lit_r :
    skel_raw_lit_off_r == 2'd1 ? {skel_raw_hi_r[7:0],  skel_lit_r[31:8]}  :
    skel_raw_lit_off_r == 2'd2 ? {skel_raw_hi_r[15:0], skel_lit_r[31:16]} :
                                 {skel_raw_hi_r[23:0], skel_lit_r[31:24]};
wire [31:0] skel_lit_win = skel_window_raw_r ? skel_raw_lit_win : skel_lit_r;
wire [31:0] winB = (sizeA == 3'd1) ? { 8'h00, skel_lit_win[31:8]}  :
                   (sizeA == 3'd2) ? {16'h0,  skel_lit_win[31:16]} :
                   (sizeA == 3'd3) ? {24'h0,  skel_lit_win[31:24]} : 32'h0;
always_comb begin
    capA  = capture_literal(skel, skel_lit_win,
                            skel.lit1_kind, skel.lit1_size,
                            skel.lit1_sign_extend, skel.lit1_mirror_disp);
    capAB = capture_literal(capA, winB, capA.lit2_kind, capA.lit2_size,
                            capA.lit2_sign_extend, capA.lit2_mirror_disp);
    capB  = capture_literal(workB, skel_lit_win,
                            workB.lit2_kind, workB.lit2_size,
                            workB.lit2_sign_extend, workB.lit2_mirror_disp);
end

wire       finishing = d2_phaseB || !has_litB || ab_fit;
wire [3:0] need_now  = d2_phaseB ? {1'b0, sizeB} :
                       !has_litA ? 4'd0 :
                       (has_litB && ab_fit) ? sizeAB : {1'b0, sizeA};
wire       bytes_ok  = (lit_avail >= {2'b00, need_now});
wire       d2_can    = skel_v && d2_window_valid;
wire       d2_complete = d2_can && finishing;
assign     d2_done   = i_pop;
// Field A may capture while the output side is full (no push happens here).
wire       d2_stepA  = d2_can && !finishing;
wire       d2_late_capture = skel_v && !d2_window_valid && bytes_ok;

decoder_work_t d2_final;
always_comb begin
    d2_final = d2_phaseB ? capB :
               !has_litA ? skel :
               (has_litB && ab_fit) ? capAB : capA;
end

always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        d2_phaseB <= 1'b0;
    end else if (q_flush) begin
        d2_phaseB <= 1'b0;
    end else if (incoming_capture) begin
        d2_phaseB <= 1'b0;
    end else if (d2_done) begin
        d2_phaseB <= 1'b0;
    end else if (d2_stepA) begin
        workB <= capA;
        d2_phaseB <= 1'b1;
    end
end

always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        skel_raw_hi_r <= 32'h0;
        skel_raw_lit_off_r <= 2'd0;
        skel_lit_r <= 32'h0;
        skel_window_valid_r <= 1'b0;
        skel_window_raw_r <= 1'b0;
    end else if (q_flush) begin
        skel_window_valid_r <= 1'b0;
    end else if (i_pop && skid_v) begin
        // A skid promotion has a full registered D2 cycle before use. Capture
        // its pop-relative literal view now instead of duplicating 64 raw bits
        // in the D1 skid entry.
        skel_lit_r <= win_lit;
        skel_window_valid_r <= incoming_bytes_ok;
        skel_window_raw_r <= 1'b0;
    end else if (promote_handoff_capture) begin
        skel_lit_r <= win_d1[31:0];
        skel_raw_hi_r <= win_d1[63:32];
        skel_raw_lit_off_r <= handoff_raw_lit_off;
        skel_window_valid_r <= handoff_raw_valid;
        skel_window_raw_r <= 1'b1;
    end else if (d2_stepA) begin
        skel_lit_r <= win_lit;
        skel_window_valid_r <= (lit_avail >= {3'b000, sizeB});
        skel_window_raw_r <= 1'b0;
    end else if (d2_late_capture) begin
        skel_lit_r <= win_lit;
        skel_window_valid_r <= 1'b1;
        skel_window_raw_r <= 1'b0;
    end else if (d2_done) begin
        skel_window_valid_r <= 1'b0;
    end
end

// Entry-point resolution moved BACK to D1
dec_entry_t push_entry;
always_comb begin
    push_entry = d2_final.entry;
    {push_entry.ea_index_onehot, push_entry.ea_base_onehot} = dec_ea_onehots(push_entry);
end

assign d2_entry = push_entry;
assign d2_push  = d2_complete;
assign i_bus = push_entry;
assign decq_empty = !skel_v;
assign i_bus2 = skid.entry;
assign decq_has2 = skel_v && skid_v && skid_one_d2;

assign pop_now = d2_done;
assign pop_len = skel.entry.length;

// synthesis translate_off
// V52 A2 equivalence oracle. Recipe metadata and live FAST control come from
// the optimized microcode inventory; the legacy opcode classifier remains in
// simulation to prove every decoded instruction produces identical control.
wire [2:0] push_recipe_early = recipe_early_kind(push_entry.entry_point);
fast_class_t push_legacy_fc, push_recipe_fc;
assign push_legacy_fc = dec_fast_class(push_entry);
assign push_recipe_fc = recipe_fast_class(push_entry);
reg recipe_cov_en = 1'b0;
initial recipe_cov_en = $test$plusargs("recipe_cov");
always @(posedge clk) begin
    if (reset_n && d2_done) begin
        if (push_recipe_fc !== push_legacy_fc)
            $fatal(1, "FAST recipe classifier mismatch: entry=%03x opcode=%02x modrm=%02x old=%04x new=%04x",
                   push_entry.entry_point, push_entry.opcode, push_entry.modrm,
                   push_legacy_fc, push_recipe_fc);
    end
    if (reset_n && d2_done && push_legacy_fc.fast) begin
        if (push_recipe_early == RECIPE_EARLY_SEQ)
            $fatal(1, "FAST recipe missing: entry=%03x opcode=%02x modrm=%02x 0f=%b",
                   push_entry.entry_point, push_entry.opcode, push_entry.modrm,
                   push_entry.has_0f);
        unique case (push_recipe_early)
            RECIPE_EARLY_NONE:
                if (push_legacy_fc.uses_ea || push_legacy_fc.br_rel)
                    $fatal(1, "FAST recipe NONE role mismatch: entry=%03x fc=%04x",
                           push_entry.entry_point, push_legacy_fc);
            RECIPE_EARLY_EA, RECIPE_EARLY_LOAD, RECIPE_EARLY_STORE,
            RECIPE_EARLY_RMW, RECIPE_EARLY_STACK:
                if (!push_legacy_fc.uses_ea)
                    $fatal(1, "FAST recipe address role mismatch: entry=%03x kind=%0d fc=%04x",
                           push_entry.entry_point, push_recipe_early, push_legacy_fc);
            RECIPE_EARLY_BRANCH:
                if (!push_legacy_fc.br_rel)
                    $fatal(1, "FAST recipe branch role mismatch: entry=%03x fc=%04x",
                           push_entry.entry_point, push_legacy_fc);
            default:
                $fatal(1, "FAST recipe invalid early kind: entry=%03x kind=%0d",
                       push_entry.entry_point, push_recipe_early);
        endcase
        if (recipe_cov_en)
            $display("RECIPE_COV entry=%03x kind=%0d opcode=%02x modrm=%02x fc=%04x",
                     push_entry.entry_point, push_recipe_early,
                     push_entry.opcode, push_entry.modrm, push_legacy_fc);
    end
end

reg D1_EVT = 1'b0;
initial if ($test$plusargs("pf_cur")) D1_EVT = 1'b1;
always @(posedge clk) begin
    if (D1_EVT && d1_handoff)
        $display("%0t D1HO op=%02x modrm=%02x len=%0d adv=%0d(sib=%b slen=%0d) litoff=%0d A=%0d/%0d B=%0d/%0d pcnt=%0d 0f=%b",
                 $time, handoff_work.entry.opcode, handoff_work.entry.modrm,
                 handoff_work.entry.length, d1_adv, d1_sib, struct_len,
                 handoff_lit_off, handoff_work.lit1_kind, handoff_work.lit1_size,
                 handoff_work.lit2_kind, handoff_work.lit2_size,
                 prefix_count, prefix_0f);
    if (D1_EVT && d2_done)
        $display("%0t D2DN op=%02x len=%0d imm=%08x disp=%08x phB=%b",
                 $time, push_entry.opcode, push_entry.length,
                 push_entry.immediate, push_entry.displacement, d2_phaseB);
end
always @(posedge clk) begin
    if (reset_n && d1_handoff && (handoff_raw_lit_off_w[4:2] != 3'b000))
        $fatal(1, "D1: raw literal offset exceeds opcode window: off=%0d op=%02x",
               handoff_raw_lit_off_w, handoff_work.entry.opcode);
    // The literal plan must tile the instruction exactly: pop_len bytes =
    // struct bytes (lit_off) + literal bytes.
    if (reset_n && skel_v &&
        ({1'b0, skel.entry.length} !==
         {1'b0, skel_lit_off} + {3'b000, skel.lit1_size} + {3'b000, skel.lit2_size}))
        $fatal(1, "D2: literal plan does not tile: len=%0d lit_off=%0d A=%0d B=%0d",
               skel.entry.length, skel_lit_off, skel.lit1_size, skel.lit2_size);
    if (reset_n && d2_phaseB && !skel_v)
        $fatal(1, "D2: phase B with no skeleton");
end
// synthesis translate_on

// JMP/CALL rel is unconditionally taken, so its speculative fall-through prefetch is always wasted.
function automatic logic entry_jmp_call(input dec_entry_t e);
    entry_jmp_call = !e.has_0f && (e.opcode == 8'hEB || e.opcode == 8'hE9 ||
                                   e.opcode == 8'hE8);
endfunction

assign decq_has_jmp_call =
    (d2_push && entry_jmp_call(push_entry)) ||
    (skid_v && skid_one_d2 && entry_jmp_call(skid.entry)) ||
    (d1_handoff && handoff_one_d2 && entry_jmp_call(d1_issue_entry));

//=============================================================================
// Structural Decode
//=============================================================================

task automatic build_struct_work(
    output decoder_work_t w,
    output logic [2:0]    s_len
);
    logic [11:0] ctl_bits;
    logic [15:0] entry_first;
    logic [15:0] entry_final;
    logic        entry_group;
    logic [5:0]  group_code;
    logic        has_modrm;
    logic        has_sib;
    logic [2:0]  disp_size;
    logic [2:0]  imm_total_size;
    logic [2:0]  imm_first_size;
    logic        imm_sign_extend;
    logic        invalid_lock;
    begin
        w = '0;
        s_len = 3'd1;
        ctl_bits = pla_control_opcode_lookup(prefix_0f, opcode);

        w.entry.opcode = opcode;
        w.entry.has_0f = prefix_0f;
        w.entry.has_rep = prefix_rep;
        w.entry.prefix_count = prefix_count;
        w.entry.data32 = data32;
        w.entry.addr32 = addr32;
        w.entry.rep_lock = prefix_rep_lock;
        w.entry.seg = prefix_seg;
        w.entry.has_d_bit = ctl_bits[11] & ~ctl_bits[10] & ctl_bits[9] &
                             ctl_bits[8] & ctl_bits[2] & ctl_bits[1];
        w.entry.has_embedded_register = ~ctl_bits[2];
        w.entry.has_w_bit = ctl_bits[1];
        w.entry.is_pushpop_seg = ctl_bits[3];
        w.entry.update_flags = ctl_bits[4];

        has_modrm = ctl_bits[2] & ~ctl_bits[0];
        imm_sign_extend = 1'b0;
        imm_total_size = 3'd0;
        imm_first_size = 3'd0;

        // A PE/D change flushes the frontend, so a skeleton is never consumed
        // across a mode switch.
        entry_first = pla_entry_lookup({data32, opcode, prefix_rep, pe_enable,
                                        1'b1, prefix_0f});
        entry_group = ~(|entry_first[11:6]) && has_modrm;
        // Parallel copy of entry_first[5:0] for group rows, so the
        // second-level lookup does not chain behind the first.
        group_code = pla_group_lookup({data32, opcode, pe_enable, prefix_0f});
        entry_final = entry_group ?
            pla_group_entry_lookup({data32, group_code[5:4], modrm[5:3],
                                    group_code[3:0], (modrm[7:6] != 2'b11),
                                    1'b0, prefix_0f}) :
            entry_first;
        invalid_lock = check_lock_invalid(prefix_rep_lock, prefix_0f, opcode,
                                          has_modrm, modrm);
        w.entry.entry_point = invalid_lock ? 12'h82B : entry_final[11:0];
        w.entry.stack_op = invalid_lock ? 1'b0 : entry_final[13];
        w.entry.stack_dir = invalid_lock ? 1'b0 : entry_final[12];

        if (has_modrm) begin
            has_sib = addr32 && (modrm[7:6] != 2'b11) && (modrm[2:0] == 3'b100);
            disp_size = has_sib ? 3'd0 : modrm_disp_size(addr32, modrm, 8'h00, 1'b0);
            s_len = 3'd2;

            unique case (ctl_bits[11:10])
                2'b00: imm_total_size = 3'd1;
                2'b01: imm_total_size = data32 ? 3'd4 : 3'd2;
                2'b10: imm_total_size = 3'd0;
                default: begin
                    imm_total_size = 3'd1;
                    imm_sign_extend = 1'b1;
                end
            endcase

            if ((opcode[7:1] == 7'b1111011) && (modrm[5:3] >= 3'd2))
                imm_total_size = 3'd0;

            w.entry.has_modrm = 1'b1;
            w.entry.modrm = modrm;
            w.entry.has_sib = has_sib;
            w.entry.sib = 8'h00;
            w.entry.imm_size = imm_total_size;
            w.need_sib = has_sib;
            w.pending_imm_size = has_sib ? imm_total_size : 3'd0;
            w.pending_imm_sign_extend = has_sib ? imm_sign_extend : 1'b0;
            select_register_fields(prefix_0f, opcode, 1'b1, modrm,
                                   w.entry.src_reg_sel, w.entry.dst_reg_sel);

            if (!has_sib) begin
                if (disp_size != 3'd0) begin
                    w.lit1_kind = LIT_DISP;
                    w.lit1_size = disp_size;
                    // Normalize disp8 while D2 captures the literal so the
                    // live EA path consumes one ready 32-bit addend.
                    w.lit1_sign_extend = (disp_size == 3'd1);
                end
                if (imm_total_size != 3'd0) begin
                    if (w.lit1_kind == LIT_NONE) begin
                        w.lit1_kind = LIT_IMM;
                        w.lit1_size = imm_total_size;
                        w.lit1_sign_extend = imm_sign_extend;
                    end else begin
                        w.lit2_kind = LIT_IMM;
                        w.lit2_size = imm_total_size;
                        w.lit2_sign_extend = imm_sign_extend;
                    end
                end
            end
        end else begin
            has_sib = 1'b0;
            disp_size = 3'd0;
            select_register_fields(prefix_0f, opcode, 1'b0, 8'h00,
                                   w.entry.src_reg_sel, w.entry.dst_reg_sel);

            if (!prefix_0f && opcode[7:2] == 6'b101000) begin
                // MOV AL/eAX,moffs and MOV moffs,AL/eAX.
                w.entry.has_moffs = 1'b1;
                imm_total_size = addr32 ? 3'd4 : 3'd2;
                imm_first_size = imm_total_size;
            end else begin
                unique case (ctl_bits[11:6])
                    6'b100111: imm_total_size = 3'd1;
                    6'b110111: imm_total_size = data32 ? 3'd4 : 3'd2;
                    6'b000111: begin
                        imm_total_size = 3'd1;
                        imm_sign_extend = 1'b1;
                    end
                    6'b010111: imm_total_size = 3'd2;
                    6'b011111: imm_total_size = data32 ? 3'd4 : 3'd2;
                    6'b100010: begin
                        imm_total_size = 3'd1;
                        imm_sign_extend = 1'b1;
                    end
                    6'b101111: imm_total_size = 3'd3;
                    6'b111111: imm_total_size = data32 ? 3'd6 : 3'd4;
                    default:   imm_total_size = 3'd0;
                endcase

                imm_first_size = imm_total_size;
                unique case (ctl_bits[11:6])
                    6'b100010: imm_first_size = 3'd0;
                    6'b101111: imm_first_size = 3'd2;
                    6'b111111: imm_first_size = data32 ? 3'd4 : 3'd2;
                    default: ;
                endcase
            end

            w.entry.imm_size = imm_total_size;
            if (imm_total_size != 3'd0) begin
                if (imm_first_size != 3'd0) begin
                    w.lit1_kind = LIT_IMM;
                    w.lit1_size = imm_first_size;
                    w.lit1_sign_extend = imm_sign_extend && (imm_first_size == 3'd1);
                    w.lit1_mirror_disp = 1'b1;
                end
                if (imm_total_size != imm_first_size) begin
                    if (w.lit1_kind == LIT_NONE) begin
                        w.lit1_kind = LIT_DISP;
                        w.lit1_size = imm_total_size - imm_first_size;
                        w.lit1_sign_extend = imm_sign_extend;
                    end else begin
                        w.lit2_kind = LIT_DISP;
                        w.lit2_size = imm_total_size - imm_first_size;
                        w.lit2_sign_extend = imm_sign_extend;
                    end
                end
            end
        end

        w.entry.length = {1'b0, prefix_count} + (prefix_0f ? 5'd1 : 5'd0) +
                         {2'b00, s_len} + {2'b00, disp_size} +
                         {2'b00, imm_total_size};
    end
endtask

function automatic decoder_work_t capture_sib(input decoder_work_t in,
                                              input logic [7:0]   sib_byte);
    decoder_work_t out;
    logic [2:0] disp_size;
    begin
        out = in;
        disp_size = modrm_disp_size(in.entry.addr32, in.entry.modrm,
                                    sib_byte, in.entry.has_sib);

        out.need_sib = 1'b0;
        out.entry.sib = sib_byte;
        out.entry.length = in.entry.length + 5'd1 + {2'b00, disp_size};

        if (disp_size != 3'd0) begin
            out.lit1_kind = LIT_DISP;
            out.lit1_size = disp_size;
            out.lit1_sign_extend = (disp_size == 3'd1);
        end

        if (in.pending_imm_size != 3'd0) begin
            if (out.lit1_kind == LIT_NONE) begin
                out.lit1_kind = LIT_IMM;
                out.lit1_size = in.pending_imm_size;
                out.lit1_sign_extend = in.pending_imm_sign_extend;
            end else begin
                out.lit2_kind = LIT_IMM;
                out.lit2_size = in.pending_imm_size;
                out.lit2_sign_extend = in.pending_imm_sign_extend;
            end
        end

        out.pending_imm_size = 3'd0;
        out.pending_imm_sign_extend = 1'b0;
        capture_sib = out;
    end
endfunction

function automatic decoder_work_t capture_literal(
    input decoder_work_t in,
    input logic [31:0]   win,    // literal bytes at offset 0 of this view
    input lit_kind_t     kind,
    input logic [2:0]    size,
    input logic          sign_extend,
    input logic          mirror_disp
);
    decoder_work_t out;
    logic [31:0] value;
    begin
        out = in;
        value = literal_value(win, size, sign_extend);
        if (kind == LIT_IMM) begin
            out.entry.immediate = value;
            if (mirror_disp)
                out.entry.displacement = value;
        end else if (kind == LIT_DISP) begin
            if (in.lit1_kind == LIT_NONE) begin
                unique case (size)
                    3'd1: out.entry.displacement = {out.entry.displacement[31:8], value[7:0]};
                    3'd2: out.entry.displacement = {out.entry.displacement[31:16], value[15:0]};
                    3'd3: out.entry.displacement = {out.entry.displacement[31:24], value[23:0]};
                    default: out.entry.displacement = value;
                endcase
            end else begin
                out.entry.displacement = value;
            end
        end

        if (kind == out.lit1_kind) begin
            out.lit1_kind = LIT_NONE;
            out.lit1_size = 3'd0;
            out.lit1_sign_extend = 1'b0;
            out.lit1_mirror_disp = 1'b0;
        end else begin
            out.lit2_kind = LIT_NONE;
            out.lit2_size = 3'd0;
            out.lit2_sign_extend = 1'b0;
            out.lit2_mirror_disp = 1'b0;
        end
        capture_literal = out;
    end
endfunction

//=============================================================================
// Helpers
//=============================================================================

function automatic logic is_prefix(input logic [7:0] b);
    unique case (b)
        8'h26, 8'h2e, 8'h36, 8'h3e,
        8'h64, 8'h65, 8'h66, 8'h67,
        8'hf0, 8'hf2, 8'hf3: is_prefix = 1'b1;
        default: is_prefix = 1'b0;
    endcase
endfunction

function automatic logic [2:0] prefix_seg_code(input logic [7:0] b);
    unique case (b)
        8'h2e: prefix_seg_code = PREFIX_CS;
        8'h36: prefix_seg_code = PREFIX_SS;
        8'h3e: prefix_seg_code = PREFIX_DS;
        8'h26: prefix_seg_code = PREFIX_ES;
        8'h64: prefix_seg_code = PREFIX_FS;
        8'h65: prefix_seg_code = PREFIX_GS;
        default: prefix_seg_code = PREFIX_NOSEG;
    endcase
endfunction

function automatic logic [31:0] literal_value(
    input logic [31:0] bytes,
    input logic [2:0]  size,
    input logic        sign_extend
);
    begin
        unique case (size)
            3'd1: literal_value = sign_extend ? {{24{bytes[7]}}, bytes[7:0]} :
                                                 {24'h0, bytes[7:0]};
            3'd2: literal_value = {16'h0, bytes[15:0]};
            3'd3: literal_value = {8'h0, bytes[23:0]};
            3'd4: literal_value = bytes;
            default: literal_value = 32'h0;
        endcase
    end
endfunction

function automatic logic [2:0] modrm_disp_size(
    input logic       addr32_in,
    input logic [7:0] modrm_in,
    input logic [7:0] sib_in,
    input logic       has_sib_in
);
    if (modrm_in[7:6] == 2'b11)
        modrm_disp_size = 3'd0;
    else if (modrm_in[7:6] == 2'b01)
        modrm_disp_size = 3'd1;
    else if (modrm_in[7:6] == 2'b10)
        modrm_disp_size = addr32_in ? 3'd4 : 3'd2;
    else if (addr32_in && has_sib_in && sib_in[2:0] == 3'b101)
        modrm_disp_size = 3'd4;
    else if (addr32_in && !has_sib_in && modrm_in[2:0] == 3'b101)
        modrm_disp_size = 3'd4;
    else if (!addr32_in && modrm_in[2:0] == 3'b110)
        modrm_disp_size = 3'd2;
    else
        modrm_disp_size = 3'd0;
endfunction

task automatic select_register_fields(
    input  logic        has_0f_in,
    input  logic [7:0]  opcode_in,
    input  logic        has_modrm_in,
    input  logic [7:0]  modrm_in,
    output logic [2:0]  src_reg_sel_out,
    output logic [2:0]  dst_reg_sel_out
);
    begin
        src_reg_sel_out = has_modrm_in ? modrm_in[5:3] : opcode_in[2:0];
        dst_reg_sel_out = has_modrm_in ? (opcode_in[1] ? modrm_in[5:3] :
                                                        modrm_in[2:0]) :
                                         opcode_in[2:0];

        if (has_0f_in && has_modrm_in) begin
            src_reg_sel_out = modrm_in[5:3];
            dst_reg_sel_out = modrm_in[2:0];
            if (opcode_in == 8'h22 || opcode_in == 8'h23 || opcode_in == 8'h26)
                src_reg_sel_out = modrm_in[2:0];
        end else if (!has_0f_in) begin
            unique casez (opcode_in)
                8'b00???10?: begin
                    src_reg_sel_out = 3'd0;
                    dst_reg_sel_out = 3'd0;
                end
                8'b00???0??: begin
                    src_reg_sel_out = opcode_in[1] ? modrm_in[2:0] : modrm_in[5:3];
                    dst_reg_sel_out = opcode_in[1] ? modrm_in[5:3] : modrm_in[2:0];
                end
                8'h8a, 8'h8b: begin
                    src_reg_sel_out = modrm_in[2:0];
                    dst_reg_sel_out = modrm_in[5:3];
                end
                8'h8c, 8'h8e: begin
                    src_reg_sel_out = modrm_in[5:3];
                    dst_reg_sel_out = modrm_in[2:0];
                end
                8'h62: begin
                    src_reg_sel_out = modrm_in[5:3];
                    dst_reg_sel_out = modrm_in[2:0];
                end
                8'h8f, 8'hfe, 8'hff: dst_reg_sel_out = modrm_in[2:0];
                8'h63: begin
                    src_reg_sel_out = modrm_in[5:3];
                    dst_reg_sel_out = modrm_in[2:0];
                end
                8'h69, 8'h6b: begin
                    src_reg_sel_out = modrm_in[5:3];
                    dst_reg_sel_out = modrm_in[2:0];
                end
                8'b100000??: dst_reg_sel_out = modrm_in[2:0];
                8'hc0, 8'hc1, 8'hd0, 8'hd1,
                8'hd2, 8'hd3: dst_reg_sel_out = modrm_in[2:0];
                8'h86, 8'h87: begin
                    src_reg_sel_out = modrm_in[5:3];
                    dst_reg_sel_out = (modrm_in[7:6] == 2'b11) ? modrm_in[2:0] :
                                                                    modrm_in[5:3];
                end
                8'hd8, 8'hd9, 8'hda, 8'hdb,
                8'hdc, 8'hdd, 8'hde, 8'hdf: begin
                    src_reg_sel_out = modrm_in[5:3];
                    dst_reg_sel_out = modrm_in[2:0];
                end
                8'b10010???: begin
                    src_reg_sel_out = opcode_in[2:0];
                    dst_reg_sel_out = 3'd0;
                end
                8'b101000??, 8'ha8, 8'ha9: begin
                    src_reg_sel_out = 3'd0;
                    dst_reg_sel_out = 3'd0;
                end
                8'hc6, 8'hc7, 8'hf6, 8'hf7: dst_reg_sel_out = modrm_in[2:0];
                default: ;
            endcase
        end
    end
endtask

endmodule
