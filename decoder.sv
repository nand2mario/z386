//=============================================================================
// Z386 Instruction Decoder Module
//=============================================================================
// Structural decoder for the z386 0.2 frontend.
//
// The public interface is intentionally unchanged: the decoder still pushes a
// completed dec_entry_t into the existing decoded-instruction queue.  Internally
// it follows the new frontend model: consume prefixes/0F in DEC_STRUCT,
// decode opcode-only and opcode+ModR/M forms in one structural cycle, use a
// second structural cycle only for the rare opcode+ModR/M+SIB case, then
// capture one or two literal fields.
//
module decoder
    import z386_pkg::*;
(
    input               clk,
    input               reset_n,

    // Prefetch queue interface
    input        [31:0] q_window,       // 4-byte window at current queue head
    input        [5:0]  pf_count,       // Buffered prefetch bytes available
    input               pf_empty,       // Prefetch queue empty
    output       [2:0]  q_pop_bytes,    // Pop 1/2/3/4 bytes from prefetch queue

    // Mode signals
    input               D,              // Default operand/address size (CS.D bit)
    input               mode_32,        // 32-bit mode; kept for interface compatibility
    input               pe_enable,      // Protected mode enable (CR0.PE)

    // Control signals
    input               q_flush,        // Flush decoder on branch
    input               i_pop,          // Pop decoded instruction from queue
    input               halted,         // CPU is halted; decoder may still fill queue
    input               stall,          // Core stall; decoder may still fill queue

    // Decoded instruction output
    output dec_entry_t  i_bus,          // Decoded instruction
    output              decq_empty,     // Instruction queue empty
    output              decq_full       // Instruction queue full
);

typedef enum logic [1:0] {
    DEC_STRUCT = 2'd0,                  // prefix/0F or opcode+ModR/M
    DEC_SIB    = 2'd1,                  // SIB byte for 32-bit ModR/M r/m=100
    DEC_LIT1   = 2'd2,                  // first literal field
    DEC_LIT2   = 2'd3                   // second literal field
} dec_state_t;

typedef enum logic [1:0] {
    LIT_NONE = 2'd0,
    LIT_IMM  = 2'd1,
    LIT_DISP = 2'd2
} lit_kind_t;

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

localparam DECQ_DEPTH = 2;
localparam DECQ_PTR_W = (DECQ_DEPTH <= 2) ? 1 : 2;
localparam [2:0] DECQ_DEPTH_COUNT = DECQ_DEPTH[2:0];

dec_state_t dec_state;
decoder_work_t work;

// Prefix state is not part of dec_entry_t until a non-prefix opcode is decoded.
logic        prefix_66;
logic        prefix_67;
logic        prefix_0f;
logic        prefix_rep;
logic [3:0]  prefix_count;
logic [1:0]  prefix_rep_lock;
logic [2:0]  prefix_seg;

// Queue
reg [DECQ_PTR_W-1:0] decq_rptr;
reg [DECQ_PTR_W-1:0] decq_wptr;
reg [2:0]            decq_count;
(* ramstyle = "logic" *) dec_entry_t decq [0:DECQ_DEPTH-1];

assign i_bus = decq[decq_rptr];
assign decq_empty = (decq_count == 3'd0);
assign decq_full  = (decq_count == DECQ_DEPTH_COUNT);

// Current byte window aliases.
wire [7:0] opcode = q_window[7:0];
wire [7:0] modrm  = q_window[15:8];
wire [7:0] sib    = q_window[23:16];
wire       data32 = D ^ prefix_66;
wire       addr32 = D ^ prefix_67;

wire consume_prefix = !prefix_0f && is_prefix(opcode);
wire consume_0f = !prefix_0f && (opcode == 8'h0f);

// One-cycle structural decode candidate.
decoder_work_t struct_work;
logic          struct_valid;
logic [2:0]    struct_len;
logic          struct_complete;
logic          sib_ready;
logic          lit1_ready;
logic          lit2_ready;
decoder_work_t sib_work;
decoder_work_t lit1_work;
decoder_work_t lit2_work;

always_comb begin
    build_struct_work(struct_work, struct_len);
    struct_valid = !pf_empty && !decq_full && !q_flush &&
                   (pf_count >= {3'b000, struct_len});
    struct_complete = struct_valid && !struct_work.need_sib &&
                      (struct_work.lit1_kind == LIT_NONE);
    sib_ready = !pf_empty && !decq_full && !q_flush && (pf_count >= 6'd1);
    lit1_ready = !pf_empty && !decq_full && !q_flush &&
                 (pf_count >= {3'b000, work.lit1_size});
    lit2_ready = !pf_empty && !decq_full && !q_flush &&
                 (pf_count >= {3'b000, work.lit2_size});

    sib_work = capture_sib(work);
    lit1_work = capture_literal(work, work.lit1_kind, work.lit1_size,
                                work.lit1_sign_extend, work.lit1_mirror_disp);
    lit2_work = capture_literal(work, work.lit2_kind, work.lit2_size,
                                work.lit2_sign_extend, work.lit2_mirror_disp);
end

assign q_pop_bytes =
    (dec_state == DEC_STRUCT) ? ((consume_prefix || consume_0f) && !pf_empty && !q_flush ? 3'd1 :
                                 struct_valid ? struct_len : 3'd0) :
    (dec_state == DEC_SIB)    ? (sib_ready ? 3'd1 : 3'd0) :
    (dec_state == DEC_LIT1)   ? (lit1_ready ? work.lit1_size : 3'd0) :
    (dec_state == DEC_LIT2)   ? (lit2_ready ? work.lit2_size : 3'd0) :
                                3'd0;

wire i_push =
    (dec_state == DEC_STRUCT && struct_complete) ||
    (dec_state == DEC_SIB && sib_ready && sib_work.lit1_kind == LIT_NONE) ||
    (dec_state == DEC_LIT1 && lit1_ready && lit1_work.lit2_kind == LIT_NONE) ||
    (dec_state == DEC_LIT2 && lit2_ready);

wire [DECQ_PTR_W-1:0] decq_wptr_next = decq_wptr + 1'b1;
wire [DECQ_PTR_W-1:0] decq_rptr_next = decq_rptr + 1'b1;

dec_entry_t push_entry;
always_comb begin
    if (dec_state == DEC_STRUCT)
        push_entry = struct_work.entry;
    else if (dec_state == DEC_SIB)
        push_entry = sib_work.entry;
    else if (dec_state == DEC_LIT1)
        push_entry = lit1_work.entry;
    else
        push_entry = lit2_work.entry;
end

always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        dec_state <= DEC_STRUCT;
        work <= '0;
        prefix_66 <= 1'b0;
        prefix_67 <= 1'b0;
        prefix_0f <= 1'b0;
        prefix_rep <= 1'b0;
        prefix_count <= 4'd0;
        prefix_rep_lock <= PREFIX_NOREPLOCK;
        prefix_seg <= PREFIX_NOSEG;
    end else if (q_flush) begin
        dec_state <= DEC_STRUCT;
        work <= '0;
        prefix_66 <= 1'b0;
        prefix_67 <= 1'b0;
        prefix_0f <= 1'b0;
        prefix_rep <= 1'b0;
        prefix_count <= 4'd0;
        prefix_rep_lock <= PREFIX_NOREPLOCK;
        prefix_seg <= PREFIX_NOSEG;
    end else begin
        unique case (dec_state)
            DEC_STRUCT: begin
                if ((q_pop_bytes != 3'd0) && (consume_prefix || consume_0f)) begin
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
                end else if (struct_valid) begin
                    if (struct_work.need_sib) begin
                        work <= struct_work;
                        dec_state <= DEC_SIB;
                    end else if (!struct_complete) begin
                        work <= struct_work;
                        dec_state <= DEC_LIT1;
                    end
                    if (struct_complete) begin
                        prefix_66 <= 1'b0;
                        prefix_67 <= 1'b0;
                        prefix_0f <= 1'b0;
                        prefix_rep <= 1'b0;
                        prefix_count <= 4'd0;
                        prefix_rep_lock <= PREFIX_NOREPLOCK;
                        prefix_seg <= PREFIX_NOSEG;
                    end
                end
            end

            DEC_SIB: begin
                if (sib_ready) begin
                    if (sib_work.lit1_kind != LIT_NONE) begin
                        work <= sib_work;
                        dec_state <= DEC_LIT1;
                    end else begin
                        dec_state <= DEC_STRUCT;
                        work <= sib_work;
                        prefix_66 <= 1'b0;
                        prefix_67 <= 1'b0;
                        prefix_0f <= 1'b0;
                        prefix_rep <= 1'b0;
                        prefix_count <= 4'd0;
                        prefix_rep_lock <= PREFIX_NOREPLOCK;
                        prefix_seg <= PREFIX_NOSEG;
                    end
                end
            end

            DEC_LIT1: begin
                if (lit1_ready) begin
                    work <= lit1_work;
                    if (lit1_work.lit2_kind != LIT_NONE) begin
                        dec_state <= DEC_LIT2;
                    end else begin
                        dec_state <= DEC_STRUCT;
                        prefix_66 <= 1'b0;
                        prefix_67 <= 1'b0;
                        prefix_0f <= 1'b0;
                        prefix_rep <= 1'b0;
                        prefix_count <= 4'd0;
                        prefix_rep_lock <= PREFIX_NOREPLOCK;
                        prefix_seg <= PREFIX_NOSEG;
                    end
                end
            end

            DEC_LIT2: begin
                if (lit2_ready) begin
                    dec_state <= DEC_STRUCT;
                    work <= lit2_work;
                    prefix_66 <= 1'b0;
                    prefix_67 <= 1'b0;
                    prefix_0f <= 1'b0;
                    prefix_rep <= 1'b0;
                    prefix_count <= 4'd0;
                    prefix_rep_lock <= PREFIX_NOREPLOCK;
                    prefix_seg <= PREFIX_NOSEG;
                end
            end

            default: dec_state <= DEC_STRUCT;
        endcase
    end
end

//=============================================================================
// Instruction Queue Management
//=============================================================================

always_ff @(posedge clk) begin
    if (!reset_n || q_flush) begin
        decq_rptr <= {DECQ_PTR_W{1'b0}};
        decq_wptr <= {DECQ_PTR_W{1'b0}};
        decq_count <= 3'd0;
    end else begin
        unique case ({i_push, i_pop})
            2'b10: begin
                decq[decq_wptr] <= push_entry;
                decq_wptr <= decq_wptr_next;
                decq_count <= decq_count + 3'd1;
            end
            2'b01: begin
                decq_rptr <= decq_rptr_next;
                decq_count <= decq_count - 3'd1;
            end
            2'b11: begin
                decq[decq_wptr] <= push_entry;
                decq_wptr <= decq_wptr_next;
                decq_rptr <= decq_rptr_next;
            end
            default: ;
        endcase
    end
end

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

        entry_first = pla_entry_lookup({data32, opcode, prefix_rep, pe_enable,
                                        1'b1, prefix_0f});
        entry_group = ~(|entry_first[11:6]) && has_modrm;
        // Parallel copy of entry_first[5:0] for group rows, so the
        // second-level lookup below does not chain behind the first.
        group_code = pla_group_lookup({data32, opcode, pe_enable, prefix_0f});

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

            entry_final = entry_group ?
                pla_entry_lookup({data32, group_code[5:4], modrm[5:3],
                                  group_code[3:0], (modrm[7:6] != 2'b11),
                                  1'b0, prefix_0f}) :
                entry_first;

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
            entry_final = entry_first;
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

        invalid_lock = check_lock_invalid(prefix_rep_lock, prefix_0f, opcode,
                                          has_modrm, modrm);
        w.entry.entry_point = invalid_lock ? 12'h82B : entry_final[11:0];
        w.entry.stack_op = invalid_lock ? 1'b0 : entry_final[13];
        w.entry.stack_dir = invalid_lock ? 1'b0 : entry_final[12];
        w.entry.length = {1'b0, prefix_count} + (prefix_0f ? 5'd1 : 5'd0) +
                         {2'b00, s_len} + {2'b00, disp_size} +
                         {2'b00, imm_total_size};
    end
endtask

function automatic decoder_work_t capture_sib(input decoder_work_t in);
    decoder_work_t out;
    logic [7:0] sib_byte;
    logic [2:0] disp_size;
    begin
        out = in;
        sib_byte = q_window[7:0];
        disp_size = modrm_disp_size(in.entry.addr32, in.entry.modrm,
                                    sib_byte, in.entry.has_sib);

        out.need_sib = 1'b0;
        out.entry.sib = sib_byte;
        out.entry.length = in.entry.length + 5'd1 + {2'b00, disp_size};

        if (disp_size != 3'd0) begin
            out.lit1_kind = LIT_DISP;
            out.lit1_size = disp_size;
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
    input lit_kind_t     kind,
    input logic [2:0]    size,
    input logic          sign_extend,
    input logic          mirror_disp
);
    decoder_work_t out;
    logic [31:0] value;
    begin
        out = in;
        value = literal_value(q_window, size, sign_extend);
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
