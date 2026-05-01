//=============================================================================
// Z386 Instruction Decoder Module
//=============================================================================
// Decodes x86 instructions byte-by-byte using pla_control and pla_entry
// Produces: prefixes, opcode, modrm, and microcode entry point
//
module decoder
    import z386_pkg::*;
(
    input               clk,
    input               reset_n,

    // Prefetch queue interface
    input        [7:0]  q_bus,          // Byte from prefetch queue
    input        [31:0] q_window,       // 4-byte aligned window at current queue head
    input        [4:0]  pf_count,       // Buffered prefetch bytes available
    input               pf_empty,       // Prefetch queue empty
    output       [2:0]  q_pop_bytes,    // Pop 1/2/4 bytes from prefetch queue

    // Mode signals
    input               D,              // Default operand size (CS.D bit)
    input               mode_32,        // 32-bit mode
    input               pe_enable,      // Protected mode enable (CR0.PE)

    // Control signals
    input               q_flush,        // Flush decoder on branch
    input               i_pop,          // Pop decoded instruction from queue
    input               halted,         // CPU is halted
    input               stall,          // Stall decoder

    // Decoded instruction output
    output dec_entry_t  i_bus,          // Decoded instruction
    output              decq_empty,     // Instruction queue empty
    output              decq_full       // Instruction queue full
);

// Decode state machine
typedef enum logic [2:0] {
    DEC_IDLE        = 3'd0, // Waiting for instruction (prefix or opcode byte)
    DEC_MODRM       = 3'd1, // Consuming ModR/M byte
    DEC_SIB         = 3'd2, // Consuming SIB byte (32-bit addressing with r/m=100)
    DEC_DISP        = 3'd3, // Consuming Displacement bytes
    DEC_IMM         = 3'd4, // Consuming Immediate bytes
    DEC_DONE        = 3'd5  // Instruction decoded
} dec_state_t;

`include "pla_control.svh"
`include "pla_entry.svh"

dec_state_t dec_state;

// Decoded instruction being built
dec_entry_t dec;
dec_entry_t dec_next;
dec_entry_t dec_push_entry;
dec_state_t dec_state_next;
logic        dec_complete_next;

// Prefix size tracking
reg dec_prefix_data_size;           // 66h prefix seen
reg dec_prefix_addr_size;           // 67h prefix seen
logic dec_prefix_data_size_next;
logic dec_prefix_addr_size_next;

// PLA Control state (7-bit state for pattern matching)
reg [6:0]  ctl_state;               // PLA Control state: ?000000 (initial) -> ?011000 (modrm) -> ?111000 (complex)

// Internal decoder state (not part of output)
reg [11:0] dec_ctl_first;           // First-pass PLA Control result (for ModR/M instructions)
reg [15:0] dec_entry_first;         // First-pass PLA Entry result (for group instructions)
reg [11:0] dec_ctl_opcode_bits;     // PLA Control output captured for opcode byte
reg [2:0]  dec_disp_count;          // Byte counter for displacement loading
reg [2:0]  dec_disp_size;           // Expected displacement size (0, 1, or 4 bytes)
reg [2:0]  dec_imm_count;           // Byte counter for immediate loading
reg        dec_imm_sign_extended;   // Immediate is sign-extended
reg [2:0]  dec_imm_first_size;      // Bytes to immediate (rest go to displacement), default=imm_size
logic [6:0]  ctl_state_next;
logic [11:0] dec_ctl_first_next;
logic [15:0] dec_entry_first_next;
logic [2:0]  dec_disp_count_next;
logic [2:0]  dec_disp_size_next;
logic [2:0]  dec_imm_count_next;
logic        dec_imm_sign_extended_next;
logic [2:0]  dec_imm_first_size_next;
logic        entry_2nd_pass_next;
dec_entry_t  dec_reg_next;
dec_state_t  dec_state_reg_next;
logic [6:0]  ctl_state_reg_next;
logic [11:0] dec_ctl_first_reg_next;
logic [15:0] dec_entry_first_reg_next;
logic [2:0]  dec_disp_count_reg_next;
logic [2:0]  dec_disp_size_reg_next;
logic [2:0]  dec_imm_count_reg_next;
logic        dec_imm_sign_extended_reg_next;
logic [2:0]  dec_imm_first_size_reg_next;
logic        dec_prefix_data_size_reg_next;
logic        dec_prefix_addr_size_reg_next;
logic        entry_2nd_pass_reg_next;

localparam bit TRACE_DEBUG_EN = 1'b0;

localparam DECQ_DEPTH = 4;
localparam DECQ_PTR_W = 2;

reg [DECQ_PTR_W-1:0] decq_rptr;
reg [DECQ_PTR_W-1:0] decq_wptr;
reg [2:0]            decq_count;
(* ramstyle = "logic" *) dec_entry_t decq [0:DECQ_DEPTH-1];

assign i_bus = decq[decq_rptr];   // Instruction queue head
assign decq_empty = (decq_count == 3'd0);
assign decq_full  = (decq_count == 3'd4);

//=============================================================================
// PLA Control Decoding
//=============================================================================

wire [7:0] q_idle_byte   = q_bus;
wire [7:0] q_modrm_byte  = q_bus;
wire [7:0] q_capture_byte= q_bus;

wire mode_x = dec.addr32;
wire mode_w = ~mode_x & (q_modrm_byte[7:6] == 2'b00) & (q_modrm_byte[2:0] == 3'b110);  // 16-bit disp-only
wire mode_v = mode_x & (q_modrm_byte[7:6] == 2'b00) & (q_modrm_byte[2:0] == 3'b101);   // 32-bit disp-only
wire mode_u = mode_x & (q_modrm_byte[7:6] != 2'b11) & (q_modrm_byte[2:0] == 3'b100);   // SIB follows

wire [11:0] idle_ctl_bits = pla_control_lookup(7'b0000000, q_idle_byte, {dec.has_0f, 4'b0000});
wire [4:0] modrm_ctl_mode = {dec.has_0f, mode_u, mode_v, mode_w, mode_x};
wire [11:0] modrm_ctl_bits = pla_control_lookup(ctl_state, q_modrm_byte, modrm_ctl_mode);

// PLA Entry lookup: first-pass for all instructions, second-pass for group instructions
reg entry_2nd_pass;
// Effective operand size for PLA Entry lookup: D (CS.D default) XOR 66 prefix
wire entry_data32 = D ^ dec_prefix_data_size;
wire [15:0] idle_entry_result =
    pla_entry_lookup({entry_data32, q_idle_byte, dec.has_rep, pe_enable, 1'b1, dec.has_0f});
wire [15:0] modrm_entry_result =
    pla_entry_lookup({entry_data32, dec_entry_first[5:4], q_modrm_byte[5:3], dec_entry_first[3:0], (q_modrm_byte[7:6] != 2'b11), 1'b0, dec.has_0f});
wire idle_entry_is_group = ~(|idle_entry_result[11:6]);

wire [6:0] ctl_next_state = {ctl_state[6], 1'b0, 1'b1, 1'b1, 1'b0, 1'b0 /*ctl_f & ~ctl_k*/, 1'b0};
wire idle_is_prefix = ~idle_ctl_bits[9] & ~idle_ctl_bits[8] & idle_ctl_bits[2] & ~idle_ctl_bits[1] & ~idle_ctl_bits[0];

// First-pass PLA Control-derived signals for dec_entry_t (see doc/decode.md)
wire idle_ctl_has_d_bit =
    idle_ctl_bits[11] & ~idle_ctl_bits[10] & idle_ctl_bits[9] & idle_ctl_bits[8] & idle_ctl_bits[2] & idle_ctl_bits[1];
wire idle_ctl_has_embedded_reg = ~idle_ctl_bits[2];  // j=0 means opcode[2:0] is register
wire idle_ctl_has_w_bit = idle_ctl_bits[1];          // k=1 means opcode[0] selects byte/word
wire idle_ctl_is_pushpop_seg = idle_ctl_bits[3];     // i=1 for segment register PUSH/POP
wire idle_ctl_update_flags = idle_ctl_bits[4];       // h=1 for ALU ops that modify EFLAGS

function automatic [2:0] choose_literal_chunk(input [2:0] remaining, input [4:0] available);
    begin
        if (remaining >= 3'd4 && available >= 5'd4)
            choose_literal_chunk = 3'd4;
        else if (remaining >= 3'd2 && available >= 5'd2)
            choose_literal_chunk = 3'd2;
        else
            choose_literal_chunk = 3'd1;
    end
endfunction

wire [2:0] dec_disp_chunk_bytes = choose_literal_chunk(dec_disp_size - dec_disp_count, pf_count);
wire [2:0] dec_imm_chunk_bytes = choose_literal_chunk(dec.imm_size - dec_imm_count, pf_count);
wire [2:0] dec_consume_bytes = (dec_state == DEC_DISP) ? dec_disp_chunk_bytes :
                               (dec_state == DEC_IMM) ? dec_imm_chunk_bytes :
                               3'd1;

wire dec_can_consume = !pf_empty && !decq_full && !q_flush && (dec_state != DEC_DONE);
wire i_push = dec_can_consume && dec_complete_next;
wire dec_restart = i_push;

always_comb begin
    dec_next = dec;
    dec_state_next = dec_state;
    dec_complete_next = 1'b0;
    ctl_state_next = ctl_state;
    dec_ctl_first_next = dec_ctl_first;
    dec_entry_first_next = dec_entry_first;
    dec_disp_count_next = dec_disp_count;
    dec_disp_size_next = dec_disp_size;
    dec_imm_count_next = dec_imm_count;
    dec_imm_sign_extended_next = dec_imm_sign_extended;
    dec_imm_first_size_next = dec_imm_first_size;
    dec_prefix_data_size_next = dec_prefix_data_size;
    dec_prefix_addr_size_next = dec_prefix_addr_size;
    entry_2nd_pass_next = entry_2nd_pass;

    if (dec_can_consume) begin
        case (dec_state)
            DEC_IDLE: begin
                    if (!dec.has_0f && q_idle_byte == 8'h0F) begin
                        dec_next.has_0f = 1'b1;
                    end else if (idle_is_prefix) begin
                        if (q_idle_byte != 8'h0F)
                            dec_next.prefix_count = dec.prefix_count + 4'd1;

                        case (q_idle_byte)
                            8'hF0: dec_next.rep_lock = PREFIX_LOCK;
                            8'hF2: begin
                                dec_next.rep_lock = PREFIX_REPNE;
                            dec_next.has_rep = 1'b1;
                        end
                        8'hF3: begin
                            dec_next.rep_lock = PREFIX_REP;
                            dec_next.has_rep = 1'b1;
                        end
                        8'h26: dec_next.seg = PREFIX_ES;
                        8'h2E: dec_next.seg = PREFIX_CS;
                        8'h36: dec_next.seg = PREFIX_SS;
                        8'h3E: dec_next.seg = PREFIX_DS;
                        8'h64: dec_next.seg = PREFIX_FS;
                        8'h65: dec_next.seg = PREFIX_GS;
                        8'h66: dec_prefix_data_size_next = 1'b1;
                        8'h67: dec_prefix_addr_size_next = 1'b1;
                        default: ;
                    endcase
                end else begin
                    logic [2:0] imm_size_calc;
                    logic       data32_calc;

                    imm_size_calc = 3'd0;
                    data32_calc = D ^ dec_prefix_data_size;
                    dec_imm_sign_extended_next = 1'b0;
                    dec_next.data32 = data32_calc;
                    dec_next.addr32 = D ^ dec_prefix_addr_size;
                    dec_next.opcode = q_capture_byte;
                    dec_next.has_d_bit = idle_ctl_has_d_bit;
                    dec_next.has_embedded_register = idle_ctl_has_embedded_reg;
                    dec_next.has_w_bit = idle_ctl_has_w_bit;
                    dec_next.is_pushpop_seg = idle_ctl_is_pushpop_seg;
                    dec_next.update_flags = idle_ctl_update_flags;

                    if (idle_ctl_bits[2] & ~idle_ctl_bits[0]) begin
                        dec_next.has_modrm = 1'b1;
                        dec_ctl_first_next = idle_ctl_bits;
                        ctl_state_next = ctl_next_state;

                        unique case (idle_ctl_bits[11:10])
                            2'b00: imm_size_calc = 3'd1;
                            2'b01: imm_size_calc = data32_calc ? 3'd4 : 3'd2;
                            2'b10: imm_size_calc = 3'd0;
                            2'b11: begin
                                imm_size_calc = 3'd1;
                                dec_imm_sign_extended_next = 1'b1;
                            end
                        endcase

                        if (idle_entry_is_group) begin
                            dec_entry_first_next = idle_entry_result;
                            entry_2nd_pass_next = 1'b1;
                        end else begin
                            dec_next.entry_point = idle_entry_result[11:0];
                            dec_next.stack_op = idle_entry_result[13];
                            dec_next.stack_dir = idle_entry_result[12];
                            dec_entry_first_next = 16'h0000;
                            entry_2nd_pass_next = 1'b0;
                        end

                        dec_state_next = DEC_MODRM;
                    end else begin
                        dec_next.entry_point = (dec.rep_lock == PREFIX_LOCK) ? 12'h82B : idle_entry_result[11:0];
                        dec_next.stack_op = idle_entry_result[13];
                        dec_next.stack_dir = idle_entry_result[12];

                        if (idle_ctl_has_embedded_reg) begin
                            if (q_idle_byte[7:3] == 5'b10010 && q_idle_byte[2:0] != 3'b000) begin
                                dec_next.src_reg_sel = q_idle_byte[2:0];
                                dec_next.dst_reg_sel = 3'h0;
                            end else begin
                                dec_next.src_reg_sel = q_idle_byte[2:0];
                                dec_next.dst_reg_sel = q_idle_byte[2:0];
                            end
                        end else begin
                            dec_next.dst_reg_sel = 3'h0;
                            dec_next.src_reg_sel = 3'h0;
                        end

                        if (!dec.has_0f && q_idle_byte[7:2] == 6'b101000) begin
                            imm_size_calc = (D ^ dec_prefix_addr_size) ? 3'd4 : 3'd2;
                            dec_next.has_moffs = 1'b1;
                        end else begin
                            dec_next.has_moffs = 1'b0;
                            unique case (idle_ctl_bits[11:6])
                                6'b100111: imm_size_calc = 3'd1;
                                6'b110111: imm_size_calc = data32_calc ? 3'd4 : 3'd2;
                                6'b000111: begin
                                    imm_size_calc = 3'd1;
                                    dec_imm_sign_extended_next = 1'b1;
                                end
                                6'b010111: imm_size_calc = 3'd2;
                                6'b011111: imm_size_calc = data32_calc ? 3'd4 : 3'd2;
                                6'b100010: imm_size_calc = 3'd1;
                                6'b101111: imm_size_calc = 3'd3;
                                6'b111111: imm_size_calc = data32_calc ? 3'd6 : 3'd4;
                                default:   imm_size_calc = 3'd0;
                            endcase
                        end

                        if (imm_size_calc > 3'd0) begin
                            dec_state_next = DEC_IMM;
                            dec_imm_count_next = 3'd0;
                            dec_next.immediate = 32'h0;
                            dec_next.displacement = 32'h0;
                            dec_imm_first_size_next = imm_size_calc;

                            unique case (idle_ctl_bits[11:6])
                                6'b100010: begin
                                    dec_imm_sign_extended_next = 1'b1;
                                    dec_imm_first_size_next = 3'd0;
                                end
                                6'b101111: dec_imm_first_size_next = 3'd2;
                                6'b111111: dec_imm_first_size_next = data32_calc ? 3'd4 : 3'd2;
                                default: ;
                            endcase
                        end else begin
                            dec_next.length = {1'b0, dec.prefix_count} + (dec.has_0f ? 5'd1 : 5'd0) + 5'd1;
                            dec_complete_next = 1'b1;
                        end
                    end

                    dec_next.imm_size = imm_size_calc;
                end
            end

            DEC_MODRM: begin
                logic [2:0] disp_size_calc;
                logic [2:0] imm_size_calc;
                logic       suppress_imm;

                dec_next.modrm = q_capture_byte;

                if (dec.opcode[7:1] == 7'b1000011) begin
                    dec_next.src_reg_sel = q_modrm_byte[5:3];
                    dec_next.dst_reg_sel = (q_modrm_byte[7:6] == 2'b11) ? q_modrm_byte[2:0] : q_modrm_byte[5:3];
                end else if (dec.opcode == 8'h8D) begin
                    dec_next.src_reg_sel = q_modrm_byte[5:3];
                    dec_next.dst_reg_sel = q_modrm_byte[2:0];
                end else if (dec.has_0f && dec.opcode[7:4] == 4'b1011 && dec.opcode[2:1] == 2'b11) begin
                    dec_next.dst_reg_sel = q_modrm_byte[2:0];
                    dec_next.src_reg_sel = q_modrm_byte[5:3];
                end else if (dec.has_0f && dec.opcode[7:3] == 5'b00100) begin
                    if (dec.opcode[1]) begin
                        dec_next.dst_reg_sel = q_modrm_byte[5:3];
                        dec_next.src_reg_sel = q_modrm_byte[2:0];
                    end else begin
                        dec_next.dst_reg_sel = q_modrm_byte[2:0];
                        dec_next.src_reg_sel = q_modrm_byte[5:3];
                    end
                end else if (dec.has_d_bit && dec.opcode[1]) begin
                    dec_next.dst_reg_sel = q_modrm_byte[5:3];
                    dec_next.src_reg_sel = q_modrm_byte[2:0];
                end else begin
                    dec_next.dst_reg_sel = q_modrm_byte[2:0];
                    dec_next.src_reg_sel = q_modrm_byte[5:3];
                end

                if (entry_2nd_pass) begin
                    dec_next.entry_point = modrm_entry_result[11:0];
                    dec_next.stack_op = modrm_entry_result[13];
                    dec_next.stack_dir = modrm_entry_result[12];
                end

                if (check_lock_invalid(dec.rep_lock, dec.has_0f, dec.opcode, 1'b1, q_modrm_byte))
                    dec_next.entry_point = 12'h82B;

                suppress_imm = (dec.opcode[7:1] == 7'b1111011) && (q_modrm_byte[5:3] >= 3'b010);
                imm_size_calc = suppress_imm ? 3'd0 : dec.imm_size;
                dec_next.imm_size = imm_size_calc;
                dec_next.has_sib = modrm_ctl_bits[6];

                case ({modrm_ctl_bits[6], modrm_ctl_bits[7], modrm_ctl_bits[8]})
                    3'b000: disp_size_calc = 3'd0;
                    3'b010: disp_size_calc = 3'd1;
                    3'b011: disp_size_calc = dec.addr32 ? 3'd4 : 3'd2;
                    3'b100: disp_size_calc = 3'd0;
                    3'b101: disp_size_calc = 3'd1;
                    3'b110: disp_size_calc = dec.addr32 ? 3'd4 : 3'd2;
                    3'b111: disp_size_calc = 3'd4;
                    default: disp_size_calc = 3'd0;
                endcase
                dec_disp_size_next = disp_size_calc;

                if (modrm_ctl_bits[6]) begin
                    dec_state_next = DEC_SIB;
                end else if (modrm_ctl_bits[7] | modrm_ctl_bits[8]) begin
                    dec_state_next = DEC_DISP;
                    dec_disp_count_next = 3'd0;
                    dec_next.immediate = 32'h0;
                end else if (imm_size_calc != 3'd0) begin
                    dec_state_next = DEC_IMM;
                    dec_imm_count_next = 3'd0;
                    dec_next.immediate = 32'h0;
                    dec_imm_first_size_next = imm_size_calc;
                end else begin
                    dec_next.length = {1'b0, dec.prefix_count} + (dec.has_0f ? 5'd1 : 5'd0) + 5'd2;
                    dec_complete_next = 1'b1;
                end
            end

            DEC_SIB: begin
                logic [2:0] disp_size_calc;

                dec_next.sib = q_capture_byte;
                disp_size_calc = dec_disp_size;
                if (dec.modrm[7:6] == 2'b00 && q_capture_byte[2:0] == 3'b101)
                    disp_size_calc = 3'd4;
                dec_disp_size_next = disp_size_calc;

                if (disp_size_calc != 3'd0) begin
                    dec_state_next = DEC_DISP;
                    dec_disp_count_next = 3'd0;
                    dec_next.immediate = 32'h0;
                end else if (dec.imm_size != 3'd0) begin
                    dec_state_next = DEC_IMM;
                    dec_imm_count_next = 3'd0;
                    dec_next.immediate = 32'h0;
                    dec_imm_first_size_next = dec.imm_size;
                end else begin
                    dec_next.length = {1'b0, dec.prefix_count} + (dec.has_0f ? 5'd1 : 5'd0) + 5'd3;
                    dec_complete_next = 1'b1;
                end
            end

            DEC_DISP: begin
                integer disp_idx;

                for (disp_idx = 0; disp_idx < 4; disp_idx = disp_idx + 1) begin
                    if (disp_idx < dec_disp_chunk_bytes)
                        dec_next.displacement[(dec_disp_count + disp_idx) * 8 +: 8] = q_window[disp_idx * 8 +: 8];
                end

                if (dec_disp_count + dec_disp_chunk_bytes >= dec_disp_size) begin
                    if (dec.imm_size != 3'd0) begin
                        dec_state_next = DEC_IMM;
                        dec_imm_count_next = 3'd0;
                        dec_imm_first_size_next = dec.imm_size;
                    end else begin
                        dec_next.length = {1'b0, dec.prefix_count} + (dec.has_0f ? 5'd1 : 5'd0) + 5'd2 +
                                          (dec.has_sib ? 5'd1 : 5'd0) + {2'b0, dec_disp_size};
                        dec_complete_next = 1'b1;
                    end
                end
                dec_disp_count_next = dec_disp_count + dec_disp_chunk_bytes;
            end

            DEC_IMM: begin
                integer imm_idx;
                integer imm_byte_idx;
                integer disp_byte_idx;

                for (imm_idx = 0; imm_idx < 4; imm_idx = imm_idx + 1) begin
                    if (imm_idx < dec_imm_chunk_bytes) begin
                        imm_byte_idx = dec_imm_count + imm_idx;
                        if (imm_byte_idx < dec_imm_first_size) begin
                            dec_next.immediate[imm_byte_idx * 8 +: 8] = q_window[imm_idx * 8 +: 8];
                            if (~dec.has_modrm)
                                dec_next.displacement[imm_byte_idx * 8 +: 8] = q_window[imm_idx * 8 +: 8];
                        end else begin
                            disp_byte_idx = imm_byte_idx - dec_imm_first_size;
                            dec_next.displacement[disp_byte_idx * 8 +: 8] = q_window[imm_idx * 8 +: 8];
                        end
                    end
                end

                if (dec_imm_sign_extended) begin
                    if (dec_imm_count < dec_imm_first_size) begin
                        if (dec_imm_first_size == 3'd1) begin
                            dec_next.immediate[31:8] = {24{q_window[7]}};
                            if (~dec.has_modrm)
                                dec_next.displacement[31:8] = {24{q_window[7]}};
                        end
                    end else begin
                        dec_next.displacement[31:8] = {24{q_window[7]}};
                    end
                end

                if (dec_imm_count + dec_imm_chunk_bytes >= dec.imm_size) begin
                    dec_next.length = {1'b0, dec.prefix_count} + (dec.has_0f ? 5'd1 : 5'd0) + 5'd1 +
                                      (dec.has_modrm ? 5'd1 : 5'd0) + (dec.has_sib ? 5'd1 : 5'd0) +
                                      {2'b0, dec_disp_size} + {2'b0, dec.imm_size};
                    dec_complete_next = 1'b1;
                end
                dec_imm_count_next = dec_imm_count + dec_imm_chunk_bytes;
            end

            default: dec_state_next = DEC_IDLE;
        endcase
    end
end

assign q_pop_bytes = dec_can_consume ? dec_consume_bytes : 3'd0;

always_comb begin
    dec_reg_next = dec_next;
    dec_state_reg_next = dec_state_next;
    ctl_state_reg_next = ctl_state_next;
    dec_ctl_first_reg_next = dec_ctl_first_next;
    dec_entry_first_reg_next = dec_entry_first_next;
    dec_disp_count_reg_next = dec_disp_count_next;
    dec_disp_size_reg_next = dec_disp_size_next;
    dec_imm_count_reg_next = dec_imm_count_next;
    dec_imm_sign_extended_reg_next = dec_imm_sign_extended_next;
    dec_imm_first_size_reg_next = dec_imm_first_size_next;
    dec_prefix_data_size_reg_next = dec_prefix_data_size_next;
    dec_prefix_addr_size_reg_next = dec_prefix_addr_size_next;
    entry_2nd_pass_reg_next = entry_2nd_pass_next;

    if (dec_restart) begin
        dec_state_reg_next = DEC_IDLE;
        ctl_state_reg_next = 7'b0000000;
        dec_reg_next.opcode = 8'h00;
        dec_reg_next.modrm = 8'h00;
        dec_reg_next.sib = 8'h00;
        dec_reg_next.has_0f = 1'b0;
        dec_reg_next.has_modrm = 1'b0;
        dec_reg_next.has_sib = 1'b0;
        dec_reg_next.immediate = 32'h00000000;
        dec_reg_next.displacement = 32'h00000000;
        dec_reg_next.rep_lock = 2'b00;
        dec_reg_next.seg = PREFIX_NOSEG;
        dec_reg_next.prefix_count = 4'h0;
        dec_reg_next.has_rep = 1'b0;
        dec_disp_size_reg_next = 3'd0;
        dec_disp_count_reg_next = 3'd0;
        dec_reg_next.imm_size = 3'd0;
        dec_imm_count_reg_next = 3'd0;
        dec_imm_sign_extended_reg_next = 1'b0;
        dec_reg_next.data32 = D;
        dec_reg_next.addr32 = D;
        dec_reg_next.has_d_bit = 1'b0;
        dec_reg_next.has_embedded_register = 1'b0;
        dec_reg_next.has_w_bit = 1'b0;
        dec_reg_next.is_pushpop_seg = 1'b0;
        dec_reg_next.update_flags = 1'b0;
        dec_reg_next.has_moffs = 1'b0;
        dec_reg_next.stack_op = 1'b0;
        dec_reg_next.stack_dir = 1'b0;
        dec_reg_next.src_reg_sel = 3'h0;
        dec_reg_next.dst_reg_sel = 3'h0;
        dec_prefix_data_size_reg_next = 1'b0;
        dec_prefix_addr_size_reg_next = 1'b0;
        entry_2nd_pass_reg_next = 1'b0;
        dec_entry_first_reg_next = 16'h0000;
        dec_ctl_first_reg_next = 12'h000;
        dec_imm_first_size_reg_next = 3'd4;
    end
end

always_comb begin
    dec_push_entry = dec_next;

    if (dec_complete_next) begin
        unique case (dec_state)
            DEC_IDLE,
            DEC_MODRM,
            DEC_SIB: begin
                dec_push_entry.immediate = 32'h0;
                dec_push_entry.displacement = 32'h0;
            end

            DEC_DISP: begin
                dec_push_entry.immediate = 32'h0;
            end

            default: begin
            end
        endcase
    end
end

always_ff @(posedge clk) begin
    if (!reset_n || q_flush) begin
        dec_state <= DEC_IDLE;
        ctl_state <= 7'b0000000;
        dec.opcode <= 8'h00;
        dec.modrm <= 8'h00;
        dec.sib <= 8'h00;
        dec.has_0f <= 1'b0;
        dec.has_modrm <= 1'b0;
        dec.has_sib <= 1'b0;
        dec.immediate <= 32'h0;
        dec.displacement <= 32'h0;
        dec.rep_lock <= 2'b00;
        dec.seg <= PREFIX_NOSEG;
        dec.prefix_count <= 4'h0;
        dec.has_rep <= 1'b0;
        dec_disp_size <= 3'd0;
        dec_disp_count <= 3'd0;
        dec.imm_size <= 3'd0;
        dec_imm_count <= 3'd0;
        dec_imm_sign_extended <= 1'b0;
        dec.data32 <= D;
        dec.addr32 <= D;
        dec.has_d_bit <= 1'b0;
        dec.has_embedded_register <= 1'b0;
        dec.has_w_bit <= 1'b0;
        dec.is_pushpop_seg <= 1'b0;
        dec.update_flags <= 1'b0;
        dec.has_moffs <= 1'b0;
        dec.stack_op <= 1'b0;
        dec.stack_dir <= 1'b0;
        dec.src_reg_sel <= 3'h0;
        dec.dst_reg_sel <= 3'h0;
        dec_prefix_data_size <= 1'b0;
        dec_prefix_addr_size <= 1'b0;
        entry_2nd_pass <= 1'b0;
        dec_entry_first <= 16'h0000;
        dec_ctl_first <= 12'h000;
        dec_imm_first_size <= 3'd4;
        if (!reset_n)
            dec.entry_point <= 12'h9A6;
    end else begin
        dec <= dec_reg_next;
        dec_state <= dec_state_reg_next;
        ctl_state <= ctl_state_reg_next;
        dec_ctl_first <= dec_ctl_first_reg_next;
        dec_entry_first <= dec_entry_first_reg_next;
        dec_disp_count <= dec_disp_count_reg_next;
        dec_disp_size <= dec_disp_size_reg_next;
        dec_imm_count <= dec_imm_count_reg_next;
        dec_imm_sign_extended <= dec_imm_sign_extended_reg_next;
        dec_imm_first_size <= dec_imm_first_size_reg_next;
        dec_prefix_data_size <= dec_prefix_data_size_reg_next;
        dec_prefix_addr_size <= dec_prefix_addr_size_reg_next;
        entry_2nd_pass <= entry_2nd_pass_reg_next;
    end
end

//=============================================================================
// Instruction Queue Management
//=============================================================================

always_ff @(posedge clk) begin
    if (!reset_n) begin
        decq_rptr <= {DECQ_PTR_W{1'b0}};
        decq_wptr <= {DECQ_PTR_W{1'b0}};
        decq_count <= 3'd0;
    end else if (q_flush) begin
        decq_rptr <= {DECQ_PTR_W{1'b0}};
        decq_wptr <= {DECQ_PTR_W{1'b0}};
        decq_count <= 3'd0;
    end else begin
        case ({i_push, i_pop})
            2'b10: begin
                // Push: use the shared next-state decode result
                decq[decq_wptr] <= dec_push_entry;
                decq_wptr <= decq_wptr + 1'b1;
                decq_count <= decq_count + 3'd1;
            end
            2'b01: begin
                decq_rptr <= decq_rptr + 1'b1;
                decq_count <= decq_count - 3'd1;
            end
            2'b11: begin
                // Push and pop simultaneously
                decq[decq_wptr] <= dec_push_entry;
                decq_wptr <= decq_wptr + 1'b1;
                decq_rptr <= decq_rptr + 1'b1;
                decq_count <= decq_count;
            end
            default: ;
        endcase
    end
end

endmodule
