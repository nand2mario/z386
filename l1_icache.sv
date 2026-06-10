// Read-only physically indexed, physically tagged L1 instruction cache.
//
// CPU-side contract:
//   * cpu_addr is a physical byte address.
//   * A cache-hit read accepted in cycle N returns cpu_resp_valid in N+1.
//   * Miss/refill and uncached accesses use the memory-side burst interface.
//
// This is the instruction-cache half of l1_cache.sv with the store buffer and
// write datapath removed.
module l1_icache #(
    parameter integer SET_BITS = 8
) (
    input         clk,
    input         reset,

    // CPU side — physical read request/response.
    input  [31:0] cpu_addr,
    output [31:0] cpu_dout,
    output [127:0] cpu_line,
    input         cpu_valid,
    output        cpu_ready,
    output        cpu_resp_valid,

    // Memory side.
    output [31:0] mem_addr,
    input  [31:0] mem_dout,
    output  [3:0] mem_be,
    output  [7:0] mem_burstcount,
    input         mem_busy,
    output        mem_valid,
    input         mem_ready,
    input         mem_resp_valid,

    // Physical-address snoop.  Data-bearing CPU store snoops patch matching
    // cached words; address-only external snoops invalidate matching lines.
    input  [31:0] snoop_addr,
    input  [31:0] snoop_data,
    input   [3:0] snoop_be,
    input         snoop_patch,
    input         snoop_valid,

    input         cache_enable
);

localparam integer WORD_OFFSET_BITS = 2;
localparam integer BYTE_OFFSET_BITS = 2;
localparam integer LINE_OFFSET_BITS = WORD_OFFSET_BITS + BYTE_OFFSET_BITS;
localparam integer NUM_SETS = 1 << SET_BITS;
localparam integer TAG_BITS = 25 - LINE_OFFSET_BITS - SET_BITS;
localparam integer SET_LSB = LINE_OFFSET_BITS;
localparam integer SET_MSB = SET_LSB + SET_BITS - 1;
localparam integer TAG_LSB = SET_MSB + 1;
localparam integer TAG_MSB = 24;
localparam integer TAG_RAM_BITS = (TAG_BITS < 16) ? 16 : TAG_BITS;
localparam [SET_BITS-1:0] LAST_SET = SET_BITS'(NUM_SETS - 1);
localparam integer PATCHQ_DEPTH = 3;
localparam integer PATCHQ_IDX_BITS = 2;
localparam [PATCHQ_IDX_BITS-1:0] PATCHQ_LAST_IDX = 2'd2;
localparam integer SET_DW_LSB = SET_LSB - BYTE_OFFSET_BITS;
localparam integer SET_DW_MSB = SET_MSB - BYTE_OFFSET_BITS;
localparam integer TAG_DW_LSB = TAG_LSB - BYTE_OFFSET_BITS;
localparam integer TAG_DW_MSB = TAG_MSB - BYTE_OFFSET_BITS;

wire [TAG_BITS-1:0] cpu_tag = cpu_addr[TAG_MSB:TAG_LSB];
wire [SET_BITS-1:0] cpu_set = cpu_addr[SET_MSB:SET_LSB];
wire [WORD_OFFSET_BITS-1:0] cpu_word = cpu_addr[LINE_OFFSET_BITS-1:BYTE_OFFSET_BITS];
wire [TAG_BITS-1:0] snoop_tag = snoop_addr[TAG_MSB:TAG_LSB];
wire [SET_BITS-1:0] snoop_set = snoop_addr[SET_MSB:SET_LSB];
wire [WORD_OFFSET_BITS-1:0] snoop_word = snoop_addr[LINE_OFFSET_BITS-1:BYTE_OFFSET_BITS];
// The prefetcher consumes whole 16-byte lines.  Unlike demand data accesses,
// an instruction fetch cannot be satisfied by a single uncacheable DWORD
// bypass without corrupting branch targets in the middle of the line.
wire cpu_uncacheable = !cache_enable;

(* ramstyle = "M10K" *) reg [TAG_RAM_BITS-1:0] tag_way0 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg [TAG_RAM_BITS-1:0] tag_way1 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg [TAG_RAM_BITS-1:0] tag_way2 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg [TAG_RAM_BITS-1:0] tag_way3 [0:NUM_SETS-1];
reg valid_way0 [0:NUM_SETS-1];
reg valid_way1 [0:NUM_SETS-1];
reg valid_way2 [0:NUM_SETS-1];
reg valid_way3 [0:NUM_SETS-1];
reg [2:0] plru_set [0:NUM_SETS-1];

(* ramstyle = "M10K" *) reg [127:0] data_way0 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg [127:0] data_way1 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg [127:0] data_way2 [0:NUM_SETS-1];
(* ramstyle = "M10K" *) reg [127:0] data_way3 [0:NUM_SETS-1];

reg [TAG_BITS-1:0] rd_tag0_r, rd_tag1_r, rd_tag2_r, rd_tag3_r;
reg rd_valid0_r, rd_valid1_r, rd_valid2_r, rd_valid3_r;
reg [127:0] rd_line0_r, rd_line1_r, rd_line2_r, rd_line3_r;
reg [2:0] rd_plru_r;

reg        req_valid_r;
reg [31:0] req_addr_r;
reg        req_uncacheable_r;
reg [TAG_BITS-1:0] req_tag_r;
reg [SET_BITS-1:0] req_set_r;
reg [WORD_OFFSET_BITS-1:0] req_word_r;

reg        mem_valid_r;
reg [31:0] mem_addr_r;
reg  [7:0] mem_burstcount_r;

assign mem_valid = mem_valid_r;
assign mem_addr = mem_addr_r;
assign mem_be = 4'hF;
assign mem_burstcount = mem_burstcount_r;

localparam [2:0] S_RESET_INIT  = 3'd0;
localparam [2:0] S_IDLE        = 3'd1;
localparam [2:0] S_LOOKUP      = 3'd2;
localparam [2:0] S_FILL        = 3'd3;
localparam [2:0] S_BYPASS_WAIT = 3'd4;

reg [2:0] state;
reg [SET_BITS-1:0] init_set;
reg [TAG_BITS-1:0] snoop_tag_r;
reg [SET_BITS-1:0] snoop_set_r;
reg [WORD_OFFSET_BITS-1:0] snoop_word_r;
reg [29:0] snoop_addr_dw_r;
reg [31:0] snoop_data_r;
reg [3:0] snoop_be_r;
reg snoop_patch_r;
reg snoop_valid_r;
reg [29:0] patchq_addr [0:PATCHQ_DEPTH-1];
reg [31:0] patchq_data [0:PATCHQ_DEPTH-1];
reg  [3:0] patchq_be   [0:PATCHQ_DEPTH-1];
reg        patchq_valid[0:PATCHQ_DEPTH-1];
reg [PATCHQ_IDX_BITS-1:0] patchq_head;
reg [WORD_OFFSET_BITS-1:0] fill_count;
reg [WORD_OFFSET_BITS-1:0] fill_target_word;
reg [SET_BITS-1:0] fill_set;
reg [TAG_BITS-1:0] fill_tag;
reg [1:0] fill_way;
reg [127:0] fill_line;
reg [2:0] fill_plru_r;
reg fill_requested;
reg fill_valid0_r, fill_valid1_r, fill_valid2_r, fill_valid3_r;

reg [31:0] dout_r;
reg [127:0] line_r;
reg resp_valid_r;
reg ready_r;

assign cpu_ready = ready_r;

function automatic [1:0] way_encode(input [3:0] hit_vec);
begin
    way_encode = hit_vec[0] ? 2'd0 :
                 hit_vec[1] ? 2'd1 :
                 hit_vec[2] ? 2'd2 : 2'd3;
end
endfunction

function automatic [127:0] way_line_mux(
    input [1:0] way,
    input [127:0] data0,
    input [127:0] data1,
    input [127:0] data2,
    input [127:0] data3
);
begin
    case (way)
        2'd0: way_line_mux = data0;
        2'd1: way_line_mux = data1;
        2'd2: way_line_mux = data2;
        default: way_line_mux = data3;
    endcase
end
endfunction

function automatic [31:0] select_word(input [127:0] line, input [1:0] word);
begin
    select_word = line[{word, 5'b0} +: 32];
end
endfunction

function automatic [127:0] patch_line_word(input [127:0] line, input [1:0] word, input [31:0] data);
begin
    patch_line_word = line;
    patch_line_word[{word, 5'b0} +: 32] = data;
end
endfunction

function automatic [31:0] be_mask(input [3:0] be);
begin
    be_mask = {{8{be[3]}}, {8{be[2]}}, {8{be[1]}}, {8{be[0]}}};
end
endfunction

function automatic [31:0] merge32(input [31:0] old_data, input [31:0] new_data, input [3:0] be);
    automatic reg [31:0] mask;
begin
    mask = be_mask(be);
    merge32 = (old_data & ~mask) | (new_data & mask);
end
endfunction

function automatic [127:0] patch_line_word_be(
    input [127:0] line,
    input [1:0] word,
    input [31:0] data,
    input [3:0] be
);
begin
    patch_line_word_be = line;
    patch_line_word_be[{word, 5'b0} +: 32] =
        merge32(line[{word, 5'b0} +: 32], data, be);
end
endfunction

function automatic [PATCHQ_IDX_BITS-1:0] patchq_next_idx(input [PATCHQ_IDX_BITS-1:0] idx);
begin
    patchq_next_idx = (idx == PATCHQ_LAST_IDX) ? {PATCHQ_IDX_BITS{1'b0}} : (idx + 1'b1);
end
endfunction

function automatic logic line_match_dw(
    input [29:0] addr_dw,
    input [TAG_BITS-1:0] tag,
    input [SET_BITS-1:0] set
);
begin
    line_match_dw = (addr_dw[TAG_DW_MSB:TAG_DW_LSB] == tag) &&
                    (addr_dw[SET_DW_MSB:SET_DW_LSB] == set);
end
endfunction

function automatic logic word_match_dw(
    input [29:0] addr_dw,
    input [TAG_BITS-1:0] tag,
    input [SET_BITS-1:0] set,
    input [WORD_OFFSET_BITS-1:0] word
);
begin
    word_match_dw = line_match_dw(addr_dw, tag, set) &&
                    (addr_dw[WORD_OFFSET_BITS-1:0] == word);
end
endfunction

function automatic [2:0] plru_update(input [2:0] plru, input [1:0] way);
begin
    case (way)
        2'd0: plru_update = {plru[2], 1'b1, 1'b1};
        2'd1: plru_update = {plru[2], 1'b0, 1'b1};
        2'd2: plru_update = {1'b1, plru[1], 1'b0};
        default: plru_update = {1'b0, plru[1], 1'b0};
    endcase
end
endfunction

function automatic [1:0] plru_victim(input [2:0] plru);
begin
    if (!plru[0])
        plru_victim = plru[1] ? 2'd1 : 2'd0;
    else
        plru_victim = plru[2] ? 2'd3 : 2'd2;
end
endfunction

wire [3:0] lookup_hit_vec = {
    rd_valid3_r && (rd_tag3_r == req_tag_r),
    rd_valid2_r && (rd_tag2_r == req_tag_r),
    rd_valid1_r && (rd_tag1_r == req_tag_r),
    rd_valid0_r && (rd_tag0_r == req_tag_r)
};
wire lookup_hit = |lookup_hit_vec;
wire [1:0] lookup_way = way_encode(lookup_hit_vec);
wire [127:0] lookup_way_line = way_line_mux(lookup_way, rd_line0_r, rd_line1_r, rd_line2_r, rd_line3_r);
wire can_accept_cpu = (state == S_IDLE) && !reset;
wire accept_cpu = cpu_valid && ready_r && can_accept_cpu;
wire lookup_read_hit_now = (state == S_LOOKUP) && req_valid_r &&
                           !req_uncacheable_r && lookup_hit;
logic [PATCHQ_DEPTH-1:0] patchq_snoop_match;
logic patchq_snoop_hit;
logic [31:0] fill_word_next;
logic [127:0] fill_line_base;
logic [127:0] fill_line_next;

always_comb begin
    patchq_snoop_match = {PATCHQ_DEPTH{1'b0}};
    for (int p = 0; p < PATCHQ_DEPTH; p++)
        patchq_snoop_match[p] = patchq_valid[p] && patchq_addr[p] == snoop_addr_dw_r;
    patchq_snoop_hit = |patchq_snoop_match;
end

always_comb begin
    fill_word_next = mem_dout;
    for (int p = 0; p < PATCHQ_DEPTH; p++) begin
        if (patchq_valid[p] && word_match_dw(patchq_addr[p], fill_tag, fill_set, fill_count))
            fill_word_next = merge32(fill_word_next, patchq_data[p], patchq_be[p]);
    end
    if (snoop_valid_r && snoop_patch_r && word_match_dw(snoop_addr_dw_r, fill_tag, fill_set, fill_count))
        fill_word_next = merge32(fill_word_next, snoop_data_r, snoop_be_r);
    if (snoop_valid && snoop_patch && word_match_dw(snoop_addr[31:2], fill_tag, fill_set, fill_count))
        fill_word_next = merge32(fill_word_next, snoop_data, snoop_be);

    fill_line_base = fill_line;
    if (snoop_valid_r && snoop_patch_r && line_match_dw(snoop_addr_dw_r, fill_tag, fill_set))
        fill_line_base = patch_line_word_be(fill_line_base, snoop_word_r, snoop_data_r, snoop_be_r);
    if (snoop_valid && snoop_patch && line_match_dw(snoop_addr[31:2], fill_tag, fill_set))
        fill_line_base = patch_line_word_be(fill_line_base, snoop_word, snoop_data, snoop_be);
    fill_line_next = patch_line_word(fill_line_base, fill_count, fill_word_next);
end

assign cpu_dout = lookup_read_hit_now ? select_word(lookup_way_line, req_word_r) : dout_r;
assign cpu_line = lookup_read_hit_now ? lookup_way_line : line_r;
assign cpu_resp_valid = lookup_read_hit_now || resp_valid_r;

task automatic write_cache_line(input [1:0] way, input [SET_BITS-1:0] set, input [127:0] line);
begin
    case (way)
        2'd0: data_way0[set] <= line;
        2'd1: data_way1[set] <= line;
        2'd2: data_way2[set] <= line;
        default: data_way3[set] <= line;
    endcase
end
endtask

task automatic write_cache_tag(input [1:0] way, input [SET_BITS-1:0] set, input [TAG_BITS-1:0] tag);
begin
    case (way)
        2'd0: begin tag_way0[set] <= {{(TAG_RAM_BITS-TAG_BITS){1'b0}}, tag}; valid_way0[set] <= 1'b1; end
        2'd1: begin tag_way1[set] <= {{(TAG_RAM_BITS-TAG_BITS){1'b0}}, tag}; valid_way1[set] <= 1'b1; end
        2'd2: begin tag_way2[set] <= {{(TAG_RAM_BITS-TAG_BITS){1'b0}}, tag}; valid_way2[set] <= 1'b1; end
        default: begin tag_way3[set] <= {{(TAG_RAM_BITS-TAG_BITS){1'b0}}, tag}; valid_way3[set] <= 1'b1; end
    endcase
end
endtask

always_ff @(posedge clk) begin
    if (accept_cpu) begin
        rd_tag0_r <= tag_way0[cpu_set][TAG_BITS-1:0];
        rd_tag1_r <= tag_way1[cpu_set][TAG_BITS-1:0];
        rd_tag2_r <= tag_way2[cpu_set][TAG_BITS-1:0];
        rd_tag3_r <= tag_way3[cpu_set][TAG_BITS-1:0];
        rd_valid0_r <= valid_way0[cpu_set];
        rd_valid1_r <= valid_way1[cpu_set];
        rd_valid2_r <= valid_way2[cpu_set];
        rd_valid3_r <= valid_way3[cpu_set];
        rd_line0_r <= data_way0[cpu_set];
        rd_line1_r <= data_way1[cpu_set];
        rd_line2_r <= data_way2[cpu_set];
        rd_line3_r <= data_way3[cpu_set];
        rd_plru_r <= plru_set[cpu_set];
    end
end

always_ff @(posedge clk) begin
    if (reset) begin
        state <= S_RESET_INIT;
        init_set <= {SET_BITS{1'b0}};
        req_valid_r <= 1'b0;
        ready_r <= 1'b0;
        resp_valid_r <= 1'b0;
        dout_r <= 32'h0;
        line_r <= 128'h0;
        mem_valid_r <= 1'b0;
        mem_addr_r <= 32'h0;
        mem_burstcount_r <= 8'h0;
        fill_line <= 128'h0;
        fill_requested <= 1'b0;
        snoop_tag_r <= {TAG_BITS{1'b0}};
        snoop_set_r <= {SET_BITS{1'b0}};
        snoop_word_r <= {WORD_OFFSET_BITS{1'b0}};
        snoop_addr_dw_r <= 30'h0;
        snoop_data_r <= 32'h0;
        snoop_be_r <= 4'h0;
        snoop_patch_r <= 1'b0;
        snoop_valid_r <= 1'b0;
        patchq_head <= {PATCHQ_IDX_BITS{1'b0}};
        for (integer p = 0; p < PATCHQ_DEPTH; p = p + 1)
            patchq_valid[p] <= 1'b0;
    end else begin
        ready_r <= (state == S_IDLE);
        resp_valid_r <= 1'b0;
        snoop_valid_r <= snoop_valid;
        if (snoop_valid) begin
            snoop_tag_r <= snoop_tag;
            snoop_set_r <= snoop_set;
            snoop_word_r <= snoop_word;
            snoop_addr_dw_r <= snoop_addr[31:2];
            snoop_data_r <= snoop_data;
            snoop_be_r <= snoop_be;
            snoop_patch_r <= snoop_patch;
        end

        if (mem_valid_r && mem_ready)
            mem_valid_r <= 1'b0;

        if (snoop_valid_r) begin
            // CPU stores can race ahead of an instruction-cache line fill.
            // Keep the recent data-bearing snoops so a later fill of the same
            // physical line returns self-modified code after a branch flush.
            if (snoop_patch_r) begin
                for (int p = 0; p < PATCHQ_DEPTH; p++) begin
                    if (patchq_snoop_match[p]) begin
                        patchq_data[p] <= merge32(patchq_data[p], snoop_data_r, snoop_be_r);
                        patchq_be[p] <= patchq_be[p] | snoop_be_r;
                    end
                end
                if (!patchq_snoop_hit) begin
                    patchq_valid[patchq_head] <= 1'b1;
                    patchq_addr[patchq_head] <= snoop_addr_dw_r;
                    patchq_data[patchq_head] <= snoop_data_r;
                    patchq_be[patchq_head] <= snoop_be_r;
                    patchq_head <= patchq_next_idx(patchq_head);
                end
            end else begin
                for (int p = 0; p < PATCHQ_DEPTH; p++) begin
                    if (patchq_valid[p] && line_match_dw(patchq_addr[p], snoop_tag_r, snoop_set_r))
                        patchq_valid[p] <= 1'b0;
                end
            end

            if (valid_way0[snoop_set_r] && tag_way0[snoop_set_r][TAG_BITS-1:0] == snoop_tag_r) begin
                valid_way0[snoop_set_r] <= 1'b0;
            end
            if (valid_way1[snoop_set_r] && tag_way1[snoop_set_r][TAG_BITS-1:0] == snoop_tag_r) begin
                valid_way1[snoop_set_r] <= 1'b0;
            end
            if (valid_way2[snoop_set_r] && tag_way2[snoop_set_r][TAG_BITS-1:0] == snoop_tag_r) begin
                valid_way2[snoop_set_r] <= 1'b0;
            end
            if (valid_way3[snoop_set_r] && tag_way3[snoop_set_r][TAG_BITS-1:0] == snoop_tag_r) begin
                valid_way3[snoop_set_r] <= 1'b0;
            end
        end

        case (state)
            S_RESET_INIT: begin
                valid_way0[init_set] <= 1'b0;
                valid_way1[init_set] <= 1'b0;
                valid_way2[init_set] <= 1'b0;
                valid_way3[init_set] <= 1'b0;
                plru_set[init_set] <= 3'b000;
                if (init_set == LAST_SET) begin
                    state <= S_IDLE;
                    ready_r <= 1'b1;
                end else begin
                    init_set <= init_set + 1'b1;
                end
            end

            S_IDLE: begin
                if (accept_cpu) begin
                    ready_r <= 1'b0;
                    req_valid_r <= 1'b1;
                    req_addr_r <= cpu_addr;
                    req_uncacheable_r <= cpu_uncacheable;
                    req_tag_r <= cpu_tag;
                    req_set_r <= cpu_set;
                    req_word_r <= cpu_word;
                    state <= S_LOOKUP;
                end
            end

            S_LOOKUP: begin
                req_valid_r <= 1'b0;

                if (req_uncacheable_r) begin
                    if (!mem_valid_r && !mem_busy) begin
                        mem_valid_r <= 1'b1;
                        mem_addr_r <= req_addr_r;
                        mem_burstcount_r <= 8'd1;
                        state <= S_BYPASS_WAIT;
                    end
                end else if (lookup_hit) begin
                    plru_set[req_set_r] <= plru_update(rd_plru_r, lookup_way);
                    state <= S_IDLE;
                    ready_r <= 1'b1;
                end else begin
                    fill_set <= req_set_r;
                    fill_tag <= req_tag_r;
                    fill_way <= plru_victim(rd_plru_r);
                    fill_plru_r <= rd_plru_r;
                    fill_valid0_r <= rd_valid0_r;
                    fill_valid1_r <= rd_valid1_r;
                    fill_valid2_r <= rd_valid2_r;
                    fill_valid3_r <= rd_valid3_r;
                    fill_count <= {WORD_OFFSET_BITS{1'b0}};
                    fill_target_word <= req_word_r;
                    fill_line <= 128'h0;
                    fill_requested <= 1'b0;
                    state <= S_FILL;
                end
            end

            S_FILL: begin
                if (!fill_requested && !mem_valid_r && !mem_busy) begin
                    mem_valid_r <= 1'b1;
                    mem_addr_r <= {req_addr_r[31:4], 4'b0000};
                    mem_burstcount_r <= 8'd4;
                    fill_requested <= 1'b1;
                end

                if (mem_resp_valid) begin
                    fill_line <= fill_line_next;

                    if (fill_count == {WORD_OFFSET_BITS{1'b1}}) begin
                        write_cache_line(fill_way, fill_set, fill_line_next);
                        write_cache_tag(fill_way, fill_set, fill_tag);
                        dout_r <= select_word(fill_line_next, fill_target_word);
                        line_r <= fill_line_next;
                        resp_valid_r <= 1'b1;
                        case (fill_way)
                            2'd0: begin valid_way1[fill_set] <= fill_valid1_r; valid_way2[fill_set] <= fill_valid2_r; valid_way3[fill_set] <= fill_valid3_r; end
                            2'd1: begin valid_way0[fill_set] <= fill_valid0_r; valid_way2[fill_set] <= fill_valid2_r; valid_way3[fill_set] <= fill_valid3_r; end
                            2'd2: begin valid_way0[fill_set] <= fill_valid0_r; valid_way1[fill_set] <= fill_valid1_r; valid_way3[fill_set] <= fill_valid3_r; end
                            default: begin valid_way0[fill_set] <= fill_valid0_r; valid_way1[fill_set] <= fill_valid1_r; valid_way2[fill_set] <= fill_valid2_r; end
                        endcase
                        plru_set[fill_set] <= plru_update(fill_plru_r, fill_way);
                        state <= S_IDLE;
                        ready_r <= 1'b1;
                    end
                    fill_count <= fill_count + 1'b1;
                end
            end

            S_BYPASS_WAIT: begin
                if (mem_resp_valid) begin
                    dout_r <= mem_dout;
                    line_r <= {4{mem_dout}};
                    resp_valid_r <= 1'b1;
                    state <= S_IDLE;
                    ready_r <= 1'b1;
                end
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule
