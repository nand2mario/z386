// Physically indexed, physically tagged L1 cache for z386 0.3.
//
// CPU-side contract:
//   * cpu_addr is a physical byte address.
//   * A cache-hit read accepted in cycle N returns cpu_resp_valid in N+1.
//   * A write accepted in cycle N is posted to the write-through store queue.
//   * Miss/refill and uncached accesses use the memory-side burst interface.
//
// This deliberately avoids the old VIPT preread/finalize split.  The paging
// unit owns translation and only sends physical requests to this module.
module l1_cache #(
    // Four ways, 16 bytes per line. SET_BITS=8 gives a 16KB data cache.
    parameter integer SET_BITS = 8,
    parameter PROTECT_UMA_ROM = 0
) (
    input         clk,
    input         reset,

    // CPU side — physical address request/response.
    input  [31:0] cpu_addr,
    input  [31:0] cpu_din,
    output [31:0] cpu_dout,
    input   [3:0] cpu_be,
    input         cpu_valid,
    input         cpu_write,
    output        cpu_ready,
    output        cpu_resp_valid,

    // Memory side.
    output [31:0] mem_addr,
    output [31:0] mem_din,
    input  [31:0] mem_dout,
    output  [3:0] mem_be,
    output  [7:0] mem_burstcount,
    input         mem_busy,
    output        mem_valid,
    output        mem_write,
    input         mem_ready,
    input         mem_resp_valid,

    // Physical-address snoop.  The first implementation invalidates a whole
    // set; this is conservative and keeps snoop matching off the read hit path.
    input  [31:0] snoop_addr,
    input         snoop_valid,

    input         cache_enable
);

localparam integer WORD_OFFSET_BITS = 2;
localparam integer BYTE_OFFSET_BITS = 2;
localparam integer LINE_OFFSET_BITS = WORD_OFFSET_BITS + BYTE_OFFSET_BITS;
localparam integer NUM_SETS = 1 << SET_BITS;
localparam integer BRAM_ADDR_BITS = SET_BITS + WORD_OFFSET_BITS;
localparam integer TAG_BITS = 25 - LINE_OFFSET_BITS - SET_BITS;
localparam integer SET_LSB = LINE_OFFSET_BITS;
localparam integer SET_MSB = SET_LSB + SET_BITS - 1;
localparam integer TAG_LSB = SET_MSB + 1;
localparam integer TAG_MSB = 24;
localparam integer TAG_RAM_BITS = (TAG_BITS < 16) ? 16 : TAG_BITS;
localparam integer STOREQ_DEPTH = 3;
localparam integer STOREQ_IDX_BITS = 2;
localparam integer STOREQ_CNT_BITS = 2;
localparam [STOREQ_CNT_BITS-1:0] STOREQ_DEPTH_VALUE = 2'd3;
localparam [STOREQ_IDX_BITS-1:0] STOREQ_LAST_IDX = 2'd2;
localparam [SET_BITS-1:0] LAST_SET = SET_BITS'(NUM_SETS - 1);

// Address decomposition.  The cache covers the low 32MB physical window.
wire [TAG_BITS-1:0] cpu_tag = cpu_addr[TAG_MSB:TAG_LSB];
wire [SET_BITS-1:0] cpu_set = cpu_addr[SET_MSB:SET_LSB];
wire [WORD_OFFSET_BITS-1:0] cpu_word = cpu_addr[LINE_OFFSET_BITS-1:BYTE_OFFSET_BITS];
wire [BRAM_ADDR_BITS-1:0] cpu_bram_addr = {cpu_set, cpu_word};
wire [SET_BITS-1:0] snoop_set = snoop_addr[SET_MSB:SET_LSB];
wire cpu_write_enabled = cpu_write;
wire cpu_uncacheable = !cache_enable || (cpu_addr[31:17] == 15'h5);
wire cpu_protect_write = PROTECT_UMA_ROM && cpu_write_enabled && (cpu_addr[24:18] == 7'b000_0011);

// Tag/data storage.
(* ram_style = "block" *) reg [TAG_RAM_BITS-1:0] tag_way0 [0:NUM_SETS-1] /* synthesis syn_ramstyle="block_ram" */;
(* ram_style = "block" *) reg [TAG_RAM_BITS-1:0] tag_way1 [0:NUM_SETS-1] /* synthesis syn_ramstyle="block_ram" */;
(* ram_style = "block" *) reg [TAG_RAM_BITS-1:0] tag_way2 [0:NUM_SETS-1] /* synthesis syn_ramstyle="block_ram" */;
(* ram_style = "block" *) reg [TAG_RAM_BITS-1:0] tag_way3 [0:NUM_SETS-1] /* synthesis syn_ramstyle="block_ram" */;
reg valid_way0 [0:NUM_SETS-1];
reg valid_way1 [0:NUM_SETS-1];
reg valid_way2 [0:NUM_SETS-1];
reg valid_way3 [0:NUM_SETS-1];
reg [2:0] plru_set [0:NUM_SETS-1];

reg [31:0] data_way0 [0:(NUM_SETS << WORD_OFFSET_BITS)-1];
reg [31:0] data_way1 [0:(NUM_SETS << WORD_OFFSET_BITS)-1];
reg [31:0] data_way2 [0:(NUM_SETS << WORD_OFFSET_BITS)-1];
reg [31:0] data_way3 [0:(NUM_SETS << WORD_OFFSET_BITS)-1];

// Synchronous cache read result for the request accepted in the previous cycle.
reg [TAG_BITS-1:0] rd_tag0_r, rd_tag1_r, rd_tag2_r, rd_tag3_r;
reg rd_valid0_r, rd_valid1_r, rd_valid2_r, rd_valid3_r;
reg [31:0] rd_data0_r, rd_data1_r, rd_data2_r, rd_data3_r;
reg [2:0] rd_plru_r;

// Accepted request register.
reg        req_valid_r;
reg [31:0] req_addr_r;
reg [31:0] req_din_r;
reg  [3:0] req_be_r;
reg        req_write_r;
reg        req_uncacheable_r;
reg        req_protect_write_r;
reg [TAG_BITS-1:0] req_tag_r;
reg [SET_BITS-1:0] req_set_r;
reg [WORD_OFFSET_BITS-1:0] req_word_r;

// Write-through store queue.
reg [29:0] storeq_addr [0:STOREQ_DEPTH-1];
reg [31:0] storeq_data [0:STOREQ_DEPTH-1];
reg  [3:0] storeq_be   [0:STOREQ_DEPTH-1];
reg        storeq_valid[0:STOREQ_DEPTH-1];
reg [STOREQ_IDX_BITS-1:0] storeq_head;
reg [STOREQ_IDX_BITS-1:0] storeq_tail;
reg [STOREQ_CNT_BITS-1:0] storeq_count;
reg        storeq_draining;

wire storeq_full = (storeq_count == STOREQ_DEPTH_VALUE);
wire storeq_empty = (storeq_count == {STOREQ_CNT_BITS{1'b0}});
wire storeq_can_accept = !storeq_full || (storeq_draining && mem_ready);

// Memory-side registers.
reg        mem_valid_r;
reg        mem_write_r;
reg [31:0] mem_addr_r;
reg [31:0] mem_din_r;
reg  [3:0] mem_be_r;
reg  [7:0] mem_burstcount_r;

assign mem_valid = mem_valid_r;
assign mem_write = mem_write_r;
assign mem_addr = mem_addr_r;
assign mem_din = mem_din_r;
assign mem_be = mem_be_r;
assign mem_burstcount = mem_burstcount_r;

// Cache FSM.
localparam [2:0] S_RESET_INIT  = 3'd0;
localparam [2:0] S_IDLE        = 3'd1;
localparam [2:0] S_LOOKUP      = 3'd2;
localparam [2:0] S_FILL        = 3'd3;
localparam [2:0] S_BYPASS_WAIT = 3'd4;

reg [2:0] state;
reg [SET_BITS-1:0] init_set;
reg [SET_BITS-1:0] snoop_set_r;
reg snoop_valid_r;
reg [WORD_OFFSET_BITS-1:0] fill_count;
reg [WORD_OFFSET_BITS-1:0] fill_target_word;
reg [SET_BITS-1:0] fill_set;
reg [TAG_BITS-1:0] fill_tag;
reg [1:0] fill_way;
reg [2:0] fill_plru_r;
reg fill_requested;
reg fill_target_returned;
reg fill_valid0_r, fill_valid1_r, fill_valid2_r, fill_valid3_r;

reg [31:0] dout_r;
reg resp_valid_r;
reg ready_r;

assign cpu_ready = ready_r;

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

function automatic [31:0] forward_storeq_slot(
    input [31:0] value,
    input        slot_live,
    input [29:0] slot_addr,
    input [31:0] slot_data,
    input  [3:0] slot_be,
    input [29:0] addr_dw
);
begin
    forward_storeq_slot = (slot_live && slot_addr == addr_dw) ?
                          merge32(value, slot_data, slot_be) : value;
end
endfunction

function automatic [STOREQ_IDX_BITS-1:0] storeq_next_idx(input [STOREQ_IDX_BITS-1:0] idx);
begin
    storeq_next_idx = (idx == STOREQ_LAST_IDX) ? {STOREQ_IDX_BITS{1'b0}} : (idx + 1'b1);
end
endfunction

function automatic [1:0] way_encode(input [3:0] hit_vec);
begin
    way_encode = hit_vec[0] ? 2'd0 :
                 hit_vec[1] ? 2'd1 :
                 hit_vec[2] ? 2'd2 : 2'd3;
end
endfunction

function automatic [31:0] way_data_mux(
    input [1:0] way,
    input [31:0] data0,
    input [31:0] data1,
    input [31:0] data2,
    input [31:0] data3
);
begin
    case (way)
        2'd0: way_data_mux = data0;
        2'd1: way_data_mux = data1;
        2'd2: way_data_mux = data2;
        default: way_data_mux = data3;
    endcase
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
wire [31:0] lookup_way_data = way_data_mux(lookup_way, rd_data0_r, rd_data1_r, rd_data2_r, rd_data3_r);
wire [BRAM_ADDR_BITS-1:0] req_bram_addr = {req_set_r, req_word_r};
wire can_accept_cpu = (state == S_IDLE) && !reset && (!cpu_write_enabled || cpu_protect_write || storeq_can_accept);
wire ready_when_idle = !reset && storeq_can_accept;
wire accept_cpu = cpu_valid && ready_r && can_accept_cpu;
wire accepted_store_enqueue = accept_cpu && cpu_write_enabled && !cpu_protect_write;
wire [29:0] req_addr_dw = req_addr_r[31:2];
wire [29:0] fill_addr_dw = {req_addr_r[31:4], fill_count};
logic [31:0] lookup_forward_data;
logic [31:0] fill_word_data;
logic [31:0] bypass_forward_data;
wire lookup_read_hit_now = (state == S_LOOKUP) && req_valid_r &&
                           !req_write_r && !req_uncacheable_r && lookup_hit;

assign cpu_dout = lookup_read_hit_now ? lookup_forward_data : dout_r;
assign cpu_resp_valid = lookup_read_hit_now || resp_valid_r;

always_comb begin
    lookup_forward_data = lookup_way_data;
    fill_word_data = mem_dout;
    bypass_forward_data = mem_dout;

    unique case (storeq_tail)
        2'd0: begin
            if (storeq_count > 0) begin
                lookup_forward_data = forward_storeq_slot(lookup_forward_data, storeq_valid[0], storeq_addr[0], storeq_data[0], storeq_be[0], req_addr_dw);
                fill_word_data = forward_storeq_slot(fill_word_data, storeq_valid[0], storeq_addr[0], storeq_data[0], storeq_be[0], fill_addr_dw);
                bypass_forward_data = forward_storeq_slot(bypass_forward_data, storeq_valid[0], storeq_addr[0], storeq_data[0], storeq_be[0], req_addr_dw);
            end
            if (storeq_count > 1) begin
                lookup_forward_data = forward_storeq_slot(lookup_forward_data, storeq_valid[1], storeq_addr[1], storeq_data[1], storeq_be[1], req_addr_dw);
                fill_word_data = forward_storeq_slot(fill_word_data, storeq_valid[1], storeq_addr[1], storeq_data[1], storeq_be[1], fill_addr_dw);
                bypass_forward_data = forward_storeq_slot(bypass_forward_data, storeq_valid[1], storeq_addr[1], storeq_data[1], storeq_be[1], req_addr_dw);
            end
            if (storeq_count > 2) begin
                lookup_forward_data = forward_storeq_slot(lookup_forward_data, storeq_valid[2], storeq_addr[2], storeq_data[2], storeq_be[2], req_addr_dw);
                fill_word_data = forward_storeq_slot(fill_word_data, storeq_valid[2], storeq_addr[2], storeq_data[2], storeq_be[2], fill_addr_dw);
                bypass_forward_data = forward_storeq_slot(bypass_forward_data, storeq_valid[2], storeq_addr[2], storeq_data[2], storeq_be[2], req_addr_dw);
            end
        end
        2'd1: begin
            if (storeq_count > 0) begin
                lookup_forward_data = forward_storeq_slot(lookup_forward_data, storeq_valid[1], storeq_addr[1], storeq_data[1], storeq_be[1], req_addr_dw);
                fill_word_data = forward_storeq_slot(fill_word_data, storeq_valid[1], storeq_addr[1], storeq_data[1], storeq_be[1], fill_addr_dw);
                bypass_forward_data = forward_storeq_slot(bypass_forward_data, storeq_valid[1], storeq_addr[1], storeq_data[1], storeq_be[1], req_addr_dw);
            end
            if (storeq_count > 1) begin
                lookup_forward_data = forward_storeq_slot(lookup_forward_data, storeq_valid[2], storeq_addr[2], storeq_data[2], storeq_be[2], req_addr_dw);
                fill_word_data = forward_storeq_slot(fill_word_data, storeq_valid[2], storeq_addr[2], storeq_data[2], storeq_be[2], fill_addr_dw);
                bypass_forward_data = forward_storeq_slot(bypass_forward_data, storeq_valid[2], storeq_addr[2], storeq_data[2], storeq_be[2], req_addr_dw);
            end
            if (storeq_count > 2) begin
                lookup_forward_data = forward_storeq_slot(lookup_forward_data, storeq_valid[0], storeq_addr[0], storeq_data[0], storeq_be[0], req_addr_dw);
                fill_word_data = forward_storeq_slot(fill_word_data, storeq_valid[0], storeq_addr[0], storeq_data[0], storeq_be[0], fill_addr_dw);
                bypass_forward_data = forward_storeq_slot(bypass_forward_data, storeq_valid[0], storeq_addr[0], storeq_data[0], storeq_be[0], req_addr_dw);
            end
        end
        default: begin
            if (storeq_count > 0) begin
                lookup_forward_data = forward_storeq_slot(lookup_forward_data, storeq_valid[2], storeq_addr[2], storeq_data[2], storeq_be[2], req_addr_dw);
                fill_word_data = forward_storeq_slot(fill_word_data, storeq_valid[2], storeq_addr[2], storeq_data[2], storeq_be[2], fill_addr_dw);
                bypass_forward_data = forward_storeq_slot(bypass_forward_data, storeq_valid[2], storeq_addr[2], storeq_data[2], storeq_be[2], req_addr_dw);
            end
            if (storeq_count > 1) begin
                lookup_forward_data = forward_storeq_slot(lookup_forward_data, storeq_valid[0], storeq_addr[0], storeq_data[0], storeq_be[0], req_addr_dw);
                fill_word_data = forward_storeq_slot(fill_word_data, storeq_valid[0], storeq_addr[0], storeq_data[0], storeq_be[0], fill_addr_dw);
                bypass_forward_data = forward_storeq_slot(bypass_forward_data, storeq_valid[0], storeq_addr[0], storeq_data[0], storeq_be[0], req_addr_dw);
            end
            if (storeq_count > 2) begin
                lookup_forward_data = forward_storeq_slot(lookup_forward_data, storeq_valid[1], storeq_addr[1], storeq_data[1], storeq_be[1], req_addr_dw);
                fill_word_data = forward_storeq_slot(fill_word_data, storeq_valid[1], storeq_addr[1], storeq_data[1], storeq_be[1], fill_addr_dw);
                bypass_forward_data = forward_storeq_slot(bypass_forward_data, storeq_valid[1], storeq_addr[1], storeq_data[1], storeq_be[1], req_addr_dw);
            end
        end
    endcase
end

task automatic write_cache_word(input [1:0] way, input [BRAM_ADDR_BITS-1:0] addr, input [31:0] data);
begin
    case (way)
        2'd0: data_way0[addr] <= data;
        2'd1: data_way1[addr] <= data;
        2'd2: data_way2[addr] <= data;
        default: data_way3[addr] <= data;
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
        rd_data0_r <= data_way0[cpu_bram_addr];
        rd_data1_r <= data_way1[cpu_bram_addr];
        rd_data2_r <= data_way2[cpu_bram_addr];
        rd_data3_r <= data_way3[cpu_bram_addr];
        rd_plru_r <= plru_set[cpu_set];
    end
end

always_ff @(posedge clk) begin
    automatic reg [31:0] patched;

    if (reset) begin
        state <= S_RESET_INIT;
        init_set <= {SET_BITS{1'b0}};
        req_valid_r <= 1'b0;
        ready_r <= 1'b0;
        resp_valid_r <= 1'b0;
        dout_r <= 32'h0;
        mem_valid_r <= 1'b0;
        mem_write_r <= 1'b0;
        mem_addr_r <= 32'h0;
        mem_din_r <= 32'h0;
        mem_be_r <= 4'h0;
        mem_burstcount_r <= 8'h0;
        storeq_head <= {STOREQ_IDX_BITS{1'b0}};
        storeq_tail <= {STOREQ_IDX_BITS{1'b0}};
        storeq_count <= {STOREQ_CNT_BITS{1'b0}};
        storeq_draining <= 1'b0;
        fill_requested <= 1'b0;
        fill_target_returned <= 1'b0;
        snoop_set_r <= {SET_BITS{1'b0}};
        snoop_valid_r <= 1'b0;
        for (integer i = 0; i < STOREQ_DEPTH; i = i + 1)
            storeq_valid[i] <= 1'b0;
    end else begin
        ready_r <= (state == S_IDLE) && ready_when_idle;
        resp_valid_r <= 1'b0;
        snoop_valid_r <= snoop_valid;
        if (snoop_valid)
            snoop_set_r <= snoop_set;

        if (mem_valid_r && mem_ready)
            mem_valid_r <= 1'b0;

        if (storeq_draining && mem_ready) begin
            storeq_valid[storeq_tail] <= 1'b0;
            storeq_tail <= storeq_next_idx(storeq_tail);
            storeq_count <= storeq_count - 1'b1;
            storeq_draining <= 1'b0;
        end

        if (snoop_valid_r) begin
            valid_way0[snoop_set_r] <= 1'b0;
            valid_way1[snoop_set_r] <= 1'b0;
            valid_way2[snoop_set_r] <= 1'b0;
            valid_way3[snoop_set_r] <= 1'b0;
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
                    ready_r <= ready_when_idle;
                end else begin
                    init_set <= init_set + 1'b1;
                end
            end

            S_IDLE: begin
                if (accept_cpu) begin
                    ready_r <= 1'b0;
                    req_valid_r <= 1'b1;
                    req_addr_r <= cpu_addr;
                    req_din_r <= cpu_din;
                    req_be_r <= cpu_be;
                    req_write_r <= cpu_write_enabled;
                    req_uncacheable_r <= cpu_uncacheable;
                    req_protect_write_r <= cpu_protect_write;
                    req_tag_r <= cpu_tag;
                    req_set_r <= cpu_set;
                    req_word_r <= cpu_word;
                    if (accepted_store_enqueue) begin
                        storeq_addr[storeq_head] <= cpu_addr[31:2];
                        storeq_data[storeq_head] <= cpu_din;
                        storeq_be[storeq_head] <= cpu_be;
                        storeq_valid[storeq_head] <= 1'b1;
                        storeq_head <= storeq_next_idx(storeq_head);
                        storeq_count <= (storeq_draining && mem_ready) ?
                                        storeq_count : storeq_count + 1'b1;
                    end
                    state <= S_LOOKUP;
                end else if (!storeq_empty && !storeq_draining && !mem_valid_r && !mem_busy) begin
                    mem_valid_r <= 1'b1;
                    mem_write_r <= 1'b1;
                    mem_addr_r <= {storeq_addr[storeq_tail], 2'b00};
                    mem_din_r <= storeq_data[storeq_tail];
                    mem_be_r <= storeq_be[storeq_tail];
                    mem_burstcount_r <= 8'd1;
                    storeq_draining <= 1'b1;
                end
            end

            S_LOOKUP: begin
                req_valid_r <= 1'b0;

                if (req_protect_write_r) begin
                    state <= S_IDLE;
                    ready_r <= ready_when_idle;
                end else if (req_write_r) begin
                    if (lookup_hit && !req_uncacheable_r) begin
                        patched = merge32(lookup_way_data, req_din_r, req_be_r);
                        write_cache_word(lookup_way, req_bram_addr, patched);
                        plru_set[req_set_r] <= plru_update(rd_plru_r, lookup_way);
                    end
                    state <= S_IDLE;
                    ready_r <= ready_when_idle;
                end else if (req_uncacheable_r) begin
                    if (!mem_valid_r && !mem_busy) begin
                        mem_valid_r <= 1'b1;
                        mem_write_r <= 1'b0;
                        mem_addr_r <= req_addr_r;
                        mem_din_r <= 32'h0;
                        mem_be_r <= req_be_r;
                        mem_burstcount_r <= 8'd1;
                        state <= S_BYPASS_WAIT;
                    end
                end else if (lookup_hit) begin
                    plru_set[req_set_r] <= plru_update(rd_plru_r, lookup_way);
                    state <= S_IDLE;
                    ready_r <= ready_when_idle;
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
                    fill_requested <= 1'b0;
                    fill_target_returned <= 1'b0;
                    state <= S_FILL;
                end
            end

            S_FILL: begin
                if (!fill_requested && !mem_valid_r && !mem_busy) begin
                    mem_valid_r <= 1'b1;
                    mem_write_r <= 1'b0;
                    mem_addr_r <= {req_addr_r[31:4], 4'b0000};
                    mem_din_r <= 32'h0;
                    mem_be_r <= 4'hF;
                    mem_burstcount_r <= 8'd4;
                    fill_requested <= 1'b1;
                end

                if (mem_resp_valid) begin
                    write_cache_word(fill_way, {fill_set, fill_count}, fill_word_data);

                    if (fill_count == fill_target_word && !fill_target_returned) begin
                        dout_r <= fill_word_data;
                        resp_valid_r <= 1'b1;
                        fill_target_returned <= 1'b1;
                    end

                    if (fill_count == {WORD_OFFSET_BITS{1'b1}}) begin
                        write_cache_tag(fill_way, fill_set, fill_tag);
                        case (fill_way)
                            2'd0: begin valid_way1[fill_set] <= fill_valid1_r; valid_way2[fill_set] <= fill_valid2_r; valid_way3[fill_set] <= fill_valid3_r; end
                            2'd1: begin valid_way0[fill_set] <= fill_valid0_r; valid_way2[fill_set] <= fill_valid2_r; valid_way3[fill_set] <= fill_valid3_r; end
                            2'd2: begin valid_way0[fill_set] <= fill_valid0_r; valid_way1[fill_set] <= fill_valid1_r; valid_way3[fill_set] <= fill_valid3_r; end
                            default: begin valid_way0[fill_set] <= fill_valid0_r; valid_way1[fill_set] <= fill_valid1_r; valid_way2[fill_set] <= fill_valid2_r; end
                        endcase
                        plru_set[fill_set] <= plru_update(fill_plru_r, fill_way);
                        state <= S_IDLE;
                        ready_r <= ready_when_idle;
                    end
                    fill_count <= fill_count + 1'b1;
                end
            end

            S_BYPASS_WAIT: begin
                if (mem_resp_valid) begin
                    dout_r <= bypass_forward_data;
                    resp_valid_r <= 1'b1;
                    state <= S_IDLE;
                    ready_r <= ready_when_idle;
                end
            end

            default: state <= S_IDLE;
        endcase
    end
end

// synthesis translate_off
always_ff @(posedge clk) begin
    if (!reset && state != S_RESET_INIT && cpu_valid && !cpu_ready && !(state == S_IDLE))
        ;
end
// synthesis translate_on

endmodule
