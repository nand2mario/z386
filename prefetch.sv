//
// Prefetch Unit - 32-byte circular buffer, filled one 16-byte cache line at a time
//
// The decoder-facing byte window is registered.  q_window_next is computed
// from the queue's NEXT state (this cycle's pop, flush and fill included via
// bypass), so the registered window always equals the byte rotate of the new
// queue head: the decoder sees exactly the same value per cycle as the old
// combinational window, but the 8:1 word mux + byte rotate is paid in the
// cycle before decode instead of on the decode critical path.
//

module prefetch
    import z386_pkg::*;
(
    input             clk,
    input             reset_n,

    // Queue output to decoder
    output     [31:0] q_window,      // 4-byte window at current queue head
    output            q_full,
    output            q_empty,
    output     [5:0]  pf_count,
    input      [2:0]  q_pop_bytes,

    // Flush from microcode
    input             q_flush,
    input      [31:0] pf_flush_addr, // LINEAR address

    // Toggle interface to paging unit
    output reg        pf_req_toggle,
    output reg [31:0] pf_linear_addr,
    output reg        pf_redirect_queued,
    input             pf_ack_toggle,
    input      [127:0] pf_rdata,
    input             pf_fault,      // page fault on this fetch (silently drop)

    // Control
    input             pf_suspend     // external suspend (e.g. page fault handler active)
);

// 32-byte prefetch queue (8 x 32-bit words).  Cache fills write up to four
// queue words at once.  After a branch into the middle of a cache line, the
// first fill drops words before the target address and starts decoding at the
// requested byte offset.
reg [31:0] prefetch_queue [7:0];
reg [3:0]  pf_rptr;                  // Read pointer (0-7) with wraparound bit
reg [3:0]  pf_wptr;                  // Write pointer (0-7) with wraparound bit
reg [1:0]  pf_byte_offset;           // Byte offset within current dword (0-3)
reg [1:0]  pf_fetch_word_start;      // First word to keep from next fetched line
reg        pf_suspended;             // Prefetch suspended (page fault until flush)
reg        pf_drop_inflight;         // Drop next prefetch result (flush during in-flight)
reg [31:0] pf_fetch_addr;            // Next LINEAR cache-line address to prefetch
reg [31:0] q_window_r;               // Registered head window seen by the decoder

// synthesis translate_off
bit TRACE_FLUSH_EN;
initial TRACE_FLUSH_EN = $test$plusargs("trace_flush");
// synthesis translate_on

function automatic [2:0] ptr_idx(input [3:0] ptr);
    begin
        ptr_idx = ptr[2:0];
    end
endfunction

assign q_window = q_window_r;

wire [3:0] pf_word_count = pf_wptr - pf_rptr; // 0..8 valid queue words
wire [5:0] pf_byte_count = q_empty ? 6'd0 :
                           ({2'b00, pf_word_count} << 2) - {4'b0000, pf_byte_offset};
assign pf_count = pf_byte_count;
assign q_empty = (pf_word_count == 4'd0);
assign q_full = (pf_word_count == 4'd8);

wire pf_inflight = (pf_req_toggle != pf_ack_toggle);

reg pf_ack_prev;
wire pf_ack_edge = (pf_ack_toggle != pf_ack_prev);

wire [2:0] fetch_write_words = 3'd4 - {1'b0, pf_fetch_word_start};
wire good_ack = pf_ack_edge && !pf_drop_inflight && !pf_fault;
wire [4:0] pf_words_after_ack =
    {1'b0, pf_word_count} + (good_ack ? {2'b00, fetch_write_words} : 5'd0);
wire pf_has_line_space = (pf_words_after_ack <= 5'd4);

wire pf_can_fetch = pf_has_line_space && !pf_suspended && !pf_suspend &&
                    !q_flush && !pf_inflight;
wire pf_can_fetch_after_flush = q_flush && !pf_suspend && !pf_inflight;

function automatic [31:0] line_word(input [127:0] line, input [1:0] word);
    begin
        line_word = line[{word, 5'b0} +: 32];
    end
endfunction

// Next-state of the queue head, mirroring the update priority of the
// registered always_ff below: pop, then fill, then flush, then the
// unaligned-seed case at fetch launch.
wire fill_commit = good_ack && !q_flush;
wire seed_now = pf_can_fetch && !good_ack && q_empty &&
                (pf_fetch_addr[3:0] != 4'h0);
wire [2:0] byte_advance = {1'b0, pf_byte_offset} + q_pop_bytes;

logic [3:0]  rptr_next;
logic [3:0]  wptr_next;
logic [1:0]  byte_offset_next;
logic [31:0] queue_next [7:0];

always_comb begin
    rptr_next = pf_rptr;
    wptr_next = pf_wptr;
    byte_offset_next = pf_byte_offset;
    for (int k = 0; k < 8; k++)
        queue_next[k] = prefetch_queue[k];

    if ((q_pop_bytes != 3'd0) && !q_empty) begin
        byte_offset_next = byte_advance[1:0];
        rptr_next = pf_rptr + {3'd0, byte_advance[2]};
    end

    if (fill_commit) begin
        unique case (pf_fetch_word_start)
            2'd0: begin
                queue_next[ptr_idx(pf_wptr)] = line_word(pf_rdata, 2'd0);
                queue_next[ptr_idx(pf_wptr + 4'd1)] = line_word(pf_rdata, 2'd1);
                queue_next[ptr_idx(pf_wptr + 4'd2)] = line_word(pf_rdata, 2'd2);
                queue_next[ptr_idx(pf_wptr + 4'd3)] = line_word(pf_rdata, 2'd3);
            end
            2'd1: begin
                queue_next[ptr_idx(pf_wptr)] = line_word(pf_rdata, 2'd1);
                queue_next[ptr_idx(pf_wptr + 4'd1)] = line_word(pf_rdata, 2'd2);
                queue_next[ptr_idx(pf_wptr + 4'd2)] = line_word(pf_rdata, 2'd3);
            end
            2'd2: begin
                queue_next[ptr_idx(pf_wptr)] = line_word(pf_rdata, 2'd2);
                queue_next[ptr_idx(pf_wptr + 4'd1)] = line_word(pf_rdata, 2'd3);
            end
            default: begin
                queue_next[ptr_idx(pf_wptr)] = line_word(pf_rdata, 2'd3);
            end
        endcase
        wptr_next = pf_wptr + {1'b0, fetch_write_words};
    end

    if (q_flush) begin
        rptr_next = 4'h0;
        wptr_next = 4'h0;
        byte_offset_next = pf_flush_addr[1:0];
    end

    if (seed_now)
        byte_offset_next = pf_fetch_addr[1:0];
end

wire [31:0] q_word_cur_next = queue_next[ptr_idx(rptr_next)];
wire [31:0] q_word_nxt_next = queue_next[ptr_idx(rptr_next + 4'd1)];

wire [31:0] q_window_next =
    byte_offset_next == 2'd0 ? q_word_cur_next :
    byte_offset_next == 2'd1 ? {q_word_nxt_next[7:0],  q_word_cur_next[31:8]} :
    byte_offset_next == 2'd2 ? {q_word_nxt_next[15:0], q_word_cur_next[31:16]} :
                               {q_word_nxt_next[23:0], q_word_cur_next[31:24]};

always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        pf_rptr <= 4'h0;
        pf_wptr <= 4'h0;
        pf_byte_offset <= 2'h0;
        pf_fetch_word_start <= 2'h0;
        pf_suspended <= 1'b0;
        pf_drop_inflight <= 1'b0;
        pf_fetch_addr <= 32'hFFFF_FFF0;  // Reset vector, cache-line aligned
        pf_req_toggle <= 1'b0;
        pf_linear_addr <= 32'h0;
        pf_redirect_queued <= 1'b0;
        pf_ack_prev <= 1'b0;
        q_window_r <= 32'h0;
    end else begin
        pf_ack_prev <= pf_ack_toggle;

        pf_rptr <= rptr_next;
        pf_wptr <= wptr_next;
        pf_byte_offset <= byte_offset_next;
        q_window_r <= q_window_next;
        for (int k = 0; k < 8; k++)
            prefetch_queue[k] <= queue_next[k];

        if (q_flush) begin
            pf_fetch_addr <= {pf_flush_addr[31:4], 4'b0000};
            pf_fetch_word_start <= pf_flush_addr[3:2];
            pf_linear_addr <= {pf_flush_addr[31:4], 4'b0000};
            pf_suspended <= 1'b0;
            if (pf_inflight && !pf_ack_edge) begin
                // Queue the redirect request behind the current prefetch.  When
                // the old line completes, paging can immediately launch this
                // target request instead of waiting for prefetch to toggle in
                // the following cycle.
                pf_drop_inflight <= 1'b1;
                pf_redirect_queued <= 1'b1;
                pf_req_toggle <= pf_ack_toggle;
            end
            // synthesis translate_off
            if (TRACE_FLUSH_EN)
                $display("BIU FLUSH: pf_flush_addr=%08x word_start=%d byte_offset=%d",
                         pf_flush_addr, pf_flush_addr[3:2], pf_flush_addr[1:0]);
            // synthesis translate_on
        end

        if (pf_ack_edge && !q_flush) begin
            if (pf_drop_inflight || pf_fault) begin
                pf_drop_inflight <= 1'b0;
                pf_redirect_queued <= 1'b0;
                if (pf_fault && !pf_drop_inflight)
                    pf_suspended <= 1'b1;
            end else begin
                pf_fetch_addr <= pf_fetch_addr + 32'd16;
                pf_fetch_word_start <= 2'd0;
            end
        end

        if (pf_can_fetch_after_flush || pf_can_fetch) begin
            pf_req_toggle <= ~pf_req_toggle;
            if (pf_can_fetch_after_flush) begin
                pf_linear_addr <= {pf_flush_addr[31:4], 4'b0000};
            end else if (good_ack) begin
                pf_linear_addr <= pf_fetch_addr + 32'd16;
            end else begin
                // Testbenches seed pf_fetch_addr directly to CS.base+EIP.
                // If that initial address is in the middle of a cache line,
                // derive the queue start position from it before the first
                // fill returns.  After the first fill, pf_fetch_addr is kept
                // line-aligned and pf_fetch_word_start remains zero.
                pf_linear_addr <= {pf_fetch_addr[31:4], 4'b0000};
                if (q_empty && pf_fetch_addr[3:0] != 4'h0) begin
                    pf_fetch_word_start <= pf_fetch_addr[3:2];
                    pf_fetch_addr <= {pf_fetch_addr[31:4], 4'b0000};
                end
            end
        end
    end
end

endmodule
