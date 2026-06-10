//
// Prefetch Unit - 32-byte circular buffer, filled one 16-byte cache line at a time
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

// synthesis translate_off
bit TRACE_FLUSH_EN;
initial TRACE_FLUSH_EN = $test$plusargs("trace_flush");
// synthesis translate_on

function automatic [2:0] ptr_idx(input [3:0] ptr);
    begin
        ptr_idx = ptr[2:0];
    end
endfunction

wire [31:0] q_word_cur = prefetch_queue[ptr_idx(pf_rptr)];
wire [31:0] q_word_next = prefetch_queue[ptr_idx(pf_rptr + 4'd1)];

assign q_window = pf_byte_offset == 2'd0 ? q_word_cur :
                  pf_byte_offset == 2'd1 ? {q_word_next[7:0], q_word_cur[31:8]} :
                  pf_byte_offset == 2'd2 ? {q_word_next[15:0], q_word_cur[31:16]} :
                                           {q_word_next[23:0], q_word_cur[31:24]};

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
    end else begin
        pf_ack_prev <= pf_ack_toggle;

        if ((q_pop_bytes != 3'd0) && !q_empty) begin
            automatic logic [2:0] byte_advance = {1'b0, pf_byte_offset} + q_pop_bytes;
            pf_byte_offset <= byte_advance[1:0];
            pf_rptr <= pf_rptr + {3'd0, byte_advance[2]};
        end

        if (q_flush) begin
            pf_rptr <= 4'h0;
            pf_wptr <= 4'h0;
            pf_fetch_addr <= {pf_flush_addr[31:4], 4'b0000};
            pf_fetch_word_start <= pf_flush_addr[3:2];
            pf_byte_offset <= pf_flush_addr[1:0];
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
                unique case (pf_fetch_word_start)
                    2'd0: begin
                        prefetch_queue[ptr_idx(pf_wptr)] <= line_word(pf_rdata, 2'd0);
                        prefetch_queue[ptr_idx(pf_wptr + 4'd1)] <= line_word(pf_rdata, 2'd1);
                        prefetch_queue[ptr_idx(pf_wptr + 4'd2)] <= line_word(pf_rdata, 2'd2);
                        prefetch_queue[ptr_idx(pf_wptr + 4'd3)] <= line_word(pf_rdata, 2'd3);
                    end
                    2'd1: begin
                        prefetch_queue[ptr_idx(pf_wptr)] <= line_word(pf_rdata, 2'd1);
                        prefetch_queue[ptr_idx(pf_wptr + 4'd1)] <= line_word(pf_rdata, 2'd2);
                        prefetch_queue[ptr_idx(pf_wptr + 4'd2)] <= line_word(pf_rdata, 2'd3);
                    end
                    2'd2: begin
                        prefetch_queue[ptr_idx(pf_wptr)] <= line_word(pf_rdata, 2'd2);
                        prefetch_queue[ptr_idx(pf_wptr + 4'd1)] <= line_word(pf_rdata, 2'd3);
                    end
                    default: begin
                        prefetch_queue[ptr_idx(pf_wptr)] <= line_word(pf_rdata, 2'd3);
                    end
                endcase

                pf_wptr <= pf_wptr + {1'b0, fetch_write_words};
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
                    pf_byte_offset <= pf_fetch_addr[1:0];
                    pf_fetch_addr <= {pf_fetch_addr[31:4], 4'b0000};
                end
            end
        end
    end
end

endmodule
