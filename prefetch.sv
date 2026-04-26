//
// Prefetch Unit - 16-byte circular buffer
//

module prefetch
    import z386_pkg::*;
(
    input             clk,
    input             reset_n,

    // Queue output to decoder
    output     [7:0]  q_bus,
    output     [31:0] q_window,      // 4-byte aligned window at current queue head
    output            q_full,
    output            q_empty,
    output     [4:0]  pf_count,
    input      [2:0]  q_pop_bytes,

    // Flush from microcode
    input             q_flush,
    input      [31:0] pf_flush_addr, // LINEAR address

    // Toggle interface to paging unit
    output reg        pf_req_toggle,
    output reg [31:0] pf_linear_addr,
    input             pf_ack_toggle,
    input      [31:0] pf_rdata,
    input             pf_fault,      // page fault on this fetch (silently drop)

    // Control
    input             pf_suspend     // external suspend (e.g. page fault handler active)
);

// 16-byte prefetch queue (4 x 32-bit words)
reg [31:0] prefetch_queue [3:0];
reg [2:0]  pf_rptr;                  // Read pointer (0-3) with wraparound bit
reg [2:0]  pf_wptr;                  // Write pointer (0-3) with wraparound bit
reg [1:0]  pf_byte_offset;           // Byte offset within current dword (0-3)
reg        pf_suspended;             // Prefetch suspended (page fault until flush)
reg        pf_drop_inflight;         // Drop next prefetch result (flush during in-flight)
reg [31:0] pf_fetch_addr;            // Next LINEAR address to prefetch
// synthesis translate_off
bit TRACE_FLUSH_EN;
initial TRACE_FLUSH_EN = $test$plusargs("trace_flush");
// synthesis translate_on
wire [31:0] q_word_cur = prefetch_queue[pf_rptr[1:0]];
wire [31:0] q_word_next = prefetch_queue[pf_rptr[1:0] + 2'd1];

// Queue output: select byte from current DWORD based on byte offset
assign q_bus = pf_byte_offset == 2'd0 ? prefetch_queue[pf_rptr[1:0]][7:0] :
               pf_byte_offset == 2'd1 ? prefetch_queue[pf_rptr[1:0]][15:8] :
               pf_byte_offset == 2'd2 ? prefetch_queue[pf_rptr[1:0]][23:16] :
                                         prefetch_queue[pf_rptr[1:0]][31:24];
assign q_window = pf_byte_offset == 2'd0 ? q_word_cur :
                  pf_byte_offset == 2'd1 ? {q_word_next[7:0], q_word_cur[31:8]} :
                  pf_byte_offset == 2'd2 ? {q_word_next[15:0], q_word_cur[31:16]} :
                                           {q_word_next[23:0], q_word_cur[31:24]};

// Queue status
assign q_full  = (pf_wptr == {~pf_rptr[2], pf_rptr[1:0]});
assign q_empty = (pf_rptr == pf_wptr);

// Calculate number of valid bytes in queue
wire [2:0] pf_diff = pf_wptr - pf_rptr;
assign pf_count = {pf_diff[2:0], 2'b0} - {3'b0, pf_byte_offset};

// Toggle-based in-flight tracking
wire pf_inflight = (pf_req_toggle != pf_ack_toggle);

// Ack edge detection
reg pf_ack_prev;
wire pf_ack_edge = (pf_ack_toggle != pf_ack_prev);

// When a good ack writes to the queue this cycle, check if wptr+1 makes it full
wire good_ack = pf_ack_edge && !pf_drop_inflight && !pf_fault;
wire q_full_next = ((pf_wptr + 3'd1) == {~pf_rptr[2], pf_rptr[1:0]});

// Can we issue a new fetch?
// Use q_full_next when ack is writing to queue (wptr about to advance)
wire pf_can_fetch = !(good_ack ? q_full_next : q_full) && !pf_suspended && !pf_suspend && !q_flush && !pf_inflight;
wire pf_can_fetch_after_flush = q_flush && !pf_suspend && !pf_inflight;

always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        pf_rptr <= 3'h0;
        pf_wptr <= 3'h0;
        pf_byte_offset <= 2'h0;
        pf_suspended <= 1'b0;
        pf_drop_inflight <= 1'b0;
        pf_fetch_addr <= 32'hFFFFFFF0;  // Reset vector
        pf_req_toggle <= 1'b0;
        pf_linear_addr <= 32'h0;
        pf_ack_prev <= 1'b0;
    end else begin
        pf_ack_prev <= pf_ack_toggle;

        // Queue pop: advance by 1/2/4 bytes as directed by the decoder
        if ((q_pop_bytes != 3'd0) && !q_empty) begin
            automatic logic [2:0] byte_advance = {1'b0, pf_byte_offset} + q_pop_bytes;
            pf_byte_offset <= byte_advance[1:0];
            pf_rptr <= pf_rptr + {2'd0, byte_advance[2]};
        end

        // Queue flush: reset pointers and restart prefetch at new address
        if (q_flush) begin
            pf_rptr <= 3'h0;
            pf_wptr <= 3'h0;
            pf_fetch_addr <= pf_flush_addr;
            pf_byte_offset <= pf_flush_addr[1:0];
            pf_suspended <= 1'b0;
            // Only set drop_inflight if there's a request in-flight AND the ack
            // hasn't arrived this same cycle. If ack arrives on the same cycle as
            // flush, the inflight request is already done (no future ack to drop).
            if (pf_inflight && !pf_ack_edge)
                pf_drop_inflight <= 1'b1;
            // synthesis translate_off
            if (TRACE_FLUSH_EN)
                $display("BIU FLUSH: pf_flush_addr=%08x pf_byte_offset=%d",
                         pf_flush_addr, pf_flush_addr[1:0]);
            // synthesis translate_on
        end

        // Handle ack edge: paging unit completed our request
        // Gate on !q_flush: if flush and ack collide, flush wins (data is stale)
        if (pf_ack_edge && !q_flush) begin
            if (pf_drop_inflight || pf_fault) begin
                // Discard: flush happened while in flight, or page fault
                pf_drop_inflight <= 1'b0;
                if (pf_fault && !pf_drop_inflight)
                    pf_suspended <= 1'b1;  // Suspend until next flush
            end else begin
                // Good data: write to queue and advance fetch address
                prefetch_queue[pf_wptr[1:0]] <= pf_rdata;
                pf_wptr <= pf_wptr + 3'd1;
                pf_fetch_addr[31:2] <= pf_fetch_addr[31:2] + 30'd1;
                pf_fetch_addr[1:0] <= 2'd0;
            end
        end

        // Issue new fetch request (can overlap with ack processing)
        // When ack with good data arrives same cycle, pf_fetch_addr NBA hasn't
        // taken effect yet — use incremented address to avoid re-fetching.
        if (pf_can_fetch_after_flush || pf_can_fetch) begin
            pf_req_toggle <= ~pf_req_toggle;
            if (pf_can_fetch_after_flush)
                pf_linear_addr <= {pf_flush_addr[31:2], 2'b00};
            else if (good_ack)
                pf_linear_addr <= {pf_fetch_addr[31:2] + 30'd1, 2'b00};
            else
                pf_linear_addr <= {pf_fetch_addr[31:2], 2'b00};
        end
    end
end

endmodule
