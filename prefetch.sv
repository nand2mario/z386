// Prefetch Unit - 32-byte circular buffer, filled one 16-byte cache line at a time M2v2 two-cursor read protocol (doc/z386x/m2v2.md): *...
// Details: doc/z386x/implementation_notes.md#src-24-z386x-prefetch-sv-1

module prefetch
    import z386_pkg::*;
(
    input             clk,
    input             reset_n,

    // Queue read interface to the decoder (two cursors, one pop point)
    output     [63:0] win_d1,        // registered raw window at the D1 cursor
    output     [5:0]  d1_avail,      // bytes fetched beyond the D1 cursor
    input      [3:0]  d1_adv,        // D1 cursor advance this cycle (0-11: a
                                     //   prefix, or the instruction rest at handoff)
    output     [31:0] win_lit,       // 4 bytes at pop_cursor + lit_off
    output     [5:0]  lit_avail,     // bytes fetched beyond that point
    input      [4:0]  lit_off,       // literal offset from the pop cursor
    output            q_full,
    input             pop_now,       // one instruction completed D2
    input      [4:0]  pop_len,       // its full byte length (registered source)

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
    input             pf_suspend,    // external suspend (e.g. page fault handler active)
    input             halt_speculative, // decode queue holds a taken JMP/CALL: stop fetching past it

    // z386x speculative branch-target line (doc/z386x/m4.md). spec_req at a relative branch's i_pop latches the target line address; the...
    // Details: doc/z386x/implementation_notes.md#src-24-z386x-prefetch-sv-54
    input             spec_req,
    input      [31:0] spec_linear,
    input             spec_owner,    // the currently-executing instruction is the
                                     // one that requested the buffered/in-flight
                                     // spec fetch: its taken-flush address equals
                                     // the spec target BY CONSTRUCTION (same
                                     // adder inputs), so no address compare
    input             spec_store_valid,
    input      [31:0] spec_store_linear,
    input             spec_global_kill
);

// 32-byte prefetch queue (8 x 32-bit words).  Cache fills write up to four
// queue words at once.  After a branch into the middle of a cache line, the
// first fill drops words before the target address and starts decoding at the
// requested byte offset.
reg [31:0] prefetch_queue [7:0];
reg [3:0]  pf_rptr;                  // Pop cursor word (0-7) with wraparound bit
reg [3:0]  pf_wptr;                  // Write pointer (0-7) with wraparound bit
reg [1:0]  pf_byte_offset;           // Pop cursor byte offset within dword (0-3)
reg [3:0]  d1_word;                  // D1 cursor word, >= pop cursor
reg [1:0]  d1_boff;                  // D1 cursor byte offset
reg [1:0]  pf_fetch_word_start;      // First word to keep from next fetched line
reg        pf_suspended;             // Prefetch suspended (page fault until flush)
reg        pf_drop_inflight;         // Drop next prefetch result (flush during in-flight)
reg [31:0] pf_fetch_addr;            // Next LINEAR cache-line address to prefetch

// Speculative branch-target line buffer. spec_addr is the line address of the OUTSTANDING or BUFFERED fetch and only updates at launch; a...
// Details: doc/z386x/implementation_notes.md#src-24-z386x-prefetch-sv-87
reg         spec_pend;               // latched request waiting for the port
reg  [31:0] spec_pend_addr;          // requested target (full: line + in-line offset)
reg  [31:4] spec_addr;               // line address of the in-flight/buffered fetch
reg  [3:0]  spec_off;                // target's in-line offset, latched at launch:
                                     //   drives the flush-hit seed placement from
                                     //   REGISTERS (pf_flush_addr arrives too late)
reg         spec_inflight;           // the outstanding paging request is the spec fetch
reg         spec_valid;              // spec_line holds the line at spec_addr
reg         spec_adopted_r;          // post-flush fill is an adopted spec line: buffer it too
reg         spec_poison;             // a store/snoop occurred since the fetch started
reg [127:0] spec_line;

// Normal 386 self-modifying code performs a frontend-flushing branch after
// the store. Keep the buffered target coherent for that branch without
// discarding it for unrelated data stores. Global events remain conservative.
wire spec_store_hit = spec_store_valid &&
                      (spec_addr == spec_store_linear[31:4]);
wire spec_kill = spec_global_kill || spec_store_hit;

// synthesis translate_off
bit TRACE_FLUSH_EN;
initial TRACE_FLUSH_EN = $test$plusargs("trace_flush");
// Sim-only: the queue powers up X.  win_d1 is reset to 0, but win_lit
// (combinational over the queue) would be X until the first fill.  Hardware
// defines the queue via reset+fill before any decode; zero it here so sim
// matches.
initial for (int k = 0; k < 8; k++) prefetch_queue[k] = 32'h0;
// synthesis translate_on

function automatic [2:0] ptr_idx(input [3:0] ptr);
    begin
        ptr_idx = ptr[2:0];
    end
endfunction

wire [3:0] pf_word_count = pf_wptr - pf_rptr; // 0..8 valid queue words
wire       q_empty = (pf_word_count == 4'd0);
wire [5:0] pf_byte_count = q_empty ? 6'd0 :
                           ({2'b00, pf_word_count} << 2) - {4'b0000, pf_byte_offset};
assign q_full = (pf_word_count == 4'd8);

// D1 cursor byte availability.  The cursor may legitimately sit BEYOND the
// write pointer (a skeleton handoff advances it past literal bytes that are
// still being fetched), so derive it as pop-relative lead vs the buffered
// byte count and clamp at zero.
wire [3:0] d1_lead_words = d1_word - pf_rptr;
wire [6:0] d1_lead = {1'b0, d1_lead_words, 2'b00} + {5'b00000, d1_boff} -
                     {5'b00000, pf_byte_offset};
wire [7:0] d1_avail_s = {2'b00, pf_byte_count} - {1'b0, d1_lead};
assign d1_avail = d1_avail_s[7] ? 6'd0 : d1_avail_s[5:0];

// D2 literal window: pop_cursor + lit_off, over REGISTERED queue words.
wire [5:0] lit_sum  = {4'b0000, pf_byte_offset} + {1'b0, lit_off};
wire [3:0] lit_word = pf_rptr + {1'b0, lit_sum[4:2]};
wire [31:0] lit_word_cur = prefetch_queue[ptr_idx(lit_word)];
wire [31:0] lit_word_nxt = prefetch_queue[ptr_idx(lit_word + 4'd1)];
assign win_lit =
    lit_sum[1:0] == 2'd0 ? lit_word_cur :
    lit_sum[1:0] == 2'd1 ? {lit_word_nxt[7:0],  lit_word_cur[31:8]} :
    lit_sum[1:0] == 2'd2 ? {lit_word_nxt[15:0], lit_word_cur[31:16]} :
                           {lit_word_nxt[23:0], lit_word_cur[31:24]};
wire [6:0] lit_avail_s = {1'b0, pf_byte_count} - {2'b00, lit_off};
assign lit_avail = lit_avail_s[6] ? 6'd0 : lit_avail_s[5:0];

wire pf_inflight = (pf_req_toggle != pf_ack_toggle);

reg pf_ack_prev;
wire pf_ack_edge = (pf_ack_toggle != pf_ack_prev);

wire [2:0] fetch_write_words = 3'd4 - {1'b0, pf_fetch_word_start};
// A spec-fetch response goes to the spec buffer, never the queue.
wire good_ack = pf_ack_edge && !pf_drop_inflight && !pf_fault && !spec_inflight;
wire [4:0] pf_words_after_ack =
    {1'b0, pf_word_count} + (good_ack ? {2'b00, fetch_write_words} : 5'd0);
wire pf_has_line_space = (pf_words_after_ack <= 5'd4);

wire pf_can_fetch = pf_has_line_space && !pf_suspended && !pf_suspend &&
                    !q_flush && !pf_inflight && !halt_speculative;
wire pf_can_fetch_after_flush = q_flush && !pf_suspend && !pf_inflight;

// Spec fetch launch: takes priority over sequential prefetch for the shared port; never launches during a flush or while a request is...
// Details: doc/z386x/implementation_notes.md#src-24-z386x-prefetch-sv-165
wire spec_match_now = spec_req && spec_valid && !spec_poison && !spec_inflight &&
                      (spec_addr == spec_linear[31:4]);
wire spec_want   = ((spec_req && !spec_match_now) || spec_pend);
// !pf_redirect_queued/!pf_drop_inflight: the queued-redirect handshake makes pf_inflight look idle (req toggled back to ack) while the...
// Details: doc/z386x/implementation_notes.md#src-24-z386x-prefetch-sv-178
wire spec_launch = spec_want && !pf_inflight && !q_flush && !pf_suspend &&
                   !pf_redirect_queued && !pf_drop_inflight;
// Flush-time spec outcomes (evaluated during q_flush): Hit decision is OWNERSHIP, not an address compare: the flushing branch is the same...
// Details: doc/z386x/implementation_notes.md#src-24-z386x-prefetch-sv-185
wire spec_line_match   = spec_valid && !spec_poison && spec_owner;
wire spec_adopt        = spec_inflight && !spec_poison && !pf_ack_edge && spec_owner;
wire spec_data_now     = spec_inflight && !spec_poison && pf_ack_edge && !pf_fault &&
                         spec_owner;
wire spec_flush_hit    = q_flush && (spec_line_match || spec_data_now);
wire [127:0] spec_hit_line = spec_data_now ? pf_rdata : spec_line;

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
// One pop per instruction: the advance amount is a REGISTERED value
// (skel.length from the decoder), up to 15 bytes + offset 3 -> 5 bits.
wire [5:0] byte_advance = {4'b0000, pf_byte_offset} + {1'b0, pop_len};
// D1 cursor advance: a prefix byte or the instruction rest at handoff.
wire [3:0] d1_sum = {2'b00, d1_boff} + d1_adv;

logic [3:0]  rptr_next;
logic [3:0]  wptr_next;
logic [1:0]  byte_offset_next;
logic [3:0]  d1_word_next;
logic [1:0]  d1_boff_next;
logic [31:0] queue_next [7:0];

always_comb begin
    rptr_next = pf_rptr;
    wptr_next = pf_wptr;
    byte_offset_next = pf_byte_offset;
    d1_word_next = d1_word + {2'b00, d1_sum[3:2]};
    d1_boff_next = d1_sum[1:0];
    for (int k = 0; k < 8; k++)
        queue_next[k] = prefetch_queue[k];

    if (pop_now) begin
        byte_offset_next = byte_advance[1:0];
        rptr_next = pf_rptr + byte_advance[5:2];
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
        byte_offset_next = spec_flush_hit ? spec_off[1:0] : pf_flush_addr[1:0];
        d1_word_next = 4'h0;
        d1_boff_next = spec_flush_hit ? spec_off[1:0] : pf_flush_addr[1:0];
        if (spec_flush_hit) begin
            // Seed the queue from the buffered target line right now. All
            // placement selects come from the LATCHED spec_off - only the hit
            // control bit sees the late flush address.
            unique case (spec_off[3:2])
                2'd0: begin
                    queue_next[0] = line_word(spec_hit_line, 2'd0);
                    queue_next[1] = line_word(spec_hit_line, 2'd1);
                    queue_next[2] = line_word(spec_hit_line, 2'd2);
                    queue_next[3] = line_word(spec_hit_line, 2'd3);
                    wptr_next = 4'd4;
                end
                2'd1: begin
                    queue_next[0] = line_word(spec_hit_line, 2'd1);
                    queue_next[1] = line_word(spec_hit_line, 2'd2);
                    queue_next[2] = line_word(spec_hit_line, 2'd3);
                    wptr_next = 4'd3;
                end
                2'd2: begin
                    queue_next[0] = line_word(spec_hit_line, 2'd2);
                    queue_next[1] = line_word(spec_hit_line, 2'd3);
                    wptr_next = 4'd2;
                end
                default: begin
                    queue_next[0] = line_word(spec_hit_line, 2'd3);
                    wptr_next = 4'd1;
                end
            endcase
        end
    end

    if (seed_now) begin
        byte_offset_next = pf_fetch_addr[1:0];
        d1_boff_next = pf_fetch_addr[1:0];
    end
end

// synthesis translate_off
reg PF_CUR = 1'b0;
initial if ($test$plusargs("pf_cur")) PF_CUR = 1'b1;
always @(posedge clk) begin
    if (PF_CUR && (pop_now || (d1_adv != 4'd0) || q_flush || fill_commit))
        $display("%0t PFCUR pop=%b len=%0d adv=%0d flush=%b fill=%b | rptr=%0d.%0d wptr=%0d d1=%0d.%0d cnt=%0d d1av=%0d litoff=%0d litav=%0d",
                 $time, pop_now, pop_len, d1_adv, q_flush, fill_commit,
                 pf_rptr, pf_byte_offset, pf_wptr, d1_word, d1_boff,
                 pf_byte_count, d1_avail, lit_off, lit_avail);
    if (reset_n && pop_now && ({1'b0, pf_byte_count} < {2'b00, pop_len}))
        $fatal(1, "PF: pop_now for %0d bytes with only %0d buffered (rptr=%0d.%0d wptr=%0d d1=%0d.%0d flush=%b)",
               pop_len, pf_byte_count, pf_rptr, pf_byte_offset, pf_wptr,
               d1_word, d1_boff, q_flush);
end
// synthesis translate_on

// synthesis translate_off
reg PF_EVT = 1'b0;
initial if ($test$plusargs("pf_evt")) PF_EVT = 1'b1;
always @(posedge clk) if (PF_EVT) begin
    if (q_flush)
        $display("%0t PF flush addr=%08x hit=%b adopt=%b datanow=%b specv=%b specinfl=%b pfinfl=%b ack=%b off=%h",
                 $time, pf_flush_addr, spec_flush_hit, spec_adopt, spec_data_now,
                 spec_valid, spec_inflight, pf_inflight, pf_ack_edge, spec_off);
    if (fill_commit)
        $display("%0t PF fill wptr=%h ws=%h data0=%08x (fetch=%08x)",
                 $time, pf_wptr, pf_fetch_word_start, pf_rdata[31:0], pf_fetch_addr);
    if (spec_req)
        $display("%0t PF specreq lin=%08x match=%b (specaddr=%07x0 v=%b)",
                 $time, spec_linear, spec_match_now, spec_addr, spec_valid);
    if (spec_launch)
        $display("%0t PF speclaunch tgt=%08x", $time,
                 (spec_req && !q_flush) ? spec_linear : spec_pend_addr);
    if (pf_ack_edge)
        $display("%0t PF ack drop=%b fault=%b specinfl=%b adopted=%b data0=%08x",
                 $time, pf_drop_inflight, pf_fault, spec_inflight, spec_adopted_r, pf_rdata[31:0]);
end
// synthesis translate_on

// D1 window: registered from the queue's NEXT state at the NEXT cursor, so the decoder always sees the byte rotate of the new cursor...
// Details: doc/z386x/implementation_notes.md#src-24-z386x-prefetch-sv-342
wire [31:0] d1_word_cur_next = queue_next[ptr_idx(d1_word_next)];
wire [31:0] d1_word_nxt_next = queue_next[ptr_idx(d1_word_next + 4'd1)];
wire [31:0] d1_word_2nd_next = queue_next[ptr_idx(d1_word_next + 4'd2)];

wire [63:0] win_d1_next =
    d1_boff_next == 2'd0 ? {d1_word_nxt_next,       d1_word_cur_next} :
    d1_boff_next == 2'd1 ? {d1_word_2nd_next[7:0],  d1_word_nxt_next,
                                                     d1_word_cur_next[31:8]} :
    d1_boff_next == 2'd2 ? {d1_word_2nd_next[15:0], d1_word_nxt_next,
                                                     d1_word_cur_next[31:16]} :
                           {d1_word_2nd_next[23:0], d1_word_nxt_next,
                                                     d1_word_cur_next[31:24]};

// Keep the queue/decoder boundary physical. Quartus retiming this register
// turns an icache response into a same-cycle cache -> aligner -> D1 PLA path.
(* preserve *) reg [63:0] win_d1_r;
assign win_d1 = win_d1_r;

always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        pf_rptr <= 4'h0;
        pf_wptr <= 4'h0;
        pf_byte_offset <= 2'h0;
        d1_word <= 4'h0;
        d1_boff <= 2'h0;
        pf_fetch_word_start <= 2'h0;
        pf_suspended <= 1'b0;
        pf_drop_inflight <= 1'b0;
        pf_fetch_addr <= 32'hFFFF_FFF0;  // Reset vector, cache-line aligned
        pf_req_toggle <= 1'b0;
        pf_linear_addr <= 32'h0;
        pf_redirect_queued <= 1'b0;
        pf_ack_prev <= 1'b0;
        win_d1_r <= 64'h0;
        spec_pend <= 1'b0;
        spec_inflight <= 1'b0;
        spec_valid <= 1'b0;
        spec_adopted_r <= 1'b0;
        spec_poison <= 1'b0;
        spec_pend_addr <= '0;
        spec_addr <= '0;
        spec_off <= '0;
    end else begin
        pf_ack_prev <= pf_ack_toggle;

        pf_rptr <= rptr_next;
        pf_wptr <= wptr_next;
        pf_byte_offset <= byte_offset_next;
        d1_word <= d1_word_next;
        d1_boff <= d1_boff_next;
        win_d1_r <= win_d1_next;
        for (int k = 0; k < 8; k++)
            prefetch_queue[k] <= queue_next[k];

        if (q_flush) begin
            pf_fetch_addr <= {pf_flush_addr[31:4], 4'b0000};
            pf_fetch_word_start <= pf_flush_addr[3:2];
            pf_linear_addr <= {pf_flush_addr[31:4], 4'b0000};
            pf_suspended <= 1'b0;
            if (spec_flush_hit) begin
                // Target line already buffered (or arriving right now): the
                // queue was seeded combinationally; continue sequentially.
                pf_fetch_addr <= {pf_flush_addr[31:4], 4'b0000} + 32'd16;
                pf_fetch_word_start <= 2'd0;
                pf_linear_addr <= {pf_flush_addr[31:4], 4'b0000} + 32'd16;
                // A sequential prefetch may still be in flight down the OLD path: its fill would land on top of the freshly seeded queue (at the...
                // Details: doc/z386x/implementation_notes.md#src-24-z386x-prefetch-sv-403
                if (pf_inflight && !pf_ack_edge && !spec_inflight) begin
                    pf_drop_inflight <= 1'b1;
                    pf_redirect_queued <= 1'b1;
                    pf_req_toggle <= pf_ack_toggle;
                end
            end else if (spec_adopt) begin
                // The spec fetch for exactly this target is still in flight:
                // adopt it as the post-flush fill (no drop, no new request).
            end else if (pf_inflight && !pf_ack_edge && !spec_inflight) begin
                // Queue the redirect request behind the current prefetch.  When
                // the old line completes, paging can immediately launch this
                // target request instead of waiting for prefetch to toggle in
                // the following cycle.
                pf_drop_inflight <= 1'b1;
                pf_redirect_queued <= 1'b1;
                pf_req_toggle <= pf_ack_toggle;
            end
            spec_pend <= 1'b0;
            // spec_valid deliberately SURVIVES the flush: the buffered line's data is linear-tagged and stays correct (CR3/CR0 changes and...
            // Details: doc/z386x/implementation_notes.md#src-24-z386x-prefetch-sv-428
            if (spec_adopt || (spec_inflight && pf_ack_edge))
                spec_inflight <= 1'b0;
            // Track the adopted fill so its arriving line is ALSO captured into the buffer (spec_addr/spec_off still hold the target): otherwise a...
            // Details: doc/z386x/implementation_notes.md#src-24-z386x-prefetch-sv-438
            spec_adopted_r <= spec_adopt;
            if (spec_data_now) begin
                // Ack in the flush cycle itself: seed consumed it; buffer too.
                spec_line  <= pf_rdata;
                spec_valid <= 1'b1;
            end
            spec_poison <= 1'b0;
            // A flush that bypasses an in-flight spec fetch invalidates its
            // context (CR3/CS may change before the late response lands).
            if (spec_inflight && !spec_adopt && !pf_ack_edge)
                spec_poison <= 1'b1;
            // synthesis translate_off
            if (TRACE_FLUSH_EN)
                $display("BIU FLUSH: pf_flush_addr=%08x word_start=%d byte_offset=%d",
                         pf_flush_addr, pf_flush_addr[3:2], pf_flush_addr[1:0]);
            // synthesis translate_on
        end

        if (pf_ack_edge && !q_flush) begin
            spec_adopted_r <= 1'b0;
            if (spec_inflight) begin
                // Speculative target line: buffer it; a fault only drops the
                // buffer (never suspends - the fetch may be down a wrong path).
                spec_inflight <= 1'b0;
                spec_line <= pf_rdata;
                spec_valid <= !pf_fault && !spec_poison;
            end else if (spec_adopted_r && !pf_drop_inflight && !pf_fault) begin
                // Adopted post-flush fill arriving: it fills the queue as
                // usual (fill_commit), AND populates the buffer for the
                // loop's next iteration (re-own via spec_match_now, no
                // refetch).  Faults/drops fall through to the branch below.
                spec_line  <= pf_rdata;
                spec_valid <= !spec_poison;
                pf_fetch_addr <= pf_fetch_addr + 32'd16;
                pf_fetch_word_start <= 2'd0;
            end else if (pf_drop_inflight || pf_fault) begin
                pf_drop_inflight <= 1'b0;
                pf_redirect_queued <= 1'b0;
                if (pf_fault && !pf_drop_inflight)
                    pf_suspended <= 1'b1;
            end else begin
                pf_fetch_addr <= pf_fetch_addr + 32'd16;
                pf_fetch_word_start <= 2'd0;
            end
        end

        // Latch a spec request; a newer request replaces an older PENDING one but never the address of a fetch already in flight. Ownership-based...
        // Details: doc/z386x/implementation_notes.md#src-24-z386x-prefetch-sv-489
        if (spec_req && !q_flush) begin
            if (spec_match_now) begin
                // Buffered line already holds this target: re-own, no refetch.
                spec_off <= spec_linear[3:0];
            end else begin
                spec_pend <= 1'b1;
                spec_pend_addr <= spec_linear;
                spec_valid <= 1'b0;
                if (spec_inflight)
                    spec_poison <= 1'b1;
            end
        end

        // Store commit / external snoop: the buffered (or in-flight) target
        // line may be stale; poison it (386 jump-must-refetch semantics).
        // A pending (not yet launched) request will fetch fresh data.
        if (spec_kill) begin
            spec_valid <= 1'b0;
            // Also cancel a pending adopted-fill capture: after an adopting flush spec_inflight is already clear (no poison path), but the arriving...
            // Details: doc/z386x/implementation_notes.md#src-24-z386x-prefetch-sv-512
            spec_adopted_r <= 1'b0;
            if (spec_inflight)
                spec_poison <= 1'b1;
        end

        if (spec_launch) begin
            automatic logic [31:0] tgt = (spec_req && !q_flush) ? spec_linear : spec_pend_addr;
            pf_req_toggle <= ~pf_req_toggle;
            pf_linear_addr <= {tgt[31:4], 4'b0000};
            spec_addr      <= tgt[31:4];
            spec_off       <= tgt[3:0];
            spec_valid <= 1'b0;
            // A store committing in this very cycle may still race the icache
            // read: keep the poison.
            spec_poison <= spec_kill;
            spec_inflight <= 1'b1;
            spec_pend <= 1'b0;
        end else if (pf_can_fetch_after_flush || pf_can_fetch) begin
            pf_req_toggle <= ~pf_req_toggle;
            if (pf_can_fetch_after_flush) begin
                // On a spec-hit flush the target line is already in the queue:
                // the post-flush fetch continues at the NEXT line.
                pf_linear_addr <= {pf_flush_addr[31:4], 4'b0000} +
                                  (spec_flush_hit ? 32'd16 : 32'd0);
            end else if (good_ack) begin
                pf_linear_addr <= pf_fetch_addr + 32'd16;
            end else begin
                // Testbenches seed pf_fetch_addr directly to CS.base+EIP. If that initial address is in the middle of a cache line, derive the queue...
                // Details: doc/z386x/implementation_notes.md#src-24-z386x-prefetch-sv-545
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
