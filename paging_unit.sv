//
// Paging Unit for 80386 Processor
// Integrates TLB and Page Walker for address translation.
// Also handles DWORD-crossing splits: receives linear address + op details,
// does TLB translation, splits crossings into two aligned sub-requests,
// and feeds tuples to BIU.
//
// This is the single arbiter for ALL memory accesses:
//   - Memory/IO from microcode (pulse: mem_req / mem_servicing)
//   - Prefetch (toggle protocol: pf_req_toggle / pf_ack_toggle)
//   - Page walks (internal, routed to BIU)
//
// When CR0.PG=0, bypasses paging (linear = physical) and still handles
// crossing detection and splitting.
//

module paging_unit
    import z386_pkg::*;
(
    input               clk,
    input               reset_n,

    // Control registers
    input        [31:0] cr0,
    input        [31:0] cr3,
    input               cr3_write,         // TLB flush on CR3 write

    //=========================================================================
    // Memory/IO request from z386.sv
    //=========================================================================
    input               mem_req,           // Valid: memory/IO request pending
    input               mem_req_upcoming,  // Combinational early hint: suppresses prefetch start
    output reg          mem_accepted,      // Ready: request accepted this cycle (one-cycle pulse)
    output reg          mem_servicing,     // High while accepted request is in flight
    output              mem_complete_now,  // Completion shortcut (disabled: use registered mem_servicing clear)
    // Parameters needs to be valid on the cycle mem_req is high only and will be registered
    input        [31:0] linear_addr,       // Linear address for this request
    input        [1:0]  mem_op_size,       // 0=byte, 1=word, 2=dword
    input               mem_write,         // 1=write, 0=read
    input        [31:0] mem_wdata,         // Write data (pre-computed by z386.sv)
    input               mem_rd_ind,        // BUSOP_RD_IND flag
    input               is_write_access,   // Is this a write (for permission check)
    input               mem_check_only,    // CW: check write permission only, no actual bus write
    input        [1:0]  cpl,               // Current privilege level
    input               mem_is_io,         // This request is IO (skip translation)
    input               mem_is_inta,       // IACK bus cycle (bypass paging, like IO)
    input        [3:0]  mem_be,            // Pre-computed byte enables (for IO and non-crossing mem)

    //=========================================================================
    // Prefetch request (toggle protocol)
    //=========================================================================
    input               pf_req_toggle,
    output              pf_ack_toggle,
    input        [31:0] pf_linear_addr,    // LINEAR address (DWORD-aligned)
    output       [31:0] pf_rdata,          // Read data returned to prefetch
    output reg          pf_fault,          // Page fault (silently drop, suspend)

    //=========================================================================
    // BIU interface (physical bus driver)
    //=========================================================================
    output reg          biu_req_valid,     // Aligned request ready for BIU
    (* syn_replicate = 1 *)
    output reg   [31:0] biu_req_phys_addr, // Physical address (full 32-bit)
    output reg          biu_req_write,     // 1=write
    output reg   [3:0]  biu_req_be,        // Byte enables (pre-computed)
    output reg   [31:0] biu_req_wdata,     // Write data (pre-positioned on bus)
    output reg          biu_req_is_io,     // Request is IO space
    output reg          biu_req_is_inta,   // Request is INTA cycle

    // BIU completion feedback
    input               biu_req_accepted,  // BIU accepted the request this cycle
    input               biu_req_complete,  // BIU bus cycle complete
    input        [31:0] biu_rdata,         // Raw bus read data

    output reg   [31:0] OPR_R,

    output reg          page_fault,        // Page fault occurred (mem/IO requests only)
    output reg   [2:0]  fault_code,        // Error code for page fault
    output reg   [31:0] cr2_out,           // Faulting address (written to CR2)

    // rd_ind pass-through
    output reg          rd_ind_active,     // BUSOP_RD_IND is active for this request

    // VIPT: combinational early cache lookup handshake (1 cycle before BIU accepts request)
    // For translated accesses this uses the linear address; page-walk accesses use
    // the physical walker_mem_addr directly because they bypass the TLB.
    output              cache_lookup,       // Valid: reserve tag+data pre-read this cycle
    output       [31:0] cache_lookup_addr,  // Cache index address for the upcoming BIU request
    output              cache_lookup_write, // Lookup corresponds to a write request
    output              cache_lookup_cancel,// Drop a preread that will not launch a cacheable request
    input               cache_lookup_ready  // Cache can accept a new pre-read
);

// Control register bits
wire pg_enable = cr0[31];   // PG - Paging enable
wire wp_enable = cr0[16];   // WP - Write protect

// Compile-time-off sim trace flags to keep hot scheduler paths free of
// per-cycle $test$plusargs overhead under Verilator.
localparam bit TRACE_MEM_EN    = 1'b0;
localparam bit TRACE_PAGING_EN = 1'b0;

reg pf_ack_toggle_r;
reg [31:0] pf_rdata_r;        // Registered read data for prefetch
wire pf_ack_bypass;
assign pf_ack_toggle = pf_ack_toggle_r ^ pf_ack_bypass;
assign pf_rdata = pf_ack_bypass ? biu_rdata : pf_rdata_r;

wire pf_pending  = (pf_req_toggle != pf_ack_toggle_r);

//=============================================================================
// TLB Interface
//=============================================================================
wire        tlb_hit;
wire [31:0] tlb_physical_addr;
wire        tlb_writable;
wire        tlb_user;
wire        tlb_dirty;
wire        tlb_live_hit;
wire [31:0] tlb_live_physical_addr;
wire        tlb_live_writable;
wire        tlb_live_user;
wire        tlb_live_dirty;

// TLB update signals (from page walker)
logic        tlb_update_valid;
logic [19:0] tlb_update_vpn;
logic [19:0] tlb_update_pfn;
logic        tlb_update_writable;
logic        tlb_update_user;
logic        tlb_update_dirty;
logic        tlb_update_accessed;

// TLB lookup address for prefetch/walker and other registered slow paths.
wire [31:0] tlb_lookup_addr;
reg  [31:0] tlb_lookup_addr_r;
assign tlb_lookup_addr = tlb_lookup_addr_r;

paging_tlb tlb_inst (
    .clk            (clk),
    .reset_n        (reset_n),
    .linear_addr    (tlb_lookup_addr),
    .hit            (tlb_hit),
    .physical_addr  (tlb_physical_addr),
    .writable       (tlb_writable),
    .user           (tlb_user),
    .dirty          (tlb_dirty),
    .linear_addr_live(linear_addr),
    .live_hit       (tlb_live_hit),
    .live_physical_addr(tlb_live_physical_addr),
    .live_writable  (tlb_live_writable),
    .live_user      (tlb_live_user),
    .live_dirty     (tlb_live_dirty),
    .update_valid   (tlb_update_valid),
    .update_vpn     (tlb_update_vpn),
    .update_pfn     (tlb_update_pfn),
    .update_writable(tlb_update_writable),
    .update_user    (tlb_update_user),
    .update_dirty   (tlb_update_dirty),
    .update_accessed(tlb_update_accessed),
    .invalidate_all (cr3_write)
);

//=============================================================================
// Page Walker Interface
//=============================================================================
logic       walk_request;
wire        walk_done;
wire        walk_fault;
wire [2:0]  walk_fault_code;
wire [19:0] walk_result_pfn;
wire        walk_result_writable;
wire        walk_result_user;
wire        walk_result_dirty;
wire        walk_result_accessed;

wire        walker_mem_rd;
wire        walker_mem_wr;
wire [31:0] walker_mem_addr;
wire [31:0] walker_mem_wdata;

// Walker bus read/write tracking: prevents re-emission while op is in flight
reg walk_biu_pending;
wire walker_feed_ready = biu_req_complete && walk_biu_pending;
wire walker_issue_ready = (walker_mem_rd || walker_mem_wr) && !walk_biu_pending && !biu_req_valid;

// Forward declarations — Gowin synthesis requires these before first use
reg        req_is_write;     // declared fully at line ~217
reg [1:0]  req_cpl;          // declared fully at line ~220

paging_walker walker_inst (
    .clk            (clk),
    .reset_n        (reset_n),
    .walk_request   (walk_request),
    .linear_addr    (tlb_lookup_addr),
    .is_write       (req_is_write),
    .cpl            (req_cpl),
    .cr3            (cr3),
    .wp_enable      (wp_enable),
    .walk_done      (walk_done),
    .walk_fault     (walk_fault),
    .fault_code     (walk_fault_code),
    .result_pfn     (walk_result_pfn),
    .result_writable(walk_result_writable),
    .result_user    (walk_result_user),
    .result_dirty   (walk_result_dirty),
    .result_accessed(walk_result_accessed),
    .mem_rd         (walker_mem_rd),
    .mem_wr         (walker_mem_wr),
    .mem_addr       (walker_mem_addr),
    .mem_wdata      (walker_mem_wdata),
    .mem_data       (biu_rdata),
    .mem_ready      (walker_feed_ready)
);

// Permission Checking
wire idle_is_user_mode = (cpl == 2'd3);
wire idle_tlb_user_ok = !idle_is_user_mode || tlb_live_user;
wire idle_tlb_write_ok = !is_write_access || tlb_live_writable || (!idle_is_user_mode && !wp_enable);
wire idle_tlb_access_ok = idle_tlb_user_ok && idle_tlb_write_ok;

wire slow_is_user_mode = (req_cpl == 2'd3);
wire slow_tlb_user_ok = !slow_is_user_mode || tlb_user;
wire slow_tlb_write_ok = !req_is_write || tlb_writable || (!slow_is_user_mode && !wp_enable);
wire slow_tlb_access_ok = slow_tlb_user_ok && slow_tlb_write_ok;

// Prefetch is always a read at the current CPL, so only the U/S check matters.
wire pf_tlb_user_ok = (cpl != 2'd3) || tlb_user;

reg [31:0] req_linear;       // Linear address
reg [1:0]  req_op_size;      // Operand size
//  req_is_write declared above (forward declaration for Gowin)
reg [31:0] req_wdata;        // Write data
//  req_cpl declared above (forward declaration for Gowin)
reg [1:0]  req_offset;       // Address offset [1:0]
reg        req_check_only;   // CW: check write only, no bus write

// Crossing detection (from latched values)
reg        req_crossing;     // Current request crosses DWORD boundary

// Second half tracking
reg [31:0] req_linear2;      // Second half linear address (next page start)
reg        req_is_io;        // IO request (for IO crossing path)

// CR2 register
reg [31:0] cr2_reg;
assign cr2_out = cr2_reg;

reg [1:0]  opr_offset_r;     // OPR_R byte lane offset
reg [1:0]  opr_bytes_r;      // Number of bytes - 1
reg        opr_is_write_r;   // Is write (no OPR_R update on writes)
reg        opr_suppress_r;  // Suppress OPR_R update (INTA first cycle)
reg [1:0]  opr_phys_low_r;   // Physical address [1:0] for byte extraction
reg        opr_is_walk_r;    // Is walker request (no OPR_R)
reg        opr_is_pf_r;      // Is prefetch request (no OPR_R)

// Fast path metadata (mem non-crossing emits directly from PG_IDLE)
reg        fast_path_pending; // A fast-path BIU request is in flight

// Combinational completion: bus op finishing THIS cycle. Allows z386 DLY to
// release stall 1 cycle early (before mem_servicing NBA clears next cycle).
assign mem_complete_now = biu_req_complete && fast_path_pending;

//=============================================================================
// State Machine
//=============================================================================
typedef enum logic [3:0] {
    PG_IDLE,
    PG_WALKING,          // Page walk in progress (mem/IO)
    PG_WALK_LOOKUP,      // Wait for lookup slot before launching walked access
    PG_CROSS_WAIT1,      // First half sent to BIU, waiting for completion
    PG_CROSS_PREP2,      // One-cycle handoff before second-half TLB work
    PG_CROSS_TLB2,       // Check TLB for second half
    PG_CROSS_WALK2,      // Page walk for second half
    PG_CROSS_LOOKUP2,    // Wait for lookup slot before second-half launch
    PG_CROSS_WAIT2,      // Second half sent to BIU, waiting for completion
    PG_PF_WALKING,       // Page walk for prefetch TLB miss
    PG_PF_LOOKUP,        // Wait for lookup slot before prefetch launch
    PG_PF_BIU_WAIT       // Prefetch BIU read in progress
} pg_state_t;

pg_state_t state;
wire s_idle = (state == PG_IDLE);
assign pf_ack_bypass = (state == PG_PF_BIU_WAIT) && biu_req_complete;

wire idle_mem_req = s_idle && mem_req && !mem_servicing;
wire idle_pf_req = s_idle && pf_pending && !mem_req && !fast_path_pending && !mem_req_upcoming;
wire idle_mem_crossing = access_crosses_dword(mem_op_size, linear_addr[1:0]);
wire [31:0] idle_mem_phys = pg_enable ? tlb_live_physical_addr : linear_addr;
wire idle_tlb_dirty_ok = !is_write_access || tlb_live_dirty;
wire idle_can_translate = !pg_enable || (tlb_live_hit && idle_tlb_access_ok && idle_tlb_dirty_ok);
wire idle_perm_fault = pg_enable && tlb_live_hit && !idle_tlb_access_ok;
wire cross2_tlb_dirty_ok = !req_is_write || tlb_dirty;
wire cross2_can_translate = !pg_enable || (tlb_hit && slow_tlb_access_ok && cross2_tlb_dirty_ok);
wire pf_tlb_match = !pg_enable || (tlb_lookup_addr_r == pf_linear_addr);

reg        cache_lookup_r;
reg [31:0] cache_lookup_addr_r;
reg        cache_lookup_write_r;
reg        cache_lookup_cancel_r;
reg        lookup_cancel_pulse_r;
always_comb begin
    cache_lookup_r = 1'b0;
    cache_lookup_addr_r = 32'h0;
    cache_lookup_write_r = 1'b0;
    cache_lookup_cancel_r = lookup_cancel_pulse_r;

    case (state)
        PG_IDLE: begin
            if (idle_mem_req && !mem_is_io && !mem_is_inta && !mem_check_only) begin
                cache_lookup_r = 1'b1;
                cache_lookup_addr_r = linear_addr;
                cache_lookup_write_r = mem_write;
            end else if (idle_pf_req && (!pg_enable || pf_tlb_match)) begin
                cache_lookup_r = 1'b1;
                cache_lookup_addr_r = pf_linear_addr;
            end
        end

        PG_WALKING: begin
            if (walk_done && !walk_fault && !req_check_only) begin
                cache_lookup_r = 1'b1;
                cache_lookup_addr_r = req_linear;
                cache_lookup_write_r = req_is_write;
            end else if (walker_issue_ready) begin
                cache_lookup_r = 1'b1;
                cache_lookup_addr_r = walker_mem_addr;
                cache_lookup_write_r = walker_mem_wr;
            end
        end

        PG_CROSS_TLB2: begin
            if (!req_is_io && !req_check_only) begin
                cache_lookup_r = 1'b1;
                cache_lookup_addr_r = req_linear2;
                cache_lookup_write_r = req_is_write;
            end
        end

        PG_CROSS_WALK2: begin
            if (walk_done && !walk_fault && !req_check_only) begin
                cache_lookup_r = 1'b1;
                cache_lookup_addr_r = req_linear2;
                cache_lookup_write_r = req_is_write;
            end else if (walker_issue_ready) begin
                cache_lookup_r = 1'b1;
                cache_lookup_addr_r = walker_mem_addr;
                cache_lookup_write_r = walker_mem_wr;
            end
        end

        PG_WALK_LOOKUP: begin
            cache_lookup_r = 1'b1;
            cache_lookup_addr_r = req_linear;
            cache_lookup_write_r = req_is_write;
        end

        PG_CROSS_LOOKUP2: begin
            cache_lookup_r = 1'b1;
            cache_lookup_addr_r = req_linear2;
            cache_lookup_write_r = req_is_write;
        end

        PG_PF_WALKING: begin
            if (walk_done && !walk_fault) begin
                cache_lookup_r = 1'b1;
                cache_lookup_addr_r = pf_linear_addr;
            end else if (walker_issue_ready) begin
                cache_lookup_r = 1'b1;
                cache_lookup_addr_r = walker_mem_addr;
                cache_lookup_write_r = walker_mem_wr;
            end
        end

        PG_PF_LOOKUP: begin
            cache_lookup_r = 1'b1;
            cache_lookup_addr_r = pf_linear_addr;
        end

        default: begin
        end
    endcase
end
assign cache_lookup = cache_lookup_r;
assign cache_lookup_addr = cache_lookup_addr_r;
assign cache_lookup_write = cache_lookup_write_r;
assign cache_lookup_cancel = cache_lookup_cancel_r;
wire cache_lookup_granted = cache_lookup_r && cache_lookup_ready;

// Keep the registered TLB lookup address on a dedicated write-enable path.
// Capture the idle linear/prefetch address as soon as the request is pending,
// even if the fast path ends up using the combinational lookup directly. This
// keeps the address register independent of TLB/cache hit logic.
wire idle_mem_lookup_capture = idle_mem_req && !mem_is_io && !mem_is_inta;
wire idle_pf_lookup_capture = pg_enable && idle_pf_req && !pf_tlb_match;
wire walk_cross_lookup_load = (state == PG_WALKING) &&
                              walk_done && !walk_fault &&
                              req_check_only && req_crossing;
wire cross2_lookup_load = (state == PG_CROSS_PREP2);

logic        tlb_lookup_addr_load;
logic [31:0] tlb_lookup_addr_next;
always_comb begin
    tlb_lookup_addr_load = 1'b0;
    tlb_lookup_addr_next = tlb_lookup_addr_r;

    if (idle_mem_lookup_capture) begin
        tlb_lookup_addr_load = 1'b1;
        tlb_lookup_addr_next = linear_addr;
    end else if (idle_pf_lookup_capture) begin
        tlb_lookup_addr_load = 1'b1;
        tlb_lookup_addr_next = pf_linear_addr;
    end else if (walk_cross_lookup_load || cross2_lookup_load) begin
        tlb_lookup_addr_load = 1'b1;
        tlb_lookup_addr_next = req_linear2;
    end
end

always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n)
        tlb_lookup_addr_r <= 32'h0;
    else if (tlb_lookup_addr_load)
        tlb_lookup_addr_r <= tlb_lookup_addr_next;
end

// TLB update on successful page walk
always_comb begin
    tlb_update_valid = walk_done && !walk_fault;
    tlb_update_vpn = tlb_lookup_addr[31:12];
    tlb_update_pfn = walk_result_pfn;
    tlb_update_writable = walk_result_writable;
    tlb_update_user = walk_result_user;
    tlb_update_dirty = walk_result_dirty;
    tlb_update_accessed = walk_result_accessed;
end

function automatic [1:0] op_size_bytes_m1(input [1:0] op_size);
    op_size_bytes_m1 = (op_size == 2'd0) ? 2'd0 :
                       (op_size == 2'd1) ? 2'd1 : 2'd3;
endfunction

// Compute number of bytes in first half of crossing access
// Returns bytes before DWORD boundary
function automatic [1:0] first_half_bytes(input [1:0] offset);
    // Bytes remaining in current DWORD = 4 - offset
    // Since we only call this when crossing is true, first half is always (4 - offset)
    first_half_bytes = 2'd0 - offset;  // 4 - offset (2-bit wrap gives correct value)
endfunction

// Compute number of bytes in second half of crossing access
function automatic [1:0] second_half_bytes(input [1:0] offset, input [1:0] op_size);
    logic [2:0] total = (op_size == 2'd0) ? 3'd1 : (op_size == 2'd1) ? 3'd2 : 3'd4;
    logic [2:0] first = {1'b0, first_half_bytes(offset)};
    second_half_bytes = total[1:0] - first[1:0];
endfunction

task automatic write_opr_r_bytes(
    input [31:0] din,
    input [1:0]  phys_low,     // Physical address [1:0] for byte extraction
    input [1:0]  opr_offset,   // Where in OPR_R to write
    input [1:0]  opr_bytes     // Number of bytes - 1
);
    // First extract the relevant bytes from bus_din based on physical address alignment
    logic [31:0] extracted;
    case (phys_low)
        2'b00: extracted = din;
        2'b01: extracted = {8'h0, din[31:8]};
        2'b10: extracted = {16'h0, din[31:16]};
        2'b11: extracted = {24'h0, din[31:24]};
    endcase

    // Now place extracted bytes into OPR_R at opr_offset
    case (opr_offset)
        2'd0: begin
            // A read starting at OPR_R byte 0 should zero-extend the fetched
            // fragment. Leaving upper bytes untouched leaks stale data into
            // 8/16-bit operands such as far pointers and selectors.
            OPR_R <= extracted;
        end
        2'd1: begin
            case (opr_bytes)
                2'd0: OPR_R[15:8]  <= extracted[7:0];    // 1 byte at [1]
                2'd1: OPR_R[23:8]  <= extracted[15:0];   // 2 bytes at [1:2]
                2'd2: OPR_R[31:8]  <= extracted[23:0];   // 3 bytes at [1:3]
                default: ;
            endcase
        end
        2'd2: begin
            case (opr_bytes)
                2'd0: OPR_R[23:16] <= extracted[7:0];    // 1 byte at [2]
                2'd1: OPR_R[31:16] <= extracted[15:0];   // 2 bytes at [2:3]
                default: ;
            endcase
        end
        2'd3: begin
            OPR_R[31:24] <= extracted[7:0];               // 1 byte at [3]
        end
    endcase
endtask

//=============================================================================
// Main State Machine
//=============================================================================
always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state <= PG_IDLE;
        mem_servicing <= 1'b0;
        mem_accepted <= 1'b0;
        lookup_cancel_pulse_r <= 1'b0;
        pf_ack_toggle_r <= 1'b0;
        pf_rdata_r <= 32'h0;
        biu_req_valid <= 1'b0;
        biu_req_is_io <= 1'b0;
        biu_req_is_inta <= 1'b0;
        page_fault <= 1'b0;
        pf_fault <= 1'b0;
        fault_code <= 3'b000;
        rd_ind_active <= 1'b0;
        req_linear <= 32'h0;
        req_op_size <= 2'b0;
        req_is_write <= 1'b0;
        req_wdata <= 32'h0;
        req_cpl <= 2'b0;
        req_offset <= 2'b0;
        req_crossing <= 1'b0;
        req_linear2 <= 32'h0;
        req_check_only <= 1'b0;
        cr2_reg <= 32'h0;
        walk_request <= 1'b0;
        walk_biu_pending <= 1'b0;
        OPR_R <= 32'h0;
        opr_offset_r <= 2'b0;
        opr_bytes_r <= 2'b0;
        opr_is_write_r <= 1'b0;
        opr_suppress_r <= 1'b0;
        opr_phys_low_r <= 2'b0;
        opr_is_walk_r <= 1'b0;
        opr_is_pf_r <= 1'b0;
        fast_path_pending <= 1'b0;
        biu_req_phys_addr <= 32'h0;
        biu_req_write <= 1'b0;
        biu_req_be <= 4'h0;
        biu_req_wdata <= 32'h0;
    end else begin
        // Default: clear one-shot signals
        if (biu_req_accepted) begin
            biu_req_valid <= 1'b0;
            biu_req_is_io <= 1'b0;
            biu_req_is_inta <= 1'b0;
        end
        page_fault <= 1'b0;
        pf_fault <= 1'b0;
        fault_code <= 3'b000;
        walk_request <= 1'b0;
        lookup_cancel_pulse_r <= 1'b0;
        mem_accepted <= 1'b0;

        // Clear walker pending on completion
        if (biu_req_complete && walk_biu_pending)
            walk_biu_pending <= 1'b0;

        //=====================================================================
        // BIU completion: write OPR_R / pf_rdata / walker data
        //=====================================================================
        if (biu_req_complete) begin
            if (opr_is_pf_r) begin
                // Prefetch completion: handled in state machine (PG_PF_BIU_WAIT)
            end else if (opr_is_walk_r) begin
                // Walker: raw data routed to walker via biu_rdata (combinational)
                // synthesis translate_off
                if (TRACE_PAGING_EN)
                    $display("BIU WALK DONE: data=%08x", biu_rdata);
                // synthesis translate_on
            end else if (!opr_is_write_r && !opr_suppress_r) begin
                // Memory/IO/INTA read: byte-lane extraction (suppressed for first INTA dummy)
                write_opr_r_bytes(biu_rdata, opr_phys_low_r, opr_offset_r, opr_bytes_r);
                // synthesis translate_off
                if (TRACE_MEM_EN)
                    $display("PG MEM RD done: din=%08x phys_low=%0d offset=%0d bytes=%0d",
                             biu_rdata, opr_phys_low_r, opr_offset_r, opr_bytes_r + 1);
                // synthesis translate_on
            end

            // Clear fast_path_pending and ack mem toggle for non-crossing fast path
            if (fast_path_pending) begin
                fast_path_pending <= 1'b0;
                rd_ind_active <= 1'b0;
                mem_servicing <= 1'b0;
            end
        end

        case (state)
            PG_IDLE: begin
                if (idle_mem_req) begin
                    if (mem_is_io || mem_is_inta) begin
                        automatic logic io_crossing = mem_is_io && access_crosses_dword(mem_op_size, linear_addr[1:0]);
                        mem_accepted <= 1'b1;
                        mem_servicing <= 1'b1;
                        if (!io_crossing) begin
                            // IO/INTA fast path: no translation, no crossing, no state change
                            biu_req_valid <= 1'b1;
                            biu_req_phys_addr <= linear_addr;
                            biu_req_write <= mem_write;
                            biu_req_be <= mem_be;
                            biu_req_wdata <= shift_write_data(mem_wdata, mem_op_size, linear_addr[1:0]);
                            biu_req_is_io <= mem_is_io;
                            biu_req_is_inta <= mem_is_inta;
                            // First INTA cycle (addr=4) is dummy — suppress OPR_R update.
                            // Second INTA (addr=0) delivers the vector to OPR_R.
                            latch_biu_meta(2'd0, op_size_bytes_m1(mem_op_size), mem_write,
                                           linear_addr[1:0],
                                           mem_is_inta && (linear_addr[2:0] == 3'd4),
                                           1'b0, 1'b0);
                            rd_ind_active <= 1'b0;
                            fast_path_pending <= 1'b1;
                        end else begin
                            // IO crossing: split into two DWORD-aligned bus cycles
                            latch_mem_request(linear_addr, 1'b1);
                            req_is_io <= 1'b1;
                            rd_ind_active <= 1'b0;
                            // Emit first half
                            biu_req_valid <= 1'b1;
                            biu_req_phys_addr <= linear_addr;
                            biu_req_write <= mem_write;
                            biu_req_be <= calc_be_first(mem_op_size, linear_addr[1:0]);
                            biu_req_wdata <= split_write_first(mem_wdata, linear_addr[1:0], mem_op_size);
                            biu_req_is_io <= 1'b1;
                            biu_req_is_inta <= 1'b0;
                            latch_biu_meta(2'd0, first_half_bytes(linear_addr[1:0]) - 2'd1,
                                           mem_write, linear_addr[1:0], 1'b0, 1'b0, 1'b0);
                            state <= PG_CROSS_WAIT1;
                        end
                    end else begin
                        // Memory request: TLB lookup + crossing detection
                        // synthesis translate_off
                        if (TRACE_PAGING_EN)
                            $display("PG_UNIT: req linear=%08x size=%0d wr=%0d crossing=%0d fast=%0d",
                                     linear_addr, mem_op_size, mem_write,
                                     idle_mem_crossing, idle_can_translate && !idle_mem_crossing);
                        // synthesis translate_on

                        if (idle_perm_fault) begin
                            // Permission fault - no bus op, ack immediately
                            mem_accepted <= 1'b1;
                            lookup_cancel_pulse_r <= 1'b1;
                            raise_perm_fault(linear_addr, is_write_access, (cpl == 2'd3));

                        end else if (idle_can_translate && !idle_mem_crossing) begin
                            // FAST PATH: non-crossing, translation available
                            if (mem_check_only) begin
                                // CW: permission check passed, ack immediately
                                mem_accepted <= 1'b1;
                                complete_mem_request();
                            end else if (cache_lookup_granted) begin
                                // Emit directly, stay in PG_IDLE, ack on biu_req_complete
                                mem_accepted <= 1'b1;
                                mem_servicing <= 1'b1;
                                biu_req_valid <= 1'b1;
                                biu_req_phys_addr <= idle_mem_phys;
                                biu_req_write <= mem_write;
                                // Paging only translates bits [31:12]; bits [1:0] are always
                                // linear_addr[1:0]. Using it directly removes TLB from the
                                // calc_be/shift_write_data critical path.
                                biu_req_be <= mem_be;
                                biu_req_wdata <= shift_write_data(mem_wdata, mem_op_size, linear_addr[1:0]);
                                biu_req_is_io <= 1'b0;
                                rd_ind_active <= mem_rd_ind;
                                latch_biu_meta(2'd0, op_size_bytes_m1(mem_op_size), mem_write,
                                               linear_addr[1:0], 1'b0, 1'b0, 1'b0);
                                fast_path_pending <= 1'b1;
                            end
                            // synthesis translate_off
                            if (TRACE_PAGING_EN)
                                $display("PG_UNIT FAST: phys=%08x be=%04b wr=%0d check=%0d",
                                         idle_mem_phys, calc_be(mem_op_size, idle_mem_phys[1:0]), mem_write, mem_check_only);
                            // synthesis translate_on

                        end else if (idle_can_translate && idle_mem_crossing) begin
                            // Crossing with translation available for first half
                            if (mem_check_only) begin
                                mem_accepted <= 1'b1;
                                mem_servicing <= 1'b1;
                                latch_mem_request(linear_addr, idle_mem_crossing);
                                rd_ind_active <= mem_rd_ind;
                                state <= PG_CROSS_TLB2;
                            end else if (cache_lookup_granted) begin
                                mem_accepted <= 1'b1;
                                mem_servicing <= 1'b1;
                                latch_mem_request(linear_addr, idle_mem_crossing);
                                rd_ind_active <= mem_rd_ind;
                                biu_req_valid <= 1'b1;
                                biu_req_phys_addr <= idle_mem_phys;
                                biu_req_write <= mem_write;
                                biu_req_be <= calc_be_first(mem_op_size, linear_addr[1:0]);
                                biu_req_wdata <= split_write_first(mem_wdata, linear_addr[1:0], mem_op_size);
                                biu_req_is_io <= 1'b0;
                                latch_biu_meta(2'd0, first_half_bytes(linear_addr[1:0]) - 2'd1,
                                               mem_write, linear_addr[1:0], 1'b0, 1'b0, 1'b0);
                                state <= PG_CROSS_WAIT1;
                            end
                            // synthesis translate_off
                            if (TRACE_PAGING_EN)
                                $display("PG_UNIT CROSS FAST: phys=%08x be=%04b",
                                         idle_mem_phys, calc_be_first(mem_op_size, linear_addr[1:0]));
                            // synthesis translate_on

                        end else begin
                            // TLB miss - need page walk
                            mem_accepted <= 1'b1;
                            mem_servicing <= 1'b1;
                            lookup_cancel_pulse_r <= 1'b1;
                            latch_mem_request(linear_addr, idle_mem_crossing);
                            rd_ind_active <= mem_rd_ind;
                            walk_request <= 1'b1;
                            state <= PG_WALKING;
                        end
                    end

                end else if (idle_pf_req) begin
                    // Prefetch request (lower priority than mem/IO)
                    if (!pg_enable) begin
                        if (cache_lookup_granted) begin
                            // Paging disabled: emit directly to BIU
                            emit_pf_biu_req(pf_linear_addr);
                            state <= PG_PF_BIU_WAIT;
                        end
                    end else if (pf_tlb_match && tlb_hit) begin
                        // TLB hit - check permission (code fetch: read at current CPL)
                        // Prefetch is always supervisor read, only user check matters
                        if (pf_tlb_user_ok) begin
                            if (cache_lookup_granted) begin
                                emit_pf_biu_req(tlb_physical_addr);
                                state <= PG_PF_BIU_WAIT;
                            end
                        end else begin
                            // Permission fail: silently fault, ack prefetch
                            lookup_cancel_pulse_r <= 1'b1;
                            ack_prefetch_fault();
                        end
                    end else if (pf_tlb_match) begin
                        // TLB miss: start page walk for prefetch
                        // For prefetch walks, use supervisor read permissions
                        lookup_cancel_pulse_r <= 1'b1;
                        req_is_write <= 1'b0;
                        req_cpl <= cpl;
                        walk_request <= 1'b1;
                        state <= PG_PF_WALKING;
                    end
                end
            end

            PG_WALKING: begin
                if (walker_issue_ready && cache_lookup_granted)
                    emit_walker_biu_req();

                if (walk_done) begin
                    if (walk_fault) begin
                        raise_walk_fault(req_linear, walk_fault_code);
                    end else if (req_check_only) begin
                        if (req_crossing) begin
                            state <= PG_CROSS_TLB2;
                        end else begin
                            complete_mem_request();
                        end
                    end else if (cache_lookup_granted) begin
                        automatic logic [31:0] phys = {walk_result_pfn, req_linear[11:0]};
                        if (req_crossing) begin
                            emit_first_half(phys);
                            state <= PG_CROSS_WAIT1;
                        end else begin
                            emit_single(phys);
                            // Don't ack yet - ack on biu_req_complete via fast_path_pending
                            fast_path_pending <= 1'b1;
                            state <= PG_IDLE;
                        end
                    end else begin
                        state <= PG_WALK_LOOKUP;
                    end
                end
            end

            PG_WALK_LOOKUP: begin
                if (cache_lookup_granted) begin
                    automatic logic [31:0] phys = {walk_result_pfn, req_linear[11:0]};
                    if (req_crossing) begin
                        emit_first_half(phys);
                        state <= PG_CROSS_WAIT1;
                    end else begin
                        emit_single(phys);
                        fast_path_pending <= 1'b1;
                        state <= PG_IDLE;
                    end
                end
            end

            PG_CROSS_WAIT1: begin
                if (biu_req_complete) begin
                    state <= PG_CROSS_PREP2;
                end
            end

            PG_CROSS_PREP2: begin
                // Break the same-cycle cache-complete -> next TLB lookup enable path.
                state <= PG_CROSS_TLB2;
            end

            PG_CROSS_TLB2: begin
                if (req_is_io) begin
                    // IO crossing: no TLB needed, emit second half directly
                    emit_second_half(req_linear2);
                    biu_req_is_io <= 1'b1;
                    state <= PG_CROSS_WAIT2;
                end else if (!pg_enable) begin
                    if (req_check_only) begin
                        complete_mem_request();
                    end else if (cache_lookup_granted) begin
                        emit_second_half(req_linear2);
                        state <= PG_CROSS_WAIT2;
                    end
                end else if (tlb_hit && slow_tlb_access_ok && (!req_is_write || tlb_dirty)) begin
                    if (req_check_only) begin
                        complete_mem_request();
                    end else if (cache_lookup_granted) begin
                        emit_second_half(tlb_physical_addr);
                        state <= PG_CROSS_WAIT2;
                    end
                end else if (tlb_hit && !slow_tlb_access_ok) begin
                    lookup_cancel_pulse_r <= 1'b1;
                    raise_perm_fault(req_linear2, req_is_write, slow_is_user_mode);
                end else begin
                    lookup_cancel_pulse_r <= 1'b1;
                    walk_request <= 1'b1;
                    state <= PG_CROSS_WALK2;
                end
            end

            PG_CROSS_WALK2: begin
                if (walker_issue_ready && cache_lookup_granted)
                    emit_walker_biu_req();

                if (walk_done) begin
                    if (walk_fault) begin
                        raise_walk_fault(req_linear2, walk_fault_code);
                    end else if (req_check_only) begin
                        complete_mem_request();
                    end else if (cache_lookup_granted) begin
                        automatic logic [31:0] phys2 = {walk_result_pfn, req_linear2[11:0]};
                        emit_second_half(phys2);
                        state <= PG_CROSS_WAIT2;
                    end else begin
                        state <= PG_CROSS_LOOKUP2;
                    end
                end
            end

            PG_CROSS_LOOKUP2: begin
                if (cache_lookup_granted) begin
                    automatic logic [31:0] phys2 = {walk_result_pfn, req_linear2[11:0]};
                    emit_second_half(phys2);
                    state <= PG_CROSS_WAIT2;
                end
            end

            PG_CROSS_WAIT2: begin
                if (biu_req_complete)
                    complete_mem_request();
            end

            PG_PF_WALKING: begin
                if (walker_issue_ready && cache_lookup_granted)
                    emit_walker_biu_req();

                if (walk_done) begin
                    if (walk_fault) begin
                        // Prefetch page fault: silently ack with fault flag
                        ack_prefetch_fault();
                        state <= PG_IDLE;
                    end else if (cache_lookup_granted) begin
                        // Walk succeeded, emit BIU request with translated address
                        automatic logic [31:0] pf_phys = {walk_result_pfn, pf_linear_addr[11:0]};
                        emit_pf_biu_req(pf_phys);
                        state <= PG_PF_BIU_WAIT;
                    end else begin
                        state <= PG_PF_LOOKUP;
                    end
                end
            end

            PG_PF_LOOKUP: begin
                if (cache_lookup_granted) begin
                    automatic logic [31:0] pf_phys = {walk_result_pfn, pf_linear_addr[11:0]};
                    emit_pf_biu_req(pf_phys);
                    state <= PG_PF_BIU_WAIT;
                end
            end

            PG_PF_BIU_WAIT: begin
                if (biu_req_complete) begin
                    pf_rdata_r <= biu_rdata;           // latch read data
                    pf_ack_toggle_r <= ~pf_ack_toggle_r; // registered ack
                    state <= PG_IDLE;
                end
            end

            default: state <= PG_IDLE;
        endcase
    end
end

// Latch mem request parameters (shared by crossing and TLB miss paths)
task automatic latch_mem_request(input [31:0] addr, input crossing);
    req_linear <= addr;
    req_op_size <= mem_op_size;
    req_is_write <= is_write_access;
    req_wdata <= mem_wdata;
    req_cpl <= cpl;
    req_offset <= addr[1:0];
    req_crossing <= crossing;
    req_check_only <= mem_check_only;
    req_linear2 <= {addr[31:2] + 30'd1, 2'b00};
    req_is_io <= 1'b0;
endtask

task automatic latch_biu_meta(
    input [1:0] opr_offset,
    input [1:0] opr_bytes,
    input       is_write,
    input [1:0] phys_low,
    input       suppress,
    input       is_walk,
    input       is_pf
);
    opr_offset_r <= opr_offset;
    opr_bytes_r <= opr_bytes;
    opr_is_write_r <= is_write;
    opr_suppress_r <= suppress;
    opr_phys_low_r <= phys_low;
    opr_is_walk_r <= is_walk;
    opr_is_pf_r <= is_pf;
endtask

task automatic complete_mem_request();
    rd_ind_active <= 1'b0;
    mem_servicing <= 1'b0;
    state <= PG_IDLE;
endtask

task automatic raise_perm_fault(
    input [31:0] fault_addr,
    input        fault_is_write,
    input        fault_is_user
);
    cr2_reg <= fault_addr;
    page_fault <= 1'b1;
    fault_code <= {fault_is_user, fault_is_write, 1'b1};
    complete_mem_request();
endtask

task automatic raise_walk_fault(
    input [31:0] fault_addr,
    input [2:0]  walk_code
);
    cr2_reg <= fault_addr;
    page_fault <= 1'b1;
    fault_code <= walk_code;
    complete_mem_request();
endtask

task automatic ack_prefetch_fault();
    pf_fault <= 1'b1;
    pf_ack_toggle_r <= ~pf_ack_toggle_r;
endtask

task automatic emit_walker_biu_req();
    biu_req_valid <= 1'b1;
    biu_req_phys_addr <= walker_mem_addr;
    biu_req_write <= walker_mem_wr;
    biu_req_be <= 4'b1111;
    biu_req_wdata <= walker_mem_wdata;
    biu_req_is_io <= 1'b0;
    biu_req_is_inta <= 1'b0;
    latch_biu_meta(2'd0, 2'd0, 1'b0, 2'b00, 1'b0, 1'b1, 1'b0);
    walk_biu_pending <= 1'b1;
endtask

// Emit a single non-crossing request
task automatic emit_single(input [31:0] phys_addr);
    biu_req_valid <= 1'b1;
    biu_req_phys_addr <= phys_addr;
    biu_req_write <= req_is_write;
    // req_offset == phys_addr[1:0] (paging preserves bits [11:0])
    biu_req_be <= calc_be(req_op_size, req_offset);
    biu_req_wdata <= shift_write_data(req_wdata, req_op_size, req_offset);
    biu_req_is_io <= 1'b0;
    biu_req_is_inta <= 1'b0;
    latch_biu_meta(2'd0, op_size_bytes_m1(req_op_size), req_is_write,
                   req_offset, 1'b0, 1'b0, 1'b0);
    // synthesis translate_off
    if (TRACE_PAGING_EN)
        $display("PG_UNIT EMIT SINGLE: phys=%08x be=%04b wr=%0d",
                 phys_addr, calc_be(req_op_size, req_offset), req_is_write);
    // synthesis translate_on
endtask

// Emit first half of a crossing request
task automatic emit_first_half(input [31:0] phys_addr);
    biu_req_valid <= 1'b1;
    biu_req_phys_addr <= phys_addr;
    biu_req_write <= req_is_write;
    biu_req_be <= calc_be_first(req_op_size, req_offset);
    biu_req_wdata <= split_write_first(req_wdata, req_offset, req_op_size);
    biu_req_is_io <= 1'b0;
    biu_req_is_inta <= 1'b0;
    latch_biu_meta(2'd0, first_half_bytes(req_offset) - 2'd1,
                   req_is_write, req_offset, 1'b0, 1'b0, 1'b0);
    // synthesis translate_off
    if (TRACE_PAGING_EN)
        $display("PG_UNIT EMIT FIRST: phys=%08x be=%04b wr=%0d offset=%0d",
                 phys_addr, calc_be_first(req_op_size, req_offset), req_is_write, req_offset);
    // synthesis translate_on
endtask

// Emit second half of a crossing request
task automatic emit_second_half(input [31:0] phys_addr);
    automatic logic [1:0] fb = first_half_bytes(req_offset);
    biu_req_valid <= 1'b1;
    biu_req_phys_addr <= phys_addr;
    biu_req_write <= req_is_write;
    biu_req_be <= calc_be_second(req_op_size, req_offset);
    biu_req_wdata <= split_write_second(req_wdata, req_offset, req_op_size);
    biu_req_is_io <= 1'b0;
    biu_req_is_inta <= 1'b0;
    latch_biu_meta(fb, second_half_bytes(req_offset, req_op_size) - 2'd1,
                   req_is_write, 2'b00, 1'b0, 1'b0, 1'b0);
    // synthesis translate_off
    if (TRACE_PAGING_EN)
        $display("PG_UNIT EMIT SECOND: phys=%08x be=%04b wr=%0d opr_offset=%0d",
                 phys_addr, calc_be_second(req_op_size, req_offset), req_is_write, fb);
    // synthesis translate_on
endtask

// Emit prefetch BIU request (always DWORD read, no crossing)
task automatic emit_pf_biu_req(input [31:0] phys_addr);
    biu_req_valid <= 1'b1;
    biu_req_phys_addr <= phys_addr;
    biu_req_write <= 1'b0;
    biu_req_be <= 4'b1111;
    biu_req_wdata <= 32'h0;
    biu_req_is_io <= 1'b0;
    biu_req_is_inta <= 1'b0;
    latch_biu_meta(2'd0, 2'd0, 1'b0, 2'b00, 1'b0, 1'b0, 1'b1);
endtask

endmodule
