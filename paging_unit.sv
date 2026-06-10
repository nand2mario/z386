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
    input               mem_lookup_valid,  // Address capture hint before segment-fault gating
    input               mem_req_upcoming,  // Combinational early hint: suppresses prefetch start
    output logic        mem_accepted,      // Ready: demand request may be handed off this cycle
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
    input               pf_redirect_queued,// Redirect request queued behind current prefetch
    output      [127:0] pf_rdata,          // Cache line returned to prefetch
    output reg          pf_fault,          // Page fault (silently drop, suspend)

    //=========================================================================
    // Demand-side physical request interface
    //=========================================================================
    output logic        dcache_req_valid,     // Demand/page-walk/IO request valid
    (* syn_replicate = 1 *)
    output logic [31:0] dcache_req_phys_addr, // Physical address (full 32-bit)
    output logic        dcache_req_write,     // 1=write
    output logic [3:0]  dcache_req_be,        // Byte enables (pre-computed)
    output logic [31:0] dcache_req_wdata,     // Write data (pre-positioned on bus)
    output logic        dcache_req_is_io,     // Request is IO space
    output logic        dcache_req_is_inta,   // Request is INTA cycle

    input               dcache_req_accepted,  // Demand-side request accepted this cycle
    input               dcache_req_complete,  // Demand-side request complete
    input        [31:0] dcache_rdata,         // Demand-side read data

    //=========================================================================
    // Instruction-prefetch physical request interface
    //=========================================================================
    output logic        icache_req_valid,     // Prefetch request valid
    output logic [31:0] icache_req_phys_addr, // Prefetch physical address
    input               icache_req_accepted,  // Icache accepted request this cycle
    input               icache_req_complete,  // Icache read complete
    input       [127:0] icache_rdata,         // Icache read line

    output reg   [31:0] OPR_R,

    output reg          page_fault,        // Page fault occurred (mem/IO requests only)
    output reg   [2:0]  fault_code,        // Error code for page fault
    output reg   [31:0] cr2_out,           // Faulting address (written to CR2)

    // rd_ind pass-through
    output reg          rd_ind_active      // BUSOP_RD_IND is active for this request
);

// Control register bits
wire pg_enable = cr0[31];   // PG - Paging enable
wire wp_enable = cr0[16];   // WP - Write protect

// Compile-time-off sim trace flags to keep hot scheduler paths free of
// per-cycle $test$plusargs overhead under Verilator.
localparam bit TRACE_MEM_EN    = 1'b0;
localparam bit TRACE_PAGING_EN = 1'b0;

reg pf_ack_toggle_r;
reg [127:0] pf_rdata_r;       // Registered read line for prefetch
wire pf_ack_bypass;
assign pf_ack_toggle = pf_ack_toggle_r ^ pf_ack_bypass;
assign pf_rdata = pf_ack_bypass ? icache_rdata : pf_rdata_r;

wire pf_pending  = (pf_req_toggle != pf_ack_toggle_r);

//=============================================================================
// TLB Interface
//=============================================================================
wire        tlb_hit;
wire [31:0] tlb_physical_addr;
wire        tlb_writable;
wire        tlb_user;
wire        tlb_dirty;
// TLB update signals (from page walker)
logic        tlb_update_valid;
logic [19:0] tlb_update_vpn;
logic [19:0] tlb_update_pfn;
logic        tlb_update_writable;
logic        tlb_update_user;
logic        tlb_update_dirty;
logic        tlb_update_accessed;

//=============================================================================
// State Machine
//=============================================================================
typedef enum logic [3:0] {
    PG_IDLE,
    PG_MEM_TLB,         // Registered demand-memory TLB/permission cycle
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

// TLB lookup address for prefetch/walker and other registered slow paths.
wire [31:0] tlb_lookup_addr;
reg  [31:0] tlb_lookup_addr_r;
assign tlb_lookup_addr = tlb_lookup_addr_r;

wire s_idle = (state == PG_IDLE);
wire idle_mem_req = s_idle && mem_req && !mem_servicing;
wire idle_mem_lookup_req = s_idle && mem_lookup_valid && !mem_servicing;
// P0/P1 prefetch timing:
//   P0 prefetch toggles pf_req_toggle and presents pf_linear_addr.
//   P1 paging translates the registered prefetch address and launches icache.
// Demand memory still has priority through mem_req_upcoming, which is generated
// before segment-fault masking. Do not also gate on mem_req here, since mem_req
// includes same-cycle fault suppression and would route EA/segmentation fault
// logic into the icache launch path.
wire idle_pf_req = s_idle && pf_pending && !fast_path_pending && !mem_req_upcoming;

paging_tlb tlb_inst (
    .clk            (clk),
    .reset_n        (reset_n),
    .linear_addr    (tlb_lookup_addr),
    .hit            (tlb_hit),
    .physical_addr  (tlb_physical_addr),
    .writable       (tlb_writable),
    .user           (tlb_user),
    .dirty          (tlb_dirty),
    .linear_addr_live(32'h0),
    .live_hit       (),
    .live_physical_addr(),
    .live_writable  (),
    .live_user      (),
    .live_dirty     (),
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

reg        mem_accepted_r;
reg        dcache_req_valid_r;
reg [31:0] dcache_req_phys_addr_r;
reg        dcache_req_write_r;
reg [3:0]  dcache_req_be_r;
reg [31:0] dcache_req_wdata_r;
reg        dcache_req_is_io_r;
reg        dcache_req_is_inta_r;
reg        icache_req_valid_r;
reg [31:0] icache_req_phys_addr_r;

// Walker bus read/write tracking: prevents re-emission while op is in flight
reg walk_biu_pending;
wire walker_feed_ready = dcache_req_complete && walk_biu_pending;
wire walker_issue_ready = (walker_mem_rd || walker_mem_wr) && !walk_biu_pending && !dcache_req_valid;

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
    .mem_data       (dcache_rdata),
    .mem_ready      (walker_feed_ready)
);

// Permission Checking
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

// Fast path metadata (mem non-crossing emits directly from PG_IDLE)
reg        fast_path_pending; // A fast-path BIU request is in flight

// PIPT cache completion is deliberately registered through mem_servicing clear.
// Do not feed cache response/tag-compare timing back into the microsequencer.
assign mem_complete_now = 1'b0;

assign pf_ack_bypass = (state == PG_PF_BIU_WAIT) && icache_req_complete;

wire idle_mem_crossing = access_crosses_dword(mem_op_size, linear_addr[1:0]);
wire cache_lookup_granted = 1'b1;
wire idle_mem_ready = s_idle && !mem_servicing;
wire idle_mem_accept = idle_mem_req;
wire req_tlb_dirty_ok = !req_is_write || tlb_dirty;
wire req_can_translate = !pg_enable || (tlb_hit && slow_tlb_access_ok && req_tlb_dirty_ok);
wire req_perm_fault = pg_enable && tlb_hit && !slow_tlb_access_ok;
wire [31:0] req_mem_phys = pg_enable ? {tlb_physical_addr[31:12], req_linear[11:0]} : req_linear;
wire req_mem_dcache_candidate = (state == PG_MEM_TLB) && req_can_translate &&
                                !req_perm_fault && !req_check_only &&
                                cache_lookup_granted;
wire req_mem_dcache_accept = req_mem_dcache_candidate && dcache_req_accepted;
wire req_mem_posted_done = req_mem_dcache_accept && req_is_write && dcache_req_complete;
wire dcache_posted_write_done = dcache_req_valid && dcache_req_write &&
                                dcache_req_accepted && dcache_req_complete;
wire cross2_tlb_dirty_ok = !req_is_write || tlb_dirty;
wire cross2_can_translate = !pg_enable || (tlb_hit && slow_tlb_access_ok && cross2_tlb_dirty_ok);
// Prefetch only needs the registered TLB lookup to cover the same 4KB page.
// Sequential fetches usually advance by one DWORD, so exact-address matching
// would reload the TLB lookup register every request and add a frontend cycle.
wire pf_tlb_match = !pg_enable || (tlb_lookup_addr_r[31:12] == pf_linear_addr[31:12]);
wire fast_pf_candidate = idle_pf_req && cache_lookup_granted &&
                         (!pg_enable || (pf_tlb_match && tlb_hit && pf_tlb_user_ok));
wire [31:0] fast_pf_phys = pg_enable ? {tlb_physical_addr[31:12], pf_linear_addr[11:0]} : pf_linear_addr;

assign mem_accepted = mem_accepted_r || idle_mem_ready;
assign dcache_req_valid = dcache_req_valid_r || req_mem_dcache_candidate;
assign dcache_req_phys_addr = req_mem_dcache_candidate ? req_mem_phys : dcache_req_phys_addr_r;
assign dcache_req_write = req_mem_dcache_candidate ? req_is_write : dcache_req_write_r;
assign dcache_req_be = req_mem_dcache_candidate ?
                       (req_crossing ? calc_be_first(req_op_size, req_offset) :
                                       calc_be(req_op_size, req_offset)) :
                       dcache_req_be_r;
assign dcache_req_wdata = req_mem_dcache_candidate ?
                          (req_crossing ? split_write_first(req_wdata, req_offset, req_op_size) :
                                          shift_write_data(req_wdata, req_op_size, req_offset)) :
                          dcache_req_wdata_r;
assign dcache_req_is_io = req_mem_dcache_candidate ? 1'b0 : dcache_req_is_io_r;
assign dcache_req_is_inta = req_mem_dcache_candidate ? 1'b0 : dcache_req_is_inta_r;
assign icache_req_valid = icache_req_valid_r || fast_pf_candidate;
// Address is only consumed when icache_req_valid is high.  Use the registered
// slow-path address only when that request is live; otherwise present the fast
// prefetch address unconditionally so demand-memory arbitration does not become
// a mux-select path into the icache set read.
assign icache_req_phys_addr = icache_req_valid_r ? icache_req_phys_addr_r : fast_pf_phys;

// Keep the registered TLB lookup address on a dedicated write-enable path.
// Capture the idle linear/prefetch address as soon as the request is pending,
// even if the fast path ends up using the combinational lookup directly. This
// keeps the address register independent of TLB/cache hit logic.
wire idle_mem_lookup_capture = idle_mem_lookup_req && !mem_is_io && !mem_is_inta;
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
        mem_accepted_r <= 1'b0;
        pf_ack_toggle_r <= 1'b0;
        pf_rdata_r <= 128'h0;
        dcache_req_valid_r <= 1'b0;
        dcache_req_is_io_r <= 1'b0;
        dcache_req_is_inta_r <= 1'b0;
        icache_req_valid_r <= 1'b0;
        icache_req_phys_addr_r <= 32'h0;
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
        fast_path_pending <= 1'b0;
        dcache_req_phys_addr_r <= 32'h0;
        dcache_req_write_r <= 1'b0;
        dcache_req_be_r <= 4'h0;
        dcache_req_wdata_r <= 32'h0;
    end else begin
        // Default: clear one-shot signals
        if (dcache_req_accepted) begin
            dcache_req_valid_r <= 1'b0;
            dcache_req_is_io_r <= 1'b0;
            dcache_req_is_inta_r <= 1'b0;
        end
        if (icache_req_accepted)
            icache_req_valid_r <= 1'b0;
        page_fault <= 1'b0;
        pf_fault <= 1'b0;
        fault_code <= 3'b000;
        walk_request <= 1'b0;
        mem_accepted_r <= 1'b0;

        // Clear walker pending on completion
        if (dcache_req_complete && walk_biu_pending)
            walk_biu_pending <= 1'b0;

        //=====================================================================
        // BIU completion: write OPR_R / pf_rdata / walker data
        //=====================================================================
        if (dcache_req_complete) begin
            if (opr_is_walk_r) begin
                // Walker: raw data routed to walker via dcache_rdata (combinational)
                // synthesis translate_off
                if (TRACE_PAGING_EN)
                    $display("BIU WALK DONE: data=%08x", dcache_rdata);
                // synthesis translate_on
            end else if (!dcache_posted_write_done && !opr_is_write_r && !opr_suppress_r) begin
                // Memory/IO/INTA read: byte-lane extraction (suppressed for first INTA dummy)
                write_opr_r_bytes(dcache_rdata, opr_phys_low_r, opr_offset_r, opr_bytes_r);
                // synthesis translate_off
                if (TRACE_MEM_EN)
                    $display("PG MEM RD done: din=%08x phys_low=%0d offset=%0d bytes=%0d",
                             dcache_rdata, opr_phys_low_r, opr_offset_r, opr_bytes_r + 1);
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
                        mem_accepted_r <= 1'b1;
                        mem_servicing <= 1'b1;
                        if (!io_crossing) begin
                            // IO/INTA fast path: no translation, no crossing, no state change
                            dcache_req_valid_r <= 1'b1;
                            dcache_req_phys_addr_r <= linear_addr;
                            dcache_req_write_r <= mem_write;
                            dcache_req_be_r <= mem_be;
                            dcache_req_wdata_r <= shift_write_data(mem_wdata, mem_op_size, linear_addr[1:0]);
                            dcache_req_is_io_r <= mem_is_io;
                            dcache_req_is_inta_r <= mem_is_inta;
                            // First INTA cycle (addr=4) is dummy — suppress OPR_R update.
                            // Second INTA (addr=0) delivers the vector to OPR_R.
                            latch_biu_meta(2'd0, op_size_bytes_m1(mem_op_size), mem_write,
                                           linear_addr[1:0],
                                           mem_is_inta && (linear_addr[2:0] == 3'd4),
                                           1'b0);
                            rd_ind_active <= 1'b0;
                            fast_path_pending <= 1'b1;
                        end else begin
                            // IO crossing: split into two DWORD-aligned bus cycles
                            latch_mem_request(linear_addr, 1'b1);
                            req_is_io <= 1'b1;
                            rd_ind_active <= 1'b0;
                            // Emit first half
                            dcache_req_valid_r <= 1'b1;
                            dcache_req_phys_addr_r <= linear_addr;
                            dcache_req_write_r <= mem_write;
                            dcache_req_be_r <= calc_be_first(mem_op_size, linear_addr[1:0]);
                            dcache_req_wdata_r <= split_write_first(mem_wdata, linear_addr[1:0], mem_op_size);
                            dcache_req_is_io_r <= 1'b1;
                            dcache_req_is_inta_r <= 1'b0;
                            latch_biu_meta(2'd0, first_half_bytes(linear_addr[1:0]) - 2'd1,
                                           mem_write, linear_addr[1:0], 1'b0, 1'b0);
                            state <= PG_CROSS_WAIT1;
                        end
                    end else begin
                        // Memory request: RD only captures the linear request.
                        // The next DLY cycle uses tlb_lookup_addr_r for TLB and
                        // dcache launch, keeping EA/segment and TLB in separate
                        // cycles.
                        // synthesis translate_off
                        if (TRACE_PAGING_EN)
                            $display("PG_UNIT CAPTURE: linear=%08x size=%0d wr=%0d crossing=%0d",
                                     linear_addr, mem_op_size, mem_write, idle_mem_crossing);
                        // synthesis translate_on

                        mem_accepted_r <= 1'b1;
                        mem_servicing <= 1'b1;
                        latch_mem_request(linear_addr, idle_mem_crossing);
                        rd_ind_active <= mem_rd_ind;
                        state <= PG_MEM_TLB;
                    end

                end else if (idle_pf_req) begin
                    // Prefetch request (lower priority than mem/IO)
                    if (fast_pf_candidate) begin
                        if (icache_req_accepted)
                            state <= PG_PF_BIU_WAIT;
                    end else if (pg_enable && pf_tlb_match && tlb_hit) begin
                        // Permission fail: silently fault, ack prefetch.
                        ack_prefetch_fault();
                    end else if (pf_tlb_match) begin
                        // TLB miss: start page walk for prefetch
                        // For prefetch walks, use supervisor read permissions
                        req_is_write <= 1'b0;
                        req_cpl <= cpl;
                        walk_request <= 1'b1;
                        state <= PG_PF_WALKING;
                    end
                end
            end

            PG_MEM_TLB: begin
                if (req_perm_fault) begin
                    raise_perm_fault(req_linear, req_is_write, slow_is_user_mode);
                end else if (req_can_translate) begin
                    if (req_check_only) begin
                        if (req_crossing)
                            state <= PG_CROSS_TLB2;
                        else
                            complete_mem_request();
                    end else if (req_mem_dcache_accept) begin
                        if (req_crossing) begin
                            latch_biu_meta(2'd0, first_half_bytes(req_offset) - 2'd1,
                                           req_is_write, req_offset, 1'b0, 1'b0);
                            state <= req_mem_posted_done ? PG_CROSS_PREP2 : PG_CROSS_WAIT1;
                        end else begin
                            latch_biu_meta(2'd0, op_size_bytes_m1(req_op_size), req_is_write,
                                           req_offset, 1'b0, 1'b0);
                            if (req_mem_posted_done) begin
                                rd_ind_active <= 1'b0;
                                mem_servicing <= 1'b0;
                                fast_path_pending <= 1'b0;
                                state <= PG_IDLE;
                            end else begin
                                fast_path_pending <= 1'b1;
                                state <= PG_IDLE;
                            end
                        end
                    end
                end else begin
                    walk_request <= 1'b1;
                    state <= PG_WALKING;
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
                            // Don't ack yet - ack on dcache_req_complete via fast_path_pending
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
                if (dcache_req_complete) begin
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
                    dcache_req_is_io_r <= 1'b1;
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
                    raise_perm_fault(req_linear2, req_is_write, slow_is_user_mode);
                end else begin
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
                if (dcache_req_complete)
                    complete_mem_request();
            end

            PG_PF_WALKING: begin
                if (walker_issue_ready && cache_lookup_granted)
                    emit_walker_biu_req();

                if (walk_done) begin
                    if (pf_redirect_queued) begin
                        // q_flush canceled this in-flight prefetch while the page
                        // walk was active.  Acknowledge/drop the old request; the
                        // queued redirect will re-enter through PG_IDLE with its
                        // own TLB lookup/walk instead of reusing this walk result.
                        pf_ack_toggle_r <= ~pf_ack_toggle_r;
                        state <= PG_IDLE;
                    end else if (walk_fault) begin
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
                if (pf_redirect_queued) begin
                    // The active prefetch was canceled after its walk completed
                    // but before cache lookup could launch. Drop it and let the
                    // queued redirect become a fresh request.
                    pf_ack_toggle_r <= ~pf_ack_toggle_r;
                    state <= PG_IDLE;
                end else if (cache_lookup_granted) begin
                    automatic logic [31:0] pf_phys = {walk_result_pfn, pf_linear_addr[11:0]};
                    emit_pf_biu_req(pf_phys);
                    state <= PG_PF_BIU_WAIT;
                end
            end

            PG_PF_BIU_WAIT: begin
                if (icache_req_complete) begin
                    pf_rdata_r <= icache_rdata;           // latch read data
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
    input       is_walk
);
    opr_offset_r <= opr_offset;
    opr_bytes_r <= opr_bytes;
    opr_is_write_r <= is_write;
    opr_suppress_r <= suppress;
    opr_phys_low_r <= phys_low;
    opr_is_walk_r <= is_walk;
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
    dcache_req_valid_r <= 1'b1;
    dcache_req_phys_addr_r <= walker_mem_addr;
    dcache_req_write_r <= walker_mem_wr;
    dcache_req_be_r <= 4'b1111;
    dcache_req_wdata_r <= walker_mem_wdata;
    dcache_req_is_io_r <= 1'b0;
    dcache_req_is_inta_r <= 1'b0;
    latch_biu_meta(2'd0, 2'd0, 1'b0, 2'b00, 1'b0, 1'b1);
    walk_biu_pending <= 1'b1;
endtask

// Emit a single non-crossing request
task automatic emit_single(input [31:0] phys_addr);
    dcache_req_valid_r <= 1'b1;
    dcache_req_phys_addr_r <= phys_addr;
    dcache_req_write_r <= req_is_write;
    // req_offset == phys_addr[1:0] (paging preserves bits [11:0])
    dcache_req_be_r <= calc_be(req_op_size, req_offset);
    dcache_req_wdata_r <= shift_write_data(req_wdata, req_op_size, req_offset);
    dcache_req_is_io_r <= 1'b0;
    dcache_req_is_inta_r <= 1'b0;
    latch_biu_meta(2'd0, op_size_bytes_m1(req_op_size), req_is_write,
                   req_offset, 1'b0, 1'b0);
    // synthesis translate_off
    if (TRACE_PAGING_EN)
        $display("PG_UNIT EMIT SINGLE: phys=%08x be=%04b wr=%0d",
                 phys_addr, calc_be(req_op_size, req_offset), req_is_write);
    // synthesis translate_on
endtask

// Emit first half of a crossing request
task automatic emit_first_half(input [31:0] phys_addr);
    dcache_req_valid_r <= 1'b1;
    dcache_req_phys_addr_r <= phys_addr;
    dcache_req_write_r <= req_is_write;
    dcache_req_be_r <= calc_be_first(req_op_size, req_offset);
    dcache_req_wdata_r <= split_write_first(req_wdata, req_offset, req_op_size);
    dcache_req_is_io_r <= 1'b0;
    dcache_req_is_inta_r <= 1'b0;
    latch_biu_meta(2'd0, first_half_bytes(req_offset) - 2'd1,
                   req_is_write, req_offset, 1'b0, 1'b0);
    // synthesis translate_off
    if (TRACE_PAGING_EN)
        $display("PG_UNIT EMIT FIRST: phys=%08x be=%04b wr=%0d offset=%0d",
                 phys_addr, calc_be_first(req_op_size, req_offset), req_is_write, req_offset);
    // synthesis translate_on
endtask

// Emit second half of a crossing request
task automatic emit_second_half(input [31:0] phys_addr);
    automatic logic [1:0] fb = first_half_bytes(req_offset);
    dcache_req_valid_r <= 1'b1;
    dcache_req_phys_addr_r <= phys_addr;
    dcache_req_write_r <= req_is_write;
    dcache_req_be_r <= calc_be_second(req_op_size, req_offset);
    dcache_req_wdata_r <= split_write_second(req_wdata, req_offset, req_op_size);
    dcache_req_is_io_r <= 1'b0;
    dcache_req_is_inta_r <= 1'b0;
    latch_biu_meta(fb, second_half_bytes(req_offset, req_op_size) - 2'd1,
                   req_is_write, 2'b00, 1'b0, 1'b0);
    // synthesis translate_off
    if (TRACE_PAGING_EN)
        $display("PG_UNIT EMIT SECOND: phys=%08x be=%04b wr=%0d opr_offset=%0d",
                 phys_addr, calc_be_second(req_op_size, req_offset), req_is_write, fb);
    // synthesis translate_on
endtask

// Emit prefetch BIU request (always DWORD read, no crossing)
task automatic emit_pf_biu_req(input [31:0] phys_addr);
    icache_req_valid_r <= 1'b1;
    icache_req_phys_addr_r <= phys_addr;
endtask

endmodule
