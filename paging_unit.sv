// Paging Unit for 80386 Processor Integrates TLB and Page Walker for address translation. Also handles DWORD-crossing splits: receives...
// Details: doc/z386x/implementation_notes.md#src-24-z386x-paging-unit-sv-1

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
    input               mem_inta_req,      // INTA request; kept out of the demand TLB/cache cone
    input        [31:0] mem_inta_addr,
    input               mem_ea_read,       // This demand read uses the early-start EA (SET-read eligible)
    input               mem_req_precheck,  // mem_req before the GP/segment-fault gate; drives
                                           // speculative address capture + TLB-lookup-addr load
    input               mem_req_upcoming,  // Combinational early hint: suppresses prefetch start
    output logic        mem_accepted,      // Ready: demand request may be handed off this cycle
    output reg          mem_servicing,     // High while accepted request is in flight
    output              mem_complete_now,  // Completion shortcut (disabled: use registered mem_servicing clear)
    output reg          mem_dly_grace,     // Pulse: dcache lookup cycle of an optimistic demand read;
                                           // a pure-DLY uop may execute this cycle (data lands end of cycle)
    output              mem_write_dly_grace, // PG_MEM_TLB cycle of a write that posts THIS cycle (no fault,
                                           // no page walk): it has no result, so the following non-bus uop
                                           // (e.g. a push's SIGMA->ESP DLY) may execute now instead of
                                           // waiting out mem_servicing.  Gated on the posted-done outcome so
                                           // a faulting / TLB-missing write never releases the delay slot.
    output reg          mem_opt_wait,      // Optimistic read past its grace cycle and still in flight:
                                           // sequencer must stall any uop until completion
    output              mem_write_wait,    // Demand WRITE that did not post in its PG_MEM_TLB cycle
                                           // and can still fault (permission now, walk fault later, or the second half of a crossing access). The issuing instruction may already...
                                           // Details: doc/z386x/implementation_notes.md#src-24-z386x-paging-unit-sv-49
    input        [31:0] linear_addr,       // Registered linear (reloc(IND)) for BOTH the demand path
                                           // (-> req_linear / tlb_lookup_addr) and the live TLB; the
                                           // seg-adder stays off the live cone since it is registered
    input               live_valid,        // linear_addr is the true linear (trust live write-post)
    input        [1:0]  mem_op_size,       // 0=byte, 1=word, 2=dword
    input               mem_write,         // 1=write, 0=read
    input        [31:0] mem_wdata,         // Write data (pre-computed by z386.sv)
    input               mem_rd_ind,        // BUSOP_RD_IND flag
    input               is_write_access,   // Is this a write (for permission check)
    input               mem_check_only,    // CW: check write permission only, no actual bus write
    input        [1:0]  cpl,               // Current privilege level
    input               mem_is_io,         // This request is IO (skip translation)
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
    output logic [31:0] dcache_req_phys_addr, // Physical address (full 32-bit); the
                                              // cache indexes off [11:2] (page-offset,
                                              // TLB-free), tags off [31:12]
    output logic        dcache_req_write,     // 1=write
    output logic [3:0]  dcache_req_be,        // Byte enables (pre-computed)
    output logic [31:0] dcache_req_wdata,     // Write data (pre-positioned on bus)
    output logic        dcache_req_is_io,     // Request is IO space
    output logic        dcache_req_is_inta,   // Request is INTA cycle
    output logic        dcache_req_is_vga_mem,// Physical VGA aperture

    input               dcache_req_accepted,  // Demand-side request accepted this cycle
    input               dcache_req_complete,  // Demand-side request complete
    input               dcache_read_complete, // Demand-side read data is valid this cycle
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
    output reg   [31:0] cr2_out            // Faulting address (written to CR2)
);

reg                 rd_ind_active;     // BUSOP_RD_IND active (internal; demoted from output)

// Control register bits
wire pg_enable = cr0[31];   // PG - Paging enable
wire wp_enable = cr0[16];   // WP - Write protect

// Compile-time-off sim trace flags to keep hot scheduler paths free of
// per-cycle $test$plusargs overhead under Verilator.
localparam bit TRACE_MEM_EN    = 1'b0;
localparam bit TRACE_PAGING_EN = 1'b0;

reg pf_ack_toggle_r;
reg [127:0] pf_rdata_r;       // Registered read line for prefetch
reg         pf_fast_pending;  // TLB-hit icache request, independent of demand FSM
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
wire        tlb_is_vga_mem;
// Live (combinational) TLB lookup of the demand linear, used at PG_IDLE to
// precompute whether a write will post, so the post-write DLY grace at
// PG_MEM_TLB needs no combinational TLB term on the uc_exec path.
wire        live_tlb_hit;
wire [31:0] live_tlb_physical;
wire        live_tlb_writable;
wire        live_tlb_user;
wire        live_tlb_dirty;
wire        live_tlb_is_vga_mem;
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
    PG_MEM_TLB,          // Registered demand-memory TLB/permission cycle
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
wire idle_data_req = s_idle && mem_req && !mem_servicing;
wire idle_inta_req = s_idle && mem_inta_req && !mem_servicing;
wire idle_mem_req = idle_data_req || idle_inta_req;
wire idle_mem_precheck = s_idle && mem_req_precheck && !mem_servicing;
wire idle_mem_capture = idle_mem_precheck || idle_inta_req;
// P0/P1 prefetch timing: P0 prefetch toggles pf_req_toggle and presents pf_linear_addr. P1 paging translates the registered prefetch...
// Details: doc/z386x/implementation_notes.md#src-24-z386x-paging-unit-sv-189
wire idle_pf_req = s_idle && pf_pending && !fast_path_pending && !pf_fast_pending;

paging_tlb tlb_inst (
    .clk            (clk),
    .reset_n        (reset_n),
    .linear_addr    (tlb_lookup_addr),
    .hit            (tlb_hit),
    .physical_addr  (tlb_physical_addr),
    .writable       (tlb_writable),
    .user           (tlb_user),
    .dirty          (tlb_dirty),
    .is_vga_mem     (tlb_is_vga_mem),
    .linear_addr_live(linear_addr),       // same registered linear -> seg-adder off the live-TLB cone
    .live_hit       (live_tlb_hit),
    .live_physical_addr(live_tlb_physical),
    .live_writable  (live_tlb_writable),
    .live_user      (live_tlb_user),
    .live_dirty     (live_tlb_dirty),
    .live_is_vga_mem(live_tlb_is_vga_mem),
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

// Live precompute of "this demand write will post" (TLB hit, writable, dirty — no fault, no page walk), from the live TLB + the...
// Details: doc/z386x/implementation_notes.md#src-24-z386x-paging-unit-sv-292
wire live_is_user     = (cpl == 2'd3);
wire live_user_ok     = !live_is_user || live_tlb_user;
wire live_write_ok    = !mem_write || live_tlb_writable || (!live_is_user && !wp_enable);
wire live_dirty_ok    = !mem_write || live_tlb_dirty;
// live_valid gates the optimistic write-post: when the live-TLB input is a
// registered don't-care (complex modrm whose true linear is seg_linear, only
// in req_linear), defer the write instead of trusting the wrong-address lookup.
wire live_write_posts = !pg_enable || (live_valid && live_tlb_hit && live_user_ok && live_write_ok && live_dirty_ok);
reg  write_will_post; // registered live_write_posts, valid in the PG_MEM_TLB cycle

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

assign pf_ack_bypass = ((state == PG_PF_BIU_WAIT) || pf_fast_pending) &&
                       icache_req_complete;

wire idle_mem_crossing = access_crosses_dword(mem_op_size, linear_addr[1:0]);
wire idle_io_crossing = mem_is_io && idle_mem_crossing;
wire [31:0] idle_request_linear = idle_inta_req ? mem_inta_addr : linear_addr;
wire cache_lookup_granted = 1'b1;
wire idle_mem_ready = s_idle && !mem_servicing;
wire req_tlb_dirty_ok = !req_is_write || tlb_dirty;
wire req_can_translate = !pg_enable || (tlb_hit && slow_tlb_access_ok && req_tlb_dirty_ok);
wire req_perm_fault = pg_enable && tlb_hit && !slow_tlb_access_ok;
wire [31:0] req_mem_phys = pg_enable ? {tlb_physical_addr[31:12], req_linear[11:0]} : req_linear;
wire req_mem_dcache_candidate = (state == PG_MEM_TLB) && req_can_translate &&
                                !req_perm_fault && !req_check_only &&
                                cache_lookup_granted;
wire req_mem_dcache_accept = req_mem_dcache_candidate && dcache_req_accepted;

// synthesis translate_off
// SET-read-at-019 validation: the live TLB physical captured at PG_IDLE must
// equal the registered TLB physical at PG_MEM_TLB (same TLB, same linear), so a
// demand read can present the live physical to the dcache one cycle early.
reg [19:0] live_pfn_pre;
reg        live_hit_pre;
always_ff @(posedge clk) begin
    if (idle_data_req && !mem_write && !mem_is_io) begin
        live_pfn_pre <= live_tlb_physical[31:12];
        // Only trust the live lookup when linear_addr_live is the true linear;
        // for complex modrm it is a registered don't-care, so don't compare.
        live_hit_pre <= live_valid && live_tlb_hit;
    end
    if (reset_n && (state == PG_MEM_TLB) && !req_is_write && live_hit_pre && tlb_hit &&
        (live_pfn_pre !== tlb_physical_addr[31:12]))
        $display("%0t: LIVE PFN MISMATCH live=%05x reg=%05x", $time, live_pfn_pre, tlb_physical_addr[31:12]);
end
// synthesis translate_on
wire req_mem_posted_done = req_mem_dcache_accept && req_is_write && dcache_req_complete;
// Loop 1: release the post-write DLY when the write posts. write_will_post (registered from the live TLB at PG_IDLE) replaces the...
// Details: doc/z386x/implementation_notes.md#src-24-z386x-paging-unit-sv-377
assign mem_write_dly_grace = (state == PG_MEM_TLB) && req_is_write && write_will_post && !req_crossing;
// Fault-capable window of a demand write (see the port comment).  States
// after translation succeeds (WALK_LOOKUP, CROSS_LOOKUP2/WAIT2, IDLE with
// fast_path_pending) cannot fault and are excluded.
assign mem_write_wait = req_is_write && !mem_write_dly_grace &&
                        (state == PG_MEM_TLB   || state == PG_WALKING     ||
                         state == PG_CROSS_WAIT1 || state == PG_CROSS_PREP2 ||
                         state == PG_CROSS_TLB2  || state == PG_CROSS_WALK2);
// Posted-write completion, computed from the registered/demand write terms only. The combinational dcache_req_valid/dcache_req_write...
// Details: doc/z386x/implementation_notes.md#src-24-z386x-paging-unit-sv-390
wire posted_write_isw = (state == PG_MEM_TLB) ? req_is_write : dcache_req_write_r;
wire dcache_posted_write_done = (dcache_req_valid_r || req_mem_dcache_candidate) &&
                                posted_write_isw && dcache_req_accepted && dcache_req_complete;
wire cross2_tlb_dirty_ok = !req_is_write || tlb_dirty;
wire cross2_can_translate = !pg_enable || (tlb_hit && slow_tlb_access_ok && cross2_tlb_dirty_ok);
wire pf_tlb_match = !pg_enable || (tlb_lookup_addr_r[31:12] == pf_linear_addr[31:12]);
wire fast_pf_candidate = idle_pf_req && cache_lookup_granted &&
                         (!pg_enable || (pf_tlb_match && tlb_hit && pf_tlb_user_ok));
wire [31:0] fast_pf_phys = pg_enable ? {tlb_physical_addr[31:12], pf_linear_addr[11:0]} : pf_linear_addr;

assign mem_accepted = mem_accepted_r || idle_mem_ready;
wire        req_mem_present    = (state == PG_MEM_TLB);

// Early read drives dcache at PG_IDLE / RD microcode time
wire        early_rd_idx_drive = idle_data_req && !mem_write && !mem_is_io;
wire        early_rd_tlb_ok    = !pg_enable || (live_tlb_hit && live_user_ok);
wire        early_rd_present   = early_rd_idx_drive && mem_ea_read && !idle_mem_crossing &&
                                 !mem_rd_ind && early_rd_tlb_ok;
// Early write: post a cacheable, non-crossing, non-check-only write at PG_IDLE from the live TLB. Validated (zero EARLY-WRITE mismatches...
// Details: doc/z386x/implementation_notes.md#src-24-z386x-paging-unit-sv-415
wire        early_wr_idx_drive = idle_data_req && mem_write && !mem_is_io;
wire        early_wr_present   = early_wr_idx_drive && !idle_mem_crossing &&
                                 !mem_check_only && live_write_posts;
wire        early_idx_drive    = early_rd_idx_drive || early_wr_idx_drive;
wire [31:0] early_phys         = pg_enable ? {live_tlb_physical[31:12], linear_addr[11:0]}
                                          : linear_addr;
wire        early_is_vga_mem   = pg_enable ? live_tlb_is_vga_mem
                                           : (linear_addr[31:17] == 15'h5);
wire        req_is_vga_mem     = pg_enable ? tlb_is_vga_mem
                                           : (req_linear[31:17] == 15'h5);
wire        early_rd_accept    = early_rd_present && dcache_req_accepted;
wire        early_wr_accept    = early_wr_present && dcache_req_accepted;
wire        early_present      = early_rd_present || early_wr_present;

assign dcache_req_valid = dcache_req_valid_r || req_mem_dcache_candidate || early_present;
// PIPT cache request: drive the cache off the physical address only -- no separate early-index port. Page frame [31:12] uses the...
// Details: doc/z386x/implementation_notes.md#src-24-z386x-paging-unit-sv-433
wire [19:0] dcache_req_frame  = early_present   ? early_phys[31:12] :
                                req_mem_present  ? req_mem_phys[31:12] :
                                                   dcache_req_phys_addr_r[31:12];
wire [11:0] dcache_req_offset = early_idx_drive  ? linear_addr[11:0] :
                                req_mem_present   ? req_linear[11:0] :
                                                    dcache_req_phys_addr_r[11:0];
assign dcache_req_phys_addr = {dcache_req_frame, dcache_req_offset};
assign dcache_req_write = early_wr_present ? 1'b1 :
                          early_rd_present ? 1'b0 :
                          req_mem_present  ? req_is_write : dcache_req_write_r;
assign dcache_req_be = early_present ? mem_be :
                       req_mem_present ?
                       (req_crossing ? calc_be_first(req_op_size, req_offset) :
                                       calc_be(req_op_size, req_offset)) :
                       dcache_req_be_r;
assign dcache_req_wdata = early_wr_present ?
                          shift_write_data(mem_wdata, mem_op_size, linear_addr[1:0]) :
                          req_mem_present ?
                          (req_crossing ? split_write_first(req_wdata, req_offset, req_op_size) :
                                          shift_write_data(req_wdata, req_op_size, req_offset)) :
                          dcache_req_wdata_r;
assign dcache_req_is_io = (early_present || req_mem_present) ? 1'b0 : dcache_req_is_io_r;
assign dcache_req_is_inta = (early_present || req_mem_present) ? 1'b0 : dcache_req_is_inta_r;
assign dcache_req_is_vga_mem = early_present ? early_is_vga_mem :
                               req_mem_present ? req_is_vga_mem :
                               (dcache_req_phys_addr_r[31:17] == 15'h5);
assign icache_req_valid = icache_req_valid_r || fast_pf_candidate;
assign icache_req_phys_addr = icache_req_valid_r ? icache_req_phys_addr_r : fast_pf_phys;

// Keep the registered TLB lookup address on a dedicated write-enable path.
wire idle_mem_precheck_capture = idle_mem_precheck && !mem_is_io;
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

    if (idle_mem_precheck_capture) begin
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
        pf_fast_pending <= 1'b0;
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
        mem_dly_grace <= 1'b0;
        write_will_post <= 1'b0;
        mem_opt_wait <= 1'b0;
        dcache_req_phys_addr_r <= 32'h0;
        dcache_req_write_r <= 1'b0;
        dcache_req_be_r <= 4'h0;
        dcache_req_wdata_r <= 32'h0;
    end else begin
        // Default: clear one-shot signals
        mem_dly_grace <= 1'b0;
        write_will_post <= 1'b0;
        // Optimistic read tracking: completion (this includes a hit during the
        // grace cycle itself) clears the wait; an expired grace without
        // completion turns into a hard wait until the miss/fill finishes.
        if (dcache_req_complete && fast_path_pending)
            mem_opt_wait <= 1'b0;
        else if (mem_dly_grace)
            mem_opt_wait <= 1'b1;
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

        // A translated icache request is independent of the demand-memory
        // state machine. This allows a redirect fetch and a posted stack store
        // (CALL) to launch together through the split I/D caches.
        if (fast_pf_candidate && icache_req_accepted)
            pf_fast_pending <= 1'b1;
        if (pf_fast_pending && icache_req_complete) begin
            pf_rdata_r <= icache_rdata;
            pf_ack_toggle_r <= ~pf_ack_toggle_r;
            pf_fast_pending <= 1'b0;
        end

        // Clear walker pending on completion
        if (dcache_req_complete && walk_biu_pending)
            walk_biu_pending <= 1'b0;

        //=====================================================================
        // BIU completion: write OPR_R / pf_rdata / walker data
        //=====================================================================
        if (dcache_req_complete) begin
            if (dcache_read_complete && opr_is_walk_r) begin
                // Walker: raw data routed to walker via dcache_rdata (combinational)
                // synthesis translate_off
                if (TRACE_PAGING_EN)
                    $display("BIU WALK DONE: data=%08x", dcache_rdata);
                // synthesis translate_on
            end else if (dcache_read_complete && !opr_is_write_r && !opr_suppress_r) begin
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
                if (idle_mem_capture) begin
                    if (mem_is_io || idle_inta_req) begin
                        if (!idle_io_crossing) begin
                            // IO/INTA fast path data (cannot segment-fault)
                            dcache_req_phys_addr_r <= idle_request_linear;
                            dcache_req_write_r <= mem_write;
                            dcache_req_be_r <= mem_be;
                            dcache_req_wdata_r <= shift_write_data(mem_wdata, mem_op_size, idle_request_linear[1:0]);
                            // First INTA cycle (addr=4) is dummy — suppress OPR_R update.
                            // Second INTA (addr=0) delivers the vector to OPR_R.
                            latch_biu_meta(2'd0, op_size_bytes_m1(mem_op_size), mem_write,
                                           idle_request_linear[1:0],
                                           idle_inta_req && (idle_request_linear[2:0] == 3'd4),
                                           1'b0);
                            rd_ind_active <= 1'b0;
                        end else begin
                            // IO crossing: split into two DWORD-aligned bus cycles
                            latch_mem_request(linear_addr, 1'b1);
                            req_is_io <= 1'b1;
                            rd_ind_active <= 1'b0;
                            // First half data
                            dcache_req_phys_addr_r <= linear_addr;
                            dcache_req_write_r <= mem_write;
                            dcache_req_be_r <= calc_be_first(mem_op_size, linear_addr[1:0]);
                            dcache_req_wdata_r <= split_write_first(mem_wdata, linear_addr[1:0], mem_op_size);
                            latch_biu_meta(2'd0, first_half_bytes(linear_addr[1:0]) - 2'd1,
                                           mem_write, linear_addr[1:0], 1'b0, 1'b0);
                        end
                    end else begin
                        // synthesis translate_off
                        if (TRACE_PAGING_EN && idle_mem_req)
                            $display("PG_UNIT CAPTURE: linear=%08x size=%0d wr=%0d crossing=%0d",
                                     linear_addr, mem_op_size, mem_write, idle_mem_crossing);
                        // synthesis translate_on

                        latch_mem_request(linear_addr, idle_mem_crossing);
                        rd_ind_active <= mem_rd_ind;
                    end
                end

                if (idle_pf_req && !idle_mem_capture) begin
                    if (fast_pf_candidate) begin
                        // Completion is tracked by pf_fast_pending, allowing
                        // demand translation to proceed in parallel.
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

                // Fault-gated control/state updates for the captured request.
                if (idle_mem_req) begin
                    mem_accepted_r <= 1'b1;
                    mem_servicing <= 1'b1;
                    if (mem_is_io || idle_inta_req) begin
                        dcache_req_valid_r <= 1'b1;
                        if (!idle_io_crossing) begin
                            dcache_req_is_io_r <= mem_is_io;
                            dcache_req_is_inta_r <= idle_inta_req;
                            fast_path_pending <= 1'b1;
                        end else begin
                            dcache_req_is_io_r <= 1'b1;
                            dcache_req_is_inta_r <= 1'b0;
                            state <= PG_CROSS_WAIT1;
                        end
                    end else if (early_rd_accept) begin
                        // SET-read-at-019: the dcache accepted the early read (presented this cycle with the live physical), so the SET preread already ran at...
                        // Details: doc/z386x/implementation_notes.md#src-24-z386x-paging-unit-sv-753
                        latch_biu_meta(2'd0, op_size_bytes_m1(mem_op_size), 1'b0,
                                       linear_addr[1:0], 1'b0, 1'b0);
                        fast_path_pending <= 1'b1;
                        mem_dly_grace <= 1'b1;   // optimistic release for the finalize cycle
                        rd_ind_active <= 1'b0;
                    end else if (early_wr_accept) begin
                        // Early posted write: the store-queue write was enqueued this cycle from the live physical (accept => post), so the access is done. Clear...
                        // Details: doc/z386x/implementation_notes.md#src-24-z386x-paging-unit-sv-764
                        latch_biu_meta(2'd0, op_size_bytes_m1(mem_op_size), 1'b1,
                                       linear_addr[1:0], 1'b0, 1'b0);
                        rd_ind_active <= 1'b0;
                        mem_servicing <= 1'b0;
                    end else begin
                        state <= PG_MEM_TLB;
                        // Loop 1: precompute (from the live TLB) whether this
                        // memory write will post next cycle, so the post-write
                        // DLY grace carries no live TLB cone into uc_exec.
                        write_will_post <= mem_write && live_write_posts;
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
                                mem_dly_grace <= !rd_ind_active;    // for optimistic read release
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
