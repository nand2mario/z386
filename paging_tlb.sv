//
// TLB (Translation Lookaside Buffer) for 80386 Paging Unit
// 32-entry 4-way set-associative cache with PLRU replacement per set
//
// 8 sets × 4 ways = 32 entries
// Set index: VPN[2:0] (linear_addr[14:12])
// Tag: VPN[19:3] (linear_addr[31:15]), 17 bits
//
`timescale 1ns/1ns

module paging_tlb
    import z386_pkg::*;
(
    input               clk,
    input               reset_n,

    // Registered lookup interface (combinational output)
    input        [31:0] linear_addr,
    output reg          hit,
    output reg   [31:0] physical_addr,
    output reg          writable,       // Combined PDE & PTE R/W
    output reg          user,           // Combined PDE & PTE U/S
    output reg          dirty,          // D bit from PTE

    // Live demand lookup interface. This keeps the idle demand fast path off
    // the registered-address mux used by prefetch/walker lookups.
    input        [31:0] linear_addr_live,
    output reg          live_hit,
    output reg   [31:0] live_physical_addr,
    output reg          live_writable,
    output reg          live_user,
    output reg          live_dirty,

    // Update interface (from page walker)
    input               update_valid,
    input        [19:0] update_vpn,     // Virtual page number
    input        [19:0] update_pfn,     // Physical frame number
    input               update_writable,
    input               update_user,
    input               update_dirty,
    input               update_accessed,

    // Invalidate all entries (on CR3 write)
    input               invalidate_all
);

// 8 sets × 4 ways
tlb_entry_t tlb [7:0][3:0];

// PLRU bits per set: 3 bits each for 4-way replacement
//        [B0]           B0: 0=left subtree, 1=right subtree
//       /    \
//    [B1]    [B2]       B1: 0=way0, 1=way1
//    /  \    /  \       B2: 0=way2, 1=way3
//   W0  W1  W2  W3
reg [2:0] plru [7:0];

localparam bit TRACE_PAGING_EN = 1'b0;

// Registered lookup address decomposition
wire [19:0] lookup_vpn = linear_addr[31:12];
wire [2:0]  lookup_set = lookup_vpn[2:0];       // Set index: VPN[2:0]
wire [16:0] lookup_tag = lookup_vpn[19:3];       // Tag: VPN[19:3]

// Hit detection - combinational, parallel comparison within selected set
wire hit0 = tlb[lookup_set][0].valid && (tlb[lookup_set][0].vpn[19:3] == lookup_tag);
wire hit1 = tlb[lookup_set][1].valid && (tlb[lookup_set][1].vpn[19:3] == lookup_tag);
wire hit2 = tlb[lookup_set][2].valid && (tlb[lookup_set][2].vpn[19:3] == lookup_tag);
wire hit3 = tlb[lookup_set][3].valid && (tlb[lookup_set][3].vpn[19:3] == lookup_tag);

// Encode hit into 2-bit way index
wire [1:0] hit_way = hit0 ? 2'd0 :
                     hit1 ? 2'd1 :
                     hit2 ? 2'd2 :
                     hit3 ? 2'd3 : 2'd0;

// Live demand lookup address decomposition
wire [19:0] live_lookup_vpn = linear_addr_live[31:12];
wire [2:0]  live_lookup_set = live_lookup_vpn[2:0];
wire [16:0] live_lookup_tag = live_lookup_vpn[19:3];

wire live_hit0 = tlb[live_lookup_set][0].valid && (tlb[live_lookup_set][0].vpn[19:3] == live_lookup_tag);
wire live_hit1 = tlb[live_lookup_set][1].valid && (tlb[live_lookup_set][1].vpn[19:3] == live_lookup_tag);
wire live_hit2 = tlb[live_lookup_set][2].valid && (tlb[live_lookup_set][2].vpn[19:3] == live_lookup_tag);
wire live_hit3 = tlb[live_lookup_set][3].valid && (tlb[live_lookup_set][3].vpn[19:3] == live_lookup_tag);

wire [1:0] live_hit_way = live_hit0 ? 2'd0 :
                          live_hit1 ? 2'd1 :
                          live_hit2 ? 2'd2 :
                          live_hit3 ? 2'd3 : 2'd0;

// Output signals - combinational
always_comb begin
    hit = hit0 | hit1 | hit2 | hit3;

    // Select physical address from matching entry
    case (hit_way)
        2'd0: begin
            physical_addr = {tlb[lookup_set][0].pfn, linear_addr[11:0]};
            writable = tlb[lookup_set][0].writable;
            user = tlb[lookup_set][0].user;
            dirty = tlb[lookup_set][0].dirty;
        end
        2'd1: begin
            physical_addr = {tlb[lookup_set][1].pfn, linear_addr[11:0]};
            writable = tlb[lookup_set][1].writable;
            user = tlb[lookup_set][1].user;
            dirty = tlb[lookup_set][1].dirty;
        end
        2'd2: begin
            physical_addr = {tlb[lookup_set][2].pfn, linear_addr[11:0]};
            writable = tlb[lookup_set][2].writable;
            user = tlb[lookup_set][2].user;
            dirty = tlb[lookup_set][2].dirty;
        end
        2'd3: begin
            physical_addr = {tlb[lookup_set][3].pfn, linear_addr[11:0]};
            writable = tlb[lookup_set][3].writable;
            user = tlb[lookup_set][3].user;
            dirty = tlb[lookup_set][3].dirty;
        end
    endcase

    // If no hit, output linear address (will be overridden by page walker result)
    if (!hit) begin
        physical_addr = linear_addr;
        writable = 1'b1;
        user = 1'b0;
        dirty = 1'b0;
    end
end

always_comb begin
    live_hit = live_hit0 | live_hit1 | live_hit2 | live_hit3;

    case (live_hit_way)
        2'd0: begin
            live_physical_addr = {tlb[live_lookup_set][0].pfn, linear_addr_live[11:0]};
            live_writable = tlb[live_lookup_set][0].writable;
            live_user = tlb[live_lookup_set][0].user;
            live_dirty = tlb[live_lookup_set][0].dirty;
        end
        2'd1: begin
            live_physical_addr = {tlb[live_lookup_set][1].pfn, linear_addr_live[11:0]};
            live_writable = tlb[live_lookup_set][1].writable;
            live_user = tlb[live_lookup_set][1].user;
            live_dirty = tlb[live_lookup_set][1].dirty;
        end
        2'd2: begin
            live_physical_addr = {tlb[live_lookup_set][2].pfn, linear_addr_live[11:0]};
            live_writable = tlb[live_lookup_set][2].writable;
            live_user = tlb[live_lookup_set][2].user;
            live_dirty = tlb[live_lookup_set][2].dirty;
        end
        2'd3: begin
            live_physical_addr = {tlb[live_lookup_set][3].pfn, linear_addr_live[11:0]};
            live_writable = tlb[live_lookup_set][3].writable;
            live_user = tlb[live_lookup_set][3].user;
            live_dirty = tlb[live_lookup_set][3].dirty;
        end
    endcase

    if (!live_hit) begin
        live_physical_addr = linear_addr_live;
        live_writable = 1'b1;
        live_user = 1'b0;
        live_dirty = 1'b0;
    end
end

// Update address decomposition
wire [2:0]  update_set = update_vpn[2:0];
wire [2:0]  update_plru = plru[update_set];

// If the VPN is already present in the set, update that way in place.
// Blind PLRU allocation creates duplicate entries, and the hit priority
// mux then keeps returning the stale duplicate — e.g. a dirty=0 entry
// inserted by a code-fetch walk shadows the dirty=1 entry from a write
// walk, forcing a page walk on every subsequent write to the page.
wire match0 = tlb[update_set][0].valid && (tlb[update_set][0].vpn == update_vpn);
wire match1 = tlb[update_set][1].valid && (tlb[update_set][1].vpn == update_vpn);
wire match2 = tlb[update_set][2].valid && (tlb[update_set][2].vpn == update_vpn);
wire match3 = tlb[update_set][3].valid && (tlb[update_set][3].vpn == update_vpn);

// PLRU victim selection for the update set (existing entry wins)
wire [1:0] victim_way = match0 ? 2'd0 :
                        match1 ? 2'd1 :
                        match2 ? 2'd2 :
                        match3 ? 2'd3 :
                        update_plru[0] ? (update_plru[2] ? 2'd3 : 2'd2) :
                                          (update_plru[1] ? 2'd1 : 2'd0);

// TLB update and PLRU management
integer s;
always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        // Invalidate all entries on reset
        for (s = 0; s < 8; s = s + 1) begin
            tlb[s][0].valid <= 1'b0;
            tlb[s][1].valid <= 1'b0;
            tlb[s][2].valid <= 1'b0;
            tlb[s][3].valid <= 1'b0;
            plru[s] <= 3'b000;
        end
    end else if (invalidate_all) begin
        // CR3 write - flush entire TLB
        for (s = 0; s < 8; s = s + 1) begin
            tlb[s][0].valid <= 1'b0;
            tlb[s][1].valid <= 1'b0;
            tlb[s][2].valid <= 1'b0;
            tlb[s][3].valid <= 1'b0;
            plru[s] <= 3'b000;
        end
    end else begin
        // Update PLRU on hit (point away from accessed way in the hit set)
        if (hit) begin
            case (hit_way)
                2'd0: begin plru[lookup_set][0] <= 1'b1; plru[lookup_set][1] <= 1'b1; end
                2'd1: begin plru[lookup_set][0] <= 1'b1; plru[lookup_set][1] <= 1'b0; end
                2'd2: begin plru[lookup_set][0] <= 1'b0; plru[lookup_set][2] <= 1'b1; end
                2'd3: begin plru[lookup_set][0] <= 1'b0; plru[lookup_set][2] <= 1'b0; end
            endcase
        end

        // Insert new entry from page walker
        if (update_valid) begin
            case (victim_way)
                2'd0: begin
                    tlb[update_set][0].valid <= 1'b1;
                    tlb[update_set][0].vpn <= update_vpn;
                    tlb[update_set][0].pfn <= update_pfn;
                    tlb[update_set][0].writable <= update_writable;
                    tlb[update_set][0].user <= update_user;
                    tlb[update_set][0].dirty <= update_dirty;
                    tlb[update_set][0].accessed <= update_accessed;
                    plru[update_set][0] <= 1'b1; plru[update_set][1] <= 1'b1;
                end
                2'd1: begin
                    tlb[update_set][1].valid <= 1'b1;
                    tlb[update_set][1].vpn <= update_vpn;
                    tlb[update_set][1].pfn <= update_pfn;
                    tlb[update_set][1].writable <= update_writable;
                    tlb[update_set][1].user <= update_user;
                    tlb[update_set][1].dirty <= update_dirty;
                    tlb[update_set][1].accessed <= update_accessed;
                    plru[update_set][0] <= 1'b1; plru[update_set][1] <= 1'b0;
                end
                2'd2: begin
                    tlb[update_set][2].valid <= 1'b1;
                    tlb[update_set][2].vpn <= update_vpn;
                    tlb[update_set][2].pfn <= update_pfn;
                    tlb[update_set][2].writable <= update_writable;
                    tlb[update_set][2].user <= update_user;
                    tlb[update_set][2].dirty <= update_dirty;
                    tlb[update_set][2].accessed <= update_accessed;
                    plru[update_set][0] <= 1'b0; plru[update_set][2] <= 1'b1;
                end
                2'd3: begin
                    tlb[update_set][3].valid <= 1'b1;
                    tlb[update_set][3].vpn <= update_vpn;
                    tlb[update_set][3].pfn <= update_pfn;
                    tlb[update_set][3].writable <= update_writable;
                    tlb[update_set][3].user <= update_user;
                    tlb[update_set][3].dirty <= update_dirty;
                    tlb[update_set][3].accessed <= update_accessed;
                    plru[update_set][0] <= 1'b0; plru[update_set][2] <= 1'b0;
                end
            endcase

            // synthesis translate_off
            if (TRACE_PAGING_EN)
                $display("TLB UPDATE: vpn=%05x pfn=%05x writable=%b user=%b set=%0d victim_way=%0d",
                         update_vpn, update_pfn, update_writable, update_user, update_set, victim_way);
            // synthesis translate_on
        end
    end
end

endmodule
