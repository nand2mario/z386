//
// Page Table Walker for 80386 Paging Unit
// Two-level page table walk: Page Directory Entry (PDE) -> Page Table Entry (PTE)
//
`timescale 1ns/1ns

module paging_walker
    import z386_pkg::*;
(
    input               clk,
    input               reset_n,

    // Request interface
    input               walk_request,
    input        [31:0] linear_addr,
    input               is_write,
    input        [1:0]  cpl,            // Current privilege level
    input        [31:0] cr3,            // Page directory base register
    input               wp_enable,      // CR0.WP - write protect

    // Result interface
    output reg          walk_done,
    output reg          walk_fault,
    output reg   [2:0]  fault_code,     // [2]=U, [1]=W, [0]=P
    output reg   [19:0] result_pfn,
    output reg          result_writable,
    output reg          result_user,
    output reg          result_dirty,
    output reg          result_accessed,

    // Memory interface for page table reads and write-backs
    output reg          mem_rd,
    output reg          mem_wr,
    output reg   [31:0] mem_addr,
    output reg   [31:0] mem_wdata,
    input        [31:0] mem_data,
    input               mem_ready
);

// Page walk state machine
typedef enum logic [3:0] {
    PW_IDLE,
    PW_READ_PDE,        // Issue PDE read
    PW_WAIT_PDE,        // Wait for PDE data
    PW_READ_PTE,        // Issue PTE read
    PW_WAIT_PTE,        // Wait for PTE data
    PW_CHECK_PERM,      // Check permissions before write-back
    PW_WRITE_PDE,       // Write back PDE with ACCESSED bit
    PW_WAIT_WR_PDE,     // Wait for PDE write completion
    PW_WRITE_PTE,       // Write back PTE with ACCESSED (+ DIRTY if write)
    PW_WAIT_WR_PTE,     // Wait for PTE write completion
    PW_DONE,            // Walk complete (success, after write-back)
    PW_FAULT            // Walk complete (fault, no write-back)
} pw_state_t;

pw_state_t state, next_state;

// Latched request parameters
reg [31:0] saved_linear;
reg        saved_is_write;
reg [1:0]  saved_cpl;
reg [31:0] saved_cr3;
reg        saved_wp;

// Page directory entry and page table entry
reg [31:0] pde;
reg [31:0] pte;

localparam bit TRACE_PAGING_EN = 1'b0;

// Linear address components
wire [9:0] dir_index   = saved_linear[31:22];   // Page directory index
wire [9:0] table_index = saved_linear[21:12];   // Page table index

// PDE address = CR3[31:12] | dir_index << 2
wire [31:0] pde_addr = {saved_cr3[31:12], dir_index, 2'b00};

// PTE address = PDE[31:12] | table_index << 2
wire [31:0] pte_addr = {pde[31:12], table_index, 2'b00};

// Permission checking
wire pde_present  = pde[PTE_P];
wire pde_writable = pde[PTE_RW];
wire pde_user     = pde[PTE_US];

wire pte_present  = pte[PTE_P];
wire pte_writable = pte[PTE_RW];
wire pte_user     = pte[PTE_US];
wire pte_dirty    = pte[PTE_D];
wire pte_accessed = pte[PTE_A];

// Combined permissions (most restrictive of PDE and PTE)
wire combined_writable = pde_writable & pte_writable;
wire combined_user     = pde_user & pte_user;

// Access checking
wire is_user_mode = (saved_cpl == 2'd3);
wire user_access_ok = !is_user_mode || combined_user;
wire write_access_ok = !saved_is_write ||
                       combined_writable ||
                       (!is_user_mode && !saved_wp);

// State machine
always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state <= PW_IDLE;
        saved_linear <= 32'h0;
        saved_is_write <= 1'b0;
        saved_cpl <= 2'b00;
        saved_cr3 <= 32'h0;
        saved_wp <= 1'b0;
        pde <= 32'h0;
        pte <= 32'h0;
    end else begin
        state <= next_state;

        // Latch request parameters at start
        if (state == PW_IDLE && walk_request) begin
            saved_linear <= linear_addr;
            saved_is_write <= is_write;
            saved_cpl <= cpl;
            saved_cr3 <= cr3;
            saved_wp <= wp_enable;
        end

        // Latch PDE when memory returns
        if (state == PW_WAIT_PDE && mem_ready) begin
            pde <= mem_data;
        end

        // Latch PTE when memory returns
        if (state == PW_WAIT_PTE && mem_ready) begin
            pte <= mem_data;
        end
    end
end

// Next state logic
always_comb begin
    next_state = state;

    case (state)
        PW_IDLE: begin
            if (walk_request)
                next_state = PW_READ_PDE;
        end

        PW_READ_PDE: begin
            next_state = PW_WAIT_PDE;
        end

        PW_WAIT_PDE: begin
            if (mem_ready) begin
                if (!mem_data[PTE_P])
                    next_state = PW_FAULT;      // PDE not present
                else
                    next_state = PW_READ_PTE;   // PDE OK, read PTE
            end
        end

        PW_READ_PTE: begin
            next_state = PW_WAIT_PTE;
        end

        PW_WAIT_PTE: begin
            if (mem_ready) begin
                if (!mem_data[PTE_P])
                    next_state = PW_FAULT;      // PTE not present
                else
                    next_state = PW_CHECK_PERM; // Check permissions before write-back
            end
        end

        PW_CHECK_PERM: begin
            // Permission check: only write back A/D bits if access is permitted
            if (!user_access_ok || !write_access_ok)
                next_state = PW_FAULT;          // Protection fault, no write-back
            else
                next_state = PW_WRITE_PDE;      // Permissions OK, write back A/D bits
        end

        PW_WRITE_PDE: begin
            next_state = PW_WAIT_WR_PDE;
        end

        PW_WAIT_WR_PDE: begin
            if (mem_ready)
                next_state = PW_WRITE_PTE;
        end

        PW_WRITE_PTE: begin
            next_state = PW_WAIT_WR_PTE;
        end

        PW_WAIT_WR_PTE: begin
            if (mem_ready)
                next_state = PW_DONE;
        end

        PW_DONE: begin
            next_state = PW_IDLE;
        end

        PW_FAULT: begin
            next_state = PW_IDLE;
        end

        default: next_state = PW_IDLE;
    endcase
end

// Output logic
always_comb begin
    // Defaults
    walk_done = 1'b0;
    walk_fault = 1'b0;
    fault_code = 3'b000;
    mem_rd = 1'b0;
    mem_wr = 1'b0;
    mem_addr = 32'h0;
    mem_wdata = 32'h0;
    result_pfn = 20'h0;
    result_writable = 1'b0;
    result_user = 1'b0;
    result_dirty = 1'b0;
    result_accessed = 1'b0;

    case (state)
        PW_READ_PDE: begin
            mem_rd = 1'b1;
            mem_addr = pde_addr;
        end

        PW_WAIT_PDE: begin
            if (!mem_ready) begin
                mem_rd = 1'b1;
                mem_addr = pde_addr;
            end
        end

        PW_READ_PTE: begin
            mem_rd = 1'b1;
            mem_addr = pte_addr;
        end

        PW_WAIT_PTE: begin
            if (!mem_ready) begin
                mem_rd = 1'b1;
                mem_addr = pte_addr;
            end
        end

        PW_CHECK_PERM: begin
            // Pure transition state — walk_done/walk_fault signaled in PW_FAULT or PW_DONE
        end

        PW_WRITE_PDE: begin
            mem_wr = 1'b1;
            mem_addr = pde_addr;
            mem_wdata = pde | (32'h1 << PTE_A);  // Set ACCESSED bit
        end

        PW_WAIT_WR_PDE: begin
            if (!mem_ready) begin
                mem_wr = 1'b1;
                mem_addr = pde_addr;
                mem_wdata = pde | (32'h1 << PTE_A);
            end
        end

        PW_WRITE_PTE: begin
            mem_wr = 1'b1;
            mem_addr = pte_addr;
            // Set ACCESSED, and DIRTY if this is a write access
            mem_wdata = pte | (32'h1 << PTE_A) | (saved_is_write ? (32'h1 << PTE_D) : 32'h0);
        end

        PW_WAIT_WR_PTE: begin
            if (!mem_ready) begin
                mem_wr = 1'b1;
                mem_addr = pte_addr;
                mem_wdata = pte | (32'h1 << PTE_A) | (saved_is_write ? (32'h1 << PTE_D) : 32'h0);
            end
        end

        PW_DONE: begin
            // Walk succeeded with write-back complete
            walk_done = 1'b1;
            result_pfn = pte[31:12];
            result_writable = combined_writable;
            result_user = combined_user;
            result_dirty = pte_dirty || saved_is_write;  // Updated after write-back
            result_accessed = 1'b1;                       // Always set after successful walk
        end

        PW_FAULT: begin
            walk_done = 1'b1;
            walk_fault = 1'b1;
            // Determine fault code based on what failed
            if (!pde_present) begin
                fault_code[PF_P] = 1'b0;  // PDE not present
            end else if (!pte_present) begin
                fault_code[PF_P] = 1'b0;  // PTE not present
            end else begin
                fault_code[PF_P] = 1'b1;  // Protection violation
            end
            fault_code[PF_W] = saved_is_write;
            fault_code[PF_U] = is_user_mode;

        end

        default: ;
    endcase
end

endmodule
