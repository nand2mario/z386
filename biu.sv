//
// Bus Interface Unit - Combinational Pass-Through
//
module biu
    import z386_pkg::*;
(
    input clk,
    input reset_n,

    // External bus (ready/valid handshake)
    output     [31:2] bus_addr,
    output      [3:0] bus_be,
    input      [31:0] bus_din,
    output     [31:0] bus_dout,
    output            bus_valid,    // request pending (held by paging unit)
    output            bus_write,    // 1=write, 0=read
    output            bus_io,
    input             bus_ready,    // handshake: transfer on bus_valid && bus_ready
    input             bus_resp_valid, // read data valid (1-cycle pulse)
    output            bus_inta,

    // Single request port from paging unit
    input             req_valid,
    input      [31:0] req_phys_addr,
    input             req_write,
    input      [3:0]  req_be,
    input      [31:0] req_wdata,
    input             req_is_io,
    input             req_is_inta,    // Interrupt acknowledge cycle
    output            req_accepted,   // combinational: downstream accepted
    output            req_complete,   // combinational: write accepted or read data ready
    output     [31:0] rdata           // combinational: bus_din pass-through
);

// Combinational pass-through: no latching, saves 1 pipeline cycle
assign bus_addr  = req_phys_addr[31:2];
assign bus_be    = req_be;
assign bus_dout  = req_wdata;
assign bus_write = req_write;
assign bus_valid = req_valid;
assign bus_io    = req_is_io;
assign bus_inta  = req_is_inta;

// Acceptance: downstream accepted this cycle
assign req_accepted = req_valid && bus_ready;

// Completion: write done on acceptance, read done on resp_valid
assign req_complete = bus_resp_valid || (req_valid && bus_ready && req_write);
assign rdata = bus_din;

localparam bit TRACE_MEM_EN = 1'b0;

// synthesis translate_off
always @(posedge clk) begin
    if (reset_n && req_accepted) begin
        if (TRACE_MEM_EN)
            $display("BIU: addr=%08x be=%04b wr=%0d io=%0d",
                     req_phys_addr, req_be, req_write, req_is_io);
    end
end
// synthesis translate_on

endmodule
