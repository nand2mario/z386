// Generate and exhaustively verify the mode-correct BRAM image for pla_entry_lookup.
// Details: doc/z386x/implementation_notes.md#src-24-z386x-scripts-gen-pla-entry-rom-sv-1
module gen_pla_entry_rom;
`include "pla_entry.svh"
    localparam int N = 1024;            // 2^10 = {opcode, prefix_rep, prefix_0f}
    logic [63:0] rom  [0:N-1];          // generated from the function
    logic [63:0] back [0:N-1];          // read back from the file
    integer a10, mism, d32, pe;

    function automatic logic [15:0] entry(input logic [9:0] a,
                                          input logic d, input logic p);
        // a = {opcode[7:0], rep, 0f}
        entry = pla_entry_lookup({d, a[9:2], a[1], p, 1'b1, a[0]});
    endfunction

    initial begin
        // 1. Generate: pack the 4 {data32,pe} entries per address.
        for (a10 = 0; a10 < N; a10 = a10 + 1)
            rom[a10] = {entry(a10[9:0], 1'b1, 1'b1), entry(a10[9:0], 1'b1, 1'b0),
                        entry(a10[9:0], 1'b0, 1'b1), entry(a10[9:0], 1'b0, 1'b0)};
        $writememh("pla_entry_rom.hex", rom);

        // 2. Exhaustive check: read back, compare every lane to the function.
        $readmemh("pla_entry_rom.hex", back);
        mism = 0;
        for (a10 = 0; a10 < N; a10 = a10 + 1)
            for (d32 = 0; d32 < 2; d32 = d32 + 1)
                for (pe = 0; pe < 2; pe = pe + 1) begin
                    automatic logic [15:0] sel = back[a10][{d32[0], pe[0]}*16 +: 16];
                    automatic logic [15:0] gold = entry(a10[9:0], d32[0], pe[0]);
                    if (sel !== gold) begin
                        if (mism < 10)
                            $display("  MISMATCH a10=%03x d32=%0d pe=%0d  rom=%04x fn=%04x",
                                     a10[9:0], d32, pe, sel, gold);
                        mism = mism + 1;
                    end
                end

        $display("Wrote pla_entry_rom.hex (%0d x 64-bit entries)", N);
        $display("Exhaustive check: %0d / %0d mismatches", mism, N*4);
        if (mism == 0)
            $display("PASS: ROM image == pla_entry_lookup for ALL %0d {addr,d32,pe}", N*4);
        else
            $display("FAIL: ROM image does NOT match the PLA");
    end
endmodule
