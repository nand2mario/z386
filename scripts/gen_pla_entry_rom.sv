// Generate + exhaustively verify the BRAM image for pla_entry_lookup.
//
// pla_entry_lookup maps {data32, opcode, prefix_rep, pe_enable, 1'b1, prefix_0f}
// -> 16-bit microcode entry.  To serve it from a synchronous M10K (read address
// registered one cycle early) we must split the address bits by who can supply
// them a cycle early:
//   * opcode, prefix_rep, prefix_0f  -- prefetch/decoder NEXT-state, available
//     one cycle early (q_window_next / prefix_*_n).  -> ROM ADDRESS (10 bits).
//   * data32 (= D^prefix_66) and pe_enable -- external mode bits that change
//     ASYNCHRONOUSLY on a mode switch; latching them early returns the wrong
//     entry (e.g. MOV Sreg differs real vs protected).  -> applied at the ROM
//     OUTPUT via a 4:1 select using the CURRENT {data32, pe} this cycle.
//
// So the ROM is 1024 x 64: each 10-bit address {opcode, rep, 0f} stores the 4
// entries for {data32,pe} in {00,01,10,11}, packed
//     word = { e(d32=1,pe=1), e(1,0), e(0,1), e(0,0) }   (lane = {data32,pe})
// and the decoder selects word[{data32,pe}*16 +: 16].  Same M10K bits as a
// 4096x16 ROM, but mode-correct.  Proven == pla_entry_lookup for ALL inputs.
//
//   Run (from 21.z386/):
//     $ verilator --binary -I. scripts/gen_pla_entry_rom.sv -o gen_pla_entry_rom
//     $ ./obj_dir/gen_pla_entry_rom
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
