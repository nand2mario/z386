
# z386 - 80386 FPGA CPU running original 386 microcode

z386 is a compact 80386-compatible CPU core written in SystemVerilog and driven by the original Intel 386 microcode. Rather than implementing x86 instruction behavior directly in RTL, it recreates the processor around a microcode sequencer and the supporting units needed by a real 386: prefetch, decode, segmentation, paging, protection checks, ALU, and bus access. It is intended as an educational reference, a potential ao486 replacement for MiSTer, and a reusable embedded x86 CPU core.

Comparison with ao486 on DE10-Nano:

|     | z386 | ao486 |
|-----|------|-------|
|Lines of code by `cloc` | 8K | 17.6K |
|ALUTs| 18K  | 21K   |
|Registers| 5K | 6.5K |
|BRAM| 116K | 131K |
|Frequency| 85Mhz | 90Mhz |
|DOOM FPS (max details)| 16.5 | 21.0 |

