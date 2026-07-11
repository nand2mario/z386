
# z386 - an 80386-class FPGA CPU built around original microcode

z386 is an 80386-compatible CPU core written in SystemVerilog and built around the original Intel 386 microcode. Instead of implementing each x86 instruction as a separate RTL behavior, z386 implements the hardware structures the microcode expects to control: instruction prefetch, decode, the microcode sequencer, segmentation, paging, protection checks, ALU, shifter, and bus access.

This `z386x` branch contains the eXtended core used by current z386_MiSTer
builds. It adds a faster frontend and bounded hardwired fast paths around the
original microcode engine. The `master` branch remains the 80386-faithful z386
implementation.

The project is intended as an educational reconstruction, a MiSTer PC core, and a reusable embedded x86 CPU core.

*For the MiSTer core based on z386, see [z386_MiSTer](https://github.com/nand2mario/z386_MiSTer).*

Comparison of z386 v0.4 with ao486 on a DE10-Nano:

|     | z386 | ao486 |
|-----|------|-------|
|Lines of code by `cloc` | 10.6K | 17.6K |
|ALMs| 22.4K | 15.9K |
|Registers| 10.2K | 9.4K |
|BRAM| 398K | 131K |
|Frequency| 85 MHz | 90 MHz |
|DOOM FPS (max details)| 23.0 | 21.0 |
|Boots Windows | Not yet | Yes |

z386's BRAM is ~76% L1 cache (16 KB instruction + 16 KB data, twice ao486's caches) and ~24% the microcode ROM (37 bits × 2560 entries). The L1 cache size is tunable via the `DCACHE_SET_BITS` and `ICACHE_SET_BITS` parameters.

To learn more about the 80386 microcode, read [80386 microcode disassembled](https://www.reenigne.org/blog/80386-microcode-disassembled/).

I also wrote a blog series analyzing the 386 microarchitecture and documenting the process of building z386:

* [80386 Multiplication and Division](https://nand2mario.github.io/posts/2026/80386_multiplication_and_division/)
* [80386 Barrel Shifter](https://nand2mario.github.io/posts/2026/80386_barrel_shifter/)
* [80386 Protection](https://nand2mario.github.io/posts/2026/80386_protection/)
* [80386 Memory Pipeline](https://nand2mario.github.io/posts/2026/80386_memory_pipeline/)
* [z386: An Open-Source 80386 Built Around Original Microcode](https://nand2mario.github.io/posts/2026/z386/)
* [80386 Early Start Memory Access](https://nand2mario.github.io/posts/2026/80386_early_start/)

z386 was written by nand2mario. It builds on Intel 386 microcode disassembly and silicon reverse-engineering work by [reenigne](https://www.reenigne.org/blog/), [gloriouscow](https://github.com/dbalsom), [smartest blob](https://github.com/a-mcego), and [Ken Shirriff](https://www.righto.com/).
