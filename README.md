
# z386 - an 80386-class FPGA CPU built around original microcode

z386 is a compact 80386-compatible CPU core written in SystemVerilog and built around the original Intel 386 microcode. Instead of implementing each x86 instruction as a separate RTL behavior, z386 implements the hardware structures the microcode expects to control: instruction prefetch, decode, the microcode sequencer, segmentation, paging, protection checks, ALU, shifter, and bus access.

The project is intended as an educational reconstruction, a usable MiSTer PC core, and a reusable embedded x86 CPU core.

*For the MiSTer core based on z386, see [z386_MiSTer](https://github.com/nand2mario/z386_MiSTer).*

Comparison with ao486 on a DE10-Nano:

|     | z386 | ao486 |
|-----|------|-------|
|Lines of code by `cloc` | 8K | 17.6K |
|ALUTs| 18K  | 21K   |
|Registers| 5K | 6.5K |
|BRAM| 116K | 131K |
|Frequency| 85 MHz | 90 MHz |
|DOOM FPS (max details)| 16.5 | 21.0 |

z386 was written mainly from January to April 2026. These blog posts record the process of studying the 386 microarchitecture and building z386:
* [80386 Multiplication and Division](https://nand2mario.github.io/posts/2026/80386_multiplication_and_division/)
* [80386 Barrel Shifter](https://nand2mario.github.io/posts/2026/80386_barrel_shifter/)
* [80386 Protection](https://nand2mario.github.io/posts/2026/80386_protection/)
* [80386 Memory Pipeline](https://nand2mario.github.io/posts/2026/80386_memory_pipeline/)

z386 was written by nand2mario. It builds on Intel 386 microcode disassembly and silicon reverse-engineering work by [reenigne](https://www.reenigne.org/blog/), [gloriouscow](https://github.com/dbalsom), [smartest blob](https://github.com/a-mcego), and [Ken Shirriff](https://www.righto.com/).
