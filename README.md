# z386 - an 80386-class FPGA CPU built around original microcode

z386 is an 80386-compatible CPU core written in SystemVerilog. It reconstructs
the hardware controlled by the original Intel 386 microcode rather than
reimplementing every x86 instruction directly in RTL.

The core includes instruction prefetch and decode, a microcode sequencer,
segmentation, paging, protection checks, integer arithmetic, shifting, and bus
access. Separate configurable instruction and data caches support FPGA system
integration. The project is intended both as an educational reconstruction of
the 80386 and as a reusable embedded x86 core.

## Performance

### Dhrystone 2.1

z386 delivers 16% more Dhrystone performance per MHz than ao486 while using
only 2.3% more ALMs.

| Core | DMIPS/MHz | CPI | Cyclone V ALMs |
| --- | ---: | ---: | ---: |
| **z386** | **0.225** | **4.101** | **15,545** |
| ao486 | 0.194 | 4.556 | 15,190 |
| z486 | 0.330 | 2.800 | 21,906 |

All cores execute the same i386 binary. z386 and z486 use matched 8 KB
instruction and 8 KB data caches; ao486 uses its native cache. Area figures are
standalone seed-1 fits on the same DE10-Nano Cyclone V with identical Quartus
settings. The z486 figure includes its experimental x87 unit. ao486 counts
retirement at a different pipeline boundary, so its CPI is less directly
comparable.

The z386 L1 cache size is tunable through the `DCACHE_SET_BITS` and
`ICACHE_SET_BITS` parameters.

## Build and test

The regression tests use Verilator and Python:

```bash
cd tests
make test-simple
make test-protected
```

The test directory also contains cache and instruction-timing tests, a
Dhrystone harness, the broader `test386.py` suite, and runners for external
real- and protected-mode single-step reference datasets.

## Related projects

For the faster 80486-class successor with a pipelined frontend, hardwired
common instructions, and an integrated x87 unit, see
[z486](https://github.com/nand2mario/z486). The current MiSTer PC integration
is [z486_MiSTer](https://github.com/nand2mario/z486_MiSTer).

## Further reading

For background on the recovered ROM, read
[80386 microcode disassembled](https://www.reenigne.org/blog/80386-microcode-disassembled/).

The following articles describe the 80386 microarchitecture and the development
of z386:

* [z386: An Open-Source 80386 Built Around Original Microcode](https://nand2mario.github.io/posts/2026/z386/)
* [80386 Multiplication and Division](https://nand2mario.github.io/posts/2026/80386_multiplication_and_division/)
* [80386 Barrel Shifter](https://nand2mario.github.io/posts/2026/80386_barrel_shifter/)
* [80386 Protection](https://nand2mario.github.io/posts/2026/80386_protection/)
* [80386 Memory Pipeline](https://nand2mario.github.io/posts/2026/80386_memory_pipeline/)
* [80386 Early Start Memory Access](https://nand2mario.github.io/posts/2026/80386_early_start/)

## Credits

z386 was written by nand2mario. It builds on Intel 386 microcode disassembly
and silicon reverse-engineering by
[reenigne](https://www.reenigne.org/blog/),
[gloriouscow](https://github.com/dbalsom),
[smartest blob](https://github.com/a-mcego), and
[Ken Shirriff](https://www.righto.com/).

## License

Copyright 2026 nand2mario. The SystemVerilog, Python, and Markdown files
(`*.sv`, `*.svh`, `*.py`, and `*.md`) are licensed under the
[Apache License 2.0](LICENSE). See [License Scope](LICENSE-SCOPE.md) for
details. The recovered microcode image is not covered by this license.
