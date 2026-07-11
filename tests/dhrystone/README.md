# z386 Dhrystone revision comparison

This directory contains a freestanding Dhrystone 2.1 build for the
local protected-mode Verilator harness. It can build and run both the
released `z386` core and the current `24.z386x` core side-by-side.

The imported Dhrystone sources are `dhry.h`, `dhry_1.c`, and `dhry_2.c`.
They are based on the public Dhrystone 2.1 C sources from the SiFive
`benchmark-dhrystone` repository. The local changes are intentionally small:

* `DHRY_EMBEDDED` disables host I/O and host timer includes.
* The original `main()` in `dhry_1.c` is excluded for embedded builds.
* `dhrystone_main.c` provides static allocation, the benchmark loop, result
  validation, and testbench I/O markers.
* `support.c` provides the small C library subset needed by the benchmark.

Build both core simulators and run from `21.z386/tests`:

```sh
make dhrystone
```

The Dhrystone Makefile uses the host Linux `gcc` by default, with `-m32` to
produce an i386 freestanding binary. On macOS it defaults to `i686-elf-gcc`
and the matching binutils. Override with `CROSS=i686-elf-` or `CROSS=` if a
different toolchain setup is needed.

Or run directly:

```sh
cd dhrystone
./run_dhrystone.py --iters 200 --cycles 2000000
```

Unlike the general debug-oriented testbenches, this benchmark defaults to an
SDRAM-like timing model so CPI is closer to the board memory path. Use
`--mem-model simple` for the old fast behavioral memory when only checking
functionality.

By default the runner compares these source directories:

```text
z386_release -> ../../../z386_MiSTer/src/z386
z386_current -> ../..
```

Override them with `--z386-release-dir` and `--z386-current-dir` when comparing
different trees. The Verilator binaries and memory image are kept under
`dhrystone/build/`, with separate build directories for each core.

The harness instantiates the same external L1 cache for both cores, using
`12.386tang/src/memory/l1_cache.sv` by default. This keeps the memory
hierarchy closer to an apples-to-apples comparison: both cores run the same
binary with cached instruction and data accesses.

Instruction count is measured at `dut.i_pop` for both cores.

The reported CPI is total testbench cycles divided by retired instructions.
That includes startup and validation, but for moderate iteration counts the
Dhrystone loop dominates. Use larger `--iters` values when comparing CPI
optimization changes.
