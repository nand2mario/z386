#!/usr/bin/env python3
"""Build and run the freestanding Dhrystone benchmark on z386 revisions."""

from __future__ import annotations

import argparse
import re
import shutil
import struct
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
REPO_ROOT = THIS_DIR.parents[2]
DEFAULT_Z386_RELEASE_DIR = REPO_ROOT / "z386_MiSTer/src/z386"
DEFAULT_Z386_CURRENT_DIR = THIS_DIR.parents[1]  # the core dir containing this tests/ tree (24.z386x)
DEFAULT_Z386_CACHE = REPO_ROOT / "z386_MiSTer/src/memory/l1_cache.sv"
BUILD_DIR = THIS_DIR / "build"
BIN_FILE = BUILD_DIR / "dhrystone.bin"
HEX_FILE = BUILD_DIR / "dhrystone.hex"

CODE_PHYS_BASE = 0x00010000
LINEAR_BASE = 0x00010000
PAGE_COUNT = 16

COMMON_RTL = [
    "z386_pkg.sv",
    "ucode_rom.sv",
    "z386.sv",
    "alu.sv",
    "biu.sv",
    "paging_unit.sv",
    "paging_tlb.sv",
    "paging_walker.sv",
    "dsp_mul.sv",
    "protection.sv",
    "segmentation_unit.sv",
]

CORE_RTL = {
    "z386": ["decoder.sv", "prefetch.sv"],
}


@dataclass(frozen=True)
class RunResult:
    core: str
    passed: bool
    failed: bool
    timed_out: bool
    cycles: int | None
    instructions: int | None
    data: str | None
    output: str

    @property
    def cpi(self) -> float | None:
        if self.cycles is None or not self.instructions:
            return None
        return self.cycles / self.instructions


def run_checked(cmd: list[str], cwd: Path, verbose: bool = False) -> None:
    if verbose:
        print(" ".join(cmd))
        subprocess.run(cmd, cwd=cwd, check=True)
        return

    result = subprocess.run(cmd, cwd=cwd, text=True, capture_output=True)
    if result.returncode != 0:
        print(result.stdout)
        print(result.stderr, file=sys.stderr)
        raise subprocess.CalledProcessError(result.returncode, cmd)


def generate_page_table_entry(frame_addr: int, flags: str = "RW") -> int:
    pte = frame_addr & 0xFFFFF000
    pte |= 0x01
    if "W" in flags.upper():
        pte |= 0x02
    if "U" in flags.upper():
        pte |= 0x04
    pte |= 0x20
    return pte


def generate_page_tables() -> dict[int, bytes]:
    memory: dict[int, bytes] = {}
    page_dir = [0] * 1024
    page_table_addr = 0x1000

    page_dir[0] = generate_page_table_entry(page_table_addr, "RW")
    for idx in range(1024):
        memory[page_table_addr + idx * 4] = struct.pack("<I", 0)

    for idx in range(PAGE_COUNT):
        linear_addr = LINEAR_BASE + idx * 0x1000
        physical_addr = CODE_PHYS_BASE + idx * 0x1000
        pte_idx = (linear_addr >> 12) & 0x3FF
        memory[page_table_addr + pte_idx * 4] = struct.pack(
            "<I", generate_page_table_entry(physical_addr, "RW")
        )

    for idx, pde in enumerate(page_dir):
        memory[idx * 4] = struct.pack("<I", pde)

    return memory


def build_benchmark_binary(iters: int, verbose: bool = False) -> None:
    run_checked(["make", f"DHRY_ITERS={iters}", "all"], cwd=THIS_DIR, verbose=verbose)


def build_memory_image() -> None:
    binary = BIN_FILE.read_bytes()
    if len(binary) > PAGE_COUNT * 0x1000:
        raise RuntimeError(
            f"Dhrystone binary is {len(binary)} bytes, larger than mapped "
            f"{PAGE_COUNT * 0x1000} bytes"
        )

    memory: dict[int, int] = {}
    for addr, data in generate_page_tables().items():
        for offset, byte in enumerate(data):
            memory[addr + offset] = byte

    for offset, byte in enumerate(binary):
        memory[CODE_PHYS_BASE + offset] = byte

    HEX_FILE.parent.mkdir(parents=True, exist_ok=True)
    max_addr = max(memory.keys()) if memory else 0
    with HEX_FILE.open("w") as f:
        for addr in range(max_addr + 1):
            f.write(f"{memory.get(addr, 0):02X}\n")


def rtl_sources(core: str, core_dir: Path) -> list[Path]:
    names = COMMON_RTL + CORE_RTL[core]
    sources = [core_dir / name for name in names]
    for optional_name in ("l1_cache.sv", "l1_icache.sv"):
        optional_src = core_dir / optional_name
        if optional_src.exists():
            sources.append(optional_src)
    missing = [str(path) for path in sources if not path.exists()]
    if missing:
        raise FileNotFoundError("missing RTL source(s): " + ", ".join(missing))
    return sources


def copy_ucode_files(core_dir: Path, build_dir: Path) -> None:
    # Copy whichever ROM images the core revision provides (older cores use
    # the expanded ucode45.hex; current cores predecode in hardware).
    for name in ("ucode.hex", "ucode45.hex", "pla_entry_rom.hex"):
        src = core_dir / name
        if src.exists():
            shutil.copy2(src, build_dir / name)


def build_core(
    label: str,
    core_dir: Path,
    rebuild: bool = False,
    verbose: bool = False,
) -> Path:
    core_dir = core_dir.resolve()
    core_build_dir = BUILD_DIR / label
    exe = core_build_dir / "obj_dir/Vtb_dhrystone"
    if exe.exists() and not rebuild:
        copy_ucode_files(core_dir, core_build_dir)
        return exe

    if rebuild:
        # Verilator's generated precompiled headers depend on the exact RTL
        # source set and defines.  Reusing them across core revisions can make
        # a forced comparison rebuild fail before the simulation starts.
        shutil.rmtree(core_build_dir / "obj_dir", ignore_errors=True)
    core_build_dir.mkdir(parents=True, exist_ok=True)
    copy_ucode_files(core_dir, core_build_dir)

    cmd = [
        "verilator",
        "--timing",
        "-Wall",
        "-Wno-fatal",
        "-Wno-UNOPTFLAT",
        "-Wno-SYNCASYNCNET",
        "-cc",
        "--exe",
        "-O3",
        "--build",
        "-j",
        "0",
        f"-I{core_dir}",
        "-CFLAGS",
        "-std=c++14 -O3",
        "--trace-fst",
        "--trace-structs",
        "--top-module",
        "tb_dhrystone",
    ]

    sources = [
        THIS_DIR / "tb_dhrystone.sv",
        *rtl_sources("z386", core_dir),
        THIS_DIR / "sim_main_dhrystone.cpp",
    ]
    if (core_dir / "l1_cache.sv").exists():
        cmd.insert(cmd.index("--top-module"), "+define+Z386_INTERNAL_CACHE")
    else:
        cache_src = DEFAULT_Z386_CACHE.resolve()
        if not cache_src.exists():
            raise FileNotFoundError(f"missing external L1 cache source: {cache_src}")
        sources.append(cache_src)
    if label == "z386_release":
        cmd.insert(cmd.index("--top-module"), "+define+Z386_LEGACY_SEG_DESC")
    cmd.extend(str(path) for path in sources)
    run_checked(cmd, cwd=core_build_dir, verbose=verbose)
    return exe


def run_core(
    label: str,
    exe: Path,
    cycles: int,
    timeout: int,
    mem_model: str,
    trace: bool = False,
    trace_io: bool = False,
    progress: bool = False,
) -> RunResult:
    cmd = [
        str(exe),
        f"+mem={HEX_FILE}",
        f"+cycles={cycles}",
    ]
    if mem_model == "simple":
        cmd.append("+simple_mem")
    if trace_io:
        cmd.append("+trace_io")
    if progress:
        cmd.append("+progress")
    if trace:
        tracefile = BUILD_DIR / label / "dhrystone.fst"
        cmd.extend(["+trace", f"+tracefile={tracefile}"])

    result = subprocess.run(
        cmd,
        cwd=BUILD_DIR / label,
        text=True,
        capture_output=True,
        timeout=timeout,
    )
    if result.stderr:
        print(result.stderr, file=sys.stderr)

    output = result.stdout
    cycles_match = re.search(r"Total cycles:\s*(\d+)", output)
    instr_match = re.search(r"Total instructions:\s*(\d+)", output)
    data_matches = re.findall(r"TEST DATA:\s*0x([0-9A-Fa-f]+)", output)

    return RunResult(
        core=label,
        passed="TEST PASSED" in output,
        failed="TEST FAILED" in output,
        timed_out="TIMEOUT" in output,
        cycles=int(cycles_match.group(1)) if cycles_match else None,
        instructions=int(instr_match.group(1)) if instr_match else None,
        data=data_matches[-1] if data_matches else None,
        output=output,
    )


def print_results(iters: int, results: list[RunResult]) -> None:
    print(f"Dhrystone iterations: {iters}")
    print()
    print("core         | result  | cycles  | instr   | CPI")
    print("-------------+---------+---------+---------+------")
    for result in results:
        status = (
            "PASS" if result.passed else
            "FAIL" if result.failed else
            "TIMEOUT" if result.timed_out else
            "UNKNOWN"
        )
        cycles = f"{result.cycles}" if result.cycles is not None else "-"
        instructions = f"{result.instructions}" if result.instructions is not None else "-"
        cpi = f"{result.cpi:.3f}" if result.cpi is not None else "-"
        print(f"{result.core:<12} | {status:<7} | {cycles:>7} | {instructions:>7} | {cpi:>5}")

    by_core = {result.core: result for result in results}
    if "z386_release" in by_core and "z386_current" in by_core:
        z386_release = by_core["z386_release"]
        z386_current = by_core["z386_current"]
        if z386_release.cycles and z386_current.cycles:
            print()
            print(
                "z386 current cycle speedup vs release: "
                f"{z386_release.cycles / z386_current.cycles:.3f}x"
            )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--iters", type=int, default=200, help="Dhrystone iterations")
    parser.add_argument("--cycles", type=int, default=2_000_000, help="simulation cycle limit")
    parser.add_argument("--timeout", type=int, default=120, help="host timeout in seconds")
    parser.add_argument(
        "--core",
        choices=("both", "z386_release", "z386_current"),
        default="z386_current",
        help="core revision(s) to run",
    )
    parser.add_argument(
        "--z386-release-dir",
        type=Path,
        default=DEFAULT_Z386_RELEASE_DIR,
        help="released z386 core source directory",
    )
    parser.add_argument(
        "--z386-current-dir",
        type=Path,
        default=DEFAULT_Z386_CURRENT_DIR,
        help="current z386 core source directory",
    )
    parser.add_argument("--no-build", action="store_true", help="reuse existing Dhrystone binary")
    parser.add_argument("--no-build-cores", action="store_true", help="reuse existing Verilator core binaries")
    parser.add_argument(
        "--rebuild-cores",
        action="store_true",
        help="force Verilator core rebuilds (default behavior; kept for compatibility)",
    )
    parser.add_argument(
        "--mem-model",
        choices=("sdram", "simple"),
        default="sdram",
        help="Dhrystone memory timing model",
    )
    parser.add_argument("--trace", action="store_true", help="generate FST trace")
    parser.add_argument("--trace-io", action="store_true", help="print benchmark I/O markers")
    parser.add_argument("--progress", action="store_true", help="print simulation progress")
    parser.add_argument("-v", "--verbose", action="store_true", help="show build commands/output")
    args = parser.parse_args()

    if not args.no_build:
        build_benchmark_binary(args.iters, verbose=args.verbose)
    build_memory_image()

    cores = ["z386_release", "z386_current"] if args.core == "both" else [args.core]
    core_dirs = {
        "z386_release": args.z386_release_dir,
        "z386_current": args.z386_current_dir,
    }

    exes: dict[str, Path] = {}
    for core in cores:
        if args.no_build_cores:
            exes[core] = BUILD_DIR / core / "obj_dir/Vtb_dhrystone"
            if not exes[core].exists():
                raise FileNotFoundError(f"{exes[core]} does not exist")
        else:
            exes[core] = build_core(
                core,
                core_dirs[core],
                rebuild=True,
                verbose=args.verbose,
            )

    results = [
        run_core(
            core,
            exes[core],
            cycles=args.cycles,
            timeout=args.timeout,
            mem_model=args.mem_model,
            trace=args.trace,
            trace_io=args.trace_io,
            progress=args.progress,
        )
        for core in cores
    ]

    print_results(args.iters, results)
    failures = [result for result in results if not result.passed]
    if failures:
        print()
        for result in failures:
            print(f"[{result.core}] output:")
            print(result.output)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
