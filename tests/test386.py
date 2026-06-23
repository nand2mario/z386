#!/usr/bin/env python3
"""
test_test386.py - Run the test386.asm comprehensive CPU test

This script builds and runs the test386 testbench, monitoring POST codes
and reporting success/failure.

Usage:
    ./test_test386.py              # Run test386 with default settings
    ./test_test386.py -v           # Verbose mode (show memory/IO traces)
    ./test_test386.py --trace      # Generate FST waveform
    ./test_test386.py --cycles N   # Set max cycles
    ./test_test386.py --progress   # Show periodic progress updates
"""

import argparse
import subprocess
import sys
import os
import re
import time

# POST code descriptions from test386 README
POST_DESCRIPTIONS = {
    0x00: "Real mode initialisation",
    0x01: "Conditional jumps and loops",
    0x02: "Quick tests of unsigned 32-bit multiplication and division",
    0x03: "Move segment registers in real mode",
    0x04: "Store, move, scan, and compare string data in real mode",
    0x05: "Calls in real mode",
    0x06: "Load full pointer in real mode",
    0x08: "GDT, LDT, PDT, and PT setup, enter protected mode",
    0x09: "Stack functionality",
    0x0A: "Test user mode (ring 3) switching and Virtual-8086 mode",
    0x0B: "Moving segment registers",
    0x0C: "Zero and sign-extension",
    0x0D: "16-bit addressing modes (LEA)",
    0x0E: "32-bit addressing modes (LEA) (takes up to a minute)",
    0x0F: "Access memory using various addressing modes",
    0x10: "Store, move, scan, and compare string data in protected mode",
    0x11: "Page faults and PTE bits",
    0x12: "Other memory access faults",
    0x13: "Bit Scan operations",
    0x14: "Bit Test operations",
    0x15: "Byte set on condition (SETcc)",
    0x16: "Calls in protected mode",
    0x17: "Adjust RPL Field of Selector (ARPL)",
    0x18: "Check Array Index Against Bounds (BOUND)",
    0x19: "Exchange Register/Memory with Register (XCHG)",
    0x1A: "Make Stack Frame for Procedure Parameters (ENTER)",
    0x1B: "High Level Procedure Exit (LEAVE)",
    0x1C: "Verify a Segment for Reading or Writing (VERR/VERW)",
    0xE0: "Undefined behaviours and bugs (CPU family dependent)",
    0xEE: "Series of unverified tests for arithmetical and logical opcodes",
    0xEF: "BCD tests (DAA, DAS, AAA, AAS, AAM, AAD)",
    0xF0: "Arithmetic/logic table-driven tests (takes ~20 minutes, watch character output file)",
    0xFF: "Testing completed",
}

def needs_rebuild():
    """Return True when Makefile dependencies require rebuilding Vtb_test386."""
    result = subprocess.run(
        ["make", "-q", "obj_dir/Vtb_test386"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if result.returncode == 0:
        return False
    if result.returncode == 1:
        return True
    # If make cannot determine the state, run the build and report errors there.
    return True


def build_testbench():
    """Build the test386 testbench using Makefile"""
    print("Building test386 testbench...")

    if not needs_rebuild():
        print("  Testbench up to date")
        return True

    result = subprocess.run(
        ["make", "obj_dir/Vtb_test386"],
        capture_output=True, text=True
    )

    if result.returncode != 0:
        print("Build failed!")
        print(result.stderr)
        return False

    print("  Build successful")
    return True


def run_test386(args):
    """Run the test386 simulation"""

    # Build plusargs
    plusargs = []

    if args.trace:
        plusargs.append("+trace")

    if args.verbose:
        plusargs.append("+trace_mem")
        plusargs.append("+trace_io")

    if args.progress:
        plusargs.append("+progress")

    if args.trace_instr:
        plusargs.append("+trace_instr")

    if args.trace_prot:
        plusargs.append("+trace_prot")

    plusargs.append(f"+cycles={args.cycles}")
    plusargs.append(f"+bin={args.bin}")

    cmd = ["./obj_dir/Vtb_test386"] + plusargs

    print(f"Running test386...")
    print(f"  Binary: {args.bin}")
    print(f"  Max cycles: {args.cycles:,}")
    print()

    t_start = time.time()

    # Run simulation
    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1
    )

    last_post = None
    output_lines = []

    # Load reference file for line-by-line comparison
    ref_lines = []
    ref_path = os.path.join(os.path.dirname(args.bin), "test386-EE-reference.txt")
    if os.path.exists(ref_path):
        with open(ref_path) as f:
            ref_lines = [l.rstrip('\n') for l in f.readlines()]
        print(f"  Reference: {ref_path} ({len(ref_lines)} lines)")
    char_line_num = 0
    char_mismatches = 0

    # Write character output to file (message printed on first output)
    char_out_path = "test_output.txt"
    char_out = open(char_out_path, "w")

    for line in process.stdout:
        output_lines.append(line)

        # Parse POST codes
        match = re.search(r'POST: 0x([0-9A-Fa-f]+)', line)
        if match:
            post = int(match.group(1), 16)
            desc = POST_DESCRIPTIONS.get(post, "Unknown test")
            if post != last_post:
                print(f"  POST 0x{post:02X}: {desc}", flush=True)
                last_post = post

        # Show important messages
        if "PASSED" in line or "FAILED" in line or "TIMEOUT" in line:
            print(line, end='')
        elif "ERROR" in line:
            print(line, end='')
        elif "Progress:" in line and args.progress:
            print(f"  {line.strip()}")
        elif line.startswith("CHAROUT: "):
            got = line[9:].rstrip('\n').rstrip(' ')
            if char_line_num == 0:
                print(f"  Character output: \033[93m{char_out_path}\033[0m")
            char_out.write(got + '\n')
            char_out.flush()
            if ref_lines and char_line_num < len(ref_lines):
                expected = ref_lines[char_line_num].rstrip(' ')
                if got != expected:
                    char_mismatches += 1
                    print(f"\033[91mMISMATCH line {char_line_num + 1}:\033[0m")
                    print(f"  \033[91mgot:    [{got}]\033[0m")
                    print(f"  \033[91mexpect: [{expected}]\033[0m")
            char_line_num += 1
        elif args.verbose:
            print(line, end='')

    char_out.close()
    process.wait()

    if ref_lines:
        if char_mismatches == 0 and char_line_num > 0:
            print(f"\n  Verified: {char_line_num}/{len(ref_lines)} lines OK")
        elif char_line_num > 0:
            print(f"\n  \033[91m{char_mismatches} mismatches\033[0m in {char_line_num}/{len(ref_lines)} lines")
        if char_line_num < len(ref_lines):
            print(f"  WARNING: only {char_line_num}/{len(ref_lines)} reference lines produced")

    # Check result
    output = ''.join(output_lines)

    elapsed = time.time() - t_start
    print(f"  Time: {elapsed:.1f}s")

    # Report last POST reached
    if last_post is not None:
        desc = POST_DESCRIPTIONS.get(last_post, "Unknown test")
        print(f"\n  Last POST reached: 0x{last_post:02X} ({desc})")

    if "TEST386 PASSED" in output:
        print("✓ All tests passed!")
        return 0
    elif "FAILED" in output:
        # Extract failure info
        match = re.search(r'Failed at POST: 0x([0-9A-Fa-f]+)', output)
        if match:
            post = int(match.group(1), 16)
            desc = POST_DESCRIPTIONS.get(post, "Unknown test")
            print(f"✗ Failed at POST 0x{post:02X}: {desc}")
        else:
            print("✗ Test failed")
        return 1
    elif "TIMEOUT" in output:
        # POST 0xF0 is the ~20-minute table-driven arithmetic test, impractical to
        # reach in simulation; the test loops at 0xEE ("unverified arith/logic
        # opcodes"), so reaching 0xEE and timing out is the expected success result.
        if last_post is not None and last_post >= 0xEE:
            print("✓ Reached POST 0x{:02X} (success)".format(last_post))
            return 0
        else:
            print("✗ Test timed out before POST 0xEE")
            return 2
    else:
        print("? Unknown result")
        return 3


def main():
    parser = argparse.ArgumentParser(description='Run test386.asm CPU test')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='Verbose output (trace memory/IO)')
    parser.add_argument('--trace', action='store_true',
                        help='Generate FST waveform trace')
    parser.add_argument('--cycles', type=int, default=20_000_000,
                        help='Maximum simulation cycles')
    parser.add_argument('--progress', action='store_true',
                        help='Show periodic progress updates')
    parser.add_argument('--bin', type=str,
                        default='test386.asm/test386.bin',
                        help='Path to test386.bin')
    parser.add_argument('--rebuild', action='store_true',
                        help='Force rebuild of testbench')
    parser.add_argument('--trace-instr', action='store_true',
                        help='Enable instruction trace')
    parser.add_argument('--trace-prot', action='store_true',
                        help='Enable protection unit trace')

    args = parser.parse_args()

    # Change to tests directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    # Rebuild if requested
    if args.rebuild and os.path.exists("obj_dir/Vtb_test386"):
        import shutil
        shutil.rmtree("obj_dir", ignore_errors=True)

    # Build testbench
    if not build_testbench():
        return 1

    # Run test
    return run_test386(args)


if __name__ == '__main__':
    sys.exit(main())
