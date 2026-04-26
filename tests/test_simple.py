#!/usr/bin/env python3
"""
Single-Instruction Test Runner for z386

Runs "386" single-instruction test set for 32-bit 80386 instructions.
Based on z8086/tests/test8088.py but adapted for 32-bit architecture.
"""

import json
import subprocess as sp
from pathlib import Path
import tempfile
import argparse
import math

ROOT = Path(__file__).resolve().parents[1]
TESTS = ROOT / 'tests'
TB = TESTS / 'obj_dir' / 'Vtb_z386'
TEST_DIR = TESTS / 'simple'
VERBOSE = False

# 80386 opcode names (00-FF for primary opcodes)
# 0F xx for extended opcodes
OPCODE_NAMES = [
"ADD Eb Gb", "ADD Ev Gv", "ADD Gb Eb", "ADD Gv Ev", "ADD AL Ib", "ADD EAX Iv", "PUSH ES", "POP ES",
"OR Eb Gb", "OR Ev Gv", "OR Gb Eb", "OR Gv Ev", "OR AL Ib", "OR EAX Iv", "PUSH CS", "0F",

"ADC Eb Gb", "ADC Ev Gv", "ADC Gb Eb", "ADC Gv Ev", "ADC AL Ib", "ADC EAX Iv", "PUSH SS", "POP SS",
"SBB Eb Gb", "SBB Ev Gv", "SBB Gb Eb", "SBB Gv Ev", "SBB AL Ib", "SBB EAX Iv", "PUSH DS", "POP DS",

"AND Eb Gb", "AND Ev Gv", "AND Gb Eb", "AND Gv Ev", "AND AL Ib", "AND EAX Iv", "ES:", "DAA",
"SUB Eb Gb", "SUB Ev Gv", "SUB Gb Eb", "SUB Gv Ev", "SUB AL Ib", "SUB EAX Iv", "CS:", "DAS",

"XOR Eb Gb", "XOR Ev Gv", "XOR Gb Eb", "XOR Gv Ev", "XOR AL Ib", "XOR EAX Iv", "SS:", "AAA",
"CMP Eb Gb", "CMP Ev Gv", "CMP Gb Eb", "CMP Gv Ev", "CMP AL Ib", "CMP EAX Iv", "DS:", "AAS",

"INC EAX", "INC ECX", "INC EDX", "INC EBX", "INC ESP", "INC EBP", "INC ESI", "INC EDI",
"DEC EAX", "DEC ECX", "DEC EDX", "DEC EBX", "DEC ESP", "DEC EBP", "DEC ESI", "DEC EDI",

"PUSH EAX", "PUSH ECX", "PUSH EDX", "PUSH EBX", "PUSH ESP", "PUSH EBP", "PUSH ESI", "PUSH EDI",
"POP EAX", "POP ECX", "POP EDX", "POP EBX", "POP ESP", "POP EBP", "POP ESI", "POP EDI",

"PUSHA", "POPA", "BOUND", "ARPL", "FS:", "GS:", "OpSize", "AdSize",
"PUSH Iv", "IMUL Gv Ev Iv", "PUSH Ib", "IMUL Gv Ev Ib", "INSB", "INSW/D", "OUTSB", "OUTSW/D",

"JO Jb", "JNO Jb", "JB Jb", "JNB Jb", "JZ Jb", "JNZ Jb", "JBE Jb", "JA Jb",
"JS Jb", "JNS Jb", "JPE Jb", "JPO Jb", "JL Jb", "JGE Jb", "JLE Jb", "JG Jb",

"GRP1 Eb Ib", "GRP1 Ev Iv", "GRP1 Eb Ib", "GRP1 Ev Ib", "TEST Gb Eb", "TEST Gv Ev", "XCHG Gb Eb", "XCHG Gv Ev",
"MOV Eb Gb", "MOV Ev Gv", "MOV Gb Eb", "MOV Gv Ev", "MOV Ew Sw", "LEA Gv M", "MOV Sw Ew", "POP Ev",

"NOP", "XCHG ECX EAX", "XCHG EDX EAX", "XCHG EBX EAX", "XCHG ESP EAX", "XCHG EBP EAX", "XCHG ESI EAX", "XCHG EDI EAX",
"CBW/CWDE", "CWD/CDQ", "CALL Ap", "WAIT", "PUSHF", "POPF", "SAHF", "LAHF",

"MOV AL Ob", "MOV EAX Ov", "MOV Ob AL", "MOV Ov EAX", "MOVSB", "MOVSW/D", "CMPSB", "CMPSW/D",
"TEST AL Ib", "TEST EAX Iv", "STOSB", "STOSW/D", "LODSB", "LODSW/D", "SCASB", "SCASW/D",

"MOV AL Ib", "MOV CL Ib", "MOV DL Ib", "MOV BL Ib", "MOV AH Ib", "MOV CH Ib", "MOV DH Ib", "MOV BH Ib",
"MOV EAX Iv", "MOV ECX Iv", "MOV EDX Iv", "MOV EBX Iv", "MOV ESP Iv", "MOV EBP Iv", "MOV ESI Iv", "MOV EDI Iv",

"X", "X", "RET Iw", "RET", "LES Gv Mp", "LDS Gv Mp", "MOV Eb Ib", "MOV Ev Iv",
"ENTER", "LEAVE", "RETF Iw", "RETF", "INT 3", "INT Ib", "INTO", "IRET",

"GRP2 Eb 1", "GRP2 Ev 1", "GRP2 Eb CL", "GRP2 Ev CL", "AAM", "AAD", "X", "XLAT",
"ESC 0", "ESC 1", "ESC 2", "ESC 3", "ESC 4", "ESC 5", "ESC 6", "ESC 7",

"LOOPNZ Jb", "LOOPZ Jb", "LOOP Jb", "JCXZ Jb", "IN AL Ib", "IN EAX Ib", "OUT Ib AL", "OUT Ib EAX",
"CALL Jv", "JMP Jv", "JMP Ap", "JMP Jb", "IN AL DX", "IN EAX DX", "OUT DX AL", "OUT DX EAX",

"LOCK", "X", "REPNZ", "REPZ", "HLT", "CMC", "GRP3a Eb", "GRP3b Ev",
"CLC", "STC", "CLI", "STI", "CLD", "STD", "GRP4 Eb", "GRP5 Ev",
]

def build_if_needed():
    """Build the testbench if it doesn't exist or source files have changed."""
    # Source files to check (matching Makefile)
    source_files = [
        ROOT / 'z386_pkg.sv',
        ROOT / 'z386.sv',
        ROOT / 'decoder.sv',
        ROOT / 'alu.sv',
        ROOT / 'segmentation.sv',
        TESTS / 'tb_z386.sv',
        TESTS / 'sim_main.cpp',
        # Include header files
        ROOT / 'pla_entry.svh',
        ROOT / 'pla_control.svh',
    ]

    if not TB.exists():
        print('[build] testbench not found, building...')
        sp.check_call(['make', '-s', 'build'], cwd=str(TESTS))
        return

    # Check if any source file is newer than the testbench
    tb_mtime = TB.stat().st_mtime
    needs_rebuild = False
    for src in source_files:
        if src.exists() and src.stat().st_mtime > tb_mtime:
            if not needs_rebuild:
                print(f'[build] source files changed, rebuilding...')
            needs_rebuild = True
            break

    if needs_rebuild:
        sp.check_call(['make', '-s', 'build'], cwd=str(TESTS))

def write_memhex(ram_pairs) -> str:
    """Write memory contents to hex file using sparse format."""
    lines = []
    cur = None
    for addr, val in sorted(ram_pairs, key=lambda p: p[0]):
        if cur != addr:
            lines.append(f"@{addr:08X}\n")
            cur = addr
        lines.append(f"{val:02X}\n")
        cur += 1
    tf = tempfile.NamedTemporaryFile('w', delete=False, suffix='.hex')
    tf.write(''.join(lines))
    tf.flush()
    tf.close()
    return tf.name

def run_case(case):
    """Run a single test case."""
    name = case.get('name', '')
    init = case['initial']
    fin = case['final']

    # Get initial register values (32-bit)
    cs = int(init['regs'].get('cs', 0))
    ip = int(init['regs'].get('ip', 0))
    eax = int(init['regs'].get('eax', 0))
    ram = init.get('ram', [])
    memhex = write_memhex(ram)

    nbytes = case.get('bytes')
    reads = 0
    if nbytes:
        reads = math.ceil(len(nbytes) / 4)  # 32-bit reads

    # Limit runtime
    cycles = max(50000, reads * 1000)
    cmd = [str(TB), f"+mem={memhex}", f"+cycles={cycles}", "--trace"]

    if VERBOSE:
        cmd.extend(["+trace_flags", "+trace_decode", "+trace_ucode"])
    
    # Set all initial registers
    for k, v in init['regs'].items():
        cmd.append(f"+{k.lower()}={v}")
    if reads:
        cmd.append(f"+reads={reads}")

    # Request outputs we need to check
    final_regs = fin.get('regs', {}) if fin else {}
    final_ram = fin.get('ram', []) if fin else []
    for i, (addr, _) in enumerate(final_ram):
        cmd.append(f"+ram{i}={addr}")

    if VERBOSE:
        print("START REGS:", ' '.join(f"{k}:{v:08x}" for k, v in init['regs'].items()))
        print('END   REGS:', ' '.join(f"{k}:{v:08x}" for k, v in final_regs.items()))
        print("START RAM: ", ' '.join(f"{k:08x}:{v:02x}" for k, v in ram))
        print('END   RAM: ', ' '.join(f"{k:08x}:{v:02x}" for k, v in final_ram))
        print(f"Command: {' '.join(cmd)}")

    p = sp.run(cmd, cwd=str(TESTS), stdout=sp.PIPE, stderr=sp.STDOUT, text=True)
    out = p.stdout

    # Parse results
    regs = {}
    mem_results = {}
    import re
    for line in out.splitlines():
        if line.startswith('RESULT REG:'):
            vals = line.split(':')[1].strip().split(' ')
            for val in vals:
                m = val.split('=')
                if len(m) == 2:
                    regs[m[0].lower()] = int(m[1], 16) if m[1].startswith('0x') else int(m[1])
        if line.startswith('RESULT MEM:'):
            m = line.split(':')[1].strip().split('=')
            if len(m) == 2:
                mem_results[int(m[0][1:])] = int(m[1])

    # Adjust IP (placeholder - needs proper tracking)
    q_len = regs.get('q_len', 0)
    q_consumed = regs.get('q_consumed', 0)
    if 'ip' in regs:
        regs['ip'] = regs['ip'] - q_len - q_consumed

    # Check results
    ok = True
    for reg, val in final_regs.items():
        reg = reg.lower()
        got_val = regs.get(reg)
        if VERBOSE:
            if got_val is not None:
                print(f"Checking {reg} = {val:08x} (got {got_val:08x})")
            else:
                print(f"Checking {reg} = {val:08x} (got None)")

        # For EFLAGS, might need masking for undefined flags
        if reg == 'eflags':
            # Simplified - check all flags for now
            ok = ok and (regs.get(reg) == int(val))
        else:
            ok = ok and (regs.get(reg) == int(val))

    if final_ram:
        for addr, val in final_ram:
            v = mem_results.get(int(addr))
            if v is None:
                print(f"\033[91mRAM {addr:08x} not found in results\033[0m")
                ok = False
            else:
                if v != int(val):
                    print(f"\033[91mChecking RAM {addr:08x} = {val:02x} (got {v:02x})\033[0m")
                ok = ok and (v == int(val))

    return ok, out

def main():
    global VERBOSE
    ap = argparse.ArgumentParser(description='Run 80386 instruction tests on z386 core')
    ap.add_argument('--file', '-f', help='Run only this JSON test file (basename or path)')
    ap.add_argument('--idx', '-i', type=int, help='Run only this case index within the JSON file')
    ap.add_argument('--limit', '-n', type=int, default=10, help='Max cases per file (default: 10)')
    ap.add_argument('--no-color', action='store_true', help='Disable ANSI colors in grid')
    ap.add_argument('--verbose', '-v', action='store_true', help='Verbose output')
    args = ap.parse_args()
    VERBOSE = args.verbose
    use_color = not args.no_color

    build_if_needed()

    # Resolve test files
    if args.file:
        jf = Path(args.file)
        if not jf.suffix:
            jf = TEST_DIR / (jf.name + '.json') if not (TEST_DIR / jf.name).suffix else (TEST_DIR / jf.name)
        if not jf.exists():
            cand = TEST_DIR / jf.name
            if cand.exists():
                jf = cand
            else:
                raise SystemExit(f"Test file not found: {args.file}")
        test_files = [jf]
    else:
        test_files = sorted(TEST_DIR.glob('*.json'))

    total = 0
    passed = 0
    agg = {}

    for jf in test_files:
        data = json.loads(jf.read_text())
        indices = range(len(data)) if args.idx is None else [args.idx]
        count = 0
        for i in indices:
            if i < 0 or i >= len(data):
                continue
            if args.idx is None and count >= args.limit:
                break
            case = data[i]
            total += 1
            ok, out = run_case(case)
            status_plain = 'ok' if ok else 'fail'
            if use_color:
                col = '\033[92m' if ok else '\033[91m'
                status = f"{col}{status_plain}\033[0m"
            else:
                status = status_plain
            print(f"=== {jf.name}[{case.get('idx', i)}] {case.get('name', '')}: {status} ===")
            if ok:
                passed += 1
            if not ok or VERBOSE:
                print("BYTES:", ' '.join(f"{b:02x}" for b in case['bytes']))
                tail = '\n'.join(out.splitlines()[-5:])
                if VERBOSE:
                    print(f"OUTPUT:\n{out}")
                else:
                    print(f"TAIL:\n{tail}")

            # Aggregate by opcode
            base = Path(jf).stem.upper()
            agg.setdefault(base, {'p': 0, 't': 0})
            agg[base]['t'] += 1
            if ok:
                agg[base]['p'] += 1
            count += 1

    # Summary
    pct = (passed / total * 100.0) if total else 0.0
    summary_line = f"\nSummary: {passed}/{total} ({pct:.1f}%)"
    print(summary_line)

    # Write summary to result.txt
    try:
        result_path = Path(__file__).parent / 'result.txt'
        with open(result_path, 'w') as f:
            f.write(summary_line + "\n")
            f.write(f"\nTest Results by Opcode:\n")
            for opcode, stats in sorted(agg.items()):
                f.write(f"  {opcode}: {stats['p']}/{stats['t']}\n")
    except Exception as e:
        if VERBOSE:
            print(f"[warn] Failed to write {result_path}: {e}")

if __name__ == '__main__':
    main()
