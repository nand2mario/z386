#!/usr/bin/env python3
"""Regenerate pla_group_lookup() inside pla_entry.svh.

The entry-point lookup is two serial PLA passes: a first pass keyed by the
opcode byte, and (when the first pass returns a group row, entry[11:6]==0)
a second pass keyed by {row, modrm[5:3]}.  Chaining the two lookups puts ~6
extra LUT levels on the prefetch->decq critical path.

This script evaluates the first-level table offline and emits
pla_group_lookup(), which produces the 6-bit group row directly from
{data32, opcode, pe, has_0f}.  The decoder uses it to run the second-level
lookup in parallel with the first; the result is only consumed when the first
lookup confirms a group row, so non-group entries are don't-care (emitted 0).

The generated function is written into pla_entry.svh between the
"begin/end generated: pla_group_lookup" markers (appended on first run),
since it is derived data of the entry PLA.  Inputs are parsed from
pla_entry.svh and pla_control.svh in the directory above this script.
Run from anywhere:

    ./gen_pla_group.py            # refreshes pla_group_lookup in ../pla_entry.svh
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
CORE = HERE.parent


def parse_casez(path: Path, func_name: str, width: int):
    """Return ordered (pattern, value) list for a casez function body."""
    pats = []
    in_func = False
    for line in path.read_text().splitlines():
        if f"function automatic" in line and func_name in line:
            in_func = True
        if not in_func:
            continue
        m = re.search(
            rf"{width}'b([01?_]+)\s*:\s*{func_name}\s*=\s*\d+'b([01]+)\s*;", line)
        if m:
            pats.append((m.group(1).replace("_", ""), int(m.group(2), 2)))
        if "endfunction" in line:
            break
    if not pats:
        sys.exit(f"no patterns found for {func_name} in {path}")
    return pats


def casez_eval(pats, addr: int, width: int) -> int:
    s = format(addr, f"0{width}b")
    for p, v in pats:
        if all(pc in ("?", sc) for pc, sc in zip(p, s)):
            return v
    return 0  # both tables end with `default: = 0`


def main():
    entry_pats = parse_casez(CORE / "pla_entry.svh", "pla_entry_lookup", 13)
    ctl_pats = parse_casez(CORE / "pla_control.svh",
                           "pla_control_opcode_lookup", 9)

    # group_map[(has_0f, opcode)] -> {(data32, pe): row}
    group_map = {}
    for has_0f in (0, 1):
        for opcode in range(256):
            ctl = casez_eval(ctl_pats, (has_0f << 8) | opcode, 9)
            has_modrm = ((ctl >> 2) & 1) and not (ctl & 1)
            if not has_modrm:
                continue
            for data32 in (0, 1):
                for pe in (0, 1):
                    rows = set()
                    for rep in (0, 1):
                        addr = ((data32 << 12) | (opcode << 4) | (rep << 3) |
                                (pe << 2) | (1 << 1) | has_0f)
                        rows.add(casez_eval(entry_pats, addr, 13))
                    assert len(rows) == 1, \
                        f"group row depends on rep: 0f={has_0f} op={opcode:02x}"
                    entry_first = rows.pop() & 0xFFF
                    if (entry_first >> 6) == 0 and (entry_first & 0x3F) != 0:
                        group_map.setdefault((has_0f, opcode), {})[
                            (data32, pe)] = entry_first & 0x3F
    # Emit: one casez line per (has_0f, opcode), split by data32/pe only when
    # the row differs.  Address format: {data32, opcode[7:0], pe, has_0f}.
    lines = []
    for (has_0f, opcode), rows in sorted(group_map.items()):
        vals = set(rows.values())
        full = {(d, p) for d in (0, 1) for p in (0, 1)}
        covered = set(rows.keys())
        if len(vals) == 1 and covered == full:
            lines.append((f"11'b?_{opcode:08b}_?_{has_0f}", vals.pop()))
        else:
            # Try merging on each of data32 / pe alone, else emit singles.
            done = set()
            for d in (0, 1):
                sub = {p: rows.get((d, p)) for p in (0, 1)}
                if None not in sub.values() and len(set(sub.values())) == 1:
                    lines.append((f"11'b{d}_{opcode:08b}_?_{has_0f}",
                                  sub[0]))
                    done |= {(d, 0), (d, 1)}
            for p in (0, 1):
                sub = {d: rows.get((d, p)) for d in (0, 1)}
                if all((d, p) not in done for d in (0, 1)) and \
                        None not in sub.values() and len(set(sub.values())) == 1:
                    lines.append((f"11'b?_{opcode:08b}_{p}_{has_0f}",
                                  sub[0]))
                    done |= {(0, p), (1, p)}
            for key in sorted(covered - done):
                d, p = key
                lines.append((f"11'b{d}_{opcode:08b}_{p}_{has_0f}",
                              rows[key]))

    body = []
    body.append("// ---- begin generated: pla_group_lookup (scripts/gen_pla_group.py) ----")
    body.append("// Opcode -> second-level entry PLA row, derived from")
    body.append("// pla_entry_lookup above and pla_control.svh.  DO NOT EDIT BY HAND.")
    body.append("//")
    body.append("// Returns entry_first[5:0] for opcodes whose first-level entry lookup")
    body.append("// yields a group row (entry[11:6]==0 with modrm).  Lets the decoder run")
    body.append("// the second-level lookup in parallel with the first instead of chained")
    body.append("// behind it.  Result is don't-care (0) for non-group opcodes; the")
    body.append("// entry_group select still comes from the real first-level result.")
    body.append("//")
    body.append("// Input format: {data32, opcode[7:0], pe_enable, has_0f}")
    body.append("function automatic logic [5:0] pla_group_lookup(")
    body.append("    input [10:0] addr_in")
    body.append(");")
    body.append("    casez (addr_in)")
    for pat, val in lines:
        body.append(f"        {pat}: pla_group_lookup = 6'b{val:06b};")
    body.append("        default: pla_group_lookup = 6'b000000;")
    body.append("    endcase")
    body.append("endfunction")
    body.append("// ---- end generated: pla_group_lookup ----")

    out = CORE / "pla_entry.svh"
    text = out.read_text()
    begin_mark = "// ---- begin generated: pla_group_lookup"
    end_mark = "// ---- end generated: pla_group_lookup ----"
    block = "\n".join(body) + "\n"
    if begin_mark in text:
        pre = text[:text.index(begin_mark)]
        post = text[text.index(end_mark) + len(end_mark):].lstrip("\n")
        text = pre + block + post
    else:
        text = text.rstrip("\n") + "\n\n" + block
    out.write_text(text)

    # Self-check: parallel lookup == chained lookup for every input combo.
    group_pats = parse_casez(out, "pla_group_lookup", 11)
    checked = mismatches = 0
    for has_0f in (0, 1):
        for opcode in range(256):
            ctl = casez_eval(ctl_pats, (has_0f << 8) | opcode, 9)
            has_modrm = ((ctl >> 2) & 1) and not (ctl & 1)
            if not has_modrm:
                continue
            for data32 in (0, 1):
                for pe in (0, 1):
                    for rep in (0, 1):
                        addr = ((data32 << 12) | (opcode << 4) | (rep << 3) |
                                (pe << 2) | (1 << 1) | has_0f)
                        first = casez_eval(entry_pats, addr, 13) & 0xFFF
                        if (first >> 6) != 0:
                            continue
                        gaddr = (data32 << 10) | (opcode << 2) | (pe << 1) | has_0f
                        grp = casez_eval(group_pats, gaddr, 11)
                        checked += 1
                        if grp != (first & 0x3F):
                            mismatches += 1
                            print(f"MISMATCH 0f={has_0f} op={opcode:02x} "
                                  f"d32={data32} pe={pe}: row {first & 0x3F:#x} "
                                  f"!= group {grp:#x}")
    print(f"wrote {out} ({len(lines)} patterns); "
          f"self-check: {checked} group cases, {mismatches} mismatches")
    if mismatches:
        sys.exit(1)


if __name__ == "__main__":
    main()
