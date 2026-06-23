#!/usr/bin/env python3
"""Apply z386 microcode optimizations to the extracted 80386 CROM.

`ucode_base.hex` is the original 37-bit extracted microcode and is NEVER edited.
This script applies the documented PATCHES below and writes the optimized
`ucode.hex` (+ `ucode.mif`), so every microcode change is reproducible and
auditable in one place rather than as opaque hex edits.

37-bit word field layout (see doc/microcode/fields.txt):
    bus[5:0]  sub[7:6]  op[10:8]  aluop[17:11]  src[23:18]  dst[30:24]  alusrc[36:31]
  RNI = op field 0 (default 7);  DLY = sub field 0 (default 3).
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

ROM_DEPTH = 2560
UCODE_BITS = 37

# field name -> (shift, width)
FIELDS = {
    'bus': (0, 6), 'sub': (6, 2), 'op': (8, 3), 'aluop': (11, 7),
    'src': (18, 6), 'dst': (24, 7), 'alusrc': (31, 6),
}


def set_fields(word: int, **kw: int) -> int:
    for name, val in kw.items():
        shift, width = FIELDS[name]
        mask = ((1 << width) - 1) << shift
        word = (word & ~mask) | ((val << shift) & mask)
    return word


@dataclass
class Patch:
    addr: int
    comment: str
    fields: dict | None = None    # override these fields on the base word
    copy_from: int | None = None  # set this address's word from another base word
    word: int | None = None       # absolute 37-bit word


PATCHES = [
    # ---- Load (MOV r,m) 4 -> 3 cycles --------------------------------------
    # The PIPT dcache returns the read data by 01A: with the modrm linear
    # registered at i_pop, the 019 RD is issued/accepted at i_first and
    # resp_valid lands the next cycle (01A).  The original routine then spends
    # two more cycles -- 01B (bare RNI) and 01C (the result write).  Fold RNI
    # into the 01A DLY (exactly POP's 0A0 "RNI DLY") and move the result write
    # up into 01B, the RNI delay slot.  01C becomes unreached.
    #   patched:  019 RD / 01A RNI DLY / 01B OPR_R->DSTREG
    #   was:      019 RD / 01A DLY / 01B RNI / 01C OPR_R->DSTREG
    Patch(0x01A, "MOV r,m load 4->3: 01A DLY -> RNI DLY (data is back by 01A)",
          fields=dict(op=0)),
    Patch(0x01B, "MOV r,m load 4->3: 01B RNI -> OPR_R->DSTREG (write in RNI delay slot)",
          copy_from=0x01C),
]


def read_words(path: Path) -> list[int]:
    words: list[int] = []
    for lineno, raw in enumerate(path.read_text().splitlines(), start=1):
        line = raw.split('//', 1)[0].split('#', 1)[0].strip()
        if not line:
            continue
        word = int(line, 16)
        if not 0 <= word < (1 << UCODE_BITS):
            raise ValueError(f"{path}:{lineno}: word out of {UCODE_BITS}-bit range: 0x{word:x}")
        words.append(word)
    if len(words) != ROM_DEPTH:
        raise ValueError(f"{path}: expected {ROM_DEPTH} words, found {len(words)}")
    return words


def render_hex(words: list[int]) -> str:
    return ''.join(f"{w:010X}\n" for w in words)


def render_mif(words: list[int]) -> str:
    lines = [f"WIDTH={UCODE_BITS};", f"DEPTH={ROM_DEPTH};", "",
             "ADDRESS_RADIX=HEX;", "DATA_RADIX=HEX;", "", "CONTENT BEGIN"]
    lines += [f"    {a:03X} : {w:010X};" for a, w in enumerate(words)]
    lines.append("END;")
    return "\n".join(lines) + "\n"


def apply_patches(base: list[int]) -> list[int]:
    words = base[:]
    print(f"Applying {len(PATCHES)} microcode patch(es):")
    for p in PATCHES:
        old = words[p.addr]
        if p.word is not None:
            new = p.word
        elif p.copy_from is not None:
            new = base[p.copy_from]
        elif p.fields is not None:
            new = set_fields(old, **p.fields)
        else:
            raise ValueError(f"patch at 0x{p.addr:03X} has no action")
        words[p.addr] = new
        print(f"  0x{p.addr:03X}: {old:010X} -> {new:010X}  {p.comment}")
    return words


def main() -> int:
    here = Path(__file__).resolve().parent.parent   # 21.z386/
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", type=Path, default=here / "ucode_base.hex")
    ap.add_argument("--hex", type=Path, default=here / "ucode.hex")
    ap.add_argument("--mif", type=Path, default=here / "ucode.mif")
    ap.add_argument("--check", action="store_true",
                    help="fail if ucode.hex/.mif are stale vs base+patches")
    args = ap.parse_args()

    base = read_words(args.base)
    words = apply_patches(base)
    hex_txt, mif_txt = render_hex(words), render_mif(words)

    if args.check:
        stale = (not args.hex.exists() or args.hex.read_text() != hex_txt or
                 not args.mif.exists() or args.mif.read_text() != mif_txt)
        if stale:
            raise SystemExit("ucode.hex/.mif are stale; rerun ucode_optimize.py")
        print("ucode.hex/.mif up to date")
        return 0

    args.hex.write_text(hex_txt)
    args.mif.write_text(mif_txt)
    print(f"wrote {args.hex} and {args.mif}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
