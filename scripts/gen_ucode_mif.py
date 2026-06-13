#!/usr/bin/env python3
"""Generate ucode.mif (37-bit Quartus ROM init) from ucode.hex.

The microcode ROM stores the original 37-bit word; predecode bits 44:37 are
computed in hardware in the ROM output register stage (see ucode_rom.sv), so
no expanded image is needed.
"""

from __future__ import annotations

import argparse
from pathlib import Path


ROM_DEPTH = 2560
UCODE_BITS = 37


def read_words(path: Path) -> list[int]:
    words: list[int] = []
    for lineno, raw in enumerate(path.read_text().splitlines(), start=1):
        line = raw.split("//", 1)[0].split("#", 1)[0].strip()
        if not line:
            continue
        try:
            word = int(line, 16)
        except ValueError as exc:
            raise ValueError(f"{path}:{lineno}: invalid hex word {line!r}") from exc
        if word < 0 or word >= (1 << UCODE_BITS):
            raise ValueError(f"{path}:{lineno}: word out of {UCODE_BITS}-bit range: 0x{word:x}")
        words.append(word)
    if len(words) != ROM_DEPTH:
        raise ValueError(f"{path}: expected {ROM_DEPTH} words, found {len(words)}")
    return words


def render_mif(words: list[int]) -> str:
    lines = [
        f"WIDTH={UCODE_BITS};",
        f"DEPTH={ROM_DEPTH};",
        "",
        "ADDRESS_RADIX=HEX;",
        "DATA_RADIX=HEX;",
        "",
        "CONTENT BEGIN",
    ]
    lines.extend(f"    {addr:03X} : {word:010X};" for addr, word in enumerate(words))
    lines.append("END;")
    return "\n".join(lines) + "\n"


def main() -> int:
    here = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", nargs="?", type=Path, default=here / "ucode.hex")
    parser.add_argument("output", nargs="?", type=Path, default=here / "ucode.mif")
    parser.add_argument("--check", action="store_true", help="fail if output is missing or stale")
    args = parser.parse_args()

    rendered = render_mif(read_words(args.input))
    if args.check:
        if not args.output.exists() or args.output.read_text() != rendered:
            raise SystemExit(f"{args.output} is stale; rerun {Path(__file__).name}")
        return 0

    args.output.write_text(rendered)
    print(f"wrote {ROM_DEPTH} words to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
