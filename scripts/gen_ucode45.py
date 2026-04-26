#!/usr/bin/env python3
"""Generate 45-bit microcode ROM images.

The source ucode.hex contains the original 37-bit microcode word.  z386 uses
bits 37..44 as predecoded control bits. This script expands the image offline.
"""

from __future__ import annotations

import argparse
from pathlib import Path


ROM_DEPTH = 2560
UCODE_BITS = 37
EXPANDED_BITS = 45

ALUJMP_ALU = 0x00
ALUJMP_INCDEC = 0x01
ALUJMP_CMPTST = 0x03
ALUJMP_ADC = 0x0D
ALUJMP_CMP = 0x0F
ALUJMP_AAAAAS = 0x16
ALUJMP_DAADAS = 0x17
ALUJMP_JPEREQ = 0x4E

BUSOP_RD_BW = 0x06
BUSOP_RD_D = 0x07
BUSOP_CW = 0x0A
BUSOP_WR_WORD = 0x11
BUSOP_WR = 0x12
BUSOP_RD_WORD = 0x15
BUSOP_RD = 0x16
BUSOP_RD_IND = 0x17
BUSOP_WR_OPR = 0x1A
BUSOP_IACK = 0x28


def expand_word(word: int) -> int:
    """Return the 45-bit microcode word used by z386.sv."""
    if word < 0 or word >= (1 << UCODE_BITS):
        raise ValueError(f"microcode word out of {UCODE_BITS}-bit range: 0x{word:x}")

    aluop = (word >> 11) & 0x7F
    buscode = word & 0x3F
    subcode = (word >> 6) & 0x03
    alusrc = (word >> 31) & 0x3F

    bit37 = aluop in {
        ALUJMP_ALU,
        ALUJMP_INCDEC,
        ALUJMP_CMPTST,
        ALUJMP_ADC,
        ALUJMP_CMP,
        ALUJMP_AAAAAS,
        ALUJMP_DAADAS,
    }
    bit38 = buscode in {
        BUSOP_RD_BW,
        BUSOP_RD_D,
        BUSOP_RD,
        BUSOP_RD_WORD,
        BUSOP_RD_IND,
        BUSOP_WR,
        BUSOP_WR_OPR,
        BUSOP_WR_WORD,
        BUSOP_CW,
        BUSOP_IACK,
    } or subcode == 0
    bit39 = buscode in {
        BUSOP_RD_BW,
        BUSOP_RD_D,
        BUSOP_RD,
        BUSOP_RD_WORD,
        BUSOP_RD_IND,
        BUSOP_WR,
        BUSOP_WR_OPR,
        BUSOP_WR_WORD,
        BUSOP_CW,
    }
    bit40 = buscode in {BUSOP_WR, BUSOP_WR_OPR, BUSOP_WR_WORD}
    bit41 = buscode == BUSOP_CW
    bit42 = buscode in {BUSOP_RD_WORD, BUSOP_WR_WORD}
    bit43 = buscode in {BUSOP_RD_D, BUSOP_RD_IND}
    bit44 = aluop == ALUJMP_JPEREQ and alusrc != 0x3F

    predecoded = (
        (int(bit37) << 37)
        | (int(bit38) << 38)
        | (int(bit39) << 39)
        | (int(bit40) << 40)
        | (int(bit41) << 41)
        | (int(bit42) << 42)
        | (int(bit43) << 43)
        | (int(bit44) << 44)
    )
    expanded = word | predecoded
    if expanded >= (1 << EXPANDED_BITS):
        raise AssertionError(f"expanded word out of range: 0x{expanded:x}")
    return expanded


def read_words(path: Path, *, bits: int) -> list[int]:
    words: list[int] = []
    for lineno, raw in enumerate(path.read_text().splitlines(), start=1):
        line = raw.split("//", 1)[0].split("#", 1)[0].strip()
        if not line:
            continue
        try:
            word = int(line, 16)
        except ValueError as exc:
            raise ValueError(f"{path}:{lineno}: invalid hex word {line!r}") from exc
        if word < 0 or word >= (1 << bits):
            raise ValueError(f"{path}:{lineno}: word out of {bits}-bit range: 0x{word:x}")
        words.append(word)
    if len(words) != ROM_DEPTH:
        raise ValueError(f"{path}: expected {ROM_DEPTH} words, found {len(words)}")
    return words


def write_hex(path: Path, words: list[int]) -> None:
    path.write_text("".join(f"{word:012X}\n" for word in words))


def write_mif(path: Path, words: list[int]) -> None:
    lines = [
        f"WIDTH={EXPANDED_BITS};",
        f"DEPTH={ROM_DEPTH};",
        "",
        "ADDRESS_RADIX=HEX;",
        "DATA_RADIX=HEX;",
        "",
        "CONTENT BEGIN",
    ]
    lines.extend(f"    {addr:03X} : {word:012X};" for addr, word in enumerate(words))
    lines.append("END;")
    path.write_text("\n".join(lines) + "\n")


def read_mif(path: Path) -> list[int]:
    words: list[int | None] = [None] * ROM_DEPTH
    in_content = False
    for lineno, raw in enumerate(path.read_text().splitlines(), start=1):
        line = raw.split("--", 1)[0].split("%", 1)[0].strip()
        if not line:
            continue
        upper = line.upper()
        if upper == "CONTENT BEGIN":
            in_content = True
            continue
        if upper == "END;":
            break
        if not in_content:
            continue
        if ":" not in line or not line.endswith(";"):
            raise ValueError(f"{path}:{lineno}: invalid MIF entry {line!r}")
        addr_text, word_text = line[:-1].split(":", 1)
        addr = int(addr_text.strip(), 16)
        word = int(word_text.strip(), 16)
        if addr < 0 or addr >= ROM_DEPTH:
            raise ValueError(f"{path}:{lineno}: address out of range: 0x{addr:x}")
        if word < 0 or word >= (1 << EXPANDED_BITS):
            raise ValueError(f"{path}:{lineno}: word out of {EXPANDED_BITS}-bit range: 0x{word:x}")
        words[addr] = word
    if any(word is None for word in words):
        missing = next(i for i, word in enumerate(words) if word is None)
        raise ValueError(f"{path}: missing word at address 0x{missing:x}")
    return [word for word in words if word is not None]


def read_output(path: Path) -> list[int]:
    if path.suffix.lower() == ".mif":
        return read_mif(path)
    return read_words(path, bits=EXPANDED_BITS)


def write_output(path: Path, words: list[int]) -> None:
    if path.suffix.lower() == ".mif":
        write_mif(path, words)
    else:
        write_hex(path, words)


def main() -> int:
    here = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", nargs="?", type=Path, default=here / "ucode.hex")
    parser.add_argument(
        "output",
        nargs="?",
        type=Path,
        help="optional single output path; omit to write both ucode45.hex and ucode45.mif",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="write both ucode45.hex and ucode45.mif next to the input file",
    )
    parser.add_argument("--check", action="store_true", help="fail if output is missing or stale")
    args = parser.parse_args()

    expanded = [expand_word(word) for word in read_words(args.input, bits=UCODE_BITS)]
    outputs = [args.output] if args.output is not None else [here / "ucode45.hex", here / "ucode45.mif"]
    if args.all:
        outputs = [here / "ucode45.hex", here / "ucode45.mif"]
    if args.check:
        for output in outputs:
            existing = read_output(output)
            if existing != expanded:
                raise SystemExit(f"{output} is stale; rerun {Path(__file__).name}")
        return 0

    for output in outputs:
        write_output(output, expanded)
        print(f"wrote {len(expanded)} words to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
