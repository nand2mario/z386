#!/usr/bin/env python3
"""Apply z386 microcode optimizations to the extracted 80386 CROM.

`ucode_base.hex` is the original 37-bit extracted microcode and is NEVER edited.
This script applies the documented PATCHES below and writes the optimized
40-bit `ucode.hex` (+ `ucode.mif`): the native word remains in bits 36:0 and
the v52 D2 early kind occupies bits 39:37.  This script also owns the FAST
recipe inventory and generates its SystemVerilog lookup and human-readable
manifest, so microcode words and the recipes that consume them cannot silently
drift apart.

37-bit word field layout (see doc/microcode/fields.txt):
    bus[5:0]  sub[7:6]  op[10:8]  aluop[17:11]  src[23:18]  dst[30:24]  alusrc[36:31]
  RNI = op field 0 (default 7);  DLY = sub field 0 (default 3).
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path

ROM_DEPTH = 2560
UCODE_BITS = 37
ROM_BITS = 40
DEST_USTEP_ALU = 0x7E       # Optimizer-owned: commit this word's ALU result to DSTREG.

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
                                  # (fields, if also given, are applied on top)
    word: int | None = None       # absolute 37-bit word


class EarlyKind(IntEnum):
    """D2 early-start operation encoded by the future 3 ROM metadata bits."""

    SEQ = 0
    NONE = 1
    EA = 2
    LOAD = 3
    STORE = 4
    RMW = 5
    BRANCH = 6
    STACK = 7


@dataclass(frozen=True)
class Recipe:
    name: str
    entry: int
    early: EarlyKind
    legacy: tuple[tuple[int, ...], ...]
    targets: tuple[tuple[int, ...], ...]
    commit: str
    slot: str
    hazards: tuple[str, ...] = ()
    overlay_retire: bool = False


PATCHES = [
    # ---- v52 direct ALU usteps -------------------------------------------
    # Every FAST ALU retire word owns its architectural write through one
    # destination encoding. This replaces the parallel FAST_COMMIT_ALU write
    # site while leaving SEQ/fast_off to execute the original slot writeback.
    Patch(0x003, "MOV r,r ustep: commit entry ALU result to DSTREG",
          fields=dict(dst=DEST_USTEP_ALU)),
    Patch(0x005, "MOV r,imm ustep: commit entry ALU result to DSTREG",
          fields=dict(dst=DEST_USTEP_ALU)),
    Patch(0x01D, "ALU r,r ustep: commit entry ALU result to DSTREG",
          fields=dict(dst=DEST_USTEP_ALU)),
    Patch(0x021, "INC/DEC/NOT/NEG r ustep: commit entry ALU result to DSTREG",
          fields=dict(dst=DEST_USTEP_ALU)),
    Patch(0x023, "ALU r,imm ustep: commit entry ALU result to DSTREG",
          fields=dict(dst=DEST_USTEP_ALU)),

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

    # ---- ALU r,m 4 -> 3 cycles (M5 F-ALUM) --------------------------------
    # The base routine stages the memory operand through TMPB (029 OPR_R->TMPB)
    # only because the alusrc field has no OPR_R encoding.  The m,r CMPTST
    # forms (033/037) already read OPR_R directly via the source field; the
    # z386x hardware adds ALUSRC_OPR_R (0x0F, unused in the base CROM as a
    # consumed ALU source) to complete the symmetry for r,m.
    #   patched:  027 RD / 028 DLY / 029 DSTREG(op)OPR_R +-&|^ RNI / 02A slot SIGMA->DSTREG
    #   was:      027 RD / 028 DLY / 029 OPR_R->TMPB / 02A DSTREG(op)TMPB RNI / 02B slot
    # FAST shape is unchanged (multi_word, commit_sel=ALU): the RNI word moves
    # one earlier, chainN keys on uc_next content, the ALU-commit sideband
    # keys on the RNI word's aluop.  SEQ/fast_off path speeds up identically.
    Patch(0x029, "ALU r,m 4->3: 029 = ALU DSTREG,OPR_R +-&|^ RNI (was OPR_R->TMPB)",
          copy_from=0x02A, fields=dict(alusrc=0x0F, dst=DEST_USTEP_ALU)),
    Patch(0x02A, "ALU r,m 4->3: 02A = SIGMA->DSTREG (slot writeback; was the ALU word)",
          copy_from=0x02B),

    # ---- CMP r,m 4 -> 3 cycles (M5 F-ALUM, CMPTST flavor) ------------------
    #   patched:  02C RD / 02D DLY / 02E DSTREG,OPR_R CMPTST RNI / 02F blank slot
    #   was:      02C RD / 02D DLY / 02E OPR_R->TMPB / 02F DSTREG,TMPB CMPTST RNI
    # Only 3A/3B (CMP r,m, group 0x05) enter 02C; CMP m,r (38/39, group 0x04)
    # has its own already-folded routine at 035.  02F must become a blank slot
    # word: the old CMPTST there would redo flags from stale TMPB in SEQ mode.
    Patch(0x02E, "CMP r,m 4->3: 02E = CMPTST DSTREG,OPR_R RNI (was OPR_R->TMPB)",
          copy_from=0x02F, fields=dict(alusrc=0x0F)),
    Patch(0x02F, "CMP r,m 4->3: 02F = blank slot (was the CMPTST word)",
          copy_from=0x030),

    # ---- ALU m,r / m,imm RMW 6 -> 4 cycles (M5 F-RMW) ----------------------
    # Retime into the INC/DEC-m shape (04E/04F/050), which the base CROM
    # already uses: move the JMP WRITE_RESULT up into the DLY word (fires at
    # DLY release), and do the ALU in the jump delay slot reading OPR_R
    # directly via the source field (dst port = the memory operand, correct
    # m,r operand order).  The shared 046 SIGMA->OPR_W+WR+RNI / 047 DLY tail
    # and the 04A/039 FLGSBA restart backup are untouched, so write-fault
    # restart semantics are identical.  Stays SEQ (not FAST-classified).
    #   patched:  04A FLGSBA+RD / 04B DLY+JMP(046) / 04C OPR_R,SRCREG +-&|^ / 046 WR+RNI / 047
    #   was:      04A FLGSBA+RD / 04B DLY / 04C OPR_R->TMPB+JMP / 04D TMPB,SRCREG / 046 / 047
    # Jump offset: reljump target = uaddr + sext6(alusrc) with uaddr already
    # at word+1, so 04B: 0x46-0x4C = -6 = 0x3A; 03A: 0x46-0x3B = +11 = 0x0B.
    Patch(0x04B, "ALU m,r 6->4: 04B = DLY + JMP WRITE_RESULT (was pure DLY)",
          fields=dict(aluop=0x5A, alusrc=0x3A)),
    Patch(0x04C, "ALU m,r 6->4: 04C = OPR_R,SRCREG +-&|^ in jump delay slot (04D unreached)",
          copy_from=0x04D, fields=dict(src=0x2D)),
    Patch(0x03A, "ALU m,i 6->4: 03A = DLY + JMP WRITE_RESULT (was pure DLY)",
          fields=dict(aluop=0x5A, alusrc=0x0B)),
    Patch(0x03B, "ALU m,i 6->4: 03B = OPR_R,IMM +-&|^ in jump delay slot (03C unreached)",
          copy_from=0x03C, fields=dict(src=0x2D)),
]


# Current FAST routines and their bounded v52 target recipes.  `legacy` is
# descriptive: it records the current general-sequencer control paths.
# `targets` are the future explicit recipes and must contain 1..3 logical
# usteps.  A memory ustep may hold for completion, absorbing legacy DLY/JMP
# plumbing without increasing the target step count.
FAST_RECIPES = [
    Recipe("mov-r-r", 0x003, EarlyKind.NONE,
           ((0x003, 0x004),), ((0x003,),), "alu-dst", "reclaim", ("src",)),
    Recipe("mov-r-imm", 0x005, EarlyKind.NONE,
           ((0x005, 0x006),), ((0x005,),), "alu-dst", "reclaim"),
    Recipe("alu-r-r", 0x01D, EarlyKind.NONE,
           ((0x01D, 0x01E),), ((0x01D,),), "alu-dst/flags", "reclaim",
           ("dst", "src", "flags-adc-sbb")),
    Recipe("cmp-test-r-r", 0x01F, EarlyKind.NONE,
           ((0x01F, 0x020),), ((0x01F,),), "flags", "reclaim", ("dst", "src")),
    Recipe("inc-dec-not-neg-r", 0x021, EarlyKind.NONE,
           ((0x021, 0x022),), ((0x021,),), "alu-dst/flags", "reclaim", ("dst",)),
    Recipe("alu-r-imm", 0x023, EarlyKind.NONE,
           ((0x023, 0x024),), ((0x023,),), "alu-dst/flags", "reclaim",
           ("dst", "flags-adc-sbb")),
    Recipe("cmp-test-r-imm", 0x025, EarlyKind.NONE,
           ((0x025, 0x026),), ((0x025,),), "flags", "reclaim", ("dst",)),
    Recipe("lea", 0x0B9, EarlyKind.EA,
           ((0x0B9, 0x0BA),), ((0x0B9,),), "src-reg", "reclaim", ("ea",)),

    Recipe("shift-r-imm", 0x0F9, EarlyKind.NONE,
           ((0x0F9, 0x0FA, 0x0FB),), ((0x0F9, 0x0FA),), "shift-dst/flags",
           "reclaim", ("dst",)),
    Recipe("shift-r-cl", 0x0FF, EarlyKind.NONE,
           ((0x0FF, 0x100, 0x101),), ((0x0FF, 0x100),), "shift-dst/flags",
           "reclaim", ("dst", "ecx")),
    Recipe("shxd-r-imm", 0x0FC, EarlyKind.NONE,
           ((0x0FC, 0x0FD, 0x0FE),), ((0x0FC, 0x0FD),), "shift-dst/flags",
           "reclaim", ("dst", "src")),
    Recipe("shxd-r-cl", 0x102, EarlyKind.NONE,
           ((0x102, 0x103, 0x104),), ((0x102, 0x103),), "shift-dst/flags",
           "reclaim", ("dst", "src", "ecx")),
    Recipe("shift-r-one", 0x105, EarlyKind.NONE,
           ((0x105, 0x106, 0x107),), ((0x105, 0x106),), "shift-dst/flags",
           "reclaim", ("dst",)),
    Recipe("szext-r-16", 0x1E8, EarlyKind.NONE,
           ((0x1E8, 0x1E9, 0x1EA),), ((0x1E8, 0x1E9),), "sigma-src", "reclaim",
           ("dst",)),
    Recipe("szext-r-32", 0x1F0, EarlyKind.NONE,
           ((0x1F0, 0x1F1, 0x1F2),), ((0x1F0, 0x1F1),), "sigma-src", "reclaim",
           ("dst",)),

    Recipe("store-r", 0x013, EarlyKind.STORE,
           ((0x013, 0x014),), ((0x013,),), "store", "retain", ("ea", "src")),
    Recipe("store-imm", 0x015, EarlyKind.STORE,
           ((0x015, 0x016),), ((0x015,),), "store", "retain", ("ea",)),
    Recipe("load-r", 0x019, EarlyKind.LOAD,
           ((0x019, 0x01A, 0x01B),), ((0x019, 0x01A),), "mem-dst", "retain",
           ("ea",)),
    Recipe("alu-r-m", 0x027, EarlyKind.LOAD,
           ((0x027, 0x028, 0x029, 0x02A),), ((0x027, 0x029),), "alu-dst/flags",
           "reclaim", ("ea",)),
    Recipe("cmp-r-m", 0x02C, EarlyKind.LOAD,
           ((0x02C, 0x02D, 0x02E, 0x02F),), ((0x02C, 0x02E),), "flags",
           "reclaim", ("ea",)),
    Recipe("cmp-test-m-imm", 0x031, EarlyKind.LOAD,
           ((0x031, 0x032, 0x033, 0x034),), ((0x031, 0x033),), "flags",
           "reclaim", ("ea",)),
    Recipe("cmp-test-m-r", 0x035, EarlyKind.LOAD,
           ((0x035, 0x036, 0x037, 0x038),), ((0x035, 0x037),), "flags",
           "reclaim", ("ea", "src")),
    Recipe("rmw-m-imm", 0x039, EarlyKind.RMW,
           ((0x039, 0x03A, 0x03B, 0x046, 0x047),), ((0x039, 0x03B, 0x046),),
           "store/flags", "retain", ("ea",)),
    Recipe("rmw-m-r", 0x04A, EarlyKind.RMW,
           ((0x04A, 0x04B, 0x04C, 0x046, 0x047),), ((0x04A, 0x04C, 0x046),),
           "store/flags", "retain", ("ea", "src")),
    Recipe("szext-m-16", 0x1EB, EarlyKind.LOAD,
           ((0x1EB, 0x1EC, 0x1ED, 0x1EE, 0x1EF),), ((0x1EB, 0x1ED, 0x1EE),),
           "sigma-src", "reclaim", ("ea",)),
    Recipe("szext-m-32", 0x1F3, EarlyKind.LOAD,
           ((0x1F3, 0x1F4, 0x1F5, 0x1F6, 0x1F7),), ((0x1F3, 0x1F5, 0x1F6),),
           "sigma-src", "reclaim", ("ea",)),

    Recipe("jcc-rel", 0x065, EarlyKind.BRANCH,
           ((0x065, 0x066), (0x065, 0x067, 0x068)),
           ((0x065,), (0x065, 0x068)), "eip", "conditional", ("flags",), True),
    Recipe("jmp-rel", 0x06A, EarlyKind.BRANCH,
           ((0x06A, 0x06B, 0x06C, 0x067, 0x068),), ((0x06A, 0x068),),
           "eip", "reclaim"),
    Recipe("call-rel", 0x075, EarlyKind.STACK,
           ((0x075, 0x076, 0x077, 0x078, 0x067, 0x068),),
           ((0x075, 0x077, 0x068),), "store/esp/eip", "reclaim", ("stack",)),
    Recipe("ret-near", 0x072, EarlyKind.STACK,
           ((0x072, 0x073, 0x074, 0x06B, 0x06C, 0x067, 0x068),),
           ((0x072, 0x074, 0x068),), "esp/eip", "reclaim", ("stack",)),
    Recipe("push-r", 0x086, EarlyKind.STACK,
           ((0x086, 0x087),), ((0x086,),), "store/esp", "retain", ("stack", "dst")),
    Recipe("push-seg", 0x09B, EarlyKind.STACK,
           ((0x09B, 0x09C),), ((0x09B,),), "store/esp", "retain", ("stack",)),
    Recipe("push-imm", 0x09D, EarlyKind.STACK,
           ((0x09D, 0x09E),), ((0x09D,),), "store/esp", "retain", ("stack",)),
    Recipe("pop-r", 0x09F, EarlyKind.STACK,
           ((0x09F, 0x0A0, 0x0A1),), ((0x09F, 0x0A0),), "mem-dst/esp", "retain",
           ("stack",)),
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
    lines = [f"WIDTH={ROM_BITS};", f"DEPTH={ROM_DEPTH};", "",
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
            if p.fields is not None:
                new = set_fields(new, **p.fields)
        elif p.fields is not None:
            new = set_fields(old, **p.fields)
        else:
            raise ValueError(f"patch at 0x{p.addr:03X} has no action")
        words[p.addr] = new
        print(f"  0x{p.addr:03X}: {old:010X} -> {new:010X}  {p.comment}")
    return words


def annotate_recipes(words: list[int]) -> list[int]:
    """Attach the D2 early kind to each FAST entry word."""
    annotated = words[:]
    for recipe in FAST_RECIPES:
        annotated[recipe.entry] |= int(recipe.early) << UCODE_BITS
    if any(word >= (1 << ROM_BITS) for word in annotated):
        raise ValueError(f"annotated word exceeds {ROM_BITS}-bit ROM width")
    return annotated


def get_field(word: int, name: str) -> int:
    shift, width = FIELDS[name]
    return (word >> shift) & ((1 << width) - 1)


def fmt_path(path: tuple[int, ...]) -> str:
    return " ".join(f"{addr:03X}" for addr in path)


def validate_recipes(words: list[int]) -> None:
    default_word = (1 << UCODE_BITS) - 1
    entries: dict[int, str] = {}

    for recipe in FAST_RECIPES:
        if recipe.entry in entries:
            raise ValueError(
                f"recipe {recipe.name}: entry 0x{recipe.entry:03X} already used by "
                f"{entries[recipe.entry]}"
            )
        entries[recipe.entry] = recipe.name

        if recipe.early == EarlyKind.SEQ:
            raise ValueError(f"recipe {recipe.name}: FAST recipe cannot use SEQ early kind")
        if not recipe.legacy or not recipe.targets:
            raise ValueError(f"recipe {recipe.name}: legacy and target paths are required")

        for label, paths in (("legacy", recipe.legacy), ("target", recipe.targets)):
            for path in paths:
                if not path or path[0] != recipe.entry:
                    raise ValueError(
                        f"recipe {recipe.name}: {label} path must start at entry "
                        f"0x{recipe.entry:03X}: {fmt_path(path)}"
                    )
                if label == "target" and not 1 <= len(path) <= 3:
                    raise ValueError(
                        f"recipe {recipe.name}: target has {len(path)} usteps, expected 1..3"
                    )
                for addr in path:
                    if not 0 <= addr < ROM_DEPTH:
                        raise ValueError(f"recipe {recipe.name}: address 0x{addr:X} out of ROM")
                    # A legacy path may intentionally execute a blank delay
                    # slot.  Target usteps must all perform explicit work.
                    if label == "target" and words[addr] == default_word:
                        raise ValueError(
                            f"recipe {recipe.name}: address 0x{addr:03X} is an unused ROM word"
                        )

        if not recipe.overlay_retire:
            for path in recipe.targets:
                if get_field(words[path[-1]], "op") != 0:
                    raise ValueError(
                        f"recipe {recipe.name}: final target word 0x{path[-1]:03X} is not RNI"
                    )


def render_recipe_manifest(words: list[int]) -> str:
    validate_recipes(words)
    lines = [
        "# z386x FAST recipe manifest",
        "",
        "Generated by `scripts/ucode_optimize.py`; do not edit manually.",
        "Legacy paths describe the current sequencer. Target paths are the v52",
        "bounded recipes; a memory ustep may hold while its request completes.",
        "",
        "| recipe | entry | early | target usteps | legacy paths | commit | slot | hazards |",
        "| --- | ---: | --- | --- | --- | --- | --- | --- |",
    ]
    for recipe in FAST_RECIPES:
        targets = " / ".join(fmt_path(path) for path in recipe.targets)
        legacy = " / ".join(fmt_path(path) for path in recipe.legacy)
        hazards = ", ".join(recipe.hazards) if recipe.hazards else "-"
        lines.append(
            f"| `{recipe.name}` | `{recipe.entry:03X}` | `{recipe.early.name}` | "
            f"`{targets}` | `{legacy}` | `{recipe.commit}` | `{recipe.slot}` | "
            f"{hazards} |"
        )
    lines += [
        "",
        f"Recipes: {len(FAST_RECIPES)}. Native microcode remains {UCODE_BITS}-bit.",
        "The generated 40-bit ROM image stores the D2 early kind in bits 39:37.",
        "",
    ]
    return "\n".join(lines)


def render_recipe_svh(words: list[int]) -> str:
    validate_recipes(words)
    recipes = {recipe.name: recipe for recipe in FAST_RECIPES}
    emitted: set[str] = set()

    def recipe_entries(*names: str) -> str:
        emitted.update(names)
        return ", ".join(f"12'h{recipes[name].entry:03X}" for name in names)

    lines = [
        "// Generated by scripts/ucode_optimize.py; do not edit.",
        "localparam logic [2:0] RECIPE_EARLY_SEQ    = 3'd0;",
        "localparam logic [2:0] RECIPE_EARLY_NONE   = 3'd1;",
        "localparam logic [2:0] RECIPE_EARLY_EA     = 3'd2;",
        "localparam logic [2:0] RECIPE_EARLY_LOAD   = 3'd3;",
        "localparam logic [2:0] RECIPE_EARLY_STORE  = 3'd4;",
        "localparam logic [2:0] RECIPE_EARLY_RMW    = 3'd5;",
        "localparam logic [2:0] RECIPE_EARLY_BRANCH = 3'd6;",
        "localparam logic [2:0] RECIPE_EARLY_STACK  = 3'd7;",
        "",
        "function automatic logic [2:0] recipe_early_kind(input logic [11:0] entry);",
        "    unique case (entry)",
    ]
    by_kind: dict[EarlyKind, list[int]] = {}
    for recipe in FAST_RECIPES:
        by_kind.setdefault(recipe.early, []).append(recipe.entry)
    for kind in EarlyKind:
        if kind == EarlyKind.SEQ or kind not in by_kind:
            continue
        entry_list = ", ".join(f"12'h{addr:03X}" for addr in sorted(by_kind[kind]))
        lines.append(f"        {entry_list}: recipe_early_kind = RECIPE_EARLY_{kind.name};")
    lines += [
        "        default: recipe_early_kind = RECIPE_EARLY_SEQ;",
        "    endcase",
        "endfunction",
        "",
        "// Entry-point-derived FAST control for v52 A2. The legacy",
        "// dec_fast_class() remains a simulation-only equivalence oracle.",
        "function automatic fast_class_t recipe_fast_class(input dec_entry_t e);",
        "    fast_class_t r;",
        "    logic [2:0] grp;",
        "    r = '0;",
        "    if (e.opcode[7:4] == 4'h4 || e.opcode[7:4] == 4'h5)",
        "        r.op_byte = 1'b0;",
        "    else if (e.opcode[7:4] == 4'hB && !e.has_0f)",
        "        r.op_byte = !e.opcode[3];",
        "    else",
        "        r.op_byte = !e.opcode[0];",
        "    grp = ((e.opcode == 8'h80) || (e.opcode == 8'h81) ||",
        "           (e.opcode == 8'h83)) ? e.modrm[5:3] : e.opcode[5:3];",
        "    if (e.rep_lock == PREFIX_NOREPLOCK) begin",
        "        unique case (e.entry_point)",
        f"            {recipe_entries('mov-r-r')}: begin",
        "                r.fast = 1'b1; r.commit_sel = FAST_COMMIT_ALU;",
        "                r.reads_src = 1'b1;",
        "            end",
        f"            {recipe_entries('mov-r-imm')}: begin",
        "                if ((e.opcode[7:4] == 4'hB) ||",
        "                    ((e.opcode[7:1] == 7'b1100011) &&",
        "                     (e.modrm[7:6] == 2'b11) && (e.modrm[5:3] == 3'b000))) begin",
        "                    r.fast = 1'b1; r.commit_sel = FAST_COMMIT_ALU;",
        "                end",
        "            end",
        f"            {recipe_entries('alu-r-r')}: begin",
        "                r.fast = 1'b1; r.commit_sel = FAST_COMMIT_ALU;",
        "                r.reads_flags = (grp == 3'b010) || (grp == 3'b011);",
        "                r.reads_dst = 1'b1; r.reads_src = 1'b1;",
        "                r.writes_flags = 1'b1;",
        "            end",
        f"            {recipe_entries('cmp-test-r-r')}: begin",
        "                r.fast = 1'b1; r.reads_dst = 1'b1; r.reads_src = 1'b1;",
        "                r.writes_flags = 1'b1;",
        "            end",
        f"            {recipe_entries('inc-dec-not-neg-r')}: begin",
        "                r.fast = 1'b1; r.commit_sel = FAST_COMMIT_ALU;",
        "                r.reads_dst = 1'b1; r.writes_flags = 1'b1;",
        "            end",
        f"            {recipe_entries('alu-r-imm')}: begin",
        "                if (!e.has_0f && (((e.opcode[7:6] == 2'b00) &&",
        "                    (e.opcode[2:1] == 2'b10)) ||",
        "                    (((e.opcode == 8'h80) || (e.opcode == 8'h81) ||",
        "                      (e.opcode == 8'h83)) && (e.modrm[7:6] == 2'b11) &&",
        "                     (grp != 3'b111)))) begin",
        "                    r.fast = 1'b1; r.commit_sel = FAST_COMMIT_ALU;",
        "                    r.reads_flags = (grp == 3'b010) || (grp == 3'b011);",
        "                    r.reads_dst = 1'b1; r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('cmp-test-r-imm')}: begin",
        "                if (!e.has_0f && (((e.opcode[7:6] == 2'b00) &&",
        "                    (e.opcode[2:1] == 2'b10) && (grp == 3'b111)) ||",
        "                    (((e.opcode == 8'h80) || (e.opcode == 8'h81) ||",
        "                      (e.opcode == 8'h83)) && (e.modrm[7:6] == 2'b11) &&",
        "                     (grp == 3'b111)) || (e.opcode[7:1] == 7'b1010100) ||",
        "                    ((e.opcode[7:1] == 7'b1111011) &&",
        "                     (e.modrm[7:6] == 2'b11) && (e.modrm[5:3] == 3'b000)))) begin",
        "                    r.fast = 1'b1; r.reads_dst = 1'b1;",
        "                    r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('lea')}: begin",
        "                if (e.has_modrm && (e.modrm[7:6] != 2'b11)) begin",
        "                    r.fast = 1'b1; r.uses_ea = 1'b1;",
        "                    r.writes_srcreg = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('shift-r-imm', 'shift-r-one')}: begin",
        "                if (!e.has_0f && (e.modrm[7:6] == 2'b11) &&",
        "                    (e.modrm[5:3] != 3'b010) && (e.modrm[5:3] != 3'b011) &&",
        "                    (e.modrm[5:3] != 3'b110)) begin",
        "                    r.fast = 1'b1; r.multi_word = 1'b1;",
        "                    r.commit_sel = FAST_COMMIT_SHIFT; r.reads_dst = 1'b1;",
        "                    r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('shift-r-cl')}: begin",
        "                if (!e.has_0f && (e.modrm[7:6] == 2'b11) &&",
        "                    (e.modrm[5:3] != 3'b010) && (e.modrm[5:3] != 3'b011) &&",
        "                    (e.modrm[5:3] != 3'b110)) begin",
        "                    r.fast = 1'b1; r.multi_word = 1'b1;",
        "                    r.commit_sel = FAST_COMMIT_SHIFT; r.reads_dst = 1'b1;",
        "                    r.reads_ecx = 1'b1; r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('shxd-r-imm')}: begin",
        "                if (e.has_0f && (e.modrm[7:6] == 2'b11) &&",
        "                    ((e.opcode == 8'hA4) || (e.opcode == 8'hAC))) begin",
        "                    r.fast = 1'b1; r.multi_word = 1'b1;",
        "                    r.commit_sel = FAST_COMMIT_SHIFT;",
        "                    r.reads_dst = 1'b1; r.reads_src = 1'b1;",
        "                    r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('shxd-r-cl')}: begin",
        "                if (e.has_0f && (e.modrm[7:6] == 2'b11) &&",
        "                    ((e.opcode == 8'hA5) || (e.opcode == 8'hAD))) begin",
        "                    r.fast = 1'b1; r.multi_word = 1'b1;",
        "                    r.commit_sel = FAST_COMMIT_SHIFT;",
        "                    r.reads_dst = 1'b1; r.reads_src = 1'b1;",
        "                    r.reads_ecx = 1'b1; r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('szext-r-16', 'szext-r-32')}: begin",
        "                r.fast = 1'b1; r.multi_word = 1'b1;",
        "                r.commit_sel = FAST_COMMIT_SIGSRC;",
        "                r.writes_srcreg = 1'b1; r.reads_dst = 1'b1;",
        "            end",
        f"            {recipe_entries('store-r')}: begin",
        "                r.fast = 1'b1; r.keep_slot = 1'b1; r.uses_ea = 1'b1;",
        "                r.reads_src = 1'b1;",
        "            end",
        f"            {recipe_entries('store-imm')}: begin",
        "                if ((e.opcode[7:1] == 7'b1100011) &&",
        "                    (e.modrm[7:6] != 2'b11) && (e.modrm[5:3] == 3'b000)) begin",
        "                    r.fast = 1'b1; r.keep_slot = 1'b1; r.uses_ea = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('load-r')}: begin",
        "                if (!e.has_0f && (((e.opcode[7:2] == 6'b100010) &&",
        "                    e.opcode[1] && (e.modrm[7:6] != 2'b11)) ||",
        "                    ((e.opcode[7:2] == 6'b101000) && !e.opcode[1]))) begin",
        "                    r.fast = 1'b1; r.multi_word = 1'b1;",
        "                    r.commit_sel = FAST_COMMIT_MEM; r.keep_slot = 1'b1;",
        "                    r.uses_ea = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('alu-r-m')}: begin",
        "                if (!e.has_0f && (e.opcode[7:6] == 2'b00) &&",
        "                    !e.opcode[2] && e.opcode[1] && e.has_modrm &&",
        "                    (e.modrm[7:6] != 2'b11) && (grp != 3'b111)) begin",
        "                    r.fast = 1'b1; r.multi_word = 1'b1;",
        "                    r.commit_sel = FAST_COMMIT_ALU; r.uses_ea = 1'b1;",
        "                    r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('cmp-r-m', 'cmp-test-m-r')}: begin",
        "                if (!e.has_0f) begin",
        "                    r.fast = 1'b1; r.multi_word = 1'b1;",
        "                    r.uses_ea = 1'b1; r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('cmp-test-m-imm')}: begin",
        "                if (!e.has_0f && (((e.opcode == 8'h80) ||",
        "                    (e.opcode == 8'h81) || (e.opcode == 8'h83)) &&",
        "                    (e.modrm[7:6] != 2'b11) && (grp == 3'b111) ||",
        "                    ((e.opcode[7:1] == 7'b1111011) &&",
        "                     (e.modrm[7:6] != 2'b11) &&",
        "                     (e.modrm[5:3] == 3'b000)))) begin",
        "                    r.fast = 1'b1; r.multi_word = 1'b1;",
        "                    r.uses_ea = 1'b1; r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('rmw-m-imm')}: begin",
        "                if (!e.has_0f && ((e.opcode == 8'h80) ||",
        "                    (e.opcode == 8'h81) || (e.opcode == 8'h83)) &&",
        "                    (e.modrm[7:6] != 2'b11) && (grp != 3'b111)) begin",
        "                    r.fast = 1'b1; r.multi_word = 1'b1;",
        "                    r.keep_slot = 1'b1; r.uses_ea = 1'b1;",
        "                    r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('rmw-m-r')}: begin",
        "                if (!e.has_0f) begin",
        "                    r.fast = 1'b1; r.multi_word = 1'b1;",
        "                    r.keep_slot = 1'b1; r.uses_ea = 1'b1;",
        "                    r.writes_flags = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('szext-m-16', 'szext-m-32')}: begin",
        "                r.fast = 1'b1; r.multi_word = 1'b1;",
        "                r.commit_sel = FAST_COMMIT_SIGSRC;",
        "                r.uses_ea = 1'b1; r.writes_srcreg = 1'b1;",
        "            end",
        f"            {recipe_entries('jcc-rel')}: begin",
        "                r.fast = 1'b1; r.multi_word = 1'b1; r.jcc = 1'b1;",
        "                r.reads_flags = 1'b1; r.br_rel = 1'b1;",
        "            end",
        f"            {recipe_entries('jmp-rel')}: begin",
        "                r.fast = 1'b1; r.multi_word = 1'b1; r.br_rel = 1'b1;",
        "            end",
        f"            {recipe_entries('call-rel')}: begin",
        "                r.fast = 1'b1; r.multi_word = 1'b1;",
        "                if (e.data32) r.commit_sel = FAST_COMMIT_ESP;",
        "                r.uses_ea = 1'b1; r.br_rel = 1'b1;",
        "            end",
        f"            {recipe_entries('ret-near')}: begin",
        "                r.fast = 1'b1; r.multi_word = 1'b1; r.uses_ea = 1'b1;",
        "            end",
        f"            {recipe_entries('push-r')}: begin",
        "                if (e.opcode[7:3] == 5'b01010) begin",
        "                    r.fast = 1'b1; r.commit_sel = FAST_COMMIT_ESP;",
        "                    r.keep_slot = 1'b1; r.uses_ea = 1'b1;",
        "                    r.reads_dst = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('push-seg')}: begin",
        "                if ((e.opcode[7:6] == 2'b00) && (e.opcode[2:0] == 3'b110)) begin",
        "                    r.fast = 1'b1; r.commit_sel = FAST_COMMIT_ESP;",
        "                    r.keep_slot = 1'b1; r.uses_ea = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('push-imm')}: begin",
        "                if ((e.opcode == 8'h68) || (e.opcode == 8'h6A)) begin",
        "                    r.fast = 1'b1; r.commit_sel = FAST_COMMIT_ESP;",
        "                    r.keep_slot = 1'b1; r.uses_ea = 1'b1;",
        "                end",
        "            end",
        f"            {recipe_entries('pop-r')}: begin",
        "                if (e.opcode[7:3] == 5'b01011) begin",
        "                    r.fast = 1'b1; r.multi_word = 1'b1;",
        "                    r.commit_sel = FAST_COMMIT_MEM; r.keep_slot = 1'b1;",
        "                    r.uses_ea = 1'b1;",
        "                end",
        "            end",
        "            default: ;",
        "        endcase",
        "    end",
        "    recipe_fast_class = r;",
        "endfunction",
        "",
    ]
    missing = set(recipes) - emitted
    if missing:
        raise ValueError(f"FAST recipes missing from recipe_fast_class: {sorted(missing)}")
    return "\n".join(lines)


def main() -> int:
    here = Path(__file__).resolve().parent.parent   # Core directory.
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--base", type=Path, default=here / "ucode_base.hex")
    ap.add_argument("--hex", type=Path, default=here / "ucode.hex")
    ap.add_argument("--mif", type=Path, default=here / "ucode.mif")
    ap.add_argument("--recipe-svh", type=Path, default=here / "ucode_recipes.svh")
    ap.add_argument("--recipe-manifest", type=Path, default=here / "ucode_recipes.md")
    ap.add_argument("--check", action="store_true",
                    help="fail if generated microcode or recipe files are stale")
    args = ap.parse_args()

    base = read_words(args.base)
    words = apply_patches(base)
    validate_recipes(words)
    rom_words = annotate_recipes(words)
    hex_txt, mif_txt = render_hex(rom_words), render_mif(rom_words)
    recipe_svh_txt = render_recipe_svh(words)
    recipe_manifest_txt = render_recipe_manifest(words)

    if args.check:
        stale = (not args.hex.exists() or args.hex.read_text() != hex_txt or
                 not args.mif.exists() or args.mif.read_text() != mif_txt or
                 not args.recipe_svh.exists() or
                 args.recipe_svh.read_text() != recipe_svh_txt or
                 not args.recipe_manifest.exists() or
                 args.recipe_manifest.read_text() != recipe_manifest_txt)
        if stale:
            raise SystemExit("generated microcode/recipe files are stale; rerun ucode_optimize.py")
        print("generated microcode and recipe files are up to date")
        return 0

    args.hex.write_text(hex_txt)
    args.mif.write_text(mif_txt)
    args.recipe_svh.write_text(recipe_svh_txt)
    args.recipe_manifest.write_text(recipe_manifest_txt)
    print(f"wrote {args.hex}, {args.mif}, {args.recipe_svh}, and {args.recipe_manifest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
