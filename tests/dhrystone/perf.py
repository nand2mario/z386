#!/usr/bin/env python3
"""Collect CPI-oriented statistics from Dhrystone FST traces.

The default measurement window is the Dhrystone benchmark loop, bounded by the
MARK_START/MARK_END writes to the test data port.  If those markers are not
present in the waveform, the script falls back to the whole trace.
"""

from __future__ import annotations

import argparse
import bisect
import re
import statistics
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path


THIS_DIR = Path(__file__).resolve().parent
DEFAULT_FST = THIS_DIR / "build" / "z386_current" / "dhrystone.fst"
DEFAULT_LST = THIS_DIR / "build" / "dhrystone.lst"
DEFAULT_CODE_BASE = 0x00010000
MARK_START_MASK = 0xFFFF0000
MARK_START = 0xD0010000
MARK_END = 0xD0020000


@dataclass(frozen=True)
class InstructionInfo:
    addr: int
    length: int
    asm: str
    function: str


@dataclass(frozen=True)
class FunctionInfo:
    addr: int
    name: str


@dataclass(frozen=True)
class InstructionEvent:
    time: int
    pc: int
    eip_next: int
    length: int
    opcode: int


@dataclass(frozen=True)
class CycleSample:
    time: int
    pf_empty: int
    pf_count: int
    decq_empty: int
    decq_count: int | None
    stall: int
    stall_mem: int
    mem_servicing: int
    mem_complete_now: int
    uc_is_dly: int
    uc_addr: int
    q_flush: int
    dcache_accept: int | None
    dcache_lookup_hit: int | None
    dcache_req_valid: int | None
    dcache_req_uncacheable: int | None
    dcache_state: int | None
    dcache_storeq_count: int | None
    icache_accept: int | None
    icache_lookup_hit: int | None
    icache_req_valid: int | None
    icache_req_uncacheable: int | None
    icache_state: int | None


def clean_signal_name(name: str) -> str:
    clean = name.rstrip()
    if " [" in clean:
        clean = clean[: clean.index(" [")]
    return clean


def parse_fst_int(raw: str) -> int:
    raw = raw.strip()
    if not raw:
        return 0
    raw = raw.replace("x", "0").replace("X", "0")
    raw = raw.replace("z", "0").replace("Z", "0")
    raw = raw.replace("u", "0").replace("U", "0")
    raw = raw.replace("w", "0").replace("W", "0")
    raw = raw.replace("l", "0").replace("L", "0")
    raw = raw.replace("h", "0").replace("H", "0")
    raw = raw.replace("-", "0")
    if all(ch in "01" for ch in raw):
        return int(raw, 2)
    return int(raw, 16)


def fmt_addr(value: int | None) -> str:
    if value is None:
        return "?"
    return f"{value:05x}"


def fmt_pct(numer: int | float, denom: int | float) -> str:
    if not denom:
        return "  n/a"
    return f"{100.0 * numer / denom:5.1f}%"


def mean(values: list[float]) -> float:
    return statistics.mean(values) if values else 0.0


def parse_int(text: str) -> int:
    return int(text, 0)


def load_listing(path: Path) -> tuple[dict[int, InstructionInfo], list[FunctionInfo]]:
    instrs: dict[int, InstructionInfo] = {}
    funcs: list[FunctionInfo] = []
    current_func = "?"

    sym_re = re.compile(r"^\s*([0-9a-fA-F]+)\s+<([^>]+)>:")
    insn_re = re.compile(r"^\s*([0-9a-fA-F]+):\s+((?:[0-9a-fA-F]{2}\s+)+)\s*(.*)$")

    if not path.exists():
        return instrs, funcs

    for line in path.read_text(errors="replace").splitlines():
        sym_match = sym_re.match(line)
        if sym_match:
            current_func = sym_match.group(2)
            funcs.append(FunctionInfo(int(sym_match.group(1), 16), current_func))
            continue

        insn_match = insn_re.match(line)
        if not insn_match:
            continue
        addr = int(insn_match.group(1), 16)
        byte_text = insn_match.group(2)
        length = len(byte_text.split())
        asm = " ".join(insn_match.group(3).split())
        instrs[addr] = InstructionInfo(addr, length, asm, current_func)

    funcs.sort(key=lambda item: item.addr)
    return instrs, funcs


def resolve_offset(pc: int, instrs: dict[int, InstructionInfo], code_base: int) -> int:
    if pc in instrs:
        return pc
    rel = pc - code_base
    if rel in instrs:
        return rel
    return rel if rel >= 0 else pc


def resolve_function(offset: int, funcs: list[FunctionInfo]) -> str:
    if not funcs:
        return "?"
    addrs = [func.addr for func in funcs]
    idx = bisect.bisect_right(addrs, offset) - 1
    return funcs[idx].name if idx >= 0 else "?"


def find_handle(signals, candidates: list[str]) -> tuple[int | None, str | None]:
    names = [(clean_signal_name(name), info.handle) for name, info in signals.by_name.items()]
    for candidate in candidates:
        for clean, handle in names:
            if clean.endswith(candidate):
                return int(handle), clean
    for candidate in candidates:
        for clean, handle in names:
            if candidate in clean:
                return int(handle), clean
    return None, None


def watched_signals(signals) -> tuple[dict[str, int], dict[str, str], set[str]]:
    candidates = {
        "clk": [".tb_dhrystone.clk", ".dut.clk", ".z386_cpu.clk", ".clk"],
        "eip": [".dut.EIP", ".z386_cpu.EIP"],
        "ilen": [".dut.i.length", ".z386_cpu.i.length"],
        "opcode": [".dut.i.opcode", ".z386_cpu.i.opcode", ".dut.i_bus.opcode"],
        "ibus_len": [".dut.i_bus.length", ".z386_cpu.i_bus.length", ".decoder_inst.i_bus.length"],
        "ibus_opcode": [".dut.i_bus.opcode", ".z386_cpu.i_bus.opcode", ".decoder_inst.i_bus.opcode"],
        "i_first": [".dut.i_first", ".z386_cpu.i_first"],
        "i_pop": [".dut.i_pop", ".z386_cpu.i_pop"],
        "uc_addr": [".dut.uc_addr", ".z386_cpu.uc_addr"],
        "uc_is_dly": [".dut.uc_is_dly", ".z386_cpu.uc_is_dly"],
        "stall": [".dut.stall", ".z386_cpu.stall"],
        "stall_mem": [".dut.stall_mem", ".z386_cpu.stall_mem"],
        "mem_servicing": [".dut.mem_servicing", ".z386_cpu.mem_servicing"],
        "mem_complete_now": [".dut.mem_complete_now", ".z386_cpu.mem_complete_now"],
        "pf_empty": [".dut.pf_empty", ".z386_cpu.pf_empty", ".decoder_inst.pf_empty"],
        "pf_count": [".dut.pf_count", ".z386_cpu.pf_count", ".prefetch_inst.pf_count"],
        "decq_empty": [".dut.decq_empty", ".z386_cpu.decq_empty", ".decoder_inst.decq_empty"],
        "decq_count": [".dut.decoder_inst.decq_count", ".z386_cpu.decoder_inst.decq_count"],
        "q_flush": [".dut.q_flush", ".z386_cpu.q_flush", ".prefetch_inst.q_flush"],
        "test_data": [".tb_dhrystone.test_data"],
        "dcache_accept": [".dcache_inst.accept_cpu", ".l1_cache_inst.accept_cpu"],
        "dcache_lookup_hit": [".dcache_inst.lookup_hit", ".l1_cache_inst.lookup_hit"],
        "dcache_req_valid": [".dcache_inst.req_valid_r", ".l1_cache_inst.req_valid_r"],
        "dcache_req_uncacheable": [".dcache_inst.req_uncacheable_r", ".l1_cache_inst.req_uncacheable_r"],
        "dcache_state": [".dcache_inst.state", ".l1_cache_inst.state"],
        "dcache_storeq_count": [".dcache_inst.storeq_count", ".l1_cache_inst.storeq_count"],
        "icache_accept": [".icache_inst.accept_cpu"],
        "icache_lookup_hit": [".icache_inst.lookup_hit"],
        "icache_req_valid": [".icache_inst.req_valid_r"],
        "icache_req_uncacheable": [".icache_inst.req_uncacheable_r"],
        "icache_state": [".icache_inst.state"],
    }

    required = {"clk", "eip", "i_first"}
    handles: dict[str, int] = {}
    names: dict[str, str] = {}
    missing: set[str] = set()
    for key, key_candidates in candidates.items():
        handle, name = find_handle(signals, key_candidates)
        if handle is None:
            missing.add(key)
            continue
        handles[key] = handle
        names[key] = name or "?"

    missing_required = sorted(required & missing)
    if missing_required:
        raise SystemExit(f"missing required signal(s): {', '.join(missing_required)}")

    return handles, names, missing


def infer_period(samples: list[CycleSample]) -> float:
    if len(samples) < 2:
        return 1.0
    deltas = [b.time - a.time for a, b in zip(samples, samples[1:])]
    return float(statistics.median(deltas[: min(64, len(deltas))]))


def read_trace(
    path: Path,
    boundary: str,
) -> tuple[list[CycleSample], list[InstructionEvent], list[int], list[tuple[int, int]], dict[str, str], set[str]]:
    try:
        import pylibfst
    except ImportError as exc:
        raise SystemExit("pylibfst is required to read FST traces") from exc

    fst = pylibfst.lib.fstReaderOpen(str(path).encode("utf-8"))
    if fst == pylibfst.ffi.NULL:
        raise SystemExit(f"cannot open FST trace: {path}")

    _scopes, signals = pylibfst.get_scopes_signals2(fst)
    handles, names, missing = watched_signals(signals)
    inverse = {handle: key for key, handle in handles.items()}

    pylibfst.lib.fstReaderClrFacProcessMaskAll(fst)
    for handle in handles.values():
        pylibfst.lib.fstReaderSetFacProcessMask(fst, handle)

    state = {key: 0 for key in handles}
    samples: list[CycleSample] = []
    instrs: list[InstructionEvent] = []
    flush_times: list[int] = []
    markers: list[tuple[int, int]] = []
    current_time: int | None = None
    pending: dict[str, int] = {}

    def val(key: str, default: int = 0) -> int:
        return state.get(key, default)

    def opt(key: str) -> int | None:
        return state.get(key) if key in handles else None

    if boundary not in handles:
        if boundary == "i_pop" and "i_first" in handles:
            boundary = "i_first"
        else:
            raise SystemExit(f"boundary signal is unavailable: {boundary}")

    def flush_pending() -> None:
        nonlocal pending
        if current_time is None or not pending:
            return

        old = {key: state.get(key, 0) for key in pending}
        state.update(pending)

        if "test_data" in pending:
            marker = val("test_data")
            if marker & MARK_START_MASK in (MARK_START, MARK_END):
                markers.append((current_time, marker))

        if old.get("q_flush", 0) == 0 and val("q_flush") == 1:
            flush_times.append(current_time)

        if old.get("clk", 0) == 0 and val("clk") == 1:
            samples.append(
                CycleSample(
                    time=current_time,
                    pf_empty=val("pf_empty"),
                    pf_count=val("pf_count"),
                    decq_empty=val("decq_empty"),
                    decq_count=opt("decq_count"),
                    stall=val("stall"),
                    stall_mem=val("stall_mem"),
                    mem_servicing=val("mem_servicing"),
                    mem_complete_now=val("mem_complete_now"),
                    uc_is_dly=val("uc_is_dly"),
                    uc_addr=val("uc_addr"),
                    q_flush=val("q_flush"),
                    dcache_accept=opt("dcache_accept"),
                    dcache_lookup_hit=opt("dcache_lookup_hit"),
                    dcache_req_valid=opt("dcache_req_valid"),
                    dcache_req_uncacheable=opt("dcache_req_uncacheable"),
                    dcache_state=opt("dcache_state"),
                    dcache_storeq_count=opt("dcache_storeq_count"),
                    icache_accept=opt("icache_accept"),
                    icache_lookup_hit=opt("icache_lookup_hit"),
                    icache_req_valid=opt("icache_req_valid"),
                    icache_req_uncacheable=opt("icache_req_uncacheable"),
                    icache_state=opt("icache_state"),
                )
            )
            if val(boundary):
                if boundary == "i_pop":
                    length = val("ibus_len") or val("ilen")
                    opcode = val("ibus_opcode") or val("opcode")
                    eip_next = (val("eip") + length) & 0xFFFF_FFFF if length else val("eip")
                    pc = val("eip")
                else:
                    length = val("ilen")
                    opcode = val("opcode")
                    eip_next = val("eip")
                    pc = (eip_next - length) & 0xFFFF_FFFF if length else eip_next
                instrs.append(
                    InstructionEvent(
                        time=current_time,
                        pc=pc,
                        eip_next=eip_next,
                        length=length,
                        opcode=opcode,
                    )
                )

        pending = {}

    def callback(_data, time, facidx, value) -> None:
        nonlocal current_time
        t = int(time)
        if current_time is None:
            current_time = t
        elif t != current_time:
            flush_pending()
            current_time = t

        key = inverse.get(int(facidx))
        if key is None:
            return
        pending[key] = parse_fst_int(pylibfst.helpers.string(value))

    pylibfst.helpers.fstReaderIterBlocks(fst, callback, None)
    flush_pending()
    pylibfst.lib.fstReaderClose(fst)
    return samples, instrs, flush_times, markers, names, missing


def choose_window(
    samples: list[CycleSample],
    markers: list[tuple[int, int]],
    mode: str,
) -> tuple[int, int, str]:
    if not samples:
        raise SystemExit("trace contains no clock samples")

    if mode != "all":
        start = None
        end = None
        for time, marker in markers:
            tag = marker & MARK_START_MASK
            if tag == MARK_START:
                start = time
            elif tag == MARK_END and start is not None:
                end = time
                break
        if start is not None and end is not None and end > start:
            return start, end, "Dhrystone marker window"
        if mode == "markers":
            raise SystemExit("requested marker window, but MARK_START/MARK_END were not found")

    return samples[0].time, samples[-1].time, "whole trace"


def in_window(time: int, start: int, end: int) -> bool:
    return start < time < end


def summarize_cache(label: str, samples: list[CycleSample]) -> list[str]:
    accept_key = f"{label}_accept"
    lines: list[str] = []
    accepted = 0
    lookups = 0
    hits = 0
    misses = 0
    uncacheable = 0
    refill_cycles = 0
    refill_count = 0
    prev_state = None

    for sample in samples:
        accept = getattr(sample, accept_key)
        state = getattr(sample, f"{label}_state")
        req_valid = getattr(sample, f"{label}_req_valid")
        req_uncacheable = getattr(sample, f"{label}_req_uncacheable")
        lookup_hit = getattr(sample, f"{label}_lookup_hit")
        if accept is None and state is None:
            return [f"{label}: unavailable"]

        if accept:
            accepted += 1
        if state == 2 and req_valid:
            lookups += 1
            if req_uncacheable:
                uncacheable += 1
            elif lookup_hit:
                hits += 1
            else:
                misses += 1
        if state == 3:
            refill_cycles += 1
            if prev_state != 3:
                refill_count += 1
        prev_state = state

    hit_rate = fmt_pct(hits, hits + misses)
    lines.append(
        f"{label}: accepted={accepted} lookups={lookups} hits={hits} "
        f"misses={misses} hit_rate={hit_rate} uncacheable={uncacheable} "
        f"refills={refill_count} refill_cycles={refill_cycles}"
    )
    return lines


def print_table(headers: list[str], rows: list[list[str]]) -> None:
    widths = [len(header) for header in headers]
    for row in rows:
        for idx, item in enumerate(row):
            widths[idx] = max(widths[idx], len(item))
    print(" | ".join(header.ljust(widths[idx]) for idx, header in enumerate(headers)))
    print("-+-".join("-" * width for width in widths))
    for row in rows:
        print(" | ".join(item.ljust(widths[idx]) for idx, item in enumerate(row)))


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("fst", nargs="*", type=Path, default=[DEFAULT_FST], help="Dhrystone FST trace(s)")
    parser.add_argument("--lst", type=Path, default=DEFAULT_LST, help="Dhrystone disassembly listing")
    parser.add_argument("--code-base", type=parse_int, default=DEFAULT_CODE_BASE, help="Runtime code base added to .lst offsets")
    parser.add_argument("--window", choices=("auto", "markers", "all"), default="auto", help="Measurement window")
    parser.add_argument("--boundary", choices=("i_pop", "i_first"), default="i_pop", help="Clock-sampled instruction boundary")
    parser.add_argument("--top", type=int, default=20, help="Rows in top-N reports")
    args = parser.parse_args()

    instrs, funcs = load_listing(args.lst)
    if not instrs:
        print(f"warning: no instructions loaded from {args.lst}", file=sys.stderr)

    for trace_idx, fst_path in enumerate(args.fst):
        if trace_idx:
            print()
        samples, events, flush_times, markers, names, missing = read_trace(fst_path, args.boundary)
        period = infer_period(samples)
        start, end, window_name = choose_window(samples, markers, args.window)
        win_samples = [sample for sample in samples if in_window(sample.time, start, end)]
        win_events = [event for event in events if in_window(event.time, start, end)]
        win_flushes = [time for time in flush_times if in_window(time, start, end)]

        cycles = len(win_samples)
        instructions = len(win_events)
        cpi = cycles / instructions if instructions else 0.0

        print(f"Trace: {fst_path}")
        print(f"Window: {window_name}  {start}..{end}  ({cycles} cycles)")
        print(f"Instruction boundary: {args.boundary}")
        print(f"Instructions: {instructions}")
        print(f"CPI: {cpi:.3f}")
        if missing:
            optional_missing = ", ".join(sorted(missing - {"clk", "eip", "i_first"}))
            if optional_missing:
                print(f"Missing optional signals: {optional_missing}")

        pf_empty = sum(1 for sample in win_samples if sample.pf_empty)
        decq_empty = sum(1 for sample in win_samples if sample.decq_empty)
        both_empty = sum(1 for sample in win_samples if sample.pf_empty and sample.decq_empty)
        stall_cycles = sum(1 for sample in win_samples if sample.stall)
        stall_mem_cycles = sum(1 for sample in win_samples if sample.stall_mem)
        mem_servicing_cycles = sum(1 for sample in win_samples if sample.mem_servicing)

        print()
        print("Queue / Stall Summary")
        print(f"  prefetch queue empty cycles: {pf_empty:7d} {fmt_pct(pf_empty, cycles)}")
        print(f"  decode queue empty cycles:   {decq_empty:7d} {fmt_pct(decq_empty, cycles)}")
        print(f"  both queues empty cycles:    {both_empty:7d} {fmt_pct(both_empty, cycles)}")
        print(f"  stall cycles:                {stall_cycles:7d} {fmt_pct(stall_cycles, cycles)}")
        print(f"  memory stall cycles:         {stall_mem_cycles:7d} {fmt_pct(stall_mem_cycles, cycles)}")
        print(f"  memory servicing cycles:     {mem_servicing_cycles:7d} {fmt_pct(mem_servicing_cycles, cycles)}")

        pf_hist = Counter(sample.pf_count for sample in win_samples)
        decq_hist = Counter(sample.decq_count for sample in win_samples if sample.decq_count is not None)
        storeq_hist = Counter(sample.dcache_storeq_count for sample in win_samples if sample.dcache_storeq_count is not None)

        print()
        print("Queue Depth Histograms")
        pf_rows = [[str(k), str(v), fmt_pct(v, cycles)] for k, v in sorted(pf_hist.items())]
        print_table(["pf_count", "cycles", "pct"], pf_rows)
        if decq_hist:
            print()
            decq_rows = [[str(k), str(v), fmt_pct(v, cycles)] for k, v in sorted(decq_hist.items())]
            print_table(["decq_count", "cycles", "pct"], decq_rows)
        if storeq_hist:
            print()
            storeq_rows = [[str(k), str(v), fmt_pct(v, cycles)] for k, v in sorted(storeq_hist.items())]
            print_table(["storeq_count", "cycles", "pct"], storeq_rows)

        print()
        print("Cache Summary")
        for line in summarize_cache("icache", win_samples):
            print(f"  {line}")
        for line in summarize_cache("dcache", win_samples):
            print(f"  {line}")

        dly_wait = Counter()
        for sample in win_samples:
            if sample.uc_is_dly and sample.stall:
                dly_wait[sample.uc_addr] += 1
        print()
        print("Top DLY Wait Cycles By Microcode Address")
        dly_rows = [
            [f"{addr:03x}", str(count), fmt_pct(count, cycles)]
            for addr, count in dly_wait.most_common(args.top)
        ]
        print_table(["uc_addr", "cycles", "pct"], dly_rows or [["-", "0", "n/a"]])

        cycle_hist = Counter()
        eip_cycles: dict[int, float] = defaultdict(float)
        eip_counts: Counter[int] = Counter()
        eip_max: dict[int, float] = defaultdict(float)
        func_cycles: dict[str, float] = defaultdict(float)
        func_counts: Counter[str] = Counter()
        opcode_counts: Counter[int] = Counter()

        for cur, nxt in zip(win_events, win_events[1:]):
            cyc = (nxt.time - cur.time) / period
            bucket = int(round(cyc))
            cycle_hist[bucket] += 1
            offset = resolve_offset(cur.pc, instrs, args.code_base)
            info = instrs.get(offset)
            func = info.function if info else resolve_function(offset, funcs)
            eip_cycles[offset] += cyc
            eip_counts[offset] += 1
            eip_max[offset] = max(eip_max[offset], cyc)
            func_cycles[func] += cyc
            func_counts[func] += 1
            opcode_counts[cur.opcode] += 1

        print()
        print("Instruction Cycle Histogram")
        hist_rows = [[str(k), str(v), fmt_pct(v, max(1, len(win_events) - 1))] for k, v in sorted(cycle_hist.items())]
        print_table(["cycles", "instructions", "pct"], hist_rows)

        print()
        print("Top Functions By Cycles")
        func_rows = []
        for func, total in sorted(func_cycles.items(), key=lambda item: item[1], reverse=True)[: args.top]:
            count = func_counts[func]
            func_rows.append([func, f"{total:.0f}", str(count), f"{total / count:.2f}", fmt_pct(total, cycles)])
        print_table(["function", "cycles", "instr", "avg", "pct"], func_rows)

        print()
        print("Top EIPs By Cycles")
        eip_rows = []
        for offset, total in sorted(eip_cycles.items(), key=lambda item: item[1], reverse=True)[: args.top]:
            info = instrs.get(offset)
            count = eip_counts[offset]
            asm = info.asm if info else "?"
            func = info.function if info else resolve_function(offset, funcs)
            eip_rows.append([
                fmt_addr(offset),
                func,
                f"{total:.0f}",
                str(count),
                f"{total / count:.2f}",
                f"{eip_max[offset]:.1f}",
                asm[:64],
            ])
        print_table(["eip", "function", "cycles", "count", "avg", "max", "instruction"], eip_rows)

        print()
        print("Top Opcodes By Count")
        opcode_rows = [[f"{op:02x}", str(count), fmt_pct(count, instructions)] for op, count in opcode_counts.most_common(args.top)]
        print_table(["opcode", "count", "pct"], opcode_rows)

        print()
        print("Branch / Frontend Flush Latency")
        flush_latencies: list[float] = []
        event_times = [event.time for event in win_events]
        for flush_time in win_flushes:
            idx = bisect.bisect_right(event_times, flush_time)
            if idx < len(event_times):
                flush_latencies.append((event_times[idx] - flush_time) / period)
        if flush_latencies:
            print(
                f"  flushes={len(flush_latencies)} avg={mean(flush_latencies):.2f} "
                f"min={min(flush_latencies):.1f} max={max(flush_latencies):.1f}"
            )
            lat_hist = Counter(int(round(value)) for value in flush_latencies)
            lat_rows = [[str(k), str(v), fmt_pct(v, len(flush_latencies))] for k, v in sorted(lat_hist.items())]
            print_table(["cycles_to_next_i_first", "count", "pct"], lat_rows)
        else:
            print("  unavailable or no frontend flushes in window")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
