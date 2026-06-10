#!/usr/bin/env python3
"""
Protected Mode Test Runner for z386

Runs assembly tests in protected mode with configurable segment descriptors
and paging. Tests report results via I/O ports.

Usage:
    ./test_protected_mode.py                    # Run all tests
    ./test_protected_mode.py seg_paging         # Run specific test
    ./test_protected_mode.py seg_paging -v      # Verbose with traces
    ./test_protected_mode.py seg_paging --trace # Generate VCD trace
"""

import sys
import subprocess
import os
import argparse
import struct
from pathlib import Path

# Paths
SCRIPT_DIR = Path(__file__).resolve().parent
TESTS_DIR = SCRIPT_DIR / "protected_mode_tests"
VERILATOR_EXE = SCRIPT_DIR / "obj_dir/Vtb_protected_mode"

# I/O port results
STATUS_PASS = 0x01
STATUS_FAIL = 0xFF

#==============================================================================
# Page Table Generation
#==============================================================================

def generate_page_table_entry(frame_addr, flags='RW'):
    """Generate a 32-bit page table entry.

    Args:
        frame_addr: Physical frame address (must be 4KB aligned)
        flags: 'RW' for read/write, 'RO' for read-only, 'U' adds user bit

    Returns:
        32-bit PTE value
    """
    pte = (frame_addr & 0xFFFFF000)  # Frame address [31:12]
    pte |= 0x01  # Present
    if 'W' in flags.upper():
        pte |= 0x02  # Read/Write
    if 'U' in flags.upper():
        pte |= 0x04  # User
    pte |= 0x20  # Accessed
    return pte


def generate_page_tables(mappings, page_dir_addr=0x0000):
    """Generate page directory and page tables for given mappings.

    Args:
        mappings: List of (linear_start, physical_start, num_pages, flags)
        page_dir_addr: Physical address of page directory

    Returns:
        Dict mapping physical addresses to byte data
    """
    memory = {}

    # Page directory (4KB, 1024 entries)
    page_dir = [0] * 1024

    # Track page tables we need to create
    # Key: PDE index, Value: physical address of page table
    page_tables = {}
    next_pt_addr = page_dir_addr + 0x1000  # First PT after page dir

    for linear_start, physical_start, num_pages, flags in mappings:
        for i in range(num_pages):
            linear_addr = linear_start + i * 0x1000
            physical_addr = physical_start + i * 0x1000

            # PDE index = linear[31:22]
            pde_idx = (linear_addr >> 22) & 0x3FF
            # PTE index = linear[21:12]
            pte_idx = (linear_addr >> 12) & 0x3FF

            # Create page table if needed
            if pde_idx not in page_tables:
                page_tables[pde_idx] = next_pt_addr
                next_pt_addr += 0x1000

                # Initialize page table to all zeros
                pt_base = page_tables[pde_idx]
                for j in range(1024):
                    memory[pt_base + j*4] = struct.pack('<I', 0)

                # Create PDE pointing to this page table
                pde = generate_page_table_entry(page_tables[pde_idx], 'RW')
                page_dir[pde_idx] = pde

            # Create PTE
            pt_base = page_tables[pde_idx]
            pte = generate_page_table_entry(physical_addr, flags)
            memory[pt_base + pte_idx*4] = struct.pack('<I', pte)

    # Write page directory
    for i, pde in enumerate(page_dir):
        memory[page_dir_addr + i*4] = struct.pack('<I', pde)

    return memory


#==============================================================================
# Test Configuration
#==============================================================================

# Segment descriptor flags:
# [15:12] = type, [11] = S, [10:9] = DPL, [8] = P, [7] = D_B, [6] = G, [5] = A
# Data RW:  type=0010, S=1, DPL=00, P=1, D_B=1, G=1, A=1 -> 0x21E0
# Code RX:  type=1010, S=1, DPL=00, P=1, D_B=1, G=1, A=1 -> 0xA1E0

def seg_flags(code=False, dpl=0, db32=True, granularity_4k=True):
    """Build segment descriptor flags."""
    flags = 0
    if code:
        flags |= 0xA000  # Code, readable (type=1010)
    else:
        flags |= 0x2000  # Data, writable (type=0010)
    flags |= 0x0800      # S=1 (code/data, not system)
    flags |= (dpl & 3) << 9  # DPL
    flags |= 0x0100      # P=1 (present)
    if db32:
        flags |= 0x0080  # D/B=1 (32-bit)
    if granularity_4k:
        flags |= 0x0040  # G=1 (4KB granularity)
    flags |= 0x0020      # A=1 (accessed)
    return flags


TESTS = {
    'rm_pe_transition': {
        'asm': 'rm_pe_transition.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,  # Start with PE=0
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
    },
    'pe_toggle_redecode': {
        'asm': 'pe_toggle_redecode.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
    },
    'pe_llldt_cpl_cache': {
        'asm': 'pe_llldt_cpl_cache.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
    },
    'call_gate': {
        'asm': 'call_gate.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
    },
    'call_gate_cross': {
        'asm': 'call_gate_cross.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
    },
    'real_mode_smoke': {
        'asm': 'simple_pass.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,  # PE=0, PG=0
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            # CS defaults to code_phys_base>>4 if omitted; pin DS/SS/ES for determinism.
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
    },
    'pm_init': {
        'asm': 'pm_init.asm',
        'start_mode': 'protected',
        'cr0': 0x80000001,  # PE=1, PG=1
        'cr3': 0x00000000,  # Page directory at physical 0
        'eip': 0x00000000,  # Start at offset 0 in CS
        'seg_cache': {
            # CS.base uses physical code location because prefetch bypasses paging
            'CS': {'base': 0x00010000, 'limit': 0xFFFFF, 'flags': seg_flags(code=True)},
            'DS': {'base': 0x20000000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
            'SS': {'base': 0x30000000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
            'ES': {'base': 0x00000000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
            'FS': {'base': 0x00000000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
            'GS': {'base': 0x00000000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
        },
        'page_tables': [
            # (linear_start, physical_start, num_pages, flags)
            (0x00010000, 0x00010000, 16, 'RW'),  # CS: identity mapped test code
            (0x20000000, 0x00020000, 1, 'RW'),   # DS page for data checks
            (0x30000000, 0x00060000, 1, 'RW'),   # SS page for stack checks
        ],
    },
    'seg_paging': {
        'asm': 'seg_paging.asm',
        'start_mode': 'protected',
        'cr0': 0x80000001,  # PE=1, PG=1
        'cr3': 0x00000000,  # Page directory at physical 0
        'eip': 0x00000000,  # Start at offset 0 in CS
        'seg_cache': {
            # CS.base = code physical address (identity mapping) because
            # prefetch doesn't go through paging yet
            'CS': {'base': 0x00010000, 'limit': 0xFFFFF, 'flags': seg_flags(code=True)},
            'DS': {'base': 0x20000000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
            'SS': {'base': 0x30000000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
            'ES': {'base': 0x00000000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
            'FS': {'base': 0x00000000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
            'GS': {'base': 0x00000000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
        },
        'page_tables': [
            # (linear_start, physical_start, num_pages, flags)
            (0x00010000, 0x00010000, 16, 'RW'),   # CS: 64KB (identity mapped)
            (0x20000000, 0x00020000, 1, 'RW'),    # DS page 0: phys 0x20000
            (0x20001000, 0x00040000, 1, 'RW'),    # DS page 1: phys 0x40000 (non-contiguous!)
            (0x20002000, 0x00022000, 62, 'RW'),   # DS pages 2-63
            (0x30000000, 0x00060000, 16, 'RW'),   # SS: 64KB
        ],
    },
    'hw_interrupt': {
        'asm': 'hw_interrupt.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
    },
    'sti_shadow': {
        'asm': 'sti_shadow.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
    },
    'movss_shadow': {
        'asm': 'movss_shadow.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
    },
    'pm_hw_interrupt': {
        'asm': 'pm_hw_interrupt.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
    },
    'pm_hw_intr_use16': {
        'asm': 'pm_hw_intr_use16.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
    },
    'v86_int_stack_reuse': {
        'asm': 'v86_int_stack_reuse.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
    },
    'v86_monitor_bounce_stack': {
        'asm': 'v86_monitor_bounce_stack.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
    },
    'v86_gp_int_frame': {
        'asm': 'v86_gp_int_frame.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
    },
    'fpu_detect': {
        'asm': 'fpu_detect.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
    },
    'nmi_with_cli': {
        'asm': 'nmi_with_cli.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
    },
    'nmi_during_sti_shadow': {
        'asm': 'nmi_during_sti_shadow.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
    },
    'pm_intr_gate_if_clear': {
        'asm': 'pm_intr_gate_if_clear.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
    },
    'pm_trap_gate_if_preserve': {
        'asm': 'pm_trap_gate_if_preserve.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
    },
    'hlt_wakeup_intr_rm': {
        'asm': 'hlt_wakeup_intr_rm.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
        'continue_on_hlt': True,
    },
    'hlt_wakeup_intr_pm': {
        'asm': 'hlt_wakeup_intr_pm.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
        'continue_on_hlt': True,
    },
    'intr_pop_esp': {
        'asm': 'intr_pop_esp.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
    },
    'stale_modrm': {
        'asm': 'stale_modrm.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
        'mem_latency': 5,  # Slow memory to trigger stale modrm bug
    },
    'rep_stos_decode': {
        'asm': 'rep_stos_decode.asm',
        'start_mode': 'protected',
        'cr0': 0x00000001,  # PE=1, PG=0
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'seg_cache': {
            'CS': {'base': 0x00010000, 'limit': 0xFFFFF, 'flags': seg_flags(code=True)},
            'DS': {'base': 0x00010000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
            'SS': {'base': 0x00010000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
            'ES': {'base': 0x00010000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
            'FS': {'base': 0x00010000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
            'GS': {'base': 0x00010000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
        },
        'page_tables': [],
    },
    'rep_scas_intr_high_eip': {
        'asm': 'rep_scas_intr_high_eip.asm',
        'start_mode': 'real',
        'cr0': 0x00000000,
        'cr3': 0x00000000,
        'eip': 0x00000000,
        'initial_selectors': {
            'DS': 0x0000,
            'SS': 0x0000,
            'ES': 0x0000,
            'FS': 0x0000,
            'GS': 0x0000,
        },
        'd_init': 0,
        'page_tables': [],
        'cycles': 30_000,
    },
    'prefetch_flush_loops': {
        'asm': 'prefetch_flush_loops.asm',
        'start_mode': 'protected',
        'cr0': 0x80000001,  # PE=1, PG=1
        'cr3': 0x00000000,  # Page directory at physical 0
        'eip': 0x00000000,  # Start at offset 0 in CS
        'seg_cache': {
            # CS.base = code physical address because prefetch bypasses paging
            'CS': {'base': 0x00010000, 'limit': 0xFFFFF, 'flags': seg_flags(code=True)},
            'DS': {'base': 0x20000000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
            'SS': {'base': 0x30000000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
            'ES': {'base': 0x00000000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
            'FS': {'base': 0x00000000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
            'GS': {'base': 0x00000000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
        },
        'page_tables': [
            (0x00010000, 0x00010000, 16, 'RW'),  # CS: identity mapped test code
            (0x20000000, 0x00020000, 1, 'RW'),   # DS page
            (0x30000000, 0x00060000, 1, 'RW'),   # SS page
        ],
    },
    'instruction_timing': {
        'asm': 'instruction_timing.asm',
        'start_mode': 'protected',
        'cr0': 0x80000001,  # PE=1, PG=1
        'cr3': 0x00000000,  # Page directory at physical 0
        'eip': 0x00000000,  # Start at offset 0 in CS
        'seg_cache': {
            # CS.base = code physical address because prefetch bypasses paging
            'CS': {'base': 0x00010000, 'limit': 0xFFFFF, 'flags': seg_flags(code=True)},
            'DS': {'base': 0x20000000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
            'SS': {'base': 0x30000000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
            'ES': {'base': 0x00000000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
            'FS': {'base': 0x00000000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
            'GS': {'base': 0x00000000, 'limit': 0xFFFFF, 'flags': seg_flags(code=False)},
        },
        'page_tables': [
            (0x00010000, 0x00010000, 16, 'RW'),  # CS: identity mapped test code
            (0x20000000, 0x00020000, 1, 'RW'),   # DS page
            (0x30000000, 0x00060000, 1, 'RW'),   # SS page
        ],
    },
}


#==============================================================================
# Build and Run
#==============================================================================

def build_testbench(verbose=False):
    """Build or refresh the protected-mode Verilator testbench."""
    cmd = ['make', 'obj_dir/Vtb_protected_mode']
    print(f"Building protected-mode testbench: {' '.join(cmd)}")

    result = subprocess.run(
        cmd,
        cwd=SCRIPT_DIR,
        capture_output=not verbose,
        text=True
    )
    if result.returncode != 0:
        print("Error: failed to build protected-mode testbench")
        if not verbose:
            if result.stdout:
                print(result.stdout)
            if result.stderr:
                print(result.stderr)
        return False
    if not VERILATOR_EXE.exists():
        print(f"Error: expected testbench was not produced: {VERILATOR_EXE}")
        return False
    return True


def assemble(asm_file, bin_file, lst_file, verbose=False):
    """Assemble .asm to .bin using NASM."""
    cmd = ['nasm', '-f', 'bin', '-o', str(bin_file), '-l', str(lst_file), str(asm_file)]
    if verbose:
        print(f"  Assembling: {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"NASM error:\n{result.stderr}")
        return False
    return True


def build_memory_image(test_config, code_bin, output_hex, verbose=False):
    """Build combined memory image with code and page tables.

    Memory layout:
        0x00000000 - Page Directory (4KB)
        0x00001000 - Page Tables (as needed)
        0x00010000 - Code area (CS maps here)
        0x00020000 - Data area (DS maps here)
        0x00060000 - Stack area (SS maps here)
    """
    memory = {}

    # Generate page tables
    pt_mappings = test_config.get('page_tables', [])
    if pt_mappings:
        pt_mem = generate_page_tables(pt_mappings, page_dir_addr=0x0000)
        for addr, data in pt_mem.items():
            for i, byte in enumerate(data):
                memory[addr + i] = byte

    # Load code binary
    code_data = code_bin.read_bytes()
    # Code goes at physical address where CS linear range maps
    # From page_tables config: CS linear 0x10000000 -> physical 0x00010000
    code_phys_addr = 0x00010000
    for pt in pt_mappings:
        if pt[0] == test_config['seg_cache']['CS']['base']:
            code_phys_addr = pt[1]
            break

    for i, byte in enumerate(code_data):
        memory[code_phys_addr + i] = byte

    if verbose:
        print(f"  Code loaded at physical 0x{code_phys_addr:08X} ({len(code_data)} bytes)")
        print(f"  Page tables at physical 0x00000000")

    # Write hex file
    with open(output_hex, 'w') as f:
        # Find max address
        if memory:
            max_addr = max(memory.keys())
            for addr in range(0, max_addr + 1):
                byte = memory.get(addr, 0)
                f.write(f"{byte:02X}\n")

    return code_phys_addr  # Return physical address where code was loaded


def run_simulation(test_name, test_config, hex_file, code_phys_base, verbose=False, trace=False, cycles=10_000, trace_file=None):
    """Run Verilator simulation with test configuration."""
    cmd = [str(VERILATOR_EXE)]
    start_mode = test_config.get('start_mode', 'protected').lower()
    start_protected = 0 if start_mode == 'real' else 1

    init_selectors = test_config.get('initial_selectors', {})
    default_cs = (code_phys_base >> 4) & 0xFFFF if start_protected == 0 else 0x0008
    init_cs = init_selectors.get('CS', default_cs)
    init_ds = init_selectors.get('DS', 0x0010 if start_protected else 0x0000)
    init_ss = init_selectors.get('SS', 0x0018 if start_protected else 0x0000)
    init_es = init_selectors.get('ES', 0x0020 if start_protected else 0x0000)
    init_fs = init_selectors.get('FS', 0x0028 if start_protected else 0x0000)
    init_gs = init_selectors.get('GS', 0x0030 if start_protected else 0x0000)
    d_init = test_config.get('d_init', 1 if start_protected else 0)

    cmd.append(f"+mem={hex_file}")
    cmd.append(f"+cycles={cycles}")
    cmd.append(f"+eip={test_config['eip']}")
    cmd.append(f"+cr0={test_config['cr0']}")
    cmd.append(f"+cr3={test_config['cr3']}")
    cmd.append(f"+code_phys_base={code_phys_base}")
    cmd.append(f"+start_protected={start_protected}")
    cmd.append(f"+init_cs={init_cs}")
    cmd.append(f"+init_ds={init_ds}")
    cmd.append(f"+init_ss={init_ss}")
    cmd.append(f"+init_es={init_es}")
    cmd.append(f"+init_fs={init_fs}")
    cmd.append(f"+init_gs={init_gs}")
    cmd.append(f"+d_init={d_init}")
    if test_config.get('continue_on_hlt', False):
        cmd.append("+continue_on_hlt")
    if 'mem_latency' in test_config:
        cmd.append(f"+mem_latency={test_config['mem_latency']}")

    # Segment descriptor cache parameters
    seg_cache = test_config.get('seg_cache', {})
    if seg_cache:
        for seg_name in ['CS', 'DS', 'SS', 'ES', 'FS', 'GS']:
            seg = seg_cache[seg_name]
            prefix = seg_name.lower()
            cmd.append(f"+{prefix}_base={seg['base']}")
            cmd.append(f"+{prefix}_limit={seg['limit']}")
            cmd.append(f"+{prefix}_flags={seg['flags']}")

    if verbose:
        cmd.extend(["+trace_instr", "+trace_io", "+trace_descw", "+trace_prot", "+trace_ldsg", "+trace_gate", "+trace_ucode", "+trace_mem"])
    if trace:
        cmd.append("+trace")
        if trace_file is not None:
            cmd.append(f"+tracefile={trace_file}")

    if verbose:
        print(f"  Running: {' '.join(cmd[:5])}...")

    result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)

    if verbose:
        print(result.stdout)
        if result.stderr:
            print(f"  stderr: {result.stderr}")

    # Check result
    passed = "TEST PASSED" in result.stdout
    failed = "TEST FAILED" in result.stdout
    timeout = "TIMEOUT" in result.stdout

    return passed, failed, timeout, result.stdout


def run_test(test_name, verbose=False, trace=False, keep_files=False, cycles=20_000):
    """Run a single protected mode test."""
    if test_name not in TESTS:
        return False, f"Unknown test: {test_name}"

    test_config = TESTS[test_name]
    asm_file = TESTS_DIR / test_config['asm']
    bin_file = TESTS_DIR / f"{test_name}.bin"
    lst_file = TESTS_DIR / f"{test_name}.lst"
    hex_file = TESTS_DIR / f"{test_name}.hex"

    if not asm_file.exists():
        return False, f"Assembly file not found: {asm_file}"

    try:
        # Assemble
        if verbose:
            print(f"  Building {test_name}...")
        if not assemble(asm_file, bin_file, lst_file, verbose):
            return False, "Assembly failed"

        # Build memory image
        if verbose:
            print(f"  Generating page tables...")
        code_phys_base = build_memory_image(test_config, bin_file, hex_file, verbose)
        if code_phys_base is None:
            return False, "Memory image generation failed"

        # Run simulation
        if verbose:
            print(f"  Running simulation...")
        trace_file = None
        if trace:
            trace_dir = TESTS_DIR / "traces"
            trace_dir.mkdir(parents=True, exist_ok=True)
            trace_file = trace_dir / f"{test_name}.vcd"
            if trace_file.exists():
                trace_file.unlink()
            if verbose:
                print(f"  Trace file: {trace_file}")
        sim_cycles = test_config.get('cycles', cycles)
        passed, failed, timeout, output = run_simulation(
            test_name, test_config, hex_file, code_phys_base, verbose, trace, sim_cycles, trace_file
        )

        if passed:
            return True, "PASS"
        elif failed:
            return False, "FAIL - test reported failure"
        elif timeout:
            return False, "TIMEOUT"
        else:
            return False, "Unknown result"

    except subprocess.TimeoutExpired:
        return False, "Simulation timeout (120s)"
    except Exception as e:
        if verbose:
            import traceback
            traceback.print_exc()
        return False, f"Error: {e}"
    finally:
        # Cleanup
        if not keep_files:
            if bin_file.exists():
                bin_file.unlink()
            if hex_file.exists():
                hex_file.unlink()


#==============================================================================
# Main
#==============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Run z386 protected mode tests',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
    ./test_protected_mode.py                    # Run all tests
    ./test_protected_mode.py seg_paging         # Run specific test
    ./test_protected_mode.py seg_paging -v      # Verbose
    ./test_protected_mode.py seg_paging --trace # Generate VCD
        '''
    )
    parser.add_argument('tests', nargs='*', help='Tests to run (default: all)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Verbose output')
    parser.add_argument('--trace', action='store_true', help='Generate VCD trace')
    parser.add_argument('--keep', action='store_true', help='Keep intermediate files')
    parser.add_argument('-c', '--cycles', type=int, default=20_000, help='Max cycles')
    parser.add_argument('--list', action='store_true', help='List available tests')

    args = parser.parse_args()

    if args.list:
        print("Available tests:")
        for name in TESTS:
            print(f"  {name}")
        return 0

    if not build_testbench(args.verbose):
        return 1

    # Check NASM exists
    result = subprocess.run(['which', 'nasm'], capture_output=True)
    if result.returncode != 0:
        print("Error: NASM assembler not found")
        print("Install with: brew install nasm")
        return 1

    # Determine tests to run
    tests_to_run = args.tests if args.tests else list(TESTS.keys())

    # Run tests
    passed_count = 0
    failed_count = 0

    print(f"Running {len(tests_to_run)} protected mode test(s)...\n")

    for test_name in tests_to_run:
        if args.verbose:
            print(f"Testing {test_name}:")

        passed, message = run_test(
            test_name,
            verbose=args.verbose,
            trace=args.trace,
            keep_files=args.keep,
            cycles=args.cycles
        )

        if passed:
            passed_count += 1
            color = "\033[92m"  # Green
            status = "PASS"
        else:
            failed_count += 1
            color = "\033[91m"  # Red
            status = "FAIL"

        reset = "\033[0m"
        print(f"{color}{status}{reset} {test_name:20s} - {message}")

    # Summary
    print(f"\n{'='*60}")
    total = passed_count + failed_count
    percentage = (passed_count * 100 / total) if total > 0 else 0
    print(f"Summary: {passed_count}/{total} tests passed ({percentage:.1f}%)")

    return 0 if failed_count == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
