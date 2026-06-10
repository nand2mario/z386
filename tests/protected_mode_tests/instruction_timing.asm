; instruction_timing.asm - Protected-mode instruction timing microbenchmark
;
; This benchmark is arranged as fixed-offset phases so the trace can be parsed
; by address, similar to prefetch_flush_loops.asm + measure_prefetch_flush.py.
;
; Measurement guidance:
;   - For load/store/push/mov/imm/alu/pop, measure steady-state
;     `i_first -> next i_first` between the repeated instruction labels.
;   - For conditional jump not taken, measure steady-state
;     `jnz_nt_N -> jnz_nt_{N+1}`.
;   - For conditional jump taken / unconditional jump, measure the branch
;     `i_first` to the target loop-head `i_first`.
;
; 80386 targets from doc/performance_optimization_of_the_80386.md:
;   Load                  4
;   Store                 2
;   Push reg to mem       2
;   Move reg to reg       2
;   Load immediate        2
;   LEA rv,m              2
;   Shift/rotate r,CL     3
;   XCHG r,r              3
;   XCHG m,r              5
;   Push immediate        2
;   Push segment register 2
;   Push memory value     5
;   Pop memory value      5
;   Conditional jump tk   9.25
;   Conditional jump nt   3
;   ALU reg to reg        2
;   Pop mem to reg        4
;   Unconditional jump    9.25
;
; Result protocol:
;   Port 0xE0: status (0x01 = pass, 0xFF = fail)
;   Port 0xE4: failure code

BITS 32
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4

STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

ITERATIONS  equ 8

LOAD_ADDR   equ 0x200
STORE_ADDR  equ 0x240
XCHG_ADDR   equ 0x280
STACK_TOP   equ 0x800
POP_STACK   equ 0x880

LOAD_VALUE  equ 0x11223344
STORE_VALUE equ 0x55667788
PUSH_VALUE  equ 0x89ABCDEF
MOV_VALUE   equ 0x13579BDF
IMM_VALUE   equ 0x2468ACE0
POP_VALUE   equ 0xCAFEBABE
XCHG_MEM_INIT equ 0x11112222
XCHG_REG_INIT equ 0x33334444

start:
    xor eax, eax
    mov dx, STATUS_PORT
    out dx, al
    jmp phase_load

    times (0x100 - ($ - $$)) db 0x90

;------------------------------------------------------------------------------
; Phase 1: Load
;   Simple repeated `mov eax, [esi]` from the same aligned dword.
;------------------------------------------------------------------------------
phase_load:
    mov esi, LOAD_ADDR
    mov dword [esi], LOAD_VALUE

load_01: mov eax, [esi]
load_02: mov eax, [esi]
load_03: mov eax, [esi]
load_04: mov eax, [esi]
load_05: mov eax, [esi]
load_06: mov eax, [esi]
load_07: mov eax, [esi]
load_08: mov eax, [esi]

    cmp eax, LOAD_VALUE
    jne short phase_load_fail
    jmp phase_store

phase_load_fail:
    jmp fail_01

    times (0x200 - ($ - $$)) db 0x90

;------------------------------------------------------------------------------
; Phase 2: Store
;   Simple repeated `mov [edi], eax` to the same aligned dword.
;------------------------------------------------------------------------------
phase_store:
    mov edi, STORE_ADDR
    mov eax, STORE_VALUE

store_01: mov [edi], eax
store_02: mov [edi], eax
store_03: mov [edi], eax
store_04: mov [edi], eax
store_05: mov [edi], eax
store_06: mov [edi], eax
store_07: mov [edi], eax
store_08: mov [edi], eax

    cmp dword [edi], STORE_VALUE
    jne short phase_store_fail
    jmp phase_push

phase_store_fail:
    jmp fail_02

    times (0x300 - ($ - $$)) db 0x90

;------------------------------------------------------------------------------
; Phase 3: Push reg to mem
;   Repeated `push eax` with a known stack pointer.
;------------------------------------------------------------------------------
phase_push:
    mov esp, STACK_TOP
    mov eax, PUSH_VALUE

push_01: push eax
push_02: push eax
push_03: push eax
push_04: push eax
push_05: push eax
push_06: push eax
push_07: push eax
push_08: push eax

    cmp esp, STACK_TOP - (ITERATIONS * 4)
    jne short phase_push_fail
    cmp dword [esp], PUSH_VALUE
    jne short phase_push_fail
    cmp dword [esp + (ITERATIONS - 1) * 4], PUSH_VALUE
    jne short phase_push_fail
    jmp phase_mov_rr

phase_push_fail:
    jmp fail_03

    times (0x400 - ($ - $$)) db 0x90

;------------------------------------------------------------------------------
; Phase 4: Move reg to reg
;------------------------------------------------------------------------------
phase_mov_rr:
    xor eax, eax
    mov ebx, MOV_VALUE

mov_rr_01: mov eax, ebx
mov_rr_02: mov eax, ebx
mov_rr_03: mov eax, ebx
mov_rr_04: mov eax, ebx
mov_rr_05: mov eax, ebx
mov_rr_06: mov eax, ebx
mov_rr_07: mov eax, ebx
mov_rr_08: mov eax, ebx

    cmp eax, MOV_VALUE
    jne short phase_mov_rr_fail
    jmp phase_load_imm

phase_mov_rr_fail:
    jmp fail_04

    times (0x500 - ($ - $$)) db 0x90

;------------------------------------------------------------------------------
; Phase 5: Load immediate
;------------------------------------------------------------------------------
phase_load_imm:
    xor eax, eax

imm_01: mov eax, IMM_VALUE
imm_02: mov eax, IMM_VALUE
imm_03: mov eax, IMM_VALUE
imm_04: mov eax, IMM_VALUE
imm_05: mov eax, IMM_VALUE
imm_06: mov eax, IMM_VALUE
imm_07: mov eax, IMM_VALUE
imm_08: mov eax, IMM_VALUE

    cmp eax, IMM_VALUE
    jne short phase_load_imm_fail
    jmp phase_alu_rr

phase_load_imm_fail:
    jmp fail_05

    times (0x600 - ($ - $$)) db 0x90

;------------------------------------------------------------------------------
; Phase 6: ALU reg to reg
;------------------------------------------------------------------------------
phase_alu_rr:
    xor eax, eax
    mov ebx, 1

alu_01: add eax, ebx
alu_02: add eax, ebx
alu_03: add eax, ebx
alu_04: add eax, ebx
alu_05: add eax, ebx
alu_06: add eax, ebx
alu_07: add eax, ebx
alu_08: add eax, ebx

    cmp eax, ITERATIONS
    jne short phase_alu_rr_fail
    jmp phase_pop

phase_alu_rr_fail:
    jmp fail_06

    times (0x700 - ($ - $$)) db 0x90

;------------------------------------------------------------------------------
; Phase 7: Pop mem to reg
;   Preload eight identical stack slots, then pop them with repeated `pop eax`.
;------------------------------------------------------------------------------
phase_pop:
    mov esp, POP_STACK
    mov dword [esp + 0x00], POP_VALUE
    mov dword [esp + 0x04], POP_VALUE
    mov dword [esp + 0x08], POP_VALUE
    mov dword [esp + 0x0C], POP_VALUE
    mov dword [esp + 0x10], POP_VALUE
    mov dword [esp + 0x14], POP_VALUE
    mov dword [esp + 0x18], POP_VALUE
    mov dword [esp + 0x1C], POP_VALUE
    mov esp, POP_STACK

pop_01: pop eax
pop_02: pop eax
pop_03: pop eax
pop_04: pop eax
pop_05: pop eax
pop_06: pop eax
pop_07: pop eax
pop_08: pop eax

    cmp eax, POP_VALUE
    jne short phase_pop_fail
    cmp esp, POP_STACK + (ITERATIONS * 4)
    jne short phase_pop_fail
    jmp phase_jcc_not_taken

phase_pop_fail:
    jmp fail_07

    times (0x800 - ($ - $$)) db 0x90

;------------------------------------------------------------------------------
; Phase 8: Conditional jump not taken
;   Set ZF=1 once, then execute a run of fall-through `jnz` instructions.
;------------------------------------------------------------------------------
phase_jcc_not_taken:
    xor eax, eax               ; ZF=1, preserved across not-taken JNZ

jnz_nt_01: jnz short jcc_nt_fail
jnz_nt_02: jnz short jcc_nt_fail
jnz_nt_03: jnz short jcc_nt_fail
jnz_nt_04: jnz short jcc_nt_fail
jnz_nt_05: jnz short jcc_nt_fail
jnz_nt_06: jnz short jcc_nt_fail
jnz_nt_07: jnz short jcc_nt_fail
jnz_nt_08: jnz short jcc_nt_fail

    jmp phase_jcc_taken

jcc_nt_fail:
    jmp fail_08

    times (0x900 - ($ - $$)) db 0x90

;------------------------------------------------------------------------------
; Phase 9: Conditional jump taken
;   Steady-state taken back-edge loop.
;------------------------------------------------------------------------------
phase_jcc_taken:
    xor esi, esi
    mov ecx, ITERATIONS
    jmp jcc_taken_loop

jcc_taken_loop:
jcc_taken_body:   inc esi
jcc_taken_count:  dec ecx
jcc_taken_branch: jnz short jcc_taken_loop

    cmp esi, ITERATIONS
    jne short phase_jcc_taken_fail
    jmp phase_jmp

phase_jcc_taken_fail:
    jmp fail_09

    times (0xA00 - ($ - $$)) db 0x90

;------------------------------------------------------------------------------
; Phase 10: Unconditional jump
;   Steady-state unconditional back-edge loop.
;------------------------------------------------------------------------------
phase_jmp:
    xor edi, edi
    mov ecx, ITERATIONS
    jmp jmp_loop

jmp_loop:
jmp_body:   inc edi
jmp_count:  dec ecx
           jz short jmp_done
jmp_branch: jmp short jmp_loop

jmp_done:
    cmp edi, ITERATIONS
    jne short phase_jmp_fail
    jmp phase_lea

phase_jmp_fail:
    jmp fail_0A

    times (0xB00 - ($ - $$)) db 0x90

;------------------------------------------------------------------------------
; Phase 11: LEA
;   LEA uses the address-generation datapath without a memory bus cycle.
;------------------------------------------------------------------------------
phase_lea:
    mov esi, LOAD_ADDR
    mov edi, 4

lea_01: lea eax, [esi + edi*4 + 0x20]
lea_02: lea eax, [esi + edi*4 + 0x20]
lea_03: lea eax, [esi + edi*4 + 0x20]
lea_04: lea eax, [esi + edi*4 + 0x20]
lea_05: lea eax, [esi + edi*4 + 0x20]
lea_06: lea eax, [esi + edi*4 + 0x20]
lea_07: lea eax, [esi + edi*4 + 0x20]
lea_08: lea eax, [esi + edi*4 + 0x20]

    cmp eax, LOAD_ADDR + (4 * 4) + 0x20
    jne short phase_lea_fail
    jmp phase_shift_rotate

phase_lea_fail:
    jmp fail_0B

    times (0xC00 - ($ - $$)) db 0x90

;------------------------------------------------------------------------------
; Phase 12: Shift/rotate by CL
;   386 timings are independent of shift count due to the barrel shifter.
;------------------------------------------------------------------------------
phase_shift_rotate:
    mov eax, 0x12345678
    mov ecx, 8

rol_01: rol eax, cl
rol_02: rol eax, cl
rol_03: rol eax, cl
rol_04: rol eax, cl
rol_05: rol eax, cl
rol_06: rol eax, cl
rol_07: rol eax, cl
rol_08: rol eax, cl

    cmp eax, 0x12345678
    jne short phase_shift_fail
    jmp phase_xchg_rr

phase_shift_fail:
    jmp fail_0C

    times (0xD00 - ($ - $$)) db 0x90

;------------------------------------------------------------------------------
; Phase 13: XCHG r,r
;------------------------------------------------------------------------------
phase_xchg_rr:
    mov ecx, XCHG_MEM_INIT
    mov ebx, XCHG_REG_INIT

xchg_rr_01: xchg ecx, ebx
xchg_rr_02: xchg ecx, ebx
xchg_rr_03: xchg ecx, ebx
xchg_rr_04: xchg ecx, ebx
xchg_rr_05: xchg ecx, ebx
xchg_rr_06: xchg ecx, ebx
xchg_rr_07: xchg ecx, ebx
xchg_rr_08: xchg ecx, ebx

    cmp ecx, XCHG_MEM_INIT
    jne short phase_xchg_rr_fail
    cmp ebx, XCHG_REG_INIT
    jne short phase_xchg_rr_fail
    jmp phase_xchg_mr

phase_xchg_rr_fail:
    jmp fail_0D

    times (0xE00 - ($ - $$)) db 0x90

;------------------------------------------------------------------------------
; Phase 14: XCHG m,r
;------------------------------------------------------------------------------
phase_xchg_mr:
    mov edi, XCHG_ADDR
    mov dword [edi], XCHG_MEM_INIT
    mov eax, XCHG_REG_INIT

xchg_mr_01: xchg [edi], eax
xchg_mr_02: xchg [edi], eax
xchg_mr_03: xchg [edi], eax
xchg_mr_04: xchg [edi], eax
xchg_mr_05: xchg [edi], eax
xchg_mr_06: xchg [edi], eax
xchg_mr_07: xchg [edi], eax
xchg_mr_08: xchg [edi], eax

    cmp dword [edi], XCHG_MEM_INIT
    jne short phase_xchg_mr_fail
    cmp eax, XCHG_REG_INIT
    jne short phase_xchg_mr_fail
    jmp phase_push_imm

phase_xchg_mr_fail:
    jmp fail_0E

    times (0xF00 - ($ - $$)) db 0x90

;------------------------------------------------------------------------------
; Phase 15: Push immediate
;------------------------------------------------------------------------------
phase_push_imm:
    mov esp, STACK_TOP

push_imm_01: push dword PUSH_VALUE
push_imm_02: push dword PUSH_VALUE
push_imm_03: push dword PUSH_VALUE
push_imm_04: push dword PUSH_VALUE
push_imm_05: push dword PUSH_VALUE
push_imm_06: push dword PUSH_VALUE
push_imm_07: push dword PUSH_VALUE
push_imm_08: push dword PUSH_VALUE

    cmp esp, STACK_TOP - (ITERATIONS * 4)
    jne short phase_push_imm_fail
    cmp dword [esp], PUSH_VALUE
    jne short phase_push_imm_fail
    jmp phase_push_seg

phase_push_imm_fail:
    jmp fail_0F

    times (0x1000 - ($ - $$)) db 0x90

;------------------------------------------------------------------------------
; Phase 16: Push segment register
;------------------------------------------------------------------------------
phase_push_seg:
    mov esp, STACK_TOP

push_seg_01: push ds
push_seg_02: push ds
push_seg_03: push ds
push_seg_04: push ds
push_seg_05: push ds
push_seg_06: push ds
push_seg_07: push ds
push_seg_08: push ds

    cmp esp, STACK_TOP - (ITERATIONS * 4)
    jne short phase_push_seg_fail
    cmp word [esp], 0x0010
    jne short phase_push_seg_fail
    jmp phase_push_mem

phase_push_seg_fail:
    jmp fail_10

    times (0x1100 - ($ - $$)) db 0x90

;------------------------------------------------------------------------------
; Phase 17: Push memory value
;------------------------------------------------------------------------------
phase_push_mem:
    mov esi, LOAD_ADDR
    mov dword [esi], PUSH_VALUE
    mov esp, STACK_TOP

push_mem_01: push dword [esi]
push_mem_02: push dword [esi]
push_mem_03: push dword [esi]
push_mem_04: push dword [esi]
push_mem_05: push dword [esi]
push_mem_06: push dword [esi]
push_mem_07: push dword [esi]
push_mem_08: push dword [esi]

    cmp esp, STACK_TOP - (ITERATIONS * 4)
    jne short phase_push_mem_fail
    cmp dword [esp], PUSH_VALUE
    jne short phase_push_mem_fail
    jmp phase_pop_mem

phase_push_mem_fail:
    jmp fail_11

    times (0x1200 - ($ - $$)) db 0x90

;------------------------------------------------------------------------------
; Phase 18: Pop memory value
;------------------------------------------------------------------------------
phase_pop_mem:
    mov edi, STORE_ADDR
    mov esp, POP_STACK
    mov dword [esp + 0x00], POP_VALUE
    mov dword [esp + 0x04], POP_VALUE
    mov dword [esp + 0x08], POP_VALUE
    mov dword [esp + 0x0C], POP_VALUE
    mov dword [esp + 0x10], POP_VALUE
    mov dword [esp + 0x14], POP_VALUE
    mov dword [esp + 0x18], POP_VALUE
    mov dword [esp + 0x1C], POP_VALUE

    times (0x1280 - ($ - $$)) db 0x90

pop_mem_01: pop dword [edi]
pop_mem_02: pop dword [edi]
pop_mem_03: pop dword [edi]
pop_mem_04: pop dword [edi]
pop_mem_05: pop dword [edi]
pop_mem_06: pop dword [edi]
pop_mem_07: pop dword [edi]
pop_mem_08: pop dword [edi]

    cmp esp, POP_STACK + (ITERATIONS * 4)
    jne short phase_pop_mem_fail
    cmp dword [edi], POP_VALUE
    jne short phase_pop_mem_fail

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt

phase_pop_mem_fail:
    jmp fail_12

fail_01:
    mov al, 0x01
    jmp fail
fail_02:
    mov al, 0x02
    jmp fail
fail_03:
    mov al, 0x03
    jmp fail
fail_04:
    mov al, 0x04
    jmp fail
fail_05:
    mov al, 0x05
    jmp fail
fail_06:
    mov al, 0x06
    jmp fail
fail_07:
    mov al, 0x07
    jmp fail
fail_08:
    mov al, 0x08
    jmp fail
fail_09:
    mov al, 0x09
    jmp fail
fail_0A:
    mov al, 0x0A
    jmp fail
fail_0B:
    mov al, 0x0B
    jmp fail
fail_0C:
    mov al, 0x0C
    jmp fail
fail_0D:
    mov al, 0x0D
    jmp fail
fail_0E:
    mov al, 0x0E
    jmp fail
fail_0F:
    mov al, 0x0F
    jmp fail
fail_10:
    mov al, 0x10
    jmp fail
fail_11:
    mov al, 0x11
    jmp fail
fail_12:
    mov al, 0x12

fail:
    mov dx, DATA_PORT
    out dx, al
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
