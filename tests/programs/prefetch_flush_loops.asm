; prefetch_flush_loops.asm - Baseline front-end refill microbenchmark
;
; This test builds a few tiny taken-branch loops in 32-bit protected mode.
; Each taken branch should flush the front-end, forcing prefetch + decode to
; refill before the first target instruction can begin executing again.
;
; Trace focus:
;   Measure from the taken back-edge (`jnz` or `jmp`) to the first `i_first`
;   pulse at the loop target. The four phases vary target alignment and the
;   length of the first instruction after the flush.
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

start:
    ; Phase 1: 4-byte aligned target, 1-byte first instruction.
    xor esi, esi
    mov ecx, ITERATIONS
    jmp .phase1_loop

    times (0x080 - ($ - $$)) db 0x90

.phase1_loop:
    inc esi                     ; 1-byte target instruction at offset 0x080
    dec ecx
    jnz short .phase1_loop

    cmp esi, ITERATIONS
    jne .fail_01

    ; Phase 2: target at 4N+3, 2-byte first instruction crossing a fetch boundary.
    xor ebx, ebx
    mov ecx, ITERATIONS
    jmp .phase2_loop

    times (0x103 - ($ - $$)) db 0x90

.phase2_loop:
    mov edi, edi                ; 2-byte target instruction at offset 0x103
    inc ebx
    dec ecx
    jnz short .phase2_loop

    cmp ebx, ITERATIONS
    jne .fail_02

    ; Phase 3: target at 4N+1, 5-byte first instruction spanning two fetches.
    mov eax, 0x13579BDF
    xor edx, edx
    mov ecx, ITERATIONS
    jmp .phase3_loop

    times (0x181 - ($ - $$)) db 0x90

.phase3_loop:
    cmp eax, 0x13579BDF         ; 5-byte target instruction at offset 0x181
    inc edx
    dec ecx
    jnz short .phase3_loop

    cmp edx, ITERATIONS
    jne .fail_03

    ; Phase 4: unconditional short jump back-edge.
    xor ebp, ebp
    mov ecx, ITERATIONS
    jmp .phase4_loop

    times (0x200 - ($ - $$)) db 0x90

.phase4_loop:
    inc ebp                     ; 1-byte target instruction at offset 0x200
    dec ecx
    jz short .phase4_done
    jmp short .phase4_loop

.phase4_done:
    cmp ebp, ITERATIONS
    jne .fail_04

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt

.fail_01:
    mov al, 0x01
    jmp .fail
.fail_02:
    mov al, 0x02
    jmp .fail
.fail_03:
    mov al, 0x03
    jmp .fail
.fail_04:
    mov al, 0x04

.fail:
    mov dx, DATA_PORT
    out dx, al
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
