; sti_shadow.asm - Verify STI interrupt shadow with hardware INTR
;
; Test:
;   1) Program IVT[0x20] with a simple ISR that records a marker byte
;   2) Configure testbench to assert INTR after one retired instruction
;   3) Execute STI, then one marker instruction
;   4) Verify ISR observed marker=1 (interrupt taken after the post-STI instruction)

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
SIGNAL_PORT equ 0xE8
SIGNAL_CYCLES_PORT equ 0xEC
SIGNAL_INSTR_PORT  equ 0xF0
SIGNAL_VECTOR_PORT equ 0xF4

STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

INTR_VECTOR equ 0x20

start:
    cli
    cld

    mov ax, cs
    mov ds, ax

    ; Set IVT entry for vector 0x20.
    xor ax, ax
    mov es, ax
    mov word [es:INTR_VECTOR*4], isr_intr
    mov word [es:INTR_VECTOR*4+2], 0x1000

    mov word [intr_count], 0
    mov byte [marker], 0
    mov byte [intr_seen_marker], 0

    ; Configure deterministic trigger behavior in testbench.
    mov dx, SIGNAL_VECTOR_PORT
    mov al, INTR_VECTOR
    out dx, al

    mov dx, SIGNAL_INSTR_PORT
    mov ax, 1               ; assert INTR after one retired instruction
    out dx, ax

    mov dx, SIGNAL_CYCLES_PORT
    mov ax, 1               ; fallback delay if instr-delay mode is disabled
    out dx, ax

    cli
    mov dx, SIGNAL_PORT
    mov al, 1               ; request INTR
    out dx, al

    sti
    mov byte [marker], 1    ; must execute before ISR entry
    nop

    mov cx, 10000
.wait_intr:
    cmp word [intr_count], 1
    je .intr_ok
    dec cx
    jnz .wait_intr

    mov eax, 0x00000001     ; timeout waiting for INTR
    jmp fail

.intr_ok:
    cmp byte [intr_seen_marker], 1
    jne .fail_shadow

    cmp word [intr_count], 1
    jne .fail_count

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

.fail_shadow:
    mov eax, 0x00000002     ; INTR taken before post-STI instruction
    jmp fail

.fail_count:
    mov eax, 0x00000003     ; unexpected INTR count
    jmp fail

isr_intr:
    push ax
    mov al, [cs:marker]
    mov [cs:intr_seen_marker], al
    inc word [cs:intr_count]
    pop ax
    iret

fail:
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

align 2
intr_count:       dw 0
marker:           db 0
intr_seen_marker: db 0
