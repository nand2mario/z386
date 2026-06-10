; movss_shadow.asm - Verify MOV SS interrupt shadow with hardware INTR
;
; Test:
;   1) Program IVT[0x20] with ISR recording a marker byte
;   2) Enable IF without any pending interrupt
;   3) Configure testbench to assert INTR after one retired instruction
;   4) Request INTR, execute MOV SS, then one marker instruction
;   5) Verify ISR observed marker=1 (interrupt taken after post-MOV-SS instruction)

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

    ; Set up stack and IVT.
    xor ax, ax
    mov ss, ax
    mov sp, 0x8000
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
    mov ax, 2               ; request + (mov ax,ss) + (mov ss,ax)
    out dx, ax

    mov dx, SIGNAL_CYCLES_PORT
    mov ax, 1
    out dx, ax

    ; Ensure IF=1 and no STI shadow in flight.
    sti
    nop

    ; Request INTR, then execute MOV SS and the shadowed next instruction.
    mov dx, SIGNAL_PORT
    mov al, 1
    out dx, al

    mov ax, ss
    mov ss, ax              ; establishes interrupt shadow window
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
    mov eax, 0x00000002     ; INTR taken before post-MOV-SS instruction
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
