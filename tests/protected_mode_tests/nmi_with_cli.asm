; nmi_with_cli.asm - Verify NMI is delivered even when IF=0 (CLI)

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
SIGNAL_PORT equ 0xE8
SIGNAL_CYCLES_PORT equ 0xEC
SIGNAL_INSTR_PORT  equ 0xF0

STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

NMI_VECTOR equ 0x02

start:
    cli
    cld

    mov ax, cs
    mov ds, ax

    ; Set IVT entry for NMI (vector 2).
    xor ax, ax
    mov es, ax
    mov word [es:NMI_VECTOR*4], isr_nmi
    mov word [es:NMI_VECTOR*4+2], 0x1000

    mov word [nmi_count], 0

    ; Deliver NMI after one retired instruction.
    mov dx, SIGNAL_INSTR_PORT
    mov ax, 1
    out dx, ax
    mov dx, SIGNAL_CYCLES_PORT
    mov ax, 1
    out dx, ax

    cli
    mov dx, SIGNAL_PORT
    mov al, 2               ; request NMI
    out dx, al
    nop                     ; IF remains 0

    mov cx, 10000
.wait_nmi:
    cmp word [nmi_count], 1
    je .nmi_ok
    dec cx
    jnz .wait_nmi

    mov eax, 0x00000001     ; timeout waiting for NMI under CLI
    jmp fail

.nmi_ok:
    cmp word [nmi_count], 1
    jne .fail_count

    pushf
    pop ax
    test ax, 0x0200         ; IF should still be 0
    jnz .fail_if

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

.fail_count:
    mov eax, 0x00000002
    jmp fail
.fail_if:
    mov eax, 0x00000003
    jmp fail

isr_nmi:
    inc word [cs:nmi_count]
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
nmi_count: dw 0
