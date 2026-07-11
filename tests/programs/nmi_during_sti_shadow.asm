; nmi_during_sti_shadow.asm - Verify NMI is not blocked by STI shadow
;
; We request NMI, then execute STI followed by marker write.
; If NMI is not shadowed, handler should see marker=0.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
SIGNAL_PORT equ 0xE8
SIGNAL_CYCLES_PORT equ 0xEC
SIGNAL_INSTR_PORT  equ 0xF0
SIGNAL_NMIWIDTH_PORT equ 0xF8

STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

NMI_VECTOR equ 0x02

start:
    cli
    cld

    mov ax, cs
    mov ds, ax

    ; Set IVT entry for NMI.
    xor ax, ax
    mov es, ax
    mov word [es:NMI_VECTOR*4], isr_nmi
    mov word [es:NMI_VECTOR*4+2], 0x1000

    mov word [nmi_count], 0
    mov byte [marker], 0
    mov byte [nmi_seen_marker], 0

    mov dx, SIGNAL_INSTR_PORT
    mov ax, 1               ; NMI after one retired instruction
    out dx, ax
    mov dx, SIGNAL_CYCLES_PORT
    mov ax, 1
    out dx, ax
    mov dx, SIGNAL_NMIWIDTH_PORT
    mov ax, 8               ; keep NMI high for deterministic capture
    out dx, ax

    ; Request NMI, then STI. NMI should be taken before marker write.
    mov dx, SIGNAL_PORT
    mov al, 2
    out dx, al

    sti
    mov byte [marker], 1
    nop

    mov cx, 10000
.wait_nmi:
    cmp word [nmi_count], 1
    je .nmi_ok
    dec cx
    jnz .wait_nmi

    mov eax, 0x00000001     ; timeout waiting for NMI
    jmp fail

.nmi_ok:
    cmp byte [nmi_seen_marker], 0
    jne .fail_shadowed

    cmp word [nmi_count], 1
    jne .fail_count

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

.fail_shadowed:
    mov eax, 0x00000002     ; NMI behaved as if shadowed
    jmp fail
.fail_count:
    mov eax, 0x00000003
    jmp fail

isr_nmi:
    push ax
    mov al, [cs:marker]
    mov [cs:nmi_seen_marker], al
    inc word [cs:nmi_count]
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
nmi_count:       dw 0
marker:          db 0
nmi_seen_marker: db 0
