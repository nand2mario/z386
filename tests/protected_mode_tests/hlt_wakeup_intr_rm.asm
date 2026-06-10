; hlt_wakeup_intr_rm.asm - Real-mode HLT wakes on hardware INTR

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

    ; IVT[0x20] -> ISR.
    xor ax, ax
    mov es, ax
    mov word [es:INTR_VECTOR*4], isr_intr
    mov word [es:INTR_VECTOR*4+2], 0x1000

    mov word [intr_count], 0
    mov byte [after_hlt], 0

    ; Configure INTR vector and cycle-based delay.
    mov dx, SIGNAL_VECTOR_PORT
    mov al, INTR_VECTOR
    out dx, al

    mov dx, SIGNAL_INSTR_PORT
    mov ax, 0               ; disable instruction-delay mode
    out dx, ax

    mov dx, SIGNAL_CYCLES_PORT
    mov ax, 40              ; assert while CPU is halted
    out dx, ax

    sti
    mov dx, SIGNAL_PORT
    mov al, 1               ; request INTR
    out dx, al

    hlt                     ; should wake when INTR is delivered

    mov byte [after_hlt], 1
    nop

    mov cx, 10000
.wait_intr:
    cmp word [intr_count], 1
    je .intr_ok
    dec cx
    jnz .wait_intr
    mov eax, 0x00000001
    jmp fail

.intr_ok:
    cmp byte [after_hlt], 1
    jne .fail_resume

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

.fail_resume:
    mov eax, 0x00000002     ; did not resume after HLT
    jmp fail

isr_intr:
    inc word [cs:intr_count]
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
intr_count: dw 0
after_hlt:  db 0
