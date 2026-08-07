; tf_single_step_rm.asm - TF traps after the following instruction

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

start:
    cli
    cld

    mov ax, cs
    mov ds, ax
    xor ax, ax
    mov es, ax
    mov word [es:1*4], isr_db
    mov word [es:1*4+2], 0x1000
    mov word [es:3*4], isr_bp
    mov word [es:3*4+2], 0x1000

    mov byte [marker], 0
    mov byte [seen_marker], 0
    mov word [trap_count], 0

    ; IRET restores TF but must not itself trap. The MOV at .stepped is the
    ; first instruction executed with TF active and must retire before #DB.
    pushf
    pop ax
    or ax, 0x0100
    push ax
    push cs
    push word .stepped
    iret

.stepped:
    mov byte [marker], 1

    cmp word [trap_count], 1
    jne .fail_count
    cmp byte [seen_marker], 1
    jne .fail_early

    ; INT clears TF on entry and must not also generate a #DB. Second Reality's
    ; unpacker relies on this while alternating INT 3 and single stepping.
    mov word [trap_count], 0
    mov word [bp_count], 0
    pushf
    pop ax
    or ax, 0x0100
    push ax
    push cs
    push word .breakpoint
    iret

.breakpoint:
    int3
    cmp word [bp_count], 1
    jne .fail_bp
    cmp word [trap_count], 0
    jne .fail_int_db

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

.fail_count:
    mov eax, 1
    jmp fail
.fail_early:
    mov eax, 2
    jmp fail
.fail_bp:
    mov eax, 3
    jmp fail
.fail_int_db:
    mov eax, 4
    jmp fail

isr_db:
    push bp
    mov bp, sp
    and word [ss:bp+6], 0xFEFF ; clear TF in the saved FLAGS image
    mov al, [cs:marker]
    mov [cs:seen_marker], al
    inc word [cs:trap_count]
    pop bp
    iret

isr_bp:
    push bp
    mov bp, sp
    and word [ss:bp+6], 0xFEFF ; keep TF clear after returning from INT 3
    inc word [cs:bp_count]
    pop bp
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
trap_count: dw 0
bp_count: dw 0
marker: db 0
seen_marker: db 0
