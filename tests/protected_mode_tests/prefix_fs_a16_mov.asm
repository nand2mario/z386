; prefix_fs_a16_mov.asm - 66/64/67-prefixed MOV must decode on 386
;
; FastDoom/DOS32A displayed this valid byte stream at #UD:
;   66 64 67 8B 04 1E C3 8C E8 85
;
; The first instruction is:
;   66 64 67 8B 04    mov ax, fs:[si]
;
; In 32-bit code, 66 selects a 16-bit destination, 64 selects FS, and
; 67 selects 16-bit addressing. ModR/M 04 is [SI] in 16-bit addressing,
; not a SIB byte.

BITS 32
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4

STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

SEL_CODE0   equ 0x08

start:
    cli
    lidt [cs:idt_desc]

    mov esp, stack_top
    mov esi, fs_word
    xor eax, eax

case_bytes:
    db 0x66, 0x64, 0x67, 0x8B, 0x04    ; mov ax, fs:[si]

    cmp ax, 0xBEEF
    jne fail_value

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

fail_value:
    mov eax, 0x00000001
    jmp fail

ud_handler:
    mov eax, 0x00000600
    jmp fail

fail:
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

align 8
idt:
    times 6 dq 0
    ; #UD: 32-bit interrupt gate to ud_handler
    dw ud_handler
    dw SEL_CODE0
    db 0
    db 10001110b
    dw 0
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd idt

align 4
fs_word:
    dw 0xBEEF

align 16
stack:
    times 256 db 0
stack_top:
