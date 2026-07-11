; rep_stos_decode.asm - REP STOSB must not poison following instruction decode
;
; Regression for byte streams seen in DOS/32A/FastDoom startup:
;   F3 AA 8B 44 24 08 8B 00 8B 34 ...
;   F3 AA B8 33 00 00 00 C7 05 94 ...
;
; The following MOV/C7 instructions are valid. If the REP prefix or STOSB
; decode state leaks forward, z386 can raise #UD on the next instruction.

BITS 32
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4

STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

SEL_CODE0   equ 0x08

start:
    cli
    cld

    lidt [cs:idt_desc]

    mov esp, stack_top

    ; Keep all memory operands in the same flat test segment.
    mov dword [esp + 0], 0x55667788
    mov dword [esp + 8], target1
    mov dword [target1], 0x11223344
    mov byte [buffer1], 0

    mov ecx, 1
    mov edi, buffer1
    mov al, 0xA5

case1_bytes:
    db 0xF3, 0xAA                  ; rep stosb
    db 0x8B, 0x44, 0x24, 0x08      ; mov eax, [esp+8]
    db 0x8B, 0x00                  ; mov eax, [eax]
    db 0x8B, 0x34, 0x24            ; mov esi, [esp]

    cmp byte [buffer1], 0xA5
    jne fail_case1_stos
    cmp eax, 0x11223344
    jne fail_case1_eax
    cmp esi, 0x55667788
    jne fail_case1_esi

    mov byte [buffer2], 0
    mov dword [0x194], 0
    mov ecx, 1
    mov edi, buffer2
    mov al, 0x5A

case2_bytes:
    db 0xF3, 0xAA                  ; rep stosb
    db 0xB8, 0x33, 0x00, 0x00, 0x00 ; mov eax, 0x33
    db 0xC7, 0x05, 0x94, 0x01, 0x00, 0x00
    dd 0x12345678                  ; mov dword [0x194], 0x12345678

    cmp byte [buffer2], 0x5A
    jne fail_case2_stos
    cmp eax, 0x00000033
    jne fail_case2_eax
    cmp dword [0x194], 0x12345678
    jne fail_case2_store

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

fail_case1_stos:
    mov eax, 0x00000001
    jmp fail
fail_case1_eax:
    mov eax, 0x00000002
    jmp fail
fail_case1_esi:
    mov eax, 0x00000003
    jmp fail
fail_case2_stos:
    mov eax, 0x00000004
    jmp fail
fail_case2_eax:
    mov eax, 0x00000005
    jmp fail
fail_case2_store:
    mov eax, 0x00000006
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
    dd 0x00010000 + idt

align 4
target1: dd 0
buffer1: db 0
buffer2: db 0
align 4
stack:
    times 256 db 0
stack_top:
