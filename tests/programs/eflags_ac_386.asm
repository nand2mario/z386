; The 386 keeps EFLAGS.AC reserved and clear. 486-class detection commonly
; distinguishes the CPUs by trying to toggle this bit through POPFD.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
AC_BIT      equ 0x00040000

start:
    cli
    xor ax, ax
    mov ss, ax
    mov sp, 0x8000

    pushfd
    pop ebx
    mov eax, ebx
    xor eax, AC_BIT
    push eax
    popfd
    pushfd
    pop edx

    push ebx
    popfd

    xor edx, ebx
    test edx, AC_BIT
    jnz .fail

    mov al, 0x01
    out STATUS_PORT, al
    hlt

.fail:
    mov al, 1
    out DATA_PORT, al
    mov al, 0xff
    out STATUS_PORT, al
    hlt
