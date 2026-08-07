; 80486 BSWAP extension used by HWiNFO's processor/errata probes.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4

start:
    cli
    xor ax, ax
    mov ss, ax
    mov sp, 0x8000

    mov eax, 0x12345678
    pushf
    pop si
    db 0x66, 0x0f, 0xc8       ; bswap eax
    pushf
    pop di
    cmp eax, 0x78563412
    jne .fail
    cmp si, di                 ; BSWAP must not change flags.
    jne .fail

    mov ecx, 0x89abcdef
    db 0x0f, 0xc9             ; BSWAP remains r32 without 66h.
    cmp ecx, 0xefcdab89
    jne .fail

    mov edx, 0x01020304
    db 0x66, 0x0f, 0xca       ; bswap edx
    cmp edx, 0x04030201
    jne .fail

    mov ebx, 0xa1b2c3d4
    db 0x66, 0x0f, 0xcb       ; bswap ebx
    cmp ebx, 0xd4c3b2a1
    jne .fail

    mov ebp, 0x10203040
    db 0x66, 0x0f, 0xcd       ; bswap ebp
    cmp ebp, 0x40302010
    jne .fail

    mov esi, 0x55667788
    db 0x66, 0x0f, 0xce       ; bswap esi
    cmp esi, 0x88776655
    jne .fail

    mov edi, 0xdeadbeef
    db 0x66, 0x0f, 0xcf       ; bswap edi
    cmp edi, 0xefbeadde
    jne .fail

    mov esp, 0x11223344
    db 0x66, 0x0f, 0xcc       ; bswap esp
    mov eax, esp
    mov esp, 0x8000
    cmp eax, 0x44332211
    jne .fail

    mov al, 0x01
    out STATUS_PORT, al
    hlt

.fail:
    mov al, 1
    out DATA_PORT, al
    mov al, 0xff
    out STATUS_PORT, al
    hlt
