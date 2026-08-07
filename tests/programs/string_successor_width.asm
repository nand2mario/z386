; Mixed-address-size string retirement regression.
;
; The MOVSB address size controls its index updates even when the successor
; uses address-16. Do not recreate the B1 successor-width erratum merely
; because the extracted microcode reports a B1 revision.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4

start:
    cli
    xor ax, ax
    mov ds, ax
    mov es, ax
    mov sp, 0x8000

    mov byte [0xffff], 0x5a
    mov esi, 0x0000ffff
    mov edi, 0x0000ffff

    push ax
    a32 movsb
    pop ax                      ; Address-16 successor must not alter MOVSB width.

    cmp esi, 0x00010000
    jne .fail_esi
    cmp edi, 0x00010000
    jne .fail_edi

    mov al, 0x01
    out STATUS_PORT, al
    hlt

.fail_esi:
    mov al, 1
    jmp .fail
.fail_edi:
    mov al, 2
.fail:
    out DATA_PORT, al
    mov al, 0xff
    out STATUS_PORT, al
    hlt
