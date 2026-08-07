; Early NexGen Nx586 identification distinguishes it from an Intel 80386 by
; observing ZF after DIV. The 80386 updates ZF from the quotient; Nx586 leaves
; the preceding value unchanged.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4

start:
    cli
    xor ax, ax
    mov ss, ax
    mov sp, 0x8000

    ; Exact 80386/Nx586 discriminator from the NexGen identification note.
    mov ax, 0x5555
    xor dx, dx                  ; Establish ZF=1.
    mov cx, 2
    div cx                      ; Quotient 0x2AAA must clear ZF on an 80386.
    jz .fail
    cmp ax, 0x2AAA
    jne .fail
    cmp dx, 1
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
