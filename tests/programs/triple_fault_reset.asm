; The PUSH first raises #SS. Its unusable stack faults again while delivering
; #SS, producing #DF, then faults while delivering #DF and requests reset.

BITS 16
org 0

start:
    cli
    mov sp, 1
    push ax

    ; A working triple-fault detector never reaches this path.
    mov al, 0xff
    mov dx, 0xe0
    out dx, al
    hlt
