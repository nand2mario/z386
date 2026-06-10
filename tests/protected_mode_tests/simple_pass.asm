; Simplest possible test - just report PASS
BITS 32
org 0

STATUS_PORT equ 0xE0
STATUS_PASS equ 0x01

    mov al, STATUS_PASS
    out STATUS_PORT, al
    hlt
