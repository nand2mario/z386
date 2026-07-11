; pe_llldt_cpl_cache.asm - RM->PE CPL source regression (JemmEx-style)
;
; Real-mode CS may have non-zero low bits. After setting CR0.PE=1 without
; reloading CS, privileged instructions must still run with effective CPL=0
; (from cached real-mode CS descriptor), not CS selector RPL bits.
;
; Sequence: PE=1 -> XOR AX,AX -> LLDT AX -> PE=0
; Expected: no #GP loop, test reports PASS.

BITS 16
org 0

STATUS_PORT equ 0xE0
STATUS_PASS equ 0x01

start:
    cli
    cld

    ; Force CS low bits to 2 while preserving physical execution stream:
    ; current physical = 0x1000<<4 + ip, target physical = 0x0FFE<<4 + ip2
    ; so ip2 = ip + 0x20.
    jmp 0x0FFE:cs_rpl2_entry + 0x20

cs_rpl2_entry:
    mov eax, cr0
    or  eax, 0x00000001
    mov cr0, eax

    xor ax, ax
    lldt ax

    mov eax, cr0
    and eax, 0xFFFFFFFE
    mov cr0, eax

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al

hang:
    hlt
    jmp hang
