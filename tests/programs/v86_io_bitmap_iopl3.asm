; v86_io_bitmap_iopl3.asm - VM86 always consults the TSS I/O bitmap
;
; Unlike protected mode, VM86 does not bypass the bitmap when IOPL=3.
; Port 84h is allowed and the test status port E0h is denied.  The denied OUT
; must have no bus side effect and must produce #GP(0) with the faulting VM86
; instruction address in the return frame.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4

STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

VEC_GP      equ 0x0D

SEL_CODE0   equ 0x08
SEL_DATA0   equ 0x10
SEL_STACK0  equ 0x18
SEL_TSS     equ 0x20

STACK0_TOP  equ 0x0FD8
VM86_SEG    equ 0x1000
VM86_SP     equ 0xE100

start:
    cli
    cld

    mov ax, cs
    mov ds, ax

    lgdt [gdt_desc]
    lidt [idt_desc]

    mov eax, cr0
    or eax, 1
    mov cr0, eax
    jmp SEL_CODE0:pm16_entry

pm16_entry:
    mov ax, SEL_DATA0
    mov ds, ax
    mov es, ax
    mov ax, SEL_STACK0
    mov ss, ax
    mov sp, STACK0_TOP

    mov ax, SEL_TSS
    ltr ax

    push dword VM86_SEG          ; GS
    push dword VM86_SEG          ; FS
    push dword VM86_SEG          ; DS
    push dword VM86_SEG          ; ES
    push dword VM86_SEG          ; SS
    push dword VM86_SP           ; ESP
    push dword 0x00023202        ; VM=1, IF=1, IOPL=3
    push dword VM86_SEG          ; CS
    push dword vm86_entry        ; EIP
    iretd

vm86_entry:
    mov ax, cs
    mov ds, ax
    mov ss, ax
    mov sp, VM86_SP

    mov al, 0x0F
vm86_out_allowed:
    out 0x84, al
    ; A denied operation must not reach the external bus before #GP.  Using the
    ; test status port makes a leaked write fail the regression immediately.
    mov al, STATUS_FAIL
vm86_out_denied:
    out STATUS_PORT, al

    ; Reaching here means the denied port bypassed the bitmap.
    hlt
    jmp $

; VM86 #GP frame on the USE16 ring-0 stack:
;   +00 error code, +04 EIP, +08 CS, +0C EFLAGS, then ESP/SS/ES/DS/FS/GS.
gp_handler:
    mov bp, sp
    mov ax, SEL_DATA0
    mov ds, ax

    cmp dword [ss:bp + 0x00], 0
    jne fail_error
    cmp dword [ss:bp + 0x04], vm86_out_denied
    jne fail_eip
    cmp dword [ss:bp + 0x08], VM86_SEG
    jne fail_cs
    test dword [ss:bp + 0x0c], 0x00020000
    jz fail_flags

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

fail_error:
    mov eax, 0x0D000001
    jmp fail
fail_eip:
    mov eax, [ss:bp + 0x04]
    or eax, 0x0D010000
    jmp fail
fail_cs:
    mov eax, [ss:bp + 0x08]
    or eax, 0x0D020000
    jmp fail
fail_flags:
    mov eax, [ss:bp + 0x0c]
    or eax, 0x0D030000
fail:
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

align 8
gdt:
    dq 0

    ; Ring-0 USE16 code, base=10000h, limit=ffffh.
    dw 0xffff
    dw 0x0000
    db 0x01
    db 10011011b
    db 00000000b
    db 0

    ; Ring-0 USE16 data, base=10000h, limit=ffffh.
    dw 0xffff
    dw 0x0000
    db 0x01
    db 10010011b
    db 00000000b
    db 0

    ; Ring-0 USE16 stack, base=12000h, limit=0fffh.
    dw 0x0fff
    dw 0x2000
    db 0x01
    db 10010011b
    db 00000000b
    db 0

    ; Available 386 TSS, base=10000h+tss, including the bitmap below.
tss_desc:
    dw tss_end - tss - 1
    dw tss
    db 0x01
    db 10001001b
    db 00000000b
    db 0
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x00010000

align 8
idt:
    times VEC_GP dq 0

    dw gp_handler
    dw SEL_CODE0
    db 0
    db 10001110b             ; 386 interrupt gate, DPL=0
    dw 0

    times (256 - VEC_GP - 1) dq 0
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd idt + 0x00010000

align 4
tss:
    dd 0                    ; +00 backlink
    dd STACK0_TOP           ; +04 ESP0
    dd SEL_STACK0           ; +08 SS0
    times 22 dd 0           ; +0C through +63
    dw 0                    ; +64 debug trap
    dw io_bitmap - tss      ; +66 I/O bitmap base

io_bitmap:
    times 0x10 db 0xff      ; deny ports 00h-7fh
    db 0xef                 ; 80h-87h: allow only port 84h
    times 11 db 0xff        ; deny ports 88h-dfh
    db 0xff                 ; deny status port e0h
    db 0xff                 ; mandatory all-ones terminator
tss_end:
