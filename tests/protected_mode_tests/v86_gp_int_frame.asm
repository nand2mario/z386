; v86_gp_int_frame.asm - VM86 INT with IOPL<3 must #GP with a decodable frame
;
; EMM386 depends on this exact behavior.  A VM86 software interrupt executed
; with IOPL < 3 faults as #GP(0); the monitor then decodes the faulting guest
; bytes at saved CS:EIP.  If the saved frame points past CD nn, or if the VM86
; frame layout is wrong, the monitor can reflect vector 0x0d instead of the
; requested guest interrupt.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4

STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

VEC_GP      equ 0x0D
VEC_INT21   equ 0x21

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
    or  eax, 1
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

    ; Enter VM86 with IOPL=0.  The guest INT 21h below must fault as #GP.
    push dword VM86_SEG          ; GS
    push dword VM86_SEG          ; FS
    push dword VM86_SEG          ; DS
    push dword VM86_SEG          ; ES
    push dword VM86_SEG          ; SS
    push dword VM86_SP           ; ESP
    push dword 0x00020202        ; EFLAGS: VM=1, IF=1, IOPL=0
    push dword VM86_SEG          ; CS
    push dword vm86_entry        ; EIP
    iretd

vm86_entry:
    mov ax, cs
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov sp, VM86_SP

vm86_int21:
    int VEC_INT21
vm86_after_int21:
    hlt
    jmp $

; #GP handler.  For a VM86 #GP with error code, expected USE16 ring-0 stack:
;   [SP+00] = error code
;   [SP+04] = EIP
;   [SP+08] = CS
;   [SP+0C] = EFLAGS
;   [SP+10] = ESP
;   [SP+14] = SS
;   [SP+18] = ES
;   [SP+1C] = DS
;   [SP+20] = FS
;   [SP+24] = GS
gp_handler:
    mov bp, sp
    mov ax, SEL_DATA0
    mov ds, ax

    cmp dword [ss:bp + 0x00], 0
    jne fail_error_code

    cmp dword [ss:bp + 0x04], vm86_int21
    jne fail_eip

    cmp dword [ss:bp + 0x08], VM86_SEG
    jne fail_cs

    mov eax, [ss:bp + 0x0c]
    test eax, 0x00020000
    jz fail_eflags

    cmp dword [ss:bp + 0x10], VM86_SP
    jne fail_esp

    cmp dword [ss:bp + 0x14], VM86_SEG
    jne fail_ss

    cmp dword [ss:bp + 0x18], VM86_SEG
    jne fail_es

    cmp dword [ss:bp + 0x1c], VM86_SEG
    jne fail_ds

    cmp dword [ss:bp + 0x20], VM86_SEG
    jne fail_fs

    cmp dword [ss:bp + 0x24], VM86_SEG
    jne fail_gs

    ; Confirm monitor can decode the faulting guest instruction bytes.
    cmp byte [vm86_int21], 0xcd
    jne fail_opcode

    cmp byte [vm86_int21 + 1], VEC_INT21
    jne fail_opcode

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

int21_handler:
    mov eax, 0x21000000
    jmp fail

fail_error_code:
    mov eax, 0x0d000001
    jmp fail

fail_eip:
    mov eax, [ss:bp + 0x04]
    or eax, 0x0d010000
    jmp fail

fail_cs:
    mov eax, [ss:bp + 0x08]
    or eax, 0x0d020000
    jmp fail

fail_eflags:
    mov eax, [ss:bp + 0x0c]
    or eax, 0x0d030000
    jmp fail

fail_esp:
    mov eax, [ss:bp + 0x10]
    or eax, 0x0d040000
    jmp fail

fail_ss:
    mov eax, [ss:bp + 0x14]
    or eax, 0x0d050000
    jmp fail

fail_es:
    mov eax, [ss:bp + 0x18]
    or eax, 0x0d060000
    jmp fail

fail_ds:
    mov eax, [ss:bp + 0x1c]
    or eax, 0x0d070000
    jmp fail

fail_fs:
    mov eax, [ss:bp + 0x20]
    or eax, 0x0d080000
    jmp fail

fail_gs:
    mov eax, [ss:bp + 0x24]
    or eax, 0x0d090000
    jmp fail

fail_opcode:
    movzx eax, byte [vm86_int21]
    shl eax, 8
    mov al, [vm86_int21 + 1]
    or eax, 0x0d0a0000
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
gdt:
    dq 0x0000000000000000

    ; Ring-0 USE16 code, base=0x10000, limit=0xffff.
    dw 0xffff
    dw 0x0000
    db 0x01
    db 10011011b
    db 00000000b
    db 0x00

    ; Ring-0 USE16 data, base=0x10000, limit=0xffff.
    dw 0xffff
    dw 0x0000
    db 0x01
    db 10010011b
    db 00000000b
    db 0x00

    ; Ring-0 USE16 stack, base=0x12000, limit=0x0fff.
    dw 0x0fff
    dw 0x2000
    db 0x01
    db 10010011b
    db 00000000b
    db 0x00

    ; 32-bit TSS, base=0x10000+tss, limit=0x67.
tss_desc:
    dw 0x0067
    dw tss
    db 0x01
    db 10001001b
    db 00000000b
    db 0x00
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x00010000

align 8
idt:
    times VEC_GP dq 0

    ; #GP: 386 interrupt gate, DPL=0, target USE16 ring-0 code.
    dw gp_handler
    dw SEL_CODE0
    db 0
    db 10001110b
    dw 0

    times (VEC_INT21 - VEC_GP - 1) dq 0

    ; If VM86 INT 21h is allowed directly, fail.
    dw int21_handler
    dw SEL_CODE0
    db 0
    db 11101110b
    dw 0

    times (256 - VEC_INT21 - 1) dq 0
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd idt + 0x00010000

align 4
tss:
    dd 0                    ; +00 backlink
    dd STACK0_TOP           ; +04 ESP0
    dd SEL_STACK0           ; +08 SS0
    dd 0                    ; +0C ESP1
    dd 0                    ; +10 SS1
    dd 0                    ; +14 ESP2
    dd 0                    ; +18 SS2
    dd 0                    ; +1C CR3
    dd 0                    ; +20 EIP
    dd 0                    ; +24 EFLAGS
    dd 0                    ; +28 EAX
    dd 0                    ; +2C ECX
    dd 0                    ; +30 EDX
    dd 0                    ; +34 EBX
    dd 0                    ; +38 ESP
    dd 0                    ; +3C EBP
    dd 0                    ; +40 ESI
    dd 0                    ; +44 EDI
    dd 0                    ; +48 ES
    dd 0                    ; +4C CS
    dd 0                    ; +50 SS
    dd 0                    ; +54 DS
    dd 0                    ; +58 FS
    dd 0                    ; +5C GS
    dd 0                    ; +60 LDTR
    dw 0                    ; +64 debug trap
    dw 104                  ; +66 IOPB offset
