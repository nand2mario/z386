; intgate16_errcode_frame.asm - 16-bit interrupt gate: error code frame layout
;
; Regression test for the IN+D latched-delta bug: in the MORE_PRIVILEGE
; microcode path, uop 627 sets BITS32 before the error-code push, and the
; IN+D ops at 629/62C must keep using the IND_DELTA latched by the earlier
; IN=+ (60A, NEGWSZ = -2 through a 16-bit gate).  z386 used the live word
; size, pushed the error code 4 bytes below EIP and left SP 2 too low, so a
; 16-bit handler read a frame shifted by 2 (Ergo DPMI's HLT-gate #GP
; handler saw IP/CS as 0020:0000 and bailed with "Unhandled exception").
;
; Scenario (mirrors Borland/Ergo DPMI under EMM386):
;   ring0 PM -> LTR (286 TSS: SP0@+2, SS0@+4) -> IRET to ring3 -> HLT
;   HLT at CPL3 raises #GP(0); vector 0D is a 286 interrupt gate (type 6)
;   to a USE16 ring0 handler.  With privilege change + error code the frame
;   at SS0:SP0 must be six 16-bit words:
;     SP0-2: SS3   SP0-4: SP3   SP0-6: FLAGS
;     SP0-8: CS3   SP0-10: IP(=fault addr)   SP0-12: err(=0),  SP = SP0-12
;
; Results: port 0xE0 status (0x01 pass / 0xFF fail), port 0xE4 code.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

SEL_CODE0   equ 0x08
SEL_DATA0   equ 0x10
SEL_CODE3   equ 0x18
SEL_DATA3   equ 0x20
SEL_TSS     equ 0x28
SEL_CODE0_16 equ 0x30
SEL_DATA0_16 equ 0x38

SEL_CODE3_RPL3 equ (SEL_CODE3 | 3)
SEL_DATA3_RPL3 equ (SEL_DATA3 | 3)

STACK0_TOP  equ 0x3000
STACK3_TOP  equ 0x4000

start:
    cli
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]

    mov eax, cr0
    or eax, 1
    mov cr0, eax
    jmp dword SEL_CODE0:pm_entry

%macro DIAG 1
    mov eax, %1
    mov dx, DATA_PORT
    out dx, eax
%endmacro

BITS 32
pm_entry:
    mov ax, SEL_DATA0
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov esp, STACK0_TOP
    DIAG 0x11

    mov ax, SEL_TSS
    ltr ax
    DIAG 0x12

    ; IRET to ring 3 (IOPL=3 so ring3 can use OUT for diags)
    push dword SEL_DATA3_RPL3
    push dword STACK3_TOP
    push dword 0x00003002
    push dword SEL_CODE3_RPL3
    push dword ring3_entry
    iretd

ring3_entry:
    mov ax, SEL_DATA3_RPL3
    mov ds, ax
    DIAG 0x13
fault_site:
    mov eax, cr0                ; privileged at CPL3 -> #GP(0) via 16-bit gate
                                ; (not HLT: the TB stops when a boundary occurs
                                ; with i.opcode==F4, even for a faulted HLT)
    jmp $

;------------------------------------------------------------------
; #GP handler: USE16 ring0 via 286 interrupt gate (type 6)
;------------------------------------------------------------------
BITS 16
isr_gp16:
    mov ax, SEL_DATA0
    mov ds, ax

    ; SP must be SP0 - 12 (six 16-bit words, error code included)
    cmp sp, STACK0_TOP - 12
    jne fail_21
    ; [sp] = error code 0
    pop ax
    cmp ax, 0
    jne fail_22
    ; [sp+2] = IP of the faulting HLT itself (fault, not trap)
    pop ax
    cmp ax, fault_site
    jne fail_23
    ; [sp+4] = ring3 CS
    pop ax
    cmp ax, SEL_CODE3_RPL3
    jne fail_24
    ; [sp+6] = FLAGS (IOPL=3, IF=0 -> 0x3002; bit1 always 1 -> 0x3002|2)
    pop ax
    and ax, 0x3202
    cmp ax, 0x3002
    jne fail_25
    ; [sp+8] = SP3, [sp+10] = SS3
    pop ax
    cmp ax, STACK3_TOP
    jne fail_26
    pop ax
    cmp ax, SEL_DATA3_RPL3
    jne fail_27

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt

fail_21:
    mov eax, 0x21
    jmp fail
fail_22:
    mov eax, 0x22
    jmp fail
fail_23:
    mov eax, 0x23
    jmp fail
fail_24:
    mov eax, 0x24
    jmp fail
fail_25:
    mov eax, 0x25
    jmp fail
fail_26:
    mov eax, 0x26
    jmp fail
fail_27:
    mov eax, 0x27
fail:
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt

;==================================================================
; Data
;==================================================================
align 8
gdt:
    dq 0x0000000000000000
    ; 0x08: ring0 32-bit code, base 0x10000
    dq 0x00CF9B010000FFFF
    ; 0x10: ring0 32-bit data, base 0x10000
    dq 0x00CF93010000FFFF
    ; 0x18: ring3 32-bit code, base 0x10000 (DPL=3)
    dq 0x00CFFB010000FFFF
    ; 0x20: ring3 32-bit data, base 0x10000 (DPL=3)
    dq 0x00CFF3010000FFFF
    ; 0x28: available 286 TSS (type 1), base = tss286+0x10000
    dw 0x002B
    dw tss286
    db 0x01
    db 0x81
    db 0x00
    db 0x00
    ; 0x30: ring0 16-bit code, base 0x10000, limit 0xFFF (D=0).
    ; Deliberately different from the SS0 limit: the new-stack limit stash
    ; at uop 608 (CTSSAF+SDEL) once got dropped, leaving this CS limit in
    ; the stack-slot cache and faulting the gate frame pushes (#SS loop).
    dq 0x00009B0100000FFF
    ; 0x38: ring0 16-bit data, base 0x10000, limit 64KB (D=0) - TC's kernel
    ; stack is a 16-bit segment; SS0 D bit must not widen the gate frame
    dq 0x000093010000FFFF
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x00010000

; IDT: vector 0x0D = 286 interrupt gate (type 6), USE16 ring0 handler
align 8
idt:
    times 0x0D dq 0
    dw isr_gp16                 ; offset [15:0]
    dw SEL_CODE0_16
    db 0
    db 10000110b                ; P=1 DPL=0 type=6 (16-bit interrupt gate)
    dw 0                        ; reserved in 286 gate
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd idt + 0x00010000

; 286 TSS: SP0 at +2, SS0 at +4
align 4
tss286:
    dw 0
    dw STACK0_TOP               ; +2  SP0
    dw SEL_DATA0_16             ; +4  SS0
    times 19 dw 0
tss286_end:
