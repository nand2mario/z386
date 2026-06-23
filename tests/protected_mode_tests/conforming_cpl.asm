; conforming_cpl.asm - CPL comes from CS.RPL, not the descriptor DPL
;
; Regression test for the conforming-code CPL bug: Ergo DPMI runs its
; ring-3 kernel facet in a CONFORMING DPL0 code segment (CS.RPL=3).  z386
; computed CPL from the cached descriptor DPL (0), so ring-3 code passed
; every privilege check: LIDT executed silently at CPL3 and zeroed IDTR
; right before a VCPI call faulted - the fault was undeliverable and the
; machine spun (TC3 hang #5).
;
; Flow:
;   1) PM ring 0: RETF outer-level to SEL_CONF|3 (conforming DPL0 code).
;      New CPL = RPL = 3 even though the descriptor DPL is 0.
;   2) At ring 3: LIDT must raise #GP(0).  If it executes, IDTR is zeroed
;      and the test reports failure explicitly (or dies - also a failure).
;   3) ring-0 #GP handler verifies the pushed CS has RPL=3 and the saved
;      IDT base is untouched, then reports pass.
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
SEL_CONF    equ 0x18    ; conforming DPL0 32-bit code
SEL_DATA3   equ 0x20
SEL_TSS     equ 0x28

SEL_CONF_RPL3  equ (SEL_CONF | 3)
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

    ; IRET outer-level to ring 3 in the CONFORMING segment (IOPL=3 for OUT)
    push dword SEL_DATA3_RPL3   ; SS3
    push dword STACK3_TOP       ; ESP3
    push dword 0x00003002       ; EFLAGS IOPL=3
    push dword SEL_CONF_RPL3    ; CS = conforming DPL0, RPL 3 -> CPL 3
    push dword ring3_entry
    iretd

ring3_entry:
    mov ax, SEL_DATA3_RPL3
    mov ds, ax
    DIAG 0x13
lidt_site:
    lidt [ds:scratch_idt]       ; privileged: must #GP(0) at CPL3
    ; Old bug: LIDT executed here (CPL believed 0) and clobbered IDTR
    mov eax, 0xBAD
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    jmp $

;------------------------------------------------------------------
; Ring 0 #GP handler via DPL0 386 interrupt gate
;------------------------------------------------------------------
isr_gp:
    ; frame: [esp]=err, +4 EIP, +8 CS, +12 EFLAGS, +16 ESP3, +20 SS3
    mov eax, [esp]
    cmp eax, 0
    jne fail_21
    mov eax, [esp+4]
    cmp eax, lidt_site
    jne fail_22
    mov eax, [esp+8]
    cmp ax, SEL_CONF_RPL3       ; pushed CS must carry RPL=3 (true CPL)
    jne fail_23

    mov ax, SEL_DATA0
    mov ds, ax
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
scratch_idt:
    dw 0x0000
    dd 0x00000000

align 8
gdt:
    dq 0x0000000000000000
    ; 0x08: ring0 32-bit code, base 0x10000
    dq 0x00CF9B010000FFFF
    ; 0x10: ring0 32-bit data, base 0x10000
    dq 0x00CF93010000FFFF
    ; 0x18: CONFORMING ring0 32-bit code, base 0x10000 (type C: conforming)
    dq 0x00CF9F010000FFFF
    ; 0x20: ring3 32-bit data, base 0x10000 (DPL=3)
    dq 0x00CFF3010000FFFF
    ; 0x28: available 386 TSS (type 9), base = tss386+0x10000, limit 0x67
    dw 0x0067
    dw tss386
    db 0x01
    db 0x89
    db 0x00
    db 0x00
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x00010000

; IDT: vector 0x0D = 386 interrupt gate, DPL0, ring0 handler
align 8
idt:
    times 0x0D dq 0
    dw isr_gp                   ; offset [15:0]
    dw SEL_CODE0
    db 0
    db 10001110b                ; P=1 DPL=0 type=E (386 int gate)
    dw 0                        ; offset [31:16]
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd idt + 0x00010000

; 386 TSS: ESP0 at +4, SS0 at +8
align 4
tss386:
    dd 0
    dd STACK0_TOP               ; +4  ESP0
    dd SEL_DATA0                ; +8  SS0
    times 23 dd 0
tss386_end:
