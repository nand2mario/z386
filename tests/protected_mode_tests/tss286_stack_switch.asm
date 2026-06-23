; tss286_stack_switch.asm - ring3->ring0 interrupt stack switch with a 286 TSS
;
; Regression test for the unimplemented J16BIT microcode condition
; (ALUJMP 0x40): with a 286-format TSS (type 1/3) in TR, the privilege-change
; stack switch must read SP0 at TSS+2 and SS0 at TSS+4 (MORE_PRIV16), not the
; 386 layout ESP0@+4 / SS0@+8.  Before the fix J16BIT was never taken, the
; CPU fetched garbage SS0 and looped on #GP — this is what hung Borland/Ergo
; DPMI (TC3) under EMM386.
;
; Flow:
;   1) Real mode: LGDT, LIDT, PE=1, far JMP to PM ring 0
;   2) LTR with an available 286 TSS (type 1), SP0/SS0 at the 286 offsets
;   3) IRET to ring 3
;   4) ring 3: INT 0x40 through a DPL=3 386 interrupt gate to ring 0
;      - CPU must load SS0:SP0 from the 286 TSS slots
;   5) ISR: verify SS == SEL_DATA0 and ESP == STACK0_TOP - 20, report pass
;
; Result protocol: port 0xE0 status (0x01 pass / 0xFF fail), port 0xE4 code.

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
    DIAG 0x11               ; PM ring0 entered

    ; Load TR with the 286 TSS (type 1 = available 286 TSS)
    mov ax, SEL_TSS
    ltr ax
    DIAG 0x12               ; LTR done

    ; IRET to ring 3
    push dword SEL_DATA3_RPL3   ; SS3
    push dword STACK3_TOP       ; ESP3
    push dword 0x00003002       ; EFLAGS: IF=0, IOPL=3 (ring3 uses OUT for diags)
    push dword SEL_CODE3_RPL3   ; CS3
    push dword ring3_entry      ; EIP3
    iretd

ring3_entry:
    mov ax, SEL_DATA3_RPL3
    mov ds, ax
    DIAG 0x13               ; ring3 reached
    mov ebx, 0xC0DE0001         ; marker for the ISR
    int 0x40                    ; ring3 -> ring0 through DPL=3 386 int gate
    ; ISR reports and halts; never returns here
    jmp $

;------------------------------------------------------------------
; Ring 0 ISR — entered via the interrupt gate with the 286-TSS stack
;------------------------------------------------------------------
isr40:
    ; SS must be the ring-0 stack selector from TSS+4
    mov ax, ss
    cmp ax, SEL_DATA0
    jne fail_21
    ; ESP must be SP0 (TSS+2) minus the 5-dword inner frame (SS,ESP,EFL,CS,EIP)
    cmp esp, STACK0_TOP - 20
    jne fail_22
    ; marker survived the transition
    cmp ebx, 0xC0DE0001
    jne fail_23
    ; pushed return CS must be the ring-3 selector
    mov eax, [esp+4]
    cmp ax, SEL_CODE3_RPL3
    jne fail_24

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
    jmp fail
fail_24:
    mov eax, 0x24
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
    ; 0x08: ring0 32-bit code, base 0x10000, limit 4GB
    dq 0x00CF9B010000FFFF
    ; 0x10: ring0 32-bit data, base 0x10000, limit 4GB
    dq 0x00CF93010000FFFF
    ; 0x18: ring3 32-bit code, base 0x10000, limit 4GB (DPL=3)
    dq 0x00CFFB010000FFFF
    ; 0x20: ring3 32-bit data, base 0x10000, limit 4GB (DPL=3)
    dq 0x00CFF3010000FFFF
    ; 0x28: available 286 TSS (type 1), base = tss286+0x10000, limit 0x2B
    dw 0x002B                   ; limit
    dw (tss286 + 0x0000)        ; base [15:0]  (tss286 < 64KB; +0x10000 via byte 4)
    db 0x01                     ; base [23:16] = 0x01 (code loaded at 0x10000)
    db 0x81                     ; P=1 DPL=0 S=0 type=1 (available 286 TSS)
    db 0x00                     ; flags / limit[19:16]
    db 0x00                     ; base [31:24]
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x00010000

; IDT: 0x41 gates of 8 bytes; only vector 0x40 is populated
align 8
idt:
    times 0x40 dq 0
    ; vector 0x40: 386 interrupt gate, DPL=3, SEL_CODE0:isr40
    dw isr40                    ; offset [15:0]
    dw SEL_CODE0
    db 0
    db 11101110b                ; P=1 DPL=3 type=0xE (386 int gate)
    dw 0                        ; offset [31:16]
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd idt + 0x00010000

; 286 TSS: 16-bit fields; SP0 at +2, SS0 at +4
align 4
tss286:
    dw 0                        ; +0  back link
    dw STACK0_TOP               ; +2  SP0
    dw SEL_DATA0                ; +4  SS0
    dw 0                        ; +6  SP1
    dw 0                        ; +8  SS1  (the 386-layout SS0 slot — left 0 so
    dw 0                        ; +10 SP2   the old bug reads a null selector)
    dw 0                        ; +12 SS2
    dw 0                        ; +14 IP
    dw 0                        ; +16 FLAGS
    times 10 dw 0               ; AX..DI
    dw 0                        ; ES
    dw 0                        ; CS
    dw 0                        ; SS
    dw 0                        ; DS
    dw 0                        ; LDT
tss286_end:
