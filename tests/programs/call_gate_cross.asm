; call_gate_cross.asm - Test cross-privilege (ring 3 -> ring 0) call gates
;
; Tests:
;   1) Real mode: LGDT, LTR, set PE=1, far JMP to PM ring 0
;   2) Load ring 0 data/stack segments
;   3) IRET to ring 3 (push ring 3 SS:ESP, EFLAGS, CS:EIP)
;   4) In ring 3: far CALL through DPL=3 call gate targeting ring 0
;      - CPU loads SS0:ESP0 from TSS (stack switch)
;      - Pushes caller SS3:ESP3, return CS3:EIP3 on ring 0 stack
;   5) Handler: verify CS=ring 0, verify caller SS:ESP on stack, set EBX marker
;   6) RETF back to ring 3 (restores ring 3 stack)
;   7) Verify EBX marker, report pass
;
;   8) Push 2 test params, far CALL through gate with 2 DWORD params
;   9) Handler: verify params copied to ring 0 stack, set marker
;  10) RETF 8 to clean up params and restore ring 3 stack
;
; Result protocol:
;   Port 0xE0: status (0x01 = pass, 0xFF = fail)
;   Port 0xE4: diagnostic id:value pairs (high byte = id, low 24 = value)
;              Also failure code on fail

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

; GDT selectors (index * 8)
SEL_CODE0   equ 0x08       ; ring 0 code, DPL=0
SEL_DATA0   equ 0x10       ; ring 0 data, DPL=0
SEL_CODE3   equ 0x18       ; ring 3 code, DPL=3
SEL_DATA3   equ 0x20       ; ring 3 data, DPL=3
SEL_TSS     equ 0x28       ; 32-bit TSS
SEL_GATE    equ 0x30       ; call gate, DPL=3, 0 params
SEL_GATE2   equ 0x38       ; call gate, DPL=3, 2 params

; RPL=3 versions for ring 3 usage
SEL_CODE3_RPL3 equ (SEL_CODE3 | 3)  ; 0x1B
SEL_DATA3_RPL3 equ (SEL_DATA3 | 3)  ; 0x23

; Stack addresses (offsets from segment base 0x10000)
STACK0_TOP  equ 0x3000     ; ring 0 stack top (in TSS)
STACK3_TOP  equ 0x4000     ; ring 3 stack top

; Diagnostic checkpoint IDs (high byte of DATA_PORT output)
;   0x01 = CS after PM entry
;   0x02 = DS after segment load
;   0x03 = SS after segment load
;   0x04 = ESP after segment load
;   0x05 = TR after LTR
;   0x10 = CS in ring 3 after IRET
;   0x11 = SS in ring 3 after IRET
;   0x12 = ESP in ring 3 after IRET
;   0x13 = DS in ring 3 after IRET
;   0x20 = CS in gate_handler
;   0x21 = ESP in gate_handler (raw)
;   0x22 = [esp+0] return EIP
;   0x23 = [esp+4] return CS
;   0x24 = [esp+8] caller ESP3
;   0x25 = [esp+12] caller SS3
;   0x30 = CS after RETF (back in ring 3)
;   0x31 = SS after RETF
;   0x32 = ESP after RETF
;   0x33 = EBX after RETF
;   0x40 = CS in gate_handler_params
;   0x41 = [esp+4] return CS (params)
;   0x42 = [esp+8] param 1
;   0x43 = [esp+12] param 2
;   0x44 = [esp+16] caller ESP3 (params)
;   0x45 = [esp+20] caller SS3 (params)

; Macro: output diagnostic (id << 24) | (reg_value & 0xFFFFFF)
; Clobbers EAX, EDX
%macro DIAG 2     ; %1 = id (immediate), %2 = 32-bit value (register or memory)
    mov eax, %2
    and eax, 0x00FFFFFF
    or  eax, (%1 << 24)
    mov dx, DATA_PORT
    out dx, eax
%endmacro

; Macro: output diagnostic for segment register
; Clobbers EAX, EDX
%macro DIAG_SEG 2     ; %1 = id, %2 = segment register
    xor eax, eax
    mov ax, %2
    or  eax, (%1 << 24)
    mov dx, DATA_PORT
    out dx, eax
%endmacro

start:
    cli
    lgdt [cs:gdt_desc]

    ; Enter protected mode
    mov eax, cr0
    or  eax, 1
    mov cr0, eax

    ; Far JMP to load CS from GDT (ring 0)
    db 0x66, 0xEA           ; jmp far ptr16:32
    dd pm32_entry
    dw SEL_CODE0

BITS 32
pm32_entry:
    ; Load ring 0 data/stack segments
    mov ax, SEL_DATA0
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov esp, STACK0_TOP

    ; Diagnostic: ring 0 segments after load
    DIAG_SEG 0x01, cs
    DIAG_SEG 0x02, ds
    DIAG_SEG 0x03, ss
    DIAG     0x04, esp

    ; Load Task Register (must be done in ring 0)
    mov ax, SEL_TSS
    ltr ax

    DIAG_SEG 0x05, ax       ; TR selector we loaded

    ;------------------------------------------------------------------
    ; Switch to ring 3 via IRET
    ; Stack frame for IRET to ring 3: SS3, ESP3, EFLAGS, CS3, EIP3
    ;------------------------------------------------------------------
    push dword SEL_DATA3_RPL3   ; SS3
    push dword STACK3_TOP       ; ESP3
    push dword 0x3002           ; EFLAGS: IF=0, IOPL=3 (bits 13:12 = 11)
    push dword SEL_CODE3_RPL3   ; CS3
    push dword ring3_entry      ; EIP3
    iretd

ring3_entry:
    ; Now in ring 3! IOPL=3 allows IO from ring 3.
    ; Load ring 3 data segments
    mov ax, SEL_DATA3_RPL3
    mov ds, ax
    mov es, ax

    ; Diagnostic: ring 3 state after IRET
    DIAG_SEG 0x10, cs
    DIAG_SEG 0x11, ss
    DIAG     0x12, esp
    DIAG_SEG 0x13, ds

    ;------------------------------------------------------------------
    ; TEST 1: Simple cross-privilege call gate (no parameters)
    ;------------------------------------------------------------------
    xor ebx, ebx            ; EBX = 0 (not visited yet)

    ; Far CALL through call gate (DPL=3, so callable from ring 3)
    ; CPU will: load SS0:ESP0 from TSS, push SS3:ESP3, push CS3:EIP3
    db 0x9A                  ; CALL FAR ptr16:32
    dd 0x00000000            ;   offset (ignored for gates)
    dw SEL_GATE              ;   selector = call gate

    ; After return: back in ring 3, EBX should be set by handler
    DIAG_SEG 0x30, cs
    DIAG_SEG 0x31, ss
    DIAG     0x32, esp
    DIAG     0x33, ebx

    cmp ebx, 0xCAFEBABE
    jne fail_02

    ;------------------------------------------------------------------
    ; TEST 2: Cross-privilege call gate with 2 DWORD parameters
    ;------------------------------------------------------------------
    push dword 0xDEADBEEF   ; param 2
    push dword 0x12345678   ; param 1

    db 0x9A                  ; CALL FAR ptr16:32
    dd 0x00000000
    dw SEL_GATE2             ; call gate with 2 params

    ; After RETF 8: params cleaned, ESP should be restored
    ; Note: ring 3 ESP should be back to STACK3_TOP
    cmp esp, STACK3_TOP
    jne fail_06

    cmp ebx, 0x0BADF00D
    jne fail_07

    ;------------------------------------------------------------------
    ; PASS
    ;------------------------------------------------------------------
    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt

;==================================================================
; Gate handler (no parameters) - runs in ring 0
;==================================================================
gate_handler:
    ; Diagnostic: dump ring 0 state on entry
    DIAG_SEG 0x20, cs
    DIAG     0x21, esp

    ; Cross-privilege call gate stack layout (ring 0 stack):
    ;   [esp+0]  = return EIP
    ;   [esp+4]  = return CS (with RPL=3)
    ;   [esp+8]  = caller ESP3
    ;   [esp+12] = caller SS3
    mov eax, [esp+0]
    DIAG 0x22, eax
    mov eax, [esp+4]
    DIAG 0x23, eax
    mov eax, [esp+8]
    DIAG 0x24, eax
    mov eax, [esp+12]
    DIAG 0x25, eax

    ; Verify we're in ring 0 (CS should be ring 0 code selector)
    mov ax, cs
    cmp ax, SEL_CODE0
    jne fail_03

    ; Verify return CS has RPL=3 (came from ring 3)
    mov eax, [esp+4]
    cmp ax, SEL_CODE3_RPL3
    jne fail_05

    ; Verify caller SS3
    mov eax, [esp+12]
    cmp ax, SEL_DATA3_RPL3
    jne fail_04

    ; Verify caller ESP3 is reasonable (should be STACK3_TOP)
    mov eax, [esp+8]
    cmp eax, STACK3_TOP
    jne fail_04

    ; Mark successful entry
    mov ebx, 0xCAFEBABE
    retf

;==================================================================
; Gate handler (2 DWORD parameters) - runs in ring 0
;==================================================================
gate_handler_params:
    ; Diagnostic: dump ring 0 state on entry
    DIAG_SEG 0x40, cs
    DIAG     0x41, esp

    ; Cross-privilege call gate with N params stack layout (ring 0 stack):
    ;   [esp+0]  = return EIP
    ;   [esp+4]  = return CS (RPL=3)
    ;   [esp+8]  = param 1 (copied from ring 3 stack)
    ;   [esp+12] = param 2 (copied from ring 3 stack)
    ;   [esp+16] = caller ESP3
    ;   [esp+20] = caller SS3
    mov eax, [esp+4]
    DIAG 0x42, eax
    mov eax, [esp+8]
    DIAG 0x43, eax
    mov eax, [esp+12]
    DIAG 0x44, eax
    mov eax, [esp+16]
    DIAG 0x45, eax
    mov eax, [esp+20]
    DIAG 0x46, eax

    ; Verify return CS
    mov eax, [esp+4]
    cmp ax, SEL_CODE3_RPL3
    jne fail_08

    ; Verify param 1 was copied from ring 3 stack
    mov eax, [esp+8]
    cmp eax, 0x12345678
    jne fail_09

    ; Verify param 2 was copied
    mov eax, [esp+12]
    cmp eax, 0xDEADBEEF
    jne fail_0A

    ; Verify caller SS3
    mov eax, [esp+20]
    cmp ax, SEL_DATA3_RPL3
    jne fail_0B

    mov ebx, 0x0BADF00D
    retf 8                   ; return and pop 8 bytes of copied parameters

;==================================================================
; Failure handlers
;==================================================================
fail_02:
    mov eax, 0x02           ; EBX not set by gate handler
    jmp fail
fail_03:
    mov eax, 0x03           ; CS wrong in gate handler
    jmp fail
fail_04:
    mov eax, 0x04           ; caller SS3/ESP3 wrong on ring 0 stack
    jmp fail
fail_05:
    mov eax, 0x05           ; return CS wrong (not RPL=3)
    jmp fail
fail_06:
    mov eax, 0x06           ; ESP not restored after RETF 8
    jmp fail
fail_07:
    mov eax, 0x07           ; EBX not set by param gate handler
    jmp fail
fail_08:
    mov eax, 0x08           ; return CS wrong in param handler
    jmp fail
fail_09:
    mov eax, 0x09           ; param 1 wrong
    jmp fail
fail_0A:
    mov eax, 0x0A           ; param 2 wrong
    jmp fail
fail_0B:
    mov eax, 0x0B           ; caller SS wrong in param handler
    jmp fail

fail:
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt

;==================================================================
; GDT
;==================================================================
BITS 16
align 8
gdt:
    ; Entry 0 (sel=0x00): NULL
    dq 0x0000000000000000

    ; Entry 1 (sel=0x08): Ring 0 code, base=0x10000, limit=4GB
    ;   G=1, D/B=1, P=1, DPL=0, S=1, Type=A (code rx)
    dq 0x00CF9B010000FFFF

    ; Entry 2 (sel=0x10): Ring 0 data, base=0x10000, limit=4GB
    ;   G=1, D/B=1, P=1, DPL=0, S=1, Type=3 (data rw, A=1)
    dq 0x00CF93010000FFFF

    ; Entry 3 (sel=0x18): Ring 3 code, base=0x10000, limit=4GB
    ;   G=1, D/B=1, P=1, DPL=3, S=1, Type=B (code rx, A=1)
    dq 0x00CFFB010000FFFF

    ; Entry 4 (sel=0x20): Ring 3 data, base=0x10000, limit=4GB
    ;   G=1, D/B=1, P=1, DPL=3, S=1, Type=3 (data rw, A=1)
    dq 0x00CFF3010000FFFF

    ; Entry 5 (sel=0x28): 32-bit TSS, base=phys addr of tss, limit=0x67
    ;   P=1, DPL=0, S=0, Type=9 (32-bit TSS available)
    ;   Physical base = 0x10000 + tss (code loaded at 0x10000)
    ;   We set base[15:0] using NASM label arithmetic (tss + 0x10000 fits in 17 bits)
    ;   and base[23:16] = 0x01 (since 0x10000 + offset < 0x20000), base[31:24] = 0x00
tss_desc:
    dw 0x0067                           ; Limit[15:0] = 103
    dw tss                              ; Base[15:0] (low 16 bits of tss offset)
    db 0x01                             ; Base[23:16] = 0x01 (phys = 0x0001xxxx)
    db 10001001b                        ; P=1, DPL=00, S=0, Type=1001
    db 0x00                             ; Limit[19:16]=0, G=0
    db 0x00                             ; Base[31:24] = 0x00

    ; Entry 6 (sel=0x30): 386 call gate, DPL=3, 0 params
    ;   Target: SEL_CODE0 : gate_handler
    ;   P=1, DPL=3, S=0, Type=0xC (386 call gate), Param count=0
    dw gate_handler             ; Offset[15:0]
    dw SEL_CODE0                ; Target CS selector
    db 0                        ; Param count = 0
    db 11101100b                ; P=1, DPL=11, S=0, Type=1100
    dw 0                        ; Offset[31:16] = 0

    ; Entry 7 (sel=0x38): 386 call gate, DPL=3, 2 DWORD params
    ;   Target: SEL_CODE0 : gate_handler_params
    dw gate_handler_params      ; Offset[15:0]
    dw SEL_CODE0
    db 2                        ; Param count = 2
    db 11101100b                ; P=1, DPL=11, S=0, Type=1100
    dw 0                        ; Offset[31:16] = 0
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1       ; GDT limit
    dd gdt + 0x00010000         ; GDT base (code loaded at phys 0x10000)

;==================================================================
; TSS (Task State Segment) - minimum 104 bytes
;==================================================================
align 4
tss:
    dd 0                    ; +00: Back link
    dd STACK0_TOP           ; +04: ESP0
    dd SEL_DATA0            ; +08: SS0
    dd 0                    ; +0C: ESP1
    dd 0                    ; +10: SS1
    dd 0                    ; +14: ESP2
    dd 0                    ; +18: SS2
    dd 0                    ; +1C: CR3
    dd 0                    ; +20: EIP
    dd 0                    ; +24: EFLAGS
    dd 0                    ; +28: EAX
    dd 0                    ; +2C: ECX
    dd 0                    ; +30: EDX
    dd 0                    ; +34: EBX
    dd 0                    ; +38: ESP
    dd 0                    ; +3C: EBP
    dd 0                    ; +40: ESI
    dd 0                    ; +44: EDI
    dd 0                    ; +48: ES
    dd 0                    ; +4C: CS
    dd 0                    ; +50: SS
    dd 0                    ; +54: DS
    dd 0                    ; +58: FS
    dd 0                    ; +5C: GS
    dd 0                    ; +60: LDTR
    dw 0                    ; +64: Debug trap (T flag)
    dw 104                  ; +66: IOPB offset (= TSS limit + 1, no IOPB)
tss_end:
