; call_gate.asm - Test call gate functionality in protected mode
;
; Tests same-privilege (ring 0) call gate:
;   1) Real mode: LGDT, set PE=1, far JMP to PM
;   2) Load data/stack segments
;   3) Far CALL through a 386 call gate (SEL_GATE → SEL_CODE32:gate_handler)
;   4) Handler: verify CS, check return address on stack, set EBX marker
;   5) RETF back to caller
;   6) Verify EBX marker, report pass
;
; Also tests call gate with parameters (DWORD count > 0):
;   7) Push test values on stack
;   8) Far CALL through gate with 2 parameters
;   9) Handler: verify parameters copied to new stack frame
;  10) RETF 8 to clean up parameters
;
; Result protocol:
;   Port 0xE0: status (0x01 = pass, 0xFF = fail)
;   Port 0xE4: failure code

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

; GDT selectors
SEL_CODE32  equ 0x08
SEL_DATA32  equ 0x10
SEL_GATE    equ 0x18       ; call gate, 0 params
SEL_GATE2   equ 0x20       ; call gate, 2 DWORD params

start:
    cli
    lgdt [cs:gdt_desc]

    ; Enter protected mode
    mov eax, cr0
    or  eax, 1
    mov cr0, eax

    ; Far JMP to load CS from GDT
    db 0x66, 0xEA           ; jmp far ptr16:32
    dd pm32_entry
    dw SEL_CODE32

BITS 32
pm32_entry:
    ; Load data/stack segments
    mov ax, SEL_DATA32
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov esp, 0x2000         ; Stack in data area

    ;------------------------------------------------------------------
    ; TEST 1: Simple call gate (no parameters)
    ;------------------------------------------------------------------
    xor ebx, ebx            ; EBX = 0 (not visited yet)

    ; Far CALL through call gate. Offset is ignored for gates.
    db 0x9A                  ; CALL FAR ptr16:32
    dd 0x00000000            ;   offset (ignored)
    dw SEL_GATE              ;   selector = call gate

    ; After return: EBX should be set by handler
    cmp ebx, 0xCAFEBABE
    jne fail_02

    ;------------------------------------------------------------------
    ; TEST 2: Call gate with 2 DWORD parameters
    ;------------------------------------------------------------------
    push dword 0xDEADBEEF   ; param 2 (pushed first = higher address)
    push dword 0x12345678   ; param 1 (pushed last = lower address)

    db 0x9A                  ; CALL FAR ptr16:32
    dd 0x00000000
    dw SEL_GATE2             ; call gate with 2 params

    ; After RETF 8, the two pushed DWORDs should be cleaned
    ; ESP should be back to pre-push value
    cmp esp, 0x2000
    jne fail_06

    ; EBX should be updated by handler
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
; Gate handler (no parameters)
;==================================================================
gate_handler:
    ; Verify CS = target from gate descriptor
    mov ax, cs
    cmp ax, SEL_CODE32
    jne fail_03

    ; Same-privilege call gate pushes: [return_EIP, return_CS]
    ; Verify return CS on stack
    mov eax, [esp+4]        ; return CS (zero-extended to 32)
    cmp ax, SEL_CODE32
    jne fail_05

    ; Verify return EIP is reasonable (nonzero, within code segment)
    mov eax, [esp]
    test eax, eax
    jz  fail_04

    ; Mark successful entry
    mov ebx, 0xCAFEBABE
    retf

;==================================================================
; Gate handler (2 DWORD parameters)
;==================================================================
gate_handler_params:
    ; Same-privilege call gate with N params pushes:
    ;   [param1, param2, ..., paramN, return_EIP, return_CS]
    ; Since this is same-privilege, params are just on the same stack.
    ; Actually for same-privilege, no stack switch occurs and no params
    ; are copied. The params remain where the caller pushed them.
    ; Stack layout (same privilege):
    ;   [esp+0] = return EIP
    ;   [esp+4] = return CS
    ; Caller's params are below the return frame at [esp+8], [esp+12].

    ; Verify return CS
    mov eax, [esp+4]
    cmp ax, SEL_CODE32
    jne fail_08

    ; Read caller's params (below the return frame)
    mov eax, [esp+8]        ; param 1 (last pushed = lowest addr)
    cmp eax, 0x12345678
    jne fail_09

    mov eax, [esp+12]       ; param 2 (first pushed = highest addr)
    cmp eax, 0xDEADBEEF
    jne fail_0A

    mov ebx, 0x0BADF00D
    retf 8                   ; return and pop 8 bytes of parameters

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
    mov eax, 0x04           ; return EIP is zero
    jmp fail
fail_05:
    mov eax, 0x05           ; return CS wrong on stack
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

    ; Entry 1 (sel=0x08): 32-bit code, base=0x10000, limit=4GB
    ;   Base=00010000, Limit=FFFFF, G=1, D/B=1, P=1, DPL=0, S=1, Type=A (code rx)
    dq 0x00CF9B010000FFFF

    ; Entry 2 (sel=0x10): 32-bit data, base=0x10000, limit=4GB
    ;   Base=00010000, Limit=FFFFF, G=1, D/B=1, P=1, DPL=0, S=1, Type=3 (data rw, A=1)
    dq 0x00CF93010000FFFF

    ; Entry 3 (sel=0x18): 386 call gate, 0 params
    ;   Target: SEL_CODE32 : gate_handler
    ;   P=1, DPL=0, S=0, Type=0xC (386 call gate), Param count=0
    dw gate_handler             ; Offset[15:0]  (code is < 64KB)
    dw SEL_CODE32               ; Target CS selector
    db 0                        ; Param count = 0
    db 10001100b                ; P=1, DPL=00, S=0, Type=1100
    dw 0                        ; Offset[31:16] (always 0, code < 64KB)

    ; Entry 4 (sel=0x20): 386 call gate, 2 DWORD params
    ;   Target: SEL_CODE32 : gate_handler_params
    dw gate_handler_params      ; Offset[15:0]
    dw SEL_CODE32
    db 2                        ; Param count = 2
    db 10001100b                ; P=1, DPL=00, S=0, Type=1100
    dw 0                        ; Offset[31:16]
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1       ; GDT limit
    dd gdt + 0x00010000         ; GDT base (code loaded at phys 0x10000)
