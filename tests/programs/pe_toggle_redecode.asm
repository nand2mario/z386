; pe_toggle_redecode.asm - Regression for CR0.PE transition decode coherency
;
; Scenario:
;   1) Start in real mode, interrupts masked (CLI)
;   2) Set CR0.PE=1
;   3) Execute SLDT AX immediately (no far jump on purpose)
;
; Why this catches the bug:
;   - 0F 00 /0 (SLDT r/m16) is protected-mode decode.
;   - If queued bytes are not flushed/re-decoded when PE toggles, this can run
;     with stale real-mode decode and fault/hang.
;
; Pass criteria:
;   - SLDT executes (no #UD/#GP decode fault on PE transition boundary)
;   - PE can be cleared back to 0 and observed in CR0

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4

STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

start:
    cli
    cld

    ; Enter protected mode without a far jump. This is intentional: the
    ; immediately following instruction must be decoded with PE=1 semantics.
    mov eax, cr0
    or  eax, 0x00000001
    mov cr0, eax

    ; 0F 00 C0 = SLDT AX
    ; Should execute in PM (CPL0) and store current LDTR selector into AX.
    sldt ax

    mov eax, cr0
    test eax, 0x00000001
    jz fail_pe_not_set

    ; Return to real mode and verify.
    and eax, 0xFFFFFFFE
    mov cr0, eax
    mov eax, cr0
    test eax, 0x00000001
    jnz fail_pe_not_cleared

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
hang_ok:
    hlt
    jmp hang_ok

fail_pe_not_set:
    mov eax, 0x00000001
    jmp fail
fail_pe_not_cleared:
    mov eax, 0x00000002

fail:
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
hang_fail:
    hlt
    jmp hang_fail
