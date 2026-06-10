; rm_pe_transition.asm - Real mode -> protected mode initialization test
;
; This test initializes the core data structures needed for protected mode:
;   1) Build a local GDT (null, 32-bit code, 32-bit data)
;   2) Execute LGDT
;   3) Set CR0.PE=1
;   4) Far jump to protected-mode code segment
;   5) Reload data segments and stack segment from GDT selectors
;
; The z386 implementation is expected to FAIL this test until full protected-mode
; descriptor-table and segment-load support is complete.
;
; Result protocol:
;   Port 0xE0: status (0x01 = pass, 0xFF = fail)
;   Port 0xE4: failure code / debug value
;
; Execution starts from 0x10000 (64KB) in real mode.
; CS=0x1000, EIP=0x0

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4

STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

; GDT selectors
SEL_CODE32  equ 0x08
SEL_DATA32  equ 0x10

start:
    cli
    xor ax, ax
    mov dx, STATUS_PORT
    out dx, al

    ; Must start in real mode.
    mov eax, cr0
    test eax, 0x00000001
    jnz fail_01

    ; Load GDT from code segment (test image is linked at CS base).
    lgdt [cs:gdt_desc]

    ; Enter protected mode.
    mov eax, cr0
    or eax, 0x00000001
    mov cr0, eax

    ; Mandatory far jump to flush prefetch and load CS cache from GDT.
    db 0x66, 0xEA                    ; jmp far ptr16:32 (offset32, selector16)
    dd pm32_entry
    dw SEL_CODE32

fail_02:
    mov eax, 0x00000002
    jmp fail

BITS 32
pm32_entry:
    ; Verify PE really enabled post-transition.
    mov eax, cr0
    test eax, 0x00000001
    jz fail_03

    ; Load protected-mode data segments.
    mov ax, SEL_DATA32
    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax
    mov ss, ax
    mov esp, 0x00000100

    ; Basic memory read/write in PM data segment.
    mov dword [0x80], 0x12345678
    mov eax, [0x80]
    cmp eax, 0x12345678
    jne fail_04

    ; Success when PM initialization and basic access work.
    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
hang_ok:
    hlt
    jmp hang_ok

fail_01:
    mov eax, 0x00000001
    jmp fail
fail_03:
    mov eax, 0x00000003
    jmp fail
fail_04:
    mov eax, 0x00000004

fail:
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
hang_fail:
    hlt
    jmp hang_fail

BITS 16
align 8
gdt:
    dq 0x0000000000000000           ; Null descriptor
    dq 0x00CF9B010000FFFF           ; Code32: base=0x00010000, limit=4GB-1 (G=1), A=1
    dq 0x00CF93010000FFFF           ; Data32: base=0x00010000, limit=4GB-1 (G=1), A=1
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x00010000
