; sgdt_sidt_store.asm - SGDT/SIDT memory store correctness
;
; Regression test for two bugs found via EMM386/VCPI debugging (2026-06-12):
;   1) The SGDT/SIDT base store (microcode "WR D" = BUSOP_WR_D) issued no
;      memory request at all (missing from the mem-busop predecode set).
;   2) With a 32-bit operand, the limit store was issued as a 4-byte write
;      (architecturally always 2 bytes), clobbering the adjacent base bytes.
;
; Tests, in real mode and then in a 32-bit protected-mode code segment:
;   - LIDT with a recognizable base/limit, then SIDT to a sentinel-filled
;     buffer: verify limit, base, and that bytes past the 6-byte operand
;     are untouched.
;   - Same for SGDT against the known LGDT value.
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

SEL_CODE32  equ 0x08
SEL_DATA32  equ 0x10

IDT_TEST_BASE  equ 0x00345678
IDT_TEST_LIMIT equ 0x07FF

start:
    cli
    push cs
    pop ds                      ; DS = CS so [buf] addresses our data
    lgdt [gdt_desc]
    lidt [idt_desc]

    ;--- real mode, 16-bit operand: SIDT ---
    call fill_buf
    sidt [buf]
    mov eax, 0x01
    call check_idt_buf

    ;--- real mode, 32-bit operand: SIDT ---
    call fill_buf
    o32 sidt [buf]
    mov eax, 0x02
    call check_idt_buf

    ;--- real mode, 16-bit operand: SGDT ---
    call fill_buf
    sgdt [buf]
    mov eax, 0x03
    call check_gdt_buf

    ;--- real mode, 32-bit operand: SGDT ---
    call fill_buf
    o32 sgdt [buf]
    mov eax, 0x04
    call check_gdt_buf

    ; Enter protected mode (32-bit code segment: default operand size 32)
    mov eax, cr0
    or eax, 1
    mov cr0, eax
    jmp dword SEL_CODE32:pm_entry

;------------------------------------------------------------------
; Helpers (real mode, CS=DS=0-based via cs override on data)
;------------------------------------------------------------------
fill_buf:
    mov di, buf
    mov cx, 12
.f: mov byte [cs:di], 0xAA
    inc di
    loop .f
    ret

; eax = failure code base; clobbers bx/ecx/edx
check_idt_buf:
    mov bx, [cs:buf]            ; stored limit
    cmp bx, IDT_TEST_LIMIT
    jne fail
    mov ecx, [cs:buf+2]         ; stored base (high byte 0 by construction)
    cmp ecx, IDT_TEST_BASE
    jne fail
    jmp check_tail
check_gdt_buf:
    mov bx, [cs:buf]
    cmp bx, gdt_end - gdt - 1
    jne fail
    mov ecx, [cs:buf+2]
    cmp ecx, gdt + 0x10000
    jne fail
check_tail:                     ; bytes 6..7 must be untouched sentinels
    cmp word [cs:buf+6], 0xAAAA
    jne fail
    ret

fail:
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt

;------------------------------------------------------------------
; Protected mode, 32-bit code segment (base 0x10000)
;------------------------------------------------------------------
BITS 32
pm_entry:
    mov ax, SEL_DATA32
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov esp, 0xF000

    ;--- PM, 32-bit operand (no prefix): SIDT ---
    call pm_fill_buf
    sidt [buf]
    mov eax, 0x11
    mov bx, [buf]
    cmp bx, IDT_TEST_LIMIT
    jne pm_fail
    mov ecx, [buf+2]
    cmp ecx, IDT_TEST_BASE
    jne pm_fail
    cmp word [buf+6], 0xAAAA
    jne pm_fail

    ;--- PM, 16-bit operand (66 prefix): SIDT ---
    call pm_fill_buf
    o16 sidt [buf]
    mov eax, 0x12
    mov bx, [buf]
    cmp bx, IDT_TEST_LIMIT
    jne pm_fail
    mov ecx, [buf+2]
    cmp ecx, IDT_TEST_BASE
    jne pm_fail
    cmp word [buf+6], 0xAAAA
    jne pm_fail

    ;--- PM, 32-bit operand: SGDT ---
    call pm_fill_buf
    sgdt [buf]
    mov eax, 0x13
    mov bx, [buf]
    cmp bx, gdt_end - gdt - 1
    jne pm_fail
    mov ecx, [buf+2]
    cmp ecx, gdt + 0x10000
    jne pm_fail
    cmp word [buf+6], 0xAAAA
    jne pm_fail

    ; pass
    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt

pm_fill_buf:
    mov edi, buf
    mov ecx, 12
.f: mov byte [edi], 0xAA
    inc edi
    loop .f
    ret

pm_fail:
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt

;==================================================================
; Data
;==================================================================
align 4
buf: times 12 db 0

align 8
gdt:
    dq 0x0000000000000000
    ; sel 0x08: 32-bit code, base=0x10000, limit=4GB
    dq 0x00CF9B010000FFFF
    ; sel 0x10: 32-bit data, base=0x10000, limit=4GB
    dq 0x00CF93010000FFFF
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x00010000

idt_desc:
    dw IDT_TEST_LIMIT
    dd IDT_TEST_BASE
