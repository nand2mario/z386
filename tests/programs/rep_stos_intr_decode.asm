; rep_stos_intr_decode.asm - IRQ during REP STOSB must restart cleanly
;
; This stresses the RPTI path for byte streams seen in DOS/32A/FastDoom:
;   F3 AA B8 33 00 00 00 C7 ...
;
; The IRQ is asserted while REP STOSB is active. The current core services it
; after the repeat drains, but the pending interrupt still exercises RPTI
; restarts and the subsequent B8/C7 decode.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
SIGNAL_PORT equ 0xE8
SIGNAL_CYCLES_PORT equ 0xEC
SIGNAL_INSTR_PORT  equ 0xF0
SIGNAL_VECTOR_PORT equ 0xF4

STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

SEL_CODE0   equ 0x08
SEL_DATA0   equ 0x10
INTR_VECTOR equ 0x20
REP_COUNT   equ 64

start:
    cli
    cld

    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]

    mov eax, cr0
    or  eax, 1
    mov cr0, eax

    db 0x66, 0xEA
    dd pm32_entry
    dw SEL_CODE0

BITS 32
pm32_entry:
    mov ax, SEL_DATA0
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov esp, stack_top

    mov dword [intr_count], 0
    mov dword [intr_ecx], 0
    mov dword [store_target], 0
    mov byte [buffer], 0
    mov byte [buffer + REP_COUNT - 1], 0

    mov dx, SIGNAL_VECTOR_PORT
    mov al, INTR_VECTOR
    out dx, al

    mov dx, SIGNAL_INSTR_PORT
    xor ax, ax
    out dx, ax

    mov dx, SIGNAL_CYCLES_PORT
    mov ax, 20
    out dx, ax

    mov ecx, REP_COUNT
    mov edi, buffer
    mov al, 0x5A

    sti
    mov dx, SIGNAL_PORT
    mov al, 1
    out dx, al
    mov al, 0x5A

case_bytes:
    db 0xF3, 0xAA                   ; rep stosb
    db 0xB8, 0x33, 0x00, 0x00, 0x00 ; mov eax, 0x33
    db 0xC7, 0x05
    dd store_target
    dd 0x12345678                   ; mov dword [store_target], 0x12345678

    cmp dword [intr_count], 1
    jne fail_no_irq

    cmp dword [intr_ecx], REP_COUNT
    jae fail_irq_too_early

    cmp byte [buffer], 0x5A
    jne fail_stos
    cmp byte [buffer + REP_COUNT - 1], 0x5A
    jne fail_stos
    cmp eax, 0x00000033
    jne fail_eax
    cmp dword [store_target], 0x12345678
    jne fail_store

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

fail_no_irq:
    mov eax, 0x00000001
    jmp fail
fail_irq_too_early:
    mov eax, 0x00000003
    jmp fail
fail_stos:
    mov eax, 0x00000004
    jmp fail
fail_eax:
    mov eax, 0x00000005
    jmp fail
fail_store:
    mov eax, 0x00000006
    jmp fail

ud_handler:
    mov eax, 0x00000600
    jmp fail

gp_handler:
    mov eax, 0x00000D00
    jmp fail

irq_handler:
    inc dword [intr_count]
    mov [intr_ecx], ecx
    iretd

fail:
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

BITS 16
align 8
gdt:
    dq 0x0000000000000000   ; null
    dq 0x00CF9B010000FFFF   ; ring 0 code, base=0x10000, 4GB
    dq 0x00CF93010000FFFF   ; ring 0 data, base=0x10000, 4GB
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x00010000

align 8
idt:
    times 6 dq 0
    ; #UD
    dw ud_handler
    dw SEL_CODE0
    db 0
    db 10001110b
    dw 0
    times (13 - 7) dq 0
    ; #GP
    dw gp_handler
    dw SEL_CODE0
    db 0
    db 10001110b
    dw 0
    times (INTR_VECTOR - 14) dq 0
    ; INTR_VECTOR
    dw irq_handler
    dw SEL_CODE0
    db 0
    db 10001110b
    dw 0
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd idt + 0x00010000

BITS 32
align 4
intr_count: dd 0
intr_ecx: dd 0
store_target: dd 0
buffer:
    times REP_COUNT db 0

align 4
stack:
    times 512 db 0
stack_top:
