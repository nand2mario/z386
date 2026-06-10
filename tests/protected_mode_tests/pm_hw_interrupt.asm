; pm_hw_interrupt.asm - Protected-mode external INTR via IDT interrupt gate
;
; Test:
;   1) Enter protected mode (ring 0), load GDT and IDT
;   2) Install vector 0x20 as a 32-bit interrupt gate
;   3) Configure testbench to assert INTR after one retired instruction
;   4) Verify ISR executes, returns with IRETD, and IF is restored

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

INTR_VECTOR equ 0x20

SEL_CODE0 equ 0x08
SEL_DATA0 equ 0x10

PM_STACK_TOP equ 0x9000

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
    mov esp, PM_STACK_TOP

    mov dword [intr_count], 0
    mov dword [marker], 0
    mov dword [intr_seen_marker], 0
    mov dword [intr_seen_cs], 0

    mov dx, SIGNAL_VECTOR_PORT
    mov al, INTR_VECTOR
    out dx, al

    mov dx, SIGNAL_INSTR_PORT
    mov ax, 1
    out dx, ax

    mov dx, SIGNAL_CYCLES_PORT
    mov ax, 1
    out dx, ax

    ; Enable interrupts with no shadow in flight.
    sti
    nop

    ; Request INTR.
    mov dx, SIGNAL_PORT
    mov al, 1
    out dx, al

    ; The ISR should observe marker already written.
    mov dword [marker], 0xA5A55A5A
    nop

    mov ecx, 200000
.wait_intr:
    cmp dword [intr_count], 1
    je .intr_ok
    dec ecx
    jnz .wait_intr

    mov eax, 0x00000001     ; timeout waiting for INTR
    jmp fail

.intr_ok:
    cmp dword [intr_seen_marker], 0xA5A55A5A
    jne .fail_marker

    mov ax, [intr_seen_cs]
    cmp ax, SEL_CODE0
    jne .fail_cs

    pushfd
    pop eax
    test eax, 0x00000200
    jz .fail_if

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

.fail_marker:
    mov eax, 0x00000002     ; ISR ran before marker write
    jmp fail
.fail_cs:
    mov eax, 0x00000003     ; ISR CS mismatch
    jmp fail
.fail_if:
    mov eax, 0x00000004     ; IF not restored after IRETD
    jmp fail

isr_intr:
    push eax
    mov eax, [marker]
    mov [intr_seen_marker], eax
    xor eax, eax
    mov ax, cs
    mov [intr_seen_cs], eax
    inc dword [intr_count]
    pop eax
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
    times INTR_VECTOR dq 0
    ; 32-bit interrupt gate: isr_intr, selector SEL_CODE0, type=0xE, P=1, DPL=0
    dw isr_intr
    dw SEL_CODE0
    db 0
    db 10001110b
    dw 0
    times (256 - INTR_VECTOR - 1) dq 0
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd idt + 0x00010000

BITS 32
align 4
intr_count:       dd 0
marker:           dd 0
intr_seen_marker: dd 0
intr_seen_cs:     dd 0
