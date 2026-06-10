; hlt_wakeup_intr_pm.asm - Protected-mode HLT wakes on hardware INTR

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
    mov dword [after_hlt], 0

    mov dx, SIGNAL_VECTOR_PORT
    mov al, INTR_VECTOR
    out dx, al
    mov dx, SIGNAL_INSTR_PORT
    mov ax, 0               ; cycle-delay mode
    out dx, ax
    mov dx, SIGNAL_CYCLES_PORT
    mov ax, 40
    out dx, ax

    sti
    nop

    mov dx, SIGNAL_PORT
    mov al, 1               ; request INTR
    out dx, al

    hlt
    mov dword [after_hlt], 1
    nop

    mov ecx, 200000
.wait_intr:
    cmp dword [intr_count], 1
    je .intr_ok
    dec ecx
    jnz .wait_intr
    mov eax, 0x00000001
    jmp fail

.intr_ok:
    cmp dword [after_hlt], 1
    jne .fail_resume

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

.fail_resume:
    mov eax, 0x00000002
    jmp fail

isr_intr:
    inc dword [intr_count]
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
    dq 0x0000000000000000
    dq 0x00CF9B010000FFFF
    dq 0x00CF93010000FFFF
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x00010000

align 8
idt:
    times INTR_VECTOR dq 0
    dw isr_intr
    dw SEL_CODE0
    db 0
    db 10001110b            ; 32-bit interrupt gate
    dw 0
    times (256 - INTR_VECTOR - 1) dq 0
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd idt + 0x00010000

BITS 32
align 4
intr_count: dd 0
after_hlt:  dd 0
