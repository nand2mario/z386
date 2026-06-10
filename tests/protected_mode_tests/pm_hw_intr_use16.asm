; pm_hw_intr_use16.asm - PM interrupt with USE16 code/stack segments
;
; Tests that a 32-bit interrupt gate (type 0xE) correctly handles
; USE16 code segments: the interrupt pushes 32-bit values (EIP, CS,
; EFLAGS) but the handler runs in 16-bit mode. After IRET, the
; original operand size must be restored so PUSHA pushes 16-bit
; registers (16 bytes), not 32-bit (32 bytes).
;
; Bug reproduced: z386's interrupt dispatch sets op_size=DWORD (BITS32)
; for the 32-bit gate pushes. After returning to the handler code,
; op_size remained DWORD, causing PUSHA to push 32-bit registers (32
; bytes instead of 16), corrupting the stack frame.

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

; USE16 segments: D=0, byte granular
SEL_CODE16 equ 0x08    ; 16-bit code segment
SEL_DATA16 equ 0x10    ; 16-bit data segment

PM_STACK_TOP equ 0x4000

start:
    cli
    cld

    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]

    mov eax, cr0
    or  eax, 1
    mov cr0, eax

    ; Far jump to 16-bit PM code (USE16 segment)
    db 0xEA              ; JMP FAR
    dw pm16_entry        ; 16-bit offset
    dw SEL_CODE16        ; selector

pm16_entry:
    mov ax, SEL_DATA16
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov sp, PM_STACK_TOP

    ; Clear test variables
    mov word [intr_count], 0
    mov word [pusha_delta], 0

    ; Configure testbench: vector 0x20, deliver after 1 instruction
    mov dx, SIGNAL_VECTOR_PORT
    mov al, INTR_VECTOR
    out dx, al

    mov dx, SIGNAL_INSTR_PORT
    mov ax, 1
    out dx, ax

    mov dx, SIGNAL_CYCLES_PORT
    mov ax, 1
    out dx, ax

    ; Enable interrupts
    sti
    nop

    ; Signal testbench to assert INTR
    mov dx, SIGNAL_PORT
    mov al, 1
    out dx, al

    ; Target instruction: the interrupt should fire around here.
    ; After the ISR returns, execution continues here.
    nop
    nop
    nop

    ; Wait for interrupt to be serviced
    mov cx, 0
.wait_intr:
    cmp word [intr_count], 1
    je .intr_ok
    dec cx
    jnz .wait_intr

    ; Timeout
    mov eax, 0x00000001
    jmp fail

.intr_ok:
    ; Check PUSHA delta: should be 16 (8 regs * 2 bytes) for USE16
    cmp word [pusha_delta], 16
    je .pusha_ok

    ; PUSHA pushed wrong amount (likely 32 = PUSHAD)
    mov eax, 0x00000002     ; PUSHA size mismatch
    movzx eax, word [pusha_delta]
    or eax, 0x00020000      ; error code 2 in upper word
    jmp fail

.pusha_ok:
    ; Check that IF is restored after IRETD
    pushf
    pop ax
    test ax, 0x0200
    jz .fail_if

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

.fail_if:
    mov eax, 0x00000003     ; IF not restored
    jmp fail

; ISR for INTR — entered via 32-bit interrupt gate in USE16 code
; Stack on entry (32-bit gate pushes):
;   [SP+0] = EIP (32-bit)
;   [SP+4] = CS  (32-bit, padded)
;   [SP+8] = EFLAGS (32-bit)
;
; This handler does PUSHA and checks how many bytes SP decreased.
; In USE16 mode, PUSHA should push 16 bytes (8 * 16-bit registers).
; Bug: if op_size is stuck at DWORD from interrupt dispatch,
; PUSHA pushes 32 bytes (8 * 32-bit registers).
isr_intr:
    ; Save SP before PUSHA
    mov bp, sp

    pusha       ; Should push 16 bytes in USE16 mode

    ; Calculate delta = BP - SP (should be 16)
    mov ax, bp
    sub ax, sp
    mov [pusha_delta], ax

    popa        ; Restore registers

    inc word [intr_count]

    ; 32-bit IRET to match 32-bit gate pushes
    db 0x66     ; operand size prefix
    iret        ; IRETD — pop 32-bit EIP, CS, EFLAGS

fail:
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

align 8
gdt:
    dq 0x0000000000000000       ; null descriptor
    ; SEL_CODE16 (0x08): 16-bit code, base=0x10000, limit=0xFFFF, D=0
    dw 0xFFFF                   ; limit [15:0]
    dw 0x0000                   ; base [15:0]
    db 0x01                     ; base [23:16]
    db 10011011b                ; P=1, DPL=0, S=1, type=code+read+accessed
    db 00000000b                ; G=0, D=0, limit [19:16]=0
    db 0x00                     ; base [31:24]
    ; SEL_DATA16 (0x10): 16-bit data, base=0x10000, limit=0xFFFF, D=0
    dw 0xFFFF                   ; limit [15:0]
    dw 0x0000                   ; base [15:0]
    db 0x01                     ; base [23:16]
    db 10010011b                ; P=1, DPL=0, S=1, type=data+write+accessed
    db 00000000b                ; G=0, D=0, limit [19:16]=0
    db 0x00                     ; base [31:24]
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x00010000         ; linear address of GDT

align 8
idt:
    times INTR_VECTOR dq 0      ; vectors 0-0x1F empty
    ; Vector 0x20: 32-bit interrupt gate (type 0xE)
    ; Target: isr_intr in SEL_CODE16
    dw isr_intr                 ; offset [15:0]
    dw SEL_CODE16               ; selector
    db 0                        ; reserved
    db 10001110b                ; P=1, DPL=0, type=0xE (386 int gate)
    dw 0                        ; offset [31:16] = 0 (USE16 handler)
    times (256 - INTR_VECTOR - 1) dq 0
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd idt + 0x00010000         ; linear address of IDT

align 4
intr_count:   dw 0
pusha_delta:  dw 0
