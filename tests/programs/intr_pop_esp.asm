; intr_pop_esp.asm - Test POP instruction ESP restoration across hardware interrupt
;
; Regression test for bug: when hardware interrupt fires at the instruction
; boundary between the previous instruction's RNI delay and POP's first cycle,
; POP's ESP write is suppressed and EIP advances past POP. IRET returns to
; the wrong instruction, permanently skipping POP and corrupting ESP.
;
; Test strategy:
;   1. Set up IVT for INT 0x20 (INTR vector)
;   2. Set ESP to known value, push a known register
;   3. Enable interrupts and signal testbench to assert INTR
;   4. Execute a tight loop of NOP + POP + check ESP
;   5. The INTR fires at some point during the loop
;   6. After ISR returns via IRET, verify ESP is correct
;
; The test passes if ESP is correctly restored after POP + interrupt.
; With the bug, ESP would be off by 2 bytes.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF
SIGNAL_PORT equ 0xE8

INTR_VECTOR equ 0x20

start:
    cli
    cld

    ; Set DS = CS
    mov ax, cs
    mov ds, ax

    ; Set up stack: SS=0, SP=0x8000
    xor ax, ax
    mov ss, ax
    mov sp, 0x8000

    ; Set up IVT for INT 0x20
    xor ax, ax
    mov es, ax
    mov word [es:INTR_VECTOR*4], isr_intr
    mov word [es:INTR_VECTOR*4+2], 0x1000

    ; Clear ISR counter
    mov word [cs:isr_count], 0

    ; Save expected ESP values
    mov word [cs:expected_sp], 0x8000

    ; ---- Test: PUSH DI / POP DI with interrupt ----
    ; Load DI with known value
    mov di, 0xBEEF

    ; Enable interrupts
    sti

    ; Signal testbench to assert INTR after short delay
    mov al, 1
    mov dx, SIGNAL_PORT
    out dx, al

    ; Execute PUSH DI + tight NOP loop + POP DI multiple times
    ; The interrupt should fire during one of these iterations
    mov cx, 200

.loop:
    push di             ; SP = 0x7FFE
    nop
    nop
    pop di              ; SP should return to 0x8000
    nop
    nop

    ; Check ESP after POP
    mov bx, sp
    cmp bx, 0x8000
    jne .esp_wrong

    ; Check DI value preserved
    cmp di, 0xBEEF
    jne .di_wrong

    dec cx
    jnz .loop

    ; After loop: verify ISR ran at least once
    cmp word [cs:isr_count], 0
    je fail_no_intr

    ; Verify final ESP
    mov bx, sp
    cmp bx, 0x8000
    jne .esp_wrong

    ; All good!
    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

.esp_wrong:
    ; ESP corruption detected!
    ; EAX = actual SP, EDX = expected SP
    movzx eax, sp
    mov edx, 0x8000
    jmp fail

.di_wrong:
    ; DI not restored correctly
    movzx eax, di
    mov edx, 0xBEEF0001
    jmp fail

fail_no_intr:
    mov eax, 0xDEAD0000
    jmp fail

fail:
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

; --- ISR for INTR ---
isr_intr:
    inc word [cs:isr_count]
    iret

; --- Data ---
align 2
isr_count: dw 0
expected_sp: dw 0
