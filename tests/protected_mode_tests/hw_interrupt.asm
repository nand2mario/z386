; hw_interrupt.asm - Hardware interrupt (INTR/NMI) test
;
; Tests:
;   1. Set up real-mode IVT entries for INT 0x20 (INTR vector) and INT 2 (NMI)
;   2. Enable interrupts (STI)
;   3. Wait for INTR — testbench asserts intr after some cycles, responds
;      to two INTA bus cycles with vector 0x20
;   4. ISR 0x20 increments a counter and does IRET
;   5. After IRET, verify counter was incremented
;   6. Wait for NMI — testbench pulses nmi
;   7. NMI handler increments a different counter and does IRET
;   8. Verify NMI counter, report PASS
;
; Execution starts from 0x10000 in real mode. CS=0x1000, EIP=0x0
; IVT is at physical 0x00000 (first 1KB of memory)
;
; Memory layout:
;   0x00000 - IVT (256 * 4 = 1024 bytes)
;   0x10000 - Code (CS:0 -> physical 0x10000)
;   0x10000 + data_area - Counters and flags

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

; Ports for testbench communication
SIGNAL_PORT equ 0xE8    ; Write to signal testbench (1=ready for INTR, 2=ready for NMI)

; Interrupt vector numbers
INTR_VECTOR equ 0x20    ; Vector that PIC will send for INTR
NMI_VECTOR  equ 0x02    ; NMI is always vector 2

start:
    cli
    cld

    ; Set DS = CS so data references and CS: overrides use same segment
    mov ax, cs
    mov ds, ax

    ; ----- Set up IVT entries -----
    ; IVT is at linear address 0. In real mode, IVT entry = [offset16, segment16]
    ; We need ES=0 to write to IVT
    xor ax, ax
    mov es, ax

    ; Set up INT 0x20 handler (INTR)
    ; IVT[0x20] at address 0x80 = 0x20 * 4
    mov word [es:INTR_VECTOR*4], isr_intr   ; offset
    mov word [es:INTR_VECTOR*4+2], 0x1000   ; segment (CS=0x1000)

    ; Set up INT 2 handler (NMI)
    ; IVT[2] at address 0x08 = 2 * 4
    mov word [es:NMI_VECTOR*4], isr_nmi     ; offset
    mov word [es:NMI_VECTOR*4+2], 0x1000    ; segment (CS=0x1000)

    ; ----- Clear counters -----
    mov word [intr_count], 0
    mov word [nmi_count], 0

    ; ----- Test 1: INTR -----
    ; Enable interrupts and signal testbench we're ready
    sti
    mov al, 1               ; Signal: ready for INTR
    mov dx, SIGNAL_PORT
    out dx, al

    ; Spin-wait for INTR handler to run (up to ~1000 iterations)
    mov cx, 10000
.wait_intr:
    cmp word [intr_count], 1
    je .intr_ok
    dec cx
    jnz .wait_intr

    ; Timeout waiting for INTR
    mov eax, 0x00000001     ; fail code 1: INTR timeout
    jmp fail

.intr_ok:
    ; Verify INTR counter == 1
    cmp word [intr_count], 1
    jne fail_intr_count

    ; Verify interrupts are still enabled after IRET
    pushf
    pop ax
    test ax, 0x0200         ; IF bit
    jz fail_if_cleared

    ; ----- Test 2: NMI -----
    ; Signal testbench we're ready for NMI
    mov al, 2               ; Signal: ready for NMI
    mov dx, SIGNAL_PORT
    out dx, al

    ; Spin-wait for NMI handler to run
    mov cx, 10000
.wait_nmi:
    cmp word [nmi_count], 1
    je .nmi_ok
    dec cx
    jnz .wait_nmi

    ; Timeout waiting for NMI
    mov eax, 0x00000003     ; fail code 3: NMI timeout
    jmp fail

.nmi_ok:
    ; Verify NMI counter == 1
    cmp word [nmi_count], 1
    jne fail_nmi_count

    ; ----- Test 3: INTR masked by CLI -----
    ; Disable interrupts, signal testbench to send another INTR,
    ; verify it does NOT fire
    cli
    mov al, 3               ; Signal: send INTR while masked
    mov dx, SIGNAL_PORT
    out dx, al

    ; Small delay to let the INTR arrive (it should be pending but masked)
    mov cx, 500
.delay_masked:
    dec cx
    jnz .delay_masked

    ; Verify INTR counter still 1 (masked, not serviced)
    cmp word [intr_count], 1
    jne fail_masked

    ; Now re-enable interrupts — pending INTR should fire immediately
    sti
    nop                     ; Allow one instruction for interrupt to be serviced
    nop
    nop

    ; Verify INTR counter now 2
    cmp word [intr_count], 2
    jne fail_unmasked

    ; ----- All tests passed -----
    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

; --- ISR for INTR (vector 0x20) ---
isr_intr:
    push ax
    inc word [cs:intr_count]
    pop ax
    iret

; --- ISR for NMI (vector 2) ---
isr_nmi:
    push ax
    inc word [cs:nmi_count]
    pop ax
    iret

; --- Failure paths ---
fail_intr_count:
    mov eax, 0x00000002     ; fail code 2: INTR count wrong
    jmp fail
fail_if_cleared:
    mov eax, 0x00000004     ; fail code 4: IF cleared after IRET
    jmp fail
fail_nmi_count:
    mov eax, 0x00000005     ; fail code 5: NMI count wrong
    jmp fail
fail_masked:
    mov eax, 0x00000006     ; fail code 6: INTR fired while masked
    jmp fail
fail_unmasked:
    mov eax, 0x00000007     ; fail code 7: INTR not fired after STI
    jmp fail

fail:
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

; --- Data area (in code segment) ---
align 2
intr_count: dw 0
nmi_count:  dw 0
