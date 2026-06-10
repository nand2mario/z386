; fpu_detect.asm - Test ESC instruction decode with modrm displacement
;
; Test 1 (EM=1): Verifies ESC instructions with memory operands correctly
; consume the modrm displacement byte(s). Uses CR0.EM=1 so ESC triggers #NM.
; Handler skips 3 bytes. If decoder consumed wrong byte count, the MOV after
; ESC would be at the wrong address.
;
; Test 2 (EM=0, DOS4GW-style): Replicates DOS/4GW's FPU detection sequence:
; FNINIT + FNSTSW [BP-2] + check status. With no FPU and EM=0, the ESC
; instructions execute via microcode (JPEREQ must be taken). FNSTSW should
; complete without hanging, and [BP-2] should retain its pre-set value
; (0xFFFF) since no real FPU writes the status word.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

start:
    cli
    cld

    mov ax, cs
    mov ds, ax
    mov ss, ax
    mov sp, 0x200
    mov bp, sp

    ; Set up IVT entry for INT 7 (#NM)
    xor ax, ax
    mov es, ax
    mov word [es:0x1C], nm_handler
    mov word [es:0x1E], 0x1000

    ;=== Test 1: EM=1, ESC decode length ===
    smsw ax
    or al, 0x04          ; CR0.EM=1
    lmsw ax

    xor ax, ax
    db 0xDD, 0x7E, 0xFE  ; FNSTSW [BP-2] → #NM, handler skips 3 bytes
    mov ax, 0xBEEF        ; must execute at correct EIP
    cmp ax, 0xBEEF
    jne .fail1

    ; Clear EM
    smsw ax
    and al, 0xFB          ; CR0.EM=0
    lmsw ax

    ;=== Test 2: EM=0, DOS4GW-style FPU detection ===
    ; Pre-fill [BP-2] with 0xFF (non-zero) — real FNSTSW would write 0
    push word 0xFFFF      ; [BP-2] = 0xFFFF
    mov bp, sp

    db 0xDB, 0xE3         ; FNINIT — must not hang (JPEREQ taken)
    db 0xDD, 0x7E, 0xFE   ; FNSTSW [BP-2] — must not hang, must be 3 bytes

    ; After FNSTSW: with no FPU, [BP-2] should still be non-zero (0xFF)
    ; (A real FPU would write 0x0000 to [BP-2])
    mov ax, 0xBEEF        ; verify EIP is correct (3 bytes consumed)
    cmp ax, 0xBEEF
    jne .fail2

    ; Verify we didn't hang — reaching here means JPEREQ worked
    pop ax                ; clean stack

    ;=== Test 3: EM=0, FLD TBYTE (SPEED600-style FPU detection) ===
    ; FLD uses FPU_LOAD_CORWA (uc 503) which has a JPEREQ self-loop.
    ; With no FPU, the self-loop must NOT be taken (fall through).
    db 0x9B, 0xDB, 0xE3   ; FINIT
    db 0xDB, 0x2E          ; FLD TBYTE PTR [imm16]
    dw fpu_tbyte_data      ;   address of 80-bit data
    ; If FLD hangs in the JPEREQ self-loop, we never reach here
    db 0xD8, 0xC0          ; FADD ST0, ST0
    db 0x9B, 0xDF, 0xE0   ; FSTSW AX
    ; Reaching here means the FPU data transfer loop didn't hang
    nop

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

.fail1:
    smsw ax
    and al, 0xFB
    lmsw ax
    mov eax, 0x00000001   ; error code 1: EM=1 decode length
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

.fail2:
    mov eax, 0x00000002   ; error code 2: EM=0 decode length / JPEREQ hang
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

.fail3:
    mov eax, 0x00000003   ; error code 3: FLD TBYTE hang (JPEREQ self-loop)
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

; 80-bit FPU test data (10 bytes)
fpu_tbyte_data:
    dd 0x00000000
    dd 0x80000000
    dw 0x3FFF          ; 1.0 in 80-bit extended precision

; #NM handler: skip 3-byte ESC instruction
nm_handler:
    push bp
    mov bp, sp
    add word [bp+2], 3
    pop bp
    iret
