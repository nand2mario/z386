; stale_modrm.asm - Test for pipeline bug: stale modrm on back-to-back ALU r,r
;
; Bug: When two ALU r,r instructions execute back-to-back and the first has a
; 66h operand-size prefix (in 16-bit mode), the second instruction executes with
; stale modrm/opcode from the first. Both instructions share entry point 0x01D.
;
; Root cause: The critical bytes "66 03 EA 31 C0" must cross a 4-byte prefetch
; boundary. When "66 03 EA" occupies the last 3 bytes of one fetch and "31 C0"
; is in the next fetch, the decoder hasn't decoded the XOR by the time the ADD
; completes. The XOR's entry point loads but i_pop hasn't latched new modrm yet,
; so the microcode runs with stale modrm=0xEA from the ADD.
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

CS_SEG      equ 0x1000

start:
    cli

    ;==================================================================
    ; Test 1: Critical code at offset 4N+1 within fetch boundary
    ;   Fetch N:   [..., 66, 03, EA]  <- ADD EBP,EDX fills rest of fetch
    ;   Fetch N+1: [31, C0, ...]      <- XOR AX,AX needs new fetch
    ;==================================================================
    mov ebp, 0x1000
    mov edx, 0x2000
    mov eax, 0xDEAD5555
    jmp CS_SEG:.test1_critical  ; Far jump flushes prefetch

    ; Pad so .test1_critical lands at offset 4N+1
    ; Far jump is 5 bytes (EA xx xx SEG SEG), setup is ~18 bytes
    ; We need target at 4N+1 from code base (0x10000)
    ; After setup instrs, align the target
    times (0x41 - ($ - $$)) db 0xCC

    ; 0x41 = 4*16+1, so physical 0x10041 is at offset 1 in a 4-byte boundary
.test1_critical:
    db 0x66, 0x03, 0xEA     ; ADD EBP, EDX (o32) - fills bytes 1,2,3 of fetch
    db 0x31, 0xC0            ; XOR AX, AX - needs next fetch

    cmp ebp, 0x3000
    jne .fail_01
    cmp ax, 0
    jne .fail_02

    ;==================================================================
    ; Test 2: Same pattern, different registers
    ;==================================================================
    mov esi, 0x1000
    mov edi, 0x2000
    mov eax, 0xBBBB7777
    jmp CS_SEG:.test2_critical

    times (0x81 - ($ - $$)) db 0xCC

.test2_critical:
    db 0x66, 0x03, 0xF7     ; ADD ESI, EDI (o32, modrm=0xF7)
    db 0x31, 0xC0            ; XOR AX, AX

    cmp esi, 0x3000
    jne .fail_03
    cmp ax, 0
    jne .fail_04

    ;==================================================================
    ; Test 3: ADD EBP,EDX (o32) + SUB CX,CX
    ;==================================================================
    mov ebp, 0x1000
    mov edx, 0x2000
    mov ecx, 0x9999
    jmp CS_SEG:.test3_critical

    times (0xC1 - ($ - $$)) db 0xCC

.test3_critical:
    db 0x66, 0x03, 0xEA     ; ADD EBP, EDX (o32)
    db 0x29, 0xC9            ; SUB CX, CX

    cmp ebp, 0x3000
    jne .fail_05
    cmp cx, 0
    jne .fail_06

    ;==================================================================
    ; Test 4: Control - same code at 4-byte aligned offset (shouldn't bug)
    ;   When ADD starts at 4N+0, both ADD and XOR are in same fetch.
    ;==================================================================
    mov ebp, 0x1000
    mov edx, 0x2000
    mov eax, 0xDEAD5555
    jmp CS_SEG:.test4_critical

    times (0x100 - ($ - $$)) db 0xCC

    ; 0x100 is 4-byte aligned, so both ADD (3 bytes) and XOR (2 bytes) fit in
    ; one 4-byte fetch + next (but XOR decodes in time from first fetch)
.test4_critical:
    db 0x66, 0x03, 0xEA     ; ADD EBP, EDX (o32)
    db 0x31, 0xC0            ; XOR AX, AX

    cmp ebp, 0x3000
    jne .fail_07
    cmp ax, 0
    jne .fail_08

    ;==================================================================
    ; Test 5: Three back-to-back ALU r,r at offset 4N+1
    ;==================================================================
    mov ebp, 0x1000
    mov edx, 0x2000
    mov eax, 0x5555
    mov ecx, 0x100
    mov ebx, 0x200
    jmp CS_SEG:.test5_critical

    times (0x141 - ($ - $$)) db 0xCC

.test5_critical:
    db 0x66, 0x03, 0xEA     ; ADD EBP, EDX (o32)
    db 0x31, 0xC0            ; XOR AX, AX
    db 0x01, 0xD9            ; ADD CX, BX

    cmp ebp, 0x3000
    jne .fail_09
    cmp ax, 0
    jne .fail_0a
    cmp cx, 0x300
    jne .fail_0b

    ;==================================================================
    ; Test 6: Control - no 66h prefix at 4N+1 (2-byte ADD + 2-byte XOR)
    ;   Without prefix, ADD is only 2 bytes and both fit in same fetch.
    ;==================================================================
    mov bp, 0x1000
    mov dx, 0x2000
    mov eax, 0xDEAD5555
    jmp CS_SEG:.test6_critical

    times (0x181 - ($ - $$)) db 0xCC

.test6_critical:
    db 0x03, 0xEA            ; ADD BP, DX (16-bit, only 2 bytes)
    db 0x31, 0xC0            ; XOR AX, AX

    cmp bp, 0x3000
    jne .fail_0c
    cmp ax, 0
    jne .fail_0d

    ; All tests passed
    mov al, STATUS_PASS
    out STATUS_PORT, al
    hlt

.fail_01:
    mov al, 0x01
    jmp .fail
.fail_02:
    mov al, 0x02
    jmp .fail
.fail_03:
    mov al, 0x03
    jmp .fail
.fail_04:
    mov al, 0x04
    jmp .fail
.fail_05:
    mov al, 0x05
    jmp .fail
.fail_06:
    mov al, 0x06
    jmp .fail
.fail_07:
    mov al, 0x07
    jmp .fail
.fail_08:
    mov al, 0x08
    jmp .fail
.fail_09:
    mov al, 0x09
    jmp .fail
.fail_0a:
    mov al, 0x0A
    jmp .fail
.fail_0b:
    mov al, 0x0B
    jmp .fail
.fail_0c:
    mov al, 0x0C
    jmp .fail
.fail_0d:
    mov al, 0x0D
    jmp .fail
.fail:
    out DATA_PORT, al
    mov al, STATUS_FAIL
    out STATUS_PORT, al
    hlt
