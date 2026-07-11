; flag_chain.asm - Cross-instruction flag-chain regression (FAST-chain hazards)
;
; The singlestep suites reset state per instruction, so they cannot catch
; flag hazards BETWEEN chained instructions (doc: two-cycle flag validation
; gap).  This test exercises the patterns that broke in the field when FAST
; chaining ran flag producer + CF-preserving consumer back-to-back (1/cycle):
;
;   1. adc/add -> dec -> jnc   (doomfps1 FPS bug: Watcom FP emulator
;      mantissa-normalize loop `add di,di; adc...; adc ax,ax; dec si; js; jnc`)
;   2. shr -> dec -> jnc       (Landmark 6.0 hang: PKLITE-style depacker
;      `shr bp,1; dec dx; jz reload; jnc literal`)
;   3. add -> not -> jc        (NOT preserves CF through the ALU passthrough,
;      update_carry=1 path)
;
; Root cause was the ALU .flags input reading raw EFLAGS: in the
; predecessor's flag2/sh2 commit cycle the preserved-CF passthrough captured
; the stale pre-commit value and the consumer's commit clobbered CF.
; Fix: .flags(eflags_fwd).  Each pattern is run with both CF=1 and CF=0 so a
; stale capture in either direction fails.

BITS 16
org 0

STATUS_PORT equ 0xE0
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

start:
    cli
    cld
    mov ax, cs
    mov ds, ax
    mov ss, ax
    mov sp, 0x400

    mov bx, 0x100        ; scratch counters for dec (values irrelevant,
    mov cx, 0x100        ; must stay > 0 so ZF stays clear)
    mov dx, 0x100

;--- 1a: add sets CF=1, dec preserves, jnc must NOT be taken
    mov ax, 0xFFFF
    add ax, 1            ; CF=1
    dec bx               ; chained; must preserve CF=1
    jnc fail
;--- 1b: add sets CF=0
    mov ax, 1
    add ax, 1            ; CF=0
    dec bx
    jc  fail
;--- 1c: the exact doomfps1 shape: adc chain then dec+js+jnc
    mov ax, 0x8000
    stc
    adc ax, ax           ; CF=1 (0x8000<<1 +1 carries out)
    dec bx               ; preserve CF=1
    js  fail             ; SF from dec (bx>0 → clear)
    jnc fail             ; must see adc's CF=1
    mov ax, 0x0001
    clc
    adc ax, ax           ; CF=0
    dec bx
    js  fail
    jc  fail             ; must see adc's CF=0

;--- 2a: shr sets CF=1 (Landmark depacker shape)
    mov ax, 3
    shr ax, 1            ; CF=1
    dec cx               ; preserve
    jz  fail             ; cx>0
    jnc fail             ; must see shr's CF=1
;--- 2b: shr sets CF=0
    mov ax, 4
    shr ax, 1            ; CF=0
    dec cx
    jz  fail
    jc  fail

;--- 3a: NOT preserves CF=1 (ALU passthrough, update_carry=1 encoding)
    mov ax, 0xFFFF
    add ax, 1            ; CF=1
    not dx               ; must preserve CF=1
    jnc fail
;--- 3b: NOT preserves CF=0
    mov ax, 1
    add ax, 1            ; CF=0
    not dx
    jc  fail

;--- 4: INC preserves CF both ways
    mov ax, 0xFFFF
    add ax, 1            ; CF=1
    inc bx
    jnc fail
    mov ax, 1
    add ax, 1            ; CF=0
    inc bx
    jc  fail

;--- 5: repeat the depacker loop shape for a few control words to stress
;       back-to-back shr/dec/jcc chains with alternating carries
    mov bp, 0xA5C3       ; control bits pattern
    mov dx, 16
    mov si, 0            ; count of 1-bits seen
.bitloop:
    shr bp, 1            ; CF = control bit
    dec dx               ; preserve CF
    jc  .one
    jmp .next
.one:
    inc si
.next:
    test dx, dx
    jnz .bitloop
    cmp si, 8            ; 0xA5C3 has 8 set bits
    jne fail

pass:
    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt

fail:
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
