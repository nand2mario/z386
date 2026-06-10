; v86_int_stack_reuse.asm - VM86 software INT must reuse the same monitor stack
;
; Regression target from the FreeDOS/EMM386 traces:
;   - 486tang enters EMM386 VM86 code at 9024:0cd7 with VM=1.
;   - z386_MiSTer repeatedly enters the monitor at 0048:0cd4/0cd7 with
;     the ring-0 stack drifting downward by 0x10 per reflection, eventually
;     underflowing the small monitor stack and taking #SS.
;
; This test builds the same class of transition:
;   1. Enter protected mode with a USE16 ring-0 interrupt-gate target.
;   2. IRETD into VM86 mode with IOPL=3.
;   3. VM86 code executes repeated INT 21h.
;   4. The ring-0 handler verifies that every VM86 INT enters with the same
;      ring-0 SP, then returns to VM86 with IRETD.
;
; A stack-frame size mismatch in V86 interrupt entry/IRETD return shows up as
; different SP values on repeated handler entries.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4

STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

VEC_INT21   equ 0x21
VEC_DONE    equ 0x22

SEL_CODE0   equ 0x08
SEL_DATA0   equ 0x10
SEL_TSS     equ 0x18

STACK0_TOP  equ 0x7000
VM86_SEG    equ 0x1000
VM86_SP     equ 0xE100
LOOP_COUNT  equ 8

start:
    cli
    cld

    mov ax, cs
    mov ds, ax

    lgdt [gdt_desc]
    lidt [idt_desc]

    mov eax, cr0
    or  eax, 1
    mov cr0, eax

    jmp SEL_CODE0:pm16_entry

pm16_entry:
    mov ax, SEL_DATA0
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov sp, STACK0_TOP

    mov word [first_entry_sp], 0
    mov word [int21_count], 0

    mov ax, SEL_TSS
    ltr ax

    ; IRETD frame for entry to VM86 mode.  The extra segment dwords are
    ; mandatory for VM86 return and mirror the frame consumed by IRETd_V86.
    push dword VM86_SEG          ; GS
    push dword VM86_SEG          ; FS
    push dword VM86_SEG          ; DS
    push dword VM86_SEG          ; ES
    push dword VM86_SEG          ; SS
    push dword VM86_SP           ; ESP
    push dword 0x00023202        ; EFLAGS: VM=1, IF=1, IOPL=3
    push dword VM86_SEG          ; CS
    push dword vm86_entry        ; EIP
    iretd

; ---------------------------------------------------------------------------
; VM86 code.  CS=DS=SS=ES=FS=GS=0x1000, so labels in this image are reachable
; as real-mode offsets from the same segment.
; ---------------------------------------------------------------------------
vm86_entry:
    mov ax, cs
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov sp, VM86_SP

    mov cx, LOOP_COUNT
.loop:
    int VEC_INT21
    dec cx
    jnz .loop

    int VEC_DONE
    hlt
    jmp $

; ---------------------------------------------------------------------------
; Ring-0 USE16 monitor handlers.
; ---------------------------------------------------------------------------
int21_handler:
    ; The CPU has already switched to the TSS ring-0 stack and pushed the
    ; VM86 interrupt frame.  Check SP before the handler prologue touches it.
    mov bp, sp
    mov ax, SEL_DATA0
    mov ds, ax

    cmp word [first_entry_sp], 0
    jne .check_sp
    mov [first_entry_sp], bp
    jmp .sp_ok

.check_sp:
    cmp bp, [first_entry_sp]
    jne fail_stack_drift

.sp_ok:
    inc word [int21_count]

    ; Small EMM386-like USE16 prologue/epilogue: allocate locals, save 32-bit
    ; registers, then restore SP exactly before the VM86 IRETD.
    push bp
    mov bp, sp
    sub sp, 0x2c
    push eax
    push edx
    pop edx
    pop eax
    mov sp, bp
    pop bp

    iretd

done_handler:
    mov bp, sp
    mov ax, SEL_DATA0
    mov ds, ax

    cmp word [int21_count], LOOP_COUNT
    jne fail_count

    cmp bp, [first_entry_sp]
    jne fail_done_sp

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

fail_stack_drift:
    movzx eax, bp
    shl eax, 16
    mov ax, [first_entry_sp]
    or eax, 0x21000000
    jmp fail

fail_count:
    movzx eax, word [int21_count]
    or eax, 0x22000000
    jmp fail

fail_done_sp:
    movzx eax, bp
    shl eax, 16
    mov ax, [first_entry_sp]
    or eax, 0x23000000
    jmp fail

fail:
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
    jmp $

; ---------------------------------------------------------------------------
; Descriptor tables.
; ---------------------------------------------------------------------------
align 8
gdt:
    dq 0x0000000000000000

    ; Ring-0 USE16 code, base=0x10000, limit=0xffff.
    dw 0xffff
    dw 0x0000
    db 0x01
    db 10011011b
    db 00000000b
    db 0x00

    ; Ring-0 USE16 data/stack, base=0x10000, limit=0xffff.
    dw 0xffff
    dw 0x0000
    db 0x01
    db 10010011b
    db 00000000b
    db 0x00

    ; 32-bit TSS, base=0x10000+tss, limit=0x67.
tss_desc:
    dw 0x0067
    dw tss
    db 0x01
    db 10001001b
    db 00000000b
    db 0x00
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x00010000

align 8
idt:
    times VEC_INT21 dq 0

    ; INT 21h: 386 interrupt gate, DPL=3, target USE16 ring-0 code.
    dw int21_handler
    dw SEL_CODE0
    db 0
    db 11101110b
    dw 0

    ; INT 22h: finish test from VM86.
    dw done_handler
    dw SEL_CODE0
    db 0
    db 11101110b
    dw 0

    times (256 - VEC_DONE - 1) dq 0
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd idt + 0x00010000

align 4
tss:
    dd 0                    ; +00 backlink
    dd STACK0_TOP           ; +04 ESP0
    dd SEL_DATA0            ; +08 SS0
    dd 0                    ; +0C ESP1
    dd 0                    ; +10 SS1
    dd 0                    ; +14 ESP2
    dd 0                    ; +18 SS2
    dd 0                    ; +1C CR3
    dd 0                    ; +20 EIP
    dd 0                    ; +24 EFLAGS
    dd 0                    ; +28 EAX
    dd 0                    ; +2C ECX
    dd 0                    ; +30 EDX
    dd 0                    ; +34 EBX
    dd 0                    ; +38 ESP
    dd 0                    ; +3C EBP
    dd 0                    ; +40 ESI
    dd 0                    ; +44 EDI
    dd 0                    ; +48 ES
    dd 0                    ; +4C CS
    dd 0                    ; +50 SS
    dd 0                    ; +54 DS
    dd 0                    ; +58 FS
    dd 0                    ; +5C GS
    dd 0                    ; +60 LDTR
    dw 0                    ; +64 debug trap
    dw 104                  ; +66 IOPB offset

first_entry_sp: dw 0
int21_count:    dw 0
