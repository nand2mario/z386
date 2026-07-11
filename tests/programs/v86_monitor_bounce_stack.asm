; v86_monitor_bounce_stack.asm - VM86 monitor reflection must not leak stack
;
; This is aligned to the FreeDOS/EMM386 hang trace more closely than the
; simple v86_int_stack_reuse loop:
;   1. VM86 code enters a USE16 ring-0 monitor through INT 21h.
;   2. The monitor records SP at the first instruction, before its prologue.
;   3. The monitor edits the saved VM86 IRETD frame in place so return goes
;      to a BIOS-like alias segment instead of the original interrupted EIP.
;   4. The alias path re-enters the monitor several times.
;
; If VM86 interrupt entry or IRETD return consumes the wrong frame size, the
; monitor entry SP drifts by 0x10-ish per reflection, matching the bad trace.

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
SEL_STACK0  equ 0x18
SEL_TSS     equ 0x20

; Match the bad trace scale: ESP0 around 0x0fd8, entry SP around 0x0fb4.
STACK0_TOP  equ 0x0FD8

VM86_MAIN_SEG    equ 0x1000
VM86_BIOS_SEG    equ 0x0F00
VM86_CLIENT_SEG  equ 0x0292
VM86_BIOS_DELTA   equ ((VM86_MAIN_SEG - VM86_BIOS_SEG) << 4)
VM86_CLIENT_DELTA equ ((VM86_MAIN_SEG - VM86_CLIENT_SEG) << 4)
VM86_SP          equ 0xE100
VM86_IRET_SP     equ (VM86_SP - 6)
LOOP_COUNT       equ 8

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
    mov ax, SEL_STACK0
    mov ss, ax
    mov sp, STACK0_TOP

    mov word [first_entry_sp], 0
    mov word [int21_count], 0
    mov word [vm86_flag], 0

    ; VM86 BIOS IRET target frame.  Each monitor reflection resets VM86 SS:SP
    ; to this frame, so the BIOS-like stub can IRET to the 0292-like client.
    mov word [VM86_IRET_SP + 0], vm86_client_entry + VM86_CLIENT_DELTA
    mov word [VM86_IRET_SP + 2], VM86_CLIENT_SEG
    mov word [VM86_IRET_SP + 4], 0x7002

    mov ax, SEL_TSS
    ltr ax

    ; Initial VM86 IRETD frame.  The extra segment dwords are required when
    ; returning to VM86 mode.
    push dword VM86_MAIN_SEG        ; GS
    push dword VM86_MAIN_SEG        ; FS
    push dword VM86_MAIN_SEG        ; DS
    push dword VM86_MAIN_SEG        ; ES
    push dword VM86_MAIN_SEG        ; SS
    push dword VM86_SP              ; ESP
    push dword 0x00023202           ; EFLAGS: VM=1, IF=1, IOPL=3
    push dword VM86_MAIN_SEG        ; CS
    push dword vm86_entry           ; EIP
    iretd

; ---------------------------------------------------------------------------
; VM86 code.
; ---------------------------------------------------------------------------
vm86_entry:
    mov ax, cs
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov sp, VM86_SP

    int VEC_INT21
    hlt
    jmp $

; Entered using CS=VM86_BIOS_SEG and IP=(vm86_bios_entry+delta), like the
; trace's F000:E9E6 BIOS path.  It acknowledges PIC and IRETs to the 0292-like
; VM86 client frame prepared above.
vm86_bios_entry:
    push ax
    call vm86_bios_eoi
    pop ax
    iret

vm86_bios_eoi:
    mov al, 0x20
    out 0x20, al
    ret

; Entered through VM86 IRET using CS=VM86_CLIENT_SEG.  This mirrors the trace's
; 0292:04c4 path: CLI, OR a memory flag, then INT 21h back to the monitor.
vm86_client_entry:
    cli
    or word [vm86_flag], 0x20
    int VEC_INT21
    hlt
    jmp $

vm86_done_entry:
    int VEC_DONE
    hlt
    jmp $

; ---------------------------------------------------------------------------
; Ring-0 USE16 monitor handlers.
; ---------------------------------------------------------------------------
int21_handler:
    ; Check the exact monitor-entry SP before any handler pushes.  On a
    ; correct CPU this remains constant even though the monitor patches the
    ; saved VM86 return CS:EIP each time.
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

    mov [entry_sp_this], bp

    ; Trace-shaped EMM386 USE16 prologue:
    ;   0258: 66 55        push ebp
    ;   025a: 0f b7 ec     movzx ebp, sp
    ;   025e: 6a 0d        push byte +0x0d
    ;   0311: 66 53        push ebx
    ;   0313: 66 56        push esi
    ;   0318: 1e           push ds
    ;   031c: 66 50        push eax
    ;   033f: 66 58        pop eax
    ;   0cd4: 83 ec 2c     sub sp, 0x2c
    ;   0cd7: 83 ed 2c     sub bp, 0x2c
    push ebp
    movzx ebp, sp

    mov ax, [entry_sp_this]
    sub ax, 0x0004
    cmp bp, ax
    jne fail_trace_delta

    push byte 0x0d
    push ebx
    push esi
    push ds
    mov si, VM86_CLIENT_SEG
    push eax
    pop eax

    mov ax, [entry_sp_this]
    sub ax, 0x0010
    cmp sp, ax
    jne fail_trace_delta

    sub sp, 0x2c
    mov ax, [entry_sp_this]
    sub ax, 0x003c
    cmp sp, ax
    jne fail_trace_delta

    sub bp, 0x2c
    mov ax, [entry_sp_this]
    sub ax, 0x0030
    cmp bp, ax
    jne fail_trace_delta

    push eax
    push edx

    ; Restore the original VM86 interrupt frame top before editing it, so any
    ; later drift comes from CPU entry/IRETD behavior rather than this prologue.
    mov bp, [entry_sp_this]
    mov sp, bp

    mov ax, [int21_count]
    cmp ax, LOOP_COUNT
    jae .return_done

    ; VM86 interrupt frame at SS:BP:
    ;   +00 EIP, +04 CS, +08 EFLAGS, +0C ESP, +10 SS,
    ;   +14 ES, +18 DS, +1C FS, +20 GS.
    mov dword [bp + 0x00], vm86_bios_entry + VM86_BIOS_DELTA
    mov dword [bp + 0x04], VM86_BIOS_SEG
    mov dword [bp + 0x0c], VM86_IRET_SP
    mov dword [bp + 0x10], VM86_MAIN_SEG
    mov dword [bp + 0x14], VM86_MAIN_SEG
    mov dword [bp + 0x18], VM86_MAIN_SEG
    mov dword [bp + 0x1c], VM86_MAIN_SEG
    mov dword [bp + 0x20], VM86_MAIN_SEG
    iretd

.return_done:
    mov dword [bp + 0x00], vm86_done_entry
    mov dword [bp + 0x04], VM86_MAIN_SEG
    mov dword [bp + 0x14], VM86_MAIN_SEG
    mov dword [bp + 0x18], VM86_MAIN_SEG
    mov dword [bp + 0x1c], VM86_MAIN_SEG
    mov dword [bp + 0x20], VM86_MAIN_SEG
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

fail_trace_delta:
    movzx eax, sp
    shl eax, 16
    mov ax, [entry_sp_this]
    or eax, 0x24000000
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

    ; Ring-0 USE16 data, base=0x10000, limit=0xffff.
    dw 0xffff
    dw 0x0000
    db 0x01
    db 10010011b
    db 00000000b
    db 0x00

    ; Ring-0 USE16 stack, base=0x12000, limit=0x0fff like the bad trace.
    dw 0x0fff
    dw 0x2000
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
    dd SEL_STACK0           ; +08 SS0
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
entry_sp_this:  dw 0
vm86_flag:      dw 0
