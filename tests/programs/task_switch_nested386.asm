; task_switch_nested386.asm - nested 386 task CALL and IRET return
;
; LOAD_TASK clears TASK_SAVED before loading the previous task during a
; nested-task return.  The CS value read from either TSS must therefore set
; CPL from its own RPL, independent of the TASK_SAVED latch.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

SEL_CODE0   equ 0x08
SEL_DATA0   equ 0x10
SEL_TSS_OLD equ 0x18
SEL_TSS_NEW equ 0x20
SEL_CODE3   equ 0x2B
SEL_DATA3   equ 0x33

STACK_OLD   equ 0x3000
STACK_NEW   equ 0x4000

start:
    cli
    lgdt [cs:gdt_desc]

    mov eax, cr0
    or eax, 1
    mov cr0, eax
    jmp dword SEL_CODE0:pm_entry

BITS 32
pm_entry:
    mov ax, SEL_DATA0
    mov ds, ax
    mov es, ax
    mov ss, ax
    mov esp, STACK_OLD

    mov ax, SEL_TSS_OLD
    ltr ax

    ; A task CALL records SEL_TSS_OLD in the new TSS backlink and sets NT.
    call SEL_TSS_NEW:0

task_returned:
    mov ax, cs
    cmp ax, SEL_CODE0
    jne fail_31
    mov ax, ss
    cmp ax, SEL_DATA0
    jne fail_32
    pushfd
    pop eax
    test eax, 0x4000
    jnz fail_33

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt

task_entry:
    mov ax, cs
    cmp ax, SEL_CODE3
    jne fail_21
    mov ax, ss
    cmp ax, SEL_DATA3
    jne fail_22
    pushfd
    pop eax
    test eax, 0x4000
    jz fail_23

    ; NT=1 makes IRET perform the nested task return through the backlink.
    iretd
    mov eax, 0x24
    jmp fail

fail_21:
    mov eax, 0x21
    jmp fail
fail_22:
    mov eax, 0x22
    jmp fail
fail_23:
    mov eax, 0x23
    jmp fail
fail_31:
    mov eax, 0x31
    jmp fail
fail_32:
    mov eax, 0x32
    jmp fail
fail_33:
    mov eax, 0x33
fail:
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt

align 8
gdt:
    dq 0
    dq 0x00CF9B010000FFFF       ; 08: ring-0 code, base 10000h
    dq 0x00CF93010000FFFF       ; 10: ring-0 data, base 10000h

    ; 18: available 386 TSS, base 10000h + tss_old, limit 67h
    dw 0x0067
    dw tss_old
    db 0x01
    db 10001001b
    db 0
    db 0

    ; 20: available 386 TSS, base 10000h + tss_new, limit 67h
    dw 0x0067
    dw tss_new
    db 0x01
    db 10001001b
    db 0
    db 0

    dq 0x00CFFB010000FFFF       ; 28: ring-3 code, base 10000h
    dq 0x00CFF3010000FFFF       ; 30: ring-3 data, base 10000h
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd gdt + 0x10000

align 4
tss_old:
    times 26 dd 0

align 4
tss_new:
    dd 0                        ; 00 backlink (written by task CALL)
    dd 0                        ; 04 ESP0
    dd 0                        ; 08 SS0
    dd 0                        ; 0C ESP1
    dd 0                        ; 10 SS1
    dd 0                        ; 14 ESP2
    dd 0                        ; 18 SS2
    dd 0                        ; 1C CR3 (paging disabled)
    dd task_entry               ; 20 EIP
    dd 0x00003002               ; 24 EFLAGS (IOPL=3; NT set by task CALL)
    dd 0                        ; 28 EAX
    dd 0                        ; 2C ECX
    dd 0                        ; 30 EDX
    dd 0                        ; 34 EBX
    dd STACK_NEW                ; 38 ESP
    dd 0                        ; 3C EBP
    dd 0                        ; 40 ESI
    dd 0                        ; 44 EDI
    dd SEL_DATA3                ; 48 ES
    dd SEL_CODE3                ; 4C CS
    dd SEL_DATA3                ; 50 SS
    dd SEL_DATA3                ; 54 DS
    dd 0                        ; 58 FS
    dd 0                        ; 5C GS
    dd 0                        ; 60 LDTR
    dw 0                        ; 64 debug trap
    dw 0x0068                   ; 66 I/O bitmap beyond TSS limit
