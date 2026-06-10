; pm_init.asm - Protected-mode initialization smoke test
;
; Verifies basic protected-mode testbench initialization:
; 1) CR0 has PE=1 and PG=1
; 2) Segment selectors are initialized as expected
; 3) DS memory access works through segmentation + paging
; 4) SS stack access (push/pop) works through segmentation + paging
;
; Result protocol:
;   Port 0xE0: status (0x01 = pass, 0xFF = fail)
;   Port 0xE4: failure code / debug value

BITS 32
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4

STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

start:
    ; Mark running.
    xor eax, eax
    mov dx, STATUS_PORT
    out dx, al

    ; Step 1: Protected mode + paging must both be enabled.
    mov eax, cr0
    test eax, 0x00000001        ; PE
    jz .fail_01
    test eax, 0x80000000        ; PG
    jz .fail_02

    ; Step 2: Expected selector values from testbench setup.
    mov ax, cs
    cmp ax, 0x0008
    jne .fail_03
    mov ax, ds
    cmp ax, 0x0010
    jne .fail_04
    mov ax, ss
    cmp ax, 0x0018
    jne .fail_05

    ; Step 3: DS-mapped write/read should round-trip.
    mov dword [0x120], 0x11223344
    mov eax, [0x120]
    cmp eax, 0x11223344
    jne .fail_06

    ; Step 4: SS-mapped stack push/pop round-trip.
    mov esp, 0x00000100
    mov eax, 0xA5A5A5A5
    push eax
    mov ebx, [esp]
    cmp ebx, eax
    jne .fail_07
    xor ecx, ecx
    pop ecx
    cmp ecx, eax
    jne .fail_08

    ; PASS
    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    hlt

.fail_01:
    mov eax, 0x00000001
    jmp .fail
.fail_02:
    mov eax, 0x00000002
    jmp .fail
.fail_03:
    mov eax, 0x00000003
    jmp .fail
.fail_04:
    mov eax, 0x00000004
    jmp .fail
.fail_05:
    mov eax, 0x00000005
    jmp .fail
.fail_06:
    mov eax, 0x00000006
    jmp .fail
.fail_07:
    mov eax, 0x00000007
    jmp .fail
.fail_08:
    mov eax, 0x00000008

.fail:
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
    hlt
