; mov_dr_cr3_preserve.asm - MOV to/from DR0-3 must not clobber CR3
;
; Regression test for the cr3_write decode bug: the MOV DR/TR microcode
; routines end with a "0 -> DESABS IND=" uop (38F/3B4) to access the DR
; register file via absolute IND addressing.  z386 decoded any IND=+DESABS
; as a CR3 write, so MOV rd,DRn zeroed CR3 (and flushed the TLB).  Under
; EMM386, the VCPI/DPMI debug-register context save then killed paging and
; the machine looped on undeliverable page faults (TC3 hang).
; The real CR3 commit is the SPCR+PDBR uop (36F/794/93B/97B/9AA).
;
; Runs with paging on and CR3 = 0x5000 (page directory NOT at physical 0,
; so a zero-clobber is observable).  After each DR access, CR3 is read back
; and a never-touched page is accessed to force a page walk through the
; current CR3 (a clobbered CR3 walks an empty directory and faults).
;
; Results: port 0xE0 status (0x01 pass / 0xFF fail), port 0xE4 fail code.

BITS 32
ORG 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

CR3_EXPECT  equ 0x00005000

section .text
start:
    mov esp, 0x0F00

    ; Baseline: CR3 must read back as configured
    mov eax, cr3
    cmp eax, CR3_EXPECT
    jne .fail_1

    ; Populate a TLB entry and verify data path works
    mov dword [0x100], 0xDEADBEEF
    mov eax, [0x100]
    cmp eax, 0xDEADBEEF
    jne .fail_2

    ; MOV DRn,rd (microcode 38A-38F, tail "0 -> DESABS IND=")
    mov eax, 0x12340000
    mov dr0, eax
    mov ebx, cr3
    cmp ebx, CR3_EXPECT
    jne .fail_3

    ; MOV rd,DRn (microcode 3AE-3B4, the tail that zeroed CR3)
    mov ecx, dr0
    mov ebx, cr3
    cmp ebx, CR3_EXPECT
    jne .fail_4

    ; DR6/DR7 use dedicated microcode destinations and readback paths.
    mov eax, 0x13579BDF
    mov dr6, eax
    mov ecx, dr6
    cmp ecx, eax
    jne .fail_5

    ; Keep breakpoint enables and GD clear; bit 10 is the inert fixed-one bit.
    mov eax, 0x00000400
    mov dr7, eax
    mov ecx, dr7
    cmp ecx, eax
    jne .fail_5

    mov ebx, cr3
    cmp ebx, CR3_EXPECT
    jne .fail_5

    ; Cold page: never touched, so this forces a page walk through the
    ; current CR3.  With CR3 clobbered to 0 the walk reads an empty
    ; directory and the test dies on an undeliverable page fault.
    mov dword [0x1000], 0xCAFEBABE
    mov eax, [0x1000]
    cmp eax, 0xCAFEBABE
    jne .fail_6

    mov al, STATUS_PASS
    out STATUS_PORT, al
    hlt

.fail_1:
    mov eax, 1
    jmp .fail
.fail_2:
    mov eax, 2
    jmp .fail
.fail_3:
    mov eax, 3
    jmp .fail
.fail_4:
    mov eax, 4
    jmp .fail
.fail_5:
    mov eax, 5
    jmp .fail
.fail_6:
    mov eax, 6
.fail:
    out DATA_PORT, eax
    mov al, STATUS_FAIL
    out STATUS_PORT, al
    hlt
