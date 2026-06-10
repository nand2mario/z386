; seg_paging.asm - Segmentation + Paging Test
;
; Tests address translation through both segmentation and paging.
; Configured by test runner with:
;   CS: base=0x00010000 (identity mapped, prefetch bypasses paging)
;   DS: base=0x20000000, linear 0x20000000 -> physical 0x00020000
;   SS: base=0x30000000, linear 0x30000000 -> physical 0x00060000
;
; Test sequence:
; 1. Write patterns through DS segment (base 0x20000000)
; 2. Read back and verify
; 3. Cross-page boundary access
; 4. TLB eviction (5 pages in same set with 4-way TLB)
;
; Results reported via I/O ports:
;   Port 0xE0: Status (0x00=running, 0x01=pass, 0xFF=fail)
;   Port 0xE4: Debug data (32-bit values for debugging)

BITS 32
ORG 0                   ; Code at offset 0 in CS segment

; I/O ports for test results
STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4

; Status values
STATUS_RUNNING equ 0x00
STATUS_PASS    equ 0x01
STATUS_FAIL    equ 0xFF

; Test patterns
PATTERN1    equ 0xDEADBEEF
PATTERN2    equ 0xCAFEBABE
PATTERN3    equ 0x12345678
PATTERN4    equ 0xFEEDFACE

section .text
start:
    ; Signal test running
    mov eax, STATUS_RUNNING
    out STATUS_PORT, al

    ;===========================================================
    ; Test 1: Basic write/read through DS segment
    ;===========================================================
    ; DS base = 0x20000000, paging maps to physical 0x00020000
    ; Write at DS:0x100 -> linear 0x20000100 -> physical 0x00020100

    mov dword [0x100], PATTERN1
    mov eax, [0x100]
    cmp eax, PATTERN1
    jne .fail_test1

    ; Write second pattern at different offset
    mov dword [0x200], PATTERN2
    mov eax, [0x200]
    cmp eax, PATTERN2
    jne .fail_test1

    ; Verify first pattern not corrupted
    mov eax, [0x100]
    cmp eax, PATTERN1
    jne .fail_test1

    ;===========================================================
    ; Test 2: Multiple locations in same page
    ;===========================================================
    mov dword [0x300], PATTERN3
    mov dword [0x304], PATTERN4
    mov dword [0x308], PATTERN1
    mov dword [0x30C], PATTERN2

    ; Read back all
    mov eax, [0x300]
    cmp eax, PATTERN3
    jne .fail_test2

    mov eax, [0x304]
    cmp eax, PATTERN4
    jne .fail_test2

    mov eax, [0x308]
    cmp eax, PATTERN1
    jne .fail_test2

    mov eax, [0x30C]
    cmp eax, PATTERN2
    jne .fail_test2

    ;===========================================================
    ; Test 3: Cross-page boundary access
    ;===========================================================
    ; Write DWORD at DS:0x0FFE -> crosses page boundary at 0x1000
    ; Page 0: linear 0x20000000 -> phys 0x00020000
    ; Page 1: linear 0x20001000 -> phys 0x00040000 (non-contiguous!)
    ; Low 2 bytes go to phys 0x00020FFE, high 2 bytes go to phys 0x00040000

    mov dword [0x0FFE], PATTERN3
    mov eax, [0x0FFE]
    cmp eax, PATTERN3
    jne .fail_test3

    ; Another cross-page write at different offset
    mov dword [0x0FFF], PATTERN4
    mov eax, [0x0FFF]
    cmp eax, PATTERN4
    jne .fail_test3

.test4:
    ;===========================================================
    ; Test 4: TLB eviction (32-entry 4-way set-associative TLB)
    ;===========================================================
    ; TLB set = VPN[2:0]. Pages 8 apart (0x8000 linear) share a set.
    ; Each set has 4 ways, so the 5th page in the same set evicts.
    ; Offsets: 0x0000, 0x8000, 0x10000, 0x18000, 0x20000 -> all VPN[2:0]=0

    mov dword [0x00000], 0x11111111   ; Page 0,  set 0, way 0 (cold miss)
    mov dword [0x08000], 0x22222222   ; Page 8,  set 0, way 1 (cold miss)
    mov dword [0x10000], 0x33333333   ; Page 16, set 0, way 2 (cold miss)
    mov dword [0x18000], 0x44444444   ; Page 24, set 0, way 3 (cold miss)
    mov dword [0x20000], 0x55555555   ; Page 32, set 0, evicts way 0 (PLRU)

    ; Read back - page 0 was evicted, triggers page walk
    mov eax, [0x00000]
    cmp eax, 0x11111111
    jne .fail_test4

    mov eax, [0x08000]
    cmp eax, 0x22222222
    jne .fail_test4

    mov eax, [0x10000]
    cmp eax, 0x33333333
    jne .fail_test4

    mov eax, [0x18000]
    cmp eax, 0x44444444
    jne .fail_test4

    mov eax, [0x20000]
    cmp eax, 0x55555555
    jne .fail_test4

    ;===========================================================
    ; Test 5: Register indirect addressing
    ;===========================================================
    mov edi, 0x400
    mov dword [edi], PATTERN1
    mov dword [edi+4], PATTERN2
    mov dword [edi+8], PATTERN3

    mov eax, [edi]
    cmp eax, PATTERN1
    jne .fail_test5

    mov eax, [edi+4]
    cmp eax, PATTERN2
    jne .fail_test5

    mov eax, [edi+8]
    cmp eax, PATTERN3
    jne .fail_test5

    ;===========================================================
    ; Test 6: Stack operations (through SS segment)
    ;===========================================================
    ; SS base = 0x30000000, maps to physical 0x00060000
    ; Set ESP to valid stack area within mapped pages

    mov esp, 0xFF00

    push PATTERN1
    push PATTERN2
    push PATTERN3

    pop eax
    cmp eax, PATTERN3
    jne .fail_test6

    pop eax
    cmp eax, PATTERN2
    jne .fail_test6

    pop eax
    cmp eax, PATTERN1
    jne .fail_test6

    ;===========================================================
    ; All tests passed
    ;===========================================================
    mov eax, STATUS_PASS
    out STATUS_PORT, al
    hlt

    ;===========================================================
    ; Failure handlers
    ;===========================================================
.fail_test1:
    mov eax, 1
    out DATA_PORT, eax
    jmp .fail

.fail_test2:
    mov eax, 2
    out DATA_PORT, eax
    jmp .fail

.fail_test3:
    mov eax, 3
    out DATA_PORT, eax
    jmp .fail

.fail_test4:
    mov eax, 4
    out DATA_PORT, eax
    jmp .fail

.fail_test5:
    mov eax, 5
    out DATA_PORT, eax
    jmp .fail

.fail_test6:
    mov eax, 6
    out DATA_PORT, eax
    jmp .fail

.fail:
    mov eax, STATUS_FAIL
    out STATUS_PORT, al
    hlt

    ; No padding needed - code fits within available space
