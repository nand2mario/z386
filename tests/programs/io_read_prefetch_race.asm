; io_read_prefetch_race.asm
; Reproduces the z386 I/O-read data-latch race.
;
; The testbench returns 0xFFFFFFFF for every I/O read. A tight loop reads a port
; with IN EAX,DX and AND-accumulates into EBX, so EBX must stay 0xFFFFFFFF.
;
; The bug: an I/O (direct) read bypasses the cache, and its result lands in OPR_R
; via the shared external data bus (din).  When the direct read's completion is
; mis-attributed to an interleaved instruction-prefetch bus response (a memory
; resp_valid that arrives while the direct read is still pending), OPR_R latches
; the prefetched code word instead of the I/O data.  So some IN reads return code
; bytes, and EBX drops bits.
;
; Needs mem_latency > 1 (see .json) so each I/O read spans multiple bus cycles and
; a prefetch interleaves.  Pass: EBX == 0xFFFFFFFF.  Fail: any bit dropped.
;
; Result protocol: port 0xE0 status (0x01 pass / 0xFF fail), 0xE4 = bad EBX.

BITS 16
org 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
STATUS_PASS equ 0x01
STATUS_FAIL equ 0xFF

start:
    cli
    mov ax, cs
    mov ds, ax
    mov ss, ax
    mov sp, 0x7000

    mov dx, 0x03DA          ; any I/O port; tb returns 0xFFFFFFFF for every IN
    mov ebx, 0xFFFFFFFF     ; AND accumulator - must stay all-ones
    mov ecx, 3000           ; iterations
.loop:
    in eax, dx              ; I/O read -> must read 0xFFFFFFFF
    and ebx, eax            ; corruption injects code bytes -> bits drop
    dec ecx
    jnz .loop

    cmp ebx, 0xFFFFFFFF     ; any dropped bit => an IN latched stale/prefetch data
    jne .fail

    mov al, STATUS_PASS
    mov dx, STATUS_PORT
    out dx, al
    jmp .hang
.fail:
    mov eax, ebx            ; report the corrupted accumulator on 0xE4
    mov dx, DATA_PORT
    out dx, eax
    mov al, STATUS_FAIL
    mov dx, STATUS_PORT
    out dx, al
.hang:
    hlt
    jmp .hang
