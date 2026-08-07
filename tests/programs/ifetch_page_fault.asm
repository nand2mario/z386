; ifetch_page_fault.asm - demanded instruction-fetch page fault
;
; The first code page is present and the next page is absent.  A short JMP
; begins at the final byte of the present page, forcing decode to demand its
; displacement from the absent page.  Speculative prefetch faults are silent,
; but this one must become #PF once the frontend runs out of mapped bytes.

BITS 32
ORG 0

STATUS_PORT equ 0xE0
DATA_PORT   equ 0xE4
CODE_LINEAR equ 0x00010000

align 8
gdt:
    dq 0
    dq 0x00cf9b010000ffff     ; flat 32-bit code, base 0x10000
    dq 0x00cf93010000ffff     ; flat 32-bit data, base 0x10000
gdt_end:

gdt_desc:
    dw gdt_end - gdt - 1
    dd CODE_LINEAR + gdt

pf_handler:
    mov eax, cr2
    and eax, 0xfffff000
    cmp eax, CODE_LINEAR + 0x1000
    jne fail_cr2

    ; Same-level 386 interrupt gate frame: error code is at the top.
    cmp dword [ss:esp], 0
    jne fail_code
    cmp dword [ss:esp + 4], fault_site
    jne fail_eip
    cmp esp, 0x0000fef0
    jne fail_stack
    test dword [ss:esp + 12], 0x00000200
    jz fail_flags

    mov al, 1
    out STATUS_PORT, al
    hlt

fail_cr2:
    mov eax, 1
    out DATA_PORT, eax
    jmp fail

fail_code:
    mov eax, 2
    out DATA_PORT, eax
    jmp fail

fail_eip:
    mov eax, 3
    out DATA_PORT, eax
    jmp fail

fail_stack:
    mov eax, 4
    out DATA_PORT, eax
    jmp fail

fail_flags:
    mov eax, 5
    out DATA_PORT, eax

fail:
    mov al, 0xff
    out STATUS_PORT, al
    hlt

align 8
idt:
    times 14 dq 0
    dw pf_handler
    dw 0x0008
    db 0
    db 0x8e                    ; present DPL0 386 interrupt gate
    dw 0
idt_end:

idt_desc:
    dw idt_end - idt - 1
    dd CODE_LINEAR + idt

times 0x0200 - ($ - $$) db 0x90
start:
    lgdt [cs:gdt_desc]
    lidt [cs:idt_desc]

    ; A wrong-path target in the absent page may fault speculatively, but that
    ; fault must be discarded when the branch resolves not taken.
    xor eax, eax
    jnz near absent_target
    times 128 db 0x90

    ; Continue linearly.  Prefetch will discover the absent next page while
    ; these NOPs are still executable; #PF must remain deferred until D1
    ; reaches fault_site.

times 0x0ffe - ($ - $$) db 0x90
    sti                       ; FLAGSB from i_pop is intentionally stale here
fault_site:
    db 0xeb                    ; displacement byte lies in absent page
absent_target:
    db 0x00
