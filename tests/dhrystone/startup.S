    .section .text.start, "ax"
    .code32
    .global _start
    .extern dhrystone_main

_start:
    cld
    call dhrystone_main

    movw $0x00e4, %dx
    outl %eax, %dx

    testl %eax, %eax
    jnz 1f
    movl $0x01, %eax
    jmp 2f

1:
    movl $0xff, %eax

2:
    movw $0x00e0, %dx
    outb %al, %dx

3:
    hlt
    jmp 3b
