#  mp4-cp1.s version 4.0
.align 4
.section .text
.globl _start
_start:
    addi x1, x1, 25
    addi x2, x2, 20
    nop
    nop
    nop
    nop
    nop
    add x1, x1, x2
    nop
    nop
    nop
    nop
    nop
HALT:
    beq x0, x0, HALT
    nop
    nop
    nop
    nop
    nop
    nop
    nop
