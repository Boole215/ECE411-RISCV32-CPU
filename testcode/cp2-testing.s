#  mp4-cp2.s version 1.3
.align 4
.section .text
.globl _start
_start:

	add x0, x1, 5
	add x1, x2, 5
	add x3, x3, 5
	beq x0, x0, branch_after
	add x4, x4, 5
	add x5, x5, 5
	add x6, x6, 5

branch_after:
	beq x0, x0, branch_after
