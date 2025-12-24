#include "private_kdefs.h"

.section .text.init
    .globl _start
_start:
    la sp, _sbss

    call mm_init
    call task_init

    la t0, _traps
    csrw stvec, t0

    li t0, 0x20
    csrw sie, t0

    call clock_set_next_event

    li t0, 0x2
    csrs sstatus, t0

    j start_kernel

    .section .bss.stack
    .space PGSIZE