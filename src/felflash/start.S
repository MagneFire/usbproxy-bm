/* felflash: writes the card over FEL. Loaded into DRAM by sunxi-fel after
 * `sunxi-fel spl boot0.bin` brought DRAM up, run with `sunxi-fel exe` as a
 * function the BROM calls: own stack while it runs, everything restored on
 * return so the FEL session continues (the Mac reads the status word). */
        .syntax unified
        .arm
        .section .entry, "ax"
        .global _start
_start:
        push    {r4-r11, lr}
        mov     r4, sp                  /* callee-saved: survives main (a .bss slot
                                         * did not: the zero loop below wiped it and
                                         * the return to the BROM died, 2026-10-10) */
        ldr     sp, =__stack_top
        ldr     r0, =__bss_start
        ldr     r1, =__bss_end
        mov     r2, #0
1:      cmp     r0, r1
        strlo   r2, [r0], #4
        blo     1b
        bl      felflash_main
        mov     sp, r4
        pop     {r4-r11, lr}
        bx      lr

        .global irq_enable
irq_enable:
        bx      lr
        .global irq_disable
irq_disable:
        bx      lr
