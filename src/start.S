/* Entry and exception vectors. Three ways in, one state out:
 *   SPL Falcon jump : MMU/caches off, secure SVC, r2 = DTB (ignored)
 *   U-Boot `go`     : MMU and D-cache ON, U-Boot's stack, r0/r1 = argc/argv
 *   FEL `exe`       : BROM state, caches off
 * We clean the D-cache while it may still be on, switch everything off,
 * invalidate, set our own stacks and vectors, clear .bss, write CNTFRQ (only
 * the Falcon SPL path does it for us, and a watchdog reset clears it), then
 * call main() with IRQs still masked; main enables them once the GIC is up. */
        .syntax unified
        .arm

        .section .vectors, "ax"
        .global _start
_start:
        b       reset
        ldr     pc, =undef_entry
        ldr     pc, =svc_entry
        ldr     pc, =pabt_entry
        ldr     pc, =dabt_entry
        nop
        ldr     pc, =irq_entry
        ldr     pc, =fiq_entry

        .text
reset:
        cpsid   if
        /* D-cache on (U-Boot `go`)? Clean+invalidate it first: U-Boot's loady
         * wrote this very image through it. */
        mrc     p15, 0, r0, c1, c0, 0
        tst     r0, #(1 << 2)
        beq     1f
        bl      dcache_clean_invalidate_all
1:      mrc     p15, 0, r0, c1, c0, 0
        bic     r0, r0, #(1 << 0) | (1 << 2)            /* M, C */
        bic     r0, r0, #(1 << 11) | (1 << 12) | (1 << 13) /* Z, I, V */
        mcr     p15, 0, r0, c1, c0, 0
        isb
        mov     r0, #0
        mcr     p15, 0, r0, c7, c5, 0           /* ICIALLU */
        mcr     p15, 0, r0, c7, c5, 6           /* BPIALL */
        mcr     p15, 0, r0, c8, c7, 0           /* TLBIALL */
        dsb
        isb
        bl      dcache_invalidate_all

        ldr     r0, =_start
        mcr     p15, 0, r0, c12, c0, 0          /* VBAR */

        cps     #0x12
        ldr     sp, =__stack_irq_top
        cps     #0x17
        ldr     sp, =__stack_abt_top
        cps     #0x1b
        ldr     sp, =__stack_und_top
        cps     #0x11
        ldr     sp, =__stack_fiq_top
        cps     #0x13
        ldr     sp, =__stack_svc_top

        ldr     r0, =__bss_start
        ldr     r1, =__bss_end
        mov     r2, #0
2:      cmp     r0, r1
        strlo   r2, [r0], #4
        blo     2b

        ldr     r0, =24000000
        mcr     p15, 0, r0, c14, c0, 0          /* CNTFRQ */

        bl      main
3:      wfi
        b       3b

/* Exceptions: save a frame {r0-r12, lr(=faulting pc + offset)} on the mode's
 * stack and hand it to C with the type; the handler never returns. */
        .macro  fault_entry name, type, adj
\name:
        sub     lr, lr, #\adj
        push    {r0-r12, lr}
        mov     r0, #\type
        mov     r1, sp
        bl      exception_handler
4:      b       4b
        .endm

        fault_entry undef_entry, 1, 4
        fault_entry svc_entry,   2, 0
        fault_entry pabt_entry,  3, 4
        fault_entry dabt_entry,  4, 8
        fault_entry fiq_entry,   5, 4

irq_entry:
        sub     lr, lr, #4
        push    {r0-r3, r12, lr}
        bl      irq_handler
        ldmfd   sp!, {r0-r3, r12, pc}^

        .global irq_enable
irq_enable:
        cpsie   i
        bx      lr
        .global irq_disable
irq_disable:
        cpsid   i
        bx      lr
        /* uint32_t irq_save(void); void irq_restore(uint32_t); */
        .global irq_save
irq_save:
        mrs     r0, cpsr
        cpsid   i
        bx      lr
        .global irq_restore
irq_restore:
        msr     cpsr_c, r0
        bx      lr
