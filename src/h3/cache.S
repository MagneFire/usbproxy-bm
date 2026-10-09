/* ARMv7 set/way D-cache maintenance over every level in CLIDR (L1 + the A7's
 * integrated L2). Clobbers r0-r11; call before/after C code only. */
        .syntax unified
        .arm
        .text

        .macro  dcache_all op
        dmb
        mrc     p15, 1, r0, c0, c0, 1           /* CLIDR */
        ands    r3, r0, #0x7000000
        mov     r3, r3, lsr #23                 /* LoC * 2 */
        beq     5f
        mov     r10, #0
1:      add     r2, r10, r10, lsr #1            /* 3 * level */
        mov     r1, r0, lsr r2
        and     r1, r1, #7
        cmp     r1, #2
        blt     4f                              /* no D-cache at this level */
        mcr     p15, 2, r10, c0, c0, 0          /* CSSELR */
        isb
        mrc     p15, 1, r1, c0, c0, 0           /* CCSIDR */
        and     r2, r1, #7
        add     r2, r2, #4                      /* log2(line) */
        ldr     r4, =0x3ff
        ands    r4, r4, r1, lsr #3              /* ways - 1 */
        clz     r5, r4
        ldr     r7, =0x7fff
        ands    r7, r7, r1, lsr #13             /* sets - 1 */
2:      mov     r9, r4
3:      orr     r11, r10, r9, lsl r5
        orr     r11, r11, r7, lsl r2
        .ifc \op, clean_inv
        mcr     p15, 0, r11, c7, c14, 2         /* DCCISW */
        .else
        mcr     p15, 0, r11, c7, c6, 2          /* DCISW */
        .endif
        subs    r9, r9, #1
        bge     3b
        subs    r7, r7, #1
        bge     2b
4:      add     r10, r10, #2
        cmp     r3, r10
        bgt     1b
5:      mov     r10, #0
        mcr     p15, 2, r10, c0, c0, 0
        dsb
        isb
        .endm

        .global dcache_clean_invalidate_all
dcache_clean_invalidate_all:
        push    {r4-r11, lr}
        dcache_all clean_inv
        pop     {r4-r11, pc}

        .global dcache_invalidate_all
dcache_invalidate_all:
        push    {r4-r11, lr}
        dcache_all inv
        pop     {r4-r11, pc}
