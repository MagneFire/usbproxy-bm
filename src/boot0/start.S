/* boot0: the first stage the H3 BROM loads from SD sector 16 into SRAM A1 at
 * 0x0 (<= 24 KiB, eGON.BT0 header, checksum by tools/egon.py). Replaces the
 * U-Boot SPL. Two ways in:
 *   card boot : the BROM jumps to word 0 with the header's magic intact
 *   FEL       : sunxi-fel `spl` loads us the same way but patches the magic;
 *               we must return to the BROM (fel_stash) once DRAM is up.
 * The C side (main.c) tells them apart by the magic at 0x4. */
        .syntax unified
        .arm

        .section .egon, "ax"
        .global _start
_start:
        b       boot0_reset
        .ascii  "eGON.BT0"
        .word   0                       /* 0x0c check_sum (egon.py) */
        .word   0                       /* 0x10 length    (egon.py) */
        .ascii  "SPL"                   /* 0x14 "sunxi" SPL header marker */
        .byte   2                       /*      version 0.2 */
        .word   0                       /* 0x18 fel_script_address (sunxi-fel) */
        .word   0                       /* 0x1c fel_uEnv_length    (sunxi-fel) */
        .word   0                       /* 0x20 dt_name_offset */
        .global egon_dram_size
egon_dram_size:
        .word   0                       /* 0x24 dram_size, MiB: we fill it for sunxi-fel */
        .word   0                       /* 0x28 boot_media (BROM) */
        .space  13 * 4                  /* 0x2c string pool, header = 0x60 bytes */

        .text
boot0_reset:
        /* Stash what a return to FEL needs (U-Boot's fel_utils.S). */
        ldr     r0, =fel_stash
        str     sp, [r0, #0]
        str     lr, [r0, #4]
        mrs     r1, cpsr
        str     r1, [r0, #8]
        mrc     p15, 0, r1, c1, c0, 0
        str     r1, [r0, #12]
        mrc     p15, 0, r1, c12, c0, 0
        str     r1, [r0, #16]
        cpsid   if

        /* FEL request (firmware console `fel`, U-Boot patch 0004 mechanism):
         * RTC GP1 == 0xb0010fe1 -> clear it, clear SCTLR.A (sunxi-fel checks
         * the BROM's SCTLR), jump to the BROM's FEL routine. Before anything
         * else so a wedged clock/DRAM state cannot block recovery. */
        ldr     r0, =0x01f00104
        ldr     r1, [r0]
        ldr     r2, =0xb0010fe1
        cmp     r1, r2
        bne     1f
        mov     r1, #0
        str     r1, [r0]
        bl      boot0_fel_entry
1:
        /* (ACTLR.SMP is the firmware's business: writing it here, with the
         * BROM's I-cache and sunxi-fel's MMU live, wedged the FEL session.) */
        /* Known CPU state: clean+invalidate the D-cache (a FEL load may have
         * written us through sunxi-fel's MMU mapping), then MMU off, D-cache
         * off, alignment check off, I-cache + branch prediction on. */
        bl      dcache_clean_invalidate_all
        mrc     p15, 0, r0, c1, c0, 0
        bic     r0, r0, #(1 << 0) | (1 << 1) | (1 << 2)     /* M, A, C */
        orr     r0, r0, #(1 << 11) | (1 << 12)              /* Z, I */
        mcr     p15, 0, r0, c1, c0, 0
        isb
        mov     r0, #0
        mcr     p15, 0, r0, c7, c5, 0           /* ICIALLU */
        mcr     p15, 0, r0, c7, c5, 6           /* BPIALL */
        mcr     p15, 0, r0, c8, c7, 0           /* TLBIALL */
        dsb
        isb

        /* Stack and .bss live in SRAM A2 (0x44000): free in both boot modes,
         * clear of the BROM's FEL areas and of sunxi-fel's buffers in A1. */
        ldr     sp, =__stack_top
        ldr     r0, =__bss_start
        ldr     r1, =__bss_end
        mov     r2, #0
2:      cmp     r0, r1
        strlo   r2, [r0], #4
        blo     2b

        bl      boot0_main
3:      b       3b

/* void boot0_fel_entry(void): hand the CPU to the BROM FEL routine. */
        .global boot0_fel_entry
boot0_fel_entry:
        mrc     p15, 0, r1, c1, c0, 0
        bic     r1, r1, #(1 << 1)               /* A */
        mcr     p15, 0, r1, c1, c0, 0
        isb
        ldr     r0, =0xffff0020
        bx      r0

/* void return_to_fel(uint32_t sp, uint32_t lr): back into sunxi-fel's thunk
 * with the BROM's VBAR/SCTLR/CPSR restored (U-Boot fel_utils.S). */
        .global return_to_fel
return_to_fel:
        mov     sp, r0
        mov     lr, r1
        ldr     r0, =fel_stash
        ldr     r1, [r0, #16]
        mcr     p15, 0, r1, c12, c0, 0          /* VBAR */
        ldr     r1, [r0, #12]
        mcr     p15, 0, r1, c1, c0, 0           /* SCTLR */
        ldr     r1, [r0, #8]
        msr     cpsr, r1
        bx      lr

/* void boot0_jump(uint32_t entry): into the firmware with r0-r2 = 0, the
 * D-cache off and clean, the I-cache invalidated (fresh text in DRAM). */
        .global boot0_jump
boot0_jump:
        mov     r4, r0
        dsb
        mov     r0, #0
        mcr     p15, 0, r0, c7, c5, 0           /* ICIALLU */
        mcr     p15, 0, r0, c7, c5, 6           /* BPIALL */
        dsb
        isb
        mov     r1, #0
        mov     r2, #0
        bx      r4

        .global irq_enable
irq_enable:
        cpsie   i
        bx      lr
        .global irq_disable
irq_disable:
        cpsid   i
        bx      lr
