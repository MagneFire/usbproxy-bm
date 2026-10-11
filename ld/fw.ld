/* The only file that knows DRAM addresses. (An SRAM-only firmware is ruled
 * out: the EHCI reaches DRAM only.) */
MEMORY
{
    IMG     (rwx) : ORIGIN = 0x40008000, LENGTH = 256K   /* uImage load + entry */
    DMA     (rw)  : ORIGIN = 0x40100000, LENGTH = 64K    /* uncached: EHCI pool, packet rings */
    LOGRING (rw)  : ORIGIN = 0x4fc00000, LENGTH = 128K   /* uncached: the log ring */
}
ENTRY(_start)
SECTIONS
{
    .text : {
        KEEP(*(.vectors))
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
    } > IMG
    .data : { . = ALIGN(4); *(.data*) . = ALIGN(4); } > IMG
    __image_end = .;
    .bss (NOLOAD) : {
        . = ALIGN(4);
        __bss_start = .;
        *(.bss*) *(COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > IMG
    .stacks (NOLOAD) : {
        . = ALIGN(16);
        . += 256;  __stack_fiq_top = .;
        . += 1024; __stack_und_top = .;
        . += 1024; __stack_abt_top = .;
        . += 4096; __stack_irq_top = .;
        . += 8192; __stack_svc_top = .;
    } > IMG
    .mmu_l1 (NOLOAD) : { . = ALIGN(16384); __mmu_l1 = .; . += 16384; } > IMG
    __ram_end = .;
    .dma (NOLOAD) : { . = ALIGN(4096); __dma_start = .; *(.dma*) . = ALIGN(4096); __dma_end = .; } > DMA
    .logring (NOLOAD) : { __logring_start = .; . += LENGTH(LOGRING); __logring_end = .; } > LOGRING
    /DISCARD/ : { *(.ARM.exidx*) *(.ARM.extab*) *(.comment) }
}
