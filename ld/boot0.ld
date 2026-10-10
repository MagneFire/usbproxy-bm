/* boot0: eGON image in SRAM A1 at 0 (the BROM loads <= 32 KiB; 24 KiB is
 * the proven SPL size on this SoC and the limit kept here), .bss and the
 * stack in SRAM A2 (0x44000, 32 KiB): untouched by the BROM in either boot
 * mode and clear of sunxi-fel's thunk/swap buffers at 0xA200-0xC000. */
MEMORY
{
    A1 (rwx) : ORIGIN = 0x00000000, LENGTH = 24K
    A2 (rw)  : ORIGIN = 0x00044000, LENGTH = 32K
}
ENTRY(_start)
SECTIONS
{
    .text : {
        KEEP(*(.egon))
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
    } > A1
    .data : { . = ALIGN(4); *(.data*) . = ALIGN(4); } > A1
    __image_end = .;
    .felstash (NOLOAD) : { fel_stash = .; . += 32; } > A2
    .bss (NOLOAD) : {
        . = ALIGN(4);
        __bss_start = .;
        *(.bss*) *(COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > A2
    __stack_top = ORIGIN(A2) + LENGTH(A2);
    /DISCARD/ : { *(.ARM.exidx*) *(.ARM.extab*) *(.comment) }
}
ASSERT(__image_end <= 0x6000, "boot0 exceeds 24 KiB")
ASSERT(__bss_end <= __stack_top - 4096, "boot0 .bss leaves no stack")
