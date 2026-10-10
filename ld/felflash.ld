/* felflash: DRAM, 64 KiB at 0x4ff00000 for code/data/bss, stack below
 * 0x4ff80000; the command block sits at 0x4ff10000 (src/felflash/main.c)
 * and the data the Mac uploads at 0x41000000 (scripts/appliance.py). */
MEMORY
{
    IMG (rwx) : ORIGIN = 0x4ff00000, LENGTH = 64K
}
ENTRY(_start)
SECTIONS
{
    .text : { KEEP(*(.entry)) *(.text*) *(.rodata*) . = ALIGN(4); } > IMG
    .data : { . = ALIGN(4); *(.data*) . = ALIGN(4); } > IMG
    .bss (NOLOAD) : { . = ALIGN(4); __bss_start = .; *(.bss*) *(COMMON) . = ALIGN(4); __bss_end = .; } > IMG
    __stack_top = 0x4ff80000;
    /DISCARD/ : { *(.ARM.exidx*) *(.ARM.extab*) *(.comment) }
}
