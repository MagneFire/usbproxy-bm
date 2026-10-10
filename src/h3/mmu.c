/* Flat 1:1 map, 1 MiB sections, 16 KiB L1 table (TTBCR N=0: DRAM sits at
 * exactly 1 GiB, which a smaller TTBR0 window would hand to TTBR1).
 *   SRAM (0x00000000)           normal WB
 *   0x01000000..0x01ffffff      device, XN (CCU, PIO, USB, GIC, RTC)
 *   DRAM 0x40000000..0x4fffffff normal WB, except the .dma section and the
 *                               log ring, which are normal non-cacheable
 *   everything else             unmapped -> data abort (catches wild pointers) */
#include "h3.h"

extern uint32_t __mmu_l1[];
extern char __dma_start[], __dma_end[], __logring_start[], __logring_end[];

#define SEC_WB      0x1c0eu     /* TEX=001 C=1 B=1, AP=11, section */
#define SEC_NC      0x1c02u     /* TEX=001 C=0 B=0 (normal non-cacheable) */
#define SEC_DEV     0x0c16u     /* TEX=000 C=0 B=1, XN */

static void map(uint32_t from, uint32_t to, uint32_t attr)
{
	for (uint32_t a = from >> 20; a < (to >> 20); a++)
		__mmu_l1[a] = (a << 20) | attr;
}

void mmu_init(void)
{
	for (int i = 0; i < 4096; i++)
		__mmu_l1[i] = 0;
	map(0x00000000, 0x00100000, SEC_WB);
	map(0x01000000, 0x02000000, SEC_DEV);
	map(0x40000000, 0x50000000, SEC_WB);
	map((uint32_t)(uintptr_t)__dma_start, (uint32_t)(uintptr_t)__dma_end + 0xfffff, SEC_NC);
	map((uint32_t)(uintptr_t)__logring_start, (uint32_t)(uintptr_t)__logring_end + 0xfffff, SEC_NC);

	dsb();
	uint32_t ttbr = (uint32_t)(uintptr_t)__mmu_l1;     /* table walks uncached */
	__asm__ volatile("mcr p15, 0, %0, c2, c0, 2" :: "r"(0));       /* TTBCR N=0 */
	__asm__ volatile("mcr p15, 0, %0, c2, c0, 0" :: "r"(ttbr));    /* TTBR0 */
	__asm__ volatile("mcr p15, 0, %0, c3, c0, 0" :: "r"(1));       /* DACR: domain 0 client */
	__asm__ volatile("mcr p15, 0, %0, c8, c7, 0" :: "r"(0));       /* TLBIALL */
	dsb(); isb();
	uint32_t sctlr;
	__asm__ volatile("mrc p15, 0, %0, c1, c0, 0" : "=r"(sctlr));
	sctlr |= (1 << 0) | (1 << 2) | (1 << 11) | (1 << 12);          /* M, C, Z, I */
	sctlr &= ~(1 << 1);                                            /* A: allow unaligned */
	__asm__ volatile("mcr p15, 0, %0, c1, c0, 0" :: "r"(sctlr));
	isb();
}

/* D-cache maintenance by address (to the point of coherency, i.e. through
 * the A7's L2) for buffers shared with DMA engines (EHCI, musb DMA). */
#define CACHE_LINE 64u
void dcache_clean_range(const void *addr, uint32_t size)
{
	if (!size) return;
	uintptr_t a = (uintptr_t)addr & ~(CACHE_LINE - 1), e = (uintptr_t)addr + size;
	dmb();
	for (; a < e; a += CACHE_LINE)
		__asm__ volatile("mcr p15, 0, %0, c7, c10, 1" :: "r"(a));	/* DCCMVAC */
	dsb();
}
void dcache_invalidate_range(const void *addr, uint32_t size)
{
	if (!size) return;
	uintptr_t a = (uintptr_t)addr & ~(CACHE_LINE - 1), e = (uintptr_t)addr + size;
	dmb();
	for (; a < e; a += CACHE_LINE)
		__asm__ volatile("mcr p15, 0, %0, c7, c6, 1" :: "r"(a));	/* DCIMVAC */
	dsb();
}
void dcache_clean_invalidate_range(const void *addr, uint32_t size)
{
	if (!size) return;
	uintptr_t a = (uintptr_t)addr & ~(CACHE_LINE - 1), e = (uintptr_t)addr + size;
	dmb();
	for (; a < e; a += CACHE_LINE)
		__asm__ volatile("mcr p15, 0, %0, c7, c14, 1" :: "r"(a));	/* DCCIMVAC */
	dsb();
}
