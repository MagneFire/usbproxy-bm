/* TinyUSB HCD glue for the H3's EHCI1 (the USB-A port, the watch cradle):
 * PHY1 bring-up, controller reset, interrupt threshold 1 microframe (the
 * default 8 = 1 ms would put a millisecond on every completion),
 * CONFIGFLAG so the port routes to EHCI, GIC wiring. The EHCI core itself
 * is ehci_h3.c (TinyUSB's ehci.c forked). */
#include "fw.h"
#include "tusb.h"
#include "host/hcd.h"
#include "ehci_api.h"
#include "usbphy.h"

#define EHCI_CAP        (H3_EHCI1 + 0x00)
#define EHCI_OP         (H3_EHCI1 + 0x10)
#define EHCI_USBCMD     (EHCI_OP + 0x00)
#define EHCI_USBSTS     (EHCI_OP + 0x04)
#define EHCI_CONFIGFLAG (EHCI_OP + 0x40)
#define EHCI_PORTSC     (EHCI_OP + 0x44)

/* D-cache maintenance for buffers the EHCI reads/writes (ehci_h3.c calls
 * these before an OUT, before and after an IN). Addresses inside the
 * uncached .dma section (QH/qTD pool, enumeration buffers) need nothing.
 * MVA ops reach the point of coherency, i.e. through the A7's L2. */
extern char __dma_start[], __dma_end[];
#define CACHE_LINE 64u

static inline bool uncached(const void *addr)
{
	return (const char *)addr >= __dma_start && (const char *)addr < __dma_end;
}

bool hcd_dcache_clean(void const *addr, uint32_t size)
{
	if (!size || uncached(addr)) return true;
	uintptr_t a = (uintptr_t)addr & ~(CACHE_LINE - 1), e = (uintptr_t)addr + size;
	dmb();
	for (; a < e; a += CACHE_LINE)
		__asm__ volatile("mcr p15, 0, %0, c7, c10, 1" :: "r"(a));	/* DCCMVAC */
	dsb();
	return true;
}

bool hcd_dcache_invalidate(void const *addr, uint32_t size)
{
	if (!size || uncached(addr)) return true;
	uintptr_t a = (uintptr_t)addr & ~(CACHE_LINE - 1), e = (uintptr_t)addr + size;
	dmb();
	for (; a < e; a += CACHE_LINE)
		__asm__ volatile("mcr p15, 0, %0, c7, c6, 1" :: "r"(a));	/* DCIMVAC */
	dsb();
	return true;
}

bool hcd_dcache_clean_invalidate(void const *addr, uint32_t size)
{
	if (!size || uncached(addr)) return true;
	uintptr_t a = (uintptr_t)addr & ~(CACHE_LINE - 1), e = (uintptr_t)addr + size;
	dmb();
	for (; a < e; a += CACHE_LINE)
		__asm__ volatile("mcr p15, 0, %0, c7, c14, 1" :: "r"(a));	/* DCCIMVAC */
	dsb();
	return true;
}

static volatile uint32_t isr_count;
static void ehci_irq(void)
{
	isr_count++;
	hcd_int_handler(1, true);
}
uint32_t hcd_h3_isr_count(void) { return isr_count; }

void ehci_h3_poll_async(uint8_t rhport);
void hcd_h3_poll(void)
{
	gic_mask(IRQ_EHCI1);
	ehci_h3_poll_async(1);
	gic_unmask(IRQ_EHCI1);
}

bool hcd_init(uint8_t rhport, const tusb_rhport_init_t *rh_init)
{
	(void)rh_init;
	usbphy1_init();

	/* Halt, then reset the controller (EHCI 2.3.1). */
	clrbits(EHCI_USBCMD, 1);
	for (int i = 0; i < 10000 && !(readl(EHCI_USBSTS) & BIT(12)); i++)
		delay_us(1);
	setbits(EHCI_USBCMD, BIT(1));
	for (int i = 0; i < 10000 && (readl(EHCI_USBCMD) & BIT(1)); i++)
		delay_us(1);

	/* Interrupt Threshold Control = 1 microframe, while halted. ehci_init()
	 * ORs RUN/schedule enables into USBCMD and leaves this field alone. */
	uint32_t cmd = readl(EHCI_USBCMD);
	cmd = (cmd & ~(0xffu << 16)) | (1u << 16);
	writel(cmd, EHCI_USBCMD);

	if (!ehci_init(rhport, EHCI_CAP, EHCI_OP))
		return false;
	writel(1, EHCI_CONFIGFLAG);             /* ports to EHCI, not the companion */
	setbits(EHCI_PORTSC, BIT(12));          /* port power (PPC or not, harmless) */
	gic_set_handler(IRQ_EHCI1, ehci_irq);
	log_printf("ehci1: up, usbcmd=%08lx usbsts=%08lx hcsparams=%08lx portsc=%08lx\n",
		   (unsigned long)readl(EHCI_USBCMD), (unsigned long)readl(EHCI_USBSTS),
		   (unsigned long)readl(EHCI_CAP + 0x04), (unsigned long)readl(EHCI_PORTSC));
	return true;
}

bool hcd_deinit(uint8_t rhport)
{
	gic_disable(IRQ_EHCI1);
	return ehci_deinit(rhport);
}

bool hcd_configure(uint8_t rhport, uint32_t cfg_id, const void *cfg_param)
{
	(void)rhport; (void)cfg_id; (void)cfg_param;
	return false;
}

void hcd_int_enable(uint8_t rhport) { (void)rhport; gic_unmask(IRQ_EHCI1); }
void hcd_int_disable(uint8_t rhport) { (void)rhport; gic_mask(IRQ_EHCI1); }

/* Console `ehci`: controller registers and the async ring. */
void hcd_h3_dump(void (*out)(const char *))
{
	char b[160];
	uint32_t base = EHCI_OP;
	snprintf(b, sizeof b, "cmd=%08lx sts=%08lx intr=%08lx frindex=%08lx async=%08lx portsc=%08lx irqs=%lu\n",
		 (unsigned long)readl(base + 0), (unsigned long)readl(base + 4), (unsigned long)readl(base + 8),
		 (unsigned long)readl(base + 0xc), (unsigned long)readl(base + 0x18),
		 (unsigned long)readl(EHCI_PORTSC), (unsigned long)isr_count);
	out(b);
	uint32_t head = readl(base + 0x18) & ~0x1fu, qh = head;
	for (int i = 0; i < 8 && qh >= 0x40000000 && qh < 0x50000000; i++) {
		const volatile uint32_t *q = (const volatile uint32_t *)(uintptr_t)qh;
		uint32_t ep = q[1];
		snprintf(b, sizeof b, "qh@%08lx daddr=%lu ep=%lu mps=%lu H=%lu cur=%08lx next=%08lx tok=%08lx (act=%lu halt=%lu err=%lu len=%lu pid=%lu)\n",
			 (unsigned long)qh, (unsigned long)(ep & 0x7f), (unsigned long)((ep >> 8) & 0xf),
			 (unsigned long)((ep >> 16) & 0x7ff), (unsigned long)((ep >> 15) & 1),
			 (unsigned long)q[3], (unsigned long)q[4], (unsigned long)q[6],
			 (unsigned long)((q[6] >> 7) & 1), (unsigned long)((q[6] >> 6) & 1),
			 (unsigned long)((q[6] >> 10) & 3), (unsigned long)((q[6] >> 16) & 0x7fff),
			 (unsigned long)((q[6] >> 8) & 3));
		out(b);
		uint32_t next = q[4];
		if (!(next & 1)) {
			const volatile uint32_t *t = (const volatile uint32_t *)(uintptr_t)(next & ~0x1fu);
			snprintf(b, sizeof b, "   qtd@%08lx tok=%08lx (act=%lu halt=%lu len=%lu pid=%lu dt=%lu ioc=%lu) buf0=%08lx\n",
				 (unsigned long)(next & ~0x1fu), (unsigned long)t[2], (unsigned long)((t[2] >> 7) & 1),
				 (unsigned long)((t[2] >> 6) & 1), (unsigned long)((t[2] >> 16) & 0x7fff),
				 (unsigned long)((t[2] >> 8) & 3), (unsigned long)((t[2] >> 31) & 1),
				 (unsigned long)((t[2] >> 15) & 1), (unsigned long)t[3]);
			out(b);
		}
		qh = q[0] & ~0x1fu;
		if (qh == head) break;
	}
}

uint32_t hcd_h3_portsc(void) { return readl(EHCI_PORTSC); }
void hcd_h3_portsc_write(uint32_t v) { writel(v, EHCI_PORTSC); }
