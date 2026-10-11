/* GIC-400, secure state (boot0 jumps to us in secure SVC and nothing
 * installs a monitor). Everything is Group 0 and delivered as IRQ (FIQEn = 0). */
#include "h3.h"

#define GICD_CTLR       (H3_GICD + 0x000)
#define GICD_TYPER      (H3_GICD + 0x004)
#define GICD_IGROUPR(n) (H3_GICD + 0x080 + 4 * (n))
#define GICD_ISENABLER(n) (H3_GICD + 0x100 + 4 * (n))
#define GICD_ICENABLER(n) (H3_GICD + 0x180 + 4 * (n))
#define GICD_ICPENDR(n) (H3_GICD + 0x280 + 4 * (n))
#define GICD_IPRIORITYR(n) (H3_GICD + 0x400 + 4 * (n))
#define GICD_ITARGETSR(n) (H3_GICD + 0x800 + 4 * (n))
#define GICD_ICFGR(n)   (H3_GICD + 0xc00 + 4 * (n))
#define GICC_CTLR       (H3_GICC + 0x000)
#define GICC_PMR        (H3_GICC + 0x004)
#define GICC_BPR        (H3_GICC + 0x008)
#define GICC_IAR        (H3_GICC + 0x00c)
#define GICC_EOIR       (H3_GICC + 0x010)

#define MAX_IRQ 160
static void (*handlers[MAX_IRQ])(void);
static unsigned nirq;
static uint32_t spurious;

void gic_init(void)
{
	writel(0, GICD_CTLR);
	nirq = ((readl(GICD_TYPER) & 0x1f) + 1) * 32;
	if (nirq > MAX_IRQ)
		nirq = MAX_IRQ;
	for (unsigned i = 0; i < nirq / 32; i++) {
		writel(~0u, GICD_ICENABLER(i));
		writel(~0u, GICD_ICPENDR(i));
		writel(0, GICD_IGROUPR(i));             /* all Group 0 (secure) */
	}
	for (unsigned i = 0; i < nirq / 4; i++) {
		writel(0xa0a0a0a0u, GICD_IPRIORITYR(i));
		if (i >= 8)                             /* SPIs: cpu0 (reset value is 0 on GIC-400) */
			writel(0x01010101u, GICD_ITARGETSR(i));
	}
	for (unsigned i = 2; i < nirq / 16; i++)
		writel(0, GICD_ICFGR(i));               /* SPIs level-sensitive */
	writel(1, GICD_CTLR);                       /* enable Group 0 */
	writel(0xff, GICC_PMR);
	writel(0, GICC_BPR);
	writel(1, GICC_CTLR);                       /* Group 0 enabled, as IRQ */
}

void gic_enable(unsigned id, void (*fn)(void))
{
	if (id >= nirq)
		return;
	handlers[id] = fn;
	writel(BIT(id % 32), GICD_ISENABLER(id / 32));
}

void gic_set_handler(unsigned id, void (*fn)(void))
{
	if (id < nirq)
		handlers[id] = fn;
}

void gic_mask(unsigned id)
{
	if (id < nirq)
		writel(BIT(id % 32), GICD_ICENABLER(id / 32));
}

void gic_unmask(unsigned id)
{
	if (id < nirq)
		writel(BIT(id % 32), GICD_ISENABLER(id / 32));
}

void gic_disable(unsigned id)
{
	if (id >= nirq)
		return;
	writel(BIT(id % 32), GICD_ICENABLER(id / 32));
	handlers[id] = 0;
}

void irq_handler(void)
{
	uint32_t iar = readl(GICC_IAR);
	unsigned id = iar & 0x3ff;
	if (id >= 1020) {
		spurious++;
		return;
	}
	if (id < nirq && handlers[id])
		handlers[id]();
	writel(iar, GICC_EOIR);
}
