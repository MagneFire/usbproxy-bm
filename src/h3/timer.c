/* ARM architected timer: CNTPCT runs at 24 MHz from power-on; CNTFRQ is set
 * by start.S. A 1 ms tick on the non-secure physical timer (PPI 30) wakes the
 * main loop out of WFI for timeouts only; stamps read the counter directly. */
#include "h3.h"

static volatile uint32_t ticks;

uint64_t cnt_now(void)
{
	uint32_t lo, hi;
	__asm__ volatile("mrrc p15, 0, %0, %1, c14" : "=r"(lo), "=r"(hi));
	return ((uint64_t)hi << 32) | lo;
}

uint32_t time_ms(void) { return (uint32_t)(cnt_now() / (CNT_HZ / 1000)); }
uint32_t time_us(void) { return (uint32_t)(cnt_now() / (CNT_HZ / 1000000)); }
uint32_t tick_count(void) { return ticks; }

void delay_us(uint32_t us)
{
	uint64_t end = cnt_now() + (uint64_t)us * (CNT_HZ / 1000000);
	while (cnt_now() < end)
		;
}

void delay_ms(uint32_t ms) { delay_us(ms * 1000); }

void (*tick_hook)(uint32_t ticks);

static void tick_irq(void)
{
	uint32_t tval = CNT_HZ / 1000;
	__asm__ volatile("mcr p15, 0, %0, c14, c2, 0" :: "r"(tval));  /* CNTP_TVAL */
	ticks++;
	if (tick_hook)
		tick_hook(ticks);
}

void timer_init(void)
{
	uint32_t tval = CNT_HZ / 1000, ctl = 1;
	__asm__ volatile("mcr p15, 0, %0, c14, c2, 0" :: "r"(tval));
	__asm__ volatile("mcr p15, 0, %0, c14, c2, 1" :: "r"(ctl));    /* CNTP_CTL: enable */
	/* In secure PL1 (no monitor, where the SPL leaves us) CNTP_* drive the
	 * secure physical timer, PPI 29; PPI 30 is the non-secure one. Enable
	 * both so the tick works whichever state a loader hands us. */
	gic_enable(IRQ_PPI_TIMER_S, tick_irq);
	gic_enable(IRQ_PPI_TIMER, tick_irq);
}
