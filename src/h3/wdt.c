/* sun6i-style watchdog at 0x01c20ca0. boot0 arms it at 8 s before the jump
 * (MODE = 0x71, CFG = system reset); a FEL `exe` comes
 * with it off. We re-arm it the same way regardless and feed it from the main
 * loop. */
#include "h3.h"

void wdt_arm(void)
{
	writel(1, WDT_CFG);                     /* whole-system reset */
	writel((7u << 4) | 1u, WDT_MODE);       /* interval 7 = 8 s, enable */
	wdt_feed();
}

void wdt_feed(void)
{
	writel(WDT_CTRL_RESTART, WDT_CTRL);
}

int wdt_armed(void)
{
	return readl(WDT_MODE) & 1;
}

void wdt_reset_now(void)
{
	uart_flush();
	irq_disable();
	writel(1, WDT_CFG);
	writel((0u << 4) | 1u, WDT_MODE);       /* interval 0 = 0.5 s */
	wdt_feed();
	for (;;)
		wfi();
}
