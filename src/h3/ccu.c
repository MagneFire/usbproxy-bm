#include "h3.h"

void ccu_uart0_enable(void)
{
	/* APB2 gate + reset for UART0; PA4/PA5 to function 2 (UART0). The SPL
	 * already did all of this, but a FEL entry has not. */
	setbits(CCU_BUS_GATE3, BIT(16));
	setbits(CCU_BUS_RST4, BIT(16));
	uint32_t cfg = readl(PIO_CFG(PIO_PORT(0), 0));
	cfg &= ~(0xfu << 16 | 0xfu << 20);
	cfg |= 2u << 16 | 2u << 20;
	writel(cfg, PIO_CFG(PIO_PORT(0), 0));
}
