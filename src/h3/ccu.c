#include "h3.h"

void ccu_uart0_enable(void)
{
	/* APB2 gate + reset for UART0; PA4/PA5 to function 2 (UART0). A FEL
	 * entry leaves these off. */
	setbits(CCU_BUS_GATE3, BIT(16));
	setbits(CCU_BUS_RST4, BIT(16));
	uint32_t cfg = readl(PIO_CFG(PIO_PORT(0), 0));
	cfg &= ~(0xfu << 16 | 0xfu << 20);
	cfg |= 2u << 16 | 2u << 20;
	writel(cfg, PIO_CFG(PIO_PORT(0), 0));
}

/* CPU clock: PLL1 = 24 MHz * N * K / M, reprogrammed behind the 24 MHz
 * oscillator (boot0 sets 1008; console `cpuclk <MHz>`, multiples of 48).
 * 1008 is the most the boot voltage carries; N is 5 bits. */
#define CPU_MHZ_MIN 120
#define CPU_MHZ_MAX 1008
void ccu_set_cpu_mhz(unsigned mhz)
{
	if (mhz < CPU_MHZ_MIN) mhz = CPU_MHZ_MIN;
	if (mhz > CPU_MHZ_MAX) mhz = CPU_MHZ_MAX;
	uintptr_t pll1 = H3_CCU + 0x000, cpu_axi = H3_CCU + 0x050;
	unsigned k = 4, m = 2;                       /* U-Boot's choice for > 768 MHz: 24 * N * 4 / 2 */
	if (mhz <= 768) { k = 1; m = 1; }            /* 24 * N, N <= 32 */
	unsigned n = mhz / (24 * k / m);
	writel((2u << 0) | (1u << 8) | (1u << 16), cpu_axi);        /* AXI/3, ATB/2, OSC24M */
	writel((1u << 31) | ((n - 1) << 8) | ((k - 1) << 4) | (m - 1), pll1);
	delay_us(200);
	writel((2u << 0) | (1u << 8) | (2u << 16), cpu_axi);        /* PLL1 */
}

unsigned ccu_cpu_mhz(void)
{
	uint32_t v = readl(H3_CCU + 0x000);
	unsigned n = ((v >> 8) & 0x1f) + 1, k = ((v >> 4) & 3) + 1, m = (v & 3) + 1, p = (v >> 16) & 3;
	return (24 * n * k / m) >> p;
}

/* Bus clocks for the two states of the bridge: a device bound = full rates
 * (AHB1 = PLL6/3 = 200 MHz, MBUS = PLL6/4 = 150 MHz), nothing bound = half
 * AHB1, half MBUS. */
void ccu_set_bus_idle(bool idle)
{
	writel(idle ? 0x00003190u : 0x00003180u, H3_CCU + 0x054);   /* AHB1 /2 or /1 */
	writel(idle ? 0x81000007u : 0x81000003u, H3_CCU + 0x15c);   /* MBUS PLL6/8 or /4 */
}
