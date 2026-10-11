/* H3 clock tree as the U-Boot SPL leaves it (clock_sun6i.c clock_init_safe +
 * clock_set_pll1(CONFIG_SYS_CLK_FREQ) + clock_init_uart), nothing more:
 *   PLL1  1008 MHz -> CPU (AXI /3, ATB /2), switched via the 24 MHz oscillator
 *   PLL6   600 MHz -> AHB1 = PLL6/3 = 200 MHz, APB1 = AHB1/2, MBUS = PLL6/4
 *   APB2   24 MHz (UART0)
 * PLL5 (DRAM) is dram.c's. The USB gates/PHY clocks are the firmware's. */
#include "boot0/boot0.h"

#define CCU_PLL1_CFG        (H3_CCU + 0x000)
#define CCU_PLL6_CFG        (H3_CCU + 0x028)
#define CCU_CPU_AXI_CFG     (H3_CCU + 0x050)
#define CCU_AHB1_APB1_DIV   (H3_CCU + 0x054)
#define CCU_APB2_DIV        (H3_CCU + 0x058)

#define CPU_CLK_HZ          1008000000u     /* the SPL's default; 816 is the next knob */

void clock_init(void)
{
	/* CPU off the PLL while it is reprogrammed: AXI = CPU/3, ATB = /2, OSC24M */
	writel((2u << 0) | (1u << 8) | (1u << 16), CCU_CPU_AXI_CFG);
	/* PLL1 = 24 MHz * N * K / M (P = 0); U-Boot's choice for > 768 MHz is K=4, M=2 */
	unsigned k = 4, m = 2, n = CPU_CLK_HZ / (24000000u * k / m);
	writel((1u << 31) | ((n - 1) << 8) | ((k - 1) << 4) | ((m - 1) << 0), CCU_PLL1_CFG);
	delay_us(200);
	writel((2u << 0) | (1u << 8) | (2u << 16), CCU_CPU_AXI_CFG);     /* source = PLL1 */

	writel(0x90041811u, CCU_PLL6_CFG);          /* 600 MHz, PLL6_CFG_DEFAULT */
	while (!(readl(CCU_PLL6_CFG) & (1u << 28)))
		;
	writel(0x00003180u, CCU_AHB1_APB1_DIV);     /* AHB1 = PLL6/3, APB1 = AHB1/2 */
	writel(0x81000003u, CCU_MBUS0_CLK);         /* MBUS = PLL6/4, gate on */
	writel(1u << 24, CCU_APB2_DIV);             /* APB2 = OSC24M /1 /1 */
}
