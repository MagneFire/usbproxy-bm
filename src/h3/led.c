/* Orange Pi Zero: green status LED on PA17, red power LED on PL10, both
 * active high. */
#include "h3.h"

void led_init(void)
{
	uint32_t v = readl(PIO_CFG(PIO_PORT(0), 2));            /* PA16..23 */
	v = (v & ~(0xfu << 4)) | (1u << 4);                     /* PA17 output */
	writel(v, PIO_CFG(PIO_PORT(0), 2));
	v = readl(PIO_CFG(H3_R_PIO, 1));                        /* PL8..15 */
	v = (v & ~(0xfu << 8)) | (1u << 8);                     /* PL10 output */
	writel(v, PIO_CFG(H3_R_PIO, 1));
}

void led_green(int on)
{
	if (on) setbits(PIO_DAT(PIO_PORT(0)), BIT(17)); else clrbits(PIO_DAT(PIO_PORT(0)), BIT(17));
}

void led_red(int on)
{
	if (on) setbits(PIO_DAT(H3_R_PIO), BIT(10)); else clrbits(PIO_DAT(H3_R_PIO), BIT(10));
}
