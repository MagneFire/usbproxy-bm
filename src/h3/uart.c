/* UART0, a DesignWare 16550 at 24 MHz APB2: 115200 8N1, polled both ways.
 * The console the dongle sees on the stock Zero's 3-pin header. */
#include "h3.h"

#define UART_RBR  (H3_UART0 + 0x00)
#define UART_THR  (H3_UART0 + 0x00)
#define UART_DLL  (H3_UART0 + 0x00)
#define UART_DLH  (H3_UART0 + 0x04)
#define UART_IER  (H3_UART0 + 0x04)
#define UART_FCR  (H3_UART0 + 0x08)
#define UART_LCR  (H3_UART0 + 0x0c)
#define UART_LSR  (H3_UART0 + 0x14)
#define UART_USR  (H3_UART0 + 0x7c)

void uart_init(void)
{
	ccu_uart0_enable();
	/* Wait for the DW UART to go idle before touching LCR, else the write is
	 * ignored (USR.busy). The SPL/U-Boot left it configured, so this is
	 * mostly a no-op re-init at the same rate. */
	for (int i = 0; i < 100000 && (readl(UART_USR) & 1); i++)
		;
	writel(0, UART_IER);
	writel(0x80, UART_LCR);
	writel(13, UART_DLL);           /* 24 MHz / (16 * 115200) = 13.02 */
	writel(0, UART_DLH);
	writel(0x03, UART_LCR);         /* 8N1 */
	writel(0x07, UART_FCR);         /* FIFOs on, both reset */
}

void uart_putc(char c)
{
	if (c == '\n')
		uart_putc('\r');
	while (!(readl(UART_LSR) & 0x20))
		;
	writel((uint8_t)c, UART_THR);
}

void uart_puts(const char *s)
{
	while (*s)
		uart_putc(*s++);
}

int uart_getc(void)
{
	if (!(readl(UART_LSR) & 0x01))
		return -1;
	return (int)(readl(UART_RBR) & 0xff);
}

void uart_flush(void)
{
	while (!(readl(UART_LSR) & 0x40))   /* TEMT */
		;
}
