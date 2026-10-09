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

static void ehci_irq(void) { hcd_int_handler(1, true); }

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
	log_printf("ehci1: up, usbcmd=%08lx hcsparams=%08lx\n",
		   (unsigned long)readl(EHCI_USBCMD), (unsigned long)readl(EHCI_CAP + 0x04));
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

uint32_t hcd_h3_portsc(void) { return readl(EHCI_PORTSC); }
void hcd_h3_portsc_write(uint32_t v) { writel(v, EHCI_PORTSC); }
