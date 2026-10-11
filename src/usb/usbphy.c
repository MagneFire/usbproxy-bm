/* H3 USB PHYs. PHY0 sits between the OTG connector and either musb or
 * EHCI0/OHCI0 (OTGCTL routes it; we want musb). PHY1 feeds EHCI1/OHCI1, the
 * USB-A port the watch cradle is on. Register facts: BARE-METAL.md section 3.
 * Sequence per PHY mirrors the kernel: bus reset/gate for the controller,
 * PHY reset deassert + clock, SIDDQ clear in the HCI PMU block, then the
 * PHYCTL bit-bang tuning (45 ohm cal on PHY0, TX amplitude 0x14, disconnect
 * threshold 3). */
#include "fw.h"
#include "usbphy.h"

#define PHY_ISCR    (H3_USBPHY + 0x00)
#define PHY_PHYCTL  (H3_USBPHY + 0x10)
#define PHY_OTGCTL  (H3_USBPHY + 0x20)
#define PMU0_BASE   0x01c1a800u
#define PMU1_BASE   0x01c1b800u
#define PMU_ICR     0x00
#define PMU_PHYCTL  0x10
#define SIDDQ       BIT(1)

/* Linux sun4i_usb_phy_write for the A33-style PHYCTL: zero the register
 * first, then clock each data bit in on the PHY's own bit (index * 2). */
static void phy_write(unsigned index, uint32_t addr, uint32_t data, unsigned len)
{
	uint32_t bit = 1u << (index * 2);
	writel(0, PHY_PHYCTL);
	for (unsigned j = 0; j < len; j++) {
		uint32_t v = readl(PHY_PHYCTL);
		v &= ~(0xffu << 8);
		v |= (addr + j) << 8;
		writel(v, PHY_PHYCTL);
		clrbits(PHY_PHYCTL, bit);
		if (data & 1) setbits(PHY_PHYCTL, BIT(7)); else clrbits(PHY_PHYCTL, BIT(7));
		setbits(PHY_PHYCTL, bit);
		clrbits(PHY_PHYCTL, bit);
		data >>= 1;
	}
}

static void phy_tune(unsigned index)
{
	if (index == 0)
		phy_write(index, 0x0c, 0x01, 1);    /* PHY_RES45_CAL_EN */
	phy_write(index, 0x20, 0x14, 5);        /* PHY_TX_AMPLITUDE_TUNE */
	phy_write(index, 0x2a, 3, 2);           /* PHY_DISCON_TH_SEL */
}

void usbphy0_init(void)
{
	/* OTG controller bus reset + gate; EHCI0/OHCI0 too, so the PMU0 block
	 * (SIDDQ) is reachable. Everything off first, as uli's H3 bring-up does
	 * ("helps with devices not connecting on boot"). */
	clrbits(CCU_BUS_RST0, BIT(23) | BIT(24) | BIT(28));
	clrbits(CCU_BUS_GATE0, BIT(23) | BIT(24) | BIT(28));
	clrbits(CCU_USBPHY_CFG, BIT(0) | BIT(8));
	delay_ms(2);
	setbits(CCU_USBPHY_CFG, BIT(8));        /* PHY0 clock */
	setbits(CCU_USBPHY_CFG, BIT(0));        /* PHY0 reset deassert */
	setbits(CCU_BUS_GATE0, BIT(23) | BIT(24) | BIT(28));
	setbits(CCU_BUS_RST0, BIT(23) | BIT(24) | BIT(28));
	delay_ms(1);
	clrbits(PMU0_BASE + PMU_PHYCTL, SIDDQ);
	setbits(PHY_OTGCTL, BIT(0));            /* route PHY0 to musb (phy0_dual_route) */
	phy_tune(0);
	/* PHY0's host side is never used: gates off again (verified: the gadget
	 * enumerates and runs with them off, 2026-10-11). */
	clrbits(CCU_BUS_GATE0, BIT(24) | BIT(28));
}

void usbphy1_init(void)
{
	clrbits(CCU_BUS_RST0, BIT(25) | BIT(29));
	clrbits(CCU_BUS_GATE0, BIT(25) | BIT(29));
	clrbits(CCU_USBPHY_CFG, BIT(1) | BIT(9) | BIT(17));
	delay_ms(10);
	setbits(CCU_USBPHY_CFG, BIT(9) | BIT(17));   /* PHY1 clock, OHCI1 12 MHz */
	setbits(CCU_USBPHY_CFG, BIT(1));             /* PHY1 reset deassert */
	setbits(CCU_BUS_GATE0, BIT(25) | BIT(29));
	setbits(CCU_BUS_RST0, BIT(25) | BIT(29));
	delay_ms(1);
	clrbits(PMU1_BASE + PMU_PHYCTL, SIDDQ);
	writel(0x701, PMU1_BASE + PMU_ICR);          /* INCR8/INCR4/INCRX align + ULPI bypass */
	phy_tune(1);
}

/* Power measurement: stop EHCI1, gate EHCI1/OHCI1 and PHY1, SIDDQ on. Only
 * meaningful with nothing bound (the progress gate would otherwise reset the
 * board); a reboot brings the host side back. */
void usbphy1_off(void)
{
	clrbits(H3_EHCI1 + 0x10, BIT(0));            /* USBCMD.RS */
	delay_ms(2);
	setbits(PMU1_BASE + PMU_PHYCTL, SIDDQ);
	clrbits(CCU_BUS_GATE0, BIT(25) | BIT(29));
	clrbits(CCU_BUS_RST0, BIT(25) | BIT(29));
	clrbits(CCU_USBPHY_CFG, BIT(1) | BIT(9) | BIT(17));
}
