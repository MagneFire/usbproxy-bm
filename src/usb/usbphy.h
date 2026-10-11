#ifndef USBPHY_H
#define USBPHY_H
/* H3 USB PHY / clock bring-up (kernel phy-sun4i-usb.c + sunxi musb glue). */
void usbphy0_init(void);        /* PHY0 routed to musb, peripheral */
void usbphy1_init(void);        /* PHY1 for EHCI1/OHCI1 (the watch port) */
void usbphy1_off(void);         /* EHCI1/OHCI1 and PHY1 off */
#endif
