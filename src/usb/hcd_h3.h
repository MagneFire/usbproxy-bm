#ifndef HCD_H3_H
#define HCD_H3_H
/* The USB-A port's host side (hcd_h3.c): EHCI1 and its OHCI1 companion
 * behind one TinyUSB rhport. What the rest of the firmware reaches. */
#include <stdbool.h>
#include <stdint.h>
#include "h3/h3.h"

#define EHCI1_USBCMD    (H3_EHCI1 + 0x10)
#define EHCI1_FRINDEX   (H3_EHCI1 + 0x1c)

enum hcd_h3_ctrl { HCD_H3_EHCI = 0, HCD_H3_OHCI = 1 };
struct hcd_h3_port { bool connected, enabled, low_speed; enum hcd_h3_ctrl owner; };

enum hcd_h3_ctrl hcd_h3_owner(void);            /* who has the port */
void hcd_h3_port_status(struct hcd_h3_port *p); /* the owner's view of the port */
uint32_t hcd_h3_frame_counter(void);            /* the owner's frame counter (progress gate) */
void hcd_h3_reclaim(void);                      /* the OHCI's device back to the EHCI for another reset */
void hcd_h3_status_line(char *buf, unsigned n); /* console `status` */
void hcd_h3_halt_owner(void);                   /* console `hangusb` */
uint32_t hcd_h3_portsc(void);                   /* EHCI PORTSC */
void hcd_h3_portsc_write(uint32_t v);
void hcd_h3_poll(void);                         /* completion polling, main loop */
void hcd_h3_dump(void (*out)(const char *));    /* console `ehci` */
#endif
