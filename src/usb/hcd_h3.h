#ifndef HCD_H3_H
#define HCD_H3_H
/* EHCI1 glue (hcd_h3_ehci.c): what the rest of the firmware reaches. */
#include <stdint.h>
#include "h3/h3.h"

#define EHCI1_USBCMD    (H3_EHCI1 + 0x10)
#define EHCI1_FRINDEX   (H3_EHCI1 + 0x1c)

uint32_t hcd_h3_portsc(void);
void hcd_h3_portsc_write(uint32_t v);
void hcd_h3_poll(void);                         /* completion polling, main loop */
void hcd_h3_dump(void (*out)(const char *));    /* console `ehci` */
#endif
