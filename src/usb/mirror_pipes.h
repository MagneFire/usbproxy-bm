#ifndef MIRROR_PIPES_H
#define MIRROR_PIPES_H
#include <stdbool.h>
#include <stdint.h>
#include "tusb.h"
/* The mirror's data endpoints: one pipe per gadget endpoint, bulk or
 * interrupt, moving packets between it and the device's endpoint behind
 * it. The gadget side opens pipes at SET_CONFIGURATION; the device side
 * is the mirror's device, its endpoints opened by the host driver. */
void mirror_pipes_gadget_open(uint8_t gadget_ep, uint8_t dev_ep, uint8_t xfer_type, uint16_t mps);
void mirror_pipes_gadget_reset(void);		/* bus reset or unconfigure: pipes closed */
bool mirror_pipes_gadget_xfer(uint8_t gadget_ep, xfer_result_t result, uint32_t len);
void mirror_pipes_gadget_halt_cleared(uint8_t gadget_ep);
void mirror_pipes_poll(void);
bool mirror_pipes_host_pending(void);		/* a device transfer is in flight: poll, do not sleep */
void mirror_pipes_status(char *buf, unsigned n);
#endif
