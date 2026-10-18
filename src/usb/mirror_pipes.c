/* Placeholder: the pipes arrive with the next step; a mirrored device's
 * bulk and interrupt endpoints are opened on both sides but carry nothing. */
#include "fw.h"
#include "mirror_pipes.h"

static unsigned opened;

void mirror_pipes_gadget_open(uint8_t gadget_ep, uint8_t dev_ep, uint8_t xfer_type, uint16_t mps)
{
	(void)xfer_type; (void)mps;
	opened++;
	log_printf("mirror: gadget ep %02x <- device ep %02x (no pipe yet)\n", gadget_ep, dev_ep);
}
void mirror_pipes_gadget_reset(void) { opened = 0; }
bool mirror_pipes_gadget_xfer(uint8_t gadget_ep, xfer_result_t result, uint32_t len)
{
	(void)gadget_ep; (void)result; (void)len;
	return true;
}
void mirror_pipes_gadget_halt_cleared(uint8_t gadget_ep) { (void)gadget_ep; }
void mirror_pipes_poll(void) {}
bool mirror_pipes_host_pending(void) { return false; }
void mirror_pipes_status(char *buf, unsigned n) { snprintf(buf, n, "pipes=%u open, none served\n", opened); }
