#include <string.h>
#include "fw.h"
#include "tusb.h"
#include "device/usbd_pvt.h"
#include "gadget_slots.h"
#include "bridge/bridge.h"

struct slot {
	uint8_t ep_in, ep_out;
	struct slot_stats st;
};
static struct slot slots[SLOT_COUNT];

static void slots_init(void) { memset(slots, 0, sizeof slots); }
static bool slots_deinit(void) { return true; }

static void slots_reset(uint8_t rhport)
{
	(void)rhport;
	for (int i = 0; i < SLOT_COUNT; i++) {
		if (slots[i].st.open)
			bridge_gadget_gone(i);
		slots[i].st.open = false;
	}
}

static uint16_t slots_open(uint8_t rhport, tusb_desc_interface_t const *itf, uint16_t max_len)
{
	if (itf->bInterfaceClass != 0xff || itf->bInterfaceSubClass != 0x42 || itf->bNumEndpoints != 2)
		return 0;
	int idx = itf->bInterfaceProtocol == 0x01 ? SLOT_ADB :
		  itf->bInterfaceProtocol == 0x03 ? SLOT_FASTBOOT : -1;
	if (idx < 0)
		return 0;
	uint16_t need = sizeof(tusb_desc_interface_t) + 2 * sizeof(tusb_desc_endpoint_t);
	if (max_len < need)
		return 0;
	struct slot *s = &slots[idx];
	const uint8_t *p = (const uint8_t *)itf + sizeof(tusb_desc_interface_t);
	for (int k = 0; k < 2; k++) {
		const tusb_desc_endpoint_t *ep = (const tusb_desc_endpoint_t *)p;
		if (!usbd_edpt_open(rhport, ep))
			return 0;
		if (ep->bEndpointAddress & 0x80) s->ep_in = ep->bEndpointAddress;
		else s->ep_out = ep->bEndpointAddress;
		p += sizeof(tusb_desc_endpoint_t);
	}
	s->st.open = true;
	bridge_gadget_ready(idx, s->ep_in, s->ep_out);
	log_printf("slot %d open: in %02x out %02x\n", idx, s->ep_in, s->ep_out);
	return need;
}

static bool slots_control(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req)
{
	/* Standard endpoint requests are forwarded here after usbd acted on them.
	 * During the adb unbind halt pulse the host's CLEAR_FEATURE(HALT) must
	 * not end the pulse: macOS adb clears the stall once and retries the
	 * read; only a second STALL makes it drop its transport, which is the
	 * point of the pulse (then it re-opens and sends a CNXN the idle sink
	 * captures). Re-stall until the pulse ends. */
	if (stage == CONTROL_STAGE_SETUP && req->bmRequestType_bit.recipient == TUSB_REQ_RCPT_ENDPOINT &&
	    req->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD && req->bRequest == TUSB_REQ_CLEAR_FEATURE &&
	    req->wValue == TUSB_REQ_FEATURE_EDPT_HALT) {
		uint8_t ep = (uint8_t)req->wIndex;
		if (ep == slots[0].ep_in && bridge_adb_pulsing()) {
			usbd_edpt_stall(rhport, ep);
			log_printf("slots: host cleared halt on EP%02x mid-pulse; re-stalled\n", ep);
			return true;
		}
		for (int i = 0; i < SLOT_COUNT; i++) {
			if (!slots[i].st.open) continue;
			if (ep == slots[i].ep_in) bridge_gadget_in_cleared(i);
			else if (ep == slots[i].ep_out) bridge_gadget_out_cleared(i);
		}
		return true;	/* usbd ACKs standard endpoint requests itself */
	}
	return false;		/* no class requests on the bridge interfaces */
}

static bool slots_xfer(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred)
{
	(void)rhport;
	for (int i = 0; i < SLOT_COUNT; i++) {
		struct slot *s = &slots[i];
		if (!s->st.open)
			continue;
		if (ep_addr == s->ep_out) {
			s->st.out_pkts++;
			s->st.out_bytes += xferred;
			if (xferred == 0) s->st.out_zlp++;
			bridge_gadget_out_done(i, result == XFER_RESULT_SUCCESS ? xferred : 0);
			return true;
		}
		if (ep_addr == s->ep_in) {
			s->st.in_pkts++;
			s->st.in_bytes += xferred;
			bridge_gadget_in_done(i);
			return true;
		}
	}
	return false;
}

void dcd_h3_out_progress(uint8_t ep_addr, uint32_t received)
{
	for (int i = 0; i < SLOT_COUNT; i++)
		if (slots[i].st.open && ep_addr == slots[i].ep_out) {
			bridge_gadget_out_progress(i, received);
			return;
		}
}

static const usbd_class_driver_t slots_driver = {
	.name = "slots",
	.init = slots_init,
	.deinit = slots_deinit,
	.reset = slots_reset,
	.open = slots_open,
	.control_xfer_cb = slots_control,
	.xfer_cb = slots_xfer,
	.sof = NULL,
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count)
{
	*driver_count = 1;
	return &slots_driver;
}

const struct slot_stats *slots_stats(int slot) { return &slots[slot].st; }
