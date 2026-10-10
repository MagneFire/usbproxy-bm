#include <string.h>
#include "fw.h"
#include "tusb.h"
#include "device/usbd_pvt.h"
#include "gadget_slots.h"
#include "bridge/bridge.h"

struct slot {
	uint8_t ep_in, ep_out;
	bool loopback;
	bool in_busy;
	struct slot_stats st;
	CFG_TUSB_MEM_ALIGN uint8_t out_buf[512];
	CFG_TUSB_MEM_ALIGN uint8_t in_buf[512];
};
static struct slot slots[SLOT_COUNT];

static void arm_out(struct slot *s)
{
	usbd_edpt_xfer(0, s->ep_out, s->out_buf, sizeof s->out_buf, false);
}

static void slots_init(void) { memset(slots, 0, sizeof slots); }
static bool slots_deinit(void) { return true; }

static void slots_reset(uint8_t rhport)
{
	(void)rhport;
	for (int i = 0; i < SLOT_COUNT; i++) {
		if (slots[i].st.open)
			bridge_gadget_gone(i);
		slots[i].st.open = false;
		slots[i].in_busy = false;
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
	s->in_busy = false;
	if (s->loopback)
		arm_out(s);
	else
		bridge_gadget_ready(idx, s->ep_in, s->ep_out);
	log_printf("slot %d open: in %02x out %02x\n", idx, s->ep_in, s->ep_out);
	return need;
}

static bool slots_control(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req)
{
	(void)rhport; (void)stage; (void)req;
	return false;	/* no class requests on the bridge interfaces */
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
			if (!s->loopback) {
				bridge_gadget_out_done(i, result == XFER_RESULT_SUCCESS ? xferred : 0);
				return true;
			}
			if (result != XFER_RESULT_SUCCESS) { arm_out(s); return true; }
			if (s->loopback) {
				/* ping-pong: echo this packet (ZLPs too), re-arm OUT when the IN is done */
				memcpy(s->in_buf, s->out_buf, xferred);
				s->in_busy = true;
				usbd_edpt_xfer(0, s->ep_in, s->in_buf, (uint16_t)xferred, false);
			} else {
				arm_out(s);
			}
			return true;
		}
		if (ep_addr == s->ep_in) {
			s->st.in_pkts++;
			s->st.in_bytes += xferred;
			s->in_busy = false;
			if (s->loopback)
				arm_out(s);
			else
				bridge_gadget_in_done(i);
			return true;
		}
	}
	return false;
}

void dcd_h3_out_progress(uint8_t ep_addr, uint32_t received)
{
	for (int i = 0; i < SLOT_COUNT; i++)
		if (slots[i].st.open && !slots[i].loopback && ep_addr == slots[i].ep_out) {
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

void slots_loopback(int slot, bool on)
{
	if (slot < 0 || slot >= SLOT_COUNT) return;
	slots[slot].loopback = on;	/* takes effect at the next configuration */
}
bool slots_loopback_on(int slot) { return slot >= 0 && slot < SLOT_COUNT && slots[slot].loopback; }
const struct slot_stats *slots_stats(int slot) { return &slots[slot].st; }
