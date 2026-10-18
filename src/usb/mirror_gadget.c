/* The mirror's Mac side: the device's descriptors presented by the gadget,
 * with its endpoints renumbered onto the musb's four pairs, and EP0 carried
 * over. A full or low-speed device is presented at full speed with its
 * endpoint sizes as they are; a high-speed device at high speed, with a
 * full-speed variant should the link end up there. usbd keeps the
 * standard device requests (descriptors from the copies here, the
 * configuration), everything else that reaches a class driver or the
 * vendor hook is forwarded to the device, and its answer or STALL comes
 * back through the control transfer the Mac is still waiting on. */
#include <string.h>
#include "fw.h"
#include "tusb.h"
#include "device/usbd_pvt.h"
#include "device/dcd.h"
#include "mirror.h"
#include "mirror_pipes.h"

static uint8_t dev_desc[18];
static uint8_t cfg_hs[MIRROR_CFG_MAX], cfg_fs[MIRROR_CFG_MAX], other_speed[MIRROR_CFG_MAX];
static uint8_t qual[10];
static bool high_speed;
/* [dir][gadget number] -> device endpoint address, [dir][device number] -> gadget number */
static uint8_t g2d[2][MIRROR_EP_MAX], d2g[2][16];
static unsigned n_in, n_out, n_itf;

/* the request being carried; seq tells a late answer from a new request */
static tusb_control_request_t pending;
static uintptr_t seq;
CFG_TUH_MEM_SECTION CFG_TUH_MEM_ALIGN static uint8_t ctrl_buf[MIRROR_CTRL_MAX];
static uint32_t ctrl_in, ctrl_out, ctrl_nodata, ctrl_stalled, ctrl_stale, ctrl_refused, halts_cleared, halts_set;

bool mirror_gadget_high_speed(void) { return high_speed; }
const uint8_t *mirror_gadget_device(void) { return dev_desc; }
const uint8_t *mirror_gadget_config(bool high) { return high ? cfg_hs : cfg_fs; }
const uint8_t *mirror_gadget_other_speed(void) { return high_speed ? other_speed : NULL; }
const uint8_t *mirror_gadget_qualifier(void) { return high_speed ? qual : NULL; }
const uint16_t *mirror_gadget_string(uint8_t index) { return mirror_string(index); }

uint8_t mirror_gadget_to_dev(uint8_t ep)
{
	unsigned num = ep & 0xf;
	return num < MIRROR_EP_MAX ? g2d[ep >> 7][num] : 0;
}

/* ---- the plan: can this configuration be presented? ------------------- */
bool mirror_gadget_plan(const uint8_t *cfg, const char **why)
{
	const tusb_desc_configuration_t *c = (const tusb_desc_configuration_t *)cfg;
	unsigned total = c->wTotalLength < MIRROR_CFG_MAX ? c->wTotalLength : MIRROR_CFG_MAX;
	const uint8_t *p = cfg + c->bLength, *end = cfg + total;
	bool used[2][MIRROR_EP_MAX] = { { 0 } };
	memset(g2d, 0, sizeof g2d);
	memset(d2g, 0, sizeof d2g);
	n_in = n_out = n_itf = 0;
	if (p + 2 <= end && p[1] != TUSB_DESC_INTERFACE && p[1] != TUSB_DESC_INTERFACE_ASSOCIATION) {
		*why = "configuration does not start with an interface";
		return false;
	}
	for (; p + 2 <= end && p[0] >= 2 && p + p[0] <= end; p += p[0]) {
		if (p[1] == TUSB_DESC_INTERFACE && p[0] >= sizeof(tusb_desc_interface_t)) {
			const tusb_desc_interface_t *i = (const tusb_desc_interface_t *)p;
			if (i->bInterfaceNumber >= CFG_TUD_INTERFACE_MAX) { *why = "interface number too high"; return false; }
			if (i->bAlternateSetting == 0) n_itf++;
			continue;
		}
		if (p[1] != TUSB_DESC_ENDPOINT || p[0] < sizeof(tusb_desc_endpoint_t)) continue;
		const tusb_desc_endpoint_t *ep = (const tusb_desc_endpoint_t *)p;
		if (ep->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS) { *why = "isochronous endpoint"; return false; }
		unsigned dir = ep->bEndpointAddress >> 7, num = ep->bEndpointAddress & 0xf;
		if (d2g[dir][num]) continue;	/* seen in another alternate setting */
		unsigned g = 0;
		if (num && num < MIRROR_EP_MAX && !used[dir][num]) g = num;	/* its own number when free */
		else for (unsigned k = 1; k < MIRROR_EP_MAX; k++) if (!used[dir][k]) { g = k; break; }
		if (!g) { *why = "more than 4 endpoints in one direction"; return false; }
		used[dir][g] = true;
		d2g[dir][num] = (uint8_t)g;
		g2d[dir][g] = ep->bEndpointAddress;
		if (dir) n_in++; else n_out++;
	}
	return true;
}

/* ---- the descriptors to present ---------------------------------------- */
static void rewrite(uint8_t *cfg, bool full_speed)
{
	const tusb_desc_configuration_t *c = (const tusb_desc_configuration_t *)cfg;
	unsigned total = c->wTotalLength < MIRROR_CFG_MAX ? c->wTotalLength : MIRROR_CFG_MAX;
	uint8_t *p = cfg + c->bLength, *end = cfg + total;
	for (; p + 2 <= end && p[0] >= 2 && p + p[0] <= end; p += p[0]) {
		if (p[1] != TUSB_DESC_ENDPOINT || p[0] < sizeof(tusb_desc_endpoint_t)) continue;
		tusb_desc_endpoint_t *ep = (tusb_desc_endpoint_t *)p;
		unsigned dir = ep->bEndpointAddress >> 7, num = ep->bEndpointAddress & 0xf;
		ep->bEndpointAddress = (uint8_t)((dir << 7) | d2g[dir][num]);
		if (full_speed && tu_edpt_packet_size(ep) > 64)	/* bulk 64, interrupt at most 64 */
			ep->wMaxPacketSize = 64;
	}
}

void mirror_gadget_build(void)
{
	const tusb_desc_device_t *d = mirror_dev_desc();
	memcpy(dev_desc, d, sizeof dev_desc);
	dev_desc[7] = CFG_TUD_ENDPOINT0_SIZE;	/* bMaxPacketSize0: usbd stages 64-byte packets */
	dev_desc[17] = 1;			/* bNumConfigurations: usbh configured the first only */
	high_speed = mirror_dev_speed() == TUSB_SPEED_HIGH;

	const uint8_t *cfg = mirror_cfg_desc();
	const tusb_desc_configuration_t *c = (const tusb_desc_configuration_t *)cfg;
	unsigned total = c->wTotalLength < MIRROR_CFG_MAX ? c->wTotalLength : MIRROR_CFG_MAX;
	memcpy(cfg_hs, cfg, total);
	memcpy(cfg_fs, cfg, total);
	rewrite(cfg_hs, false);
	rewrite(cfg_fs, true);
	memcpy(other_speed, cfg_fs, total);
	other_speed[1] = TUSB_DESC_OTHER_SPEED_CONFIG;

	const uint8_t *q = mirror_qualifier();
	if (q) {
		memcpy(qual, q, sizeof qual);
	} else {
		qual[0] = 10; qual[1] = TUSB_DESC_DEVICE_QUALIFIER;
		qual[2] = dev_desc[2]; qual[3] = dev_desc[3];		/* bcdUSB */
		qual[4] = dev_desc[4]; qual[5] = dev_desc[5]; qual[6] = dev_desc[6];
		qual[7] = CFG_TUD_ENDPOINT0_SIZE; qual[8] = 1; qual[9] = 0;
	}
	qual[7] = CFG_TUD_ENDPOINT0_SIZE;
	qual[8] = 1;
}

/* ---- the usbd class driver: every interface of the mirror -------------- */
void mirror_gadget_init(void) {}
bool mirror_gadget_deinit(void) { return true; }

void mirror_gadget_reset(uint8_t rhport)
{
	(void)rhport;
	seq++;		/* an answer to a request from before the reset is dropped */
	mirror_pipes_gadget_reset();
}

uint16_t mirror_gadget_open(uint8_t rhport, const tusb_desc_interface_t *itf, uint16_t max_len)
{
	if (gadget_shape() != GADGET_MIRROR) return 0;
	uint16_t len = mirror_itf_span((const uint8_t *)itf, max_len);
	const uint8_t *p = (const uint8_t *)itf, *end = p + len;
	uint8_t alt = itf->bAlternateSetting;
	for (; p + 2 <= end && p[0] >= 2; p += p[0]) {
		if (p[1] == TUSB_DESC_INTERFACE) {
			alt = ((const tusb_desc_interface_t *)p)->bAlternateSetting;
		} else if (p[1] == TUSB_DESC_ENDPOINT && p[0] >= sizeof(tusb_desc_endpoint_t) && alt == 0) {
			const tusb_desc_endpoint_t *ep = (const tusb_desc_endpoint_t *)p;
			if (ep->bmAttributes.xfer != TUSB_XFER_BULK && ep->bmAttributes.xfer != TUSB_XFER_INTERRUPT)
				continue;
			if (!usbd_edpt_open(rhport, ep)) {
				log_printf("mirror: gadget ep %02x not opened\n", ep->bEndpointAddress);
				continue;
			}
			mirror_pipes_gadget_open(ep->bEndpointAddress, mirror_gadget_to_dev(ep->bEndpointAddress),
						 ep->bmAttributes.xfer, tu_edpt_packet_size(ep));
		}
	}
	return len;
}

/* The device answered (or not): finish the Mac's control transfer. */
static void fwd_done(xfer_result_t result, uint32_t len, uintptr_t tag)
{
	if (tag != seq) { ctrl_stale++; return; }
	if (result != XFER_RESULT_SUCCESS) {
		ctrl_stalled++;
		dcd_edpt_stall(0, TU_EP0_OUT);
		dcd_edpt_stall(0, TU_EP0_IN);
		return;
	}
	if (pending.bmRequestType_bit.direction == TUSB_DIR_IN && pending.wLength)
		tud_control_xfer(0, &pending, ctrl_buf, (uint16_t)len);
	else
		tud_control_status(0, &pending);
}

static bool mirror_ctrl(uint8_t rhport, uint8_t stage, const tusb_control_request_t *req)
{
	bool std_ep = req->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD &&
		      req->bmRequestType_bit.recipient == TUSB_REQ_RCPT_ENDPOINT;
	if (stage == CONTROL_STAGE_SETUP) {
		seq++;
		if (std_ep) {
			/* usbd acted on the halt feature itself and answers itself; the
			 * device follows: a cleared halt re-arms the pipe once the
			 * device's own is cleared, a set halt is set there too. */
			uint8_t gep = (uint8_t)req->wIndex, dep = mirror_gadget_to_dev(gep);
			if (req->bRequest == TUSB_REQ_CLEAR_FEATURE && req->wValue == TUSB_REQ_FEATURE_EDPT_HALT) {
				halts_cleared++;
				mirror_pipes_gadget_halt_cleared(gep);
			} else if (req->bRequest == TUSB_REQ_SET_FEATURE && req->wValue == TUSB_REQ_FEATURE_EDPT_HALT && dep) {
				halts_set++;
				tusb_control_request_t r = *req;
				r.wIndex = dep;
				mirror_host_ctrl(&r, NULL, NULL, 0);
			}
			return true;
		}
		pending = *req;
		if (pending.bmRequestType_bit.recipient == TUSB_REQ_RCPT_ENDPOINT)
			pending.wIndex = (uint16_t)((pending.wIndex & 0xff00) | mirror_gadget_to_dev((uint8_t)pending.wIndex));
		if (req->wLength == 0) {
			ctrl_nodata++;
			return mirror_host_ctrl(&pending, NULL, fwd_done, seq);
		}
		if (req->wLength > MIRROR_CTRL_MAX) { ctrl_refused++; return false; }
		if (req->bmRequestType_bit.direction == TUSB_DIR_IN) {
			ctrl_in++;
			return mirror_host_ctrl(&pending, ctrl_buf, fwd_done, seq);
		}
		ctrl_out++;
		return tud_control_xfer(rhport, req, ctrl_buf, req->wLength);	/* the data first */
	}
	if (stage == CONTROL_STAGE_DATA && !std_ep && req->bmRequestType_bit.direction == TUSB_DIR_OUT && req->wLength) {
		tud_control_status_defer(rhport);
		return mirror_host_ctrl(&pending, ctrl_buf, fwd_done, seq);
	}
	return true;
}

bool mirror_gadget_control(uint8_t rhport, uint8_t stage, const tusb_control_request_t *req)
{
	return mirror_ctrl(rhport, stage, req);
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, const tusb_control_request_t *req)
{
	if (gadget_shape() != GADGET_MIRROR) return false;
	return mirror_ctrl(rhport, stage, req);
}

bool mirror_gadget_xfer(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred)
{
	(void)rhport;
	return mirror_pipes_gadget_xfer(ep_addr, result, xferred);
}

void mirror_gadget_status(char *buf, unsigned n)
{
	int k = snprintf(buf, n, "mirror_gadget=%s itf:%u in:%u out:%u ctrl in:%lu out:%lu nodata:%lu stalled:%lu stale:%lu refused:%lu halts %lu/%lu map",
			 high_speed ? "high" : "full", n_itf, n_in, n_out, (unsigned long)ctrl_in, (unsigned long)ctrl_out,
			 (unsigned long)ctrl_nodata, (unsigned long)ctrl_stalled, (unsigned long)ctrl_stale,
			 (unsigned long)ctrl_refused, (unsigned long)halts_cleared, (unsigned long)halts_set);
	for (unsigned dir = 0; dir < 2 && k > 0 && (unsigned)k < n; dir++)
		for (unsigned g = 1; g < MIRROR_EP_MAX && (unsigned)k < n; g++)
			if (g2d[dir][g])
				k += snprintf(buf + k, n - (unsigned)k, " %02x<%02x", (dir << 7) | g, g2d[dir][g]);
	if ((unsigned)k < n - 1) { buf[k++] = '\n'; buf[k] = 0; }
}
