/* The mirror's device side: which kind of device enumerated, its
 * descriptors and strings kept for the gadget, control transfers carried
 * to it. The configuration is seen before the class drivers claim it, so
 * a watch (adb or fastboot interface) is told from anything else there;
 * the mirror driver then claims every interface of anything else. */
#include <string.h>
#include "fw.h"
#include "tusb.h"
#include "host/usbh_pvt.h"
#include "host/hcd.h"
#include "mirror.h"

static enum mirror_mode mode;
static uint8_t daddr_cur;
static tusb_speed_t speed;
static tusb_desc_device_t dev_desc;
CFG_TUH_MEM_SECTION CFG_TUH_MEM_ALIGN static uint8_t cfg_desc[MIRROR_CFG_MAX];
/* [0] holds the language ids, [1 + i] the string str_index[i] */
CFG_TUH_MEM_SECTION CFG_TUH_MEM_ALIGN static uint16_t str_buf[MIRROR_STR_MAX + 1][128];
static uint8_t str_index[MIRROR_STR_MAX];
static bool str_ok[MIRROR_STR_MAX + 1];
static unsigned str_n;
CFG_TUH_MEM_SECTION CFG_TUH_MEM_ALIGN static uint8_t qual_buf[64];
CFG_TUH_MEM_SECTION CFG_TUH_MEM_ALIGN static uint8_t bos_buf[256];
static bool qual_ok, bos_ok;
static const char *refused_why;

/* The prefetch, one request at a time from the main loop, until everything
 * the Mac may ask for synchronously is in hand or FETCH_MS have passed. */
enum { F_IDLE, F_LANGID, F_STRINGS, F_QUAL, F_BOS_HDR, F_BOS, F_DONE };
#define FETCH_MS 3000
static int fstep;
static unsigned fidx;
static bool fbusy;
static uint32_t fstart;
static uint16_t langid;

/* The control transfer carried for the gadget, one at a time. */
static tusb_control_request_t creq, hreq;
static mirror_ctrl_cb ccb, hcb;
static uintptr_t ctag, htag;
static uint8_t halt_ep;
static bool cbusy, hbusy;
static uint32_t ctrl_n, ctrl_stalls, ctrl_fails, halts;

static bool watch_interface(const tusb_desc_interface_t *itf)
{
	return itf->bInterfaceClass == 0xff && itf->bInterfaceSubClass == 0x42 &&
	       (itf->bInterfaceProtocol == 0x01 || itf->bInterfaceProtocol == 0x03);
}

enum mirror_mode mirror_mode(void) { return mode; }
uint8_t mirror_daddr(void) { return daddr_cur; }
tusb_speed_t mirror_dev_speed(void) { return speed; }
const tusb_desc_device_t *mirror_dev_desc(void) { return &dev_desc; }
const uint8_t *mirror_cfg_desc(void) { return cfg_desc; }
const uint8_t *mirror_qualifier(void) { return qual_ok ? qual_buf : NULL; }
const uint8_t *mirror_bos(void) { return bos_ok ? bos_buf : NULL; }

const uint16_t *mirror_string(uint8_t index)
{
	if (index == 0) return str_ok[0] ? str_buf[0] : NULL;
	for (unsigned i = 0; i < str_n; i++)
		if (str_index[i] == index) return str_ok[1 + i] ? str_buf[1 + i] : NULL;
	return NULL;
}

/* ---- enumeration: classify and keep the descriptors ------------------ */
void mirror_host_device_desc(uint8_t daddr, const tusb_desc_device_t *d)
{
	(void)daddr;
	dev_desc = *d;
}

static void note_string(uint8_t index)
{
	if (!index) return;
	for (unsigned i = 0; i < str_n; i++)
		if (str_index[i] == index) return;
	if (str_n < MIRROR_STR_MAX) str_index[str_n++] = index;
}

bool tuh_enum_descriptor_configuration_cb(uint8_t daddr, uint8_t idx, const tusb_desc_configuration_t *cfg)
{
	(void)daddr; (void)idx;
	unsigned total = cfg->wTotalLength < MIRROR_CFG_MAX ? cfg->wTotalLength : MIRROR_CFG_MAX;
	const uint8_t *p = (const uint8_t *)cfg, *end = p + total;
	bool watch = false;
	for (p += cfg->bLength; p + 2 <= end && p[0] >= 2 && p + p[0] <= end; p += p[0])
		if (p[1] == TUSB_DESC_INTERFACE && p[0] >= sizeof(tusb_desc_interface_t) &&
		    watch_interface((const tusb_desc_interface_t *)p))
			watch = true;
	if (watch) { mode = MIRROR_WATCH; return true; }

	memcpy(cfg_desc, cfg, total);
	if (cfg->wTotalLength > MIRROR_CFG_MAX) {
		refused_why = "configuration too long";
	} else if (!mirror_gadget_plan(cfg_desc, &refused_why)) {
		/* why is set */
	} else {
		mode = MIRROR_DEVICE;
		str_n = 0;
		note_string(dev_desc.iManufacturer);
		note_string(dev_desc.iProduct);
		note_string(dev_desc.iSerialNumber);
		note_string(cfg->iConfiguration);
		for (p = cfg_desc + cfg->bLength; p + 2 <= end && p[0] >= 2 && p + p[0] <= end; p += p[0])
			if (p[1] == TUSB_DESC_INTERFACE && p[0] >= sizeof(tusb_desc_interface_t))
				note_string(((const tusb_desc_interface_t *)p)->iInterface);
		return true;
	}
	mode = MIRROR_REFUSED;
	log_printf("mirror: %04x:%04x not mirrored: %s\n", dev_desc.idVendor, dev_desc.idProduct, refused_why);
	return true;
}

/* ---- the host class driver: every interface of a mirrored device ------ */
bool mirror_host_init(void) { return true; }
bool mirror_host_deinit(void) { return true; }

uint16_t mirror_host_open(uint8_t rhport, uint8_t daddr, const tusb_desc_interface_t *itf, uint16_t max_len)
{
	(void)rhport;
	if (mode != MIRROR_DEVICE) return 0;
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
			if (!tuh_edpt_open(daddr, ep))
				log_printf("mirror: device ep %02x not opened\n", ep->bEndpointAddress);
		}
	}
	return len;
}

bool mirror_host_set_config(uint8_t daddr, uint8_t itf_num)
{
	usbh_driver_set_config_complete(daddr, itf_num);
	return true;
}

bool mirror_host_xfer_cb(uint8_t daddr, uint8_t ep_addr, xfer_result_t result, uint32_t xferred)
{
	(void)daddr; (void)ep_addr; (void)result; (void)xferred;
	return true;	/* the tuh_edpt_xfer completion callback does the work */
}

void mirror_host_close(uint8_t daddr)
{
	if (mode != MIRROR_DEVICE || daddr != daddr_cur) return;
	log_printf("mirror: %04x:%04x gone\n", dev_desc.idVendor, dev_desc.idProduct);
	mode = MIRROR_NONE;
	daddr_cur = 0;
	fstep = F_IDLE;
	cbusy = hbusy = false;
	mirror_gone();
}

/* ---- the prefetch ------------------------------------------------------ */
static void fetch_finish(void)
{
	fstep = F_DONE;
	unsigned ok = 0;
	for (unsigned i = 0; i < str_n; i++) ok += str_ok[1 + i];
	log_printf("mirror: %04x:%04x %s speed, strings %u/%u%s%s; presenting\n", dev_desc.idVendor, dev_desc.idProduct,
		   speed == TUSB_SPEED_HIGH ? "high" : speed == TUSB_SPEED_FULL ? "full" : "low", ok, str_n,
		   qual_ok ? ", qualifier" : "", bos_ok ? ", bos" : "");
	mirror_ready();
}

static void fetch_done(tuh_xfer_t *x)
{
	fbusy = false;
	if (x->daddr != daddr_cur || fstep == F_IDLE) return;
	bool ok = x->result == XFER_RESULT_SUCCESS && x->actual_len >= 2;
	switch (fstep) {
	case F_LANGID:
		str_ok[0] = ok;
		langid = ok && x->actual_len >= 4 ? str_buf[0][1] : 0x0409;
		if (ok && (str_buf[0][0] & 0xff) > x->actual_len)	/* truncated: say what we have */
			str_buf[0][0] = (uint16_t)((str_buf[0][0] & 0xff00) | x->actual_len);
		fstep = F_STRINGS;
		fidx = 0;
		break;
	case F_STRINGS:
		str_ok[1 + fidx] = ok;
		if (ok && (str_buf[1 + fidx][0] & 0xff) > x->actual_len)
			str_buf[1 + fidx][0] = (uint16_t)((str_buf[1 + fidx][0] & 0xff00) | x->actual_len);
		fidx++;
		break;
	case F_QUAL:
		qual_ok = ok && x->actual_len >= 10;
		fstep = F_BOS_HDR;
		break;
	case F_BOS_HDR:
		if (ok && x->actual_len >= 5 && bos_buf[2] + (bos_buf[3] << 8) <= (int)sizeof bos_buf) fstep = F_BOS;
		else fetch_finish();
		break;
	case F_BOS:
		bos_ok = ok && x->actual_len == (uint32_t)(bos_buf[2] + (bos_buf[3] << 8));
		fetch_finish();
		break;
	}
}

void mirror_host_mounted(uint8_t daddr)
{
	daddr_cur = daddr;
	speed = tuh_speed_get(daddr);
	memset(str_ok, 0, sizeof str_ok);
	qual_ok = bos_ok = false;
	cbusy = hbusy = false;
	fstep = F_LANGID;
	fbusy = false;
	fstart = time_ms();
}

void mirror_host_poll(void)
{
	if (fstep == F_IDLE || fstep == F_DONE || fbusy || !daddr_cur) return;
	if (time_ms() - fstart > FETCH_MS) {
		log_printf("mirror: prefetch incomplete after %u ms\n", FETCH_MS);
		fetch_finish();
		return;
	}
	switch (fstep) {
	case F_LANGID:
		fbusy = tuh_descriptor_get_string(daddr_cur, 0, 0, str_buf[0], sizeof str_buf[0], fetch_done, 0);
		break;
	case F_STRINGS:
		if (fidx >= str_n) { fstep = F_QUAL; break; }
		fbusy = tuh_descriptor_get_string(daddr_cur, str_index[fidx], langid, str_buf[1 + fidx],
						  sizeof str_buf[0], fetch_done, 0);
		break;
	case F_QUAL:
		if (speed != TUSB_SPEED_HIGH) { fstep = F_BOS_HDR; break; }
		fbusy = tuh_descriptor_get(daddr_cur, TUSB_DESC_DEVICE_QUALIFIER, 0, qual_buf, 10, fetch_done, 0);
		break;
	case F_BOS_HDR:
		if (dev_desc.bcdUSB < 0x0201) { fetch_finish(); break; }
		fbusy = tuh_descriptor_get(daddr_cur, TUSB_DESC_BOS, 0, bos_buf, 5, fetch_done, 0);
		break;
	case F_BOS:
		fbusy = tuh_descriptor_get(daddr_cur, TUSB_DESC_BOS, 0, bos_buf, (uint16_t)(bos_buf[2] + (bos_buf[3] << 8)),
					   fetch_done, 0);
		break;
	}
}

/* ---- control transfers for the gadget --------------------------------- */
static void ctrl_done(tuh_xfer_t *x)
{
	cbusy = false;
	if (x->result == XFER_RESULT_STALLED) ctrl_stalls++;
	else if (x->result != XFER_RESULT_SUCCESS) ctrl_fails++;
	if (ccb) ccb(x->result, x->actual_len, ctag);
}

bool mirror_host_ctrl(const tusb_control_request_t *req, void *buf, mirror_ctrl_cb cb, uintptr_t tag)
{
	if (!daddr_cur || cbusy) return false;
	creq = *req;
	ccb = cb; ctag = tag;
	cbusy = true;
	ctrl_n++;
	tuh_xfer_t x = { .daddr = daddr_cur, .ep_addr = 0, .setup = &creq, .buffer = buf,
			 .complete_cb = ctrl_done, .user_data = 0 };
	if (!tuh_control_xfer(&x)) { cbusy = false; ctrl_fails++; return false; }
	return true;
}

/* CLEAR_FEATURE(ENDPOINT_HALT) to the device, then our own toggle reset. */
static void halt_done(tuh_xfer_t *x)
{
	hbusy = false;
	if (x->result == XFER_RESULT_SUCCESS && daddr_cur)
		hcd_edpt_clear_stall(usbh_get_rhport(daddr_cur), daddr_cur, halt_ep);
	if (hcb) hcb(x->result, 0, htag);
}

bool mirror_host_clear_halt(uint8_t dev_ep, mirror_ctrl_cb cb, uintptr_t tag)
{
	if (!daddr_cur || hbusy) return false;
	hreq = (tusb_control_request_t){
		.bmRequestType_bit = { .recipient = TUSB_REQ_RCPT_ENDPOINT, .type = TUSB_REQ_TYPE_STANDARD, .direction = TUSB_DIR_OUT },
		.bRequest = TUSB_REQ_CLEAR_FEATURE, .wValue = TUSB_REQ_FEATURE_EDPT_HALT, .wIndex = dev_ep, .wLength = 0 };
	halt_ep = dev_ep;
	hcb = cb; htag = tag;
	hbusy = true;
	halts++;
	tuh_xfer_t x = { .daddr = daddr_cur, .ep_addr = 0, .setup = &hreq, .buffer = NULL,
			 .complete_cb = halt_done, .user_data = 0 };
	if (!tuh_control_xfer(&x)) { hbusy = false; return false; }
	return true;
}

void mirror_host_status(char *buf, unsigned n)
{
	static const char *const names[] = { "none", "watch", "device", "refused" };
	unsigned ok = 0;
	for (unsigned i = 0; i < str_n; i++) ok += str_ok[1 + i];
	snprintf(buf, n, "mirror=%s %04x:%04x daddr:%u speed:%s strings:%u/%u qual:%d bos:%d ctrl:%lu stalls:%lu fails:%lu halts:%lu%s%s\n",
		 names[mode], dev_desc.idVendor, dev_desc.idProduct, daddr_cur,
		 speed == TUSB_SPEED_HIGH ? "high" : speed == TUSB_SPEED_FULL ? "full" : "low", ok, str_n, qual_ok, bos_ok,
		 (unsigned long)ctrl_n, (unsigned long)ctrl_stalls, (unsigned long)ctrl_fails, (unsigned long)halts,
		 mode == MIRROR_REFUSED ? " why:" : "", mode == MIRROR_REFUSED ? refused_why : "");
}
