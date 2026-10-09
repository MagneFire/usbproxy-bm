#include <string.h>
#include "fw.h"
#include "tusb.h"
#include "host/usbh_pvt.h"
#include "host/hcd.h"
#include "host_mgr.h"

extern volatile int ehci_h3_port_handed_over;
uint32_t hcd_h3_portsc(void);
void hcd_h3_portsc_write(uint32_t v);

#define PORTSC_W1C      (BIT(1) | BIT(3) | BIT(5))
#define PORTSC_CCS      BIT(0)
#define PORTSC_PE       BIT(2)
#define PORTSC_LS_MASK  (3u << 10)
#define PORTSC_OWNER    BIT(13)

static struct hslot slots[2];
static uint8_t cur_daddr;
static uint32_t reclaim_at, reenum_at;
static unsigned reclaims, hs_retries;
static bool reenum_pending;

/* ---- class driver: the watch's bridge interfaces ---------------------- */
static bool drv_init(void) { memset(slots, 0, sizeof slots); return true; }
static bool drv_deinit(void) { return true; }

static uint16_t drv_open(uint8_t rhport, uint8_t daddr, tusb_desc_interface_t const *itf, uint16_t max_len)
{
	(void)rhport;
	if (itf->bInterfaceClass != 0xff || itf->bInterfaceSubClass != 0x42 || itf->bNumEndpoints != 2)
		return 0;
	int idx = itf->bInterfaceProtocol == 0x01 ? 0 : itf->bInterfaceProtocol == 0x03 ? 1 : -1;
	if (idx < 0)
		return 0;
	uint16_t need = sizeof(tusb_desc_interface_t) + 2 * sizeof(tusb_desc_endpoint_t);
	if (max_len < need)
		return 0;
	struct hslot *s = &slots[idx];
	const uint8_t *p = (const uint8_t *)itf + sizeof(tusb_desc_interface_t);
	uint8_t in = 0, out = 0;
	for (int k = 0; k < 2; k++) {
		const tusb_desc_endpoint_t *ep = (const tusb_desc_endpoint_t *)p;
		if (ep->bDescriptorType != TUSB_DESC_ENDPOINT || ep->bmAttributes.xfer != TUSB_XFER_BULK)
			return 0;
		if (!tuh_edpt_open(daddr, ep))
			return 0;
		if (ep->bEndpointAddress & 0x80) in = ep->bEndpointAddress; else out = ep->bEndpointAddress;
		p += sizeof(tusb_desc_endpoint_t);
	}
	if (!in || !out)
		return 0;
	s->daddr = daddr; s->ep_in = in; s->ep_out = out; s->bound = true;
	s->in_pkts = s->out_pkts = s->in_bytes = s->out_bytes = 0;
	log_printf("host: interface %u (proto %02x) -> slot %d, in %02x out %02x\n",
		   itf->bInterfaceNumber, itf->bInterfaceProtocol, idx, in, out);
	return need;
}

static bool drv_set_config(uint8_t daddr, uint8_t itf_num)
{
	usbh_driver_set_config_complete(daddr, itf_num);
	return true;
}

static bool drv_xfer_cb(uint8_t daddr, uint8_t ep_addr, xfer_result_t result, uint32_t xferred)
{
	(void)daddr; (void)ep_addr; (void)result; (void)xferred;
	return true;	/* the tuh_edpt_xfer completion callback does the work */
}

static void drv_close(uint8_t daddr)
{
	for (int i = 0; i < 2; i++)
		if (slots[i].bound && slots[i].daddr == daddr) {
			slots[i].bound = false;
			log_printf("host: slot %d unbound\n", i);
		}
}

static const usbh_class_driver_t bridge_driver = {
	.name = "bridge",
	.init = drv_init,
	.deinit = drv_deinit,
	.open = drv_open,
	.set_config = drv_set_config,
	.xfer_cb = drv_xfer_cb,
	.close = drv_close,
};

usbh_class_driver_t const *usbh_app_driver_get_cb(uint8_t *driver_count)
{
	*driver_count = 1;
	return &bridge_driver;
}

/* ---- device callbacks ------------------------------------------------- */
void tuh_mount_cb(uint8_t daddr)
{
	uint16_t vid = 0, pid = 0;
	tuh_vid_pid_get(daddr, &vid, &pid);
	cur_daddr = daddr;
	tusb_speed_t sp = tuh_speed_get(daddr);
	log_printf("host: device %u mounted, %04x:%04x, %s\n", daddr, vid, pid,
		   sp == TUSB_SPEED_HIGH ? "high speed" : sp == TUSB_SPEED_FULL ? "FULL speed" : "low speed");
	if (sp != TUSB_SPEED_HIGH && hs_retries < 8) {
		/* The cold-boot chirp miss (DEVELOPMENT.md section 8): try again. */
		hs_retries++;
		reenum_pending = true;
		reenum_at = time_ms() + 200;
		log_printf("host: not high speed, re-resetting the port (try %u)\n", hs_retries);
	}
}

void tuh_umount_cb(uint8_t daddr)
{
	log_printf("host: device %u gone\n", daddr);
	if (cur_daddr == daddr)
		cur_daddr = 0;
}

/* ---- port policy ------------------------------------------------------ */
void host_mgr_poll(void)
{
	uint32_t now = time_ms();
	if (ehci_h3_port_handed_over) {
		if (!reclaim_at)
			reclaim_at = now + 1000;
		if ((int32_t)(now - reclaim_at) >= 0) {
			uint32_t v = hcd_h3_portsc() & ~PORTSC_W1C;
			hcd_h3_portsc_write(v & ~PORTSC_OWNER);
			ehci_h3_port_handed_over = 0;
			reclaim_at = 0;
			reclaims++;
			log_printf("host: port taken back from the companion (%u)\n", reclaims);
		}
	}
	if (reenum_pending && (int32_t)(now - reenum_at) >= 0) {
		reenum_pending = false;
		hcd_event_device_remove(1, false);
		hcd_event_device_attach(1, false);
	}
}

const char *host_link_speed(void)
{
	uint32_t p = hcd_h3_portsc();
	if (!(p & PORTSC_CCS)) return "none";
	if (p & PORTSC_PE) return "480";
	if ((p & PORTSC_LS_MASK) == (1u << 10)) return "1.5";
	return "12";
}

uint8_t host_daddr(void) { return cur_daddr; }
uint32_t host_portsc(void) { return hcd_h3_portsc(); }
const struct hslot *host_slot(int slot) { return &slots[slot & 1]; }

/* ---- console test transfer ------------------------------------------- */
CFG_TUH_MEM_SECTION CFG_TUH_MEM_ALIGN static uint8_t test_buf[4096];
static uint32_t test_t0;

static void test_done(tuh_xfer_t *xfer)
{
	uint32_t dt = time_us() - test_t0;
	log_printf("host: xfer ep %02x result %d, %lu bytes in %lu us\n", xfer->ep_addr,
		   xfer->result, (unsigned long)xfer->actual_len, (unsigned long)dt);
	if ((xfer->ep_addr & 0x80) && xfer->actual_len) {
		char line[64]; int n = 0;
		for (uint32_t i = 0; i < xfer->actual_len && i < 24; i++)
			n += snprintf(line + n, sizeof line - (size_t)n, "%02x", test_buf[i]);
		log_printf("host:   %s\n", line);
	}
	for (int i = 0; i < 2; i++) {
		if (!slots[i].bound || slots[i].daddr != xfer->daddr) continue;
		if (xfer->ep_addr & 0x80) { slots[i].in_pkts++; slots[i].in_bytes += xfer->actual_len; }
		else { slots[i].out_pkts++; slots[i].out_bytes += xfer->actual_len; }
	}
}

int host_test_xfer(int slot, bool in, uint32_t len)
{
	struct hslot *s = &slots[slot & 1];
	if (!s->bound) return -1;
	if (len > sizeof test_buf) len = sizeof test_buf;
	if (!in)
		for (uint32_t i = 0; i < len; i++) test_buf[i] = (uint8_t)i;
	tuh_xfer_t xfer = {
		.daddr = s->daddr,
		.ep_addr = in ? s->ep_in : s->ep_out,
		.buflen = len,
		.buffer = test_buf,
		.complete_cb = test_done,
		.user_data = 0,
	};
	test_t0 = time_us();
	return tuh_edpt_xfer(&xfer) ? 0 : -2;
}

void host_status_line(char *buf, unsigned n)
{
	snprintf(buf, n, "host_link=%s\nhost_daddr=%u\nhost_portsc=%08lx\nhost_reclaims=%u\nhost_hs_retries=%u\n"
		 "hslot0=bound:%d daddr:%u in:%02x out:%02x pkts %lu/%lu\n"
		 "hslot1=bound:%d daddr:%u in:%02x out:%02x pkts %lu/%lu\n",
		 host_link_speed(), cur_daddr, (unsigned long)hcd_h3_portsc(), reclaims, hs_retries,
		 slots[0].bound, slots[0].daddr, slots[0].ep_in, slots[0].ep_out, (unsigned long)slots[0].in_pkts, (unsigned long)slots[0].out_pkts,
		 slots[1].bound, slots[1].daddr, slots[1].ep_in, slots[1].ep_out, (unsigned long)slots[1].in_pkts, (unsigned long)slots[1].out_pkts);
}
