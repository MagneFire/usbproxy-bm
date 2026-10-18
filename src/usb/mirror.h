#ifndef MIRROR_H
#define MIRROR_H
#include <stdbool.h>
#include <stdint.h>
#include "tusb.h"
#include "host/usbh_pvt.h"
#include "device/usbd_pvt.h"
/* Mirror mode: a device on the USB-A port that is not a watch is presented
 * to the Mac as itself. mirror_host.c keeps the device's descriptors and
 * strings and carries control transfers to it; mirror_gadget.c presents
 * the descriptors on the gadget (endpoints renumbered onto the musb's)
 * and carries the Mac's requests over; mirror_pipes.c moves the bulk and
 * interrupt endpoints' data. The CDC console is not part of a mirror. */

#define MIRROR_CFG_MAX  CFG_TUH_ENUMERATION_BUFSIZE	/* usbh's own limit on a configuration */
#define MIRROR_STR_MAX  8				/* strings prefetched, besides the language ids */
#define MIRROR_CTRL_MAX 4096				/* longest control data stage carried */
#define MIRROR_EP_MAX   5				/* musb: ep0 + 4 numbers per direction */

enum mirror_mode { MIRROR_NONE, MIRROR_WATCH, MIRROR_DEVICE, MIRROR_REFUSED };

/* Bytes from an interface descriptor to the next interface of another
 * number or an interface association: the interface with its alternate
 * settings and class descriptors, what one driver claims. */
static inline uint16_t mirror_itf_span(const uint8_t *itf, uint16_t max_len)
{
	const uint8_t *p = itf, *end = itf + max_len;
	for (p += p[0]; p + 2 <= end && p[0] >= 2 && p + p[0] <= end; p += p[0]) {
		if (p[1] == TUSB_DESC_INTERFACE_ASSOCIATION) break;
		if (p[1] == TUSB_DESC_INTERFACE && p[2] != itf[2]) break;
	}
	return (uint16_t)(p - itf);
}

/* host side (mirror_host.c) */
enum mirror_mode mirror_mode(void);		/* of the device enumerated last */
void mirror_host_device_desc(uint8_t daddr, const tusb_desc_device_t *d);
void mirror_host_mounted(uint8_t daddr);	/* host_mgr: a MIRROR_DEVICE mounted: fetch the strings */
void mirror_host_poll(void);
uint8_t mirror_daddr(void);			/* 0 when no device is mirrored */
tusb_speed_t mirror_dev_speed(void);
const tusb_desc_device_t *mirror_dev_desc(void);
const uint8_t *mirror_cfg_desc(void);		/* the device's configuration as captured */
const uint16_t *mirror_string(uint8_t index);	/* a prefetched string descriptor, NULL if none */
const uint8_t *mirror_qualifier(void);		/* the device qualifier as fetched, NULL if none */
const uint8_t *mirror_bos(void);
typedef void (*mirror_ctrl_cb)(xfer_result_t result, uint32_t len, uintptr_t tag);
bool mirror_host_ctrl(const tusb_control_request_t *req, void *buf, mirror_ctrl_cb cb, uintptr_t tag);
bool mirror_host_clear_halt(uint8_t dev_ep, mirror_ctrl_cb cb, uintptr_t tag);
void mirror_host_status(char *buf, unsigned n);
/* the usbh class driver, an initializer for the application's driver list */
bool mirror_host_init(void);
bool mirror_host_deinit(void);
uint16_t mirror_host_open(uint8_t rhport, uint8_t daddr, const tusb_desc_interface_t *itf, uint16_t max_len);
bool mirror_host_set_config(uint8_t daddr, uint8_t itf_num);
bool mirror_host_xfer_cb(uint8_t daddr, uint8_t ep_addr, xfer_result_t result, uint32_t xferred);
void mirror_host_close(uint8_t daddr);
#define MIRROR_HOST_DRIVER { .name = "mirror", .init = mirror_host_init, .deinit = mirror_host_deinit, \
	.open = mirror_host_open, .set_config = mirror_host_set_config, .xfer_cb = mirror_host_xfer_cb, \
	.close = mirror_host_close }

/* gadget side (mirror_gadget.c) */
bool mirror_gadget_plan(const uint8_t *cfg, const char **why);	/* endpoint map for a configuration */
void mirror_gadget_build(void);			/* the descriptors to present, from the host side's copies */
bool mirror_gadget_high_speed(void);		/* the mirror is of a high-speed device */
const uint8_t *mirror_gadget_device(void);
const uint8_t *mirror_gadget_config(bool high);
const uint8_t *mirror_gadget_other_speed(void);
const uint8_t *mirror_gadget_qualifier(void);
const uint16_t *mirror_gadget_string(uint8_t index);
uint8_t mirror_gadget_to_dev(uint8_t gadget_ep);	/* the device's endpoint behind a gadget one, 0 if none */
void mirror_gadget_status(char *buf, unsigned n);
/* the usbd class driver, an initializer for the application's driver list */
void mirror_gadget_init(void);
bool mirror_gadget_deinit(void);
void mirror_gadget_reset(uint8_t rhport);
uint16_t mirror_gadget_open(uint8_t rhport, const tusb_desc_interface_t *itf, uint16_t max_len);
bool mirror_gadget_control(uint8_t rhport, uint8_t stage, const tusb_control_request_t *req);
bool mirror_gadget_xfer(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred);
#define MIRROR_GADGET_DRIVER { .name = "mirror", .init = mirror_gadget_init, .deinit = mirror_gadget_deinit, \
	.reset = mirror_gadget_reset, .open = mirror_gadget_open, .control_xfer_cb = mirror_gadget_control, \
	.xfer_cb = mirror_gadget_xfer, .sof = NULL }

/* main.c */
void mirror_ready(void);	/* host side: descriptors and strings in hand, present the mirror */
void mirror_gone(void);		/* host side: the device left */
#endif
