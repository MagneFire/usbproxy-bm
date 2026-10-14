/* The gadget, byte-for-byte what usb-proxy's persistent gadget presents
 * (gadget-fixed.cpp + console-acm.cpp) so the Mac treats it the same:
 * 1d6b:0104, class EF/02/01, one configuration with the CDC-ACM console on
 * interfaces 0/1 (ep3in notify, ep4 data), the adb bridge interface 2
 * (ff/42/01, ep1) and the fastboot bridge interface 3 (ff/42/03, ep2).
 * Endpoint numbers match the appliance's map: adb 0x81/0x01, fastboot
 * 0x82/0x02, console 0x83/0x84/0x04. Two shapes: the full gadget carries the
 * watch's serial, the idle one is the console alone with GADGET_SERIAL
 * (Makefile), which differs from the appliance's USBPROXY01 so both can be
 * on the same Mac. main.c picks the shape. */
#include <string.h>
#include "fw.h"
#include "tusb.h"

#ifndef GADGET_SERIAL
#define GADGET_SERIAL "USBPROXYBM"
#endif

#define EP_ADB_IN       0x81
#define EP_ADB_OUT      0x01
#define EP_FB_IN        0x82
#define EP_FB_OUT       0x02
#define EP_ACM_NOTIFY   0x83
#define EP_ACM_IN       0x84
#define EP_ACM_OUT      0x04

enum { ITF_ACM_COMM = 0, ITF_ACM_DATA, ITF_ADB, ITF_FASTBOOT, ITF_COUNT };

static const tusb_desc_device_t desc_device = {
	.bLength = sizeof(tusb_desc_device_t),
	.bDescriptorType = TUSB_DESC_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = TUSB_CLASS_MISC,
	.bDeviceSubClass = MISC_SUBCLASS_COMMON,
	.bDeviceProtocol = MISC_PROTOCOL_IAD,
	.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
	.idVendor = 0x1d6b,
	.idProduct = 0x0104,
	.bcdDevice = 0x0100,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

uint8_t const *tud_descriptor_device_cb(void) { return (uint8_t const *)&desc_device; }

/* vendor bridge interface: 9 + 7 + 7 */
#define BRIDGE_DESC_LEN 23
#define BRIDGE_DESCRIPTOR(itf, proto, epin, epout, epsize) \
	9, TUSB_DESC_INTERFACE, itf, 0, 2, 0xff, 0x42, proto, 0, \
	7, TUSB_DESC_ENDPOINT, epin, TUSB_XFER_BULK, U16_TO_U8S_LE(epsize), 0, \
	7, TUSB_DESC_ENDPOINT, epout, TUSB_XFER_BULK, U16_TO_U8S_LE(epsize), 0

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + 2 * BRIDGE_DESC_LEN)
#define IDLE_TOTAL_LEN   (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static const uint8_t desc_config_hs[] = {
	TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, CONFIG_TOTAL_LEN, 0, 500),
	TUD_CDC_DESCRIPTOR(ITF_ACM_COMM, 0, EP_ACM_NOTIFY, 10, EP_ACM_OUT, EP_ACM_IN, 512),
	BRIDGE_DESCRIPTOR(ITF_ADB, 0x01, EP_ADB_IN, EP_ADB_OUT, 512),
	BRIDGE_DESCRIPTOR(ITF_FASTBOOT, 0x03, EP_FB_IN, EP_FB_OUT, 512),
};

static const uint8_t desc_config_fs[] = {
	TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, CONFIG_TOTAL_LEN, 0, 500),
	TUD_CDC_DESCRIPTOR(ITF_ACM_COMM, 0, EP_ACM_NOTIFY, 10, EP_ACM_OUT, EP_ACM_IN, 64),
	BRIDGE_DESCRIPTOR(ITF_ADB, 0x01, EP_ADB_IN, EP_ADB_OUT, 64),
	BRIDGE_DESCRIPTOR(ITF_FASTBOOT, 0x03, EP_FB_IN, EP_FB_OUT, 64),
};

static const uint8_t desc_idle_hs[] = {
	TUD_CONFIG_DESCRIPTOR(1, ITF_ADB, 0, IDLE_TOTAL_LEN, 0, 500),
	TUD_CDC_DESCRIPTOR(ITF_ACM_COMM, 0, EP_ACM_NOTIFY, 10, EP_ACM_OUT, EP_ACM_IN, 512),
};

static const uint8_t desc_idle_fs[] = {
	TUD_CONFIG_DESCRIPTOR(1, ITF_ADB, 0, IDLE_TOTAL_LEN, 0, 500),
	TUD_CDC_DESCRIPTOR(ITF_ACM_COMM, 0, EP_ACM_NOTIFY, 10, EP_ACM_OUT, EP_ACM_IN, 64),
};

static bool full;
static char serial[31] = GADGET_SERIAL;

void gadget_set(bool bridge, const char *s)
{
	full = bridge;
	if (!s || !*s) s = GADGET_SERIAL;
	size_t n = 0;
	while (n < sizeof serial - 1 && s[n]) { serial[n] = s[n]; n++; }
	serial[n] = 0;
}

bool gadget_full(void) { return full; }

static const uint8_t *config_for(bool high)
{
	if (full) return high ? desc_config_hs : desc_config_fs;
	return high ? desc_idle_hs : desc_idle_fs;
}

static const tusb_desc_device_qualifier_t desc_qualifier = {
	.bLength = sizeof(tusb_desc_device_qualifier_t),
	.bDescriptorType = TUSB_DESC_DEVICE_QUALIFIER,
	.bcdUSB = 0x0200,
	.bDeviceClass = TUSB_CLASS_MISC,
	.bDeviceSubClass = MISC_SUBCLASS_COMMON,
	.bDeviceProtocol = MISC_PROTOCOL_IAD,
	.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
	.bNumConfigurations = 1,
	.bReserved = 0,
};

uint8_t const *tud_descriptor_device_qualifier_cb(void) { return (uint8_t const *)&desc_qualifier; }

static uint8_t desc_other_speed[CONFIG_TOTAL_LEN];

uint8_t const *tud_descriptor_other_speed_configuration_cb(uint8_t index)
{
	(void)index;
	/* The other speed's config, with the OTHER_SPEED_CONFIG type byte. */
	const uint8_t *src = config_for(tud_speed_get() != TUSB_SPEED_HIGH);
	memcpy(desc_other_speed, src, full ? CONFIG_TOTAL_LEN : IDLE_TOTAL_LEN);
	desc_other_speed[1] = TUSB_DESC_OTHER_SPEED_CONFIG;
	return desc_other_speed;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
	(void)index;
	return config_for(tud_speed_get() == TUSB_SPEED_HIGH);
}

static const char *const strings[] = { NULL, "usb-proxy", "usb-proxy", serial };
static uint16_t desc_str[32];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
	(void)langid;
	size_t n;
	if (index == 0) {
		desc_str[1] = 0x0409;
		n = 1;
	} else {
		if (index >= sizeof strings / sizeof strings[0])
			return NULL;
		const char *s = strings[index];
		n = strlen(s);
		if (n > 30) n = 30;
		for (size_t i = 0; i < n; i++)
			desc_str[1 + i] = (uint8_t)s[i];
	}
	desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * n + 2));
	return desc_str;
}

const char *gadget_serial(void) { return serial; }
