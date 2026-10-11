/* What TinyUSB needs from the board, and the device-level callbacks. */
#include "fw.h"
#include "tusb.h"
#include "dcd_h3.h"
#include "usbphy.h"

uint32_t tusb_time_millis_api(void) { return time_ms(); }
void tusb_time_delay_ms_api(uint32_t ms) { delay_ms(ms); }

static void musb_irq(void) { dcd_int_handler(0); }

void dcd_h3_board_init(void)
{
	usbphy0_init();
	gic_set_handler(IRQ_MUSB, musb_irq);
}

void dcd_h3_board_int_enable(bool enable)
{
	if (enable)
		gic_unmask(IRQ_MUSB);
	else
		gic_mask(IRQ_MUSB);
}

uint32_t dcd_h3_board_time_us(void) { return time_us(); }

/* musb buffers live in cached DRAM. */
bool dcd_dcache_clean(const void *addr, uint32_t size)
{
	dcache_clean_range(addr, size);
	return true;
}
bool dcd_dcache_invalidate(const void *addr, uint32_t size)
{
	dcache_invalidate_range(addr, size);
	return true;
}

void tud_mount_cb(void)
{
	const char *speed = tud_speed_get() == TUSB_SPEED_HIGH ? "high speed" : "full speed";
	/* "Up" for boot0's death counter: the host configured the gadget. */
	if (!g_up) {
		rtc_gp_write(1, 0);
		g_up = 1;
		log_printf("usb: configured by the host (%s); GP1 cleared\n", speed);
	} else {
		log_printf("usb: configured again (%s)\n", speed);
	}
}

void tud_umount_cb(void) { log_printf("usb: unconfigured\n"); }
void tud_suspend_cb(bool remote_wakeup_en) { (void)remote_wakeup_en; log_printf("usb: suspended\n"); }
void tud_resume_cb(void) { log_printf("usb: resumed\n"); }

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
	(void)itf; (void)rts;
	log_printf("console: host %s the port\n", dtr ? "opened" : "closed");
}
