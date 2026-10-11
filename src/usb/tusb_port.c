/* What TinyUSB needs from the board, and the device-level callbacks. */
#include "fw.h"
#include "tusb.h"

uint32_t tusb_time_millis_api(void) { return time_ms(); }
void tusb_time_delay_ms_api(uint32_t ms) { delay_ms(ms); }

void tud_mount_cb(void)
{
	/* "Up" for boot0's death counter: the host configured the gadget. */
	if (!g_up) {
		rtc_gp_write(1, 0);
		g_up = 1;
		log_printf("usb: configured by the host (%s); GP1 cleared\n",
			   tud_speed_get() == TUSB_SPEED_HIGH ? "high speed" : "full speed");
	} else {
		log_printf("usb: configured again (%s)\n",
			   tud_speed_get() == TUSB_SPEED_HIGH ? "high speed" : "full speed");
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
