/* usbproxy-bm: the superloop. M0 = runtime skeleton: UART console, watchdog,
 * RTC stamps/guards, log ring, MMU, GIC + 1 ms tick, LEDs. */
#include "fw.h"
#include "tusb.h"
#include "usb/host_mgr.h"
#include "bridge/bridge.h"

uint32_t g_entry_ms;
int g_up;

int main(void)
{
	g_entry_ms = time_ms();         /* CNTPCT runs from power-on */
	wdt_feed();
	rtc_gp_write(2, 0xB1000000u | (g_entry_ms & 0xffffffu));
	uart_init();
	uart_puts("\n\nusbproxy-bm " GITREV "\n");
	mmu_init();
	gic_init();
	timer_init();
	led_init();
	led_green(1);
	log_init();
	log_printf("boot %lu: entry at %lu ms, gp0=%08lx gp1=%08lx gp2=%08lx gp3=%08lx wdt=%d\n",
		   (unsigned long)log_boots(), (unsigned long)g_entry_ms,
		   (unsigned long)rtc_gp_read(0), (unsigned long)rtc_gp_read(1),
		   (unsigned long)rtc_gp_read(2), (unsigned long)rtc_gp_read(3), wdt_armed());
	if (log_fault())
		log_printf("previous boot fault: %s\n", log_fault());
	wdt_arm();
	irq_enable();

	if (rtc_gp_read(0) == RTC_MAGIC_HNG1) {
		rtc_gp_write(0, 0);
		log_printf("HNG1: hanging before the GP1 clear (SPL fallback test)\n");
		uart_flush();
		for (;;)
			;
	}

	/* "Up" (GP1 cleared) happens in tud_mount_cb: the host configured the
	 * gadget. Until then the SPL's guard still points at U-Boot proper. */
	bridge_init();
	const tusb_rhport_init_t rh_init = { .role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_HIGH };
	tusb_init(0, &rh_init);
	log_printf("usb: gadget %s initialised, waiting for the host\n", gadget_serial());
	const tusb_rhport_init_t host_init = { .role = TUSB_ROLE_HOST, .speed = TUSB_SPEED_HIGH };
	tusb_rhport_init(1, &host_init);
	log_printf("usb: host on EHCI1 initialised\n");

	console_init();
	uint32_t last_blink = 0;
	for (;;) {
		tud_task();
		tuh_task();
		host_mgr_poll();
		console_poll();
		wdt_feed();
		uint32_t t = tick_count();
		if (t - last_blink >= 500) {
			last_blink = t;
			led_green((t / 500) & 1);
		}
		wfi();
	}
}
