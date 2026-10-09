/* usbproxy-bm: the superloop. M0 = runtime skeleton: UART console, watchdog,
 * RTC stamps/guards, log ring, MMU, GIC + 1 ms tick, LEDs. */
#include "fw.h"

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
	log_printf("boot %u: entry at %u ms, gp0=%08x gp1=%08x gp2=%08x gp3=%08x wdt=%d\n",
		   log_boots(), g_entry_ms, rtc_gp_read(0), rtc_gp_read(1),
		   rtc_gp_read(2), rtc_gp_read(3), wdt_armed());
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

	/* "Up" for the SPL's purposes. From M1 on this moves to the point where
	 * the gadget is configured by the host. */
	rtc_gp_write(1, 0);
	g_up = 1;
	log_printf("up: GP1 cleared\n");

	console_init();
	uint32_t last_blink = 0;
	for (;;) {
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
