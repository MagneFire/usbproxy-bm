/* usbproxy-bm: the superloop. M0 = runtime skeleton: UART console, watchdog,
 * RTC stamps/guards, log ring, MMU, GIC + 1 ms tick, LEDs. */
#include "fw.h"
#include "tusb.h"
#include "usb/host_mgr.h"
#include "bridge/bridge.h"

uint32_t g_entry_ms;
int g_up;

/* Progress-gated watchdog: once a watch is bound, both USB cores must keep
 * their frame counters moving (musb FRNUM while the Mac keeps the bus
 * awake, EHCI FRINDEX always); if either is stuck for 2 s the main loop
 * stops feeding the watchdog and the board resets 8 s later with a fault
 * record. Console `hangusb` halts the EHCI to prove it. */
#define USBC_FRNUM      (0x01c19000u + 0x54)
#define EHCI1_FRINDEX   (0x01c1b010u + 0x0c)
static bool usb_progress_ok(void)
{
	static uint32_t last_check, last_frnum, last_frindex, stuck_since;
	static bool lost;
	uint32_t now = time_ms();
	if (lost) return false;
	if ((int32_t)(now - last_check) < 250) return true;
	last_check = now;
	bool bound = host_daddr() != 0;
	uint32_t fr = readl(USBC_FRNUM) & 0x7ff, fi = readl(EHCI1_FRINDEX);
	bool musb_ok = !tud_mounted() || tud_suspended() || fr != last_frnum;
	bool ehci_ok = fi != last_frindex;
	last_frnum = fr; last_frindex = fi;
	if (!bound || (musb_ok && ehci_ok)) { stuck_since = 0; return true; }
	if (!stuck_since) stuck_since = now;
	if ((int32_t)(now - stuck_since) < 2000) return true;
	lost = true;
	log_printf("watchdog: usb progress lost (musb %s, ehci %s); letting the watchdog fire\n",
		   musb_ok ? "ok" : "stuck", ehci_ok ? "ok" : "stuck");
	log_set_fault(musb_ok ? "ehci frame counter stuck" : "musb frame counter stuck");
	return false;
}

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
		if (usb_progress_ok())
			wdt_feed();
		uint32_t t = tick_count();
		if (t - last_blink >= 500) {
			last_blink = t;
			led_green((t / 500) & 1);
		}
		/* Sleep only when nothing is queued, decided with IRQs masked so an
		 * event raised between the check and the wfi still wakes us (WFI
		 * wakes on a pending IRQ regardless of CPSR.I). An unconditional
		 * wfi here parked every completion that arrived during the task
		 * pass until the next interrupt, up to one SOF period (125 us). */
		extern void hcd_h3_poll(void);
		hcd_h3_poll();
		bridge_poll();
		__asm__ volatile("cpsid i" ::: "memory");
		if (!tud_task_event_ready() && !tuh_task_event_ready() && !bridge_host_pending())
			wfi();
		__asm__ volatile("cpsie i" ::: "memory");
	}
}
