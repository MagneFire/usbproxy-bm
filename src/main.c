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
bool g_wdt_withheld;		/* the gate stopped feeding: nothing else may */
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
	/* No SOFs from the Mac is not a fault: when the Mac sleeps the musb
	 * reports SUSPEND and, 2 ms later, a RESUME that never brings SOFs, so
	 * tud_suspended() is false while the bus sits idle. Resetting the board
	 * there wakes the Mac over USB and the two then loop (observed: 57 s
	 * period). The gadget bus state is logged on change and only the EHCI
	 * (our own host controller) can fail the gate. */
	static bool sofs_idle;
	if (musb_ok == sofs_idle) {
		sofs_idle = !musb_ok;
		log_printf("usb: gadget bus %s\n", sofs_idle ? "idle (no SOFs; host asleep or gone)" : "active again");
	}
	musb_ok = true;
	if (!bound || (musb_ok && ehci_ok)) { stuck_since = 0; return true; }
	if (!stuck_since) stuck_since = now;
	if ((int32_t)(now - stuck_since) < 2000) return true;
	lost = true;
	g_wdt_withheld = true;
	log_printf("watchdog: usb progress lost (musb %s, ehci %s); letting the watchdog fire\n",
		   musb_ok ? "ok" : "stuck", ehci_ok ? "ok" : "stuck");
	log_set_fault(musb_ok ? "ehci frame counter stuck" : "musb frame counter stuck");
	return false;
}

/* The Mac can miss our disconnect: after quick power cycles it kept the old
 * device object for a minute, retried transfers into the void (transaction
 * errors every second) and never reset the new us; four cold boots sat
 * unconfigured (2026-10-10 18:16). If the host has not configured us within
 * 2 s of our last connect, drop the pull-up for 100 ms and present the
 * gadget again, which the host sees as a fresh attach. Normal enumeration
 * finishes in ~0.3 s. The console's `detach` suspends this. */
bool g_manual_detach;
uint32_t g_represents;
/* A full-speed configuration is re-presented: a fresh attach gets high speed
 * after a missed chirp. Full speed after FS_RETRIES is the link's own limit. */
#define FS_RETRIES 3
uint32_t g_fs_retries;
/* Idle accounting for `status`: main-loop passes and WFIs in the last whole
 * second (idle = most passes end in a WFI; the 1 ms tick alone gives ~1000). */
uint32_t g_loops_ps, g_wfis_ps, g_busy_us_ps;
static uint32_t loops, wfis, busy_us, rate_t0;
static void gadget_present_check(void)
{
	static uint32_t since, dropped_at;
	static bool dropped, fs_stale;
	uint32_t now = time_ms();
	if (!since) since = now;
	if (g_manual_detach) { since = now; return; }
	if (dropped) {
		if ((int32_t)(now - dropped_at) >= 100) { tud_connect(); dropped = false; since = now; }
		return;
	}
	if (tud_mounted()) {
		since = now;
		/* the dropped configuration counts until the host resets us */
		if (fs_stale && (int32_t)(now - dropped_at) < 2000) return;
		fs_stale = false;
		if (tud_speed_get() == TUSB_SPEED_HIGH) { g_fs_retries = 0; return; }
		if (!TUD_OPT_HIGH_SPEED || g_fs_retries >= FS_RETRIES) return;
		g_fs_retries++; g_represents++;
		log_printf("usb: configured at full speed; re-presenting the gadget for high speed (try %lu)\n",
			   (unsigned long)g_fs_retries);
		tud_disconnect();
		dropped = true; dropped_at = now; fs_stale = true;
		return;
	}
	fs_stale = false;
	if ((int32_t)(now - since) >= 2000) {
		g_represents++;
		log_printf("usb: not configured %lu ms after connect; re-presenting the gadget (%lu)\n",
			   (unsigned long)(now - since), (unsigned long)g_represents);
		tud_disconnect();
		dropped = true; dropped_at = now;
	}
}

/* Power state follows the bridge: full clocks while a device is bound (the
 * datapath is CPU-bound PIO), idle clocks otherwise. Console `cpuclk`
 * overrides until the next transition; `powerauto 0` stops the following. */
bool g_power_auto = true;
static void power_follow(void)
{
	static int state = -1;
	int active = host_daddr() != 0;
	if (!g_power_auto || active == state) return;
	state = active;
	ccu_set_cpu_mhz(active ? CPU_MHZ_ACTIVE : CPU_MHZ_IDLE);
	ccu_set_bus_idle(!active);
	log_printf("power: %s (cpu %u MHz)\n", active ? "active" : "idle", ccu_cpu_mhz());
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
	led_red(0);                     /* both LEDs stay off: power; console `led` drives them by hand */
	led_green(0);
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
		log_printf("HNG1: hanging before the GP1 clear (boot0 death-counter test)\n");
		uart_flush();
		for (;;)
			;
	}

	/* "Up" (GP1 cleared) happens in tud_mount_cb: the host configured the
	 * gadget. Until then boot0's death counter in GP1 still counts this boot. */
	bridge_init();
	const tusb_rhport_init_t rh_init = { .role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_HIGH };
	tusb_init(0, &rh_init);
	log_printf("usb: gadget %s initialised, waiting for the host\n", gadget_serial());
	const tusb_rhport_init_t host_init = { .role = TUSB_ROLE_HOST, .speed = TUSB_SPEED_HIGH };
	tusb_rhport_init(1, &host_init);
	log_printf("usb: host on EHCI1 initialised\n");

	console_init();
	for (;;) {
		tud_task();
		tuh_task();
		host_mgr_poll();
		console_poll();
		if (usb_progress_ok())
			wdt_feed();
		gadget_present_check();
		power_follow();
		/* Sleep only when nothing is queued, decided with IRQs masked so an
		 * event raised between the check and the wfi still wakes us (WFI
		 * wakes on a pending IRQ regardless of CPSR.I). An unconditional
		 * wfi here parked every completion that arrived during the task
		 * pass until the next interrupt, up to one SOF period (125 us). */
		extern void hcd_h3_poll(void);
		hcd_h3_poll();
		bridge_poll();
		loops++;
		__asm__ volatile("cpsid i" ::: "memory");
		if (!tud_task_event_ready() && !tuh_task_event_ready() && !bridge_host_pending()) {
			uint32_t t_sleep = time_us();
			wfi();
			busy_us -= time_us() - t_sleep;   /* sleeping time is not busy time */
			wfis++;
		}
		__asm__ volatile("cpsie i" ::: "memory");
		uint32_t now = time_ms();
		if (now - rate_t0 >= 1000) {
			g_loops_ps = loops; g_wfis_ps = wfis;
			g_busy_us_ps = busy_us + (now - rate_t0) * 1000;
			loops = wfis = 0; busy_us = 0; rate_t0 = now;
		}
	}
}
