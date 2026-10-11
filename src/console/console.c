/* Line console on the UART and on the gadget's CDC-ACM port, same command
 * set, same parser (one line buffer per port). Not a shell: a fixed command
 * set appliance.py's `bm` transport can parse. `status` prints key=value
 * lines. */
#include <string.h>
#include "fw.h"
#include "gitrev.h"            /* generated: build/gitrev.h */
#include "tusb.h"
#include "dcd_h3.h"
#include "ehci_h3.h"
#include "usb/gadget_slots.h"
#include "usb/host_mgr.h"
#include "bridge/bridge.h"

#define LINE_MAX 96

struct con {
	const char *name;
	void (*puts)(const char *);
	char line[LINE_MAX];
	int len;
	bool follow;		/* `log -f`: stream new log bytes until any key */
	uint32_t follow_total;
	bool cr;		/* the last key was CR: an LF after it ends the same line */
};

static bool cdc_write_all(const char *s, size_t n, uint32_t deadline)
{
	while (n) {
		uint32_t w = tud_cdc_write(s, n);
		s += w; n -= w;
		tud_cdc_write_flush();
		if (!w) {
			if ((int32_t)(time_ms() - deadline) > 0)
				return false;	/* host not reading; drop the rest */
			tud_task();
		}
	}
	return true;
}

/* Like the UART, the CDC console speaks CRLF: terminal programs show a
 * bare LF as a staircase. */
static void cdc_puts(const char *s)
{
	if (!tud_cdc_connected())
		return;
	uint32_t deadline = time_ms() + 200;
	while (*s) {
		const char *nl = strchr(s, '\n');
		size_t n = nl ? (size_t)(nl - s) : strlen(s);
		if (n && !cdc_write_all(s, n, deadline)) return;
		if (!nl) return;
		if (!cdc_write_all("\r\n", 2, deadline)) return;
		s = nl + 1;
	}
}

static struct con con_uart = { "uart", uart_puts, {0}, 0, false, 0, false };
static struct con con_cdc = { "cdc", cdc_puts, {0}, 0, false, 0, false };
static struct con *cur = &con_uart;

static void con_out(const char *s) { cur->puts(s); }
/* Long output (`log`, `mem`) runs inside one main-loop pass and can take
 * longer than the watchdog's period: feed it per piece, unless the
 * progress gate is letting it fire. */
static void dump_out(const char *s)
{
	con_out(s);
	if (!g_wdt_withheld) wdt_feed();
}
static void prompt(void) { con_out("bm> "); }

static int argsplit(char *s, char **argv, int max)
{
	int n = 0;
	while (*s && n < max) {
		while (*s == ' ') s++;
		if (!*s) break;
		argv[n++] = s;
		while (*s && *s != ' ') s++;
		if (*s) *s++ = 0;
	}
	return n;
}

static void cmd_status(void)
{
	char buf[200];
	snprintf(buf, sizeof buf,
		"build=%s\nserial=%s\nuptime_ms=%lu\nentry_ms=%lu\nticks=%lu\nboots=%lu\nup=%d\n",
		GITREV, gadget_serial(), (unsigned long)time_ms(), (unsigned long)g_entry_ms,
		(unsigned long)tick_count(), (unsigned long)log_boots(), g_up);
	con_out(buf);
	extern uint32_t g_loops_ps, g_wfis_ps, g_busy_us_ps;
	snprintf(buf, sizeof buf, "idle=loops:%lu wfis:%lu busy_us:%lu (per s)\n",
		 (unsigned long)g_loops_ps, (unsigned long)g_wfis_ps, (unsigned long)g_busy_us_ps);
	con_out(buf);
	snprintf(buf, sizeof buf, "gp0=%08lx\ngp1=%08lx\ngp2=%08lx\ngp3=%08lx\nwdt=%d\n",
		(unsigned long)rtc_gp_read(0), (unsigned long)rtc_gp_read(1),
		(unsigned long)rtc_gp_read(2), (unsigned long)rtc_gp_read(3), wdt_armed());
	con_out(buf);
	snprintf(buf, sizeof buf, "usb_mounted=%d\nusb_speed=%s\nusb_suspended=%d\nusb_represented=%lu\nusb_fs_retries=%lu\ncdc_connected=%d\n",
		tud_mounted(), tud_speed_get() == TUSB_SPEED_HIGH ? "high" : "full",
		tud_suspended(), (unsigned long)g_represents, (unsigned long)g_fs_retries, tud_cdc_connected());
	con_out(buf);
	{
		uint32_t x, b;
		dcd_h3_dpb_stats(&x, &b);
		snprintf(buf, sizeof buf, "musb_dpb=rx_extra:%lu tx_busy_irqs:%lu\n",
			 (unsigned long)x, (unsigned long)b);
		con_out(buf);
		uint32_t rd, rdn, wr, wrn;
		dcd_h3_fifo_stats(&rd, &rdn, &wr, &wrn);
		snprintf(buf, sizeof buf, "musb_fifo_us_x10=rd:%lu (n %lu) wr:%lu (n %lu)\n",
			 (unsigned long)rd, (unsigned long)rdn, (unsigned long)wr, (unsigned long)wrn);
		con_out(buf);
		uint32_t *grx, *gtx;
		dcd_h3_gap_stats(&grx, &gtx);
		snprintf(buf, sizeof buf, "musb_gap_rx=<12:%lu <20:%lu <30:%lu <50:%lu <100:%lu more:%lu\nmusb_gap_tx=<12:%lu <20:%lu <30:%lu <50:%lu <100:%lu more:%lu\n",
			 (unsigned long)grx[0], (unsigned long)grx[1], (unsigned long)grx[2], (unsigned long)grx[3], (unsigned long)grx[4], (unsigned long)grx[5],
			 (unsigned long)gtx[0], (unsigned long)gtx[1], (unsigned long)gtx[2], (unsigned long)gtx[3], (unsigned long)gtx[4], (unsigned long)gtx[5]);
		con_out(buf);
		uint32_t drx, dtx, dsh, dsp, dbad;
		dcd_h3_dma_stats(&drx, &dtx, &dsh, &dsp, &dbad);
		snprintf(buf, sizeof buf, "musb_dma=mask:%lu rx:%lu tx:%lu rx_short:%lu tx_spin_us:%lu bad_resid:%lu\n",
			 (unsigned long)dcd_h3_dma_enabled(), (unsigned long)drx, (unsigned long)dtx, (unsigned long)dsh,
			 (unsigned long)dsp, (unsigned long)dbad);
		con_out(buf);
	}
	for (int i = 0; i < SLOT_COUNT; i++) {
		const struct slot_stats *st = slots_stats(i);
		snprintf(buf, sizeof buf, "slot%d=open:%d out:%lu/%lu zlp:%lu in:%lu/%lu\n", i,
			st->open, (unsigned long)st->out_pkts,
			(unsigned long)st->out_bytes, (unsigned long)st->out_zlp,
			(unsigned long)st->in_pkts, (unsigned long)st->in_bytes);
		con_out(buf);
	}
	host_status_line(buf, sizeof buf);
	con_out(buf);
	host_cycle_status(buf, sizeof buf);
	con_out(buf);
	{
		static char bbuf[1200];		/* framer + two cycle lines + two bridge lines */
		bridge_status(bbuf, sizeof bbuf);
		con_out(bbuf);
	}
	snprintf(buf, sizeof buf, "fault=%s\n", log_fault() ? log_fault() : "none");
	con_out(buf);
}

static void run(char *s)
{
	char *argv[6];
	int argc = argsplit(s, argv, 6);
	if (!argc) return;
	const char *c = argv[0];
	char buf[48];
	if (!strcmp(c, "help")) {
		con_out("status  log [bytes|-f]  mem <hex> [n]  memw <hex> <hex>  gp <n> [hex]  led <g|r> <0|1>\n"
			"reboot  fel  detach  attach  hang  hangnext  hangusb  panic\n"
			"bridge: chunk [bytes]  cut [mask]  cutfloor [bytes]  cutlag [pkts]  cutcanary [0|1]  dma <mask>  zero\n"
			"host:   ehci  nakrl [n]  hostxfer <in|out> <bytes> [slot]  hostxfer cnxn [m]  hostcycle [n]  hostoff\n"
			"power:  cpuclk [MHz]  powerauto [0|1]  sofirq <0|1>\n");
	} else if (!strcmp(c, "status")) {
		cmd_status();
	} else if (!strcmp(c, "log") && argc > 1 && !strcmp(argv[1], "-f")) {
		if (cur == &con_uart) {
			con_out("the UART already mirrors the log live\n");
		} else {
			cur->follow = true; cur->follow_total = log_total();
			con_out("following the log; any key stops\n");
		}
	} else if (!strcmp(c, "log")) {
		log_dump(argc > 1 ? strtoul(argv[1], 0, 0) : 4096, dump_out);
	} else if (!strcmp(c, "reboot")) {
		log_printf("reboot requested\n");
		wdt_reset_now();
	} else if (!strcmp(c, "fel")) {
		/* This word in GP1 at reset sends boot0 into the BROM's FEL
		 * routine: sunxi-fel on the OTG port, card untouched.
		 * tools/flash.py's flash and run start here. */
		log_printf("fel requested: GP1=FEL1 + reset\n");
		if (cur == &con_cdc) {
			/* the log goes to the UART only: answer here too (flash.py
			 * waits longer for a board that did), and let it out */
			con_out("fel requested\n");
			for (uint32_t t0 = time_ms(); time_ms() - t0 < 20;)
				tud_task();
		}
		rtc_gp_write(1, RTC_MAGIC_FEL1);
		wdt_reset_now();
	} else if (!strcmp(c, "hang")) {
		log_printf("hang: not feeding the watchdog\n");
		uart_flush();
		for (;;)
			;
	} else if (!strcmp(c, "hangnext")) {
		log_printf("hangnext: GP0=HNG1 + reset; next boot hangs before the GP1 clear\n");
		rtc_gp_write(0, RTC_MAGIC_HNG1);
		wdt_reset_now();
	} else if (!strcmp(c, "panic")) {
		log_printf("panic: deliberate data abort\n");
		*(volatile uint32_t *)0xdead0000u = 1;
	} else if (!strcmp(c, "mem") && argc > 1) {
		uint32_t a = strtoul(argv[1], 0, 16) & ~3u;
		unsigned n = argc > 2 ? strtoul(argv[2], 0, 0) : 1;
		for (unsigned i = 0; i < n; i++) {
			snprintf(buf, sizeof buf, "%08lx: %08lx\n", (unsigned long)(a + 4 * i),
				 (unsigned long)readl(a + 4 * i));
			dump_out(buf);
		}
	} else if (!strcmp(c, "memw") && argc > 2) {
		uint32_t a = strtoul(argv[1], 0, 16) & ~3u;
		writel(strtoul(argv[2], 0, 16), a);
		snprintf(buf, sizeof buf, "%08lx: %08lx\n", (unsigned long)a, (unsigned long)readl(a));
		con_out(buf);
	} else if (!strcmp(c, "gp") && argc > 1) {
		unsigned n = strtoul(argv[1], 0, 0) & 3;
		if (argc > 2) rtc_gp_write(n, strtoul(argv[2], 0, 16));
		snprintf(buf, sizeof buf, "gp%u=%08lx\n", n, (unsigned long)rtc_gp_read(n));
		con_out(buf);
	} else if (!strcmp(c, "led") && argc > 2) {
		int on = argv[2][0] == '1';
		if (argv[1][0] == 'g') led_green(on); else led_red(on);
	} else if (!strcmp(c, "ehci")) {
		extern void hcd_h3_dump(void (*)(const char *));
		hcd_h3_dump(con_out);
	} else if (!strcmp(c, "hostxfer") && argc > 1 && !strcmp(argv[1], "cnxn")) {
		int r = host_test_cnxn(argc > 2 && argv[2][0] == 'm');
		snprintf(buf, sizeof buf, "hostxfer cnxn: %s\n", r == 0 ? "submitted" : "not bound/failed");
		con_out(buf);
	} else if (!strcmp(c, "hostxfer") && argc > 2) {
		/* hostxfer <in|out> <bytes> [slot]: one bulk transfer on the watch */
		int slot = argc > 3 ? (int)strtoul(argv[3], 0, 0) : 0;
		int r = host_test_xfer(slot, argv[1][0] == 'i', strtoul(argv[2], 0, 0));
		snprintf(buf, sizeof buf, "hostxfer: %s\n", r == 0 ? "submitted" : r == -1 ? "slot not bound" : "submit failed");
		con_out(buf);
	} else if (!strcmp(c, "chunk")) {
		if (argc > 1) bridge_set_chunk(strtoul(argv[1], 0, 0));
		snprintf(buf, sizeof buf, "chunk=%lu\n", (unsigned long)bridge_chunk());
		con_out(buf);
	} else if (!strcmp(c, "dma") && argc > 1) {
		dcd_h3_dma_enable(strtoul(argv[1], 0, 0));
	} else if (!strcmp(c, "nakrl")) {
		if (argc > 1) ehci_h3_set_nak_rl((uint8_t)strtoul(argv[1], 0, 0));
		snprintf(buf, sizeof buf, "nakrl=%u\n", ehci_h3_nak_rl());
		con_out(buf);
	} else if (!strcmp(c, "zero")) {
		bridge_stats_reset();
	} else if (!strcmp(c, "hangusb")) {
		/* halt the EHCI: FRINDEX stops, the progress gate must reset the board */
		clrbits(0x01c1b010u, 1u);
		con_out("ehci halted; expect a watchdog reset in ~10 s\n");
	} else if (!strcmp(c, "hostcycle")) {
		host_cycle_start(argc > 1 ? (unsigned)strtoul(argv[1], 0, 0) : 1);
	} else if (!strcmp(c, "cutlag")) {
		if (argc > 1) bridge_set_cut_lag(strtoul(argv[1], 0, 0));
		snprintf(buf, sizeof buf, "cutlag=%lu\n", (unsigned long)bridge_cut_lag());
		con_out(buf);
	} else if (!strcmp(c, "cutcanary")) {
		if (argc > 1) bridge_set_cut_canary(strtoul(argv[1], 0, 0));
		snprintf(buf, sizeof buf, "cutcanary=%lu\n", (unsigned long)bridge_cut_canary());
		con_out(buf);
	} else if (!strcmp(c, "cutfloor")) {
		if (argc > 1) bridge_set_cut_floor(strtoul(argv[1], 0, 0));
		snprintf(buf, sizeof buf, "cutfloor=%lu\n", (unsigned long)bridge_cut_floor());
		con_out(buf);
	} else if (!strcmp(c, "cut")) {
		if (argc > 1) bridge_set_cut(strtoul(argv[1], 0, 0));
		snprintf(buf, sizeof buf, "cut=%lu\n", (unsigned long)bridge_cut());
		con_out(buf);
	} else if (!strcmp(c, "sofirq") && argc > 1) {
		dcd_h3_sof_irq(strtoul(argv[1], 0, 0) != 0);
	} else if (!strcmp(c, "cpuclk")) {
		extern void ccu_set_cpu_mhz(unsigned); extern unsigned ccu_cpu_mhz(void);
		if (argc > 1) ccu_set_cpu_mhz((unsigned)strtoul(argv[1], 0, 0));
		snprintf(buf, sizeof buf, "cpu=%u MHz\n", ccu_cpu_mhz());
		con_out(buf);
	} else if (!strcmp(c, "powerauto")) {
		extern bool g_power_auto;
		if (argc > 1) g_power_auto = strtoul(argv[1], 0, 0) != 0;
		snprintf(buf, sizeof buf, "powerauto=%d\n", g_power_auto);
		con_out(buf);
	} else if (!strcmp(c, "hostoff")) {
		extern void usbphy1_off(void);
		log_printf("hostoff: EHCI1/OHCI1/PHY1 off until the next reboot (power measurement)\n");
		usbphy1_off();
	} else if (!strcmp(c, "detach")) {
		g_manual_detach = true;
		tud_disconnect();
		log_printf("usb: soft-disconnected\n");
	} else if (!strcmp(c, "attach")) {
		g_manual_detach = false;
		tud_connect();
		log_printf("usb: soft-connected\n");
	} else {
		con_out("? (help)\n");
	}
}

static void feed(struct con *con, int c)
{
	cur = con;
	bool lf_after_cr = c == '\n' && con->cr;
	con->cr = c == '\r';
	if (lf_after_cr) return;
	if (con->follow) {		/* any key ends `log -f`; the key is eaten */
		con->follow = false;
		con->len = 0;
		con_out("\n");
		prompt();
		return;
	}
	if (c == '\r' || c == '\n') {
		con_out("\n");
		con->line[con->len] = 0;
		run(con->line);
		con->len = 0;
		if (!con->follow) prompt();
	} else if (c == 0x7f || c == 0x08) {
		if (con->len) { con->len--; con_out("\b \b"); }
	} else if (c == 0x03) {
		con->len = 0; con_out("^C\n"); prompt();
	} else if (c >= 0x20 && c < 0x7f && con->len < LINE_MAX - 1) {
		con->line[con->len++] = (char)c;
		char e[2] = { (char)c, 0 };
		con_out(e);
	}
}

void console_init(void)
{
	cur = &con_uart;
	prompt();
}

static void follow_pump(struct con *con)
{
	if (!con->follow) return;
	uint32_t t = log_total();
	if (t == con->follow_total) return;
	uint32_t n = t - con->follow_total;
	if (n > 512) n = 512;		/* catch up over several polls, keep the loop short */
	log_dump_at(con->follow_total, n, con->puts);
	con->follow_total += n;
}

void console_poll(void)
{
	int c;
	follow_pump(&con_cdc);
	while ((c = uart_getc()) >= 0)
		feed(&con_uart, c);
	while (tud_cdc_available()) {
		uint8_t b[64];
		uint32_t n = tud_cdc_read(b, sizeof b);
		for (uint32_t i = 0; i < n; i++)
			feed(&con_cdc, b[i]);
	}
}
