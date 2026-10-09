/* Line console on the UART and on the gadget's CDC-ACM port, same command
 * set, same parser (one line buffer per port). Not a shell: a fixed command
 * set appliance.py's `bm` transport can parse. `status` prints key=value
 * lines. */
#include <string.h>
#include "fw.h"
#include "tusb.h"
#include "usb/gadget_slots.h"
#include "usb/host_mgr.h"

#define LINE_MAX 96

struct con {
	const char *name;
	void (*puts)(const char *);
	char line[LINE_MAX];
	int len;
};

static void cdc_puts(const char *s)
{
	if (!tud_cdc_connected())
		return;
	size_t n = strlen(s);
	uint32_t deadline = time_ms() + 200;
	while (n) {
		uint32_t w = tud_cdc_write(s, n);
		s += w; n -= w;
		tud_cdc_write_flush();
		if (!w) {
			if ((int32_t)(time_ms() - deadline) > 0)
				return;		/* host not reading; drop the rest */
			tud_task();
		}
	}
}

static struct con con_uart = { "uart", uart_puts, {0}, 0 };
static struct con con_cdc = { "cdc", cdc_puts, {0}, 0 };
static struct con *cur = &con_uart;

static void con_out(const char *s) { cur->puts(s); }
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
	snprintf(buf, sizeof buf, "gp0=%08lx\ngp1=%08lx\ngp2=%08lx\ngp3=%08lx\nwdt=%d\n",
		(unsigned long)rtc_gp_read(0), (unsigned long)rtc_gp_read(1),
		(unsigned long)rtc_gp_read(2), (unsigned long)rtc_gp_read(3), wdt_armed());
	con_out(buf);
	snprintf(buf, sizeof buf, "usb_mounted=%d\nusb_speed=%s\nusb_suspended=%d\ncdc_connected=%d\n",
		tud_mounted(), tud_speed_get() == TUSB_SPEED_HIGH ? "high" : "full",
		tud_suspended(), tud_cdc_connected());
	con_out(buf);
	for (int i = 0; i < SLOT_COUNT; i++) {
		const struct slot_stats *st = slots_stats(i);
		snprintf(buf, sizeof buf, "slot%d=open:%d loop:%d out:%lu/%lu zlp:%lu in:%lu/%lu\n", i,
			st->open, slots_loopback_on(i), (unsigned long)st->out_pkts,
			(unsigned long)st->out_bytes, (unsigned long)st->out_zlp,
			(unsigned long)st->in_pkts, (unsigned long)st->in_bytes);
		con_out(buf);
	}
	host_status_line(buf, sizeof buf);
	con_out(buf);
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
		con_out("status log [bytes] reboot uboot hang hangnext panic mem <addr> [words] "
			"gp <n> [val] led <g|r> <0|1> loop <slot> <0|1> detach attach hostxfer <in|out> <n> [slot]\n");
	} else if (!strcmp(c, "status")) {
		cmd_status();
	} else if (!strcmp(c, "log")) {
		log_dump(argc > 1 ? strtoul(argv[1], 0, 0) : 4096, con_out);
	} else if (!strcmp(c, "reboot")) {
		log_printf("reboot requested\n");
		wdt_reset_now();
	} else if (!strcmp(c, "uboot")) {
		/* Make the SPL take the "last Falcon boot died" path: U-Boot proper. */
		log_printf("uboot requested: GP1=FLCN + reset\n");
		rtc_gp_write(1, RTC_MAGIC_FLCN);
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
			con_out(buf);
		}
	} else if (!strcmp(c, "gp") && argc > 1) {
		unsigned n = strtoul(argv[1], 0, 0) & 3;
		if (argc > 2) rtc_gp_write(n, strtoul(argv[2], 0, 16));
		snprintf(buf, sizeof buf, "gp%u=%08lx\n", n, (unsigned long)rtc_gp_read(n));
		con_out(buf);
	} else if (!strcmp(c, "led") && argc > 2) {
		int on = argv[2][0] == '1';
		if (argv[1][0] == 'g') led_green(on); else led_red(on);
	} else if (!strcmp(c, "hostxfer") && argc > 2) {
		/* hostxfer <in|out> <bytes> [slot]: one bulk transfer on the watch */
		int slot = argc > 3 ? (int)strtoul(argv[3], 0, 0) : 0;
		int r = host_test_xfer(slot, argv[1][0] == 'i', strtoul(argv[2], 0, 0));
		snprintf(buf, sizeof buf, "hostxfer: %s\n", r == 0 ? "submitted" : r == -1 ? "slot not bound" : "submit failed");
		con_out(buf);
	} else if (!strcmp(c, "loop") && argc > 2) {
		slots_loopback((int)strtoul(argv[1], 0, 0), argv[2][0] == '1');
	} else if (!strcmp(c, "detach")) {
		tud_disconnect();
		log_printf("usb: soft-disconnected\n");
	} else if (!strcmp(c, "attach")) {
		tud_connect();
		log_printf("usb: soft-connected\n");
	} else {
		con_out("? (help)\n");
	}
}

static void feed(struct con *con, int c)
{
	cur = con;
	if (c == '\r' || c == '\n') {
		con_out("\n");
		con->line[con->len] = 0;
		run(con->line);
		con->len = 0;
		prompt();
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

void console_poll(void)
{
	int c;
	while ((c = uart_getc()) >= 0)
		feed(&con_uart, c);
	while (tud_cdc_available()) {
		uint8_t b[64];
		uint32_t n = tud_cdc_read(b, sizeof b);
		for (uint32_t i = 0; i < n; i++)
			feed(&con_cdc, b[i]);
	}
}
