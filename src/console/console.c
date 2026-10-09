/* Line console on the UART (and, from M1, the gadget's CDC-ACM). Not a
 * shell: a fixed command set appliance.py's `bm` transport can parse.
 * `status` prints key=value lines. */
#include <string.h>
#include "fw.h"

#define LINE_MAX 96
static char line[LINE_MAX];
static int len;

static void prompt(void) { uart_puts("bm> "); }

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
	char buf[160];
	int n = snprintf(buf, sizeof buf,
		"build=%s\nuptime_ms=%u\nentry_ms=%u\nticks=%u\nboots=%u\nup=%d\n"
		"gp0=%08x\ngp1=%08x\ngp2=%08x\ngp3=%08x\nwdt=%d\n",
		GITREV, time_ms(), g_entry_ms, tick_count(), log_boots(), g_up,
		rtc_gp_read(0), rtc_gp_read(1), rtc_gp_read(2), rtc_gp_read(3), wdt_armed());
	uart_puts(buf);
	snprintf(buf, sizeof buf, "fault=%s\n", log_fault() ? log_fault() : "none");
	uart_puts(buf);
	(void)n;
}

static void run(char *s)
{
	char *argv[6];
	int argc = argsplit(s, argv, 6);
	if (!argc) return;
	const char *c = argv[0];
	if (!strcmp(c, "help")) {
		uart_puts("status log [bytes] reboot uboot hang hangnext panic mem <addr> [words] "
			  "gp <n> [val] led <g|r> <0|1>\n");
	} else if (!strcmp(c, "status")) {
		cmd_status();
	} else if (!strcmp(c, "log")) {
		log_dump(argc > 1 ? strtoul(argv[1], 0, 0) : 4096);
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
		char buf[32];
		for (unsigned i = 0; i < n; i++) {
			snprintf(buf, sizeof buf, "%08x: %08x\n", a + 4 * i, readl(a + 4 * i));
			uart_puts(buf);
		}
	} else if (!strcmp(c, "gp") && argc > 1) {
		unsigned n = strtoul(argv[1], 0, 0) & 3;
		if (argc > 2) rtc_gp_write(n, strtoul(argv[2], 0, 16));
		char buf[24];
		snprintf(buf, sizeof buf, "gp%u=%08x\n", n, rtc_gp_read(n));
		uart_puts(buf);
	} else if (!strcmp(c, "led") && argc > 2) {
		int on = argv[2][0] == '1';
		if (argv[1][0] == 'g') led_green(on); else led_red(on);
	} else {
		uart_puts("? (help)\n");
	}
}

void console_init(void)
{
	len = 0;
	prompt();
}

void console_poll(void)
{
	int c;
	while ((c = uart_getc()) >= 0) {
		if (c == '\r' || c == '\n') {
			uart_puts("\n");
			line[len] = 0;
			run(line);
			len = 0;
			prompt();
		} else if (c == 0x7f || c == 0x08) {
			if (len) { len--; uart_puts("\b \b"); }
		} else if (c == 0x03) {
			len = 0; uart_puts("^C\n"); prompt();
		} else if (c >= 0x20 && c < 0x7f && len < LINE_MAX - 1) {
			line[len++] = (char)c;
			uart_putc((char)c);
		}
	}
}
