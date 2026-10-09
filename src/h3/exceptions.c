/* Any exception is fatal: print and record it (log ring + RTC GP0 as
 * 0xFA<type><pc offset>), then watchdog-reset. The SPL sees GP1 still set if
 * this happens before we were up and hands the next boot to U-Boot proper. */
#include "fw.h"

static const char *const names[] = { "?", "undef", "svc", "pabt", "dabt", "fiq" };

void exception_handler(uint32_t type, uint32_t *frame)
{
	uint32_t pc = frame[13];
	uint32_t dfar, dfsr, ifar, ifsr;
	__asm__ volatile("mrc p15, 0, %0, c6, c0, 0" : "=r"(dfar));
	__asm__ volatile("mrc p15, 0, %0, c5, c0, 0" : "=r"(dfsr));
	__asm__ volatile("mrc p15, 0, %0, c6, c0, 2" : "=r"(ifar));
	__asm__ volatile("mrc p15, 0, %0, c5, c0, 1" : "=r"(ifsr));
	if (type > 5) type = 0;
	char rec[100];
	snprintf(rec, sizeof rec, "%s pc=%08x dfar=%08x dfsr=%03x ifar=%08x ifsr=%03x up=%d",
		 names[type], pc, dfar, dfsr & 0xfff, ifar, ifsr & 0xfff, g_up);
	log_printf("\n!!! FAULT %s\n", rec);
	for (int i = 0; i < 13; i++)
		log_printf("r%d=%08lx%s", i, (unsigned long)frame[i], (i % 4 == 3) ? "\n" : " ");
	log_printf("\n");
	log_set_fault(rec);
	rtc_gp_write(0, 0xFA000000u | (type << 20) | ((pc - (uint32_t)IMAGE_BASE) & 0xfffffu));
	uart_flush();
	delay_ms(50);
	wdt_reset_now();
}
