/* Any exception is fatal: print and record it (log ring + RTC GP0 as
 * 0xFA<type><pc offset>), then watchdog-reset. Before we were up, GP1 still
 * holds boot0's death counter: three in a row and boot0 enters FEL. */
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
	log_set_fault(rec);			/* the record first, the decoration after */
	for (int i = 0; i < 13; i++)
		log_printf("r%d=%08lx%s", i, (unsigned long)frame[i], (i % 4 == 3) ? "\n" : " ");
	log_printf("\n");
	/* The interrupted mode's lr/sp (main loop in SVC, ISRs in IRQ mode)
	 * and the top of its stack: addr2line on the text addresses in there
	 * is the backtrace. The SPSR saved on entry tells which mode. */
	uint32_t spsr, cpsr, lr_x = 0, sp_x = 0;
	__asm__ volatile("mrs %0, spsr\n mrs %1, cpsr" : "=r"(spsr), "=r"(cpsr));
	if ((spsr & 0x1f) == 0x13)
		__asm__ volatile("cps #0x13\n mov %0, lr\n mov %1, sp\n msr cpsr_c, %2" : "=&r"(lr_x), "=&r"(sp_x) : "r"(cpsr) : "memory");
	else if ((spsr & 0x1f) == 0x12)
		__asm__ volatile("cps #0x12\n mov %0, lr\n mov %1, sp\n msr cpsr_c, %2" : "=&r"(lr_x), "=&r"(sp_x) : "r"(cpsr) : "memory");
	log_printf("spsr=%08lx lr=%08lx sp=%08lx\n", (unsigned long)spsr, (unsigned long)lr_x, (unsigned long)sp_x);
	if (sp_x >= 0x40000000u && sp_x < 0x50000000u) {
		const uint32_t *w = (const uint32_t *)sp_x;
		for (int i = 0; i < 32; i += 8)
			log_printf("stack+%02x: %08lx %08lx %08lx %08lx %08lx %08lx %08lx %08lx\n", i * 4,
				   (unsigned long)w[i], (unsigned long)w[i + 1], (unsigned long)w[i + 2], (unsigned long)w[i + 3],
				   (unsigned long)w[i + 4], (unsigned long)w[i + 5], (unsigned long)w[i + 6], (unsigned long)w[i + 7]);
	}
	rtc_gp_write(0, 0xFA000000u | (type << 20) | ((pc - (uint32_t)IMAGE_BASE) & 0xfffffu));
	uart_flush();
	delay_ms(50);
	wdt_reset_now();
}
