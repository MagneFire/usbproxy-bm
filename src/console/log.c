/* Log ring in uncached DRAM at the old ramoops address (ld/fw.ld LOGRING).
 * A versioned header makes a warm reset keep the previous boot's tail and a
 * fault record; a cold start (or a different firmware) re-initialises it.
 * Every line also goes to the UART. */
#include <string.h>
#include "fw.h"

#define LOG_MAGIC   0x474c4d42u     /* "BMLG" */
#define LOG_VERSION 1u

struct hdr {
	uint32_t magic, version, size, head, total, boots;
	uint32_t fault_valid;
	char fault[100];
};

extern char __logring_start[], __logring_end[];
static volatile struct hdr *H;
static volatile char *D;
static uint32_t boots_now;

void log_init(void)
{
	H = (volatile struct hdr *)__logring_start;
	D = (volatile char *)(__logring_start + sizeof(struct hdr));
	uint32_t size = (uint32_t)(__logring_end - __logring_start) - sizeof(struct hdr);
	if (H->magic != LOG_MAGIC || H->version != LOG_VERSION || H->size != size ||
	    H->head >= size) {
		H->magic = LOG_MAGIC; H->version = LOG_VERSION; H->size = size;
		H->head = 0; H->total = 0; H->boots = 0; H->fault_valid = 0;
	}
	H->boots++;
	boots_now = H->boots;
}

void log_write(const char *s, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		uart_putc(s[i]);
		if (!H) continue;
		D[H->head] = s[i];
		H->head = (H->head + 1) % H->size;
		H->total++;
	}
}

void log_printf(const char *fmt, ...)
{
	char buf[256];
	uint32_t ms = time_ms();
	int n = snprintf(buf, sizeof buf, "[%u.%03u] ", ms / 1000, ms % 1000);
	va_list ap; va_start(ap, fmt);
	n += vsnprintf(buf + n, sizeof buf - (size_t)n, fmt, ap);
	va_end(ap);
	if (n > (int)sizeof buf - 1) n = (int)sizeof buf - 1;
	log_write(buf, (size_t)n);
}

void log_dump(size_t last, void (*out)(const char *))
{
	if (!H) return;
	uint32_t avail = H->total < H->size ? H->total : H->size;
	if (last > avail) last = avail;
	uint32_t pos = (H->head + H->size - (uint32_t)last) % H->size;
	char chunk[65];
	while (last) {
		size_t n = last < 64 ? last : 64;
		for (size_t i = 0; i < n; i++) {
			chunk[i] = D[pos];
			pos = (pos + 1) % H->size;
		}
		chunk[n] = 0;
		out(chunk);
		last -= n;
	}
}

void log_set_fault(const char *s)
{
	if (!H) return;
	size_t n = strlen(s);
	if (n >= sizeof H->fault) n = sizeof H->fault - 1;
	for (size_t i = 0; i < n; i++) H->fault[i] = s[i];
	H->fault[n] = 0;
	H->fault_valid = 1;
}

const char *log_fault(void)
{
	return (H && H->fault_valid) ? (const char *)H->fault : NULL;
}

uint32_t log_boots(void) { return boots_now; }
