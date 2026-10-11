#ifndef FW_H
#define FW_H
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "h3/h3.h"

#include "gitrev.h"            /* generated: build/gitrev.h */
#define IMAGE_BASE ((uintptr_t)_start)
extern char _start[];

/* lib */
int vsnprintf(char *buf, size_t n, const char *fmt, va_list ap);
int snprintf(char *buf, size_t n, const char *fmt, ...);
unsigned long strtoul(const char *s, char **end, int base);

/* log ring (console/log.c): DRAM, uncached, survives a warm reset */
void log_init(void);
void log_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void log_write(const char *s, size_t n);
uint32_t log_total(void);
void log_dump_at(uint32_t from, size_t n, void (*out)(const char *));
void log_dump(size_t last, void (*out)(const char *));
void log_set_fault(const char *s);
const char *log_fault(void);
uint32_t log_boots(void);

/* usb */
const char *gadget_serial(void);

/* console */
void console_init(void);
void console_poll(void);

/* main */
extern uint32_t g_entry_ms;
extern int g_up;                /* 1 once GP1 has been cleared */
extern bool g_manual_detach;    /* console `detach`: no automatic re-present */
extern uint32_t g_represents, g_fs_retries;
#endif
