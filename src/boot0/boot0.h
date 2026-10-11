/* boot0 internals: the pieces of the firmware tree it links (uart, ccu,
 * wdt, rtc, timer, libc, fmt) come with h3.h; these are its own. */
#ifndef BOOT0_H
#define BOOT0_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "h3/h3.h"

/* lib */
#include <stdarg.h>
int snprintf(char *buf, size_t n, const char *fmt, ...);
int vsnprintf(char *buf, size_t n, const char *fmt, va_list ap);
uint32_t crc32_buf(uint32_t crc, const void *buf, size_t n);  /* zlib polynomial, start with 0 */

/* start.S */
struct fel_stash { uint32_t sp, lr, cpsr, sctlr, vbar; };
extern struct fel_stash fel_stash;
void return_to_fel(uint32_t sp, uint32_t lr) __attribute__((noreturn));
void boot0_jump(uint32_t entry) __attribute__((noreturn));
void boot0_fel_entry(void) __attribute__((noreturn));

/* h3/clock.c: PLL1 (CPU), PLL6 + AHB1/APB1/APB2 (periph), MBUS. */
void clock_init(void);

/* h3/dram.c: U-Boot's dram_sunxi_dw.c for the H3, DDR3 at DRAM_CLK MHz.
 * Returns the size in bytes, 0 when the controller never came up. */
#define DRAM_CLK 312
unsigned long dram_init(void);

/* h3/mmc.c: SDC0, PIO, 4-bit at 25 MHz. 512-byte sectors. */
int mmc_init(void);                                     /* 0 = card ready */
int mmc_read(uint32_t sector, uint32_t n, void *dst);   /* 0 = ok */
int mmc_write(uint32_t sector, uint32_t n, const void *src);

/* U-Boot legacy image header (big-endian fields), the firmware's fw.uimg. */
#define IH_MAGIC 0x27051956u
struct image_header {
	uint32_t magic, hcrc, time, size, load, ep, dcrc;
	uint8_t os, arch, type, comp;
	char name[32];
};
static inline uint32_t be32(uint32_t v) { return __builtin_bswap32(v); }

#define FW_SECTOR       0x18800u        /* 49 MiB: scripts/applib.py KERNEL_SECTOR, the SPL's raw kernel region */
#define FW_MAX_BYTES    (15u << 20)     /* the region's window, 0x7800 sectors */
#define BOOT0_SECTOR    0x10u           /* 8 KiB: where the BROM reads us from */
#endif
