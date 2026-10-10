/* felflash: one card region per call. The Mac (scripts/appliance.py bm
 * flash) puts the data in DRAM, fills the command block, runs us, reads the
 * status back; boot0's FEL run has already set the clocks and DRAM up.
 *
 *   command block at FELFLASH_CMD:
 *     u32 magic   "FLSH"
 *     u32 sector  first card sector
 *     u32 count   sectors (data is count * 512 bytes)
 *     u32 crc     crc32 of the data as the Mac computed it
 *     u32 data    DRAM address of the data
 *     u32 status  written by us: 0 ok, else the step that failed (below)
 *     u32 ms      time taken
 * The readback goes to data + count * 512 and must match the crc too. */
#include "boot0/boot0.h"
#include <stdarg.h>

#define FELFLASH_CMD    0x4ff10000u
#define FLSH_MAGIC      0x48534c46u

#define ST_OK           0
#define ST_BAD_MAGIC    1
#define ST_BAD_CRC      2       /* the data did not arrive intact */
#define ST_MMC_INIT     3
#define ST_WRITE        4
#define ST_READBACK     5
#define ST_VERIFY       6       /* readback crc mismatch */

struct cmdblk { uint32_t magic, sector, count, crc, data, status, ms; };

static void outf(const char *fmt, ...)
{
	char b[160];
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(b, sizeof b, fmt, ap);
	va_end(ap);
	if (n > 0) uart_puts(b);
}

static uint32_t run(volatile struct cmdblk *c)
{
	if (c->magic != FLSH_MAGIC) return ST_BAD_MAGIC;
	const uint8_t *data = (const uint8_t *)c->data;
	uint32_t bytes = c->count * 512;
	outf("felflash: %lu sectors at 0x%lx from %08lx\n", (unsigned long)c->count, (unsigned long)c->sector, (unsigned long)c->data);
	if (crc32_buf(0, data, bytes) != c->crc) return ST_BAD_CRC;
	int err = mmc_init();
	if (err) { outf("felflash: mmc init %d\n", err); return ST_MMC_INIT; }
	if (mmc_write(c->sector, c->count, data)) return ST_WRITE;
	uint8_t *back = (uint8_t *)(c->data + bytes);
	if (mmc_read(c->sector, c->count, back)) return ST_READBACK;
	if (crc32_buf(0, back, bytes) != c->crc) return ST_VERIFY;
	return ST_OK;
}

void felflash_main(void)
{
	volatile struct cmdblk *c = (volatile struct cmdblk *)FELFLASH_CMD;
	uint32_t t0 = time_ms();
	uart_init();
	c->status = 0xffffffffu;
	uint32_t st = run(c);
	c->ms = time_ms() - t0;
	c->status = st;
	outf("felflash: status %lu in %lu ms\n", (unsigned long)st, (unsigned long)c->ms);
	uart_flush();
}
