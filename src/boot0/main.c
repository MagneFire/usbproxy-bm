/* boot0 main: what the U-Boot SPL did for the appliance, and nothing else.
 *
 * Card boot (the BROM read us from sector 16):
 *   1. RTC GP1 death counter: 0xB00700NN means "firmware boot NN in flight";
 *      the firmware clears GP1 when the host has configured the gadget. Three
 *      consecutive deaths -> BROM FEL (sunxi-fel recovery over the OTG port).
 *   2. 8 s watchdog, GP2 = entry stamp, clocks, UART banner, DRAM.
 *   3. The firmware uImage at sector 0x18800: magic, size, CRC32 -> DRAM,
 *      GP3 = jump stamp, jump. A bad image -> FEL at once.
 * FEL boot (sunxi-fel `spl` patched the magic): clocks + DRAM, size into the
 *   header for sunxi-fel, return to the BROM: `sunxi-fel write/exe` then run
 *   the firmware from RAM, or felflash writes the card. */
#include "boot0/boot0.h"
#include "gitrev.h"
#include <stdarg.h>
#include <string.h>

#define GP1_DEATH_TAG   0xB0070000u
#define GP1_DEATH_MASK  0xFFFFFF00u
#define MAX_DEATHS      3
#define WHY_BAD_IMAGE   1
#define WHY_DEATHS      2
#define WHY_DRAM        3
#define WHY_MMC         4

static void outf(const char *fmt, ...)
{
	char b[160];
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(b, sizeof b, fmt, ap);
	va_end(ap);
	if (n > 0) uart_puts(b);
}

/* Into FEL the proven way (U-Boot patch 0004, the firmware's `fel`): the
 * request word in GP1 and a watchdog reset, so the jump to the BROM happens
 * from boot0's first instructions with the BROM's own state intact. A jump
 * from here, after clocks and SD traffic, made the BROM retry the card boot
 * instead (seen 2026-10-10). GP3 keeps the reason for `bm check`. */
static void __attribute__((noreturn)) enter_fel(unsigned why)
{
	rtc_gp_write(3, 0xB0070000u | why);
	rtc_gp_write(1, RTC_MAGIC_FEL1);
	outf("boot0: -> FEL (why %u) via reset\n", why);
	wdt_reset_now();
}

static void __attribute__((noreturn)) boot_card(void)
{
	uint32_t t0 = time_ms();
	uint32_t g1 = rtc_gp_read(1);
	unsigned deaths = (g1 & GP1_DEATH_MASK) == GP1_DEATH_TAG ? (g1 & 0xff) + 1 : 0;

	wdt_arm();
	rtc_gp_write(2, 0xB0000000u | (t0 & 0xffffffu));
	clock_init();
	uart_init();
	outf("\nboot0 " GITREV ": entry %lu ms, gp0=%08lx gp1=%08lx\n",
	     (unsigned long)t0, (unsigned long)rtc_gp_read(0), (unsigned long)g1);
	if (deaths >= MAX_DEATHS) {
		outf("boot0: %u firmware boots died in a row\n", deaths);
		enter_fel(WHY_DEATHS);
	}
	rtc_gp_write(1, GP1_DEATH_TAG | deaths);

	unsigned long size = dram_init();
	outf("boot0: DRAM %lu MiB in %lu ms\n", size >> 20, (unsigned long)(time_ms() - t0));
	if (!size) enter_fel(WHY_DRAM);

	int err = mmc_init();
	if (err) {
		outf("boot0: SD init failed (%d)\n", err);
		enter_fel(WHY_MMC);
	}

	/* The firmware's legacy uImage: header sector first (parked at the
	 * bottom of DRAM), then the data straight to its load address. */
	struct image_header *h = (struct image_header *)H3_DRAM;
	if (mmc_read(FW_SECTOR, 1, h)) { outf("boot0: header read failed\n"); enter_fel(WHY_MMC); }
	uint32_t fsize = be32(h->size), ep = be32(h->ep), load = be32(h->load);
	if (be32(h->magic) != IH_MAGIC || fsize == 0 || fsize > FW_MAX_BYTES ||
	    load < H3_DRAM + 0x8000 || load > H3_DRAM + (64u << 20) || ep != load) {
		outf("boot0: no firmware image at sector 0x%lx (magic %08lx size %lu load %08lx)\n",
		     (unsigned long)FW_SECTOR, (unsigned long)be32(h->magic), (unsigned long)fsize, (unsigned long)load);
		enter_fel(WHY_BAD_IMAGE);
	}
	/* The data follows the 64-byte header inside the first sector: read
	 * the whole file to load - 64 and the data lands at its load address. */
	uint32_t sectors = (64 + fsize + 511) / 512;
	uint32_t t1 = time_ms();
	uint8_t *data = (uint8_t *)load;
	if (mmc_read(FW_SECTOR, sectors, data - 64)) { outf("boot0: image read failed\n"); enter_fel(WHY_MMC); }
	uint32_t crc = crc32_buf(0, data, fsize);
	if (crc != be32(h->dcrc)) {
		outf("boot0: firmware CRC %08lx, header says %08lx\n", (unsigned long)crc, (unsigned long)be32(h->dcrc));
		enter_fel(WHY_BAD_IMAGE);
	}
	char name[33];
	memcpy(name, h->name, 32);
	name[32] = 0;
	uint32_t t2 = time_ms();
	outf("boot0: %s, %lu bytes, crc ok, read %lu ms; jump at %lu ms\n",
	     name, (unsigned long)fsize, (unsigned long)(t2 - t1), (unsigned long)t2);
	uart_flush();
	writel(0, H3_CCU + 0x088);                  /* SD clock off, the firmware never reads the card */
	clrbits(H3_CCU + 0x060, BIT(8));
	rtc_gp_write(3, t2 & 0xffffffu);
	wdt_feed();
	__asm__ volatile("mcr p15, 0, %0, c14, c0, 0" :: "r"(CNT_HZ));    /* CNTFRQ, as the SPL did */
	boot0_jump(ep);
}

static void __attribute__((noreturn)) boot_fel(void)
{
	uint32_t t0 = time_ms();
	clock_init();
	uart_init();
	outf("\nboot0 " GITREV " (FEL): entry %lu ms\n", (unsigned long)t0);
	unsigned long size = dram_init();
	outf("boot0: DRAM %lu MiB in %lu ms\n", size >> 20, (unsigned long)(time_ms() - t0));
	volatile uint32_t *dram_size_field = (volatile uint32_t *)0x24;   /* header field, sunxi-fel prints it */
	*dram_size_field = size >> 20;
	/* Card probe, so the SD driver and the card layout can be checked
	 * over FEL without writing anything: our own header at sector 16,
	 * the firmware's at 0x18800. */
	uint32_t t1 = time_ms();
	int err = mmc_init();
	if (err) {
		outf("boot0: card: init failed (%d)\n", err);
	} else {
		struct image_header *h = (struct image_header *)H3_DRAM;
		uint32_t *egon = (uint32_t *)(H3_DRAM + 512);
		if (mmc_read(BOOT0_SECTOR, 1, egon))
			outf("boot0: card: sector 16 read failed\n");
		else
			outf("boot0: card: sector 16 %s, boot0 length %lu\n",
			     memcmp(egon + 1, "eGON.BT0", 8) ? "is not an eGON image" : "eGON.BT0", (unsigned long)egon[4]);
		if (mmc_read(FW_SECTOR, 1, h))
			outf("boot0: card: sector 0x%lx read failed\n", (unsigned long)FW_SECTOR);
		else if (be32(h->magic) != IH_MAGIC)
			outf("boot0: card: no uImage at sector 0x%lx\n", (unsigned long)FW_SECTOR);
		else {
			char name[33];
			memcpy(name, h->name, 32); name[32] = 0;
			uint32_t fsize = be32(h->size), sectors = (64 + fsize + 511) / 512;
			uint8_t *data = (uint8_t *)be32(h->load);
			if (mmc_read(FW_SECTOR, sectors, data - 64))
				outf("boot0: card: image read failed\n");
			else
				outf("boot0: card: %s, %lu bytes, crc %s, %lu ms\n", name, (unsigned long)fsize,
				     crc32_buf(0, data, fsize) == be32(h->dcrc) ? "ok" : "BAD", (unsigned long)(time_ms() - t1));
		}
	}
	outf("boot0: back to FEL\n");
	uart_flush();
	return_to_fel(fel_stash.sp, fel_stash.lr);
}

void __attribute__((noreturn)) boot0_main(void)
{
	if (memcmp((const void *)4, "eGON.BT0", 8) == 0)
		boot_card();
	boot_fel();
}
