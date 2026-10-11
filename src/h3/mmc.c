/* SDC0 (the SD slot), PIO through the AHB, written from U-Boot's
 * sunxi_mmc.c (register map, command/transfer sequencing) and mmc.c (the SD
 * initialisation dialogue). 400 kHz 1-bit for the dialogue, 25 MHz 4-bit
 * from PLL6 for transfers. SDHC/SDXC (block addressing) and SDSC (byte
 * addressing) both handled; no MMC/eMMC, no high-speed switch. */
#include "boot0/boot0.h"

#define MMC0            0x01c0f000u
#define R_GCTRL         (MMC0 + 0x00)
#define R_CLKCR         (MMC0 + 0x04)
#define R_TIMEOUT       (MMC0 + 0x08)
#define R_WIDTH         (MMC0 + 0x0c)
#define R_BLKSZ         (MMC0 + 0x10)
#define R_BYTECNT       (MMC0 + 0x14)
#define R_CMD           (MMC0 + 0x18)
#define R_ARG           (MMC0 + 0x1c)
#define R_RESP0         (MMC0 + 0x20)
#define R_RINT          (MMC0 + 0x38)
#define R_STATUS        (MMC0 + 0x3c)
#define R_FIFO          (MMC0 + 0x200)

#define GCTRL_RESET     0x7u                    /* soft + FIFO + DMA reset */
#define GCTRL_ACCESS_BY_AHB (1u << 31)
#define CLK_ENABLE      (1u << 16)
#define CMD_START       (1u << 31)
#define CMD_UPCLK_ONLY  (1u << 21)
#define CMD_SEND_INIT   (1u << 15)
#define CMD_WAIT_PRE    (1u << 13)
#define CMD_AUTO_STOP   (1u << 12)
#define CMD_WRITE       (1u << 10)
#define CMD_DATA        (1u << 9)
#define CMD_CHK_CRC     (1u << 8)
#define CMD_LONG_RESP   (1u << 7)
#define CMD_RESP        (1u << 6)
#define RINT_CMD_DONE   (1u << 2)
#define RINT_DATA_OVER  (1u << 3)
#define RINT_AUTO_DONE  (1u << 14)
#define RINT_ERROR      0xbfc2u
#define ST_FIFO_EMPTY   (1u << 2)
#define ST_FIFO_FULL    (1u << 3)
#define ST_CARD_BUSY    (1u << 9)
#define ST_FIFO_LEVEL(s) (((s) >> 17) & 0x3fff)

#ifndef MMC_HZ
#define MMC_HZ          25000000u
#endif
#ifndef MMC_WIDTH
#define MMC_WIDTH       4
#endif

/* response kinds */
#define RSP_NONE  0
#define RSP_R1    (CMD_RESP | CMD_CHK_CRC)
#define RSP_R1B   (CMD_RESP | CMD_CHK_CRC | 0x80000000u)   /* bit 31 reused as "busy after" marker, stripped below */
#define RSP_R2    (CMD_RESP | CMD_CHK_CRC | CMD_LONG_RESP)
#define RSP_R3    (CMD_RESP)
#define RSP_R6    RSP_R1
#define RSP_R7    RSP_R1

static bool high_capacity;
static uint32_t rca;

static void dump(const char *tag)
{
	char b[160];
	snprintf(b, sizeof b, "mmc: %s: gctrl=%08lx clkcr=%08lx cmd=%08lx rint=%08lx status=%08lx sd0clk=%08lx\n", tag,
		 (unsigned long)readl(R_GCTRL), (unsigned long)readl(R_CLKCR), (unsigned long)readl(R_CMD),
		 (unsigned long)readl(R_RINT), (unsigned long)readl(R_STATUS), (unsigned long)readl(CCU_SD0_CLK));
	uart_puts(b);
}

static int update_clk(void)
{
	uint32_t t0 = time_ms();
	writel(CMD_START | CMD_UPCLK_ONLY | CMD_WAIT_PRE, R_CMD);
	while (readl(R_CMD) & CMD_START)
		if (time_ms() - t0 > 2000) { dump("update_clk timeout"); return -1; }
	writel(readl(R_RINT), R_RINT);
	return 0;
}

/* Module clock from the 24 MHz oscillator or PLL6; U-Boot's delays for <= 25 MHz. */
static int set_clock(uint32_t hz, unsigned width)
{
	/* mod clock = src / 2^N / M, M <= 16; src = OSC24M up to 24 MHz, else
	 * PLL6 (600 MHz, clock.c), as U-Boot's mmc_set_mod_clk. 24 MHz straight
	 * from the oscillator (N=0, M=1) gave response errors on every command;
	 * 25 MHz = PLL6 / 2 / 12 with U-Boot's delays (oclk 0, sclk 5) is the
	 * proven SPL setting. */
	uint32_t src = hz <= 24000000u ? 24000000u : 600000000u, pll = hz <= 24000000u ? 0 : (1u << 24);
	uint32_t div = (src + hz - 1) / hz, n = 0, dly = hz <= 400000 ? 0 : (5u << 20);
	while (div > 16) { n++; div = (div + 1) / 2; }
	writel(readl(R_CLKCR) & ~CLK_ENABLE, R_CLKCR);
	if (update_clk()) return -1;
	writel((1u << 31) | pll | (n << 16) | (div - 1) | dly, CCU_SD0_CLK);
	writel(readl(R_CLKCR) & ~0xffu, R_CLKCR);
	writel(readl(R_CLKCR) | CLK_ENABLE, R_CLKCR);
	if (update_clk()) return -1;
	writel(width == 4 ? 1 : 0, R_WIDTH);
	return 0;
}

static int wait_rint(uint32_t done, uint32_t timeout_ms)
{
	uint32_t t0 = time_ms(), st;
	do {
		st = readl(R_RINT);
		if (st & RINT_ERROR) return -2;
		if (time_ms() - t0 > timeout_ms) return -1;
	} while (!(st & done));
	return 0;
}

/* One command, optional data phase through the FIFO. resp[4] for R2. */
static int cmd(uint32_t idx, uint32_t arg, uint32_t rsp, uint32_t *resp,
	       void *buf, uint32_t blocks, bool write)
{
	bool busy_after = rsp & 0x80000000u;
	uint32_t v = CMD_START | (idx & 0x3f) | (rsp & 0x7fffffffu);
	int err = 0;
	if (idx == 0) v |= CMD_SEND_INIT;
	if (buf) {
		v |= CMD_DATA | CMD_WAIT_PRE | (write ? CMD_WRITE : 0) | (blocks > 1 ? CMD_AUTO_STOP : 0);
		writel(512, R_BLKSZ);
		writel(blocks * 512, R_BYTECNT);
	}
	writel(arg, R_ARG);
	writel(v, R_CMD);
	if (buf) {
		uint32_t *w = buf, n = blocks * 128, t0 = time_ms();
		setbits(R_GCTRL, GCTRL_ACCESS_BY_AHB);
		for (uint32_t i = 0; i < n;) {
			uint32_t st;
			while ((st = readl(R_STATUS)) & (write ? ST_FIFO_FULL : ST_FIFO_EMPTY))
				if (time_ms() - t0 > 2000) { err = -3; goto out; }
			if (write) { writel(w[i++], R_FIFO); continue; }
			uint32_t in = ST_FIFO_LEVEL(st);
			if (!in) in = 1;
			for (; in && i < n; in--) w[i++] = readl(R_FIFO);
		}
		dmb();
	}
	err = wait_rint(RINT_CMD_DONE, 1000);
	if (err) goto out;
	if (buf) {
		err = wait_rint(blocks > 1 ? RINT_AUTO_DONE : RINT_DATA_OVER, 2000);
		if (err) goto out;
	}
	if (busy_after || write) {
		uint32_t t0 = time_ms();
		while (readl(R_STATUS) & ST_CARD_BUSY)
			if (time_ms() - t0 > 2000) { err = -4; goto out; }
	}
	if (resp) {
		if (rsp & CMD_LONG_RESP) {
			resp[0] = readl(R_RESP0 + 12); resp[1] = readl(R_RESP0 + 8);
			resp[2] = readl(R_RESP0 + 4);  resp[3] = readl(R_RESP0);
		} else {
			resp[0] = readl(R_RESP0);
		}
	}
out:
	if (err) {
		char tag[32];
		snprintf(tag, sizeof tag, "cmd%lu err %d", (unsigned long)idx, err);
		dump(tag);
		writel(GCTRL_RESET, R_GCTRL);
		update_clk();
	}
	writel(0xffffffffu, R_RINT);
	setbits(R_GCTRL, 1u << 1);                  /* FIFO reset */
	return err;
}

static void pins_init(void)
{
	/* PF0-PF5: function 2 (SDC0), pull-up, drive level 2. The BROM set
	 * them to read us; set them again, the cost is nil. */
	uintptr_t pf = PIO_PORT(5);
	uint32_t cfg = readl(PIO_CFG(pf, 0));
	for (int pin = 0; pin < 6; pin++) {
		cfg &= ~(0xfu << (pin * 4));
		cfg |= 2u << (pin * 4);
	}
	writel(cfg, PIO_CFG(pf, 0));
	uint32_t drv = readl(pf + 0x14), pull = readl(pf + 0x1c);
	for (int pin = 0; pin < 6; pin++) {
		drv  = (drv  & ~(3u << (pin * 2))) | (2u << (pin * 2));
		pull = (pull & ~(3u << (pin * 2))) | (1u << (pin * 2));
	}
	writel(drv, pf + 0x14);
	writel(pull, pf + 0x1c);
}

int mmc_init(void)
{
	uint32_t r[4];
	pins_init();
	setbits(CCU_BUS_GATE0, BIT(8));
	setbits(CCU_BUS_RST0, BIT(8));
	writel((1u << 31) | 0u, CCU_SD0_CLK);       /* OSC24M /1 while the controller resets */
	writel(GCTRL_RESET, R_GCTRL);
	delay_us(1000);
	writel(0xffffff40u, R_TIMEOUT);
	if (set_clock(400000, 1)) return -10;

	if (cmd(0, 0, RSP_NONE, 0, 0, 0, false)) return -11;        /* GO_IDLE_STATE */
	delay_us(2000);
	bool v2 = cmd(8, 0x1aa, RSP_R7, r, 0, 0, false) == 0 && (r[0] & 0xff) == 0xaa;  /* SEND_IF_COND */
	uint32_t t0 = time_ms();
	for (;;) {                                                  /* ACMD41 until ready */
		if (cmd(55, 0, RSP_R1, r, 0, 0, false)) return -12;
		if (cmd(41, (v2 ? 0x40000000u : 0) | 0x00ff8000u, RSP_R3, r, 0, 0, false)) return -13;
		if (r[0] & 0x80000000u) break;
		if (time_ms() - t0 > 1000) return -14;
		delay_us(1000);
	}
	high_capacity = (r[0] & 0x40000000u) != 0;
	if (cmd(2, 0, RSP_R2, r, 0, 0, false)) return -15;         /* ALL_SEND_CID */
	if (cmd(3, 0, RSP_R6, r, 0, 0, false)) return -16;         /* SEND_RELATIVE_ADDR */
	rca = r[0] >> 16;
	if (cmd(7, rca << 16, RSP_R1B, r, 0, 0, false)) return -17; /* SELECT_CARD */
	if (cmd(55, rca << 16, RSP_R1, r, 0, 0, false)) return -18;
	if (MMC_WIDTH == 4 && cmd(6, 2, RSP_R1, r, 0, 0, false)) return -19;   /* ACMD6: 4-bit bus */
	if (set_clock(MMC_HZ, MMC_WIDTH)) return -20;
	if (cmd(16, 512, RSP_R1, r, 0, 0, false)) return -21;       /* SET_BLOCKLEN */
	return 0;
}

static uint32_t addr_of(uint32_t sector) { return high_capacity ? sector : sector * 512; }

int mmc_read(uint32_t sector, uint32_t n, void *dst)
{
	uint32_t r;
	uint8_t *p = dst;
	while (n) {
		uint32_t chunk = n > 64 ? 64 : n;       /* 32 KiB per command, bytecnt is 16 bits of blocks anyway */
		if (cmd(chunk > 1 ? 18 : 17, addr_of(sector), RSP_R1, &r, p, chunk, false)) return -1;
		sector += chunk; p += chunk * 512; n -= chunk;
	}
	return 0;
}

int mmc_write(uint32_t sector, uint32_t n, const void *src)
{
	uint32_t r;
	const uint8_t *p = src;
	while (n) {
		uint32_t chunk = n > 64 ? 64 : n;
		if (cmd(chunk > 1 ? 25 : 24, addr_of(sector), RSP_R1, &r, (void *)p, chunk, true)) return -1;
		sector += chunk; p += chunk * 512; n -= chunk;
	}
	return 0;
}
