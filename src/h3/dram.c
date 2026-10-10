/* H3 DRAM controller (DesignWare) init: U-Boot v2024.01 dram_sunxi_dw.c,
 * dram_helpers.c and dram_timings/ddr3_1333.c reduced to the H3 / DDR3 /
 * 32-bit path, with meta-usbproxy's U-Boot patch 0001 (16-word pattern size
 * detection: the single write pair intermittently reported 512 MiB on this
 * 256 MiB board). Parameters as the appliance's U-Boot build: DRAM_CLK 312
 * (usbproxy-uboot.cfg: the stability floor), ZQ 3881979, ODT on.
 *
 * SPDX-License-Identifier: GPL-2.0+
 * (C) Copyright 2007-2015 Allwinner Technology Co.
 * (C) Copyright 2015 Vishnu Patekar, Hans de Goede, Jens Kuske */
#include "boot0/boot0.h"

#define DRAM_ZQ         3881979u
#define DRAM_ODT_EN     1

#define MCTL_COM        0x01c62000u
#define MCTL_CTL        0x01c63000u
#define SRAMC           0x01c00000u

/* mctl_com */
#define COM_CR          (MCTL_COM + 0x00)
#define COM_MCR(p, i)   (MCTL_COM + 0x10 + 8 * (p) + 4 * (i))
#define COM_BWCR        (MCTL_COM + 0x90)
#define COM_MAER        (MCTL_COM + 0x94)
#define COM_MAPR        (MCTL_COM + 0x98)
#define COM_CCCR        (MCTL_COM + 0xd0)
#define COM_PROTECT     (MCTL_COM + 0x800)
#define PROTECT_MAGIC   0x94be6fa3u

/* mctl_ctl */
#define CTL_PIR         (MCTL_CTL + 0x000)
#define CTL_CLKEN       (MCTL_CTL + 0x00c)
#define CTL_PGSR0       (MCTL_CTL + 0x010)
#define CTL_STATR       (MCTL_CTL + 0x018)
#define CTL_MR(n)       (MCTL_CTL + 0x030 + 4 * (n))
#define CTL_PTR(n)      (MCTL_CTL + 0x044 + 4 * (n))
#define CTL_DRAMTMG(n)  (MCTL_CTL + 0x058 + 4 * (n))
#define CTL_ODTCFG      (MCTL_CTL + 0x07c)
#define CTL_PITMG0      (MCTL_CTL + 0x080)
#define CTL_RFSHCTL0    (MCTL_CTL + 0x08c)
#define CTL_RFSHTMG     (MCTL_CTL + 0x090)
#define CTL_DTCR        (MCTL_CTL + 0x0c0)
#define CTL_PGCR(n)     (MCTL_CTL + 0x100 + 4 * (n))
#define CTL_ODTMAP      (MCTL_CTL + 0x120)
#define CTL_ZQCR        (MCTL_CTL + 0x140)
#define CTL_ZQDR(n)     (MCTL_CTL + 0x148 + 4 * (n))
#define CTL_ACIOCR      (MCTL_CTL + 0x208)
#define CTL_ACBDLR(n)   (MCTL_CTL + 0x210 + 4 * (n))
#define CTL_DX_BDLR(l, n) (MCTL_CTL + 0x300 + 0x80 * (l) + 0x10 + 4 * (n))
#define CTL_DX_GCR(l)   (MCTL_CTL + 0x300 + 0x80 * (l) + 0x44)
#define CTL_DX_GSR0(l)  (MCTL_CTL + 0x300 + 0x80 * (l) + 0x48)
#define CTL_UPD2        (MCTL_CTL + 0x888)

#define PIR_CLRSR       (1u << 27)
#define PIR_QSGATE      (1u << 10)
#define PIR_DRAMINIT    (1u << 8)
#define PIR_DRAMRST     (1u << 7)
#define PIR_PHYRST      (1u << 6)
#define PIR_DCAL        (1u << 5)
#define PIR_PLLINIT     (1u << 4)
#define PIR_ZCAL        (1u << 1)
#define PIR_INIT        (1u << 0)
#define PGSR_INIT_DONE  (1u << 0)
#define ZQCR_PWRDOWN    (1u << 31)

/* CCU */
#define CCU_PLL5_CFG    (H3_CCU + 0x020)
#define CCU_AHB_GATE0   (H3_CCU + 0x060)
#define CCU_DRAM_CLK    (H3_CCU + 0x0f4)
#define CCU_MBUS_RESET  (H3_CCU + 0x0fc)
#define CCU_MBUS0_CLK   (H3_CCU + 0x15c)
#define CCU_PLL5_TUNING (H3_CCU + 0x260)
#define CCU_AHB_RESET0  (H3_CCU + 0x2c0)
#define AHB_MCTL        BIT(14)

#define MCTL_CR_BL8         (0x4u << 20)
#define MCTL_CR_2T          (0x0u << 19)
#define MCTL_CR_DDR3        (0x3u << 16)
#define MCTL_CR_INTERLEAVED (0x0u << 15)
#define MCTL_CR_BUS_FULL_WIDTH(x) ((x) << 12)
#define MCTL_CR_PAGE_SIZE(x)   ((uint32_t)(32 - __builtin_clz(x) - 4) << 8)   /* (fls(x) - 4) << 8 */
#define MCTL_CR_ROW_BITS(x)    (((x) - 1) << 4)
#define MCTL_CR_EIGHT_BANKS (0x1u << 2)
#define MCTL_CR_FOUR_BANKS  (0x0u << 2)
#define MCTL_CR_DUAL_RANK   (0x1u << 0)
#define MCTL_CR_SINGLE_RANK (0x0u << 0)

static inline void clrsetbits(uintptr_t a, uint32_t clr, uint32_t set) { writel((readl(a) & ~clr) | set, a); }
static inline void udelay(uint32_t us) { delay_us(us); }

struct rank_para { uint16_t page_size; uint8_t row_bits, bank_bits; };
struct dram_para {
	uint8_t dual_rank, bus_full_width;
	struct rank_para ranks[2];
	const uint8_t dx_read_delays[4][11];
	const uint8_t dx_write_delays[4][11];
	const uint8_t ac_delays[31];
};

/* Wait up to 1 s for (reg & mask) == val. */
static bool await(uintptr_t reg, uint32_t mask, uint32_t val)
{
	uint32_t t0 = time_ms();
	while ((readl(reg) & mask) != val)
		if (time_ms() - t0 > 1000) return false;
	return true;
}

static bool phy_init(uint32_t val)
{
	writel(val | PIR_INIT, CTL_PIR);
	return await(CTL_PGSR0, PGSR_INIT_DONE, 1);
}

static void set_bit_delays(const struct dram_para *para)
{
	clrbits(CTL_PGCR(0), 1u << 26);
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 11; j++)
			writel((uint32_t)para->dx_write_delays[i][j] << 8 | para->dx_read_delays[i][j], CTL_DX_BDLR(i, j));
	for (int i = 0; i < 31; i++)
		writel((uint32_t)para->ac_delays[i] << 8, CTL_ACBDLR(i));
	setbits(CTL_PGCR(0), 1u << 26);
}

static void mbus_port(unsigned port, bool bwlimit, unsigned qos, unsigned acs, uint16_t bwl0, uint16_t bwl1, uint16_t bwl2)
{
	writel((bwlimit ? 1u : 0) | ((qos & 3) << 2) | ((acs & 0xff) << 8) | ((uint32_t)bwl0 << 16), COM_MCR(port, 0));
	writel(((uint32_t)bwl2 << 16) | bwl1, COM_MCR(port, 1));
}

enum { QOS_LOWEST, QOS_LOW, QOS_HIGH, QOS_HIGHEST };
static void set_master_priority_h3(void)
{
	writel((1u << 16) | 400u, COM_BWCR);        /* bandwidth limit windows, 1 us */
	writel(1, COM_MAPR);                        /* CPU high priority */
	mbus_port( 0, true, QOS_HIGHEST, 0,  512,  256,  128);  /* CPU */
	mbus_port( 1, true, QOS_HIGH,    0, 1536, 1024,  256);  /* GPU */
	mbus_port( 2, true, QOS_HIGHEST, 0,  512,  256,   96);  /* unused */
	mbus_port( 3, true, QOS_HIGHEST, 0,  256,  128,   32);  /* DMA */
	mbus_port( 4, true, QOS_HIGH,    0, 1792, 1600,  256);  /* VE */
	mbus_port( 5, true, QOS_HIGHEST, 0,  256,  128,   32);  /* CSI */
	mbus_port( 6, true, QOS_HIGH,    0,  256,  128,   64);  /* NAND */
	mbus_port( 7, true, QOS_HIGHEST, 0,  256,  128,   64);  /* SS */
	mbus_port( 8, true, QOS_HIGHEST, 0,  256,  128,   64);  /* TS */
	mbus_port( 9, true, QOS_HIGH,    0, 1024,  256,   64);  /* DI */
	mbus_port(10, true, QOS_HIGHEST, 3, 8192, 6120, 1024);  /* DE */
	mbus_port(11, true, QOS_HIGH,    0, 1024,  288,   64);  /* DE_CFD */
}

static uint32_t bin_to_mgray(int val)
{
	static const uint8_t t[32] = {
		0x00, 0x01, 0x02, 0x03, 0x06, 0x07, 0x04, 0x05, 0x0c, 0x0d, 0x0e, 0x0f, 0x0a, 0x0b, 0x08, 0x09,
		0x18, 0x19, 0x1a, 0x1b, 0x1e, 0x1f, 0x1c, 0x1d, 0x14, 0x15, 0x16, 0x17, 0x12, 0x13, 0x10, 0x11,
	};
	return t[val < 0 ? 0 : val > 31 ? 31 : val];
}

static int mgray_to_bin(uint32_t val)
{
	static const uint8_t t[32] = {
		0x00, 0x01, 0x02, 0x03, 0x06, 0x07, 0x04, 0x05, 0x0e, 0x0f, 0x0c, 0x0d, 0x08, 0x09, 0x0a, 0x0b,
		0x1e, 0x1f, 0x1c, 0x1d, 0x18, 0x19, 0x1a, 0x1b, 0x10, 0x11, 0x12, 0x13, 0x16, 0x17, 0x14, 0x15,
	};
	return t[val & 0x1f];
}

static bool zq_calibration_quirk(void)
{
	if ((readl(SRAMC + 0x24) & 0xff) == 0 && (readl(SRAMC + 0xf0) & 0x1) == 0) {
		uint32_t v;
		clrsetbits(CTL_ZQCR, 0xffff, DRAM_ZQ & 0xffff);
		writel(PIR_CLRSR, CTL_PIR);
		if (!phy_init(PIR_ZCAL)) return false;
		v = readl(CTL_ZQDR(0)) & ((0x1f << 16) | 0x1f);
		v |= v << 8;
		writel(v, CTL_ZQDR(0));
		v = readl(CTL_ZQDR(1)) & ((0x1f << 16) | 0x1f);
		v |= v << 8;
		writel(v, CTL_ZQDR(1));
		writel(v, CTL_ZQDR(2));
	} else {
		uint16_t zq_val[6];
		writel(0x0a0a0a0au, CTL_ZQDR(2));
		for (int i = 0; i < 6; i++) {
			uint32_t zq = (DRAM_ZQ >> (i * 4)) & 0xf;
			writel((zq << 20) | (zq << 16) | (zq << 12) | (zq << 8) | (zq << 4) | zq, CTL_ZQCR);
			writel(PIR_CLRSR, CTL_PIR);
			if (!phy_init(PIR_ZCAL)) return false;
			zq_val[i] = readl(CTL_ZQDR(0)) & 0xff;
			writel(0x01010101u * zq_val[i], CTL_ZQDR(2));
			writel(PIR_CLRSR, CTL_PIR);
			if (!phy_init(PIR_ZCAL)) return false;
			uint8_t val = readl(CTL_ZQDR(0)) >> 24;
			zq_val[i] |= bin_to_mgray(mgray_to_bin(val) - 1) << 8;
		}
		writel(((uint32_t)zq_val[1] << 16) | zq_val[0], CTL_ZQDR(0));
		writel(((uint32_t)zq_val[3] << 16) | zq_val[2], CTL_ZQDR(1));
		writel(((uint32_t)zq_val[5] << 16) | zq_val[4], CTL_ZQDR(2));
	}
	return true;
}

static void set_cr(const struct dram_para *para)
{
	writel(MCTL_CR_BL8 | MCTL_CR_INTERLEAVED | MCTL_CR_DDR3 | MCTL_CR_2T |
	       (para->ranks[0].bank_bits == 3 ? MCTL_CR_EIGHT_BANKS : MCTL_CR_FOUR_BANKS) |
	       MCTL_CR_BUS_FULL_WIDTH(para->bus_full_width) |
	       (para->dual_rank ? MCTL_CR_DUAL_RANK : MCTL_CR_SINGLE_RANK) |
	       MCTL_CR_PAGE_SIZE(para->ranks[0].page_size) |
	       MCTL_CR_ROW_BITS(para->ranks[0].row_bits), COM_CR);
}

/* PLL5 = 24 MHz * N * K / M, DRAM_CLK * 2 (clock_sun6i.c clock_set_pll5) */
static void set_pll5(uint32_t clk)
{
	const int max_n = 32;
	int k = 1, m = 2;
	clrsetbits(CCU_PLL5_TUNING, (0x7u << 24) | (0x7fu << 16), (2u << 24) | (16u << 16));
	if (clk > 24000000u * k * max_n / m) {
		m = 1;
		if (clk > 24000000u * k * max_n / m)
			k = 2;
	}
	uint32_t n = clk / (24000000u * k / m);
	writel((1u << 31) | (1u << 20) | ((n - 1) << 8) | ((k - 1) << 4) | (m - 1), CCU_PLL5_CFG);
	udelay(5500);
}

static bool sys_init(void)
{
	clrbits(CCU_MBUS0_CLK, 1u << 31);
	clrbits(CCU_MBUS_RESET, 1u << 31);
	clrbits(CCU_AHB_GATE0, AHB_MCTL);
	clrbits(CCU_AHB_RESET0, AHB_MCTL);
	clrbits(CCU_PLL5_CFG, 1u << 31);
	udelay(10);
	clrbits(CCU_DRAM_CLK, 1u << 31);
	udelay(1000);

	set_pll5(DRAM_CLK * 2 * 1000000u);
	clrsetbits(CCU_DRAM_CLK, 0xfu | (0x3u << 20), 0u | (0u << 20) | (1u << 16));   /* DIV 1, PLL5, UPD */
	if (!await(CCU_DRAM_CLK, 1u << 16, 0)) return false;

	setbits(CCU_AHB_RESET0, AHB_MCTL);
	setbits(CCU_AHB_GATE0, AHB_MCTL);
	setbits(CCU_MBUS_RESET, 1u << 31);
	setbits(CCU_MBUS0_CLK, 1u << 31);
	setbits(CCU_DRAM_CLK, 1u << 31);
	udelay(10);
	writel(0xc00e, CTL_CLKEN);
	udelay(500);
	return true;
}

/* ddr3_1333.c mctl_set_timing_params, H3 */
static inline uint32_t ns_to_t(uint32_t ns) { return (DRAM_CLK / 2 * ns + 999) / 1000; }
#define MAX(a, b) ((a) > (b) ? (a) : (b))
static void set_timing_params(void)
{
	uint8_t tccd = 2, tfaw = ns_to_t(50), trrd = MAX(ns_to_t(10), 4), trcd = ns_to_t(15), trc = ns_to_t(53);
	uint8_t txp = MAX(ns_to_t(8), 3), twtr = MAX(ns_to_t(8), 4), trtp = MAX(ns_to_t(8), 4), twr = MAX(ns_to_t(15), 3);
	uint8_t trp = ns_to_t(15), tras = ns_to_t(38);
	uint16_t trefi = ns_to_t(7800) / 32, trfc = ns_to_t(350);
	uint8_t tmrw = 0, tmrd = 4, tmod = 12, tcke = 3, tcksrx = 5, tcksre = 5, tckesr = 4, trasmax = 24;
	uint8_t tcl = 6, tcwl = 4, t_rdata_en = 4, wr_latency = 2;
	uint32_t tdinit0 = 500 * DRAM_CLK + 1, tdinit1 = 360 * DRAM_CLK / 1000 + 1;
	uint32_t tdinit2 = 200 * DRAM_CLK + 1, tdinit3 = 1 * DRAM_CLK + 1;
	uint8_t twtp = tcwl + 2 + twr, twr2rd = tcwl + 2 + twtr, trd2wr = tcl + 2 + 1 - tcwl;

	writel(0x1c70, CTL_MR(0));                  /* CL=11, WR=12 */
	writel(0x40, CTL_MR(1));
	writel(0x18, CTL_MR(2));                    /* CWL=8 */
	writel(0x0, CTL_MR(3));

	writel((uint32_t)twtp << 24 | (uint32_t)tfaw << 16 | (uint32_t)trasmax << 8 | tras, CTL_DRAMTMG(0));
	writel((uint32_t)txp << 16 | (uint32_t)trtp << 8 | trc, CTL_DRAMTMG(1));
	writel((uint32_t)tcwl << 24 | (uint32_t)tcl << 16 | (uint32_t)trd2wr << 8 | twr2rd, CTL_DRAMTMG(2));
	writel((uint32_t)tmrw << 16 | (uint32_t)tmrd << 12 | tmod, CTL_DRAMTMG(3));
	writel((uint32_t)trcd << 24 | (uint32_t)tccd << 16 | (uint32_t)trrd << 8 | trp, CTL_DRAMTMG(4));
	writel((uint32_t)tcksrx << 24 | (uint32_t)tcksre << 16 | (uint32_t)tckesr << 8 | tcke, CTL_DRAMTMG(5));
	clrsetbits(CTL_DRAMTMG(8), (0xffu << 8) | 0xffu, (0x66u << 8) | 0x10u);      /* two-rank timing */
	writel((0x2u << 24) | ((uint32_t)t_rdata_en << 16) | (0x1u << 8) | wr_latency, CTL_PITMG0);
	writel(tdinit1 << 20 | tdinit0, CTL_PTR(3));
	writel(tdinit3 << 20 | tdinit2, CTL_PTR(4));
	writel((uint32_t)trefi << 16 | trfc, CTL_RFSHTMG);
}

#define DX_GCR_ODT_DYNAMIC (0x0u << 4)
#define DX_GCR_ODT_OFF     (0x2u << 4)

static int channel_init(struct dram_para *para)
{
	set_cr(para);
	set_timing_params();
	set_master_priority_h3();

	clrbits(CTL_PGCR(0), (1u << 30) | 0x3f);                /* VTC: disable all VT */
	clrsetbits(CTL_PGCR(1), 1u << 24, 1u << 26);

	writel(PROTECT_MAGIC, COM_PROTECT);                     /* DFI_PHY_UPD clock */
	udelay(100);
	clrsetbits(CTL_UPD2, 0xfffu << 16, 0x50u << 16);
	writel(0, COM_PROTECT);
	udelay(100);

	for (int i = 0; i < 4; i++)                             /* dramc odt */
		clrsetbits(CTL_DX_GCR(i), (0x3u << 4) | (0x1u << 1) | (0x3u << 2) | (0x3u << 12) | (0x3u << 14),
			   DRAM_ODT_EN ? DX_GCR_ODT_DYNAMIC : DX_GCR_ODT_OFF);
	clrsetbits(CTL_ACIOCR, 0, 0x1u << 1);                   /* AC PDR always on */
	setbits(CTL_PGCR(2), 0x3u << 6);                        /* DQS auto gating PD mode */
	clrbits(CTL_PGCR(0), (0x3u << 14) | (0x3u << 12));      /* dx ddr_clk & hdr_clk dynamic */
	clrsetbits(CTL_PGCR(2), (0x3u << 10) | (0x3u << 8), (0x1u << 10) | (0x2u << 8));  /* phase 270 */

	if (!para->bus_full_width) {
		writel(0, CTL_DX_GCR(2));
		writel(0, CTL_DX_GCR(3));
	}
	clrsetbits(CTL_DTCR, 0xfu << 24, (para->dual_rank ? 0x3u : 0x1u) << 24);  /* data training */
	set_bit_delays(para);
	udelay(50);

	if (!zq_calibration_quirk()) return -1;
	if (!phy_init(PIR_PLLINIT | PIR_DCAL | PIR_PHYRST | PIR_DRAMRST | PIR_DRAMINIT | PIR_QSGATE)) return -2;

	/* detect ranks and bus width */
	if (readl(CTL_PGSR0) & (0xfeu << 20)) {
		if (((readl(CTL_DX_GSR0(0)) >> 24) & 0x2) || ((readl(CTL_DX_GSR0(1)) >> 24) & 0x2)) {
			clrsetbits(CTL_DTCR, 0xfu << 24, 0x1u << 24);
			para->dual_rank = 0;
		}
		if (((readl(CTL_DX_GSR0(2)) >> 24) & 0x1) || ((readl(CTL_DX_GSR0(3)) >> 24) & 0x1)) {
			writel(0, CTL_DX_GCR(2));
			writel(0, CTL_DX_GCR(3));
			para->bus_full_width = 0;
		}
		set_cr(para);
		udelay(20);
		if (!phy_init(PIR_QSGATE)) return -3;               /* re-train */
		if (readl(CTL_PGSR0) & (0xfeu << 20)) return -4;
	}

	if (!await(CTL_STATR, 0x1, 0x1)) return -5;
	setbits(CTL_RFSHCTL0, 1u << 31);                        /* refresh debug */
	udelay(10);
	clrbits(CTL_RFSHCTL0, 1u << 31);
	udelay(10);
	writel(0x00aa0060u, CTL_PGCR(3));                       /* CKE polarity */
	setbits(CTL_ZQCR, ZQCR_PWRDOWN);                        /* ZQ module off */
	writel(0xffffffffu, COM_MAER);                          /* master access */
	return 0;
}

/* Size detection (patch 0001): 16 address-derived words at the base must all
 * read back at base + offset; the overwritten words are restored. */
static void write_pattern(uintptr_t base)
{
	uint32_t *p = (uint32_t *)base;
	for (unsigned i = 0; i < 16; p++, i++)
		writel((i & 1) ? ~(uint32_t)(uintptr_t)p : (uint32_t)(uintptr_t)p, (uintptr_t)p);
}

static bool check_pattern(uintptr_t base, uint32_t offset)
{
	uint32_t *p = (uint32_t *)base;
	for (unsigned i = 0; i < 16; p++, i++) {
		uint32_t val = (i & 1) ? ~(uint32_t)(uintptr_t)p : (uint32_t)(uintptr_t)p;
		if (val != readl((uintptr_t)p + offset))
			return false;
	}
	return true;
}

static void detect_prepare(struct dram_para *para, uintptr_t base, uint32_t *save)
{
	set_cr(para);
	udelay(50);
	for (int i = 0; i < 16; i++) save[i] = readl(base + 4 * i);
	write_pattern(base);
}

static void detect_restore(uintptr_t base, const uint32_t *save)
{
	for (int i = 0; i < 16; i++) writel(save[i], base + 4 * i);
}

static void auto_detect_dram_size(struct dram_para *para)
{
	const uintptr_t base = H3_DRAM;
	struct rank_para *rank = &para->ranks[0];
	uint32_t save[16];

	rank->page_size = 512; rank->row_bits = 16; rank->bank_bits = 2;
	detect_prepare(para, base, save);
	for (rank->row_bits = 11; rank->row_bits < 16; rank->row_bits++)
		if (check_pattern(base, (1u << (rank->row_bits + rank->bank_bits)) * rank->page_size))
			break;
	detect_restore(base, save);

	rank->bank_bits = 3;
	detect_prepare(para, base, save);
	for (rank->bank_bits = 2; rank->bank_bits < 3; rank->bank_bits++)
		if (check_pattern(base, (1u << rank->bank_bits) * rank->page_size))
			break;
	detect_restore(base, save);

	rank->page_size = 8192;
	detect_prepare(para, base, save);
	for (rank->page_size = 512; rank->page_size < 8192; rank->page_size *= 2)
		if (check_pattern(base, rank->page_size))
			break;
	detect_restore(base, save);
}

unsigned long dram_init(void)
{
	struct dram_para para = {
		.dual_rank = 1,
		.bus_full_width = 1,
		.ranks = { { 4096, 15, 3 }, { 4096, 15, 3 } },
		.dx_read_delays  = {{ 18, 18, 18, 18, 18, 18, 18, 18, 18,  0,  0 },
				    { 14, 14, 14, 14, 14, 14, 14, 14, 14,  0,  0 },
				    { 18, 18, 18, 18, 18, 18, 18, 18, 18,  0,  0 },
				    { 14, 14, 14, 14, 14, 14, 14, 14, 14,  0,  0 }},
		.dx_write_delays = {{  0,  0,  0,  0,  0,  0,  0,  0,  0, 10, 10 },
				    {  0,  0,  0,  0,  0,  0,  0,  0,  0, 10, 10 },
				    {  0,  0,  0,  0,  0,  0,  0,  0,  0, 10, 10 },
				    {  0,  0,  0,  0,  0,  0,  0,  0,  0,  6,  6 }},
		.ac_delays = { 0 },
	};

	if (!sys_init()) return 0;
	if (channel_init(&para)) return 0;

	writel(para.dual_rank ? 0x00000303u : 0x00000201u, CTL_ODTMAP);
	udelay(1);
	writel(0x0c000400u, CTL_ODTCFG);                        /* odt delay */
	setbits(COM_CCCR, 1u << 31);                            /* clear credit value */
	udelay(10);

	auto_detect_dram_size(&para);
	set_cr(&para);

	unsigned long size = (1ul << (para.ranks[0].row_bits + para.ranks[0].bank_bits)) * para.ranks[0].page_size;
	if (para.dual_rank)
		size *= 2;
	return size;
}
