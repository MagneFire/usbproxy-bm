/* Allwinner H3 (sun8i) register map, the parts this firmware touches.
 * Sources: kernel 6.6.85 ccu-sun8i-h3.c, phy-sun4i-usb.c, sunxi-h3-h5.dtsi;
 * H3 datasheet v1.2. */
#ifndef H3_H
#define H3_H
#include <stdint.h>
#include <stdbool.h>

#define REG32(a)        (*(volatile uint32_t *)(uintptr_t)(a))
static inline uint32_t readl(uintptr_t a) { return REG32(a); }
static inline void writel(uint32_t v, uintptr_t a) { REG32(a) = v; }
static inline void setbits(uintptr_t a, uint32_t m) { REG32(a) |= m; }
static inline void clrbits(uintptr_t a, uint32_t m) { REG32(a) &= ~m; }
#define BIT(n)          (1u << (n))

#define H3_DRAM         0x40000000u

#define H3_CCU          0x01c20000u
#define CCU_BUS_GATE0   (H3_CCU + 0x060) /* otg 23, ehci0 24, ehci1 25, ohci0 28, ohci1 29 */
#define CCU_BUS_GATE3   (H3_CCU + 0x06c) /* uart0 16 */
#define CCU_SD0_CLK     (H3_CCU + 0x088)
#define CCU_USBPHY_CFG  (H3_CCU + 0x0cc) /* phy rst 0..3, phy clk 8..11, ohci 12M 16..19 */
#define CCU_MBUS0_CLK   (H3_CCU + 0x15c)
#define CCU_BUS_RST0    (H3_CCU + 0x2c0) /* same bit numbers as GATE0 */
#define CCU_BUS_RST4    (H3_CCU + 0x2d8) /* uart0 16 */

#define H3_PIO          0x01c20800u     /* ports A..G, 0x24 per port */
#define H3_R_PIO        0x01f02c00u     /* port L */
#define PIO_CFG(base, n) ((base) + 0x00 + 4 * (n))
#define PIO_DAT(base)    ((base) + 0x10)
#define PIO_PORT(p)     (H3_PIO + 0x24 * (p))

#define H3_UART0        0x01c28000u
#define H3_WDT          0x01c20ca0u
#define WDT_CTRL        (H3_WDT + 0x10)
#define WDT_CFG         (H3_WDT + 0x14)
#define WDT_MODE        (H3_WDT + 0x18)
#define WDT_CTRL_RESTART (0xa57u << 1 | 1u)

#define H3_RTC          0x01f00000u
#define RTC_GP(n)       (H3_RTC + 0x100 + 4 * (n))
#define RTC_MAGIC_DEATH 0xB0070000u     /* boot0: GP1 = DEATH | n while firmware boot n is in flight (n consecutive deaths; 3 -> FEL) */
#define RTC_MAGIC_HNG1  0x484e4731u     /* "HNG1": hang before the GP1 clear next boot (test) */
#define RTC_MAGIC_FEL1  0xb0010fe1u     /* boot0 -> BROM FEL (p-boot / SUSE rbrom magic) */

#define H3_GICD         0x01c81000u
#define H3_GICC         0x01c82000u
#define IRQ_PPI_TIMER   30              /* non-secure physical timer */
#define IRQ_PPI_TIMER_S 29              /* secure physical timer (we run secure) */
#define IRQ_MUSB        (32 + 71)       /* 103 */
#define IRQ_EHCI1       (32 + 74)       /* 106 */

#define H3_MUSB         0x01c19000u
#define H3_USBPHY       0x01c19400u     /* ISCR +0, PHYCTL +0x10, OTGCTL +0x20 */
#define H3_EHCI1        0x01c1b000u

#define CNT_HZ          24000000u

/* start.S */
void irq_enable(void);
void irq_disable(void);
void dcache_clean_invalidate_all(void);

static inline void dsb(void) { __asm__ volatile("dsb" ::: "memory"); }
static inline void isb(void) { __asm__ volatile("isb" ::: "memory"); }
static inline void dmb(void) { __asm__ volatile("dmb" ::: "memory"); }
static inline void wfi(void) { __asm__ volatile("wfi" ::: "memory"); }

/* drivers */
void uart_init(void);
void uart_putc(char c);
void uart_puts(const char *s);
int uart_getc(void);            /* -1 if nothing pending */
void uart_flush(void);

void gic_init(void);
void gic_enable(unsigned id, void (*fn)(void));
void gic_disable(unsigned id);
void gic_set_handler(unsigned id, void (*fn)(void));
void gic_mask(unsigned id);
void gic_unmask(unsigned id);
void irq_handler(void);

void timer_init(void);
uint32_t time_ms(void);
uint32_t time_us(void);
uint32_t tick_count(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

void wdt_arm(void);             /* 8 s, system reset */
void wdt_feed(void);
void wdt_reset_now(void) __attribute__((noreturn));
int wdt_armed(void);

uint32_t rtc_gp_read(unsigned n);
void rtc_gp_write(unsigned n, uint32_t v);

void mmu_init(void);
void dcache_clean_range(const void *addr, uint32_t size);
void dcache_invalidate_range(const void *addr, uint32_t size);
void dcache_clean_invalidate_range(const void *addr, uint32_t size);

void led_init(void);
void led_green(int on);
void led_red(int on);

void ccu_uart0_enable(void);
void ccu_set_cpu_mhz(unsigned mhz);
unsigned ccu_cpu_mhz(void);
void ccu_set_bus_idle(bool idle);
#define CPU_MHZ_ACTIVE 1008
#define CPU_MHZ_IDLE   648

void exception_handler(uint32_t type, uint32_t *frame) __attribute__((noreturn));
#endif
