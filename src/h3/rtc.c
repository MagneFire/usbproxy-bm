/* RTC general-purpose registers: survive a warm reset, read 0 after a cold
 * start. Contract shared with boot0 (src/boot0/main.c) and appliance.py:
 *   GP0  request/fault word: HNG1 (test: hang before the GP1 clear),
 *        0xFAtxxxxx = fault record (t = type, xxxxx = pc offset in image)
 *   GP1  0xB00700NN while a firmware boot is in flight (NN = consecutive
 *        boots that died before clearing it; boot0 enters FEL at 3), cleared
 *        here once the host has configured the gadget; FEL1 = FEL request
 *   GP2  boot0: 0xB0xxxxxx entry ms. Firmware overwrites: 0xB1xxxxxx = entry ms
 *   GP3  boot0: jump stamp ms, or 0xB007000N = why it fell back to FEL */
#include "h3.h"

uint32_t rtc_gp_read(unsigned n) { return readl(RTC_GP(n & 3)); }
void rtc_gp_write(unsigned n, uint32_t v) { writel(v, RTC_GP(n & 3)); }
