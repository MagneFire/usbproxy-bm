/* RTC general-purpose registers: survive a warm reset, read 0 after a cold
 * start. Contract shared with the SPL (U-Boot patch 0003) and appliance.py:
 *   GP0  request/fault word: DFU1 (SPL -> U-Boot proper), HNG1 (test),
 *        0xFAtxxxxx = fault record (t = type, xxxxx = pc offset in image)
 *   GP1  FLCN while a Falcon boot is in flight; we clear it once up
 *   GP2  SPL: load stamp ms. Firmware overwrites: 0xB1xxxxxx = entry ms
 *   GP3  SPL: jump stamp ms (Falcon) or 0xB007xxxx (U-Boot proper + why) */
#include "h3.h"

uint32_t rtc_gp_read(unsigned n) { return readl(RTC_GP(n & 3)); }
void rtc_gp_write(unsigned n, uint32_t v) { writel(v, RTC_GP(n & 3)); }
