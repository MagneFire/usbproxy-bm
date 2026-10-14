#ifndef HOST_MGR_H
#define HOST_MGR_H
#include <stdbool.h>
#include <stdint.h>
/* The watch side: EHCI1 enumeration, the two bridge interfaces bound to
 * host slots (adb = ff/42/01, fastboot = ff/42/03), the port policy (stay on
 * EHCI, retry a reset that did not reach high speed), and a test transfer
 * for the console. */
void host_mgr_poll(void);
uint8_t host_daddr(void);
bool host_bridged(void);	/* the device has an adb or fastboot interface: a watch */
int host_test_xfer(int slot, bool in, uint32_t len);  /* 0 = submitted */
int host_test_cnxn(int modern);
void host_status_line(char *buf, unsigned n);
void host_cycle_start(unsigned n);
void host_cycle_status(char *buf, unsigned n);
#endif
