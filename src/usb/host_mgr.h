#ifndef HOST_MGR_H
#define HOST_MGR_H
#include <stdbool.h>
#include <stdint.h>
/* The watch side: EHCI1 enumeration, the two bridge interfaces bound to
 * host slots (adb = ff/42/01, fastboot = ff/42/03), the port policy (stay on
 * EHCI, retry a reset that did not reach high speed), and a test transfer
 * for the console. */
struct hslot { uint8_t daddr, ep_in, ep_out; bool bound; uint32_t in_pkts, out_pkts, in_bytes, out_bytes; };
void host_mgr_poll(void);
const struct hslot *host_slot(int slot);
const char *host_link_speed(void);        /* "480", "12", "1.5", "none" */
uint8_t host_daddr(void);
uint32_t host_portsc(void);
int host_test_xfer(int slot, bool in, uint32_t len);  /* 0 = submitted */
int host_test_cnxn(int modern);
void host_status_line(char *buf, unsigned n);
#endif
