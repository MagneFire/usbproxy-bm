#ifndef GADGET_SLOTS_H
#define GADGET_SLOTS_H
#include <stdbool.h>
#include <stdint.h>
/* The two bridge interfaces of the full gadget as a TinyUSB application
 * class driver, with per-slot counters. Packets go to the bridge. */
enum { SLOT_ADB = 0, SLOT_FASTBOOT = 1, SLOT_COUNT = 2 };
struct slot_stats { uint32_t out_pkts, out_bytes, in_pkts, in_bytes, out_zlp; bool open; };
const struct slot_stats *slots_stats(int slot);
#endif
