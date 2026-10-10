#ifndef BRIDGE_H
#define BRIDGE_H
#include <stdbool.h>
#include <stdint.h>
/* The datapath: one slot = one bulk pair on the gadget (adb or fastboot
 * interface) bridged to the same pair on the watch. Two 4 KiB buffers per
 * direction: one being filled by one side while the other drains into the
 * other side. Transfers are whole adb messages at most 4 KiB (the legacy
 * transport's maxdata), ended by a short packet or by filling the buffer.
 * Packets that arrive while no watch is bound wait in the buffers. */
#define BRIDGE_BUF 4096
#define BRIDGE_NBUF 4

void bridge_init(void);
/* gadget side (gadget_slots.c) */
void bridge_gadget_ready(int slot, uint8_t ep_in, uint8_t ep_out);   /* host configured: start reading */
void bridge_gadget_gone(int slot);
void bridge_gadget_out_done(int slot, uint32_t len);                 /* host -> gadget buffer filled */
void bridge_gadget_in_done(int slot);                                /* gadget -> host write finished */
/* watch side (host_mgr.c) */
void bridge_bind(int slot, uint8_t daddr, uint8_t ep_in, uint8_t ep_out);
void bridge_unbind(int slot);
void bridge_watch_out_done(int slot, uint32_t len, int result);
void bridge_watch_in_done(int slot, uint32_t len, int result);
void bridge_status(char *buf, unsigned n);
/* Largest single transfer the datapath reads from either side (console
 * `chunk`): smaller chunks overlap the fill of one with the drain of the
 * previous at the price of more completions per message. */
void bridge_set_chunk(uint32_t bytes);
uint32_t bridge_chunk(void);
#endif
