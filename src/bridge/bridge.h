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
void bridge_gadget_out_progress(int slot, uint32_t bytes);           /* ISR: bytes landed so far in the current read */
void bridge_gadget_in_done(int slot);                                /* gadget -> host write finished */
/* watch side (host_mgr.c) */
void bridge_bind(int slot, uint8_t daddr, uint8_t ep_in, uint8_t ep_out, uint16_t in_mps);
void bridge_unbind(int slot);
void bridge_watch_out_done(int slot, uint32_t len, int result);
void bridge_watch_in_done(int slot, uint32_t len, int result);
void bridge_status(char *buf, unsigned n);
bool bridge_host_pending(void);
void bridge_poll(void);
void bridge_stats_reset(void);
bool bridge_adb_pulsing(void);		/* adb IN halt pulse in progress */				/* main loop: cut-through forwarding */		/* an EHCI transfer is in flight: poll, do not sleep */
/* Largest single transfer the datapath reads from either side (console
 * `chunk`): smaller chunks overlap the fill of one with the drain of the
 * previous at the price of more completions per message. */
void bridge_set_chunk(uint32_t bytes);
uint32_t bridge_chunk(void);
void bridge_set_cut(uint32_t mask);
uint32_t bridge_cut(void);
void bridge_set_cut_floor(uint32_t bytes);
uint32_t bridge_cut_floor(void);
void bridge_set_cut_lag(uint32_t packets);
uint32_t bridge_cut_lag(void);
void bridge_set_cut_canary(uint32_t on);
uint32_t bridge_cut_canary(void);
#endif
