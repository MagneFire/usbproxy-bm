#include <string.h>
#include "fw.h"
#include "tusb.h"
#include "device/usbd_pvt.h"
#include "dcd_h3.h"
#include "ehci_h3.h"
#include "bridge.h"
#include "bridge/adb_proto.h"

enum { B_EMPTY = 0, B_FILLING, B_FULL, B_DRAINING };
enum { TAG_NONE = 0, TAG_PAY_LAST, TAG_OKAY };	/* cycle stamps, see cyc_* */

struct dir {
	/* Cached RAM, not the uncached .dma section: the musb side moves these
	 * bytes by PIO one word at a time and an uncached word load is a DRAM
	 * round trip. The EHCI side gets
	 * clean/invalidate by address (hcd_dcache_* in hcd_h3_ehci.c); each
	 * buffer is whole cache lines so the metadata below is never hit. */
	__attribute__((aligned(64))) uint8_t buf[BRIDGE_NBUF][BRIDGE_BUF];
	uint32_t len[BRIDGE_NBUF];
	uint8_t state[BRIDGE_NBUF];
	uint32_t seq[BRIDGE_NBUF];	/* fill order */
	uint8_t tag[BRIDGE_NBUF];	/* what the buffer holds, for the cycle stamps */
	bool hdr[BRIDGE_NBUF];		/* watch IN: the buffer is an adb header */
	bool zlp[BRIDGE_NBUF];		/* host OUT: a zero-length packet ended it, forward one after it */
	bool resync;			/* watch IN: drop buffers until a header (gadget reconnected) */
	uint32_t off[BRIDGE_NBUF];	/* bytes already forwarded early (cut-through) */
	uint32_t fwd;			/* of those, handed on */
	bool partial;			/* the drain in flight is a cut-through piece */
	uint32_t partials;
	uint32_t next_seq;
	int filling, draining;		/* buffer index or -1 */
	uint32_t msgs, bytes, zlps;
	uint32_t t_fill_done;		/* stamp: last fill completed */
	uint32_t hand_us, hand_n;	/* fill done -> drain submitted */
	uint32_t drain_us, drain_n;	/* drain submitted -> drain done */
	uint32_t t_drain_sub;
	uint32_t t_fill_sub;		/* stamp: fill submitted */
	uint32_t fill_us, fill_n;	/* fill submitted -> fill done */
	uint32_t verified;		/* watch IN cut-through: bytes of the filling buffer seen landed (canary gone) */
	uint32_t expect;		/* watch IN: bytes to read next (24 = adb header, else payload) */
	uint32_t pending_payload;	/* parsed from the last header; 0 = next read is a header */
	uint32_t errs;			/* watch IN: failed reads in a row */
	bool retry;			/* watch IN: read again at retry_at */
	uint32_t retry_at;
};

struct slot {
	bool gadget_up, bound;
	uint8_t g_in, g_out;		/* gadget endpoints */
	uint8_t daddr, w_in, w_out;	/* watch */
	uint16_t w_in_mps;		/* watch IN wMaxPacketSize */
	struct dir h2w;			/* host (gadget OUT) -> watch (OUT) */
	struct dir w2h;			/* watch (IN) -> host (gadget IN) */
};

static struct slot slots[2];
static uint32_t chunk = BRIDGE_BUF;	/* largest read (console `chunk`) */

void bridge_set_chunk(uint32_t bytes)
{
	if (bytes < 512) bytes = 512;
	if (bytes > BRIDGE_BUF) bytes = BRIDGE_BUF;
	chunk = bytes & ~511u;
}
uint32_t bridge_chunk(void) { return chunk; }
static uint32_t cut_floor = 2048;	/* Mac->watch: forward only in pieces this large (console `cutfloor`) */
void bridge_set_cut_floor(uint32_t b) { cut_floor = (b < 512 ? 512 : b > BRIDGE_BUF ? BRIDGE_BUF : b) & ~511u; }
uint32_t bridge_cut_floor(void) { return cut_floor; }
/* bit0: Mac->watch cut-through, bit1: watch->Mac (console `cut`). Mac->watch
 * is off: the watch's OKAY time does not follow our delivery. */
static uint32_t cut_mask = 2;
void bridge_set_cut(uint32_t m) { cut_mask = m & 3; }
/* watch->Mac cut-through: stay cut_lag packets behind the EHCI overlay's
 * byte count (console `cutlag`), a margin in case the count runs ahead of
 * the data in DRAM. `cutcanary 1` seeds every 64-byte line of a cut-through
 * read with a marker in its last word and forwards a line only once the
 * marker is gone; cut_stale[k] (k = line within the packet) counts lines
 * the count called landed that still held it. */
static uint32_t cut_lag = 1;
static bool cut_canary = false;
void bridge_set_cut_lag(uint32_t p) { cut_lag = p > 7 ? 7 : p; }
uint32_t bridge_cut_lag(void) { return cut_lag; }
void bridge_set_cut_canary(uint32_t on) { cut_canary = on != 0; }
uint32_t bridge_cut_canary(void) { return cut_canary; }
#define CANARY 0xC0DEFA11u
static uint32_t cut_stale[8], cut_stale_total, cut_stale_max_pkt;
uint32_t bridge_cut(void) { return cut_mask; }

/* WRTE->OKAY cycle stamps on the adb slot. One
 * tracker per direction of the WRTE: "push" = WRTE from the Mac, "pull" =
 * WRTE from the watch. adb keeps more than one WRTE in flight, so each
 * tracker holds a ring of outstanding messages and OKAYs retire them in
 * order. Phases: header seen -> last payload chunk in -> payload delivered
 * -> OKAY seen from the far side -> OKAY delivered. Means in us. */
#define CYC_RING 8
struct cyc_rec { uint32_t t_hdr, t_pay, t_sent, t_okay; uint32_t pay_left; bool paid, sent, okayed; };
struct cyc {
	struct cyc_rec r[CYC_RING];
	unsigned head, tail;		/* tail = oldest outstanding, head = next free */
	uint32_t sum[6], n, depth_sum, depth_n, dropped;
	uint32_t t_last_hdr;
};
static struct cyc cyc_push, cyc_pull;

static unsigned cyc_count(struct cyc *c) { return c->head - c->tail; }

static void cyc_hdr(struct cyc *c, uint32_t dl, uint32_t now)
{
	c->depth_sum += cyc_count(c); c->depth_n++;
	if (c->t_last_hdr) { c->sum[5] += now - c->t_last_hdr; }
	c->t_last_hdr = now;
	if (cyc_count(c) == CYC_RING) { c->tail++; c->dropped++; }
	struct cyc_rec *r = &c->r[c->head++ % CYC_RING];
	memset(r, 0, sizeof *r);
	r->t_hdr = now; r->pay_left = dl;
}

/* the newest record still waiting for payload */
static struct cyc_rec *cyc_paying(struct cyc *c)
{
	for (unsigned i = c->tail; i != c->head; i++) {
		struct cyc_rec *r = &c->r[i % CYC_RING];
		if (!r->paid) return r;
	}
	return 0;
}
static struct cyc_rec *cyc_oldest_unsent(struct cyc *c)
{
	for (unsigned i = c->tail; i != c->head; i++) {
		struct cyc_rec *r = &c->r[i % CYC_RING];
		if (r->paid && !r->sent) return r;
	}
	return 0;
}
static struct cyc_rec *cyc_oldest_unokayed(struct cyc *c)
{
	for (unsigned i = c->tail; i != c->head; i++) {
		struct cyc_rec *r = &c->r[i % CYC_RING];
		if (!r->okayed) return r;
	}
	return 0;
}

static void cyc_retire(struct cyc *c, uint32_t t_fwd)
{
	if (cyc_count(c) == 0) return;
	struct cyc_rec *r = &c->r[c->tail % CYC_RING];
	if (!r->okayed) return;
	c->tail++;
	if (!r->paid || !r->sent) return;
	c->sum[0] += r->t_pay - r->t_hdr;
	c->sum[1] += r->t_sent - r->t_pay;
	c->sum[2] += r->t_okay - r->t_sent;
	c->sum[3] += t_fwd - r->t_okay;
	c->sum[4] += r->t_okay - r->t_hdr;	/* RTT as seen here */
	c->n++;
}

/* A buffer just filled on direction d of slot 0 (h2w: from the Mac). */
static void cyc_fill_done(bool h2w, struct dir *d, int i, uint32_t len, uint32_t now)
{
	struct cyc *mine = h2w ? &cyc_push : &cyc_pull;	/* WRTEs arriving on this side */
	struct cyc *other = h2w ? &cyc_pull : &cyc_push;	/* OKAYs arriving on this side */
	uint32_t cmd, a0, a1, dl;
	d->tag[i] = TAG_NONE;
	if (len == ADB_HDR_LEN && adb_parse_hdr(d->buf[i], &cmd, &a0, &a1, &dl)) {
		if (cmd == A_WRTE && dl) {
			cyc_hdr(mine, dl, now);
		} else if (cmd == A_OKAY) {
			struct cyc_rec *r = cyc_oldest_unokayed(other);
			if (r) {
				if (!r->paid) { r->paid = true; r->t_pay = now; }
				if (!r->sent) { r->sent = true; r->t_sent = now; }
				r->okayed = true; r->t_okay = now;
				d->tag[i] = TAG_OKAY;
			}
		}
		return;
	}
	struct cyc_rec *r = cyc_paying(mine);
	if (r) {
		r->pay_left -= len < r->pay_left ? len : r->pay_left;
		if (!r->pay_left) { r->paid = true; r->t_pay = now; d->tag[i] = TAG_PAY_LAST; }
	}
}

static void cyc_drain_done(bool h2w, struct dir *d, int i, uint32_t now)
{
	struct cyc *mine = h2w ? &cyc_push : &cyc_pull;
	struct cyc *other = h2w ? &cyc_pull : &cyc_push;
	if (d->tag[i] == TAG_PAY_LAST) {
		struct cyc_rec *r = cyc_oldest_unsent(mine);
		if (r) { r->sent = true; r->t_sent = now; }
	} else if (d->tag[i] == TAG_OKAY) {
		cyc_retire(other, now);
	}
	d->tag[i] = TAG_NONE;
}

static int cyc_status(const char *name, struct cyc *c, char *buf, unsigned n)
{
	uint32_t k = c->n ? c->n : 1, dn = c->depth_n ? c->depth_n : 1;
	return snprintf(buf, n, "cycle_%s=n:%lu hdr>pay:%lu pay>sent:%lu sent>okay:%lu okay>fwd:%lu rtt:%lu hdr>hdr:%lu inflight_x10:%lu dropped:%lu\n",
			name, (unsigned long)c->n, (unsigned long)(c->sum[0] / k), (unsigned long)(c->sum[1] / k),
			(unsigned long)(c->sum[2] / k), (unsigned long)(c->sum[3] / k), (unsigned long)(c->sum[4] / k),
			(unsigned long)(c->sum[5] / dn), (unsigned long)(c->depth_sum * 10 / dn), (unsigned long)c->dropped);
}

/* ---- adb slot: host->device stream framer, CNXN capture/replay, halt pulse
 * (usb-proxy bridge.cpp, ported). adb writes each 24-byte header and each
 * payload as its own USB transfer, so one gadget read never spans a message
 * boundary: one decision per read. While no watch is bound the slot is an
 * idle sink: every read is framed and dropped, except the host's CNXN,
 * which is kept and replayed to the next device (only the host sends
 * CNXN; the Mac's transport does not know the watch changed). */
#define CNXN_MAX_PAYLOAD 1024
#define ADB_KICK_PULSE_MS 300
#define CNXN_REPLAY_GRACE_MS 500

struct framer {
	uint32_t payload_remaining;
	bool dropping, unsynced;
	bool cnxn_seen_since_bind, cnxn_pending;
	uint8_t building[ADB_HDR_LEN + CNXN_MAX_PAYLOAD];
	uint32_t building_len, building_left;	/* building_len: bytes stored; 0 = not building */
	bool building_toolong;
	uint8_t cnxn[ADB_HDR_LEN + CNXN_MAX_PAYLOAD];
	uint32_t cnxn_len;
	uint32_t captures, replays, dropped_msgs, resyncs;
	uint32_t bind_ms;
	bool replay_checked;			/* replay decision taken for this bind */
	bool pulsing;				/* IN halt pulse running */
	uint32_t pulse_until;
	uint32_t pulses;
};
static struct framer fr;

static void framer_building_clear(void)
{
	fr.building_len = 0; fr.building_left = 0; fr.building_toolong = false;
}

/* building[] holds a complete CNXN: record it unless its checksum fails
 * (a stream fed from mid-message; adbd would drop the replay silently). */
static void cnxn_complete(bool idle)
{
	if (fr.building_toolong) {
		log_printf("bridge: host CNXN payload over %u bytes, not recorded\n", CNXN_MAX_PAYLOAD);
		framer_building_clear();
		return;
	}
	uint32_t plen = fr.building_len - ADB_HDR_LEN;
	uint32_t want = adb_hdr_data_check(fr.building), have = adb_data_check(fr.building + ADB_HDR_LEN, plen);
	if (want != have) {
		log_printf("bridge: captured CNXN fails its checksum (%lu-byte banner); discarded\n", (unsigned long)plen);
		framer_building_clear();
		return;
	}
	memcpy(fr.cnxn, fr.building, fr.building_len);
	fr.cnxn_len = fr.building_len;
	framer_building_clear();
	fr.cnxn_pending = idle;
	fr.captures++;
	if (idle)
		log_printf("bridge: idle adb slot: host CNXN captured (%lu-byte banner), held for the next device\n",
			   (unsigned long)plen);
}

/* One gadget OUT read of the adb slot. Returns whether to forward it. */
static bool framer_feed(const uint8_t *data, uint32_t len, bool idle)
{
	uint32_t cmd, a0, a1, plen;
	if (len == 0) return !idle;
	/* Idle slot only: a header where payload was due means the host
	 * re-opened its transport mid-message (adb's rescan landed inside the
	 * halt pulse). Drop the partial message and start over at this header. */
	if (idle && fr.payload_remaining > 0 && len == ADB_HDR_LEN && adb_parse_hdr(data, &cmd, &a0, &a1, &plen)) {
		log_printf("bridge: idle adb slot: header where %lu payload bytes were due; dropping the partial message\n",
			   (unsigned long)fr.payload_remaining);
		fr.payload_remaining = 0;
		framer_building_clear();
		fr.dropping = false;
	}
	if (fr.payload_remaining > 0) {
		uint32_t take = len < fr.payload_remaining ? len : fr.payload_remaining;
		fr.payload_remaining -= take;
		if (fr.building_left > 0) {
			uint32_t room = sizeof fr.building - fr.building_len;
			if (take > room) { fr.building_toolong = true; }
			else { memcpy(fr.building + fr.building_len, data, take); fr.building_len += take; }
			fr.building_left -= take < fr.building_left ? take : fr.building_left;
			if (fr.building_left == 0 && !fr.dropping)
				cnxn_complete(idle);
		}
		bool forward = !fr.dropping;
		if (fr.payload_remaining == 0 && fr.dropping) {
			fr.dropping = false;
			log_printf("bridge: host stream back at a message boundary\n");
		}
		return forward && !idle;
	}
	/* a header is due */
	if (len != ADB_HDR_LEN || !adb_parse_hdr(data, &cmd, &a0, &a1, &plen)) {
		if (!fr.unsynced) {
			log_printf("bridge: %lu host bytes where a header was due; dropping until one arrives\n", (unsigned long)len);
			fr.resyncs++;
		}
		fr.unsynced = true;
		fr.dropped_msgs++;
		return false;
	}
	if (fr.unsynced) {
		log_printf("bridge: host stream re-synced\n");
		fr.unsynced = false;
	}
	fr.payload_remaining = plen;
	if (cmd == A_CNXN) {
		fr.cnxn_seen_since_bind = !idle;
		memcpy(fr.building, data, ADB_HDR_LEN);
		fr.building_len = ADB_HDR_LEN; fr.building_left = plen; fr.building_toolong = false;
		if (plen == 0) cnxn_complete(idle);
	}
	if (idle) fr.dropped_msgs++;
	return !idle;
}
static int oldest_full(struct dir *d)
{
	int best = -1;
	for (int i = 0; i < BRIDGE_NBUF; i++)
		if (d->state[i] == B_FULL && (best < 0 || (int32_t)(d->seq[i] - d->seq[best]) < 0))
			best = i;
	return best;
}

static int any_empty(struct dir *d)
{
	for (int i = 0; i < BRIDGE_NBUF; i++)
		if (d->state[i] == B_EMPTY) return i;
	return -1;
}

/* Cut-through: while a buffer is still filling, whole packets that have
 * already landed are forwarded ahead of the fill's completion (one drain in
 * flight at a time, so the drain is of everything landed since the last
 * one). The landed byte count of the filling buffer comes from the musb
 * driver for h2w and the EHCI overlay for w2h; d->fwd is what has been
 * handed on.
 * The remainder after the fill completes (short tail, or nothing) goes
 * through the ordinary FULL->drain path from offset off[i]. Only whole
 * 512-byte packets are forwarded early so the far side never sees a short
 * packet before the message ends. */
#define PKT 512u

static void dir_fill_armed(struct dir *d, int i)
{
	d->state[i] = B_FILLING; d->filling = i;
	d->zlp[i] = false;
	d->off[i] = 0; d->fwd = 0;
	d->t_fill_sub = time_us();
}

/* The filling buffer is complete with len bytes: hand the unforwarded rest
 * to the drain path. Returns the buffer index, or -1 if nothing is left and
 * the buffer could be released at once. */
static int dir_fill_complete(struct dir *d, uint32_t len)
{
	int i = d->filling;
	d->filling = -1;
	d->fill_us += time_us() - d->t_fill_sub; d->fill_n++;
	d->len[i] = len; d->off[i] = d->fwd; d->seq[i] = d->next_seq++;
	d->state[i] = B_FULL;
	return i;
}

/* The filling buffer is complete but carries nothing to forward. */
static void dir_fill_drop(struct dir *d)
{
	int i = d->filling;
	d->filling = -1;
	d->fill_us += time_us() - d->t_fill_sub; d->fill_n++;
	d->state[i] = B_EMPTY;
}

static void dir_drain_drop(struct dir *d)
{
	if (d->draining < 0) return;
	d->state[d->draining] = B_EMPTY;
	d->draining = -1;
	d->partial = false;
}

/* ---- host -> watch ---------------------------------------------------- */
static void h2w_fill(struct slot *s)
{
	struct dir *d = &s->h2w;
	if (!s->gadget_up || d->filling >= 0) return;
	int i = any_empty(d);
	if (i < 0) return;
	dir_fill_armed(d, i);
	if (!usbd_edpt_xfer(0, s->g_out, d->buf[i], (uint16_t)chunk, false)) {
		/* the stack refuses a halted endpoint: read again once the host
		 * clears the halt (bridge_gadget_out_cleared) */
		d->state[i] = B_EMPTY; d->filling = -1;
		log_printf("bridge: slot %d gadget OUT submit failed\n", (int)(s - slots));
	}
}

static void w_out_cb(tuh_xfer_t *x);

static bool h2w_submit(struct slot *s, int i, uint32_t off, uint32_t len, bool partial)
{
	struct dir *d = &s->h2w;
	d->draining = i; d->partial = partial;
	d->t_drain_sub = time_us();
	tuh_xfer_t x = { .daddr = s->daddr, .ep_addr = s->w_out, .buflen = (uint16_t)len, .buffer = d->buf[i] + off,
			 .complete_cb = w_out_cb, .user_data = (uintptr_t)(s - slots) };
	if (!tuh_edpt_xfer(&x)) {
		d->draining = -1; d->partial = false;
		log_printf("bridge: slot %d watch OUT submit failed\n", (int)(s - slots));
		return false;
	}
	return true;
}

static void h2w_drain(struct slot *s)
{
	struct dir *d = &s->h2w;
	if (!s->bound || d->draining >= 0) return;
	for (;;) {
		int i = oldest_full(d);
		if (i < 0) return;
		if (d->off[i] >= d->len[i] && d->len[i] != 0) {	/* everything went out early */
			if (!d->zlp[i]) {
				d->state[i] = B_EMPTY;
				continue;
			}
			d->zlp[i] = false; d->off[i] = d->len[i] = 0;	/* its zero-length packet is left */
		}
		d->hand_us += time_us() - d->t_fill_done; d->hand_n++;
		d->state[i] = B_DRAINING;
		if (!h2w_submit(s, i, d->off[i], d->len[i] - d->off[i], false))
			d->state[i] = B_FULL;
		return;
	}
}

/* Cut-through for the filling buffer: forward landed whole packets. */
static void h2w_drain_partial(struct slot *s)
{
	struct dir *d = &s->h2w;
	if (!(cut_mask & 1)) return;
	if (!s->bound || d->draining >= 0 || d->filling < 0) return;
	if (s == &slots[0] && (fr.payload_remaining == 0 || fr.dropping || fr.unsynced)) return;
	if (oldest_full(d) >= 0) return;		/* order: complete buffers first */
	bool dma;
	uint32_t landed = dcd_h3_out_landed(s->g_out, &dma) & ~(PKT - 1);
	if (dma) { if (landed < PKT) return; landed -= PKT; }	/* residual may run ahead of the data */
	/* Hand on cut_floor pieces, not single packets: per-packet OUT
	 * transfers slow the watch's OKAY. The remainder goes with the fill's
	 * completion. */
	if (landed < d->fwd + cut_floor) return;
	landed = d->fwd + ((landed - d->fwd) / cut_floor) * cut_floor;
	int i = d->filling;
	uint32_t off = d->fwd;
	d->fwd = landed;
	d->partials++;
	if (!h2w_submit(s, i, off, landed - off, true))
		d->fwd = off;
}

void bridge_gadget_out_done(int sl, uint32_t len)
{
	struct slot *s = &slots[sl & 1];
	struct dir *d = &s->h2w;
	int i = d->filling;
	if (i < 0) return;
	d->msgs++; d->bytes += len;
	if (!len) {
		/* adb's libusb layer terminates every payload that is a multiple of
		 * 512 with a zero-length packet; adbd reads exact lengths and never
		 * needs it, and forwarding it costs an EHCI round per WRTE. */
		d->zlps++;
		if (s == &slots[0]) {
			dir_fill_drop(d);
			h2w_fill(s);
			return;
		}
	}
	if (s == &slots[0] && !framer_feed(d->buf[i], len, !s->bound)) {
		/* idle sink, a dropped tail, or bytes where a header was due */
		dir_fill_drop(d);
		h2w_fill(s);
		return;
	}
	i = dir_fill_complete(d, len);
	/* fastboot: a read that ended short on a whole packet was ended by a
	 * zero-length packet; the watch may need it to end its own read */
	uint32_t mps = tud_speed_get() == TUSB_SPEED_HIGH ? 512 : 64;
	d->zlp[i] = s != &slots[0] && len && len < chunk && len % mps == 0;
	d->t_fill_done = time_us();
	if (s == &slots[0]) cyc_fill_done(true, d, i, len, d->t_fill_done);
	h2w_drain(s);
	h2w_fill(s);
}

static void w_out_cb(tuh_xfer_t *x)
{
	bridge_watch_out_done((int)x->user_data, x->actual_len, x->result);
}

void bridge_watch_out_done(int sl, uint32_t len, int result)
{
	struct slot *s = &slots[sl & 1];
	struct dir *d = &s->h2w;
	int i = d->draining;
	if (i < 0) return;
	d->draining = -1;
	bool partial = d->partial; d->partial = false;
	uint32_t now = time_us();
	d->drain_us += now - d->t_drain_sub; d->drain_n++;
	if (result != XFER_RESULT_SUCCESS)
		log_printf("bridge: slot %d watch OUT result %d (%lu)\n", sl, result, (unsigned long)len);
	if (s == &slots[0]) cyc_drain_done(true, d, i, now);
	if (!partial && d->zlp[i]) {
		/* the data is out, its zero-length packet next (oldest_full: it keeps its seq) */
		d->zlp[i] = false; d->off[i] = d->len[i] = 0;
		d->state[i] = B_FULL;
	} else if (!partial) {
		d->state[i] = B_EMPTY;
	}
	/* partial: the buffer is still FILLING (more to come) or already FULL
	 * with its remainder, which h2w_drain picks up (or frees) below. */
	h2w_drain(s);
	h2w_drain_partial(s);
	h2w_fill(s);
}

/* ---- watch -> host ---------------------------------------------------- */
static void w_in_cb(tuh_xfer_t *x);

/* A failed watch IN read is retried from bridge_poll after a pause, not at
 * once: a stalled endpoint fails every read. */
#define W2H_RETRY_MS 10
static void w2h_retry_later(struct dir *d)
{
	d->retry = true;
	d->retry_at = time_ms() + W2H_RETRY_MS;
}

/* The watch's adbd sends no zero-length packets, so a payload that is a
 * multiple of 512 bytes and shorter than the buffer would merge with the
 * next header in a plain 4 KiB read, and the Mac's 24-byte header read then
 * overruns. Read adb-aware instead: a 24-byte header, then exactly its
 * data_length. */
static void w2h_fill(struct slot *s)
{
	struct dir *d = &s->w2h;
	if (!s->bound || d->filling >= 0) return;
	int i = any_empty(d);
	if (i < 0) return;
	uint32_t want = BRIDGE_BUF;
	if (s == &slots[0])
		want = d->pending_payload ? TU_MIN(d->pending_payload, BRIDGE_BUF) : ADB_HDR_LEN;
	else
		/* fastboot: the host reads 64 bytes per response, so this slot
		 * must never hand the Mac a piece bigger than one response. One
		 * packet per read keeps the slot off the cut-through path
		 * (expect <= PKT); a short reply still completes the read at once. */
		want = s->w_in_mps;
	if (want > chunk) want = chunk;
	d->expect = want;
	dir_fill_armed(d, i);
	d->verified = 0;
	if (cut_canary && want > PKT) {
		for (uint32_t o = 60; o < want; o += 64) *(volatile uint32_t *)(d->buf[i] + o) = CANARY;
		dcache_clean_invalidate_range(d->buf[i], want);
	}
	tuh_xfer_t x = { .daddr = s->daddr, .ep_addr = s->w_in, .buflen = (uint16_t)want, .buffer = d->buf[i],
			 .complete_cb = w_in_cb, .user_data = (uintptr_t)(s - slots) };
	if (!tuh_edpt_xfer(&x)) {
		d->state[i] = B_EMPTY; d->filling = -1;
		log_printf("bridge: slot %d watch IN submit failed\n", (int)(s - slots));
		w2h_retry_later(d);
	}
}

static bool w2h_submit(struct slot *s, int i, uint32_t off, uint32_t len, bool partial)
{
	struct dir *d = &s->w2h;
	d->draining = i; d->partial = partial;
	d->t_drain_sub = time_us();
	if (!usbd_edpt_xfer(0, s->g_in, d->buf[i] + off, (uint16_t)len, false)) {
		/* the stack refuses a busy (halted) endpoint: no completion would
		 * ever free the buffer, so hand it back and let the next event retry */
		d->draining = -1; d->partial = false;
		log_printf("bridge: slot %d gadget IN submit failed\n", (int)(s - slots));
		return false;
	}
	return true;
}

static void w2h_drain(struct slot *s)
{
	struct dir *d = &s->w2h;
	if (!s->gadget_up || d->draining >= 0) return;
	for (;;) {
		int i = oldest_full(d);
		if (i < 0) return;
		if (d->off[i] >= d->len[i] && d->len[i] != 0) {	/* everything went out early */
			d->state[i] = B_EMPTY;
			continue;
		}
		if (d->resync) {
			if (!d->hdr[i]) { d->state[i] = B_EMPTY; continue; }
			d->resync = false;
		}
		d->hand_us += time_us() - d->t_fill_done; d->hand_n++;
		d->state[i] = B_DRAINING;
		if (!w2h_submit(s, i, d->off[i], d->len[i] - d->off[i], false))
			d->state[i] = B_FULL;
		return;
	}
}


static void w2h_drain_partial(struct slot *s)
{
	struct dir *d = &s->w2h;
	if (!(cut_mask & 2)) return;
	if (!s->gadget_up || !s->bound || d->draining >= 0 || d->filling < 0) return;
	if (d->expect <= PKT) return;			/* headers and single packets: no point */
	if (d->resync) return;				/* dropped anyway, see w2h_drain_abort */
	if (oldest_full(d) >= 0) return;
	uint32_t landed = ehci_h3_xfer_progress(s->daddr, s->w_in) & ~(PKT - 1);
	/* Stay cut_lag packets behind the overlay's count; the completion path
	 * delivers the rest. */
	if (landed < cut_lag * PKT) return;
	landed -= cut_lag * PKT;
	if (landed <= d->fwd) return;
	int i = d->filling;
	/* diagnostic: trust nothing the canary has not confirmed */
	while (cut_canary && d->verified < landed) {
		const volatile uint32_t *w = (const volatile uint32_t *)(d->buf[i] + d->verified + 60);
		dcache_invalidate_range((const void *)w, 4);
		if (*w == CANARY) {
			cut_stale[(d->verified / 64) & 7]++; cut_stale_total++;
			uint32_t ahead = (landed - d->verified + PKT - 1) / PKT;
			if (ahead > cut_stale_max_pkt) cut_stale_max_pkt = ahead;
			break;
		}
		d->verified += 64;
	}
	if (cut_canary) landed = d->verified & ~(PKT - 1);
	if (landed <= d->fwd) return;
	uint32_t off = d->fwd;
	d->fwd = landed;
	d->partials++;
	/* The buffers are cached and contiguous: while the musb PIO reads the
	 * tail of one buffer the core prefetches the first line(s) of the next,
	 * which the EHCI may still be filling. The lines handed on are
	 * invalidated here, after their DMA finished, so the PIO sees DRAM
	 * (64-128 stale bytes at packet starts otherwise; the completion path
	 * gets the same from the hcd's invalidate on completion). */
	dcache_invalidate_range(d->buf[i] + off, landed - off);
	if (!w2h_submit(s, i, off, landed - off, true))
		d->fwd = off;
}

static void w_in_cb(tuh_xfer_t *x)
{
	bridge_watch_in_done((int)x->user_data, x->actual_len, x->result);
}

void bridge_watch_in_done(int sl, uint32_t len, int result)
{
	struct slot *s = &slots[sl & 1];
	struct dir *d = &s->w2h;
	int i = d->filling;
	if (i < 0) return;
	if (result != XFER_RESULT_SUCCESS) {
		if (!d->errs++)
			log_printf("bridge: slot %d watch IN result %d\n", sl, result);
		/* what the failed read took of a message is lost: resume at a header */
		d->pending_payload = 0;
		if (s == &slots[0]) d->resync = true;
		d->filling = -1;
		if (d->fwd && d->draining == i) {
			/* a partial of this buffer is on its way to the Mac: let it
			 * finish, the buffer frees on its completion */
			d->state[i] = B_FULL; d->len[i] = d->fwd; d->off[i] = d->fwd;
		} else {
			d->state[i] = B_EMPTY;
		}
		w2h_retry_later(d);
		return;
	}
	if (d->errs > 1)
		log_printf("bridge: slot %d watch IN reads again after %lu failures\n", sl, (unsigned long)d->errs);
	d->errs = 0;
	bool hdr = false;
	if (s == &slots[0]) {
		if (d->pending_payload) {
			/* a chunk of the payload; short = framing lost, resync at a header */
			d->pending_payload = len < d->expect ? 0 : d->pending_payload - len;
		} else if (len == ADB_HDR_LEN) {
			uint32_t cmd, a0, a1, dl;
			if ((hdr = adb_parse_hdr(d->buf[i], &cmd, &a0, &a1, &dl)))
				d->pending_payload = dl;	/* read in BRIDGE_BUF pieces */
			else
				d->pending_payload = 0;		/* not adb framing: keep reading headers */
		} else {
			d->pending_payload = 0;
		}
	}
	if (len < d->fwd) d->fwd = len;			/* cannot happen; keep off <= len */
	i = dir_fill_complete(d, len);
	d->hdr[i] = hdr;
	d->msgs++; d->bytes += len; if (!len) d->zlps++;
	d->t_fill_done = time_us();
	if (s == &slots[0]) cyc_fill_done(false, d, i, len, d->t_fill_done);
	w2h_drain(s);
	w2h_fill(s);
}

void bridge_gadget_in_done(int sl)
{
	struct slot *s = &slots[sl & 1];
	struct dir *d = &s->w2h;
	int i = d->draining;
	if (i < 0) return;
	d->draining = -1;
	bool partial = d->partial; d->partial = false;
	uint32_t now = time_us();
	d->drain_us += now - d->t_drain_sub; d->drain_n++;
	if (s == &slots[0]) cyc_drain_done(false, d, i, now);
	if (!partial)
		d->state[i] = B_EMPTY;
	w2h_drain(s);
	w2h_drain_partial(s);
	w2h_fill(s);
}

bool bridge_adb_pulsing(void) { return fr.pulsing; }

/* After a re-enumeration the Mac's adb opens a new transport and sends its
 * own CNXN, later than the replay grace: a recorded one would reach the
 * device first. */
void bridge_forget_cnxn(void)
{
	fr.cnxn_len = 0;
	fr.cnxn_pending = false;
}

static void adb_slot_poll(struct slot *s);
/* Main loop: forward what has landed so far (both directions, adb slot). */
void bridge_poll(void)
{
	for (int k = 0; k < 2; k++) {
		struct slot *r = &slots[k];
		if (r->w2h.retry && (int32_t)(time_ms() - r->w2h.retry_at) >= 0) {
			r->w2h.retry = false;
			w2h_fill(r);
		}
	}
	struct slot *s = &slots[0];
	adb_slot_poll(s);
	if (!s->bound) return;
	h2w_drain_partial(s);
	w2h_drain_partial(s);
}

/* ---- lifecycle --------------------------------------------------------- */
static void dir_reset(struct dir *d)
{
	memset(d->state, 0, sizeof d->state);
	d->filling = d->draining = -1;
	d->partial = false; d->fwd = 0;
}

void bridge_init(void)
{
	memset(slots, 0, sizeof slots);
	for (int i = 0; i < 2; i++) { dir_reset(&slots[i].h2w); dir_reset(&slots[i].w2h); }
}

/* The gadget's OUT fill is gone with a (re)configuration. A cut-through
 * partial of that buffer may still be on its way to the watch: keep the
 * buffer until that completes (w_out_cb's h2w_drain frees it, nothing left),
 * as bridge_watch_in_done does for the other direction. */
static void h2w_fill_abort(struct dir *d)
{
	int i = d->filling;
	if (i < 0) return;
	d->filling = -1;
	if (d->draining == i && d->fwd) {
		d->len[i] = d->off[i] = d->fwd; d->seq[i] = d->next_seq++;
		d->state[i] = B_FULL;
	} else {
		d->state[i] = B_EMPTY;
	}
}

/* The gadget's IN drain is gone with a (re)configuration too. The host's
 * new transport must start at a message boundary: an adb header that had
 * not gone out is sent again whole, everything up to the next header is
 * dropped (w2h_drain), including a cut-through piece's buffer still
 * filling. The fastboot slot has no framing and drops the drain. */
static void w2h_drain_abort(struct slot *s)
{
	struct dir *d = &s->w2h;
	if (s == &slots[0]) d->resync = true;
	int i = d->draining;
	if (i < 0) return;
	d->draining = -1;
	bool partial = d->partial; d->partial = false;
	if (partial) return;			/* the buffer is still FILLING */
	if (d->resync && d->hdr[i]) d->state[i] = B_FULL;
	else d->state[i] = B_EMPTY;
}

void bridge_gadget_ready(int sl, uint8_t ep_in, uint8_t ep_out)
{
	struct slot *s = &slots[sl & 1];
	s->g_in = ep_in; s->g_out = ep_out; s->gadget_up = true;
	/* Anything the gadget had in flight is gone with the (re)configuration. */
	h2w_fill_abort(&s->h2w);
	w2h_drain_abort(s);
	if (s == &slots[0]) { fr.pulsing = false; fr.payload_remaining = 0; fr.unsynced = false; fr.dropping = false; framer_building_clear(); }
	h2w_fill(s);
	w2h_drain(s);
}

void bridge_gadget_gone(int sl)
{
	struct slot *s = &slots[sl & 1];
	s->gadget_up = false;
	if (s == &slots[0]) fr.pulsing = false;
	h2w_fill_abort(&s->h2w);
	w2h_drain_abort(s);
}

/* The host cleared a halt on IN. The halt cancelled the drain in flight
 * with no completion, and a drain refused while halted went back to FULL:
 * take the first back as a reconnect does, then retry both. */
void bridge_gadget_in_cleared(int sl)
{
	struct slot *s = &slots[sl & 1];
	if (s->w2h.draining >= 0) w2h_drain_abort(s);
	w2h_drain(s);
	w2h_drain_partial(s);
}

/* The host cleared a halt on OUT: the halt cancelled the read in flight with
 * no completion, or a read was refused while halted. Read again. */
void bridge_gadget_out_cleared(int sl)
{
	struct slot *s = &slots[sl & 1];
	h2w_fill_abort(&s->h2w);
	h2w_fill(s);
}

static void adb_halt_in(struct slot *s, bool on)
{
	if (!s->gadget_up) return;
	bool is = usbd_edpt_stalled(0, s->g_in);
	if (on && !is) usbd_edpt_stall(0, s->g_in);
	else if (!on && is) usbd_edpt_clear_stall(0, s->g_in);	/* only clear what is halted: musb CLRDATATOG */
	else return;
	log_printf("bridge: %s halt on EP%02x\n", on ? "set" : "clear", s->g_in);
}

void bridge_bind(int sl, uint8_t daddr, uint8_t ep_in, uint8_t ep_out, uint16_t in_mps)
{
	struct slot *s = &slots[sl & 1];
	s->daddr = daddr; s->w_in = ep_in; s->w_out = ep_out; s->bound = true;
	s->w_in_mps = in_mps ? in_mps : 512;
	s->w2h.pending_payload = 0;
	s->w2h.errs = 0; s->w2h.retry = false;
	if (s == &slots[0]) {
		/* whatever the host still owes of the previous message belonged to
		 * the device that is gone: drop that tail, resume at the boundary */
		if (fr.payload_remaining > 0) {
			log_printf("bridge: host mid-message (%lu bytes to go) from the previous device; dropping that tail\n",
				   (unsigned long)fr.payload_remaining);
			fr.dropping = true;
		}
		fr.cnxn_seen_since_bind = false;
		framer_building_clear();
		fr.bind_ms = time_ms();
		fr.replay_checked = false;
		fr.pulsing = false;
		adb_halt_in(s, false);		/* idempotent: a host SET_FEATURE or a pulse cut short */
	}
	uint32_t waiting = 0;
	for (int i = 0; i < BRIDGE_NBUF; i++)
		if (s->h2w.state[i] == B_FULL) waiting += s->h2w.len[i];
	log_printf("bridge: slot %d bound to device %u (%02x/%02x), %lu bytes waiting for it\n", sl, daddr,
		   ep_in, ep_out, (unsigned long)waiting);
	h2w_drain(s);
	w2h_fill(s);
}

/* Replay the host's last CNXN to the newly bound device as two messages
 * (header, payload), the way adb sends them. */
static void replay_cnxn(struct slot *s)
{
	struct dir *d = &s->h2w;
	int hi = -1, pi = -1;
	for (int i = 0; i < BRIDGE_NBUF; i++)
		if (d->state[i] == B_EMPTY) { if (hi < 0) hi = i; else if (pi < 0) pi = i; }
	uint32_t plen = fr.cnxn_len - ADB_HDR_LEN;
	if (hi < 0 || (plen && pi < 0)) return;		/* try again next poll */
	log_printf("bridge: %s; replaying the host's last CNXN (%lu-byte banner) to the device\n",
		   fr.cnxn_pending ? "host CNXN captured while idle" : "host sent no CNXN since the bind",
		   (unsigned long)plen);
	memcpy(d->buf[hi], fr.cnxn, ADB_HDR_LEN);
	d->len[hi] = ADB_HDR_LEN; d->off[hi] = 0; d->tag[hi] = TAG_NONE; d->seq[hi] = d->next_seq++; d->state[hi] = B_FULL;
	if (plen) {
		memcpy(d->buf[pi], fr.cnxn + ADB_HDR_LEN, plen);
		d->len[pi] = plen; d->off[pi] = 0; d->tag[pi] = TAG_NONE; d->seq[pi] = d->next_seq++; d->state[pi] = B_FULL;
	}
	d->t_fill_done = time_us();
	fr.cnxn_pending = false;
	fr.cnxn_seen_since_bind = true;
	fr.replays++;
	h2w_drain(s);
}

static void adb_slot_poll(struct slot *s)
{
	uint32_t now = time_ms();
	if (fr.pulsing && (int32_t)(now - fr.pulse_until) >= 0) {
		fr.pulsing = false;
		if (!s->bound) adb_halt_in(s, false);	/* back to NAK: the re-opened transport holds */
	}
	if (s->bound && s->gadget_up && !fr.replay_checked && !fr.cnxn_seen_since_bind) {
		/* a restarted host transport sends its CNXN within milliseconds of
		 * the bind: give it the grace, then replay the previous one. One the
		 * idle sink captured is replayed at once: the host is waiting on it. */
		if (fr.cnxn_pending || (int32_t)(now - (fr.bind_ms + CNXN_REPLAY_GRACE_MS)) >= 0) {
			if (fr.payload_remaining > 0 || fr.unsynced) return;	/* not at a boundary yet */
			fr.replay_checked = true;
			if (fr.cnxn_len < ADB_HDR_LEN) {
				log_printf("bridge: host sent no CNXN since the bind and none is on record; adb connects when the host re-opens\n");
				return;
			}
			replay_cnxn(s);
			if (!fr.cnxn_seen_since_bind) fr.replay_checked = false;	/* no free buffer yet */
		}
	}
}

void bridge_unbind(int sl)
{
	struct slot *s = &slots[sl & 1];
	s->bound = false;
	/* The host stack cancels the device's transfers at close; drop ours. */
	dir_drain_drop(&s->h2w);
	if (s->w2h.filling >= 0) { s->w2h.state[s->w2h.filling] = B_EMPTY; s->w2h.filling = -1; }
	/* adb: FULL buffers waiting to go to the watch belonged to it */
	if (s == &slots[0])
		for (int i = 0; i < BRIDGE_NBUF; i++)
			if (s->h2w.state[i] == B_FULL) s->h2w.state[i] = B_EMPTY;
	/* and what the gone device said that the host has not read yet must
	 * not reach the host ahead of the next device's first message */
	for (int i = 0; i < BRIDGE_NBUF; i++)
		if (s->w2h.state[i] == B_FULL) s->w2h.state[i] = B_EMPTY;
	s->w2h.pending_payload = 0;
	/* adb: pulse a halt on IN so the host's pending read fails at once and
	 * adb drops its transport (only the host sends CNXN; a stale transport
	 * would never see the next device), then back to NAK. IN only: halting
	 * OUT would reset the gadget's OUT toggle (musb CLRDATATOG) while the
	 * Mac keeps its own. fastboot: never halted, for the same toggle reason. */
	bool pulse = s == &slots[0] && s->gadget_up;
	if (pulse) {
		adb_halt_in(s, true);
		fr.pulsing = true; fr.pulse_until = time_ms() + ADB_KICK_PULSE_MS; fr.pulses++;
		dir_drain_drop(&s->w2h);
	}
	log_printf("bridge: slot %d unbound%s\n", sl, pulse ? " (halt pulse to kick adb, then idle NAK)" : " (idle NAK)");
	h2w_fill(s);	/* the idle sink keeps reading */
}

void bridge_stats_reset(void)
{
	memset(&cyc_push, 0, sizeof cyc_push);
	memset(&cyc_pull, 0, sizeof cyc_pull);
	memset(cut_stale, 0, sizeof cut_stale); cut_stale_total = 0; cut_stale_max_pkt = 0;
	for (int i = 0; i < 2; i++) {
		struct dir *ds[2] = { &slots[i].h2w, &slots[i].w2h };
		for (int k = 0; k < 2; k++) {
			struct dir *d = ds[k];
			d->msgs = d->bytes = d->zlps = d->partials = 0;
			d->hand_us = d->hand_n = d->drain_us = d->drain_n = d->fill_us = d->fill_n = 0;
		}
	}
}

bool bridge_host_pending(void)
{
	for (int i = 0; i < 2; i++)
		if (slots[i].h2w.draining >= 0 || slots[i].w2h.filling >= 0) return true;
	return false;
}

void bridge_status(char *buf, unsigned n)
{
	int k = 0;
	k += snprintf(buf + k, n - (unsigned)k,
		"framer=rem:%lu drop:%d unsync:%d cnxn:%lu pending:%d seen:%d captures:%lu replays:%lu dropped:%lu resyncs:%lu pulses:%lu halted:%d\n",
		(unsigned long)fr.payload_remaining, fr.dropping, fr.unsynced, (unsigned long)fr.cnxn_len, fr.cnxn_pending,
		fr.cnxn_seen_since_bind, (unsigned long)fr.captures, (unsigned long)fr.replays, (unsigned long)fr.dropped_msgs,
		(unsigned long)fr.resyncs, (unsigned long)fr.pulses, slots[0].gadget_up && usbd_edpt_stalled(0, slots[0].g_in));
	if (k >= (int)n) return;
	k += snprintf(buf + k, n - (unsigned)k,
		"cut_w2h=lag:%lu canary:%d stale:%lu max_ahead_pkts:%lu lines:%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu\n",
		(unsigned long)cut_lag, cut_canary, (unsigned long)cut_stale_total, (unsigned long)cut_stale_max_pkt,
		(unsigned long)cut_stale[0], (unsigned long)cut_stale[1], (unsigned long)cut_stale[2], (unsigned long)cut_stale[3],
		(unsigned long)cut_stale[4], (unsigned long)cut_stale[5], (unsigned long)cut_stale[6], (unsigned long)cut_stale[7]);
	if (k >= (int)n) return;
	k += cyc_status("push", &cyc_push, buf + k, n - (unsigned)k);
	if (k >= (int)n) return;
	k += cyc_status("pull", &cyc_pull, buf + k, n - (unsigned)k);
	for (int i = 0; i < 2 && k < (int)n; i++) {
		struct slot *s = &slots[i];
		k += snprintf(buf + k, n - (unsigned)k,
			"bridge%d=up:%d bound:%d chunk:%lu cut:%lu floor:%lu h2w:%lu/%lu zlp:%lu part:%lu fill:%lu hand:%lu drain:%lu w2h:%lu/%lu zlp:%lu part:%lu fill:%lu hand:%lu drain:%lu\n", i,
			s->gadget_up, s->bound, (unsigned long)chunk, (unsigned long)cut_mask, (unsigned long)cut_floor,
			(unsigned long)s->h2w.msgs, (unsigned long)s->h2w.bytes, (unsigned long)s->h2w.zlps, (unsigned long)s->h2w.partials,
			(unsigned long)(s->h2w.fill_n ? s->h2w.fill_us / s->h2w.fill_n : 0),
			(unsigned long)(s->h2w.hand_n ? s->h2w.hand_us / s->h2w.hand_n : 0),
			(unsigned long)(s->h2w.drain_n ? s->h2w.drain_us / s->h2w.drain_n : 0),
			(unsigned long)s->w2h.msgs, (unsigned long)s->w2h.bytes, (unsigned long)s->w2h.zlps, (unsigned long)s->w2h.partials,
			(unsigned long)(s->w2h.fill_n ? s->w2h.fill_us / s->w2h.fill_n : 0),
			(unsigned long)(s->w2h.hand_n ? s->w2h.hand_us / s->w2h.hand_n : 0),
			(unsigned long)(s->w2h.drain_n ? s->w2h.drain_us / s->w2h.drain_n : 0));
	}
}
