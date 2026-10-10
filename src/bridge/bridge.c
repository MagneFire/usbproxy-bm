#include <string.h>
#include "fw.h"
#include "tusb.h"
#include "device/usbd_pvt.h"
#include "bridge.h"
#include "bridge/adb_proto.h"

enum { B_EMPTY = 0, B_FILLING, B_FULL, B_DRAINING };
enum { TAG_NONE = 0, TAG_PAY_LAST, TAG_OKAY };	/* cycle stamps, see cyc_* */
#define A_WRTE 0x45545257u
#define A_OKAY 0x59414b4fu

struct dir {
	/* Cached RAM, not the uncached .dma section: the musb side moves these
	 * bytes by PIO one word at a time and an uncached word load is a DRAM
	 * round trip (~21 us per 512 B packet measured). The EHCI side gets
	 * clean/invalidate by address (hcd_dcache_* in hcd_h3_ehci.c); each
	 * buffer is whole cache lines so the metadata below is never hit. */
	__attribute__((aligned(64))) uint8_t buf[BRIDGE_NBUF][BRIDGE_BUF];
	uint32_t len[BRIDGE_NBUF];
	uint8_t state[BRIDGE_NBUF];
	uint32_t seq[BRIDGE_NBUF];	/* fill order */
	uint8_t tag[BRIDGE_NBUF];	/* what the buffer holds, for the cycle stamps */
	uint32_t off[BRIDGE_NBUF];	/* bytes already forwarded early (cut-through) */
	volatile uint32_t progress;	/* landed bytes of the filling buffer */
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
	uint32_t expect;		/* watch IN: bytes to read next (24 = adb header, else payload) */
	uint32_t pending_payload;	/* parsed from the last header; 0 = next read is a header */
};

struct slot {
	bool gadget_up, bound;
	uint8_t g_in, g_out;		/* gadget endpoints */
	uint8_t daddr, w_in, w_out;	/* watch */
	struct dir h2w;			/* host (gadget OUT) -> watch (OUT) */
	struct dir w2h;			/* watch (IN) -> host (gadget IN) */
};

static struct slot slots[2];
static uint32_t chunk = BRIDGE_BUF;	/* with cut-through a whole message per read; 2048 was best before it */

void bridge_set_chunk(uint32_t bytes)
{
	if (bytes < 512) bytes = 512;
	if (bytes > BRIDGE_BUF) bytes = BRIDGE_BUF;
	chunk = bytes & ~511u;
}
uint32_t bridge_chunk(void) { return chunk; }
static uint32_t cut_mask = 2;	/* watch->Mac on; Mac->watch off: per-packet OUT transfers made the watch respond ~200 us later */		/* bit0: Mac->watch cut-through, bit1: watch->Mac (console `cut`) */
void bridge_set_cut(uint32_t m) { cut_mask = m & 3; }
uint32_t bridge_cut(void) { return cut_mask; }

/* WRTE->OKAY cycle stamps on the adb slot (BARE-METAL.md M3 gate). One
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
 * one). d->progress is the landed byte count of the filling buffer (musb
 * ISR for h2w, EHCI overlay for w2h); d->fwd what has been handed on.
 * The remainder after the fill completes (short tail, or nothing) goes
 * through the ordinary FULL->drain path from offset off[i]. Only whole
 * 512-byte packets are forwarded early so the far side never sees a short
 * packet before the message ends. */
#define PKT 512u

static void dir_fill_armed(struct dir *d, int i)
{
	d->state[i] = B_FILLING; d->filling = i;
	d->off[i] = 0; d->fwd = 0; d->progress = 0;
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

/* ---- host -> watch ---------------------------------------------------- */
static void h2w_fill(struct slot *s)
{
	struct dir *d = &s->h2w;
	if (!s->gadget_up || d->filling >= 0) return;
	int i = any_empty(d);
	if (i < 0) return;
	dir_fill_armed(d, i);
	usbd_edpt_xfer(0, s->g_out, d->buf[i], (uint16_t)chunk, false);
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
		if (d->off[i] >= d->len[i]) {		/* everything went out early */
			d->state[i] = B_EMPTY;
			continue;
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
	if (oldest_full(d) >= 0) return;		/* order: complete buffers first */
	uint32_t landed = d->progress & ~(PKT - 1);
	if (landed <= d->fwd) return;
	int i = d->filling;
	uint32_t off = d->fwd;
	d->fwd = landed;
	d->partials++;
	if (!h2w_submit(s, i, off, landed - off, true))
		d->fwd = off;
}

void bridge_gadget_out_progress(int sl, uint32_t bytes)
{
	struct dir *d = &slots[sl & 1].h2w;
	if (d->filling >= 0 && bytes > d->progress) d->progress = bytes;
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
			d->filling = -1;
			d->fill_us += time_us() - d->t_fill_sub; d->fill_n++;
			d->state[i] = B_EMPTY;
			h2w_fill(s);
			return;
		}
	}
	i = dir_fill_complete(d, len);
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
	if (!partial)
		d->state[i] = B_EMPTY;
	/* partial: the buffer is still FILLING (more to come) or already FULL
	 * with its remainder, which h2w_drain picks up (or frees) below. */
	h2w_drain(s);
	h2w_drain_partial(s);
	h2w_fill(s);
}

/* ---- watch -> host ---------------------------------------------------- */
static void w_in_cb(tuh_xfer_t *x);

/* The watch's adbd sends no zero-length packets, so a payload that is a
 * multiple of 512 bytes and shorter than the buffer would merge with the
 * next header in a plain 4 KiB read, and the Mac's 24-byte header read then
 * overruns ("connection terminated: read failed"). Read adb-aware instead:
 * a 24-byte header, then exactly its data_length (the appliance's sized
 * bulk-IN reads). The fastboot slot keeps whole-buffer reads (its
 * device->host messages are short text). */
static void w2h_fill(struct slot *s)
{
	struct dir *d = &s->w2h;
	if (!s->bound || d->filling >= 0) return;
	int i = any_empty(d);
	if (i < 0) return;
	uint32_t want = BRIDGE_BUF;
	if (s == &slots[0])
		want = d->pending_payload ? d->pending_payload : ADB_HDR_LEN;
	if (want > chunk) want = chunk;
	d->expect = want;
	dir_fill_armed(d, i);
	tuh_xfer_t x = { .daddr = s->daddr, .ep_addr = s->w_in, .buflen = (uint16_t)want, .buffer = d->buf[i],
			 .complete_cb = w_in_cb, .user_data = (uintptr_t)(s - slots) };
	if (!tuh_edpt_xfer(&x)) {
		d->state[i] = B_EMPTY; d->filling = -1;
		log_printf("bridge: slot %d watch IN submit failed\n", (int)(s - slots));
	}
}

static void w2h_submit(struct slot *s, int i, uint32_t off, uint32_t len, bool partial)
{
	struct dir *d = &s->w2h;
	d->draining = i; d->partial = partial;
	d->t_drain_sub = time_us();
	usbd_edpt_xfer(0, s->g_in, d->buf[i] + off, (uint16_t)len, false);
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
		d->hand_us += time_us() - d->t_fill_done; d->hand_n++;
		d->state[i] = B_DRAINING;
		w2h_submit(s, i, d->off[i], d->len[i] - d->off[i], false);
		return;
	}
}

uint32_t ehci_h3_xfer_progress(uint8_t dev_addr, uint8_t ep_addr);

static void w2h_drain_partial(struct slot *s)
{
	struct dir *d = &s->w2h;
	if (!(cut_mask & 2)) return;
	if (!s->gadget_up || !s->bound || d->draining >= 0 || d->filling < 0) return;
	if (d->expect <= PKT) return;			/* headers and single packets: no point */
	if (oldest_full(d) >= 0) return;
	d->progress = ehci_h3_xfer_progress(s->daddr, s->w_in);
	uint32_t landed = d->progress & ~(PKT - 1);
	/* The HC's byte count in the overlay runs ahead of the data: the newest
	 * packet's first burst(s) were still in flight when we read them (64-128
	 * stale bytes at packet starts, md5 mismatch). Stay one packet behind;
	 * the completion path delivers the rest. */
	if (landed < PKT) return;
	landed -= PKT;
	if (landed <= d->fwd) return;
	int i = d->filling;
	uint32_t off = d->fwd;
	d->fwd = landed;
	d->partials++;
	w2h_submit(s, i, off, landed - off, true);
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
		log_printf("bridge: slot %d watch IN result %d\n", sl, result);
		d->filling = -1;
		if (d->fwd && d->draining == i) {
			/* a partial of this buffer is on its way to the Mac: let it
			 * finish, the buffer frees on its completion */
			d->state[i] = B_FULL; d->len[i] = d->fwd; d->off[i] = d->fwd;
		} else {
			d->state[i] = B_EMPTY;
		}
		if (s->bound) w2h_fill(s);
		return;
	}
	if (s == &slots[0]) {
		if (d->pending_payload) {
			/* a chunk of the payload; short = framing lost, resync at a header */
			d->pending_payload = len < d->expect ? 0 : d->pending_payload - len;
		} else if (len == ADB_HDR_LEN) {
			uint32_t cmd, a0, a1, dl;
			if (adb_parse_hdr(d->buf[i], &cmd, &a0, &a1, &dl))
				d->pending_payload = dl > BRIDGE_BUF ? BRIDGE_BUF : dl;	/* legacy transport: maxdata 4096 */
			else
				d->pending_payload = 0;		/* not adb framing: keep reading headers */
		} else {
			d->pending_payload = 0;
		}
	}
	if (len < d->fwd) d->fwd = len;			/* cannot happen; keep off <= len */
	i = dir_fill_complete(d, len);
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

/* Main loop: forward what has landed so far (both directions, adb slot). */
void bridge_poll(void)
{
	struct slot *s = &slots[0];
	if (!s->bound) return;
	h2w_drain_partial(s);
	w2h_drain_partial(s);
}

/* ---- lifecycle --------------------------------------------------------- */
static void dir_reset(struct dir *d)
{
	memset(d->state, 0, sizeof d->state);
	d->filling = d->draining = -1;
	d->partial = false; d->fwd = 0; d->progress = 0;
}

void bridge_init(void)
{
	memset(slots, 0, sizeof slots);
	for (int i = 0; i < 2; i++) { dir_reset(&slots[i].h2w); dir_reset(&slots[i].w2h); }
}

void bridge_gadget_ready(int sl, uint8_t ep_in, uint8_t ep_out)
{
	struct slot *s = &slots[sl & 1];
	s->g_in = ep_in; s->g_out = ep_out; s->gadget_up = true;
	/* Anything the gadget had in flight is gone with the (re)configuration. */
	if (s->h2w.filling >= 0) { s->h2w.state[s->h2w.filling] = B_EMPTY; s->h2w.filling = -1; }
	if (s->w2h.draining >= 0) { s->w2h.state[s->w2h.draining] = B_EMPTY; s->w2h.draining = -1; s->w2h.partial = false; }
	h2w_fill(s);
	w2h_drain(s);
}

void bridge_gadget_gone(int sl)
{
	struct slot *s = &slots[sl & 1];
	s->gadget_up = false;
	if (s->h2w.filling >= 0) { s->h2w.state[s->h2w.filling] = B_EMPTY; s->h2w.filling = -1; }
	if (s->w2h.draining >= 0) { s->w2h.state[s->w2h.draining] = B_EMPTY; s->w2h.draining = -1; s->w2h.partial = false; }
}

void bridge_bind(int sl, uint8_t daddr, uint8_t ep_in, uint8_t ep_out)
{
	struct slot *s = &slots[sl & 1];
	s->daddr = daddr; s->w_in = ep_in; s->w_out = ep_out; s->bound = true;
	s->w2h.pending_payload = 0;
	uint32_t waiting = 0;
	for (int i = 0; i < BRIDGE_NBUF; i++)
		if (s->h2w.state[i] == B_FULL) waiting += s->h2w.len[i];
	log_printf("bridge: slot %d bound to device %u (%02x/%02x), %lu bytes waiting for it\n", sl, daddr,
		   ep_in, ep_out, (unsigned long)waiting);
	h2w_drain(s);
	w2h_fill(s);
}

void bridge_unbind(int sl)
{
	struct slot *s = &slots[sl & 1];
	s->bound = false;
	/* The host stack cancels the device's transfers at close; drop ours. */
	if (s->h2w.draining >= 0) { s->h2w.state[s->h2w.draining] = B_EMPTY; s->h2w.draining = -1; s->h2w.partial = false; }
	if (s->w2h.filling >= 0) { s->w2h.state[s->w2h.filling] = B_EMPTY; s->w2h.filling = -1; }
	log_printf("bridge: slot %d unbound\n", sl);
}

void bridge_stats_reset(void)
{
	memset(&cyc_push, 0, sizeof cyc_push);
	memset(&cyc_pull, 0, sizeof cyc_pull);
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
	k += cyc_status("push", &cyc_push, buf + k, n - (unsigned)k);
	k += cyc_status("pull", &cyc_pull, buf + k, n - (unsigned)k);
	for (int i = 0; i < 2 && k < (int)n; i++) {
		struct slot *s = &slots[i];
		k += snprintf(buf + k, n - (unsigned)k,
			"bridge%d=up:%d bound:%d chunk:%lu cut:%lu h2w:%lu/%lu zlp:%lu part:%lu fill:%lu hand:%lu drain:%lu w2h:%lu/%lu zlp:%lu part:%lu fill:%lu hand:%lu drain:%lu\n", i,
			s->gadget_up, s->bound, (unsigned long)chunk, (unsigned long)cut_mask,
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
