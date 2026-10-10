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
static uint32_t chunk = 2048;	/* measured best (BARE-METAL.md 8b, M3 optimisation): 4096 store-and-forwards, 1024/512 pay per-transfer gaps */

void bridge_set_chunk(uint32_t bytes)
{
	if (bytes < 512) bytes = 512;
	if (bytes > BRIDGE_BUF) bytes = BRIDGE_BUF;
	chunk = bytes & ~511u;
}
uint32_t bridge_chunk(void) { return chunk; }

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

/* ---- host -> watch ---------------------------------------------------- */
static void h2w_fill(struct slot *s)
{
	struct dir *d = &s->h2w;
	if (!s->gadget_up || d->filling >= 0) return;
	int i = any_empty(d);
	if (i < 0) return;
	d->state[i] = B_FILLING; d->filling = i;
	d->t_fill_sub = time_us();
	usbd_edpt_xfer(0, s->g_out, d->buf[i], (uint16_t)chunk, false);
}

static void w_out_cb(tuh_xfer_t *x);

static void h2w_drain(struct slot *s)
{
	struct dir *d = &s->h2w;
	if (!s->bound || d->draining >= 0) return;
	int i = oldest_full(d);
	if (i < 0) return;
	d->state[i] = B_DRAINING; d->draining = i;
	d->t_drain_sub = time_us();
	d->hand_us += d->t_drain_sub - d->t_fill_done; d->hand_n++;
	tuh_xfer_t x = { .daddr = s->daddr, .ep_addr = s->w_out, .buflen = d->len[i], .buffer = d->buf[i],
			 .complete_cb = w_out_cb, .user_data = (uintptr_t)(s - slots) };
	if (!tuh_edpt_xfer(&x)) {
		d->state[i] = B_FULL; d->draining = -1;
		log_printf("bridge: slot %d watch OUT submit failed\n", (int)(s - slots));
	}
}

void bridge_gadget_out_done(int sl, uint32_t len)
{
	struct slot *s = &slots[sl & 1];
	struct dir *d = &s->h2w;
	int i = d->filling;
	if (i < 0) return;
	d->filling = -1;
	d->fill_us += time_us() - d->t_fill_sub; d->fill_n++;
	d->msgs++; d->bytes += len;
	if (!len) {
		/* adb's libusb layer terminates every payload that is a multiple of
		 * 512 with a zero-length packet; adbd reads exact lengths and never
		 * needs it, and forwarding it costs an EHCI round per WRTE. */
		d->zlps++;
		if (s == &slots[0]) {
			d->state[i] = B_EMPTY;
			h2w_fill(s);
			return;
		}
	}
	d->len[i] = len; d->state[i] = B_FULL; d->seq[i] = d->next_seq++;
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
	uint32_t now = time_us();
	d->drain_us += now - d->t_drain_sub; d->drain_n++;
	if (result != XFER_RESULT_SUCCESS)
		log_printf("bridge: slot %d watch OUT result %d (%lu/%lu)\n", sl, result, (unsigned long)len, (unsigned long)d->len[i]);
	if (s == &slots[0]) cyc_drain_done(true, d, i, now);
	d->state[i] = B_EMPTY;
	h2w_drain(s);
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
	d->state[i] = B_FILLING; d->filling = i;
	d->t_fill_sub = time_us();
	tuh_xfer_t x = { .daddr = s->daddr, .ep_addr = s->w_in, .buflen = (uint16_t)want, .buffer = d->buf[i],
			 .complete_cb = w_in_cb, .user_data = (uintptr_t)(s - slots) };
	if (!tuh_edpt_xfer(&x)) {
		d->state[i] = B_EMPTY; d->filling = -1;
		log_printf("bridge: slot %d watch IN submit failed\n", (int)(s - slots));
	}
}

static void w2h_drain(struct slot *s)
{
	struct dir *d = &s->w2h;
	if (!s->gadget_up || d->draining >= 0) return;
	int i = oldest_full(d);
	if (i < 0) return;
	d->state[i] = B_DRAINING; d->draining = i;
	d->t_drain_sub = time_us();
	d->hand_us += d->t_drain_sub - d->t_fill_done; d->hand_n++;
	usbd_edpt_xfer(0, s->g_in, d->buf[i], (uint16_t)d->len[i], false);
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
	d->filling = -1;
	d->fill_us += time_us() - d->t_fill_sub; d->fill_n++;
	if (result != XFER_RESULT_SUCCESS) {
		log_printf("bridge: slot %d watch IN result %d\n", sl, result);
		d->state[i] = B_EMPTY;
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
	d->len[i] = len; d->state[i] = B_FULL; d->seq[i] = d->next_seq++;
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
	uint32_t now = time_us();
	d->drain_us += now - d->t_drain_sub; d->drain_n++;
	if (s == &slots[0]) cyc_drain_done(false, d, i, now);
	d->state[i] = B_EMPTY;
	w2h_drain(s);
	w2h_fill(s);
}

/* ---- lifecycle --------------------------------------------------------- */
static void dir_reset(struct dir *d)
{
	memset(d->state, 0, sizeof d->state);
	d->filling = d->draining = -1;
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
	if (s->w2h.draining >= 0) { s->w2h.state[s->w2h.draining] = B_EMPTY; s->w2h.draining = -1; }
	h2w_fill(s);
	w2h_drain(s);
}

void bridge_gadget_gone(int sl)
{
	struct slot *s = &slots[sl & 1];
	s->gadget_up = false;
	if (s->h2w.filling >= 0) { s->h2w.state[s->h2w.filling] = B_EMPTY; s->h2w.filling = -1; }
	if (s->w2h.draining >= 0) { s->w2h.state[s->w2h.draining] = B_EMPTY; s->w2h.draining = -1; }
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
	if (s->h2w.draining >= 0) { s->h2w.state[s->h2w.draining] = B_EMPTY; s->h2w.draining = -1; }
	if (s->w2h.filling >= 0) { s->w2h.state[s->w2h.filling] = B_EMPTY; s->w2h.filling = -1; }
	log_printf("bridge: slot %d unbound\n", sl);
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
			"bridge%d=up:%d bound:%d chunk:%lu h2w:%lu/%lu zlp:%lu fill:%lu hand:%lu drain:%lu w2h:%lu/%lu zlp:%lu fill:%lu hand:%lu drain:%lu\n", i,
			s->gadget_up, s->bound, (unsigned long)chunk,
			(unsigned long)s->h2w.msgs, (unsigned long)s->h2w.bytes, (unsigned long)s->h2w.zlps,
			(unsigned long)(s->h2w.fill_n ? s->h2w.fill_us / s->h2w.fill_n : 0),
			(unsigned long)(s->h2w.hand_n ? s->h2w.hand_us / s->h2w.hand_n : 0),
			(unsigned long)(s->h2w.drain_n ? s->h2w.drain_us / s->h2w.drain_n : 0),
			(unsigned long)s->w2h.msgs, (unsigned long)s->w2h.bytes, (unsigned long)s->w2h.zlps,
			(unsigned long)(s->w2h.fill_n ? s->w2h.fill_us / s->w2h.fill_n : 0),
			(unsigned long)(s->w2h.hand_n ? s->w2h.hand_us / s->w2h.hand_n : 0),
			(unsigned long)(s->w2h.drain_n ? s->w2h.drain_us / s->w2h.drain_n : 0));
	}
}
