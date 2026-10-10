#include <string.h>
#include "fw.h"
#include "tusb.h"
#include "device/usbd_pvt.h"
#include "bridge.h"
#include "bridge/adb_proto.h"

enum { B_EMPTY = 0, B_FILLING, B_FULL, B_DRAINING };

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
static uint32_t chunk = BRIDGE_BUF;

void bridge_set_chunk(uint32_t bytes)
{
	if (bytes < 512) bytes = 512;
	if (bytes > BRIDGE_BUF) bytes = BRIDGE_BUF;
	chunk = bytes & ~511u;
}
uint32_t bridge_chunk(void) { return chunk; }

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
	d->drain_us += time_us() - d->t_drain_sub; d->drain_n++;
	if (result != XFER_RESULT_SUCCESS)
		log_printf("bridge: slot %d watch OUT result %d (%lu/%lu)\n", sl, result, (unsigned long)len, (unsigned long)d->len[i]);
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
	d->drain_us += time_us() - d->t_drain_sub; d->drain_n++;
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

void bridge_status(char *buf, unsigned n)
{
	int k = 0;
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
