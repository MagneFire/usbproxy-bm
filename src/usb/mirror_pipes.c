/* The mirror's data endpoints. A pipe is one gadget endpoint and the device
 * endpoint behind it, with two packet buffers between them: an IN pipe
 * keeps a read posted at the device and hands each completed packet (a
 * ZLP as itself) to the gadget; an OUT pipe keeps a read posted at the
 * gadget and writes each completed packet to the device in order. A STALL
 * from the device halts the gadget endpoint until the Mac clears it, which
 * clears the device's too. Store-and-forward, one transfer in flight per
 * side; the completions come through the interrupts. */
#include <string.h>
#include "fw.h"
#include "tusb.h"
#include "device/usbd_pvt.h"
#include "mirror.h"
#include "mirror_pipes.h"

#define NBUF     2
#define PIPE_BUF 1024		/* the largest packet USB 2.0 allows (interrupt at high speed) */
#define RETRY_MS 10

struct pipe {
	uint8_t gep, dep, type;
	uint16_t mps;
	bool open;			/* the gadget endpoint is open (configured) */
	bool halted;			/* the device stalled: gadget endpoint halted until cleared */
	bool g_busy, d_busy;		/* a transfer in flight on the gadget / the device side */
	bool clear_pending;		/* the Mac cleared the halt, the device's is still to clear */
	uint8_t fill, drain, nfull;
	uint8_t gen;			/* bumped at each open: a late device completion is dropped */
	uint16_t len[NBUF];
	uint32_t retry_at;
	uint32_t pkts, bytes, zlps, stalls, fails, retries, dropped;
	uint8_t __attribute__((aligned(64))) buf[NBUF][PIPE_BUF];
};
static struct pipe pipes[2][MIRROR_EP_MAX - 1];	/* [dir][number - 1] */

static struct pipe *pipe_of(uint8_t gep)
{
	unsigned num = gep & 0xf;
	return num && num < MIRROR_EP_MAX ? &pipes[gep >> 7][num - 1] : NULL;
}

static bool is_in(const struct pipe *p) { return p->gep & 0x80; }

/* One packet per transfer. A transfer completes only on a short packet or
 * a full buffer, so a buffer larger than a packet would hold data that
 * ends on a packet boundary until more followed: a 512-byte sector the Mac
 * writes and then waits on, a 64-byte line typed at a serial port. */
static uint16_t read_len(const struct pipe *p)
{
	return p->mps < PIPE_BUF ? p->mps : PIPE_BUF;
}

static void dev_in_done(tuh_xfer_t *x);
static void dev_out_done(tuh_xfer_t *x);

/* ---- IN: device -> gadget --------------------------------------------- */
static void arm_dev_in(struct pipe *p)
{
	if (p->d_busy || p->nfull >= NBUF || !p->open || p->halted || !mirror_daddr()) return;
	tuh_xfer_t x = { .daddr = mirror_daddr(), .ep_addr = p->dep, .buflen = read_len(p), .buffer = p->buf[p->fill],
			 .complete_cb = dev_in_done, .user_data = p->gen };
	if (!tuh_edpt_xfer(&x)) { p->retry_at = time_ms() + RETRY_MS; p->retries++; return; }
	p->d_busy = true;
}

static void push_gadget_in(struct pipe *p)
{
	if (p->g_busy || !p->nfull || !p->open || p->halted) return;
	if (!usbd_edpt_xfer(0, p->gep, p->buf[p->drain], p->len[p->drain], false)) { p->retry_at = time_ms() + RETRY_MS; p->retries++; return; }
	p->g_busy = true;
}

/* the pipe whose device endpoint completed (its number may differ from
 * the gadget's) */
static struct pipe *pipe_of_dev(uint8_t dep)
{
	for (unsigned i = 0; i < MIRROR_EP_MAX - 1; i++)
		if (pipes[dep >> 7][i].dep == dep && pipes[dep >> 7][i].open) return &pipes[dep >> 7][i];
	return NULL;
}

static void dev_in_done(tuh_xfer_t *x)
{
	struct pipe *p = pipe_of_dev(x->ep_addr);
	if (!p || x->user_data != p->gen) return;
	p->d_busy = false;
	if (x->result == XFER_RESULT_SUCCESS) {
		p->len[p->fill] = (uint16_t)x->actual_len;
		p->fill = (uint8_t)((p->fill + 1) % NBUF);
		p->nfull++;
		p->pkts++;
		p->bytes += x->actual_len;
		if (!x->actual_len) p->zlps++;
		push_gadget_in(p);
		arm_dev_in(p);
	} else if (x->result == XFER_RESULT_STALLED) {
		p->stalls++;
		p->halted = true;
		usbd_edpt_stall(0, p->gep);
	} else {
		p->fails++;
		p->retry_at = time_ms() + RETRY_MS;
	}
}

/* ---- OUT: gadget -> device -------------------------------------------- */
static void arm_gadget_out(struct pipe *p)
{
	if (p->g_busy || p->nfull >= NBUF || !p->open || p->halted) return;
	if (!usbd_edpt_xfer(0, p->gep, p->buf[p->fill], read_len(p), false)) { p->retry_at = time_ms() + RETRY_MS; p->retries++; return; }
	p->g_busy = true;
}

static void push_dev_out(struct pipe *p)
{
	if (p->d_busy || !p->nfull || !p->open || p->halted || !mirror_daddr()) return;
	tuh_xfer_t x = { .daddr = mirror_daddr(), .ep_addr = p->dep, .buflen = p->len[p->drain], .buffer = p->buf[p->drain],
			 .complete_cb = dev_out_done, .user_data = p->gen };
	if (!tuh_edpt_xfer(&x)) { p->retry_at = time_ms() + RETRY_MS; p->retries++; return; }
	p->d_busy = true;
}

static void dev_out_done(tuh_xfer_t *x)
{
	struct pipe *p = pipe_of_dev(x->ep_addr);
	if (!p || x->user_data != p->gen) return;
	p->d_busy = false;
	if (x->result == XFER_RESULT_SUCCESS) {
		p->pkts++;
		p->bytes += p->len[p->drain];
		if (!p->len[p->drain]) p->zlps++;
		p->drain = (uint8_t)((p->drain + 1) % NBUF);
		p->nfull--;
		push_dev_out(p);
		arm_gadget_out(p);
	} else if (x->result == XFER_RESULT_STALLED) {
		p->stalls++;
		p->halted = true;
		/* the musb cannot stall an OUT endpoint with a packet already in its
		 * FIFO; with a read posted the data is held and dropped instead */
		if (!p->g_busy) usbd_edpt_stall(0, p->gep);
	} else {
		p->fails++;
		p->retry_at = time_ms() + RETRY_MS;
	}
}

/* ---- the gadget side ---------------------------------------------------- */
void mirror_pipes_gadget_open(uint8_t gep, uint8_t dep, uint8_t type, uint16_t mps)
{
	struct pipe *p = pipe_of(gep);
	if (!p || !dep) return;
	uint8_t gen = (uint8_t)(p->gen + 1);
	memset(p, 0, offsetof(struct pipe, buf));
	p->gep = gep; p->dep = dep; p->type = type; p->mps = mps;
	p->gen = gen;
	p->open = true;
	log_printf("mirror: pipe %02x <-> device %02x, %s %u\n", gep, dep, type == TUSB_XFER_BULK ? "bulk" : "interrupt", mps);
	if (is_in(p)) arm_dev_in(p);
	else arm_gadget_out(p);
}

void mirror_pipes_gadget_reset(void)
{
	for (unsigned d = 0; d < 2; d++)
		for (unsigned i = 0; i < MIRROR_EP_MAX - 1; i++) {
			struct pipe *p = &pipes[d][i];
			p->open = false;	/* a device completion still due is dropped by gen at the next open */
			p->g_busy = false;
			p->nfull = p->fill = p->drain = 0;
			p->halted = p->clear_pending = false;
		}
}

bool mirror_pipes_gadget_xfer(uint8_t gep, xfer_result_t result, uint32_t len)
{
	struct pipe *p = pipe_of(gep);
	if (!p || !p->open) return false;
	p->g_busy = false;
	if (is_in(p)) {
		if (result == XFER_RESULT_SUCCESS) {
			p->drain = (uint8_t)((p->drain + 1) % NBUF);
			p->nfull--;
		}
		push_gadget_in(p);
		arm_dev_in(p);
	} else {
		if (result == XFER_RESULT_SUCCESS) {
			if (p->halted) { p->dropped++; }	/* the device refused this endpoint's data */
			else {
				p->len[p->fill] = (uint16_t)len;
				p->fill = (uint8_t)((p->fill + 1) % NBUF);
				p->nfull++;
			}
		}
		push_dev_out(p);
		arm_gadget_out(p);
	}
	return true;
}

static void halt_cleared(xfer_result_t result, uint32_t len, uintptr_t tag)
{
	(void)len;
	struct pipe *p = (struct pipe *)tag;
	(void)result;	/* a device that refuses the clear is left as it is; the pipe runs again */
	if (!p->open) return;
	p->halted = false;
	p->nfull = p->fill = p->drain = 0;	/* what was queued belongs to before the halt */
	if (is_in(p)) arm_dev_in(p);
	else arm_gadget_out(p);
}

void mirror_pipes_gadget_halt_cleared(uint8_t gep)
{
	struct pipe *p = pipe_of(gep);
	if (!p || !p->open) return;
	if (!mirror_host_clear_halt(p->dep, halt_cleared, (uintptr_t)p))
		p->clear_pending = true;	/* the control pipe is busy: from the poll */
}

void mirror_pipes_poll(void)
{
	uint32_t now = time_ms();
	for (unsigned d = 0; d < 2; d++)
		for (unsigned i = 0; i < MIRROR_EP_MAX - 1; i++) {
			struct pipe *p = &pipes[d][i];
			if (!p->open) continue;
			if (p->clear_pending && mirror_host_clear_halt(p->dep, halt_cleared, (uintptr_t)p))
				p->clear_pending = false;
			if (p->retry_at && (int32_t)(now - p->retry_at) >= 0) {
				p->retry_at = 0;
				if (is_in(p)) { arm_dev_in(p); push_gadget_in(p); }
				else { push_dev_out(p); arm_gadget_out(p); }
			}
		}
}

bool mirror_pipes_host_pending(void) { return false; }	/* completions come through the interrupts */

void mirror_pipes_status(char *buf, unsigned n)
{
	int k = 0;
	for (unsigned d = 0; d < 2 && k >= 0 && (unsigned)k < n; d++)
		for (unsigned i = 0; i < MIRROR_EP_MAX - 1 && (unsigned)k < n; i++) {
			const struct pipe *p = &pipes[d][i];
			if (!p->open) continue;
			k += snprintf(buf + k, n - (unsigned)k, "pipe%02x=dev:%02x %s mps:%u halted:%d busy:%d/%d full:%u pkts:%lu bytes:%lu zlp:%lu stalls:%lu fails:%lu retries:%lu dropped:%lu\n",
				      p->gep, p->dep, p->type == TUSB_XFER_BULK ? "bulk" : "intr", p->mps, p->halted, p->g_busy, p->d_busy,
				      p->nfull, (unsigned long)p->pkts, (unsigned long)p->bytes, (unsigned long)p->zlps, (unsigned long)p->stalls,
				      (unsigned long)p->fails, (unsigned long)p->retries, (unsigned long)p->dropped);
		}
	if (k == 0) snprintf(buf, n, "pipes=none\n");
}
