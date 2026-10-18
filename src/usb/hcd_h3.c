/* TinyUSB HCD for the H3's USB-A port (the watch cradle): one port, two
 * controllers. EHCI1 owns the port for high-speed devices; full and
 * low-speed devices belong to its OHCI1 companion, reached by handing the
 * port over (PORTSC PortOwner). usbh sees one rhport; each hcd_* call goes
 * to the controller that owns the port, or that enumerated the device.
 * The EHCI core is portable/ehci/h3/ehci_h3.c in the tinyusb fork, the
 * OHCI core portable/ohci/h3/ohci_h3.c. Also here: PHY1 bring-up,
 * controller resets, EHCI interrupt threshold 1 microframe (the default
 * 8 = 1 ms would put a millisecond on every completion), CONFIGFLAG so
 * the port defaults to EHCI, GIC wiring, cache hooks. */
#include "fw.h"
#include "tusb.h"
#include "host/hcd.h"
#include "ehci_api.h"
#include "ehci_h3.h"
#include "ohci_h3.h"
#include "hcd_h3.h"
#include "usbphy.h"

#define EHCI_CAP        (H3_EHCI1 + 0x00)
#define EHCI_OP         (H3_EHCI1 + 0x10)
#define EHCI_USBCMD     (EHCI_OP + 0x00)
#define EHCI_USBSTS     (EHCI_OP + 0x04)
#define EHCI_USBINTR    (EHCI_OP + 0x08)
#define EHCI_FRINDEX    (EHCI_OP + 0x0c)
#define EHCI_ASYNCLIST  (EHCI_OP + 0x18)
#define EHCI_CONFIGFLAG (EHCI_OP + 0x40)
#define EHCI_PORTSC     (EHCI_OP + 0x44)

#define PORTSC_CCS      BIT(0)
#define PORTSC_CSC      BIT(1)
#define PORTSC_PE       BIT(2)
#define PORTSC_PR       BIT(8)
#define PORTSC_LS_K     (1u << 10)
#define PORTSC_LS_MASK  (3u << 10)
#define PORTSC_OWNER    BIT(13)
#define PORTSC_W1C      (BIT(1) | BIT(3) | BIT(5))

/* OHCI HcRhPortStatus1 */
#define RH_CCS          BIT(0)
#define RH_PES          BIT(1)
#define RH_LSDA         BIT(9)
#define RH_CSC          BIT(16)

static enum hcd_h3_ctrl owner;
/* the controller a device address was enumerated on (ep0 opened), so a
 * device's transfers and close keep their controller */
static uint8_t dev_ctrl[8];
static uint32_t events_dropped, handovers, reclaims;
/* The HC handed the port over on its own (a reset that did not reach high
 * speed): the OHCI takes it once usbh has given up on the EHCI side.
 * give_back: the OHCI's device left; the port idles with the EHCI. */
static bool handover_pending, give_back;
static uint32_t handover_at;

static enum hcd_h3_ctrl ctrl_of(uint8_t daddr)
{
	return daddr < sizeof dev_ctrl ? (enum hcd_h3_ctrl)dev_ctrl[daddr] : owner;
}

/* D-cache maintenance for buffers the controllers read/write (the cores
 * call these before an OUT, before and after an IN). Addresses inside the
 * uncached .dma section (QH/qTD/ED/TD pools, enumeration buffers) need
 * nothing. MVA ops reach the point of coherency, i.e. through the A7's L2. */
extern char __dma_start[], __dma_end[];

static inline bool uncached(const void *addr)
{
	return (const char *)addr >= __dma_start && (const char *)addr < __dma_end;
}

bool hcd_dcache_clean(void const *addr, uint32_t size)
{
	if (!uncached(addr)) dcache_clean_range(addr, size);
	return true;
}
bool hcd_dcache_invalidate(void const *addr, uint32_t size)
{
	if (!uncached(addr)) dcache_invalidate_range(addr, size);
	return true;
}
bool hcd_dcache_clean_invalidate(void const *addr, uint32_t size)
{
	if (!uncached(addr)) dcache_clean_invalidate_range(addr, size);
	return true;
}

/* ---- interrupts and port events --------------------------------------- */
static volatile uint32_t isr_count, ohci_isr_count;
static void ehci_irq(void)
{
	isr_count++;
	ehci_h3_int_handler(1, true);
}
static void ohci_irq(void)
{
	ohci_isr_count++;
	ohci_h3_int_handler(1, true);
}

/* Only the port's owner reports connects and disconnects to usbh; the
 * other controller sees the handover as a disconnect or a connect of its
 * own. */
static void port_changed(enum hcd_h3_ctrl from, bool connected, bool in_isr)
{
	if (from != owner) { events_dropped++; return; }
	if (connected) hcd_event_device_attach(1, in_isr);
	else hcd_event_device_remove(1, in_isr);
}

void ehci_h3_port_changed(bool connected, bool in_isr)
{
	if (!connected && ehci_h3_port_handed_over && owner == HCD_H3_EHCI) {
		ehci_h3_port_handed_over = 0;
		handover_pending = true;
		handover_at = time_ms();
	}
	port_changed(HCD_H3_EHCI, connected, in_isr);
}

void ohci_h3_port_changed(bool connected, bool in_isr)
{
	port_changed(HCD_H3_OHCI, connected, in_isr);
	if (!connected && owner == HCD_H3_OHCI)
		give_back = true;
}

/* ---- moving the port -------------------------------------------------- */
/* To the OHCI. The EHCI's disconnect that follows is dropped (it is no
 * longer the owner) and the OHCI's connect is acknowledged here, so usbh
 * sees neither; the caller posts what usbh should see. */
static bool hand_over(void)
{
	gic_mask(IRQ_EHCI1);
	gic_mask(IRQ_OHCI1);
	owner = HCD_H3_OHCI;
	writel((readl(EHCI_PORTSC) & ~PORTSC_W1C) | PORTSC_OWNER, EHCI_PORTSC);
	bool seen = false;
	for (int i = 0; i < 10000; i++) {
		if (ohci_h3_rhport_status() & RH_CCS) { seen = true; break; }
		delay_us(1);
	}
	ohci_h3_rhport_write(RH_CSC);
	writel((readl(EHCI_PORTSC) & ~PORTSC_W1C) | PORTSC_CSC, EHCI_PORTSC);
	ehci_h3_port_handed_over = 0;
	handovers++;
	gic_unmask(IRQ_OHCI1);
	gic_unmask(IRQ_EHCI1);
	return seen;
}

/* Back to the EHCI, which then reports the device (if any) as a fresh
 * connect; usbh is told here rather than from the interrupt. */
static void take_back(void)
{
	gic_mask(IRQ_EHCI1);
	owner = HCD_H3_EHCI;
	writel(readl(EHCI_PORTSC) & ~(PORTSC_W1C | PORTSC_OWNER), EHCI_PORTSC);
	for (int i = 0; i < 10000 && !(readl(EHCI_PORTSC) & PORTSC_CSC); i++)
		delay_us(1);
	uint32_t v = readl(EHCI_PORTSC);
	writel((v & ~PORTSC_W1C) | PORTSC_CSC, EHCI_PORTSC);
	if (v & PORTSC_CCS)
		hcd_event_device_attach(1, false);
	gic_unmask(IRQ_EHCI1);
}

/* The host manager's chirp retry: the device goes away for usbh and the
 * EHCI gets the port and the device back, to reset it again. */
void hcd_h3_reclaim(void)
{
	reclaims++;
	hcd_event_device_remove(1, false);
	take_back();
}

void hcd_h3_poll(void)
{
	if (handover_pending && time_ms() - handover_at >= 100) {
		/* usbh has given up on the EHCI side: the device is the OHCI's now
		 * (moved here when the HC handed it over by itself, or already in
		 * hcd_port_reset_end) and usbh gets it as a fresh attach */
		handover_pending = false;
		gic_mask(IRQ_OHCI1);
		if (owner == HCD_H3_EHCI) {
			owner = HCD_H3_OHCI;
			handovers++;
			log_printf("host: not high speed; port handed to OHCI1 (%lu)\n", (unsigned long)handovers);
		}
		if (ohci_h3_rhport_status() & RH_CCS) {
			ohci_h3_rhport_write(RH_CSC);
			hcd_event_device_attach(1, false);
		} else {
			give_back = true;
		}
		gic_unmask(IRQ_OHCI1);
	}
	if (give_back) {
		give_back = false;
		take_back();
	}
	if (owner != HCD_H3_EHCI) return;
	gic_mask(IRQ_EHCI1);
	ehci_h3_poll_async(1);
	gic_unmask(IRQ_EHCI1);
}

/* ---- controller bring-up ---------------------------------------------- */
bool hcd_init(uint8_t rhport, const tusb_rhport_init_t *rh_init)
{
	(void)rh_init;
	usbphy1_init();

	/* Halt, then reset the controller (EHCI 2.3.1). */
	clrbits(EHCI_USBCMD, 1);
	for (int i = 0; i < 10000 && !(readl(EHCI_USBSTS) & BIT(12)); i++)
		delay_us(1);
	setbits(EHCI_USBCMD, BIT(1));
	for (int i = 0; i < 10000 && (readl(EHCI_USBCMD) & BIT(1)); i++)
		delay_us(1);

	/* Interrupt Threshold Control = 1 microframe, while halted. ehci_init()
	 * ORs RUN/schedule enables into USBCMD and leaves this field alone. */
	uint32_t cmd = readl(EHCI_USBCMD);
	cmd = (cmd & ~(0xffu << 16)) | (1u << 16);
	writel(cmd, EHCI_USBCMD);

	if (!ehci_init(rhport, EHCI_CAP, EHCI_OP))
		return false;
	writel(1, EHCI_CONFIGFLAG);             /* ports to EHCI, not the companion */
	setbits(EHCI_PORTSC, BIT(12));          /* port power (PPC or not, harmless) */
	if (!ohci_h3_init(rhport, H3_OHCI1))
		return false;
	owner = HCD_H3_EHCI;
	gic_set_handler(IRQ_EHCI1, ehci_irq);
	gic_set_handler(IRQ_OHCI1, ohci_irq);
	log_printf("ehci1: up, usbcmd=%08lx usbsts=%08lx hcsparams=%08lx portsc=%08lx\n",
		   (unsigned long)readl(EHCI_USBCMD), (unsigned long)readl(EHCI_USBSTS),
		   (unsigned long)readl(EHCI_CAP + 0x04), (unsigned long)readl(EHCI_PORTSC));
	log_printf("ohci1: up, revision=%08lx control=%08lx rhstatus=%08lx\n",
		   (unsigned long)ohci_h3_read(0x00), (unsigned long)ohci_h3_read(0x04),
		   (unsigned long)ohci_h3_rhport_status());
	return true;
}

bool hcd_deinit(uint8_t rhport)
{
	gic_disable(IRQ_EHCI1);
	gic_disable(IRQ_OHCI1);
	ohci_h3_halt();
	return ehci_deinit(rhport);
}

/* usbh brackets its event queue with these: both controllers are quiet. */
void hcd_int_enable(uint8_t rhport) { (void)rhport; gic_unmask(IRQ_EHCI1); gic_unmask(IRQ_OHCI1); }
void hcd_int_disable(uint8_t rhport) { (void)rhport; gic_mask(IRQ_EHCI1); gic_mask(IRQ_OHCI1); }
void hcd_int_handler(uint8_t rhport, bool in_isr)
{
	if (owner == HCD_H3_EHCI) ehci_h3_int_handler(rhport, in_isr);
	else ohci_h3_int_handler(rhport, in_isr);
}

/* ---- the port: whoever owns it ---------------------------------------- */
uint32_t hcd_frame_number(uint8_t rhport)
{
	return owner == HCD_H3_EHCI ? ehci_h3_frame_number(rhport) : ohci_h3_frame_number(rhport);
}
bool hcd_port_connect_status(uint8_t rhport)
{
	if (handover_pending) return false;	/* the EHCI's enumeration ends here */
	return owner == HCD_H3_EHCI ? ehci_h3_port_connect_status(rhport) : ohci_h3_port_connect_status(rhport);
}

/* usbh's reset after the attach debounce. A low-speed device (K line state)
 * is the OHCI's before any reset: the port moves and the same enumeration
 * goes on there. A full-speed device is found out by the reset itself: the
 * HC hands the port over when it ends without high speed, see
 * ehci_h3_port_changed. */
void hcd_port_reset(uint8_t rhport)
{
	if (owner == HCD_H3_EHCI) {
		uint32_t v = readl(EHCI_PORTSC);
		if ((v & PORTSC_CCS) && !(v & PORTSC_PE) && (v & PORTSC_LS_MASK) == PORTSC_LS_K) {
			bool seen = hand_over();
			log_printf("host: low-speed device; port handed to OHCI1 (%lu)%s\n",
				   (unsigned long)handovers, seen ? "" : ", not seen there");
			ohci_h3_port_reset(rhport);
			return;
		}
		ehci_h3_port_reset(rhport);
	} else {
		ohci_h3_port_reset(rhport);
	}
}
/* The end of usbh's reset. The EHCI then enables the port only for a
 * high-speed device; a device still attached to a disabled port is full
 * speed and goes to the OHCI, where usbh starts over (handover_pending:
 * this enumeration sees no device). */
void hcd_port_reset_end(uint8_t rhport)
{
	if (owner != HCD_H3_EHCI) { ohci_h3_port_reset_end(rhport); return; }
	ehci_h3_port_reset_end(rhport);
	for (int i = 0; i < 3000 && (readl(EHCI_PORTSC) & PORTSC_PR); i++)
		delay_us(1);
	uint32_t v = readl(EHCI_PORTSC);
	if ((v & PORTSC_CCS) && !(v & PORTSC_PE) && !(v & PORTSC_OWNER)) {
		bool seen = hand_over();
		handover_pending = true;
		handover_at = time_ms();
		log_printf("host: full-speed device; port handed to OHCI1 (%lu)%s\n",
			   (unsigned long)handovers, seen ? "" : ", not seen there");
	}
}
tusb_speed_t hcd_port_speed_get(uint8_t rhport)
{
	return owner == HCD_H3_EHCI ? ehci_h3_port_speed_get(rhport) : ohci_h3_port_speed_get(rhport);
}

/* ---- devices: whoever enumerated them --------------------------------- */
void hcd_device_close(uint8_t rhport, uint8_t daddr)
{
	if (ctrl_of(daddr) == HCD_H3_EHCI) ehci_h3_device_close(rhport, daddr);
	else ohci_h3_device_close(rhport, daddr);
}
bool hcd_edpt_open(uint8_t rhport, uint8_t daddr, tusb_desc_endpoint_t const *ep)
{
	if (ep->bEndpointAddress == 0 && daddr < sizeof dev_ctrl)
		dev_ctrl[daddr] = (uint8_t)owner;	/* ep0 first: the device lands here */
	return ctrl_of(daddr) == HCD_H3_EHCI ? ehci_h3_edpt_open(rhport, daddr, ep) : ohci_h3_edpt_open(rhport, daddr, ep);
}
bool hcd_edpt_close(uint8_t rhport, uint8_t daddr, uint8_t ep_addr)
{
	return ctrl_of(daddr) == HCD_H3_EHCI ? ehci_h3_edpt_close(rhport, daddr, ep_addr) : ohci_h3_edpt_close(rhport, daddr, ep_addr);
}
bool hcd_edpt_xfer(uint8_t rhport, uint8_t daddr, uint8_t ep_addr, uint8_t *buf, uint16_t len)
{
	return ctrl_of(daddr) == HCD_H3_EHCI ? ehci_h3_edpt_xfer(rhport, daddr, ep_addr, buf, len)
					     : ohci_h3_edpt_xfer(rhport, daddr, ep_addr, buf, len);
}
bool hcd_edpt_abort_xfer(uint8_t rhport, uint8_t daddr, uint8_t ep_addr)
{
	return ctrl_of(daddr) == HCD_H3_EHCI ? ehci_h3_edpt_abort_xfer(rhport, daddr, ep_addr)
					     : ohci_h3_edpt_abort_xfer(rhport, daddr, ep_addr);
}
bool hcd_setup_send(uint8_t rhport, uint8_t daddr, uint8_t const setup[8])
{
	return ctrl_of(daddr) == HCD_H3_EHCI ? ehci_h3_setup_send(rhport, daddr, setup) : ohci_h3_setup_send(rhport, daddr, setup);
}
bool hcd_edpt_clear_stall(uint8_t rhport, uint8_t daddr, uint8_t ep_addr)
{
	return ctrl_of(daddr) == HCD_H3_EHCI ? ehci_h3_edpt_clear_stall(rhport, daddr, ep_addr)
					     : ohci_h3_edpt_clear_stall(rhport, daddr, ep_addr);
}

/* ---- what the firmware reads ------------------------------------------ */
enum hcd_h3_ctrl hcd_h3_owner(void) { return owner; }

void hcd_h3_status_line(char *buf, unsigned n)
{
	snprintf(buf, n, "host_owner=%s\nhost_handovers=%lu\nhost_reclaims=%lu\nhost_events_dropped=%lu\n",
		 owner == HCD_H3_EHCI ? "ehci" : "ohci", (unsigned long)handovers, (unsigned long)reclaims,
		 (unsigned long)events_dropped);
}

void hcd_h3_port_status(struct hcd_h3_port *p)
{
	p->owner = owner;
	if (owner == HCD_H3_EHCI) {
		uint32_t v = readl(EHCI_PORTSC);
		p->connected = v & PORTSC_CCS;
		p->enabled = v & PORTSC_PE;
		p->low_speed = (v & PORTSC_LS_MASK) == PORTSC_LS_K;
	} else {
		uint32_t v = ohci_h3_rhport_status();
		p->connected = v & RH_CCS;
		p->enabled = v & RH_PES;
		p->low_speed = v & RH_LSDA;
	}
}

uint32_t hcd_h3_frame_counter(void)
{
	return owner == HCD_H3_EHCI ? readl(EHCI_FRINDEX) : ohci_h3_read(0x3c);
}

/* Console `hangusb`: stop the owning controller's frames. */
void hcd_h3_halt_owner(void)
{
	if (owner == HCD_H3_EHCI) clrbits(EHCI_USBCMD, 1u);
	else ohci_h3_halt();
}

/* Console `ehci`: both controllers' registers and the EHCI async ring. */
void hcd_h3_dump(void (*out)(const char *))
{
	char b[160];
	snprintf(b, sizeof b, "owner=%s cmd=%08lx sts=%08lx intr=%08lx frindex=%08lx async=%08lx portsc=%08lx irqs=%lu\n",
		 owner == HCD_H3_EHCI ? "ehci" : "ohci",
		 (unsigned long)readl(EHCI_USBCMD), (unsigned long)readl(EHCI_USBSTS), (unsigned long)readl(EHCI_USBINTR),
		 (unsigned long)readl(EHCI_FRINDEX), (unsigned long)readl(EHCI_ASYNCLIST),
		 (unsigned long)readl(EHCI_PORTSC), (unsigned long)isr_count);
	out(b);
	snprintf(b, sizeof b, "ohci control=%08lx cmdsts=%08lx intsts=%08lx fmnum=%08lx rhport=%08lx irqs=%lu dropped=%lu\n",
		 (unsigned long)ohci_h3_read(0x04), (unsigned long)ohci_h3_read(0x08), (unsigned long)ohci_h3_read(0x0c),
		 (unsigned long)ohci_h3_read(0x3c), (unsigned long)ohci_h3_rhport_status(), (unsigned long)ohci_isr_count,
		 (unsigned long)events_dropped);
	out(b);
	uint32_t head = readl(EHCI_ASYNCLIST) & ~0x1fu, qh = head;
	for (int i = 0; i < 8 && qh >= 0x40000000 && qh < 0x50000000; i++) {
		const volatile uint32_t *q = (const volatile uint32_t *)(uintptr_t)qh;
		uint32_t ep = q[1];
		snprintf(b, sizeof b, "qh@%08lx daddr=%lu ep=%lu mps=%lu H=%lu cur=%08lx next=%08lx tok=%08lx (act=%lu halt=%lu err=%lu len=%lu pid=%lu)\n",
			 (unsigned long)qh, (unsigned long)(ep & 0x7f), (unsigned long)((ep >> 8) & 0xf),
			 (unsigned long)((ep >> 16) & 0x7ff), (unsigned long)((ep >> 15) & 1),
			 (unsigned long)q[3], (unsigned long)q[4], (unsigned long)q[6],
			 (unsigned long)((q[6] >> 7) & 1), (unsigned long)((q[6] >> 6) & 1),
			 (unsigned long)((q[6] >> 10) & 3), (unsigned long)((q[6] >> 16) & 0x7fff),
			 (unsigned long)((q[6] >> 8) & 3));
		out(b);
		uint32_t next = q[4];
		if (!(next & 1)) {
			const volatile uint32_t *t = (const volatile uint32_t *)(uintptr_t)(next & ~0x1fu);
			snprintf(b, sizeof b, "   qtd@%08lx tok=%08lx (act=%lu halt=%lu len=%lu pid=%lu dt=%lu ioc=%lu) buf0=%08lx\n",
				 (unsigned long)(next & ~0x1fu), (unsigned long)t[2], (unsigned long)((t[2] >> 7) & 1),
				 (unsigned long)((t[2] >> 6) & 1), (unsigned long)((t[2] >> 16) & 0x7fff),
				 (unsigned long)((t[2] >> 8) & 3), (unsigned long)((t[2] >> 31) & 1),
				 (unsigned long)((t[2] >> 15) & 1), (unsigned long)t[3]);
			out(b);
		}
		qh = q[0] & ~0x1fu;
		if (qh == head) break;
	}
}

uint32_t hcd_h3_portsc(void) { return readl(EHCI_PORTSC); }
void hcd_h3_portsc_write(uint32_t v) { writel(v, EHCI_PORTSC); }
