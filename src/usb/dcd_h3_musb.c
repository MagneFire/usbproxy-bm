/* Forked from tinyusb 0.21.0 src/portable/sunxi/dcd_sunxi_musb.c (Allwinner
 * F1C100s, MIT, Copyright (c) 2021 Tian Yunhao / Koji Kitayama / Ha Thach)
 * for the Allwinner H3 (usbproxy-bm). Same sunxi musb register layout; the
 * H3 differences are the clock/reset/PHY glue (usbphy.c), the GIC, an 8 KiB
 * FIFO RAM, and no SRAM-controller FIFO mapping. Look for "H3". */
/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Koji KITAYAMA
 * Copyright (c) 2021 Tian Yunhao (t123yh)
 * Copyright (c) 2021 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include <stdint.h>
#include "fw.h"
#include "tusb_option.h"

#if CFG_TUD_ENABLED && CFG_TUSB_MCU == OPT_MCU_NONE   /* H3 fork */

#include "osal/osal.h"
#include <device/dcd.h>
#include "musb_h3_def.h"
#include "h3/h3.h"
#include "usbphy.h"

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;


#define REQUEST_TYPE_INVALID  (0xFFu)

typedef struct {
  uint_fast16_t beg; /* offset of including first element */
  uint_fast16_t end; /* offset of excluding the last element */
} free_block_t;

typedef struct TU_ATTR_PACKED
{
  void      *buf;      /* the start address of a transfer data buffer */
  uint16_t  length;    /* the number of bytes in the buffer */
  uint16_t  remaining; /* the number of bytes remaining in the buffer */
} pipe_state_t;

typedef struct
{
  CFG_TUD_MEM_ALIGN tusb_control_request_t setup_packet;
  uint16_t     remaining_ctrl; /* The number of bytes remaining in data stage of control transfer. */
  int8_t       status_out;
  pipe_state_t pipe0;
  pipe_state_t pipe[2][7];   /* pipe[direction][endpoint number - 1] */
  uint16_t     pipe_buf_is_fifo[2]; /* Bitmap. Each bit means whether 1:TU_FIFO or 0:POD. */
  uint16_t     dpb_in;              /* H3: IN endpoints with double packet buffering */
  uint32_t     dpb_tx_waits, dpb_tx_early, dpb_rx_extra, tx_busy_irqs; /* H3 DPB counters (console status) */
  uint32_t     fifo_rd_us, fifo_rd_n, fifo_wr_us, fifo_wr_n;            /* PIO copy cost, packets >= 256 B */
  uint16_t     in_zlp_pending;      /* H3: bitmap, a zero-length IN transfer still to be sent */
  uint16_t     rx_held;             /* H3: bitmap, the OUT packet in the FIFO was read (back-pressure) */
  uint16_t     in_armed;            /* H3: bitmap, an IN transfer is in progress */
} dcd_data_t;

/*------------------------------------------------------------------
 * SUNXI FUNCTION
 *------------------------------------------------------------------*/

static void USBC_HardwareReset(void)
{
  /* H3: clocks, resets, PHY0 routing to musb and the PMU SIDDQ clear are in
   * usbphy.c (usbphy0_init), shared with the H3-specific bits the F1C100s
   * never had. */
  usbphy0_init();
}

static unsigned int USBC_WakeUp_ClearChangeDetect(unsigned int reg_val)
{
	unsigned int temp = reg_val;
    /* vbus, id, dpdm, these bit is set 1 to clear, so we clear these bit when operate other bits */
	temp &= ~(1 << USBC_BP_ISCR_VBUS_CHANGE_DETECT);
	temp &= ~(1 << USBC_BP_ISCR_ID_CHANGE_DETECT);
	temp &= ~(1 << USBC_BP_ISCR_DPDM_CHANGE_DETECT);

	return temp;
}

static void USBC_EnableDpDmPullUp(void)
{
	u32 reg_val = USBC_Readl(USBC_REG_ISCR(USBC0_BASE));
	reg_val |= (1 << USBC_BP_ISCR_DPDM_PULLUP_EN);
	reg_val |= 3<<USBC_BP_ISCR_VBUS_VALID_SRC;
	reg_val = USBC_WakeUp_ClearChangeDetect(reg_val);
	USBC_Writel(reg_val, USBC_REG_ISCR(USBC0_BASE));
}

static void USBC_ForceIdToHigh(void)
{
	/* first write 00, then write 10 */
	u32 reg_val = USBC_Readl(USBC_REG_ISCR(USBC0_BASE));
	reg_val |= (0x03 << USBC_BP_ISCR_FORCE_ID);
	reg_val = USBC_WakeUp_ClearChangeDetect(reg_val);
	USBC_Writel(reg_val, USBC_REG_ISCR(USBC0_BASE));
}

static void USBC_ForceVbusValidToHigh(void)
{
	/* first write 00, then write 11 */
	u32 reg_val = USBC_Readl(USBC_REG_ISCR(USBC0_BASE));
	reg_val |= (0x03 << USBC_BP_ISCR_FORCE_VBUS_VALID);
	reg_val = USBC_WakeUp_ClearChangeDetect(reg_val);
	USBC_Writel(reg_val, USBC_REG_ISCR(USBC0_BASE));
}

static void USBC_SelectBus(u32 io_type, u32 ep_type, u32 ep_index)
{
	u32 reg_val = 0;

	reg_val = USBC_Readb(USBC_REG_VEND0(USBC0_BASE));
	if (io_type == USBC_IO_TYPE_DMA) {
		if (ep_type == USBC_EP_TYPE_TX) {
			reg_val |= ((ep_index - 0x01) << 1) << USBC_BP_VEND0_DRQ_SEL;  //drq_sel
			reg_val |= 0x1<<USBC_BP_VEND0_BUS_SEL;   //io_dma
		} else {
			reg_val |= ((ep_index << 1) - 0x01) << USBC_BP_VEND0_DRQ_SEL;
			reg_val |= 0x1<<USBC_BP_VEND0_BUS_SEL;
		}
	} else {
		//reg_val &= ~(0x1 << USBC_BP_VEND0_DRQ_SEL);  //clear drq_sel, select pio
		reg_val &= 0x00;  // clear drq_sel, select pio
	}

	/* in 1667 1673 and later ic, FIFO_BUS_SEL bit(bit24 of reg0x40 for host/device)
	 * is fixed to 1, the hw guarantee that it's ok for cpu/inner_dma/outer_dma transfer */

//	reg_val |= 0x1<<USBC_BP_VEND0_BUS_SEL;  //for 1663 set 1: enable dma, set 0: enable fifo

	USBC_Writeb(reg_val, USBC_REG_VEND0(USBC0_BASE));
}

static void USBC_SelectActiveEp(u8 ep_index)
{
	USBC_Writeb(ep_index, USBC_REG_EPIND(USBC0_BASE));
}

static u8 USBC_GetActiveEp(void)
{
	return USBC_Readb(USBC_REG_EPIND(USBC0_BASE));
}

static void __USBC_Dev_ep0_SendStall(void)
{
	USBC_REG_set_bit_w(USBC_BP_CSR0_D_SEND_STALL, USBC_REG_CSR0(USBC0_BASE));
}

static void __USBC_Dev_ep0_ClearStall(void)
{
	USBC_REG_clear_bit_w(USBC_BP_CSR0_D_SEND_STALL, USBC_REG_CSR0(USBC0_BASE));
	USBC_REG_clear_bit_w(USBC_BP_CSR0_D_SENT_STALL, USBC_REG_CSR0(USBC0_BASE));
}

static void USBC_Dev_Ctrl_ClearSetupEnd(void)
{
	USBC_REG_set_bit_w(USBC_BP_CSR0_D_SERVICED_SETUP_END, USBC_REG_CSR0(USBC0_BASE));
}

static void USBC_Dev_SetAddress(u8 address)
{
	USBC_Writeb(address, USBC_REG_FADDR(USBC0_BASE));
}

static void __USBC_Dev_Tx_SendStall(void)
{
	//send stall, and fifo is flushed automatically
	USBC_REG_set_bit_w(USBC_BP_TXCSR_D_SEND_STALL, USBC_REG_TXCSR(USBC0_BASE));
}
static u32 __USBC_Dev_Tx_IsEpStall(void)
{
	return USBC_REG_test_bit_w(USBC_BP_TXCSR_D_SENT_STALL, USBC_REG_TXCSR(USBC0_BASE));
}
/* H3: a STALL went out. Acknowledge it but keep SEND_STALL: the endpoint
 * stays halted until dcd_edpt_clear_stall() (the host clearing its side of
 * the pipe does not reach us: IOKit's ClearPipeStall is host-only). The
 * upstream sunxi DCD cleared SEND_STALL here, so a halt stalled exactly one
 * transaction and the bridge's halt pulse never kicked macOS adb. */
static void __USBC_Dev_Tx_AckSentStall(void)
{
	u16 reg_val = USBC_Readw(USBC_REG_TXCSR(USBC0_BASE));
	reg_val &= (u16)~((1 << USBC_BP_TXCSR_D_SENT_STALL) | (1 << USBC_BP_TXCSR_D_UNDER_RUN) | (1 << USBC_BP_TXCSR_D_TX_READY));
	USBC_Writew(reg_val, USBC_REG_TXCSR(USBC0_BASE));
}
static void __USBC_Dev_Rx_AckSentStall(void)
{
	u16 reg_val = USBC_Readw(USBC_REG_RXCSR(USBC0_BASE));
	reg_val &= (u16)~((1 << USBC_BP_RXCSR_D_SENT_STALL) | (1 << USBC_BP_RXCSR_D_OVERRUN));
	USBC_Writew(reg_val, USBC_REG_RXCSR(USBC0_BASE));
}

static void __USBC_Dev_Tx_ClearStall(void)
{
	u32 reg_val = USBC_Readw(USBC_REG_TXCSR(USBC0_BASE));
	reg_val &= ~((1 << USBC_BP_TXCSR_D_SENT_STALL)|(1 << USBC_BP_TXCSR_D_SEND_STALL)|(1<<USBC_BP_TXCSR_D_UNDER_RUN));
  reg_val |= (1 << USBC_BP_TXCSR_D_CLEAR_DATA_TOGGLE);
	USBC_Writew(reg_val, USBC_REG_TXCSR(USBC0_BASE));
}

static void __USBC_Dev_Rx_SendStall(void)
{
	USBC_REG_set_bit_w(USBC_BP_RXCSR_D_SEND_STALL, USBC_REG_RXCSR(USBC0_BASE));
}

static u32 __USBC_Dev_Rx_IsEpStall(void)
{
	return USBC_REG_test_bit_w(USBC_BP_RXCSR_D_SENT_STALL, USBC_REG_RXCSR(USBC0_BASE));
}

static void __USBC_Dev_Rx_ClearStall(void)
{
	u32 reg_val = USBC_Readw(USBC_REG_RXCSR(USBC0_BASE));
	reg_val &= ~((1 << USBC_BP_RXCSR_D_SENT_STALL)|(1 << USBC_BP_RXCSR_D_SEND_STALL)|(1<<USBC_BP_RXCSR_D_OVERRUN));
  reg_val |= (1 << USBC_BP_RXCSR_D_CLEAR_DATA_TOGGLE);
	USBC_Writew(reg_val, USBC_REG_RXCSR(USBC0_BASE));
}

static tusb_speed_t USBC_Dev_QueryTransferMode(void)
{
	if (USBC_REG_test_bit_b(USBC_BP_POWER_D_HIGH_SPEED_FLAG, USBC_REG_PCTL(USBC0_BASE)))
		return TUSB_SPEED_HIGH;
  else
		return TUSB_SPEED_FULL;
}

static void __USBC_Dev_ep0_ReadDataHalf(void)
{
	USBC_Writew(1<<USBC_BP_CSR0_D_SERVICED_RX_PKT_READY, USBC_REG_CSR0(USBC0_BASE));
}

static void __USBC_Dev_ep0_ReadDataComplete(void)
{
	USBC_Writew((1<<USBC_BP_CSR0_D_SERVICED_RX_PKT_READY) | (1<<USBC_BP_CSR0_D_DATA_END),
	USBC_REG_CSR0(USBC0_BASE));
}


static void __USBC_Dev_ep0_WriteDataHalf(void)
{
	USBC_Writew(1<<USBC_BP_CSR0_D_TX_PKT_READY, USBC_REG_CSR0(USBC0_BASE));
}

static void __USBC_Dev_ep0_WriteDataComplete(void)
{
	USBC_Writew((1<<USBC_BP_CSR0_D_TX_PKT_READY) | (1<<USBC_BP_CSR0_D_DATA_END),
	USBC_REG_CSR0(USBC0_BASE));
}

/* keeps SEND_STALL: the byte write clears every other bit it leaves at 0 */
static void __USBC_Dev_Tx_WriteDataComplete(void)
{
	u8 v = USBC_Readb(USBC_REG_TXCSR(USBC0_BASE)) & (1 << USBC_BP_TXCSR_D_SEND_STALL);
	USBC_Writeb(v | (1 << USBC_BP_TXCSR_D_TX_READY), USBC_REG_TXCSR(USBC0_BASE));
}

__attribute__((unused)) static void __USBC_Dev_Rx_ReadDataComplete(void)
{
	USBC_Writeb(0, USBC_REG_RXCSR(USBC0_BASE));
}

static u32 __USBC_Dev_Rx_IsReadDataReady(void)
{
	return USBC_REG_test_bit_w(USBC_BP_RXCSR_D_RX_PKT_READY, USBC_REG_RXCSR(USBC0_BASE));
}

/* H3 DPB: release the packet just read and tell whether the other buffer
 * already holds the next one. FIFOFULL (both buffers occupied) is sampled
 * before the release because RXPKTRDY takes a few cycles to re-assert for
 * the second packet, and its interrupt flag may have been folded into the
 * one we are servicing: a plain RXPKTRDY check right after the release read
 * 0 and left the packet stranded (OUT flood stalled in loopback). */
static bool rx_more;
static void __USBC_Dev_Rx_Release(void)
{
	bool full = USBC_REG_test_bit_w(USBC_BP_RXCSR_D_FIFO_FULL, USBC_REG_RXCSR(USBC0_BASE));
	USBC_Writeb(0, USBC_REG_RXCSR(USBC0_BASE));
	if (full)
		for (int i = 0; i < 200 && !__USBC_Dev_Rx_IsReadDataReady(); i++)
			;
	rx_more = __USBC_Dev_Rx_IsReadDataReady();
}

/* open a tx ep's interrupt */
static void USBC_INT_EnableTxEp(u8 ep_index)
{
	USBC_REG_set_bit_w(ep_index, USBC_REG_INTTxE(USBC0_BASE));
}

/* open a rx ep's interrupt */
static void USBC_INT_EnableRxEp(u8 ep_index)
{
	USBC_REG_set_bit_w(ep_index, USBC_REG_INTRxE(USBC0_BASE));
}

/* close a tx ep's interrupt */
static void USBC_INT_DisableTxEp(u8 ep_index)
{
	USBC_REG_clear_bit_w(ep_index, USBC_REG_INTTxE(USBC0_BASE));
}

/* close a rx ep's interrupt */
static void USBC_INT_DisableRxEp(u8 ep_index)
{
	USBC_REG_clear_bit_w(ep_index, USBC_REG_INTRxE(USBC0_BASE));
}

/*------------------------------------------------------------------
 * INTERNAL FUNCTION DECLARATION
 *------------------------------------------------------------------*/

CFG_TUD_MEM_ALIGN static dcd_data_t _dcd;

static inline free_block_t *find_containing_block(free_block_t *beg, free_block_t *end, uint_fast16_t addr)
{
  free_block_t *cur = beg;
  for (; cur < end && ((addr < cur->beg) || (cur->end <= addr)); ++cur) ;
  return cur;
}

static inline int update_free_block_list(free_block_t *blks, unsigned num, uint_fast16_t addr, uint_fast16_t size)
{
  free_block_t *p = find_containing_block(blks, blks + num, addr);
  TU_ASSERT(p != blks + num, -2);
  if (p->beg == addr) {
    /* Shrink block */
    p->beg = addr + size;
    if (p->beg != p->end) return 0;
    /* remove block */
    free_block_t *end = blks + num;
    while (p + 1 < end) {
      *p = *(p + 1);
      ++p;
    }
    return -1;
  } else {
    /* Split into 2 blocks */
    free_block_t tmp = {
      .beg = addr + size,
      .end = p->end
    };
    p->end = addr;
    if (p->beg == p->end) {
      if (tmp.beg != tmp.end) {
        *p = tmp;
        return 0;
      }
      /* remove block */
      free_block_t *end = blks + num;
      while (p + 1 < end) {
        *p = *(p + 1);
        ++p;
      }
      return -1;
    }
    if (tmp.beg == tmp.end) return 0;
    blks[num] = tmp;
    return 1;
  }
}

static inline unsigned free_block_size(free_block_t const *blk)
{
  return blk->end - blk->beg;
}

#if 0
static inline void print_block_list(free_block_t const *blk, unsigned num)
{
  TU_LOG1("*************\r\n");
  for (unsigned i = 0; i < num; ++i) {
    TU_LOG1(" Blk%u %u %u\r\n", i, blk->beg, blk->end);
    ++blk;
  }
}
#else
#define print_block_list(a,b)
#endif

#define USB_FIFO_SIZE_KB 8   /* H3: ram_bits 11 (Linux sunxi.c) */

__attribute__((unused)) static unsigned find_free_memory(uint_fast16_t size_in_log2_minus3)
{
  free_block_t free_blocks[2 * (TUP_DCD_ENDPOINT_MAX - 1)];
  unsigned num_blocks = 1;
  /* Backup current EP to restore later */
  u8 backup_ep = USBC_GetActiveEp();

  /* Initialize free memory block list */
  free_blocks[0].beg = 64 / 8;
  free_blocks[0].end = (USB_FIFO_SIZE_KB << 10) / 8; /* 2KiB / 8 bytes */
  for (int i = 1; i < TUP_DCD_ENDPOINT_MAX; ++i) {
    uint_fast16_t addr;
    int num;
    USBC_SelectActiveEp(i);
    addr = USBC_Readw(USBC_REG_TXFIFOAD(USBC0_BASE));
    if (addr) {
      unsigned sz  = USBC_Readb(USBC_REG_TXFIFOSZ(USBC0_BASE));
      unsigned sft = (sz & USB_TXFIFOSZ_SIZE_M) + ((sz & USB_TXFIFOSZ_DPB) ? 1: 0);
      num = update_free_block_list(free_blocks, num_blocks, addr, 1 << sft);
      TU_ASSERT(-2 < num, 0);
      num_blocks += num;
      print_block_list(free_blocks, num_blocks);
    }
    addr = USBC_Readw(USBC_REG_RXFIFOAD(USBC0_BASE));
    if (addr) {
      unsigned sz  = USBC_Readb(USBC_REG_RXFIFOSZ(USBC0_BASE));
      unsigned sft = (sz & USB_RXFIFOSZ_SIZE_M) + ((sz & USB_RXFIFOSZ_DPB) ? 1: 0);
      num = update_free_block_list(free_blocks, num_blocks, addr, 1 << sft);
      TU_ASSERT(-2 < num, 0);
      num_blocks += num;
      print_block_list(free_blocks, num_blocks);
    }
  }
  print_block_list(free_blocks, num_blocks);

  USBC_SelectActiveEp(backup_ep);

  /* Find the best fit memory block */
  uint_fast16_t size_in_8byte_unit = 1 << size_in_log2_minus3;
  free_block_t const *min = NULL;
  uint_fast16_t    min_sz = 0xFFFFu;
  free_block_t const *end = &free_blocks[num_blocks];
  for (free_block_t const *cur = &free_blocks[0]; cur < end; ++cur) {
    uint_fast16_t sz = free_block_size(cur);
    if (sz < size_in_8byte_unit) continue;
    if (size_in_8byte_unit == sz) return cur->beg;
    if (sz < min_sz) min = cur;
  }
  TU_ASSERT(min, 0);
  return min->beg;
}

static void pipe_write_packet(void *buff, volatile void *fifo, unsigned cnt)
{
 	u32 len = 0;
	u32 i32 = 0;
	u32 i8  = 0;
	u8  *buf8  = 0;
	u32 *buf32 = 0;

	//--<1>-- adjust data
	buf32 = buff;
	len   = cnt;

	i32 = len >> 2;
	i8  = len & 0x03;

	//--<2>-- deal with 4byte part
	while (i32--) {
		USBC_Writel(*buf32++, fifo);
	}

	//--<3>-- deal with no 4byte part
	buf8 = (u8 *)buf32;
	while (i8--) {
		USBC_Writeb(*buf8++, fifo);
	}
}

static void pipe_read_packet(void *buff, volatile void *fifo, unsigned cnt)
{
	u32 len = 0;
	u32 i32 = 0;
	u32 i8  = 0;
	u8  *buf8  = 0;
	u32 *buf32 = 0;

	//--<1>-- adjust data
	buf32 = buff;
	len   = cnt;

	i32 = len >> 2;
	i8  = len & 0x03;

	//--<2>-- deal with 4byte part
	while (i32--) {
		*buf32++ = USBC_Readl(fifo);
	}

	//--<3>-- deal with no 4byte part
	buf8 = (u8 *)buf32;
	while (i8--) {
		*buf8++ = USBC_Readb(fifo);
	}
}

static void pipe_read_write_packet_ff(tu_fifo_t *f, volatile void *fifo, unsigned len, unsigned dir)
{
  static const struct {
    void (*tu_fifo_get_info)(tu_fifo_t *f, tu_fifo_buffer_info_t *info);
    void (*tu_fifo_advance)(tu_fifo_t *f, uint16_t n);
    void (*pipe_read_write)(void *buf, volatile void *fifo, unsigned len);
  } ops[] = {
    /* OUT */ {tu_fifo_get_write_info,tu_fifo_advance_write_pointer,pipe_read_packet},
    /* IN  */ {tu_fifo_get_read_info, tu_fifo_advance_read_pointer, pipe_write_packet},
  };
  tu_fifo_buffer_info_t info;
  ops[dir].tu_fifo_get_info(f, &info);
  unsigned total_len = len;
  len = TU_MIN(total_len, info.linear.len);
  ops[dir].pipe_read_write(info.linear.ptr, fifo, len);
  unsigned rem = total_len - len;
  if (rem) {
    len = TU_MIN(rem, info.wrapped.len);
    ops[dir].pipe_read_write(info.wrapped.ptr, fifo, len);
    rem -= len;
  }
  ops[dir].tu_fifo_advance(f, total_len - rem);
}

/*------------------------------------------------------------------
 * TRANSFER FUNCTION DECLARATION
 *------------------------------------------------------------------*/

static void process_setup_packet(uint8_t rhport)
{
  uint32_t *p = (uint32_t*)(uintptr_t) &_dcd.setup_packet;
  p[0]        = USBC_Readl(USBC_REG_EPFIFO0(USBC0_BASE));
  p[1]        = USBC_Readl(USBC_REG_EPFIFO0(USBC0_BASE));

  _dcd.pipe0.buf       = NULL;
  _dcd.pipe0.length    = 0;
  _dcd.pipe0.remaining = 0;
  dcd_event_setup_received(rhport, (const uint8_t*)(uintptr_t)&_dcd.setup_packet, true);

  const unsigned len    = _dcd.setup_packet.wLength;
  _dcd.remaining_ctrl   = len;
  const unsigned dir_in = tu_edpt_dir(_dcd.setup_packet.bmRequestType);
  /* Clear RX FIFO and reverse the transaction direction */
  if (len && dir_in) __USBC_Dev_ep0_ReadDataHalf();
}

/* ---- H3 USB inner DMA (Allwinner sunxi_udc_dma.c, SW_UDC_DMA_INNER) -----
 * Eight channels behind the musb block; a channel moves whole packets
 * between a FIFO and SDRAM in musb "DMA mode 1" (AUTOSET / AUTOCLEAR, one
 * interrupt per transfer, endpoint interrupt only on a short packet).
 * Register map and CSR sequences from the vendor driver; the burst field
 * carries the packet size there, set to 512. Used for the adb pair only, on
 * 512-byte endpoints (high speed), for the whole-packet part of transfers
 * >= DMA_MIN_BYTES; the short tail stays PIO. */
#define DMA_INTE        (USBC0_BASE + 0x500)
#define DMA_INTS        (USBC0_BASE + 0x504)
#define DMA_CFG(n)      (USBC0_BASE + 0x540 + 0x10 * (n))
#define DMA_ADDR(n)     (USBC0_BASE + 0x544 + 0x10 * (n))
#define DMA_BC(n)       (USBC0_BASE + 0x548 + 0x10 * (n))   /* byte count; reads back residual */
#define DMA_CHAN_RX     0
#define DMA_CHAN_TX     1
#define DMA_MIN_BYTES   1024

struct h3_dma { bool active; bool tx; uint8_t ep_addr; uint32_t len; };
static struct h3_dma dma_ch[2];
static uint32_t dma_rx_n, dma_tx_n, dma_rx_short, dma_tx_spin_us, dma_bad_resid;
/* inter-packet gap histograms on the adb pair (us): <12, <20, <30, <50, <100, >=100 */
static uint32_t gap_rx[6], gap_tx[6], gap_rx_last, gap_tx_last;
static void gap_note(uint32_t *h, uint32_t *last, uint32_t now, bool first)
{
  if (!first && *last) {
    uint32_t g = now - *last;
    h[g < 12 ? 0 : g < 20 ? 1 : g < 30 ? 2 : g < 50 ? 3 : g < 100 ? 4 : 5]++;
  }
  *last = now;
}
void dcd_h3_gap_stats(uint32_t **rx, uint32_t **tx) { *rx = gap_rx; *tx = gap_tx; }
static uint32_t dma_mask = 1;		/* bit0: RX (Mac->gadget) DMA, bit1: TX DMA; console `dma` */
void dcd_h3_dma_enable(uint32_t mask) { dma_mask = mask & 3; }
uint32_t dcd_h3_dma_enabled(void) { return dma_mask; }
void dcd_h3_dma_stats(uint32_t *rx, uint32_t *tx, uint32_t *rx_short, uint32_t *tx_spin_us, uint32_t *bad)
{
  *rx = dma_rx_n; *tx = dma_tx_n; *rx_short = dma_rx_short; *tx_spin_us = dma_tx_spin_us; *bad = dma_bad_resid;
}

/* high-byte RMW of the CSRs, as the vendor does: the low byte holds the
 * write-sensitive RXPKTRDY / TXPKTRDY / FLUSH bits */
static void tx_dma_csr_on(void)
{
  u8 v = USBC_Readb(USBC_REG_TXCSR(USBC0_BASE) + 1);
  v |= ((1 << USBC_BP_TXCSR_D_AUTOSET) | (1 << USBC_BP_TXCSR_D_MODE) |
        (1 << USBC_BP_TXCSR_D_DMA_REQ_EN) | (1 << USBC_BP_TXCSR_D_DMA_REQ_MODE)) >> 8;
  USBC_Writeb(v, USBC_REG_TXCSR(USBC0_BASE) + 1);
}
static void tx_dma_csr_off(void)
{
  u8 v = USBC_Readb(USBC_REG_TXCSR(USBC0_BASE) + 1);
  v &= (u8)~(((1 << USBC_BP_TXCSR_D_AUTOSET) | (1 << USBC_BP_TXCSR_D_DMA_REQ_EN)) >> 8);
  USBC_Writeb(v, USBC_REG_TXCSR(USBC0_BASE) + 1);
  v &= (u8)~((1 << USBC_BP_TXCSR_D_DMA_REQ_MODE) >> 8);    /* "cannot be cleared in the same cycle" */
  USBC_Writeb(v, USBC_REG_TXCSR(USBC0_BASE) + 1);
}
static void rx_dma_csr_on(void)
{
  volatile void *r = (volatile void *)(USBC_REG_RXCSR(USBC0_BASE) + 1);
  u8 v = USBC_Readb(r);
  v |= (1 << USBC_BP_RXCSR_D_DMA_REQ_MODE) >> 8;            USBC_Writeb(v, r);
  v |= ((1 << USBC_BP_RXCSR_D_AUTO_CLEAR) | (1 << USBC_BP_RXCSR_D_DMA_REQ_EN)) >> 8; USBC_Writeb(v, r);
  v &= (u8)~((1 << USBC_BP_RXCSR_D_DMA_REQ_MODE) >> 8);     USBC_Writeb(v, r);
  v |= (1 << USBC_BP_RXCSR_D_DMA_REQ_MODE) >> 8;            USBC_Writeb(v, r);
}
static void rx_dma_csr_off(void)
{
  volatile void *r = (volatile void *)(USBC_REG_RXCSR(USBC0_BASE) + 1);
  u8 v = USBC_Readb(r);
  v &= (u8)~(((1 << USBC_BP_RXCSR_D_AUTO_CLEAR) | (1 << USBC_BP_RXCSR_D_DMA_REQ_MODE) |
              (1 << USBC_BP_RXCSR_D_DMA_REQ_EN)) >> 8);
  USBC_Writeb(v, r);
}

static void dma_start(unsigned ch, uint8_t ep_addr, void *buf, uint32_t len, bool tx)
{
  dma_ch[ch].active = true; dma_ch[ch].tx = tx; dma_ch[ch].ep_addr = ep_addr; dma_ch[ch].len = len;
  USBC_Writel(0, DMA_CFG(ch));
  USBC_Writel((uint32_t)(uintptr_t)buf, DMA_ADDR(ch));
  USBC_Writel(len, DMA_BC(ch));
  USBC_Writew(USBC_Readw(DMA_INTE) | (1u << ch), DMA_INTE);
  uint32_t cfg = (512u << 16) | ((tx ? 0u : 1u) << 4) | (tu_edpt_number(ep_addr) & 0xfu);
  USBC_Writel(cfg | (1u << 31), DMA_CFG(ch));
}

/* stop the channel; returns the bytes it moved */
static uint32_t dma_stop(unsigned ch)
{
  uint32_t resid = USBC_Readl(DMA_BC(ch));
  USBC_Writel(0, DMA_CFG(ch));
  USBC_Writew(USBC_Readw(DMA_INTE) & ~(1u << ch), DMA_INTE);
  USBC_Writel(1u << ch, DMA_INTS);
  dma_ch[ch].active = false;
  if (resid > dma_ch[ch].len) { dma_bad_resid++; resid = dma_ch[ch].len; }
  return dma_ch[ch].len - resid;
}

static bool handle_xfer_in(uint_fast8_t ep_addr);
static bool handle_xfer_out(uint_fast8_t ep_addr);

/* Bytes of the current OUT transfer on ep_addr that have reached memory, for
 * cut-through forwarding (bridge_poll). PIO bytes are exact (the ISR copied
 * them); with RX DMA running the channel's residual count is added and
 * *dma says so, the caller holds a packet back as with the EHCI overlay. */
uint32_t dcd_h3_out_landed(uint8_t ep_addr, bool *dma)
{
  unsigned epn = tu_edpt_number(ep_addr);
  pipe_state_t *pipe = &_dcd.pipe[TUSB_DIR_OUT][epn - 1];
  *dma = false;
  if (!pipe->buf) return 0;
  uint32_t landed = pipe->length - pipe->remaining;
  struct h3_dma *d = &dma_ch[DMA_CHAN_RX];
  if (d->active && d->ep_addr == ep_addr) {
    uint32_t resid = USBC_Readl(DMA_BC(DMA_CHAN_RX));
    if (resid <= d->len) landed += d->len - resid;
    *dma = true;
  }
  return landed;
}

/* DMA channel finished its byte count (ISR) */
static void handle_dma_done(uint8_t rhport, unsigned ch)
{
  struct h3_dma *d = &dma_ch[ch];
  uint8_t ep_addr = d->ep_addr;
  unsigned epn = tu_edpt_number(ep_addr), dir_in = tu_edpt_dir(ep_addr);
  pipe_state_t *pipe = &_dcd.pipe[dir_in][epn - 1];
  USBC_SelectActiveEp(epn);
  uint32_t moved = dma_stop(ch);
  /* RX: drop lines the core may have prefetched while the DMA wrote them */
  if (ch == DMA_CHAN_RX && moved) dcache_invalidate_range(pipe->buf, moved);
  if (d->tx) {
    /* the last packet(s) may still sit in the FIFO: the vendor waits for it */
    uint32_t t0 = time_us();
    while (USBC_REG_test_bit_w(USBC_BP_TXCSR_D_FIFO_NOT_EMPTY, USBC_REG_TXCSR(USBC0_BASE)) && time_us() - t0 < 400)
      ;
    dma_tx_spin_us += time_us() - t0;
    tx_dma_csr_off();
  } else {
    rx_dma_csr_off();
  }
  pipe->buf = (uint8_t *)pipe->buf + moved;
  pipe->remaining = (uint16_t)(pipe->remaining - (moved > pipe->remaining ? pipe->remaining : moved));
  if (pipe->remaining == 0) {
    pipe->buf = NULL;
    if (dir_in) _dcd.in_armed &= (uint16_t)~TU_BIT(epn - 1);
    dcd_event_xfer_complete(rhport, ep_addr, pipe->length, XFER_RESULT_SUCCESS, true);
    return;
  }
  /* a short tail by PIO */
  if (d->tx) {
    handle_xfer_in(ep_addr);
  } else if (__USBC_Dev_Rx_IsReadDataReady()) {
    if (handle_xfer_out(ep_addr))
      dcd_event_xfer_complete(rhport, ep_addr, pipe->length - pipe->remaining, XFER_RESULT_SUCCESS, true);
  }
}

static bool handle_xfer_in(uint_fast8_t ep_addr)
{
  unsigned epnum_minus1 = tu_edpt_number(ep_addr) - 1;
  pipe_state_t  *pipe = &_dcd.pipe[tu_edpt_dir(ep_addr)][epnum_minus1];
  const unsigned rem  = pipe->remaining;

  /* nothing armed: an interrupt for a transfer the halt cancelled */
  if (!(_dcd.in_armed & TU_BIT(epnum_minus1))) return false;
  if (dma_ch[DMA_CHAN_TX].active && dma_ch[DMA_CHAN_TX].ep_addr == ep_addr) return false;
  if (!rem) {
    /* H3 fix: a zero-length transfer must still put one empty packet on
     * the wire; upstream returned "complete" without ever setting TXPKTRDY,
     * so the ZLP never went out and the class driver never got its
     * completion. Send it now, complete on the TX interrupt. */
    if (_dcd.in_zlp_pending & TU_BIT(epnum_minus1)) {
      _dcd.in_zlp_pending &= (uint16_t)~TU_BIT(epnum_minus1);
      __USBC_Dev_Tx_WriteDataComplete();
      return false;
    }
    /* H3 DPB: the interrupt for the next-to-last packet arrives with the
     * last one still in the other buffer; complete only once the FIFO is
     * empty, so the transfer is really on the wire when the stack hears
     * of it (another interrupt follows when that packet goes out). */
    if ((_dcd.dpb_in & TU_BIT(epnum_minus1)) &&
        USBC_REG_test_bit_w(USBC_BP_TXCSR_D_FIFO_NOT_EMPTY, USBC_REG_TXCSR(USBC0_BASE))) {
      _dcd.dpb_tx_waits++;
      return false;
    }
    pipe->buf = NULL;
    _dcd.in_armed &= (uint16_t)~TU_BIT(epnum_minus1);
    return true;
  }

  /* Never load the FIFO while TXPKTRDY is set (both DPB buffers full, or
   * the single buffer still pending): the write would land on top of a
   * packet waiting to go out. Linux's txstate() has the same guard. The
   * interrupt that frees the buffer brings us back. */
  if (dma_ch[DMA_CHAN_TX].active && dma_ch[DMA_CHAN_TX].ep_addr == ep_addr) return false;
  if (USBC_REG_test_bit_w(USBC_BP_TXCSR_D_TX_READY, USBC_REG_TXCSR(USBC0_BASE))) {
    _dcd.tx_busy_irqs++;
    return false;
  }
  const unsigned mps = USBC_Readw(USBC_REG_TXMAXP(USBC0_BASE));
  const unsigned len = TU_MIN(mps, rem);
  uint8_t          *buf = pipe->buf;
  // TU_LOG1("   %p mps %d len %d rem %d\r\n", buf, mps, len, rem);
  if (len) {
    volatile void* addr = (volatile void*)(USBC_REG_EPFIFO1(USBC0_BASE) + (epnum_minus1 << 2));
    if (_dcd.pipe_buf_is_fifo[TUSB_DIR_IN] & TU_BIT(epnum_minus1)) {
      pipe_read_write_packet_ff((tu_fifo_t *)(uintptr_t) buf, addr, len, TUSB_DIR_IN);
    } else {
      uint32_t t0 = time_us();
      if (epnum_minus1 == 0 && len >= 256) gap_note(gap_tx, &gap_tx_last, t0, rem == pipe->length);
      pipe_write_packet(buf, addr, len);
      if (len >= 256) { _dcd.fifo_wr_us += time_us() - t0; _dcd.fifo_wr_n++; }
      pipe->buf       = buf + len;
    }
    pipe->remaining = rem - len;
  }
  __USBC_Dev_Tx_WriteDataComplete();
  // TU_LOG1(" TXCSRL%d = %x %d\r\n", epnum_minus1 + 1, regs->TXCSRL, rem - len);
  /* H3 DPB: TXPKTRDY drops at once when the other buffer is free; fill it
   * now instead of waiting for the interrupt that announces the same. */
  if ((_dcd.dpb_in & TU_BIT(epnum_minus1)) && pipe->remaining &&
      !USBC_REG_test_bit_w(USBC_BP_TXCSR_D_TX_READY, USBC_REG_TXCSR(USBC0_BASE))) {
    _dcd.dpb_tx_early++;
    return handle_xfer_in(ep_addr);
  }
  return false;
}


/* H3: per-packet progress of an OUT transfer, for cut-through forwarding
 * (gadget_slots.c). ISR context. */
TU_ATTR_WEAK void dcd_h3_out_progress(uint8_t ep_addr, uint32_t received) { (void)ep_addr; (void)received; }

static bool handle_xfer_out(uint_fast8_t ep_addr)
{
  unsigned epnum_minus1 = tu_edpt_number(ep_addr) - 1;
  pipe_state_t  *pipe = &_dcd.pipe[tu_edpt_dir(ep_addr)][epnum_minus1];
  // TU_LOG1(" RXCSRL%d = %x\r\n", epnum_minus1 + 1, regs->RXCSRL);

  TU_ASSERT(__USBC_Dev_Rx_IsReadDataReady());
  /* Nothing armed: the packet waits in the FIFO for the next transfer. A
   * drain loop that read a packet takes its latched interrupt here too. */
  if (!pipe->buf) return false;

  const unsigned mps = USBC_Readw(USBC_REG_RXMAXP(USBC0_BASE));
  if (dma_ch[DMA_CHAN_RX].active && dma_ch[DMA_CHAN_RX].ep_addr == ep_addr) {
    /* DMA mode 1: an endpoint interrupt means a short packet (the DMA does
     * not take it) -- or a full one it has not fetched yet: leave that. */
    if (USBC_Readw(USBC_REG_RXCOUNT(USBC0_BASE)) >= mps) return false;
    uint32_t moved = dma_stop(DMA_CHAN_RX);
    /* as in handle_dma_done: drop lines prefetched while the DMA wrote them */
    if (moved) dcache_invalidate_range(pipe->buf, moved);
    rx_dma_csr_off();
    dma_rx_short++;
    pipe->buf = (uint8_t *)pipe->buf + moved;
    pipe->remaining = (uint16_t)(pipe->remaining - (moved > pipe->remaining ? pipe->remaining : moved));
  }
  const unsigned rem = pipe->remaining;
  const unsigned vld = USBC_Readw(USBC_REG_RXCOUNT(USBC0_BASE));
  const unsigned len = TU_MIN(TU_MIN(rem, mps), vld);
  uint8_t          *buf = pipe->buf;
  if (len) {
    volatile void* addr = (volatile void*)(USBC_REG_EPFIFO1(USBC0_BASE) + (epnum_minus1 << 2));
    if (_dcd.pipe_buf_is_fifo[TUSB_DIR_OUT] & TU_BIT(epnum_minus1)) {
      pipe_read_write_packet_ff((tu_fifo_t *)(uintptr_t )buf, addr, len, TUSB_DIR_OUT);
    } else {
      uint32_t t0 = time_us();
      if (epnum_minus1 == 0 && len >= 256) gap_note(gap_rx, &gap_rx_last, t0, rem == pipe->length);
      pipe_read_packet(buf, addr, len);
      if (len >= 256) { _dcd.fifo_rd_us += time_us() - t0; _dcd.fifo_rd_n++; }
      pipe->buf       = buf + len;
    }
    pipe->remaining = rem - len;
    if (!(_dcd.pipe_buf_is_fifo[TUSB_DIR_OUT] & TU_BIT(epnum_minus1)))
      dcd_h3_out_progress(ep_addr, pipe->length - pipe->remaining);
  }
  if ((len < mps) || (rem == len)) {
    /* the packet stays in the FIFO as back-pressure until the next arm */
    pipe->buf = NULL;
    _dcd.rx_held |= (uint16_t)TU_BIT(epnum_minus1);
    return NULL != buf;
  }
  __USBC_Dev_Rx_Release();
  return false;
}

static bool edpt_n_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes)
{
  (void)rhport;

  unsigned epnum_minus1 = tu_edpt_number(ep_addr) - 1;
  unsigned dir_in       = tu_edpt_dir(ep_addr);

  pipe_state_t *pipe = &_dcd.pipe[dir_in][epnum_minus1];
  pipe->buf          = buffer;
  pipe->length       = total_bytes;
  pipe->remaining    = total_bytes;

  USBC_SelectActiveEp(tu_edpt_number(ep_addr));

  if (dir_in) {
    _dcd.in_armed |= (uint16_t)TU_BIT(epnum_minus1);
    if (total_bytes == 0)
      _dcd.in_zlp_pending |= (uint16_t)TU_BIT(epnum_minus1);
    if ((dma_mask & 2) && epnum_minus1 == 0 && total_bytes >= DMA_MIN_BYTES && !dma_ch[DMA_CHAN_TX].active &&
        USBC_Readw(USBC_REG_TXMAXP(USBC0_BASE)) == 512) {
      uint32_t n = total_bytes & ~511u;
      dcache_clean_range(buffer, n);
      tx_dma_csr_on();
      dma_start(DMA_CHAN_TX, ep_addr, buffer, n, true);
      dma_tx_n++;
      return true;
    }
    handle_xfer_in(ep_addr);
  } else {
    bool held = _dcd.rx_held & TU_BIT(epnum_minus1);
    _dcd.rx_held &= (uint16_t)~TU_BIT(epnum_minus1);
    if (__USBC_Dev_Rx_IsReadDataReady()) {
      bool completed = false;
      if (held) {
        /* Release the packet the previous transfer left as back-pressure.
         * With DPB the next one may be waiting behind it: take it now, its
         * interrupt came and went while nothing was armed. */
        __USBC_Dev_Rx_Release();
      } else {
        /* Nobody read this one: it came while nothing was armed (a DMA
         * transfer leaves none held, AUTOCLEAR frees every packet the DMA
         * read). It is this transfer's first. */
        completed = handle_xfer_out(ep_addr);
      }
      while (!completed && pipe->buf && rx_more) {
        _dcd.dpb_rx_extra++;
        completed = handle_xfer_out(ep_addr);
      }
      if (completed) {
        dcd_event_xfer_complete(rhport, ep_addr, pipe->length - pipe->remaining, XFER_RESULT_SUCCESS, false);
        return true;
      }
    }
    if ((dma_mask & 1) && epnum_minus1 == 0 && pipe->buf && (pipe->remaining & ~511u) >= DMA_MIN_BYTES &&
        USBC_Readw(USBC_REG_RXMAXP(USBC0_BASE)) == 512 &&
        !dma_ch[DMA_CHAN_RX].active) {
      uint32_t n = pipe->remaining & ~511u;
      dcache_invalidate_range(pipe->buf, n);
      rx_dma_csr_on();
      dma_start(DMA_CHAN_RX, ep_addr, pipe->buf, n, false);
      dma_rx_n++;
    }
  }
  return true;
}

static bool edpt0_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t *buffer, uint16_t total_bytes)
{
  (void)rhport;
  TU_ASSERT(total_bytes <= 64); /* Current implementation supports for only up to 64 bytes. */

  const unsigned req = _dcd.setup_packet.bmRequestType;
  TU_ASSERT(req != REQUEST_TYPE_INVALID || total_bytes == 0);

  USBC_SelectActiveEp(0);

  if (req == REQUEST_TYPE_INVALID || _dcd.status_out) {
    /* STATUS OUT stage.
     * MUSB controller automatically handles STATUS OUT packets without
     * software helps. We do not have to do anything. And STATUS stage
     * may have already finished and received the next setup packet
     * without calling this function, so we have no choice but to
     * invoke the callback function of status packet here. */
    // TU_LOG1(" STATUS OUT CSRL0 = %x\r\n", CSRL0);
    _dcd.status_out = 0;
    if (req == REQUEST_TYPE_INVALID) {
      dcd_event_xfer_complete(rhport, ep_addr, total_bytes, XFER_RESULT_SUCCESS, false);
    } else {
      /* The next setup packet has already been received, it aborts
       * invoking callback function to avoid confusing TUSB stack. */
      TU_LOG1("Drop CONTROL_STAGE_ACK\r\n");
    }
    return true;
  }
  const unsigned dir_in = tu_edpt_dir(ep_addr);
  if (tu_edpt_dir(req) == dir_in) { /* DATA stage */
    TU_ASSERT(total_bytes <= _dcd.remaining_ctrl);
    const unsigned rem = _dcd.remaining_ctrl;
    const unsigned len = TU_MIN(TU_MIN(rem, 64), total_bytes);
    if (dir_in) {
      pipe_write_packet(buffer, (volatile void*) USBC_REG_EPFIFO0(USBC0_BASE), len);

      _dcd.pipe0.buf       = buffer + len;
      _dcd.pipe0.length    = len;
      _dcd.pipe0.remaining = 0;

      _dcd.remaining_ctrl  = rem - len;
      if ((len < 64) || (rem == len)) {
        _dcd.setup_packet.bmRequestType = REQUEST_TYPE_INVALID; /* Change to STATUS/SETUP stage */
        _dcd.status_out = 1;
        /* Flush TX FIFO and reverse the transaction direction. */
        __USBC_Dev_ep0_WriteDataComplete();
      } else {
        __USBC_Dev_ep0_WriteDataHalf();
      }
      // TU_LOG1(" IN CSRL0 = %x\r\n", CSRL0);
    } else {
      // TU_LOG1(" OUT CSRL0 = %x\r\n", CSRL0);
      _dcd.pipe0.buf       = buffer;
      _dcd.pipe0.length    = len;
      _dcd.pipe0.remaining = len;
      __USBC_Dev_ep0_ReadDataHalf();
    }
  } else if (dir_in) {
    // TU_LOG1(" STATUS IN CSRL0 = %x\r\n", CSRL0);
    _dcd.pipe0.buf = NULL;
    _dcd.pipe0.length    = 0;
    _dcd.pipe0.remaining = 0;
    /* Clear RX FIFO and reverse the transaction direction */
    __USBC_Dev_ep0_ReadDataComplete();
  }
  return true;
}

static void process_ep0(uint8_t rhport)
{
  USBC_SelectActiveEp(0);
  uint_fast8_t csrl = USBC_Readw(USBC_REG_CSR0(USBC0_BASE));

  // TU_LOG1(" EP0 CSRL0 = %x\r\n", csrl);

  if (csrl & USB_CSRL0_STALLED) {
    /* Returned STALL packet to HOST. */
    __USBC_Dev_ep0_ClearStall();
    return;
  }

  unsigned req = _dcd.setup_packet.bmRequestType;
  if (csrl & USB_CSRL0_SETEND) {
    // TU_LOG1("   ABORT by the next packets\r\n");
    USBC_Dev_Ctrl_ClearSetupEnd();
    if (req != REQUEST_TYPE_INVALID && _dcd.pipe0.buf) {
      /* DATA stage was aborted by receiving STATUS or SETUP packet. */
      _dcd.pipe0.buf = NULL;
      _dcd.setup_packet.bmRequestType = REQUEST_TYPE_INVALID;
      dcd_event_xfer_complete(rhport,
                              req & TUSB_DIR_IN_MASK,
                              _dcd.pipe0.length - _dcd.pipe0.remaining,
                              XFER_RESULT_SUCCESS, true);
    }
    req = REQUEST_TYPE_INVALID;
    if (!(csrl & USB_CSRL0_RXRDY)) return; /* Received SETUP packet */
  }

  if (csrl & USB_CSRL0_RXRDY) {
    /* Received SETUP or DATA OUT packet */
    if (req == REQUEST_TYPE_INVALID) {
      /* SETUP */
      TU_ASSERT(sizeof(tusb_control_request_t) == USBC_Readw(USBC_REG_COUNT0(USBC0_BASE)),);
      process_setup_packet(rhport);
      return;
    }
    if (_dcd.pipe0.buf) {
      /* DATA OUT */
      const unsigned vld = USBC_Readw(USBC_REG_COUNT0(USBC0_BASE));
      const unsigned rem = _dcd.pipe0.remaining;
      const unsigned len = TU_MIN(TU_MIN(rem, 64), vld);
      pipe_read_packet(_dcd.pipe0.buf, (volatile void*)USBC_REG_EPFIFO0(USBC0_BASE), len);

      _dcd.pipe0.remaining = rem - len;
      _dcd.remaining_ctrl -= len;

      _dcd.pipe0.buf = NULL;
      dcd_event_xfer_complete(rhport,
                              tu_edpt_addr(0, TUSB_DIR_OUT),
                              _dcd.pipe0.length - _dcd.pipe0.remaining,
                              XFER_RESULT_SUCCESS, true);
    }
    return;
  }

  /* When CSRL0 is zero, it means that completion of sending a any length packet
   * or receiving a zero length packet. */
  if (req != REQUEST_TYPE_INVALID && !tu_edpt_dir(req)) {
    /* STATUS IN */
    if (*(const uint16_t*)(uintptr_t)&_dcd.setup_packet == 0x0500) {
      /* The address must be changed on completion of the control transfer. */
	  USBC_Dev_SetAddress((uint8_t)_dcd.setup_packet.wValue);
    }
    _dcd.setup_packet.bmRequestType = REQUEST_TYPE_INVALID;
    dcd_event_xfer_complete(rhport,
                            tu_edpt_addr(0, TUSB_DIR_IN),
                            _dcd.pipe0.length - _dcd.pipe0.remaining,
                            XFER_RESULT_SUCCESS, true);
    return;
  }
  if (_dcd.pipe0.buf) {
    /* DATA IN */
    _dcd.pipe0.buf = NULL;
    dcd_event_xfer_complete(rhport,
                            tu_edpt_addr(0, TUSB_DIR_IN),
                            _dcd.pipe0.length - _dcd.pipe0.remaining,
                            XFER_RESULT_SUCCESS, true);
  }
}

static void process_edpt_n(uint8_t rhport, uint_fast8_t ep_addr)
{
  bool completed;
  const unsigned dir_in     = tu_edpt_dir(ep_addr);
  const unsigned epn        = tu_edpt_number(ep_addr);

  USBC_SelectActiveEp(epn);

  if (dir_in) {
    // TU_LOG1(" TXCSRL%d = %x\r\n", epn_minus1 + 1, regs->TXCSRL);
    if (__USBC_Dev_Tx_IsEpStall()) {
      __USBC_Dev_Tx_AckSentStall();
      return;
    }
    completed = handle_xfer_in(ep_addr);
  } else {
    // TU_LOG1(" RXCSRL%d = %x\r\n", epn_minus1 + 1, regs->RXCSRL);
    if (__USBC_Dev_Rx_IsEpStall()) {
      __USBC_Dev_Rx_AckSentStall();
      return;
    }
    /* H3 DPB: the other buffer may hold the next packet already and its
     * interrupt was folded into this one; drain while RXPKTRDY re-asserts. */
    completed = handle_xfer_out(ep_addr);
    /* Only while a transfer is armed: with none, RXPKTRDY stays set on
     * purpose (back-pressure) and this loop would never end. */
    while (!completed && _dcd.pipe[TUSB_DIR_OUT][epn - 1].buf && rx_more) {
      _dcd.dpb_rx_extra++;
      completed = handle_xfer_out(ep_addr);
    }
  }

  if (completed) {
    pipe_state_t *pipe = &_dcd.pipe[dir_in][tu_edpt_number(ep_addr) - 1];
    dcd_event_xfer_complete(rhport, ep_addr,
                            pipe->length - pipe->remaining,
                            XFER_RESULT_SUCCESS, true);
  }
}

static void process_bus_reset(uint8_t rhport)
{
  /* When bmRequestType is REQUEST_TYPE_INVALID(0xFF),
   * a control transfer state is SETUP or STATUS stage. */
  _dcd.setup_packet.bmRequestType = REQUEST_TYPE_INVALID;
  _dcd.status_out = 0;
  /* When pipe0.buf has not NULL, DATA stage works in progress. */
  _dcd.pipe0.buf = NULL;
  /* H3: TinyUSB closes endpoints only on SET_CONFIGURATION, not here, so an
   * armed inner DMA would survive the reset: still `active`, still aimed
   * at the old buffer, and blocking the next arm (Codex review). */
  for (unsigned ch = 0; ch < 2; ch++) if (dma_ch[ch].active) dma_stop(ch);
  memset(_dcd.pipe, 0, sizeof _dcd.pipe);
  _dcd.in_zlp_pending = 0;
  _dcd.rx_held = 0;
  _dcd.in_armed = 0;

  USBC_Writew(1, USBC_REG_INTTxE(USBC0_BASE)); /* Enable only EP0 */
  USBC_Writew(0, USBC_REG_INTRxE(USBC0_BASE));

  dcd_event_bus_reset(rhport, USBC_Dev_QueryTransferMode(), true);
}

/*------------------------------------------------------------------
 * Device API
 *------------------------------------------------------------------*/

void dcd_h3_dpb_stats(uint32_t *tx_waits, uint32_t *tx_early, uint32_t *rx_extra, uint32_t *tx_busy)
{
  *tx_waits = _dcd.dpb_tx_waits; *tx_early = _dcd.dpb_tx_early; *rx_extra = _dcd.dpb_rx_extra; *tx_busy = _dcd.tx_busy_irqs;
}
/* mean us per >=256-byte packet copied from / to the FIFO by PIO */
void dcd_h3_fifo_stats(uint32_t *rd_us_x10, uint32_t *rd_n, uint32_t *wr_us_x10, uint32_t *wr_n)
{
  *rd_n = _dcd.fifo_rd_n; *wr_n = _dcd.fifo_wr_n;
  *rd_us_x10 = _dcd.fifo_rd_n ? _dcd.fifo_rd_us * 10 / _dcd.fifo_rd_n : 0;
  *wr_us_x10 = _dcd.fifo_wr_n ? _dcd.fifo_wr_us * 10 / _dcd.fifo_wr_n : 0;
}

static void usb_isr_handler(void) {
	dcd_int_handler(0);
}

bool dcd_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  (void) rh_init;

  /* H3: clocks/resets first; the OTG block is bus-gated after a Falcon boot
   * and a register access before USBC_HardwareReset() would abort. */
  USBC_HardwareReset();
  dcd_disconnect(rhport);
  USBC_EnableDpDmPullUp();
  USBC_ForceIdToHigh(); // Force device mode
  USBC_ForceVbusValidToHigh();
  USBC_SelectBus(USBC_IO_TYPE_PIO, 0, 0);
  dcd_edpt_close_all(rhport);

  #if TUD_OPT_HIGH_SPEED
    USBC_REG_set_bit_b(USBC_BP_POWER_D_HIGH_SPEED_EN, USBC_REG_PCTL(USBC0_BASE));
  #else
    USBC_REG_clear_bit_b(USBC_BP_POWER_D_HIGH_SPEED_EN, USBC_REG_PCTL(USBC0_BASE));
  #endif

  /* No SOF interrupt: at high speed it is 8000 wakeups/s for nothing (the
   * progress gate reads FRNUM itself); console `sofirq 1` turns it on. */
  USBC_Writeb((1 << USBC_BP_INTUSBE_EN_SUSPEND)
    | (1 << USBC_BP_INTUSBE_EN_RESUME)
    | (1 << USBC_BP_INTUSBE_EN_RESET)
    | (1 << USBC_BP_INTUSBE_EN_DISCONNECT)
    , USBC_REG_INTUSBE(USBC0_BASE));
  gic_set_handler(IRQ_MUSB, usb_isr_handler);

  dcd_connect(rhport);

  return true;
}

// Connect by enabling internal pull-up resistor on D+/D-
void dcd_connect(uint8_t rhport)
{
  (void)rhport;
  USBC_REG_set_bit_b(USBC_BP_POWER_D_SOFT_CONNECT, USBC_REG_PCTL(USBC0_BASE));
}

// Disconnect by disabling internal pull-up resistor on D+/D-
void dcd_disconnect(uint8_t rhport)
{
  (void)rhport;
  USBC_REG_clear_bit_b(USBC_BP_POWER_D_SOFT_CONNECT, USBC_REG_PCTL(USBC0_BASE));
}

void dcd_sof_enable(uint8_t rhport, bool en)
{
  (void) rhport;
  (void) en;

  // TODO implement later
}

void dcd_int_enable(uint8_t rhport)
{
  (void)rhport;
  gic_unmask(IRQ_MUSB);
}

static void musb_int_mask(void)
{
  gic_mask(IRQ_MUSB);
}

void dcd_int_disable(uint8_t rhport)
{
  (void)rhport;
  gic_mask(IRQ_MUSB);
}

static void musb_int_unmask(void)
{
  gic_unmask(IRQ_MUSB);
}

// Receive Set Address request, mcu port must also include status IN response
void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
  (void)rhport;
  (void)dev_addr;
  _dcd.pipe0.buf       = NULL;
  _dcd.pipe0.length    = 0;
  _dcd.pipe0.remaining = 0;
  /* Clear RX FIFO to return ACK. */
  USBC_SelectActiveEp(0);
  __USBC_Dev_ep0_ReadDataComplete();
}

// Wake up host
void dcd_remote_wakeup(uint8_t rhport)
{
  (void)rhport;
  USBC_REG_set_bit_b(USBC_BP_POWER_D_RESUME, USBC_REG_PCTL(USBC0_BASE));
  tusb_time_delay_ms_api(10);
  USBC_REG_clear_bit_b(USBC_BP_POWER_D_RESUME, USBC_REG_PCTL(USBC0_BASE));
}

//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

#ifndef __ARMCC_VERSION
#define __clz __builtin_clz
#endif

// Configure endpoint's registers according to descriptor
bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * ep_desc)
{
  (void) rhport;

  uint16_t reg_val;

  const unsigned ep_addr = ep_desc->bEndpointAddress;
  const unsigned epn     = tu_edpt_number(ep_addr);
  const unsigned dir_in  = tu_edpt_dir(ep_addr);
  const unsigned xfer    = ep_desc->bmAttributes.xfer;
  const unsigned mps     = tu_edpt_packet_size(ep_desc);

  TU_ASSERT(epn < TUP_DCD_ENDPOINT_MAX);

  pipe_state_t *pipe = &_dcd.pipe[dir_in][epn - 1];
  pipe->buf       = NULL;
  pipe->length    = 0;
  pipe->remaining = 0;

  musb_int_mask();

  // volatile hw_endpoint_t *regs = edpt_regs(epn - 1);
  USBC_SelectActiveEp(epn);
  if (dir_in) {
    USBC_Writew(mps, USBC_REG_TXMAXP(USBC0_BASE));

    reg_val = (1 << USBC_BP_TXCSR_D_MODE)
      | (1 << USBC_BP_TXCSR_D_FLUSH_FIFO)
      | (1 << USBC_BP_TXCSR_D_CLEAR_DATA_TOGGLE);
    if  (xfer == TUSB_XFER_ISOCHRONOUS)
      reg_val |= (1 << USBC_BP_TXCSR_D_ISO);
	  USBC_Writew(reg_val, USBC_REG_TXCSR(USBC0_BASE));

    USBC_INT_EnableTxEp(epn);
  } else {
    USBC_Writew(mps, USBC_REG_RXMAXP(USBC0_BASE));

    _dcd.rx_held &= (uint16_t)~TU_BIT(epn - 1);
    reg_val = (1 << USBC_BP_RXCSR_D_FLUSH_FIFO)
      | (1 << USBC_BP_RXCSR_D_CLEAR_DATA_TOGGLE);
    if  (xfer == TUSB_XFER_ISOCHRONOUS)
      reg_val |= (1 << USBC_BP_RXCSR_D_ISO);
    USBC_Writew(reg_val, USBC_REG_RXCSR(USBC0_BASE));

    USBC_INT_EnableRxEp(epn);
  }

  /* Setup FIFO. The adb pair (endpoint 1, the datapath) gets double packet
   * buffering: the host can land/fetch the next packet while the CPU moves
   * the previous one by PIO. 8 KiB of FIFO RAM: 64 (ep0) + 2*1024 (adb) +
   * 3*2*512 (the rest) fits; doubling every endpoint would not. */
  /* Static, size-aligned FIFO map instead of upstream's best-fit free list.
   * Byte offsets; code = log2(size)-3; DPB doubles the allocation. */
  static const struct { uint16_t addr; uint8_t code; uint8_t dpb; } fifo_map[TUP_DCD_ENDPOINT_MAX][2] = {
    /* [ep][0=OUT/RX, 1=IN/TX] */
    { {0, 3, 0},      {0, 3, 0}    },   /* ep0: fixed 64 */
    /* ep1 adb: RX double-buffered, TX single. PIO loads into a
     * double-buffered TX FIFO lost their first 16 words about once per
     * 2000 packets while the RX side was busy (concurrent push+pull): the
     * packet went out with the other buffer's old first 64 bytes. Same
     * with and without the early second-buffer fill below, never with the
     * inner DMA loading the FIFO, never single-buffered (20/20 soak runs).
     * Linux keeps the sunxi FIFOs single-buffered too. DPB gave no
     * throughput anyway (M3). */
    { {2048, 6, 1},   {1024, 6, 0} },
    { {3584, 6, 0},   {3072, 6, 0} },   /* ep2 fastboot */
    { {0, 0, 0},      {64, 3, 0}   },   /* ep3 CDC notify (IN only, 64) */
    { {4096, 6, 0},   {512, 6, 0}  },   /* ep4 CDC data */
  };
  TU_ASSERT(mps <= (8u << fifo_map[epn][dir_in].code));
  bool dpb = fifo_map[epn][dir_in].dpb;
  unsigned addr = fifo_map[epn][dir_in].addr / 8;
  TU_ASSERT(addr);
  uint8_t fifosz = (uint8_t)(fifo_map[epn][dir_in].code | (dpb ? USB_TXFIFOSZ_DPB : 0));

  if (dir_in) {
    USBC_Writew(addr, USBC_REG_TXFIFOAD(USBC0_BASE));
    USBC_Writeb(fifosz, USBC_REG_TXFIFOSZ(USBC0_BASE));
    if (dpb) _dcd.dpb_in |= TU_BIT(epn - 1); else _dcd.dpb_in &= (uint16_t)~TU_BIT(epn - 1);
  } else {
    USBC_Writew(addr, USBC_REG_RXFIFOAD(USBC0_BASE));
    USBC_Writeb(fifosz, USBC_REG_RXFIFOSZ(USBC0_BASE));
  }

  musb_int_unmask();

  return true;
}

void dcd_edpt_close_all(uint8_t rhport)
{
  (void) rhport;
  musb_int_mask();
  for (unsigned ch = 0; ch < 2; ch++) if (dma_ch[ch].active) dma_stop(ch);
  USBC_Writew(1, USBC_REG_INTTxE(USBC0_BASE)); /* Enable only EP0 */
  USBC_Writew(0, USBC_REG_INTRxE(USBC0_BASE));
  _dcd.rx_held = 0;
  _dcd.in_armed = 0;
  for (unsigned i = 1; i < TUP_DCD_ENDPOINT_MAX; ++i) {
    USBC_SelectActiveEp(i);
    USBC_Writew(0, USBC_REG_TXMAXP(USBC0_BASE));
		USBC_Writew((1 << USBC_BP_TXCSR_D_MODE) | (1 << USBC_BP_TXCSR_D_CLEAR_DATA_TOGGLE) | (1 << USBC_BP_TXCSR_D_FLUSH_FIFO),
      USBC_REG_TXCSR(USBC0_BASE));

    USBC_Writew(0, USBC_REG_RXMAXP(USBC0_BASE));
	  USBC_Writew((1 << USBC_BP_RXCSR_D_CLEAR_DATA_TOGGLE) | (1 << USBC_BP_RXCSR_D_FLUSH_FIFO),
      USBC_REG_RXCSR(USBC0_BASE));

    USBC_Writew(0, USBC_REG_TXFIFOAD(USBC0_BASE));
    USBC_Writeb(0, USBC_REG_TXFIFOSZ(USBC0_BASE));
    USBC_Writew(0, USBC_REG_RXFIFOAD(USBC0_BASE));
    USBC_Writeb(0, USBC_REG_RXFIFOSZ(USBC0_BASE));
  }
  musb_int_unmask();
}

void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr)
{
  (void)rhport;
  unsigned const epn    = tu_edpt_number(ep_addr);
  unsigned const dir_in = tu_edpt_dir(ep_addr);

  musb_int_mask();
  USBC_SelectActiveEp(epn);
  if (dir_in) {
    USBC_INT_DisableTxEp(epn);
    _dcd.in_armed &= (uint16_t)~TU_BIT(epn - 1);
    USBC_Writew(0, USBC_REG_TXMAXP(USBC0_BASE));
		USBC_Writew((1 << USBC_BP_TXCSR_D_MODE) | (1 << USBC_BP_TXCSR_D_CLEAR_DATA_TOGGLE) | (1 << USBC_BP_TXCSR_D_FLUSH_FIFO),
      USBC_REG_TXCSR(USBC0_BASE));

    USBC_Writew(0, USBC_REG_TXFIFOAD(USBC0_BASE));
    USBC_Writeb(0, USBC_REG_TXFIFOSZ(USBC0_BASE));
  } else {
    USBC_INT_DisableRxEp(epn);
    _dcd.rx_held &= (uint16_t)~TU_BIT(epn - 1);
    USBC_Writew(0, USBC_REG_RXMAXP(USBC0_BASE));
	  USBC_Writew((1 << USBC_BP_RXCSR_D_CLEAR_DATA_TOGGLE) | (1 << USBC_BP_RXCSR_D_FLUSH_FIFO),
      USBC_REG_RXCSR(USBC0_BASE));

    USBC_Writew(0, USBC_REG_RXFIFOAD(USBC0_BASE));
    USBC_Writeb(0, USBC_REG_RXFIFOSZ(USBC0_BASE));
  }
  musb_int_unmask();
}

  #if 0
bool dcd_edpt_iso_alloc(uint8_t rhport, uint8_t ep_addr, uint16_t largest_packet_size) {
  (void)rhport;
  (void)ep_addr;
  (void)largest_packet_size;
  return false;
}

bool dcd_edpt_iso_activate(uint8_t rhport, const tusb_desc_endpoint_t *desc_ep) {
  (void)rhport;
  (void)desc_ep;
  return false;
}
  #endif

// Submit a transfer, When complete dcd_event_xfer_complete() is invoked to notify the stack
bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes, bool is_isr)
{
  (void) is_isr;
  (void)rhport;
  bool ret;
  // TU_LOG1("X %x %d\r\n", ep_addr, total_bytes);
  unsigned const epnum = tu_edpt_number(ep_addr);
  musb_int_mask();

  if (epnum) {
    _dcd.pipe_buf_is_fifo[tu_edpt_dir(ep_addr)] &= ~TU_BIT(epnum - 1);
    ret = edpt_n_xfer(rhport, ep_addr, buffer, total_bytes);
  } else {
    ret = edpt0_xfer(rhport, ep_addr, buffer, total_bytes);
  }
  musb_int_unmask();
  return ret;
}

// Submit a transfer where is managed by FIFO, When complete dcd_event_xfer_complete() is invoked to notify the stack - optional, however, must be listed in usbd.c
bool dcd_edpt_xfer_fifo(uint8_t rhport, uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes, bool is_isr)
{
  (void) is_isr;
  (void)rhport;
  bool ret;
  // TU_LOG1("X %x %d\r\n", ep_addr, total_bytes);
  unsigned const epnum = tu_edpt_number(ep_addr);
  TU_ASSERT(epnum);

  musb_int_mask();
  _dcd.pipe_buf_is_fifo[tu_edpt_dir(ep_addr)] |= TU_BIT(epnum - 1);
  ret = edpt_n_xfer(rhport, ep_addr, (uint8_t*)ff, total_bytes);
  musb_int_unmask();

  return ret;
}

// Stall endpoint
/* A halt ends the IN transfer in progress (TinyUSB drops it on the clear,
 * with no completion): stop its DMA and flush what it loaded, so nothing
 * of it goes out once the halt is cleared. */
static void in_cancel(uint8_t ep_addr)
{
  unsigned epnum_minus1 = tu_edpt_number(ep_addr) - 1;
  pipe_state_t *pipe = &_dcd.pipe[TUSB_DIR_IN][epnum_minus1];
  if (dma_ch[DMA_CHAN_TX].active && dma_ch[DMA_CHAN_TX].ep_addr == ep_addr) {
    dma_stop(DMA_CHAN_TX);
    tx_dma_csr_off();
  }
  if (USBC_REG_test_bit_w(USBC_BP_TXCSR_D_FIFO_NOT_EMPTY, USBC_REG_TXCSR(USBC0_BASE)))
    USBC_Writeb((1 << USBC_BP_TXCSR_D_FLUSH_FIFO), USBC_REG_TXCSR(USBC0_BASE));
  pipe->buf = NULL; pipe->length = pipe->remaining = 0;
  _dcd.in_armed &= (uint16_t)~TU_BIT(epnum_minus1);
  _dcd.in_zlp_pending &= (uint16_t)~TU_BIT(epnum_minus1);
}

void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void)rhport;
  unsigned const epn = tu_edpt_number(ep_addr);
  musb_int_mask();
  USBC_SelectActiveEp(epn);
  if (0 == epn) {
    if (!ep_addr) { /* Ignore EP80 */
      _dcd.setup_packet.bmRequestType = REQUEST_TYPE_INVALID;
      _dcd.pipe0.buf = NULL;
      __USBC_Dev_ep0_SendStall();
    }
  } else {
    if (tu_edpt_dir(ep_addr)) { /* IN */
      in_cancel(ep_addr);
      __USBC_Dev_Tx_SendStall();
    } else { /* OUT */
      TU_ASSERT(!__USBC_Dev_Rx_IsReadDataReady(),);
      __USBC_Dev_Rx_SendStall();
    }
  }
  musb_int_unmask();
}

// clear stall, data toggle is also reset to DATA0
void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
  (void)rhport;
  unsigned const epn = tu_edpt_number(ep_addr);
  musb_int_mask();
  USBC_SelectActiveEp(epn);
  if (0 != epn) {
    if (tu_edpt_dir(ep_addr)) { /* IN */
      __USBC_Dev_Tx_ClearStall();
      USBC_REG_set_bit_w(USBC_BP_TXCSR_D_CLEAR_DATA_TOGGLE, USBC_REG_TXCSR(USBC0_BASE));
    } else { /* OUT */
      __USBC_Dev_Rx_ClearStall();
      USBC_REG_set_bit_w(USBC_BP_RXCSR_D_CLEAR_DATA_TOGGLE, USBC_REG_RXCSR(USBC0_BASE));
    }
  }
  musb_int_unmask();
}


void dcd_int_handler(uint8_t rhport)
{
  uint8_t is;
  uint16_t txis, rxis;

  is   = USBC_Readb(USBC_REG_INTUSB(USBC0_BASE));   /* read interrupt status */
  txis = USBC_Readw(USBC_REG_INTTx(USBC0_BASE)); /* read interrupt status */
  rxis = USBC_Readw(USBC_REG_INTRx(USBC0_BASE)); /* read interrupt status */

  /* H3 inner DMA channels (adb pair) */
  uint32_t dis = USBC_Readw(DMA_INTS);
  for (unsigned ch = 0; ch < 2 && dis; ch++)
    if ((dis & (1u << ch)) && dma_ch[ch].active)
      handle_dma_done(rhport, ch);
  if (dis & ~3u) USBC_Writel(dis & ~3u, DMA_INTS);

  is &= USBC_Readb(USBC_REG_INTUSBE(USBC0_BASE)); /* ignore disabled interrupts */
  USBC_Writeb(is, USBC_REG_INTUSB(USBC0_BASE)); /* sunxi musb requires a write to interrupt register to clear */
  if (is & USBC_INTUSB_DISCONNECT) {
	dcd_event_bus_signal(rhport, DCD_EVENT_UNPLUGGED, true);
  }
  if (is & USBC_INTUSB_SOF) {
    dcd_event_bus_signal(rhport, DCD_EVENT_SOF, true);
  }
  if (is & USBC_INTUSB_RESET) {
    /* ep0 FADDR must be 0 when (re)entering peripheral mode */
    USBC_SelectActiveEp(0);
    USBC_Dev_SetAddress(0);
    process_bus_reset(rhport);
  }
  if (is & USBC_INTUSB_RESUME) {
    dcd_event_bus_signal(rhport, DCD_EVENT_RESUME, true);
  }
  if (is & USBC_INTUSB_SUSPEND) {
    dcd_event_bus_signal(rhport, DCD_EVENT_SUSPEND, true);
  }

  txis &= USBC_Readw(USBC_REG_INTTxE(USBC0_BASE));
  USBC_Writew(txis, USBC_REG_INTTx(USBC0_BASE));
  if (txis & USBC_INTTx_FLAG_EP0) {
    process_ep0(rhport);
    txis &= ~TU_BIT(0);
  }
  while (txis) {
    unsigned const num = __builtin_ctz(txis);
    process_edpt_n(rhport, tu_edpt_addr(num, TUSB_DIR_IN));
    txis &= ~TU_BIT(num);
  }

  rxis &= USBC_Readw(USBC_REG_INTRxE(USBC0_BASE));
  USBC_Writew(rxis, USBC_REG_INTRx(USBC0_BASE));
  while (rxis) {
    unsigned const num = __builtin_ctz(rxis);
    process_edpt_n(rhport, tu_edpt_addr(num, TUSB_DIR_OUT));
    rxis &= ~TU_BIT(num);
  }
}
#endif

void dcd_h3_sof_irq(bool on)
{
  if (on) USBC_REG_set_bit_b(USBC_BP_INTUSBE_EN_SOF, USBC_REG_INTUSBE(USBC0_BASE));
  else    USBC_REG_clear_bit_b(USBC_BP_INTUSBE_EN_SOF, USBC_REG_INTUSBE(USBC0_BASE));
}
