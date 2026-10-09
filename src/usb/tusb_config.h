/* TinyUSB configuration for usbproxy-bm. tusb_mcu.h has no OPT_MCU_NONE
 * branch, so the TUP_* facts about the H3 are stated here. */
#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#define CFG_TUSB_MCU            OPT_MCU_NONE
#define CFG_TUSB_OS             OPT_OS_NONE
#define CFG_TUSB_DEBUG          0

#define TUP_DCD_ENDPOINT_MAX    5       /* ep0 + 4 (H3 musb) */
#define TUP_RHPORT_HIGHSPEED    1
#define TUP_MCU_STRICT_ALIGN    0

/* Device (musb on the OTG port, to the Mac) */
#define CFG_TUD_ENABLED         1
#define CFG_TUD_MAX_SPEED       OPT_MODE_HIGH_SPEED
#define CFG_TUD_ENDPOINT0_SIZE  64
#define CFG_TUD_CDC             1
#define CFG_TUD_CDC_RX_BUFSIZE  512
#define CFG_TUD_CDC_TX_BUFSIZE  2048
#define CFG_TUD_CDC_EP_BUFSIZE  512

/* Host (EHCI1, the watch) */
#define CFG_TUH_ENABLED         1
#define CFG_TUH_MAX_SPEED       OPT_MODE_HIGH_SPEED
#define TUP_USBIP_EHCI
#define CFG_TUH_DEVICE_MAX      1
#define CFG_TUH_HUB             0
#define CFG_TUH_ENUMERATION_BUFSIZE 512
#define CFG_TUH_API_EDPT_XFER   1
/* EHCI DMA structures and host buffers live in the uncached .dma section */
#define CFG_TUH_MEM_SECTION     __attribute__((section(".dma")))
#define CFG_TUH_MEM_ALIGN       __attribute__((aligned(64)))

/* musb is PIO: ordinary cached RAM is fine for device buffers. */
#define CFG_TUSB_MEM_ALIGN      __attribute__((aligned(4)))

#endif
