/*
 * Copyright (c) 2025 Alif Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Synopsys DesignWare DWC3 USB Host Controller driver.
 *
 * This driver operates the DWC3 IP in host mode via its xHCI interface,
 * implementing the Zephyr UHC (USB Host Controller) driver API.
 *
 * Hardware register map (from probing on Alif E8 DK):
 *   0x000: xHCI Capability registers (CAPLENGTH=0x20)
 *   0x020: xHCI Operational registers
 *   0x420: Port registers (1 port)
 *   0x440: Runtime registers (RTSOFF)
 *   0x480: Doorbell registers (DBOFF)
 *   0xC100: DWC3 Global registers
 *
 * Key capabilities: AC64=0 (32-bit), CSZ=1 (64-byte contexts),
 *   MaxSlots=64, MaxPorts=1, MaxIntrs=1, PAGESIZE=4KB
 */

#define DT_DRV_COMPAT snps_dwc3_host

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/usb/uhc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/usb/usb_ch9.h>

#include "uhc_common.h"
#include "../common/usb_dwc3_hw.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uhc_dwc3, CONFIG_UHC_DRIVER_LOG_LEVEL);

/* ---- xHCI register offsets (probed from hardware) ---- */
#define XHCI_CAPLENGTH_OFF	0x000
#define XHCI_OP_OFF		0x020  /* Operational registers */
#define XHCI_PORT_OFF		0x420  /* Port 0 register set */
#define XHCI_RT_OFF		0x440  /* Runtime registers */
#define XHCI_DB_OFF		0x480  /* Doorbell registers */

/* xHCI Operational Register offsets (from operational base) */
#define XHCI_USBCMD		0x00
#define XHCI_USBSTS		0x04
#define XHCI_PAGESIZE		0x08
#define XHCI_DNCTRL		0x14
#define XHCI_CRCR_LO		0x18
#define XHCI_CRCR_HI		0x1C
#define XHCI_DCBAAP_LO		0x30
#define XHCI_DCBAAP_HI		0x34
#define XHCI_CONFIG		0x38

/* USBCMD bits */
#define XHCI_CMD_RS		BIT(0)   /* Run/Stop */
#define XHCI_CMD_HCRST		BIT(1)   /* Host Controller Reset */
#define XHCI_CMD_INTE		BIT(2)   /* Interrupter Enable */

/* USBSTS bits */
#define XHCI_STS_HCH		BIT(0)   /* HC Halted */
#define XHCI_STS_CNR		BIT(11)  /* Controller Not Ready */
#define XHCI_STS_EINT		BIT(3)   /* Event Interrupt */

/* PORTSC bits */
#define XHCI_PORTSC_CCS	BIT(0)
#define XHCI_PORTSC_PED	BIT(1)
#define XHCI_PORTSC_PR		BIT(4)
#define XHCI_PORTSC_PLS_MASK	(0xF << 5)
#define XHCI_PORTSC_PLS(x)	(((x) & 0xF) << 5)
#define XHCI_PORTSC_PP		BIT(9)
#define XHCI_PORTSC_SPEED(v)	(((v) >> 10) & 0xF)
#define XHCI_PORTSC_CSC	BIT(17)
#define XHCI_PORTSC_PEC	BIT(18)
#define XHCI_PORTSC_PRC	BIT(21)
/* Preserve mask: bits that should not be accidentally cleared by W1C */
#define XHCI_PORTSC_PRESERVE	(XHCI_PORTSC_PP | XHCI_PORTSC_PED)

#define XHCI_SPEED_FS		1
#define XHCI_SPEED_LS		2
#define XHCI_SPEED_HS		3

/* Runtime register offsets (from runtime base, interrupter 0) */
#define XHCI_IMAN		0x20  /* Interrupter Management */
#define XHCI_IMOD		0x24  /* Interrupter Moderation */
#define XHCI_ERSTSZ		0x28  /* Event Ring Segment Table Size */
#define XHCI_ERSTBA_LO		0x30  /* Event Ring Segment Table Base Address */
#define XHCI_ERSTBA_HI		0x34
#define XHCI_ERDP_LO		0x38  /* Event Ring Dequeue Pointer */
#define XHCI_ERDP_HI		0x3C

/* ---- xHCI TRB (Transfer Request Block) ---- */
struct xhci_trb {
	uint32_t param_lo;
	uint32_t param_hi;
	uint32_t status;
	uint32_t control;
} __aligned(16);

/* TRB types (bits [15:10] of control) */
#define TRB_TYPE(t)		(((t) & 0x3F) << 10)
#define TRB_TYPE_GET(c)		(((c) >> 10) & 0x3F)

#define TRB_NORMAL		1
#define TRB_SETUP		2
#define TRB_DATA		3
#define TRB_STATUS		4
#define TRB_ISOCH		5
#define TRB_LINK		6
#define TRB_ENABLE_SLOT		9
#define TRB_ADDRESS_DEVICE	11
#define TRB_DISABLE_SLOT	10
#define TRB_EVAL_CTX		13
#define TRB_NOOP_CMD		23
#define TRB_TRANSFER_EVENT	32
#define TRB_CMD_COMPLETION	33
#define TRB_PORT_STATUS_CHANGE	34

/* TRB control flags */
#define TRB_CYCLE		BIT(0)
#define TRB_IOC			BIT(5)   /* Interrupt on Completion */
#define TRB_IDT			BIT(6)   /* Immediate Data */
#define TRB_BSR			BIT(9)   /* Block Set Address Request */
#define TRB_DIR_IN		BIT(16)  /* Data direction for Setup TRB */
#define TRB_SIA			BIT(31)  /* Start Isoch ASAP (Isoch TRB) */
#define TRB_TRT_NO_DATA		0
#define TRB_TRT_OUT		(2 << 16)
#define TRB_TRT_IN		(3 << 16)

/* TRB completion codes (bits [31:24] of status) */
#define TRB_COMP_CODE(s)	(((s) >> 24) & 0xFF)
#define TRB_COMP_SUCCESS	1
#define TRB_COMP_SHORT_PKT	13
#define TRB_COMP_RING_UNDERRUN	14  /* Isoch OUT: no TRB available */
#define TRB_COMP_RING_OVERRUN	15  /* Isoch IN: no TRB available */

/* ---- xHCI Slot/Endpoint Context (64 bytes each when CSZ=1) ---- */
struct xhci_slot_ctx {
	uint32_t field[4];
	uint32_t reserved[12]; /* pad to 64 bytes */
} __aligned(64);

struct xhci_ep_ctx {
	uint32_t field[5];
	uint32_t reserved[11]; /* pad to 64 bytes */
} __aligned(64);

/* Input Control Context */
struct xhci_input_ctrl_ctx {
	uint32_t drop_flags;
	uint32_t add_flags;
	uint32_t reserved[14]; /* pad to 64 bytes */
} __aligned(64);

/* ---- Event Ring Segment Table Entry ---- */
struct xhci_erst_entry {
	uint32_t seg_addr_lo;
	uint32_t seg_addr_hi;
	uint32_t seg_size;
	uint32_t reserved;
} __aligned(64);

/* Sizes */
#define CMD_RING_SIZE		16  /* TRBs in command ring */
#define EVT_RING_SIZE		64  /* TRBs in event ring (needs headroom for isoch) */
#define EP0_RING_SIZE		32  /* TRBs in EP0 transfer ring */

/* Link TRB Toggle Cycle bit - tells controller to toggle its CCS on follow */
#define TRB_TC			BIT(1)
#define MAX_SLOTS_EN		1   /* We only need 1 slot for enumeration */

/* ---- Static DMA-accessible buffers ---- */
/* Must be in shared SRAM (not DTCM) so USB DMA engine can access them.
 * On Alif Ensemble, SRAM0 at 0x02000000 is globally bus-accessible.
 * DTCM (default .bss) is M55-local and NOT reachable by USB DMA.
 */
#define UHC_DMA_SECTION __attribute__((section(".sram0.bss.uhc_dma")))

/* Device Context Base Address Array: (MaxSlots+1) entries × 8 bytes */
static uint32_t dcbaa[(MAX_SLOTS_EN + 1) * 2] __aligned(64) UHC_DMA_SECTION;

/* Command Ring */
static struct xhci_trb cmd_ring[CMD_RING_SIZE + 1] __aligned(64) UHC_DMA_SECTION;
static uint8_t cmd_ring_cycle = 1;
static uint8_t cmd_ring_enq;

/* Event Ring */
static struct xhci_trb evt_ring[EVT_RING_SIZE] __aligned(64) UHC_DMA_SECTION;
static struct xhci_erst_entry erst[1] __aligned(64) UHC_DMA_SECTION;
static uint8_t evt_ring_cycle = 1;
static uint8_t evt_ring_deq;

/* Port speed captured during reset (before PORTSC can change) */
static uint8_t priv_port_speed;

/* EP0 Transfer Ring for slot 1 */
static struct xhci_trb ep0_ring[EP0_RING_SIZE + 1] __aligned(64) UHC_DMA_SECTION;
static uint8_t ep0_ring_cycle = 1;
static uint8_t ep0_ring_enq;

/* Device Context for slot 1 (Slot + 31 EP contexts, each 64 bytes = 2048 bytes) */
static uint8_t dev_ctx_buf[32 * 64] __aligned(64) UHC_DMA_SECTION;

/* Input Context (Input Control + Slot + 31 EPs, each 64 bytes) */
static uint8_t input_ctx_buf[33 * 64] __aligned(64) UHC_DMA_SECTION;

/* Data buffer for USB control transfers (audio config descriptors can exceed 256 bytes) */
static uint8_t xfer_data_buf[512] __aligned(64) UHC_DMA_SECTION;

/* Bulk IN/OUT transfer rings */
#define BULK_RING_SIZE		16
static struct xhci_trb bulk_in_ring[BULK_RING_SIZE + 1] __aligned(64) UHC_DMA_SECTION;
static uint8_t bulk_in_cycle = 1;
static uint8_t bulk_in_enq;

static struct xhci_trb bulk_out_ring[BULK_RING_SIZE + 1] __aligned(64) UHC_DMA_SECTION;
static uint8_t bulk_out_cycle = 1;
static uint8_t bulk_out_enq;

/* Bulk data buffers. Sized for one High-Speed bulk max-packet (512 B),
 * which also holds a single 512-byte mass-storage logical block, a 31-byte
 * Command Block Wrapper or a 13-byte Command Status Wrapper.
 */
static uint8_t bulk_in_buf[512] __aligned(64) UHC_DMA_SECTION;
static uint8_t bulk_out_buf[512] __aligned(64) UHC_DMA_SECTION;

/* Isochronous IN/OUT transfer rings */
#define ISOCH_RING_SIZE		32
static struct xhci_trb isoch_in_ring[ISOCH_RING_SIZE + 1] __aligned(64) UHC_DMA_SECTION;
static uint8_t isoch_in_cycle = 1;
static uint8_t isoch_in_enq;

static struct xhci_trb isoch_out_ring[ISOCH_RING_SIZE + 1] __aligned(64) UHC_DMA_SECTION;
static uint8_t isoch_out_cycle = 1;
static uint8_t isoch_out_enq;

/* Isochronous data buffers — quad-buffered for click-free streaming.
 * With 4 TRBs queued ahead at 1ms interval, we have 3ms of headroom
 * for processing jitter (memcpy, cache flush, log output, etc).
 * Speaker: 48kHz stereo 16-bit = 192 bytes/frame
 * Mic: 48kHz mono 16-bit = 96 bytes/frame
 */
#define ISOCH_OUT_BUF_SIZE	192
#define ISOCH_IN_BUF_SIZE	96
#define ISOCH_NUM_BUFS		4
static uint8_t isoch_out_bufs[ISOCH_NUM_BUFS][ISOCH_OUT_BUF_SIZE]
	__aligned(64) UHC_DMA_SECTION;
static uint8_t isoch_in_bufs[ISOCH_NUM_BUFS][ISOCH_IN_BUF_SIZE]
	__aligned(64) UHC_DMA_SECTION;

/* Track which buffer slot is next to submit */
static uint8_t isoch_out_buf_idx;
static uint8_t isoch_in_buf_idx;

/* ---- Mic->speaker loopback state ----
 * The mic IN endpoint is asynchronous: it delivers a variable number of
 * samples per frame that encodes the device's true ADC rate. Because the
 * device's ADC and DAC share one crystal, feeding the mic's exact per-frame
 * sample count straight to the speaker rate-matches automatically (no
 * SHED fudge needed). We keep the last good upmixed stereo frame to play
 * when a mic frame is missing (overrun).
 */
static bool isoch_lb_started;
static uint8_t isoch_lb_last_stereo[ISOCH_OUT_BUF_SIZE] __aligned(64) UHC_DMA_SECTION;
static int isoch_lb_last_nsamp = 48; /* mono samples in last good mic frame */


/* ---- Thread stack ---- */
#define UHC_DWC3_STACK_SIZE	4096
static K_KERNEL_STACK_DEFINE(uhc_dwc3_stack, UHC_DWC3_STACK_SIZE);
static struct k_thread uhc_dwc3_thread_data;

struct uhc_dwc3_config {
	uintptr_t base;
	void (*irq_enable_func)(const struct device *dev);
	void (*irq_disable_func)(const struct device *dev);
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
};

struct uhc_dwc3_data {
	struct uhc_transfer *last_xfer;
	struct k_sem xfer_sem;
	struct k_sem evt_sem;
	bool port_connected;
	bool enumerating;    /* Suppress ISR disconnect during enumeration */
	enum usb_device_speed port_speed;
	uint8_t slot_id;     /* Assigned slot after Enable Slot */
	uint8_t bulk_in_ep;  /* Bulk IN endpoint address */
	uint8_t bulk_out_ep; /* Bulk OUT endpoint address */
	uint8_t isoch_in_ep;  /* Isochronous IN endpoint address */
	uint8_t isoch_out_ep; /* Isochronous OUT endpoint address */
};

/* ---- Register access helpers ---- */

static inline void xhci_write32(const struct uhc_dwc3_config *cfg,
				uint32_t offset, uint32_t val)
{
	sys_write32(val, cfg->base + offset);
}

static inline uint32_t xhci_read32(const struct uhc_dwc3_config *cfg,
				   uint32_t offset)
{
	return sys_read32(cfg->base + offset);
}

/* Operational register access */
static inline void xhci_op_write(const struct uhc_dwc3_config *cfg,
				 uint32_t reg, uint32_t val)
{
	xhci_write32(cfg, XHCI_OP_OFF + reg, val);
}

static inline uint32_t xhci_op_read(const struct uhc_dwc3_config *cfg,
				    uint32_t reg)
{
	return xhci_read32(cfg, XHCI_OP_OFF + reg);
}

/* Runtime register access (interrupter 0) */
static inline void xhci_rt_write(const struct uhc_dwc3_config *cfg,
				 uint32_t reg, uint32_t val)
{
	xhci_write32(cfg, XHCI_RT_OFF + reg, val);
}

static inline uint32_t xhci_rt_read(const struct uhc_dwc3_config *cfg,
				    uint32_t reg)
{
	return xhci_read32(cfg, XHCI_RT_OFF + reg);
}

/* Doorbell */
static inline void xhci_ring_doorbell(const struct uhc_dwc3_config *cfg,
				      uint8_t slot, uint32_t val)
{
	xhci_write32(cfg, XHCI_DB_OFF + slot * 4, val);
}

/* Port register */
static inline uint32_t xhci_portsc_read(const struct uhc_dwc3_config *cfg)
{
	return xhci_read32(cfg, XHCI_PORT_OFF);
}

static inline void xhci_portsc_write(const struct uhc_dwc3_config *cfg, uint32_t val)
{
	xhci_write32(cfg, XHCI_PORT_OFF, val);
}

/* ---- Command Ring helpers ---- */

static void cmd_ring_init(void)
{
	memset(cmd_ring, 0, sizeof(cmd_ring));
	/* Link TRB: TC=1 so controller toggles cycle on follow, C=0 initially */
	cmd_ring[CMD_RING_SIZE].param_lo = (uint32_t)cmd_ring;
	cmd_ring[CMD_RING_SIZE].param_hi = 0;
	cmd_ring[CMD_RING_SIZE].status = 0;
	cmd_ring[CMD_RING_SIZE].control = TRB_TYPE(TRB_LINK) | TRB_TC;
	cmd_ring_cycle = 1;
	cmd_ring_enq = 0;
}

static void cmd_ring_enqueue(uint32_t p_lo, uint32_t p_hi,
			     uint32_t status, uint32_t control)
{
	struct xhci_trb *trb = &cmd_ring[cmd_ring_enq];

	trb->param_lo = p_lo;
	trb->param_hi = p_hi;
	trb->status = status;
	/* Set cycle bit last */
	trb->control = control | (cmd_ring_cycle ? TRB_CYCLE : 0);

	/* Flush TRB to SRAM so xHCI DMA can see it */
	sys_cache_data_flush_range(trb, sizeof(*trb));

	cmd_ring_enq++;
	if (cmd_ring_enq >= CMD_RING_SIZE) {
		/* Arm Link TRB: set C=current cycle so controller follows it */
		cmd_ring[CMD_RING_SIZE].control =
			TRB_TYPE(TRB_LINK) | TRB_TC |
			(cmd_ring_cycle ? TRB_CYCLE : 0);
		sys_cache_data_flush_range(&cmd_ring[CMD_RING_SIZE],
					   sizeof(struct xhci_trb));
		cmd_ring_cycle ^= 1;
		cmd_ring_enq = 0;
	}
}

/* ---- EP0 Transfer Ring helpers ---- */

static void ep0_ring_init(void)
{
	memset(ep0_ring, 0, sizeof(ep0_ring));
	ep0_ring[EP0_RING_SIZE].param_lo = (uint32_t)ep0_ring;
	ep0_ring[EP0_RING_SIZE].param_hi = 0;
	ep0_ring[EP0_RING_SIZE].status = 0;
	ep0_ring[EP0_RING_SIZE].control = TRB_TYPE(TRB_LINK) | TRB_TC;
	ep0_ring_cycle = 1;
	ep0_ring_enq = 0;
}

static void ep0_ring_enqueue(uint32_t p_lo, uint32_t p_hi,
			     uint32_t status, uint32_t control)
{
	struct xhci_trb *trb = &ep0_ring[ep0_ring_enq];

	trb->param_lo = p_lo;
	trb->param_hi = p_hi;
	trb->status = status;
	trb->control = control | (ep0_ring_cycle ? TRB_CYCLE : 0);

	/* Flush TRB to SRAM so xHCI DMA can see it */
	sys_cache_data_flush_range(trb, sizeof(*trb));

	ep0_ring_enq++;
	if (ep0_ring_enq >= EP0_RING_SIZE) {
		/* Arm Link TRB: set C=current cycle so controller follows it */
		ep0_ring[EP0_RING_SIZE].control =
			TRB_TYPE(TRB_LINK) | TRB_TC |
			(ep0_ring_cycle ? TRB_CYCLE : 0);
		sys_cache_data_flush_range(&ep0_ring[EP0_RING_SIZE],
					   sizeof(struct xhci_trb));
		ep0_ring_cycle ^= 1;
		ep0_ring_enq = 0;
	}
}

/* ---- Event Ring helpers ---- */

static void evt_ring_init(void)
{
	memset(evt_ring, 0, sizeof(evt_ring));
	memset(erst, 0, sizeof(erst));

	erst[0].seg_addr_lo = (uint32_t)evt_ring;
	erst[0].seg_addr_hi = 0;
	erst[0].seg_size = EVT_RING_SIZE;

	evt_ring_cycle = 1;
	evt_ring_deq = 0;
}

static struct xhci_trb *evt_ring_peek(void)
{
	struct xhci_trb *trb = &evt_ring[evt_ring_deq];

	/* Invalidate cache to see what xHCI DMA actually wrote */
	sys_cache_data_invd_range(trb, sizeof(*trb));

	uint8_t c = trb->control & TRB_CYCLE;

	/* Check if the event is valid (cycle bit matches expected) */
	if ((c && evt_ring_cycle) || (!c && !evt_ring_cycle)) {
		return trb;
	}

	return NULL;
}

static void evt_ring_advance(void)
{
	evt_ring_deq++;
	if (evt_ring_deq >= EVT_RING_SIZE) {
		evt_ring_deq = 0;
		evt_ring_cycle ^= 1;
	}
}

/* Wait for an event with timeout */
static struct xhci_trb *evt_ring_wait(const struct uhc_dwc3_config *cfg,
				      int timeout_ms)
{
	struct xhci_trb *evt;
	int elapsed = 0;

	while (elapsed < timeout_ms) {
		evt = evt_ring_peek();
		if (evt != NULL) {
			return evt;
		}
		k_busy_wait(1000);
		elapsed++;
	}

	return NULL;
}

/* Update ERDP to tell controller we consumed events */
static void evt_ring_update_erdp(const struct uhc_dwc3_config *cfg)
{
	uint32_t addr = (uint32_t)&evt_ring[evt_ring_deq];

	/* Set EHB (Event Handler Busy) bit to clear it */
	xhci_rt_write(cfg, XHCI_ERDP_LO, addr | BIT(3));
	xhci_rt_write(cfg, XHCI_ERDP_HI, 0);
}

/* ---- Bulk ring helpers ---- */

static void bulk_in_ring_init(void)
{
	memset(bulk_in_ring, 0, sizeof(bulk_in_ring));
	bulk_in_ring[BULK_RING_SIZE].param_lo = (uint32_t)bulk_in_ring;
	bulk_in_ring[BULK_RING_SIZE].param_hi = 0;
	bulk_in_ring[BULK_RING_SIZE].status = 0;
	bulk_in_ring[BULK_RING_SIZE].control = TRB_TYPE(TRB_LINK) | TRB_TC;
	bulk_in_cycle = 1;
	bulk_in_enq = 0;
}

static void bulk_out_ring_init(void)
{
	memset(bulk_out_ring, 0, sizeof(bulk_out_ring));
	bulk_out_ring[BULK_RING_SIZE].param_lo = (uint32_t)bulk_out_ring;
	bulk_out_ring[BULK_RING_SIZE].param_hi = 0;
	bulk_out_ring[BULK_RING_SIZE].status = 0;
	bulk_out_ring[BULK_RING_SIZE].control = TRB_TYPE(TRB_LINK) | TRB_TC;
	bulk_out_cycle = 1;
	bulk_out_enq = 0;
}

static void isoch_in_ring_init(void)
{
	memset(isoch_in_ring, 0, sizeof(isoch_in_ring));
	isoch_in_ring[ISOCH_RING_SIZE].param_lo = (uint32_t)isoch_in_ring;
	isoch_in_ring[ISOCH_RING_SIZE].param_hi = 0;
	isoch_in_ring[ISOCH_RING_SIZE].status = 0;
	isoch_in_ring[ISOCH_RING_SIZE].control = TRB_TYPE(TRB_LINK) | TRB_TC;
	isoch_in_cycle = 1;
	isoch_in_enq = 0;
}

static void isoch_out_ring_init(void)
{
	memset(isoch_out_ring, 0, sizeof(isoch_out_ring));
	isoch_out_ring[ISOCH_RING_SIZE].param_lo = (uint32_t)isoch_out_ring;
	isoch_out_ring[ISOCH_RING_SIZE].param_hi = 0;
	isoch_out_ring[ISOCH_RING_SIZE].status = 0;
	isoch_out_ring[ISOCH_RING_SIZE].control = TRB_TYPE(TRB_LINK) | TRB_TC;
	isoch_out_cycle = 1;
	isoch_out_enq = 0;
}

static void bulk_ring_enqueue(struct xhci_trb *ring, int ring_size,
			      uint8_t *cycle, uint8_t *enq,
			      uint32_t p_lo, uint32_t p_hi,
			      uint32_t status, uint32_t control)
{
	struct xhci_trb *trb = &ring[*enq];

	trb->param_lo = p_lo;
	trb->param_hi = p_hi;
	trb->status = status;
	trb->control = control | (*cycle ? TRB_CYCLE : 0);
	sys_cache_data_flush_range(trb, sizeof(*trb));

	(*enq)++;
	if (*enq >= ring_size) {
		ring[ring_size].control =
			TRB_TYPE(TRB_LINK) | TRB_TC |
			(*cycle ? TRB_CYCLE : 0);
		sys_cache_data_flush_range(&ring[ring_size], sizeof(struct xhci_trb));
		*cycle ^= 1;
		*enq = 0;
	}
}

/* ---- Generic EP0 Control Transfer ---- */

static int xhci_control_transfer(const struct uhc_dwc3_config *cfg,
				 uint8_t slot_id,
				 uint8_t bmRequestType,
				 uint8_t bRequest,
				 uint16_t wValue,
				 uint16_t wIndex,
				 uint16_t wLength,
				 void *data)
{
	struct xhci_trb *evt;
	bool is_in = (bmRequestType & 0x80) != 0;

	if (wLength > 0 && data != NULL) {
		if (!is_in) {
			/* OUT: copy data to DMA buffer and flush */
			memcpy(xfer_data_buf, data, wLength);
		} else {
			memset(xfer_data_buf, 0, wLength);
		}
		sys_cache_data_flush_range(xfer_data_buf, sizeof(xfer_data_buf));
	}

	/* Setup TRB */
	uint32_t setup_lo = bmRequestType | (bRequest << 8) | (wValue << 16);
	uint32_t setup_hi = wIndex | (wLength << 16);
	uint32_t trt = (wLength == 0) ? TRB_TRT_NO_DATA :
		       (is_in ? TRB_TRT_IN : TRB_TRT_OUT);

	ep0_ring_enqueue(setup_lo, setup_hi, 8,
			 TRB_TYPE(TRB_SETUP) | TRB_IDT | trt);

	/* Data TRB if needed */
	if (wLength > 0) {
		uint32_t dir = is_in ? TRB_DIR_IN : 0;

		ep0_ring_enqueue((uint32_t)xfer_data_buf, 0, wLength,
				 TRB_TYPE(TRB_DATA) | dir | TRB_IOC);
	}

	/* Status TRB */
	uint32_t status_dir = (wLength > 0 && is_in) ? 0 : TRB_DIR_IN;

	ep0_ring_enqueue(0, 0, 0,
			 TRB_TYPE(TRB_STATUS) | status_dir |
			 (wLength == 0 ? TRB_IOC : 0));

	xhci_ring_doorbell(cfg, slot_id, 1);

	/* Wait for transfer completion - skip non-transfer events */
	evt = NULL;
	for (int attempt = 0; attempt < 10; attempt++) {
		evt = evt_ring_wait(cfg, 5000);
		if (evt == NULL) {
			break;
		}
		uint8_t evt_type = TRB_TYPE_GET(evt->control);

		if (evt_type == TRB_TRANSFER_EVENT || evt_type == TRB_CMD_COMPLETION) {
			break; /* This is our event */
		}
		/* Skip non-transfer events (e.g., Port Status Change) */
		LOG_INF("Skipping event type=%u during control xfer", evt_type);
		evt_ring_advance();
		evt_ring_update_erdp(cfg);
		evt = NULL;
	}

	if (evt == NULL) {
		LOG_ERR("Control transfer: no event (req=0x%02x)", bRequest);
		return -ETIMEDOUT;
	}

	uint8_t comp = TRB_COMP_CODE(evt->status);
	uint32_t residual = evt->status & 0xFFFFFF;

	evt_ring_advance();
	evt_ring_update_erdp(cfg);

	/* Check for a second transfer event (status stage completion) */
	struct xhci_trb *evt2 = evt_ring_wait(cfg, 100);

	if (evt2 && TRB_TYPE_GET(evt2->control) == TRB_TRANSFER_EVENT) {
		evt_ring_advance();
		evt_ring_update_erdp(cfg);
	}

	if (comp != TRB_COMP_SUCCESS && comp != TRB_COMP_SHORT_PKT) {
		LOG_ERR("Control transfer failed: comp=%u", comp);
		return -EIO;
	}

	if (is_in && wLength > 0 && data != NULL) {
		sys_cache_data_invd_range(xfer_data_buf, sizeof(xfer_data_buf));
		memcpy(data, xfer_data_buf, wLength - residual);
	}

	return (int)(wLength - residual);
}

/* ---- DWC3 Global register helpers ---- */

static void dwc3_core_soft_reset(uintptr_t base)
{
	volatile udc_dwc3_reg_t *regs = (void *)base;
	uint32_t reg;

	reg = regs->GCTL;
	reg |= USB_GCTL_CORESOFTRESET;
	regs->GCTL = reg;

	reg = regs->GUSB2PHYCFG0;
	reg |= USB_GUSB2PHYCFG_PHYSOFTRST;
	regs->GUSB2PHYCFG0 = reg;

	k_busy_wait(50000);

	reg = regs->GUSB2PHYCFG0;
	reg &= ~USB_GUSB2PHYCFG_PHYSOFTRST;
	regs->GUSB2PHYCFG0 = reg;

	k_busy_wait(50000);

	reg = regs->GCTL;
	reg &= ~USB_GCTL_CORESOFTRESET;
	regs->GCTL = reg;
}

static void dwc3_set_host_mode(uintptr_t base)
{
	volatile udc_dwc3_reg_t *regs = (void *)base;
	uint32_t reg;

	reg = regs->GCTL;
	reg &= ~USB_GCTL_PRTCAPDIR(USB_GCTL_PRTCAP_OTG);
	reg |= USB_GCTL_PRTCAPDIR(USB_GCTL_PRTCAP_HOST);
	regs->GCTL = reg;
}

static void dwc3_configure_phy(uintptr_t base)
{
	volatile udc_dwc3_reg_t *regs = (void *)base;
	uint32_t reg;

	reg = regs->GUSB2PHYCFG0;
	reg &= ~USB_GUSB2PHYCFG_ULPIAUTORES;
	reg &= ~(USB_GUSB2PHYCFG_PHYIF_MASK | USB_GUSB2PHYCFG_USBTRDTIM_MASK);
	reg |= USB_GUSB2PHYCFG_PHYIF(UTMI_PHYIF_8_BIT) |
	       USB_GUSB2PHYCFG_USBTRDTIM(USBTRDTIM_UTMI_8_BIT);
	regs->GUSB2PHYCFG0 = reg;
}

static void dwc3_configure_host_params(uintptr_t base)
{
	volatile udc_dwc3_reg_t *regs = (void *)base;
	uint32_t reg;

	reg = regs->GCTL;
	reg |= USB_GCTL_DSBLCLKGTNG;
	regs->GCTL = reg;

	reg = regs->GSBUSCFG0;
	reg |= USB_GSBUSCFG0_INCRBRSTENA | USB_GSBUSCFG0_INCR16BRSTENA;
	regs->GSBUSCFG0 = reg;
}

/* ---- xHCI Controller Init ---- */

static int xhci_controller_init(const struct uhc_dwc3_config *cfg)
{
	uint32_t sts;

	/* Log DMA buffer addresses for debugging */
	LOG_INF("DMA buffer addresses:");
	LOG_INF("  dcbaa:     %p", (void *)dcbaa);
	LOG_INF("  cmd_ring:  %p", (void *)cmd_ring);
	LOG_INF("  evt_ring:  %p", (void *)evt_ring);
	LOG_INF("  erst:      %p", (void *)erst);
	LOG_INF("  ep0_ring:  %p", (void *)ep0_ring);
	LOG_INF("  dev_ctx:   %p", (void *)dev_ctx_buf);
	LOG_INF("  input_ctx: %p", (void *)input_ctx_buf);
	LOG_INF("  xfer_data: %p", (void *)xfer_data_buf);

	/* Wait for Controller Not Ready to clear */
	for (int i = 0; i < 1000; i++) {
		sts = xhci_op_read(cfg, XHCI_USBSTS);
		if (!(sts & XHCI_STS_CNR)) {
			break;
		}
		k_busy_wait(1000);
	}

	sts = xhci_op_read(cfg, XHCI_USBSTS);
	LOG_INF("USBSTS after CNR wait: 0x%08x", sts);

	if (sts & XHCI_STS_CNR) {
		LOG_ERR("xHCI controller not ready (CNR still set)");
		return -ETIMEDOUT;
	}

	/* Clear any pending error bits (W1C) */
	if (sts & (BIT(2) | BIT(3) | BIT(4) | BIT(12))) {
		LOG_WRN("Clearing USBSTS error/status bits: 0x%08x", sts);
		xhci_op_write(cfg, XHCI_USBSTS, sts & (BIT(2) | BIT(3) | BIT(4) | BIT(12)));
		sts = xhci_op_read(cfg, XHCI_USBSTS);
		LOG_INF("USBSTS after clear: 0x%08x", sts);
	}

	/* Step 1: Halt the controller if running */
	uint32_t cmd = xhci_op_read(cfg, XHCI_USBCMD);

	if (cmd & XHCI_CMD_RS) {
		xhci_op_write(cfg, XHCI_USBCMD, cmd & ~XHCI_CMD_RS);
		for (int i = 0; i < 100; i++) {
			k_busy_wait(1000);
			if (xhci_op_read(cfg, XHCI_USBSTS) & XHCI_STS_HCH) {
				break;
			}
		}
	}

	LOG_INF("Controller halted: USBSTS=0x%08x", xhci_op_read(cfg, XHCI_USBSTS));

	/* Step 2: xHCI Reset (HCRST) - puts controller in known state */
	xhci_op_write(cfg, XHCI_USBCMD, XHCI_CMD_HCRST);

	/* Wait for HCRST to self-clear */
	for (int i = 0; i < 1000; i++) {
		k_busy_wait(1000);
		cmd = xhci_op_read(cfg, XHCI_USBCMD);
		if (!(cmd & XHCI_CMD_HCRST)) {
			break;
		}
	}

	if (xhci_op_read(cfg, XHCI_USBCMD) & XHCI_CMD_HCRST) {
		LOG_ERR("xHCI reset did not complete");
		return -ETIMEDOUT;
	}

	/* Wait for CNR to clear after reset */
	for (int i = 0; i < 1000; i++) {
		sts = xhci_op_read(cfg, XHCI_USBSTS);
		if (!(sts & XHCI_STS_CNR)) {
			break;
		}
		k_busy_wait(1000);
	}

	LOG_INF("After xHCI reset: USBCMD=0x%08x USBSTS=0x%08x",
		xhci_op_read(cfg, XHCI_USBCMD),
		xhci_op_read(cfg, XHCI_USBSTS));

	/* Make sure controller is halted */
	if (!(xhci_op_read(cfg, XHCI_USBSTS) & XHCI_STS_HCH)) {
		LOG_ERR("Controller not halted after reset, cannot program CRCR");
		return -EIO;
	}

	/* Initialize data structures */
	memset(dcbaa, 0, sizeof(dcbaa));
	cmd_ring_init();
	evt_ring_init();
	ep0_ring_init();

	/* Flush all DMA buffers to SRAM after initialization */
	sys_cache_data_flush_range(dcbaa, sizeof(dcbaa));
	sys_cache_data_flush_range(cmd_ring, sizeof(cmd_ring));
	sys_cache_data_flush_range(evt_ring, sizeof(evt_ring));
	sys_cache_data_flush_range(erst, sizeof(erst));
	sys_cache_data_flush_range(ep0_ring, sizeof(ep0_ring));
	sys_cache_data_flush_range(dev_ctx_buf, sizeof(dev_ctx_buf));
	sys_cache_data_flush_range(input_ctx_buf, sizeof(input_ctx_buf));

	/* Set DCBAAP */
	xhci_op_write(cfg, XHCI_DCBAAP_LO, (uint32_t)dcbaa);
	xhci_op_write(cfg, XHCI_DCBAAP_HI, 0);

	/* Set Command Ring Control Register (must be written while halted!) */
	uint32_t crcr_val = (uint32_t)cmd_ring | cmd_ring_cycle;

	xhci_op_write(cfg, XHCI_CRCR_LO, crcr_val);
	xhci_op_write(cfg, XHCI_CRCR_HI, 0);

	/* Readback verification */
	uint32_t crcr_rb = xhci_op_read(cfg, XHCI_CRCR_LO);

	LOG_INF("CRCR write: 0x%08x, readback: 0x%08x %s",
		crcr_val, crcr_rb,
		(crcr_rb & ~0x7) == ((uint32_t)cmd_ring & ~0x7) ? "OK" : "MISMATCH!");

	/* Set CONFIG.MaxSlotsEn */
	xhci_op_write(cfg, XHCI_CONFIG, MAX_SLOTS_EN);

	/* Set up Event Ring: ERSTSZ, ERDP, ERSTBA */
	xhci_rt_write(cfg, XHCI_ERSTSZ, 1);
	xhci_rt_write(cfg, XHCI_ERDP_LO, (uint32_t)evt_ring);
	xhci_rt_write(cfg, XHCI_ERDP_HI, 0);
	/* ERSTBA must be written last (it triggers HW to load the table) */
	xhci_rt_write(cfg, XHCI_ERSTBA_LO, (uint32_t)erst);
	xhci_rt_write(cfg, XHCI_ERSTBA_HI, 0);

	/* Readback verify event ring setup */
	LOG_INF("Event ring setup readback:");
	LOG_INF("  ERSTSZ:    %u (expect 1)", xhci_rt_read(cfg, XHCI_ERSTSZ));
	LOG_INF("  ERDP_LO:   0x%08x (expect 0x%08x)",
		xhci_rt_read(cfg, XHCI_ERDP_LO), (uint32_t)evt_ring);
	LOG_INF("  ERSTBA_LO: 0x%08x (expect 0x%08x)",
		xhci_rt_read(cfg, XHCI_ERSTBA_LO), (uint32_t)erst);

	/* Verify ERST contents */
	LOG_INF("  ERST[0]: addr=0x%08x size=%u",
		erst[0].seg_addr_lo, erst[0].seg_size);

	/* Readback CRCR while still halted */
	LOG_INF("CRCR readback (halted): LO=0x%08x HI=0x%08x",
		xhci_op_read(cfg, XHCI_CRCR_LO),
		xhci_op_read(cfg, XHCI_CRCR_HI));

	/* Enable interrupts on interrupter 0 */
	xhci_rt_write(cfg, XHCI_IMAN, BIT(1)); /* IE=1 */
	xhci_rt_write(cfg, XHCI_IMOD, 0);

	/* Start the controller: set R/S, enable interrupts */
	xhci_op_write(cfg, XHCI_USBCMD, XHCI_CMD_RS | XHCI_CMD_INTE);

	/* Wait for HCH to clear */
	for (int i = 0; i < 100; i++) {
		sts = xhci_op_read(cfg, XHCI_USBSTS);
		if (!(sts & XHCI_STS_HCH)) {
			LOG_INF("xHCI controller running (USBSTS=0x%08x)", sts);
			/* Check for immediate errors */
			k_busy_wait(5000);
			sts = xhci_op_read(cfg, XHCI_USBSTS);
			cmd = xhci_op_read(cfg, XHCI_USBCMD);
			LOG_INF("Post-start check: USBCMD=0x%08x USBSTS=0x%08x",
				cmd, sts);
			if (sts & BIT(2)) {
				LOG_ERR("HSE (Host System Error) after start!");
			}
			if (!(cmd & XHCI_CMD_RS)) {
				LOG_ERR("RS bit cleared after start (controller halted)");
			}
			return 0;
		}
		k_busy_wait(1000);
	}

	LOG_ERR("xHCI controller failed to start (USBSTS=0x%08x)",
		xhci_op_read(cfg, XHCI_USBSTS));
	return -ETIMEDOUT;
}

/* ---- Port Reset ---- */

/* PORTSC W1C (Write-1-to-Clear) bits - must NOT accidentally write 1 to these */
#define XHCI_PORTSC_W1C_BITS	(XHCI_PORTSC_CSC | XHCI_PORTSC_PEC | BIT(19) | \
				 BIT(20) | XHCI_PORTSC_PRC | BIT(22) | BIT(23))

static int xhci_port_reset(const struct uhc_dwc3_config *cfg)
{
	uint32_t portsc;

	/* Verify controller is running */
	uint32_t usbcmd = xhci_op_read(cfg, XHCI_USBCMD);
	uint32_t usbsts = xhci_op_read(cfg, XHCI_USBSTS);

	LOG_INF("Pre-reset: USBCMD=0x%08x USBSTS=0x%08x", usbcmd, usbsts);

	if (!(usbcmd & XHCI_CMD_RS)) {
		LOG_WRN("xHCI not running (RS=0), starting...");
		xhci_op_write(cfg, XHCI_USBCMD, usbcmd | XHCI_CMD_RS | XHCI_CMD_INTE);
		k_busy_wait(10000);
		usbcmd = xhci_op_read(cfg, XHCI_USBCMD);
		usbsts = xhci_op_read(cfg, XHCI_USBSTS);
		LOG_INF("After start: USBCMD=0x%08x USBSTS=0x%08x", usbcmd, usbsts);
	}

	portsc = xhci_portsc_read(cfg);
	if (!(portsc & XHCI_PORTSC_CCS)) {
		LOG_ERR("No device connected for port reset");
		return -ENODEV;
	}

	LOG_INF("Port reset: PORTSC before=0x%08x", portsc);

	/* xHCI PORTSC read-modify-write:
	 * - Preserve non-W1C bits (like PP)
	 * - Clear W1C bits (write 0 to preserve them)
	 * - Set PR=1 to initiate reset
	 */
	portsc = portsc & ~XHCI_PORTSC_W1C_BITS; /* Mask off W1C bits */
	portsc &= ~XHCI_PORTSC_PED; /* Don't accidentally disable port */
	portsc |= XHCI_PORTSC_PR;   /* Set Port Reset */
	xhci_portsc_write(cfg, portsc);

	/* Read back immediately to verify PR took */
	k_busy_wait(1000);
	portsc = xhci_portsc_read(cfg);
	LOG_INF("Port reset: PORTSC after write=0x%08x (PR=%d)",
		portsc, !!(portsc & XHCI_PORTSC_PR));

	/* Wait for Port Reset Change (PRC) — indicates reset completed */
	for (int i = 0; i < 1000; i++) {
		k_busy_wait(1000);
		portsc = xhci_portsc_read(cfg);
		if (portsc & XHCI_PORTSC_PRC) {
			LOG_INF("PRC detected at %d ms, PORTSC=0x%08x", i, portsc);
			/* Clear PRC (W1C) - preserve other bits */
			uint32_t clr = portsc & ~XHCI_PORTSC_W1C_BITS;
			clr &= ~XHCI_PORTSC_PED; /* Don't accidentally clear PED */
			clr |= XHCI_PORTSC_PRC;  /* W1C: write 1 to clear PRC */
			xhci_portsc_write(cfg, clr);
			break;
		}
		if (portsc & XHCI_PORTSC_PED) {
			LOG_INF("PED set at %d ms (no PRC), PORTSC=0x%08x", i, portsc);
			break;
		}
	}

	portsc = xhci_portsc_read(cfg);
	LOG_INF("Port reset: PORTSC final=0x%08x", portsc);
	LOG_INF("  CCS=%d PED=%d PR=%d PP=%d Speed=%u PLS=%u",
		!!(portsc & XHCI_PORTSC_CCS),
		!!(portsc & XHCI_PORTSC_PED),
		!!(portsc & XHCI_PORTSC_PR),
		!!(portsc & XHCI_PORTSC_PP),
		XHCI_PORTSC_SPEED(portsc),
		(portsc >> 5) & 0xF);

	if (!(portsc & XHCI_PORTSC_PED)) {
		LOG_ERR("Port not enabled after reset");
		return -EIO;
	}

	/* Capture speed immediately while port is stable */
	priv_port_speed = XHCI_PORTSC_SPEED(portsc);

	/* Consume port status change event if any */
	struct xhci_trb *evt = evt_ring_wait(cfg, 100);

	if (evt && TRB_TYPE_GET(evt->control) == TRB_PORT_STATUS_CHANGE) {
		LOG_INF("Port Status Change Event received");
		evt_ring_advance();
		evt_ring_update_erdp(cfg);
	}

	return 0;
}

/* ---- Enable Slot Command ---- */

static int xhci_enable_slot(const struct uhc_dwc3_config *cfg, uint8_t *slot_id)
{
	struct xhci_trb *evt;

	/* Debug: verify ring addresses and controller state */
	uint32_t crcr_lo = xhci_op_read(cfg, XHCI_CRCR_LO);
	uint32_t crcr_hi = xhci_op_read(cfg, XHCI_CRCR_HI);

	LOG_INF("Enable Slot: cmd_ring addr=%p, CRCR=0x%08x:%08x",
		(void *)cmd_ring, crcr_hi, crcr_lo);
	LOG_INF("Enable Slot: evt_ring addr=%p, enq=%u cycle=%u",
		(void *)evt_ring, cmd_ring_enq, cmd_ring_cycle);
	LOG_INF("Enable Slot: USBCMD=0x%08x USBSTS=0x%08x",
		xhci_op_read(cfg, XHCI_USBCMD),
		xhci_op_read(cfg, XHCI_USBSTS));

	/* Enqueue Enable Slot Command */
	uint8_t enq_idx = cmd_ring_enq;

	cmd_ring_enqueue(0, 0, 0, TRB_TYPE(TRB_ENABLE_SLOT));

	LOG_INF("Enqueued TRB at idx=%u: ctrl=0x%08x",
		enq_idx, cmd_ring[enq_idx].control);

	/* Ring host controller doorbell (slot 0, target 0) */
	xhci_ring_doorbell(cfg, 0, 0);

	/* Check USBSTS right after doorbell */
	k_busy_wait(1000);
	uint32_t sts = xhci_op_read(cfg, XHCI_USBSTS);

	LOG_INF("USBSTS after doorbell: 0x%08x (HCH=%d HSE=%d EINT=%d)",
		sts, !!(sts & BIT(0)), !!(sts & BIT(2)), !!(sts & BIT(3)));

	/* Dump event ring contents */
	LOG_INF("Event ring (deq=%u cycle=%u):", evt_ring_deq, evt_ring_cycle);
	for (int i = 0; i < 4; i++) {
		LOG_INF("  evt[%d]: param=0x%08x:%08x sts=0x%08x ctrl=0x%08x",
			i, evt_ring[i].param_hi, evt_ring[i].param_lo,
			evt_ring[i].status, evt_ring[i].control);
	}

	/* Wait for Command Completion Event */
	evt = evt_ring_wait(cfg, 2000);
	if (evt == NULL) {
		/* Final state dump */
		LOG_ERR("Enable Slot: no completion event");
		LOG_INF("Final USBCMD=0x%08x USBSTS=0x%08x",
			xhci_op_read(cfg, XHCI_USBCMD),
			xhci_op_read(cfg, XHCI_USBSTS));
		LOG_INF("Final event ring:");
		for (int i = 0; i < 4; i++) {
			LOG_INF("  evt[%d]: ctrl=0x%08x sts=0x%08x",
				i, evt_ring[i].control, evt_ring[i].status);
		}
		return -ETIMEDOUT;
	}

	/* Ring host controller doorbell (slot 0, target 0) */
	xhci_ring_doorbell(cfg, 0, 0);

	/* Wait for Command Completion Event */
	evt = evt_ring_wait(cfg, 2000);
	if (evt == NULL) {
		LOG_ERR("Enable Slot: no completion event");
		return -ETIMEDOUT;
	}

	uint8_t type = TRB_TYPE_GET(evt->control);
	uint8_t comp = TRB_COMP_CODE(evt->status);

	LOG_INF("Enable Slot event: type=%u comp=%u", type, comp);

	if (type != TRB_CMD_COMPLETION || comp != TRB_COMP_SUCCESS) {
		LOG_ERR("Enable Slot failed: type=%u comp=%u", type, comp);
		evt_ring_advance();
		evt_ring_update_erdp(cfg);
		return -EIO;
	}

	*slot_id = (evt->control >> 24) & 0xFF;
	LOG_INF("Slot %u enabled", *slot_id);

	evt_ring_advance();
	evt_ring_update_erdp(cfg);

	return 0;
}

/* ---- Disable Slot Command ---- */

static int xhci_disable_slot(const struct uhc_dwc3_config *cfg, uint8_t slot_id)
{
	struct xhci_trb *evt;

	if (slot_id == 0) {
		return 0; /* No slot to disable */
	}

	LOG_INF("Disabling slot %u", slot_id);

	/* Enqueue Disable Slot Command */
	cmd_ring_enqueue(0, 0, 0,
			 TRB_TYPE(TRB_DISABLE_SLOT) | (slot_id << 24));

	xhci_ring_doorbell(cfg, 0, 0);

	evt = evt_ring_wait(cfg, 2000);
	if (evt == NULL) {
		LOG_ERR("Disable Slot: no completion event");
		return -ETIMEDOUT;
	}

	uint8_t comp = TRB_COMP_CODE(evt->status);

	LOG_INF("Disable Slot event: comp=%u", comp);

	evt_ring_advance();
	evt_ring_update_erdp(cfg);

	/* Clear DCBAA entry */
	dcbaa[slot_id * 2] = 0;
	dcbaa[slot_id * 2 + 1] = 0;
	sys_cache_data_flush_range(dcbaa, sizeof(dcbaa));

	return (comp == TRB_COMP_SUCCESS) ? 0 : -EIO;
}

/* ---- Address Device Command ---- */

static int xhci_address_device(const struct uhc_dwc3_config *cfg,
			       uint8_t slot_id, uint8_t port_speed)
{
	struct xhci_trb *evt;

	/* Clear contexts */
	memset(input_ctx_buf, 0, sizeof(input_ctx_buf));
	memset(dev_ctx_buf, 0, sizeof(dev_ctx_buf));

	/* Input Control Context (offset 0): add Slot (bit 0) and EP0 (bit 1) */
	struct xhci_input_ctrl_ctx *icc = (void *)input_ctx_buf;

	icc->add_flags = BIT(0) | BIT(1);

	/* Slot Context (offset 64 bytes) */
	uint32_t *slot = (uint32_t *)(input_ctx_buf + 64);
	/* Field 0: Route String=0, Speed, Context Entries=1 (EP0) */
	uint32_t speed_val;

	switch (port_speed) {
	case XHCI_SPEED_HS:
		speed_val = 3;
		break;
	case XHCI_SPEED_LS:
		speed_val = 2;
		break;
	default:
		speed_val = 1; /* Full Speed */
		break;
	}
	slot[0] = (1 << 27) | (speed_val << 20); /* Context Entries=1, Speed */
	/* Field 1: Root Hub Port Number = 1 */
	slot[1] = (1 << 16); /* Root Hub Port = 1 */

	/* EP0 Context (offset 128 bytes) */
	uint32_t *ep0 = (uint32_t *)(input_ctx_buf + 128);
	/* EP0 is always Control, MaxPacketSize depends on speed.
	 * For Full Speed, use 64 (max possible) to avoid babble errors —
	 * actual MPS is read from bMaxPacketSize0 in device descriptor.
	 */
	uint16_t mps = (port_speed == XHCI_SPEED_LS) ? 8 : 64;
	/* Field 1: EP Type=4 (Control Bidirectional), CErr=3, MaxPacketSize */
	ep0[1] = (3 << 1) | (4 << 3) | (mps << 16); /* CErr=3, EPType=Control, MPS */
	/* Field 2: TR Dequeue Pointer (low) with DCS=1 */
	ep0[2] = (uint32_t)ep0_ring | 1; /* DCS=1 */
	/* Field 3: TR Dequeue Pointer (high) - 0 for 32-bit */
	ep0[3] = 0;
	/* Field 4: Average TRB Length = 8 (for control) */
	ep0[4] = 8;

	/* Set output device context pointer in DCBAA */
	dcbaa[slot_id * 2] = (uint32_t)dev_ctx_buf;
	dcbaa[slot_id * 2 + 1] = 0;

	/* Flush input context and DCBAA to SRAM */
	sys_cache_data_flush_range(input_ctx_buf, sizeof(input_ctx_buf));
	sys_cache_data_flush_range(dev_ctx_buf, sizeof(dev_ctx_buf));
	sys_cache_data_flush_range(dcbaa, sizeof(dcbaa));

	/* Enqueue Address Device Command */
	cmd_ring_enqueue((uint32_t)input_ctx_buf, 0, 0,
			 TRB_TYPE(TRB_ADDRESS_DEVICE) | (slot_id << 24));

	xhci_ring_doorbell(cfg, 0, 0);

	/* Wait for completion */
	evt = evt_ring_wait(cfg, 5000);
	if (evt == NULL) {
		LOG_ERR("Address Device: no completion event");
		return -ETIMEDOUT;
	}

	uint8_t comp = TRB_COMP_CODE(evt->status);

	LOG_INF("Address Device event: comp=%u", comp);

	evt_ring_advance();
	evt_ring_update_erdp(cfg);

	if (comp != TRB_COMP_SUCCESS) {
		LOG_ERR("Address Device failed: comp=%u", comp);
		return -EIO;
	}

	/* Invalidate output device context cache to read DMA-written data */
	sys_cache_data_invd_range(dev_ctx_buf, sizeof(dev_ctx_buf));

	/* Read back assigned address from output Slot Context */
	uint32_t *out_slot = (uint32_t *)dev_ctx_buf;

	LOG_INF("Device addressed: slot_ctx[3]=0x%08x (addr=%u)",
		out_slot[3], out_slot[3] & 0xFF);

	return 0;
}

/* ---- Get Device Descriptor via EP0 ---- */

static int xhci_get_device_descriptor(const struct uhc_dwc3_config *cfg,
				      uint8_t slot_id,
				      struct usb_device_descriptor *desc,
				      uint16_t req_len)
{
	struct xhci_trb *evt;

	memset(xfer_data_buf, 0, sizeof(xfer_data_buf));
	sys_cache_data_flush_range(xfer_data_buf, sizeof(xfer_data_buf));

	/*
	 * Setup TRB: GET_DESCRIPTOR(Device, index 0) for req_len bytes.
	 *
	 * The xHCI Setup TRB uses Immediate Data (IDT=1), where the 8-byte
	 * USB setup packet is placed directly in param_lo and param_hi
	 * as two little-endian 32-bit words:
	 *
	 * param_lo = bmRequestType | (bRequest << 8) | (wValue << 16)
	 * param_hi = wIndex | (wLength << 16)
	 */
	uint32_t setup_lo = 0x80               /* bmRequestType: Device-to-Host, Standard */
			  | (0x06 << 8)        /* bRequest: GET_DESCRIPTOR */
			  | (0x0100 << 16);    /* wValue: type=1(DEVICE), index=0 -> 0x0100 in LE16 */
	uint32_t setup_hi = 0                  /* wIndex: 0 */
			  | ((uint32_t)req_len << 16); /* wLength */

	LOG_INF("Setup TRB: param_lo=0x%08x param_hi=0x%08x", setup_lo, setup_hi);

	ep0_ring_enqueue(setup_lo, setup_hi,
			 8, /* TRB Transfer Length = 8 (setup packet) */
			 TRB_TYPE(TRB_SETUP) | TRB_IDT | TRB_TRT_IN);

	/* Data TRB: point to data buffer, length=req_len, direction=IN */
	ep0_ring_enqueue((uint32_t)xfer_data_buf, 0,
			 req_len, /* Transfer Length */
			 TRB_TYPE(TRB_DATA) | TRB_DIR_IN | TRB_IOC);

	/* Status TRB: zero-length, direction=OUT */
	ep0_ring_enqueue(0, 0,
			 0,
			 TRB_TYPE(TRB_STATUS));

	/* Ring doorbell for slot, target EP0 (DCI=1) */
	xhci_ring_doorbell(cfg, slot_id, 1);

	/* Wait for Transfer Event */
	evt = evt_ring_wait(cfg, 5000);
	if (evt == NULL) {
		LOG_ERR("Get Descriptor: no transfer event");
		return -ETIMEDOUT;
	}

	uint8_t type = TRB_TYPE_GET(evt->control);
	uint8_t comp = TRB_COMP_CODE(evt->status);

	LOG_INF("Transfer event: type=%u comp=%u residual=%u",
		type, comp, evt->status & 0xFFFFFF);

	evt_ring_advance();
	evt_ring_update_erdp(cfg);

	/* There might be a second event for the status stage */
	evt = evt_ring_wait(cfg, 1000);
	if (evt) {
		LOG_INF("Status event: type=%u comp=%u",
			TRB_TYPE_GET(evt->control), TRB_COMP_CODE(evt->status));
		evt_ring_advance();
		evt_ring_update_erdp(cfg);
	}

	if (comp != TRB_COMP_SUCCESS && comp != TRB_COMP_SHORT_PKT) {
		LOG_ERR("Get Descriptor failed: comp=%u", comp);
		return -EIO;
	}

	/* Invalidate data buffer cache to read what USB DMA wrote */
	sys_cache_data_invd_range(xfer_data_buf, sizeof(xfer_data_buf));

	memcpy(desc, xfer_data_buf, sizeof(*desc));

	return 0;
}

/* ---- Evaluate Context: update EP0 Max Packet Size ----
 * After the first 8-byte device-descriptor read reveals bMaxPacketSize0,
 * the EP0 context (set to a default during Address Device) must be updated
 * to the device's real value, or subsequent control transfers mis-frame
 * (e.g. an 8-byte-MPS device truncates at the first packet under a 64 MPS).
 */
static int xhci_evaluate_context_mps(const struct uhc_dwc3_config *cfg,
				     uint8_t slot_id, uint16_t mps)
{
	struct xhci_trb *evt;

	memset(input_ctx_buf, 0, sizeof(input_ctx_buf));

	/* Input Control Context: evaluate EP0 only (bit 1) */
	struct xhci_input_ctrl_ctx *icc = (void *)input_ctx_buf;

	icc->add_flags = BIT(1);

	/* EP0 Context (offset 128): set EPType=Control, CErr=3, new MPS */
	uint32_t *ep0 = (uint32_t *)(input_ctx_buf + 128);

	ep0[1] = (3 << 1) | (4 << 3) | ((uint32_t)mps << 16);

	sys_cache_data_flush_range(input_ctx_buf, sizeof(input_ctx_buf));

	cmd_ring_enqueue((uint32_t)input_ctx_buf, 0, 0,
			 TRB_TYPE(TRB_EVAL_CTX) | (slot_id << 24));
	xhci_ring_doorbell(cfg, 0, 0);

	evt = evt_ring_wait(cfg, 5000);
	if (evt == NULL) {
		LOG_ERR("Evaluate Context: no completion event");
		return -ETIMEDOUT;
	}

	uint8_t comp = TRB_COMP_CODE(evt->status);

	evt_ring_advance();
	evt_ring_update_erdp(cfg);

	if (comp != TRB_COMP_SUCCESS) {
		LOG_ERR("Evaluate Context failed: comp=%u", comp);
		return -EIO;
	}

	LOG_INF("EP0 MaxPacketSize updated to %u", mps);
	return 0;
}

/* ---- Configure Endpoint Command ---- */

/* xHCI Endpoint Types (for EP Context field 1 bits [5:3]) */
#define XHCI_EP_TYPE_ISOCH_OUT	1
#define XHCI_EP_TYPE_BULK_OUT	2
#define XHCI_EP_TYPE_INT_OUT	3
#define XHCI_EP_TYPE_CTRL	4
#define XHCI_EP_TYPE_ISOCH_IN	5
#define XHCI_EP_TYPE_BULK_IN	6
#define XHCI_EP_TYPE_INT_IN	7

#define TRB_CONFIG_EP		12

/*
 * Calculate xHCI Device Context Index (DCI) from USB endpoint address.
 * DCI = 2 * ep_num + direction (0=OUT, 1=IN).
 * EP0 is DCI=1.
 */
static inline uint8_t ep_to_dci(uint8_t ep_addr)
{
	uint8_t ep_num = ep_addr & 0x0F;
	uint8_t dir = (ep_addr & 0x80) ? 1 : 0;

	return (ep_num * 2) + dir;
}

static int xhci_configure_endpoint(const struct uhc_dwc3_config *cfg,
				   uint8_t slot_id,
				   uint8_t bulk_in_ep,
				   uint8_t bulk_out_ep,
				   uint16_t max_packet_size)
{
	struct xhci_trb *evt;
	uint8_t dci_in = ep_to_dci(bulk_in_ep);
	uint8_t dci_out = ep_to_dci(bulk_out_ep);
	uint8_t max_dci = (dci_in > dci_out) ? dci_in : dci_out;

	LOG_INF("Configure EP: in=0x%02x(DCI=%u) out=0x%02x(DCI=%u) MPS=%u",
		bulk_in_ep, dci_in, bulk_out_ep, dci_out, max_packet_size);

	/* Init bulk rings */
	bulk_in_ring_init();
	bulk_out_ring_init();
	sys_cache_data_flush_range(bulk_in_ring, sizeof(bulk_in_ring));
	sys_cache_data_flush_range(bulk_out_ring, sizeof(bulk_out_ring));

	/* Build Input Context */
	memset(input_ctx_buf, 0, sizeof(input_ctx_buf));

	/* Input Control Context: add Slot + bulk EP contexts */
	struct xhci_input_ctrl_ctx *icc = (void *)input_ctx_buf;

	icc->add_flags = BIT(0) | BIT(dci_in) | BIT(dci_out);

	/* Slot Context: update Context Entries to include new endpoints */
	uint32_t *slot = (uint32_t *)(input_ctx_buf + 64);

	/* Read current slot context from output and modify */
	sys_cache_data_invd_range(dev_ctx_buf, sizeof(dev_ctx_buf));
	memcpy(slot, dev_ctx_buf, 64);
	/* Update Context Entries field (bits [31:27]) */
	slot[0] = (slot[0] & ~(0x1F << 27)) | (max_dci << 27);

	/* Bulk IN EP Context (at offset 64 * (1 + dci_in)) */
	uint32_t *ep_in = (uint32_t *)(input_ctx_buf + 64 * (1 + dci_in));

	ep_in[1] = (3 << 1) |                /* CErr = 3 */
		   (XHCI_EP_TYPE_BULK_IN << 3) | /* EP Type */
		   (max_packet_size << 16);   /* Max Packet Size */
	ep_in[2] = (uint32_t)bulk_in_ring | 1; /* TR Dequeue Ptr + DCS=1 */
	ep_in[3] = 0;
	ep_in[4] = 0; /* Average TRB Length (0 = let HW decide) */

	/* Bulk OUT EP Context */
	uint32_t *ep_out = (uint32_t *)(input_ctx_buf + 64 * (1 + dci_out));

	ep_out[1] = (3 << 1) |
		    (XHCI_EP_TYPE_BULK_OUT << 3) |
		    (max_packet_size << 16);
	ep_out[2] = (uint32_t)bulk_out_ring | 1;
	ep_out[3] = 0;
	ep_out[4] = 0;

	sys_cache_data_flush_range(input_ctx_buf, sizeof(input_ctx_buf));

	/* Enqueue Configure Endpoint Command */
	cmd_ring_enqueue((uint32_t)input_ctx_buf, 0, 0,
			 TRB_TYPE(TRB_CONFIG_EP) | (slot_id << 24));

	xhci_ring_doorbell(cfg, 0, 0);

	evt = evt_ring_wait(cfg, 5000);
	if (evt == NULL) {
		LOG_ERR("Configure EP: no completion event");
		return -ETIMEDOUT;
	}

	uint8_t comp = TRB_COMP_CODE(evt->status);

	LOG_INF("Configure EP event: comp=%u", comp);

	evt_ring_advance();
	evt_ring_update_erdp(cfg);

	if (comp != TRB_COMP_SUCCESS) {
		LOG_ERR("Configure EP failed: comp=%u", comp);
		return -EIO;
	}

	return 0;
}

/* ---- Bulk Transfers ---- */

static int xhci_bulk_transfer_out(const struct uhc_dwc3_config *cfg,
				  uint8_t slot_id, uint8_t ep_addr,
				  const uint8_t *data, size_t len)
{
	struct xhci_trb *evt;
	uint8_t dci = ep_to_dci(ep_addr);

	if (len > sizeof(bulk_out_buf)) {
		len = sizeof(bulk_out_buf);
	}

	memcpy(bulk_out_buf, data, len);
	sys_cache_data_flush_range(bulk_out_buf, sizeof(bulk_out_buf));

	bulk_ring_enqueue(bulk_out_ring, BULK_RING_SIZE,
			  &bulk_out_cycle, &bulk_out_enq,
			  (uint32_t)bulk_out_buf, 0, (uint32_t)len,
			  TRB_TYPE(TRB_NORMAL) | TRB_IOC);

	xhci_ring_doorbell(cfg, slot_id, dci);

	evt = evt_ring_wait(cfg, 5000);
	if (evt == NULL) {
		LOG_ERR("Bulk OUT: no completion event");
		return -ETIMEDOUT;
	}

	uint8_t comp = TRB_COMP_CODE(evt->status);
	uint32_t residual = evt->status & 0xFFFFFF;

	evt_ring_advance();
	evt_ring_update_erdp(cfg);

	if (comp != TRB_COMP_SUCCESS && comp != TRB_COMP_SHORT_PKT) {
		LOG_ERR("Bulk OUT failed: comp=%u", comp);
		return -EIO;
	}

	return (int)(len - residual);
}

static int xhci_bulk_transfer_in(const struct uhc_dwc3_config *cfg,
				 uint8_t slot_id, uint8_t ep_addr,
				 uint8_t *data, size_t len)
{
	struct xhci_trb *evt;
	uint8_t dci = ep_to_dci(ep_addr);

	if (len > sizeof(bulk_in_buf)) {
		len = sizeof(bulk_in_buf);
	}

	memset(bulk_in_buf, 0, len);
	sys_cache_data_flush_range(bulk_in_buf, sizeof(bulk_in_buf));

	bulk_ring_enqueue(bulk_in_ring, BULK_RING_SIZE,
			  &bulk_in_cycle, &bulk_in_enq,
			  (uint32_t)bulk_in_buf, 0, (uint32_t)len,
			  TRB_TYPE(TRB_NORMAL) | TRB_IOC);

	xhci_ring_doorbell(cfg, slot_id, dci);

	evt = evt_ring_wait(cfg, 5000);
	if (evt == NULL) {
		LOG_ERR("Bulk IN: no completion event");
		return -ETIMEDOUT;
	}

	uint8_t comp = TRB_COMP_CODE(evt->status);
	uint32_t residual = evt->status & 0xFFFFFF;

	evt_ring_advance();
	evt_ring_update_erdp(cfg);

	if (comp != TRB_COMP_SUCCESS && comp != TRB_COMP_SHORT_PKT) {
		LOG_ERR("Bulk IN failed: comp=%u", comp);
		return -EIO;
	}

	sys_cache_data_invd_range(bulk_in_buf, sizeof(bulk_in_buf));
	int received = (int)(len - residual);

	if (data != NULL && received > 0) {
		memcpy(data, bulk_in_buf, received);
	}

	return received;
}

/* ---- Isochronous endpoint configuration ---- */

/*
 * Configure isochronous endpoints for audio streaming.
 * This issues a Configure Endpoint command to the xHCI controller
 * to set up isoch IN and/or OUT endpoints.
 *
 * @param isoch_out_ep  Isochronous OUT endpoint address (0 to skip)
 * @param isoch_in_ep   Isochronous IN endpoint address (0 to skip)
 * @param out_mps       Max packet size for OUT endpoint
 * @param in_mps        Max packet size for IN endpoint
 * @param interval      xHCI interval value (4 = 1ms for FS)
 */
static int xhci_configure_isoch_endpoint(const struct uhc_dwc3_config *cfg,
					 uint8_t slot_id,
					 uint8_t isoch_out_ep,
					 uint8_t isoch_in_ep,
					 uint16_t out_mps,
					 uint16_t in_mps,
					 uint8_t interval)
{
	struct xhci_trb *evt;
	uint8_t dci_in = isoch_in_ep ? ep_to_dci(isoch_in_ep) : 0;
	uint8_t dci_out = isoch_out_ep ? ep_to_dci(isoch_out_ep) : 0;
	uint8_t max_dci = (dci_in > dci_out) ? dci_in : dci_out;

	LOG_INF("Configure Isoch EP: out=0x%02x(DCI=%u) in=0x%02x(DCI=%u)",
		isoch_out_ep, dci_out, isoch_in_ep, dci_in);

	/* Init isoch rings */
	if (isoch_out_ep) {
		isoch_out_ring_init();
		sys_cache_data_flush_range(isoch_out_ring,
					   sizeof(isoch_out_ring));
	}
	if (isoch_in_ep) {
		isoch_in_ring_init();
		sys_cache_data_flush_range(isoch_in_ring,
					   sizeof(isoch_in_ring));
	}

	/* Build Input Context */
	memset(input_ctx_buf, 0, sizeof(input_ctx_buf));

	struct xhci_input_ctrl_ctx *icc = (void *)input_ctx_buf;

	icc->add_flags = BIT(0); /* Slot context */
	if (dci_out) {
		icc->add_flags |= BIT(dci_out);
	}
	if (dci_in) {
		icc->add_flags |= BIT(dci_in);
	}

	/* Slot Context: update Context Entries */
	uint32_t *slot = (uint32_t *)(input_ctx_buf + 64);

	sys_cache_data_invd_range(dev_ctx_buf, sizeof(dev_ctx_buf));
	memcpy(slot, dev_ctx_buf, 64);
	slot[0] = (slot[0] & ~(0x1F << 27)) | (max_dci << 27);

	/* Isoch OUT EP Context */
	if (dci_out) {
		uint32_t *ep = (uint32_t *)(input_ctx_buf + 64 * (1 + dci_out));

		/*
		 * Field 0: Interval, Mult=0, MaxPStreams=0, LSA=0
		 * For FS isoch, interval=4 means 2^(4-1)*125us = 1ms
		 * CErr must be 0 for isoch endpoints (xHCI spec 4.8.2.1)
		 */
		ep[0] = ((uint32_t)interval << 16);
		ep[1] = (0 << 1) |                      /* CErr=0 for isoch */
			(XHCI_EP_TYPE_ISOCH_OUT << 3) |
			((uint32_t)out_mps << 16);
		ep[2] = (uint32_t)isoch_out_ring | 1;    /* DCS=1 */
		ep[3] = 0;
		ep[4] = out_mps; /* Average TRB Length = MPS */
	}

	/* Isoch IN EP Context */
	if (dci_in) {
		uint32_t *ep = (uint32_t *)(input_ctx_buf + 64 * (1 + dci_in));

		ep[0] = ((uint32_t)interval << 16);
		ep[1] = (0 << 1) |                     /* CErr=0 for isoch */
			(XHCI_EP_TYPE_ISOCH_IN << 3) |
			((uint32_t)in_mps << 16);
		ep[2] = (uint32_t)isoch_in_ring | 1;
		ep[3] = 0;
		ep[4] = in_mps;
	}

	sys_cache_data_flush_range(input_ctx_buf, sizeof(input_ctx_buf));

	cmd_ring_enqueue((uint32_t)input_ctx_buf, 0, 0,
			 TRB_TYPE(TRB_CONFIG_EP) | (slot_id << 24));

	xhci_ring_doorbell(cfg, 0, 0);

	evt = evt_ring_wait(cfg, 5000);
	if (evt == NULL) {
		LOG_ERR("Configure Isoch EP: no completion event");
		return -ETIMEDOUT;
	}

	uint8_t comp = TRB_COMP_CODE(evt->status);

	LOG_INF("Configure Isoch EP event: comp=%u", comp);

	evt_ring_advance();
	evt_ring_update_erdp(cfg);

	if (comp != TRB_COMP_SUCCESS) {
		LOG_ERR("Configure Isoch EP failed: comp=%u", comp);
		return -EIO;
	}

	return 0;
}

/* ---- Isochronous Transfers ---- */

/*
 * Double-buffered isochronous OUT transfer.
 *
 * The key insight: with SIA (Start Isoch ASAP), if we only have 1 TRB
 * pending, by the time we get the completion and submit the next one,
 * the next frame boundary has already passed → 2ms/frame = half speed.
 *
 * Solution: always keep 2 TRBs queued. After getting a completion,
 * immediately queue another so there's always one pending for the
 * next frame.
 *
 * State: isoch_out_started tracks if we've primed the double buffer.
 */
static bool isoch_out_started;

static int xhci_isoch_out_prime(const struct uhc_dwc3_config *cfg,
				uint8_t slot_id, uint8_t out_ep,
				const uint8_t *data0, const uint8_t *data1,
				size_t len)
{
	uint8_t dci = ep_to_dci(out_ep);

	/* Fill first buffer with provided data, rest with silence */
	memcpy(isoch_out_bufs[0], data0, len);
	sys_cache_data_flush_range(isoch_out_bufs[0], ISOCH_OUT_BUF_SIZE);

	memcpy(isoch_out_bufs[1], data1, len);
	sys_cache_data_flush_range(isoch_out_bufs[1], ISOCH_OUT_BUF_SIZE);

	memset(isoch_out_bufs[2], 0, len);
	sys_cache_data_flush_range(isoch_out_bufs[2], ISOCH_OUT_BUF_SIZE);

	memset(isoch_out_bufs[3], 0, len);
	sys_cache_data_flush_range(isoch_out_bufs[3], ISOCH_OUT_BUF_SIZE);

	/* Queue 4 TRBs — all with SIA (DWC3 requires it) */
	for (int i = 0; i < ISOCH_NUM_BUFS; i++) {
		bulk_ring_enqueue(isoch_out_ring, ISOCH_RING_SIZE,
				  &isoch_out_cycle, &isoch_out_enq,
				  (uint32_t)isoch_out_bufs[i], 0,
				  (uint32_t)len,
				  TRB_TYPE(TRB_ISOCH) | TRB_IOC | TRB_SIA);
	}

	xhci_ring_doorbell(cfg, slot_id, dci);

	isoch_out_buf_idx = 0; /* Next completion will be buf[0] */
	isoch_out_started = true;

	/* Disable xHCI interrupt during streaming — we poll completions
	 * directly. The ISR would just waste cycles and cause jitter.
	 */
	uint32_t usbcmd = xhci_op_read(cfg, XHCI_USBCMD);

	xhci_op_write(cfg, XHCI_USBCMD, usbcmd & ~XHCI_CMD_INTE);

	return 0;
}

/*
 * Steady-state OUT: wait for oldest completion (frees a buffer),
 * then fill and enqueue the new frame. With 4 primed TRBs, after
 * one completion there are still 3 queued → plenty of headroom.
 * We must wait FIRST to avoid overwriting a DMA buffer still in use.
 */
static int xhci_isoch_out_pump(const struct uhc_dwc3_config *cfg,
			       uint8_t slot_id, uint8_t out_ep,
			       const uint8_t *next_data, size_t len)
{
	uint8_t dci = ep_to_dci(out_ep);
	static uint32_t last_time;
	static int pump_count;
	static int skip_count;

	/* Step 1: Wait for the oldest TRB completion (paces at ~1ms) */
	struct xhci_trb *evt = NULL;

	for (int elapsed = 0; elapsed < 20000; elapsed++) {
		uint32_t sts = xhci_op_read(cfg, XHCI_USBSTS);

		if (sts & XHCI_STS_EINT) {
			xhci_op_write(cfg, XHCI_USBSTS, XHCI_STS_EINT);
		}
		uint32_t iman = xhci_rt_read(cfg, XHCI_IMAN);

		if (iman & BIT(0)) {
			xhci_rt_write(cfg, XHCI_IMAN, iman | BIT(0));
		}

		evt = evt_ring_peek();
		if (evt != NULL) {
			break;
		}
		k_busy_wait(10);
	}

	if (evt == NULL) {
		LOG_ERR("Isoch OUT: no event");
		return -ETIMEDOUT;
	}

	uint8_t comp = TRB_COMP_CODE(evt->status);
	uint32_t residual = evt->status & 0xFFFFFF;

	evt_ring_advance();
	evt_ring_update_erdp(cfg);

	/* Timing diagnostic */
	uint32_t now = k_cycle_get_32();
	uint32_t delta_cycles = now - last_time;
	uint32_t delta_us = k_cyc_to_us_floor32(delta_cycles);

	last_time = now;
	pump_count++;

	if (comp == TRB_COMP_RING_UNDERRUN) {
		skip_count++;
	}

	/* Log every 1000 pumps (buffered — drained when streaming stops) */
	if ((pump_count % 1000) == 0) {
		LOG_INF("Isoch pump: %d calls, %d underruns, last_delta=%u us",
			pump_count, skip_count, delta_us);
	}

	if (comp != TRB_COMP_SUCCESS && comp != TRB_COMP_SHORT_PKT &&
	    comp != TRB_COMP_RING_UNDERRUN) {
		LOG_ERR("Isoch OUT comp=%u", comp);
		return -EIO;
	}

	/* Step 2: The completed buffer slot is now safe to reuse */
	uint8_t idx = isoch_out_buf_idx;

	memcpy(isoch_out_bufs[idx], next_data, len);
	sys_cache_data_flush_range(isoch_out_bufs[idx], ISOCH_OUT_BUF_SIZE);

	bulk_ring_enqueue(isoch_out_ring, ISOCH_RING_SIZE,
			  &isoch_out_cycle, &isoch_out_enq,
			  (uint32_t)isoch_out_bufs[idx], 0,
			  (uint32_t)len,
			  TRB_TYPE(TRB_ISOCH) | TRB_IOC | TRB_SIA);

	xhci_ring_doorbell(cfg, slot_id, dci);

	isoch_out_buf_idx = (idx + 1) % ISOCH_NUM_BUFS;

	return (comp == TRB_COMP_RING_UNDERRUN) ? 0 : (int)(len - residual);
}

/*
 * Simple single-frame isochronous IN transfer (mic doesn't need
 * double-buffering since we're not sensitive to its timing).
 */
static int xhci_isoch_in_single(const struct uhc_dwc3_config *cfg,
				uint8_t slot_id, uint8_t in_ep,
				uint8_t *data, size_t len)
{
	uint8_t dci = ep_to_dci(in_ep);

	if (len > ISOCH_IN_BUF_SIZE) {
		len = ISOCH_IN_BUF_SIZE;
	}

	memset(isoch_in_bufs[0], 0, len);
	sys_cache_data_flush_range(isoch_in_bufs[0], ISOCH_IN_BUF_SIZE);

	bulk_ring_enqueue(isoch_in_ring, ISOCH_RING_SIZE,
			  &isoch_in_cycle, &isoch_in_enq,
			  (uint32_t)isoch_in_bufs[0], 0,
			  (uint32_t)len,
			  TRB_TYPE(TRB_ISOCH) | TRB_IOC | TRB_SIA);

	xhci_ring_doorbell(cfg, slot_id, dci);

	struct xhci_trb *evt = NULL;

	for (int elapsed = 0; elapsed < 20000; elapsed++) {
		uint32_t sts = xhci_op_read(cfg, XHCI_USBSTS);

		if (sts & XHCI_STS_EINT) {
			xhci_op_write(cfg, XHCI_USBSTS, XHCI_STS_EINT);
		}
		uint32_t iman = xhci_rt_read(cfg, XHCI_IMAN);

		if (iman & BIT(0)) {
			xhci_rt_write(cfg, XHCI_IMAN, iman | BIT(0));
		}

		evt = evt_ring_peek();
		if (evt != NULL) {
			break;
		}
		k_busy_wait(10);
	}

	if (evt == NULL) {
		return -ETIMEDOUT;
	}

	uint8_t comp = TRB_COMP_CODE(evt->status);
	uint32_t residual = evt->status & 0xFFFFFF;

	evt_ring_advance();
	evt_ring_update_erdp(cfg);

	if (comp == TRB_COMP_SUCCESS || comp == TRB_COMP_SHORT_PKT) {
		sys_cache_data_invd_range(isoch_in_bufs[0], ISOCH_IN_BUF_SIZE);
		int rcvd = (int)(len - residual);

		if (data && rcvd > 0) {
			memcpy(data, isoch_in_bufs[0], rcvd);
		}
		return rcvd;
	} else if (comp == TRB_COMP_RING_OVERRUN) {
		return 0;
	}

	LOG_ERR("Isoch IN comp=%u", comp);
	return -EIO;
}

/*
 * ---- Mic -> Speaker loopback ----
 *
 * Both the speaker (isoch OUT) and mic (isoch IN) endpoints post their
 * completions to the same event ring. We therefore cannot use the
 * single-endpoint OUT pump here (it assumes the next event is always its
 * own). Instead we poll the ring and dispatch each completion by its
 * Endpoint ID (DCI), carried in bits [20:16] of the event TRB control word.
 *
 * Data path: each mic frame is upmixed mono->stereo and copied into the
 * freed speaker buffer. The mic's actual sample count (from residual)
 * drives the speaker frame size, so the two device clocks stay matched.
 */
static int xhci_isoch_loopback_prime(const struct uhc_dwc3_config *cfg,
				     uint8_t slot_id, uint8_t out_ep,
				     uint8_t in_ep, size_t out_len,
				     size_t in_len)
{
	uint8_t out_dci = ep_to_dci(out_ep);
	uint8_t in_dci = ep_to_dci(in_ep);

	/* Prime speaker buffers with silence */
	for (int i = 0; i < ISOCH_NUM_BUFS; i++) {
		memset(isoch_out_bufs[i], 0, out_len);
		sys_cache_data_flush_range(isoch_out_bufs[i], ISOCH_OUT_BUF_SIZE);
		bulk_ring_enqueue(isoch_out_ring, ISOCH_RING_SIZE,
				  &isoch_out_cycle, &isoch_out_enq,
				  (uint32_t)isoch_out_bufs[i], 0,
				  (uint32_t)out_len,
				  TRB_TYPE(TRB_ISOCH) | TRB_IOC | TRB_SIA);
	}
	isoch_out_buf_idx = 0;

	/* Prime mic IN buffers */
	for (int i = 0; i < ISOCH_NUM_BUFS; i++) {
		bulk_ring_enqueue(isoch_in_ring, ISOCH_RING_SIZE,
				  &isoch_in_cycle, &isoch_in_enq,
				  (uint32_t)isoch_in_bufs[i], 0,
				  (uint32_t)in_len,
				  TRB_TYPE(TRB_ISOCH) | TRB_IOC | TRB_SIA);
	}
	isoch_in_buf_idx = 0;

	/* Last-good frame starts as silence */
	memset(isoch_lb_last_stereo, 0, ISOCH_OUT_BUF_SIZE);
	isoch_lb_last_nsamp = (int)(in_len / 2);

	xhci_ring_doorbell(cfg, slot_id, out_dci);
	xhci_ring_doorbell(cfg, slot_id, in_dci);

	isoch_lb_started = true;

	/* Poll completions directly — disable the xHCI interrupt */
	uint32_t usbcmd = xhci_op_read(cfg, XHCI_USBCMD);

	xhci_op_write(cfg, XHCI_USBCMD, usbcmd & ~XHCI_CMD_INTE);

	return 0;
}

static int xhci_isoch_loopback_pump(const struct uhc_dwc3_config *cfg,
				    uint8_t slot_id, uint8_t out_ep,
				    uint8_t in_ep, size_t out_len,
				    size_t in_len)
{
	uint8_t out_dci = ep_to_dci(out_ep);
	uint8_t in_dci = ep_to_dci(in_ep);
	bool out_done = false;

	/* Process events until we have produced exactly one speaker frame
	 * (the OUT completion paces us at ~1 ms). Any mic completions seen
	 * along the way are folded into the latest stereo frame.
	 */
	for (int spin = 0; spin < 40000 && !out_done; spin++) {
		uint32_t sts = xhci_op_read(cfg, XHCI_USBSTS);

		if (sts & XHCI_STS_EINT) {
			xhci_op_write(cfg, XHCI_USBSTS, XHCI_STS_EINT);
		}
		uint32_t iman = xhci_rt_read(cfg, XHCI_IMAN);

		if (iman & BIT(0)) {
			xhci_rt_write(cfg, XHCI_IMAN, iman | BIT(0));
		}

		struct xhci_trb *evt = evt_ring_peek();

		if (evt == NULL) {
			k_busy_wait(10);
			continue;
		}

		uint8_t ep = (evt->control >> 16) & 0x1F;
		uint8_t comp = TRB_COMP_CODE(evt->status);
		uint32_t residual = evt->status & 0xFFFFFF;

		evt_ring_advance();
		evt_ring_update_erdp(cfg);

		if (ep == in_dci) {
			/* Mic frame arrived — upmix mono->stereo */
			uint8_t bi = isoch_in_buf_idx;

			if (comp == TRB_COMP_SUCCESS ||
			    comp == TRB_COMP_SHORT_PKT) {
				sys_cache_data_invd_range(isoch_in_bufs[bi],
							  ISOCH_IN_BUF_SIZE);

				int n = (int)((in_len - residual) / 2);
				int16_t *m = (int16_t *)isoch_in_bufs[bi];
				int16_t *s = (int16_t *)isoch_lb_last_stereo;

				if (n > ISOCH_OUT_BUF_SIZE / 4) {
					n = ISOCH_OUT_BUF_SIZE / 4;
				}
				for (int i = 0; i < n; i++) {
					s[i * 2] = m[i];
					s[i * 2 + 1] = m[i];
				}
				isoch_lb_last_nsamp = n;
			}

			/* Re-queue this mic buffer */
			bulk_ring_enqueue(isoch_in_ring, ISOCH_RING_SIZE,
					  &isoch_in_cycle, &isoch_in_enq,
					  (uint32_t)isoch_in_bufs[bi], 0,
					  (uint32_t)in_len,
					  TRB_TYPE(TRB_ISOCH) | TRB_IOC |
					  TRB_SIA);
			xhci_ring_doorbell(cfg, slot_id, in_dci);
			isoch_in_buf_idx = (bi + 1) % ISOCH_NUM_BUFS;
		} else if (ep == out_dci) {
			/* Speaker buffer freed — fill with latest mic frame.
			 * Use the mic's sample count so output rate tracks
			 * the device ADC/DAC crystal (no drift).
			 */
			uint8_t bo = isoch_out_buf_idx;
			int n = isoch_lb_last_nsamp;
			size_t bytes = (size_t)n * 4; /* stereo 16-bit */

			if (bytes > out_len) {
				bytes = out_len;
			}
			memcpy(isoch_out_bufs[bo], isoch_lb_last_stereo, bytes);
			sys_cache_data_flush_range(isoch_out_bufs[bo],
						   ISOCH_OUT_BUF_SIZE);

			bulk_ring_enqueue(isoch_out_ring, ISOCH_RING_SIZE,
					  &isoch_out_cycle, &isoch_out_enq,
					  (uint32_t)isoch_out_bufs[bo], 0,
					  (uint32_t)bytes,
					  TRB_TYPE(TRB_ISOCH) | TRB_IOC |
					  TRB_SIA);
			xhci_ring_doorbell(cfg, slot_id, out_dci);
			isoch_out_buf_idx = (bo + 1) % ISOCH_NUM_BUFS;
			out_done = true;
		}
	}

	if (!out_done) {
		LOG_ERR("Isoch loopback: no OUT completion");
		return -ETIMEDOUT;
	}

	return 0;
}

/*
 * Public loopback entry: primes on first call, pumps thereafter.
 */
int uhc_dwc3_isoch_loopback(const struct device *dev)
{
	const struct uhc_dwc3_config *cfg = dev->config;
	struct uhc_dwc3_data *priv = uhc_get_private(dev);
	size_t out_len = ISOCH_OUT_BUF_SIZE; /* 192 = 48 stereo samples */
	size_t in_len = ISOCH_IN_BUF_SIZE;   /* 96 = 48 mono samples */

	if (!isoch_lb_started) {
		return xhci_isoch_loopback_prime(cfg, priv->slot_id,
						 priv->isoch_out_ep,
						 priv->isoch_in_ep,
						 out_len, in_len);
	}

	return xhci_isoch_loopback_pump(cfg, priv->slot_id,
					priv->isoch_out_ep,
					priv->isoch_in_ep,
					out_len, in_len);
}

/* ---- UHC API implementation ---- */

static int dwc3_lock(const struct device *dev)
{
	return uhc_lock_internal(dev, K_FOREVER);
}

static int dwc3_unlock(const struct device *dev)
{
	return uhc_unlock_internal(dev);
}

static int uhc_dwc3_init(const struct device *dev)
{
	const struct uhc_dwc3_config *cfg = dev->config;
	struct uhc_data *data = dev->data;

	/* DWC3 core init */
	dwc3_core_soft_reset(cfg->base);
	dwc3_configure_phy(cfg->base);
	dwc3_configure_host_params(cfg->base);
	dwc3_set_host_mode(cfg->base);

	/* xHCI controller init (includes xHCI reset) */
	int ret = xhci_controller_init(cfg);

	if (ret) {
		return ret;
	}

	/* Re-apply DWC3 host mode after xHCI reset may have cleared it */
	dwc3_set_host_mode(cfg->base);

	atomic_set_bit(&data->status, UHC_STATUS_INITIALIZED);

	volatile udc_dwc3_reg_t *regs = (void *)cfg->base;

	LOG_INF("DWC3 UHC initialized, SNPSID=0x%08x", regs->GSNPSID);

	return 0;
}

static int uhc_dwc3_enable(const struct device *dev)
{
	const struct uhc_dwc3_config *cfg = dev->config;
	struct uhc_data *data = dev->data;

	if (cfg->irq_enable_func) {
		cfg->irq_enable_func(dev);
	}

	atomic_set_bit(&data->status, UHC_STATUS_ENABLED);

	if (xhci_portsc_read(cfg) & XHCI_PORTSC_CCS) {
		uint8_t speed = XHCI_PORTSC_SPEED(xhci_portsc_read(cfg));

		if (speed == XHCI_SPEED_HS) {
			uhc_submit_event(dev, UHC_EVT_DEV_CONNECTED_HS, 0);
		} else {
			uhc_submit_event(dev, UHC_EVT_DEV_CONNECTED_FS, 0);
		}
	}

	return 0;
}

static int uhc_dwc3_disable(const struct device *dev)
{
	const struct uhc_dwc3_config *cfg = dev->config;
	struct uhc_data *data = dev->data;

	if (cfg->irq_disable_func) {
		cfg->irq_disable_func(dev);
	}

	atomic_clear_bit(&data->status, UHC_STATUS_ENABLED);

	return 0;
}

static int uhc_dwc3_shutdown(const struct device *dev)
{
	const struct uhc_dwc3_config *cfg = dev->config;
	struct uhc_data *data = dev->data;

	/* Stop the controller */
	xhci_op_write(cfg, XHCI_USBCMD, 0);
	atomic_clear_bit(&data->status, UHC_STATUS_INITIALIZED);

	return 0;
}

static int uhc_dwc3_bus_reset(const struct device *dev)
{
	const struct uhc_dwc3_config *cfg = dev->config;

	int ret = xhci_port_reset(cfg);

	if (ret == 0) {
		uhc_submit_event(dev, UHC_EVT_RESETED, 0);
	}

	return ret;
}

static int uhc_dwc3_sof_enable(const struct device *dev)
{
	return 0;
}

static int uhc_dwc3_bus_suspend(const struct device *dev)
{
	const struct uhc_dwc3_config *cfg = dev->config;
	uint32_t portsc = xhci_portsc_read(cfg);

	portsc &= XHCI_PORTSC_PRESERVE;
	portsc |= XHCI_PORTSC_PLS(3); /* U3/Suspend */
	xhci_portsc_write(cfg, portsc);

	uhc_submit_event(dev, UHC_EVT_SUSPENDED, 0);

	return 0;
}

static int uhc_dwc3_bus_resume(const struct device *dev)
{
	const struct uhc_dwc3_config *cfg = dev->config;
	uint32_t portsc = xhci_portsc_read(cfg);

	portsc &= XHCI_PORTSC_PRESERVE;
	portsc |= XHCI_PORTSC_PLS(0); /* U0/Active */
	xhci_portsc_write(cfg, portsc);

	k_busy_wait(20000);

	uhc_submit_event(dev, UHC_EVT_RESUMED, 0);

	return 0;
}

static int uhc_dwc3_ep_enqueue(const struct device *dev,
			       struct uhc_transfer *const xfer)
{
	struct uhc_dwc3_data *priv = uhc_get_private(dev);
	int ret;

	ret = uhc_xfer_append(dev, xfer);
	if (ret) {
		return ret;
	}

	k_sem_give(&priv->xfer_sem);

	return 0;
}

static int uhc_dwc3_ep_dequeue(const struct device *dev,
			       struct uhc_transfer *const xfer)
{
	xfer->err = -ECONNRESET;

	return 0;
}

/* Worker thread */
static void uhc_dwc3_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct uhc_dwc3_data *priv = uhc_get_private(dev);

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		struct uhc_transfer *xfer;

		k_sem_take(&priv->xfer_sem, K_FOREVER);

		if (!uhc_is_enabled(dev)) {
			continue;
		}

		xfer = uhc_xfer_get_next(dev);
		if (xfer == NULL) {
			continue;
		}

		uhc_xfer_return(dev, xfer, 0);
	}
}

/* ISR */
static void uhc_dwc3_isr(const struct device *dev)
{
	const struct uhc_dwc3_config *cfg = dev->config;
	struct uhc_dwc3_data *priv = uhc_get_private(dev);

	/* Clear USBSTS.EINT */
	uint32_t sts = xhci_op_read(cfg, XHCI_USBSTS);

	if (sts & XHCI_STS_EINT) {
		xhci_op_write(cfg, XHCI_USBSTS, XHCI_STS_EINT);
	}

	/* Clear IMAN.IP */
	uint32_t iman = xhci_rt_read(cfg, XHCI_IMAN);

	if (iman & BIT(0)) {
		xhci_rt_write(cfg, XHCI_IMAN, iman | BIT(0));
	}

	/* Check port status */
	uint32_t portsc = xhci_portsc_read(cfg);

	if (portsc & XHCI_PORTSC_CCS) {
		if (!priv->port_connected) {
			priv->port_connected = true;
			priv->port_speed = XHCI_PORTSC_SPEED(portsc);
			if (priv->port_speed == XHCI_SPEED_HS) {
				uhc_submit_event(dev, UHC_EVT_DEV_CONNECTED_HS, 0);
			} else {
				uhc_submit_event(dev, UHC_EVT_DEV_CONNECTED_FS, 0);
			}
		}
	} else {
		if (priv->port_connected && !priv->enumerating) {
			priv->port_connected = false;
			uhc_submit_event(dev, UHC_EVT_DEV_REMOVED, 0);
		}
	}

	k_sem_give(&priv->evt_sem);
}

/* ---- Device initialization ---- */

static int uhc_dwc3_driver_init(const struct device *dev)
{
	const struct uhc_dwc3_config *cfg = dev->config;
	struct uhc_data *data = dev->data;
	struct uhc_dwc3_data *priv = data->priv;
	int ret;

	if (cfg->clock_dev != NULL) {
		if (!device_is_ready(cfg->clock_dev)) {
			LOG_ERR("Clock device not ready");
			return -EIO;
		}

		ret = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
		if (ret != 0 && ret != -EALREADY) {
			LOG_ERR("Failed to enable clock: %d", ret);
			return ret;
		}
	}

	k_mutex_init(&data->mutex);
	k_sem_init(&priv->xfer_sem, 0, K_SEM_MAX_LIMIT);
	k_sem_init(&priv->evt_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&uhc_dwc3_thread_data, uhc_dwc3_stack,
			K_KERNEL_STACK_SIZEOF(uhc_dwc3_stack),
			uhc_dwc3_thread,
			(void *)dev, NULL, NULL,
			K_PRIO_COOP(2), 0, K_NO_WAIT);
	k_thread_name_set(&uhc_dwc3_thread_data, "uhc_dwc3");

	LOG_DBG("DWC3 UHC driver initialized");

	return 0;
}

/* ---- Public enumeration API ---- */

int uhc_dwc3_enumerate_device(const struct device *dev,
			      struct usb_device_descriptor *desc)
{
	const struct uhc_dwc3_config *cfg = dev->config;
	struct uhc_dwc3_data *priv = uhc_get_private(dev);
	uint8_t slot_id;
	int ret;

	/* Suppress ISR disconnect events during enumeration */
	priv->enumerating = true;

	/* Drain any stale events from previous attempts */
	LOG_INF("=== Step 1: Port Reset ===");
	for (int i = 0; i < EVT_RING_SIZE; i++) {
		struct xhci_trb *stale = evt_ring_peek();

		if (stale == NULL) {
			break;
		}
		LOG_INF("Draining stale event: type=%u comp=%u",
			TRB_TYPE_GET(stale->control),
			TRB_COMP_CODE(stale->status));
		evt_ring_advance();
	}
	evt_ring_update_erdp(cfg);

	/* Free any previously allocated slot */
	if (priv->slot_id != 0) {
		xhci_disable_slot(cfg, priv->slot_id);
		priv->slot_id = 0;
	}

	/* Re-init EP0 ring for clean transfer state */
	ep0_ring_init();
	memset(dev_ctx_buf, 0, sizeof(dev_ctx_buf));
	memset(input_ctx_buf, 0, sizeof(input_ctx_buf));
	sys_cache_data_flush_range(ep0_ring, sizeof(ep0_ring));
	sys_cache_data_flush_range(dev_ctx_buf, sizeof(dev_ctx_buf));
	sys_cache_data_flush_range(input_ctx_buf, sizeof(input_ctx_buf));

	/* Wait for port to stabilize before reset (OTG adapters can be slow) */
	for (int i = 0; i < 10; i++) {
		uint32_t ps = xhci_portsc_read(cfg);

		if (ps & XHCI_PORTSC_CCS) {
			break;
		}
		LOG_INF("Waiting for CCS... PORTSC=0x%08x (%d/10)", ps, i + 1);
		k_sleep(K_MSEC(100));
	}

	ret = xhci_port_reset(cfg);
	if (ret) {
		priv->enumerating = false;
		return ret;
	}

	uint8_t port_speed = priv_port_speed;

	LOG_INF("Port speed after reset: %u (%s)", port_speed,
		port_speed == XHCI_SPEED_HS ? "High Speed" :
		port_speed == XHCI_SPEED_FS ? "Full Speed" :
		port_speed == XHCI_SPEED_LS ? "Low Speed" : "Unknown");

	/* Step 2: Enable Slot */
	LOG_INF("=== Step 2: Enable Slot ===");
	ret = xhci_enable_slot(cfg, &slot_id);
	if (ret) {
		priv->enumerating = false;
		return ret;
	}
	priv->slot_id = slot_id;

	/* Step 3: Address Device (retry up to 3 times for flaky connections) */
	LOG_INF("=== Step 3: Address Device ===");
	for (int attempt = 0; attempt < 3; attempt++) {
		if (attempt > 0) {
			LOG_INF("Address Device retry %d/3", attempt + 1);
			/* Re-init EP0 ring and contexts for retry */
			ep0_ring_init();
			memset(dev_ctx_buf, 0, sizeof(dev_ctx_buf));
			memset(input_ctx_buf, 0, sizeof(input_ctx_buf));
			sys_cache_data_flush_range(ep0_ring, sizeof(ep0_ring));
			sys_cache_data_flush_range(dev_ctx_buf,
						   sizeof(dev_ctx_buf));
			sys_cache_data_flush_range(input_ctx_buf,
						   sizeof(input_ctx_buf));
			k_sleep(K_MSEC(100));
		}
		ret = xhci_address_device(cfg, slot_id, port_speed);
		if (ret == 0) {
			break;
		}
	}
	if (ret) {
		priv->enumerating = false;
		return ret;
	}

	/* Step 4: Get Device Descriptor.
	 *
	 * Two-phase to handle any bMaxPacketSize0 (8/16/32/64):
	 *  (a) Read the first 8 bytes — enough to reach bMaxPacketSize0 —
	 *      with the default EP0 MPS set during Address Device.
	 *  (b) If the device's real MPS0 differs, update the EP0 context via
	 *      Evaluate Context, then re-read the full 18-byte descriptor.
	 *
	 * Without this, an 8-byte-MPS device (e.g. FTDI) under a 64-byte EP0
	 * context truncates at the first packet (short pkt after 8 bytes).
	 */
	LOG_INF("=== Step 4: Get Device Descriptor ===");
	ret = xhci_get_device_descriptor(cfg, slot_id, desc, 8);
	if (ret) {
		priv->enumerating = false;
		return ret;
	}

	uint16_t real_mps = desc->bMaxPacketSize0;
	uint16_t cur_mps = (port_speed == XHCI_SPEED_LS) ? 8 : 64;

	if (real_mps == 8 || real_mps == 16 || real_mps == 32 ||
	    real_mps == 64) {
		if (real_mps != cur_mps) {
			ret = xhci_evaluate_context_mps(cfg, slot_id, real_mps);
			if (ret) {
				priv->enumerating = false;
				return ret;
			}
		}
	} else {
		LOG_WRN("Unexpected bMaxPacketSize0=%u, keeping %u",
			real_mps, cur_mps);
	}

	/* Re-read the full descriptor now that EP0 MPS is correct */
	ret = xhci_get_device_descriptor(cfg, slot_id, desc, 18);
	if (ret) {
		priv->enumerating = false;
		return ret;
	}

	priv->enumerating = false;

	LOG_INF("=== Device Descriptor ===");
	LOG_INF("  bLength:         %u", desc->bLength);
	LOG_INF("  bDescriptorType: %u", desc->bDescriptorType);
	LOG_INF("  bcdUSB:          0x%04x", sys_le16_to_cpu(desc->bcdUSB));
	LOG_INF("  bDeviceClass:    %u", desc->bDeviceClass);
	LOG_INF("  bDeviceSubClass: %u", desc->bDeviceSubClass);
	LOG_INF("  bDeviceProtocol: %u", desc->bDeviceProtocol);
	LOG_INF("  bMaxPacketSize0: %u", desc->bMaxPacketSize0);
	LOG_INF("  idVendor:        0x%04x", sys_le16_to_cpu(desc->idVendor));
	LOG_INF("  idProduct:       0x%04x", sys_le16_to_cpu(desc->idProduct));
	LOG_INF("  bcdDevice:       0x%04x", sys_le16_to_cpu(desc->bcdDevice));
	LOG_INF("  bNumConfigurations: %u", desc->bNumConfigurations);

	return 0;
}

/*
 * Full device setup: enumerate + read config descriptor + set configuration
 * + configure bulk endpoints. Returns bulk endpoint addresses.
 */
int uhc_dwc3_setup_device(const struct device *dev,
			  struct usb_device_descriptor *desc,
			  uint8_t *out_bulk_in_ep,
			  uint8_t *out_bulk_out_ep)
{
	const struct uhc_dwc3_config *cfg = dev->config;
	struct uhc_dwc3_data *priv = uhc_get_private(dev);
	int ret;

	/* Enumerate first */
	ret = uhc_dwc3_enumerate_device(dev, desc);
	if (ret) {
		return ret;
	}

	uint8_t slot_id = priv->slot_id;

	/* Step 5: Get Configuration Descriptor */
	LOG_INF("=== Step 5: Get Config Descriptor ===");

	/* First, get just the 9-byte config descriptor header */
	uint8_t cfg_hdr[9];

	ret = xhci_control_transfer(cfg, slot_id,
				    0x80, 0x06, 0x0200, 0, 9, cfg_hdr);
	if (ret < 0) {
		LOG_ERR("Get Config Descriptor header failed: %d", ret);
		return ret;
	}

	uint16_t total_len = cfg_hdr[2] | (cfg_hdr[3] << 8);

	LOG_INF("Config descriptor: total_len=%u num_ifaces=%u", total_len, cfg_hdr[4]);

	if (total_len > sizeof(xfer_data_buf)) {
		total_len = sizeof(xfer_data_buf);
	}

	/* Now get the full configuration descriptor */
	uint8_t full_cfg[256];

	ret = xhci_control_transfer(cfg, slot_id,
				    0x80, 0x06, 0x0200, 0, total_len, full_cfg);
	if (ret < 0) {
		LOG_ERR("Get full Config Descriptor failed: %d", ret);
		return ret;
	}

	/* Parse the config descriptor to find bulk endpoints */
	uint8_t bulk_in = 0, bulk_out = 0;
	uint16_t bulk_mps = 64;
	int offset = 0;

	while (offset < ret) {
		uint8_t len = full_cfg[offset];
		uint8_t type = full_cfg[offset + 1];

		if (len == 0) {
			break;
		}

		if (type == USB_DESC_INTERFACE && (offset + 9) <= ret) {
			LOG_INF("  Interface %u: class=0x%02x sub=0x%02x proto=0x%02x eps=%u",
				full_cfg[offset + 2], full_cfg[offset + 5],
				full_cfg[offset + 6], full_cfg[offset + 7],
				full_cfg[offset + 4]);
		}

		if (type == USB_DESC_ENDPOINT && (offset + 7) <= ret) {
			uint8_t ep_addr = full_cfg[offset + 2];
			uint8_t ep_attr = full_cfg[offset + 3];
			uint16_t ep_mps = full_cfg[offset + 4] | (full_cfg[offset + 5] << 8);

			LOG_INF("  Endpoint 0x%02x: attr=0x%02x mps=%u",
				ep_addr, ep_attr, ep_mps);

			/* Check for bulk endpoint (attr bits [1:0] = 0x02) */
			if ((ep_attr & 0x03) == 0x02) {
				if (ep_addr & 0x80) {
					bulk_in = ep_addr;
				} else {
					bulk_out = ep_addr;
				}
				bulk_mps = ep_mps;
			}
		}

		offset += len;
	}

	if (bulk_in == 0 || bulk_out == 0) {
		LOG_ERR("Bulk endpoints not found (in=0x%02x out=0x%02x)", bulk_in, bulk_out);
		return -ENOTSUP;
	}

	*out_bulk_in_ep = bulk_in;
	*out_bulk_out_ep = bulk_out;

	/* Step 6: Set Configuration */
	LOG_INF("=== Step 6: Set Configuration ===");
	uint8_t cfg_val = full_cfg[5]; /* bConfigurationValue */

	ret = xhci_control_transfer(cfg, slot_id,
				    0x00, 0x09, cfg_val, 0, 0, NULL);
	if (ret < 0) {
		LOG_ERR("Set Configuration failed: %d", ret);
		return ret;
	}
	LOG_INF("Configuration %u set", cfg_val);

	/* Step 7: Configure Endpoint (xHCI) */
	LOG_INF("=== Step 7: Configure Endpoints ===");

	/* Use the max packet size advertised by the endpoint descriptor
	 * (64 for Full Speed, 512 for High Speed bulk).
	 */
	ret = xhci_configure_endpoint(cfg, slot_id, bulk_in, bulk_out, bulk_mps);
	if (ret) {
		return ret;
	}

	LOG_INF("Device fully configured: bulk_in=0x%02x bulk_out=0x%02x",
		bulk_in, bulk_out);

	priv->bulk_in_ep = bulk_in;
	priv->bulk_out_ep = bulk_out;

	return 0;
}

/*
 * Perform a bulk OUT transfer.
 */
int uhc_dwc3_bulk_out(const struct device *dev,
		      const uint8_t *data, size_t len)
{
	const struct uhc_dwc3_config *cfg = dev->config;
	struct uhc_dwc3_data *priv = uhc_get_private(dev);

	return xhci_bulk_transfer_out(cfg, priv->slot_id,
				      priv->bulk_out_ep, data, len);
}

/*
 * Perform a bulk IN transfer.
 */
int uhc_dwc3_bulk_in(const struct device *dev,
		     uint8_t *data, size_t len)
{
	const struct uhc_dwc3_config *cfg = dev->config;
	struct uhc_dwc3_data *priv = uhc_get_private(dev);

	return xhci_bulk_transfer_in(cfg, priv->slot_id,
				     priv->bulk_in_ep, data, len);
}

/*
 * Public wrapper for EP0 control transfers.
 */
int uhc_dwc3_control_transfer(const struct device *dev,
			      uint8_t bmRequestType,
			      uint8_t bRequest,
			      uint16_t wValue,
			      uint16_t wIndex,
			      uint16_t wLength,
			      void *data)
{
	const struct uhc_dwc3_config *cfg = dev->config;
	struct uhc_dwc3_data *priv = uhc_get_private(dev);

	return xhci_control_transfer(cfg, priv->slot_id,
				     bmRequestType, bRequest,
				     wValue, wIndex, wLength, data);
}

/*
 * Configure isochronous endpoints for audio streaming.
 */
int uhc_dwc3_configure_isoch(const struct device *dev,
			     uint8_t isoch_out_ep,
			     uint8_t isoch_in_ep,
			     uint16_t out_mps,
			     uint16_t in_mps)
{
	const struct uhc_dwc3_config *cfg = dev->config;
	struct uhc_dwc3_data *priv = uhc_get_private(dev);

	priv->isoch_out_ep = isoch_out_ep;
	priv->isoch_in_ep = isoch_in_ep;

	/* For FS isoch with bInterval=1: xHCI interval = 3
	 * This DWC3 uses 2^Interval * 125us:
	 *   interval=3 → 2^3 * 125us = 1000us = 1ms ✓
	 * (interval=4 gives 2ms = half-speed playback)
	 */
	return xhci_configure_isoch_endpoint(cfg, priv->slot_id,
					     isoch_out_ep, isoch_in_ep,
					     out_mps, in_mps, 3);
}

/*
 * Send audio data via isochronous OUT transfer (double-buffered).
 * First call primes the double buffer, subsequent calls pump one frame.
 */
int uhc_dwc3_isoch_out(const struct device *dev,
		       const uint8_t *data, size_t len)
{
	const struct uhc_dwc3_config *cfg = dev->config;
	struct uhc_dwc3_data *priv = uhc_get_private(dev);

	if (!isoch_out_started) {
		/* Prime: need 2 frames. Send the same data twice to start. */
		return xhci_isoch_out_prime(cfg, priv->slot_id,
					    priv->isoch_out_ep,
					    data, data, len);
	}

	return xhci_isoch_out_pump(cfg, priv->slot_id,
				   priv->isoch_out_ep, data, len);
}

/*
 * Receive audio data via isochronous IN transfer.
 */
int uhc_dwc3_isoch_in(const struct device *dev,
		      uint8_t *data, size_t len)
{
	const struct uhc_dwc3_config *cfg = dev->config;
	struct uhc_dwc3_data *priv = uhc_get_private(dev);

	return xhci_isoch_in_single(cfg, priv->slot_id,
				    priv->isoch_in_ep, data, len);
}

/*
 * Combined isochronous OUT+IN frame transfer.
 */
int uhc_dwc3_isoch_stream(const struct device *dev,
			  const uint8_t *out_data, size_t out_len,
			  uint8_t *in_data, size_t in_len,
			  int *out_ret, int *in_ret)
{
	int o_ret = uhc_dwc3_isoch_out(dev, out_data, out_len);
	int i_ret = uhc_dwc3_isoch_in(dev, in_data, in_len);

	if (out_ret) {
		*out_ret = o_ret;
	}
	if (in_ret) {
		*in_ret = i_ret;
	}

	/* Return error only on fatal failure */
	if (o_ret < 0 && o_ret != -EIO) {
		return o_ret;
	}
	if (i_ret < 0 && i_ret != -EIO) {
		return i_ret;
	}
	return 0;
}

/*
 * Start isochronous streaming (resets double-buffer state).
 */
int uhc_dwc3_isoch_start(const struct device *dev, size_t out_frame_size,
			 size_t in_frame_size)
{
	ARG_UNUSED(out_frame_size);
	ARG_UNUSED(in_frame_size);

	isoch_out_started = false;
	isoch_out_buf_idx = 0;
	isoch_in_buf_idx = 0;
	isoch_lb_started = false;

	LOG_INF("Isoch streaming ready (double-buffered OUT)");
	return 0;
}

/* ---- UHC API struct ---- */

static const struct uhc_api dwc3_uhc_api = {
	.lock = dwc3_lock,
	.unlock = dwc3_unlock,
	.init = uhc_dwc3_init,
	.enable = uhc_dwc3_enable,
	.disable = uhc_dwc3_disable,
	.shutdown = uhc_dwc3_shutdown,
	.bus_reset = uhc_dwc3_bus_reset,
	.sof_enable = uhc_dwc3_sof_enable,
	.bus_suspend = uhc_dwc3_bus_suspend,
	.bus_resume = uhc_dwc3_bus_resume,
	.ep_enqueue = uhc_dwc3_ep_enqueue,
	.ep_dequeue = uhc_dwc3_ep_dequeue,
};

/*
 * Instantiate for snps,dwc3-host compatible nodes.
 * This uses a separate compatible from the UDC (device) driver
 * which uses snps,dwc3, avoiding any DT_DRV_COMPAT conflicts.
 */
#define UHC_DWC3_DEVICE_DEFINE(inst)						\
	static void uhc_dwc3_irq_enable_##inst(const struct device *dev)	\
	{									\
		IRQ_CONNECT(DT_INST_IRQN(inst),					\
			    DT_INST_IRQ(inst, priority),			\
			    uhc_dwc3_isr,					\
			    DEVICE_DT_INST_GET(inst), 0);			\
		irq_enable(DT_INST_IRQN(inst));					\
	}									\
										\
	static void uhc_dwc3_irq_disable_##inst(const struct device *dev)	\
	{									\
		irq_disable(DT_INST_IRQN(inst));				\
	}									\
										\
	static struct uhc_dwc3_data uhc_dwc3_priv_##inst;			\
										\
	static struct uhc_data uhc_dwc3_data_##inst = {				\
		.priv = &uhc_dwc3_priv_##inst,					\
	};									\
										\
	static const struct uhc_dwc3_config uhc_dwc3_cfg_##inst = {		\
		.base = DT_INST_REG_ADDR(inst),					\
		.irq_enable_func = uhc_dwc3_irq_enable_##inst,			\
		.irq_disable_func = uhc_dwc3_irq_disable_##inst,		\
		.clock_dev = DEVICE_DT_GET_OR_NULL(				\
			DT_INST_CLOCKS_CTLR(inst)),				\
		.clock_subsys = (clock_control_subsys_t)			\
			DT_INST_CLOCKS_CELL_BY_IDX(inst, 0, clkid),		\
	};									\
										\
	DEVICE_DT_INST_DEFINE(inst, uhc_dwc3_driver_init, NULL,			\
			      &uhc_dwc3_data_##inst,				\
			      &uhc_dwc3_cfg_##inst,				\
			      POST_KERNEL, 99,					\
			      &dwc3_uhc_api);

DT_INST_FOREACH_STATUS_OKAY(UHC_DWC3_DEVICE_DEFINE)
