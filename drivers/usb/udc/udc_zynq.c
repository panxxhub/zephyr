/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * USB device controller (UDC) driver zynq
 *
 * This is a zynq for a device controller driver using the UDC API.
 * Please use it as a starting point for a driver implementation for your
 * USB device controller. Maintaining a common style, terminology and
 * abbreviations will allow us to speed up reviews and reduce maintenance.
 * Copy UDC driver zynq, remove all unrelated comments and replace the
 * copyright notice with your own.
 *
 * Typically, a driver implementation contains only a single source file,
 * but the large list of e.g. register definitions should be in a separate
 * .h file.
 *
 * If you want to define a helper macro, check if there is something similar
 * in include/zephyr/sys/util.h or include/zephyr/usb/usb_ch9.h that you can use.
 * Please keep all identifiers and logging messages concise and clear.
 */

#include "udc_common.h"
#include "zephyr/arch/arm/cortex_a_r/sys_io.h"
#include "zephyr/device.h"
#include "zephyr/sys/device_mmio.h"
#include "zephyr/sys/sys_io.h"
#include "zephyr/toolchain.h"
#include "udc_zynq.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>

#include <zephyr/logging/log.h>
#include <sys/cdefs.h>
#include <sys/types.h>
LOG_MODULE_REGISTER(udc_zynq, CONFIG_UDC_DRIVER_LOG_LEVEL);

#define USB_EP_LUT_IDX(ep) (USB_EP_DIR_IS_IN(ep) ? (ep & BIT_MASK(4)) + 16 : ep & BIT_MASK(4))

/* The following type definitions are used for referencing Queue Heads and
 * Transfer Descriptors. The structures themselves are not used, however, the
 * types are used in the API to avoid using (void *) pointers.
 */

struct dqh_aligned {
	struct zynq_udc_dqh dqh;
	uint8_t padding[64 - sizeof(struct zynq_udc_dqh)];
};
struct dtd_aligned {
	struct zynq_udc_dtd dtd;
	uint32_t user_data;
};

typedef struct ep_priv {
	struct zynq_udc_dqh *dqh;
	struct zynq_udc_dtd *dtds[4];
	union {
		struct out_priv {
			struct zynq_udc_dtd *td_curr;
			uint8_t *td_bufs;
		} out;
		struct in_priv {
			struct zynq_udc_dtd *td_head;
			struct zynq_udc_dtd *td_tail;
		} in;
	};
	// sys list, we for use
	uint32_t requested_bytes;
	uint32_t bytes_xfered;
	uint8_t *buf;
} ep_priv_t;

enum zynq_drv_event_type {
	/* Trigger next transfer, must not be used for control OUT */
	ZYNQ_DRV_EVT_XFER,
	/* Setup packet received */
	ZYNQ_DRV_EVT_SETUP,
	/* OUT transaction for specific endpoint is finished */
	ZYNQ_DRV_EVT_DOUT,
	/* IN transaction for specific endpoint is finished */
	ZYNQ_DRV_EVT_DIN,
};

struct zynq_drv_event {
	const struct device *dev;
	enum zynq_drv_event_type type;
	uint32_t bcnt;
	uint8_t ep;
};

K_MSGQ_DEFINE(drv_msgq, sizeof(struct zynq_drv_event), CONFIG_UDC_ZYNQ_MAX_QMESSAGES,
	      sizeof(void *));


/**
 * @brief 
 * 
 */
typedef void (*zynq_udc_irq_config_func)(const struct device *dev, bool en);
/*
 * Structure for holding controller configuration items that can remain in
 * non-volatile memory. This is usually accessed as
 *   const struct udc_zynq_config *config = dev->config;
 */
struct udc_zynq_config {
	size_t num_of_eps;
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
	void (*make_thread)(const struct device *dev);
	int speed_idx;
	struct dqh_aligned *qh_arr;
	struct dtd_aligned *td_arr;

	zynq_udc_irq_config_func config_func;
};

/*
 * Structure to hold driver private data.
 * Note that this is not accessible via dev->data, but as
 * struct udc_zynq_data *priv = udc_get_private(dev);
 * since the ep0 won't be enqueued from upper layer, we need to prepare the buffer here, in case we
 * have setup with data out stage
 */
struct zynq_udc_data {
	struct k_thread thread_data;

	struct ep_priv *ep_priv_in;
	struct ep_priv *ep_priv_out;
};

#define DEV_PRIV(dev) ((struct zynq_udc_data *)udc_get_private(dev))
#define DEV_CFG(dev)  ((const struct udc_zynq_config *)dev->config)
#define PRIV_SETUP(dev) (DEV_PRIV(dev)->ep_priv_out->dqh->setup_buffer)

static ALWAYS_INLINE void zynq_usb_set_bits(const struct device *dev, uint16_t reg_offset,
					    uint32_t bits)
{
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	sys_write32(sys_read32(base + reg_offset) | bits, base + reg_offset);
}

static ALWAYS_INLINE void zynq_usb_mask_write(const struct device *dev, uint16_t reg_offset,
					      uint32_t mask, uint32_t val)
{
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	mm_reg_t reg = base + reg_offset;
	sys_write32((sys_read32(reg) & ~mask) | (val & mask), reg);
}

static ALWAYS_INLINE void zynq_usb_clear_bits(const struct device *dev, uint16_t reg_offset,
					      uint32_t bits)
{
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	sys_write32(sys_read32(base + reg_offset) & ~bits, base + reg_offset);
}

static ALWAYS_INLINE void zynq_usb_write32(const struct device *dev, uint16_t reg_offset,
					   uint32_t val)
{
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	sys_write32(val, base + reg_offset);
}
static ALWAYS_INLINE uint32_t zynq_usb_read_write32(const struct device *dev, uint16_t reg_offset)
{
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	uint32_t read_val = sys_read32(base + reg_offset);
	sys_write32(read_val, base + reg_offset);
	return read_val;
}

static ALWAYS_INLINE uint32_t zynq_usb_read32(const struct device *dev, uint16_t reg_offset)
{
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	return sys_read32(base + reg_offset);
}

static ALWAYS_INLINE void zynq_usb_clear_setup_token_semaphores(const struct device *dev)
{
	zynq_usb_read_write32(dev, XUSBPS_EPSTAT_OFFSET);
}

static ALWAYS_INLINE void zynq_usb_clear_ep_complete(const struct device *dev)
{
	uint32_t val = zynq_usb_read32(dev, XUSBPS_EPCOMPL_OFFSET);
	zynq_usb_write32(dev, XUSBPS_EPCOMPL_OFFSET, val);
}

static ALWAYS_INLINE void zynq_usb_prime_and_flush_all(const struct device *dev)
{
	uint32_t timeout = CONFIG_UDC_ZYNQ_TIMEOUT;
	while ((zynq_usb_read32(dev, XUSBPS_EPPRIME_OFFSET) & XUSBPS_EP_ALL_MASK) && --timeout) {
	}
	zynq_usb_write32(dev, XUSBPS_EPFLUSH_OFFSET, 0xFFFFFFFF);
}

static ALWAYS_INLINE void zynq_usb_stop(const struct device *dev)
{
	zynq_usb_clear_bits(dev, XUSBPS_CMD_OFFSET, XUSBPS_CMD_RS_MASK);
}

static ALWAYS_INLINE void zynq_usb_hw_reset(const struct device *dev)
{
	uint32_t timeout = CONFIG_UDC_ZYNQ_TIMEOUT;
	while((zynq_usb_read32(dev, XUSBPS_CMD_OFFSET) & XUSBPS_CMD_RST_MASK) && --timeout) {
		/* NOP */
	}
}

static ALWAYS_INLINE void zynq_prime_ep(const struct device *dev, const struct udc_ep_config* ep)
{
	uint32_t ep_num = USB_EP_LUT_IDX(ep->addr);
	uint32_t mask = USB_EP_DIR_IS_IN(ep->addr) ? 0x00010000 : 0x00000001;
	uint32_t prime_val = mask << ep_num;
	zynq_usb_write32(dev, XUSBPS_EPPRIME_OFFSET, prime_val);
}
	
/**
 * @brief USB Controller Reset
 * 
 * @param dev 
 */
static void zynq_usb_controller_reset(const struct device *dev)
{
	/* stop the controller first */
	zynq_usb_stop(dev);

	/* reset the controller */
	uint32_t cmd_val = zynq_usb_read32(dev, XUSBPS_CMD_OFFSET);
	uint32_t cmd_val_rst = cmd_val | XUSBPS_CMD_RST_MASK;
	zynq_usb_write32(dev, XUSBPS_CMD_OFFSET, cmd_val_rst);
	// poll the reset bit
	uint32_t timeout = 10000;
	while ((zynq_usb_read32(dev, XUSBPS_CMD_OFFSET) & XUSBPS_CMD_RST_MASK) && --timeout) {
	}
}

static int zynq_prep_rx(const struct device *dev, struct net_buf *buf,
			 struct udc_ep_config *const cfg, const bool ncnak)
{
	// write data size of transfer descriptor
	// then prime the endpoint
	struct udc_buf_info *bi = udc_get_buf_info(buf);
	struct zynq_udc_dqh *dqh = DEV_PRIV(dev)->ep_priv_out->dqh; // RxData, DATA(OUT) endpoint
	struct zynq_udc_dtd *first_dtd = DEV_PRIV(dev)->ep_priv_out->dtds[0];

	uintptr_t buf_addr;
	uintptr_t buf_end;
	uint32_t ptr_num;
	if (buf->len > XUSBPS_dTD_BUF_MAX_SIZE) {
		LOG_ERR("Data size exceeds max transfer size");
		return -EINVAL;
	}
	buf_addr = (uintptr_t)buf->data;
	first_dtd->pages[0] = buf_addr;

	if (buf->len > 0) {
		buf_end = buf_addr + buf->len - 1;
		ptr_num = 1;

		while ((buf_addr & 0xFFFFF000) != (buf_end & 0xFFFFF000)) {
			buf_addr = (buf_addr + 0x1000) & 0xFFFFF000;
			first_dtd->pages[ptr_num] = buf_addr;
			ptr_num++;
		}
	}
	first_dtd->total_bytes = buf->len;
	first_dtd->terminate = 1;

	((struct dtd_aligned *)(first_dtd))->user_data = (uintptr_t)buf;

	dqh->zlt = bi->zlp;

	return 0;
}

static int zynq_ctrl_feed_dout(const struct device *dev, const size_t length)
{
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	struct net_buf *buf;

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, length);
	if (buf == NULL) {
		return -ENOMEM;
	}
	udc_buf_put(ep_cfg, buf);
	int ret = zynq_prep_rx(dev, buf, ep_cfg, 0);
	if(ret) {
		return ret;
	}
	// TODO(pan), prime the buffer to the endpoint

	LOG_DBG("feed buf %p", buf);

	return 0;
}


static int zynq_handle_evt_setup(const struct device *dev)
{
	struct net_buf *buf;
	int err;

	/* Get the buffer from the CTRL FIFO */
	buf = udc_buf_get(dev, USB_CONTROL_EP_OUT);
	if (buf == NULL) {
		LOG_ERR("No buffer queued for control ep");
		return -ENODATA;
	}

	/* copy the setup packet to the buffer */ 
	net_buf_add_mem(buf, PRIV_SETUP(dev), sizeof(PRIV_SETUP(dev)));
	/* toggle the buf's user_data setup flag */
	udc_ep_buf_set_setup(buf);
	LOG_HEXDUMP_DBG(buf->data, buf->len, "setup");

	/* Update to next stage of control transfer */
	udc_ctrl_update_stage(dev, buf);

	/* We always allocate and feed buffer large enough for a setup packet. */
	if (udc_ctrl_stage_is_data_out(dev)) {
		/*  Allocate and feed buffer for data OUT stage */
		LOG_DBG("s:%p|feed for -out-", buf);

		/* Allocate at least 8 bytes in case the host decides to send
		 * SETUP DATA instead of OUT DATA packet.
		 */
		err = zynq_ctrl_feed_dout(dev, MAX(udc_data_stage_length(buf), 8));
		if (err) {
			err = udc_submit_ep_event(dev, buf, err);
		}
	} else if (udc_ctrl_stage_is_data_in(dev)) {
		LOG_DBG("s:%p|feed for -in-status", buf);

		err = zynq_ctrl_feed_dout(dev, 8);
		if (err) {
			err = udc_submit_ep_event(dev, buf, err);
		}

		err = udc_ctrl_submit_s_in_status(dev);
	} else {
		LOG_DBG("s:%p|feed >setup", buf);

		err = zynq_ctrl_feed_dout(dev, 8);
		if (err) {
			err = udc_submit_ep_event(dev, buf, err);
		}

		err = udc_ctrl_submit_s_status(dev);
	}

	return err;
}



static void zynq_handle_xfer_next(const struct device *dev, struct udc_ep_config *const cfg)
{
	// struct net_buf *buf;

	// buf = udc_buf_peek(dev, cfg->addr);
	// if (buf == NULL) {
	// 	return;
	// }

	// if (USB_EP_DIR_IS_OUT(cfg->addr)) {
	// 	dwc2_prep_rx(dev, buf, cfg, 0);
	// } else {
	// 	if (dwc2_tx_fifo_write(dev, cfg, buf)) {
	// 		LOG_ERR("Failed to start write to TX FIFO, ep 0x%02x",
	// 			cfg->addr);
	// 	}
	// }

	// udc_ep_set_busy(dev, cfg->addr, true);
}

/*
 * You can use one thread per driver instance model or UDC driver workqueue,
 * whichever model suits your needs best. If you decide to use the UDC workqueue,
 * enable Kconfig option UDC_WORKQUEUE and remove the handler below and
 * caller from the UDC_ZYNQ_DEVICE_DEFINE macro.
 */
static ALWAYS_INLINE void zynq_thread_handler(void *const arg)
{
	const struct device *dev = (const struct device *)arg;
	struct udc_ep_config *ep_cfg;
	struct zynq_drv_event evt;

	/* This is the bottom-half of the ISR handler and the place where
	 * a new transfer can be fed.
	 */
	k_msgq_get(&drv_msgq, &evt, K_FOREVER);
	ep_cfg = udc_get_ep_cfg(dev, evt.ep);

	switch (evt.type) {
	case ZYNQ_DRV_EVT_XFER:
		LOG_DBG("New transfer in the queue");
		break;
	case ZYNQ_DRV_EVT_SETUP:
		LOG_DBG("SETUP event");
		zynq_handle_evt_setup(dev);
		break;
	case ZYNQ_DRV_EVT_DOUT:
		LOG_DBG("DOUT event ep 0x%02x", ep_cfg->addr);
		// dwc2_handle_evt_dout(dev, ep_cfg);
		break;
	case ZYNQ_DRV_EVT_DIN:
		LOG_DBG("DIN event");
		// dwc2_handle_evt_din(dev, ep_cfg);
		break;
	}

	if (ep_cfg->addr != USB_CONTROL_EP_OUT && !udc_ep_is_busy(dev, ep_cfg->addr)) {
		// zynq_handle_xfer_next(dev, ep_cfg);
	} else {
		LOG_DBG("ep 0x%02x busy", ep_cfg->addr);
	}
}

/*
 * This is called in the context of udc_ep_enqueue() and must
 * not block. The driver can immediately claim the buffer if the queue is empty,
 * but usually it is offloaded to a thread or workqueue to handle transfers
 * in a single location. Please refer to existing driver implementations
 * for examples.
 */
static int udc_zynq_ep_enqueue(const struct device *dev, struct udc_ep_config *const cfg,
			       struct net_buf *buf)
{
	struct zynq_drv_event evt = {
		.ep = cfg->addr,
		.type = ZYNQ_DRV_EVT_XFER,
	};

	LOG_DBG("%p enqueue %p", dev, buf);
	udc_buf_put(cfg, buf);

	if (!cfg->stat.halted) {
		k_msgq_put(&drv_msgq, &evt, K_NO_WAIT);
	}
	// bool pipe_empty;
	// udc_td* desc_ptr;
	// uint8_t ep_num = USB_EP_LUT_IDX(cfg->addr);
	// bool is_in = USB_EP_DIR_IS_IN(cfg->addr);
	// struct udc_zynq_data* priv = DEV_DATA(dev);

	// // struct ep_priv *ep_priv = UDC_EP_PRIV(cfg);

	// if(ep_priv->in.td_head != ep_priv->in.td_tail) {
	// 	pipe_empty = false;
	// }

	// if (zynq_usb_td_is_active(ep_priv->in.td_head)) {
	// 	// TODO(pan), submit event to upper layer
	// 	return -EBUSY;
	// }

	// desc_ptr = ep_priv->in.td_head;

	// ep_priv->requested_bytes = buf->len;

	return 0;
}

/*
 * This is called in the context of udc_ep_dequeue()
 * and must remove all requests from an endpoint queue
 * Successful removal should be reported to the higher level with
 * ECONNABORTED as the request result.
 * It is up to the request owner to clean up or reuse the buffer.
 */
static int udc_zynq_ep_dequeue(const struct device *dev, struct udc_ep_config *const cfg)
{
	struct net_buf *buf;
	/* read ep prime sts */

	/* flush the */


	buf = udc_buf_get_all(dev, cfg->addr);
	if (buf) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}

	udc_ep_set_busy(dev, cfg->addr, false);



	return 0;
}

static void zynq_flush_tx_fifo(const struct device *dev, const uint8_t idx)
{
	uint32_t val = (1 << idx) << XUSBPS_EPFLUSH_TX_SHIFT;
	zynq_usb_set_bits(dev, XUSBPS_EPFLUSH_OFFSET, val);

}

static void zynq_flush_rx_fifo(const struct device *dev, const uint8_t idx)
{
	uint32_t val = (1 << idx) << XUSBPS_EPFLUSH_RX_SHIFT;
	zynq_usb_set_bits(dev, XUSBPS_EPFLUSH_OFFSET, val);
}

/*
 * Configure and make an endpoint ready for use.
 * This is called in the context of udc_ep_enable() or udc_ep_enable_internal(),
 * the latter of which may be used by the driver to enable control endpoints.
 */

/**
 * @brief According ug585-zynq-7000-TRM.pdf, table 15-6, USB Device Endpoint Initialization
 * 1. Data Toggle Reset, Restart transfers with DATA0
 * 2. Data Toggle Inhibit, Do not inhibit data toggle
 * 3. Endpoint Type, Set the endpoint type
 * 4. Disable Endpoint Stall
 * 5. Enable Endpoint
 * @note the ctrl endpoint is always enabled, but we still need to clear the stall bit
 * 
 * @param dev 
 * @param cfg 
 * @return int 
 */
static int udc_zynq_ep_enable(const struct device *dev, struct udc_ep_config *const cfg)
{
	LOG_DBG("Enable ep 0x%02x", cfg->addr);
	uint8_t ep_idx = USB_EP_LUT_IDX(cfg->addr);
	bool ep_is_out = USB_EP_DIR_IS_OUT(cfg->addr);
	bool ep_is_ctrl = ep_idx == 0;
	uint16_t ep_ctrl_offset = XUSBPS_EPCRn_OFFSET(ep_idx);

	if (!ep_is_ctrl) {
		/* toggle reset */
		uint32_t toggle_reset = ep_is_out ? XUSBPS_EPCR_RXR_MASK : XUSBPS_EPCR_TXR_MASK;
		zynq_usb_set_bits(dev, ep_ctrl_offset, toggle_reset);

		/* toggle inhibit */
		uint32_t toggle_inhibit = ep_is_out ? XUSBPS_EPCR_RXI_MASK : XUSBPS_EPCR_TXI_MASK;
		zynq_usb_clear_bits(dev, ep_ctrl_offset, toggle_inhibit);

		/* set endpoint type */
		uint32_t type_mask = ep_is_out ? XUSBPS_EPCR_RXT_TYPE_MASK : XUSBPS_EPCR_TXT_TYPE_MASK;
		uint32_t type_val = 0;
		if (cfg->caps.bulk) {
			type_val = ep_is_out ? XUSBPS_EPCR_RXT_BULK_MASK : XUSBPS_EPCR_TXT_BULK_MASK;
		} else if (cfg->caps.interrupt) {
			type_val = ep_is_out ? XUSBPS_EPCR_RXT_INTR_MASK : XUSBPS_EPCR_TXT_INTR_MASK;
		} else if (cfg->caps.iso) {
			type_val = ep_is_out ? XUSBPS_EPCR_RXT_ISO_MASK : XUSBPS_EPCR_TXT_ISO_MASK;
		}
		zynq_usb_mask_write(dev, ep_ctrl_offset, type_mask, type_val);
	}

	/* clear stall */
	uint32_t stall_clear = ep_is_out ? XUSBPS_EPCR_RXS_MASK : XUSBPS_EPCR_TXS_MASK;
	zynq_usb_clear_bits(dev, ep_ctrl_offset, stall_clear);

	/* trans enable */
	if (!ep_is_ctrl) {
		uint32_t en_bit = (ep_is_out ? XUSBPS_EPCR_RXE_MASK : XUSBPS_EPCR_TXE_MASK);
		zynq_usb_set_bits(dev, ep_ctrl_offset, en_bit);
	}

	return 0;
}

/*
 * Opposite function to udc_zynq_ep_enable(). udc_ep_disable_internal()
 * may be used by the driver to disable control endpoints.
 */
static int udc_zynq_ep_disable(const struct device *dev, struct udc_ep_config *const cfg)
{
	LOG_DBG("Disable ep 0x%02x", cfg->addr);
	uint8_t ep_idx = USB_EP_LUT_IDX(cfg->addr);
	bool ep_is_ctrl = ep_idx == 0;
	if (ep_is_ctrl) {
		return 0;
	}

	bool ep_is_out = USB_EP_DIR_IS_OUT(cfg->addr);
	uint16_t ctrl_offset = XUSBPS_EPCRn_OFFSET(ep_idx);
	uint32_t clear_bits = (ep_is_out ? XUSBPS_EPCR_RXE_MASK : XUSBPS_EPCR_TXE_MASK);
	zynq_usb_clear_bits(dev, ctrl_offset, clear_bits);

	return 0;
}

/* Halt endpoint. Halted endpoint should respond with a STALL handshake. */
static int udc_zynq_ep_set_halt(const struct device *dev, struct udc_ep_config *const cfg)
{
	LOG_DBG("Set halt ep 0x%02x", cfg->addr);
	cfg->stat.halted = true;
	uint8_t ep_num = USB_EP_LUT_IDX(cfg->addr);
	bool ep_is_out = USB_EP_DIR_IS_OUT(cfg->addr);
	uint16_t ctrl_offset = XUSBPS_EPCRn_OFFSET(ep_num);
	uint32_t set_bits = (ep_is_out ? XUSBPS_EPCR_RXS_MASK : XUSBPS_EPCR_TXS_MASK);
	/* force controller to send stall responses */
	zynq_usb_set_bits(dev, ctrl_offset, set_bits);

	return 0;
}

/*
 * Opposite to halt endpoint. If there are requests in the endpoint queue,
 * the next transfer should be prepared.
 */
static int udc_zynq_ep_clear_halt(const struct device *dev, struct udc_ep_config *const cfg)
{
	LOG_DBG("Clear halt ep 0x%02x", cfg->addr);
	cfg->stat.halted = false;
	uint8_t ep_num = USB_EP_LUT_IDX(cfg->addr);
	bool ep_is_out = USB_EP_DIR_IS_OUT(cfg->addr);
	uint16_t ctrl_offset = XUSBPS_EPCRn_OFFSET(ep_num);
	/* do not force stall responses */
	uint32_t clear_bits = (ep_is_out ? XUSBPS_EPCR_RXS_MASK : XUSBPS_EPCR_TXS_MASK);
	zynq_usb_clear_bits(dev, ctrl_offset, clear_bits);

	return 0;
}

static int udc_zynq_set_address(const struct device *dev, const uint8_t addr)
{
	LOG_DBG("Set new address %u for %p", addr, dev);
	if (addr > XUSBPS_DEVICEADDR_MAX) {
		return -EINVAL;
	}

	zynq_usb_write32(dev, XUSBPS_DEVICEADDR_OFFSET, (addr << XUSBPS_DEVICEADDR_ADDR_SHIFT) | XUSBPS_DEVICEADDR_DEVICEAADV_MASK);
	return 0;
}

static int udc_zynq_host_wakeup(const struct device *dev)
{
	LOG_DBG("Remote wakeup from %p", dev);
	zynq_usb_set_bits(dev, XUSBPS_PORTSCR1_OFFSET, XUSBPS_PORTSCR_FPR_MASK);
	return 0;
}

/* Return actual USB device speed */
static enum udc_bus_speed udc_zynq_device_speed(const struct device *dev)
{
	// read from port status register
	uint32_t port_status = zynq_usb_read32(dev, XUSBPS_PORTSCR1_OFFSET) & XUSBPS_PORTSCR_PSPD_MASK;

	if(port_status == XUSBPS_PORTSCR_PSPD_FS) {
		return UDC_BUS_SPEED_FS;
	} else if(port_status == XUSBPS_PORTSCR_PSPD_HS) {
		return UDC_BUS_SPEED_HS;
	} 
	return UDC_BUS_UNKNOWN;
}

/**
 * @brief The function enables the device. configures the endpoints
 * logic same as 'Xilinx Standalone USB Device Stack.XUsbPs_ConfigureDevice'
 * @param dev
 * @return int
 */
static int udc_zynq_enable(const struct device *dev)
{
	const struct udc_zynq_config *cfg = dev->config;
	int ret;
	LOG_DBG("Enable device %p", dev);
	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL,
				     cfg->ep_cfg_out[0].mps, 0);
	if (ret) {
		LOG_ERR("Failed to enable control endpoint");
		return ret;
	}
	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL,
				     cfg->ep_cfg_in[0].mps, 0);
	if (ret) {
		LOG_ERR("Failed to enable control endpoint");
		return ret;
	}

	/**
	 *@brief configure the interrupt
	 * i. Disable all the interrupts
	 * ii. configure the interrupts 1. USB Transaction Complete, 2. Trans Error 3. Port Change 4. SoF 5. Suspend
	 */
	zynq_usb_write32(dev, XUSBPS_IER_OFFSET, 0);
	cfg->config_func(dev, true); // connect the interrupt
	zynq_usb_set_bits(dev, XUSBPS_IER_OFFSET,
			   /* usb transaction complete */ XUSBPS_IXR_UI_MASK |
			   /* Trans Error */ XUSBPS_IXR_UE_MASK |
			   /* Port Change Detect */ XUSBPS_IXR_PC_MASK |
			   /* Reset Received */ XUSBPS_IXR_UR_MASK |
			   /* Start Of Frame */ XUSBPS_IXR_SR_MASK |
			   /* Device Controller Suspend */ XUSBPS_IXR_SLE_MASK);

	/* toggle the start bit */
	zynq_usb_set_bits(dev, XUSBPS_CMD_OFFSET, XUSBPS_CMD_RS_MASK);


	return 0;
}

static int udc_zynq_disable(const struct device *dev)
{
	LOG_DBG("Disable device %p", dev);
	const struct udc_zynq_config *cfg = dev->config;
	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	/* disable all interrupt */
	zynq_usb_write32(dev, XUSBPS_IER_OFFSET, 0);

	/* disable func */
	cfg->config_func(dev, false);

	zynq_usb_stop(dev);
	return 0;
}

static void udc_zynq_device_partial_reset(const struct device *dev)
{
	/* clear all setup token sema4 by writing the read value of the XUSBPS_EPSTAT_OFFSET */
	zynq_usb_clear_setup_token_semaphores(dev);
	/* prime clear and flush */
	zynq_usb_prime_and_flush_all(dev);
}

/**
 * @brief Reset logic
 * 1. setup sema4 clear
 * 2. prime clear, flush
 * 3. USBCMD[RS]=0, stop the controller
 * 4. USBCMD[RST]=1, reset the controller, then poll the reset bit
 */
static void udc_zynq_device_reset(const struct device *dev)
{
	/* clear all setup token sema4 by writing the read value of the XUSBPS_EPSTAT_OFFSET */
	zynq_usb_clear_setup_token_semaphores(dev);
	/* prime clear and flush */
	zynq_usb_prime_and_flush_all(dev);
	/* stop the controller */
	zynq_usb_stop(dev);
	/* hw_reset */
	zynq_usb_hw_reset(dev);
}


/**
 * @brief Prepare and configure most of the parts, if the controller has a way
 * of detecting VBUS activity it should be enabled here.
 * Only udc_zynq_enable() makes device visible to the host.
 * 
 * @param dev 
 * @return int 
 */
static int udc_zynq_init(const struct device *dev)
{
	udc_zynq_device_reset(dev);
	return 0;
}

/* Shut down the controller completely */
static int udc_zynq_shutdown(const struct device *dev)
{
	udc_zynq_device_reset(dev);
	return 0;
}

static void zynq_udc_isr(const struct device *dev)
{
	uint32_t usb_sts = zynq_usb_read_write32(dev, XUSBPS_ISR_OFFSET);
	if(usb_sts & XUSBPS_IXR_UI_MASK) {
		/* USB Transaction Complete */
		uint32_t ep_stat = zynq_usb_read32(dev, XUSBPS_EPSTAT_OFFSET);

		if(ep_stat & 0x1) {
			/* ENDPOINT 0 SETUP PACKET HANDLING
		 	 *
		 	 * Check if we got a setup packet on endpoint 0. Currently we
		 	 * only check for setup packets on endpoint 0 as we would not
		 	 * expect setup packets on any other endpoint (even though it
		 	 * is possible to send setup packets on other endpoints).
		 	 */
			struct zynq_drv_event evt = {
				.dev = dev,
				.type = ZYNQ_DRV_EVT_SETUP,
			};
			k_msgq_put(&drv_msgq, &evt, K_NO_WAIT);
		}

		/* check for Rx/Tx complete intr */

	}
	 


}

/*
 * This is called once to initialize the controller and endpoints
 * capabilities, and register endpoint structures.
 */
static int udc_zynq_driver_preinit(const struct device *dev)
{
	const struct udc_zynq_config *config = dev->config;
	struct udc_data *data = dev->data;
	struct zynq_udc_data *priv = DEV_PRIV(dev);
	int err;

	/*
	 * You do not need to initialize it if your driver does not use
	 * udc_lock_internal() / udc_unlock_internal(), but implements its
	 * own mechanism.
	 */
	k_mutex_init(&data->mutex);

	data->caps.rwup = false;
	data->caps.mps0 = UDC_MPS0_64;
	if (config->speed_idx == 2) {
		data->caps.hs = true;
	}

	for (int i = 0; i < config->num_of_eps; i++) {
		config->ep_cfg_out[i].caps.out = 1;
		if (i == 0) {
			config->ep_cfg_out[i].caps.control = 1;
			config->ep_cfg_out[i].caps.mps = 64;
		} else {
			config->ep_cfg_out[i].caps.bulk = 1;
			config->ep_cfg_out[i].caps.interrupt = 1;
			config->ep_cfg_out[i].caps.iso = 1;
			config->ep_cfg_out[i].caps.mps = 1024;
		}

		config->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
		err = udc_register_ep(dev, &config->ep_cfg_out[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	for (int i = 0; i < config->num_of_eps; i++) {
		config->ep_cfg_in[i].caps.in = 1;
		if (i == 0) {
			config->ep_cfg_in[i].caps.control = 1;
			config->ep_cfg_in[i].caps.mps = 64;
		} else {
			config->ep_cfg_in[i].caps.bulk = 1;
			config->ep_cfg_in[i].caps.interrupt = 1;
			config->ep_cfg_in[i].caps.iso = 1;
			config->ep_cfg_in[i].caps.mps = 1024;
		}

		config->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		err = udc_register_ep(dev, &config->ep_cfg_in[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	for (int i = 0; i < config->num_of_eps; i++) {
		struct ep_priv *ep_priv_out = &priv->ep_priv_out[i];
		struct ep_priv *ep_priv_in = &priv->ep_priv_in[i];

		ep_priv_out->dqh = (struct zynq_udc_dqh *)(&config->qh_arr[2 * i]);
		ep_priv_in->dqh = (struct zynq_udc_dqh *)(&config->qh_arr[2 * i + 1]);

		for (int j = 0; j < 4; j++) {
			ep_priv_out->dtds[j] = (struct zynq_udc_dtd *)(&config->td_arr[8 * i + j]);
			ep_priv_in->dtds[j] = (struct zynq_udc_dtd *)(&config->td_arr[8 * i + 4 + j]);
		}


		for(int j = 0; j < 4; j++) {
			// impl the dtd in a circular manner
			int next = (j + 1) % 4;
			struct zynq_udc_dtd *out_dtd = ep_priv_out->dtds[j];
			struct zynq_udc_dtd *in_dtd = ep_priv_in->dtds[j];

			out_dtd->next_dtd_nlp = (uint32_t)(ep_priv_out->dtds[next]);
			out_dtd->terminate = 1; // set terminate default
			in_dtd->next_dtd_nlp = (uint32_t)(ep_priv_in->dtds[next]);
			in_dtd->terminate = 1; // set terminate default
		}

		ep_priv_out->dqh->ep_cur_dtd = (uint32_t)ep_priv_out->dtds[0];
		ep_priv_in->dqh->ep_cur_dtd = (uint32_t)ep_priv_in->dtds[0];
	}
	/* set queue head list addr */
	zynq_usb_write32(dev, XUSBPS_EPLISTADDR_OFFSET, (uint32_t)config->qh_arr);
	LOG_INF("Device %p (max. speed %d)", dev, config->speed_idx);

	/* CM(controller mode) in device mode, SLO(setup lockout) mode */
	zynq_usb_write32(dev, XUSBPS_MODE_OFFSET, XUSBPS_MODE_CM_DEVICE_MASK | XUSBPS_MODE_SLOM_MASK);
	/* OTG Termination, mandatory in device mode */
	zynq_usb_set_bits(dev, XUSBPS_OTGCSR_OFFSET, XUSBPS_OTGSC_OT_MASK);




	return 0;
}

static int udc_zynq_lock(const struct device *dev)
{
	return udc_lock_internal(dev, K_FOREVER);
}

static int udc_zynq_unlock(const struct device *dev)
{
	return udc_unlock_internal(dev);
}

/*
 * UDC API structure.
 * Note, you do not need to implement basic checks, these are done by
 * the UDC common layer udc_common.c
 */
static const struct udc_api udc_zynq_api = {
	.lock = udc_zynq_lock,
	.unlock = udc_zynq_unlock,
	.device_speed = udc_zynq_device_speed,
	.init = udc_zynq_init,
	.enable = udc_zynq_enable,
	.disable = udc_zynq_disable,
	.shutdown = udc_zynq_shutdown,
	.set_address = udc_zynq_set_address,
	.host_wakeup = udc_zynq_host_wakeup,
	.ep_enable = udc_zynq_ep_enable,
	.ep_disable = udc_zynq_ep_disable,
	.ep_set_halt = udc_zynq_ep_set_halt,
	.ep_clear_halt = udc_zynq_ep_clear_halt,
	.ep_enqueue = udc_zynq_ep_enqueue,
	.ep_dequeue = udc_zynq_ep_dequeue,
};

#define DT_DRV_COMPAT xlnx_zynq_usb

/*
 * A UDC driver should always be implemented as a multi-instance
 * driver, even if your platform does not require it.
 */
#define UDC_ZYNQ_DEVICE_DEFINE(n)                                                                  \
	K_THREAD_STACK_DEFINE(udc_zynq_stack_##n, CONFIG_UDC_ZYNQ);                                \
	static void zynq_udc_##n##_irq_config_func(const struct device *dev, bool en)              \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
		if (en) {                                                                          \
			IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), zynq_udc_isr,       \
				    DEVICE_DT_INST_GET(n), 0);                                     \
			irq_enable(DT_INST_IRQN(n));                                               \
		} else {                                                                           \
			irq_disable(DT_INST_IRQN(n));                                              \
		}                                                                                  \
	}                                                                                          \
                                                                                                   \
	static void udc_zynq_thread_##n(void *dev, void *arg1, void *arg2)                         \
	{                                                                                          \
		zynq_thread_handler(dev);                                                          \
	}                                                                                          \
                                                                                                   \
	static void udc_zynq_make_thread_##n(const struct device *dev)                             \
	{                                                                                          \
		struct zynq_udc_data *priv = udc_get_private(dev);                                 \
                                                                                                   \
		k_thread_create(&priv->thread_data, udc_zynq_stack_##n,                            \
				K_THREAD_STACK_SIZEOF(udc_zynq_stack_##n), udc_zynq_thread_##n,    \
				(void *)dev, NULL, NULL,                                           \
				K_PRIO_COOP(CONFIG_UDC_ZYNQ_THREAD_PRIORITY), K_ESSENTIAL,         \
				K_NO_WAIT);                                                        \
		k_thread_name_set(&priv->thread_data, dev->name);                                  \
	}                                                                                          \
                                                                                                   \
	static struct udc_ep_config ep_cfg_out_##n[DT_INST_PROP(n, num_bidir_endpoints)];          \
	static struct udc_ep_config ep_cfg_in_##n[DT_INST_PROP(n, num_bidir_endpoints)];           \
	static struct ep_priv ep_priv_out_##n[DT_INST_PROP(n, num_bidir_endpoints)];               \
	static struct ep_priv ep_priv_in_##n[DT_INST_PROP(n, num_bidir_endpoints)];                \
	static struct dqh_aligned qh_##n[2 * DT_INST_PROP(n, num_bidir_endpoints)];                \
	static struct dtd_aligned td_##n[8 * DT_INST_PROP(n, num_bidir_endpoints)];                \
                                                                                                   \
	static const struct udc_zynq_config udc_zynq_config_##n = {                                \
		.num_of_eps = DT_INST_PROP(n, num_bidir_endpoints),                                \
		.ep_cfg_in = ep_cfg_in_##n,                                                        \
		.ep_cfg_out = ep_cfg_out_##n,                                                      \
		.make_thread = udc_zynq_make_thread_##n,                                           \
		.speed_idx = DT_ENUM_IDX(DT_DRV_INST(n), maximum_speed),                           \
		.qh_arr = qh_##n,                                                                  \
		.td_arr = td_##n,                                                                  \
		.config_func = zynq_udc_##n##_irq_config_func,                                     \
	};                                                                                         \
                                                                                                   \
	static struct zynq_udc_data udc_priv_##n = {                                               \
		.ep_priv_in = ep_priv_in_##n,                                                      \
		.ep_priv_out = ep_priv_out_##n,                                                    \
	};                                                                                         \
                                                                                                   \
	static struct udc_data udc_data_##n = {                                                    \
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),                                  \
		.priv = &udc_priv_##n,                                                             \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, udc_zynq_driver_preinit, NULL, &udc_data_##n,                     \
			      &udc_zynq_config_##n, POST_KERNEL,                                   \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &udc_zynq_api);

DT_INST_FOREACH_STATUS_OKAY(UDC_ZYNQ_DEVICE_DEFINE)
