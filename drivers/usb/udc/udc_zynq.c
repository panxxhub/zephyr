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
#include "zephyr/kernel/mm.h"
#include "zephyr/sys/device_mmio.h"
#include "zephyr/sys/sys_io.h"
#include "zephyr/toolchain.h"
#include "udc_zynq.h"
#include <zephyr/drivers/gpio.h>


#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>

#include <zephyr/logging/log.h>
#include <strings.h>
#include <sys/cdefs.h>
#include <sys/types.h>
LOG_MODULE_REGISTER(UDC_ZYNQ, CONFIG_UDC_DRIVER_LOG_LEVEL);

#define USB_EP_LUT_IDX(ep)  (USB_EP_DIR_IS_IN(ep) ? (ep & BIT_MASK(4)) + 16 : ep & BIT_MASK(4))


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

enum zynq_drv_event_type {
	/* Trigger Next Xfer */
	ZYNQ_DRV_EVT_XFER,
	/* Setup packet received */
	ZYNQ_DRV_EVT_SETUP,
	/* Reset event */
	ZYNQ_DRV_EVT_RESET,
	/* OUT transaction for specific endpoint is finished, some data is received */
	ZYNQ_DRV_EVT_DOUT,
	/* IN transaction for specific endpoint is finished, some data is sent */
	ZYNQ_DRV_EVT_DIN,
};

struct zynq_drv_event {
	const struct device *dev;
	enum zynq_drv_event_type type;
	uint32_t flag;
	uint8_t ep_addr;
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
	DEVICE_MMIO_ROM;
	const struct gpio_dt_spec phy_reset;
	size_t num_of_eps;
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
	void (*make_thread)(const struct device *dev);
	int speed_idx;
	struct dqh_aligned *qh_arr;
	struct dtd_aligned *td_arr;
	zynq_udc_irq_config_func config_func;
#ifdef CONFIG_PINCTRL
	const struct pinctrl_dev_config *pincfg;
#endif
};

typedef struct ep_priv {
	struct zynq_udc_dqh *dqh_out;
	struct zynq_udc_dqh *dqh_in;

	struct zynq_udc_dtd *dtd_ptrs[CONFIG_UDC_ZYNQ_TD_PER_EP];
	struct idx_ring {
		uint32_t head_idx; // head_idx is the next available td
		uint32_t tail_idx; // tail_idx is the next td to be processed, if(head_idx ==
				   // tail_idx) the pipe is empty, if (head_idx == (tail_idx + 1) %
				   // size) the pipe is full
		uint32_t size; // we advance the idx by (idx + 1) % size, always keep the size as
			       // power of 2, if not enabled, the size is 0
	} out, in;
} ep_priv_t;

/*
 * Structure to hold driver private data.
 * Note that this is not accessible via dev->data, but as
 * struct udc_zynq_data *priv = udc_get_private(dev);
 * since the ep0 won't be enqueued from upper layer, we need to prepare the buffer here, in case we
 * have setup with data out stage
 */
struct zynq_udc_data {
	DEVICE_MMIO_RAM;
	struct k_thread thread_data;
	struct ep_priv *eps;
};

#define DEV_PRIV(dev)             ((struct zynq_udc_data *)udc_get_private(dev))
#define DEV_CFG(dev)              ((const struct udc_zynq_config *)dev->config)
#define DEV_DATA(dev)             ((struct udc_data *)dev->data)
#define EP_CFG_IDX(dev, idx, dir) (DEV_DATA(dev)->ep_lut[USB_EP_GET_ADDR(idx, dir)])
#define DEV_EP(dev, idx)          (DEV_PRIV(dev)->eps[(idx)])
#define DEV_EP_PTR(dev, idx)      (&DEV_EP(dev, idx))
#define EP_DQH_OUT(dev, idx)      (DEV_EP(dev, idx).dqh_out)
#define EP_DQH_IN(dev, idx)       (DEV_EP(dev, idx).dqh_in)
#define OUT_SETUP(dev, idx)       (EP_DQH_OUT(dev, idx)->setup_buffer)
#define IN_SETUP(dev, idx)        (EP_DQH_IN(dev, idx)->setup_buffer)
#define CTRL_EP(dev)              (DEV_EP(dev, 0))
#define CTRL_OUT_SETUP(dev)       (OUT_SETUP(dev, 0)) // default setup buffer

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
	if (read_val) {
		sys_write32(read_val, base + reg_offset);
	}
	return read_val;
}

static ALWAYS_INLINE uint32_t zynq_usb_read32(const struct device *dev, uint16_t reg_offset)
{
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	return sys_read32(base + reg_offset);
}

/******************************************************************************************
 *  HELPER FUNCTIONS
 *****************************************************************************************/

static ALWAYS_INLINE void zynq_usb_clear_setup_interrupts(const struct device *dev)
{
	uint32_t timeout = CONFIG_UDC_ZYNQ_TIMEOUT;
	while (zynq_usb_read_write32(dev, XUSBPS_EPSTAT_OFFSET) & --timeout) {

		/* According to UG585, PG 456
		 * The time from writing a 1 to usb.ENDPTSETUPSTAT register and reading back a 0 is
		 * very short (approximately 1 to 2 microsecond) so a poll loop in the DCD should
		 * not be harmful in most systems
		 */

		/* NOP */
	}
}

static ALWAYS_INLINE void zynq_usb_set_setup_sema4(const struct device *dev)
{
	zynq_usb_set_bits(dev, XUSBPS_CMD_OFFSET, XUSBPS_CMD_SUTW_MASK);
}

static ALWAYS_INLINE bool zynq_usb_test_setup_sema4(const struct device *dev)
{
	return zynq_usb_read32(dev, XUSBPS_CMD_OFFSET) & XUSBPS_CMD_SUTW_MASK;
}

static ALWAYS_INLINE void zynq_usb_ep_clear_stall(const struct device *dev,
						  const struct udc_ep_config *ep)
{
	uint16_t ctrl_offset = XUSBPS_EPCRn_OFFSET(USB_EP_GET_IDX(ep->addr));
	uint32_t clear_bits = USB_EP_DIR_IS_OUT(ep->addr) ? XUSBPS_EPCR_RXS_MASK : XUSBPS_EPCR_TXS_MASK;
	zynq_usb_clear_bits(dev, ctrl_offset, clear_bits);
}

static ALWAYS_INLINE void zynq_usb_ep_set_stall(const struct device *dev, const struct udc_ep_config *ep)
{
	uint16_t ctrl_offset = XUSBPS_EPCRn_OFFSET(USB_EP_GET_IDX(ep->addr));
	uint32_t set_bits = USB_EP_DIR_IS_OUT(ep->addr) ? XUSBPS_EPCR_RXS_MASK : XUSBPS_EPCR_TXS_MASK;
	zynq_usb_set_bits(dev, ctrl_offset, set_bits);
}

static ALWAYS_INLINE void zynq_usb_write_complete(const struct device *dev, uint32_t val)
{
	zynq_usb_write32(dev, XUSBPS_EPCOMPL_OFFSET, val);
}

static ALWAYS_INLINE void zynq_usb_prime_and_flush_all(const struct device *dev)
{
	uint32_t timeout = CONFIG_UDC_ZYNQ_TIMEOUT;
	while ((zynq_usb_read32(dev, XUSBPS_EPPRIME_OFFSET) & XUSBPS_EP_ALL_MASK) && --timeout) {
	}
	zynq_usb_write32(dev, XUSBPS_EPFLUSH_OFFSET, 0xFFFFFFFF);
}

static ALWAYS_INLINE void zynq_usb_hw_reset(const struct device *dev)
{
	/* reset the controller */
	zynq_usb_set_bits(dev, XUSBPS_CMD_OFFSET, XUSBPS_CMD_RST_MASK);

	uint32_t timeout = CONFIG_UDC_ZYNQ_TIMEOUT;
	while ((zynq_usb_read32(dev, XUSBPS_CMD_OFFSET) & XUSBPS_CMD_RST_MASK) && --timeout) {
		/* NOP */
	}
}

static ALWAYS_INLINE void zynq_prime_ep(const struct device *dev, const struct udc_ep_config *ep)
{
	uint32_t prime_val = USB_EP_LUT_IDX(ep->addr);
	zynq_usb_write32(dev, XUSBPS_EPPRIME_OFFSET, prime_val);
}

static ALWAYS_INLINE void zynq_usb_run(const struct device *dev)
{
	/* toggle the start bit */
	zynq_usb_set_bits(dev, XUSBPS_CMD_OFFSET, XUSBPS_CMD_RS_MASK);
}

static ALWAYS_INLINE bool zynq_usb_ep_is_ctrl(const struct udc_ep_config *ep)
{
	return (ep->addr == USB_CONTROL_EP_OUT) || (ep->addr == USB_CONTROL_EP_IN);
}

static ALWAYS_INLINE void zynq_usb_stop(const struct device *dev)
{
	/* stop the controller */
	zynq_usb_clear_bits(dev, XUSBPS_CMD_OFFSET, XUSBPS_CMD_RS_MASK);
}

static ALWAYS_INLINE void zynq_udc_td_din_tail_advance(const struct device *dev, uint8_t ep_num)
{

	struct ep_priv *ep = DEV_EP_PTR(dev, ep_num);
	/**
	 * for control point,
	 * out idx [0, 1, 2, 3], in idx [4, 5, 6, 7], if we have 8 td per ep
	 */
	ep->in.tail_idx = ((ep->in.tail_idx + 1) % ep->in.size) + (ep_num == 0 ? ep->out.size : 0);
}

static ALWAYS_INLINE void zynq_udc_td_dout_tail_advance(const struct device *dev, uint8_t ep_num)
{
	struct ep_priv *ep = DEV_EP_PTR(dev, ep_num);
	ep->out.tail_idx = (ep->out.tail_idx + 1) % ep->out.size;
}

static ALWAYS_INLINE void zynq_udc_td_din_head_advance(const struct device *dev, uint8_t ep_num)
{
	struct ep_priv *ep = DEV_EP_PTR(dev, ep_num);
	ep->in.head_idx = (ep->in.head_idx + 1) % ep->in.size + (ep_num == 0 ? ep->out.size : 0);
}

static ALWAYS_INLINE void zynq_udc_td_dout_head_advance(const struct device *dev, uint8_t ep_num)
{
	struct ep_priv *ep = DEV_EP_PTR(dev, ep_num);
	ep->out.head_idx = (ep->out.head_idx + 1) % ep->out.size;
}

static ALWAYS_INLINE bool zynq_udc_din_empty(const struct device *dev, uint8_t ep_num)
{
	struct ep_priv *ep = DEV_EP_PTR(dev, ep_num);
	return ep->in.head_idx == ep->in.tail_idx;
}

static ALWAYS_INLINE bool zynq_udc_din_full(const struct device *dev, uint8_t ep_num)
{
	struct ep_priv *ep = DEV_EP_PTR(dev, ep_num);
	uint8_t next_idx = (ep->in.head_idx + 1) % ep->in.size + (ep_num == 0 ? ep->out.size : 0);
	return  next_idx == ep->in.tail_idx;
}

static ALWAYS_INLINE bool zynq_udc_dout_empty(const struct device *dev, uint8_t ep_num)
{
	struct ep_priv *ep = DEV_EP_PTR(dev, ep_num);
	return ep->out.head_idx == ep->out.tail_idx;
}
static ALWAYS_INLINE bool zynq_udc_dout_full(const struct device *dev, uint8_t ep_num)
{
	struct ep_priv *ep = DEV_EP_PTR(dev, ep_num);
	uint8_t next_idx = (ep->out.head_idx + 1) % ep->out.size;
	return next_idx == ep->out.tail_idx;
}


static ALWAYS_INLINE struct zynq_udc_dtd * zynq_udc_din_td_tail(const struct device *dev, uint8_t ep_num)
{
	struct ep_priv *ep = DEV_EP_PTR(dev, ep_num);
	if(zynq_udc_din_empty(dev, ep_num)) {
		return NULL;
	}
	return ep->dtd_ptrs[ep->in.tail_idx];
}

static ALWAYS_INLINE struct zynq_udc_dtd * zynq_udc_din_td_head(const struct device *dev, uint8_t ep_num)
{
	struct ep_priv *ep = DEV_EP_PTR(dev, ep_num);
	// check if the pipe is full
	if(zynq_udc_din_full(dev, ep_num)) {
		return NULL;
	}
	return ep->dtd_ptrs[ep->in.head_idx];
}

static ALWAYS_INLINE struct zynq_udc_dtd * zynq_udc_dout_td_tail(const struct device *dev, uint8_t ep_num)
{
	struct ep_priv *ep = DEV_EP_PTR(dev, ep_num);
	if(zynq_udc_dout_empty(dev, ep_num)) {
		return NULL;
	}
	return ep->dtd_ptrs[ep->out.tail_idx];
}

static ALWAYS_INLINE struct zynq_udc_dtd * zynq_udc_dout_td_head(const struct device *dev, uint8_t ep_num)
{
	struct ep_priv *ep = DEV_EP_PTR(dev, ep_num);
	// check if the pipe is full
	if(zynq_udc_dout_full(dev, ep_num)) {
		return NULL;
	}
	return ep->dtd_ptrs[ep->out.head_idx];
}

static ALWAYS_INLINE void zynq_udc_set_max_packet_len_iso(struct zynq_udc_dqh *dqh,
							  const struct udc_ep_config *ep_cfg,
							  uint32_t len)
{
	uint32_t mult = (len & ENDPOINT_MAXP_MULT_MASK) >> ENDPOINT_MAXP_MULT_SHIFT;
	uint32_t max_pkt_size = (mult > 1) ? ep_cfg->mps : len;

	if (max_pkt_size > ep_cfg->mps) {
		return;
	}
	if (mult > 3) {
		return;
	}
	dqh->max_packet_len = max_pkt_size;
	dqh->mult = mult;
}

/**
 * @brief
 *
 * @param dev
 */
static void udc_zynq_init_eplist(const struct device *dev);

/**
 * @brief
 *
 * @param dev
 */
static void udc_zynq_reset_post(const struct device *dev);

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
	zynq_usb_hw_reset(dev);
}

/**
 * @brief Attach buffer to transfer descriptor
 *
 * @param td
 * @param buf
 * @param cfg
 * @return int
 */
static int zynq_td_attach_buf(struct zynq_udc_dtd *const td, struct net_buf *buf,
			      struct udc_ep_config *const cfg)
{
	// write data size of transfer descriptor
	// then prime the endpoint

	uintptr_t buf_addr;
	uintptr_t buf_end;
	uint32_t ptr_num;

	if (unlikely(buf->len > XUSBPS_dTD_BUF_MAX_SIZE)) {
		LOG_ERR("Data size exceeds max transfer size");
		return -EINVAL;
	}
	buf_addr = (uintptr_t)buf->data;
	td->pages[0] = buf_addr;

	if (buf->len > 0) {
		buf_end = buf_addr + buf->len - 1;
		ptr_num = 1;

		while ((buf_addr & 0xFFFFF000) != (buf_end & 0xFFFFF000)) {
			buf_addr = (buf_addr + 0x1000) & 0xFFFFF000;
			td->pages[ptr_num] = buf_addr;
			ptr_num++;
		}
	}
	td->total_bytes = buf->len;
	td->terminate = 1;
	td->irq_on_cmpl = 1;

	((struct dtd_aligned *)(td))->user_data = (uintptr_t)buf;

	/**
	 * zlt=0, with zero length termination
	 * zlt=1, without zero length termination
	 */
	// dqh->zlt = !bi->zlp; we can't set zlt per buffer, we need to set it per transfer
	// we shall set the zlt before we prime the endpoint(before sent we get the buffer info from
	// the td's user_data)

	return 0;
}

static int zynq_handle_evt_reset(const struct device *dev)
{
	// partial reset
	zynq_usb_prime_and_flush_all(dev);
	return udc_submit_event(dev, UDC_EVT_RESET, 0);
}

static int zynq_handle_evt_dout_i(const struct device *dev, uint8_t i)
{
	const struct udc_ep_config *cfg = EP_CFG_IDX(dev, i, USB_EP_DIR_OUT);
	bool is_ctrl = (i == 0);

	struct net_buf *buf;
	buf = udc_buf_peek(dev, cfg->addr);
	if (buf == NULL) {
		LOG_ERR("No buffer for ep 0x%02x", cfg->addr);
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
		return -ENOBUFS;
	}
	struct zynq_udc_dtd *td = zynq_udc_dout_td_tail(dev, i);
	if (td == NULL) {
		LOG_ERR("No active td for ep 0x%02x", cfg->addr);
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
		return -ENOBUFS;
	}

	/* check if the td is inactive(RX) */
	if (!td->active) {
		/* we don't have available transfer descriptor */
		udc_submit_event(dev, UDC_EVT_ERROR, -EIO);
	}

	udc_ep_set_busy(dev, cfg->addr, false);

	if (is_ctrl) {
		if (udc_ctrl_stage_is_status_out(dev) || udc_ctrl_stage_is_no_data(dev)) {
			/* Status stage finished, notify upper layer */
			udc_ctrl_submit_status(dev, buf);
		}

		/* Update to next stage of control transfer */
		udc_ctrl_update_stage(dev, buf);

		if (udc_ctrl_stage_is_status_in(dev)) {
			/*
			 * OUT transfer finished, release buffer,
			 * control IN buffer should be already fed.
			 */
			net_buf_unref(buf);
		}

		return 0;
	}

	zynq_udc_td_dout_tail_advance(dev, i);

	/* check if we still have pending rx bufs */
	if (!zynq_udc_dout_empty(dev, i)) {
		// we don't need check if the pipe is full, since we have already advanced the tail idx, at least we have one available td
		const struct zynq_udc_dtd* td_head_ptr = DEV_EP(dev, i).dtd_ptrs[DEV_EP(dev, i).out.head_idx];
		struct zynq_udc_dqh *dqh = EP_DQH_OUT(dev, i);
		// write the td address to the dqh
		dqh->xfer_overlay.next_dtd_nlp = (uint32_t)td_head_ptr;
		// prime the endpoint
		zynq_prime_ep(dev, cfg);
	}

	return udc_submit_ep_event(dev, buf, 0);


}

/**
 * @brief Data out(data received) event handler
 *
 * @param dev
 * @return int
 */
static int zynq_handle_evt_dout(const struct device *dev, uint16_t flag)
{
	const struct udc_zynq_config *cfg = dev->config;
	int start_idx = ffs(flag);
	if (unlikely(start_idx == 0)) {
		return -EINVAL;
	}
	for (uint8_t i = start_idx - 1; i < cfg->num_of_eps; i++) {
		if (flag & BIT(i)) {
			// clear the flag
			flag &= ~BIT(i);
			// handle the event
			zynq_handle_evt_dout_i(dev, i);
		}
		// test if all the flags are cleared
		if (flag == 0) {
			break;
		}
	}
	return 0;
}

/**
 * @brief DIN(data sent) event handler
 * 
 * @param dev 
 * @param i 
 * @return ALWAYS_INLINE 
 */
static int zynq_handle_evt_din_i(const struct device *dev, uint8_t i)
{
	const struct udc_ep_config *cfg = EP_CFG_IDX(dev, i, USB_EP_DIR_IN);
	bool is_ctrl = (i == 0);

	struct net_buf *buf;
	buf = udc_buf_peek(dev, cfg->addr);
	if (buf == NULL) {
		LOG_ERR("No buffer for ep 0x%02x", cfg->addr);
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
		return -ENOBUFS;
	}

	/* check if the td is active */
	struct zynq_udc_dtd *td = zynq_udc_din_td_tail(dev, i);
	if (td == NULL) {
		LOG_ERR("No active td for ep 0x%02x", cfg->addr);
		udc_submit_event(dev, UDC_EVT_ERROR, -ENOBUFS);
		return -ENOBUFS;
	}

	/* check if the td is active */
	if (td->active) {
		/* we don't have available transfer descriptor */
		udc_submit_event(dev, UDC_EVT_ERROR, -EIO);
	}

	udc_ep_set_busy(dev, cfg->addr, false);

	if (is_ctrl) {
		if (udc_ctrl_stage_is_status_in(dev) || udc_ctrl_stage_is_no_data(dev)) {
			/* Status stage finished, notify upper layer */
			udc_ctrl_submit_status(dev, buf);
		}

		/* Update to next stage of control transfer */
		udc_ctrl_update_stage(dev, buf);

		if (udc_ctrl_stage_is_status_out(dev)) {
			/*
			 * IN transfer finished, release buffer,
			 * control OUT buffer should be already fed.
			 */
			net_buf_unref(buf);
		}

		return 0;
	}

	zynq_udc_td_din_tail_advance(dev, i);

	/* check if we still have pending tx bufs */
	if (!zynq_udc_din_empty(dev, i)) {
		// we don't need check if the pipe is full, since we have already advanced the tail idx, at least we have one available td
		const struct zynq_udc_dtd* td_head_ptr = DEV_EP(dev, i).dtd_ptrs[DEV_EP(dev, i).in.head_idx];
		struct zynq_udc_dqh *dqh = EP_DQH_IN(dev, i);
		// write the td address to the dqh
		dqh->xfer_overlay.next_dtd_nlp = (uint32_t)td_head_ptr;
		// prime the endpoint
		zynq_prime_ep(dev, cfg);
	}

	return udc_submit_ep_event(dev, buf, 0);
}

/**
 * @brief Data in(data sent) event handler
 *
 * @param dev
 * @return int
 */
static int zynq_handle_evt_din(const struct device *dev, uint16_t flag)
{
	const struct udc_zynq_config *cfg = dev->config;
	/*
	 * use find first set as the iterator start point
	 */
	int start_idx = ffs(flag);
	if (unlikely(start_idx == 0)) {
		return -EINVAL;
	}
	for (uint8_t i = start_idx - 1; i < cfg->num_of_eps; i++) {
		if (flag & BIT(i)) {
			// clear the flag
			flag &= ~BIT(i);
			// handle the event
			zynq_handle_evt_din_i(dev, i);
		}
		// test if all the flags are cleared
		if (flag == 0) {
			break;
		}
	}
	return 0;
}
 

static int zynq_handle_evt_setup(const struct device *dev)
{
	struct net_buf *buf;
	int err = 0;

	do {
		/* give setup sema4 */
		zynq_usb_set_setup_sema4(dev);
		/* Get the buffer from the CTRL FIFO(Removed from the buffer list) */
		buf = udc_buf_get(dev, USB_CONTROL_EP_OUT);
		if (buf == NULL) {
			LOG_ERR("No buffer queued for control ep");
			return -ENODATA;
		}

		/* copy the setup packet to the buffer */
		net_buf_add_mem(buf, CTRL_OUT_SETUP(dev), sizeof(CTRL_OUT_SETUP(dev)));
		/* toggle the buf's user_data setup flag */
		udc_ep_buf_set_setup(buf);
		LOG_HEXDUMP_DBG(buf->data, buf->len, "setup");

	} while (!zynq_usb_test_setup_sema4(dev));
	zynq_usb_clear_setup_interrupts(dev);

	/* Update to next stage of control transfer */
	udc_ctrl_update_stage(dev, buf);

	/**
	 * @brief Since the USB_STD_REQUEST is always 8 bytes for out channel, except the
	 * SET_DESCRIPTOR(variable length) by give the trip_wire semaphore is enough for buffer
	 * allocation 'We always allocate and feed buffer large enough for a setup packet' is always
	 * true
	 * @note for data in stage, buffer will be allocated at upper layer, and we will enqueued by
	 * ctrl_ep_enqueue
	 */
	if (udc_ctrl_stage_is_data_in(dev)) {
		LOG_DBG("s:%p|feed for -in-status", buf);
		err = udc_ctrl_submit_s_in_status(dev);
	} else {
		LOG_DBG("s:%p|feed >setup", buf);
		err = udc_ctrl_submit_s_status(dev);
	}

	return err;
}

static int zynq_handle_xfer_next(const struct device *dev, struct udc_ep_config *const cfg)
{
	// TODO(pan), we don't support transfer across multiple td, impl it later

	bool dir_in = USB_EP_DIR_IS_IN(cfg->addr);
	struct net_buf *buf;
	buf = udc_buf_peek(dev, cfg->addr);
	if (buf == NULL) {
		return -ENODATA;
	}

	if (buf->len > XUSBPS_dTD_BUF_MAX_SIZE) {
		LOG_ERR("Data size exceeds max transfer size");
		return -EINVAL;
	}

	if (dir_in) {

		/* if full */
		struct zynq_udc_dtd *td = zynq_udc_din_td_head(dev,USB_EP_GET_IDX(cfg->addr));
		if (td == NULL) {
			LOG_ERR("No available td for ep 0x%02x", cfg->addr);
			udc_submit_ep_event(dev, buf, -ENOBUFS);
			return -ENOBUFS;
		}

		/* attach buffer to td */
		zynq_td_attach_buf(td, buf, cfg);

		struct zynq_udc_dqh *dqh = EP_DQH_IN(dev, USB_EP_GET_IDX(cfg->addr));
		dqh->xfer_overlay.next_dtd_nlp = (uint32_t)td;

		zynq_udc_td_din_head_advance(dev, USB_EP_GET_IDX(cfg->addr));

		/* prime the endpoint */
		zynq_prime_ep(dev, cfg);

	} else {
		/* if full */
		struct zynq_udc_dtd *td = zynq_udc_dout_td_head(dev, USB_EP_GET_IDX(cfg->addr));
		if (td == NULL) {
			LOG_ERR("No available td for ep 0x%02x", cfg->addr);
			udc_submit_ep_event(dev, buf, -ENOBUFS);
			return -ENOBUFS;
		}

		/* attach buffer to td */
		zynq_td_attach_buf(td, buf, cfg);

		struct zynq_udc_dqh *dqh = EP_DQH_OUT(dev, USB_EP_GET_IDX(cfg->addr));
		dqh->xfer_overlay.next_dtd_nlp = (uint32_t)td;

		zynq_udc_td_dout_head_advance(dev, USB_EP_GET_IDX(cfg->addr));

		/* prime the endpoint */
		zynq_prime_ep(dev, cfg);
	}
	// TODO(pan), set busy only when the td_fifo is full
	udc_ep_set_busy(dev, cfg->addr, true);
	return 0;
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
	ep_cfg = udc_get_ep_cfg(dev, evt.ep_addr);
	switch (evt.type) {
	case ZYNQ_DRV_EVT_SETUP:
		LOG_DBG("SETUP event");
		zynq_handle_evt_setup(dev);
		break;
	case ZYNQ_DRV_EVT_RESET:
		LOG_DBG("RESET event");
		zynq_handle_evt_reset(dev);
		break;
	case ZYNQ_DRV_EVT_XFER:
		LOG_DBG("New transfer in the queue");
		if (ep_cfg->addr != USB_CONTROL_EP_OUT && !udc_ep_is_busy(dev, ep_cfg->addr)) {
			zynq_handle_xfer_next(dev, ep_cfg);
		}
		break;
	case ZYNQ_DRV_EVT_DIN:
		LOG_DBG("DIN event");
		zynq_handle_evt_din(dev, evt.flag);
		break;
	case ZYNQ_DRV_EVT_DOUT:
		LOG_DBG("DOUT event");
		zynq_handle_evt_dout(dev, evt.flag);
		break;
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
		.ep_addr = cfg->addr,
		.type = ZYNQ_DRV_EVT_XFER,
	};

	LOG_DBG("%p enqueue %p", dev, buf);
	udc_buf_put(cfg, buf);

	if (!cfg->stat.halted) {
		k_msgq_put(&drv_msgq, &evt, K_NO_WAIT);
	}
	return 0;
}

static void udc_zynq_device_prime_and_flush(const struct device *dev,
					    const struct udc_ep_config *cfg)
{

	const uint32_t prime_mask = USB_EP_LUT_IDX(cfg->addr);
	/* clear all setup interrupts by writing the read value of the XUSBPS_EPSTAT_OFFSET */
	if (zynq_usb_ep_is_ctrl(cfg)) {
		zynq_usb_clear_setup_interrupts(dev);
	}
	/* prime clear and flush */
	/* read ep prime sts */
	uint32_t timeout = CONFIG_UDC_ZYNQ_TIMEOUT;
	while ((zynq_usb_read32(dev, XUSBPS_EPPRIME_OFFSET) & prime_mask) && --timeout) {
		/* NOP */
	}

	zynq_usb_set_bits(dev, XUSBPS_EPFLUSH_OFFSET, prime_mask);
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

	buf = udc_buf_get_all(dev, cfg->addr);
	if (buf) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}

	udc_zynq_device_prime_and_flush(dev, cfg);

	/* flush the */
	udc_ep_set_busy(dev, cfg->addr, false);

	return 0;
}

/*
 * Configure and make an endpoint ready for use.
 * This is called in the context of udc_ep_enable() or udc_ep_enable_internal(),
 * the latter of which may be used by the driver to enable control endpoints.
 */

/**
 * @brief According ug585-zynq-7000-TRM.pdf, table 15-6, USB Device Endpoint Initialization
 * 0. Configure the queue head
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
	uint8_t ep_idx = USB_EP_GET_IDX(cfg->addr);
	bool ep_is_out = USB_EP_DIR_IS_OUT(cfg->addr);
	bool ep_is_ctrl = zynq_usb_ep_is_ctrl(cfg);
	uint16_t ep_ctrl_offset = XUSBPS_EPCRn_OFFSET(ep_idx);

	/** Configure the queue head
	 * 0. max packet size
	 * 1. mult, non-isochronous mult=0, isochronous mult config as needed
	 * 2. ios(1), interrupt on setup
	 * 3. zlt(1), zero length termination
	 */

	/* out queue heads */
	if (ep_is_ctrl) {

		if(cfg->addr == USB_CONTROL_EP_OUT) {
			LOG_DBG("Enable ctrl ep 0x%02x, idx %d, out", cfg->addr, ep_idx);
			struct zynq_udc_dqh *dqh_out = EP_DQH_OUT(dev, ep_idx);
			memset(dqh_out, 0, sizeof(struct zynq_udc_dqh));
			dqh_out->max_packet_len = cfg->mps;
			dqh_out->irq_on_setup = 1;
		} else {
			LOG_DBG("Enable ctrl ep 0x%02x, idx %d, din", cfg->addr, ep_idx);
			struct zynq_udc_dqh *dqh_in = EP_DQH_IN(dev, ep_idx);
			memset(dqh_in, 0, sizeof(struct zynq_udc_dqh));
			dqh_in->max_packet_len = cfg->mps;
			dqh_in->irq_on_setup = 1;
		}
		/* set the dqh address */

	}
	/* Configure Endpoint Registers */
	else {
		LOG_DBG("Enable non-ctrl ep 0x%02x, idx %d", cfg->addr, ep_idx);


		/* set endpoint type */
		uint32_t type_mask = ep_is_out ? XUSBPS_EPCR_RXT_TYPE_MASK : XUSBPS_EPCR_TXT_TYPE_MASK;
		uint32_t type_val = 0;
		bool is_iso = false;
		struct zynq_udc_dqh *dqh = ep_is_out ? EP_DQH_OUT(dev, ep_idx) : EP_DQH_IN(dev, ep_idx);

		switch (cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) {
		case USB_EP_TYPE_BULK:
			type_val = ep_is_out ? XUSBPS_EPCR_RXT_BULK_MASK : XUSBPS_EPCR_TXT_BULK_MASK;
			break;
		case USB_EP_TYPE_INTERRUPT:
			type_val = ep_is_out ? XUSBPS_EPCR_RXT_INTR_MASK : XUSBPS_EPCR_TXT_INTR_MASK;
			break;
		case USB_EP_TYPE_ISO:
			is_iso = true;
			type_val = ep_is_out ? XUSBPS_EPCR_RXT_ISO_MASK : XUSBPS_EPCR_TXT_ISO_MASK;
			break;
		default:
			return -EINVAL;
		}

		zynq_usb_mask_write(dev, ep_ctrl_offset, type_mask, type_val);

		/* ISO transactions for full-speed device do not support
		 * toggle sequencing and should only send DATA0 PID.
		 */
		uint32_t toggle_reset = ep_is_out ? XUSBPS_EPCR_RXR_MASK : XUSBPS_EPCR_TXR_MASK;
		/* data toggle reset restart transfers with DATA0 */
		zynq_usb_set_bits(dev, ep_ctrl_offset, toggle_reset);

		if (!is_iso) {
			// for non-iso, we need to clear the toggle inhibit, DATA0/1 will be used
			uint32_t toggle_inhibit = ep_is_out ? XUSBPS_EPCR_RXI_MASK : XUSBPS_EPCR_TXI_MASK;
			zynq_usb_clear_bits(dev, ep_ctrl_offset, toggle_inhibit);
		}

		/* configure the qh */
		if (is_iso) {
			zynq_udc_set_max_packet_len_iso(dqh, cfg, cfg->mps);
			dqh->zlt = 1; // we disable the zero length termination on isochronous transfer
		} else {
			dqh->max_packet_len = cfg->mps;
			dqh->zlt = 0;
		}
	}

	/* clear stall */
	zynq_usb_ep_clear_stall(dev, cfg);

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
	uint8_t ep_idx = USB_EP_GET_IDX(cfg->addr);
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
	/* we can't halt the control endpoint */
	if (cfg->addr == USB_CONTROL_EP_OUT || cfg->addr == USB_CONTROL_EP_IN) {
		return 0;
	}

	zynq_usb_ep_set_stall(dev, cfg);
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

	if (cfg->addr == USB_CONTROL_EP_OUT || cfg->addr == USB_CONTROL_EP_IN) {
		return 0;
	}

	zynq_usb_ep_clear_stall(dev, cfg);
	return 0;
}

static int udc_zynq_set_address(const struct device *dev, const uint8_t addr)
{
	LOG_DBG("Set new address %u for %p", addr, dev);
	if (addr > XUSBPS_DEVICEADDR_MAX) {
		return -EINVAL;
	}

	zynq_usb_write32(dev, XUSBPS_DEVICEADDR_OFFSET,
			 (addr << XUSBPS_DEVICEADDR_ADDR_SHIFT) |
				 XUSBPS_DEVICEADDR_DEVICEAADV_MASK);
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
	uint32_t port_status =
		zynq_usb_read32(dev, XUSBPS_PORTSCR1_OFFSET) & XUSBPS_PORTSCR_PSPD_MASK;

	if (port_status == XUSBPS_PORTSCR_PSPD_FS) {
		return UDC_BUS_SPEED_FS;
	} else if (port_status == XUSBPS_PORTSCR_PSPD_HS) {
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
		LOG_ERR("Failed to enable control endpoint %d", ret);
		return ret;
	}
	ret = udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL,
				     cfg->ep_cfg_in[0].mps, 0);
	if (ret) {
		LOG_ERR("Failed to enable control endpoint %d", ret);
		return ret;
	}

	/**
	 *@brief configure the interrupt
	 * i. Disable all the interrupts
	 * ii. configure the interrupts 1. USB Transaction Complete, 2. Trans Error 3. Port
	 *Change 4. SoF 5. Suspend
	 */
	zynq_usb_write32(dev, XUSBPS_IER_OFFSET, 0);
	zynq_usb_set_bits(dev, XUSBPS_IER_OFFSET,
			/* usb transaction complete */ XUSBPS_IXR_UI_MASK |
				  /* Trans Error */ XUSBPS_IXR_UE_MASK |
				  /* Port Change Detect */ XUSBPS_IXR_PC_MASK |
				  /* Reset Received */ XUSBPS_IXR_UR_MASK |
				  /* Start Of Frame */ XUSBPS_IXR_SR_MASK |
				  /* Device Controller Suspend */ XUSBPS_IXR_SLE_MASK);


	cfg->config_func(dev, true); // connect the interrupt
	/* toggle the start bit */
	zynq_usb_run(dev);

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

/**
 * @brief Reset logic
 * 1. setup interrupts clear
 * 2. prime clear, flush
 * 3. USBCMD[RS]=0, stop the controller
 * 4. USBCMD[RST]=1, reset the controller, then poll the reset bit
 */
static void udc_zynq_device_reset(const struct device *dev)
{
	/* clear all setup token sema4 by writing the read value of the XUSBPS_EPSTAT_OFFSET */
	zynq_usb_clear_setup_interrupts(dev);
	/* prime clear and flush */
	zynq_usb_prime_and_flush_all(dev);

	/* stop the controller */
	/* hw_reset */
	zynq_usb_controller_reset(dev);
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
	LOG_DBG("Init device %p", dev);
	udc_zynq_device_reset(dev);

	udc_zynq_init_eplist(dev);

	udc_zynq_reset_post(dev);
	return 0;
}

/* Shut down the controller completely */
static int udc_zynq_shutdown(const struct device *dev)
{
	LOG_DBG("Shutdown device %p", dev);
	// udc_zynq_device_reset(dev);
	zynq_usb_stop(dev);
	return 0;
}

static void zynq_udc_isr(const struct device *dev)
{
	uint32_t usb_sts = zynq_usb_read_write32(dev, XUSBPS_ISR_OFFSET);
	if (usb_sts & XUSBPS_IXR_UI_MASK) {
		/* USB Transaction Complete */
		uint32_t ep_stat = zynq_usb_read32(dev, XUSBPS_EPSTAT_OFFSET);
		if (ep_stat) {
			if (ep_stat & 0x1) {
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
			zynq_usb_write32(dev, XUSBPS_EPSTAT_OFFSET, ep_stat);
		}

		/* check for Rx/Tx complete intr */
		uint32_t ep_compl = zynq_usb_read32(dev, XUSBPS_EPCOMPL_OFFSET);
		if (ep_compl) {
			struct zynq_drv_event evt = {
				.dev = dev,
				.type = (ep_compl & XUSBPS_EPCOMPL_RX_MASK) ? ZYNQ_DRV_EVT_DOUT
									    : ZYNQ_DRV_EVT_DIN,
				.flag = ep_compl,
			};
			k_msgq_put(&drv_msgq, &evt, K_NO_WAIT);

			// clear the complete
			zynq_usb_write_complete(dev, ep_compl);
		}
	}

	if (usb_sts & XUSBPS_IXR_PC_MASK) {
		uint32_t port_sts = zynq_usb_read32(dev, XUSBPS_PORTSCR1_OFFSET);
		udc_submit_event(dev,
				 (port_sts & XUSBPS_PORTSCR_CCS_MASK) ? UDC_EVT_VBUS_READY
								      : UDC_EVT_VBUS_REMOVED,
				 0);
	}

	if (usb_sts & XUSBPS_IXR_UE_MASK) {
		udc_submit_event(dev, UDC_EVT_ERROR, 0);
	}

	if (usb_sts & XUSBPS_IXR_UR_MASK) {
		struct zynq_drv_event evt = {
			.dev = dev,
			.type = ZYNQ_DRV_EVT_RESET,
		};
		k_msgq_put(&drv_msgq, &evt, K_NO_WAIT);
	}

	if (usb_sts & XUSBPS_IXR_SR_MASK) {
		udc_submit_event(dev, UDC_EVT_SOF, 0);
	}
}

static void udc_zynq_init_eplist(const struct device *dev)
{
	const struct udc_zynq_config *config = DEV_CFG(dev);

	memset(config->qh_arr, 0, 2 * sizeof(struct dqh_aligned) * config->num_of_eps);
	memset(config->td_arr, 0, 8 * sizeof(struct dtd_aligned) * config->num_of_eps);

	for (int i = 0; i < config->num_of_eps; i++) {
		struct ep_priv *ep = DEV_EP_PTR(dev, i);
		const struct udc_ep_config *ep_cfg = &config->ep_cfg_out[i];

		ep->dqh_out = (struct zynq_udc_dqh *)(&config->qh_arr[2 * i]);
		ep->dqh_in = (struct zynq_udc_dqh *)(&config->qh_arr[2 * i + 1]);

		for (int j = 0; j < CONFIG_UDC_ZYNQ_TD_PER_EP; j++) {
			ep->dtd_ptrs[j] = (struct zynq_udc_dtd *)(&config->td_arr[8 * i + j]);
		}
#define CONFIG_UDC_ZYNQ_TD_PER_EP_HALF (CONFIG_UDC_ZYNQ_TD_PER_EP / 2)
		if (i == 0) {
			for (int j = 0; j < CONFIG_UDC_ZYNQ_TD_PER_EP_HALF; j++) {
				uint32_t next_idx = (j + 1) % (CONFIG_UDC_ZYNQ_TD_PER_EP_HALF);
				ep->dtd_ptrs[j]->next_dtd_nlp = (uint32_t)ep->dtd_ptrs[next_idx];
			}
			for (int j = CONFIG_UDC_ZYNQ_TD_PER_EP / 2; j < CONFIG_UDC_ZYNQ_TD_PER_EP;
			     j++) {
				uint32_t next_idx = (j + 1) % CONFIG_UDC_ZYNQ_TD_PER_EP_HALF +
						    CONFIG_UDC_ZYNQ_TD_PER_EP_HALF;
				ep->dtd_ptrs[j]->next_dtd_nlp = (uint32_t)ep->dtd_ptrs[next_idx];
			}
			/* set the dqh, both direction are used */
			ep->dqh_out->ep_cur_dtd = (uint32_t)ep->dtd_ptrs[0];
			ep->dqh_in->ep_cur_dtd = (uint32_t)ep->dtd_ptrs[4];

			ep->dqh_out->max_packet_len = config->ep_cfg_out[i].mps;
			ep->dqh_out->irq_on_setup = 1;

			ep->dqh_in->max_packet_len = config->ep_cfg_in[i].mps;
			ep->dqh_in->irq_on_setup = 1;

			ep->dqh_out->irq_on_setup = 1;
			ep->dqh_out->max_packet_len = ep_cfg->mps;
		}
		/* non-ctrl endpoint */
		else {
			/* set the pointer of transfer descriptor */
			for (int j = 0; j < CONFIG_UDC_ZYNQ_TD_PER_EP; j++) {
				uint32_t next_idx = (j + 1) % CONFIG_UDC_ZYNQ_TD_PER_EP;
				ep->dtd_ptrs[j]->next_dtd_nlp = (uint32_t)ep->dtd_ptrs[next_idx];
			}
			/* set the dqh */
			ep->dqh_out->ep_cur_dtd = (uint32_t)ep->dtd_ptrs[0];
			ep->dqh_in->ep_cur_dtd = (uint32_t)ep->dtd_ptrs[0];
		}
	}
	// const struct udc_zynq_config *config = DEV_CFG(dev);
	/* set queue head list addr */
	zynq_usb_write32(dev, XUSBPS_EPLISTADDR_OFFSET, (uint32_t)config->qh_arr);

}

static void udc_zynq_reset_post(const struct device *dev)
{
	/* CM(controller mode) in device mode, SLO(setup lockout) mode */
	zynq_usb_write32(dev, XUSBPS_MODE_OFFSET, XUSBPS_MODE_CM_DEVICE_MASK | XUSBPS_MODE_SLOM_MASK);
	/* OTG Termination, mandatory in device mode */
	zynq_usb_set_bits(dev, XUSBPS_OTGCSR_OFFSET, XUSBPS_OTGSC_OT_MASK);
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

#ifdef CONFIG_PINCTRL
#include <zephyr/drivers/pinctrl.h>
#endif

/*
 * This is called once to initialize the controller and endpoints
 * capabilities, and register endpoint structures.
 */
static int udc_zynq_driver_preinit(const struct device *dev)
{
	const struct udc_zynq_config *config = dev->config;
	struct udc_data *data = dev->data;
	int err;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

#ifdef CONFIG_PINCTRL
	/** note！
	we only configure the pins but not the PLL/CLK of SDIO [0xF8000150:0xF8001E03]
	(we shall configure these part in u-boot/fsbl) */
	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	LOG_DBG("pinctrl apply %d pins", config->pincfg->state_cnt);
	if (err < 0) {
		LOG_ERR("Failed to apply pin state %d", err);
		return err;
	}
#endif
	if (gpio_is_ready_dt(&config->phy_reset)) {

		err = gpio_pin_configure_dt(&config->phy_reset, 0);
		if (err < 0) {
			LOG_ERR("Could not configure reset GPIO (%d)", err);
			return err;
		}
		gpio_pin_set(config->phy_reset.port, config->phy_reset.pin, 0);
		k_sleep(K_MSEC(1));
		gpio_pin_set(config->phy_reset.port, config->phy_reset.pin, 1);
	}

	/*
	 * You do not need to initialize it if your driver does not use
	 * udc_lock_internal() / udc_unlock_internal(), but implements its
	 * own mechanism.
	 */
	k_mutex_init(&data->mutex);

	data->caps.rwup = true;
	data->caps.out_ack = false;
	data->caps.mps0 = UDC_MPS0_64;
	if (config->speed_idx == 2) {
		data->caps.hs = true;
	}
	data->caps.can_detect_vbus = true;

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

	LOG_INF("Device %p (max. speed %d)", dev, config->speed_idx);

	return 0;
}

#if CONFIG_PINCTRL
#define UDC_ZYNQ_PINCTRL_DEFINE(port) PINCTRL_DT_INST_DEFINE(port)
#define UDC_ZYNQ_PINCTRL_INIT(port)   .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(port)
#else
#define UDC_ZYNQ_PINCTRL_DEFINE(port)
#define UDC_ZYNQ_PINCTRL_INIT(port)
#endif /* CONFIG_PINCTRL */



/*
 * A UDC driver should always be implemented as a multi-instance
 * driver, even if your platform does not require it.
 */
#define UDC_ZYNQ_DEVICE_DEFINE(n)                                                                  \
	UDC_ZYNQ_PINCTRL_DEFINE(n);                                                                 \
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
		while (true) {                                                                     \
			zynq_thread_handler(dev);                                                  \
		}                                                                                  \
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
	static struct ep_priv eps_##n[DT_INST_PROP(n, num_bidir_endpoints)];                       \
	static struct dqh_aligned qh_##n[2 * DT_INST_PROP(n, num_bidir_endpoints)];                \
	static struct dtd_aligned td_##n[8 * DT_INST_PROP(n, num_bidir_endpoints)];                \
                                                                                                   \
	static const struct udc_zynq_config udc_zynq_config_##n = {                                \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.phy_reset = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {0}),			\
		.num_of_eps = DT_INST_PROP(n, num_bidir_endpoints),                                \
		.ep_cfg_in = ep_cfg_in_##n,                                                        \
		.ep_cfg_out = ep_cfg_out_##n,                                                      \
		.make_thread = udc_zynq_make_thread_##n,                                           \
		.speed_idx = DT_ENUM_IDX(DT_DRV_INST(n), maximum_speed),                           \
		.qh_arr = qh_##n,                                                                  \
		.td_arr = td_##n,                                                                  \
		.config_func = zynq_udc_##n##_irq_config_func,                                     \
		UDC_ZYNQ_PINCTRL_INIT(n),                                                          \
	};                                                                                         \
                                                                                                   \
	static struct zynq_udc_data udc_priv_##n = {                                               \
		.eps = eps_##n,                                                                    \
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
