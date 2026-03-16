/*
 * Xilinx Zynq PS UART (Cadence) driver
 *
 * Copyright (c) 2018 Xilinx, Inc.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Register reference: Zynq-7000 TRM (ug585), chapter B.33
 */

#define DT_DRV_COMPAT xlnx_xuartps

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/types.h>
#include <zephyr/init.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/irq.h>

#ifdef CONFIG_PINCTRL
#include <zephyr/drivers/pinctrl.h>
#endif

/* Register offsets */
#define XUARTPS_CR_OFFSET      0x0000U
#define XUARTPS_MR_OFFSET      0x0004U
#define XUARTPS_IER_OFFSET     0x0008U
#define XUARTPS_IDR_OFFSET     0x000CU
#define XUARTPS_IMR_OFFSET     0x0010U
#define XUARTPS_ISR_OFFSET     0x0014U
#define XUARTPS_BAUDGEN_OFFSET 0x0018U
#define XUARTPS_RXTOUT_OFFSET  0x001CU
#define XUARTPS_RXWM_OFFSET    0x0020U
#define XUARTPS_MODEMCR_OFFSET 0x0024U
#define XUARTPS_MODEMSR_OFFSET 0x0028U
#define XUARTPS_SR_OFFSET      0x002CU
#define XUARTPS_FIFO_OFFSET    0x0030U
#define XUARTPS_BAUDDIV_OFFSET 0x0034U
#define XUARTPS_FLOWDEL_OFFSET 0x0038U
#define XUARTPS_TXWM_OFFSET    0x0044U

/* Control Register */
#define XUARTPS_CR_STOPBRK     0x00000100U
#define XUARTPS_CR_TX_DIS      0x00000020U
#define XUARTPS_CR_TX_EN       0x00000010U
#define XUARTPS_CR_RX_DIS      0x00000008U
#define XUARTPS_CR_RX_EN       0x00000004U
#define XUARTPS_CR_EN_DIS_MASK 0x0000003CU
#define XUARTPS_CR_TXRST       0x00000002U
#define XUARTPS_CR_RXRST       0x00000001U

/* Mode Register */
#define XUARTPS_MR_STOPMODE_2_BIT   0x00000080U
#define XUARTPS_MR_STOPMODE_1_5_BIT 0x00000040U
#define XUARTPS_MR_STOPMODE_1_BIT   0x00000000U
#define XUARTPS_MR_STOPMODE_MASK    0x000000C0U /* bits [7:6] */
#define XUARTPS_MR_PARITY_NONE      0x00000020U
#define XUARTPS_MR_PARITY_MARK      0x00000018U
#define XUARTPS_MR_PARITY_SPACE     0x00000010U
#define XUARTPS_MR_PARITY_ODD       0x00000008U
#define XUARTPS_MR_PARITY_EVEN      0x00000000U
#define XUARTPS_MR_PARITY_MASK      0x00000038U /* bits [5:3] */
#define XUARTPS_MR_CHARLEN_6_BIT    0x00000006U
#define XUARTPS_MR_CHARLEN_7_BIT    0x00000004U
#define XUARTPS_MR_CHARLEN_8_BIT    0x00000000U
#define XUARTPS_MR_CHARLEN_MASK     0x00000006U /* bits [2:1] */

/* Interrupt bits (shared by IER/IDR/IMR/ISR) */
#define XUARTPS_IXR_RBRK    0x00002000U
#define XUARTPS_IXR_TOVR    0x00001000U
#define XUARTPS_IXR_TTRIG   0x00000400U
#define XUARTPS_IXR_TOUT    0x00000100U
#define XUARTPS_IXR_PARITY  0x00000080U
#define XUARTPS_IXR_FRAMING 0x00000040U
#define XUARTPS_IXR_RXOVR   0x00000020U
#define XUARTPS_IXR_TXEMPTY 0x00000008U
#define XUARTPS_IXR_RTRIG   0x00000001U
#define XUARTPS_IXR_MASK    0x00003FFFU

#define XUARTPS_IXR_TX_IRQS (XUARTPS_IXR_TTRIG | XUARTPS_IXR_TXEMPTY)
#define XUARTPS_IXR_ERR_IRQS \
	(XUARTPS_IXR_RBRK | XUARTPS_IXR_TOVR | XUARTPS_IXR_TOUT | \
	 XUARTPS_IXR_PARITY | XUARTPS_IXR_FRAMING | XUARTPS_IXR_RXOVR)

/* Modem Control Register */
#define XUARTPS_MODEMCR_FCM_RTS_CTS 0x00000020U
#define XUARTPS_MODEMCR_FCM_NONE    0x00000000U
#define XUARTPS_MODEMCR_FCM_MASK    0x00000020U

/* Channel Status Register */
#define XUARTPS_SR_TTRIG   0x00002000U
#define XUARTPS_SR_TXFULL  0x00000010U
#define XUARTPS_SR_TXEMPTY 0x00000008U
#define XUARTPS_SR_RXEMPTY 0x00000002U

/* Max iterations for poll_out before giving up */
#define POLL_OUT_TIMEOUT_US 100000U

struct uart_xlnx_ps_dev_config {
	DEVICE_MMIO_ROM;
	uint32_t sys_clk_freq;
	uint32_t baud_rate;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif
#ifdef CONFIG_PINCTRL
	const struct pinctrl_dev_config *pincfg;
#endif
};

struct uart_xlnx_ps_dev_data_t {
	DEVICE_MMIO_RAM;
	uint32_t baud_rate;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	const struct device *dev;
	struct k_work tx_work;
	struct k_work rx_work;
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
#endif
};

static void xlnx_ps_disable_uart(uintptr_t reg_base)
{
	uint32_t reg_val = sys_read32(reg_base + XUARTPS_CR_OFFSET);

	reg_val &= ~XUARTPS_CR_EN_DIS_MASK;
	reg_val |= XUARTPS_CR_TX_DIS | XUARTPS_CR_RX_DIS;
	sys_write32(reg_val, reg_base + XUARTPS_CR_OFFSET);
}

static void xlnx_ps_enable_uart(uintptr_t reg_base)
{
	uint32_t reg_val = sys_read32(reg_base + XUARTPS_CR_OFFSET);

	reg_val &= ~XUARTPS_CR_EN_DIS_MASK;
	reg_val |= XUARTPS_CR_TX_EN | XUARTPS_CR_RX_EN;
	sys_write32(reg_val, reg_base + XUARTPS_CR_OFFSET);
}

/**
 * @brief Calculate and apply baud rate divisors.
 *
 * Searches all valid divisor values (4..254) for the combination that
 * produces the lowest baud rate error. Writes BAUDDIV and BAUDGEN
 * registers. Must be called with UART disabled.
 */
static void set_baudrate(const struct device *dev, uint32_t baud_rate)
{
	const struct uart_xlnx_ps_dev_config *dev_cfg = dev->config;
	uint32_t clk_freq = dev_cfg->sys_clk_freq;
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	if (baud_rate == 0 || clk_freq == 0) {
		return;
	}

	uint32_t baud = baud_rate;

	if (clk_freq < 1000000U && baud > 4800U) {
		baud = 4800;
	}

	uint32_t best_divisor = 4;
	uint32_t best_generator = 0;
	uint32_t best_err = UINT32_MAX;

	for (uint32_t divisor = 4; divisor < 255; divisor++) {
		uint32_t generator = clk_freq / (baud * (divisor + 1));

		if (generator < 2 || generator > 65535) {
			continue;
		}

		uint32_t actual = clk_freq / (generator * (divisor + 1));
		uint32_t err = (baud > actual) ? (baud - actual) : (actual - baud);

		if (err < best_err) {
			best_err = err;
			best_divisor = divisor;
			best_generator = generator;
			if (err == 0) {
				break;
			}
		}
	}

	if (best_generator >= 2) {
		sys_write32(best_divisor, reg_base + XUARTPS_BAUDDIV_OFFSET);
		sys_write32(best_generator, reg_base + XUARTPS_BAUDGEN_OFFSET);
	}
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static void uart_xlnx_ps_soft_isr(struct k_work *work)
{
	struct uart_xlnx_ps_dev_data_t *data =
		CONTAINER_OF(work, struct uart_xlnx_ps_dev_data_t, tx_work);

	if (data->user_cb) {
		data->user_cb(data->dev, data->user_data);
	}
}

static void uart_xlnx_ps_soft_rx_isr(struct k_work *work)
{
	struct uart_xlnx_ps_dev_data_t *data =
		CONTAINER_OF(work, struct uart_xlnx_ps_dev_data_t, rx_work);

	if (data->user_cb) {
		data->user_cb(data->dev, data->user_data);
	}
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int uart_xlnx_ps_init(const struct device *dev)
{
	const struct uart_xlnx_ps_dev_config *dev_cfg = dev->config;
	struct uart_xlnx_ps_dev_data_t *dev_data = dev->data;
	uint32_t reg_val;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

#ifdef CONFIG_PINCTRL
	int err = pinctrl_apply_state(dev_cfg->pincfg, PINCTRL_STATE_DEFAULT);

	if (err < 0) {
		return err;
	}
#endif

	/* Reset TX/RX and disable */
	sys_write32(XUARTPS_CR_STOPBRK | XUARTPS_CR_TX_DIS | XUARTPS_CR_RX_DIS |
		    XUARTPS_CR_TXRST | XUARTPS_CR_RXRST,
		    reg_base + XUARTPS_CR_OFFSET);

	/* Default: 8N1 */
	reg_val = sys_read32(reg_base + XUARTPS_MR_OFFSET);
	reg_val &= ~(XUARTPS_MR_CHARLEN_MASK | XUARTPS_MR_STOPMODE_MASK | XUARTPS_MR_PARITY_MASK);
	reg_val |= XUARTPS_MR_CHARLEN_8_BIT | XUARTPS_MR_STOPMODE_1_BIT | XUARTPS_MR_PARITY_NONE;
	sys_write32(reg_val, reg_base + XUARTPS_MR_OFFSET);

	/* RX FIFO trigger at 1 byte */
	sys_write32(0x01U, reg_base + XUARTPS_RXWM_OFFSET);

	/* Disable all interrupts */
	sys_write32(XUARTPS_IXR_MASK, reg_base + XUARTPS_IDR_OFFSET);

	/* Set baud rate */
	dev_data->baud_rate = dev_cfg->baud_rate;
	set_baudrate(dev, dev_data->baud_rate);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	sys_write32(XUARTPS_IXR_MASK, reg_base + XUARTPS_ISR_OFFSET);
	dev_data->dev = dev;
	k_work_init(&dev_data->tx_work, uart_xlnx_ps_soft_isr);
	k_work_init(&dev_data->rx_work, uart_xlnx_ps_soft_rx_isr);
	dev_cfg->irq_config_func(dev);
#endif

	xlnx_ps_enable_uart(reg_base);
	return 0;
}

static int uart_xlnx_ps_poll_in(const struct device *dev, unsigned char *c)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	if (sys_read32(reg_base + XUARTPS_SR_OFFSET) & XUARTPS_SR_RXEMPTY) {
		return -1;
	}

	*c = (unsigned char)sys_read32(reg_base + XUARTPS_FIFO_OFFSET);
	return 0;
}

static void uart_xlnx_ps_poll_out(const struct device *dev, unsigned char c)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t retries = POLL_OUT_TIMEOUT_US / 10;

	/* Bounded wait — poll_out() returns void per Zephyr API, so a
	 * timeout silently drops the byte. This is a deliberate trade-off:
	 * upstream has an unbounded loop that can hang forever if TX FIFO
	 * is stuck. The 100ms timeout makes poll_out lossy under sustained
	 * backpressure but prevents infinite hangs. Callers needing
	 * reliable output should use interrupt-driven or async UART APIs.
	 */
	while ((sys_read32(reg_base + XUARTPS_SR_OFFSET) & XUARTPS_SR_TXFULL) != 0) {
		if (--retries == 0) {
			return;
		}
		k_busy_wait(10);
	}

	sys_write32((uint32_t)(c & 0xFF), reg_base + XUARTPS_FIFO_OFFSET);
}

static int uart_xlnx_ps_err_check(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t isr = sys_read32(reg_base + XUARTPS_ISR_OFFSET);
	int err = 0;

	if (isr & XUARTPS_IXR_RBRK) {
		err |= UART_BREAK;
	}
	if (isr & XUARTPS_IXR_PARITY) {
		err |= UART_ERROR_PARITY;
	}
	if (isr & XUARTPS_IXR_FRAMING) {
		err |= UART_ERROR_FRAMING;
	}
	if (isr & XUARTPS_IXR_RXOVR) {
		err |= UART_ERROR_OVERRUN;
	}

	/* Clear only the error flags we checked */
	sys_write32(isr & (XUARTPS_IXR_RBRK | XUARTPS_IXR_PARITY |
			   XUARTPS_IXR_FRAMING | XUARTPS_IXR_RXOVR),
		    reg_base + XUARTPS_ISR_OFFSET);
	return err;
}

/* Mode register <-> uart_config conversion helpers */

static inline bool cfg2ll_parity(uint32_t *mr, enum uart_config_parity p)
{
	switch (p) {
	case UART_CFG_PARITY_NONE:  *mr |= XUARTPS_MR_PARITY_NONE;  break;
	case UART_CFG_PARITY_ODD:   *mr |= XUARTPS_MR_PARITY_ODD;   break;
	case UART_CFG_PARITY_EVEN:  *mr |= XUARTPS_MR_PARITY_EVEN;  break;
	case UART_CFG_PARITY_MARK:  *mr |= XUARTPS_MR_PARITY_MARK;  break;
	case UART_CFG_PARITY_SPACE: *mr |= XUARTPS_MR_PARITY_SPACE; break;
	default: return false;
	}
	return true;
}

static inline bool cfg2ll_stopbits(uint32_t *mr, enum uart_config_stop_bits s)
{
	switch (s) {
	case UART_CFG_STOP_BITS_1:   *mr |= XUARTPS_MR_STOPMODE_1_BIT;   break;
	case UART_CFG_STOP_BITS_1_5: *mr |= XUARTPS_MR_STOPMODE_1_5_BIT; break;
	case UART_CFG_STOP_BITS_2:   *mr |= XUARTPS_MR_STOPMODE_2_BIT;   break;
	default: return false;
	}
	return true;
}

static inline bool cfg2ll_databits(uint32_t *mr, enum uart_config_data_bits d)
{
	switch (d) {
	case UART_CFG_DATA_BITS_8: *mr |= XUARTPS_MR_CHARLEN_8_BIT; break;
	case UART_CFG_DATA_BITS_7: *mr |= XUARTPS_MR_CHARLEN_7_BIT; break;
	case UART_CFG_DATA_BITS_6: *mr |= XUARTPS_MR_CHARLEN_6_BIT; break;
	default: return false;
	}
	return true;
}

static inline bool cfg2ll_hwctrl(uint32_t *mcr, enum uart_config_flow_control f)
{
	switch (f) {
	case UART_CFG_FLOW_CTRL_NONE:    *mcr |= XUARTPS_MODEMCR_FCM_NONE;    break;
	case UART_CFG_FLOW_CTRL_RTS_CTS: *mcr |= XUARTPS_MODEMCR_FCM_RTS_CTS; break;
	default: return false;
	}
	return true;
}

static inline enum uart_config_parity ll2cfg_parity(uint32_t mr)
{
	switch (mr & XUARTPS_MR_PARITY_MASK) {
	case XUARTPS_MR_PARITY_ODD:   return UART_CFG_PARITY_ODD;
	case XUARTPS_MR_PARITY_SPACE: return UART_CFG_PARITY_SPACE;
	case XUARTPS_MR_PARITY_MARK:  return UART_CFG_PARITY_MARK;
	case XUARTPS_MR_PARITY_NONE:  return UART_CFG_PARITY_NONE;
	default:                      return UART_CFG_PARITY_EVEN;
	}
}

static inline enum uart_config_stop_bits ll2cfg_stopbits(uint32_t mr)
{
	switch (mr & XUARTPS_MR_STOPMODE_MASK) {
	case XUARTPS_MR_STOPMODE_1_5_BIT: return UART_CFG_STOP_BITS_1_5;
	case XUARTPS_MR_STOPMODE_2_BIT:   return UART_CFG_STOP_BITS_2;
	default:                          return UART_CFG_STOP_BITS_1;
	}
}

static inline enum uart_config_data_bits ll2cfg_databits(uint32_t mr)
{
	switch (mr & XUARTPS_MR_CHARLEN_MASK) {
	case XUARTPS_MR_CHARLEN_7_BIT: return UART_CFG_DATA_BITS_7;
	case XUARTPS_MR_CHARLEN_6_BIT: return UART_CFG_DATA_BITS_6;
	default:                       return UART_CFG_DATA_BITS_8;
	}
}

static inline enum uart_config_flow_control ll2cfg_hwctrl(uint32_t mcr)
{
	return (mcr & XUARTPS_MODEMCR_FCM_MASK) == XUARTPS_MODEMCR_FCM_RTS_CTS
		       ? UART_CFG_FLOW_CTRL_RTS_CTS
		       : UART_CFG_FLOW_CTRL_NONE;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE

static int uart_xlnx_ps_configure(const struct device *dev, const struct uart_config *cfg)
{
	struct uart_xlnx_ps_dev_data_t *dev_data = dev->data;
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t mr, mcr;

	mr = sys_read32(reg_base + XUARTPS_MR_OFFSET);
	mcr = sys_read32(reg_base + XUARTPS_MODEMCR_OFFSET);

	mr &= ~(XUARTPS_MR_PARITY_MASK | XUARTPS_MR_STOPMODE_MASK | XUARTPS_MR_CHARLEN_MASK);
	mcr &= ~XUARTPS_MODEMCR_FCM_MASK;

	if (!cfg2ll_parity(&mr, cfg->parity) ||
	    !cfg2ll_stopbits(&mr, cfg->stop_bits) ||
	    !cfg2ll_databits(&mr, cfg->data_bits) ||
	    !cfg2ll_hwctrl(&mcr, cfg->flow_ctrl)) {
		return -ENOTSUP;
	}

	xlnx_ps_disable_uart(reg_base);

	set_baudrate(dev, cfg->baudrate);
	dev_data->baud_rate = cfg->baudrate;

	sys_write32(mr, reg_base + XUARTPS_MR_OFFSET);
	sys_write32(mcr, reg_base + XUARTPS_MODEMCR_OFFSET);

	xlnx_ps_enable_uart(reg_base);
	return 0;
}

static int uart_xlnx_ps_config_get(const struct device *dev, struct uart_config *cfg)
{
	const struct uart_xlnx_ps_dev_data_t *dev_data = dev->data;
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t mr = sys_read32(reg_base + XUARTPS_MR_OFFSET);
	uint32_t mcr = sys_read32(reg_base + XUARTPS_MODEMCR_OFFSET);

	cfg->baudrate = dev_data->baud_rate;
	cfg->parity = ll2cfg_parity(mr);
	cfg->stop_bits = ll2cfg_stopbits(mr);
	cfg->data_bits = ll2cfg_databits(mr);
	cfg->flow_ctrl = ll2cfg_hwctrl(mcr);
	return 0;
}

#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int uart_xlnx_ps_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	int count = 0;

	while (count < size &&
	       (sys_read32(reg_base + XUARTPS_SR_OFFSET) & XUARTPS_SR_TXFULL) == 0) {
		sys_write32((uint32_t)tx_data[count++], reg_base + XUARTPS_FIFO_OFFSET);
	}

	return count;
}

static int uart_xlnx_ps_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	int count = 0;

	while (count < size &&
	       (sys_read32(reg_base + XUARTPS_SR_OFFSET) & XUARTPS_SR_RXEMPTY) == 0) {
		rx_data[count++] = (uint8_t)sys_read32(reg_base + XUARTPS_FIFO_OFFSET);
	}

	return count;
}

static void uart_xlnx_ps_irq_tx_enable(const struct device *dev)
{
	struct uart_xlnx_ps_dev_data_t *dev_data = dev->data;
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	sys_write32(XUARTPS_IXR_TX_IRQS, reg_base + XUARTPS_IER_OFFSET);

	/* If FIFO is already empty/below trigger, HW won't fire a new IRQ. Kick via work. */
	if (sys_read32(reg_base + XUARTPS_SR_OFFSET) & (XUARTPS_SR_TTRIG | XUARTPS_SR_TXEMPTY)) {
		k_work_submit(&dev_data->tx_work);
	}
}

static void uart_xlnx_ps_irq_tx_disable(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	sys_write32(XUARTPS_IXR_TX_IRQS, reg_base + XUARTPS_IDR_OFFSET);
}

static int uart_xlnx_ps_irq_tx_ready(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	return (sys_read32(reg_base + XUARTPS_SR_OFFSET) &
		(XUARTPS_SR_TTRIG | XUARTPS_SR_TXEMPTY)) != 0;
}

static int uart_xlnx_ps_irq_tx_complete(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	return (sys_read32(reg_base + XUARTPS_SR_OFFSET) & XUARTPS_SR_TXEMPTY) != 0;
}

static void uart_xlnx_ps_irq_rx_enable(const struct device *dev)
{
	struct uart_xlnx_ps_dev_data_t *dev_data = dev->data;
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	sys_write32(XUARTPS_IXR_RTRIG, reg_base + XUARTPS_IER_OFFSET);

	/* If FIFO already has data, HW won't fire a new trigger. Kick via work. */
	if ((sys_read32(reg_base + XUARTPS_SR_OFFSET) & XUARTPS_SR_RXEMPTY) == 0) {
		k_work_submit(&dev_data->rx_work);
	}
}

static void uart_xlnx_ps_irq_rx_disable(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	sys_write32(XUARTPS_IXR_RTRIG, reg_base + XUARTPS_IDR_OFFSET);
}

static int uart_xlnx_ps_irq_rx_ready(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	return (sys_read32(reg_base + XUARTPS_SR_OFFSET) & XUARTPS_SR_RXEMPTY) == 0;
}

static void uart_xlnx_ps_irq_err_enable(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	sys_write32(XUARTPS_IXR_ERR_IRQS, reg_base + XUARTPS_IER_OFFSET);
}

static void uart_xlnx_ps_irq_err_disable(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);

	sys_write32(XUARTPS_IXR_ERR_IRQS, reg_base + XUARTPS_IDR_OFFSET);
}

static int uart_xlnx_ps_irq_is_pending(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t imr = sys_read32(reg_base + XUARTPS_IMR_OFFSET);
	uint32_t isr = sys_read32(reg_base + XUARTPS_ISR_OFFSET);

	return (imr & isr) != 0;
}

static int uart_xlnx_ps_irq_update(const struct device *dev)
{
	uintptr_t reg_base = DEVICE_MMIO_GET(dev);
	uint32_t isr = sys_read32(reg_base + XUARTPS_ISR_OFFSET);
	uint32_t imr = sys_read32(reg_base + XUARTPS_IMR_OFFSET);

	/* Clear only the flags that are both pending and enabled */
	sys_write32(isr & imr, reg_base + XUARTPS_ISR_OFFSET);
	return 1;
}

static void uart_xlnx_ps_irq_callback_set(const struct device *dev,
					   uart_irq_callback_user_data_t cb, void *cb_data)
{
	struct uart_xlnx_ps_dev_data_t *dev_data = dev->data;

	dev_data->user_cb = cb;
	dev_data->user_data = cb_data;
}

static void uart_xlnx_ps_isr(const struct device *dev)
{
	const struct uart_xlnx_ps_dev_data_t *data = dev->data;

	if (data->user_cb) {
		data->user_cb(dev, data->user_data);
	}
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static DEVICE_API(uart, uart_xlnx_ps_driver_api) = {
	.poll_in = uart_xlnx_ps_poll_in,
	.poll_out = uart_xlnx_ps_poll_out,
	.err_check = uart_xlnx_ps_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_xlnx_ps_configure,
	.config_get = uart_xlnx_ps_config_get,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_xlnx_ps_fifo_fill,
	.fifo_read = uart_xlnx_ps_fifo_read,
	.irq_tx_enable = uart_xlnx_ps_irq_tx_enable,
	.irq_tx_disable = uart_xlnx_ps_irq_tx_disable,
	.irq_tx_ready = uart_xlnx_ps_irq_tx_ready,
	.irq_tx_complete = uart_xlnx_ps_irq_tx_complete,
	.irq_rx_enable = uart_xlnx_ps_irq_rx_enable,
	.irq_rx_disable = uart_xlnx_ps_irq_rx_disable,
	.irq_rx_ready = uart_xlnx_ps_irq_rx_ready,
	.irq_err_enable = uart_xlnx_ps_irq_err_enable,
	.irq_err_disable = uart_xlnx_ps_irq_err_disable,
	.irq_is_pending = uart_xlnx_ps_irq_is_pending,
	.irq_update = uart_xlnx_ps_irq_update,
	.irq_callback_set = uart_xlnx_ps_irq_callback_set,
#endif
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_XLNX_PS_IRQ_CONF_FUNC_SET(port) .irq_config_func = uart_xlnx_ps_irq_config_##port,
#define UART_XLNX_PS_IRQ_CONF_FUNC(port)                                                          \
	static void uart_xlnx_ps_irq_config_##port(const struct device *dev)                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(port), DT_INST_IRQ(port, priority), uart_xlnx_ps_isr,    \
			    DEVICE_DT_INST_GET(port), 0);                                          \
		irq_enable(DT_INST_IRQN(port));                                                    \
	}
#else
#define UART_XLNX_PS_IRQ_CONF_FUNC_SET(port)
#define UART_XLNX_PS_IRQ_CONF_FUNC(port)
#endif

#ifdef CONFIG_PINCTRL
#define UART_XLNX_PS_PINCTRL_DEFINE(port) PINCTRL_DT_INST_DEFINE(port);
#define UART_XLNX_PS_PINCTRL_INIT(port)   .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(port),
#else
#define UART_XLNX_PS_PINCTRL_DEFINE(port)
#define UART_XLNX_PS_PINCTRL_INIT(port)
#endif

#define UART_XLNX_INSTANTIATE(inst)                                                                \
	UART_XLNX_PS_PINCTRL_DEFINE(inst)                                                         \
	UART_XLNX_PS_IRQ_CONF_FUNC(inst);                                                         \
	static struct uart_xlnx_ps_dev_data_t uart_xlnx_ps_dev_data_##inst;                        \
	static const struct uart_xlnx_ps_dev_config uart_xlnx_ps_dev_cfg_##inst = {                \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                           \
		.sys_clk_freq = DT_INST_PROP(inst, clock_frequency),                               \
		.baud_rate = DT_INST_PROP(inst, current_speed),                                    \
		UART_XLNX_PS_IRQ_CONF_FUNC_SET(inst)                                              \
		UART_XLNX_PS_PINCTRL_INIT(inst)                                                    \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, uart_xlnx_ps_init, NULL,                                       \
			      &uart_xlnx_ps_dev_data_##inst,                                       \
			      &uart_xlnx_ps_dev_cfg_##inst,                                        \
			      PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,                           \
			      &uart_xlnx_ps_driver_api);

DT_INST_FOREACH_STATUS_OKAY(UART_XLNX_INSTANTIATE)
