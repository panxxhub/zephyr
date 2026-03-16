/*
 * Copyright (c) 2024 Moton Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/sd/sd_spec.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/cache.h>
#include <zephyr/device.h>

#ifdef CONFIG_PINCTRL
#include <zephyr/drivers/pinctrl.h>
#endif

#define DT_DRV_COMPAT xlnx_zynq_sdhc

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sdhc, CONFIG_SDHC_LOG_LEVEL);

#include "xlnx_zynq_sdhc.h"

#ifdef CONFIG_XLNX_ZYNQ_SDHC_HOST_ADMA_DESC_SIZE
#define ADMA_DESC_SIZE CONFIG_XLNX_ZYNQ_SDHC_HOST_ADMA_DESC_SIZE
#else
#define ADMA_DESC_SIZE 32
#endif

/* Maximum iterations for hardware polling loops */
#define HW_POLL_TIMEOUT_ITERS 10000
#define HW_POLL_INTERVAL_US   10

/*---------------------------------------------------------------------------
 * Structure definitions
 *---------------------------------------------------------------------------*/
#define DEV_CFG(dev) ((const struct zynq_sdhc_config *const)((dev)->config))
#define DEV_REG(dev) ((volatile struct zynq_sdhc_reg *)((uintptr_t)DEVICE_MMIO_GET(dev)))

typedef void (*zynq_sdhc_isr_cb_t)(const struct device *dev);

struct zynq_sdhc_config {
	DEVICE_MMIO_ROM;
#ifdef CONFIG_PINCTRL
	const struct pinctrl_dev_config *pincfg;
#endif
	zynq_sdhc_isr_cb_t config_func;
	uint32_t clock_freq;
	uint32_t max_bus_freq;
	uint32_t min_bus_freq;
	uint32_t power_delay_ms;
	uint8_t hs200_mode: 1;
	uint8_t hs400_mode: 1;
	uint8_t dw_4bit: 1;
	uint8_t dw_8bit: 1;
};

struct zynq_sdhc_cmd_config {
	struct sdhc_command *sdhc_cmd;
	uint32_t cmd_idx;
	enum zynq_sdhc_cmd_type cmd_type;
	bool data_present;
	bool idx_check_en;
	bool crc_check_en;
};

struct zynq_sdhc_data {
	DEVICE_MMIO_RAM;
	struct sdhc_host_props props;
	uint32_t rca;
	struct sdhc_io host_io;
	struct k_event irq_event;

	bool card_present;

	enum sd_spec_version hc_ver;
	enum sdhc_bus_width bus_width;
	enum zynq_sdhc_slot_type slot_type;
	uint64_t host_caps;
#if defined(CONFIG_XLNX_ZYNQ_SDHC_HOST_ADMA)
	uint8_t xfer_flag;
#endif
	adma_desc_t *const adma_desc_tbl;
};

/*---------------------------------------------------------------------------
 * Forward declarations
 *---------------------------------------------------------------------------*/
static int zynq_sdhc_set_voltage(const struct device *dev, enum sd_voltage signal_voltage);
static int zynq_sdhc_set_power(const struct device *dev, enum sdhc_power state);
static uint32_t zynq_get_clock_speed(enum sdhc_clock_speed speed);
static int zynq_sdhc_clock_set(const struct device *dev, enum sdhc_clock_speed speed);
static int zynq_sdhc_enable_clock(const struct device *dev);
static int zynq_sdhc_disable_clock(const struct device *dev);
static uint16_t zynq_sdhc_calc_clock(const struct device *dev, uint32_t tgt_freq);
static uint16_t zynq_sdhc_calc_timeout(const struct device *dev, uint32_t timeout_ms);
static int set_timing(const struct device *dev, enum sdhc_timing_mode timing);
static int zynq_sdhc_set_io(const struct device *dev, struct sdhc_io *ios);

#if defined(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)
static int wait_for_cmd_complete(struct zynq_sdhc_data *sdhc_data, uint32_t time_out);
#else
static int poll_cmd_complete(const struct device *dev, uint32_t time_out);
#endif

static int zynq_sdhc_host_send_cmd(const struct device *dev,
				   const struct zynq_sdhc_cmd_config *config);
static int zynq_sdhc_send_cmd_no_data(const struct device *dev, struct sdhc_command *cmd);
static int zynq_sdhc_send_cmd_data(const struct device *dev, struct sdhc_command *cmd,
				   struct sdhc_data *data, bool read);
static uint16_t zynq_sdhc_gen_xfer_mode(const struct device *dev, struct sdhc_data *data,
					 bool read);

/*---------------------------------------------------------------------------
 * Voltage and power control
 *---------------------------------------------------------------------------*/
static int zynq_sdhc_set_voltage(const struct device *dev, enum sd_voltage signal_voltage)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	struct zynq_sdhc_data *sdhc = dev->data;
	bool power_was_on = (regs->power_ctrl & ZYNQ_SDHC_HOST_POWER_CTRL_SD_BUS_POWER) != 0;
	uint64_t host_caps = sdhc->host_caps;
	int ret = 0;
	uint8_t power_level = 0;

	if (power_was_on) {
		regs->power_ctrl &= ~ZYNQ_SDHC_HOST_POWER_CTRL_SD_BUS_POWER;
	}

	switch (signal_voltage) {
	case SD_VOL_3_3_V:
		if (host_caps & ZYNQ_SDHC_HOST_VOL_3_3_V_SUPPORT) {
			if (sdhc->hc_ver == SD_SPEC_VER3_0) {
				regs->host_ctrl2 &= ~(ZYNQ_SDHC_HOST_CTRL2_1P8V_SIG_EN
						      << ZYNQ_SDHC_HOST_CTRL2_1P8V_SIG_LOC);
			}
			power_level = ZYNQ_SDHC_HOST_VOL_3_3_V_SELECT;
		} else {
			LOG_ERR("3.3V not supported by host");
			ret = -ENOTSUP;
		}
		break;

	case SD_VOL_3_0_V:
		if (host_caps & ZYNQ_SDHC_HOST_VOL_3_0_V_SUPPORT) {
			if (sdhc->hc_ver == SD_SPEC_VER3_0) {
				regs->host_ctrl2 &= ~(ZYNQ_SDHC_HOST_CTRL2_1P8V_SIG_EN
						      << ZYNQ_SDHC_HOST_CTRL2_1P8V_SIG_LOC);
			}
			power_level = ZYNQ_SDHC_HOST_VOL_3_0_V_SELECT;
		} else {
			LOG_ERR("3.0V not supported by host");
			ret = -ENOTSUP;
		}
		break;

	case SD_VOL_1_8_V:
		if (host_caps & ZYNQ_SDHC_HOST_VOL_1_8_V_SUPPORT) {
			if (sdhc->hc_ver == SD_SPEC_VER3_0) {
				regs->host_ctrl2 |= ZYNQ_SDHC_HOST_CTRL2_1P8V_SIG_EN
						    << ZYNQ_SDHC_HOST_CTRL2_1P8V_SIG_LOC;
			}
			power_level = ZYNQ_SDHC_HOST_VOL_1_8_V_SELECT;
		} else {
			LOG_ERR("1.8V not supported by host");
			ret = -ENOTSUP;
		}
		break;

	default:
		ret = -EINVAL;
	}

	regs->power_ctrl = power_was_on ? (power_level | ZYNQ_SDHC_HOST_POWER_CTRL_SD_BUS_POWER)
					: (power_level);

	return ret;
}

static int zynq_sdhc_set_power(const struct device *dev, enum sdhc_power state)
{
	struct zynq_sdhc_data *sdhc = dev->data;
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);

	if (state == SDHC_POWER_ON) {
		uint8_t val = XSDPS_PC_BUS_VSEL_3V3_MASK | XSDPS_PC_BUS_PWR_MASK;

		if (sdhc->hc_ver == SD_SPEC_VER3_0) {
			val &= ~XSDPS_PC_EMMC_HW_RST_MASK;
		}
		regs->power_ctrl = val;
	} else {
		regs->power_ctrl = (sdhc->hc_ver == SD_SPEC_VER3_0) ? XSDPS_PC_EMMC_HW_RST_MASK
								     : 0;
	}

	k_msleep(1);
	return 0;
}

/*---------------------------------------------------------------------------
 * Clock control
 *---------------------------------------------------------------------------*/
static int zynq_sdhc_disable_clock(const struct device *dev)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);

	if (regs->present_state & ZYNQ_SDHC_HOST_PSTATE_CMD_INHIBIT) {
		LOG_ERR("CMD inhibit active, cannot disable clock");
		return -EBUSY;
	}
	if (regs->present_state & ZYNQ_SDHC_HOST_PSTATE_DAT_INHIBIT) {
		LOG_ERR("DAT inhibit active, cannot disable clock");
		return -EBUSY;
	}

	regs->clock_ctrl &= ~ZYNQ_SDHC_HOST_SD_CLOCK_EN;
	regs->clock_ctrl &= ~ZYNQ_SDHC_HOST_INTERNAL_CLOCK_EN;

	for (int i = 0; i < HW_POLL_TIMEOUT_ITERS; i++) {
		if ((regs->clock_ctrl & ZYNQ_SDHC_HOST_SD_CLOCK_EN) == 0) {
			return 0;
		}
		k_busy_wait(HW_POLL_INTERVAL_US);
	}

	LOG_ERR("Clock disable timeout");
	return -ETIMEDOUT;
}

static int zynq_sdhc_enable_clock(const struct device *dev)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);

	regs->clock_ctrl |= ZYNQ_SDHC_HOST_INTERNAL_CLOCK_EN;

	for (int i = 0; i < HW_POLL_TIMEOUT_ITERS; i++) {
		if (regs->clock_ctrl & ZYNQ_SDHC_HOST_INTERNAL_CLOCK_STABLE) {
			break;
		}
		k_busy_wait(HW_POLL_INTERVAL_US);
		if (i == HW_POLL_TIMEOUT_ITERS - 1) {
			LOG_ERR("Internal clock stabilization timeout");
			return -ETIMEDOUT;
		}
	}

	regs->clock_ctrl |= ZYNQ_SDHC_HOST_SD_CLOCK_EN;
	return 0;
}

static uint16_t zynq_sdhc_calc_timeout(const struct device *dev, uint32_t timeout_ms)
{
	uint16_t timeout_val = 0xeU;
	const struct zynq_sdhc_config *cfg = DEV_CFG(dev);
	const uint8_t divider = zynq_sdhc_read8(dev, XSDPS_CLK_CTRL_OFFSET + 1);
	const uint32_t base_freq = cfg->clock_freq;

	if (divider == 0) {
		return timeout_val;
	}

	const uint32_t freq = base_freq / divider;

	if (freq == 0) {
		return timeout_val;
	}

	for (uint8_t i = 0; i < 0xe; i++) {
		uint32_t multiplier = 1U << (i + 13);
		uint32_t t = multiplier * 1000 / freq;

		if (t >= timeout_ms) {
			timeout_val = i;
			break;
		}
	}
	return timeout_val;
}

static uint16_t zynq_sdhc_calc_clock(const struct device *dev, uint32_t tgt_freq)
{
	uint16_t clock_val = 0;
	uint16_t div_cnt;
	uint16_t divisor = 0;
	struct zynq_sdhc_data *sdhc = dev->data;
	const struct zynq_sdhc_config *cfg = DEV_CFG(dev);
	const uint32_t base_freq = cfg->clock_freq;

	if (sdhc->hc_ver == SD_SPEC_VER3_0) {
		if (base_freq <= tgt_freq) {
			divisor = 0;
		} else {
			for (div_cnt = 2; div_cnt <= XSDPS_CC_EXT_MAX_DIV_CNT; div_cnt += 2) {
				if ((base_freq / div_cnt) <= tgt_freq) {
					divisor = div_cnt >> 1;
					break;
				}
			}
		}
	} else {
		for (div_cnt = 1; div_cnt <= XSDPS_CC_MAX_DIV_CNT; div_cnt <<= 1) {
			if ((base_freq / div_cnt) <= tgt_freq) {
				divisor = div_cnt >> 1;
				break;
			}
		}
	}

	clock_val |= ((divisor & XSDPS_CC_SDCLK_FREQ_SEL_MASK) << XSDPS_CC_DIV_SHIFT);
	clock_val |= (((divisor >> 8) & XSDPS_CC_SDCLK_FREQ_SEL_EXT_MASK)
		      << XSDPS_CC_EXT_DIV_SHIFT);

	return clock_val;
}

static uint32_t zynq_get_clock_speed(enum sdhc_clock_speed speed)
{
	switch (speed) {
	case SDMMC_CLOCK_400KHZ:
		return 400000;
	case SD_CLOCK_25MHZ:
	case MMC_CLOCK_26MHZ:
		return 25000000;
	case SD_CLOCK_50MHZ:
	case MMC_CLOCK_52MHZ:
		return 50000000;
	case SD_CLOCK_100MHZ:
		return 100000000;
	case MMC_CLOCK_HS200:
	case SD_CLOCK_208MHZ:
		return 200000000;
	default:
		return 0;
	}
}

static int zynq_sdhc_clock_set(const struct device *dev, enum sdhc_clock_speed speed)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	uint16_t clk_reg;
	int ret;

	ret = zynq_sdhc_disable_clock(dev);
	if (ret) {
		return ret;
	}

	clk_reg = zynq_sdhc_calc_clock(dev, zynq_get_clock_speed(speed));
	clk_reg |= ZYNQ_SDHC_HOST_INTERNAL_CLOCK_EN;
	regs->clock_ctrl = clk_reg;

	/* Wait for internal clock to stabilize */
	for (int i = 0; i < HW_POLL_TIMEOUT_ITERS; i++) {
		if (regs->clock_ctrl & ZYNQ_SDHC_HOST_INTERNAL_CLOCK_STABLE) {
			break;
		}
		k_busy_wait(HW_POLL_INTERVAL_US);
		if (i == HW_POLL_TIMEOUT_ITERS - 1) {
			LOG_ERR("Clock stabilization timeout");
			return -ETIMEDOUT;
		}
	}

	/* Enable SD clock */
	clk_reg = regs->clock_ctrl;
	clk_reg |= ZYNQ_SDHC_HOST_SD_CLOCK_EN;
	regs->clock_ctrl = clk_reg;

	return 0;
}

/*---------------------------------------------------------------------------
 * Timing mode
 *---------------------------------------------------------------------------*/
static int set_timing(const struct device *dev, enum sdhc_timing_mode timing)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	uint8_t mode;

	switch (timing) {
	case SDHC_TIMING_LEGACY:
		regs->host_ctrl1 &= ~XSDPS_HC_SPEED_MASK;
		return 0;

	case SDHC_TIMING_HS:
		regs->host_ctrl1 |= XSDPS_HC_SPEED_MASK;
		return 0;

	case SDHC_TIMING_SDR12:
		mode = ZYNQ_SDHC_HOST_UHSMODE_SDR12;
		break;
	case SDHC_TIMING_SDR25:
		mode = ZYNQ_SDHC_HOST_UHSMODE_SDR25;
		break;
	case SDHC_TIMING_SDR50:
		mode = ZYNQ_SDHC_HOST_UHSMODE_SDR50;
		break;
	case SDHC_TIMING_SDR104:
		mode = ZYNQ_SDHC_HOST_UHSMODE_SDR104;
		break;
	case SDHC_TIMING_DDR50:
	case SDHC_TIMING_DDR52:
		mode = ZYNQ_SDHC_HOST_UHSMODE_DDR50;
		break;
	case SDHC_TIMING_HS200:
		mode = ZYNQ_SDHC_HOST_UHSMODE_SDR104;
		break;
	case SDHC_TIMING_HS400:
		mode = ZYNQ_SDHC_HOST_UHSMODE_HS400;
		break;
	default:
		return -ENOTSUP;
	}

	/* UHS modes require clock disable/re-enable and 1.8V signaling */
	int ret = zynq_sdhc_disable_clock(dev);

	if (ret) {
		LOG_ERR("Disable clock failed for UHS timing");
		return ret;
	}

	regs->host_ctrl2 |= ZYNQ_SDHC_HOST_CTRL2_1P8V_SIG_EN
			    << ZYNQ_SDHC_HOST_CTRL2_1P8V_SIG_LOC;
	SET_BITS(regs->host_ctrl2, ZYNQ_SDHC_HOST_CTRL2_UHS_MODE_SEL_LOC,
		 ZYNQ_SDHC_HOST_CTRL2_UHS_MODE_SEL_MASK, mode);

	return zynq_sdhc_enable_clock(dev);
}

/*---------------------------------------------------------------------------
 * Interrupt and status management
 *---------------------------------------------------------------------------*/
static void configure_status_interrupts_enable_signals(const struct device *dev)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);

	regs->normal_int_stat_en = XSDPS_NORM_INTR_ALL_MASK & ~XSDPS_INTR_CARD_MASK;
	regs->err_int_stat_en = ZYNQ_SDHC_HOST_ERROR_INTR_MASK;
	regs->normal_int_signal_en = XSDPS_NORM_INTR_ALL_MASK & ~XSDPS_INTR_CARD_MASK;
	regs->err_int_signal_en = ZYNQ_SDHC_HOST_ERROR_INTR_MASK;
	regs->timeout_ctrl = ZYNQ_SDHC_HOST_MAX_TIMEOUT;
}

static void configure_status_interrupts_disable_signals(const struct device *dev)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);

	regs->normal_int_stat_en = XSDPS_NORM_INTR_ALL_MASK & ~XSDPS_INTR_CARD_MASK;
	regs->err_int_stat_en = ZYNQ_SDHC_HOST_ERROR_INTR_MASK;
	regs->normal_int_signal_en = 0;
	regs->err_int_signal_en = 0;
	regs->timeout_ctrl = ZYNQ_SDHC_HOST_MAX_TIMEOUT;
}

static void clear_interrupts(const struct device *dev)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);

	regs->normal_int_stat = XSDPS_NORM_INTR_ALL_MASK;
	regs->err_int_stat = XSDPS_ERROR_INTR_ALL_MASK;
}

/*---------------------------------------------------------------------------
 * Software reset
 *---------------------------------------------------------------------------*/
static int zynq_sdhc_host_sw_reset(const struct device *dev, enum zynq_sdhc_swrst sr)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	uint8_t write_val = 0;

	switch (sr) {
	case ZYNQ_SDHC_HOST_SWRST_ALL:
		write_val = ZYNQ_SDHC_HOST_SW_RESET_REG_ALL;
		break;
	case ZYNQ_SDHC_HOST_SWRST_CMD_LINE:
		write_val = ZYNQ_SDHC_HOST_SW_RESET_REG_CMD;
		break;
	case ZYNQ_SDHC_HOST_SWRST_DATA_LINE:
		write_val = ZYNQ_SDHC_HOST_SW_RESET_REG_DATA;
		break;
	}

	regs->sw_reset = write_val;

	for (uint32_t i = 0; i < 100; i++) {
		if ((regs->sw_reset & write_val) == 0) {
			return 0;
		}
		k_sleep(K_MSEC(1));
	}

	LOG_ERR("Software reset failed");
	return -EIO;
}

/*---------------------------------------------------------------------------
 * DMA setup
 *---------------------------------------------------------------------------*/
static int zynq_sdhc_dma_init(const struct device *dev, struct sdhc_data *data, bool read)
{
	struct zynq_sdhc_data *sdhc_data = dev->data;
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);

	sys_cache_data_flush_and_invd_range(data->data, data->blocks * data->block_size);

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_ADMA)) {
		uintptr_t buff_base = (uintptr_t)data->data;

		if (data->blocks > ADMA_DESC_SIZE) {
			LOG_ERR("ADMA: %u blocks exceeds table size %u",
				data->blocks, ADMA_DESC_SIZE);
			return -ENOMEM;
		}
		memset(sdhc_data->adma_desc_tbl, 0, sizeof(adma_desc_t) * data->blocks);
		for (int i = 0; i < data->blocks; i++) {
			adma_desc_t *desc = &sdhc_data->adma_desc_tbl[i];

			desc->address = buff_base + (i * data->block_size);
			desc->len = data->block_size;
			desc->attr.attr_bits.valid = 1;
			desc->attr.attr_bits.end = (i == data->blocks - 1) ? 1 : 0;
			desc->attr.attr_bits.int_en = 1;
			desc->attr.attr_bits.act = 2; /* Transfer */
		}

		regs->adma_sys_addr1 = (uint32_t)(uintptr_t)sdhc_data->adma_desc_tbl;
#if defined(CONFIG_64BIT)
		if (regs->host_cntrl_version == ZYNQ_SDHC_HC_SPEC_V3) {
			regs->adma_sys_addr2 =
				(uint32_t)(((uint64_t)(uintptr_t)sdhc_data->adma_desc_tbl) >> 32);
		}
#endif
		sys_cache_data_flush_range(sdhc_data->adma_desc_tbl,
					  sizeof(adma_desc_t) * (data->blocks + 1));
	} else {
		regs->sdma_sysaddr = (uint32_t)(uintptr_t)data->data;
	}
	return 0;
}

/*---------------------------------------------------------------------------
 * Transfer mode
 *---------------------------------------------------------------------------*/
static uint16_t zynq_sdhc_gen_xfer_mode(const struct device *dev, struct sdhc_data *data,
					 bool read)
{
	struct zynq_sdhc_data *sdhc_data = dev->data;
	uint16_t multi_block = (data->blocks > 1) ? 1u : 0u;
	uint16_t transfer_mode = 0;

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_AUTO_STOP)) {
		if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_ADMA) &&
		    sdhc_data->host_io.timing == SDHC_TIMING_SDR104) {
			/* Auto CMD23 for ADMA in SDR104 */
			SET_BITS(transfer_mode, ZYNQ_SDHC_HOST_XFER_AUTO_CMD_EN_LOC,
				 ZYNQ_SDHC_HOST_XFER_AUTO_CMD_EN_MASK, multi_block ? 2 : 0);
		} else {
			/* Auto CMD12 */
			SET_BITS(transfer_mode, ZYNQ_SDHC_HOST_XFER_AUTO_CMD_EN_LOC,
				 ZYNQ_SDHC_HOST_XFER_AUTO_CMD_EN_MASK, multi_block ? 1 : 0);
		}
		SET_BITS(transfer_mode, ZYNQ_SDHC_HOST_XFER_BLOCK_CNT_EN_LOC,
			 ZYNQ_SDHC_HOST_XFER_BLOCK_CNT_EN_MASK, multi_block ? 1 : 0);
	}

	SET_BITS(transfer_mode, ZYNQ_SDHC_HOST_XFER_MULTI_BLOCK_SEL_LOC,
		 ZYNQ_SDHC_HOST_XFER_MULTI_BLOCK_SEL_MASK, multi_block);

	SET_BITS(transfer_mode, ZYNQ_SDHC_HOST_XFER_DATA_DIR_LOC,
		 ZYNQ_SDHC_HOST_XFER_DATA_DIR_MASK, read ? 1u : 0u);

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_DMA)) {
		SET_BITS(transfer_mode, ZYNQ_SDHC_HOST_XFER_DMA_EN_LOC,
			 ZYNQ_SDHC_HOST_XFER_DMA_EN_MASK, 1u);
	}

	return transfer_mode;
}

static int zynq_sdhc_init_xfr(const struct device *dev, struct sdhc_data *data, bool read)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	struct zynq_sdhc_data *sdhc_data = dev->data;

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_DMA)) {
		int dma_rc = zynq_sdhc_dma_init(dev, data, read);

		if (dma_rc) {
			return dma_rc;
		}
#if defined(CONFIG_XLNX_ZYNQ_SDHC_HOST_ADMA)
		sdhc_data->xfer_flag = 1;
#endif
	}

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_ADMA)) {
		SET_BITS(regs->host_ctrl1, ZYNQ_SDHC_HOST_CTRL1_DMA_SEL_LOC,
			 ZYNQ_SDHC_HOST_CTRL1_DMA_SEL_MASK, 2U);
	} else {
		SET_BITS(regs->host_ctrl1, ZYNQ_SDHC_HOST_CTRL1_DMA_SEL_LOC,
			 ZYNQ_SDHC_HOST_CTRL1_DMA_SEL_MASK, 0U);
	}

	regs->block_size = data->block_size & XSDPS_BLK_SIZE_MASK;
	regs->transfer_mode = zynq_sdhc_gen_xfer_mode(dev, data, read);

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_AUTO_STOP)) {
		regs->block_count = data->blocks & XSDPS_BLK_CNT_MASK;
	} else {
		regs->block_count = 0;
	}

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_BLOCK_GAP)) {
		SET_BITS(regs->block_gap_ctrl, ZYNQ_SDHC_HOST_BLOCK_GAP_LOC,
			 ZYNQ_SDHC_HOST_BLOCK_GAP_MASK, 1);
	} else {
		SET_BITS(regs->block_gap_ctrl, ZYNQ_SDHC_HOST_BLOCK_GAP_LOC,
			 ZYNQ_SDHC_HOST_BLOCK_GAP_MASK, 0);
	}

	regs->timeout_ctrl = zynq_sdhc_calc_timeout(dev, data->timeout_ms);

	return 0;
}

/*---------------------------------------------------------------------------
 * Transfer completion
 *---------------------------------------------------------------------------*/
static int wait_xfr_intr_complete(const struct device *dev, uint32_t time_out)
{
	struct zynq_sdhc_data *sdhc_data = dev->data;
	k_timeout_t wait_time;
	uint32_t events;

	if (time_out == SDHC_TIMEOUT_FOREVER) {
		wait_time = K_FOREVER;
	} else {
		wait_time = K_MSEC(time_out);
	}

	if (!IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
		return -ENOTSUP;
	}

	events = k_event_wait(&sdhc_data->irq_event,
			      ZYNQ_SDHC_HOST_XFER_COMPLETE |
				      ERR_INTR_STATUS_EVENT(ZYNQ_SDHC_HOST_DMA_TXFR_ERR |
						    ZYNQ_SDHC_HOST_DATA_TIMEOUT_ERR |
						    ZYNQ_SDHC_HOST_DATA_CRC_ERR |
						    ZYNQ_SDHC_HOST_DATA_END_BIT_ERR),
			      false, wait_time);

	if (events & ZYNQ_SDHC_HOST_XFER_COMPLETE) {
		return 0;
	} else if (events & ERR_INTR_STATUS_EVENT(0xFFFF)) {
		LOG_ERR("Transfer complete error: 0x%04x", events);
		return -EIO;
	}

	LOG_ERR("Transfer complete timeout");
	return -EAGAIN;
}

static int wait_xfr_poll_complete(const struct device *dev, uint32_t time_out)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	int32_t retry = (int32_t)time_out;

	while (retry > 0) {
		if (regs->normal_int_stat & ZYNQ_SDHC_HOST_XFER_COMPLETE) {
			/* W1C: write 1 to clear the bit */
			regs->normal_int_stat = ZYNQ_SDHC_HOST_XFER_COMPLETE;
			return 0;
		}
		k_busy_wait(ZYNQ_SDHC_HOST_MSEC_DELAY);
		retry--;
	}

	return -EAGAIN;
}

static int wait_xfr_complete(const struct device *dev, uint32_t time_out)
{
	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
		return wait_xfr_intr_complete(dev, time_out);
	}
	return wait_xfr_poll_complete(dev, time_out);
}

/*---------------------------------------------------------------------------
 * Command framing and response
 *---------------------------------------------------------------------------*/
static enum zynq_sdhc_resp_type zynq_sdhc_decode_resp_type(enum sd_rsp_type type, uint8_t slottype,
							   uint32_t opcode)
{
	enum zynq_sdhc_resp_type resp_type;
	uint8_t resp_type_sel = type & 0xF;

	switch (resp_type_sel) {
	case SD_RSP_TYPE_NONE:
		resp_type = ZYNQ_SDHC_HOST_RESP_NONE;
		break;
	case SD_RSP_TYPE_R1:
	case SD_RSP_TYPE_R3:
	case SD_RSP_TYPE_R4:
	case SD_RSP_TYPE_R5:
		resp_type = ZYNQ_SDHC_HOST_RESP_LEN_48;
		break;
	case SD_RSP_TYPE_R1b:
		resp_type = ZYNQ_SDHC_HOST_RESP_LEN_48_BUSY;
		break;
	case SD_RSP_TYPE_R2:
		resp_type = ZYNQ_SDHC_HOST_RESP_LEN_136;
		break;
	case SD_RSP_TYPE_R6:
	case SD_RSP_TYPE_R7:
		if (slottype == XLNX_SDHC_EMMC_SLOT) {
			resp_type = ZYNQ_SDHC_HOST_INVAL_HOST_RESP;
		} else {
			resp_type = ZYNQ_SDHC_HOST_RESP_LEN_48;
		}
		break;
	default:
		resp_type = ZYNQ_SDHC_HOST_INVAL_HOST_RESP;
		break;
	}

	if (slottype == XLNX_SDHC_EMMC_SLOT && opcode == SD_APP_CMD) {
		resp_type = ZYNQ_SDHC_HOST_INVAL_HOST_RESP;
	}

	return resp_type;
}

#if defined(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)
static int wait_for_cmd_complete(struct zynq_sdhc_data *sdhc_data, uint32_t time_out)
{
	k_timeout_t wait_time;
	uint32_t events;

	if (time_out == SDHC_TIMEOUT_FOREVER) {
		wait_time = K_FOREVER;
	} else {
		wait_time = K_MSEC(time_out);
	}

	events = k_event_wait(&sdhc_data->irq_event,
			      ZYNQ_SDHC_HOST_CMD_COMPLETE |
				      ERR_INTR_STATUS_EVENT(ZYNQ_SDHC_HOST_ERR_STATUS),
			      false, wait_time);

	if (events & ZYNQ_SDHC_HOST_CMD_COMPLETE) {
		return 0;
	} else if (events & ERR_INTR_STATUS_EVENT(ZYNQ_SDHC_HOST_ERR_STATUS)) {
		LOG_ERR("Command complete error: 0x%04x", events);
		return -EIO;
	}

	LOG_ERR("Command complete timeout");
	return -EAGAIN;
}
#else
static int poll_cmd_complete(const struct device *dev, uint32_t time_out)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	int ret = -EAGAIN;
	int32_t remaining = (int32_t)time_out;
#if defined(CONFIG_XLNX_ZYNQ_SDHC_HOST_ADMA)
	struct zynq_sdhc_data *sdhc_data = dev->data;
#endif

	while (remaining > 0) {
		if (regs->normal_int_stat & ZYNQ_SDHC_HOST_CMD_COMPLETE) {
			/* W1C: write 1 to clear */
			regs->normal_int_stat = ZYNQ_SDHC_HOST_CMD_COMPLETE;
			ret = 0;
			break;
		}
		k_busy_wait(1000);
		remaining--;
	}

	if (remaining == 0) {
		LOG_ERR("Command complete poll timeout");
	}

	/* Check for error interrupts */
	if (regs->normal_int_stat & XSDPS_INTR_ERR_MASK) {
		uint16_t err_stat = regs->err_int_stat;

		LOG_ERR("Error interrupt: 0x%04x", err_stat);
		/* W1C: clear error status */
		regs->err_int_stat = err_stat;
		ret = -EIO;
	}

#if defined(CONFIG_XLNX_ZYNQ_SDHC_HOST_ADMA)
	if (sdhc_data->xfer_flag) {
		for (int32_t t = (int32_t)time_out; t > 0; t--) {
			if ((regs->adma_err_stat & ZYNQ_SDHC_HOST_ADMA_ERR_MASK) == 0) {
				break;
			}
			k_busy_wait(1000);
			if (t == 1) {
				LOG_ERR("ADMA error: 0x%02x", regs->adma_err_stat);
				ret = -EIO;
			}
		}
		sdhc_data->xfer_flag = 0;
	}
#endif

	return ret;
}
#endif

static void update_cmd_response(const struct device *dev, struct sdhc_command *sdhc_cmd)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);

	if (sdhc_cmd->response_type == SD_RSP_TYPE_NONE) {
		return;
	}

	uint32_t resp0 = regs->resp_01;

	if (sdhc_cmd->response_type == SD_RSP_TYPE_R2) {
		uint32_t resp1 = (regs->resp_3 << 16) | regs->resp_2;
		uint32_t resp2 = (regs->resp_5 << 16) | regs->resp_4;
		uint32_t resp3 = (regs->resp_7 << 16) | regs->resp_6;

		/* CRC is stripped, shift response bytes */
		sdhc_cmd->response[3] = (resp3 << 8) | (resp2 >> 24);
		sdhc_cmd->response[2] = (resp2 << 8) | (resp1 >> 24);
		sdhc_cmd->response[1] = (resp1 << 8) | (resp0 >> 24);
		sdhc_cmd->response[0] = (resp0 << 8);
	} else {
		sdhc_cmd->response[0] = resp0;
	}
}

/*---------------------------------------------------------------------------
 * Command send
 *---------------------------------------------------------------------------*/
static int zynq_sdhc_host_send_cmd(const struct device *dev,
				   const struct zynq_sdhc_cmd_config *config)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	struct zynq_sdhc_data *sdhc = dev->data;
	struct sdhc_command *sdhc_cmd = config->sdhc_cmd;
	enum zynq_sdhc_resp_type resp_type_select = zynq_sdhc_decode_resp_type(
		sdhc_cmd->response_type, sdhc->slot_type, sdhc_cmd->opcode);
	uint32_t cmd_reg;
	int ret;

	if (regs->present_state & ZYNQ_SDHC_HOST_PSTATE_CMD_INHIBIT) {
		LOG_ERR("CMD line busy");
		return -EBUSY;
	}

	if (config->data_present && (regs->present_state & ZYNQ_SDHC_HOST_PSTATE_DAT_INHIBIT)) {
		LOG_ERR("DAT line busy");
		return -EBUSY;
	}

	if (resp_type_select == ZYNQ_SDHC_HOST_INVAL_HOST_RESP) {
		LOG_DBG("Unsupported resp type for slot (cmd %d)", config->cmd_idx);
		return -EINVAL;
	}

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
		/* Clear both normal and error events to prevent stale errors
		 * from a prior command causing immediate false -EIO.
		 */
		k_event_clear(&sdhc->irq_event,
			      (XSDPS_NORM_INTR_ALL_MASK & ~XSDPS_INTR_CARD_MASK) |
			      ERR_INTR_STATUS_EVENT(0xFFFF));
	}

	regs->argument = sdhc_cmd->arg;

	cmd_reg = (config->cmd_idx << ZYNQ_SDHC_HOST_CMD_INDEX_LOC) |
		  (config->cmd_type << ZYNQ_SDHC_HOST_CMD_TYPE_LOC) |
		  (config->data_present << ZYNQ_SDHC_HOST_CMD_DATA_PRESENT_LOC) |
		  (config->idx_check_en << ZYNQ_SDHC_HOST_CMD_IDX_CHECK_EN_LOC) |
		  (config->crc_check_en << ZYNQ_SDHC_HOST_CMD_CRC_CHECK_EN_LOC) |
		  (resp_type_select << ZYNQ_SDHC_HOST_CMD_RESP_TYPE_LOC);
	regs->cmd = cmd_reg;

#if defined(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)
	ret = wait_for_cmd_complete(sdhc, sdhc_cmd->timeout_ms);
#else
	ret = poll_cmd_complete(dev, sdhc_cmd->timeout_ms);
#endif
	if (ret) {
		LOG_ERR("CMD%d failed (%d), cmd_reg=0x%04x", config->cmd_idx, ret, cmd_reg);
		return ret;
	}

	update_cmd_response(dev, sdhc_cmd);
	return 0;
}

static int zynq_sdhc_send_cmd_no_data(const struct device *dev, struct sdhc_command *cmd)
{
	struct zynq_sdhc_cmd_config sdhc_cmd = {
		.sdhc_cmd = cmd,
		.cmd_idx = cmd->opcode,
		.cmd_type = ZYNQ_SDHC_HOST_CMD_NORMAL,
		.data_present = false,
		.idx_check_en = false,
		.crc_check_en = false,
	};

	return zynq_sdhc_host_send_cmd(dev, &sdhc_cmd);
}

/*---------------------------------------------------------------------------
 * PIO data transfer
 *---------------------------------------------------------------------------*/
static int read_data_port(const struct device *dev, struct sdhc_data *sdhc)
{
	struct zynq_sdhc_data *sdhc_data = dev->data;
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	uint32_t block_size = sdhc->block_size;
	uint32_t block_cnt = sdhc->blocks;
	uint32_t *data = (uint32_t *)sdhc->data;
	k_timeout_t wait_time;

	if (sdhc->timeout_ms == SDHC_TIMEOUT_FOREVER) {
		wait_time = K_FOREVER;
	} else {
		wait_time = K_MSEC(sdhc->timeout_ms);
	}

	while (block_cnt--) {
		if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
			uint32_t events;

			events = k_event_wait(&sdhc_data->irq_event, ZYNQ_SDHC_HOST_BUF_RD_READY,
					      false, wait_time);
			k_event_clear(&sdhc_data->irq_event, ZYNQ_SDHC_HOST_BUF_RD_READY);
			if (!(events & ZYNQ_SDHC_HOST_BUF_RD_READY)) {
				LOG_ERR("Read buffer ready timeout at block %d",
					sdhc->blocks - block_cnt);
				return -EIO;
			}
		} else {
			int poll = HW_POLL_TIMEOUT_ITERS;

			while (!(regs->present_state & ZYNQ_SDHC_HOST_PSTATE_BUF_READ_EN)) {
				if (--poll == 0) {
					LOG_ERR("PIO read buffer poll timeout");
					return -ETIMEDOUT;
				}
				k_busy_wait(HW_POLL_INTERVAL_US);
			}
		}

		if (regs->present_state & ZYNQ_SDHC_HOST_PSTATE_DAT_INHIBIT) {
			for (uint32_t i = block_size >> 2; i != 0; i--) {
				*data++ = regs->data_port;
			}
		}
	}

	return wait_xfr_complete(dev, sdhc->timeout_ms);
}

static int write_data_port(const struct device *dev, struct sdhc_data *sdhc)
{
	struct zynq_sdhc_data *sdhc_data = dev->data;
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	uint32_t block_size = sdhc->block_size;
	uint32_t block_cnt = sdhc->blocks;
	uint32_t *data = (uint32_t *)sdhc->data;
	k_timeout_t wait_time;

	if (sdhc->timeout_ms == SDHC_TIMEOUT_FOREVER) {
		wait_time = K_FOREVER;
	} else {
		wait_time = K_MSEC(sdhc->timeout_ms);
	}

	/* Wait for initial write buffer ready */
	for (int poll = HW_POLL_TIMEOUT_ITERS; poll > 0; poll--) {
		if (regs->present_state & ZYNQ_SDHC_HOST_PSTATE_BUF_WRITE_EN) {
			break;
		}
		if (poll == 1) {
			LOG_ERR("PIO write buffer poll timeout");
			return -ETIMEDOUT;
		}
		k_busy_wait(HW_POLL_INTERVAL_US);
	}

	while (1) {
		if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
			k_event_clear(&sdhc_data->irq_event, ZYNQ_SDHC_HOST_BUF_WR_READY);
		}

		if (regs->present_state & ZYNQ_SDHC_HOST_PSTATE_DAT_INHIBIT) {
			for (uint32_t i = block_size >> 2; i != 0; i--) {
				regs->data_port = *data++;
			}
		}

		if (!(--block_cnt)) {
			break;
		}

		if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
			uint32_t events;

			events = k_event_wait(&sdhc_data->irq_event, ZYNQ_SDHC_HOST_BUF_WR_READY,
					      false, wait_time);
			k_event_clear(&sdhc_data->irq_event, ZYNQ_SDHC_HOST_BUF_WR_READY);
			if (!(events & ZYNQ_SDHC_HOST_BUF_WR_READY)) {
				LOG_ERR("Write buffer ready timeout");
				return -EIO;
			}
		} else {
			int poll = HW_POLL_TIMEOUT_ITERS;

			while (!(regs->present_state & ZYNQ_SDHC_HOST_PSTATE_BUF_WRITE_EN)) {
				if (--poll == 0) {
					LOG_ERR("PIO write buffer poll timeout");
					return -ETIMEDOUT;
				}
				k_busy_wait(HW_POLL_INTERVAL_US);
			}
		}
	}

	return wait_xfr_complete(dev, sdhc->timeout_ms);
}

/*---------------------------------------------------------------------------
 * Data transfer commands
 *---------------------------------------------------------------------------*/
static int zynq_sdhc_send_cmd_data(const struct device *dev, struct sdhc_command *cmd,
				   struct sdhc_data *data, bool read)
{
	struct zynq_sdhc_data *sdhc_data = dev->data;
	struct zynq_sdhc_cmd_config cmd_config = {
		.sdhc_cmd = cmd,
		.cmd_idx = cmd->opcode,
		.cmd_type = ZYNQ_SDHC_HOST_CMD_NORMAL,
		.data_present = true,
		.idx_check_en = true,
		.crc_check_en = true,
	};
	int ret;

	ret = zynq_sdhc_init_xfr(dev, data, read);
	if (ret) {
		LOG_ERR("Transfer init failed");
		return ret;
	}

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
		k_event_clear(&sdhc_data->irq_event, ZYNQ_SDHC_HOST_XFER_COMPLETE);
		k_event_clear(&sdhc_data->irq_event,
			      read ? ZYNQ_SDHC_HOST_BUF_RD_READY : ZYNQ_SDHC_HOST_BUF_WR_READY);
	}

	ret = zynq_sdhc_host_send_cmd(dev, &cmd_config);
	if (ret) {
		return ret;
	}

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_DMA)) {
		ret = wait_xfr_complete(dev, data->timeout_ms);
	} else {
		ret = read ? read_data_port(dev, data) : write_data_port(dev, data);
	}

	return ret;
}

static int zynq_sdhc_stop_transfer(const struct device *dev, bool is_multi_block)
{
	struct zynq_sdhc_data *sdhc_data = dev->data;

	if (!is_multi_block) {
		return 0;
	}

	struct sdhc_command stop_cmd = {
		.arg = sdhc_data->rca << ZYNQ_SDHC_HOST_RCA_SHIFT,
		.response_type = SD_RSP_TYPE_R1b,
		.timeout_ms = 1000,
	};
	struct zynq_sdhc_cmd_config cmd = {
		.sdhc_cmd = &stop_cmd,
		.cmd_idx = SD_STOP_TRANSMISSION,
		.cmd_type = ZYNQ_SDHC_HOST_CMD_NORMAL,
	};

	return zynq_sdhc_host_send_cmd(dev, &cmd);
}

/*---------------------------------------------------------------------------
 * SDHC API: reset
 *---------------------------------------------------------------------------*/
static int zynq_sdhc_reset(const struct device *dev)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	int ret;

	if (!(regs->present_state & ZYNQ_SDHC_HOST_PSTATE_CARD_INSERTED)) {
		LOG_WRN("No card inserted — controller ready for hot-insert");
		return 0;
	}

	ret = zynq_sdhc_host_sw_reset(dev, ZYNQ_SDHC_HOST_SWRST_ALL);
	if (ret) {
		return ret;
	}

	clear_interrupts(dev);

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
		configure_status_interrupts_enable_signals(dev);
	} else {
		configure_status_interrupts_disable_signals(dev);
	}

	return 0;
}

/*---------------------------------------------------------------------------
 * SDHC API: request
 *---------------------------------------------------------------------------*/
static int zynq_sdhc_request(const struct device *dev, struct sdhc_command *cmd,
			     struct sdhc_data *data)
{
	int ret;

	if (!data) {
		return zynq_sdhc_send_cmd_no_data(dev, cmd);
	}

	bool read;

	switch (cmd->opcode) {
	case SD_WRITE_SINGLE_BLOCK:
	case SD_WRITE_MULTIPLE_BLOCK:
		read = false;
		break;
	default:
		read = true;
		break;
	}

	ret = zynq_sdhc_send_cmd_data(dev, cmd, data, read);

	if (!ret && !IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_AUTO_STOP)) {
		zynq_sdhc_stop_transfer(dev, data->blocks > 1);
	}

	return ret;
}

/*---------------------------------------------------------------------------
 * SDHC API: set_io
 *---------------------------------------------------------------------------*/
static int zynq_sdhc_set_io(const struct device *dev, struct sdhc_io *ios)
{
	const struct zynq_sdhc_config *cfg = dev->config;
	struct zynq_sdhc_data *data = dev->data;
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	struct sdhc_io *host_io = &data->host_io;
	int ret = 0;
	uint32_t tgt_freq = zynq_get_clock_speed(ios->clock);

	LOG_DBG("I/O: DW=%d, Clk=%d Hz, power=%s, voltage=%s", ios->bus_width, ios->clock,
		ios->power_mode == SDHC_POWER_ON ? "ON" : "OFF",
		ios->signal_voltage == SD_VOL_1_8_V ? "1.8V" : "3.3V");

	if (tgt_freq && (tgt_freq > cfg->max_bus_freq || tgt_freq < cfg->min_bus_freq)) {
		LOG_ERR("Invalid clock: %d Hz (range %d-%d)", ios->clock, cfg->min_bus_freq,
			cfg->max_bus_freq);
		return -EINVAL;
	}

	if (host_io->clock != ios->clock) {
		if (ios->clock != 0) {
			ret = zynq_sdhc_clock_set(dev, ios->clock);
			if (ret) {
				return ret;
			}
		} else {
			zynq_sdhc_disable_clock(dev);
		}
		host_io->clock = ios->clock;
	}

	if (host_io->bus_width != ios->bus_width) {
		if (data->bus_width < ios->bus_width) {
			return -ENOTSUP;
		}

		if (ios->bus_width == SDHC_BUS_WIDTH8BIT) {
			SET_BITS(regs->host_ctrl1, ZYNQ_SDHC_HOST_CTRL1_EXT_DAT_WIDTH_LOC,
				 ZYNQ_SDHC_HOST_CTRL1_EXT_DAT_WIDTH_MASK, 1);
		} else {
			SET_BITS(regs->host_ctrl1, ZYNQ_SDHC_HOST_CTRL1_DAT_WIDTH_LOC,
				 ZYNQ_SDHC_HOST_CTRL1_DAT_WIDTH_MASK,
				 ios->bus_width == SDHC_BUS_WIDTH4BIT ? 1 : 0);
		}
		host_io->bus_width = ios->bus_width;
	}

	if (ios->signal_voltage != host_io->signal_voltage) {
		ret = zynq_sdhc_set_voltage(dev, ios->signal_voltage);
		if (ret) {
			LOG_ERR("Set voltage failed: %d", ret);
			return ret;
		}
		host_io->signal_voltage = ios->signal_voltage;
	}

	if (host_io->power_mode != ios->power_mode) {
		ret = zynq_sdhc_set_power(dev, ios->power_mode);
		if (ret) {
			LOG_ERR("Set power failed: %d", ret);
			return ret;
		}
		host_io->power_mode = ios->power_mode;
	}

	if (host_io->timing != ios->timing) {
		if (data->hc_ver == SD_SPEC_VER3_0) {
			ret = set_timing(dev, ios->timing);
			if (ret) {
				LOG_ERR("Set timing failed: %d", ret);
				return ret;
			}
		}
		host_io->timing = ios->timing;
	}

	return 0;
}

/*---------------------------------------------------------------------------
 * SDHC API: card present, tuning, busy, host props
 *---------------------------------------------------------------------------*/
static int zynq_sdhc_get_card_present(const struct device *dev)
{
	struct zynq_sdhc_data *sdhc_data = dev->data;
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);

	sdhc_data->card_present = (regs->present_state & ZYNQ_SDHC_HOST_PSTATE_CARD_INSERTED) != 0;
	if (!sdhc_data->card_present) {
		LOG_ERR("No card inserted");
	}

	return (int)sdhc_data->card_present;
}

static int zynq_sdhc_execute_tuning(const struct device *dev)
{
	if (!IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_TUNING)) {
		return 0;
	}

	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);

	regs->host_ctrl2 |= ZYNQ_SDHC_HOST_START_TUNING;

	for (int i = 0; i < HW_POLL_TIMEOUT_ITERS; i++) {
		if (!(regs->host_ctrl2 & ZYNQ_SDHC_HOST_START_TUNING)) {
			break;
		}
		k_busy_wait(HW_POLL_INTERVAL_US);
		if (i == HW_POLL_TIMEOUT_ITERS - 1) {
			LOG_ERR("Tuning timeout");
			return -ETIMEDOUT;
		}
	}

	if (!(regs->host_ctrl2 & ZYNQ_SDHC_HOST_TUNING_SUCCESS)) {
		LOG_ERR("Tuning failed");
		return -EIO;
	}

	return 0;
}

static int zynq_sdhc_card_busy(const struct device *dev)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	uint32_t busy_mask = ZYNQ_SDHC_HOST_PSTATE_CMD_INHIBIT |
			     ZYNQ_SDHC_HOST_PSTATE_DAT_INHIBIT |
			     ZYNQ_SDHC_HOST_PSTATE_DAT_LINE_ACTIVE;

	return (regs->present_state & busy_mask) ? 1 : 0;
}

static int zynq_sdhc_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
	const struct zynq_sdhc_config *cfg = dev->config;
	struct zynq_sdhc_data *data = dev->data;
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	uint64_t cap = regs->capabilities;

	data->host_caps = cap;

	memset(props, 0, sizeof(*props));
	props->f_max = cfg->max_bus_freq;
	props->f_min = cfg->min_bus_freq;
	props->power_delay = cfg->power_delay_ms;

	props->host_caps.vol_180_support = (bool)(cap & BIT(26));
	props->host_caps.vol_300_support = (bool)(cap & BIT(25));
	props->host_caps.vol_330_support = (bool)(cap & BIT(24));
	props->host_caps.suspend_res_support = false;
	props->host_caps.sdma_support = (bool)(cap & BIT(22));
	props->host_caps.high_spd_support = (bool)(cap & BIT(21));
	props->host_caps.adma_2_support = (bool)(cap & BIT(19));
	props->host_caps.max_blk_len = (cap >> 16) & 0x3;
	props->host_caps.ddr50_support = (bool)(cap & BIT64(34));
	props->host_caps.sdr104_support = (bool)(cap & BIT64(33));
	props->host_caps.sdr50_support = (bool)(cap & BIT64(32));
	props->host_caps.bus_8_bit_support = (data->bus_width == SDHC_BUS_WIDTH8BIT);
	props->bus_4_bit_support = (data->bus_width == SDHC_BUS_WIDTH4BIT);
	props->hs200_support = cfg->hs200_mode;
	props->hs400_support = cfg->hs400_mode;

	data->props = *props;
	return 0;
}

/*---------------------------------------------------------------------------
 * ISR
 *---------------------------------------------------------------------------*/
static void zynq_sdhc_isr(const struct device *dev)
{
	struct zynq_sdhc_data *sdhc_data = dev->data;
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	uint16_t norm_stat = regs->normal_int_stat;
	uint16_t err_stat = regs->err_int_stat;

	/* Clear and post normal interrupt events (W1C) */
	if (norm_stat) {
		regs->normal_int_stat = norm_stat;
		k_event_post(&sdhc_data->irq_event, norm_stat);
	}

	/* Clear and post error interrupt events (W1C) */
	if (err_stat) {
		LOG_ERR("ISR error: 0x%04x", err_stat);
		regs->err_int_stat = err_stat;
		k_event_post(&sdhc_data->irq_event, ERR_INTR_STATUS_EVENT(err_stat));
	}
}

/*---------------------------------------------------------------------------
 * Init
 *---------------------------------------------------------------------------*/
static int zynq_sdhc_init(const struct device *dev)
{
	const struct zynq_sdhc_config *config = dev->config;
	struct zynq_sdhc_data *data = dev->data;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

#ifdef CONFIG_PINCTRL
	int err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);

	if (err < 0) {
		LOG_ERR("Failed to apply pinctrl state: %d", err);
		return err;
	}
#endif

	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	uint8_t spec_ver = regs->host_cntrl_version & XSDPS_HC_SPEC_VER_MASK;

	switch (spec_ver) {
	case ZYNQ_SDHC_HC_SPEC_V1:
		data->hc_ver = SD_SPEC_VER1_0;
		break;
	case ZYNQ_SDHC_HC_SPEC_V2:
		data->hc_ver = SD_SPEC_VER2_0;
		break;
	case ZYNQ_SDHC_HC_SPEC_V3:
		data->hc_ver = SD_SPEC_VER3_0;
		break;
	default:
		data->hc_ver = SD_SPEC_VER1_0;
		break;
	}

	data->host_caps = regs->capabilities;

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
		k_event_init(&data->irq_event);
		config->config_func(dev);
	}

	memset(&data->host_io, 0, sizeof(data->host_io));
	memset(data->adma_desc_tbl, 0, sizeof(adma_desc_t) * ADMA_DESC_SIZE);

	return zynq_sdhc_reset(dev);
}

/*---------------------------------------------------------------------------
 * API and DT instantiation
 *---------------------------------------------------------------------------*/
static DEVICE_API(sdhc, zynq_sdhc_api) = {
	.reset = zynq_sdhc_reset,
	.request = zynq_sdhc_request,
	.set_io = zynq_sdhc_set_io,
	.get_card_present = zynq_sdhc_get_card_present,
	.execute_tuning = zynq_sdhc_execute_tuning,
	.card_busy = zynq_sdhc_card_busy,
	.get_host_props = zynq_sdhc_get_host_props,
};

#ifdef CONFIG_PINCTRL
#define XLNX_SDHC_PINCTRL_DEFINE(port) PINCTRL_DT_INST_DEFINE(port);
#define XLNX_SDHC_PINCTRL_INIT(port)   .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(port),
#else
#define XLNX_SDHC_PINCTRL_DEFINE(port)
#define XLNX_SDHC_PINCTRL_INIT(port)
#endif

#if CONFIG_XLNX_ZYNQ_DESC_ALIGN_OCM
BUILD_ASSERT(DT_NODE_HAS_STATUS(DT_NODELABEL(ocm_high), okay));
#define XLNX_SDHC_ADMA_DESC_DEFINE(port)                                                           \
	static adma_desc_t adma_desc_tbl_##port[ADMA_DESC_SIZE] __aligned(32)                      \
		__attribute__((section("OCM_HIGH")));
#else
#define XLNX_SDHC_ADMA_DESC_DEFINE(port)                                                           \
	static adma_desc_t adma_desc_tbl_##port[ADMA_DESC_SIZE] __aligned(32);
#endif

#define ZYNQ_SDHC_INIT(n)                                                                          \
	XLNX_SDHC_PINCTRL_DEFINE(n)                                                                \
	XLNX_SDHC_ADMA_DESC_DEFINE(n)                                                              \
	static void zynq_sdhc_##n##_irq_config_func(const struct device *dev)                      \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), zynq_sdhc_isr,             \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
	static const struct zynq_sdhc_config zynq_sdhc_##n##_config = {                            \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                              \
		.config_func = zynq_sdhc_##n##_irq_config_func,                                    \
		.clock_freq = DT_INST_PROP(n, clock_frequency),                                    \
		.min_bus_freq = DT_INST_PROP(n, min_bus_freq),                                     \
		.max_bus_freq = DT_INST_PROP(n, max_bus_freq),                                     \
		.power_delay_ms = DT_INST_PROP(n, power_delay_ms),                                 \
		.dw_4bit = DT_INST_ENUM_HAS_VALUE(n, bus_width, 4),                                \
		.dw_8bit = DT_INST_ENUM_HAS_VALUE(n, bus_width, 8),                                \
		.hs200_mode = DT_INST_PROP(n, mmc_hs200_1_8v),                                    \
		.hs400_mode = DT_INST_PROP(n, mmc_hs400_1_8v),                                    \
		XLNX_SDHC_PINCTRL_INIT(n)                                                         \
	};                                                                                         \
	static struct zynq_sdhc_data zynq_sdhc_##n##_data = {                                      \
		.card_present = false,                                                             \
		.bus_width = DT_INST_PROP(n, bus_width),                                           \
		.slot_type = DT_INST_PROP(n, slot_type),                                           \
		.adma_desc_tbl = adma_desc_tbl_##n,                                                \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, &zynq_sdhc_init, NULL, &zynq_sdhc_##n##_data,                    \
			      &zynq_sdhc_##n##_config, POST_KERNEL,                                \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &zynq_sdhc_api);

DT_INST_FOREACH_STATUS_OKAY(ZYNQ_SDHC_INIT)
