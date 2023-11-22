#include "zephyr/cache.h"
#include "zephyr/device.h"
#include "zephyr/drivers/sdhc.h"
#include "xlnx_zynq_sdhc.h"
#include "zephyr/sys/util_macro.h"
#include <sys/errno.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/sd/sd_spec.h>
#include <zephyr/logging/log.h>
#include <soc.h>
#include <zephyr/irq.h>

#define DT_DRV_COMPAT xlnx_zynq_sdhc

LOG_MODULE_REGISTER(sdhc, CONFIG_SDHC_LOG_LEVEL);

#ifdef CONFIG_XLNX_ZYNQ_SDHC_HOST_ADMA_DESC_SIZE
#define ADMA_DESC_SIZE CONFIG_XLNX_ZYNQ_SDHC_HOST_ADMA_DESC_SIZE
#else
#define ADMA_DESC_SIZE 32
#endif
/*----------------------------------------------------------------------------------------
			STRUCTURE DEFINITIONS
----------------------------------------------------------------------------------------*/
#define DEV_CFG(dev) ((const struct zynq_sdhc_config *const)((dev)->config))
#define DEV_REG(dev) ((volatile struct zynq_sdhc_reg *)(DEV_CFG(dev))->base)

typedef void (*zynq_sdhc_isr_cb_t)(const struct device *dev);

struct zynq_sdhc_config {
	intptr_t base;
	zynq_sdhc_isr_cb_t config_func;
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
	const struct device *dev;
	struct zynq_sdhc_config config;
	struct sdhc_host_props props;

	uint8_t cart_type;
	uint8_t card_ver;
	uint8_t hc_ver;
	uint64_t caps;
	uint8_t bus_width;
	uint32_t rca;

	struct sdhc_io host_io;
	struct k_sem lock;
	struct k_event irq_event;

	bool card_present;

	struct adma_desc adma_desc_tbl[ADMA_DESC_SIZE] __aligned((32));
};

/*----------------------------------------------------------------------------------------
			PROTOTYPES
----------------------------------------------------------------------------------------*/

static void enable_interrupts(const struct device *dev);
static void disable_interrupts(const struct device *dev);
static void clear_interrupts(const struct device *dev);

static int zynq_sdhc_host_sw_reset(const struct device *dev, enum zynq_sdhc_swrst sr);
static int zynq_sdhc_dma_init(const struct device *dev, struct sdhc_data *data, bool read);
static int zynq_sdhc_init_xfr(const struct device *dev, struct sdhc_data *data, bool read);

static int read_data_port(const struct device *dev, struct sdhc_data *sdhc);
static int write_data_port(const struct device *dev, struct sdhc_data *sdhc);
static int wait_xfr_poll_complete(const struct device *dev, uint32_t time_out);

static int zynq_sdhc_set_voltage(const struct device *dev, enum sd_voltage signal_voltage);
static int zynq_sdhc_set_power(const struct device *dev, enum sdhc_power state);

static bool zynq_sdhc_clock_set(const struct device *dev, enum sdhc_clock_speed speed);
static bool zynq_sdhc_enable_clock(const struct device *dev);
static bool zynq_sdhc_disable_clock(const struct device *dev);
static bool zynq_sdhc_clock_set(const struct device *dev, enum sdhc_clock_speed speed);
static int set_timing(const struct device *dev, enum sdhc_timing_mode timing);

static int zynq_sdhc_set_io(const struct device *dev, struct sdhc_io *ios);

#if defined(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)
static int wait_for_cmd_complete(struct zynq_sdhc_data *sdhc_data, uint32_t time_out);
#else
static int poll_cmd_complete(const struct device *dev, uint32_t time_out);
#endif

/*----------------------------------------------------------------------------------------*/

static int zynq_sdhc_set_voltage(const struct device *dev, enum sd_voltage signal_voltage)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	bool power_state = regs->power_ctrl & ZYNQ_SDHC_HOST_POWER_CTRL_SD_BUS_POWER ? true : false;
	int ret = 0;

	if (power_state) {
		/* Turn OFF Bus Power before config clock */
		regs->power_ctrl &= ~ZYNQ_SDHC_HOST_POWER_CTRL_SD_BUS_POWER;
	}

	switch (signal_voltage) {
	case SD_VOL_3_3_V:
		if (regs->capabilities & ZYNQ_SDHC_HOST_VOL_3_3_V_SUPPORT) {
			regs->host_ctrl2 &= ~(ZYNQ_SDHC_HOST_CTRL2_1P8V_SIG_EN << ZYNQ_SDHC_HOST_CTRL2_1P8V_SIG_LOC);

			/* 3.3v voltage select */
			regs->power_ctrl = ZYNQ_SDHC_HOST_VOL_3_3_V_SELECT;
			LOG_DBG("3.3V Selected for MMC Card");
		} else {
			LOG_ERR("3.3V not supported by MMC Host");
			ret = -ENOTSUP;
		}
		break;

	case SD_VOL_3_0_V:
		if (regs->capabilities & ZYNQ_SDHC_HOST_VOL_3_0_V_SUPPORT) {
			regs->host_ctrl2 &= ~(ZYNQ_SDHC_HOST_CTRL2_1P8V_SIG_EN << ZYNQ_SDHC_HOST_CTRL2_1P8V_SIG_LOC);

			/* 3.0v voltage select */
			regs->power_ctrl = ZYNQ_SDHC_HOST_VOL_3_0_V_SELECT;
			LOG_DBG("3.0V Selected for MMC Card");
		} else {
			LOG_ERR("3.0V not supported by MMC Host");
			ret = -ENOTSUP;
		}
		break;

	case SD_VOL_1_8_V:
		if (regs->capabilities & ZYNQ_SDHC_HOST_VOL_1_8_V_SUPPORT) {
			regs->host_ctrl2 |= ZYNQ_SDHC_HOST_CTRL2_1P8V_SIG_EN << ZYNQ_SDHC_HOST_CTRL2_1P8V_SIG_LOC;

			/* 1.8v voltage select */
			regs->power_ctrl = ZYNQ_SDHC_HOST_VOL_1_8_V_SELECT;
			LOG_DBG("1.8V Selected for MMC Card");
		} else {
			LOG_ERR("1.8V not supported by MMC Host");
			ret = -ENOTSUP;
		}
		break;

	default:
		ret = -EINVAL;
	}

	if (power_state) {
		/* Turn ON Bus Power */
		regs->power_ctrl |= ZYNQ_SDHC_HOST_POWER_CTRL_SD_BUS_POWER;
	}

	return ret;
}

static int zynq_sdhc_set_power(const struct device *dev, enum sdhc_power state)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);

	if (state == SDHC_POWER_ON) {
		/* Turn ON Bus Power */
		regs->power_ctrl |= ZYNQ_SDHC_HOST_POWER_CTRL_SD_BUS_POWER;
	} else {
		/* Turn OFF Bus Power */
		regs->power_ctrl &= ~ZYNQ_SDHC_HOST_POWER_CTRL_SD_BUS_POWER;
	}

	k_msleep(10u);

	return 0;
}

static bool zynq_sdhc_disable_clock(const struct device *dev)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);

	if (regs->present_state & ZYNQ_SDHC_HOST_PSTATE_CMD_INHIBIT) {
		LOG_ERR("present_state:%x", regs->present_state);
		return false;
	}
	if (regs->present_state & ZYNQ_SDHC_HOST_PSTATE_DAT_INHIBIT) {
		LOG_ERR("present_state:%x", regs->present_state);
		return false;
	}

	regs->clock_ctrl &= ~ZYNQ_SDHC_HOST_INTERNAL_CLOCK_EN;
	regs->clock_ctrl &= ~ZYNQ_SDHC_HOST_SD_CLOCK_EN;

	while ((regs->clock_ctrl & ZYNQ_SDHC_HOST_SD_CLOCK_EN) != 0) {
		;
	}

	return true;
}

static bool zynq_sdhc_enable_clock(const struct device *dev)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);

	regs->clock_ctrl |= ZYNQ_SDHC_HOST_INTERNAL_CLOCK_EN;
	/* Wait for the stable Internal Clock */
	while ((regs->clock_ctrl & ZYNQ_SDHC_HOST_INTERNAL_CLOCK_STABLE) == 0) {
		;
	}

	/* Enable SD Clock */
	regs->clock_ctrl |= ZYNQ_SDHC_HOST_SD_CLOCK_EN;
	while ((regs->clock_ctrl & ZYNQ_SDHC_HOST_SD_CLOCK_EN) == 0) {
		;
	}

	return true;
}

static bool zynq_sdhc_clock_set(const struct device *dev, enum sdhc_clock_speed speed)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	uint8_t base_freq;
	uint32_t clock_divider;
	float freq;
	bool ret;

	switch (speed) {
	case SDMMC_CLOCK_400KHZ:
		freq = ZYNQ_SDHC_HOST_CLK_FREQ_400K;
		break;

	case SD_CLOCK_25MHZ:
	case MMC_CLOCK_26MHZ:
		freq = ZYNQ_SDHC_HOST_CLK_FREQ_25M;
		break;

	case SD_CLOCK_50MHZ:
	case MMC_CLOCK_52MHZ:
		freq = ZYNQ_SDHC_HOST_CLK_FREQ_50M;
		break;

	case SD_CLOCK_100MHZ:
		freq = ZYNQ_SDHC_HOST_CLK_FREQ_100M;
		break;

	case MMC_CLOCK_HS200:
		freq = ZYNQ_SDHC_HOST_CLK_FREQ_200M;
		break;

	case SD_CLOCK_208MHZ:
	default:
		return false;
	}

	ret = zynq_sdhc_disable_clock(dev);
	if (!ret) {
		return false;
	}

	base_freq = regs->capabilities >> 8;
	clock_divider = (int)((float)base_freq / (freq * 2));

	LOG_DBG("Clock divider for MMC Clk: %d Hz is %d", speed, clock_divider);

	SET_BITS(regs->clock_ctrl, ZYNQ_SDHC_HOST_CLK_SDCLCK_FREQ_SEL_LOC, ZYNQ_SDHC_HOST_CLK_SDCLCK_FREQ_SEL_MASK,
		 clock_divider);
	SET_BITS(regs->clock_ctrl, ZYNQ_SDHC_HOST_CLK_SDCLCK_FREQ_SEL_UPPER_LOC,
		 ZYNQ_SDHC_HOST_CLK_SDCLCK_FREQ_SEL_UPPER_MASK, clock_divider >> 8);

	zynq_sdhc_enable_clock(dev);

	return true;
}

static int set_timing(const struct device *dev, enum sdhc_timing_mode timing)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	int ret = 0;
	uint8_t mode;

	LOG_DBG("UHS Mode: %d", timing);

	switch (timing) {
	case SDHC_TIMING_LEGACY:
	case SDHC_TIMING_HS:
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

	case SDHC_TIMING_HS400:
	case SDHC_TIMING_HS200:
		mode = ZYNQ_SDHC_HOST_UHSMODE_HS400;
		break;

	default:
		ret = -ENOTSUP;
	}

	if (!ret) {
		if (!zynq_sdhc_disable_clock(dev)) {
			LOG_ERR("Disable clk failed");
			return -EIO;
		}
		regs->host_ctrl2 |= ZYNQ_SDHC_HOST_CTRL2_1P8V_SIG_EN << ZYNQ_SDHC_HOST_CTRL2_1P8V_SIG_LOC;
		SET_BITS(regs->host_ctrl2, ZYNQ_SDHC_HOST_CTRL2_UHS_MODE_SEL_LOC,
			 ZYNQ_SDHC_HOST_CTRL2_UHS_MODE_SEL_MASK, mode);

		zynq_sdhc_enable_clock(dev);
	}

	return ret;
}

static void enable_interrupts(const struct device *dev)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	regs->normal_int_stat_en = ZYNQ_SDHC_HOST_NORMAL_INTR_MASK;
	regs->err_int_stat_en = ZYNQ_SDHC_HOST_ERROR_INTR_MASK;
	regs->normal_int_signal_en = ZYNQ_SDHC_HOST_NORMAL_INTR_MASK;
	regs->err_int_signal_en = ZYNQ_SDHC_HOST_ERROR_INTR_MASK;
	regs->timeout_ctrl = ZYNQ_SDHC_HOST_MAX_TIMEOUT;
}

static void disable_interrupts(const struct device *dev)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	regs->normal_int_stat_en = ZYNQ_SDHC_HOST_NORMAL_INTR_MASK;
	regs->err_int_stat_en = ZYNQ_SDHC_HOST_ERROR_INTR_MASK;

	regs->normal_int_signal_en &= 0;
	regs->err_int_signal_en &= 0;
	regs->timeout_ctrl = ZYNQ_SDHC_HOST_MAX_TIMEOUT;
}

static void clear_interrupts(const struct device *dev)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	regs->normal_int_stat = ZYNQ_SDHC_HOST_NORMAL_INTR_MASK_CLR;
	regs->err_int_stat = ZYNQ_SDHC_HOST_ERROR_INTR_MASK;
}

static int zynq_sdhc_host_sw_reset(const struct device *dev, enum zynq_sdhc_swrst sr)
{

	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	uint32_t timeout = 100U;
	switch (sr) {
	case ZYNQ_SDHC_HOST_SWRST_ALL:
		regs->sw_reset = ZYNQ_SDHC_HOST_SW_RESET_REG_ALL;
		break;
	case ZYNQ_SDHC_HOST_SWRST_CMD_LINE:
		regs->sw_reset = ZYNQ_SDHC_HOST_SW_RESET_REG_CMD;
		break;
	case ZYNQ_SDHC_HOST_SWRST_DATA_LINE:
		regs->sw_reset = ZYNQ_SDHC_HOST_SW_RESET_REG_DATA;
		break;
	}

	for (; timeout > 0; timeout--) {
		if (regs->sw_reset == 0) {
			break;
		}
		k_sleep(K_MSEC(1U));
	}

	if (timeout == 0) {
		LOG_ERR("Software reset failed");
		return -EIO;
	}
	return 0;
}

static int zynq_sdhc_dma_init(const struct device *dev, struct sdhc_data *data, bool read) // NOLINT
{
	struct zynq_sdhc_data *sdhc_data = dev->data;
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);

	if (!read) {
		sys_cache_data_flush_range(data->data, (data->blocks * data->block_size));
	}

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_ADMA)) {
		uint8_t *buff = data->data;

		for (int i = 0; i < data->blocks; i++) {
			bool is_last = (i == (data->blocks - 1));
#if defined(CONFIG_64BIT)
			sdhc_data->adma_desc_tbl[i].address = (uint64_t)(buff);
#else
			sdhc_data->adma_desc_tbl[i].address = (uint32_t)(buff);
#endif
			sdhc_data->adma_desc_tbl[i].len = data->block_size;

			if (is_last) {
				sdhc_data->adma_desc_tbl[i].attr |= ZYNQ_SDHC_HOST_ADMA_BUFF_LAST;
				sdhc_data->adma_desc_tbl[i].attr |= ZYNQ_SDHC_HOST_ADMA_INTR_EN;
				sdhc_data->adma_desc_tbl[i].attr |= ZYNQ_SDHC_HOST_ADMA_BUFF_LAST;
			} else {
				sdhc_data->adma_desc_tbl[i].attr |= ZYNQ_SDHC_HOST_ADMA_BUFF_LINK_NEXT;
			}

			sdhc_data->adma_desc_tbl[i].attr |= ZYNQ_SDHC_HOST_ADMA_BUFF_VALID;
			buff += data->block_size;
			LOG_DBG("adma_tbl entry: addr:%llx, attr:%u len:%u", sdhc_data->adma_desc_tbl[i].address,
				sdhc_data->adma_desc_tbl[i].attr, sdhc_data->adma_desc_tbl[i].len);
		}
		regs->adma_sys_addr1 = (uint32_t)(sdhc_data->adma_desc_tbl);
#if defined(CONFIG_64BIT)

		bool is_hc_v3 = (sdhc_data->hc_ver == ZYNQ_SDHC_HC_SPEC_V3);
		if (is_hc_v3) {
			regs->adma_sys_addr2 = (uint32_t)(((uint64_t)sdhc_data->adma_desc_tbl) >> 32U);
		}
#endif

	} else {
		/* Setup DMA transfer using SDMA */
		// regs->sdma_sysaddr = (uint32_t)((uint64_t)data->data);
#if defined(CONFIG_64BIT)
		regs->sdma_sysaddr = (uint32_t)(((uint64_t)data->data));
#else
		regs->sdma_sysaddr = (uint32_t)(data->data);
#endif
	}
	return 0;
}

static int zynq_sdhc_init_xfr(const struct device *dev, struct sdhc_data *data, bool read)
{
	struct zynq_sdhc_data *sdhc_data = dev->data;
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	uint16_t multi_block = 0u;

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_DMA)) {
		zynq_sdhc_dma_init(dev, data, read);
	}

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_ADMA)) {
		SET_BITS(regs->host_ctrl1, ZYNQ_SDHC_HOST_CTRL1_DMA_SEL_LOC, ZYNQ_SDHC_HOST_CTRL1_DMA_SEL_MASK, 2U);
	} else {
		SET_BITS(regs->host_ctrl1, ZYNQ_SDHC_HOST_CTRL1_DMA_SEL_LOC, ZYNQ_SDHC_HOST_CTRL1_DMA_SEL_MASK, 0U);
	}
	/* set block size register */
	SET_BITS(regs->block_size, ZYNQ_SDHC_HOST_DMA_BUF_SIZE_LOC, ZYNQ_SDHC_HOST_DMA_BUF_SIZE_MASK,
		 ZYNQ_SDHC_HOST_SDMA_BOUNDARY);
	SET_BITS(regs->block_size, ZYNQ_SDHC_HOST_BLOCK_SIZE_LOC, ZYNQ_SDHC_HOST_BLOCK_SIZE_MASK, data->block_size);

	if (data->blocks > 1) {
		multi_block = 1u;
	}

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_AUTO_STOP)) {
		if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_ADMA) && sdhc_data->host_io.timing == SDHC_TIMING_SDR104) {
			/* Auto cmd23 only applicable for ADMA */
			SET_BITS(regs->transfer_mode, ZYNQ_SDHC_HOST_XFER_AUTO_CMD_EN_LOC,
				 ZYNQ_SDHC_HOST_XFER_AUTO_CMD_EN_MASK, multi_block ? 2 : 0);
		} else {
			SET_BITS(regs->transfer_mode, ZYNQ_SDHC_HOST_XFER_AUTO_CMD_EN_LOC,
				 ZYNQ_SDHC_HOST_XFER_AUTO_CMD_EN_MASK, multi_block ? 1 : 0);
		}
	} else {
		SET_BITS(regs->transfer_mode, ZYNQ_SDHC_HOST_XFER_AUTO_CMD_EN_LOC, ZYNQ_SDHC_HOST_XFER_AUTO_CMD_EN_MASK,
			 0);
	}

	if (!IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_AUTO_STOP)) {
		/* Set block count register to 0 for infinite transfer mode */
		regs->block_count = 0;
		SET_BITS(regs->transfer_mode, ZYNQ_SDHC_HOST_XFER_BLOCK_CNT_EN_LOC,
			 ZYNQ_SDHC_HOST_XFER_BLOCK_CNT_EN_MASK, 0);
	} else {
		regs->block_count = (uint16_t)data->blocks;
		/* Enable block count in transfer register */
		SET_BITS(regs->transfer_mode, ZYNQ_SDHC_HOST_XFER_BLOCK_CNT_EN_LOC,
			 ZYNQ_SDHC_HOST_XFER_BLOCK_CNT_EN_MASK, multi_block ? 1 : 0);
	}

	SET_BITS(regs->transfer_mode, ZYNQ_SDHC_HOST_XFER_MULTI_BLOCK_SEL_LOC, ZYNQ_SDHC_HOST_XFER_MULTI_BLOCK_SEL_MASK,
		 multi_block);

	/* Set data transfer direction, Read = 1, Write = 0 */
	SET_BITS(regs->transfer_mode, ZYNQ_SDHC_HOST_XFER_DATA_DIR_LOC, ZYNQ_SDHC_HOST_XFER_DATA_DIR_MASK,
		 read ? 1u : 0u);

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_DMA)) {
		/* Enable DMA or not */
		SET_BITS(regs->transfer_mode, ZYNQ_SDHC_HOST_XFER_DMA_EN_LOC, ZYNQ_SDHC_HOST_XFER_DMA_EN_MASK, 1u);
	} else {
		SET_BITS(regs->transfer_mode, ZYNQ_SDHC_HOST_XFER_DMA_EN_LOC, ZYNQ_SDHC_HOST_XFER_DMA_EN_MASK, 0u);
	}

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_BLOCK_GAP)) {
		/* Set an interrupt at the block gap */
		SET_BITS(regs->block_gap_ctrl, ZYNQ_SDHC_HOST_BLOCK_GAP_LOC, ZYNQ_SDHC_HOST_BLOCK_GAP_MASK, 1u);
	} else {
		SET_BITS(regs->block_gap_ctrl, ZYNQ_SDHC_HOST_BLOCK_GAP_LOC, ZYNQ_SDHC_HOST_BLOCK_GAP_MASK, 0u);
	}

	/* Set data timeout time */
	regs->timeout_ctrl = data->timeout_ms;
	return 0;
}

static int wait_xfr_intr_complete(const struct device *dev, uint32_t time_out)
{
	struct zynq_sdhc_data *emmc = dev->data;
	uint32_t events;
	int ret;
	k_timeout_t wait_time;

	LOG_DBG("");

	if (time_out == SDHC_TIMEOUT_FOREVER) {
		wait_time = K_FOREVER;
	} else {
		wait_time = K_MSEC(time_out);
	}

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
		events = k_event_wait(&emmc->irq_event,
				      ZYNQ_SDHC_HOST_XFER_COMPLETE | ERR_INTR_STATUS_EVENT(ZYNQ_SDHC_HOST_DMA_TXFR_ERR),
				      false, wait_time);
	}

	if (events & ZYNQ_SDHC_HOST_XFER_COMPLETE) {
		ret = 0;
	} else if (events & ERR_INTR_STATUS_EVENT(0xFFFF)) {
		LOG_ERR("wait for xfer complete error: %x", events);
		ret = -EIO;
	} else {
		LOG_ERR("wait for xfer complete timeout");
		ret = -EAGAIN;
	}

	return ret;
}

static int wait_xfr_poll_complete(const struct device *dev, uint32_t time_out)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	int ret = -EAGAIN;
	int32_t retry = (int32_t)time_out;

	LOG_DBG("");

	while (retry > 0) {
		if (regs->normal_int_stat & ZYNQ_SDHC_HOST_XFER_COMPLETE) {
			regs->normal_int_stat |= ZYNQ_SDHC_HOST_XFER_COMPLETE;
			ret = 0;
			break;
		}

		k_busy_wait(ZYNQ_SDHC_HOST_MSEC_DELAY);
		retry--;
	}

	return ret;
}

static int wait_xfr_complete(const struct device *dev, uint32_t time_out)
{
	int ret;

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
		ret = wait_xfr_intr_complete(dev, time_out);
	} else {
		ret = wait_xfr_poll_complete(dev, time_out);
	}
	return ret;
}

static enum zynq_sdhc_resp_type zynq_sdhc_decode_resp_type(enum sd_rsp_type type)
{
	enum zynq_sdhc_resp_type resp_type;

	switch (type & 0xF) {
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

	case SD_RSP_TYPE_R5b:
	case SD_RSP_TYPE_R6:
	case SD_RSP_TYPE_R7:
	default:
		resp_type = ZYNQ_SDHC_HOST_INVAL_HOST_RESP;
	}

	return resp_type;
}

#if defined(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)
static int wait_for_cmd_complete(struct zynq_sdhc_data *sdhc_data, uint32_t time_out)
{
	int ret;
	k_timeout_t wait_time;
	uint32_t events;

	if (time_out == SDHC_TIMEOUT_FOREVER) {
		wait_time = K_FOREVER;
	} else {
		wait_time = K_MSEC(time_out);
	}

	events = k_event_wait(&sdhc_data->irq_event,
			      ZYNQ_SDHC_HOST_CMD_COMPLETE | ERR_INTR_STATUS_EVENT(ZYNQ_SDHC_HOST_ERR_STATUS), false,
			      wait_time);

	if (events & ZYNQ_SDHC_HOST_CMD_COMPLETE) {
		ret = 0;
	} else if (events & ERR_INTR_STATUS_EVENT(ZYNQ_SDHC_HOST_ERR_STATUS)) {
		LOG_ERR("wait for cmd complete error: %x", events);
		ret = -EIO;
	} else {
		LOG_ERR("wait for cmd complete timeout");
		ret = -EAGAIN;
	}

	return ret;
}
#else
static int poll_cmd_complete(const struct device *dev, uint32_t time_out)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	int ret = -EAGAIN;
	int32_t retry = time_out;

	while (retry > 0) {
		if (regs->normal_int_stat & ZYNQ_SDHC_HOST_CMD_COMPLETE) {
			regs->normal_int_stat = ZYNQ_SDHC_HOST_CMD_COMPLETE;
			ret = 0;
			break;
		}

		k_busy_wait(1000u);
		retry--;
	}

	if (regs->err_int_stat) {
		LOG_ERR("err_int_stat:%x", regs->err_int_stat);
		regs->err_int_stat &= regs->err_int_stat;
		ret = -EIO;
	}

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_ADMA)) {
		if (regs->adma_err_stat) {
			LOG_ERR("adma error: %x", regs->adma_err_stat);
			ret = -EIO;
		}
	}
	return ret;
}
#endif

static void update_cmd_response(const struct device *dev, struct sdhc_command *sdhc_cmd) // NOLINT
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	uint32_t resp0, resp1, resp2, resp3;

	if (sdhc_cmd->response_type == SD_RSP_TYPE_NONE) {
		return;
	}

	resp0 = regs->resp_01;

	if (sdhc_cmd->response_type == SD_RSP_TYPE_R2) {
		resp1 = regs->resp_2 | (regs->resp_3 << 16u);
		resp2 = regs->resp_4 | (regs->resp_5 << 16u);
		resp3 = regs->resp_6 | (regs->resp_7 << 16u);

		LOG_DBG("cmd resp: %x %x %x %x", resp0, resp1, resp2, resp3);

		sdhc_cmd->response[0u] = resp3;
		sdhc_cmd->response[1U] = resp2;
		sdhc_cmd->response[2U] = resp1;
		sdhc_cmd->response[3U] = resp0;
	} else {
		LOG_DBG("cmd resp: %x", resp0);
		sdhc_cmd->response[0u] = resp0;
	}
}

static int zynq_sdhc_host_send_cmd(const struct device *dev, const struct zynq_sdhc_cmd_config *config)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	struct zynq_sdhc_data *emmc = dev->data;
	struct sdhc_command *sdhc_cmd = config->sdhc_cmd;
	enum zynq_sdhc_resp_type resp_type = zynq_sdhc_decode_resp_type(sdhc_cmd->response_type);
	uint16_t cmd_reg;
	int ret;

	LOG_DBG("");

	/* Check if CMD line is available */
	if (regs->present_state & ZYNQ_SDHC_HOST_PSTATE_CMD_INHIBIT) {
		LOG_ERR("CMD line is not available");
		return -EBUSY;
	}

	if (config->data_present && (regs->present_state & ZYNQ_SDHC_HOST_PSTATE_DAT_INHIBIT)) {
		LOG_ERR("Data line is not available");
		return -EBUSY;
	}

	if (resp_type == ZYNQ_SDHC_HOST_INVAL_HOST_RESP) {
		LOG_ERR("Invalid eMMC resp type:%d", resp_type);
		return -EINVAL;
	}

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
		k_event_clear(&emmc->irq_event, ZYNQ_SDHC_HOST_CMD_COMPLETE);
	}

	regs->argument = sdhc_cmd->arg;

	cmd_reg = config->cmd_idx << ZYNQ_SDHC_HOST_CMD_INDEX_LOC | config->cmd_type << ZYNQ_SDHC_HOST_CMD_TYPE_LOC |
		  config->data_present << ZYNQ_SDHC_HOST_CMD_DATA_PRESENT_LOC |
		  config->idx_check_en << ZYNQ_SDHC_HOST_CMD_IDX_CHECK_EN_LOC |
		  config->crc_check_en << ZYNQ_SDHC_HOST_CMD_CRC_CHECK_EN_LOC |
		  resp_type << ZYNQ_SDHC_HOST_CMD_RESP_TYPE_LOC;
	regs->cmd = cmd_reg;

	LOG_DBG("CMD REG:%x %x", cmd_reg, regs->cmd);

#if defined(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)
	ret = wait_for_cmd_complete(emmc, sdhc_cmd->timeout_ms);
#else
	ret = poll_cmd_complete(dev, sdhc_cmd->timeout_ms);
#endif
	if (ret) {
		LOG_ERR("Error on send cmd: %d, status:%d", config->cmd_idx, ret);
		return ret;
	}

	update_cmd_response(dev, sdhc_cmd);

	return 0;
}

static int zynq_sdhc_send_cmd_no_data(const struct device *dev, uint32_t cmd_idx, struct sdhc_command *cmd)
{
	struct zynq_sdhc_cmd_config emmc_cmd;

	emmc_cmd.sdhc_cmd = cmd;
	emmc_cmd.cmd_idx = cmd_idx;
	emmc_cmd.cmd_type = ZYNQ_SDHC_HOST_CMD_NORMAL;
	emmc_cmd.data_present = false;
	emmc_cmd.idx_check_en = false;
	emmc_cmd.crc_check_en = false;

	return zynq_sdhc_host_send_cmd(dev, &emmc_cmd);
}

static int zynq_sdhc_send_cmd_data(const struct device *dev, uint32_t cmd_idx, struct sdhc_command *cmd,
				   struct sdhc_data *data, bool read)
{
	struct zynq_sdhc_cmd_config emmc_cmd;
	int ret;

	emmc_cmd.sdhc_cmd = cmd;
	emmc_cmd.cmd_idx = cmd_idx;
	emmc_cmd.cmd_type = ZYNQ_SDHC_HOST_CMD_NORMAL;
	emmc_cmd.data_present = true;
	emmc_cmd.idx_check_en = true;
	emmc_cmd.crc_check_en = true;

	ret = zynq_sdhc_init_xfr(dev, data, read);
	if (ret) {
		LOG_ERR("Error on init xfr");
		return ret;
	}

	ret = zynq_sdhc_host_send_cmd(dev, &emmc_cmd);
	if (ret) {
		return ret;
	}

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_DMA)) {
		ret = wait_xfr_complete(dev, data->timeout_ms);
	} else {
		if (read) {
			ret = read_data_port(dev, data);
		} else {
			ret = write_data_port(dev, data);
		}
	}

	return ret;
}

static int read_data_port(const struct device *dev, struct sdhc_data *sdhc)
{
	struct zynq_sdhc_data *emmc = dev->data;
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	uint32_t block_size = sdhc->block_size;
	uint32_t i, block_cnt = sdhc->blocks;
	uint32_t *data = (uint32_t *)sdhc->data;
	k_timeout_t wait_time;

	if (sdhc->timeout_ms == SDHC_TIMEOUT_FOREVER) {
		wait_time = K_FOREVER;
	} else {
		wait_time = K_MSEC(sdhc->timeout_ms);
	}

	LOG_DBG("");

	while (block_cnt--) {
		if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
			uint32_t events;

			events = k_event_wait(&emmc->irq_event, ZYNQ_SDHC_HOST_BUF_RD_READY, false, wait_time);
			k_event_clear(&emmc->irq_event, ZYNQ_SDHC_HOST_BUF_RD_READY);
			if (!(events & ZYNQ_SDHC_HOST_BUF_RD_READY)) {
				LOG_ERR("time out on ZYNQ_SDHC_HOST_BUF_RD_READY:%d", (sdhc->blocks - block_cnt));
				return -EIO;
			}
		} else {
			while ((regs->present_state & ZYNQ_SDHC_HOST_PSTATE_BUF_READ_EN) == 0) {
				;
			}
		}

		if (regs->present_state & ZYNQ_SDHC_HOST_PSTATE_DAT_INHIBIT) {
			for (i = block_size >> 2u; i != 0u; i--) {
				*data = regs->data_port;
				data++;
			}
		}
	}

	return wait_xfr_complete(dev, sdhc->timeout_ms);
}

static int write_data_port(const struct device *dev, struct sdhc_data *sdhc)
{
	struct zynq_sdhc_data *emmc = dev->data;
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	uint32_t block_size = sdhc->block_size;
	uint32_t i, block_cnt = sdhc->blocks;
	uint32_t *data = (uint32_t *)sdhc->data;
	k_timeout_t wait_time;

	if (sdhc->timeout_ms == SDHC_TIMEOUT_FOREVER) {
		wait_time = K_FOREVER;
	} else {
		wait_time = K_MSEC(sdhc->timeout_ms);
	}

	LOG_DBG("");

	while ((regs->present_state & ZYNQ_SDHC_HOST_PSTATE_BUF_WRITE_EN) == 0) {
		;
	}

	while (1) {
		uint32_t events;

		if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
			k_event_clear(&emmc->irq_event, ZYNQ_SDHC_HOST_BUF_WR_READY);
		}

		if (regs->present_state & ZYNQ_SDHC_HOST_PSTATE_DAT_INHIBIT) {
			for (i = block_size >> 2u; i != 0u; i--) {
				regs->data_port = *data;
				data++;
			}
		}

		LOG_DBG("ZYNQ_SDHC_HOST_BUF_WR_READY");

		if (!(--block_cnt)) {
			break;
		}
		if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
			events = k_event_wait(&emmc->irq_event, ZYNQ_SDHC_HOST_BUF_WR_READY, false, wait_time);
			k_event_clear(&emmc->irq_event, ZYNQ_SDHC_HOST_BUF_WR_READY);

			if (!(events & ZYNQ_SDHC_HOST_BUF_WR_READY)) {
				LOG_ERR("time out on ZYNQ_SDHC_HOST_BUF_WR_READY");
				return -EIO;
			}
		} else {
			while ((regs->present_state & ZYNQ_SDHC_HOST_PSTATE_BUF_WRITE_EN) == 0) {
				;
			}
		}
	}

	return wait_xfr_complete(dev, sdhc->timeout_ms);
}

static int zynq_sdhc_stop_transfer(const struct device *dev)
{
	struct zynq_sdhc_data *emmc = dev->data;
	struct sdhc_command hdc_cmd = {0};
	struct zynq_sdhc_cmd_config cmd;

	hdc_cmd.arg = emmc->rca << ZYNQ_SDHC_HOST_RCA_SHIFT;
	hdc_cmd.response_type = SD_RSP_TYPE_R1;
	hdc_cmd.timeout_ms = 1000;

	cmd.sdhc_cmd = &hdc_cmd;
	cmd.cmd_idx = SD_STOP_TRANSMISSION;
	cmd.cmd_type = ZYNQ_SDHC_HOST_CMD_NORMAL;
	cmd.data_present = false;
	cmd.idx_check_en = false;
	cmd.crc_check_en = false;

	return zynq_sdhc_host_send_cmd(dev, &cmd);
}

static int zynq_sdhc_xfr(const struct device *dev, struct sdhc_command *cmd, struct sdhc_data *data, bool read)
{
	struct zynq_sdhc_data *sdhc_data = dev->data;
	int ret = 0;
	struct zynq_sdhc_cmd_config zynq_sdhc_cmd;
	// struct
	ret = zynq_sdhc_init_xfr(dev, data, read);
	if (ret) {
		LOG_ERR("error emmc init xfr");
		return ret;
	}

	zynq_sdhc_cmd.sdhc_cmd = cmd;
	zynq_sdhc_cmd.cmd_type = ZYNQ_SDHC_HOST_CMD_NORMAL;
	zynq_sdhc_cmd.data_present = true;
	zynq_sdhc_cmd.idx_check_en = true;
	zynq_sdhc_cmd.crc_check_en = true;

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
		k_event_clear(&sdhc_data->irq_event, ZYNQ_SDHC_HOST_XFER_COMPLETE);
		k_event_clear(&sdhc_data->irq_event, read ? ZYNQ_SDHC_HOST_BUF_RD_READY : ZYNQ_SDHC_HOST_BUF_WR_READY);
	}

	if (data->blocks > 1) {
		zynq_sdhc_cmd.cmd_idx = read ? SD_READ_MULTIPLE_BLOCK : SD_WRITE_MULTIPLE_BLOCK;
		ret = zynq_sdhc_host_send_cmd(dev, &zynq_sdhc_cmd);
	} else {
		zynq_sdhc_cmd.cmd_idx = read ? SD_READ_SINGLE_BLOCK : SD_WRITE_SINGLE_BLOCK;
		ret = zynq_sdhc_host_send_cmd(dev, &zynq_sdhc_cmd);
	}

	if (ret) {
		return ret;
	}

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_DMA)) {
		ret = wait_xfr_complete(dev, data->timeout_ms);
	} else {
		if (read) {
			ret = read_data_port(dev, data);
		} else {
			ret = write_data_port(dev, data);
		}
	}

	if (!IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_AUTO_STOP)) {
		zynq_sdhc_stop_transfer(dev);
	}
	return ret;
}

/**
 * @brief RESET the SDHC controller
 *
 * @param dev
 * @return int
 */
static int zynq_sdhc_reset(const struct device *dev)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	int ret = 0;

	if (!(regs->present_state & ZYNQ_SDHC_HOST_PSTATE_CARD_INSERTED)) {
		LOG_ERR("No card inserted");
		return -ENODEV;
	}

	ret = zynq_sdhc_host_sw_reset(dev, ZYNQ_SDHC_HOST_SWRST_ALL);
	if (ret != 0) {
		return ret;
	}
	clear_interrupts(dev);

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
		enable_interrupts(dev);
	} else {
		disable_interrupts(dev);
	}

	return 0;
}

static int zynq_sdhc_request(const struct device *dev, struct sdhc_command *cmd, struct sdhc_data *data)
{
	// ARG_UNUSED(dev);
	// ARG_UNUSED(cmd);
	// ARG_UNUSED(data);
	int ret = 0;

	if (data) {
		switch (cmd->opcode) {
		case SD_WRITE_SINGLE_BLOCK:
		case SD_WRITE_MULTIPLE_BLOCK:
			LOG_DBG("SD_WRITE_SINGLE_BLOCK");
			ret = zynq_sdhc_xfr(dev, cmd, data, false);
			break;

		case SD_READ_SINGLE_BLOCK:
		case SD_READ_MULTIPLE_BLOCK:
			LOG_DBG("SD_READ_SINGLE_BLOCK");
			ret = zynq_sdhc_xfr(dev, cmd, data, true);
			break;

		case MMC_SEND_EXT_CSD:
			LOG_DBG("EMMC_HOST_SEND_EXT_CSD");
			ret = zynq_sdhc_send_cmd_data(dev, MMC_SEND_EXT_CSD, cmd, data, true);
			break;

		default:
			ret = zynq_sdhc_send_cmd_data(dev, cmd->opcode, cmd, data, true);
		}
	} else {
		ret = zynq_sdhc_send_cmd_no_data(dev, cmd->opcode, cmd);
	}

	return ret;
}

static int zynq_sdhc_set_io(const struct device *dev, struct sdhc_io *ios)
{
	struct zynq_sdhc_data *emmc = dev->data;
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	struct sdhc_io *host_io = &emmc->host_io;
	int ret;

	LOG_DBG("emmc I/O: DW %d, Clk %d Hz, card power state %s, voltage %s", ios->bus_width, ios->clock,
		ios->power_mode == SDHC_POWER_ON ? "ON" : "OFF", ios->signal_voltage == SD_VOL_1_8_V ? "1.8V" : "3.3V");

	if (ios->clock && (ios->clock > emmc->props.f_max || ios->clock < emmc->props.f_min)) {
		LOG_ERR("Invalid argument for clock freq: %d Support max:%d and Min:%d", ios->clock, emmc->props.f_max,
			emmc->props.f_min);
		return -EINVAL;
	}

	/* Set HC clock */
	if (host_io->clock != ios->clock) {
		LOG_DBG("Clock: %d", host_io->clock);
		if (ios->clock != 0) {
			/* Enable clock */
			LOG_DBG("CLOCK: %d", ios->clock);
			if (!zynq_sdhc_clock_set(dev, ios->clock)) {
				return -ENOTSUP;
			}
		} else {
			zynq_sdhc_disable_clock(dev);
		}
		host_io->clock = ios->clock;
	}

	/* Set data width */
	if (host_io->bus_width != ios->bus_width) {
		LOG_DBG("bus_width: %d", host_io->bus_width);

		if (ios->bus_width == SDHC_BUS_WIDTH4BIT) {
			SET_BITS(regs->host_ctrl1, ZYNQ_SDHC_HOST_CTRL1_EXT_DAT_WIDTH_LOC,
				 ZYNQ_SDHC_HOST_CTRL1_EXT_DAT_WIDTH_MASK, ios->bus_width == SDHC_BUS_WIDTH8BIT ? 1 : 0);
		} else {
			SET_BITS(regs->host_ctrl1, ZYNQ_SDHC_HOST_CTRL1_DAT_WIDTH_LOC,
				 ZYNQ_SDHC_HOST_CTRL1_DAT_WIDTH_MASK, ios->bus_width == SDHC_BUS_WIDTH4BIT ? 1 : 0);
		}
		host_io->bus_width = ios->bus_width;
	}

	/* Set HC signal voltage */
	if (ios->signal_voltage != host_io->signal_voltage) {
		LOG_DBG("signal_voltage: %d", ios->signal_voltage);
		ret = zynq_sdhc_set_voltage(dev, ios->signal_voltage);
		if (ret) {
			LOG_ERR("Set signal volatge failed:%d", ret);
			return ret;
		}
		host_io->signal_voltage = ios->signal_voltage;
	}

	/* Set card power */
	if (host_io->power_mode != ios->power_mode) {
		LOG_DBG("power_mode: %d", ios->power_mode);

		ret = zynq_sdhc_set_power(dev, ios->power_mode);
		if (ret) {
			LOG_ERR("Set Bus power failed:%d", ret);
			return ret;
		}
		host_io->power_mode = ios->power_mode;
	}

	/* Set I/O timing */
	if (host_io->timing != ios->timing) {
		LOG_DBG("timing: %d", ios->timing);

		ret = set_timing(dev, ios->timing);
		if (ret) {
			LOG_ERR("Set timing failed:%d", ret);
			return ret;
		}
		host_io->timing = ios->timing;
	}

	return ret;
}

static int zynq_sdhc_get_card_present(const struct device *dev)
{
	struct zynq_sdhc_data *sdhc_data = dev->data;
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);

	sdhc_data->card_present = (bool)((regs->present_state >> 16U) & 1u);
	if (!sdhc_data->card_present) {
		LOG_ERR("No card inserted");
	}

	return ((int)sdhc_data->card_present);
}

static int zynq_sdhc_execute_tuning(const struct device *dev) // NOLINT
{
	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_TUNING)) {
		volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);

		LOG_DBG("Executing tuning...");
		regs->host_ctrl2 |= ZYNQ_SDHC_HOST_START_TUNING;
		while (!(regs->host_ctrl2 & ZYNQ_SDHC_HOST_START_TUNING)) {
			;
		}

		if (regs->host_ctrl2 & ZYNQ_SDHC_HOST_TUNING_SUCCESS) {
			LOG_DBG("Tuning Completed successful");
		} else {
			LOG_ERR("Tuning Failed");
			return -EIO;
		}
	}

	return 0;
}

static int zynq_sdhc_card_busy(const struct device *dev)
{
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	if (regs->present_state & 7U) {
		return 1;
	}
	return 0;
}

static int zynq_sdhc_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
	const struct zynq_sdhc_config *cfg = dev->config;
	struct zynq_sdhc_data *data = dev->data;
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	uint64_t cap = regs->capabilities;

	memset(props, 0, sizeof(struct sdhc_host_props));
	props->f_max = cfg->max_bus_freq;
	props->f_min = cfg->min_bus_freq;
	props->power_delay = cfg->power_delay_ms;

	props->host_caps.vol_180_support = (bool)(cap & BIT(26U));
	props->host_caps.vol_300_support = (bool)(cap & BIT(25U));
	props->host_caps.vol_330_support = (bool)(cap & BIT(24U));
	props->host_caps.suspend_res_support = false;
	props->host_caps.sdma_support = (bool)(cap & BIT(22U));
	props->host_caps.high_spd_support = (bool)(cap & BIT(21U));
	props->host_caps.adma_2_support = (bool)(cap & BIT(19U));
	// FIXME(pan): may differ behavior for v3/v2
	props->host_caps.max_blk_len = (cap >> 16U) & (0x3U);
	props->host_caps.ddr50_support = (bool)(cap & BIT64(34U));
	props->host_caps.sdr104_support = (bool)(cap & BIT64(33U));
	props->host_caps.sdr50_support = (bool)(cap & BIT64(32U));
	props->host_caps.bus_8_bit_support = true;
	props->host_caps.bus_4_bit_support = true;
	props->host_caps.hs200_support = (bool)(cfg->hs200_mode);
	props->host_caps.hs400_support = (bool)(cfg->hs400_mode);

	data->props = *props;

	return 0;
}

static void zynq_sdhc_isr(const struct device *dev)
{
	struct zynq_sdhc_data *emmc = dev->data;
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);

	if (regs->normal_int_stat & ZYNQ_SDHC_HOST_CMD_COMPLETE) {
		regs->normal_int_stat |= ZYNQ_SDHC_HOST_CMD_COMPLETE;
		if(IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
			k_event_post(&emmc->irq_event, ZYNQ_SDHC_HOST_CMD_COMPLETE);
		}
	}

	if (regs->normal_int_stat & ZYNQ_SDHC_HOST_XFER_COMPLETE) {
		regs->normal_int_stat |= ZYNQ_SDHC_HOST_XFER_COMPLETE;
		if(IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
			k_event_post(&emmc->irq_event, ZYNQ_SDHC_HOST_XFER_COMPLETE);
		}
	}

	if (regs->normal_int_stat & ZYNQ_SDHC_HOST_DMA_INTR) {
		regs->normal_int_stat |= ZYNQ_SDHC_HOST_DMA_INTR;
		if(IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
			k_event_post(&emmc->irq_event, ZYNQ_SDHC_HOST_DMA_INTR);
		}
	}

	if (regs->normal_int_stat & ZYNQ_SDHC_HOST_BUF_WR_READY) {
		regs->normal_int_stat |= ZYNQ_SDHC_HOST_BUF_WR_READY;
		if(IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
			k_event_post(&emmc->irq_event, ZYNQ_SDHC_HOST_BUF_WR_READY);
		}
	}

	if (regs->normal_int_stat & ZYNQ_SDHC_HOST_BUF_RD_READY) {
		regs->normal_int_stat |= ZYNQ_SDHC_HOST_BUF_RD_READY;
		if(IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
			k_event_post(&emmc->irq_event, ZYNQ_SDHC_HOST_BUF_RD_READY);
		}
	}

	if (regs->err_int_stat) {
		LOG_ERR("err int:%x", regs->err_int_stat);
		if(IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
			k_event_post(&emmc->irq_event, ERR_INTR_STATUS_EVENT(regs->err_int_stat));
		}
		if (regs->err_int_stat & ZYNQ_SDHC_HOST_DMA_TXFR_ERR) {
			regs->err_int_stat |= ZYNQ_SDHC_HOST_DMA_TXFR_ERR;
		} else {
			regs->err_int_stat |= regs->err_int_stat;
		}
	}

	if (regs->normal_int_stat) {
		if(IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
			k_event_post(&emmc->irq_event, regs->normal_int_stat);
		}
		regs->normal_int_stat |= regs->normal_int_stat;
	}

	if (regs->adma_err_stat) {
		LOG_ERR("adma err:%x", regs->adma_err_stat);
	}
}

static int zynq_sdhc_init(const struct device *dev)
{
	struct zynq_sdhc_data *data = dev->data;
	volatile struct zynq_sdhc_reg *regs = DEV_REG(dev);
	const struct zynq_sdhc_config *config = dev->config;

	k_sem_init(&data->lock, 1, 1);

	if (IS_ENABLED(CONFIG_XLNX_ZYNQ_SDHC_HOST_INTR)) {
		k_event_init(&data->irq_event);
		config->config_func(dev);
	}
	data->hc_ver = (regs->host_cntrl_version & 0xFFU);
	data->caps = regs->capabilities;

	return zynq_sdhc_reset(dev);
}

static const struct sdhc_driver_api zynq_sdhc_api = {
	.reset = zynq_sdhc_reset,
	.request = zynq_sdhc_request,
	.set_io = zynq_sdhc_set_io,
	.get_card_present = zynq_sdhc_get_card_present,
	.execute_tuning = zynq_sdhc_execute_tuning,
	.card_busy = zynq_sdhc_card_busy,
	.get_host_props = zynq_sdhc_get_host_props,
};

#define ZYNQ_SDHC_INIT(n)                                                                                              \
	static void zynq_sdhc_##n##_irq_config_func(const struct device *dev)                                          \
	{                                                                                                              \
		ARG_UNUSED(dev);                                                                                       \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), zynq_sdhc_isr, DEVICE_DT_INST_GET(n), 0);       \
		irq_enable(DT_INST_IRQN(n));                                                                           \
	}                                                                                                              \
	static const struct zynq_sdhc_config zynq_sdhc_##n##_config = {                                                \
		.base = DT_INST_REG_ADDR(n),                                                                           \
		.config_func = zynq_sdhc_##n##_irq_config_func,                                                        \
		.min_bus_freq = DT_INST_PROP(n, min_bus_freq),                                                         \
		.max_bus_freq = DT_INST_PROP(n, max_bus_freq),                                                         \
		.power_delay_ms = DT_INST_PROP(n, power_delay_ms),                                                     \
		.dw_4bit = DT_INST_ENUM_HAS_VALUE(n, bus_width, 4),                                                    \
		.dw_8bit = DT_INST_ENUM_HAS_VALUE(n, bus_width, 8),                                                    \
		.hs200_mode = DT_INST_PROP(n, mmc_hs200_1_8v),                                                         \
		.hs400_mode = DT_INST_PROP(n, mmc_hs400_1_8v),                                                         \
	};                                                                                                             \
	static struct zynq_sdhc_data zynq_sdhc_##n##_data = {                                                          \
		.card_present = false,                                                                                 \
	};                                                                                                             \
	DEVICE_DT_INST_DEFINE(n, zynq_sdhc_init, NULL, &zynq_sdhc_##n##_data, &zynq_sdhc_##n##_config, POST_KERNEL,    \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &zynq_sdhc_api);

DT_INST_FOREACH_STATUS_OKAY(ZYNQ_SDHC_INIT)
