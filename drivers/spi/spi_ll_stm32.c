/*
 * Copyright (c) 2016 BayLibre, SAS
 * DMA Additions (c) Dave Marples <dave@marples.net>
 * DMA Fixes and rework (c) Vincent van der Locht <vincent@bluecats.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(spi_ll_stm32);

#include <sys/util.h>
#include <kernel.h>
#include <soc.h>
#include <errno.h>
#include <drivers/spi.h>
#include <toolchain.h>

#include <clock_control/stm32_clock_control.h>
#include <drivers/clock_control.h>

#include "spi_ll_stm32.h"

#ifdef CONFIG_SPI_STM32_DMA
#include <dma.h>

/* A couple of configuration sanity checks */
#if defined(CONFIG_SPI_6) && defined(CONFIG_SPI_5)
#error Both SPI_5 and SPI_6 cannot use DMA at the same time!!
#endif

#if defined(CONFIG_SPI_INTERRUPT)
#error Cannot employ both INTERRUPT and DMA mechanisms at the sane time
#endif

/* Pointers to each stream for DMA use */
#define TX_STREAM (0)
#define RX_STREAM (1)
#endif

#define DEV_CFG(dev)						\
(const struct spi_stm32_config * const)(dev->config->config_info)

#define DEV_DATA(dev)					\
(struct spi_stm32_data * const)(dev->driver_data)

/*
 * Check for SPI_SR_FRE to determine support for TI mode frame format
 * error flag, because STM32F1 SoCs do not support it and  STM32CUBE
 * for F1 family defines an unused LL_SPI_SR_FRE.
 */
#ifdef CONFIG_SOC_SERIES_STM32MP1X
#define SPI_STM32_ERR_MSK (LL_SPI_SR_UDR | LL_SPI_SR_CRCE | LL_SPI_SR_MODF | \
			   LL_SPI_SR_OVR | LL_SPI_SR_TIFRE)
#else
#if defined(LL_SPI_SR_UDR)
#define SPI_STM32_ERR_MSK (LL_SPI_SR_UDR | LL_SPI_SR_CRCERR | LL_SPI_SR_MODF | \
			   LL_SPI_SR_OVR | LL_SPI_SR_FRE)
#elif defined(SPI_SR_FRE)
#define SPI_STM32_ERR_MSK (LL_SPI_SR_CRCERR | LL_SPI_SR_MODF | \
			   LL_SPI_SR_OVR | LL_SPI_SR_FRE)
#else
#define SPI_STM32_ERR_MSK (LL_SPI_SR_CRCERR | LL_SPI_SR_MODF | LL_SPI_SR_OVR)
#endif
#endif /* CONFIG_SOC_SERIES_STM32MP1X */

/* Value to shift out when no application data needs transmitting. */
#define SPI_STM32_TX_NOP 0x00


static int spi_stm32_get_err(SPI_TypeDef *spi)
{
	u32_t sr = LL_SPI_ReadReg(spi, SR);

	if (sr & SPI_STM32_ERR_MSK) {
		LOG_ERR("%s: err=%d", __func__,
			    sr & (u32_t)SPI_STM32_ERR_MSK);

		/* OVR error must be explicitly cleared */
		if (LL_SPI_IsActiveFlag_OVR(spi)) {
			LL_SPI_ClearFlag_OVR(spi);
		}

		return -EIO;
	}

	return 0;
}


#ifndef CONFIG_SPI_STM32_DMA
static bool spi_stm32_transfer_ongoing(struct spi_stm32_data *data)
{
	return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}

static inline u16_t spi_stm32_next_tx(struct spi_stm32_data *data)
{
	u16_t tx_frame = SPI_STM32_TX_NOP;

	if (spi_context_tx_buf_on(&data->ctx)) {
		if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
			tx_frame = UNALIGNED_GET((u8_t *)(data->ctx.tx_buf));
		} else {
			tx_frame = UNALIGNED_GET((u16_t *)(data->ctx.tx_buf));
		}
	}

	return tx_frame;
}

/* Shift a SPI frame as master. */
static void spi_stm32_shift_m(SPI_TypeDef *spi, struct spi_stm32_data *data)
{
	u16_t tx_frame;
	u16_t rx_frame;

	tx_frame = spi_stm32_next_tx(data);
	while (!ll_func_tx_is_empty(spi)) {
		/* NOP */
	}

#ifdef CONFIG_SOC_SERIES_STM32MP1X
	/* With the STM32MP1, if the device is the SPI master, we need to enable
	 * the start of the transfer with LL_SPI_StartMasterTransfer(spi)
	 */
	if (LL_SPI_GetMode(spi) == LL_SPI_MODE_MASTER) {
		LL_SPI_StartMasterTransfer(spi);
		while (!LL_SPI_IsActiveMasterTransfer(spi)) {
			/* NOP */
		}
	}
#endif

	if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
		LL_SPI_TransmitData8(spi, tx_frame);
		/* The update is ignored if TX is off. */
		spi_context_update_tx(&data->ctx, 1, 1);
	} else {
		LL_SPI_TransmitData16(spi, tx_frame);
		/* The update is ignored if TX is off. */
		spi_context_update_tx(&data->ctx, 2, 1);
	}

	while (!ll_func_rx_is_not_empty(spi)) {
		/* NOP */
	}

	if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
		rx_frame = LL_SPI_ReceiveData8(spi);
		if (spi_context_rx_buf_on(&data->ctx)) {
			UNALIGNED_PUT(rx_frame, (u8_t *)data->ctx.rx_buf);
		}
		spi_context_update_rx(&data->ctx, 1, 1);
	} else {
		rx_frame = LL_SPI_ReceiveData16(spi);
		if (spi_context_rx_buf_on(&data->ctx)) {
			UNALIGNED_PUT(rx_frame, (u16_t *)data->ctx.rx_buf);
		}
		spi_context_update_rx(&data->ctx, 2, 1);
	}
}

/* Shift a SPI frame as slave. */
static void spi_stm32_shift_s(SPI_TypeDef *spi, struct spi_stm32_data *data)
{
	if (ll_func_tx_is_empty(spi) && spi_context_tx_on(&data->ctx)) {
		u16_t tx_frame = spi_stm32_next_tx(data);

		if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
			LL_SPI_TransmitData8(spi, tx_frame);
			spi_context_update_tx(&data->ctx, 1, 1);
		} else {
			LL_SPI_TransmitData16(spi, tx_frame);
			spi_context_update_tx(&data->ctx, 2, 1);
		}
	} else {
		ll_func_disable_int_tx_empty(spi);
	}

	if (ll_func_rx_is_not_empty(spi) &&
	    spi_context_rx_buf_on(&data->ctx)) {
		u16_t rx_frame;

		if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
			rx_frame = LL_SPI_ReceiveData8(spi);
			UNALIGNED_PUT(rx_frame, (u8_t *)data->ctx.rx_buf);
			spi_context_update_rx(&data->ctx, 1, 1);
		} else {
			rx_frame = LL_SPI_ReceiveData16(spi);
			UNALIGNED_PUT(rx_frame, (u16_t *)data->ctx.rx_buf);
			spi_context_update_rx(&data->ctx, 2, 1);
		}
	}
}

/*
 * Without a FIFO, we can only shift out one frame's worth of SPI
 * data, and read the response back.
 *
 * TODO: support 16-bit data frames.
 */
static int spi_stm32_shift_frames(SPI_TypeDef *spi, struct spi_stm32_data *data)
{
	u16_t operation = data->ctx.config->operation;

	if (SPI_OP_MODE_GET(operation) == SPI_OP_MODE_MASTER) {
		spi_stm32_shift_m(spi, data);
	} else {
		spi_stm32_shift_s(spi, data);
	}

	return spi_stm32_get_err(spi);
}
#endif

static void spi_stm32_complete(struct device *dev, SPI_TypeDef *spi,
			       int status)
{

#if defined(CONFIG_SPI_STM32_DMA)
	ll_func_disable_int_errors(spi);
#elif defined(CONFIG_SPI_STM32_INTERRUPT)
	ll_func_disable_int_tx_empty(spi);
	ll_func_disable_int_rx_not_empty(spi);
	ll_func_disable_int_errors(spi);
#endif

	struct spi_stm32_data *data = dev->driver_data;
	const struct spi_stm32_config *cfg = dev->config->config_info;

#if (!defined(CONFIG_SPI_STM32_DMA)) && defined(CONFIG_SPI_STM32_HAS_FIFO)
	/* Flush RX buffer */
	while (ll_func_rx_is_not_empty(spi)) {
		(void) LL_SPI_ReceiveData8(spi);
	}
#endif

	if (LL_SPI_GetMode(spi) == LL_SPI_MODE_MASTER) {
		while (ll_func_spi_is_busy(spi)) {
			/* NOP */
		}
	}

#if defined(CONFIG_SPI_STM32_DMA)
	// TODO: make compatible with CONFIG_SOC_SERIES_STM32MP1X
	LL_SPI_DisableDMAReq_RX(spi);
	LL_SPI_DisableDMAReq_TX(spi);

	dma_stop(data->d, cfg->stream[TX_STREAM]);
	dma_stop(data->d, cfg->stream[RX_STREAM]);
#endif

	spi_context_cs_control(&data->ctx, false);
	ll_func_disable_spi(spi);

#if defined(CONFIG_SPI_STM32_INTERRUPT) || defined(CONFIG_SPI_STM32_DMA)
	spi_context_complete(&data->ctx, status);
#endif
}

#ifdef CONFIG_SPI_STM32_DMA
void spi_stm32_dma_callback(void *arg, u32_t channel, int error_code)

{
	/* Function callback at end of DMA transfers */
	struct device *const dev = (struct device *) arg;
	const struct spi_stm32_config *cfg = dev->config->config_info;
	struct spi_stm32_data *data = dev->driver_data;
	SPI_TypeDef *spi = cfg->spi;

#if 0
	__ASSERT_NO_MSG(data->nDMA);

	if (!--data->nDMA) {
		spi_stm32_complete(dev, spi, error_code);
	}
#elif 1

	int ret;

	if (!--data->nDMA) {

		// LL_SPI_DisableDMAReq_TX(spi);
		// LL_SPI_DisableDMAReq_RX(spi);

		spi_context_update_tx(&data->ctx, 1, data->ctx.current_tx->len);
		spi_context_update_rx(&data->ctx, 1, data->ctx.current_rx->len);

		if (!(spi_context_tx_on(&data->ctx)) && !(spi_context_rx_on(&data->ctx))) {
			spi_stm32_complete(dev, spi, error_code);
			return;
		}

		if (spi_context_rx_on(&data->ctx)) {
			data->b[RX_STREAM].dest_address = (u32_t)data->ctx.rx_buf;
			data->b[RX_STREAM].block_size = data->ctx.rx_len;
			data->nDMA++;
			
#if (defined(CONFIG_SPI_STM32_DMA) && defined(CONFIG_SPI_5) && defined(CONFIG_SPI_6))
			if(cfg->spi == DT_SPI_5_BASE_ADDRESS) {
				ret = dma_config(data->d, cfg->stream[RX_STREAM],
						&data->dma_conf[RX_STREAM]);
			}
			else {
				ret = dma_reload(data->d, cfg->stream[RX_STREAM], data->b[RX_STREAM].source_address,
						data->b[RX_STREAM].dest_address, data->b[RX_STREAM].block_size);
			}
#else
			ret = dma_reload(data->d, cfg->stream[RX_STREAM], data->b[RX_STREAM].source_address,
					data->b[RX_STREAM].dest_address, data->b[RX_STREAM].block_size);
#endif

			if (ret) {
				LOG_ERR("Failed to configure RX_STREAM");
				return;
			}

			// LL_SPI_EnableDMAReq_RX(spi);
			ret = dma_start(data->d, cfg->stream[RX_STREAM]);
			if (ret) {
				LOG_ERR("Failed to start RX_STREAM");
				return;
			}
		}

		if (spi_context_tx_on(&data->ctx)) {
			data->b[TX_STREAM].source_address = (u32_t)data->ctx.tx_buf;
			data->b[TX_STREAM].block_size = data->ctx.tx_len;
			data->nDMA++;

#if (defined(CONFIG_SPI_STM32_DMA) && defined(CONFIG_SPI_5) && defined(CONFIG_SPI_6))
			if(cfg->spi == DT_SPI_6_BASE_ADDRESS) {
				ret = dma_config(data->d, cfg->stream[TX_STREAM],
						&data->dma_conf[TX_STREAM]);
			}
			else {
				ret = dma_reload(data->d, cfg->stream[TX_STREAM], data->b[TX_STREAM].source_address,
						data->b[TX_STREAM].dest_address, data->b[TX_STREAM].block_size);
			}
#else
			ret = dma_reload(data->d, cfg->stream[TX_STREAM], data->b[TX_STREAM].source_address,
					data->b[TX_STREAM].dest_address, data->b[TX_STREAM].block_size);
#endif

			if (ret) {
				LOG_ERR("Failed to configure TX_STREAM");
				return;
			}

			// LL_SPI_EnableDMAReq_TX(spi);
			ret = dma_start(data->d, cfg->stream[TX_STREAM]);
			if (ret) {
				LOG_ERR("Failed to start TX_STREAM");
				return;
			}
		}

	}

#else

	int ret;

	printk(" i: %u", data->nDMA);

	if(channel == cfg->stream[RX_STREAM]) {
		printk(" r");
		spi_context_update_rx(&data->ctx, 1, data->ctx.current_rx->len);

		if (spi_context_rx_on(&data->ctx)) {
			data->b[RX_STREAM].dest_address = (u32_t)data->ctx.rx_buf;
			data->b[RX_STREAM].block_size = data->ctx.rx_len;
			data->nDMA++;

			ret = dma_config(data->d, cfg->stream[RX_STREAM],
					&data->dma_conf[RX_STREAM]);
			if (ret) {
				LOG_ERR("Failed to configure RX_STREAM");
				printk(" $");
				return;
			}

			// LL_SPI_EnableDMAReq_RX(spi);
			ret = dma_start(data->d, cfg->stream[RX_STREAM]);
			if (ret) {
				LOG_ERR("Failed to start RX_STREAM");
				printk(" ^");
				return;
			}

		}

	}

	if(channel == cfg->stream[TX_STREAM]) {
		printk(" t");
		spi_context_update_tx(&data->ctx, 1, data->ctx.current_tx->len);

		if (spi_context_tx_on(&data->ctx)) {
			data->b[TX_STREAM].source_address = (u32_t)data->ctx.tx_buf;
			data->b[TX_STREAM].block_size = data->ctx.tx_len;
			data->nDMA++;

			ret = dma_config(data->d, cfg->stream[TX_STREAM],
						&data->dma_conf[TX_STREAM]);
			if (ret) {
				LOG_ERR("Failed to configure TX_STREAM");
				printk(" &");
				return;
			}

			// LL_SPI_EnableDMAReq_TX(spi);
			ret = dma_start(data->d, cfg->stream[TX_STREAM]);
			if (ret) {
				LOG_ERR("Failed to start TX_STREAM");
				printk(" #");
				return;
			}
		}
	}

	if (!--data->nDMA) {

		if (!(spi_context_tx_on(&data->ctx)) && !(spi_context_rx_on(&data->ctx))) {
			spi_stm32_complete(dev, spi, error_code);
			return;
		}

	}

	printk(" o: %u\n", data->nDMA);

#endif
}

static void spi_stm32_isr(void *arg)
{
	struct device * const dev = (struct device *) arg;
	const struct spi_stm32_config *cfg = dev->config->config_info;
	//struct spi_stm32_data *data = dev->driver_data;
	SPI_TypeDef *spi = cfg->spi;
	int err;

	err = spi_stm32_get_err(spi);
	if (err) {
		printk("SPI ERROR OCCURRED - %i\n", err);

		//spi_stm32_complete(dev, spi, err);
		return;
	}

}



#elif defined( CONFIG_SPI_STM32_INTERRUPT)
static void spi_stm32_isr(void *arg)
{
	struct device * const dev = (struct device *) arg;
	const struct spi_stm32_config *cfg = dev->config->config_info;
	struct spi_stm32_data *data = dev->driver_data;
	SPI_TypeDef *spi = cfg->spi;
	int err;

	err = spi_stm32_get_err(spi);
	if (err) {
		spi_stm32_complete(dev, spi, err);
		return;
	}

	if (spi_stm32_transfer_ongoing(data)) {
		err = spi_stm32_shift_frames(spi, data);
	}

	if (err || !spi_stm32_transfer_ongoing(data)) {
		spi_stm32_complete(dev, spi, err);
	}
}
#endif

static int spi_stm32_configure(struct device *dev,
			       const struct spi_config *config)
{
	const struct spi_stm32_config *cfg = DEV_CFG(dev);
	struct spi_stm32_data *data = DEV_DATA(dev);
	const u32_t scaler[] = {
		LL_SPI_BAUDRATEPRESCALER_DIV2,
		LL_SPI_BAUDRATEPRESCALER_DIV4,
		LL_SPI_BAUDRATEPRESCALER_DIV8,
		LL_SPI_BAUDRATEPRESCALER_DIV16,
		LL_SPI_BAUDRATEPRESCALER_DIV32,
		LL_SPI_BAUDRATEPRESCALER_DIV64,
		LL_SPI_BAUDRATEPRESCALER_DIV128,
		LL_SPI_BAUDRATEPRESCALER_DIV256
	};
	SPI_TypeDef *spi = cfg->spi;
	u32_t clock;
	int br;

	if (spi_context_configured(&data->ctx, config)) {
		/* Nothing to do */
		return 0;
	}

	if ((SPI_WORD_SIZE_GET(config->operation) != 8)
	    && (SPI_WORD_SIZE_GET(config->operation) != 16)) {
		return -ENOTSUP;
	}

	clock_control_get_rate(device_get_binding(STM32_CLOCK_CONTROL_NAME),
			       (clock_control_subsys_t) &cfg->pclken, &clock);

	for (br = 1 ; br <= ARRAY_SIZE(scaler) ; ++br) {
		u32_t clk = clock >> br;

		if (clk <= config->frequency) {
			break;
		}
	}

	if (br > ARRAY_SIZE(scaler)) {
		LOG_ERR("Unsupported frequency %uHz, max %uHz, min %uHz",
			    config->frequency,
			    clock >> 1,
			    clock >> ARRAY_SIZE(scaler));
		return -EINVAL;
	}

	LL_SPI_Disable(spi);
	LL_SPI_SetBaudRatePrescaler(spi, scaler[br - 1]);

	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) {
		LL_SPI_SetClockPolarity(spi, LL_SPI_POLARITY_HIGH);
	} else {
		LL_SPI_SetClockPolarity(spi, LL_SPI_POLARITY_LOW);
	}

	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) {
		LL_SPI_SetClockPhase(spi, LL_SPI_PHASE_2EDGE);
	} else {
		LL_SPI_SetClockPhase(spi, LL_SPI_PHASE_1EDGE);
	}

	LL_SPI_SetTransferDirection(spi, LL_SPI_FULL_DUPLEX);

	if (config->operation & SPI_TRANSFER_LSB) {
		LL_SPI_SetTransferBitOrder(spi, LL_SPI_LSB_FIRST);
	} else {
		LL_SPI_SetTransferBitOrder(spi, LL_SPI_MSB_FIRST);
	}

	LL_SPI_DisableCRC(spi);

	if (config->cs || !IS_ENABLED(CONFIG_SPI_STM32_USE_HW_SS)) {
		LL_SPI_SetNSSMode(spi, LL_SPI_NSS_SOFT);
	} else {
		if (config->operation & SPI_OP_MODE_SLAVE) {
			LL_SPI_SetNSSMode(spi, LL_SPI_NSS_HARD_INPUT);
		} else {
			LL_SPI_SetNSSMode(spi, LL_SPI_NSS_HARD_OUTPUT);
		}
	}

	if (config->operation & SPI_OP_MODE_SLAVE) {
		LL_SPI_SetMode(spi, LL_SPI_MODE_SLAVE);
	} else {
		LL_SPI_SetMode(spi, LL_SPI_MODE_MASTER);
	}

	if (SPI_WORD_SIZE_GET(config->operation) ==  8) {
		LL_SPI_SetDataWidth(spi, LL_SPI_DATAWIDTH_8BIT);
	} else {
		LL_SPI_SetDataWidth(spi, LL_SPI_DATAWIDTH_16BIT);
	}

#if defined(CONFIG_SPI_STM32_HAS_FIFO)
	ll_func_set_fifo_threshold_8bit(spi);
#endif

#ifndef CONFIG_SOC_SERIES_STM32F1X
	LL_SPI_SetStandard(spi, LL_SPI_PROTOCOL_MOTOROLA);
#endif

	/* At this point, it's mandatory to set this on the context! */
	data->ctx.config = config;

	spi_context_cs_configure(&data->ctx);

	dma_config(data->d, cfg->stream[RX_STREAM],
						&data->dma_conf[RX_STREAM]);
	dma_config(data->d, cfg->stream[TX_STREAM],
						&data->dma_conf[TX_STREAM]);

	LOG_DBG("Installed config %p: freq %uHz (div = %u),"
		    " mode %u/%u/%u, slave %u",
		    config, clock >> br, 1 << br,
		    (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) ? 1 : 0,
		    (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) ? 1 : 0,
		    (SPI_MODE_GET(config->operation) & SPI_MODE_LOOP) ? 1 : 0,
		    config->slave);

	return 0;
}

static int spi_stm32_release(struct device *dev,
			     const struct spi_config *config)
{
	struct spi_stm32_data *data = DEV_DATA(dev);

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static int transceive(struct device *dev,
		      const struct spi_config *config,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous, struct spi_async_event *signal)
{
	const struct spi_stm32_config *cfg = DEV_CFG(dev);
	struct spi_stm32_data *data = DEV_DATA(dev);
	SPI_TypeDef *spi = cfg->spi;
	int ret;

	if (!tx_bufs && !rx_bufs) {
		return 0;
	}

#if !(defined(CONFIG_SPI_STM32_INTERRUPT) || defined(CONFIG_SPI_STM32_DMA))
	if (asynchronous) {
		return -ENOTSUP;
	}
#endif

	spi_context_lock(&data->ctx, asynchronous, signal);

	ret = spi_stm32_configure(dev, config);
	if (ret) {
		return ret;
	}

	/* Set buffers info */
	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);

#if defined(CONFIG_SPI_STM32_HAS_FIFO)
	/* Flush RX buffer */
	while (ll_func_rx_is_not_empty(spi)) {
		(void) LL_SPI_ReceiveData8(spi);
	}
#endif

	spi_context_cs_control(&data->ctx, true);

#ifdef CONFIG_SPI_STM32_DMA
	/* Perform DMA based transfer activity */
	data->nDMA = 0;

	//printk("\n\n - ");

	if (spi_context_rx_on(&data->ctx)) {
		data->b[RX_STREAM].dest_address = (u32_t)data->ctx.rx_buf;
		data->b[RX_STREAM].block_size = data->ctx.rx_len;
		data->nDMA++;

		//printk("RX ");
#if (defined(CONFIG_SPI_STM32_DMA) && defined(CONFIG_SPI_5) && defined(CONFIG_SPI_6))
		if(cfg->spi == DT_SPI_5_BASE_ADDRESS) {
			ret = dma_config(data->d, cfg->stream[RX_STREAM],
					&data->dma_conf[RX_STREAM]);
		}
		else {
			ret = dma_reload(data->d, cfg->stream[RX_STREAM], data->b[RX_STREAM].source_address,
					data->b[RX_STREAM].dest_address, data->b[RX_STREAM].block_size);
		}
#else
		ret = dma_reload(data->d, cfg->stream[RX_STREAM], data->b[RX_STREAM].source_address,
					data->b[RX_STREAM].dest_address, data->b[RX_STREAM].block_size);
#endif
		if (ret) {
			LOG_ERR("Failed to configure RX_STREAM");
			spi_context_cs_control(&data->ctx, false);
			spi_context_release(&data->ctx, ret);
			return ret;
		}

		// LL_SPI_EnableDMAReq_RX(spi);
		dma_start(data->d, cfg->stream[RX_STREAM]);
	}

	if (spi_context_tx_on(&data->ctx)) {
		data->b[TX_STREAM].source_address = (u32_t)data->ctx.tx_buf;
		data->b[TX_STREAM].block_size = data->ctx.tx_len;
		data->nDMA++;

		//printk("TX ");

#if (defined(CONFIG_SPI_STM32_DMA) && defined(CONFIG_SPI_5) && defined(CONFIG_SPI_6))
		if(cfg->spi == DT_SPI_6_BASE_ADDRESS) {
			ret = dma_config(data->d, cfg->stream[TX_STREAM],
					&data->dma_conf[TX_STREAM]);
		}
		else {
			ret = dma_reload(data->d, cfg->stream[TX_STREAM], data->b[TX_STREAM].source_address,
					data->b[TX_STREAM].dest_address, data->b[TX_STREAM].block_size);
		}
#else
		ret = dma_reload(data->d, cfg->stream[TX_STREAM], data->b[TX_STREAM].source_address,
				data->b[TX_STREAM].dest_address, data->b[TX_STREAM].block_size);
#endif
		
		if (ret) {
			LOG_ERR("Failed to configure TX_STREAM");
			spi_context_cs_control(&data->ctx, false);
			spi_context_release(&data->ctx, ret);
			return ret;
		}

		// LL_SPI_EnableDMAReq_TX(spi);
		dma_start(data->d, cfg->stream[TX_STREAM]);
	}

	//printk("\n\nstart: %u", data->nDMA);

	/* Everything is now set, spin the plates, assuming */
	/* we're not waiting for a trigger later */
	if (!SPI_DEFER_GET(config->operation)) {
		if (spi_context_rx_on(&data->ctx)) {
			LL_SPI_EnableDMAReq_RX(spi);
		}
		if (spi_context_tx_on(&data->ctx)) {
			LL_SPI_EnableDMAReq_TX(spi);
		}
	LL_SPI_Enable(spi);
	} else   {
		data->armed = true;
	}

	/* Now just wait for it to complete, or an error */
	ret = spi_context_wait_for_completion(&data->ctx);

#elif defined(CONFIG_SPI_STM32_INTERRUPT)

	LL_SPI_Enable(spi);

	/* Else perform Interrupt based transfer activity */
	ll_func_enable_int_errors(spi);

	if (rx_bufs) {
		ll_func_enable_int_rx_not_empty(spi);
	}

	ll_func_enable_int_tx_empty(spi);

	ret = spi_context_wait_for_completion(&data->ctx);
#else
	LL_SPI_Enable(spi);
	do {
		ret = spi_stm32_shift_frames(spi, data);
	} while (!ret && spi_stm32_transfer_ongoing(data));

	spi_stm32_complete(dev, spi, ret);

#ifdef CONFIG_SPI_SLAVE
	if (spi_context_is_slave(&data->ctx) && !ret) {
		ret = data->ctx.recv_frames;
	}
#endif /* CONFIG_SPI_SLAVE */

#endif

	spi_context_release(&data->ctx, ret);

	return ret;
}

static int spi_stm32_transceive(struct device *dev,
				const struct spi_config *config,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	return transceive(dev, config, tx_bufs, rx_bufs, false, NULL);
}

static int spi_stm32_trigger(struct device *dev,
			     const struct spi_config *config)

{
	const struct spi_stm32_config *cfg = DEV_CFG(dev);
	struct spi_stm32_data *data = DEV_DATA(dev);
	SPI_TypeDef *spi = cfg->spi;

	if ((!SPI_DEFER_GET(config->operation)) || (!data->armed)) {
		return -EBUSY;
	}

	/* This is turned off in spi_stm32_complete(). */
	spi_context_cs_control(&data->ctx, true);

	data->armed = false;
	LL_SPI_Enable(spi);
	return 0;
}

#ifdef CONFIG_SPI_ASYNC
static int spi_stm32_transceive_async(struct device *dev,
				      const struct spi_config *config,
				      const struct spi_buf_set *tx_bufs,
				      const struct spi_buf_set *rx_bufs,
				      struct spi_async_event *async)
{
	return transceive(dev, config, tx_bufs, rx_bufs, true, async);
}
#endif /* CONFIG_SPI_ASYNC */

static const struct spi_driver_api api_funcs = {
	.transceive = spi_stm32_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_stm32_transceive_async,
#endif
	.release = spi_stm32_release,
	.trigger = spi_stm32_trigger
};

static int spi_stm32_init(struct device *dev)
{
	struct spi_stm32_data *data = dev->driver_data;
	const struct spi_stm32_config *cfg = dev->config->config_info;

	__ASSERT_NO_MSG(device_get_binding(STM32_CLOCK_CONTROL_NAME));

#ifdef CONFIG_SPI_STM32_DMA
	data->dma_conf[RX_STREAM].callback_arg = dev;
	data->dma_conf[TX_STREAM].callback_arg = dev;
	data->d = device_get_binding(cfg->dmadev);
#endif

	if (clock_control_on(device_get_binding(STM32_CLOCK_CONTROL_NAME),
			       (clock_control_subsys_t) &cfg->pclken) != 0) {
		LOG_ERR("Could not enable SPI clock");
		return -EIO;
	}

#if defined(CONFIG_SPI_STM32_INTERRUPT) || defined(CONFIG_SPI_STM32_DMA)
	cfg->irq_config(dev);
#endif

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

#ifdef CONFIG_SPI_1

#if defined(CONFIG_SPI_STM32_INTERRUPT) || defined(CONFIG_SPI_STM32_DMA)
static void spi_stm32_irq_config_func_1(struct device *port);
#endif

static const struct spi_stm32_config spi_stm32_cfg_1 = {
	.spi = (SPI_TypeDef *) DT_SPI_1_BASE_ADDRESS,
	.pclken = {
		.enr = DT_SPI_1_CLOCK_BITS,
		.bus = DT_SPI_1_CLOCK_BUS
	},
#if defined(CONFIG_SPI_STM32_INTERRUPT) || defined(CONFIG_SPI_STM32_DMA)
	.irq_config = spi_stm32_irq_config_func_1,
#endif

#ifdef CONFIG_SPI_STM32_DMA
	.stream[TX_STREAM] = 3,
	.stream[RX_STREAM] = 2,
	.dmadev = CONFIG_DMA_2_NAME,
#endif
};

static struct spi_stm32_data spi_stm32_dev_data_1 = {
	SPI_CONTEXT_INIT_LOCK(spi_stm32_dev_data_1, ctx),
	SPI_CONTEXT_INIT_SYNC(spi_stm32_dev_data_1, ctx),

#ifdef CONFIG_SPI_STM32_DMA
	/* Transmit block config */
	.b[TX_STREAM].dest_address = (u32_t)&(SPI1->DR),
	.b[TX_STREAM].source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
	.b[TX_STREAM].dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,

	/* Receive block config */
	.b[RX_STREAM].source_address = (u32_t)&(SPI1->DR),
	.b[RX_STREAM].source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
	.b[RX_STREAM].dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,

	/* Setup for transmit stream */
	.dma_conf[TX_STREAM].dma_slot = 3,
	.dma_conf[TX_STREAM].channel_direction = MEMORY_TO_PERIPHERAL,
	.dma_conf[TX_STREAM].source_data_size = 0,      /* 8 bit data */
	.dma_conf[TX_STREAM].dest_data_size = 0,        /* 8 bit data */
	.dma_conf[TX_STREAM].source_burst_length = 0,
	.dma_conf[TX_STREAM].dest_burst_length = 0,
	.dma_conf[TX_STREAM].dma_callback = spi_stm32_dma_callback,
	.dma_conf[TX_STREAM].complete_callback_en = 1,
	.dma_conf[TX_STREAM].error_callback_en = 1,
	.dma_conf[TX_STREAM].head_block = &spi_stm32_dev_data_1.b[TX_STREAM],

	/* Setup for receive stream */
	.dma_conf[RX_STREAM].dma_slot = 3,
	.dma_conf[RX_STREAM].channel_direction = PERIPHERAL_TO_MEMORY,
	.dma_conf[RX_STREAM].source_data_size = 0,      /* 8 bit data */
	.dma_conf[RX_STREAM].dest_data_size = 0,        /* 8 bit data */
	.dma_conf[RX_STREAM].source_burst_length = 0,
	.dma_conf[RX_STREAM].dest_burst_length = 0,
	.dma_conf[RX_STREAM].dma_callback = spi_stm32_dma_callback,
	.dma_conf[RX_STREAM].complete_callback_en = 1,
	.dma_conf[RX_STREAM].error_callback_en = 1,
	.dma_conf[RX_STREAM].head_block = &spi_stm32_dev_data_1.b[RX_STREAM],
#endif
};

DEVICE_AND_API_INIT(spi_stm32_1, DT_SPI_1_NAME, &spi_stm32_init,
		    &spi_stm32_dev_data_1, &spi_stm32_cfg_1,
		    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,
		    &api_funcs);

#if defined(CONFIG_SPI_STM32_INTERRUPT) || defined(CONFIG_SPI_STM32_DMA)
static void spi_stm32_irq_config_func_1(struct device *dev)
{
	IRQ_CONNECT(DT_SPI_1_IRQ, DT_SPI_1_IRQ_PRI,
		    spi_stm32_isr, DEVICE_GET(spi_stm32_1), 0);
	irq_enable(DT_SPI_1_IRQ);
}
#endif

#endif /* CONFIG_SPI_1 */

#ifdef CONFIG_SPI_2

#if defined(CONFIG_SPI_STM32_INTERRUPT) || defined(CONFIG_SPI_STM32_DMA)
static void spi_stm32_irq_config_func_2(struct device *port);
#endif

static const struct spi_stm32_config spi_stm32_cfg_2 = {
	.spi = (SPI_TypeDef *) DT_SPI_2_BASE_ADDRESS,
	.pclken = {
		.enr = DT_SPI_2_CLOCK_BITS,
		.bus = DT_SPI_2_CLOCK_BUS
	},
#if defined(CONFIG_SPI_STM32_INTERRUPT) || defined(CONFIG_SPI_STM32_DMA)
	.irq_config = spi_stm32_irq_config_func_2,
#endif

#ifdef CONFIG_SPI_STM32_DMA
	.stream[TX_STREAM] = 4,
	.stream[RX_STREAM] = 3,
	.dmadev = CONFIG_DMA_1_NAME,
#endif
};

static struct spi_stm32_data spi_stm32_dev_data_2 = {
	SPI_CONTEXT_INIT_LOCK(spi_stm32_dev_data_2, ctx),
	SPI_CONTEXT_INIT_SYNC(spi_stm32_dev_data_2, ctx),

#ifdef CONFIG_SPI_STM32_DMA
	/* Transmit block config */
	.b[TX_STREAM].dest_address = (u32_t)&(SPI2->DR),
	.b[TX_STREAM].source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
	.b[TX_STREAM].dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,

	/* Receive block config */
	.b[RX_STREAM].source_address = (u32_t)&(SPI2->DR),
	.b[RX_STREAM].source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
	.b[RX_STREAM].dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,

	/* Setup for transmit stream */
	.dma_conf[TX_STREAM].dma_slot = 0,
	.dma_conf[TX_STREAM].channel_direction = MEMORY_TO_PERIPHERAL,
	.dma_conf[TX_STREAM].source_data_size = 0,      /* 8 bit data */
	.dma_conf[TX_STREAM].dest_data_size = 0,        /* 8 bit data */
	.dma_conf[TX_STREAM].source_burst_length = 0,
	.dma_conf[TX_STREAM].dest_burst_length = 0,
	.dma_conf[TX_STREAM].dma_callback = spi_stm32_dma_callback,
	.dma_conf[TX_STREAM].complete_callback_en = 1,
	.dma_conf[TX_STREAM].error_callback_en = 1,
	.dma_conf[TX_STREAM].head_block = &spi_stm32_dev_data_2.b[TX_STREAM],

	/* Setup for receive stream */
	.dma_conf[RX_STREAM].dma_slot = 0,
	.dma_conf[RX_STREAM].channel_direction = PERIPHERAL_TO_MEMORY,
	.dma_conf[RX_STREAM].source_data_size = 0,      /* 8 bit data */
	.dma_conf[RX_STREAM].dest_data_size = 0,        /* 8 bit data */
	.dma_conf[RX_STREAM].source_burst_length = 0,
	.dma_conf[RX_STREAM].dest_burst_length = 0,
	.dma_conf[RX_STREAM].dma_callback = spi_stm32_dma_callback,
	.dma_conf[TX_STREAM].complete_callback_en = 1,
	.dma_conf[TX_STREAM].error_callback_en = 1,
	.dma_conf[RX_STREAM].head_block = &spi_stm32_dev_data_2.b[RX_STREAM],
#endif
};

DEVICE_AND_API_INIT(spi_stm32_2, DT_SPI_2_NAME, &spi_stm32_init,
		    &spi_stm32_dev_data_2, &spi_stm32_cfg_2,
		    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,
		    &api_funcs);

#if defined(CONFIG_SPI_STM32_INTERRUPT) || defined(CONFIG_SPI_STM32_DMA)
static void spi_stm32_irq_config_func_2(struct device *dev)
{
	IRQ_CONNECT(DT_SPI_2_IRQ, DT_SPI_2_IRQ_PRI,
		    spi_stm32_isr, DEVICE_GET(spi_stm32_2), 0);
	irq_enable(DT_SPI_2_IRQ);
}
#endif

#endif /* CONFIG_SPI_2 */

#ifdef CONFIG_SPI_3

#if defined(CONFIG_SPI_STM32_INTERRUPT) || defined(CONFIG_SPI_STM32_DMA)
static void spi_stm32_irq_config_func_3(struct device *port);
#endif

static const  struct spi_stm32_config spi_stm32_cfg_3 = {
	.spi = (SPI_TypeDef *) DT_SPI_3_BASE_ADDRESS,
	.pclken = {
		.enr = DT_SPI_3_CLOCK_BITS,
		.bus = DT_SPI_3_CLOCK_BUS
	},
#if defined(CONFIG_SPI_STM32_INTERRUPT) || defined(CONFIG_SPI_STM32_DMA)
	.irq_config = spi_stm32_irq_config_func_3,
#endif

#ifdef CONFIG_SPI_STM32_DMA
	.stream[TX_STREAM] = 5,
	.stream[RX_STREAM] = 0,
	.dmadev = CONFIG_DMA_1_NAME,
#endif
};

static struct spi_stm32_data spi_stm32_dev_data_3 = {
	SPI_CONTEXT_INIT_LOCK(spi_stm32_dev_data_3, ctx),
	SPI_CONTEXT_INIT_SYNC(spi_stm32_dev_data_3, ctx),
#ifdef CONFIG_SPI_STM32_DMA
	/* Transmit block config */
	.b[TX_STREAM].dest_address = (u32_t)&(SPI3->DR),
	.b[TX_STREAM].source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
	.b[TX_STREAM].dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,

	/* Receive block config */
	.b[RX_STREAM].source_address = (u32_t)&(SPI3->DR),
	.b[RX_STREAM].source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
	.b[RX_STREAM].dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,

	/* Setup for transmit stream */
	.dma_conf[TX_STREAM].dma_slot = 0,
	.dma_conf[TX_STREAM].channel_direction = MEMORY_TO_PERIPHERAL,
	.dma_conf[TX_STREAM].source_data_size = 0,      /* 8 bit data */
	.dma_conf[TX_STREAM].dest_data_size = 0,        /* 8 bit data */
	.dma_conf[TX_STREAM].source_burst_length = 0,
	.dma_conf[TX_STREAM].dest_burst_length = 0,
	.dma_conf[TX_STREAM].dma_callback = spi_stm32_dma_callback,
	.dma_conf[TX_STREAM].complete_callback_en = 1,
	.dma_conf[TX_STREAM].error_callback_en = 1,
	.dma_conf[TX_STREAM].head_block = &spi_stm32_dev_data_3.b[TX_STREAM],

	/* Setup for receive stream */
	.dma_conf[RX_STREAM].dma_slot = 0,
	.dma_conf[RX_STREAM].channel_direction = PERIPHERAL_TO_MEMORY,
	.dma_conf[RX_STREAM].source_data_size = 0,      /* 8 bit data */
	.dma_conf[RX_STREAM].dest_data_size = 0,        /* 8 bit data */
	.dma_conf[RX_STREAM].source_burst_length = 0,
	.dma_conf[RX_STREAM].dest_burst_length = 0,
	.dma_conf[RX_STREAM].dma_callback = spi_stm32_dma_callback,
	.dma_conf[TX_STREAM].complete_callback_en = 1,
	.dma_conf[TX_STREAM].error_callback_en = 1,
	.dma_conf[RX_STREAM].head_block = &spi_stm32_dev_data_3.b[RX_STREAM],
#endif
};

DEVICE_AND_API_INIT(spi_stm32_3, DT_SPI_3_NAME, &spi_stm32_init,
		    &spi_stm32_dev_data_3, &spi_stm32_cfg_3,
		    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,
		    &api_funcs);

#if defined(CONFIG_SPI_STM32_INTERRUPT) || defined(CONFIG_SPI_STM32_DMA)
static void spi_stm32_irq_config_func_3(struct device *dev)
{
	IRQ_CONNECT(DT_SPI_3_IRQ, DT_SPI_3_IRQ_PRI,
		    spi_stm32_isr, DEVICE_GET(spi_stm32_3), 0);
	irq_enable(DT_SPI_3_IRQ);
}
#endif

#endif /* CONFIG_SPI_3 */

#ifdef CONFIG_SPI_4

#if defined(CONFIG_SPI_STM32_INTERRUPT) || defined(CONFIG_SPI_STM32_DMA)
static void spi_stm32_irq_config_func_4(struct device *port);
#endif

static const  struct spi_stm32_config spi_stm32_cfg_4 = {
	.spi = (SPI_TypeDef *) DT_SPI_4_BASE_ADDRESS,
	.pclken = {
		.enr = DT_SPI_4_CLOCK_BITS,
		.bus = DT_SPI_4_CLOCK_BUS
	},
#if defined(CONFIG_SPI_STM32_INTERRUPT) || defined(CONFIG_SPI_STM32_DMA)
	.irq_config = spi_stm32_irq_config_func_4,
#endif

#ifdef CONFIG_SPI_STM32_DMA
	.stream[TX_STREAM] = 1,
	.stream[RX_STREAM] = 0,
	.dmadev = CONFIG_DMA_2_NAME,
#endif
};

static struct spi_stm32_data spi_stm32_dev_data_4 = {
	SPI_CONTEXT_INIT_LOCK(spi_stm32_dev_data_4, ctx),
	SPI_CONTEXT_INIT_SYNC(spi_stm32_dev_data_4, ctx),
#ifdef CONFIG_SPI_STM32_DMA
	/* Transmit block config */
	.b[TX_STREAM].dest_address = (u32_t)&(SPI4->DR),
	.b[TX_STREAM].source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
	.b[TX_STREAM].dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,

	/* Receive block config */
	.b[RX_STREAM].source_address = (u32_t)&(SPI4->DR),
	.b[RX_STREAM].source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
	.b[RX_STREAM].dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,

	/* Setup for transmit stream */
	.dma_conf[TX_STREAM].dma_slot = 4,
	.dma_conf[TX_STREAM].channel_direction = MEMORY_TO_PERIPHERAL,
	.dma_conf[TX_STREAM].source_data_size = 0,      /* 8 bit data */
	.dma_conf[TX_STREAM].dest_data_size = 0,        /* 8 bit data */
	.dma_conf[TX_STREAM].source_burst_length = 0,
	.dma_conf[TX_STREAM].dest_burst_length = 0,
	.dma_conf[TX_STREAM].dma_callback = spi_stm32_dma_callback,
	.dma_conf[TX_STREAM].complete_callback_en = 1,
	.dma_conf[TX_STREAM].error_callback_en = 1,
	.dma_conf[TX_STREAM].head_block = &spi_stm32_dev_data_4.b[TX_STREAM],

	/* Setup for receive stream */
	.dma_conf[RX_STREAM].dma_slot = 4,
	.dma_conf[RX_STREAM].channel_direction = PERIPHERAL_TO_MEMORY,
	.dma_conf[RX_STREAM].source_data_size = 0,      /* 8 bit data */
	.dma_conf[RX_STREAM].dest_data_size = 0,        /* 8 bit data */
	.dma_conf[RX_STREAM].source_burst_length = 0,
	.dma_conf[RX_STREAM].dest_burst_length = 0,
	.dma_conf[RX_STREAM].dma_callback = spi_stm32_dma_callback,
	.dma_conf[TX_STREAM].complete_callback_en = 1,
	.dma_conf[TX_STREAM].error_callback_en = 1,
	.dma_conf[RX_STREAM].head_block = &spi_stm32_dev_data_4.b[RX_STREAM],
#endif
};

DEVICE_AND_API_INIT(spi_stm32_4, DT_SPI_4_NAME, &spi_stm32_init,
		    &spi_stm32_dev_data_4, &spi_stm32_cfg_4,
		    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,
		    &api_funcs);

#if defined(CONFIG_SPI_STM32_INTERRUPT) || defined(CONFIG_SPI_STM32_DMA)
static void spi_stm32_irq_config_func_4(struct device *dev)
{
	IRQ_CONNECT(DT_SPI_4_IRQ, DT_SPI_4_IRQ_PRI,
		    spi_stm32_isr, DEVICE_GET(spi_stm32_4), 0);
	irq_enable(DT_SPI_4_IRQ);
}
#endif

#endif /* CONFIG_SPI_4 */

#ifdef CONFIG_SPI_5

#if defined(CONFIG_SPI_STM32_INTERRUPT) || defined(CONFIG_SPI_STM32_DMA)
static void spi_stm32_irq_config_func_5(struct device *port);
#endif

static const  struct spi_stm32_config spi_stm32_cfg_5 = {
	.spi = (SPI_TypeDef *) DT_SPI_5_BASE_ADDRESS,
	.pclken = {
		.enr = DT_SPI_5_CLOCK_BITS,
		.bus = DT_SPI_5_CLOCK_BUS
	},
#if defined(CONFIG_SPI_STM32_INTERRUPT) || defined(CONFIG_SPI_STM32_DMA)
	.irq_config = spi_stm32_irq_config_func_5,
#endif

#ifdef CONFIG_SPI_STM32_DMA
	.stream[TX_STREAM] = 4,
	.stream[RX_STREAM] = 5,
	.dmadev = CONFIG_DMA_2_NAME,
#endif
};

static struct spi_stm32_data spi_stm32_dev_data_5 = {
	SPI_CONTEXT_INIT_LOCK(spi_stm32_dev_data_5, ctx),
	SPI_CONTEXT_INIT_SYNC(spi_stm32_dev_data_5, ctx),
#ifdef CONFIG_SPI_STM32_DMA
	/* Transmit block config */
	.b[TX_STREAM].dest_address = (u32_t)&(SPI5->DR),
	.b[TX_STREAM].source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
	.b[TX_STREAM].dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,

	/* Receive block config */
	.b[RX_STREAM].source_address = (u32_t)&(SPI5->DR),
	.b[RX_STREAM].source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
	.b[RX_STREAM].dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,

	/* Setup for transmit stream */
	.dma_conf[TX_STREAM].dma_slot = 7,
	.dma_conf[TX_STREAM].channel_direction = MEMORY_TO_PERIPHERAL,
	.dma_conf[TX_STREAM].source_data_size = 0,      /* 8 bit data */
	.dma_conf[TX_STREAM].dest_data_size = 0,        /* 8 bit data */
	.dma_conf[TX_STREAM].source_burst_length = 0,
	.dma_conf[TX_STREAM].dest_burst_length = 0,
	.dma_conf[TX_STREAM].dma_callback = spi_stm32_dma_callback,
	.dma_conf[TX_STREAM].complete_callback_en = 1,
	.dma_conf[TX_STREAM].error_callback_en = 1,
	.dma_conf[TX_STREAM].head_block = &spi_stm32_dev_data_5.b[TX_STREAM],

	/* Setup for receive stream */
	.dma_conf[RX_STREAM].dma_slot = 7,
	.dma_conf[RX_STREAM].channel_direction = PERIPHERAL_TO_MEMORY,
	.dma_conf[RX_STREAM].source_data_size = 0,      /* 8 bit data */
	.dma_conf[RX_STREAM].dest_data_size = 0,        /* 8 bit data */
	.dma_conf[RX_STREAM].source_burst_length = 0,
	.dma_conf[RX_STREAM].dest_burst_length = 0,
	.dma_conf[RX_STREAM].dma_callback = spi_stm32_dma_callback,
	.dma_conf[TX_STREAM].complete_callback_en = 1,
	.dma_conf[TX_STREAM].error_callback_en = 1,
	.dma_conf[RX_STREAM].head_block = &spi_stm32_dev_data_5.b[RX_STREAM],
#endif
};

DEVICE_AND_API_INIT(spi_stm32_5, DT_SPI_5_NAME, &spi_stm32_init,
		    &spi_stm32_dev_data_5, &spi_stm32_cfg_5,
		    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,
		    &api_funcs);

#if defined(CONFIG_SPI_STM32_INTERRUPT) || defined(CONFIG_SPI_STM32_DMA)
static void spi_stm32_irq_config_func_5(struct device *dev)
{
	IRQ_CONNECT(DT_SPI_5_IRQ, DT_SPI_5_IRQ_PRI,
		    spi_stm32_isr, DEVICE_GET(spi_stm32_5), 0);
	irq_enable(DT_SPI_5_IRQ);
}
#endif

#endif /* CONFIG_SPI_5 */

#ifdef CONFIG_SPI_6

#if defined(CONFIG_SPI_STM32_INTERRUPT) || defined(CONFIG_SPI_STM32_DMA)
static void spi_stm32_irq_config_func_6(struct device *port);
#endif

static const  struct spi_stm32_config spi_stm32_cfg_6 = {
	.spi = (SPI_TypeDef *) DT_SPI_6_BASE_ADDRESS,
	.pclken = {
		.enr = DT_SPI_6_CLOCK_BITS,
		.bus = DT_SPI_6_CLOCK_BUS
	},
#if defined(CONFIG_SPI_STM32_INTERRUPT) || defined(CONFIG_SPI_STM32_DMA)
	.irq_config = spi_stm32_irq_config_func_6,
#endif
#ifdef CONFIG_SPI_STM32_DMA
	.stream[TX_STREAM] = 5,
	.stream[RX_STREAM] = 6,
	.dmadev = CONFIG_DMA_2_NAME,
#endif
};

static struct spi_stm32_data spi_stm32_dev_data_6 = {
	SPI_CONTEXT_INIT_LOCK(spi_stm32_dev_data_6, ctx),
	SPI_CONTEXT_INIT_SYNC(spi_stm32_dev_data_6, ctx),
#ifdef CONFIG_SPI_STM32_DMA
	/* Transmit block config */
	.b[TX_STREAM].dest_address = (u32_t)&(SPI6->DR),
	.b[TX_STREAM].source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
	.b[TX_STREAM].dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,

	/* Receive block config */
	.b[RX_STREAM].source_address = (u32_t)&(SPI6->DR),
	.b[RX_STREAM].source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
	.b[RX_STREAM].dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,

	/* Setup for transmit stream */
	.dma_conf[TX_STREAM].dma_slot = 0,
	.dma_conf[TX_STREAM].channel_direction = MEMORY_TO_PERIPHERAL,
	.dma_conf[TX_STREAM].source_data_size = 0,      /* 8 bit data */
	.dma_conf[TX_STREAM].dest_data_size = 0,        /* 8 bit data */
	.dma_conf[TX_STREAM].source_burst_length = 0,
	.dma_conf[TX_STREAM].dest_burst_length = 0,
	.dma_conf[TX_STREAM].dma_callback = spi_stm32_dma_callback,
	.dma_conf[TX_STREAM].complete_callback_en = 1,
	.dma_conf[TX_STREAM].error_callback_en = 1,
	.dma_conf[TX_STREAM].head_block = &spi_stm32_dev_data_6.b[TX_STREAM],

	/* Setup for receive stream */
	.dma_conf[RX_STREAM].dma_slot = 0,
	.dma_conf[RX_STREAM].channel_direction = PERIPHERAL_TO_MEMORY,
	.dma_conf[RX_STREAM].source_data_size = 0,      /* 8 bit data */
	.dma_conf[RX_STREAM].dest_data_size = 0,        /* 8 bit data */
	.dma_conf[RX_STREAM].source_burst_length = 0,
	.dma_conf[RX_STREAM].dest_burst_length = 0,
	.dma_conf[RX_STREAM].dma_callback = spi_stm32_dma_callback,
	.dma_conf[TX_STREAM].complete_callback_en = 1,
	.dma_conf[TX_STREAM].error_callback_en = 1,
	.dma_conf[RX_STREAM].head_block = &spi_stm32_dev_data_6.b[RX_STREAM],
#endif
};

DEVICE_AND_API_INIT(spi_stm32_6, DT_SPI_6_NAME, &spi_stm32_init,
		    &spi_stm32_dev_data_6, &spi_stm32_cfg_6,
		    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,
		    &api_funcs);

#if defined(CONFIG_SPI_STM32_INTERRUPT) || defined(CONFIG_SPI_STM32_DMA)
static void spi_stm32_irq_config_func_6(struct device *dev)
{
	IRQ_CONNECT(DT_SPI_6_IRQ, DT_SPI_6_IRQ_PRI,
		    spi_stm32_isr, DEVICE_GET(spi_stm32_6), 0);
	irq_enable(DT_SPI_6_IRQ);
}
#endif

#endif /* CONFIG_SPI_6 */
