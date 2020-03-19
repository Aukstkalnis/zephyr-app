/*
 * Copyright (c) 2016 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(spi_bluenrg);

#include <arch/cpu.h>
#include <sys/__assert.h>
#include <init.h>
#include <linker/sections.h>
#include <clock_bluenrg.h>

#include <sys/util.h>
#include <kernel.h>
#include <soc.h>
#include <errno.h>
#include <drivers/spi.h>
#include <toolchain.h>
#include <device.h>

#include <clock_bluenrg.h>
#include <drivers/clock_control.h>

#include "spi_bluenrg.h"
#include "spi_context.h"

#define DEV_CFG(dev)						\
(const struct spi_bluenrg_config * const)(dev->config->config_info)

#define DEV_DATA(dev)					\
(struct spi_bluenrg_data * const)(dev->driver_data)


static bool spi_bluenrg_transfer_ongoing(struct spi_bluenrg_data *data)
{
	return spi_context_tx_on(&data->ctx) || spi_context_rx_on(&data->ctx);
}

static int spi_bluenrg_get_err(void)
{
	int err = 0;
	if(SPI_GetITStatus(SPI_IT_TUR)) {
		SPI_ClearITPendingBit(SPI_IT_TUR);
		err = -EIO;
	}
	if(SPI_GetITStatus(SPI_IT_ROR)) {
		SPI_ClearITPendingBit(SPI_IT_ROR);
		err = -EIO;
	}
	return err;
}

/* Shift a SPI frame as master. */
static void spi_bluenrg_shift_m(struct spi_bluenrg_data *data)
{
	u16_t tx_frame = 0x00;
	u16_t rx_frame;


	while (!ll_func_tx_is_empty()) {
		/* NOP */
	}

	if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
		if (spi_context_tx_buf_on(&data->ctx)) {
			tx_frame = UNALIGNED_GET((u8_t *)(data->ctx.tx_buf));
		}
		SPI_SendData(tx_frame);
		/* The update is ignored if TX is off. */
		spi_context_update_tx(&data->ctx, 1, 1);
	} else {
		if (spi_context_tx_buf_on(&data->ctx)) {
			tx_frame = UNALIGNED_GET((u16_t *)(data->ctx.tx_buf));
		}
		SPI_SendData(tx_frame);
		/* The update is ignored if TX is off. */
		spi_context_update_tx(&data->ctx, 2, 1);
	}

	while (!ll_func_rx_is_not_empty()) {
		/* NOP */
	}

	if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
		rx_frame = SPI_ReceiveData();
		if (spi_context_rx_buf_on(&data->ctx)) {
			UNALIGNED_PUT(rx_frame, (u8_t *)data->ctx.rx_buf);
		}
		spi_context_update_rx(&data->ctx, 1, 1);
	} else {
		rx_frame = SPI_ReceiveData();
		if (spi_context_rx_buf_on(&data->ctx)) {
			UNALIGNED_PUT(rx_frame, (u16_t *)data->ctx.rx_buf);
		}
		spi_context_update_rx(&data->ctx, 2, 1);
	}
}

/* Shift a SPI frame as slave. */
static void spi_bluenrg_shift_s(struct spi_bluenrg_data *data)
{
	if (ll_func_tx_is_empty() && spi_context_tx_on(&data->ctx)) {
		u16_t tx_frame;

		if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
			tx_frame = UNALIGNED_GET((u8_t *)(data->ctx.tx_buf));
			SPI_SendData(tx_frame);
			spi_context_update_tx(&data->ctx, 1, 1);
		} else {
			tx_frame = UNALIGNED_GET((u16_t *)(data->ctx.tx_buf));
			SPI_SendData(tx_frame);
			spi_context_update_tx(&data->ctx, 2, 1);
		}
	} else {
		ll_func_disable_int_tx_empty();
	}

	if (ll_func_rx_is_not_empty() &&
	    spi_context_rx_buf_on(&data->ctx)) {
		u16_t rx_frame;

		if (SPI_WORD_SIZE_GET(data->ctx.config->operation) == 8) {
			rx_frame = SPI_ReceiveData();
			UNALIGNED_PUT(rx_frame, (u8_t *)data->ctx.rx_buf);
			spi_context_update_rx(&data->ctx, 1, 1);
		} else {
			rx_frame = SPI_ReceiveData();
			UNALIGNED_PUT(rx_frame, (u16_t *)data->ctx.rx_buf);
			spi_context_update_rx(&data->ctx, 2, 1);
		}
	}
}

/*
 * Without a FIFO, we can only shift out one frame's worth of SPI
 * data, and read the response back.
 *
 */
static int spi_bluenrg_shift_frames(struct spi_bluenrg_data *data)
{
	u16_t operation = data->ctx.config->operation;
	
	if (SPI_OP_MODE_GET(operation) == SPI_OP_MODE_MASTER) {
		spi_bluenrg_shift_m(data);
	} else {
		
		spi_bluenrg_shift_s(data);
	}

	return spi_bluenrg_get_err();
}

static void spi_bluenrg_complete(struct spi_bluenrg_data *data, SPI_Type *spi,
			       int status)
{
#ifdef CONFIG_SPI_BLUENRG_INTERRUPT
	ll_func_disable_int_tx_empty();
	ll_func_disable_int_rx_not_empty();
	ll_func_disable_int_errors();
#endif

	SysCtrl_PeripheralClockCmd (CLOCK_PERIPH_GPIO | CLOCK_PERIPH_SPI, ENABLE);
	// GPIO_InitType GPIO_InitStructure;
	spi_context_cs_control (&data->ctx, false);
	printk ("Switch CS true\n\r");

	while (SET == SPI_GetFlagStatus(SPI_FLAG_BSY));
	/* BSY flag is cleared when MODF flag is raised */

	ll_func_disable_spi();

#ifdef CONFIG_SPI_BLUENRG_INTERRUPT
	spi_context_complete(&data->ctx, status);
#endif
}

#ifdef CONFIG_SPI_BLUENRG_INTERRUPT
static void spi_bluenrg_isr(void *arg)
{
	struct device * const dev = (struct device *) arg;
	const struct spi_bluenrg_config *cfg = dev->config->config_info;
	struct spi_bluenrg_data *data = dev->driver_data;
	SPI_Type= *spi = cfg->spi;
	int err;

	err = spi_bluenrg_get_err();
	if (err) {
		spi_bluenrg_complete(data, spi, err);
		return;
	}

	if (spi_bluenrg_transfer_ongoing(data)) {
		err = spi_bluenrg_shift_frames(spi, data);
	}

	if (err || !spi_bluenrg_transfer_ongoing(data)) {
		spi_bluenrg_complete(data, spi, err);
	}
}
#endif

static int spi_bluenrg_configure(struct device *dev,
			       const struct spi_config *config)
{
	const struct spi_bluenrg_config *cfg = DEV_CFG(dev);
	struct spi_bluenrg_data *data = DEV_DATA(dev);
	// SPI_Type *spi = cfg->spi;
	u32_t clock;

	if (spi_context_configured(&data->ctx, config)) {
		/* Nothing to do */
		return 0;
	}

	if ((SPI_WORD_SIZE_GET(config->operation) != 8)
	    && (SPI_WORD_SIZE_GET(config->operation) != 16)) {
		return -ENOTSUP;
	}

	if (clock_control_get_rate(device_get_binding(BLUENRG_CLOCK_CONTROL_NAME),
			(clock_control_subsys_t) &cfg->pclken, &clock) < 0) {
		LOG_ERR("Failed call clock_control_get_rate");
		return -EIO;
	}

	SPI_Cmd(DISABLE);

	SPI_SetMasterCommunicationMode(SPI_FULL_DUPLEX_MODE);

	SPI_InitType SPI_InitStructure;
	SPI_StructInit(&SPI_InitStructure);

	if (config->operation & SPI_OP_MODE_SLAVE) {
		SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
	} else {
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	}


	if (SPI_WORD_SIZE_GET(config->operation) ==  8) {
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	} else {
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	}

	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) {
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	} else {
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	}

	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) {
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	} else {
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	}

	SPI_InitStructure.SPI_BaudRate = 1000000;
	SPI_Init(&SPI_InitStructure);

	/* Clear RX and TX FIFO */
	SPI_ClearTXFIFO();
	SPI_ClearRXFIFO();
	
	/* Set null character */
	SPI_SetDummyCharacter(0xFF);

	/* At this point, it's mandatory to set this on the context! */
	data->ctx.config = config;

	SPI_Cmd(ENABLE);

	spi_context_cs_configure(&data->ctx);

	printk("Installed"
		    " mode %u/%u/%u, slave %u",
		    (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) ? 1 : 0,
		    (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) ? 1 : 0,
		    (SPI_MODE_GET(config->operation) & SPI_MODE_LOOP) ? 1 : 0,
		    config->slave);

	return 0;
}

static int spi_bluenrg_release(struct device *dev,
			     const struct spi_config *config)
{
	struct spi_bluenrg_data *data = DEV_DATA(dev);

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static int transceive(struct device *dev,
		      const struct spi_config *config,
		      const struct spi_buf_set *tx_bufs,
		      const struct spi_buf_set *rx_bufs,
		      bool asynchronous, struct k_poll_signal *signal)
{
	const struct spi_bluenrg_config *cfg = DEV_CFG(dev);
	struct spi_bluenrg_data *data = DEV_DATA(dev);
	SPI_Type *spi = cfg->spi;
	int ret;

	if (!tx_bufs && !rx_bufs) {
		return 0;
	}

#ifndef CONFIG_SPI_BLUENRG_INTERRUPT
	if (asynchronous) {
		return -ENOTSUP;
	}
#endif

	spi_context_lock(&data->ctx, asynchronous, signal);

	ret = spi_bluenrg_configure(dev, config);
	if (ret) {
		return ret;
	}

	/* Set buffers info */
	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, 1);
	SPI_Cmd(ENABLE);

	/* This is turned off in spi_bluenrg_complete(). */
	spi_context_cs_control(&data->ctx, true);


#ifdef CONFIG_SPI_BLUENRG_INTERRUPT
	ll_func_enable_int_errors();

	if (rx_bufs) {
		ll_func_enable_int_rx_not_empty();
	}

	ll_func_enable_int_tx_empty();

	ret = spi_context_wait_for_completion(&data->ctx);
#else

	do {
		ret = spi_bluenrg_shift_frames(data);
	} while (!ret && spi_bluenrg_transfer_ongoing(data));

	spi_bluenrg_complete(data, spi, ret);

#ifdef CONFIG_SPI_SLAVE
	if (spi_context_is_slave(&data->ctx) && !ret) {
		ret = data->ctx.recv_frames;
	}
#endif /* CONFIG_SPI_SLAVE */

#endif

	spi_context_release(&data->ctx, ret);

	return ret;
}

static int spi_bluenrg_transceive(struct device *dev,
				const struct spi_config *config,
				const struct spi_buf_set *tx_bufs,
				const struct spi_buf_set *rx_bufs)
{
	return transceive(dev, config, tx_bufs, rx_bufs, false, NULL);
}

#ifdef CONFIG_SPI_ASYNC
static int spi_bluenrg_transceive_async(struct device *dev,
				      const struct spi_config *config,
				      const struct spi_buf_set *tx_bufs,
				      const struct spi_buf_set *rx_bufs,
				      struct k_poll_signal *async)
{
	return transceive(dev, config, tx_bufs, rx_bufs, true, async);
}
#endif /* CONFIG_SPI_ASYNC */

static const struct spi_driver_api api_funcs = {
	.transceive = spi_bluenrg_transceive,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_bluenrg_transceive_async,
#endif
	.release = spi_bluenrg_release,
};

ErrorStatus SdkEvalSpiRead(uint8_t* pBuffer, uint8_t RegisterAddr, uint8_t NumByteToRead)
{
  /* Set communication mode */
  SPI_SetMasterCommunicationMode(SPI_FULL_DUPLEX_MODE);
  
  GPIO_ResetBits(GPIO_Pin_1);    
  
  /* Write data to send to TX FIFO */
  while(RESET == SPI_GetFlagStatus(SPI_FLAG_TFE));
  SPI_SendData(0x80 | RegisterAddr);  
  while(RESET == SPI_GetFlagStatus(SPI_FLAG_RNE));
  SPI_ReceiveData();
  
  for(uint8_t i = 0; i< NumByteToRead; i++) {
    while(RESET == SPI_GetFlagStatus(SPI_FLAG_TFE));
    SPI_SendData(0xFF);  
    while(RESET == SPI_GetFlagStatus(SPI_FLAG_RNE));
    pBuffer[i] = SPI_ReceiveData();
  }
  
  while (SET == SPI_GetFlagStatus(SPI_FLAG_BSY));
  
  GPIO_SetBits(GPIO_Pin_1);
  
  return SUCCESS;
}

static int spi_bluenrg_init(struct device *dev)
{
	struct spi_bluenrg_data *data __attribute__((unused)) = dev->driver_data;
	const struct spi_bluenrg_config *cfg = dev->config->config_info;

	__ASSERT_NO_MSG(device_get_binding(BLUENRG_CLOCK_CONTROL_NAME));

	if (clock_control_on(device_get_binding(BLUENRG_CLOCK_CONTROL_NAME),
			       (clock_control_subsys_t) &cfg->pclken) != 0) {
		LOG_ERR("Could not enable SPI clock");
		return -EIO;
	}

#ifdef CONFIG_SPI_BLUENRG_INTERRUPT
	cfg->irq_config(dev);
#endif

	spi_context_unlock_unconditionally (&data->ctx);

	return 0;
}

#ifdef CONFIG_SPI_BLUENRG_INTERRUPT
#define BLUENRG_SPI_IRQ_HANDLER_DECL()					\
	static void spi_bluenrg_irq_config_func(struct device *dev)
#define BLUENRG_SPI_IRQ_HANDLER_FUNC()					\
	.irq_config = spi_bluenrg_irq_config_func,
#define BLUENRG_SPI_IRQ_HANDLER()					\
static void spi_bluenrg_irq_config_func(struct device *dev)		\
{									\
	IRQ_CONNECT(DT_SPI_IRQ, DT_SPI_IRQ_PRI,		\
		    spi_bluenrg_isr, DEVICE_GET(spi_bluenrg), 0);	\
	irq_enable(DT_SPI_IRQ);					\
}
#else
#define BLUENRG_SPI_IRQ_HANDLER_DECL(id)
#define BLUENRG_SPI_IRQ_HANDLER_FUNC(id)
#define BLUENRG_SPI_IRQ_HANDLER(id)
#endif

#define BLUENRG_SPI_INIT()						\
BLUENRG_SPI_IRQ_HANDLER_DECL();						\
									\
static const struct spi_bluenrg_config _spi_bluenrg_cfg = {		\
	.spi = (SPI_Type *) DT_SPI_BASE_ADDRESS,		\
	.pclken = {							\
		.bus = DT_SPI_CLOCK_BUS				\
	},								\
	BLUENRG_SPI_IRQ_HANDLER_FUNC(id)					\
};									\
									\
static struct spi_bluenrg_data spi_bluenrg_dev_data = {		\
	SPI_CONTEXT_INIT_LOCK(spi_bluenrg_dev_data, ctx),		\
	SPI_CONTEXT_INIT_SYNC(spi_bluenrg_dev_data, ctx),		\
};									\
									\
DEVICE_AND_API_INIT(spi_bluenrg, DT_SPI_NAME, &spi_bluenrg_init, \
		    &spi_bluenrg_dev_data, &_spi_bluenrg_cfg,	\
		    POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,		\
		    &api_funcs);					\
									\
BLUENRG_SPI_IRQ_HANDLER()

#ifdef CONFIG_SPI_BLUENRG
BLUENRG_SPI_INIT()
#endif /* CONFIG_SPI_BLUENRG */
