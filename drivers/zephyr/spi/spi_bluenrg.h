/*
 * Copyright (c) 2016 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SPI_SPI_LL_BLUENRG_H_
#define ZEPHYR_DRIVERS_SPI_SPI_LL_BLUENRG_H_

#include "spi_context.h"

typedef void (*irq_config_func_t)(struct device *port);

struct spi_bluenrg_config {
	struct bluenrg_pclken pclken;
	SPI_Type *spi;
#ifdef CONFIG_SPI_BLUENRG_INTERRUPT
	irq_config_func_t irq_config;
#endif
};

struct spi_bluenrg_data {
	struct spi_context ctx;
};

static inline u32_t ll_func_tx_is_empty(void)
{
	return SPI_GetFlagStatus(SPI_FLAG_TFE);
}

static inline u32_t ll_func_rx_is_not_empty(void)
{
	return SPI_GetFlagStatus(SPI_FLAG_RNE);
}

static inline void ll_func_enable_int_tx_empty(void)
{
	SPI_ITConfig(SPI_IT_TE, ENABLE);
}

static inline void ll_func_enable_int_rx_not_empty(void)
{
	SPI_ITConfig(SPI_IT_RT, ENABLE);
}

static inline void ll_func_enable_int_errors(void)
{
	SPI_ITConfig(SPI_IT_TUR, ENABLE);
	SPI_ITConfig(SPI_IT_ROR, ENABLE);
}

static inline void ll_func_disable_int_tx_empty(void)
{
	SPI_ITConfig(SPI_IT_TE, DISABLE);
}

static inline void ll_func_disable_int_rx_not_empty(void)
{
	SPI_ITConfig(SPI_IT_RT, DISABLE);
}

static inline void ll_func_disable_int_errors(void)
{
	SPI_ITConfig(SPI_IT_TUR, DISABLE);
	SPI_ITConfig(SPI_IT_ROR, DISABLE);
}

static inline u32_t ll_func_spi_is_busy(void)
{
	return SPI_GetFlagStatus(SPI_FLAG_BSY);
}

static inline void ll_func_disable_spi(void)
{
	SPI_Cmd(DISABLE);
}

#endif	/* ZEPHYR_DRIVERS_SPI_SPI_LL_BLUENRG_H_ */
