/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2016 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for UART port on BlueNRG family processor.
 * @note  LPUART and U(S)ART have the same base and
 *        majority of operations are performed the same way.
 *        Please validate for newly added series.
 */

#include <kernel.h>
#include <arch/cpu.h>
#include <sys/__assert.h>
#include <soc.h>
#include <init.h>
#include <drivers/uart.h>
#include <drivers/clock_control.h>

#include <linker/sections.h>
#include <clock_bluenrg.h>
#include "uart_bluenrg.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(uart_bluenrg);

/* convenience defines */
#define DEV_CFG(dev)							\
	((const struct uart_bluenrg_config * const)(dev)->config->config_info)
#define DEV_DATA(dev)							\
	((struct uart_bluenrg_data * const)(dev)->driver_data)
#define UART_STRUCT(dev)					\
	((UART_Type *)(DEV_CFG(dev))->uconf.base)

#define TIMEOUT 1000
#define UART_CLOCK       (16000000)
#define UART_CLOCK_CYCLE16        (16)      
#define UART_CLOCK_CYCLE8         (8)

static inline void uart_bluenrg_set_baudrate(struct device *dev, u32_t baud_rate)
{
	// u32_t clock_rate;
	uint32_t divider;
  	uint16_t ibrd, fbrd;

	if (UART->CR_b.OVSFACT == 0) {
		divider = (UART_CLOCK<<7) / (UART_CLOCK_CYCLE16 * baud_rate);
	} else {
		divider = (UART_CLOCK<<7) / (UART_CLOCK_CYCLE8 * baud_rate);
	}
	ibrd = divider >> 7;
	UART->IBRD = ibrd;
	fbrd = ((divider - (ibrd <<7) + 1) >> 1);
	if (fbrd > 0x3f) {
		ibrd++;
		fbrd = (fbrd - 0x3F) & 0x3F;
	}
	UART->FBRD = fbrd;
}

static inline void uart_bluenrg_set_parity(struct device *dev, u32_t parity)
{	
	switch(parity) {
		case UART_Parity_No: {
			UART->LCRH_TX_b.PEN_TX = RESET;
			UART->LCRH_RX_b.PEN_RX = RESET;
		} break;
		case UART_Parity_Odd: {
			UART->LCRH_TX_b.PEN_TX = SET;
    		UART->LCRH_TX_b.EPS_TX = RESET;
    		UART->LCRH_RX_b.PEN_RX = SET;
    		UART->LCRH_RX_b.EPS_RX = RESET;
		} break;
		default: {
			UART->LCRH_TX_b.PEN_TX = SET;
			UART->LCRH_TX_b.EPS_TX = SET;    
			UART->LCRH_RX_b.PEN_RX = SET;
			UART->LCRH_RX_b.EPS_RX = SET; 
		} break;
	}
}

static inline u32_t uart_bluenrg_get_parity(struct device *dev)
{
	if((UART->LCRH_RX_b.PEN_RX == RESET) && (UART->LCRH_RX_b.PEN_RX) == RESET) {
		return UART_Parity_No;
	}
	if ((UART->LCRH_TX_b.PEN_TX == SET) && (UART->LCRH_TX_b.EPS_TX == RESET)
		&& (UART->LCRH_RX_b.PEN_RX == SET) && (UART->LCRH_RX_b.EPS_RX == RESET)) {
		return UART_Parity_Odd;
	}
	return UART_Parity_Even;
}

static inline void uart_bluenrg_set_stopbits(struct device *dev, u32_t stopbits)
{
	UART->LCRH_TX_b.STP2_TX = stopbits;
 	UART->LCRH_RX_b.STP2_RX = stopbits;
}

static inline u32_t uart_bluenrg_get_stopbits(struct device *dev)
{
	return UART->LCRH_TX_b.STP2_TX;
}

static inline void uart_bluenrg_set_databits(struct device *dev, u32_t databits)
{
	UART->LCRH_TX_b.WLEN_TX = databits;
	UART->LCRH_RX_b.WLEN_RX = databits;
}

static inline u32_t uart_bluenrg_get_databits(struct device *dev)
{
	return UART->LCRH_TX_b.WLEN_TX;
}

static inline void uart_bluenrg_set_hwctrl(struct device *dev, u32_t hwctrl)
{
	UART->CR_b.RTSEN = hwctrl & 1;
  	UART->CR_b.CTSEN = (hwctrl>>1) & 1;
}

static inline u32_t uart_bluenrg_get_hwctrl(struct device *dev)
{
	return UART->CR_b.RTSEN;
}

static inline u32_t uart_bluenrg_cfg2ll_parity(enum uart_config_parity parity)
{
	switch (parity) {
	case UART_CFG_PARITY_ODD:
		return UART_Parity_Odd;
	case UART_CFG_PARITY_EVEN:
		return UART_Parity_Even;
	case UART_CFG_PARITY_NONE:
	default:
		return UART_Parity_No;
	}
}

static inline enum uart_config_parity uart_bluenrg_ll2cfg_parity(u32_t parity)
{
	switch (parity) {
	case UART_Parity_Odd:
		return UART_CFG_PARITY_ODD;
	case UART_Parity_Even:
		return UART_CFG_PARITY_EVEN;
	case UART_Parity_No:
	default:
		return UART_CFG_PARITY_NONE;
	}
}

static inline u32_t uart_bluenrg_cfg2ll_stopbits(enum uart_config_stop_bits sb)
{
	switch (sb) {
	case UART_CFG_STOP_BITS_1:
		return UART_StopBits_1;
	case UART_CFG_STOP_BITS_2:
	default:
		return UART_StopBits_2;
	}
}

static inline enum uart_config_stop_bits uart_bluenrg_ll2cfg_stopbits(u32_t sb)
{
	switch (sb) {
	case UART_StopBits_1:
		return UART_CFG_STOP_BITS_1;
	case UART_StopBits_2:
	default:
		return UART_CFG_STOP_BITS_2;
	}
}

static inline u32_t uart_bluenrg_cfg2ll_databits(enum uart_config_data_bits db)
{
	switch (db) {
	case UART_CFG_DATA_BITS_5:
		return UART_WordLength_5b;
	case UART_CFG_DATA_BITS_6:
		return UART_WordLength_6b;
	case UART_CFG_DATA_BITS_7:
		return UART_WordLength_7b;
	case UART_CFG_DATA_BITS_8:
	default:
		return UART_WordLength_8b;
	}
}

static inline enum uart_config_data_bits uart_bluenrg_ll2cfg_databits(u32_t db)
{
	switch (db) {
	case UART_WordLength_5b:
		return UART_CFG_DATA_BITS_5;
	case UART_WordLength_6b:
		return UART_CFG_DATA_BITS_6;
	case UART_WordLength_7b:
		return UART_CFG_DATA_BITS_7;
	case UART_WordLength_8b:
	default:
		return UART_CFG_DATA_BITS_8;
	}
}

/**
 * @brief  Get LL hardware flow control define from
 *         Zephyr hardware flow control option.
 * @note   Supports only UART_CFG_FLOW_CTRL_RTS_CTS.
 * @param  fc: Zephyr hardware flow control option.
 * @retval LL_USART_HWCONTROL_RTS_CTS, or LL_USART_HWCONTROL_NONE.
 */
static inline u32_t uart_bluenrg_cfg2ll_hwctrl(enum uart_config_flow_control fc)
{
	if (fc == UART_CFG_FLOW_CTRL_RTS_CTS) {
		return UART_HardwareFlowControl_RTS_CTS;
	}

	return UART_HardwareFlowControl_None;
}

/**
 * @brief  Get Zephyr hardware flow control option from
 *         LL hardware flow control define.
 * @note   Supports only LL_USART_HWCONTROL_RTS_CTS.
 * @param  fc: LL hardware flow control definition.
 * @retval UART_CFG_FLOW_CTRL_RTS_CTS, or UART_CFG_FLOW_CTRL_NONE.
 */
static inline enum uart_config_flow_control uart_bluenrg_ll2cfg_hwctrl(u32_t fc)
{
	if (fc == UART_HardwareFlowControl_RTS_CTS) {
		return UART_CFG_FLOW_CTRL_RTS_CTS;
	}

	return UART_CFG_FLOW_CTRL_NONE;
}

static int uart_bluenrg_configure(struct device *dev,
				const struct uart_config *cfg)
{
	struct uart_bluenrg_data *data = DEV_DATA(dev);
	// UART_InitType *UartInstance = UART_STRUCT(dev);
	const u32_t parity = uart_bluenrg_cfg2ll_parity(cfg->parity);
	const u32_t stopbits = uart_bluenrg_cfg2ll_stopbits(cfg->stop_bits);
	const u32_t databits = uart_bluenrg_cfg2ll_databits(cfg->data_bits);
	const u32_t flowctrl = uart_bluenrg_cfg2ll_hwctrl(cfg->flow_ctrl);

	/* Hardware doesn't support mark or space parity */
	if ((UART_CFG_PARITY_MARK == cfg->parity) ||
	    (UART_CFG_PARITY_SPACE == cfg->parity)) {
		return -ENOTSUP;
	}

	if (UART_CFG_STOP_BITS_0_5 == cfg->stop_bits) {
		return -ENOTSUP;
	}

	if (UART_CFG_STOP_BITS_1_5 == cfg->stop_bits) {
		return -ENOTSUP;
	}

	/* Driver doesn't support 9B */
	if (UART_CFG_DATA_BITS_9 == cfg->data_bits) {
		return -ENOTSUP;
	}

	/* Driver supports only RTS CTS flow control */
	// if (UART_CFG_FLOW_CTRL_NONE != cfg->flow_ctrl) {
	// 	if (!IS_UART_HWFLOW_INSTANCE(UartInstance) ||
	// 	    UART_CFG_FLOW_CTRL_RTS_CTS != cfg->flow_ctrl) {
	// 		return -ENOTSUP;
	// 	}
	// }
	UART_Cmd(DISABLE);

	if (parity != uart_bluenrg_get_parity(dev)) {
		uart_bluenrg_set_parity(dev, parity);
	}

	if (stopbits != uart_bluenrg_get_stopbits(dev)) {
		uart_bluenrg_set_stopbits(dev, stopbits);
	}

	if (databits != uart_bluenrg_get_databits(dev)) {
		uart_bluenrg_set_databits(dev, databits);
	}

	if (flowctrl != uart_bluenrg_get_hwctrl(dev)) {
		uart_bluenrg_set_hwctrl(dev, flowctrl);
	}

	if (cfg->baudrate != data->baud_rate) {
		uart_bluenrg_set_baudrate(dev, cfg->baudrate);
		data->baud_rate = cfg->baudrate;
	}

	UART_Cmd(ENABLE);
	return 0;
};

static int uart_bluenrg_config_get(struct device *dev, struct uart_config *cfg)
{
	struct uart_bluenrg_data *data = DEV_DATA(dev);

	cfg->baudrate = data->baud_rate;
	cfg->parity = uart_bluenrg_ll2cfg_parity(uart_bluenrg_get_parity(dev));
	cfg->stop_bits = uart_bluenrg_ll2cfg_stopbits(
		uart_bluenrg_get_stopbits(dev));
	cfg->data_bits = uart_bluenrg_ll2cfg_databits(
		uart_bluenrg_get_databits(dev));
	cfg->flow_ctrl = uart_bluenrg_ll2cfg_hwctrl(
		uart_bluenrg_get_hwctrl(dev));
	return 0;
}

static int uart_bluenrg_poll_in(struct device *dev, unsigned char *c)
{
	UART_Type *uart = UART_STRUCT(dev);
	/* Clear overrun error flag */
	if (UART_GetFlagStatus(UART_FLAG_OE)) {
		UART_ClearFlag(UART_FLAG_OE);
	}
	while (UART_GetFlagStatus(UART_FLAG_RXFE) == RESET) {
      ;
    }

	*c = (unsigned char)(uart->DR & 0x00FF);

	return 0;
}

static void uart_bluenrg_poll_out(struct device *dev,
					unsigned char c)
{
	/* Wait for TXE flag to be raised */
	while (!UART_GetFlagStatus(UART_FLAG_TXFE)) {
	}
	UART->DR = (c & (uint16_t)0x01FF);
}

static int uart_bluenrg_err_check(struct device *dev)
{
	// UART_InitType *UartInstance = UART_STRUCT(dev);
	u32_t err = 0U;

	/* Check for errors, but don't clear them here.
	 * Some SoC clear all error flags when at least
	 * one is cleared. (e.g. F4X, F1X, and F2X)
	 */
	if (UART_GetFlagStatus(UART_FLAG_OE)) {
		err |= UART_ERROR_OVERRUN;
	}

	if (UART_GetFlagStatus(UART_FLAG_PE)) {
		err |= UART_ERROR_PARITY;
	}

	if (UART_GetFlagStatus(UART_FLAG_FE)) {
		err |= UART_ERROR_FRAMING;
	}

	if (UART_GetFlagStatus(UART_FLAG_BE)) {
		err |= UART_BREAK;
	}

	if (err & UART_ERROR_OVERRUN) {
		UART_ClearFlag(UART_FLAG_OE);
	}

	if (err & UART_ERROR_PARITY) {
		UART_ClearFlag(UART_FLAG_PE);
	}

	if (err & UART_ERROR_FRAMING) {
		UART_ClearFlag(UART_FLAG_FE);
	}

	if (err & UART_BREAK) {
		UART_ClearFlag(UART_FLAG_BE);
	}


	return err;
}

static inline void __uart_bluenrg_get_clock(struct device *dev)
{
	struct uart_bluenrg_data *data = DEV_DATA(dev);
	struct device *clk =
		device_get_binding(BLUENRG_CLOCK_CONTROL_NAME);

	__ASSERT_NO_MSG(clk);

	data->clock = clk;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int uart_bluenrg_fifo_fill(struct device *dev, const u8_t *tx_data,
				  int size)
{
	u8_t num_tx = 0U;

	while ((size - num_tx > 0) &&
	       UART_GetFlagStatus(UART_FLAG_TXFE)) {
		/* TXE flag will be cleared with byte write to DR|RDR register */

		/* Send a character (8bit , parity none) */
		UART->DR = (tx_data[num_tx++] & (uint16_t)0x01FF);
	}

	return num_tx;
}

static int uart_bluenrg_fifo_read(struct device *dev, u8_t *rx_data,
				  const int size)
{
	u8_t num_rx = 0U;

	while ((size - num_rx > 0) && UART_GetITStatus(UART_IT_RX)) {
		/* RXNE flag will be cleared upon read from DR|RDR register */

		/* Receive a character (8bit , parity none) */
		rx_data[num_rx++] = (unsigned char)(UART->DR & 0x00FF);
		if(UART_GetFlagStatus(UART_IT_OE)) {
		/* Clear overrun error flag */
			UART_ClearFlag(UART_IT_OE);
		}
		UART_ClearITPendingBit(UART_IT_RX);
	}

	return num_rx;
}

static void uart_bluenrg_irq_tx_enable(struct device *dev)
{
	NVIC_InitType NVIC_InitStructure;
  
	/* Enable the UART Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = UART_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	UART_ITConfig(UART_IT_TX, ENABLE);
}

static void uart_bluenrg_irq_tx_disable(struct device *dev)
{
	UART_ITConfig(UART_IT_TX, DISABLE);
}

static int uart_bluenrg_irq_tx_ready(struct device *dev)
{
	
	return UART_GetITStatus(UART_IT_TXFE);
}

static int uart_bluenrg_irq_tx_complete(struct device *dev)
{
	return UART_GetITStatus(UART_IT_TXFE);
}

static void uart_bluenrg_irq_rx_enable(struct device *dev)
{
	NVIC_InitType NVIC_InitStructure;
  
	/* Enable the UART Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = UART_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LOW_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	UART_ITConfig(UART_IT_RX, ENABLE);
}

static void uart_bluenrg_irq_rx_disable(struct device *dev)
{
	UART_ITConfig(UART_IT_RX, DISABLE);
}

static int uart_bluenrg_irq_rx_ready(struct device *dev)
{
	return UART_GetITStatus(UART_IT_RX);
}

static void uart_bluenrg_irq_err_enable(struct device *dev)
{
	UART_ITConfig(UART_IT_RT | UART_IT_FE | UART_IT_PE | UART_IT_BE | UART_IT_OE, ENABLE);
}

static void uart_bluenrg_irq_err_disable(struct device *dev)
{
	UART_ITConfig(UART_IT_RT | UART_IT_FE | UART_IT_PE | UART_IT_BE | UART_IT_OE, DISABLE);
}

static int uart_bluenrg_irq_is_pending(struct device *dev)
{
	return (

		((UART_GetITStatus(UART_IT_RX)) && UART_GetFlagStatus(UART_FLAG_RXFE)) || 
		((UART_GetITStatus(UART_IT_TXFE)) && (UART_GetFlagStatus(UART_FLAG_TXFE)))
	);
}

static int uart_bluenrg_irq_update(struct device *dev)
{
	return 1;
}

static void uart_bluenrg_irq_callback_set(struct device *dev,
					uart_irq_callback_user_data_t cb,
					void *cb_data)
{
	struct uart_bluenrg_data *data = DEV_DATA(dev);

	data->user_cb = cb;
	data->user_data = cb_data;
}

static void uart_bluenrg_isr(void *arg)
{
	struct device *dev = arg;
	struct uart_bluenrg_data *data = DEV_DATA(dev);

	if (data->user_cb) {
		data->user_cb(data->user_data);
	}
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_bluenrg_driver_api = {
	.poll_in = uart_bluenrg_poll_in,
	.poll_out = uart_bluenrg_poll_out,
	.err_check = uart_bluenrg_err_check,
	.configure = uart_bluenrg_configure,
	.config_get = uart_bluenrg_config_get,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_bluenrg_fifo_fill,
	.fifo_read = uart_bluenrg_fifo_read,
	.irq_tx_enable = uart_bluenrg_irq_tx_enable,
	.irq_tx_disable = uart_bluenrg_irq_tx_disable,
	.irq_tx_ready = uart_bluenrg_irq_tx_ready,
	.irq_tx_complete = uart_bluenrg_irq_tx_complete,
	.irq_rx_enable = uart_bluenrg_irq_rx_enable,
	.irq_rx_disable = uart_bluenrg_irq_rx_disable,
	.irq_rx_ready = uart_bluenrg_irq_rx_ready,
	.irq_err_enable = uart_bluenrg_irq_err_enable,
	.irq_err_disable = uart_bluenrg_irq_err_disable,
	.irq_is_pending = uart_bluenrg_irq_is_pending,
	.irq_update = uart_bluenrg_irq_update,
	.irq_callback_set = uart_bluenrg_irq_callback_set,
#endif	/* CONFIG_UART_INTERRUPT_DRIVEN */
};

/**
 * @brief Initialize UART channel
 *
 * This routine is called to reset the chip in a quiescent state.
 * It is assumed that this function is called only once per UART.
 *
 * @param dev UART device struct
 *
 * @return 0
 */
static int uart_bluenrg_init(struct device *dev)
{
	const struct uart_bluenrg_config *config = DEV_CFG(dev);

	struct device *clk = device_get_binding(BLUENRG_CLOCK_CONTROL_NAME);

	if (clock_control_on(clk, (clock_control_subsys_t *)&config->pclken) != 0) {
		return -EIO;
	}
	UART_Cmd(DISABLE);

	UART_InitType UART_InitStructure;
  	UART_InitStructure.UART_BaudRate = (uint32_t)115200;
  	UART_InitStructure.UART_WordLengthTransmit = UART_WordLength_8b;
  	UART_InitStructure.UART_WordLengthReceive = UART_WordLength_8b;
  	UART_InitStructure.UART_StopBits = UART_StopBits_1;
  	UART_InitStructure.UART_Parity = UART_Parity_No;
 	UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
  	UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;
  	UART_InitStructure.UART_FifoEnable = DISABLE;
  	UART_Init(&UART_InitStructure);
	UART_Cmd(ENABLE);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->uconf.irq_config_func(dev);
#endif
	return 0;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define BLUENRG_UART_IRQ_HANDLER_DECL(name)				\
	static void uart_bluenrg_irq_config_func_##name(struct device *dev)
#define BLUENRG_UART_IRQ_HANDLER_FUNC(name)				\
	.irq_config_func = uart_bluenrg_irq_config_func_##name,
#define BLUENRG_UART_IRQ_HANDLER(name)					\
static void uart_bluenrg_irq_config_func_##name(struct device *dev)	\
{									\
	IRQ_CONNECT(DT_##name##_IRQ,					\
		DT_UART_BLUENRG_##name##_IRQ_PRI,			\
		uart_bluenrg_isr, DEVICE_GET(uart_bluenrg_##name),	\
		0);							\
	irq_enable(DT_##name##_IRQ);					\
}
#else
#define BLUENRG_UART_IRQ_HANDLER_DECL(name)
#define BLUENRG_UART_IRQ_HANDLER_FUNC(name)
#define BLUENRG_UART_IRQ_HANDLER(name)
#endif

#define BLUENRG_UART_INIT(name)						\
BLUENRG_UART_IRQ_HANDLER_DECL(name);					\
									\
static const struct uart_bluenrg_config uart_bluenrg_cfg_##name = {		\
	.uconf = {							\
		.base = (u8_t *)DT_UART_BLUENRG_##name##_BASE_ADDRESS,\
		BLUENRG_UART_IRQ_HANDLER_FUNC(name)			\
	},								\
	.pclken = { .bus = DT_UART_BLUENRG_##name##_CLOCK_BUS \
	},								\
	.hw_flow_control = DT_UART_BLUENRG_##name##_HW_FLOW_CONTROL	\
};									\
									\
static struct uart_bluenrg_data uart_bluenrg_data_##name = {		\
	.baud_rate = DT_UART_BLUENRG_##name##_BAUD_RATE			\
};									\
									\
DEVICE_AND_API_INIT(uart_bluenrg_##name, DT_UART_BLUENRG_##name##_NAME,	\
		    &uart_bluenrg_init,					\
		    &uart_bluenrg_data_##name, &uart_bluenrg_cfg_##name,	\
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,	\
		    &uart_bluenrg_driver_api);				\
									\
BLUENRG_UART_IRQ_HANDLER(name)

// #ifdef CONFIG_UART_BLUENRG
BLUENRG_UART_INIT(USART_1)
// #endif	/* CONFIG_UART_BLUENRG */
