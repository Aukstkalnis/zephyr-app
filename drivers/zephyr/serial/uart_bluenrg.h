/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for UART port on BlueNRG family processor.
 *
 */

#ifndef ZEPHYR_DRIVERS_SERIAL_UART_BLUENRG_H_
#define ZEPHYR_DRIVERS_SERIAL_UART_BLUENRG_H_

/* device config */
struct uart_bluenrg_config {
	struct uart_device_config uconf;
	struct bluenrg_pclken pclken;
	/* initial hardware flow control, 1 for RTS/CTS */
	bool hw_flow_control;
};

/* driver data */
struct uart_bluenrg_data {
	/* Baud rate */
	u32_t baud_rate;
	/* clock device */
	struct device *clock;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
#endif
};
#include <device.h>
#endif	/* ZEPHYR_DRIVERS_SERIAL_UART_BLUENRG_H_ */
