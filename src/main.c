/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <sys/ring_buffer.h>


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   100

#ifndef DT_ALIAS_LED0_GPIOS_FLAGS
#define DT_ALIAS_LED0_GPIOS_FLAGS 0
#endif
#define TX_BUF_SIZE 256
#define RX_BUF_SIZE 256
struct ring_buf 	 tx_buf;
uint32_t 			 tx_data[TX_BUF_SIZE];
struct ring_buf 	 rx_buf;
uint32_t 			 rx_data[RX_BUF_SIZE];

int send_to_port(char* buffer, uint16_t length);

static void irq_handler_serial(struct device *dev) {
	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		/* TX IRQ */
		if (uart_irq_tx_ready(dev) ) {
			uint8_t byte = 0;
			if (!ring_buf_get(&tx_buf, &byte, 1)) {
				// Means buffer is empty and we can disable the IRQ
				uart_irq_tx_disable(dev);
			} else {
				uart_fifo_fill(dev, &byte, 1);
			}
		}

		/* RX IRQ */
		if (uart_irq_rx_ready(dev)) {
			u8_t byte;
			int rx;
			rx = uart_fifo_read(dev, &byte, 1);
			if (rx < 0) {
				return;
			}
			send_to_port("%c", byte);
			ring_buf_put(&rx_buf, &byte, 1);
		}				
	}
}

int send_to_port(char* buffer, uint16_t length) {
	int ret_val = -ENODEV;
	struct device *dev = device_get_binding("UART_1");

	if (!dev) {
		return ret_val;
	}
	ring_buf_put(&tx_buf, buffer, length);
	uart_irq_tx_enable(dev);
	ret_val = 0;
	return ret_val;
}

void main(void)
{
	struct device *dev;
	bool led_is_on = true;
	int ret;

	struct device *uart_dev = device_get_binding("UART_1");

	if(uart_dev == NULL) {
		return;
	}

	dev = device_get_binding(DT_ALIAS_LED0_GPIOS_CONTROLLER);
	if (dev == NULL) {
		return;
	}

	ret = gpio_pin_configure(dev, DT_ALIAS_LED0_GPIOS_PIN,
				 GPIO_OUTPUT_ACTIVE
				 | DT_ALIAS_LED0_GPIOS_FLAGS);
	if (ret < 0) {
		return;
	}
	unsigned char c; 

	/* Ring buffer init */
	ring_buf_init(&tx_buf, sizeof(tx_data), tx_data);
	ring_buf_init(&rx_buf, sizeof(rx_data), rx_data);

	uart_irq_rx_disable(uart_dev);
	uart_irq_tx_disable(uart_dev);

	uart_irq_callback_set(uart_dev, irq_handler_serial);

	uart_irq_rx_enable(uart_dev);

	while (1) {
		gpio_pin_set(dev, DT_ALIAS_LED0_GPIOS_PIN, (int)led_is_on);
		led_is_on = !led_is_on;
		u8_t pData[TX_BUF_SIZE] = {0};
		int rx = ring_buf_get(&rx_buf, pData, sizeof(pData));
		if(rx > 0) {
			printk("Received %s \n", pData);
		}
		// uart_poll_in(uart_dev, &c);
		// uart_poll_out(uart_dev, c);
		k_sleep(SLEEP_TIME_MS);
	}
}
