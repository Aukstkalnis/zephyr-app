/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/adc.h>
#include <drivers/uart.h>
#include <sys/ring_buffer.h>

#include <drivers/spi.h>

#define LSM6DS3_XG_FIFO_ODR_NA                          ((uint8_t)0x00) /*!< FIFO ODR NA */
#define LSM6DS3_XG_FIFO_ODR_10HZ                        ((uint8_t)0x08) /*!< FIFO ODR 10Hz */
#define LSM6DS3_XG_FIFO_ODR_25HZ                        ((uint8_t)0x10) /*!< FIFO ODR 25Hz */
#define LSM6DS3_XG_FIFO_ODR_50HZ                        ((uint8_t)0x18) /*!< FIFO ODR 50Hz */
#define LSM6DS3_XG_FIFO_ODR_100HZ                       ((uint8_t)0x20) /*!< FIFO ODR 100Hz */
#define LSM6DS3_XG_FIFO_ODR_200HZ                       ((uint8_t)0x28) /*!< FIFO ODR 200Hz */
#define LSM6DS3_XG_FIFO_ODR_400HZ                       ((uint8_t)0x30) /*!< FIFO ODR 400Hz */
#define LSM6DS3_XG_FIFO_ODR_800HZ                       ((uint8_t)0x38) /*!< FIFO ODR 800Hz */
#define LSM6DS3_XG_FIFO_ODR_1600HZ                      ((uint8_t)0x40) /*!< FIFO ODR 1600Hz */
#define LSM6DS3_XG_FIFO_ODR_3300HZ                      ((uint8_t)0x48) /*!< FIFO ODR 3300Hz */
#define LSM6DS3_XG_FIFO_ODR_6600HZ                      ((uint8_t)0x50) /*!< FIFO ODR 6600Hz */

#define LSM6DS3_XG_FIFO_ODR_MASK                        ((uint8_t)0x78)
/**
 * @}
 */

/** @defgroup LSM6DS3_XG_FIFO_Mode_Selection_FIFO_CTRL5 LSM6DS3_XG_FIFO_Mode_Selection_FIFO_CTRL5
 * @{
 */
#define LSM6DS3_XG_FIFO_MODE_BYPASS                     ((uint8_t)0x00) /*!< BYPASS Mode. FIFO turned off */
#define LSM6DS3_XG_FIFO_MODE_FIFO                       ((uint8_t)0x01) /*!< FIFO Mode. Stop collecting data when FIFO is full */
#define LSM6DS3_XG_FIFO_MODE_CONTINUOUS_THEN_FIFO       ((uint8_t)0x03) /*!< CONTINUOUS mode until trigger is deasserted, then FIFO mode */
#define LSM6DS3_XG_FIFO_MODE_BYPASS_THEN_CONTINUOUS     ((uint8_t)0x04) /*!< BYPASS mode until trigger is deasserted, then CONTINUOUS mode */
#define LSM6DS3_XG_FIFO_MODE_CONTINUOUS_OVERWRITE       ((uint8_t)0x05) /*!< CONTINUOUS mode. If the FIFO is full the new sample overwrite the older one */

#define LSM6DS3_XG_FIFO_MODE_MASK                       ((uint8_t)0x07)



/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

#ifndef DT_ALIAS_LED0_GPIOS_FLAGS
#define DT_ALIAS_LED0_GPIOS_FLAGS 0
#endif
#define TX_BUF_SIZE 1
#define RX_BUF_SIZE 1
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

static void button_pressed(struct device *dev, struct gpio_callback *cb,
			   u32_t pins)
{
	// if (pins & BIT(DT_ALIAS_SW0_GPIOS_PIN)) {
		printk("Button Pressed %d\n\r", pins);
	// }
}

static enum adc_action sequence_callback(struct device *dev, const struct adc_sequence *sequence, u16_t sampling_index) {
	uint16_t measured = *(uint16_t *)sequence->buffer;
	printk("Channel %u Temperature %d 'C\n\r",sequence->channels, measured);
    return ADC_ACTION_FINISH;
}

static enum adc_action sequence_voltage_callback(struct device *dev, const struct adc_sequence *sequence, u16_t sampling_index) {
	uint16_t measured = *(uint16_t *)sequence->buffer;
	printk("Channel %u Voltage %d mV\n\r", sequence->channels, measured);
    return ADC_ACTION_FINISH;
}
static uint16_t channel_data[3][2];



union lsm6ds_sample {
	u8_t raw[128];
	struct {
		u8_t status;
		s16_t xyz[3];
	} __packed;
};

struct lsm6ds_data {
	struct device *spi;
	struct spi_config spi_cfg;
	union lsm6ds_sample sample;
	/* current scaling factor, in micro m/s^2 / lsb */
	u32_t scale;
};

static int lsm6d3_spi_write(struct device *dev, u8_t reg_addr,
			     u8_t *value, u8_t len)
{
	// struct device *cs_dev = device_get_binding("GPIO");
    struct spi_cs_control cs;
    cs.gpio_dev = device_get_binding(DT_ALIAS_SW0_GPIOS_CONTROLLER);
    cs.gpio_pin = 1;
	cs.delay = 0;

    struct spi_config lsm6ds_spi_conf;
    lsm6ds_spi_conf.frequency = 8000000;
    lsm6ds_spi_conf.operation = (SPI_OP_MODE_MASTER | SPI_MODE_CPOL |
                                 SPI_MODE_CPHA | SPI_WORD_SET (8) | SPI_LINES_SINGLE);
    lsm6ds_spi_conf.slave     = 0xD4;
    lsm6ds_spi_conf.cs        = &cs;

	if(cs.gpio_dev == NULL) {
		return;
	}

	printk("SPI CS initialized.. \n\r");

	u8_t buffer_tx[1] = { reg_addr & ~0x80 };
	const struct spi_buf tx_buf[2] = {
		{
			.buf = buffer_tx,
			.len = 1,
		},
		{
			.buf = value,
			.len = len,
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 2
	};


	if (len > 64) {
		return -EIO;
	}

	if (spi_write(dev, &lsm6ds_spi_conf, &tx)) {
		return -EIO;
	}

	return 0;
}

static int lsm6d3_spi_read(struct device *dev, u8_t reg,
			    u8_t *value, u16_t len)
{
	// struct device *cs_dev = device_get_binding("GPIO");
    struct spi_cs_control cs;
    cs.gpio_dev = device_get_binding(DT_ALIAS_SW0_GPIOS_CONTROLLER);
    cs.gpio_pin = 1;
	cs.delay = 0;

    struct spi_config lsm6ds_spi_conf;
    lsm6ds_spi_conf.frequency = 8000000;
    lsm6ds_spi_conf.operation = (SPI_OP_MODE_MASTER | SPI_MODE_CPOL |
                                 SPI_MODE_CPHA | SPI_WORD_SET (8) | SPI_LINES_SINGLE);
    lsm6ds_spi_conf.slave     = 0xD4;
    lsm6ds_spi_conf.cs        = &cs;

	if(cs.gpio_dev == NULL) {
		return;
	}

	printk("SPI CS initialized.. \n\r");

	struct spi_config *spi_cfg = &lsm6ds_spi_conf;
	u8_t buffer_tx[2] = { reg | 0x80, 0 };
	const struct spi_buf tx_buf = {
			.buf = buffer_tx,
			.len = 2,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = 1,
		},
		{
			.buf = value,
			.len = len,
		}
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2
	};

	if (len > 64) {
		return -EIO;
	}

	if (spi_transceive(dev, spi_cfg, &tx, &rx)) {
		return -EIO;
	}

	return 0;
}

void main(void)
{
	bool led_is_on = true;
	int ret;
	struct device *spi_dev = device_get_binding("SPI");
	struct device *uart_dev = device_get_binding("UART_1");
	struct device *button1 = device_get_binding(DT_ALIAS_SW0_GPIOS_CONTROLLER);
	struct device *button2 = device_get_binding(DT_ALIAS_SW1_GPIOS_CONTROLLER);
	if(spi_dev == NULL) {
		return;
	}

uint8_t data = 0xFF;

	do {
		lsm6d3_spi_read(spi_dev, 0x0F, &data, 1);
		printk("1. Spi recv %02X\n\r", data);
	} while(data != 0x69);

	uint8_t tmp = 0x05;
	lsm6d3_spi_write(spi_dev, 0x12, &tmp, 1);
	uint8_t tmp1 = 0x00;
	do {
		lsm6d3_spi_read(spi_dev, 0x12, &tmp1, 1);
		printk("2. Spi recv %02X\n\r", tmp1);
	} while(tmp1 != 0x04);

	
	tmp1 &= ~(0x04);
  	tmp1 |= 0x04;
	lsm6d3_spi_write(spi_dev, 0x12, &tmp1, 1);

	lsm6d3_spi_read(spi_dev, 0x0A, &tmp1, 1);
	printk("3. Spi recv %02X\n\r", tmp1);

	tmp1 &= ~(LSM6DS3_XG_FIFO_ODR_MASK);
	tmp1 |= LSM6DS3_XG_FIFO_ODR_NA;
  
	/* FIFO mode selection */
	tmp1 &= ~(LSM6DS3_XG_FIFO_MODE_MASK);
	tmp1 |= LSM6DS3_XG_FIFO_MODE_BYPASS;
	lsm6d3_spi_write(spi_dev, 0x0A, &tmp1, 1);

	uint16_t raw = 0xFFFF;
	do {
		lsm6d3_spi_read(spi_dev, 0x28, &raw, 2);
		printk("4. Spi recv %02X\n\r", raw);
		k_sleep(SLEEP_TIME_MS);
	} while(1);

	if(uart_dev == NULL) {
		return;
	}

	struct device *led0 = device_get_binding(DT_ALIAS_LED0_GPIOS_CONTROLLER);
	struct device *led1 = device_get_binding(DT_ALIAS_LED1_GPIOS_CONTROLLER);
	struct device *led2 = device_get_binding(DT_ALIAS_LED2_GPIOS_CONTROLLER);

	struct device *adc_dev = device_get_binding("ADC");

	
	if(adc_dev == NULL) {
		return;
	}
	struct adc_channel_cfg adc_cfg = {
		.channel_id = 1,
		.reference = ADC_REF_EXTERNAL0
	};
	struct adc_channel_cfg adc_cfg_temp = {
		.channel_id = 4,
		.reference = ADC_REF_EXTERNAL0
	};
	struct adc_channel_cfg adc_cfg_bat = {
		.channel_id = 5,
		.reference = ADC_REF_EXTERNAL0
	};
	struct adc_sequence_options options1 = {
		.interval_us = 5000000000,
		.callback = sequence_voltage_callback
	};
	struct adc_sequence_options options = {
		.interval_us = 5000000000,
		.callback = sequence_callback
	};
	struct adc_sequence seq = {
		.options = &options1,
		.buffer = &channel_data[0],
		.buffer_size = sizeof(channel_data[0]),
		.channels = 1,
		.resolution = 10,
	};
	struct adc_sequence seq_temp = {
		.options = &options,
		.buffer = &channel_data[1],
		.buffer_size = sizeof(channel_data[1]),
		.channels = 4,
		.resolution = 10,
	};
	struct adc_sequence seq_bat = {
		.options = &options1,
		.buffer = &channel_data[2],
		.buffer_size = sizeof(channel_data[2]),
		.channels = 5,
		.resolution = 10,
	};
	

	if ((led0 == NULL) || (led2 == NULL)) {
		return;
	}

	ret = gpio_pin_configure(led0, DT_ALIAS_LED0_GPIOS_PIN,
				 GPIO_OUTPUT_ACTIVE
				 | DT_ALIAS_LED0_GPIOS_FLAGS);
	ret = gpio_pin_configure(led1, DT_ALIAS_LED1_GPIOS_PIN,
				 GPIO_OUTPUT_ACTIVE
				 | DT_ALIAS_LED1_GPIOS_FLAGS);
	ret = gpio_pin_configure(led2, DT_ALIAS_LED2_GPIOS_PIN,
				 GPIO_OUTPUT_ACTIVE
				 | DT_ALIAS_LED2_GPIOS_FLAGS);
	
	if (ret < 0) {
		return;
	}

	ret = gpio_pin_configure(button1, DT_ALIAS_SW0_GPIOS_PIN,
					GPIO_INPUT | DT_ALIAS_SW0_GPIOS_FLAGS);
	ret = gpio_pin_configure(button2, DT_ALIAS_SW1_GPIOS_PIN,
					GPIO_INPUT | DT_ALIAS_SW1_GPIOS_FLAGS);
	

	/* Ring buffer init */
	// ring_buf_init(&tx_buf, sizeof(tx_data), tx_data);
	// ring_buf_init(&rx_buf, sizeof(rx_data), rx_data);

	// uart_irq_rx_disable(uart_dev);
	// uart_irq_tx_disable(uart_dev);

	// uart_irq_callback_set(uart_dev, irq_handler_serial);

	// uart_irq_rx_enable(uart_dev);

	static struct gpio_callback button_cb;
	ret = gpio_pin_interrupt_configure(button1, DT_ALIAS_SW0_GPIOS_PIN,
				     GPIO_INT_EDGE_TO_ACTIVE);
	ret = gpio_pin_interrupt_configure(button2, DT_ALIAS_SW1_GPIOS_PIN,
					GPIO_INT_EDGE_TO_ACTIVE);
	
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on pin %d '%s'\n",
			ret, DT_ALIAS_SW0_GPIOS_PIN, DT_ALIAS_SW0_LABEL);
		return;
	}

	gpio_init_callback(&button_cb, button_pressed,
			   BIT(DT_ALIAS_SW0_GPIOS_PIN) | BIT(DT_ALIAS_SW1_GPIOS_PIN));
	gpio_add_callback(button1, &button_cb);
	gpio_add_callback(button2, &button_cb);

	while (1) {
		gpio_pin_set(led0, DT_ALIAS_LED0_GPIOS_PIN, (int)led_is_on);
		gpio_pin_set(led1, DT_ALIAS_LED0_GPIOS_PIN, (int)led_is_on);
		gpio_pin_set(led2, DT_ALIAS_LED2_GPIOS_PIN, (int)led_is_on);
		led_is_on = !led_is_on;
		printk("--------------------------------------------\n\r");
		adc_channel_setup(adc_dev, &adc_cfg);
		adc_read(adc_dev, &seq);
		adc_channel_setup(adc_dev, &adc_cfg_temp);
		adc_read(adc_dev, &seq_temp);
		adc_channel_setup(adc_dev, &adc_cfg_bat);
		adc_read(adc_dev, &seq_bat);
		printk("--------------------------------------------\n\r");
		// u8_t pData[TX_BUF_SIZE] = {0};

		// int rx = ring_buf_get(&rx_buf, pData, sizeof(pData));
		// if(rx > 0) {
		// 	send_to_port(pData, rx);
		// }

		// u8_t pData[TX_BUF_SIZE] = {0};
		// uart_poll_in(uart_dev, &c);
		// uart_poll_out(uart_dev, c);
		k_sleep(SLEEP_TIME_MS);
	}
}
