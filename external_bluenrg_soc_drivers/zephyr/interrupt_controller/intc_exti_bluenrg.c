/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2017 RnDity Sp. z o.o.
 * Copyright (c) 2019 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for External interrupt/event controller in STM32 MCUs
 *
 * Driver is currently implemented to support following EXTI lines
 * STM32F1/STM32F3: Lines 0 to 15. Lines > 15 not supported
 * STM32F0/STM32L0/STM32L4/STM32G0/STM32G4: Lines 0 to 15. Lines > 15 are not mapped on an IRQ
 * STM32F2/STM32F4: Lines 0 to 15, 16, 17 18, 21 and 22. Others not supported
 * STM32F7: Lines 0 to 15, 16, 17 18, 21, 22 and 23. Others not supported
 *
 */
#include <device.h>
#include <soc.h>
#include <sys/__assert.h>
#include <exti_bluenrg.h>

static int bluenrg_exti_init(struct device *dev);

/* wrapper for user callback */
struct __exti_cb {
	bluenrg_exti_callback_t cb;
	void *data;
};

/* driver data */
struct bluenrg_exti_data {
	/* per-line callbacks */
	struct __exti_cb cb[CONFIG_NUM_GPIOS];
};

/**
 * @brief EXTI ISR handler
 *
 * Check EXTI lines in range @min @max for pending interrupts
 *
 * @param arg isr argument
 * @param min low end of EXTI# range
 * @param max low end of EXTI# range
 */
static inline void __bluenrg_exti_isr(void *arg)
{
	struct device *dev = arg;
	struct bluenrg_exti_data *data = dev->driver_data;
	int line;

	/* see which bits are set */
	for (line = 0; line < CONFIG_NUM_GPIOS; line++) {
		uint32_t pin = 1 << line;
		/* check if interrupt is pending */
		if(GPIO_GetITPendingBit(pin) == SET) {
			GPIO_ClearITPendingBit(pin);
			/* run callback only if one is registered */
			if (!data->cb[line].cb) {
				continue;
			}
			data->cb[line].cb(line, data->cb[line].data);
		}

	}
}

static struct bluenrg_exti_data exti_data;

DEVICE_INIT(exti_bluenrg, BLUENRG_EXTI_NAME, bluenrg_exti_init,
	    &exti_data, NULL,
	    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

static int bluenrg_exti_init(struct device *dev)
{
	IRQ_CONNECT(GPIO_IRQn, 0, __bluenrg_exti_isr, DEVICE_GET(exti_bluenrg), 0);

	return 0;
}


/**
 * @brief set & unset for the interrupt callbacks
 */
int bluenrg_exti_set_callback(int line, bluenrg_exti_callback_t cb, void *arg)
{
	struct device *dev = DEVICE_GET(exti_bluenrg);
	struct bluenrg_exti_data *data = dev->driver_data;

	if (data->cb[line].cb) {
		return -EBUSY;
	}

	data->cb[line].cb = cb;
	data->cb[line].data = arg;

	return 0;
}

void bluenrg_exti_unset_callback(int line)
{
	struct device *dev = DEVICE_GET(exti_bluenrg);
	struct bluenrg_exti_data *data = dev->driver_data;

	data->cb[line].cb = NULL;
	data->cb[line].data = NULL;
}
