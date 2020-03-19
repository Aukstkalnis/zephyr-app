/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Include esp-idf headers first to avoid redefining BIT() macro */
// #include <soc/gpio_reg.h>
// #include <soc/io_mux_reg.h>
// #include <soc/soc.h>

#include <errno.h>
#include <sys/util.h>
#include <drivers/pinmux.h>
#include "pinmux/pinmux.h"
#include <pinmux_bluenrg.h>
#include <BlueNRG2.h>
#include <gpio_bluenrg.h>

#define GPIO_REG_SIZE 0x48



static int bluenrg_pin_configure(int pin, int func)
{
	/* determine IO port registers location */
	u32_t offset = pin * GPIO_REG_SIZE;
	u8_t *port_base = (u8_t *)(GPIO_BASE + offset);

	/* not much here, on STM32F10x the alternate function is
	 * controller by setting up GPIO pins in specific mode.
	 */
	return gpio_bluenrg_configure((u32_t *)port_base, pin, func);
}

/**
 * @brief pin setup
 *
 * @param func SoC specific function assignment
 * @param clk  optional clock device
 *
 * @return 0 on success, error otherwise
 */
int z_pinmux_bluenrg_set(u32_t pin, u32_t func)
{
	/* make sure to enable port clock first */
	return bluenrg_pin_configure(pin, func);
}

/**
 * @brief setup pins according to their assignments
 *
 * @param pinconf  board pin configuration array
 * @param pins     array size
 */
void bluenrg_setup_pins(const struct pin_config *pinconf, size_t pins)
{
	// struct device *clk;
	int i;

	for (i = 0; i < pins; i++) {
		z_pinmux_bluenrg_set(pinconf[i].pin_num, pinconf[i].mode);
	}
}
