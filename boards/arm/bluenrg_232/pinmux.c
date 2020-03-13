/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2016 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/pinmux.h>
#include <sys/sys_io.h>

#include "pinmux/pinmux.h"
#include "pinmux_bluenrg.h"
#include <dt-bindings\pinctrl\bluenrg-pinctrl.h>
#include <drivers/gpio.h>

/* pin assignments for NUCLEO-L476RG board */
static const struct pin_config pinconf[] = {
	{ BLUENRG_GPIO_8   , ( BLUENRG_GPIO_8_TX)   },
	{ BLUENRG_GPIO_11  , ( BLUENRG_GPIO_11_RX)  },
	{ BLUENRG_GPIO_6   , ( BLUENRG_MODER_OUTPUT_MODE)},
	{ BLUENRG_GPIO_7   , ( BLUENRG_MODER_OUTPUT_MODE)},
	{ BLUENRG_GPIO_14  , ( BLUENRG_MODER_OUTPUT_MODE)},
	{ BLUENRG_GPIO_13  , ( BLUENRG_MODER_INPUT_MODE)},
	{ BLUENRG_GPIO_5   , ( BLUENRG_MODER_INPUT_MODE)},
};

static int pinmux_bluenrg_init(struct device *port)
{
	ARG_UNUSED(port);

	bluenrg_setup_pins(pinconf, ARRAY_SIZE(pinconf));

	return 0;
}

SYS_INIT(pinmux_bluenrg_init, PRE_KERNEL_1, 0);
