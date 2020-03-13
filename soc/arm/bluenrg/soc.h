/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2016 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file SoC configuration macros for the STM32L4 family processors.
 *
 * Based on reference manual:
 *   STM32L4x1, STM32L4x2, STM32L431xx STM32L443xx STM32L433xx, STM32L4x5,
 *   STM32l4x6 advanced ARM(r)-based 32-bit MCUs
 *
 * Chapter 2.2.2: Memory map and register boundary addresses
 */

#ifndef _BLUENRG_SOC_H_
#define _BLUENRG_SOC_H_


#include <sys/util.h>

// #ifndef _ASMLANGUAGE

#include <autoconf.h>
#include <BlueNRG2.h>

/* Add include for DTS generated information */
#include <devicetree.h>
#include <BlueNRG1_gpio.h>
#include <BlueNRG1_uart.h>
#include <BlueNRG1_sysCtrl.h>
#include <BlueNRG1_adc.h>
#include <misc.h>
#endif /* _BLUENRG_SOC_H_ */
