# SPDX-License-Identifier: Apache-2.0

if (CONFIG_SOC_BLUENRG)
	zephyr_library()
	zephyr_library_sources(uart_bluenrg.c)
endif()
