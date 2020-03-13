# SPDX-License-Identifier: Apache-2.0

if (CONFIG_GPIO_BLUENRG)
	zephyr_include_directories(.)
	zephyr_library()
	zephyr_library_sources(gpiogpio_bluenrg.c)
endif()