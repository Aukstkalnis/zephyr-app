# SPDX-License-Identifier: Apache-2.0

if (CONFIG_PINMUX_BLUENRG)
	zephyr_include_directories(.)
	zephyr_library()
	zephyr_library_sources(pinmux_bluenrg.c)
endif()