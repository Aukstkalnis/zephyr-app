# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_BLUENRG)
  zephyr_include_directories(.)
  zephyr_library()
  zephyr_library_sources(clock_bluenrg.c)
endif()
