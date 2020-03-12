# SPDX-License-Identifier: Apache-2.0

add_definitions(-D__ZEPHYR_SUPERVISOR__)


add_subdirectory(interrupt_controller)

add_subdirectory_if_kconfig(clock_control)
add_subdirectory_if_kconfig(gpio)
add_subdirectory_if_kconfig(pinmux)

add_subdirectory_ifdef(CONFIG_SERIAL_HAS_DRIVER serial)
