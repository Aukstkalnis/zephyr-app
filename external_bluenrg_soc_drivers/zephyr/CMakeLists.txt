if(CONFIG_SOC_BLUENRG)
  add_subdirectory(interrupt_controller)

  add_subdirectory_if_kconfig(gpio)
  add_subdirectory_if_kconfig(pinmux)
  add_subdirectory_if_kconfig(clock_control)
  add_subdirectory_if_kconfig(serial)
endif()
