if(CONFIG_SOC_BLUENRG)
  # Add hello_world_driver.h to the set of global include paths.
  add_subdirectory_if_kconfig(gpio)
  add_subdirectory_if_kconfig(pinmux)
endif()
