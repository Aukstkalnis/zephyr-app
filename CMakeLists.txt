# Boilerplate code, which pulls in the Zephyr build system.
cmake_minimum_required(VERSION 3.13.1)

# If driver is a project that's managed by west, this line can removed.
list(APPEND ZEPHYR_EXTRA_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/drivers
  )

# Add an absolute directory path to the CMake variable
# SYSCALL_INCLUDE_DIRS. This ensures that the syscall machinery will
# be able to find the module's syscalls.
list(APPEND SYSCALL_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/drivers)

# Set board root
set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(SOC_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(SOC_ROOT ${CMAKE_CURRENT_LIST_DIR})
# set(DTS_ROOR )

set(BOARD bluenrg_232)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(bluenrg_app)

# Add your source file to the "app" target. This must come after
# the boilerplate code, which defines the target.
target_sources(app PRIVATE src/main.c)
