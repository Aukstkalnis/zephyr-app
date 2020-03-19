
list(APPEND ZEPHYR_EXTRA_MODULES
	${CMAKE_CURRENT_SOURCE_DIR}/drivers
)

cmake_minimum_required(VERSION 3.13.1)

set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(SOC_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(DTS_ROOT ${CMAKE_CURRENT_LIST_DIR}/dts_root)

set(BOARD bluenrg_232)
include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(bluenrg_app)

zephyr_include_directories(${CMAKE_CURRENT_LIST_DIR}/dts_root/include)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/hal_bluenrg)

# zephyr_include_directories(${CMAKE_CURRENT_LIST_DIR}/dts_root/include)

# Add your source file to the "app" target. This must come after
# the boilerplate code, which defines the target.
target_sources(app PRIVATE src/main.c)
