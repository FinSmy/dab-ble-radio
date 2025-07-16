cmake_minimum_required(VERSION 3.21)

list(APPEND EXTRA_ZEPHYR_MODULES
  ${CMAKE_CURRENT_SOURCE_DIR}/../si468x_driver_module
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dab_ble_radio LANGUAGES C)

target_sources(app PRIVATE src/main.c)