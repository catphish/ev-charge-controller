cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

project(charge_controller C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(charge_controller)
pico_generate_pio_header(charge_controller ${CMAKE_CURRENT_LIST_DIR}/charge_controller.pio)
target_sources(charge_controller PRIVATE charge_controller.c)
target_link_libraries(charge_controller PRIVATE pico_stdlib hardware_pio hardware_spi hardware_sleep hardware_dma)

pico_enable_stdio_usb(charge_controller 1)
#pico_enable_stdio_uart(charge_controller 0)

pico_add_extra_outputs(charge_controller)
