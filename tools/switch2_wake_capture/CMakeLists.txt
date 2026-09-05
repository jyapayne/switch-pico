cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico2_w CACHE STRING "Target board")
include(${CMAKE_CURRENT_LIST_DIR}/../../pico_sdk_import.cmake)
project(switch2_wake_capture C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()
add_executable(switch2-wake-capture main.c)
target_include_directories(switch2-wake-capture PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(switch2-wake-capture PRIVATE
    pico_stdlib
    pico_cyw43_arch_none
    pico_btstack_cyw43
    pico_btstack_ble
)
pico_enable_stdio_usb(switch2-wake-capture 1)
pico_enable_stdio_uart(switch2-wake-capture 0)
pico_set_program_name(switch2-wake-capture "Switch 2 wake capture")
pico_set_program_version(switch2-wake-capture "1.0")
pico_add_extra_outputs(switch2-wake-capture)
