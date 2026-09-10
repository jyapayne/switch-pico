cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico2_w CACHE STRING "Target board")
include(${CMAKE_CURRENT_LIST_DIR}/../../pico_sdk_import.cmake)
project(switch2_usb_probe C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
include(${CMAKE_CURRENT_LIST_DIR}/probe_build.cmake)
pico_sdk_init()
add_executable(switch2-usb-probe
    ../../src/firmware/configuration/configuration_storage.cpp
    ../../src/firmware/platform/pico/bootsel_pairing_button.cpp)
target_compile_definitions(switch2-usb-probe PRIVATE
    PICO_FLASH_ASSUME_CORE1_SAFE=1 PICO_FLASH_ASSERT_ON_UNSAFE=0)
switch2_usb_probe_configure(switch2-usb-probe)
pico_add_extra_outputs(switch2-usb-probe)
