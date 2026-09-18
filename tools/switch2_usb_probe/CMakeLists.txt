cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico2_w CACHE STRING "Target board")
include(${CMAKE_CURRENT_LIST_DIR}/../../pico_sdk_import.cmake)
project(switch2_usb_probe C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
include(${CMAKE_CURRENT_LIST_DIR}/probe_build.cmake)
if(SWITCH2_PROBE_NEUTRAL_INPUT)
    # Match the proven hub clock/flash timing without enabling the radio.
    add_compile_definitions(
        SWITCH_PICO_SYS_CLOCK_MHZ=240 SWITCH_PICO_OVERCLOCK_MV=1300
        PICO_FLASH_SPI_CLKDIV=4 PICO_EMBED_XIP_SETUP=1
        PICO_STACK_SIZE=16384 PICO_CORE1_STACK_SIZE=4096
        CYW43_PIO_CLOCK_DIV_INT=0 CYW43_PIO_CLOCK_DIV_FRAC8=0)
endif()
pico_sdk_init()
add_executable(switch2-usb-probe
    ../../src/firmware/configuration/configuration_storage.cpp
    ../../src/firmware/platform/pico/bootsel_pairing_button.cpp)
target_compile_definitions(switch2-usb-probe PRIVATE
    PICO_FLASH_ASSUME_CORE1_SAFE=1 PICO_FLASH_ASSERT_ON_UNSAFE=0)
switch2_usb_probe_configure(switch2-usb-probe)
if(SWITCH2_PROBE_NEUTRAL_INPUT)
    target_sources(switch2-usb-probe PRIVATE
        bootsel.cpp
        ../../src/firmware/usb/usb_configuration_management.cpp
        ../../src/firmware/platform/pico/system_clock.cpp)
    target_link_libraries(switch2-usb-probe PRIVATE hardware_adc hardware_vreg hardware_powman)
endif()
pico_add_extra_outputs(switch2-usb-probe)
