cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico2_w CACHE STRING "Target board")
include(${CMAKE_CURRENT_LIST_DIR}/../../pico_sdk_import.cmake)
project(pico_usb_address_probe C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

if(NOT PICO_PLATFORM STREQUAL "rp2350-arm-s")
    message(FATAL_ERROR "The native PHY timing probe requires the RP2350 ARM platform")
endif()

option(PROBE_USB_GPIO_MODE "Enable native-pad SIO observation with SIO outputs disabled" ON)
set(TINYUSB_DIR ${PICO_SDK_PATH}/lib/tinyusb/src)
set(RP_USB_DIR ${TINYUSB_DIR}/portable/raspberrypi/rp2040)
add_executable(native_usb_address_probe
    main.c
    router.c
    usb_probe.c
    ${RP_USB_DIR}/dcd_rp2040.c
    ${RP_USB_DIR}/rp2040_usb.c
    ${TINYUSB_DIR}/common/tusb_fifo.c
)
target_include_directories(native_usb_address_probe PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${TINYUSB_DIR}
    ${RP_USB_DIR}
)
target_compile_definitions(native_usb_address_probe PRIVATE
    CFG_TUSB_CONFIG_FILE="${CMAKE_CURRENT_LIST_DIR}/tusb_config.h"
    CFG_TUSB_MCU=OPT_MCU_RP2040
    RP2040_USB_DEVICE_MODE=1
    PROBE_USB_GPIO_MODE=$<BOOL:${PROBE_USB_GPIO_MODE}>
)
target_compile_options(native_usb_address_probe PRIVATE -O3 -Wall -Wextra)
target_link_libraries(native_usb_address_probe PRIVATE
    pico_stdlib pico_multicore hardware_structs hardware_irq hardware_resets
    hardware_sync hardware_timer hardware_clocks hardware_vreg hardware_watchdog
)
pico_set_binary_type(native_usb_address_probe no_flash)
pico_enable_stdio_usb(native_usb_address_probe 0)
pico_enable_stdio_uart(native_usb_address_probe 1)
pico_set_program_name(native_usb_address_probe "RAM-only native USB address capability probe")
pico_set_program_version(native_usb_address_probe "0.5-native-sio-hub-probe")
pico_add_extra_outputs(native_usb_address_probe)
