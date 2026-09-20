cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico2_w CACHE STRING "Target board")
include(${CMAKE_CURRENT_LIST_DIR}/../../pico_sdk_import.cmake)
project(switch2_wake_beacon C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(FIRMWARE_DIR ${CMAKE_CURRENT_LIST_DIR}/../../src/firmware)
if(NOT EXISTS "${FIRMWARE_DIR}/platform/pico/switch2_wake_config.h")
    message(FATAL_ERROR
        "Wake-only firmware requires the existing private switch2_wake_config.h. Capture/configure it first; no generic wake packet is provided.")
endif()

pico_sdk_init()
if(NOT PICO_CYW43_SUPPORTED)
    message(FATAL_ERROR "Wake-only firmware requires a wireless Pico board (default: pico2_w).")
endif()

add_executable(switch2-wake-beacon
    main.cpp
    beacon_protocol.cpp
    ${FIRMWARE_DIR}/input/switch2_wake.cpp
)
target_include_directories(switch2-wake-beacon PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${FIRMWARE_DIR}
)
target_compile_definitions(switch2-wake-beacon PRIVATE
    SWITCH2_WAKE_CONFIGURED=1
    ENABLE_BLE=1
    CYW43_LWIP=0
    PICO_CYW43_LOGGING_ENABLED=0
    PICO_BTSTACK_CYW43_MAX_HCI_PROCESS_LOOP_COUNT=4
    USBD_VID=0xCAFE
    USBD_PID=0x4030
    USBD_MANUFACTURER="switch-pico"
    USBD_PRODUCT="switch-pico wake beacon"
    USBD_DESC_STR_MAX=32
    PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=0
    PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=0
    PICO_STDIO_USB_RESET_INTERFACE_SUPPORT_MS_OS_20_DESCRIPTOR=0
    PICO_STDIO_USB_ENABLE_IRQ_BACKGROUND_TASK=0
    PICO_STDIO_USB_SUPPORT_CHARS_AVAILABLE_CALLBACK=0
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=0
    PICO_STDIO_USB_DEFAULT_CRLF=0
    CFG_TUSB_OS=OPT_OS_PICO
    CFG_TUD_TASK_QUEUE_SZ=16
)
# Do not link pico_btstack_cyw43: its startup initializes writable flash TLV.
# HCI-only BLE needs neither pico_btstack_ble's SM/GATT/HID profiles nor Classic.
# The exported low-level SDK driver and transport use our Core-0 polled context.
target_link_libraries(switch2-wake-beacon PRIVATE
    pico_stdlib
    pico_async_context_poll
    pico_cyw43_driver
    cyw43_driver_picow
    pico_btstack_base
    pico_btstack_hci_transport_cyw43
    pico_btstack_run_loop_async_context
)
pico_enable_stdio_usb(switch2-wake-beacon 1)
pico_enable_stdio_uart(switch2-wake-beacon 0)
pico_enable_stdio_rtt(switch2-wake-beacon 0)
pico_set_program_name(switch2-wake-beacon "switch-pico wake beacon")
pico_set_program_version(switch2-wake-beacon "1.0.0")
pico_add_extra_outputs(switch2-wake-beacon)
