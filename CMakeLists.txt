# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
option(SWITCH_PICO_LOG "Enable UART debug logging" OFF)
option(SWITCH_PICO_ADAPTER_FEASIBILITY
       "Build the automatic Switch/XInput feasibility prototype" OFF)
set(SWITCH_PICO_INPUT_BACKEND "UART" CACHE STRING "Controller input backend")
set_property(CACHE SWITCH_PICO_INPUT_BACKEND PROPERTY STRINGS UART BLUEPAD32)
if(NOT SWITCH_PICO_INPUT_BACKEND STREQUAL "UART"
        AND NOT SWITCH_PICO_INPUT_BACKEND STREQUAL "BLUEPAD32")
    message(FATAL_ERROR
            "Unknown SWITCH_PICO_INPUT_BACKEND='${SWITCH_PICO_INPUT_BACKEND}'. "
            "Expected UART or BLUEPAD32.")
endif()
set(PICO_BOARD pico CACHE STRING "Board type")
if(SWITCH_PICO_INPUT_BACKEND STREQUAL "BLUEPAD32"
        AND NOT PICO_BOARD STREQUAL "pico2_w")
    message(FATAL_ERROR
            "SWITCH_PICO_INPUT_BACKEND=BLUEPAD32 requires PICO_BOARD=pico2_w")
endif()
if(SWITCH_PICO_ADAPTER_FEASIBILITY
        AND NOT SWITCH_PICO_INPUT_BACKEND STREQUAL "BLUEPAD32")
    message(FATAL_ERROR
            "SWITCH_PICO_ADAPTER_FEASIBILITY requires the BLUEPAD32 backend")
endif()

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(switch-pico C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Configure BLUEPAD32 input backend if selected
if(SWITCH_PICO_INPUT_BACKEND STREQUAL "BLUEPAD32")
    # Ensure Python3 is available and execute patch preparation
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    
    set(BLUEPAD32_PREP_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/tools/prepare_bluepad32.py)
    execute_process(
        COMMAND ${Python3_EXECUTABLE} ${BLUEPAD32_PREP_SCRIPT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        RESULT_VARIABLE BLUEPAD32_PREP_RESULT
        OUTPUT_VARIABLE BLUEPAD32_PREP_OUTPUT
        ERROR_VARIABLE BLUEPAD32_PREP_ERROR
    )
    
    if(NOT BLUEPAD32_PREP_RESULT EQUAL 0)
        message(FATAL_ERROR
            "Failed to prepare Bluepad32: Patch application or validation failed. "
            "Details: ${BLUEPAD32_PREP_ERROR}")
    endif()
    
    # Configure Bluepad32 include paths and subdirectory
    set(BLUEPAD32_ROOT ${CMAKE_CURRENT_LIST_DIR}/external/bluepad32)
    set(BTSTACK_ROOT ${PICO_SDK_PATH}/lib/btstack)
    include_directories(
            ${CMAKE_CURRENT_LIST_DIR}/bluepad32_config
            ${BTSTACK_ROOT}/3rd-party/bluedroid/encoder/include
            ${BTSTACK_ROOT}/3rd-party/bluedroid/decoder/include
            ${BTSTACK_ROOT}/src
    )
    add_subdirectory(
            ${BLUEPAD32_ROOT}/src/components/bluepad32
            ${CMAKE_CURRENT_BINARY_DIR}/libbluepad32
    )
endif()

# Firmware sources live under one include root and are grouped by responsibility.
set(SWITCH_PICO_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/src/firmware)

add_executable(switch-pico
        ${SWITCH_PICO_SOURCE_DIR}/main.cpp
        ${SWITCH_PICO_SOURCE_DIR}/usb/usb_output_driver.cpp
        ${SWITCH_PICO_SOURCE_DIR}/usb/switch/switch_pro_driver.cpp
        ${SWITCH_PICO_SOURCE_DIR}/usb/switch/switch_haptics.cpp
)
if(SWITCH_PICO_INPUT_BACKEND STREQUAL "BLUEPAD32")
    target_sources(switch-pico PRIVATE
            ${SWITCH_PICO_SOURCE_DIR}/adapter/adapter_host_probe.cpp
            ${SWITCH_PICO_SOURCE_DIR}/adapter/adapter_mode_controller.cpp
            ${SWITCH_PICO_SOURCE_DIR}/configuration/adapter_configuration.cpp
            ${SWITCH_PICO_SOURCE_DIR}/configuration/configuration_service.cpp
            ${SWITCH_PICO_SOURCE_DIR}/configuration/configuration_storage.cpp
            ${SWITCH_PICO_SOURCE_DIR}/configuration/configuration_transaction.cpp
            ${SWITCH_PICO_SOURCE_DIR}/core/controller_identity.cpp
            ${SWITCH_PICO_SOURCE_DIR}/input/bluepad32_input_backend.cpp
            ${SWITCH_PICO_SOURCE_DIR}/platform/pico/bootsel_pairing_button.cpp
            ${SWITCH_PICO_SOURCE_DIR}/platform/pico/pico_configuration_storage.cpp
            ${SWITCH_PICO_SOURCE_DIR}/platform/pico/pico_profile_storage.cpp
            ${SWITCH_PICO_SOURCE_DIR}/profile/controller_profile.cpp
            ${SWITCH_PICO_SOURCE_DIR}/profile/controller_profile_runtime.cpp
            ${SWITCH_PICO_SOURCE_DIR}/profile/controller_profile_transform.cpp
            ${SWITCH_PICO_SOURCE_DIR}/profile/controller_synthetic_input.cpp
            ${SWITCH_PICO_SOURCE_DIR}/profile/profile_service.cpp
            ${SWITCH_PICO_SOURCE_DIR}/profile/profile_storage.cpp
            ${SWITCH_PICO_SOURCE_DIR}/usb/generic_hid/generic_hid_driver.cpp
            ${SWITCH_PICO_SOURCE_DIR}/usb/usb_configuration_management.cpp
            ${SWITCH_PICO_SOURCE_DIR}/usb/xinput/xinput_driver.cpp
    )
    target_compile_definitions(switch-pico PRIVATE
            SWITCH_PICO_BLUEPAD32=1
            SWITCH_PICO_HID_INSTANCE_COUNT=4
            SWITCH_PICO_USB_OUTPUT_MODES=1
            PICO_FLASH_ASSUME_CORE1_SAFE=0
            PICO_BTSTACK_CYW43_MAX_HCI_PROCESS_LOOP_COUNT=16
    )
else()
    target_compile_definitions(switch-pico PRIVATE
            SWITCH_PICO_HID_INSTANCE_COUNT=1
    )
endif()

pico_set_program_name(switch-pico "switch-pico")
pico_set_program_version(switch-pico "0.2.0")

# Modify the below lines to enable/disable output over UART/USB
# UART0 is enabled for debug logging; USB stdio remains off.
pico_enable_stdio_uart(switch-pico 1)
pico_enable_stdio_usb(switch-pico 0)

# Add the standard library to the build
target_link_libraries(switch-pico
        pico_stdlib
        tinyusb_device
        tinyusb_board
        hardware_uart
        pico_rand
)
if(SWITCH_PICO_INPUT_BACKEND STREQUAL "BLUEPAD32")
    target_link_libraries(switch-pico
            bluepad32
            pico_cyw43_arch_none
            pico_btstack_ble
            pico_btstack_classic
            pico_btstack_cyw43
            pico_multicore
            pico_flash
    )
    target_link_libraries(switch-pico hardware_watchdog)
endif()

if (SWITCH_PICO_LOG)
    target_compile_definitions(switch-pico PRIVATE SWITCH_PICO_LOG=1)
endif()

# Internal includes are rooted at src/firmware. TinyUSB discovers tusb_config.h
# through the Pico platform directory.
target_include_directories(switch-pico PRIVATE
        ${SWITCH_PICO_SOURCE_DIR}
        ${SWITCH_PICO_SOURCE_DIR}/platform/pico
)

pico_add_extra_outputs(switch-pico)
