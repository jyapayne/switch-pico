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
if(SWITCH_PICO_INPUT_BACKEND STREQUAL "BLUEPAD32")
    set(SWITCH_PICO_NATIVE_DEFAULT ON)
    set(SWITCH_PICO_CLOCK_DEFAULT 300)
else()
    set(SWITCH_PICO_NATIVE_DEFAULT OFF)
    set(SWITCH_PICO_CLOCK_DEFAULT 150)
endif()
option(SWITCH_PICO_HAPTICS_EXPERIMENT
       "Enable native DualSense haptics and transport diagnostics" ${SWITCH_PICO_NATIVE_DEFAULT})
option(SWITCH_PICO_HAPTICS_EXPERIMENT_RAM
       "Execute the native haptics hot path from SRAM" ON)
option(SWITCH_PICO_HD_RUMBLE
       "Auto-arm the first eligible DualSense native haptics stream" ${SWITCH_PICO_NATIVE_DEFAULT})
set(SWITCH_PICO_SYS_CLOCK_MHZ "${SWITCH_PICO_CLOCK_DEFAULT}" CACHE STRING
    "Pico 2 W CPU clock: 300 MHz default, 150 stock or 400 opt-in")
set_property(CACHE SWITCH_PICO_SYS_CLOCK_MHZ PROPERTY STRINGS 150 300 400)
if(NOT SWITCH_PICO_SYS_CLOCK_MHZ MATCHES "^(150|300|400)$")
    message(FATAL_ERROR "SWITCH_PICO_SYS_CLOCK_MHZ must be 150, 300, or 400")
endif()
set(SWITCH_PICO_OVERCLOCK_MV "1300" CACHE STRING
    "Experimental core voltage: 1300 mV, or explicit 1400 mV at 400 MHz")
set_property(CACHE SWITCH_PICO_OVERCLOCK_MV PROPERTY STRINGS 1300 1400)
if(NOT SWITCH_PICO_OVERCLOCK_MV MATCHES "^(1300|1400)$")
    message(FATAL_ERROR "SWITCH_PICO_OVERCLOCK_MV must be 1300 or 1400")
endif()
if(SWITCH_PICO_OVERCLOCK_MV STREQUAL "1400"
        AND NOT SWITCH_PICO_SYS_CLOCK_MHZ STREQUAL "400")
    message(FATAL_ERROR "The 1400 mV experiment is restricted to 400 MHz")
endif()
option(SWITCH_PICO_CYW43_PACKET_READ
       "Use packet-level CYW43 receive transactions" ${SWITCH_PICO_NATIVE_DEFAULT})
option(SWITCH_PICO_HCI_CREDIT_BATCH
       "Batch incoming HCI credit returns" ${SWITCH_PICO_NATIVE_DEFAULT})
if(SWITCH_PICO_HD_RUMBLE)
    set(SWITCH_PICO_HAPTICS_EXPERIMENT ON)
endif()
set(SWITCH_PICO_HD_PACKET_FRAMES "64" CACHE STRING
    "Native gameplay frames per packet: 64 standard, 32 single-controller experiment")
set_property(CACHE SWITCH_PICO_HD_PACKET_FRAMES PROPERTY STRINGS 64 32)
if(NOT SWITCH_PICO_HD_PACKET_FRAMES MATCHES "^(32|64)$")
    message(FATAL_ERROR "SWITCH_PICO_HD_PACKET_FRAMES must be 32 or 64")
endif()
if(SWITCH_PICO_HD_PACKET_FRAMES STREQUAL "32")
    if(NOT SWITCH_PICO_HAPTICS_EXPERIMENT OR NOT SWITCH_PICO_CYW43_PACKET_READ
            OR NOT SWITCH_PICO_HCI_CREDIT_BATCH OR SWITCH_PICO_SYS_CLOCK_MHZ LESS 300)
        message(FATAL_ERROR "32-frame qualification requires native transport, packet reads, credit batching and at least 300 MHz")
    endif()
endif()
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
if(SWITCH_PICO_HAPTICS_EXPERIMENT
        AND NOT SWITCH_PICO_INPUT_BACKEND STREQUAL "BLUEPAD32")
    message(FATAL_ERROR
            "SWITCH_PICO_HAPTICS_EXPERIMENT requires the BLUEPAD32 backend")
endif()
if(NOT SWITCH_PICO_SYS_CLOCK_MHZ STREQUAL "150"
        AND NOT SWITCH_PICO_INPUT_BACKEND STREQUAL "BLUEPAD32")
    message(FATAL_ERROR "Experimental clocks require the Pico 2 W backend")
endif()
if(SWITCH_PICO_INPUT_BACKEND STREQUAL "BLUEPAD32")
    # Boot stage 2 must see the divider too: establish safe flash timing
    # before main raises clk_sys. Keep the CYW43 SPI bus at or below 37.5 MHz.
    math(EXPR SWITCH_PICO_FLASH_CLKDIV "(${SWITCH_PICO_SYS_CLOCK_MHZ} + 74) / 75")
    math(EXPR SWITCH_PICO_PIO_DIV256 "(${SWITCH_PICO_SYS_CLOCK_MHZ} * 256 + 74) / 75")
    math(EXPR SWITCH_PICO_PIO_DIV_INT "${SWITCH_PICO_PIO_DIV256} / 256")
    math(EXPR SWITCH_PICO_PIO_DIV_FRAC "${SWITCH_PICO_PIO_DIV256} % 256")
    add_compile_definitions(
        SWITCH_PICO_SYS_CLOCK_MHZ=${SWITCH_PICO_SYS_CLOCK_MHZ}
        SWITCH_PICO_OVERCLOCK_MV=${SWITCH_PICO_OVERCLOCK_MV}
        PICO_FLASH_SPI_CLKDIV=${SWITCH_PICO_FLASH_CLKDIV}
        CYW43_PIO_CLOCK_DIV_INT=${SWITCH_PICO_PIO_DIV_INT}
        CYW43_PIO_CLOCK_DIV_FRAC8=${SWITCH_PICO_PIO_DIV_FRAC})
    if(NOT SWITCH_PICO_SYS_CLOCK_MHZ STREQUAL "150")
        # RP2350 otherwise discards boot2 and uses the ROM's XIP setup,
        # ignoring PICO_FLASH_SPI_CLKDIV despite reporting a boot2 name.
        add_compile_definitions(PICO_EMBED_XIP_SETUP=1)
    endif()
endif()

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(switch-pico C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()
if(SWITCH_PICO_CYW43_PACKET_READ)
    if(NOT SWITCH_PICO_HAPTICS_EXPERIMENT)
        message(FATAL_ERROR "Packet-level CYW43 reads require the transport experiment")
    endif()
    find_package(Git REQUIRED)
    set(CYW43_BUS_DIR ${PICO_SDK_PATH}/src/rp2_common/pico_cyw43_driver/cybt_shared_bus)
    set(CYW43_LOCAL_DIR ${CMAKE_CURRENT_BINARY_DIR}/cyw43-packet-read)
    set(CYW43_STATUS_PATCH ${CMAKE_CURRENT_LIST_DIR}/patches/cyw43-packet-read-status.patch)
    file(MAKE_DIRECTORY ${CYW43_LOCAL_DIR})
    configure_file(${CYW43_BUS_DIR}/cybt_shared_bus_driver.c
                   ${CYW43_LOCAL_DIR}/cybt_shared_bus_driver.c COPYONLY)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CYW43_STATUS_PATCH})
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E env
                GIT_CEILING_DIRECTORIES=${CMAKE_CURRENT_BINARY_DIR}
                ${GIT_EXECUTABLE} apply --no-index ${CYW43_STATUS_PATCH}
        WORKING_DIRECTORY ${CYW43_LOCAL_DIR}
        RESULT_VARIABLE CYW43_PATCH_RESULT ERROR_VARIABLE CYW43_PATCH_ERROR)
    if(NOT CYW43_PATCH_RESULT EQUAL 0)
        message(FATAL_ERROR "Cannot prepare packet-read driver: ${CYW43_PATCH_ERROR}")
    endif()
    get_target_property(CYW43_BUS_SOURCES cybt_shared_bus INTERFACE_SOURCES)
    list(FIND CYW43_BUS_SOURCES ${CYW43_BUS_DIR}/cybt_shared_bus_driver.c CYW43_DRIVER_INDEX)
    if(CYW43_DRIVER_INDEX EQUAL -1)
        message(FATAL_ERROR "SDK shared-bus source layout changed")
    endif()
    list(REMOVE_ITEM CYW43_BUS_SOURCES ${CYW43_BUS_DIR}/cybt_shared_bus_driver.c)
    list(APPEND CYW43_BUS_SOURCES ${CYW43_LOCAL_DIR}/cybt_shared_bus_driver.c)
    set_property(TARGET cybt_shared_bus PROPERTY INTERFACE_SOURCES ${CYW43_BUS_SOURCES})
endif()
if(SWITCH_PICO_HCI_CREDIT_BATCH)
    if(NOT SWITCH_PICO_HAPTICS_EXPERIMENT)
        message(FATAL_ERROR "HCI credit batching requires the transport experiment")
    endif()
    find_package(Git REQUIRED)
    set(HCI_LOCAL_ROOT ${CMAKE_CURRENT_BINARY_DIR}/btstack-credit-batch)
    set(HCI_LOCAL_SOURCE ${HCI_LOCAL_ROOT}/lib/btstack/src/hci.c)
    set(HCI_BATCH_PATCH ${CMAKE_CURRENT_LIST_DIR}/patches/btstack-credit-batch.patch)
    file(MAKE_DIRECTORY ${HCI_LOCAL_ROOT}/lib/btstack/src)
    configure_file(${PICO_BTSTACK_PATH}/src/hci.c ${HCI_LOCAL_SOURCE} COPYONLY)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${HCI_BATCH_PATCH})
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E env
                GIT_CEILING_DIRECTORIES=${CMAKE_CURRENT_BINARY_DIR}
                ${GIT_EXECUTABLE} apply --no-index ${HCI_BATCH_PATCH}
        WORKING_DIRECTORY ${HCI_LOCAL_ROOT}
        RESULT_VARIABLE HCI_PATCH_RESULT ERROR_VARIABLE HCI_PATCH_ERROR)
    if(NOT HCI_PATCH_RESULT EQUAL 0)
        message(FATAL_ERROR "Cannot prepare credit-batch HCI: ${HCI_PATCH_ERROR}")
    endif()
    get_target_property(HCI_SOURCES pico_btstack_base INTERFACE_SOURCES)
    list(FIND HCI_SOURCES ${PICO_BTSTACK_PATH}/src/hci.c HCI_SOURCE_INDEX)
    if(HCI_SOURCE_INDEX EQUAL -1)
        message(FATAL_ERROR "SDK HCI source layout changed")
    endif()
    list(REMOVE_ITEM HCI_SOURCES ${PICO_BTSTACK_PATH}/src/hci.c)
    list(APPEND HCI_SOURCES ${HCI_LOCAL_SOURCE})
    set_property(TARGET pico_btstack_base PROPERTY INTERFACE_SOURCES ${HCI_SOURCES})
endif()

# Configure BLUEPAD32 input backend if selected
if(SWITCH_PICO_INPUT_BACKEND STREQUAL "BLUEPAD32")
    find_package(Python3 REQUIRED COMPONENTS Interpreter)

    set(BLUEPAD32_PREP_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/tools/prepare_bluepad32.py)
    set(BLUEPAD32_SOURCE ${CMAKE_CURRENT_LIST_DIR}/external/bluepad32)
    set(BLUEPAD32_PATCH ${CMAKE_CURRENT_LIST_DIR}/patches/bluepad32-sdl3-imu.patch)
    set(BLUEPAD32_ROOT ${CMAKE_CURRENT_BINARY_DIR}/_deps/bluepad32-src)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        ${BLUEPAD32_PREP_SCRIPT} ${BLUEPAD32_PATCH})
    execute_process(
        COMMAND ${Python3_EXECUTABLE} ${BLUEPAD32_PREP_SCRIPT}
                --bluepad32 ${BLUEPAD32_SOURCE}
                --patch ${BLUEPAD32_PATCH}
                --output ${BLUEPAD32_ROOT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        RESULT_VARIABLE BLUEPAD32_PREP_RESULT
        OUTPUT_VARIABLE BLUEPAD32_PREP_OUTPUT
        ERROR_VARIABLE BLUEPAD32_PREP_ERROR
    )

    if(NOT BLUEPAD32_PREP_RESULT EQUAL 0)
        message(FATAL_ERROR
            "Failed to prepare isolated Bluepad32 source. "
            "Details: ${BLUEPAD32_PREP_ERROR}")
    endif()

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
    target_sources(bluepad32 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/bluepad32_config/parser/uni_hid_parser_switch2.c
        ${CMAKE_CURRENT_LIST_DIR}/bluepad32_config/parser/uni_switch2_pairing.c
        ${CMAKE_CURRENT_LIST_DIR}/bluepad32_config/parser/uni_switch2_haptics.c)
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
            ${SWITCH_PICO_SOURCE_DIR}/input/switch_native_output.cpp
            ${SWITCH_PICO_SOURCE_DIR}/input/native_output_scheduler.cpp
            ${SWITCH_PICO_SOURCE_DIR}/usb/switch/switch_native_haptics.cpp
            ${SWITCH_PICO_SOURCE_DIR}/input/controller_macro_capture.cpp
            ${SWITCH_PICO_SOURCE_DIR}/input/switch2_wake.cpp
            ${SWITCH_PICO_SOURCE_DIR}/platform/pico/bootsel_pairing_button.cpp
            ${SWITCH_PICO_SOURCE_DIR}/platform/pico/system_clock.cpp
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
    set(SWITCH2_WAKE_CONFIG
        ${SWITCH_PICO_SOURCE_DIR}/platform/pico/switch2_wake_config.h)
    set_property(
        DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        ${SWITCH2_WAKE_CONFIG})
    if(EXISTS ${SWITCH2_WAKE_CONFIG})
        set(SWITCH2_WAKE_CONFIGURED_VALUE 1)
    else()
        set(SWITCH2_WAKE_CONFIGURED_VALUE 0)
    endif()
    # The experiment yields after one received packet; its transport wrapper
    # explicitly reschedules remaining input so timers run between packets.
    target_compile_definitions(switch-pico PRIVATE
            SWITCH_PICO_BLUEPAD32=1
            SWITCH_PICO_NATIVE_SWITCH_RUMBLE=1
            SWITCH_PICO_HID_INSTANCE_COUNT=4
            SWITCH_PICO_USB_OUTPUT_MODES=1
            PICO_FLASH_ASSUME_CORE1_SAFE=0
            PICO_STACK_SIZE=4096
            PICO_BTSTACK_CYW43_MAX_HCI_PROCESS_LOOP_COUNT=$<IF:$<BOOL:${SWITCH_PICO_HAPTICS_EXPERIMENT}>,1,16>
            SWITCH2_WAKE_CONFIGURED=${SWITCH2_WAKE_CONFIGURED_VALUE}
    )
    # GATT's automatic SMP retry is incompatible with Switch 2's proprietary
    # pairing. Reject only those links; retain normal SMP for every other pad.
    target_link_options(switch-pico PRIVATE "LINKER:--wrap=sm_request_pairing")
    if(SWITCH_PICO_CYW43_PACKET_READ)
        target_sources(switch-pico PRIVATE
            ${SWITCH_PICO_SOURCE_DIR}/platform/pico/cyw43_packet_transport.c)
        target_compile_definitions(switch-pico PRIVATE SWITCH_PICO_CYW43_PACKET_READ=1)
        target_link_options(switch-pico PRIVATE
            "LINKER:--wrap=cyw43_btbus_read" "LINKER:--wrap=cyw43_btbus_init")
    endif()
    if(SWITCH_PICO_HCI_CREDIT_BATCH)
        target_compile_definitions(switch-pico PRIVATE SWITCH_PICO_HCI_CREDIT_BATCH=1)
    endif()
    if(SWITCH_PICO_HAPTICS_EXPERIMENT)
        target_sources(switch-pico PRIVATE
                ${SWITCH_PICO_SOURCE_DIR}/input/haptics_experiment.cpp
                ${SWITCH_PICO_SOURCE_DIR}/input/switch_hd_rumble_synth.cpp
                ${SWITCH_PICO_SOURCE_DIR}/input/haptics_transport_probe.cpp)
        target_compile_definitions(switch-pico PRIVATE
                SWITCH_PICO_HAPTICS_EXPERIMENT=1
                SWITCH_PICO_HD_PACKET_FRAMES=${SWITCH_PICO_HD_PACKET_FRAMES}
                SWITCH_PICO_HAPTICS_EXPERIMENT_RAM=$<BOOL:${SWITCH_PICO_HAPTICS_EXPERIMENT_RAM}>)
        target_compile_definitions(bluepad32 PRIVATE
                SWITCH_PICO_HAPTICS_EXPERIMENT=1)
        # These boundaries are defined in different SDK translation units from
        # their callers. GNU ld --wrap intercepts unresolved symbol references;
        # an LTO build must verify actual caller edges, not just wrapper symbols.
        target_link_options(switch-pico PRIVATE
                "LINKER:--wrap=cyw43_bluetooth_hci_write"
                "LINKER:--wrap=cyw43_bluetooth_hci_read"
                "LINKER:--wrap=btstack_run_loop_base_poll_data_sources")
    endif()
    if(SWITCH_PICO_HD_RUMBLE)
        target_compile_definitions(switch-pico PRIVATE SWITCH_PICO_HD_RUMBLE=1)
    endif()
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
    target_link_libraries(switch-pico hardware_watchdog hardware_adc hardware_vreg hardware_powman)
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
