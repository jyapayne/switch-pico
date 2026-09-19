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
option(SWITCH_PICO_WII_IR_MOUSE
       "Experiment: Wii IR as a composite USB mouse with Switch controllers" OFF)
option(SWITCH_PICO_WII_IR_GYRO
       "Experiment: selectable Wii IR or MotionPlus gyro aiming" OFF)
option(SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
       "Experiment: capture Joy-Con 2 mouse reports through USB management" OFF)
option(SWITCH_PICO_SWITCH2_MOUSE_CAPTURE_NATIVE
       "Capture native Joy-Con mouse reports instead of common controller reports" OFF)
option(SWITCH_PICO_SWITCH2_MEMORY_CAPTURE
       "Research: log read-only Joy-Con factory/user calibration banks during setup" OFF)
option(SWITCH_PICO_SWITCH2_USB_BRIDGE
       "Experiment: forward a selected native Joy-Con 2 through the verified USB probe" OFF)
option(SWITCH2_PROBE_COMPOSITE
       "Experiment: independent right and left Joy-Con 2 functions on one USB port" OFF)
option(SWITCH2_PROBE_HUB
       "Native Joy-Con 2 R/L devices behind a stock-socket SIO USB hub" OFF)
if(SWITCH2_PROBE_HUB AND (NOT SWITCH_PICO_SWITCH2_USB_BRIDGE OR SWITCH2_PROBE_COMPOSITE))
    message(FATAL_ERROR "Native hub requires the Switch2 bridge and excludes composite mode")
endif()
if(SWITCH2_PROBE_COMPOSITE AND NOT SWITCH_PICO_SWITCH2_USB_BRIDGE)
    message(FATAL_ERROR "The composite Joy-Con 2 experiment requires SWITCH_PICO_SWITCH2_USB_BRIDGE")
endif()
if(NOT SWITCH_PICO_SWITCH2_USB_BRIDGE)
    add_compile_definitions(SWITCH2_PROBE_COMPOSITE=0 PROBE_CONTROLLER_COUNT=1)
endif()
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
if(SWITCH_PICO_HD_RUMBLE)
    set(SWITCH_PICO_HAPTICS_EXPERIMENT ON)
endif()
if(SWITCH2_PROBE_HUB)
    if(SWITCH_PICO_HAPTICS_EXPERIMENT)
        set(SWITCH_PICO_CLOCK_DEFAULT 300)
    else()
        set(SWITCH_PICO_CLOCK_DEFAULT 240)
    endif()
endif()
set(SWITCH_PICO_SYS_CLOCK_MHZ "${SWITCH_PICO_CLOCK_DEFAULT}" CACHE STRING
    "Pico 2 W CPU clock: 300 MHz default, 150 stock or 400 opt-in")
set_property(CACHE SWITCH_PICO_SYS_CLOCK_MHZ PROPERTY STRINGS 150 240 300 400)
if(NOT SWITCH_PICO_SYS_CLOCK_MHZ MATCHES "^(150|240|300|400)$")
    message(FATAL_ERROR "SWITCH_PICO_SYS_CLOCK_MHZ must be 150, 240, 300, or 400")
endif()
if(SWITCH2_PROBE_HUB AND NOT SWITCH_PICO_SYS_CLOCK_MHZ MATCHES "^(240|300)$")
    message(FATAL_ERROR "Native SIO hub requires SWITCH_PICO_SYS_CLOCK_MHZ=240 or 300")
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
option(SWITCH_PICO_HCI_CREDIT_BUFFER
       "Return receive credits independently of the ACL packet buffer" ${SWITCH_PICO_HCI_CREDIT_BATCH})
if(SWITCH_PICO_HCI_CREDIT_BUFFER AND NOT SWITCH_PICO_HCI_CREDIT_BATCH)
    message(FATAL_ERROR "The dedicated HCI credit buffer requires credit batching")
endif()
set(SWITCH_PICO_HD_PACKET_FRAMES "32" CACHE STRING
    "Native stereo frames per packet: only the qualified 32-frame format is supported")
set_property(CACHE SWITCH_PICO_HD_PACKET_FRAMES PROPERTY STRINGS 32)
if(NOT SWITCH_PICO_HD_PACKET_FRAMES STREQUAL "32")
    message(FATAL_ERROR "Native haptics requires 32-frame packets; the non-working 64-frame format is unsupported")
endif()
if(SWITCH_PICO_INPUT_BACKEND STREQUAL "BLUEPAD32" AND SWITCH_PICO_HAPTICS_EXPERIMENT)
    if(NOT SWITCH_PICO_CYW43_PACKET_READ OR NOT SWITCH_PICO_HCI_CREDIT_BATCH
            OR SWITCH_PICO_SYS_CLOCK_MHZ LESS 300)
        message(FATAL_ERROR "32-frame native transport requires packet reads, credit batching and at least 300 MHz")
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
set(SWITCH_PICO_BLUETOOTH_MODE "MIXED" CACHE STRING "Bluetooth radio mode: MIXED, BLE, or CLASSIC")
set_property(CACHE SWITCH_PICO_BLUETOOTH_MODE PROPERTY STRINGS MIXED BLE CLASSIC)
if(NOT SWITCH_PICO_BLUETOOTH_MODE MATCHES "^(MIXED|BLE|CLASSIC)$")
    message(FATAL_ERROR "SWITCH_PICO_BLUETOOTH_MODE must be MIXED, BLE, or CLASSIC")
endif()
if(SWITCH_PICO_INPUT_BACKEND STREQUAL "BLUEPAD32")
    if(SWITCH_PICO_BLUETOOTH_MODE STREQUAL "CLASSIC")
        set(SWITCH_PICO_ENABLE_BLE 0)
    else()
        set(SWITCH_PICO_ENABLE_BLE 1)
    endif()
    if(SWITCH_PICO_BLUETOOTH_MODE STREQUAL "BLE")
        set(SWITCH_PICO_ENABLE_CLASSIC 0)
    else()
        set(SWITCH_PICO_ENABLE_CLASSIC 1)
    endif()
    # Directory scope also covers Bluepad32 and its build-local SDK sources.
    add_compile_definitions(
        SWITCH_PICO_ENABLE_BLE=${SWITCH_PICO_ENABLE_BLE}
        SWITCH_PICO_ENABLE_CLASSIC=${SWITCH_PICO_ENABLE_CLASSIC})
elseif(NOT SWITCH_PICO_BLUETOOTH_MODE STREQUAL "MIXED")
    message(FATAL_ERROR "Single-transport Bluetooth modes require the BLUEPAD32 backend")
endif()
if(SWITCH_PICO_WII_IR_MOUSE AND SWITCH_PICO_WII_IR_GYRO)
    message(FATAL_ERROR "Select either Wii IR mouse or selectable gyro output")
endif()
if(SWITCH_PICO_WII_IR_GYRO)
    add_compile_definitions(SWITCH_PICO_WII_IR_GYRO=1)
endif()
if(SWITCH_PICO_WII_IR_MOUSE OR SWITCH_PICO_WII_IR_GYRO)
    if(NOT SWITCH_PICO_INPUT_BACKEND STREQUAL "BLUEPAD32"
            OR NOT SWITCH_PICO_ENABLE_CLASSIC)
        message(FATAL_ERROR "Wii IR requires BLUEPAD32 with Classic Bluetooth")
    endif()
    add_compile_definitions(SWITCH_PICO_WII_IR=1)
endif()
if(SWITCH_PICO_WII_IR_MOUSE)
    add_compile_definitions(SWITCH_PICO_WII_IR_MOUSE=1)
endif()
if(SWITCH_PICO_SWITCH2_MOUSE_CAPTURE)
    if(NOT SWITCH_PICO_INPUT_BACKEND STREQUAL "BLUEPAD32"
            OR NOT SWITCH_PICO_ENABLE_BLE)
        message(FATAL_ERROR "Joy-Con mouse capture requires BLUEPAD32 with BLE")
    endif()
    add_compile_definitions(
        SWITCH_PICO_SWITCH2_MOUSE_CAPTURE=1
        SWITCH_PICO_SWITCH2_MOUSE_CAPTURE_NATIVE=$<BOOL:${SWITCH_PICO_SWITCH2_MOUSE_CAPTURE_NATIVE}>)
endif()
if(SWITCH_PICO_SWITCH2_MEMORY_CAPTURE)
    if(NOT SWITCH_PICO_SWITCH2_MOUSE_CAPTURE OR NOT SWITCH_PICO_LOG
            OR SWITCH_PICO_SWITCH2_USB_BRIDGE)
        message(FATAL_ERROR
            "Joy-Con memory capture requires a logging capture build, not the USB bridge")
    endif()
    add_compile_definitions(SWITCH_PICO_SWITCH2_MEMORY_CAPTURE=1)
endif()
set(SWITCH2_BRIDGE_WII_INPUT OFF)
set(SWITCH2_BRIDGE_DUALSENSE_INPUT OFF)
set(SWITCH2_BRIDGE_GAMEPAD_INPUT OFF)
set(SWITCH2_BRIDGE_FULL_INPUT OFF)
if(SWITCH_PICO_SWITCH2_USB_BRIDGE)
    set(SWITCH2_BRIDGE_INPUT "JOYCON2" CACHE STRING "Native source: JOYCON2, WII, DUALSENSE or GAMEPAD")
    set_property(CACHE SWITCH2_BRIDGE_INPUT PROPERTY STRINGS JOYCON2 WII DUALSENSE GAMEPAD)
    set(SWITCH2_BRIDGE_IMU_TARGET "BOTH" CACHE STRING "Translated native-pair IMU target: LEFT, RIGHT or BOTH")
    set_property(CACHE SWITCH2_BRIDGE_IMU_TARGET PROPERTY STRINGS LEFT RIGHT BOTH)
    if(SWITCH2_BRIDGE_IMU_TARGET STREQUAL "RIGHT")
        set(SWITCH2_BRIDGE_IMU_TARGET_MASK 1)
    elseif(SWITCH2_BRIDGE_IMU_TARGET STREQUAL "LEFT")
        set(SWITCH2_BRIDGE_IMU_TARGET_MASK 2)
    elseif(SWITCH2_BRIDGE_IMU_TARGET STREQUAL "BOTH")
        set(SWITCH2_BRIDGE_IMU_TARGET_MASK 3)
    else()
        message(FATAL_ERROR "SWITCH2_BRIDGE_IMU_TARGET must be LEFT, RIGHT or BOTH")
    endif()
    add_compile_definitions(SWITCH2_BRIDGE_IMU_TARGET_MASK=${SWITCH2_BRIDGE_IMU_TARGET_MASK})
    if(NOT SWITCH_PICO_INPUT_BACKEND STREQUAL "BLUEPAD32")
        message(FATAL_ERROR "Switch 2 USB bridge requires BLUEPAD32")
    endif()
    if(SWITCH2_BRIDGE_INPUT STREQUAL "DUALSENSE" OR SWITCH2_BRIDGE_INPUT STREQUAL "GAMEPAD")
        if(NOT SWITCH2_PROBE_HUB)
            message(FATAL_ERROR "Translated full-controller input requires the native R/L HUB")
        endif()
        set(SWITCH2_BRIDGE_FULL_INPUT ON)
        add_compile_definitions(SWITCH2_BRIDGE_FULL_INPUT=1)
        if(SWITCH2_BRIDGE_INPUT STREQUAL "DUALSENSE")
            if(NOT SWITCH_PICO_ENABLE_CLASSIC)
                message(FATAL_ERROR "DualSense input requires Classic Bluetooth")
            endif()
            set(SWITCH2_BRIDGE_DUALSENSE_INPUT ON)
            add_compile_definitions(SWITCH2_BRIDGE_DUALSENSE_INPUT=1)
        else()
            set(SWITCH2_BRIDGE_GAMEPAD_INPUT ON)
            add_compile_definitions(SWITCH2_BRIDGE_GAMEPAD_INPUT=1)
        endif()
    elseif(NOT SWITCH_PICO_ENABLE_BLE OR NOT SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
            OR NOT SWITCH_PICO_SWITCH2_MOUSE_CAPTURE_NATIVE)
        message(FATAL_ERROR "Joy-Con/Wii USB bridge requires BLE and native Joy-Con capture")
    endif()
    if(SWITCH_PICO_WII_IR_MOUSE OR SWITCH_PICO_WII_IR_GYRO)
        message(FATAL_ERROR "Switch 2 USB bridge owns USB; Wii IR USB experiments cannot be combined")
    endif()
    if(SWITCH2_BRIDGE_INPUT STREQUAL "WII")
        if(NOT SWITCH_PICO_ENABLE_CLASSIC)
            message(FATAL_ERROR "Wii native bridge requires Bluetooth MIXED mode")
        endif()
        set(SWITCH2_BRIDGE_WII_INPUT ON)
        add_compile_definitions(SWITCH2_BRIDGE_WII_INPUT=1 SWITCH_PICO_WII_IR=1)
    elseif(NOT SWITCH2_BRIDGE_INPUT STREQUAL "JOYCON2" AND NOT SWITCH2_BRIDGE_FULL_INPUT)
        message(FATAL_ERROR "SWITCH2_BRIDGE_INPUT must be JOYCON2, WII, DUALSENSE or GAMEPAD")
    endif()
    if(NOT SWITCH2_BRIDGE_FULL_INPUT AND NOT SWITCH2_BRIDGE_IMU_TARGET STREQUAL "BOTH")
        message(FATAL_ERROR "LEFT/RIGHT IMU routing requires a translated full-controller native pair")
    endif()
    add_compile_definitions(SWITCH_PICO_SWITCH2_USB_BRIDGE=1)
endif()
set(PICO_BOARD pico CACHE STRING "Board type")
if(SWITCH_PICO_INPUT_BACKEND STREQUAL "BLUEPAD32"
        AND NOT PICO_BOARD STREQUAL "pico2_w")
    message(FATAL_ERROR
            "SWITCH_PICO_INPUT_BACKEND=BLUEPAD32 requires PICO_BOARD=pico2_w")
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
if(SWITCH_PICO_SWITCH2_USB_BRIDGE)
    include(${CMAKE_CURRENT_LIST_DIR}/tools/switch2_usb_probe/probe_build.cmake)
endif()

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
    if(SWITCH2_PROBE_HUB)
        # Upstream Bluepad32 also links arch_none (background IRQ execution).
        # Replace that transitive selection, not just the executable's link.
        foreach(property LINK_LIBRARIES INTERFACE_LINK_LIBRARIES)
            get_target_property(bluepad_links bluepad32 ${property})
            if(bluepad_links)
                list(REMOVE_ITEM bluepad_links pico_cyw43_arch_none)
                list(APPEND bluepad_links pico_cyw43_arch_poll)
                set_property(TARGET bluepad32 PROPERTY ${property} "${bluepad_links}")
            endif()
        endforeach()
        target_compile_definitions(bluepad32 PUBLIC CYW43_LWIP=0)
    endif()
    target_sources(bluepad32 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/bluepad32_config/parser/uni_hid_parser_switch2.c
        ${CMAKE_CURRENT_LIST_DIR}/bluepad32_config/parser/uni_switch2_pairing.c
        ${CMAKE_CURRENT_LIST_DIR}/bluepad32_config/parser/uni_switch2_haptics.c)
    if(SWITCH2_BRIDGE_FULL_INPUT)
        target_sources(bluepad32 PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}/bluepad32_config/parser/uni_hid_parser_native_motion.c)
    endif()
endif()

# Firmware sources live under one include root and are grouped by responsibility.
set(SWITCH_PICO_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/src/firmware)

add_executable(switch-pico
        ${SWITCH_PICO_SOURCE_DIR}/usb/usb_output_driver.cpp
        ${SWITCH_PICO_SOURCE_DIR}/usb/switch/switch_pro_driver.cpp
        ${SWITCH_PICO_SOURCE_DIR}/usb/switch/switch_haptics.cpp
)
if(SWITCH_PICO_SWITCH2_USB_BRIDGE)
    switch2_usb_probe_configure(switch-pico)
    if(NOT SWITCH2_PROBE_USB_INIT OR NOT SWITCH2_PROBE_FACTORY_FILE
            OR NOT SWITCH2_PROBE_USER_CALIBRATION_FILE)
        message(FATAL_ERROR "Switch 2 USB bridge requires the initialized probe with factory and user calibration captures")
    endif()
    target_sources(switch-pico PRIVATE
        ${SWITCH2_USB_PROBE_DIR}/bootsel.cpp
        ${SWITCH2_USB_PROBE_DIR}/controller_input.cpp)
    if(SWITCH2_BRIDGE_WII_INPUT)
        set(LIBOGC_IR_PREP_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/tools/prepare_libogc_ir.py)
        set(LIBOGC_IR_GENERATED ${CMAKE_CURRENT_BINARY_DIR}/libogc_ir.c)
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
            ${LIBOGC_IR_PREP_SCRIPT}
            ${CMAKE_CURRENT_LIST_DIR}/external/libogc_ir/UPSTREAM.json
            ${CMAKE_CURRENT_LIST_DIR}/external/libogc_ir/ir.h
            ${CMAKE_CURRENT_LIST_DIR}/external/libogc_ir/license_libogc.txt
            ${CMAKE_CURRENT_LIST_DIR}/external/libogc_ir/libogc_license.txt
            ${CMAKE_CURRENT_LIST_DIR}/external/libogc_ir/upstream/ir.c
            ${CMAKE_CURRENT_LIST_DIR}/external/libogc_ir/upstream/ir.h
            ${CMAKE_CURRENT_LIST_DIR}/external/libogc_ir/upstream/wiiuse.h
            ${CMAKE_CURRENT_LIST_DIR}/external/libogc_ir/upstream/wiiuse_internal.h
            ${CMAKE_CURRENT_LIST_DIR}/external/libogc_ir/upstream/definitions.h)
        execute_process(
            COMMAND ${Python3_EXECUTABLE} ${LIBOGC_IR_PREP_SCRIPT} --output ${LIBOGC_IR_GENERATED}
            RESULT_VARIABLE LIBOGC_IR_PREP_RESULT)
        if(NOT LIBOGC_IR_PREP_RESULT EQUAL 0)
            message(FATAL_ERROR "Preparing the pinned libogc IR pipeline failed")
        endif()
        target_include_directories(switch-pico PRIVATE ${CMAKE_CURRENT_LIST_DIR}/external)
        target_sources(switch-pico PRIVATE ${SWITCH2_USB_PROBE_DIR}/native_imu.cpp ${LIBOGC_IR_GENERATED})
    elseif(SWITCH2_BRIDGE_FULL_INPUT)
        target_sources(switch-pico PRIVATE
            ${SWITCH2_USB_PROBE_DIR}/native_gamepad_input.cpp
            ${SWITCH2_USB_PROBE_DIR}/native_imu.cpp)
    endif()
    if(NOT SWITCH2_BRIDGE_SOURCE_AUTO)
        target_compile_definitions(switch-pico PRIVATE
            SWITCH2_BRIDGE_SOURCE_ADDRESS_BYTES=${SWITCH2_BRIDGE_SOURCE_ADDRESS_BYTES})
    endif()
    if((SWITCH2_PROBE_COMPOSITE OR SWITCH2_PROBE_HUB) AND
            (NOT SWITCH2_BRIDGE_FULL_INPUT OR
             (probe_controller_count GREATER 2 AND NOT SWITCH2_BRIDGE_SECOND_SOURCE_AUTO)))
        target_compile_definitions(switch-pico PRIVATE
            SWITCH2_BRIDGE_SECOND_SOURCE_ADDRESS_BYTES=${SWITCH2_BRIDGE_SECOND_SOURCE_ADDRESS_BYTES})
    endif()
else()
    target_sources(switch-pico PRIVATE ${SWITCH_PICO_SOURCE_DIR}/main.cpp)
endif()
if(SWITCH_PICO_WII_IR_MOUSE OR SWITCH_PICO_WII_IR_GYRO OR SWITCH2_BRIDGE_WII_INPUT)
    target_sources(switch-pico PRIVATE ${SWITCH_PICO_SOURCE_DIR}/input/wii_ir_pointer.cpp)
    if(NOT SWITCH2_BRIDGE_WII_INPUT)
        target_sources(switch-pico PRIVATE ${SWITCH_PICO_SOURCE_DIR}/input/wii_ir_tracker.cpp)
    endif()
endif()
if(SWITCH_PICO_WII_IR_MOUSE)
    target_sources(switch-pico PRIVATE
        ${SWITCH_PICO_SOURCE_DIR}/usb/wii_ir_mouse_usb.cpp)
endif()
if(SWITCH_PICO_SWITCH2_MOUSE_CAPTURE)
    target_sources(switch-pico PRIVATE
        ${SWITCH_PICO_SOURCE_DIR}/input/switch2_mouse_capture.cpp)
endif()
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
            ${SWITCH_PICO_SOURCE_DIR}/input/wii_swing.cpp
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
            $<$<NOT:$<OR:$<BOOL:${SWITCH_PICO_WII_IR_MOUSE}>,$<BOOL:${SWITCH_PICO_WII_IR_GYRO}>>>:SWITCH_PICO_USB_OUTPUT_MODES=1>
            PICO_FLASH_ASSUME_CORE1_SAFE=$<BOOL:${SWITCH2_PROBE_HUB}>
            PICO_STACK_SIZE=$<IF:$<BOOL:${SWITCH2_PROBE_HUB}>,16384,4096>
            PICO_CORE1_STACK_SIZE=4096
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
    if(SWITCH_PICO_HCI_CREDIT_BUFFER)
        target_compile_definitions(switch-pico PRIVATE SWITCH_PICO_HCI_CREDIT_BUFFER=1)
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

if(NOT SWITCH_PICO_SWITCH2_USB_BRIDGE)
    pico_set_program_name(switch-pico "switch-pico")
    pico_set_program_version(switch-pico "0.2.0")
endif()

# Modify the below lines to enable/disable output over UART/USB
# UART0 is enabled for debug logging; USB stdio remains off.
pico_enable_stdio_uart(switch-pico 1)
pico_enable_stdio_usb(switch-pico 0)

# Add the standard library to the build
target_link_libraries(switch-pico PRIVATE
        pico_stdlib
        hardware_uart
        pico_rand
)
if(NOT SWITCH2_PROBE_HUB)
    target_link_libraries(switch-pico PRIVATE tinyusb_device tinyusb_board)
endif()
if(SWITCH_PICO_INPUT_BACKEND STREQUAL "BLUEPAD32")
    if(SWITCH2_PROBE_HUB)
        target_link_libraries(switch-pico PRIVATE pico_cyw43_arch_poll)
        target_compile_definitions(switch-pico PRIVATE CYW43_LWIP=0)
    else()
        target_link_libraries(switch-pico PRIVATE pico_cyw43_arch_none)
    endif()
    target_link_libraries(switch-pico PRIVATE
            bluepad32
            pico_btstack_ble
            pico_btstack_classic
            pico_btstack_cyw43
            pico_multicore
            pico_flash
    )
    target_link_libraries(switch-pico PRIVATE hardware_watchdog hardware_adc hardware_vreg hardware_powman)
endif()

if (SWITCH_PICO_LOG)
    target_compile_definitions(switch-pico PRIVATE SWITCH_PICO_LOG=1)
endif()

# Internal includes are rooted at src/firmware. Ordinary builds discover
# tusb_config.h here; the bridge selects the probe config explicitly.
target_include_directories(switch-pico PRIVATE
        ${SWITCH_PICO_SOURCE_DIR}
        ${SWITCH_PICO_SOURCE_DIR}/platform/pico
)

pico_add_extra_outputs(switch-pico)
