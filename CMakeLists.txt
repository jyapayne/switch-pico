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
option(SWITCH_PICO_AUTOTEST "Auto-play grip-screen connection sequence on startup" OFF)
option(SWITCH_PICO_LOG "Enable UART debug logging" OFF)
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(switch-pico C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(switch-pico
        switch-pico.cpp
        switch_pro_driver.cpp
)

pico_set_program_name(switch-pico "switch-pico")
pico_set_program_version(switch-pico "0.1")

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

if (SWITCH_PICO_AUTOTEST)
    target_compile_definitions(switch-pico PRIVATE SWITCH_PICO_AUTOTEST=1)
endif()
if (SWITCH_PICO_LOG)
    target_compile_definitions(switch-pico PRIVATE SWITCH_PICO_LOG=1)
endif()

# Add the standard include files to the build
target_include_directories(switch-pico PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(switch-pico)
