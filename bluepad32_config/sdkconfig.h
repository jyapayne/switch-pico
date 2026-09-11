#pragma once

#include "bluetooth_transport_config.h"

// The AIO firmware exposes one fixed Bluepad32 device slot per USB interface.
#define CONFIG_BLUEPAD32_MAX_DEVICES 4
#define CONFIG_BLUEPAD32_MAX_ALLOWLIST 4
#define CONFIG_BLUEPAD32_GAP_SECURITY 1
#define CONFIG_BLUEPAD32_ENABLE_BLE_BY_DEFAULT SWITCH_PICO_ENABLE_BLE

#define CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#define CONFIG_TARGET_PICO_W

// 2 == Info
#define CONFIG_BLUEPAD32_LOG_LEVEL 2

// Standard Wii camera presets; native diagnostic builds may select level 2.
#ifndef SWITCH_PICO_WII_IR_SENSITIVITY_LEVEL
#define SWITCH_PICO_WII_IR_SENSITIVITY_LEVEL 3
#endif
#if SWITCH_PICO_WII_IR_SENSITIVITY_LEVEL != 2 && SWITCH_PICO_WII_IR_SENSITIVITY_LEVEL != 3
#error "Wii IR sensitivity must be standard preset 2 or 3"
#endif
