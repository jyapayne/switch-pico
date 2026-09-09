#pragma once

// Radio policy, independent of the BTstack libraries linked for shared services.
#ifndef SWITCH_PICO_ENABLE_BLE
#define SWITCH_PICO_ENABLE_BLE 1
#endif
#ifndef SWITCH_PICO_ENABLE_CLASSIC
#define SWITCH_PICO_ENABLE_CLASSIC 1
#endif

#if (SWITCH_PICO_ENABLE_BLE != 0 && SWITCH_PICO_ENABLE_BLE != 1) || \
    (SWITCH_PICO_ENABLE_CLASSIC != 0 && SWITCH_PICO_ENABLE_CLASSIC != 1)
#error "Bluetooth transport selections must be 0 or 1"
#endif
#if !SWITCH_PICO_ENABLE_BLE && !SWITCH_PICO_ENABLE_CLASSIC
#error "At least one Bluetooth transport must be enabled"
#endif
