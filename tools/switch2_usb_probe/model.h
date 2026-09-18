#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifndef SWITCH2_PROBE_HUB
#define SWITCH2_PROBE_HUB 0
#endif

#if SWITCH2_PROBE_HUB != 0 && SWITCH2_PROBE_HUB != 1
#error "SWITCH2_PROBE_HUB must be 0 or 1"
#endif

#ifndef SWITCH2_PROBE_COMPOSITE
#define SWITCH2_PROBE_COMPOSITE 0
#endif

#if SWITCH2_PROBE_COMPOSITE != 0 && SWITCH2_PROBE_COMPOSITE != 1
#error "SWITCH2_PROBE_COMPOSITE must be 0 or 1"
#endif

#if SWITCH2_PROBE_HUB && SWITCH2_PROBE_COMPOSITE
#error "Native hub and composite USB backends are mutually exclusive"
#endif

#ifndef SWITCH2_PROBE_NEUTRAL_INPUT
#define SWITCH2_PROBE_NEUTRAL_INPUT 0
#endif

#if SWITCH2_PROBE_NEUTRAL_INPUT != 0 && SWITCH2_PROBE_NEUTRAL_INPUT != 1
#error "SWITCH2_PROBE_NEUTRAL_INPUT must be 0 or 1"
#endif

#if SWITCH2_PROBE_NEUTRAL_INPUT && (!SWITCH2_PROBE_HUB || defined(SWITCH_PICO_SWITCH2_USB_BRIDGE))
#error "Neutral input is only supported by the standalone native hub"
#endif

#ifndef SWITCH2_PROBE_JOYCON_LEFT
#define SWITCH2_PROBE_JOYCON_LEFT 0
#endif

#if SWITCH2_PROBE_JOYCON_LEFT != 0 && SWITCH2_PROBE_JOYCON_LEFT != 1
#error "SWITCH2_PROBE_JOYCON_LEFT must be 0 or 1"
#endif

#if SWITCH2_PROBE_COMPOSITE || SWITCH2_PROBE_HUB
#if SWITCH2_PROBE_JOYCON_LEFT
#error "Multi-controller primary must be Joy-Con 2 (R)"
#endif
#ifndef PROBE_CONTROLLER_COUNT
#define PROBE_CONTROLLER_COUNT 2
#endif
#if SWITCH2_PROBE_HUB
#if PROBE_CONTROLLER_COUNT != 2 && PROBE_CONTROLLER_COUNT != 4
#error "Native hub requires two or four controller instances"
#endif
#if PROBE_CONTROLLER_COUNT == 4 && !SWITCH2_PROBE_NEUTRAL_INPUT && !SWITCH2_BRIDGE_FULL_INPUT
#error "Two-pair hub requires full-gamepad input or explicit neutral transport"
#endif
#elif PROBE_CONTROLLER_COUNT != 2
#error "Composite output requires two controller instances"
#endif
#else
#ifndef PROBE_CONTROLLER_COUNT
#define PROBE_CONTROLLER_COUNT 1
#endif
#if PROBE_CONTROLLER_COUNT != 1
#error "Standalone requires one controller instance"
#endif
#endif

#if SWITCH2_PROBE_JOYCON_LEFT
#define PROBE_JOYCON_PID 0x2067u
#define PROBE_JOYCON_PRODUCT "Joy-Con 2 (L)"
#define PROBE_JOYCON_SIDE "left"
#define PROBE_NATIVE_REPORT_ID 0x07u
#define PROBE_IMU_LENGTH_OFFSET 14u
#define PROBE_IMU_DATA_OFFSET 15u
#else
#define PROBE_JOYCON_PID 0x2066u
#define PROBE_JOYCON_PRODUCT "Joy-Con 2 (R)"
#define PROBE_JOYCON_SIDE "right"
#define PROBE_NATIVE_REPORT_ID 0x08u
#define PROBE_IMU_LENGTH_OFFSET 15u
#define PROBE_IMU_DATA_OFFSET 16u
#endif

// Instance zero is the standalone model or the first pair's right function.
// Composite/hub instances alternate right/left, with later pairs following.
static inline bool probe_model_is_left(uint8_t instance) {
#if SWITCH2_PROBE_COMPOSITE || SWITCH2_PROBE_HUB
    return (instance & 1u) != 0;
#else
    (void)instance;
    return SWITCH2_PROBE_JOYCON_LEFT != 0;
#endif
}

static inline uint16_t probe_model_pid(uint8_t instance) {
    return probe_model_is_left(instance) ? 0x2067u : 0x2066u;
}

static inline uint8_t probe_model_report_id(uint8_t instance) {
    return probe_model_is_left(instance) ? 0x07u : 0x08u;
}

static inline uint8_t probe_model_imu_length_offset(uint8_t instance) {
    return probe_model_is_left(instance) ? 14u : 15u;
}

static inline uint8_t probe_model_imu_data_offset(uint8_t instance) {
    return probe_model_is_left(instance) ? 15u : 16u;
}
