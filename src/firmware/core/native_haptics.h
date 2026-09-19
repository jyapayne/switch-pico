#pragma once

#include <stdint.h>

// Native Switch 2 LRA samples: two independent frequency/amplitude bands.
// Frequency codes use Hz = 10 * 2^((code - 1) / 96); amplitude codes are
// unsigned linear 10-bit values. Keep the wire precision until the output
// backend selects PCM synthesis or conventional-motor approximation.
typedef struct {
    uint16_t low_frequency_code;
    uint16_t high_frequency_code;
    uint16_t low_amplitude;
    uint16_t high_amplitude;
} NativeHapticsSample;

typedef struct {
    uint8_t sample_count; // 0: no update, including no watchdog refresh; max 3.
    NativeHapticsSample samples[3];
} NativeHapticsActuatorFrame;

typedef struct {
    NativeHapticsActuatorFrame actuators[2]; // Physical left, right.
} NativeHapticsFrame;

// Existing conventional-motor approximation; do not use this for HD timing.
#define NATIVE_HAPTICS_COMPAT_FRAME_US 12000u
// Native sample spacing at 3 kHz. Existing acoustic characterization measured
// 5.27 ms +/-0.16 ms; the native sender uses ceil(count * 16 / 3) ms guards.
#define NATIVE_HAPTICS_SAMPLE_PCM_FRAMES 16u
#define NATIVE_HAPTICS_WATCHDOG_US 50000u
