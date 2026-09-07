// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define UNI_SWITCH2_HAPTICS_MAX_SAMPLES 3
#define UNI_SWITCH2_HAPTICS_MAX_AMPLITUDE 453
#define UNI_SWITCH2_HAPTICS_WATCHDOG_MS 50

typedef struct {
    uint8_t count;  // Zero leaves this side's host state/watchdog untouched.
    uint8_t samples[UNI_SWITCH2_HAPTICS_MAX_SAMPLES][5];
} uni_switch2_haptics_side_t;
typedef struct {
    uni_switch2_haptics_side_t sides[2];  // Physical left/right; Joy-Con uses side 0.
} uni_switch2_haptics_frame_t;

// Source indices64 mean160Hz(low)/320Hz(high); amplitudes are linear Q0.15.
// Measured native frequency law: Hz ~=10*2^((code-1)/96), hence exact index mapping.
// Linear amplitude scaling uses SDL's29000/65535 safety envelope (10-bit max453).
void uni_switch2_haptics_encode_sample(uint8_t out[5], uint8_t low_index, uint8_t high_index,
                                      uint16_t low_q15, uint16_t high_q15);
void uni_switch2_haptics_silence(uni_switch2_haptics_frame_t* frame);
bool uni_switch2_haptics_valid(const uni_switch2_haptics_frame_t* frame);
bool uni_switch2_haptics_is_stop(const uni_switch2_haptics_frame_t* frame);
// Native callers validate the host envelope separately; conventional feedback
// retains its existing full10-bit amplitude codes.
// Requires count1..3; unused slots are zero, header selects only the valid samples.
bool uni_switch2_haptics_write_block(uint8_t out[16], const uni_switch2_haptics_side_t* side,
                                    uint8_t sequence);
#ifdef __cplusplus
}
#endif
