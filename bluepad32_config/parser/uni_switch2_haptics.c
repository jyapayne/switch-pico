// SPDX-License-Identifier: Apache-2.0
#include "parser/uni_switch2_haptics.h"
#include <stddef.h>
#include <string.h>

static uint8_t frequency_index(uint8_t index) {
    if (index < 1) return 1;
    return index > 127 ? 127 : index;
}

static uint16_t amplitude_code(uint16_t q15) {
    if (q15 > 32767) q15 = 32767;
    // Match SDL's conservative native envelope while retaining a linear input
    // curve. This is transport gain, not a claim of calibrated physical force.
    return (uint16_t)(((uint32_t)q15 * 29000u / 32767u) >> 6);
}

static uint64_t load_sample(const uint8_t data[5]) {
    uint64_t value = 0;
    for (unsigned i = 0; i < 5; ++i) value |= (uint64_t)data[i] << (8 * i);
    return value;
}

static bool valid_side(const uni_switch2_haptics_side_t* side) {
    if (side->count > UNI_SWITCH2_HAPTICS_MAX_SAMPLES) return false;
    for (unsigned i = 0; i < side->count; ++i) {
        uint64_t value = load_sample(side->samples[i]);
        unsigned first_frequency = value & 1023u;
        unsigned second_frequency = (value >> 20) & 1023u;
        if (first_frequency == 0 || first_frequency > 670 ||
            second_frequency == 0 || second_frequency > 670 ||
            ((value >> 10) & 1023u) > UNI_SWITCH2_HAPTICS_MAX_AMPLITUDE ||
            ((value >> 30) & 1023u) > UNI_SWITCH2_HAPTICS_MAX_AMPLITUDE)
            return false;
    }
    return true;
}

void uni_switch2_haptics_encode_sample(uint8_t out[5], uint8_t low_index, uint8_t high_index,
                                      uint16_t low_q15, uint16_t high_q15) {
    if (!out) return;
    // The two physical frequency fields share one measured logarithmic scale.
    // Original Switch indices have32 steps/octave; Switch2 has96 steps/octave.
    uint64_t value = 193u + 3u * frequency_index(low_index);
    value |= (uint64_t)amplitude_code(low_q15) << 10;
    value |= (uint64_t)(289u + 3u * frequency_index(high_index)) << 20;
    value |= (uint64_t)amplitude_code(high_q15) << 30;
    for (unsigned i = 0; i < 5; ++i) out[i] = (uint8_t)(value >> (8 * i));
}

void uni_switch2_haptics_silence(uni_switch2_haptics_frame_t* frame) {
    if (!frame) return;
    memset(frame, 0, sizeof(*frame));
    for (unsigned side = 0; side < 2; ++side) {
        frame->sides[side].count = 1;
        uni_switch2_haptics_encode_sample(frame->sides[side].samples[0], 64, 64, 0, 0);
    }
}

bool uni_switch2_haptics_valid(const uni_switch2_haptics_frame_t* frame) {
    return frame && (frame->sides[0].count || frame->sides[1].count) &&
           valid_side(&frame->sides[0]) && valid_side(&frame->sides[1]);
}

bool uni_switch2_haptics_is_stop(const uni_switch2_haptics_frame_t* frame) {
    if (!uni_switch2_haptics_valid(frame) || !frame->sides[0].count || !frame->sides[1].count)
        return false;
    for (unsigned side = 0; side < 2; ++side) {
        for (unsigned i = 0; i < frame->sides[side].count; ++i) {
            uint64_t value = load_sample(frame->sides[side].samples[i]);
            if (((value >> 10) & 1023u) || ((value >> 30) & 1023u)) return false;
        }
    }
    return true;
}

bool uni_switch2_haptics_write_block(uint8_t out[16], const uni_switch2_haptics_side_t* side,
                                    uint8_t sequence) {
    if (!out || !side || !side->count || side->count > UNI_SWITCH2_HAPTICS_MAX_SAMPLES) return false;
    memset(out, 0, 16);
    out[0] = (uint8_t)(0x40u | (side->count << 4) | (sequence & 15u));
    memcpy(out + 1, side->samples, 5u * side->count);
    return true;
}
