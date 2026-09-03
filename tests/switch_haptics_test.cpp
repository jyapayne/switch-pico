#include "usb/switch/switch_haptics.h"
#include <array>
#include <cstdint>
#include <iostream>

namespace {

int failures = 0;

void expect_output(const char* scenario, ControllerRumbleOutput actual,
                   uint8_t expected_low, uint8_t expected_high) {
    if (actual.low_frequency_magnitude == expected_low &&
        actual.high_frequency_magnitude == expected_high) {
        return;
    }
    std::cerr << scenario << ": expected low/high "
              << static_cast<unsigned>(expected_low) << "/"
              << static_cast<unsigned>(expected_high) << ", got "
              << static_cast<unsigned>(actual.low_frequency_magnitude) << "/"
              << static_cast<unsigned>(actual.high_frequency_magnitude) << '\n';
    ++failures;
}

uint32_t type_2(uint8_t high_frequency, uint8_t high_amplitude,
                uint8_t low_frequency, uint8_t low_amplitude) {
    return (1u << 30u) |
           ((static_cast<uint32_t>(low_amplitude) & 0x7fu) << 23u) |
           ((static_cast<uint32_t>(low_frequency) & 0x7fu) << 16u) |
           ((static_cast<uint32_t>(high_amplitude) & 0x7fu) << 9u) |
           ((static_cast<uint32_t>(high_frequency) & 0x7fu) << 2u);
}

uint32_t type_1_one_sample(uint8_t high_command, uint8_t low_command) {
    return (1u << 30u) |
           ((static_cast<uint32_t>(low_command) & 0x1fu) << 25u) |
           ((static_cast<uint32_t>(high_command) & 0x1fu) << 20u);
}

uint32_t type_1_three_samples(uint8_t high_0, uint8_t low_0,
                              uint8_t high_1, uint8_t low_1,
                              uint8_t high_2, uint8_t low_2) {
    return (3u << 30u) |
           ((static_cast<uint32_t>(low_0) & 0x1fu) << 25u) |
           ((static_cast<uint32_t>(high_0) & 0x1fu) << 20u) |
           ((static_cast<uint32_t>(low_1) & 0x1fu) << 15u) |
           ((static_cast<uint32_t>(high_1) & 0x1fu) << 10u) |
           ((static_cast<uint32_t>(low_2) & 0x1fu) << 5u) |
           (static_cast<uint32_t>(high_2) & 0x1fu);
}

std::array<uint8_t, 8> payload(uint32_t left, uint32_t right) {
    std::array<uint8_t, 8> bytes{};
    const uint32_t words[2] = {left, right};
    for (unsigned actuator = 0; actuator < 2; ++actuator) {
        const unsigned offset = actuator * 4u;
        bytes[offset] = static_cast<uint8_t>(words[actuator]);
        bytes[offset + 1u] = static_cast<uint8_t>(words[actuator] >> 8u);
        bytes[offset + 2u] = static_cast<uint8_t>(words[actuator] >> 16u);
        bytes[offset + 3u] = static_cast<uint8_t>(words[actuator] >> 24u);
    }
    return bytes;
}

void test_neutral_and_per_actuator_reset() {
    constexpr uint32_t neutral = 0x40400100u;
    SwitchHapticsDecoder decoder;

    auto frame = payload(neutral, neutral);
    expect_output("explicit neutral", decoder.decode(frame.data()), 0, 0);

    frame = payload(type_2(90, 16, 50, 127), type_2(100, 32, 40, 16));
    expect_output("active actuators", decoder.decode(frame.data()), 250, 32);

    decoder.reset();
    frame = payload(1u << 5u, 1u << 5u);
    expect_output("explicit decoder reset", decoder.decode(frame.data()), 0, 0);

    frame = payload(type_2(90, 16, 50, 127), type_2(100, 32, 40, 16));
    decoder.decode(frame.data());

    frame = payload(0, type_2(100, 32, 40, 16));
    expect_output("zero resets only left actuator", decoder.decode(frame.data()), 16, 32);

    frame = payload(0, neutral);
    expect_output("neutral resets right actuator", decoder.decode(frame.data()), 0, 0);
}

void test_type_2_full_state_and_band_mapping() {
    constexpr uint32_t neutral = 0x40400100u;
    SwitchHapticsDecoder decoder;
    const auto frame = payload(type_2(100, 32, 20, 16), neutral);
    expect_output("type-2 low/high mapping", decoder.decode(frame.data()), 16, 32);
}

void test_type_1_relative_update_and_idempotence() {
    constexpr uint32_t neutral = 0x40400100u;
    SwitchHapticsDecoder decoder;

    auto frame = payload(type_2(64, 16, 64, 16), neutral);
    expect_output("relative update initial state", decoder.decode(frame.data()), 16, 16);

    frame = payload(type_1_one_sample(17, 20), neutral);
    expect_output("type-1 relative update", decoder.decode(frame.data()), 17, 18);
    expect_output("identical delta is idempotent", decoder.decode(frame.data()), 17, 18);
}

void test_subsample_peak_and_repeated_current_state() {
    constexpr uint32_t neutral = 0x40400100u;
    SwitchHapticsDecoder decoder;

    auto frame = payload(type_2(64, 16, 64, 16), neutral);
    decoder.decode(frame.data());

    frame = payload(type_1_three_samples(17, 17, 29, 29, 24, 24), neutral);
    expect_output("peak across three subsamples", decoder.decode(frame.data()), 18, 18);
    expect_output("repeat returns final cumulative state", decoder.decode(frame.data()), 16, 16);
}

void test_left_right_peak_combination() {
    SwitchHapticsDecoder decoder;
    const auto frame = payload(type_2(90, 1, 50, 127), type_2(100, 32, 40, 1));
    expect_output("independent actuator band peaks", decoder.decode(frame.data()), 250, 32);
}

void test_type_3_and_type_4_frames() {
    constexpr uint32_t neutral = 0x40400100u;
    SwitchHapticsDecoder decoder;

    auto frame = payload(type_2(64, 16, 64, 16), neutral);
    decoder.decode(frame.data());

    const uint32_t type3 = (2u << 30u) | 1u | (70u << 1u) |
                           (24u << 8u) | (17u << 13u) |
                           (20u << 18u) | (32u << 23u);
    frame = payload(type3, neutral);
    expect_output("type-3 full plus relative samples", decoder.decode(frame.data()), 18, 32);

    const uint32_t type4_low_amplitude = (1u << 30u) | 2u | (32u << 23u);
    frame = payload(type4_low_amplitude, neutral);
    expect_output("type-4 low amplitude selection", decoder.decode(frame.data()), 32, 32);

    const uint32_t type4_high_amplitude = (1u << 30u) | 3u | (127u << 23u);
    frame = payload(type4_high_amplitude, neutral);
    expect_output("type-4 high amplitude selection", decoder.decode(frame.data()), 32, 250);
}

void test_malformed_and_reserved_words_preserve_state() {
    constexpr uint32_t neutral = 0x40400100u;
    SwitchHapticsDecoder decoder;

    auto frame = payload(type_2(100, 32, 20, 16), neutral);
    decoder.decode(frame.data());

    frame = payload((1u << 30u) | 1u, neutral);
    expect_output("reserved type discriminator", decoder.decode(frame.data()), 16, 32);

    frame = payload(1u << 5u, neutral);
    expect_output("zero-frame word clears high band", decoder.decode(frame.data()), 16, 0);
}

void test_output_report_normalization() {
    const uint8_t stripped[] = {
        0x0a,
        0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40,
    };
    uint8_t output[64]{};

    size_t size = normalize_switch_output_report(0x01, stripped, sizeof(stripped), output);
    if (size != sizeof(stripped) + 1 || output[0] != 0x01 ||
        output[1] != 0x0a || output[2] != 0x00 || output[9] != 0x40) {
        std::cerr << "stripped 0x01 report normalization failed\n";
        ++failures;
    }

    size = normalize_switch_output_report(0x10, stripped, sizeof(stripped), output);
    if (size != sizeof(stripped) + 1 || output[0] != 0x10 ||
        output[1] != 0x0a || output[2] != 0x00 || output[9] != 0x40) {
        std::cerr << "stripped 0x10 report normalization failed\n";
        ++failures;
    }

    const uint8_t complete[] = {
        0x10, 0x0a,
        0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40,
    };
    size = normalize_switch_output_report(0, complete, sizeof(complete), output);
    if (size != sizeof(complete) || output[0] != 0x10 ||
        output[1] != 0x0a || output[9] != 0x40) {
        std::cerr << "complete interrupt report normalization failed\n";
        ++failures;
    }

    std::array<uint8_t, 64> oversized{};
    if (normalize_switch_output_report(0x01, oversized.data(), oversized.size(), output) != 0) {
        std::cerr << "oversized stripped report was accepted\n";
        ++failures;
    }
}

}  // namespace

int main() {
    test_neutral_and_per_actuator_reset();
    test_type_2_full_state_and_band_mapping();
    test_type_1_relative_update_and_idempotence();
    test_subsample_peak_and_repeated_current_state();
    test_left_right_peak_combination();
    test_type_3_and_type_4_frames();
    test_malformed_and_reserved_words_preserve_state();
    test_output_report_normalization();

    if (failures != 0) {
        std::cerr << failures << " haptics test(s) failed\n";
        return 1;
    }
    return 0;
}
