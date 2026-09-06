#include "usb/switch/switch_haptics.h"
#include "usb/switch/switch_haptics_amplitudes.h"
#include "usb/switch/switch_haptics_commands.h"
#include <cstring>

namespace {

using namespace SwitchHapticsCommands;

constexpr uint32_t kNeutralWord = 0x40400100u;
constexpr uint8_t kDefaultFrequency = 64;

template <unsigned Shift, uint32_t Mask>
constexpr uint8_t extract(uint32_t word) {
    static_assert(Shift < 32u, "32-bit word extraction shift must be bounded");
    static_assert(Mask <= 0xffu && Mask <= (0xffffffffu >> Shift),
                  "word extraction mask must fit the shifted byte");
    return static_cast<uint8_t>((word >> Shift) & Mask);
}


uint32_t load_little_endian_word(const uint8_t* bytes) {
    return static_cast<uint32_t>(bytes[0]) |
           (static_cast<uint32_t>(bytes[1]) << 8u) |
           (static_cast<uint32_t>(bytes[2]) << 16u) |
           (static_cast<uint32_t>(bytes[3]) << 24u);
}

}  // namespace

size_t normalize_switch_output_report(uint8_t report_id,
                                      const uint8_t* payload,
                                      size_t payload_size,
                                      uint8_t output[64]) {
    if (payload == nullptr || output == nullptr) {
        return 0;
    }
    if (report_id == 0) {
        if (payload_size > 64) {
            return 0;
        }
        std::memcpy(output, payload, payload_size);
        return payload_size;
    }
    if (payload_size >= 64) {
        return 0;
    }
    output[0] = report_id;
    std::memcpy(output + 1, payload, payload_size);
    return payload_size + 1;
}

SwitchHapticsDecoder::SwitchHapticsDecoder() {
    reset();
}

void SwitchHapticsDecoder::reset_actuator(ActuatorState& state) {
    state.high_amplitude = 0;
    state.low_amplitude = 0;
    state.high_frequency = kDefaultFrequency;
    state.low_frequency = kDefaultFrequency;
    state.last_word = 0;
    state.have_last_word = false;
}

void SwitchHapticsDecoder::reset() {
    reset_actuator(actuators_[0]);
    reset_actuator(actuators_[1]);
}

void SwitchHapticsDecoder::append_sample(
    const ActuatorState& state, SwitchHapticsActuatorFrame& output) {
    if (output.sample_count >= 3) {
        return;
    }
    output.samples[output.sample_count++] = {
        state.low_frequency, state.high_frequency,
        SwitchHapticsTables::kAmplitudeQ15[state.low_amplitude],
        SwitchHapticsTables::kAmplitudeQ15[state.high_amplitude],
    };
}

SwitchHapticsDecoder::AmplitudePeak SwitchHapticsDecoder::decode_actuator(
    ActuatorState& state, uint32_t word, SwitchHapticsActuatorFrame& output) {
    output = {};
    if (word == 0 || word == kNeutralWord) {
        reset_actuator(state);
        state.last_word = word;
        state.have_last_word = true;
        append_sample(state, output);
        return {0, 0};
    }

    if (state.have_last_word && state.last_word == word) {
        append_sample(state, output);
        return {state.low_amplitude, state.high_amplitude};
    }
    state.last_word = word;
    state.have_last_word = true;

    AmplitudePeak peak{0, 0};
    bool decoded = false;
    const uint8_t frame_count = extract<30u, 0x03u>(word);
    const uint32_t data = word & 0x3fffffffu;

    if (frame_count == 0) {
        state.high_amplitude = 0;
        append_sample(state, output);
        return {state.low_amplitude, 0};
    }

    const auto record_sample = [&]() {
        append_sample(state, output);
        if (state.low_amplitude > peak.low) {
            peak.low = state.low_amplitude;
        }
        if (state.high_amplitude > peak.high) {
            peak.high = state.high_amplitude;
        }
    };

    const auto apply_pair = [&](bool high_band, uint8_t command_index) {
        const HapticCommand& command = kCommands[command_index & 0x1fu];
        uint8_t& amplitude = high_band ? state.high_amplitude : state.low_amplitude;
        uint8_t& frequency = high_band ? state.high_frequency : state.low_frequency;
        amplitude = apply_command(command.amplitude_action, command.amplitude_offset,
                                  amplitude, 0, 255);
        frequency = apply_command(command.frequency_action, command.frequency_offset,
                                  frequency, kDefaultFrequency, 127);
    };

    const auto decode_type_1 = [&]() {
        const uint8_t high_commands[3] = {
            extract<20u, 0x1fu>(word),
            extract<10u, 0x1fu>(word),
            extract<0u, 0x1fu>(word),
        };
        const uint8_t low_commands[3] = {
            extract<25u, 0x1fu>(word),
            extract<15u, 0x1fu>(word),
            extract<5u, 0x1fu>(word),
        };
        for (uint8_t sample = 0; sample < frame_count; ++sample) {
            apply_pair(true, high_commands[sample]);
            apply_pair(false, low_commands[sample]);
            record_sample();
        }
        decoded = true;
    };

    if (frame_count == 1) {
        if ((data & 0x000fffffu) == 0) {
            decode_type_1();
        } else if ((data & 0x03u) == 0) {
            state.high_frequency = extract<2u, 0x7fu>(word);
            state.high_amplitude = host_amplitude_to_lut_index(extract<9u, 0x7fu>(word));
            state.low_frequency = extract<16u, 0x7fu>(word);
            state.low_amplitude = host_amplitude_to_lut_index(extract<23u, 0x7fu>(word));
            record_sample();
            decoded = true;
        } else if ((data & 0x02u) != 0) {
            const bool high_band = extract<0u, 0x01u>(word) != 0;
            const bool frequency_selected = extract<2u, 0x01u>(word) != 0;
            const uint8_t value = extract<23u, 0x7fu>(word);
            if (frequency_selected) {
                if (high_band) {
                    state.high_frequency = value;
                } else {
                    state.low_frequency = value;
                }
            } else if (high_band) {
                state.high_amplitude = host_amplitude_to_lut_index(value);
            } else {
                state.low_amplitude = host_amplitude_to_lut_index(value);
            }
            record_sample();
            decoded = true;
        }
    } else if (frame_count == 2) {
        if ((data & 0x03ffu) == 0) {
            decode_type_1();
        } else {
            const bool high_band = extract<0u, 0x01u>(word) != 0;
            const uint8_t frequency = extract<1u, 0x7fu>(word);
            const uint8_t command = extract<18u, 0x1fu>(word);
            const uint8_t amplitude = host_amplitude_to_lut_index(extract<23u, 0x7fu>(word));
            if (high_band) {
                state.high_frequency = frequency;
                state.high_amplitude = amplitude;
                apply_pair(false, command);
            } else {
                state.low_frequency = frequency;
                state.low_amplitude = amplitude;
                apply_pair(true, command);
            }
            record_sample();

            apply_pair(true, extract<8u, 0x1fu>(word));
            apply_pair(false, extract<13u, 0x1fu>(word));
            record_sample();
            decoded = true;
        }
    } else if (frame_count == 3) {
        decode_type_1();
    }

    if (!decoded) {
        append_sample(state, output);
        return {state.low_amplitude, state.high_amplitude};
    }
    return peak;
}

uint8_t SwitchHapticsDecoder::amplitude_to_magnitude(uint8_t amplitude_index) {
    return SwitchHapticsTables::kMagnitude[amplitude_index];
}

ControllerRumbleOutput SwitchHapticsDecoder::decode(const uint8_t payload[8]) {
    ControllerRumbleOutput output{};
    AmplitudePeak peaks[2] = {
        {actuators_[0].low_amplitude, actuators_[0].high_amplitude},
        {actuators_[1].low_amplitude, actuators_[1].high_amplitude},
    };

    if (payload != nullptr) {
        peaks[0] = decode_actuator(
            actuators_[0], load_little_endian_word(payload), output.hd.actuators[0]);
        peaks[1] = decode_actuator(
            actuators_[1], load_little_endian_word(payload + 4), output.hd.actuators[1]);
        std::memcpy(output.raw, payload, sizeof(output.raw));
        output.raw_valid = true;
        output.raw_unmodified = true;
    }

    const uint8_t low_peak = peaks[0].low > peaks[1].low ? peaks[0].low : peaks[1].low;
    const uint8_t high_peak = peaks[0].high > peaks[1].high ? peaks[0].high : peaks[1].high;
    output.low_frequency_magnitude = amplitude_to_magnitude(low_peak);
    output.high_frequency_magnitude = amplitude_to_magnitude(high_peak);
    return output;
}
