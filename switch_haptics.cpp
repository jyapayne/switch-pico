#include "switch_haptics.h"

#include <cmath>
#include <cstring>

namespace {

enum class CommandAction : uint8_t {
    Ignore,
    Default,
    Substitute,
    Sum,
};

struct HapticCommand {
    CommandAction amplitude_action;
    CommandAction frequency_action;
    int16_t amplitude_offset;
    int16_t frequency_offset;
};

constexpr HapticCommand kCommands[32] = {
    {CommandAction::Default, CommandAction::Default, 0, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 0, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 240, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 224, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 208, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 192, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 176, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 160, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 144, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 128, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 112, 0},
    {CommandAction::Substitute, CommandAction::Ignore, 96, 0},
    {CommandAction::Ignore, CommandAction::Substitute, 0, 5},
    {CommandAction::Ignore, CommandAction::Substitute, 0, 5},
    {CommandAction::Ignore, CommandAction::Substitute, 0, 0},
    {CommandAction::Ignore, CommandAction::Substitute, 0, 7},
    {CommandAction::Ignore, CommandAction::Substitute, 0, 7},
    {CommandAction::Sum, CommandAction::Sum, 4, 1},
    {CommandAction::Sum, CommandAction::Ignore, 4, 0},
    {CommandAction::Sum, CommandAction::Sum, 4, -1},
    {CommandAction::Sum, CommandAction::Sum, 1, 1},
    {CommandAction::Sum, CommandAction::Ignore, 1, 0},
    {CommandAction::Sum, CommandAction::Sum, 1, -1},
    {CommandAction::Ignore, CommandAction::Sum, 0, 1},
    {CommandAction::Ignore, CommandAction::Ignore, 0, 0},
    {CommandAction::Ignore, CommandAction::Sum, 0, -1},
    {CommandAction::Sum, CommandAction::Sum, -1, 1},
    {CommandAction::Sum, CommandAction::Ignore, -1, 0},
    {CommandAction::Sum, CommandAction::Sum, -1, -1},
    {CommandAction::Sum, CommandAction::Sum, -4, 1},
    {CommandAction::Sum, CommandAction::Ignore, -4, 0},
    {CommandAction::Sum, CommandAction::Sum, -4, -1},
};

constexpr uint32_t kNeutralWord = 0x40400100u;
constexpr uint8_t kDefaultFrequency = 64;

template <unsigned Shift, uint32_t Mask>
constexpr uint8_t extract(uint32_t word) {
    static_assert(Shift < 32u, "32-bit word extraction shift must be bounded");
    static_assert(Mask <= 0xffu && Mask <= (0xffffffffu >> Shift),
                  "word extraction mask must fit the shifted byte");
    return static_cast<uint8_t>((word >> Shift) & Mask);
}

uint8_t apply_command(CommandAction action, int16_t offset, uint8_t current,
                      uint8_t default_value, uint8_t maximum) {
    switch (action) {
    case CommandAction::Ignore:
        return current;
    case CommandAction::Default:
        return default_value;
    case CommandAction::Substitute:
        return static_cast<uint8_t>(offset);
    case CommandAction::Sum: {
        int result = static_cast<int>(current) + static_cast<int>(offset);
        if (result < 0) {
            result = 0;
        } else if (result > maximum) {
            result = maximum;
        }
        return static_cast<uint8_t>(result);
    }
}
    return default_value;
}

uint8_t host_amplitude_to_lut_index(uint8_t host_index) {
    const unsigned index = host_index & 0x7fu;
    if (index == 0) {
        return 0;
    }
    if (index < 16) {
        return static_cast<uint8_t>(7u + 8u * index);
    }
    if (index < 32) {
        return static_cast<uint8_t>(97u + 2u * index);
    }
    return static_cast<uint8_t>(128u + index);
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

SwitchHapticsDecoder::AmplitudePeak SwitchHapticsDecoder::decode_actuator(
    ActuatorState& state, uint32_t word) {
    if (word == 0 || word == kNeutralWord) {
        reset_actuator(state);
        state.last_word = word;
        state.have_last_word = true;
        return {0, 0};
    }

    if (state.have_last_word && state.last_word == word) {
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
        return {state.low_amplitude, 0};
    }

    const auto record_sample = [&]() {
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
        return {state.low_amplitude, state.high_amplitude};
    }
    return peak;
}

uint8_t SwitchHapticsDecoder::amplitude_to_magnitude(uint8_t amplitude_index) {
    if (amplitude_index < 2) {
        return 0;
    }

    const double exponent = -8.0 + static_cast<double>(amplitude_index) / 32.0;
    const double scaled = std::exp2(exponent) * 255.0;
    unsigned magnitude = static_cast<unsigned>(scaled + 0.5);
    if (magnitude > 255u) {
        magnitude = 255u;
    }
    return static_cast<uint8_t>(magnitude);
}

SwitchRumbleOutput SwitchHapticsDecoder::decode(const uint8_t payload[8]) {
    AmplitudePeak peaks[2] = {
        {actuators_[0].low_amplitude, actuators_[0].high_amplitude},
        {actuators_[1].low_amplitude, actuators_[1].high_amplitude},
    };

    if (payload != nullptr) {
        peaks[0] = decode_actuator(actuators_[0], load_little_endian_word(payload));
        peaks[1] = decode_actuator(actuators_[1], load_little_endian_word(payload + 4));
    }

    const uint8_t low_peak = peaks[0].low > peaks[1].low ? peaks[0].low : peaks[1].low;
    const uint8_t high_peak = peaks[0].high > peaks[1].high ? peaks[0].high : peaks[1].high;
    return {amplitude_to_magnitude(low_peak), amplitude_to_magnitude(high_peak)};
}
