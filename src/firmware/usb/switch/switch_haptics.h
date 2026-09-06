#ifndef SWITCH_HAPTICS_H
#define SWITCH_HAPTICS_H

#include <stddef.h>
#include <stdint.h>

// Decoded indices are logarithmic frequencies; amplitudes are linear Q0.15.
// Low/high frequency index 64 means 160/320 Hz respectively.
struct SwitchHapticsSample {
    uint8_t low_frequency_index = 64;
    uint8_t high_frequency_index = 64;
    uint16_t low_amplitude_q15 = 0;
    uint16_t high_amplitude_q15 = 0;
};

struct SwitchHapticsActuatorFrame {
    uint8_t sample_count = 0;
    SwitchHapticsSample samples[3]{};
};

struct SwitchHapticsFrame {
    SwitchHapticsActuatorFrame actuators[2]{};  // Left, right.
};

struct ControllerRumbleOutput {
    uint8_t low_frequency_magnitude;
    uint8_t high_frequency_magnitude;
    // Counts are zero for conventional rumble (XInput/UART/local feedback).
    // Switch packets carry ordered per-side substeps in addition to the
    // compatibility magnitudes consumed by existing non-native backends.
    SwitchHapticsFrame hd{};
    // Only the Switch decoder grants provenance. Scaling retains the source
    // bytes for diagnostics but revokes unmodified unless both gains are unity.
    uint8_t raw[8]{};
    bool raw_valid = false;
    bool raw_unmodified = false;
};
typedef void (*ControllerRumbleCallback)(
    uint8_t instance, const ControllerRumbleOutput& rumble);


size_t normalize_switch_output_report(uint8_t report_id,
                                      const uint8_t* payload,
                                      size_t payload_size,
                                      uint8_t output[64]);

class SwitchHapticsDecoder {
public:
    SwitchHapticsDecoder();

    void reset();
    ControllerRumbleOutput decode(const uint8_t payload[8]);

private:
    friend class SwitchNativeHapticsEncoder;
    struct ActuatorState {
        uint8_t high_amplitude;
        uint8_t low_amplitude;
        uint8_t high_frequency;
        uint8_t low_frequency;
        uint32_t last_word;
        bool have_last_word;
    };

    struct AmplitudePeak {
        uint8_t low;
        uint8_t high;
    };

    static void reset_actuator(ActuatorState& state);
    static AmplitudePeak decode_actuator(ActuatorState& state, uint32_t word,
                                         SwitchHapticsActuatorFrame& output);
    static void append_sample(const ActuatorState& state,
                              SwitchHapticsActuatorFrame& output);
    static uint8_t amplitude_to_magnitude(uint8_t amplitude_index);

    ActuatorState actuators_[2];
};

#endif
