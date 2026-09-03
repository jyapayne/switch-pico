#ifndef SWITCH_HAPTICS_H
#define SWITCH_HAPTICS_H

#include <stddef.h>
#include <stdint.h>

struct ControllerRumbleOutput {
    uint8_t low_frequency_magnitude;
    uint8_t high_frequency_magnitude;
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
    static AmplitudePeak decode_actuator(ActuatorState& state, uint32_t word);
    static uint8_t amplitude_to_magnitude(uint8_t amplitude_index);

    ActuatorState actuators_[2];
};

#endif
