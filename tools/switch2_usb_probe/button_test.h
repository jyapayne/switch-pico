#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool armed;
    uint8_t pressed_samples;
} probe_button_state;

// Sample at 10 ms intervals. Require a release after reset/unavailability,
// then two pressed samples. Release/error/not-ready clears output immediately.
bool probe_button_update(probe_button_state* state, int sample, bool ready);
