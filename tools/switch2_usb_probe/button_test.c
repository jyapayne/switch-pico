#include "button_test.h"

bool probe_button_update(probe_button_state* state, int sample, bool ready) {
    if (!ready || (sample != 0 && sample != 1)) {
        state->armed = false;
        state->pressed_samples = 0;
        return false;
    }
    if (sample == 0) {
        state->armed = true;
        state->pressed_samples = 0;
        return false;
    }
    if (!state->armed) return false;
    if (state->pressed_samples < 2) ++state->pressed_samples;
    return state->pressed_samples == 2;
}
