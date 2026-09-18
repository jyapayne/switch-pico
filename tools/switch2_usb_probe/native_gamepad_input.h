#pragma once

#include "controller_input.h"

#if SWITCH2_BRIDGE_FULL_INPUT
// Core 0 only. Each source pair shares one coherent profile/motion evaluation.
// Child instances remain A_R, A_L, B_R, B_L; transport state is child-local.
void probe_native_gamepad_input_init();
void probe_native_gamepad_input_set_stick_calibration(uint8_t instance, const uint8_t calibration[9]);
void probe_native_gamepad_input_set_native_stream(uint8_t instance, bool enabled);
void probe_native_gamepad_input_poll(uint8_t instance, uint32_t now_ms, probe_controller_input* out);
uint32_t probe_native_gamepad_input_peek_native_report(uint8_t instance, uint32_t now_ms, uint8_t report[63]);
bool probe_native_gamepad_input_commit_native_report(uint8_t instance, uint32_t token);
#endif
