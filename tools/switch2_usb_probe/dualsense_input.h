#pragma once

#include "controller_input.h"

#if SWITCH2_BRIDGE_DUALSENSE_INPUT
// Core 0 only. One coherent profile/motion evaluation feeds both native children.
void probe_dualsense_input_init();
void probe_dualsense_input_set_stick_calibration(uint8_t instance, const uint8_t calibration[9]);
void probe_dualsense_input_set_native_stream(uint8_t instance, bool enabled);
void probe_dualsense_input_poll(uint8_t instance, uint32_t now_ms, probe_controller_input* out);
uint32_t probe_dualsense_input_peek_native_report(uint8_t instance, uint32_t now_ms, uint8_t report[63]);
bool probe_dualsense_input_commit_native_report(uint8_t instance, uint32_t token);
#endif
