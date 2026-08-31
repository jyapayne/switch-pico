#pragma once

#include <stdint.h>

#include "switch_haptics.h"
#include "switch_pro_driver.h"

constexpr uint8_t BLUEPAD32_INPUT_BACKEND_SLOT_COUNT = 4;

void bluepad32_input_backend_init();
void bluepad32_input_backend_start();
void bluepad32_input_backend_open_pairing_window();
bool bluepad32_input_backend_snapshot(uint8_t slot, SwitchInputState* out);
void bluepad32_input_backend_report_sent(uint8_t slot);
void bluepad32_input_backend_queue_rumble(uint8_t slot,
                                          const SwitchRumbleOutput& rumble);
