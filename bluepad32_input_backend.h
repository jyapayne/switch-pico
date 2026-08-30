#pragma once

#include <stdint.h>

#include "switch_pro_driver.h"
#include "switch_haptics.h"

void bluepad32_input_backend_init();
void bluepad32_input_backend_start();
bool bluepad32_input_backend_snapshot(SwitchInputState* out);
void bluepad32_input_backend_report_sent();
void bluepad32_input_backend_queue_rumble(const SwitchRumbleOutput& rumble);
