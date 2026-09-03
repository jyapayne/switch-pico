#pragma once

#include <stdint.h>

#include "controller_state.h"
#include "tusb.h"
namespace GenericHid {

enum class ReportDescriptorVariant : uint8_t {
    kDInput,
    kMac,
};

}  // namespace GenericHid


void generic_hid_init(uint8_t instance);
void generic_hid_set_input(uint8_t instance, const ControllerState& state);
bool generic_hid_task(uint8_t instance);
bool generic_hid_is_ready(uint8_t instance);

uint16_t generic_hid_get_report(uint8_t instance, uint8_t report_id,
                                hid_report_type_t report_type, uint8_t* buffer,
                                uint16_t requested_length);
const uint8_t* generic_hid_report_descriptor(
    uint8_t instance, GenericHid::ReportDescriptorVariant variant);
