#include "generic_hid_driver.h"

#include <string.h>

#include "generic_hid_descriptors.h"

namespace {

struct GenericHidContext {
    ControllerState input{};
    GenericHid::InputReport input_report{};
};

GenericHidContext g_contexts[SWITCH_PICO_HID_INSTANCE_COUNT]{};

GenericHidContext* context_for(uint8_t instance) {
    if (instance >= SWITCH_PICO_HID_INSTANCE_COUNT) {
        return nullptr;
    }
    return &g_contexts[instance];
}


void reset_context(GenericHidContext& context) {
    context = {};
    context.input_report = GenericHid::build_input_report(context.input);
}

}  // namespace

void generic_hid_init(uint8_t instance) {
    GenericHidContext* context = context_for(instance);
    if (context != nullptr) {
        reset_context(*context);
    }
}

void generic_hid_set_input(uint8_t instance, const ControllerState& state) {
    GenericHidContext* context = context_for(instance);
    if (context == nullptr) {
        return;
    }
    context->input = state;
    context->input_report = GenericHid::build_input_report(state);
}

bool generic_hid_task(uint8_t instance) {
    GenericHidContext* context = context_for(instance);
    return context != nullptr && tud_hid_n_ready(instance) &&
           tud_hid_n_report(instance, 0, &context->input_report,
                            sizeof(context->input_report));
}

bool generic_hid_is_ready(uint8_t instance) {
    return context_for(instance) != nullptr &&
           tud_hid_n_ready(instance);
}

uint16_t generic_hid_get_report(uint8_t instance, uint8_t report_id,
                                hid_report_type_t report_type, uint8_t* buffer,
                                uint16_t requested_length) {
    const GenericHidContext* context = context_for(instance);
    if (context == nullptr || report_id != 0 ||
        report_type != HID_REPORT_TYPE_INPUT || buffer == nullptr ||
        requested_length == 0) {
        return 0;
    }

    uint16_t report_size = sizeof(context->input_report);
    if (requested_length < report_size) {
        report_size = requested_length;
    }
    memcpy(buffer, &context->input_report, report_size);
    return report_size;
}

const uint8_t* generic_hid_report_descriptor(
    uint8_t instance, GenericHid::ReportDescriptorVariant variant) {
    if (context_for(instance) == nullptr) {
        return nullptr;
    }
    return variant == GenericHid::ReportDescriptorVariant::kMac
               ? GenericHid::kMacReportDescriptor
               : GenericHid::kDInputReportDescriptor;
}
