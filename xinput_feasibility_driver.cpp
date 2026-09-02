#include "xinput_feasibility_driver.h"

#include <stddef.h>
#include <string.h>

#include "device/usbd_pvt.h"
#include "tusb.h"
#include "xinput_feasibility_descriptors.h"
#include "xinput_feasibility_protocol.h"

namespace {

constexpr uint8_t kRhport = 0;
constexpr uint8_t kEndpointBufferSize = 32;

struct XInputContext {
    SwitchInputState input{};
    XInputFeasibility::InputReport input_report{};
    uint8_t output_report[kEndpointBufferSize]{};
    SwitchRumbleCallback rumble_callback = nullptr;
    uint8_t endpoint_in = 0;
    uint8_t endpoint_out = 0;
    bool configured = false;
};

XInputContext g_contexts[SWITCH_PICO_HID_INSTANCE_COUNT]{};

XInputContext *context_for(uint8_t instance) {
    if (instance >= SWITCH_PICO_HID_INSTANCE_COUNT) {
        return nullptr;
    }
    return &g_contexts[instance];
}

XInputContext *context_for_endpoint(uint8_t endpoint) {
    for (XInputContext &context : g_contexts) {
        if (context.endpoint_in == endpoint ||
            context.endpoint_out == endpoint) {
            return &context;
        }
    }
    return nullptr;
}

void reset_context(XInputContext &context) {
    const SwitchRumbleCallback callback = context.rumble_callback;
    context = {};
    context.rumble_callback = callback;
}

void driver_init() {
    for (XInputContext &context : g_contexts) {
        reset_context(context);
    }
}

bool driver_deinit() {
    driver_init();
    return true;
}

void driver_reset(uint8_t rhport) {
    (void)rhport;
    driver_init();
}

uint16_t driver_open(uint8_t rhport,
                     tusb_desc_interface_t const *interface_descriptor,
                     uint16_t max_length) {
    if (interface_descriptor == nullptr ||
        interface_descriptor->bInterfaceClass != 0xff ||
        interface_descriptor->bInterfaceSubClass != 0x5d ||
        interface_descriptor->bInterfaceProtocol != 0x01 ||
        interface_descriptor->bInterfaceNumber >=
            SWITCH_PICO_HID_INSTANCE_COUNT ||
        max_length < XInputFeasibility::kInterfaceDescriptorSize) {
        return 0;
    }

    XInputContext &context = g_contexts[interface_descriptor->bInterfaceNumber];
    reset_context(context);

    uint16_t consumed = sizeof(tusb_desc_interface_t);
    uint8_t const *descriptor = tu_desc_next(interface_descriptor);
    uint8_t endpoints_found = 0;
    while (consumed < XInputFeasibility::kInterfaceDescriptorSize) {
        const uint8_t descriptor_length = descriptor[0];
        if (descriptor_length == 0 ||
            consumed + descriptor_length >
                XInputFeasibility::kInterfaceDescriptorSize) {
            reset_context(context);
            return 0;
        }
        if (tu_desc_type(descriptor) == TUSB_DESC_ENDPOINT) {
            auto const *endpoint =
                reinterpret_cast<tusb_desc_endpoint_t const *>(descriptor);
            if (!usbd_edpt_open(rhport, endpoint)) {
                reset_context(context);
                return 0;
            }
            if (tu_edpt_dir(endpoint->bEndpointAddress) == TUSB_DIR_IN) {
                context.endpoint_in = endpoint->bEndpointAddress;
            } else {
                context.endpoint_out = endpoint->bEndpointAddress;
            }
            ++endpoints_found;
        }
        consumed = static_cast<uint16_t>(consumed + descriptor_length);
        descriptor = tu_desc_next(descriptor);
    }

    if (endpoints_found != 2 || context.endpoint_in == 0 ||
        context.endpoint_out == 0) {
        reset_context(context);
        return 0;
    }
    context.configured = true;
    if (!usbd_edpt_xfer(rhport, context.endpoint_out, context.output_report,
                        sizeof(context.output_report))) {
        reset_context(context);
        return 0;
    }
    return consumed;
}

bool driver_control(uint8_t rhport, uint8_t stage,
                    tusb_control_request_t const *request) {
    (void)rhport;
    (void)stage;
    (void)request;
    return false;
}

bool driver_transfer(uint8_t rhport, uint8_t endpoint, xfer_result_t result,
                     uint32_t transferred) {
    XInputContext *context = context_for_endpoint(endpoint);
    if (context == nullptr || result != XFER_RESULT_SUCCESS) {
        return false;
    }
    if (endpoint == context->endpoint_out) {
        SwitchRumbleOutput rumble{};
        if (XInputFeasibility::parse_rumble_report(context->output_report,
                                                   transferred, &rumble) &&
            context->rumble_callback != nullptr) {
            const uint8_t instance = static_cast<uint8_t>(context - g_contexts);
            context->rumble_callback(instance, rumble);
        }
        memset(context->output_report, 0, sizeof(context->output_report));
        return usbd_edpt_xfer(rhport, context->endpoint_out,
                              context->output_report,
                              sizeof(context->output_report));
    }
    return true;
}

usbd_class_driver_t const kDriver = {
    "XINPUT-FEASIBILITY", driver_init,    driver_deinit,   driver_reset,
    driver_open,          driver_control, driver_transfer, nullptr,
};

} // namespace

void xinput_feasibility_init(uint8_t instance) {
    XInputContext *context = context_for(instance);
    if (context != nullptr) {
        reset_context(*context);
    }
}

void xinput_feasibility_set_rumble_callback(uint8_t instance,
                                            SwitchRumbleCallback callback) {
    XInputContext *context = context_for(instance);
    if (context != nullptr) {
        context->rumble_callback = callback;
    }
}

void xinput_feasibility_set_input(uint8_t instance,
                                  const SwitchInputState &state) {
    XInputContext *context = context_for(instance);
    if (context != nullptr) {
        context->input = state;
    }
}

bool xinput_feasibility_task(uint8_t instance) {
    XInputContext *context = context_for(instance);
    if (context == nullptr || !context->configured || !tud_ready() ||
        usbd_edpt_busy(kRhport, context->endpoint_in)) {
        return false;
    }

    context->input_report =
        XInputFeasibility::build_input_report(context->input);
    if (!usbd_edpt_claim(kRhport, context->endpoint_in)) {
        return false;
    }
    if (!usbd_edpt_xfer(kRhport, context->endpoint_in,
                        reinterpret_cast<uint8_t *>(&context->input_report),
                        sizeof(context->input_report))) {
        usbd_edpt_release(kRhport, context->endpoint_in);
        return false;
    }
    return true;
}

bool xinput_feasibility_is_ready(uint8_t instance) {
    XInputContext *context = context_for(instance);
    return context != nullptr && context->configured && tud_ready();
}

extern "C" usbd_class_driver_t const *
usbd_app_driver_get_cb(uint8_t *driver_count) {
    if (driver_count == nullptr) {
        return nullptr;
    }
    *driver_count = 1;
    return &kDriver;
}
