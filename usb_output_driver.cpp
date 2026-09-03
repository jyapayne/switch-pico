#include "usb_output_driver.h"

#include <cstring>
#include <stdio.h>

#include "device/usbd_pvt.h"
#include "switch_pro_driver.h"
#include "switch_pro_descriptors.h"
#include "tusb.h"

#ifdef SWITCH_PICO_BLUEPAD32
#include "usb_configuration_management.h"
#endif

#ifdef SWITCH_PICO_USB_OUTPUT_MODES
#include "adapter_host_probe.h"
#include "generic_hid_descriptors.h"
#include "generic_hid_driver.h"
#include "xinput_descriptors.h"
#include "xinput_driver.h"
#endif

#ifdef SWITCH_PICO_LOG
#define LOG_PRINTF(...) printf(__VA_ARGS__)
#else
#define LOG_PRINTF(...) ((void)0)
#endif

namespace {

AdapterUsbMode g_mode = AdapterUsbMode::kSwitch;
uint16_t g_string_descriptor[32]{};

bool xinput_selected() {
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    return g_mode == AdapterUsbMode::kXInput;
#else
    return false;
#endif
}

bool generic_selected() {
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    return g_mode == AdapterUsbMode::kDInput ||
           g_mode == AdapterUsbMode::kMac;
#else
    return false;
#endif
}

bool switch_selected() {
    return !xinput_selected() && !generic_selected();
}

}  // namespace

void usb_output_driver_init(AdapterUsbMode mode) {
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    g_mode = mode;
#else
    (void)mode;
    g_mode = AdapterUsbMode::kSwitch;
#endif

    for (uint8_t instance = 0;
         instance < SWITCH_PICO_HID_INSTANCE_COUNT; ++instance) {
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
        if (xinput_selected()) {
            xinput_init(instance);
        } else if (generic_selected()) {
            generic_hid_init(instance);
        } else
#endif
        {
            switch_pro_init(instance);
        }
    }
}

const char* usb_output_driver_mode_name() {
    switch (g_mode) {
        case AdapterUsbMode::kSwitch:
            return "Switch";
        case AdapterUsbMode::kSwitchProbe:
            return "Switch probe";
        case AdapterUsbMode::kXInput:
            return "XInput";
        case AdapterUsbMode::kDInput:
            return "DInput";
        case AdapterUsbMode::kMac:
            return "Mac";
    }
    return "Switch";
}

AdapterUsbMode usb_output_driver_mode() {
    return g_mode;
}

const char* usb_output_driver_name() {
    switch (g_mode) {
        case AdapterUsbMode::kXInput:
            return "XINPUT";
        case AdapterUsbMode::kDInput:
            return "DINPUT";
        case AdapterUsbMode::kMac:
            return "MAC";
        case AdapterUsbMode::kSwitch:
        case AdapterUsbMode::kSwitchProbe:
            return "SWITCH";
    }
    return "SWITCH";
}

uint8_t usb_output_driver_capabilities() {
    switch (g_mode) {
        case AdapterUsbMode::kSwitch:
        case AdapterUsbMode::kSwitchProbe:
            return USB_OUTPUT_CAPABILITY_INPUT |
                   USB_OUTPUT_CAPABILITY_RUMBLE |
                   USB_OUTPUT_CAPABILITY_MOTION;
        case AdapterUsbMode::kXInput:
            return USB_OUTPUT_CAPABILITY_INPUT |
                   USB_OUTPUT_CAPABILITY_RUMBLE;
        case AdapterUsbMode::kDInput:
        case AdapterUsbMode::kMac:
            return USB_OUTPUT_CAPABILITY_INPUT;
    }
    return USB_OUTPUT_CAPABILITY_INPUT;
}

void usb_output_driver_set_input(uint8_t instance,
                                 const ControllerState& state,
                                 uint16_t left_trigger_threshold,
                                 uint16_t right_trigger_threshold) {
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    if (xinput_selected()) {
        xinput_set_input(instance, state);
        return;
    }
    if (generic_selected()) {
        generic_hid_set_input(instance, state);
        return;
    }
#endif
    switch_pro_set_input(instance, state, left_trigger_threshold,
                         right_trigger_threshold);
}

bool usb_output_driver_task(uint8_t instance) {
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    if (xinput_selected()) {
        return xinput_task(instance);
    }
    if (generic_selected()) {
        return generic_hid_task(instance);
    }
#endif
    return switch_pro_task(instance);
}

bool usb_output_driver_is_ready(uint8_t instance) {
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    if (xinput_selected()) {
        return xinput_is_ready(instance);
    }
    if (generic_selected()) {
        return generic_hid_is_ready(instance);
    }
#endif
    return switch_pro_is_ready(instance);
}

void usb_output_driver_set_rumble_callback(
    uint8_t instance, ControllerRumbleCallback callback) {
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    if (xinput_selected()) {
        xinput_set_rumble_callback(instance, callback);
        return;
    }
    if (generic_selected()) {
        return;
    }
#endif
    switch_pro_set_rumble_callback(instance, callback);
}

extern "C" uint16_t tud_hid_get_report_cb(
    uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
    uint8_t* buffer, uint16_t requested_length) {
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    if (xinput_selected()) {
        return 0;
    }
    if (generic_selected()) {
        return generic_hid_get_report(instance, report_id, report_type,
                                      buffer, requested_length);
    }
#endif
    return switch_pro_hid_get_report(instance, report_id, report_type, buffer,
                                     requested_length);
}

extern "C" void tud_hid_set_report_cb(
    uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
    const uint8_t* buffer, uint16_t buffer_size) {
    if (switch_selected()) {
        switch_pro_hid_set_report(instance, report_id, report_type, buffer,
                                  buffer_size);
    }
}

extern "C" void tud_hid_report_received_cb(
    uint8_t instance, uint8_t report_id, const uint8_t* buffer,
    uint16_t buffer_size) {
    if (switch_selected()) {
        switch_pro_hid_report_received(instance, report_id, buffer,
                                       buffer_size);
    }
}

extern "C" uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    if (xinput_selected()) {
        return nullptr;
    }
    if (generic_selected()) {
        return generic_hid_report_descriptor(instance);
    }
#endif
    return switch_pro_hid_report_descriptor(instance);
}

extern "C" uint8_t const* tud_descriptor_device_cb() {
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    if (xinput_selected()) {
        return XInput::kDeviceDescriptor;
    }
    if (generic_selected()) {
        return g_mode == AdapterUsbMode::kDInput
                   ? GenericHid::kDInputDeviceDescriptor
                   : GenericHid::kMacDeviceDescriptor;
    }
    if (g_mode == AdapterUsbMode::kSwitchProbe) {
        return XInput::kSwitchProbeDeviceDescriptor;
    }
#endif
    return switch_pro_device_descriptor;
}

extern "C" uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    if (xinput_selected()) {
        return XInput::kConfigurationDescriptor;
    }
    if (generic_selected()) {
        return GenericHid::kConfigurationDescriptor;
    }
#endif
    return switch_pro_configuration_descriptor;
}

extern "C" uint16_t const* tud_descriptor_string_cb(uint8_t index,
                                                     uint16_t langid) {
    (void)langid;

#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    adapter_host_probe_note_string_descriptor(index);
    if (index == 0xee &&
        (xinput_selected() ||
         g_mode == AdapterUsbMode::kSwitchProbe)) {
        static constexpr char kSignature[] = "MSFT100";
        for (uint8_t i = 0; i < sizeof(kSignature) - 1; ++i) {
            g_string_descriptor[1 + i] = kSignature[i];
        }
        g_string_descriptor[8] = XInput::kMsVendorRequest;
        g_string_descriptor[0] =
            static_cast<uint16_t>((TUSB_DESC_STRING << 8) | 18);
        return g_string_descriptor;
    }
#endif

    uint8_t character_count = 0;
    if (index == 0) {
        memcpy(&g_string_descriptor[1], switch_pro_string_language, 2);
        character_count = 1;
    } else {
        const uint8_t* string = nullptr;
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
        static const uint8_t kManufacturer[] = "Switch Pico";
        static const uint8_t kProduct[] = "XInput Feasibility";
        static const uint8_t kSerial[] = "XINPUT-PROTOTYPE";
        static const uint8_t* const kXInputStrings[] = {
            nullptr, kManufacturer, kProduct, kSerial};
        if (xinput_selected()) {
            if (index >= sizeof(kXInputStrings) /
                             sizeof(kXInputStrings[0])) {
                return nullptr;
            }
            string = kXInputStrings[index];
        } else if (generic_selected()) {
            if (index == 1) {
                string = reinterpret_cast<const uint8_t*>(
                    GenericHid::kManufacturerString);
            } else if (index == 2) {
                string = reinterpret_cast<const uint8_t*>(
                    g_mode == AdapterUsbMode::kDInput
                        ? GenericHid::kDInputProductString
                        : GenericHid::kMacProductString);
            } else if (index == 3) {
                string = reinterpret_cast<const uint8_t*>(
                    g_mode == AdapterUsbMode::kDInput
                        ? GenericHid::kDInputSerialString
                        : GenericHid::kMacSerialString);
            } else {
                return nullptr;
            }
        } else
#endif
        {
            if (index >= sizeof(switch_pro_string_descriptors) /
                             sizeof(switch_pro_string_descriptors[0])) {
                return nullptr;
            }
            string = switch_pro_string_descriptors[index];
        }

        while (string[character_count] != 0) {
            ++character_count;
        }
        if (character_count > 31) {
            character_count = 31;
        }
        for (uint8_t i = 0; i < character_count; ++i) {
            g_string_descriptor[1 + i] = string[i];
        }
    }

    g_string_descriptor[0] = static_cast<uint16_t>(
        (TUSB_DESC_STRING << 8) | (2 * character_count + 2));
    return g_string_descriptor;
}

extern "C" bool tud_vendor_control_xfer_cb(
    uint8_t rhport, uint8_t stage,
    tusb_control_request_t const* request) {
#ifdef SWITCH_PICO_BLUEPAD32
    return usb_configuration_management_vendor_control(rhport, stage,
                                                       request);
#else
    (void)rhport;
    (void)stage;
    (void)request;
    return false;
#endif
}

extern "C" bool tud_control_request_cb(
    uint8_t rhport, tusb_control_request_t const* request) {
    (void)rhport;
    if (request != nullptr) {
        LOG_PRINTF(
            "[CTRL] bmReq=0x%02x bReq=0x%02x wValue=0x%04x "
            "wIndex=0x%04x wLen=%u\n",
            request->bmRequestType, request->bRequest, request->wValue,
            request->wIndex, request->wLength);
    }
    return false;
}

extern "C" void tud_mount_cb() {
    LOG_PRINTF("[USB] mount_cb\n");
    if (switch_selected()) {
        switch_pro_mount();
    }
}

extern "C" void tud_umount_cb() {
    LOG_PRINTF("[USB] umount_cb\n");
    if (switch_selected()) {
        switch_pro_unmount();
    }
}

extern "C" usbd_class_driver_t const* usbd_app_driver_get_cb(
    uint8_t* driver_count) {
    if (driver_count == nullptr) {
        return nullptr;
    }
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    if (xinput_selected()) {
        *driver_count = 1;
        return xinput_class_driver();
    }
#endif
    *driver_count = 0;
    return nullptr;
}
