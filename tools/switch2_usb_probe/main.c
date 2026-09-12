// Joy-Con 2 USB instrument and optional Bluetooth controller/mouse bridge.
// Only documented/observed transactions are implemented. Built-in vibration
// cues wait for the source ACK; native sensor packets are relayed without decoding.
// Only virtual pairing storage is writable here.
#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
#include "bootsel.h"
#include "controller_input.h"
#else
#include "platform/pico/bootsel_button_sample.h"
#include "button_test.h"
#endif
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "tusb.h"
#include "descriptors.h"
#include "protocol.h"
#include "storage.h"
#include "transport.h"
#ifdef SWITCH2_PROBE_MEMORY
#include "memory.h"
#endif
#ifdef SWITCH2_PROBE_IDENTITY_REPLY
#include "probe_identity.h"
_Static_assert(sizeof(probe_identity_replies[0]) == 64, "factory identity response size");
_Static_assert(sizeof(probe_identity_replies) / sizeof(probe_identity_replies[0]) ==
               PROBE_CONTROLLER_COUNT, "one captured identity per controller");
#endif
#ifdef SWITCH2_PROBE_VERSION_REPLY
#include "probe_version.h"
_Static_assert(sizeof(probe_version_replies[0]) == 16, "version/address response size");
_Static_assert(sizeof(probe_version_replies) / sizeof(probe_version_replies[0]) ==
               PROBE_CONTROLLER_COUNT, "one captured version/address per controller");
#endif

#define LOG_CAPACITY 8192u
static char log_bytes[LOG_CAPACITY];
static uint32_t log_written, log_read, log_dropped;
static uint32_t bulk_packets, hid_packets;
static uint32_t identity_requests, version_requests, setup_completions;
static uint16_t string_descriptors[PROBE_CONTROLLER_COUNT][64];
static uint32_t input_reports, command_drops;
#ifdef SWITCH2_PROBE_USB_INIT
#define REPLY_CAPACITY 4u
typedef struct {
    uint8_t data[PROBE_REPLY_MAX_SIZE];
    uint8_t length;
    uint64_t deferred_token;
} queued_reply;
typedef struct {
    uint8_t instance;
    probe_protocol_state protocol;
    queued_reply replies[REPLY_CAPACITY];
    uint8_t reply_head, reply_count;
    bool reply_inflight;
    uint16_t reply_remaining;
    uint8_t command_frame[PROBE_COMMAND_MAX_SIZE];
    uint16_t command_used, command_expected;
    uint32_t last_input_ms, input_reports;
    uint32_t command_drops;
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
    uint32_t last_controller_poll_ms;
    uint16_t last_delivered_buttons;
    bool native_stream_ready;
    uint32_t last_hid_complete_ms;
    bool hid_completion_seen;
    uint32_t mouse_delivered_reports, mouse_logged_reports;
    int64_t mouse_delivered_x, mouse_delivered_y;
#ifdef SWITCH2_PROBE_TRACE_NATIVE_INPUT
    uint32_t last_native_trace_ms;
#endif
#else
    probe_button_state button_test;
    uint32_t last_button_ms;
    bool button_sample_error;
#endif
    uint8_t last_delivered_rails;
} probe_usb_controller;
static probe_usb_controller controllers[PROBE_CONTROLLER_COUNT];
#endif

_Static_assert(sizeof(probe_device_descriptor) == 18, "device descriptor size");
_Static_assert(sizeof(probe_configuration_descriptor) ==
               (SWITCH2_PROBE_COMPOSITE ? 151 : 80), "configuration descriptor size");
_Static_assert(sizeof(probe_hid_report_descriptors[0]) == 100, "HID descriptor size");

#if defined(SWITCH2_PROBE_JOIN_CHORD_GATE) && !SWITCH2_PROBE_HUB
_Static_assert(PROBE_CONTROLLER_COUNT == 2, "L+R gate requires both native functions");
static uint8_t last_join_shoulder_mask;

static uint8_t join_shoulder_mask(void) {
    uint8_t mask = 0;
    for (uint8_t instance = 0; instance < PROBE_CONTROLLER_COUNT; ++instance) {
        const probe_protocol_state* state = &controllers[instance].protocol;
        if (state->controller_active && (state->controller_buttons[0] & 0x10))
            mask |= (uint8_t)(1u << instance);
    }
    return mask;
}

static void gate_join_shoulders(uint8_t instance, uint8_t report_id,
                                uint8_t input[PROBE_INPUT_SIZE]) {
    if (join_shoulder_mask() == 3) return;
    if (report_id == probe_model_report_id(instance))
        input[2] &= (uint8_t)~0x10u;
    else if (report_id == 0x05)
        input[probe_model_is_left(instance) ? 6 : 4] &= (uint8_t)~0x40u;
    // Only suppress a real shoulder bit; never insert a counterpart press.
}
#endif

int probe_debug_printf(const char* format, ...) {
    char message[512];
    va_list args;
    va_start(args, format);
    const int result = vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    if (result <= 0) return result;
    const size_t size = (size_t)result < sizeof(message) ? (size_t)result : sizeof(message) - 1;
    const uint32_t interrupts = save_and_disable_interrupts();
    if (LOG_CAPACITY - (log_written - log_read) >= size) {
        for (size_t i = 0; i < size; ++i)
            log_bytes[(log_written + i) % LOG_CAPACITY] = message[i];
        log_written += (uint32_t)size;
        log_dropped += (uint32_t)result - (uint32_t)size;
    } else {
        log_dropped += (uint32_t)result;
    }
    restore_interrupts(interrupts);
    return result;
}

static void drain_log(void) {
    while (uart_is_writable(uart0)) {
        const uint32_t interrupts = save_and_disable_interrupts();
        if (log_read == log_written) {
            restore_interrupts(interrupts);
            break;
        }
        const char value = log_bytes[log_read++ % LOG_CAPACITY];
        restore_interrupts(interrupts);
        uart_putc_raw(uart0, value);
    }
}

static void log_packet(const char* kind, uint8_t instance, uint8_t report_id,
                       const uint8_t* data, uint16_t length) {
    if (length >= 4 && data[0] == 0x15 && (data[3] == 0x02 || data[3] == 0x04)) {
        probe_debug_printf("[PROBE] %s itf=%u command=15/%02x len=%u [pairing payload redacted]\n",
                           kind, instance, data[3], length);
        return;
    }
    static const char hex[] = "0123456789abcdef";
    char message[288];
    size_t at = (size_t)snprintf(message, sizeof(message),
        "[PROBE %" PRIu32 "] %s itf=%u report=%02x len=%u:",
        to_ms_since_boot(get_absolute_time()), kind, instance, report_id, length);
    const uint16_t count = length < 64 ? length : 64;
    for (uint16_t i = 0; i < count && at + 4 < sizeof(message); ++i) {
        message[at++] = ' ';
        message[at++] = hex[data[i] >> 4];
        message[at++] = hex[data[i] & 15];
    }
    message[at] = 0;
    probe_debug_printf("%s%s\n", message, length > count ? " [truncated]" : "");
}

uint8_t const* tud_descriptor_device_cb(void) {
    probe_debug_printf("[PROBE] DEVICE_DESCRIPTOR 057e:%04x\n", (unsigned)PROBE_JOYCON_PID);
    return probe_device_descriptor;
}

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    probe_debug_printf("[PROBE] CONFIG_DESCRIPTOR index=%u\n", index);
    return index == 0 ? probe_configuration_descriptor : NULL;
}

uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {
    probe_debug_printf("[PROBE] HID_DESCRIPTOR itf=%u\n", instance);
    return instance < PROBE_CONTROLLER_COUNT ? probe_hid_report_descriptors[instance] : NULL;
}

static uint16_t const* controller_string_descriptor(uint8_t instance, uint8_t index,
                                                     uint16_t langid) {
    // Manufacturer/product/serial match the published reference. Remaining
    // descriptor labels describe the probe; their genuine strings are unknown.
    static const char* const strings[] = {
        "", "Nintendo", PROBE_JOYCON_PRODUCT, "00", "USB configuration", "HID", "Commands",
        "Left HID", "Left commands"};
    if (instance >= PROBE_CONTROLLER_COUNT) return NULL;
    uint16_t* descriptor = string_descriptors[instance];
    probe_debug_printf("[PROBE] STRING_DESCRIPTOR itf=%u index=%u lang=%04x\n",
                       instance, index, langid);
    if (index == 0) {
        descriptor[0] = (TUSB_DESC_STRING << 8) | 4;
        descriptor[1] = 0x0409;
        return descriptor;
    }
    if (index >= sizeof(strings) / sizeof(strings[0])) return NULL;
    const char* text = index == 2 && probe_model_is_left(instance) ?
        "Joy-Con 2 (L)" : strings[index];
    size_t count = strlen(text);
    if (count > 63) count = 63;
    descriptor[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 + count * 2));
    for (size_t i = 0; i < count; ++i)
        descriptor[i + 1] = (uint8_t)text[i];
    return descriptor;
}

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    return controller_string_descriptor(0, index, langid);
}

#if SWITCH2_PROBE_HUB
const uint8_t* native_joycon_device_descriptor(uint8_t instance) {
    if (instance >= PROBE_CONTROLLER_COUNT) return NULL;
    probe_debug_printf("[PROBE] DEVICE_DESCRIPTOR itf=%u 057e:%04x\n",
                       instance, probe_model_pid(instance));
    return probe_model_is_left(instance) ? probe_left_device_descriptor : probe_device_descriptor;
}

const uint8_t* native_joycon_configuration_descriptor(uint8_t instance) {
    // HUB is mutually exclusive with COMPOSITE: each address uses native EP1/2.
    return instance < PROBE_CONTROLLER_COUNT ? probe_configuration_descriptor : NULL;
}

const uint16_t* native_joycon_string_descriptor(uint8_t instance, uint8_t index,
                                                uint16_t language_id) {
    if (index > 6) return NULL;
    return controller_string_descriptor(instance, index, language_id);
}
#endif

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                              hid_report_type_t report_type, uint8_t* buffer,
                              uint16_t requested_length) {
#ifdef SWITCH2_PROBE_USB_INIT
    uint8_t input[PROBE_INPUT_SIZE];
    if (instance < PROBE_CONTROLLER_COUNT && report_type == HID_REPORT_TYPE_INPUT &&
        probe_protocol_report(&controllers[instance].protocol, report_id, input, sizeof(input))) {
        const uint16_t size = requested_length < sizeof(input) ? requested_length : sizeof(input);
#if defined(SWITCH2_PROBE_JOIN_CHORD_GATE) && !SWITCH2_PROBE_HUB
        gate_join_shoulders(instance, report_id, input);
#endif
        memcpy(buffer, input, size);
        probe_debug_printf("[PROBE] GET_REPORT itf=%u id=%02x diagnostic length=%u\n",
                           instance, report_id, size);
        return size;
    }
#else
    (void)buffer;
#endif
    probe_debug_printf("[PROBE] GET_REPORT itf=%u report=%02x type=%u length=%u -> STALL\n",
                       instance, report_id, report_type, requested_length);
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type, const uint8_t* buffer,
                           uint16_t length) {
    ++hid_packets;
    probe_debug_printf("[PROBE] HID_REPORT_TYPE=%u\n", report_type);
    log_packet("HID_OUT", instance, report_id, buffer, length);
}

#ifdef SWITCH2_PROBE_USB_INIT
static bool save_pairing(void* context, const uint8_t* data, size_t length) {
    const probe_usb_controller* controller = context;
    const bool saved = probe_storage_save(controller->instance, data, length);
    probe_debug_printf("[PROBE] Virtual pairing persistence itf=%u %s\n",
                       controller->instance, saved ? "verified" : "failed");
    return saved;
}

#ifdef SWITCH2_PROBE_MEMORY
static bool read_memory(void* context, uint32_t address, uint8_t* data, size_t length) {
    const probe_usb_controller* controller = context;
    return probe_memory_read(controller->instance, address, data, length);
}
#endif

#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
static bool play_sample(void* context, uint8_t sample_id, uint64_t* token) {
    const probe_usb_controller* controller = context;
    return probe_controller_input_play_sample(controller->instance, sample_id, token);
}
#endif

static void reset_controller_protocol(uint8_t instance) {
    probe_usb_controller* controller = &controllers[instance];
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
    probe_controller_input_cancel_sample(instance);
    probe_controller_input_set_native_stream(instance, false);
#endif
    memset(controller, 0, sizeof(*controller));
    controller->instance = instance;
    controller->command_expected = 8;
    probe_protocol_state* protocol = &controller->protocol;
    probe_protocol_reset(protocol, probe_model_is_left(instance));
    protocol->context = controller;
    memcpy(protocol->controller_address, probe_version_replies[instance] + 10,
           sizeof(protocol->controller_address));
    protocol->firmware_version = probe_firmware_versions[instance];
    protocol->save_pairing = save_pairing;
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
    protocol->play_sample = play_sample;
#endif
#ifdef SWITCH2_PROBE_MEMORY
    uint8_t stick_calibration[9];
    if (!probe_memory_stick_calibration(instance, stick_calibration))
        panic("Invalid captured Joy-Con stick calibration");
    memcpy(protocol->stick_center, stick_calibration, sizeof(protocol->stick_center));
#if SWITCH2_BRIDGE_WII_INPUT
    probe_controller_input_set_stick_calibration(stick_calibration);
    probe_controller_input_set_native_features(0);
#endif
    protocol->read_memory = read_memory;
#endif
    uint8_t pairing[PROBE_PAIRING_BLOB_SIZE];
    if (probe_storage_load(instance, pairing, sizeof(pairing)) &&
        probe_protocol_restore_pairing(protocol, pairing, sizeof(pairing))) {
        probe_debug_printf("[PROBE] Restored own virtual pairing record itf=%u\n", instance);
    }
}

static void reset_protocol(void) {
#if defined(SWITCH2_PROBE_JOIN_CHORD_GATE) && !SWITCH2_PROBE_HUB
    last_join_shoulder_mask = 0;
#endif
    for (uint8_t instance = 0; instance < PROBE_CONTROLLER_COUNT; ++instance)
        reset_controller_protocol(instance);
}

static void complete_command(probe_usb_controller* controller) {
    const uint8_t instance = controller->instance;
    probe_protocol_state* protocol = &controller->protocol;
    const uint8_t* command = controller->command_frame;
    // Log complete frames rather than fragments so key material can be redacted.
    log_packet("BULK_OUT", instance, 0, command, controller->command_used);
    if (controller->reply_count == REPLY_CAPACITY) {
        ++command_drops;
        ++controller->command_drops;
        probe_debug_printf("[PROBE] Command not executed itf=%u: reply queue full\n", instance);
        return;
    }
    queued_reply* reply = &controller->replies[
        (controller->reply_head + controller->reply_count) % REPLY_CAPACITY];
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
    const uint8_t previous_report_id = protocol->report_id;
    const uint8_t previous_features = protocol->enabled_features;
#endif
    const size_t length = probe_protocol_command(
        protocol, command, controller->command_used, reply->data, sizeof(reply->data),
        &reply->deferred_token);
    if (length) {
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
        if (previous_report_id != protocol->report_id ||
            previous_features != protocol->enabled_features) {
            probe_controller_input_set_native_stream(instance, false);
            controller->native_stream_ready = false;
#if SWITCH2_BRIDGE_WII_INPUT
            probe_controller_input_set_native_features(protocol->enabled_features);
#endif
        }
#endif
        reply->length = (uint8_t)length;
        ++controller->reply_count;
        if (reply->deferred_token) {
            probe_debug_printf("[PROBE] Sample %u itf=%u awaiting source token=%" PRIu64 "\n",
                               command[8], instance, reply->deferred_token);
        } else {
            log_packet("BULK_REPLY_QUEUED", instance, 0, reply->data, reply->length);
        }
        if (command[0] == 0x09)
            probe_debug_printf("[PROBE] Virtual player LEDs itf=%u mask=%x flashing=%u\n",
                               instance, protocol->player_leds, protocol->player_leds_flashing);
        if (command[0] == 0x0c)
            probe_debug_printf("[PROBE] Virtual features itf=%u mask=%02x enabled=%02x\n",
                               instance, protocol->feature_mask, protocol->enabled_features);
        if (command[0] == 0x0a && command[3] == 8)
            probe_debug_printf("[PROBE] Virtual vibration parameters itf=%u stored; no motor output\n", instance);
        if (command[0] == 0x03 && command[3] == 0x0c)
            probe_debug_printf("[PROBE] Runtime 03/0C itf=%u value=%u\n", instance, protocol->runtime03_0c);
    } else {
        probe_debug_printf("[PROBE] Command itf=%u %02x/%02x length=%u capture-only or malformed\n",
                           instance, command[0], command[3], controller->command_used);
    }
}

static void consume_bulk_packet(probe_usb_controller* controller, const uint8_t* buffer,
                                uint16_t length) {
    for (uint16_t i = 0; i < length; ++i) {
        controller->command_frame[controller->command_used++] = buffer[i];
        if (controller->command_used == 8)
            controller->command_expected = (uint16_t)(8 + controller->command_frame[5]);
        if (controller->command_used == controller->command_expected) {
            complete_command(controller);
            controller->command_used = 0;
            controller->command_expected = 8;
        }
    }
    // A short packet/ZLP ends this interface's OUT transfer, not its sibling's.
    if (length < CFG_TUD_VENDOR_EPSIZE && controller->command_used) {
        probe_debug_printf("[PROBE] Truncated command discarded itf=%u received=%u expected=%u\n",
                           controller->instance, controller->command_used, controller->command_expected);
        controller->command_used = 0;
        controller->command_expected = 8;
    }
}

#ifndef SWITCH_PICO_SWITCH2_USB_BRIDGE
static void button_test_task(probe_usb_controller* controller, uint32_t now) {
    probe_protocol_state* protocol = &controller->protocol;
    const bool ready = probe_transport_mounted(controller->instance) &&
                       !probe_transport_suspended(controller->instance) &&
                       protocol->initialized && (protocol->enabled_features & 1);
    bool pressed;
    if (!ready) {
        pressed = probe_button_update(&controller->button_test, -1, false);
        controller->last_button_ms = now;
    } else {
        if (now - controller->last_button_ms < 10) return;
        controller->last_button_ms = now;
        const int sample = bootsel_button_sample();
        if (sample < 0 && !controller->button_sample_error)
            probe_debug_printf("[PROBE] BOOTSEL sampling unavailable; test buttons released and disarmed\n");
        controller->button_sample_error = sample < 0;
        pressed = probe_button_update(&controller->button_test, sample, true);
    }
    if (protocol->test_rail_buttons != pressed) {
        protocol->test_rail_buttons = pressed;
        probe_debug_printf("[PROBE] TEST_BUTTON SL+SR %s\n", pressed ? "pressed" : "released");
    }
}
#endif

#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
static void controller_input_task(probe_usb_controller* controller, uint32_t now) {
    if (now - controller->last_controller_poll_ms < 4) return;
    controller->last_controller_poll_ms = now;
    const uint8_t instance = controller->instance;
    probe_protocol_state* protocol = &controller->protocol;
    probe_controller_input source = {0};
    probe_controller_input_poll(instance, now, &source);
    const bool output_active = source.active && probe_transport_mounted(instance) &&
                               !probe_transport_suspended(instance);
    if (protocol->controller_active != output_active)
        probe_debug_printf("[PROBE] Controller input itf=%u %s\n",
                           instance, output_active ? "active" : "neutral (disconnected/stale)");
    protocol->controller_active = output_active;
    if (output_active) {
        memcpy(protocol->controller_buttons, source.buttons, sizeof(source.buttons));
        memcpy(protocol->controller_stick, source.stick, sizeof(source.stick));
    } else {
        memset(protocol->controller_buttons, 0, sizeof(protocol->controller_buttons));
    }
    // Each HID endpoint must make its own progress before consuming donor packets.
    controller->native_stream_ready = probe_transport_mounted(instance) &&
        !probe_transport_suspended(instance) &&
        protocol->initialized && protocol->report_id == probe_model_report_id(instance) &&
        controller->hid_completion_seen &&
        (uint32_t)(now - controller->last_hid_complete_ms) < 500;
    probe_controller_input_set_native_stream(instance, controller->native_stream_ready);
}

#endif

static void protocol_task(probe_usb_controller* controller, uint32_t now) {
    const uint8_t instance = controller->instance;
    if (!probe_transport_mounted(instance) || probe_transport_suspended(instance)) return;
    probe_protocol_state* protocol = &controller->protocol;
    if (controller->reply_count && !controller->reply_inflight) {
        queued_reply* reply = &controller->replies[controller->reply_head];
        bool ready = reply->deferred_token == 0;
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
        if (!ready) {
            const int result = probe_controller_input_sample_result(instance, reply->deferred_token, now);
            if (result > 0) {
#if SWITCH2_BRIDGE_WII_INPUT
                probe_debug_printf("[PROBE] Wii cue dispatched itf=%u token=%" PRIu64 "\n",
                                   instance, reply->deferred_token);
#else
                probe_debug_printf("[PROBE] Source sample ACK itf=%u token=%" PRIu64 "\n",
                                   instance, reply->deferred_token);
#endif
                reply->deferred_token = 0;
                ready = true;
                log_packet("BULK_REPLY_QUEUED", instance, 0, reply->data, reply->length);
            } else if (result < 0) {
                probe_debug_printf("[PROBE] Source sample failed itf=%u token=%" PRIu64 "; no USB ACK\n",
                                   instance, reply->deferred_token);
                controller->reply_head = (uint8_t)((controller->reply_head + 1) % REPLY_CAPACITY);
                --controller->reply_count;
                ++controller->command_drops;
                ++command_drops;
            }
        }
#endif
        if (ready && probe_transport_vendor_write_available(instance) >= reply->length) {
            if (probe_transport_vendor_write(instance, reply->data, reply->length) == reply->length) {
                controller->reply_inflight = true;
                controller->reply_remaining = reply->length;
                probe_transport_vendor_write_flush(instance);
                controller->reply_head = (uint8_t)((controller->reply_head + 1) % REPLY_CAPACITY);
                --controller->reply_count;
            }
        }
    }
    if (protocol->initialized && now - controller->last_input_ms >= 4 &&
        probe_transport_hid_ready(instance)) {
        uint8_t input[PROBE_INPUT_SIZE];
        size_t length = 0;
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
        uint32_t native_serial = 0;
        if (protocol->report_id == probe_model_report_id(instance) &&
            protocol->controller_active && controller->native_stream_ready) {
            native_serial = probe_controller_input_peek_native_report(instance, now, input);
            if (!native_serial) return;
            length = sizeof(input);
#if SWITCH2_BRIDGE_WII_INPUT
            input[8] = (uint8_t)(0x30 | ((protocol->enabled_features & 0x20) ? 8 : 0));
#endif
            probe_protocol_gate_native_report(protocol, input);
        }
#endif
        if (!length)
            length = probe_protocol_report(protocol, protocol->report_id, input, sizeof(input));
#if defined(SWITCH2_PROBE_JOIN_CHORD_GATE) && !SWITCH2_PROBE_HUB
        if (length) gate_join_shoulders(instance, protocol->report_id, input);
#endif
        if (length && probe_transport_hid_report(instance, protocol->report_id, input, (uint16_t)length)) {
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
            if (native_serial) {
                if (!probe_controller_input_commit_native_report(instance, native_serial))
                    probe_debug_printf("[PROBE] Native stream changed during HID submission itf=%u\n", instance);
                protocol->report_counter = input[0];
            }
#endif
            ++protocol->report_counter;
            ++controller->input_reports;
            ++input_reports;
            controller->last_input_ms = now;
        }
    }
}
#endif

#if SWITCH2_PROBE_HUB
void native_joycon_usb_reset(uint8_t instance) {
    if (instance >= PROBE_CONTROLLER_COUNT) return;
#ifdef SWITCH2_PROBE_USB_INIT
    reset_controller_protocol(instance);
#endif
    probe_debug_printf("[PROBE] USB_RESET itf=%u\n", instance);
}
#endif

#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
static int32_t signed_mouse_delta(const uint8_t* data) {
    const uint16_t raw = (uint16_t)(data[0] | ((uint16_t)data[1] << 8));
    return raw >= 0x8000 ? (int32_t)raw - 0x10000 : raw;
}

void tud_hid_report_failed_cb(uint8_t instance, hid_report_type_t report_type,
                             const uint8_t* report, uint16_t length) {
    (void)report;
    (void)length;
    if (instance < PROBE_CONTROLLER_COUNT && report_type == HID_REPORT_TYPE_INPUT) {
        probe_controller_input_set_native_stream(instance, false);
        controllers[instance].native_stream_ready = false;
        controllers[instance].hid_completion_seen = false;
        probe_debug_printf("[PROBE] HID input transfer failed itf=%u; pending native packets discarded\n", instance);
    }
}
#endif

void tud_hid_report_complete_cb(uint8_t instance, const uint8_t* report, uint16_t length) {
#ifdef SWITCH2_PROBE_USB_INIT
    if (instance >= PROBE_CONTROLLER_COUNT || length != PROBE_INPUT_SIZE + 1) return;
    probe_usb_controller* controller = &controllers[instance];
    const uint8_t native_id = probe_model_report_id(instance);
    const uint8_t common_buttons_offset = probe_model_is_left(instance) ? 7 : 5;
    uint8_t rails;
    if (report[0] == native_id) rails = (report[4] >> 6) & 3;
    else if (report[0] == 0x05) rails = (report[common_buttons_offset] >> 4) & 3;
    else return;
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
    controller->last_hid_complete_ms = to_ms_since_boot(get_absolute_time());
    controller->hid_completion_seen = true;
    if (report[0] == native_id) {
        const int32_t dx = signed_mouse_delta(report + 10);
        const int32_t dy = signed_mouse_delta(report + 12);
#ifdef SWITCH2_PROBE_TRACE_NATIVE_INPUT
        if ((uint32_t)(controller->last_hid_complete_ms - controller->last_native_trace_ms) >= 1000) {
            controller->last_native_trace_ms = controller->last_hid_complete_ms;
            log_packet("NATIVE_INPUT_DELIVERED", instance, report[0], report, length);
        }
#endif
        if (dx || dy) {
            ++controller->mouse_delivered_reports;
            controller->mouse_delivered_x += dx;
            controller->mouse_delivered_y += dy;
        }
    }
    const uint16_t buttons = report[0] == native_id ?
        (uint16_t)(report[3] | ((uint16_t)report[4] << 8)) :
        (uint16_t)(report[common_buttons_offset] | ((uint16_t)report[6] << 8));
    if (buttons != controller->last_delivered_buttons) {
        controller->last_delivered_buttons = buttons;
        probe_debug_printf("[PROBE] CONTROLLER_REPORT delivered itf=%u id=%02x buttons=%04x\n",
                           instance, report[0], buttons);
    }
#endif
    if (rails != controller->last_delivered_rails) {
        controller->last_delivered_rails = rails;
        probe_debug_printf("[PROBE] TEST_REPORT delivered itf=%u id=%02x SL=%u SR=%u\n",
                           instance, report[0], (rails >> 1) & 1, rails & 1);
    }
#else
    (void)instance;
    (void)report;
    (void)length;
#endif
}

void tud_vendor_rx_cb(uint8_t instance, const uint8_t* buffer, uint16_t length) {
    ++bulk_packets;
    if (instance >= PROBE_CONTROLLER_COUNT) return;
#ifdef SWITCH2_PROBE_USB_INIT
    consume_bulk_packet(&controllers[instance], buffer, length);
#else
    log_packet("BULK_OUT", instance, 0, buffer, length);
#endif
    probe_transport_vendor_discard_received(instance);
}

void tud_vendor_tx_cb(uint8_t instance, uint32_t length) {
#ifdef SWITCH2_PROBE_USB_INIT
    if (instance >= PROBE_CONTROLLER_COUNT) return;
    probe_usb_controller* controller = &controllers[instance];
    if (controller->reply_inflight) {
        if (length <= controller->reply_remaining) {
            controller->reply_remaining -= (uint16_t)length;
            // Exact packet multiples finish only after the transport's ZLP.
            if (controller->reply_remaining == 0 && length < CFG_TUD_VENDOR_EPSIZE)
                controller->reply_inflight = false;
        } else {
            probe_debug_printf("[PROBE] Unexpected bulk completion itf=%u pending=%u\n",
                               instance, controller->reply_remaining);
        }
    }
#endif
    probe_debug_printf("[PROBE] BULK_TX_COMPLETE itf=%u length=%" PRIu32 "\n", instance, length);
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                const tusb_control_request_t* request) {
    if (stage == CONTROL_STAGE_SETUP)
        log_packet("VENDOR_CONTROL", rhport, 0, (const uint8_t*)request, sizeof(*request));
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
    if (probe_bootsel_vendor_control(rhport, stage, request))
        return true;
#endif
#if SWITCH2_PROBE_HUB
    // Each USB address owns native interfaces 0/1; rhport is its device slot.
    if (rhport < 1 || rhport > PROBE_CONTROLLER_COUNT || request->wIndex >= 2) return false;
    const uint8_t instance = (uint8_t)(rhport - 1);
#else
    // Device-level index zero names the primary identity. Explicit interface
    // indexes can select the sibling; never infer an identity from timing.
    if (request->wIndex >= 2 * PROBE_CONTROLLER_COUNT) return false;
    const uint8_t instance = (uint8_t)(request->wIndex / 2);
#endif
#ifdef SWITCH2_PROBE_IDENTITY_REPLY
    if ((request->bmRequestType == 0xc0 || request->bmRequestType == 0xc1) &&
        request->bRequest == 0x03 && request->wValue == 0) {
        if (stage == CONTROL_STAGE_SETUP) {
            ++identity_requests;
            log_packet("IDENTITY_REPLY", instance, 3, probe_identity_replies[instance],
                       sizeof(probe_identity_replies[instance]));
            return probe_transport_control_xfer(rhport, request, (void*)probe_identity_replies[instance],
                                    sizeof(probe_identity_replies[instance]));
        }
        return true;
    }
#endif
#ifdef SWITCH2_PROBE_VERSION_REPLY
    if ((request->bmRequestType == 0xc0 || request->bmRequestType == 0xc1) &&
        request->bRequest == 0x02 && request->wValue == 0) {
        if (stage == CONTROL_STAGE_SETUP) {
            ++version_requests;
            log_packet("VERSION_REPLY", instance, 2, probe_version_replies[instance],
                       sizeof(probe_version_replies[instance]));
            return probe_transport_control_xfer(rhport, request, (void*)probe_version_replies[instance],
                                    sizeof(probe_version_replies[instance]));
        }
        return true;
    }
#endif
#ifdef SWITCH2_PROBE_ACK_SETUP04
    // Preserve the observed setup payload; its parameter semantics are unknown.
    if ((request->bmRequestType == 0x40 || request->bmRequestType == 0x41) &&
        request->bRequest == 0x04 && request->wValue == 0x0276 && request->wLength == 0) {
        if (stage == CONTROL_STAGE_SETUP)
            return probe_transport_control_status(rhport, request);
        if (stage == CONTROL_STAGE_ACK) {
            ++setup_completions;
            probe_debug_printf("[PROBE] SETUP04 itf=%u acknowledged value=%04x\n",
                               instance, request->wValue);
        }
        return true;
    }
#endif
    return false;
}

#if !SWITCH2_PROBE_HUB
void tud_mount_cb(void) {
#ifdef SWITCH2_PROBE_USB_INIT
    reset_protocol();
#endif
    probe_debug_printf("[PROBE] MOUNT\n");
}
void tud_umount_cb(void) {
#ifdef SWITCH2_PROBE_USB_INIT
    reset_protocol();
#endif
    probe_debug_printf("[PROBE] UNMOUNT\n");
}
void tud_suspend_cb(bool remote_wakeup_en) {
    probe_debug_printf("[PROBE] SUSPEND wake=%u\n", remote_wakeup_en);
#ifdef SWITCH2_PROBE_USB_INIT
    for (uint8_t instance = 0; instance < PROBE_CONTROLLER_COUNT; ++instance) {
        probe_usb_controller* controller = &controllers[instance];
#ifndef SWITCH_PICO_SWITCH2_USB_BRIDGE
        (void)probe_button_update(&controller->button_test, -1, false);
#endif
        controller->protocol.test_rail_buttons = false;
        controller->protocol.controller_active = false;
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
        probe_controller_input_set_native_stream(instance, false);
        controller->native_stream_ready = false;
        controller->hid_completion_seen = false;
#endif
    }
#endif
}
void tud_resume_cb(void) { probe_debug_printf("[PROBE] RESUME\n"); }
#endif

int main(void) {
    #if SWITCH2_PROBE_HUB
    native_hub_startup_guard();
    #endif
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
    probe_controller_input_clock_init();
#endif
    stdio_init_all();
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
    probe_debug_printf("\n[PROBE] " PROBE_JOYCON_PRODUCT " Bluetooth-to-USB controller/native mouse bridge\n");
#else
    probe_debug_printf("\n[PROBE] " PROBE_JOYCON_PRODUCT " USB enumeration recorder\n");
#endif
#if SWITCH2_PROBE_HUB
    probe_debug_printf("[PROBE] NATIVE_HUB: device1 right PID2066, device2 left PID2067; each HID0/vendor1 EP1/2; no shoulder gate\n");
#endif
#if SWITCH2_PROBE_COMPOSITE
#ifdef SWITCH2_PROBE_JOIN_CHORD_GATE
    probe_debug_printf("[PROBE] JOIN_CHORD_GATE enabled: physical L+R required; no synthesized presses or USB initialization\n");
#endif
    probe_debug_printf("[PROBE] COMPOSITE: right HID0/vendor1, left HID2/vendor3; shared USB PID2066\n");
#endif
#ifdef SWITCH2_PROBE_OMIT_NATIVE_IMU
    probe_debug_printf("[PROBE] ACTIVATION_TEST=no-imu: native%02x bytes %u..%u omitted; features, power, status, counters and cadence unchanged\n",
                       PROBE_NATIVE_REPORT_ID, PROBE_IMU_LENGTH_OFFSET, PROBE_IMU_DATA_OFFSET + 39u);
#elif defined(SWITCH2_PROBE_ZERO_NATIVE_IMU_PAYLOAD)
    probe_debug_printf("[PROBE] ACTIVATION_TEST=zero-imu-payload: native%02x bytes %u..%u zeroed; length, features and all other fields unchanged\n",
                       PROBE_NATIVE_REPORT_ID, PROBE_IMU_DATA_OFFSET, PROBE_IMU_DATA_OFFSET + 39u);
#endif
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
    probe_controller_input_init();
#if SWITCH2_BRIDGE_WII_INPUT
    probe_debug_printf("[PROBE] UART0 GP0=TX, 115200 8N1; selected Wii IR/MotionPlus source enabled\n");
#else
    probe_debug_printf("[PROBE] UART0 GP0=TX, 115200 8N1; %u selected Joy-Con Bluetooth source(s)\n",
                       PROBE_CONTROLLER_COUNT);
#endif
#else
    probe_debug_printf("[PROBE] UART0 GP0=TX, 115200 8N1; Bluetooth disabled\n");
#endif
#ifdef SWITCH2_PROBE_IDENTITY_REPLY
    probe_debug_printf("[PROBE] Configured factory-format identity reply enabled for vendor read 03\n");
#else
    probe_debug_printf("[PROBE] Factory identity reply disabled\n");
#endif
#ifdef SWITCH2_PROBE_VERSION_REPLY
    probe_debug_printf("[PROBE] Captured firmware version with configured controller address enabled\n");
#endif
#ifdef SWITCH2_PROBE_ACK_SETUP04
    probe_debug_printf("[PROBE] Observed vendor-04 setup acknowledgement enabled\n");
#endif
#ifdef SWITCH2_PROBE_USB_INIT
    reset_protocol();
    for (uint8_t instance = 0; instance < PROBE_CONTROLLER_COUNT; ++instance)
        probe_debug_printf("[PROBE] Own virtual pairing itf=%u offset=%08" PRIx32 "\n",
                           instance, probe_storage_offset(instance));
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
#if SWITCH2_BRIDGE_WII_INPUT
    probe_debug_printf("[PROBE] Wii IR drives native mouse movement; buttons retain profile mapping; keep Wii still for MotionPlus calibration\n");
    probe_debug_printf("[PROBE] Hold BOOTSEL2s for pairing; Wii cue feedback uses bounded ERM patterns, not HD audio waveforms\n");
#else
    probe_debug_printf("[PROBE] Live Joy-Con buttons/stick/native mouse; hold BOOTSEL 2s for Bluetooth pairing (never clears pairings)\n");
#endif
#else
    probe_debug_printf("[PROBE] Manual input test: hold BOOTSEL for SL+SR, release for neutral; no controller forwarding\n");
#endif
#else
    probe_debug_printf("[PROBE] Bulk commands are capture-only; no input reports\n");
#endif
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
    if (!probe_controller_input_start())
        panic("Bluetooth source startup failed");
#endif
#if SWITCH2_PROBE_HUB
    if (!native_hub_init())
        panic("Native hub startup failed");
#else
    tud_init(0);
#endif
    uint32_t last_heartbeat = 0;
    while (true) {
#if SWITCH2_PROBE_HUB
        probe_controller_input_task();
        native_hub_task();
#else
        tud_task();
#endif
        drain_log();
        const uint32_t now = to_ms_since_boot(get_absolute_time());
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
        probe_bootsel_task(now);
#endif
#ifdef SWITCH2_PROBE_USB_INIT
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
        if (probe_controller_input_pairing_task())
            probe_debug_printf("[PROBE] Bluetooth pairing window requested; put the selected source in SYNC pairing mode\n");
#endif
        for (uint8_t instance = 0; instance < PROBE_CONTROLLER_COUNT; ++instance) {
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
            controller_input_task(&controllers[instance], now);
#else
            button_test_task(&controllers[instance], now);
#endif
#if !defined(SWITCH2_PROBE_JOIN_CHORD_GATE) || SWITCH2_PROBE_HUB
            protocol_task(&controllers[instance], now);
#endif
        }
#if defined(SWITCH2_PROBE_JOIN_CHORD_GATE) && !SWITCH2_PROBE_HUB
        const uint8_t shoulder_mask = join_shoulder_mask();
        if (shoulder_mask != last_join_shoulder_mask) {
            last_join_shoulder_mask = shoulder_mask;
            probe_debug_printf("[PROBE %" PRIu32 "] JOIN_CHORD right=%u left=%u open=%u initialized=%u/%u\n",
                               now, (shoulder_mask & 1) != 0, (shoulder_mask & 2) != 0,
                               shoulder_mask == 3, controllers[0].protocol.initialized,
                               controllers[1].protocol.initialized);
        }
        // Poll both sources before either USB submission observes the chord.
        for (uint8_t instance = 0; instance < PROBE_CONTROLLER_COUNT; ++instance)
            protocol_task(&controllers[instance], now);
#endif
#endif
        if (now - last_heartbeat >= 1000) {
            last_heartbeat = now;
            probe_debug_printf("[PROBE %" PRIu32 "] alive mounted=%u bulk=%" PRIu32
                               " hid=%" PRIu32 " identity=%" PRIu32
                               " version=%" PRIu32 " setup=%" PRIu32
                               " inputs=%" PRIu32 " command_drops=%" PRIu32
                               " log_dropped_bytes=%" PRIu32 "\n",
                               now, probe_transport_mounted(0), bulk_packets, hid_packets,
                               identity_requests, version_requests, setup_completions,
                               input_reports, command_drops, log_dropped);
#ifdef SWITCH2_PROBE_USB_INIT
            for (uint8_t instance = 0; instance < PROBE_CONTROLLER_COUNT; ++instance) {
                probe_usb_controller* controller = &controllers[instance];
                probe_debug_printf("[PROBE] CHANNEL itf=%u mounted=%u suspended=%u initialized=%u active=%u report=%02x LEDs=%x inputs=%" PRIu32
                                   " drops=%" PRIu32 "\n", instance, probe_transport_mounted(instance),
                                   probe_transport_suspended(instance), controller->protocol.initialized,
                                   controller->protocol.controller_active, controller->protocol.report_id,
                                   controller->protocol.player_leds, controller->input_reports,
                                   controller->command_drops);
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
                if (controller->mouse_delivered_reports != controller->mouse_logged_reports) {
                    controller->mouse_logged_reports = controller->mouse_delivered_reports;
                    probe_debug_printf("[PROBE] MOUSE_REPORT itf=%u packets=%" PRIu32
                                       " total_x=%" PRId64 " total_y=%" PRId64 "\n", instance,
                                       controller->mouse_delivered_reports,
                                       controller->mouse_delivered_x, controller->mouse_delivered_y);
                }
#endif
            }
#endif
        }
        sleep_us(100);
    }
}
