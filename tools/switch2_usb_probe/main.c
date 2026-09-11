// Joy-Con 2 (R) USB instrument and optional Bluetooth controller/mouse bridge.
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
#ifdef SWITCH2_PROBE_MEMORY
#include "memory.h"
#endif
#ifdef SWITCH2_PROBE_IDENTITY_REPLY
#include "probe_identity.h"
_Static_assert(sizeof(probe_identity_reply) == 64, "factory identity response size");
#endif
#ifdef SWITCH2_PROBE_VERSION_REPLY
#include "probe_version.h"
_Static_assert(sizeof(probe_version_reply) == 16, "version/address response size");
#endif

#define LOG_CAPACITY 8192u
static char log_bytes[LOG_CAPACITY];
static uint32_t log_written, log_read, log_dropped;
static uint32_t bulk_packets, hid_packets;
static uint32_t identity_requests, version_requests, setup_completions;
static uint16_t string_descriptor[64];
static uint32_t input_reports, command_drops;
#ifdef SWITCH2_PROBE_USB_INIT
#define REPLY_CAPACITY 4u
typedef struct {
    uint8_t data[PROBE_REPLY_MAX_SIZE];
    uint8_t length;
    uint64_t deferred_token;
} queued_reply;
static probe_protocol_state protocol;
static queued_reply replies[REPLY_CAPACITY];
static uint8_t reply_head, reply_count;
static bool reply_inflight;
static uint16_t reply_remaining;
static uint8_t command_frame[PROBE_COMMAND_MAX_SIZE];
static uint16_t command_used, command_expected = 8;
static uint32_t last_input_ms;
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
static uint32_t last_controller_poll_ms;
static uint16_t last_delivered_buttons;
static bool native_stream_ready;
static uint32_t last_hid_complete_ms;
static bool hid_completion_seen;
static uint32_t mouse_delivered_reports, mouse_logged_reports;
static int64_t mouse_delivered_x, mouse_delivered_y;
#ifdef SWITCH2_PROBE_TRACE_NATIVE_INPUT
static uint32_t last_native_trace_ms;
#endif
#endif
#ifndef SWITCH_PICO_SWITCH2_USB_BRIDGE
static probe_button_state button_test;
static uint32_t last_button_ms;
static bool button_sample_error;
#endif
static uint8_t last_delivered_rails;
#endif

_Static_assert(sizeof(probe_device_descriptor) == 18, "device descriptor size");
_Static_assert(sizeof(probe_configuration_descriptor) == 80, "configuration descriptor size");
_Static_assert(sizeof(probe_hid_report_descriptor) == 100, "HID descriptor size");

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
    probe_debug_printf("[PROBE] DEVICE_DESCRIPTOR 057e:2066\n");
    return probe_device_descriptor;
}

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    probe_debug_printf("[PROBE] CONFIG_DESCRIPTOR index=%u\n", index);
    return index == 0 ? probe_configuration_descriptor : NULL;
}

uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {
    probe_debug_printf("[PROBE] HID_DESCRIPTOR itf=%u\n", instance);
    return instance == 0 ? probe_hid_report_descriptor : NULL;
}

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    // Manufacturer/product/serial match the published reference. Remaining
    // descriptor labels describe the probe; their genuine strings are unknown.
    static const char* const strings[] = {
        "", "Nintendo", "Joy-Con 2 (R)", "00", "USB configuration", "HID", "Commands"};
    probe_debug_printf("[PROBE] STRING_DESCRIPTOR index=%u lang=%04x\n", index, langid);
    if (index == 0) {
        string_descriptor[0] = (TUSB_DESC_STRING << 8) | 4;
        string_descriptor[1] = 0x0409;
        return string_descriptor;
    }
    if (index >= sizeof(strings) / sizeof(strings[0])) return NULL;
    size_t count = strlen(strings[index]);
    if (count > 63) count = 63;
    string_descriptor[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 + count * 2));
    for (size_t i = 0; i < count; ++i)
        string_descriptor[i + 1] = (uint8_t)strings[index][i];
    return string_descriptor;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                              hid_report_type_t report_type, uint8_t* buffer,
                              uint16_t requested_length) {
#ifdef SWITCH2_PROBE_USB_INIT
    uint8_t input[PROBE_INPUT_SIZE];
    if (instance == 0 && report_type == HID_REPORT_TYPE_INPUT &&
        probe_protocol_report(&protocol, report_id, input, sizeof(input))) {
        const uint16_t size = requested_length < sizeof(input) ? requested_length : sizeof(input);
        memcpy(buffer, input, size);
        probe_debug_printf("[PROBE] GET_REPORT id=%02x diagnostic length=%u\n", report_id, size);
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
static bool save_pairing(const uint8_t* data, size_t length) {
    const bool saved = probe_storage_save(data, length);
    probe_debug_printf("[PROBE] Virtual pairing persistence %s\n", saved ? "verified" : "failed");
    return saved;
}

static void reset_protocol(void) {
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
    probe_controller_input_cancel_sample();
#endif
    probe_protocol_reset(&protocol);
    memcpy(protocol.controller_address, probe_version_reply + 10, sizeof(protocol.controller_address));
    protocol.firmware_version = probe_firmware_version;
    protocol.save_pairing = save_pairing;
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
    protocol.play_sample = probe_controller_input_play_sample;
#endif
#ifdef SWITCH2_PROBE_MEMORY
    uint8_t stick_calibration[9];
    if (!probe_memory_right_stick_calibration(stick_calibration))
        panic("Invalid captured Joy-Con stick calibration");
    memcpy(protocol.right_stick_center, stick_calibration, sizeof(protocol.right_stick_center));
#if SWITCH2_BRIDGE_WII_INPUT
    probe_controller_input_set_stick_calibration(stick_calibration);
#endif
    protocol.read_memory = probe_memory_read;
#endif
    uint8_t pairing[PROBE_PAIRING_BLOB_SIZE];
    if (probe_storage_load(pairing, sizeof(pairing)) &&
        probe_protocol_restore_pairing(&protocol, pairing, sizeof(pairing))) {
        probe_debug_printf("[PROBE] Restored own virtual pairing record\n");
    }
    reply_head = reply_count = 0;
    reply_inflight = false;
    reply_remaining = 0;
    command_used = 0;
    command_expected = 8;
    last_input_ms = 0;
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
#if SWITCH2_BRIDGE_WII_INPUT
    probe_controller_input_set_native_features(0);
#endif
    last_controller_poll_ms = 0;
    last_delivered_buttons = 0;
    probe_controller_input_set_native_stream(false);
    native_stream_ready = false;
    last_hid_complete_ms = 0;
    hid_completion_seen = false;
#ifdef SWITCH2_PROBE_TRACE_NATIVE_INPUT
    last_native_trace_ms = 0;
#endif
#endif
#ifndef SWITCH_PICO_SWITCH2_USB_BRIDGE
    (void)probe_button_update(&button_test, -1, false);
    last_button_ms = 0;
    button_sample_error = false;
#endif
    last_delivered_rails = 0;
}

static void complete_command(void) {
    // Log complete frames rather than fragments so key material can be redacted.
    log_packet("BULK_OUT", 0, 0, command_frame, command_used);
    if (reply_count == REPLY_CAPACITY) {
        ++command_drops;
        probe_debug_printf("[PROBE] Command captured but not executed: reply queue full\n");
        return;
    }
    queued_reply* reply = &replies[(reply_head + reply_count) % REPLY_CAPACITY];
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
    const uint8_t previous_report_id = protocol.report_id;
    const uint8_t previous_features = protocol.enabled_features;
#endif
    const size_t length = probe_protocol_command(
        &protocol, command_frame, command_used, reply->data, sizeof(reply->data),
        &reply->deferred_token);
    if (length) {
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
        if (previous_report_id != protocol.report_id ||
            previous_features != protocol.enabled_features) {
            probe_controller_input_set_native_stream(false);
            native_stream_ready = false;
#if SWITCH2_BRIDGE_WII_INPUT
            probe_controller_input_set_native_features(protocol.enabled_features);
#endif
        }
#endif
        reply->length = (uint8_t)length;
        ++reply_count;
        if (reply->deferred_token) {
            probe_debug_printf("[PROBE] Sample %u awaiting source completion token=%" PRIu64 "\n",
                               command_frame[8], reply->deferred_token);
        } else {
            log_packet("BULK_REPLY_QUEUED", 0, 0, reply->data, reply->length);
        }
        if (command_frame[0] == 0x09) {
            probe_debug_printf("[PROBE] Virtual player LEDs mask=%x flashing=%u\n",
                               protocol.player_leds, protocol.player_leds_flashing);
        }
        if (command_frame[0] == 0x0c) {
            probe_debug_printf("[PROBE] Virtual features mask=%02x enabled=%02x\n",
                               protocol.feature_mask, protocol.enabled_features);
        }
        if (command_frame[0] == 0x0a && command_frame[3] == 8)
            probe_debug_printf("[PROBE] Virtual vibration parameters stored; no motor output\n");
        if (command_frame[0] == 0x03 && command_frame[3] == 0x0c)
            probe_debug_printf("[PROBE] Runtime 03/0C value=%u\n", protocol.runtime03_0c);
    } else {
        probe_debug_printf("[PROBE] Command %02x/%02x length=%u capture-only or malformed\n",
                           command_frame[0], command_frame[3], command_used);
    }
}

static void consume_bulk_packet(const uint8_t* buffer, uint16_t length) {
    for (uint16_t i = 0; i < length; ++i) {
        command_frame[command_used++] = buffer[i];
        if (command_used == 8) command_expected = (uint16_t)(8 + command_frame[5]);
        if (command_used == command_expected) {
            complete_command();
            command_used = 0;
            command_expected = 8;
        }
    }
    // A short USB packet/ZLP ends an OUT transfer. Never let a malformed
    // truncated frame consume a subsequent independent host command.
    if (length < CFG_TUD_VENDOR_EPSIZE && command_used) {
        probe_debug_printf("[PROBE] Truncated command discarded: received=%u expected=%u\n",
                           command_used, command_expected);
        command_used = 0;
        command_expected = 8;
    }
}

#ifndef SWITCH_PICO_SWITCH2_USB_BRIDGE
static void button_test_task(uint32_t now) {
    const bool ready = tud_mounted() && !tud_suspended() &&
                       protocol.initialized && (protocol.enabled_features & 1);
    bool pressed;
    if (!ready) {
        pressed = probe_button_update(&button_test, -1, false);
        last_button_ms = now;
    } else {
        if (now - last_button_ms < 10) return;
        last_button_ms = now;
        const int sample = bootsel_button_sample();
        if (sample < 0 && !button_sample_error)
            probe_debug_printf("[PROBE] BOOTSEL sampling unavailable; test buttons released and disarmed\n");
        button_sample_error = sample < 0;
        pressed = probe_button_update(&button_test, sample, true);
    }
    if (protocol.test_rail_buttons != pressed) {
        protocol.test_rail_buttons = pressed;
        probe_debug_printf("[PROBE] TEST_BUTTON SL+SR %s\n", pressed ? "pressed" : "released");
    }
}
#endif

#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
static void controller_input_task(uint32_t now) {
    if (now - last_controller_poll_ms < 4) return;
    last_controller_poll_ms = now;
    probe_controller_input source = {0};
    probe_controller_input_poll(now, &source);
    const bool output_active = source.active && tud_mounted() && !tud_suspended();
    if (protocol.controller_active != output_active)
        probe_debug_printf("[PROBE] Controller input %s\n", output_active ? "active" : "neutral (disconnected/stale)");
    protocol.controller_active = output_active;
    if (output_active) {
        memcpy(protocol.controller_buttons, source.buttons, sizeof(source.buttons));
        memcpy(protocol.controller_stick, source.stick, sizeof(source.stick));
    } else {
        memset(protocol.controller_buttons, 0, sizeof(protocol.controller_buttons));
    }
    // Bootstrap with diagnostic reports until the host polls HID. Then consume
    // complete source packets once, including opaque packed motion samples.
    native_stream_ready = tud_mounted() && !tud_suspended() && protocol.initialized &&
        protocol.report_id == 0x08 && hid_completion_seen &&
        (uint32_t)(now - last_hid_complete_ms) < 500;
    probe_controller_input_set_native_stream(native_stream_ready);
}

static void gate_native_report(uint8_t input[PROBE_INPUT_SIZE]) {
#if SWITCH2_BRIDGE_WII_INPUT
    // Generated status follows virtual feature state, not a donor snapshot.
    input[8] = (uint8_t)(0x30 | ((protocol.enabled_features & 0x20) ? 8 : 0));
#endif
    if (!(protocol.enabled_features & 1)) memset(input + 2, 0, 2);
    if (!(protocol.enabled_features & 2))
        memcpy(input + 5, protocol.right_stick_center, sizeof(protocol.right_stick_center));
    if (!(protocol.enabled_features & 0x10)) memset(input + 9, 0, 5);
#ifdef SWITCH2_PROBE_OMIT_NATIVE_IMU
    // Deliberate A/B fault injection: leave every other field and feature bit intact.
    memset(input + 15, 0, 41);
#elif defined(SWITCH2_PROBE_ZERO_NATIVE_IMU_PAYLOAD)
    if (!(protocol.enabled_features & 4)) input[15] = 0;
    memset(input + 16, 0, 40); // Otherwise preserve the genuine length byte.
#else
    if (!(protocol.enabled_features & 4)) memset(input + 15, 0, 41);
#endif
}
#endif

static void protocol_task(uint32_t now) {
    if (!tud_mounted() || tud_suspended()) return;
    if (reply_count && !reply_inflight) {
        queued_reply* reply = &replies[reply_head];
        bool ready = reply->deferred_token == 0;
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
        if (!ready) {
            const int result = probe_controller_input_sample_result(reply->deferred_token, now);
            if (result > 0) {
#if SWITCH2_BRIDGE_WII_INPUT
                probe_debug_printf("[PROBE] Wii cue dispatched token=%" PRIu64 "\n",
                                   reply->deferred_token);
#else
                probe_debug_printf("[PROBE] Source sample ACK token=%" PRIu64 "\n",
                                   reply->deferred_token);
#endif
                reply->deferred_token = 0;
                ready = true;
                log_packet("BULK_REPLY_QUEUED", 0, 0, reply->data, reply->length);
            } else if (result < 0) {
                probe_debug_printf("[PROBE] Source sample failed or expired token=%" PRIu64
                                   "; no USB ACK\n", reply->deferred_token);
                reply_head = (uint8_t)((reply_head + 1) % REPLY_CAPACITY);
                --reply_count;
                ++command_drops;
            }
        }
#endif
        if (ready && tud_vendor_n_write_available(0) >= reply->length) {
            if (tud_vendor_n_write(0, reply->data, reply->length) == reply->length) {
                reply_inflight = true;
                reply_remaining = reply->length;
                tud_vendor_n_write_flush(0);
                reply_head = (uint8_t)((reply_head + 1) % REPLY_CAPACITY);
                --reply_count;
            }
        }
    }
    if (protocol.initialized && now - last_input_ms >= 4 && tud_hid_n_ready(0)) {
        uint8_t input[PROBE_INPUT_SIZE];
        size_t length = 0;
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
        uint32_t native_serial = 0;
        if (protocol.report_id == 0x08 && protocol.controller_active && native_stream_ready) {
            native_serial = probe_controller_input_peek_native_report(now, input);
            if (!native_serial) return; // Never replay a packet's mouse or motion samples.
            length = sizeof(input);
            gate_native_report(input);
        }
#endif
        if (!length) {
            length = probe_protocol_report(&protocol, protocol.report_id, input, sizeof(input));
        }
        if (length && tud_hid_n_report(0, protocol.report_id, input, (uint16_t)length)) {
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
            if (native_serial) {
                if (!probe_controller_input_commit_native_report(native_serial))
                    probe_debug_printf("[PROBE] Native stream changed during HID submission\n");
                protocol.report_counter = input[0];
            }
#endif
            ++protocol.report_counter;
            ++input_reports;
            last_input_ms = now;
        }
    }
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
    if (instance == 0 && report_type == HID_REPORT_TYPE_INPUT) {
        probe_controller_input_set_native_stream(false);
        native_stream_ready = false;
        hid_completion_seen = false;
        probe_debug_printf("[PROBE] HID input transfer failed; pending native packets discarded\n");
    }
}
#endif

void tud_hid_report_complete_cb(uint8_t instance, const uint8_t* report, uint16_t length) {
#ifdef SWITCH2_PROBE_USB_INIT
    if (instance != 0 || length != PROBE_INPUT_SIZE + 1) return;
    uint8_t rails;
    if (report[0] == 0x08) rails = (report[4] >> 6) & 3;
    else if (report[0] == 0x05) rails = (report[5] >> 4) & 3;
    else return;
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
    last_hid_complete_ms = to_ms_since_boot(get_absolute_time());
    hid_completion_seen = true;
    if (report[0] == 0x08) {
        const int32_t dx = signed_mouse_delta(report + 10);
        const int32_t dy = signed_mouse_delta(report + 12);
#ifdef SWITCH2_PROBE_TRACE_NATIVE_INPUT
        if ((uint32_t)(last_hid_complete_ms - last_native_trace_ms) >= 1000) {
            last_native_trace_ms = last_hid_complete_ms;
            // Observe the completed transfer, not a proposed or ungated report.
            log_packet("NATIVE_INPUT_DELIVERED", instance, report[0], report, length);
        }
#endif
        if (dx || dy) {
            ++mouse_delivered_reports;
            mouse_delivered_x += dx;
            mouse_delivered_y += dy;
        }
    }
    const uint16_t buttons = report[0] == 0x08 ?
        (uint16_t)(report[3] | ((uint16_t)report[4] << 8)) :
        (uint16_t)(report[5] | ((uint16_t)report[6] << 8));
    if (buttons != last_delivered_buttons) {
        last_delivered_buttons = buttons;
        probe_debug_printf("[PROBE] CONTROLLER_REPORT delivered id=%02x buttons=%04x\n", report[0], buttons);
    }
#endif
    if (rails != last_delivered_rails) {
        last_delivered_rails = rails;
        probe_debug_printf("[PROBE] TEST_REPORT delivered id=%02x SL=%u SR=%u\n",
                           report[0], (rails >> 1) & 1, rails & 1);
    }
#else
    (void)instance;
    (void)report;
    (void)length;
#endif
}

void tud_vendor_rx_cb(uint8_t instance, const uint8_t* buffer, uint16_t length) {
    ++bulk_packets;
#ifdef SWITCH2_PROBE_USB_INIT
    if (instance == 0) consume_bulk_packet(buffer, length);
#else
    log_packet("BULK_OUT", instance, 0, buffer, length);
#endif
    // Drain TinyUSB's receive FIFO; raw packet data above is consumed once.
    uint8_t discarded[64];
    while (tud_vendor_n_available(instance)) {
        if (!tud_vendor_n_read(instance, discarded, sizeof(discarded))) break;
    }
}

void tud_vendor_tx_cb(uint8_t instance, uint32_t length) {
#ifdef SWITCH2_PROBE_USB_INIT
    if (instance == 0 && reply_inflight) {
        if (length <= reply_remaining) {
            reply_remaining -= (uint16_t)length;
            // TinyUSB calls this per packet. Exact packet multiples finish only
            // after its automatic ZLP, not after the last full-size packet.
            if (reply_remaining == 0 && length < CFG_TUD_VENDOR_EPSIZE)
                reply_inflight = false;
        } else {
            probe_debug_printf("[PROBE] Unexpected bulk completion; pending=%u\n", reply_remaining);
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
#ifdef SWITCH2_PROBE_IDENTITY_REPLY
    if (request->bmRequestType == 0xc0 && request->bRequest == 0x03 &&
        request->wValue == 0 && request->wIndex == 0) {
        if (stage == CONTROL_STAGE_SETUP) {
            ++identity_requests;
            log_packet("IDENTITY_REPLY", 0, 3, probe_identity_reply, sizeof(probe_identity_reply));
            return tud_control_xfer(rhport, request, (void*)probe_identity_reply,
                                    sizeof(probe_identity_reply));
        }
        return true;
    }
#endif
#ifdef SWITCH2_PROBE_VERSION_REPLY
    if (request->bmRequestType == 0xc0 && request->bRequest == 0x02 &&
        request->wValue == 0 && request->wIndex == 0) {
        if (stage == CONTROL_STAGE_SETUP) {
            ++version_requests;
            log_packet("VERSION_REPLY", 0, 2, probe_version_reply, sizeof(probe_version_reply));
            return tud_control_xfer(rhport, request, (void*)probe_version_reply,
                                    sizeof(probe_version_reply));
        }
        return true;
    }
#endif
#ifdef SWITCH2_PROBE_ACK_SETUP04
    // Exact control-transfer contract observed on the console and on two
    // genuine controllers. The meaning of 0x0276 is unresolved: do not infer
    // UART baud, USB speed, or any flash operation from it.
    if (request->bmRequestType == 0x40 && request->bRequest == 0x04 &&
        request->wValue == 0x0276 && request->wIndex == 0 && request->wLength == 0) {
        if (stage == CONTROL_STAGE_SETUP)
            return tud_control_status(rhport, request);
        if (stage == CONTROL_STAGE_ACK) {
            ++setup_completions;
            probe_debug_printf("[PROBE] SETUP04 acknowledged value=%04x (parameter semantics unresolved)\n",
                               request->wValue);
        }
        return true;
    }
#endif
    return false;
}

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
#ifndef SWITCH_PICO_SWITCH2_USB_BRIDGE
    (void)probe_button_update(&button_test, -1, false);
#endif
    protocol.test_rail_buttons = false;
    protocol.controller_active = false;
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
    probe_controller_input_set_native_stream(false);
    native_stream_ready = false;
    hid_completion_seen = false;
#endif
#endif
}
void tud_resume_cb(void) { probe_debug_printf("[PROBE] RESUME\n"); }

int main(void) {
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
    probe_controller_input_clock_init();
#endif
    stdio_init_all();
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
    probe_debug_printf("\n[PROBE] Joy-Con 2 (R) Bluetooth-to-USB controller/native mouse bridge\n");
#else
    probe_debug_printf("\n[PROBE] Joy-Con 2 (R) USB enumeration recorder\n");
#endif
#ifdef SWITCH2_PROBE_OMIT_NATIVE_IMU
    probe_debug_printf("[PROBE] ACTIVATION_TEST=no-imu: native08 bytes 15..55 omitted; features, power, status, counters and cadence unchanged\n");
#elif defined(SWITCH2_PROBE_ZERO_NATIVE_IMU_PAYLOAD)
    probe_debug_printf("[PROBE] ACTIVATION_TEST=zero-imu-payload: native08 bytes 16..55 zeroed; length, features and all other fields unchanged\n");
#endif
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
    probe_controller_input_init();
#if SWITCH2_BRIDGE_WII_INPUT
    probe_debug_printf("[PROBE] UART0 GP0=TX, 115200 8N1; selected Wii IR/MotionPlus source enabled\n");
#else
    probe_debug_printf("[PROBE] UART0 GP0=TX, 115200 8N1; selected right Joy-Con Bluetooth source enabled\n");
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
    probe_debug_printf("[PROBE] Own virtual pairing storage offset=%08" PRIx32 "\n",
                       probe_storage_offset());
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
#if SWITCH2_BRIDGE_WII_INPUT
    probe_debug_printf("[PROBE] Wii IR drives native mouse movement; buttons retain profile mapping; keep Wii still for MotionPlus calibration\n");
    probe_debug_printf("[PROBE] Hold BOOTSEL2s for pairing; Wii cue feedback uses bounded ERM patterns, not HD audio waveforms\n");
#else
    probe_debug_printf("[PROBE] Live right Joy-Con buttons/stick/native mouse; hold BOOTSEL 2s for Bluetooth pairing (never clears pairings)\n");
#endif
#else
    probe_debug_printf("[PROBE] Manual input test: hold BOOTSEL for SL+SR, release for neutral; no controller forwarding\n");
#endif
#else
    probe_debug_printf("[PROBE] Bulk commands are capture-only; no input reports\n");
#endif
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
    if (!probe_controller_input_start())
        panic("Bluetooth source could not establish multicore flash coordination");
#endif
    tud_init(0);
    uint32_t last_heartbeat = 0;
    while (true) {
        tud_task();
        drain_log();
        const uint32_t now = to_ms_since_boot(get_absolute_time());
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
        probe_bootsel_task(now);
#endif
#ifdef SWITCH2_PROBE_USB_INIT
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
        if (probe_controller_input_pairing_task())
            probe_debug_printf("[PROBE] Bluetooth pairing window requested; put the right Joy-Con in SYNC pairing mode\n");
        controller_input_task(now);
#else
        button_test_task(now);
#endif
        protocol_task(now);
#endif
        if (now - last_heartbeat >= 1000) {
            last_heartbeat = now;
            probe_debug_printf("[PROBE %" PRIu32 "] alive mounted=%u bulk=%" PRIu32
                               " hid=%" PRIu32 " identity=%" PRIu32
                               " version=%" PRIu32 " setup=%" PRIu32
                               " inputs=%" PRIu32 " command_drops=%" PRIu32
                               " log_dropped_bytes=%" PRIu32 "\n",
                               now, tud_mounted(), bulk_packets, hid_packets,
                               identity_requests, version_requests, setup_completions,
                               input_reports, command_drops, log_dropped);
#ifdef SWITCH_PICO_SWITCH2_USB_BRIDGE
            if (mouse_delivered_reports != mouse_logged_reports) {
                mouse_logged_reports = mouse_delivered_reports;
                probe_debug_printf("[PROBE] MOUSE_REPORT delivered packets=%" PRIu32
                                   " total_x=%" PRId64 " total_y=%" PRId64 "\n",
                                   mouse_delivered_reports, mouse_delivered_x, mouse_delivered_y);
            }
#endif
        }
        sleep_us(100);
    }
}
