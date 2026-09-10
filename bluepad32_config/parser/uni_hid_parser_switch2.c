// SPDX-License-Identifier: Apache-2.0
// Proprietary, unencrypted Switch 2 BLE transport. No SMP or bond-store changes.
// Protocol descriptions: https://github.com/ndeadly/switch2_controller_research
// Pairing/calibration/rumble payloads: https://github.com/Nadeflore/switch2-controllers
// Hold-frame cadence: https://github.com/TommyWabg/Switch2Connect
// Sensor conversion independently adapted from SDL_hidapi_switch2.c (SDL/Valve).

#include "parser/uni_hid_parser_switch2.h"

#include <math.h>
#include <stdatomic.h>
#include <string.h>
#if SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
#include <stdio.h>
#endif

#include <btstack.h>
#include "bt/uni_bt_defines.h"
#include "bt/uni_bt_le.h"
#include "parser/uni_hid_parser_imu.h"
#include "parser/uni_switch2_pairing.h"
#include "sdkconfig.h"
#include "uni_hid_device.h"
#include "uni_log.h"

#define SW2_TIMEOUT_MS 2000
#define SW2_OUTPUT_INTERVAL_MS 13
#define SW2_NEUTRAL_WRITE_BUDGET 3
#define SW2_HAPTICS_CAPACITY 16
#define SW2_REPORT_SIZE 63
#define SW2_ACK 0x78
#define SW2_CCCD_UUID 0x2902

// BTstack deserializes UUIDs into canonical (not ATT wire) byte order.
static const uint8_t sw2_service_uuid[16] = {
    0xab, 0x7d, 0xe9, 0xbe, 0x89, 0xfe, 0x49, 0xad, 0x82, 0x8f, 0x11, 0x8f, 0x09, 0xdf, 0x7f, 0xd0};
static const uint8_t sw2_input_uuid[16] = {
    0xab, 0x7d, 0xe9, 0xbe, 0x89, 0xfe, 0x49, 0xad, 0x82, 0x8f, 0x11, 0x8f, 0x09, 0xdf, 0x7f, 0xd2};
#if SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
static const uint8_t sw2_secondary_left_uuid[16] = {
    0xcc, 0x1b, 0xbb, 0xb5, 0x73, 0x54, 0x4d, 0x32, 0xa7, 0x16, 0xa8, 0x1c, 0xb2, 0x41, 0xa3, 0x2a};
static const uint8_t sw2_secondary_right_uuid[16] = {
    0xd5, 0xa9, 0xe0, 0x1e, 0x2f, 0xfc, 0x4c, 0xca, 0xb2, 0x0c, 0x8b, 0x67, 0x14, 0x2b, 0xf4, 0x42};
#endif
static const uint8_t sw2_command_uuid[16] = {
    0x64, 0x9d, 0x4a, 0xc9, 0x8e, 0xb7, 0x4e, 0x6c, 0xaf, 0x44, 0x1e, 0xa5, 0x4f, 0xe5, 0xf0, 0x05};
static const uint8_t sw2_response_uuid[16] = {
    0xc7, 0x65, 0xa9, 0x61, 0xd9, 0xd8, 0x4d, 0x36, 0xa2, 0x0a, 0x53, 0x15, 0xb1, 0x11, 0x83, 0x6a};
static const uint8_t sw2_rumble_pro_uuid[16] = {
    0xcc, 0x48, 0x3f, 0x51, 0x92, 0x58, 0x42, 0x7d, 0xa9, 0x39, 0x63, 0x0c, 0x31, 0xf7, 0x2b, 0x05};
static const uint8_t sw2_rumble_left_uuid[16] = {
    0x28, 0x93, 0x26, 0xcb, 0xa4, 0x71, 0x48, 0x5d, 0xa8, 0xf4, 0x24, 0x0c, 0x14, 0xf1, 0x82, 0x41};
static const uint8_t sw2_rumble_right_uuid[16] = {
    0xfa, 0x19, 0xb0, 0xfb, 0xcd, 0x1f, 0x46, 0xa7, 0x84, 0xa1, 0xbb, 0xb0, 0x9e, 0x00, 0xc1, 0x49};

// Public application-protocol payloads from Nadeflore Controller.pair(), NOT
// secret keys or an authenticated BLE pairing mechanism. No security claim.
static const uint8_t sw2_pair_exchange[17] = {
    0x00, 0xea, 0xbd, 0x47, 0x13, 0x89, 0x35, 0x42, 0xc6, 0x79, 0xee, 0x07, 0xf2, 0x53, 0x2c, 0x6c, 0x31};
static const uint8_t sw2_pair_confirm[17] = {
    0x00, 0x40, 0xb0, 0x8a, 0x5f, 0xcd, 0x1f, 0x9b, 0x41, 0x12, 0x5c, 0xac, 0xc6, 0x3f, 0x38, 0xa0, 0x73};

// Every state transition requires its own completed ATT operation and/or a
// matching application ACK. A timeout disconnects, never skips a setup step.
typedef enum {
    SW2_OFF, SW2_ADMITTED, SW2_SERVICE, SW2_CHARACTERISTICS,
    SW2_RESPONSE_DESCRIPTOR, SW2_INPUT_DESCRIPTOR, SW2_SUBSCRIBE_RESPONSE,
    SW2_INFO, SW2_PAIR, SW2_CALIBRATION, SW2_GYRO_CALIBRATION,
    SW2_SUBSCRIBE_INPUT, SW2_FEATURES, SW2_READY,
#if SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
    SW2_SECONDARY_DESCRIPTOR, SW2_SUBSCRIBE_SECONDARY,
#endif
} sw2_state_t;
typedef enum { SW2_QUERY_NONE, SW2_QUERY_DISCOVERY, SW2_QUERY_CCCD, SW2_QUERY_COMMAND, SW2_QUERY_RUMBLE } sw2_query_t;
typedef struct {
    uint16_t center[2];
    uint16_t positive[2];
    uint16_t negative[2];
} sw2_stick_t;
typedef struct {
    uni_switch2_haptics_frame_t frame;
    uint32_t expires, serial;
    bool held, native;
} sw2_host_command_t;
typedef struct {
    uint8_t sample[5];
    uint32_t expires;
    bool valid, held;
} sw2_host_side_t;
typedef struct {
    uni_hid_device_t* device;
    bd_addr_t address;
    hci_con_handle_t handle;
    bool allocated;
    sw2_state_t state;
    sw2_query_t query;
    uint8_t address_type;
    bool needs_pair;
    gatt_client_service_t service;
    gatt_client_characteristic_t input, response, command, rumble;
    uint16_t response_cccd, input_cccd;
    gatt_client_notification_t response_listener, input_listener;
    bool response_listening, input_listening;
#if SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
    gatt_client_characteristic_t secondary;
    uint16_t secondary_cccd;
    gatt_client_notification_t secondary_listener;
    bool secondary_listening;
#endif
    btstack_timer_source_t timeout_timer, output_timer;
    bool timeout_active, output_active;
    // ATT write-request buffers must outlive the asynchronous call.
    uint8_t command_data[32], rumble_data[33], cccd_data[2];
    uint8_t command_length;
    bool command_pending, command_sent, command_acked;
#if SWITCH_PICO_SWITCH2_USB_BRIDGE
    uint64_t sample_token;
#endif
    uint8_t step, calibration_slot;
    bool factory_calibration, calibration_done;
    uint32_t memory_address;
    uint8_t memory_length;
    sw2_stick_t sticks[2];
    int32_t gyro_bias[3];
    uint8_t extra_buttons, leds;
    bool leds_pending;
    uint8_t rumble_id, weak, strong;
    bool rumble_scheduled, rumble_held;
    uint32_t rumble_start, rumble_end;
    sw2_host_command_t host_queue[SW2_HAPTICS_CAPACITY];
    sw2_host_side_t host_sides[2];
    uint8_t host_head, host_count;
    uint32_t host_serial, haptics_epoch, output_revision;
    uint32_t pending_serial, pending_epoch, pending_revision;
    uint32_t playback_until;
    uint8_t pending_guard_ms;
    uint8_t neutral_writes;
    bool pending_neutral;
    bool pending_host, pending_barrier, barrier_pending, output_urgent;
    bool feedback_active, playback_guard;
    uint32_t sensor_start, sensor_host_start, sensor_last;
    uint8_t sensor_warmup;
    int32_t gyro_full_scale;
} sw2_instance_t;

// Separate bounded storage: never squeeze transport resources into parser_data
// (256 bytes). Retired buffers are not reused until their old BLE link is gone.
static sw2_instance_t sw2_instances[CONFIG_BLUEPAD32_MAX_DEVICES];
static atomic_uint_least32_t sw2_haptics_drops;

static void sw2_gatt_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size);
static void sw2_output_tick(btstack_timer_source_t* timer);
static void sw2_continue(sw2_instance_t* ins);
static void sw2_complete_command(sw2_instance_t* ins);
static void sw2_rumble_complete(sw2_instance_t* ins);
static void sw2_discard_host(sw2_instance_t* ins);

static bool sw2_product(uint16_t pid) {
    return pid == UNI_SW2_PRO_PID || pid == UNI_SW2_JOYCON_L_PID || pid == UNI_SW2_JOYCON_R_PID;
}

#if SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
static bool sw2_mouse_capture_enabled(const sw2_instance_t* ins) {
    return ins->device->product_id == UNI_SW2_JOYCON_L_PID || ins->device->product_id == UNI_SW2_JOYCON_R_PID;
}

static void sw2_capture(sw2_instance_t* ins, uint8_t report_id, const uint8_t* report, uint16_t length) {
    if (sw2_mouse_capture_enabled(ins))
        switch_pico_switch2_mouse_report(ins->device->product_id, ins->address, report_id, report,
                                        length > 64 ? 64 : length, btstack_run_loop_get_time_ms());
}

static void sw2_log_capture(const char* label, const sw2_instance_t* ins,
                            const uint8_t* data, unsigned length) {
    if (length > 64 || !sw2_mouse_capture_enabled(ins)) return;
    static const char hex[] = "0123456789abcdef";
    char text[129];
    for (unsigned i = 0; i < length; ++i) {
        text[2 * i] = hex[data[i] >> 4];
        text[2 * i + 1] = hex[data[i] & 15];
    }
    text[length * 2] = 0;
    printf("[SW2_%s] pid=%04x address=%02x:%02x:%02x:%02x:%02x:%02x data=%s\n",
           label, ins->device->product_id, ins->address[0], ins->address[1],
           ins->address[2], ins->address[3], ins->address[4], ins->address[5], text);
}
#endif

bool uni_hid_parser_switch2_is_ble_device(const uni_hid_device_t* d) {
    return d && d->conn.protocol == UNI_BT_CONN_PROTOCOL_BLE && d->vendor_id == UNI_SW2_NINTENDO_VID &&
           sw2_product(d->product_id);
}

static sw2_instance_t* sw2_instance(const uni_hid_device_t* d) {
    if (!uni_hid_parser_switch2_is_ble_device(d))
        return NULL;
    for (unsigned i = 0; i < CONFIG_BLUEPAD32_MAX_DEVICES; ++i) {
        sw2_instance_t* ins = &sw2_instances[i];
        if (ins->device == d && ins->state != SW2_OFF && memcmp(ins->address, d->conn.btaddr, 6) == 0)
            return ins;
    }
    return NULL;
}

static bool sw2_live(sw2_instance_t* ins) {
    return ins->device && sw2_instance(ins->device) == ins && ins->handle == ins->device->conn.handle;
}

static void sw2_disarm_timeout(sw2_instance_t* ins) {
    if (ins->timeout_active)
        btstack_run_loop_remove_timer(&ins->timeout_timer);
    ins->timeout_active = false;
}

void uni_hid_parser_switch2_teardown(uni_hid_device_t* d) {
    sw2_instance_t* ins = sw2_instance(d);
    if (!ins)
        return;
#if SWITCH_PICO_SWITCH2_USB_BRIDGE
    if (ins->sample_token) {
        switch_pico_switch2_sample_result(d->product_id, ins->address, ins->sample_token,
                                          -1, btstack_run_loop_get_time_ms());
        ins->sample_token = 0;
    }
#endif
    sw2_disarm_timeout(ins);
    if (ins->output_active)
        btstack_run_loop_remove_timer(&ins->output_timer);
    ins->output_active = false;
    if (ins->response_listening)
        gatt_client_stop_listening_for_characteristic_value_updates(&ins->response_listener);
    if (ins->input_listening)
        gatt_client_stop_listening_for_characteristic_value_updates(&ins->input_listener);
    ins->response_listening = ins->input_listening = false;
#if SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
    if (ins->secondary_listening)
        gatt_client_stop_listening_for_characteristic_value_updates(&ins->secondary_listener);
    ins->secondary_listening = false;
    sw2_capture(ins, 0, NULL, 0);
#endif
    sw2_discard_host(ins);
    ++ins->haptics_epoch;
    ins->command_pending = ins->rumble_scheduled = false;
    ins->extra_buttons = 0;
    ins->state = SW2_OFF;
    ins->device = NULL;
}

static void sw2_fail(sw2_instance_t* ins, const char* reason, unsigned status) {
    if (!ins->device)
        return;
    uni_hid_device_t* d = ins->device;
    loge("Switch2: %s (state=%u, status=%u)\n", reason, (unsigned)ins->state, status);
    uni_hid_parser_switch2_teardown(d);
    uni_hid_device_disconnect(d);
    uni_hid_device_delete(d);
}

static void sw2_timeout(btstack_timer_source_t* timer) {
    sw2_instance_t* ins = btstack_run_loop_get_timer_context(timer);
    ins->timeout_active = false;
    if (sw2_live(ins))
        sw2_fail(ins, "transaction timeout", 0);
}

static void sw2_arm_timeout(sw2_instance_t* ins) {
    sw2_disarm_timeout(ins);
    btstack_run_loop_set_timer_handler(&ins->timeout_timer, sw2_timeout);
    btstack_run_loop_set_timer_context(&ins->timeout_timer, ins);
    btstack_run_loop_set_timer(&ins->timeout_timer, SW2_TIMEOUT_MS);
    ins->timeout_active = true;
    btstack_run_loop_add_timer(&ins->timeout_timer);
}

static void sw2_schedule_output(sw2_instance_t* ins, uint32_t ms) {
    if (ins->output_active)
        btstack_run_loop_remove_timer(&ins->output_timer);
    btstack_run_loop_set_timer_handler(&ins->output_timer, sw2_output_tick);
    btstack_run_loop_set_timer_context(&ins->output_timer, ins);
    btstack_run_loop_set_timer(&ins->output_timer, ms ? ms : 1);
    ins->output_active = true;
    btstack_run_loop_add_timer(&ins->output_timer);
}

static bool sw2_check_query(sw2_instance_t* ins, uint8_t status) {
    if (status != ERROR_CODE_SUCCESS) {
        sw2_fail(ins, "GATT request rejected", status);
        return false;
    }
    return true;
}

static void sw2_discover_descriptors(sw2_instance_t* ins, bool input) {
    ins->state = input ? SW2_INPUT_DESCRIPTOR : SW2_RESPONSE_DESCRIPTOR;
    ins->query = SW2_QUERY_DISCOVERY;
    sw2_arm_timeout(ins);
    sw2_check_query(ins, gatt_client_discover_characteristic_descriptors(
                             sw2_gatt_handler, ins->handle, input ? &ins->input : &ins->response));
}

static void sw2_subscribe(sw2_instance_t* ins, bool input) {
    ins->state = input ? SW2_SUBSCRIBE_INPUT : SW2_SUBSCRIBE_RESPONSE;
    ins->query = SW2_QUERY_CCCD;
    ins->cccd_data[0] = 1;
    ins->cccd_data[1] = 0;
    if (input) {
        gatt_client_listen_for_characteristic_value_updates(&ins->input_listener, sw2_gatt_handler, ins->handle, &ins->input);
        ins->input_listening = true;
    } else {
        gatt_client_listen_for_characteristic_value_updates(&ins->response_listener, sw2_gatt_handler, ins->handle, &ins->response);
        ins->response_listening = true;
    }
    sw2_arm_timeout(ins);
    sw2_check_query(ins, gatt_client_write_characteristic_descriptor_using_descriptor_handle(
                             sw2_gatt_handler, ins->handle, input ? ins->input_cccd : ins->response_cccd, 2, ins->cccd_data));
}

#if SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
static void sw2_discover_secondary_descriptors(sw2_instance_t* ins) {
    ins->state = SW2_SECONDARY_DESCRIPTOR;
    ins->query = SW2_QUERY_DISCOVERY;
    sw2_arm_timeout(ins);
    sw2_check_query(ins, gatt_client_discover_characteristic_descriptors(
                             sw2_gatt_handler, ins->handle, &ins->secondary));
}

static void sw2_subscribe_secondary(sw2_instance_t* ins) {
    ins->state = SW2_SUBSCRIBE_SECONDARY;
    ins->query = SW2_QUERY_CCCD;
    ins->cccd_data[0] = 1;
    ins->cccd_data[1] = 0;
    gatt_client_listen_for_characteristic_value_updates(&ins->secondary_listener, sw2_gatt_handler,
                                                       ins->handle, &ins->secondary);
    ins->secondary_listening = true;
    sw2_arm_timeout(ins);
    sw2_check_query(ins, gatt_client_write_characteristic_descriptor_using_descriptor_handle(
                             sw2_gatt_handler, ins->handle, ins->secondary_cccd, 2, ins->cccd_data));
}
#endif

static bool sw2_transient_write_error(uint8_t status) {
    return status == BTSTACK_ACL_BUFFERS_FULL || status == GATT_CLIENT_BUSY ||
           status == GATT_CLIENT_IN_WRONG_STATE;
}

static void sw2_try_command(sw2_instance_t* ins) {
    if (!ins->command_pending || ins->command_sent || ins->query != SW2_QUERY_NONE)
        return;
#if SWITCH_PICO_SWITCH2_USB_BRIDGE
    if (ins->sample_token &&
        !switch_pico_switch2_sample_result(ins->device->product_id, ins->address,
                                           ins->sample_token, 0, btstack_run_loop_get_time_ms())) {
        // Nothing was sent yet. Once sent, keep the old transaction serialized
        // until its ACK/timeout: the empty response has no request nonce.
        ins->sample_token = 0;
        ins->command_pending = false;
        sw2_disarm_timeout(ins);
        return;
    }
#endif
    // Re-check immediately before every application-pairing write, including a
    // deferred write after ACL backpressure. A closed window cannot write MACs.
    if (ins->command_data[0] == 0x15 && !switch_pico_switch2_pairing_allowed()) {
        sw2_fail(ins, "pairing window closed", 0);
        return;
    }
    bool no_response = (ins->command.properties & ATT_PROPERTY_WRITE_WITHOUT_RESPONSE) != 0;
    uint8_t status;
    if (no_response) {
        status = gatt_client_write_value_of_characteristic_without_response(
            ins->handle, ins->command.value_handle, ins->command_length, ins->command_data);
    } else {
        ins->query = SW2_QUERY_COMMAND;
        status = gatt_client_write_value_of_characteristic(sw2_gatt_handler, ins->handle,
                                                          ins->command.value_handle, ins->command_length, ins->command_data);
    }
    if (status != ERROR_CODE_SUCCESS) {
        ins->query = SW2_QUERY_NONE;
        if (sw2_transient_write_error(status)) {
            sw2_schedule_output(ins, SW2_OUTPUT_INTERVAL_MS);
            return;
        }
        sw2_fail(ins, "command write failed", status);
        return;
    }
    ins->command_sent = true;
}

static void sw2_send_command(sw2_instance_t* ins, uint8_t command, uint8_t subcommand,
                             const uint8_t* data, uint8_t length) {
    if (ins->command_pending || length > sizeof(ins->command_data) - 8) {
        sw2_fail(ins, "overlapping or oversized command", length);
        return;
    }
    uint8_t* out = ins->command_data;
    memset(out, 0, 8);
    out[0] = command;
    out[1] = 0x91;
    out[2] = 1;
    out[3] = subcommand;
    out[5] = length;
    if (length)
        memcpy(out + 8, data, length);
    ins->command_length = 8 + length;
    ins->command_pending = true;
    ins->command_sent = ins->command_acked = false;
    sw2_arm_timeout(ins);
    sw2_try_command(ins);
}

static void sw2_read_memory(sw2_instance_t* ins, uint32_t address, uint8_t length) {
    uint8_t data[8] = {length, 0x7e, 0, 0};
    little_endian_store_32(data, 4, address);
    ins->memory_address = address;
    ins->memory_length = length;
    sw2_send_command(ins, 0x02, 0x04, data, sizeof(data));
}

static void sw2_unpack_stick(const uint8_t* data, uint16_t out[2]) {
    out[0] = data[0] | ((uint16_t)(data[1] & 15) << 8);
    out[1] = (data[1] >> 4) | ((uint16_t)data[2] << 4);
}

static bool sw2_calibration(sw2_stick_t* out, const uint8_t* data) {
    sw2_stick_t value;
    sw2_unpack_stick(data, value.center);
    sw2_unpack_stick(data + 3, value.positive);
    sw2_unpack_stick(data + 6, value.negative);
    for (unsigned i = 0; i < 2; ++i) {
        if (!value.center[i] || value.center[i] == 4095 || !value.positive[i] || !value.negative[i] ||
            value.positive[i] > 4095 - value.center[i] || value.negative[i] > value.center[i])
            return false;
    }
    *out = value;
    return true;
}

static void sw2_continue(sw2_instance_t* ins) {
    switch (ins->state) {
        case SW2_INFO:
#if SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
            if (sw2_mouse_capture_enabled(ins) && ins->step == 1) {
                sw2_send_command(ins, 0x10, 0x01, NULL, 0);
                break;
            }
#endif
            sw2_read_memory(ins, 0x13000, 0x40);
            break;
        case SW2_PAIR: {
            uint8_t data[14] = {0, 2};
            if (ins->step == 0) {
                bd_addr_t local;
                gap_local_bd_addr(local);
                for (unsigned i = 0; i < 6; ++i)
                    data[2 + i] = data[8 + i] = local[5 - i];
                sw2_send_command(ins, 0x15, 0x01, data, sizeof(data));
            } else if (ins->step == 1) {
                sw2_send_command(ins, 0x15, 0x04, sw2_pair_exchange, sizeof(sw2_pair_exchange));
            } else if (ins->step == 2) {
                sw2_send_command(ins, 0x15, 0x02, sw2_pair_confirm, sizeof(sw2_pair_confirm));
            } else {
                sw2_send_command(ins, 0x15, 0x03, data, 1);
            }
            break;
        }
        case SW2_CALIBRATION: {
            uint32_t address = ins->factory_calibration ? 0x130a8 : 0x1fc042;
            if (ins->calibration_slot)
                address += ins->factory_calibration ? 0x40 : 0x20;
            sw2_read_memory(ins, address, 0x0b);
            break;
        }
        case SW2_GYRO_CALIBRATION:
            sw2_read_memory(ins, 0x13044, 12);
            break;
        case SW2_FEATURES: {
#if SWITCH_PICO_SWITCH2_USB_BRIDGE
            // Match the console's complete native feature set, including the
            // status associated with bit 5, rather than a mouse-only capture.
            const uint8_t features[4] = {sw2_mouse_capture_enabled(ins) ? 0x37 : 0x04, 0, 0, 0};
#elif SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
            const uint8_t features[4] = {sw2_mouse_capture_enabled(ins) ? 0x14 : 0x04, 0, 0, 0};
#else
            const uint8_t features[4] = {0x04, 0, 0, 0};
#endif
            sw2_send_command(ins, 0x0c, ins->step ? 0x04 : 0x02, features, sizeof(features));
            break;
        }
        case SW2_READY:
            if (ins->leds_pending && !ins->command_pending) {
                const uint8_t data[8] = {ins->leds, 0, 0, 0, 0, 0, 0, 0};
                ins->leds_pending = false;
                sw2_send_command(ins, 0x09, 0x07, data, sizeof(data));
            }
            break;
        default:
            break;
    }
}

static void sw2_complete_command(sw2_instance_t* ins) {
    if (!ins->command_pending || !ins->command_acked || ins->query == SW2_QUERY_COMMAND)
        return;
    ins->command_pending = false;
    sw2_disarm_timeout(ins);
#if SWITCH_PICO_SWITCH2_USB_BRIDGE
    if (ins->sample_token) {
        switch_pico_switch2_sample_result(ins->device->product_id, ins->address,
                                           ins->sample_token, 1, btstack_run_loop_get_time_ms());
        ins->sample_token = 0;
    }
#endif
    switch (ins->state) {
        case SW2_INFO:
#if SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
            if (sw2_mouse_capture_enabled(ins) && ins->step == 0) {
                ins->step = 1;
                break;
            }
            ins->step = 0;
#endif
            ins->state = ins->needs_pair ? SW2_PAIR : SW2_CALIBRATION;
            break;
        case SW2_PAIR:
            if (++ins->step == 4) {
                if (!uni_switch2_pairing_remember(ins->address_type, ins->address)) {
                    sw2_fail(ins, "could not persist application pairing", 0);
                    return;
                }
                ins->needs_pair = false;
                ins->step = 0;
                ins->state = SW2_CALIBRATION;
            }
            break;
        case SW2_CALIBRATION:
            if (ins->calibration_done)
                ins->state = SW2_GYRO_CALIBRATION;
            break;
        case SW2_GYRO_CALIBRATION:
            sw2_subscribe(ins, true);
            return;
        case SW2_FEATURES:
            if (++ins->step == 2) {
                ins->state = SW2_READY;
                ins->device->controller.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
                if (!uni_hid_device_set_ready_complete(ins->device))
                    return; // Platform may have destroyed the device.
                sw2_schedule_output(ins, 1);
            }
            break;
        default:
            break;
    }
    sw2_continue(ins);
}

static void sw2_response(sw2_instance_t* ins, const uint8_t* data, uint16_t length) {
    if (!ins->command_pending || !ins->command_sent || ins->command_acked || length < 8 ||
        data[0] != ins->command_data[0] || data[3] != ins->command_data[3] || data[1] != 1 || data[2] != 1)
        return; // Unsolicited, truncated or stale ACK is not progress.
    if (data[5] != SW2_ACK) {
        sw2_fail(ins, "negative application ACK", data[5]);
        return;
    }
#if SWITCH_PICO_SWITCH2_USB_BRIDGE
    // 0A/02 has an empty application ACK, not a returned sample ID or status
    // payload. Never interpret a truncated/extended response as its success.
    if (ins->sample_token && (length != 8 || data[4] != 0x10 || data[6] || data[7]))
        return;
#endif
#if SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
    if (ins->state == SW2_INFO && ins->step == 1) {
        const uint8_t expected_type = ins->device->product_id == UNI_SW2_JOYCON_R_PID ? 1 : 0;
        if (data[0] != 0x10 || length < 20 || data[11] != expected_type) return;
        sw2_log_capture("VERSION", ins, data + 8, 12);
    }
#endif
    if (data[0] == 0x02) {
        if (length < 16 || data[8] != ins->memory_length ||
            little_endian_read_32(data, 12) != ins->memory_address || length < 16 + ins->memory_length)
            return; // Includes a stale memory response with the same cmd/subcmd.
        const uint8_t* value = data + 16;
        if (ins->state == SW2_INFO) {
            if (little_endian_read_16(value, 18) != UNI_SW2_NINTENDO_VID ||
                little_endian_read_16(value, 20) != ins->device->product_id) {
                sw2_fail(ins, "controller identity mismatch", 0);
                return;
            }
#if SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
            // This read is already part of normal setup. Capture its validated
            // factory identity for USB enumeration research, without new reads
            // or writes to the controller. Keep donor data out of source files.
            sw2_log_capture("IDENTITY", ins, value, 64);
#endif
        } else if (ins->state == SW2_CALIBRATION) {
            // Both solo Joy-Cons store their one stick in calibration slot 1.
            unsigned stick = ins->device->product_id == UNI_SW2_JOYCON_R_PID ? 1 : ins->calibration_slot;
            bool valid = sw2_calibration(&ins->sticks[stick], value);
            if (!valid && !ins->factory_calibration) {
                ins->factory_calibration = true;
            } else {
                if (!valid)
                    logi("Switch2: invalid factory stick calibration, using full-range mapping\n");
                ins->factory_calibration = false;
                if (ins->device->product_id == UNI_SW2_PRO_PID && ins->calibration_slot == 0)
                    ins->calibration_slot = 1;
                else
                    ins->calibration_done = true;
            }
        } else if (ins->state == SW2_GYRO_CALIBRATION) {
            for (unsigned i = 0; i < 3; ++i) {
                uint32_t bits = little_endian_read_32(value, 4 * i);
                float bias;
                memcpy(&bias, &bits, sizeof(bias));
                // Erased/invalid flash is not a floating-point sensor value.
                if (isfinite(bias) && bias >= -40.0f && bias <= 40.0f)
                    ins->gyro_bias[i] = (int32_t)(bias * (57.295779513f * UNI_IMU_GYRO_RES_PER_DEG_S));
            }
        }
    } else if (ins->state == SW2_PAIR) {
        unsigned payload_length = ins->step == 0 ? 9 : ins->step == 3 ? 1 : 17;
        if (length < 8 + payload_length)
            return;
        if (data[8] != 1) {
            sw2_fail(ins, "pairing command rejected", data[8]);
            return;
        }
    }
#if SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
    if (ins->state == SW2_FEATURES)
        sw2_capture(ins, 0xc0, data, length);
#endif
    ins->command_acked = true;
    sw2_complete_command(ins);
}

static void sw2_query_complete(sw2_instance_t* ins, uint8_t status) {
    sw2_query_t query = ins->query;
    if (query == SW2_QUERY_NONE)
        return;
    ins->query = SW2_QUERY_NONE;
    if (status != ATT_ERROR_SUCCESS) {
        sw2_fail(ins, "GATT transaction failed", status);
        return;
    }
    if (query == SW2_QUERY_COMMAND) {
        sw2_complete_command(ins);
        return;
    }
    if (query == SW2_QUERY_RUMBLE) {
        sw2_rumble_complete(ins);
        // A command queued behind this write has its own timeout already.
        if (!ins->command_pending)
            sw2_disarm_timeout(ins);
        sw2_try_command(ins);
        if (sw2_live(ins))
            sw2_schedule_output(ins, 1);
        return;
    }
    sw2_disarm_timeout(ins);
    switch (ins->state) {
        case SW2_SERVICE:
            if (!ins->service.start_group_handle) {
                sw2_fail(ins, "missing proprietary service", 0);
                return;
            }
            ins->state = SW2_CHARACTERISTICS;
            ins->query = SW2_QUERY_DISCOVERY;
            sw2_arm_timeout(ins);
            sw2_check_query(ins, gatt_client_discover_characteristics_for_service(sw2_gatt_handler, ins->handle, &ins->service));
            break;
        case SW2_CHARACTERISTICS:
            if (!ins->input.value_handle || !ins->response.value_handle || !ins->command.value_handle || !ins->rumble.value_handle) {
                sw2_fail(ins, "missing required characteristic", 0);
                return;
            }
#if SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
            if (sw2_mouse_capture_enabled(ins) && !ins->secondary.value_handle) {
                sw2_fail(ins, "missing secondary input characteristic", 0);
                return;
            }
#endif
            sw2_discover_descriptors(ins, false);
            break;
        case SW2_RESPONSE_DESCRIPTOR:
            if (!ins->response_cccd) {
                sw2_fail(ins, "missing response CCCD", 0);
                return;
            }
            sw2_discover_descriptors(ins, true);
            break;
        case SW2_INPUT_DESCRIPTOR:
            if (!ins->input_cccd) {
                sw2_fail(ins, "missing input CCCD", 0);
                return;
            }
#if SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
            if (sw2_mouse_capture_enabled(ins)) {
                sw2_discover_secondary_descriptors(ins);
                return;
            }
#endif
            sw2_subscribe(ins, false);
            break;
#if SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
        case SW2_SECONDARY_DESCRIPTOR:
            if (!ins->secondary_cccd) {
                sw2_fail(ins, "missing secondary input CCCD", 0);
                return;
            }
            sw2_subscribe(ins, false);
            break;
#endif
        case SW2_SUBSCRIBE_RESPONSE:
            ins->state = SW2_INFO;
            sw2_continue(ins);
            break;
        case SW2_SUBSCRIBE_INPUT:
#if SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
        case SW2_SUBSCRIBE_SECONDARY:
            // This Joy-Con emits the last subscribed input format, not both.
            // Native capture is explicit: its packed input is not yet decoded
            // by the normal gamepad parser. Common capture preserves controls.
            if (SWITCH_PICO_SWITCH2_MOUSE_CAPTURE_NATIVE &&
                ins->state == SW2_SUBSCRIBE_INPUT && sw2_mouse_capture_enabled(ins)) {
                sw2_subscribe_secondary(ins);
                return;
            }
#endif
            ins->state = SW2_FEATURES;
            ins->step = 0;
            sw2_continue(ins);
            break;
        default:
            sw2_fail(ins, "unexpected query completion", query);
            break;
    }
}

static void sw2_characteristic(sw2_instance_t* ins, const gatt_client_characteristic_t* characteristic) {
    if (ins->state != SW2_CHARACTERISTICS)
        return;
    const uint8_t* rumble_uuid = ins->device->product_id == UNI_SW2_PRO_PID ? sw2_rumble_pro_uuid :
                                ins->device->product_id == UNI_SW2_JOYCON_L_PID ? sw2_rumble_left_uuid : sw2_rumble_right_uuid;
    gatt_client_characteristic_t* target = NULL;
    uint16_t properties = ATT_PROPERTY_NOTIFY;
    if (memcmp(characteristic->uuid128, sw2_input_uuid, 16) == 0)
        target = &ins->input;
    else if (memcmp(characteristic->uuid128, sw2_response_uuid, 16) == 0)
        target = &ins->response;
    else if (memcmp(characteristic->uuid128, sw2_command_uuid, 16) == 0) {
        target = &ins->command;
        properties = ATT_PROPERTY_WRITE | ATT_PROPERTY_WRITE_WITHOUT_RESPONSE;
    } else if (memcmp(characteristic->uuid128, rumble_uuid, 16) == 0) {
        target = &ins->rumble;
        properties = ATT_PROPERTY_WRITE | ATT_PROPERTY_WRITE_WITHOUT_RESPONSE;
    }
#if SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
    if (!target && sw2_mouse_capture_enabled(ins) &&
             memcmp(characteristic->uuid128, ins->device->product_id == UNI_SW2_JOYCON_L_PID ?
                    sw2_secondary_left_uuid : sw2_secondary_right_uuid, 16) == 0)
        target = &ins->secondary;
#endif
    if (!target)
        return;
    if (target->value_handle || !(characteristic->properties & properties) ||
        characteristic->start_handle <= ins->service.start_group_handle ||
        characteristic->value_handle <= characteristic->start_handle ||
        characteristic->end_handle < characteristic->value_handle ||
        characteristic->end_handle > ins->service.end_group_handle) {
        sw2_fail(ins, "invalid or ambiguous characteristic", characteristic->value_handle);
        return;
    }
    *target = *characteristic;
}

static void sw2_gatt_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
    (void)channel;
    if (packet_type != HCI_EVENT_PACKET || !packet || size < 4)
        return;
    uint8_t event = packet[0];
    // BTstack GATT events have extended lengths: notification length is uint16
    // and event[1] may wrap. Validate the actual callback size before accessors.
    uint16_t required;
    switch (event) {
        case GATT_EVENT_SERVICE_QUERY_RESULT: required = 28; break;
        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT: required = 32; break;
        case GATT_EVENT_ALL_CHARACTERISTIC_DESCRIPTORS_QUERY_RESULT: required = 26; break;
        case GATT_EVENT_QUERY_COMPLETE: required = 9; break;
        case GATT_EVENT_NOTIFICATION: required = 12; break;
        default: return;
    }
    if (size < required)
        return;
    uni_hid_device_t* d = uni_hid_device_get_instance_for_connection_handle(little_endian_read_16(packet, 2));
    sw2_instance_t* ins = sw2_instance(d);
    if (!ins || !sw2_live(ins))
        return;
    switch (event) {
        case GATT_EVENT_SERVICE_QUERY_RESULT: {
            if (ins->state != SW2_SERVICE)
                return;
            gatt_client_service_t service;
            gatt_event_service_query_result_get_service(packet, &service);
            if (ins->service.start_group_handle || memcmp(service.uuid128, sw2_service_uuid, 16) != 0 ||
                !service.start_group_handle || service.end_group_handle <= service.start_group_handle) {
                sw2_fail(ins, "invalid or ambiguous service", 0);
                return;
            }
            ins->service = service;
            break;
        }
        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT: {
            gatt_client_characteristic_t characteristic;
            gatt_event_characteristic_query_result_get_characteristic(packet, &characteristic);
            sw2_characteristic(ins, &characteristic);
            break;
        }
        case GATT_EVENT_ALL_CHARACTERISTIC_DESCRIPTORS_QUERY_RESULT: {
#if SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
            if (ins->state != SW2_RESPONSE_DESCRIPTOR && ins->state != SW2_INPUT_DESCRIPTOR &&
                ins->state != SW2_SECONDARY_DESCRIPTOR)
                return;
#else
            if (ins->state != SW2_RESPONSE_DESCRIPTOR && ins->state != SW2_INPUT_DESCRIPTOR)
                return;
#endif
            gatt_client_characteristic_descriptor_t descriptor;
            gatt_event_all_characteristic_descriptors_query_result_get_characteristic_descriptor(packet, &descriptor);
            if (descriptor.uuid16 != SW2_CCCD_UUID)
                return;
            bool input = ins->state == SW2_INPUT_DESCRIPTOR;
            gatt_client_characteristic_t* characteristic = input ? &ins->input : &ins->response;
            uint16_t* handle = input ? &ins->input_cccd : &ins->response_cccd;
#if SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
            if (ins->state == SW2_SECONDARY_DESCRIPTOR) {
                characteristic = &ins->secondary;
                handle = &ins->secondary_cccd;
            }
#endif
            if (*handle || descriptor.handle <= characteristic->value_handle || descriptor.handle > characteristic->end_handle) {
                sw2_fail(ins, "invalid or ambiguous CCCD", descriptor.handle);
                return;
            }
            *handle = descriptor.handle;
            break;
        }
        case GATT_EVENT_QUERY_COMPLETE:
            sw2_query_complete(ins, gatt_event_query_complete_get_att_status(packet));
            break;
        case GATT_EVENT_NOTIFICATION: {
            uint16_t length = gatt_event_notification_get_value_length(packet);
            if (length > size - 12)
                return;
            uint16_t handle = gatt_event_notification_get_value_handle(packet);
            const uint8_t* value = gatt_event_notification_get_value(packet);
#if SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
            // BLE omits report IDs. Keep both formats raw and never route the
            // secondary packed motion data through the common gamepad parser.
            if (length && ins->input_listening && handle == ins->input.value_handle)
                sw2_capture(ins, 0x05, value, length);
            else if (length && ins->secondary_listening && handle == ins->secondary.value_handle)
                sw2_capture(ins, ins->device->product_id == UNI_SW2_JOYCON_L_PID ? 0x07 : 0x08, value, length);
#endif
            if (ins->response_listening && handle == ins->response.value_handle)
                sw2_response(ins, value, length);
            else if (ins->state == SW2_READY && ins->input_listening && handle == ins->input.value_handle && length == SW2_REPORT_SIZE) {
                uni_hid_parser_switch2_parse_input_report(d, value, length);
                uni_hid_device_process_controller(d);
            }
            break;
        }
    }
}

bool uni_bt_le_switch2_handle_advertisement(const uint8_t* packet, uint16_t size) {
    if (!packet || size < 12 || packet[0] != GAP_EVENT_ADVERTISING_REPORT ||
        (unsigned)packet[1] + 2 > size || packet[1] < 10 || packet[11] > packet[1] - 10)
        return false;
    // Standard ADV_IND only; never connect to scan responses or wake-host data.
    if (gap_event_advertising_report_get_advertising_event_type(packet) != 0)
        return false;
    const uint8_t* data = gap_event_advertising_report_get_data(packet);
    unsigned length = gap_event_advertising_report_get_data_length(packet);
    const uint8_t* manufacturer = NULL;
    for (unsigned offset = 0; offset < length;) {
        unsigned field_length = data[offset];
        if (!field_length)
            break;
        if (field_length > length - offset - 1)
            return false;
        if (data[offset + 1] == 0xff && field_length >= 19) {
            const uint8_t* candidate = data + offset + 2;
            if (little_endian_read_16(candidate, 0) == 0x0553 &&
                little_endian_read_16(candidate, 5) == UNI_SW2_NINTENDO_VID &&
                sw2_product(little_endian_read_16(candidate, 7))) {
                if (manufacturer)
                    return false;
                manufacturer = candidate;
            }
        }
        offset += field_length + 1;
    }
    if (!manufacturer)
        return false;
    bd_addr_t local, address;
    gap_local_bd_addr(local);
    gap_event_advertising_report_get_address(packet, address);
    bool fresh = true, reconnect = true;
    for (unsigned i = 0; i < 6; ++i) {
        fresh &= manufacturer[12 + i] == 0;
        reconnect &= manufacturer[12 + i] == local[5 - i];
    }
    // Returning true consumes recognized but disallowed Switch2 advertisements,
    // preventing generic HID/SMP admission from bypassing this protocol policy.
    if ((!fresh && !reconnect) || (fresh && !switch_pico_switch2_pairing_allowed()))
        return true;
    uint8_t address_type = gap_event_advertising_report_get_address_type(packet);
    if (address_type != BD_ADDR_TYPE_LE_PUBLIC &&
        !(address_type == BD_ADDR_TYPE_LE_RANDOM && (address[0] & 0xc0) == 0xc0))
        return true;
    if (!fresh && !uni_switch2_pairing_known(address_type, address))
        return true;
    if (uni_hid_device_get_instance_for_address(address))
        return true;
    uint8_t rssi = gap_event_advertising_report_get_rssi(packet);
    const uint16_t cod = UNI_BT_COD_MAJOR_PERIPHERAL | UNI_BT_COD_MINOR_GAMEPAD;
    sw2_instance_t* ins = NULL;
    for (unsigned i = 0; i < CONFIG_BLUEPAD32_MAX_DEVICES; ++i) {
        if (sw2_instances[i].state == SW2_OFF &&
            (!sw2_instances[i].allocated || gap_get_connection_type(sw2_instances[i].handle) == GAP_CONNECTION_INVALID)) {
            ins = &sw2_instances[i];
            break;
        }
    }
    if (!ins)
        return true;
    uni_hid_device_t* d = uni_hid_device_create(address);
    if (!d)
        return true;
    memset(ins, 0, sizeof(*ins));
    ins->allocated = true;
    ins->device = d;
    memcpy(ins->address, address, sizeof(ins->address));
    ins->address_type = address_type;
    ins->handle = UNI_BT_CONN_HANDLE_INVALID;
    ins->needs_pair = fresh;
    ins->state = SW2_ADMITTED;
    for (unsigned stick = 0; stick < 2; ++stick) {
        for (unsigned axis = 0; axis < 2; ++axis) {
            ins->sticks[stick].center[axis] = 2048;
            ins->sticks[stick].positive[axis] = 2047;
            ins->sticks[stick].negative[axis] = 2048;
        }
    }
    uni_hid_device_set_vendor_id(d, UNI_SW2_NINTENDO_VID);
    uni_hid_device_set_product_id(d, little_endian_read_16(manufacturer, 7));
    uni_bt_conn_set_protocol(&d->conn, UNI_BT_CONN_PROTOCOL_BLE);
    uni_hid_device_guess_controller_type_from_pid_vid(d);
    d->sdp_query_type = SDP_QUERY_NOT_NEEDED;
    uni_hid_device_set_cod(d, cod);
    uni_hid_device_set_name(d, "Switch2");
    uni_bt_conn_set_state(&d->conn, UNI_BT_CONN_STATE_DEVICE_DISCOVERED);
    d->conn.rssi = rssi;
    // Platform admission needs the validated transport and identity metadata.
    // Keep generic allowlist/RSSI filtering authoritative before touching scan.
    if (uni_hid_device_on_device_discovered(address, "Switch2", cod, rssi) != UNI_ERROR_SUCCESS) {
        uni_hid_device_delete(d);
        return true;
    }
    gap_stop_scan(); // Existing BLE lifecycle owns resuming scan under policy.
    uint8_t status = gap_connect(address, address_type);
    if (status != ERROR_CODE_SUCCESS) {
        sw2_fail(ins, "BLE connection request failed", status);
        uni_bt_le_resume_scanning_if_enabled();
    }
    return true;
}

void uni_hid_parser_switch2_on_le_connected(uni_hid_device_t* d) {
    sw2_instance_t* ins = sw2_instance(d);
    if (!ins || ins->state != SW2_ADMITTED)
        return;
    ins->handle = d->conn.handle;
    if (!d->conn.connected)
        uni_hid_device_connect(d);
    // A platform callback can reject/disconnect and destroy this device.
    if (sw2_instance(d) == ins && d->conn.connected)
        uni_hid_device_set_ready(d);
}

void uni_hid_parser_switch2_setup(uni_hid_device_t* d) {
    sw2_instance_t* ins = sw2_instance(d);
    if (!ins || ins->state != SW2_ADMITTED)
        return;
    ins->handle = d->conn.handle;
    // Standard-compliant 7.5ms minimum; negotiation failure is not setup failure.
    int status = gap_update_connection_parameters(ins->handle, 6, 6, 0, 600);
    if (status != ERROR_CODE_SUCCESS)
        logi("Switch2: connection interval request declined (%d)\n", status);
    ins->state = SW2_SERVICE;
    ins->query = SW2_QUERY_DISCOVERY;
    sw2_arm_timeout(ins);
    sw2_check_query(ins, gatt_client_discover_primary_services_by_uuid128(sw2_gatt_handler, ins->handle, sw2_service_uuid));
}

uint8_t uni_hid_parser_switch2_extra_buttons(const uni_hid_device_t* d) {
    sw2_instance_t* ins = sw2_instance(d);
    return ins ? ins->extra_buttons : 0;
}

bool uni_hid_parser_switch2_identity_address_type(const uni_hid_device_t* d, uint8_t* out) {
    sw2_instance_t* ins = sw2_instance(d);
    if (!ins || !out)
        return false;
    *out = ins->address_type;
    return true;
}

void uni_hid_parser_switch2_init_report(uni_hid_device_t* d) {
    (void)d; // Full snapshots replace state only after their complete length is validated.
}

static int32_t sw2_axis(uint16_t raw, const sw2_stick_t* stick, unsigned axis, bool invert) {
    int32_t delta = (int32_t)raw - stick->center[axis];
    int32_t result = delta * 512 / (delta < 0 ? stick->negative[axis] : stick->positive[axis]);
    if (invert)
        result = -result;
    return result < -512 ? -512 : result > 511 ? 511 : result;
}

static void sw2_motion(sw2_instance_t* ins, uni_gamepad_t* gp, const uint8_t* report) {
    uint32_t timestamp = little_endian_read_32(report, 42);
    if (!timestamp || (timestamp == ins->sensor_last && !ins->gyro_full_scale))
        return;
    ins->sensor_last = timestamp;
    if (!ins->gyro_full_scale) {
        // SDL detects two sensor clock/range variants. BLE reports need not be
        // 4ms apart as USB reports are: compare sensor ticks against elapsed
        // host time over >=400ms instead of assuming USB's 100 * 4ms interval.
        uint32_t now = btstack_run_loop_get_time_ms();
        if (ins->sensor_warmup < 5) {
            if (++ins->sensor_warmup == 5) {
                ins->sensor_start = timestamp;
                ins->sensor_host_start = now;
            }
            return;
        }
        uint32_t elapsed = now - ins->sensor_host_start;
        if (elapsed < 400)
            return;
        uint32_t ticks = timestamp - ins->sensor_start; // Defined wraparound.
        if (!ticks || elapsed > 2000) {
            ins->sensor_start = timestamp;
            ins->sensor_host_start = now;
            return;
        }
        uint32_t rate = ticks / elapsed;
        // round(34.8 or 40 radians/s * 180/pi * q10).
        ins->gyro_full_scale = rate >= 900 && rate <= 1100 ? 2041747 : 2346835;
    }
    // The common report is SDL's USB 0x05 report without the report-ID byte.
    // SDL vertical/paired axes: X, Z, -Y. Backend alone rotates solo Joy-Cons.
    static const uint8_t axes[3] = {0, 2, 1};
    for (unsigned i = 0; i < 3; ++i) {
        unsigned axis = axes[i];
        int32_t accel = (int16_t)little_endian_read_16(report, 48 + 2 * axis);
        int32_t gyro = (int16_t)little_endian_read_16(report, 54 + 2 * axis);
        gp->accel[i] = uni_imu_scale(accel, 32767, 8 * UNI_IMU_ACCEL_RES_PER_G);
        gp->gyro[i] = uni_imu_scale(gyro, 32767, ins->gyro_full_scale) - ins->gyro_bias[axis];
        if (i == 2) {
            gp->accel[i] = -gp->accel[i];
            gp->gyro[i] = -gp->gyro[i];
        }
    }
}

void uni_hid_parser_switch2_parse_input_report(uni_hid_device_t* d, const uint8_t* report, uint16_t len) {
    sw2_instance_t* ins = sw2_instance(d);
    if (!ins || ins->state != SW2_READY || !report || len != SW2_REPORT_SIZE)
        return;
    uni_gamepad_t* gp = &d->controller.gamepad;
    memset(gp, 0, sizeof(*gp));
    uint32_t buttons = little_endian_read_32(report, 4);
    gp->buttons = ((buttons & 0x04) ? BUTTON_A : 0) | ((buttons & 0x08) ? BUTTON_B : 0) |
                  ((buttons & 0x01) ? BUTTON_X : 0) | ((buttons & 0x02) ? BUTTON_Y : 0) |
                  ((buttons & 0x40) ? BUTTON_SHOULDER_R : 0) | ((buttons & 0x400000) ? BUTTON_SHOULDER_L : 0) |
                  ((buttons & 0x80) ? BUTTON_TRIGGER_R : 0) | ((buttons & 0x800000) ? BUTTON_TRIGGER_L : 0) |
                  ((buttons & 0x400) ? BUTTON_THUMB_R : 0) | ((buttons & 0x800) ? BUTTON_THUMB_L : 0);
    gp->dpad = ((buttons & 0x10000) ? DPAD_DOWN : 0) | ((buttons & 0x20000) ? DPAD_UP : 0) |
               ((buttons & 0x40000) ? DPAD_RIGHT : 0) | ((buttons & 0x80000) ? DPAD_LEFT : 0);
    gp->misc_buttons = ((buttons & 0x100) ? MISC_BUTTON_SELECT : 0) | ((buttons & 0x200) ? MISC_BUTTON_START : 0) |
                       ((buttons & 0x1000) ? MISC_BUTTON_SYSTEM : 0) | ((buttons & 0x2000) ? MISC_BUTTON_CAPTURE : 0);
    gp->brake = (buttons & 0x800000) ? 1023 : 0;
    gp->throttle = (buttons & 0x80) ? 1023 : 0;
    ins->extra_buttons = (buttons & 0x4000) ? UNI_SW2_BUTTON_C : 0;
    if (d->product_id == UNI_SW2_PRO_PID)
        ins->extra_buttons |= ((buttons & 0x2000000) ? UNI_SW2_BUTTON_GL : 0) | ((buttons & 0x1000000) ? UNI_SW2_BUTTON_GR : 0);
    else if (d->product_id == UNI_SW2_JOYCON_L_PID)
        ins->extra_buttons |= ((buttons & 0x200000) ? UNI_SW2_BUTTON_LEFT_SL : 0) | ((buttons & 0x100000) ? UNI_SW2_BUTTON_LEFT_SR : 0);
    else
        ins->extra_buttons |= ((buttons & 0x20) ? UNI_SW2_BUTTON_RIGHT_SL : 0) | ((buttons & 0x10) ? UNI_SW2_BUTTON_RIGHT_SR : 0);
    uint16_t axes[2];
    if (d->product_id != UNI_SW2_JOYCON_R_PID) {
        sw2_unpack_stick(report + 10, axes);
        gp->axis_x = sw2_axis(axes[0], &ins->sticks[0], 0, false);
        gp->axis_y = sw2_axis(axes[1], &ins->sticks[0], 1, true);
    }
    if (d->product_id != UNI_SW2_JOYCON_L_PID) {
        sw2_unpack_stick(report + 13, axes);
        gp->axis_rx = sw2_axis(axes[0], &ins->sticks[1], 0, false);
        gp->axis_ry = sw2_axis(axes[1], &ins->sticks[1], 1, true);
    }
    sw2_motion(ins, gp, report);
    d->controller.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
}

void uni_hid_parser_switch2_set_player_leds(uni_hid_device_t* d, uint8_t leds) {
    sw2_instance_t* ins = sw2_instance(d);
    if (!ins)
        return;
    ins->leds = leds & 0x0f; // Bluepad32 passes a bitmask, not a player number.
    ins->leds_pending = true;
    if (ins->state == SW2_READY)
        sw2_continue(ins);
}

uint32_t uni_hid_parser_switch2_haptics_dropped(void) {
    return atomic_load_explicit(&sw2_haptics_drops, memory_order_relaxed);
}

static void sw2_count_drops(unsigned count) {
    atomic_fetch_add_explicit(&sw2_haptics_drops, count, memory_order_relaxed);
}

static bool sw2_due(uint32_t now, uint32_t deadline) {
    return (int32_t)(now - deadline) >= 0;
}

static unsigned sw2_output_sides(const sw2_instance_t* ins) {
    return ins->device->product_id == UNI_SW2_PRO_PID ? 2 : 1;
}

static void sw2_pop_host(sw2_instance_t* ins) {
    ins->host_head = (ins->host_head + 1) % SW2_HAPTICS_CAPACITY;
    --ins->host_count;
}

static void sw2_discard_host(sw2_instance_t* ins) {
    sw2_count_drops(ins->host_count);
    ins->host_head = ins->host_count = 0;
    memset(ins->host_sides, 0, sizeof(ins->host_sides));
}

static void sw2_retain_host(sw2_instance_t* ins, const sw2_host_command_t* command) {
    for (unsigned i = 0; i < sw2_output_sides(ins); ++i) {
        const uni_switch2_haptics_side_t* side = &command->frame.sides[i];
        if (!side->count)
            continue;
        sw2_host_side_t* retained = &ins->host_sides[i];
        memcpy(retained->sample, side->samples[side->count - 1], sizeof(retained->sample));
        retained->expires = command->expires;
        retained->held = command->held;
        retained->valid = true;
    }
}

static bool sw2_local_active(const sw2_instance_t* ins, uint32_t now) {
    return ins->rumble_scheduled && sw2_due(now, ins->rumble_start) &&
           (ins->rumble_held || !sw2_due(now, ins->rumble_end));
}

// Logical host time keeps advancing even when ATT or the local overlay owns
// the physical output. Masked sequences become final holds, never a replay log.
static void sw2_update_haptics(sw2_instance_t* ins, uint32_t now) {
    bool active = sw2_local_active(ins, now);
    bool masked = active || ins->feedback_active;
    while (ins->host_count) {
        sw2_host_command_t* command = &ins->host_queue[ins->host_head];
        if (!command->held && sw2_due(now, command->expires)) {
            sw2_count_drops(1);
            sw2_pop_host(ins);
            // Unplayed history is not a reason to interrupt the current packet.
            // A matching in-flight packet can still complete after its expiry.
            if (!active && ins->pending_host &&
                ins->pending_epoch == ins->haptics_epoch &&
                ins->pending_serial == command->serial) {
                ins->output_urgent = true;
                ++ins->output_revision;
            }
        } else if (masked) {
            sw2_retain_host(ins, command);
            sw2_pop_host(ins);
        } else {
            break;
        }
    }
    if (masked)
        ins->barrier_pending = false;
    for (unsigned i = 0; i < sw2_output_sides(ins); ++i) {
        sw2_host_side_t* side = &ins->host_sides[i];
        if (side->valid && !side->held && sw2_due(now, side->expires)) {
            side->valid = false;
            if (!active) {
                ins->output_urgent = true;
                ++ins->output_revision;
            }
        }
    }
    if (ins->feedback_active != active) {
        if (!active)
            ins->output_urgent = true; // Resume only the current per-side hold.
        ++ins->output_revision;
    }
    ins->feedback_active = active;
    if (ins->rumble_scheduled && !ins->rumble_held && sw2_due(now, ins->rumble_end))
        ins->rumble_scheduled = false;
}

static void sw2_compat_side(uni_switch2_haptics_side_t* side, uint8_t weak, uint8_t strong) {
    // Preserve conventional/local frequency and strength, but advertise exactly
    // one sample. The other ten bytes are padding, not repeated substeps.
    uint64_t frame = 0x0e1u | ((uint64_t)strong * 4 << 10) | ((uint64_t)0x1e1 << 20) |
                     ((uint64_t)weak * 4 << 30);
    memset(side, 0, sizeof(*side));
    side->count = 1;
    for (unsigned i = 0; i < 5; ++i)
        side->samples[0][i] = (uint8_t)(frame >> (8 * i));
}

static void sw2_host_hold(const sw2_instance_t* ins, uni_switch2_haptics_frame_t* frame) {
    uni_switch2_haptics_silence(frame);
    for (unsigned i = 0; i < sw2_output_sides(ins); ++i) {
        if (ins->host_sides[i].valid)
            memcpy(frame->sides[i].samples[0], ins->host_sides[i].sample, 5);
    }
}

static void sw2_host_stop(sw2_instance_t* ins) {
    sw2_update_haptics(ins, btstack_run_loop_get_time_ms());
    sw2_discard_host(ins);
    ++ins->haptics_epoch;
    ++ins->output_revision;
    ins->barrier_pending = true;
    ins->output_urgent = true;
    sw2_schedule_output(ins, 1);
}

static bool sw2_physical_stop(const sw2_instance_t* ins, const uni_switch2_haptics_frame_t* frame) {
    for (unsigned i = 0; i < sw2_output_sides(ins); ++i) {
        const uni_switch2_haptics_side_t* side = &frame->sides[i];
        if (!side->count)
            return false;
        for (unsigned j = 0; j < side->count; ++j) {
            const uint8_t* sample = side->samples[j];
            // Amplitudes occupy bits10..19 and30..39 of each 40-bit sample.
            if ((sample[1] & 0xfc) || (sample[2] & 0x0f) || (sample[3] & 0xc0) || sample[4])
                return false;
        }
    }
    return true;
}

static bool sw2_same_hold(const uni_switch2_haptics_frame_t* a, const uni_switch2_haptics_frame_t* b) {
    for (unsigned i = 0; i < 2; ++i) {
        if (a->sides[i].count > 1 || a->sides[i].count != b->sides[i].count)
            return false;
        if (a->sides[i].count && memcmp(a->sides[i].samples[0], b->sides[i].samples[0], 5) != 0)
            return false;
    }
    return true;
}

static bool sw2_queue_host(sw2_instance_t* ins, const uni_switch2_haptics_frame_t* frame,
                            uint32_t received_ms, uint16_t duration_ms, bool native) {
    uint32_t now = btstack_run_loop_get_time_ms();
    sw2_update_haptics(ins, now);
    bool held = duration_ms == UINT16_MAX;
    uint32_t lifetime = duration_ms < UNI_SWITCH2_HAPTICS_WATCHDOG_MS ?
                            duration_ms : UNI_SWITCH2_HAPTICS_WATCHDOG_MS;
    uint32_t expires = received_ms + lifetime;
    if (!held && sw2_due(now, expires)) {
        sw2_count_drops(1);
        return true;
    }
    if (ins->host_count) {
        unsigned previous = (ins->host_head + ins->host_count - 1) % SW2_HAPTICS_CAPACITY;
        sw2_host_command_t* command = &ins->host_queue[previous];
        if (command->native == native && sw2_same_hold(&command->frame, frame)) {
            // Same ordered hold, new source lifetime. Keep its serial and wire
            // bytes intact even when ATT is still borrowing this queue head.
            command->expires = expires;
            command->held = held;
            sw2_schedule_output(ins, 1);
            return true;
        }
    }
    if (ins->host_count == SW2_HAPTICS_CAPACITY)
        return false;
    unsigned tail = (ins->host_head + ins->host_count) % SW2_HAPTICS_CAPACITY;
    ins->host_queue[tail] = (sw2_host_command_t){
        .frame = *frame, .expires = expires, .serial = ++ins->host_serial, .held = held, .native = native,
    };
    ++ins->host_count;
    sw2_schedule_output(ins, 1);
    return true;
}

bool uni_hid_parser_switch2_queue_haptics(uni_hid_device_t* d,
                                         const uni_switch2_haptics_frame_t* frame, uint32_t received_ms) {
    sw2_instance_t* ins = sw2_instance(d);
    if (!ins || ins->state != SW2_READY)
        return false;
    if (!uni_switch2_haptics_valid(frame) ||
        (sw2_output_sides(ins) == 1 && !frame->sides[0].count)) {
        sw2_count_drops(1);
        return true;
    }
    if (sw2_physical_stop(ins, frame)) {
        sw2_host_stop(ins); // A stop is a barrier, including when full or stale.
        return true;
    }
    return sw2_queue_host(ins, frame, received_ms, UNI_SWITCH2_HAPTICS_WATCHDOG_MS, true);
}

bool uni_hid_parser_switch2_queue_rumble(uni_hid_device_t* d, uint8_t weak, uint8_t strong,
                                        uint16_t duration_ms, uint32_t received_ms) {
    sw2_instance_t* ins = sw2_instance(d);
    if (!ins || ins->state != SW2_READY)
        return false;
    if ((!weak && !strong) || !duration_ms) {
        sw2_host_stop(ins);
        return true;
    }
    uni_switch2_haptics_frame_t frame;
    sw2_compat_side(&frame.sides[0], weak, strong);
    frame.sides[1] = frame.sides[0];
    return sw2_queue_host(ins, &frame, received_ms, duration_ms, false);
}

void uni_hid_parser_switch2_reset_haptics(uni_hid_device_t* d) {
    sw2_instance_t* ins = sw2_instance(d);
    if (!ins)
        return;
    sw2_discard_host(ins);
    ++ins->haptics_epoch;
    ++ins->output_revision;
    ins->rumble_scheduled = ins->feedback_active = false;
    ins->barrier_pending = ins->output_urgent = true;
    // Do not touch rumble_data or pending metadata: ATT can still borrow them.
    if (ins->state == SW2_READY)
        sw2_schedule_output(ins, 1);
}

static void sw2_rumble_complete(sw2_instance_t* ins) {
    uint32_t now = btstack_run_loop_get_time_ms();
    ++ins->rumble_id; // Only a successful write consumes the physical sequence.
    // Track transport success even for an old logical epoch: a late active
    // packet invalidates prior silence and must be followed by fresh stops.
    if (!ins->pending_neutral)
        ins->neutral_writes = 0;
    else if (ins->neutral_writes < SW2_NEUTRAL_WRITE_BUDGET)
        ++ins->neutral_writes;
    ins->playback_until = now + ins->pending_guard_ms;
    ins->playback_guard = true;
    sw2_update_haptics(ins, now);
    if (ins->pending_epoch != ins->haptics_epoch)
        return;
    if (ins->pending_host && ins->host_count) {
        sw2_host_command_t* command = &ins->host_queue[ins->host_head];
        if (command->serial == ins->pending_serial) {
            sw2_retain_host(ins, command);
            sw2_pop_host(ins);
        }
    }
    if (ins->pending_barrier)
        ins->barrier_pending = false;
    if (ins->pending_revision == ins->output_revision)
        ins->output_urgent = false;
}

static bool sw2_send_rumble(sw2_instance_t* ins, uint32_t now) {
    // Never overwrite the persistent packet while a write request borrows it.
    if (ins->query != SW2_QUERY_NONE || ins->command_pending)
        return false;
    if (ins->playback_guard && !sw2_due(now, ins->playback_until) && !ins->output_urgent)
        return false;
    uni_switch2_haptics_frame_t frame;
    ins->pending_host = false;
    ins->pending_guard_ms = 6;
    unsigned sides = sw2_output_sides(ins);
    if (ins->feedback_active) {
        sw2_compat_side(&frame.sides[0], ins->weak, ins->strong);
        frame.sides[1] = frame.sides[0];
    } else {
        sw2_host_hold(ins, &frame);
        while (!ins->barrier_pending && ins->host_count) {
            const sw2_host_command_t* command = &ins->host_queue[ins->host_head];
            unsigned count = 1;
            for (unsigned i = 0; i < sides; ++i)
                if (command->frame.sides[i].count > count) count = command->frame.sides[i].count;
            uint8_t guard_ms = (count * 16 + 2) / 3;
            // Do not begin a native sequence that its original watchdog would
            // cut off halfway through. Skip it and keep servicing fresh work.
            if (command->native && !command->held &&
                (int32_t)(command->expires - now) < guard_ms) {
                sw2_count_drops(1);
                sw2_pop_host(ins);
                continue;
            }
            for (unsigned i = 0; i < sides; ++i) {
                if (command->frame.sides[i].count)
                    frame.sides[i] = command->frame.sides[i];
            }
            ins->pending_guard_ms = guard_ms;
            ins->pending_host = true;
            ins->pending_serial = command->serial;
            break;
        }
    }
    ins->pending_neutral = sw2_physical_stop(ins, &frame);
    if (ins->pending_neutral && !ins->pending_host &&
        ins->neutral_writes == SW2_NEUTRAL_WRITE_BUDGET) {
        // Three successful neutral writes settle idle output. Repeated host
        // stops still discard queued history, but need no further radio work.
        // Never suppress a queued native sequence, even if its endpoint is zero.
        ins->barrier_pending = false;
        ins->output_urgent = false;
        return false;
    }
    ins->pending_epoch = ins->haptics_epoch;
    ins->pending_revision = ins->output_revision;
    ins->pending_barrier = ins->barrier_pending;
    ins->rumble_data[0] = 0;
    for (unsigned i = 0; i < sides; ++i) {
        uni_switch2_haptics_write_block(ins->rumble_data + 1 + 16 * i, &frame.sides[i], ins->rumble_id);
    }
    uint16_t length = 1 + 16 * sides;
    bool no_response = (ins->rumble.properties & ATT_PROPERTY_WRITE_WITHOUT_RESPONSE) != 0;
    uint8_t status;
    if (no_response) {
        status = gatt_client_write_value_of_characteristic_without_response(ins->handle, ins->rumble.value_handle,
                                                                          length, ins->rumble_data);
    } else {
        ins->query = SW2_QUERY_RUMBLE;
        sw2_arm_timeout(ins);
        status = gatt_client_write_value_of_characteristic(sw2_gatt_handler, ins->handle, ins->rumble.value_handle,
                                                          length, ins->rumble_data);
    }
    if (status == ERROR_CODE_SUCCESS) {
        if (no_response)
            sw2_rumble_complete(ins);
        return true;
    }
    ins->query = SW2_QUERY_NONE;
    if (!ins->command_pending)
        sw2_disarm_timeout(ins);
    if (!sw2_transient_write_error(status))
        sw2_fail(ins, "rumble write failed", status);
    return false;
}

static void sw2_next_boundary(uint32_t now, uint32_t boundary, uint32_t* next) {
    if (!sw2_due(now, boundary) && boundary - now < *next)
        *next = boundary - now;
}

static void sw2_output_tick(btstack_timer_source_t* timer) {
    sw2_instance_t* ins = btstack_run_loop_get_timer_context(timer);
    ins->output_active = false;
    if (!sw2_live(ins))
        return;
    sw2_try_command(ins);
    if (!ins->device)
        return;
    if (ins->state != SW2_READY)
        return; // A blocked setup command rescheduled itself, or awaits its ACK.
    uint32_t now = btstack_run_loop_get_time_ms();
#if SWITCH_PICO_SWITCH2_USB_BRIDGE
    if (!ins->command_pending && ins->query == SW2_QUERY_NONE) {
        uint8_t sample_id;
        if (switch_pico_switch2_sample_take(ins->device->product_id, ins->address, now,
                                            &sample_id, &ins->sample_token)) {
            const uint8_t data[4] = {sample_id, 0, 0, 0};
            sw2_send_command(ins, 0x0a, 0x02, data, sizeof(data));
            if (!ins->device)
                return;
        }
    }
#endif
    sw2_update_haptics(ins, now);
    bool sent = sw2_send_rumble(ins, now);
    if (!ins->device)
        return;
    uint32_t next = SW2_OUTPUT_INTERVAL_MS;
    if (ins->playback_guard && (ins->host_count || !sent))
        sw2_next_boundary(now, ins->playback_until, &next);
    if (ins->rumble_scheduled) {
        sw2_next_boundary(now, ins->rumble_start, &next);
        if (!ins->rumble_held)
            sw2_next_boundary(now, ins->rumble_end, &next);
    }
    for (unsigned i = 0; i < sw2_output_sides(ins); ++i) {
        if (ins->host_sides[i].valid && !ins->host_sides[i].held)
            sw2_next_boundary(now, ins->host_sides[i].expires, &next);
    }
    if (ins->host_count && !ins->host_queue[ins->host_head].held)
        sw2_next_boundary(now, ins->host_queue[ins->host_head].expires, &next);
    sw2_schedule_output(ins, next);
}

void uni_hid_parser_switch2_play_dual_rumble(uni_hid_device_t* d, uint16_t delay_ms, uint16_t duration_ms,
                                          uint8_t weak, uint8_t strong) {
    sw2_instance_t* ins = sw2_instance(d);
    if (!ins || ins->state != SW2_READY)
        return;
    uint32_t now = btstack_run_loop_get_time_ms();
    sw2_update_haptics(ins, now);
    ins->weak = weak;
    ins->strong = strong;
    ins->rumble_start = now + delay_ms;
    ins->rumble_end = ins->rumble_start + duration_ms;
    ins->rumble_held = duration_ms == UINT16_MAX;
    ins->rumble_scheduled = duration_ms != 0 && (weak != 0 || strong != 0);
    ++ins->output_revision;
    sw2_update_haptics(ins, now);
    sw2_schedule_output(ins, 1);
}
