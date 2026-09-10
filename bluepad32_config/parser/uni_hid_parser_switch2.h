// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "parser/uni_switch2_haptics.h"

#ifdef __cplusplus
extern "C" {
#endif

struct uni_hid_device_s;

#define UNI_SW2_NINTENDO_VID 0x057e
#define UNI_SW2_PRO_PID 0x2069
#define UNI_SW2_JOYCON_L_PID 0x2067
#define UNI_SW2_JOYCON_R_PID 0x2066

enum {
    UNI_SW2_BUTTON_C = 1u << 0,
    UNI_SW2_BUTTON_GL = 1u << 1,
    UNI_SW2_BUTTON_GR = 1u << 2,
    UNI_SW2_BUTTON_LEFT_SL = 1u << 3,
    UNI_SW2_BUTTON_LEFT_SR = 1u << 4,
    UNI_SW2_BUTTON_RIGHT_SL = 1u << 5,
    UNI_SW2_BUTTON_RIGHT_SR = 1u << 6,
};

// Supplied by the platform's existing bounded pairing-window policy.
bool switch_pico_switch2_pairing_allowed(void);

#if SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
// BTstack-core callback: canonical Bluepad32 address, unmodified BLE payload
// (without a report-ID prefix), at most 64 bytes. 0xc0 is a validated feature
// reply; report_id=0 with length=0 marks teardown and permits a NULL report.
void switch_pico_switch2_mouse_report(uint16_t product_id, const uint8_t address[6], uint8_t report_id,
                                     const uint8_t* report, uint16_t length, uint32_t received_ms);
#endif

#if SWITCH_PICO_SWITCH2_USB_BRIDGE
// BTstack-core mailbox hooks. Take only when READY with no command/query in
// flight. Result 0 checks ownership before a deferred write, 1 records verified
// command completion, and -1 fails it. False means canceled/stale/wrong source.
bool switch_pico_switch2_sample_take(uint16_t product_id, const uint8_t address[6],
                                    uint32_t now_ms, uint8_t* sample_id, uint64_t* token);
bool switch_pico_switch2_sample_result(uint16_t product_id, const uint8_t address[6],
                                      uint64_t token, int result, uint32_t now_ms);
#endif

bool uni_bt_le_switch2_handle_advertisement(const uint8_t* packet, uint16_t size);
bool uni_hid_parser_switch2_is_ble_device(const struct uni_hid_device_s* d);
void uni_hid_parser_switch2_on_le_connected(struct uni_hid_device_s* d);
void uni_hid_parser_switch2_setup(struct uni_hid_device_s* d);
void uni_hid_parser_switch2_teardown(struct uni_hid_device_s* d);
void uni_hid_parser_switch2_init_report(struct uni_hid_device_s* d);
void uni_hid_parser_switch2_parse_input_report(struct uni_hid_device_s* d, const uint8_t* report, uint16_t len);
void uni_hid_parser_switch2_set_player_leds(struct uni_hid_device_s* d, uint8_t leds);
void uni_hid_parser_switch2_play_dual_rumble(struct uni_hid_device_s* d, uint16_t delay_ms, uint16_t duration_ms,
                                          uint8_t weak, uint8_t strong);
// BTstack-core-only host queues. False means unavailable/full: nothing accepted.
// Invalid/stale input is consumed and counted. received_ms is the source clock,
// never a retry timestamp. The callback above is an independent local overlay.
bool uni_hid_parser_switch2_queue_haptics(struct uni_hid_device_s* d,
                                         const uni_switch2_haptics_frame_t* frame, uint32_t received_ms);
bool uni_hid_parser_switch2_queue_rumble(struct uni_hid_device_s* d, uint8_t weak, uint8_t strong,
                                        uint16_t duration_ms, uint32_t received_ms);
void uni_hid_parser_switch2_reset_haptics(struct uni_hid_device_s* d);
// Atomic cumulative transport losses; safe to read without looking up a device.
uint32_t uni_hid_parser_switch2_haptics_dropped(void);
uint8_t uni_hid_parser_switch2_extra_buttons(const struct uni_hid_device_s* d);
// Validated public/static-random advertisement address, never an RPA or SMP identity.
bool uni_hid_parser_switch2_identity_address_type(const struct uni_hid_device_s* d, uint8_t* out);

#ifdef __cplusplus
}
#endif
