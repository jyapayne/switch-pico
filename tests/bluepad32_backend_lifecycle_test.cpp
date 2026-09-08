#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
#include <algorithm>
#include <array>
#endif

#include <uni.h>
#include "parser/uni_hid_parser_switch2.h"
#include "parser/uni_switch2_haptics.h"
#include "parser/uni_switch2_pairing.h"
#include "platform/pico/controller_color_config.h"
#include "input/switch2_wake.h"
#include "pico/critical_section.h"
#include "profile/profile_storage.h"
#include "profile/controller_profile_runtime.h"

namespace {


bool incoming_connections = false;
bool scanning_enabled = false;
bool aggregate_scanning_enabled = false;
bool background_scan_parameters = false;
bool classic_scanning_enabled = false;
uni_platform* installed_platform = nullptr;
bool observed_status_led_on = false;
int observed_status_led_writes = 0;
uint32_t now_ms = 0;
bool bondable = true;
bool ssp_auto_accept = true;
uint8_t accepted_stk_methods = 0xff;
uint16_t link_supervision_timeout = 0;
btstack_packet_handler_t pairing_event_handler = nullptr;
btstack_packet_handler_t identity_event_handler = nullptr;
int confirmation_accepts = 0;
int confirmation_rejections = 0;
int passkey_accepts = 0;
int passkey_rejections = 0;
int delete_key_calls = 0;
bd_addr_t classic_bonds[4]{};
int classic_bond_count = 0;
bd_addr_t ble_bonds[4]{};
int ble_bond_types[4]{};
int ble_bond_count = 0;

bool flash_core_init_result = true;
int flash_core_init_calls = 0;
int core1_launch_calls = 0;
bool expect_configuration_timer_prearmed = false;
uint32_t expected_configuration_timer_add_count = 0;
int cyw43_init_calls = 0;
int uni_init_calls = 0;
void (*during_uni_init)() = nullptr;
int switch2_wake_initializations = 0;
int switch2_wake_requests = 0;
bool switch2_connections_ready = true;
int device_disconnect_calls = 0;
uni_hid_device_t* last_disconnected_device = nullptr;
uni_hid_device_t* lookup_devices[8]{};
size_t lookup_device_count = 0;
uint32_t expected_pending_clear_token = 0;
uint32_t repeated_in_progress_clear_token = 0;
bool repeat_clear_during_disconnect = false;
void require_clear_completion_pending();
void require_clear_snapshot_published();
void request_repeated_clear_during_disconnect();
gap_connection_type_t gap_connection_types[256]{};
uint16_t negotiated_intervals[256]{};
uint16_t pending_intervals[256]{};
unsigned interval_requests[256]{};
bool defer_interval_updates = false;
bool reject_interval_updates = false;
uint8_t xbox_left_trigger = 0;
uint8_t xbox_right_trigger = 0;
unsigned xbox_quad_calls = 0;
unsigned ordinary_smp_requests = 0;
bd_addr_t switch2_pairings[UNI_SWITCH2_PAIRING_CAPACITY]{};
uint8_t switch2_pairing_types[UNI_SWITCH2_PAIRING_CAPACITY]{};
uint8_t switch2_pairing_count = 0;
bool switch2_clear_succeeds = true;

struct Switch2HostEvent {
    uni_hid_device_t* device;
    uni_switch2_haptics_frame_t frame;
    uint32_t received_ms;
    uint16_t duration_ms;
    uint8_t weak;
    uint8_t strong;
    bool hd;
};
std::vector<Switch2HostEvent> switch2_host_events;
uint32_t switch2_output_drops = 0;

struct LocalRumbleEvent {
    uni_hid_device_t* device;
    uint16_t duration_ms;
    uint8_t high;
    uint8_t low;
};
std::vector<LocalRumbleEvent> local_rumble_events;

struct CoreStopped {};


void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

void play_rumble(uni_hid_device_t* device, uint16_t,
                 uint16_t duration_ms, uint8_t high, uint8_t low) {
    local_rumble_events.push_back({device, duration_ms, high, low});
    ++device->rumble_calls;
    device->last_high = high;
    device->last_low = low;
    device->last_rumble_duration_ms = duration_ms;
}
void set_lightbar(uni_hid_device_t* device, uint8_t red, uint8_t green,
                  uint8_t blue) {
    ++device->lightbar_calls;
    device->lightbar_red = red;
    device->lightbar_green = green;
    device->lightbar_blue = blue;
}

void set_player_leds(uni_hid_device_t* device, uint8_t leds) {
    ++device->player_led_calls;
    device->player_leds = leds;
}


uni_hid_device_t device(
    int idx, bool gamepad = true,
    uni_bt_conn_protocol_t protocol = UNI_BT_CONN_PROTOCOL_NONE) {
    uni_hid_device_t result{};
    result.idx = idx;
    result.gamepad = gamepad;
    result.conn.protocol = protocol;
    result.conn.handle = static_cast<hci_con_handle_t>(0x40 + idx);
    negotiated_intervals[result.conn.handle] = 6;
    pending_intervals[result.conn.handle] = 0;
    result.conn.btaddr[5] = static_cast<uint8_t>(idx + 1);
    result.vendor_id = static_cast<uint16_t>(0x1000 + idx);
    result.product_id = static_cast<uint16_t>(0x2000 + idx);
    result.report_parser.play_dual_rumble = play_rumble;
    gap_connection_types[result.conn.handle] =
        protocol == UNI_BT_CONN_PROTOCOL_BR_EDR
            ? GAP_CONNECTION_ACL
            : protocol == UNI_BT_CONN_PROTOCOL_BLE
                  ? GAP_CONNECTION_LE
                  : GAP_CONNECTION_INVALID;
    return result;
}

void register_lookup_device(uni_hid_device_t* candidate) {
    require(lookup_device_count <
                sizeof(lookup_devices) / sizeof(lookup_devices[0]),
            "test BLE lookup registry overflow");
    lookup_devices[lookup_device_count++] = candidate;
}

}  // namespace

void xboxone_play_quad_rumble(uni_hid_device_t* device, uint16_t delay,
                              uint16_t duration, uint8_t left, uint8_t right,
                              uint8_t weak, uint8_t strong) {
    ++xbox_quad_calls;
    xbox_left_trigger = left;
    xbox_right_trigger = right;
    play_rumble(device, delay, duration, weak, strong);
}

void uni_hid_parser_xboxone_play_dual_rumble(
    uni_hid_device_t* device, uint16_t delay, uint16_t duration,
    uint8_t weak, uint8_t strong) {
    xboxone_play_quad_rumble(device, delay, duration, 0, 0, weak, strong);
}


extern "C" void __real_sm_request_pairing(hci_con_handle_t) {
    ++ordinary_smp_requests;
}

extern "C" bool uni_switch2_pairing_get(
    uint8_t index, uint8_t* address_type, uint8_t address[6]) {
    if (index >= switch2_pairing_count) {
        return false;
    }
    *address_type = switch2_pairing_types[index];
    memcpy(address, switch2_pairings[index], sizeof(bd_addr_t));
    return true;
}

extern "C" bool uni_switch2_pairing_known(
    uint8_t address_type, const uint8_t address[6]) {
    for (uint8_t index = 0; index < switch2_pairing_count; ++index) {
        if (switch2_pairing_types[index] == address_type &&
            memcmp(switch2_pairings[index], address, sizeof(bd_addr_t)) == 0) {
            return true;
        }
    }
    return false;
}

extern "C" bool uni_switch2_pairing_clear(void) {
    if (!switch2_clear_succeeds) {
        return false;
    }
    switch2_pairing_count = 0;
    return true;
}

extern "C" bool uni_hid_parser_switch2_is_ble_device(
    const uni_hid_device_t* device) {
    return device != nullptr &&
           device->conn.protocol == UNI_BT_CONN_PROTOCOL_BLE &&
           device->vendor_id == UNI_SW2_NINTENDO_VID &&
           (device->product_id == UNI_SW2_PRO_PID ||
            device->product_id == UNI_SW2_JOYCON_L_PID ||
            device->product_id == UNI_SW2_JOYCON_R_PID);
}

extern "C" bool uni_hid_parser_switch2_queue_haptics(
    uni_hid_device_t* device, const uni_switch2_haptics_frame_t* frame,
    uint32_t received_ms) {
    require(uni_switch2_haptics_valid(frame), "backend emitted invalid physical HD frame");
    const bool stop = device->product_id == UNI_SW2_PRO_PID ?
        uni_switch2_haptics_is_stop(frame) : [&]() {
            uni_switch2_haptics_frame_t stereo = *frame;
            stereo.sides[1] = stereo.sides[0];
            return uni_switch2_haptics_is_stop(&stereo);
        }();
    if (device->switch2_host_blocked && !stop) return false;
    ++device->switch2_host_calls;
    switch2_host_events.push_back({device, *frame, received_ms, 0, 0, 0, true});
    return true;
}

extern "C" bool uni_hid_parser_switch2_queue_rumble(
    uni_hid_device_t* device, uint8_t weak, uint8_t strong,
    uint16_t duration_ms, uint32_t received_ms) {
    if (device->switch2_host_blocked && (weak | strong) != 0) return false;
    ++device->switch2_host_calls;
    device->last_high = weak;
    device->last_low = strong;
    device->last_rumble_duration_ms = duration_ms;
    switch2_host_events.push_back({device, {}, received_ms, duration_ms, weak, strong, false});
    return true;
}

extern "C" void uni_hid_parser_switch2_reset_haptics(uni_hid_device_t* device) {
    ++device->switch2_haptics_resets;
    device->last_high = 0;
    device->last_low = 0;
    device->last_rumble_duration_ms = 0;
}

extern "C" uint32_t uni_hid_parser_switch2_haptics_dropped(void) {
    return switch2_output_drops;
}

extern "C" uint8_t uni_hid_parser_switch2_extra_buttons(
    const uni_hid_device_t* device) {
    return uni_hid_parser_switch2_is_ble_device(device)
               ? device->switch2_extra_buttons : 0;
}

extern "C" bool uni_hid_parser_switch2_identity_address_type(
    const uni_hid_device_t* device, uint8_t* output) {
    if (!uni_hid_parser_switch2_is_ble_device(device) ||
        !device->switch2_identity_valid || output == nullptr) {
        return false;
    }
    *output = device->switch2_identity_address_type;
    return true;
}
bool uni_hid_device_is_gamepad(const uni_hid_device_t* device) {
    return device != nullptr && device->gamepad;
}

int uni_hid_device_get_idx_for_instance(const uni_hid_device_t* device) {
    return device == nullptr ? -1 : device->idx;
}

uni_hid_device_t* uni_hid_device_get_instance_for_connection_handle(
    hci_con_handle_t handle) {
    for (size_t index = 0; index < lookup_device_count; ++index) {
        if (lookup_devices[index]->conn.handle == handle) {
            return lookup_devices[index];
        }
    }
    return nullptr;
}

uni_hid_device_t* uni_hid_device_get_instance_for_address(const bd_addr_t address) {
    for (size_t index = 0; index < lookup_device_count; ++index) {
        if (memcmp(lookup_devices[index]->conn.btaddr, address, sizeof(bd_addr_t)) == 0) {
            return lookup_devices[index];
        }
    }
    return nullptr;
}
void uni_hid_device_disconnect(uni_hid_device_t* device) {
    require_clear_completion_pending();
    request_repeated_clear_during_disconnect();
    ++device_disconnect_calls;
    last_disconnected_device = device;
}

void uni_bt_allow_incoming_connections(bool enabled) {
    incoming_connections = enabled;
}


void uni_bt_bredr_scan_start() {
    classic_scanning_enabled = true;
}

void uni_bt_bredr_scan_stop() {
    classic_scanning_enabled = false;
}

void uni_bt_le_scan_start() {
    scanning_enabled = true;
}

void uni_bt_le_scan_stop() {
    scanning_enabled = false;
}

void uni_bt_le_set_background_scan(bool enabled) {
    require(!scanning_enabled, "LE scan timing must change only while scanning is stopped");
    background_scan_parameters = enabled;
}

void uni_bt_start_scanning_and_autoconnect_unsafe() {
    require_clear_completion_pending();
    require_clear_snapshot_published();
    if (aggregate_scanning_enabled) {
        return;
    }
    aggregate_scanning_enabled = true;
    uni_bt_bredr_scan_start();
    uni_bt_le_scan_start();
}

void uni_bt_stop_scanning_unsafe() {
    if (!aggregate_scanning_enabled) {
        return;
    }
    aggregate_scanning_enabled = false;
    uni_bt_bredr_scan_stop();
    uni_bt_le_scan_stop();
}
void uni_bt_del_keys_unsafe() {
    require_clear_completion_pending();
    ++delete_key_calls;
    classic_bond_count = 0;
    ble_bond_count = 0;
}

gap_connection_type_t gap_get_connection_type(
    hci_con_handle_t connection_handle) {
    return connection_handle <
                   sizeof(gap_connection_types) /
                       sizeof(gap_connection_types[0])
               ? gap_connection_types[connection_handle]
               : GAP_CONNECTION_INVALID;
}

uint16_t gap_le_connection_interval(hci_con_handle_t handle) {
    require(handle < 256, "interval read must target a valid fake connection");
    return negotiated_intervals[handle];
}

int gap_update_connection_parameters(hci_con_handle_t handle, uint16_t minimum,
                                    uint16_t maximum, uint16_t latency,
                                    uint16_t supervision_timeout) {
    require(handle < 256 && gap_get_connection_type(handle) == GAP_CONNECTION_LE,
            "interval policy must not update a Classic or invalid link");
    require(minimum == maximum && latency == 0 && supervision_timeout >= 100,
            "interval request must retain valid supervision and peripheral latency");
    ++interval_requests[handle];
    if (reject_interval_updates) return 0x0c;
    if (defer_interval_updates) pending_intervals[handle] = minimum;
    else negotiated_intervals[handle] = minimum;
    return ERROR_CODE_SUCCESS;
}

int gap_link_key_iterator_init(btstack_link_key_iterator_t* iterator) {
    iterator->index = 0;
    return 1;
}

int gap_link_key_iterator_get_next(
    btstack_link_key_iterator_t* iterator, bd_addr_t address,
    link_key_t link_key, link_key_type_t* type) {
    if (iterator->index >= classic_bond_count) {
        return 0;
    }
    memcpy(address, classic_bonds[iterator->index], sizeof(bd_addr_t));
    memset(link_key, iterator->index + 1, sizeof(link_key_t));
    *type = 0;
    ++iterator->index;
    return 1;
}

void gap_link_key_iterator_done(btstack_link_key_iterator_t*) {
}

int le_device_db_max_count() {
    return 4;
}

void le_device_db_info(
    int index, int* address_type, bd_addr_t address, sm_key_t irk) {
    if (index < ble_bond_count) {
        *address_type = ble_bond_types[index];
        memcpy(address, ble_bonds[index], sizeof(bd_addr_t));
        if (irk != nullptr) {
            memset(irk, index + 1, sizeof(sm_key_t));
        }
        return;
    }
    *address_type = BD_ADDR_TYPE_UNKNOWN;
}

void gap_set_bondable_mode(int enabled) {
    bondable = enabled != 0;
}

void gap_set_link_supervision_timeout(uint16_t timeout) {
    link_supervision_timeout = timeout;
}

void gap_ssp_set_auto_accept(int auto_accept) {
    ssp_auto_accept = auto_accept != 0;
}
void sm_set_accepted_stk_generation_methods(uint8_t methods) {
    accepted_stk_methods = methods;
}


int gap_ssp_confirmation_response(const bd_addr_t) {
    ++confirmation_accepts;
    return 0;
}

int gap_ssp_confirmation_negative(const bd_addr_t) {
    ++confirmation_rejections;
    return 0;
}

int gap_ssp_passkey_response(const bd_addr_t, uint32_t) {
    ++passkey_accepts;
    return 0;
}

int gap_ssp_passkey_negative(const bd_addr_t) {
    ++passkey_rejections;
    return 0;
}

void hci_add_event_handler(
    btstack_packet_callback_registration_t* callback_handler) {
    pairing_event_handler = callback_handler->callback;
}

void sm_add_event_handler(
    btstack_packet_callback_registration_t* callback_handler) {
    identity_event_handler = callback_handler->callback;
}

uint8_t hci_event_packet_get_type(const uint8_t* packet) {
    return packet[0];
}

void copy_event_address(const uint8_t* packet, bd_addr_t address) {
    for (size_t index = 0; index < sizeof(bd_addr_t); ++index) {
        address[index] = packet[7 - index];
    }
}

void hci_event_user_confirmation_request_get_bd_addr(
    const uint8_t* packet, bd_addr_t address) {
    copy_event_address(packet, address);
}

void hci_event_user_passkey_request_get_bd_addr(
    const uint8_t* packet, bd_addr_t address) {
    copy_event_address(packet, address);
}

hci_con_handle_t sm_event_handle(const uint8_t* packet) {
    return static_cast<hci_con_handle_t>(packet[2]) |
           static_cast<hci_con_handle_t>(packet[3] << 8);
}

void copy_sm_event_address(const uint8_t* packet, size_t offset,
                           bd_addr_t address) {
    for (size_t index = 0; index < sizeof(bd_addr_t); ++index) {
        address[index] = packet[offset + sizeof(bd_addr_t) - 1 - index];
    }
}

hci_con_handle_t sm_event_identity_resolving_started_get_handle(
    const uint8_t* packet) {
    return sm_event_handle(packet);
}

hci_con_handle_t sm_event_identity_resolving_failed_get_handle(
    const uint8_t* packet) {
    return sm_event_handle(packet);
}

hci_con_handle_t sm_event_identity_resolving_succeeded_get_handle(
    const uint8_t* packet) {
    return sm_event_handle(packet);
}

uint8_t sm_event_identity_resolving_succeeded_get_addr_type(
    const uint8_t* packet) {
    return packet[4];
}

void sm_event_identity_resolving_succeeded_get_address(
    const uint8_t* packet, bd_addr_t address) {
    copy_sm_event_address(packet, 5, address);
}

uint8_t sm_event_identity_resolving_succeeded_get_identity_addr_type(
    const uint8_t* packet) {
    return packet[11];
}

void sm_event_identity_resolving_succeeded_get_identity_address(
    const uint8_t* packet, bd_addr_t address) {
    copy_sm_event_address(packet, 12, address);
}

hci_con_handle_t sm_event_identity_created_get_handle(
    const uint8_t* packet) {
    return sm_event_handle(packet);
}

void sm_event_identity_created_get_address(
    const uint8_t* packet, bd_addr_t address) {
    copy_sm_event_address(packet, 5, address);
}

uint8_t sm_event_identity_created_get_identity_addr_type(
    const uint8_t* packet) {
    return packet[11];
}

void sm_event_identity_created_get_identity_address(
    const uint8_t* packet, bd_addr_t address) {
    copy_sm_event_address(packet, 12, address);
}

hci_con_handle_t sm_event_reencryption_started_get_handle(
    const uint8_t* packet) {
    return sm_event_handle(packet);
}

uint8_t sm_event_reencryption_started_get_addr_type(
    const uint8_t* packet) {
    return packet[4];
}

void sm_event_reencryption_started_get_address(
    const uint8_t* packet, bd_addr_t address) {
    copy_sm_event_address(packet, 5, address);
}

hci_con_handle_t sm_event_reencryption_complete_get_handle(
    const uint8_t* packet) {
    return sm_event_handle(packet);
}

uint8_t sm_event_reencryption_complete_get_addr_type(
    const uint8_t* packet) {
    return packet[4];
}

void sm_event_reencryption_complete_get_address(
    const uint8_t* packet, bd_addr_t address) {
    copy_sm_event_address(packet, 5, address);
}

uint8_t sm_event_reencryption_complete_get_status(
    const uint8_t* packet) {
    return packet[11];
}


void uni_platform_set_custom(uni_platform* platform) {
    installed_platform = platform;
}

int uni_init(int, const char**) {
    ++uni_init_calls;
    if (during_uni_init != nullptr) during_uni_init();
    return 0;
}

bool flash_safe_execute_core_init() {
    ++flash_core_init_calls;
    return flash_core_init_result;
}

int cyw43_arch_init() {
    ++cyw43_init_calls;
    return 0;
}

void cyw43_arch_gpio_put(int, bool enabled) {
    observed_status_led_on = enabled;
    ++observed_status_led_writes;
}

void multicore_launch_core1_with_stack(void (*)(), uint32_t* stack, size_t size) {
    require(stack != nullptr && size % 8 == 0,
            "core 1 launch needs an aligned bounded stack");
    ++core1_launch_calls;
}

void tight_loop_contents() {
    throw CoreStopped{};
}

uint32_t btstack_run_loop_get_time_ms() {
    return now_ms;
}

uint32_t time_us_32() {
    return now_ms * 1000u;
}


void switch2_wake_initialize() {
    ++switch2_wake_initializations;
}
bool switch2_wake_ready_for_connections() {
    return switch2_connections_ready;
}



bool switch2_wake_request() {
    ++switch2_wake_requests;
    return true;
}

void switch2_wake_diagnostics(Switch2WakeDiagnostics*) {
}

#include "core/controller_identity.cpp"
namespace {
unsigned state_lock_depth = 0;

void tracked_state_lock_enter(critical_section_t* lock) {
    critical_section_enter_blocking(lock);
    ++state_lock_depth;
}

void tracked_state_lock_exit(critical_section_t* lock) {
    require(state_lock_depth != 0, "state lock exit must match an enter");
    --state_lock_depth;
    critical_section_exit(lock);
}
}  // namespace

#define critical_section_enter_blocking tracked_state_lock_enter
#define critical_section_exit tracked_state_lock_exit
#include "input/bluepad32_input_backend.cpp"
#undef critical_section_enter_blocking
#undef critical_section_exit

#ifdef SWITCH_PICO_HAPTICS_EXPERIMENT
namespace {
std::vector<btstack_timer_source_t*> native_timers;
std::array<uint8_t, 143> last_native_packet{};
uint16_t last_native_cid = 0;
}

void native_test_add_timer(btstack_timer_source_t* timer) {
    btstack_run_loop_remove_timer(timer);
    timer->due_ms = uint64_t{now_ms} + timer->timeout_ms + 1;
    native_timers.push_back(timer);
}

int btstack_run_loop_remove_timer(btstack_timer_source_t* timer) {
    const auto found = std::find(native_timers.begin(), native_timers.end(), timer);
    if (found == native_timers.end()) return 0;
    native_timers.erase(found);
    return 1;
}

uint64_t time_us_64() { return uint64_t{now_ms} * 1000; }
uint16_t l2cap_get_remote_mtu_for_local_cid(uint16_t) { return 143; }
bool l2cap_can_send_packet_now(uint16_t) { return true; }
int hci_number_free_acl_slots_for_handle(uint16_t) { return 8; }
uint8_t l2cap_request_can_send_now_event(uint16_t cid) {
    for (const auto& slot : g_slots) {
        if (slot.device != nullptr && slot.device->conn.interrupt_cid == cid) {
            (void)uni_platform_on_l2cap_can_send_now(slot.device, cid);
            return ERROR_CODE_SUCCESS;
        }
    }
    require(false, "native permission targeted a detached connection");
    return 1;
}

uint8_t l2cap_send(uint16_t cid, const uint8_t* data, uint16_t size) {
    require(size == last_native_packet.size(), "native report size changed");
    std::copy(data, data + size, last_native_packet.begin());
    last_native_cid = cid;
    return ERROR_CODE_SUCCESS;
}

void haptics_transport_probe_prepare() {}
void haptics_transport_probe_begin(uint32_t, uint32_t, uint16_t) {}
void haptics_transport_probe_end() {}
void haptics_transport_probe_timer(uint32_t) {}
void haptics_transport_probe_permission(uint32_t) {}
void haptics_transport_probe_send(uint32_t, uint32_t, bool) {}
#endif

namespace {
uint8_t profile_flash[PROFILE_STORAGE_ARENA_COUNT][PROFILE_STORAGE_ARENA_SIZE]{};
ProfileStorage runtime_profile_storage;
bool runtime_profile_storage_initialized = false;
bool fail_profile_program = false;
unsigned profile_write_attempts = 0;
unsigned pair_observation_count = 0;
void (*before_pair_seed)(const ControllerIdentity&) = nullptr;

bool runtime_profile_read(void*, uint8_t arena, size_t offset,
                          uint8_t* output, size_t size) {
    require(state_lock_depth == 0, "profile reads must not hold the input state lock");
    if (arena >= PROFILE_STORAGE_ARENA_COUNT ||
        offset > PROFILE_STORAGE_ARENA_SIZE ||
        size > PROFILE_STORAGE_ARENA_SIZE - offset) {
        return false;
    }
    memcpy(output, &profile_flash[arena][offset], size);
    return true;
}

bool runtime_profile_erase(void*, uint8_t arena) {
    ++profile_write_attempts;
    require(state_lock_depth == 0, "profile erases must not hold the input state lock");
    if (arena >= PROFILE_STORAGE_ARENA_COUNT) return false;
    memset(profile_flash[arena], 0xff, PROFILE_STORAGE_ARENA_SIZE);
    return true;
}

bool runtime_profile_program(void*, uint8_t arena, size_t offset,
                             const uint8_t* page, size_t size) {
    ++profile_write_attempts;
    require(state_lock_depth == 0, "profile writes must not hold the input state lock");
    if (fail_profile_program || arena >= PROFILE_STORAGE_ARENA_COUNT ||
        size != PROFILE_STORAGE_PAGE_SIZE ||
        offset % PROFILE_STORAGE_PAGE_SIZE != 0 ||
        offset > PROFILE_STORAGE_ARENA_SIZE - size) {
        return false;
    }
    for (size_t index = 0; index < size; ++index) {
        profile_flash[arena][offset + index] &= page[index];
    }
    return memcmp(&profile_flash[arena][offset], page, size) == 0;
}

ProfileStorageIo runtime_profile_io() {
    return {nullptr, PROFILE_STORAGE_ARENA_SIZE,
            PROFILE_STORAGE_SECTOR_SIZE, PROFILE_STORAGE_PAGE_SIZE,
            runtime_profile_read, runtime_profile_erase, runtime_profile_program};
}

void initialize_runtime_profile_storage() {
    if (runtime_profile_storage_initialized) return;
    memset(profile_flash, 0xff, sizeof(profile_flash));
    require(runtime_profile_storage.initialize(runtime_profile_io()),
            "runtime profile catalog must initialize");
    runtime_profile_storage_initialized = true;
}

void require_clear_completion_pending() {
    if (expected_pending_clear_token != 0) {
        require(!bluepad32_input_backend_clear_pairings_completed(
                    g_pairing_snapshot, expected_pending_clear_token),
                "pairing clear token completed before Core1 work finished");
    }
}
void require_clear_snapshot_published() {
    if (expected_pending_clear_token != 0) {
        require(g_pairing_snapshot.status ==
                        Bluepad32PairingSnapshotStatus::kReady &&
                    g_pairing_snapshot.record_count == 0,
                "pairing clear policy ran before empty snapshot publication");
    }
}
void request_repeated_clear_during_disconnect() {
    if (repeat_clear_during_disconnect) {
        repeat_clear_during_disconnect = false;
        repeated_in_progress_clear_token =
            bluepad32_input_backend_clear_pairings();
    }
}
}  // namespace
ControllerIdentity observed_profile_identities[8]{};
size_t observed_profile_identity_count = 0;
AdapterConfiguration runtime_configuration{};
uint32_t runtime_configuration_generation = 1;
void configuration_service_prepare() {}
void configuration_service_initialize_on_storage_core() {}
void configuration_service_task_on_storage_core(uint32_t) {
    if (expect_configuration_timer_prearmed) {
        require(g_configuration_timer.add_count ==
                    expected_configuration_timer_add_count,
                "configuration work ran before its timer was rearmed");
    }
}
void profile_service_prepare() {}
void profile_service_initialize_on_storage_core() {}
void profile_service_task_on_storage_core(uint32_t) {
    if (expect_configuration_timer_prearmed) {
        require(g_configuration_timer.add_count ==
                    expected_configuration_timer_add_count,
                "profile work ran before its timer was rearmed");
    }
}
bool profile_service_observe_identity_on_storage_core(
    const ControllerIdentity& identity) {
    require(observed_profile_identity_count <
                sizeof(observed_profile_identities) /
                    sizeof(observed_profile_identities[0]),
            "profile identity observation fixture overflow");
    observed_profile_identities[observed_profile_identity_count++] =
        identity;
    if (runtime_profile_storage_initialized) {
        const ProfileStorageResult result = runtime_profile_storage.ensure_identity(identity);
        return result == ProfileStorageResult::kOk ||
               result == ProfileStorageResult::kUnchanged;
    }
    return true;
}
bool profile_service_observe_joycon_pair_on_storage_core(
    const ControllerIdentity& identity) {
    require(state_lock_depth == 0, "pair admission must release the input state lock");
    ++pair_observation_count;
    if (before_pair_seed != nullptr) before_pair_seed(identity);
    initialize_runtime_profile_storage();
    const ProfileStorageResult result = runtime_profile_storage.ensure_joycon_pair(identity);
    return result == ProfileStorageResult::kOk ||
           result == ProfileStorageResult::kUnchanged;
}
void configuration_service_snapshot(ConfigurationServiceSnapshot* output) {
    *output = {};
    output->state = ConfigurationServiceState::kReady;
    output->configuration = runtime_configuration;
    output->generation = runtime_configuration_generation;
}
uint32_t configuration_service_reset_generation() {
    return 0;
}
uint32_t profile_service_database_generation() {
    return runtime_profile_storage.snapshot().generation;
}
void profile_service_active_profile_snapshot(
    const ControllerIdentity& identity, ProfileServiceActiveProfileSnapshot* output) {
    *output = {};
    output->metadata.state = ProfileServiceState::kReady;
    output->metadata.generation = profile_service_database_generation();
    const auto* owner = runtime_profile_storage.find(identity);
    if (owner == nullptr) return;
    output->profile_index = owner->active_profile;
    output->valid = runtime_profile_storage.get(
        identity, owner->active_profile, &output->profile) == ProfileStorageResult::kOk;
}
ConfigurationTransactionStatus profile_service_activate_internal(
    uint32_t, const ControllerIdentity& identity, uint8_t profile_index) {
    const auto result = runtime_profile_storage.activate(identity, profile_index);
    return result == ProfileStorageResult::kOk || result == ProfileStorageResult::kUnchanged
               ? ConfigurationTransactionStatus::kCommitted
               : ConfigurationTransactionStatus::kStorageError;
}
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
AdapterUsbMode test_adapter_mode = AdapterUsbMode::kXInput;
AdapterUsbMode adapter_host_probe_mode() {
    return test_adapter_mode;
}
#endif

SwitchRgbColor switch_pro_get_slot_light_color(uint8_t instance) {
    static constexpr SwitchRgbColor grips[] = {
        {SWITCH_COLOR_SLOT_1_R, SWITCH_COLOR_SLOT_1_G,
         SWITCH_COLOR_SLOT_1_B},
        {SWITCH_COLOR_SLOT_2_R, SWITCH_COLOR_SLOT_2_G,
         SWITCH_COLOR_SLOT_2_B},
        {SWITCH_COLOR_SLOT_3_R, SWITCH_COLOR_SLOT_3_G,
         SWITCH_COLOR_SLOT_3_B},
        {SWITCH_COLOR_SLOT_4_R, SWITCH_COLOR_SLOT_4_G,
         SWITCH_COLOR_SLOT_4_B},
    };
    return instance < sizeof(grips) / sizeof(grips[0])
               ? switch_pro_calibrate_light_color(grips[instance])
               : SwitchRgbColor{};
}

namespace {

bool read_controller_state(uint8_t slot, ControllerState* output) {
    Bluepad32SlotSnapshot snapshot{};
    bluepad32_input_backend_snapshot(slot, &snapshot);
    if (output != nullptr) {
        *output = snapshot.state;
    }
    return snapshot.active;
}

void start_backend() {
    bluepad32_input_backend_init();
    platform_on_init_complete();
    require(incoming_connections && scanning_enabled &&
                classic_scanning_enabled &&
                link_supervision_timeout ==
                    kClassicLinkSupervisionTimeout &&
                !bondable && accepted_stk_methods == 0 &&
                !ssp_auto_accept && pairing_event_handler != nullptr &&
                identity_event_handler != nullptr &&
                switch2_wake_initializations == 1,
            "initialization must register Classic and BLE identity policy");
}
void start_pairing_backend() {
    start_backend();
    bluepad32_input_backend_open_pairing_window();
    process_rumble_timer(&g_rumble_timer);
    require(g_connection_policy_state == ConnectionPolicyState::Open &&
                scanning_enabled && classic_scanning_enabled &&
                incoming_connections,
            "test connection setup requires an open pairing window");
}
void dispatch_pairing_event(uint8_t event_type) {
    uint8_t packet[8] = {event_type, 6, 1, 2, 3, 4, 5, 6};
    pairing_event_handler(HCI_EVENT_PACKET, 0, packet, sizeof(packet));
}

void write_event_address(uint8_t* packet, size_t offset,
                         const bd_addr_t address) {
    for (size_t index = 0; index < sizeof(bd_addr_t); ++index) {
        packet[offset + index] =
            address[sizeof(bd_addr_t) - 1 - index];
    }
}

void dispatch_identity_event(uint8_t event_type,
                             const uni_hid_device_t& controller,
                             uint8_t identity_address_type,
                             const bd_addr_t identity_address,
                             uint8_t status = ERROR_CODE_SUCCESS) {
    uint8_t packet[20]{};
    size_t packet_size = 0;
    packet[0] = event_type;
    packet[2] = static_cast<uint8_t>(controller.conn.handle);
    packet[3] = static_cast<uint8_t>(controller.conn.handle >> 8);
    switch (event_type) {
        case SM_EVENT_IDENTITY_RESOLVING_STARTED:
        case SM_EVENT_IDENTITY_RESOLVING_FAILED:
            packet_size = 11;
            break;
        case SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED:
            packet_size = sizeof(packet);
            packet[4] = BD_ADDR_TYPE_LE_RANDOM;
            write_event_address(packet, 5, controller.conn.btaddr);
            packet[11] = identity_address_type;
            write_event_address(packet, 12, identity_address);
            break;
        case SM_EVENT_IDENTITY_CREATED:
            packet_size = sizeof(packet);
            packet[4] = identity_address_type;
            write_event_address(packet, 5, identity_address);
            packet[11] = identity_address_type;
            write_event_address(packet, 12, identity_address);
            break;
        case SM_EVENT_REENCRYPTION_STARTED:
            packet_size = 11;
            packet[4] = identity_address_type;
            write_event_address(packet, 5, identity_address);
            break;
        case SM_EVENT_REENCRYPTION_COMPLETE:
            packet_size = 12;
            packet[4] = identity_address_type;
            write_event_address(packet, 5, identity_address);
            packet[11] = status;
            break;
        default:
            require(false, "unsupported identity event fixture");
    }
    packet[1] = static_cast<uint8_t>(packet_size - 2);
    identity_event_handler(
        HCI_EVENT_PACKET, 0, packet,
        static_cast<uint16_t>(packet_size));
}

void require_identity(const ControllerIdentity& actual, bool stable,
                      ControllerTransport transport, uint8_t address_type,
                      const bd_addr_t address, uint16_t vendor_id,
                      uint16_t product_id, const char* message) {
    ControllerIdentity expected{};
    expected.stable = stable;
    expected.transport = transport;
    expected.address_type = address_type;
    memcpy(expected.address, address, sizeof(expected.address));
    expected.vendor_id = vendor_id;
    expected.product_id = product_id;
    require(controller_identity_equal(actual, expected), message);
}

void test_identity_encoding_contract() {
    ControllerIdentity identity{};
    identity.stable = true;
    identity.transport = ControllerTransport::kBle;
    identity.address_type = BD_ADDR_TYPE_LE_RANDOM_IDENTITY;
    const bd_addr_t address = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15};
    memcpy(identity.address, address, sizeof(address));
    identity.vendor_id = 0x1234;
    identity.product_id = 0xabcd;

    uint8_t encoded[CONTROLLER_IDENTITY_ENCODED_SIZE]{};
    const uint8_t expected[CONTROLLER_IDENTITY_ENCODED_SIZE] = {
        1, 2, 3, 0, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
        0x34, 0x12, 0xcd, 0xab};
    require(controller_identity_encode(identity, encoded, sizeof(encoded)) &&
                memcmp(encoded, expected, sizeof(expected)) == 0,
            "controller identity wire encoding changed");

    ControllerIdentity decoded{};
    require(controller_identity_decode(encoded, sizeof(encoded), &decoded) &&
                controller_identity_equal(identity, decoded),
            "controller identity wire round trip failed");
    decoded.product_id ^= 1;
    require(!controller_identity_equal(identity, decoded),
            "controller identity equality must include every field");
    encoded[3] = 1;
    require(!controller_identity_decode(encoded, sizeof(encoded), &decoded),
            "controller identity decoder must reject a nonzero reserved byte");

    const ControllerIdentity global = controller_identity_global();
    memset(encoded, 0xff, sizeof(encoded));
    require(controller_identity_is_global(global) &&
                controller_identity_encode(global, encoded,
                                           sizeof(encoded)),
            "global identity helper must produce an encodable fallback");
    for (uint8_t byte : encoded) {
        require(byte == 0,
                "global fallback identity must encode as all zeroes");
    }
}




void tick_backend_timer(int ticks) {
    for (int tick = 0; tick < ticks; ++tick) {
        process_rumble_timer(&g_rumble_timer);
    }
}

uni_hid_device_t switch2_device(int index, uint16_t product) {
    uni_hid_device_t result = device(index, true, UNI_BT_CONN_PROTOCOL_BLE);
    result.vendor_id = UNI_SW2_NINTENDO_VID;
    result.product_id = product;
    result.switch2_identity_valid = true;
    result.switch2_identity_address_type = BD_ADDR_TYPE_LE_PUBLIC;
    result.report_parser.set_player_leds = set_player_leds;
    return result;
}

void ready_switch2(uni_hid_device_t& controller) {
    platform_on_device_connected(&controller);
    require(platform_on_device_ready(&controller) == UNI_ERROR_SUCCESS,
            "Switch2 physical device must become ready");
}

void remember_switch2(const uni_hid_device_t& controller) {
    require(switch2_pairing_count < UNI_SWITCH2_PAIRING_CAPACITY,
            "test proprietary trust inventory overflow");
    switch2_pairing_types[switch2_pairing_count] = controller.switch2_identity_address_type;
    memcpy(switch2_pairings[switch2_pairing_count], controller.conn.btaddr, sizeof(bd_addr_t));
    ++switch2_pairing_count;
}

Bluepad32SlotSnapshot slot_snapshot(uint8_t index) {
    Bluepad32SlotSnapshot result{};
    bluepad32_input_backend_snapshot(index, &result);
    return result;
}

void require_pair_owner(const ControllerIdentity& identity,
                        const uni_hid_device_t& left,
                        const uni_hid_device_t& right) {
    ControllerIdentity left_member{};
    ControllerIdentity right_member{};
    require(controller_identity_is_joycon_pair(identity) &&
                controller_identity_joycon_pair_members(
                    identity, &left_member, &right_member) &&
                controller_identity_equal(left_member, identity_for_device(&left)) &&
                controller_identity_equal(right_member, identity_for_device(&right)) &&
                !controller_identity_equal(identity, left_member) &&
                !controller_identity_equal(identity, right_member),
            "logical owner must distinguish both typed physical members from their solo owners");
}

void require_active_profile(const ControllerIdentity& identity,
                            uint8_t expected_index, uint8_t expected_scale) {
    const ProfileStorageIdentityIndex* owner = runtime_profile_storage.find(identity);
    ControllerProfile profile{};
    require(owner != nullptr && owner->active_profile == expected_index &&
                runtime_profile_storage.get(identity, owner->active_profile, &profile) ==
                    ProfileStorageResult::kOk &&
                profile.weak_rumble_scale == expected_scale,
            "live owner must resolve its independent persisted active profile");
}

void set_runtime_joycon_mode(JoyConMode mode) {
    runtime_configuration.joycon_mode = mode;
    ++runtime_configuration_generation;
    process_configuration_timer(&g_configuration_timer);
    ConfigurationServiceSnapshot saved{};
    configuration_service_snapshot(&saved);
    require(saved.configuration.joycon_mode == mode &&
                saved.generation == runtime_configuration_generation &&
                device_disconnect_calls == 0 && uni_init_calls == 0 &&
                core1_launch_calls == 0,
            "live mode updates must retain the committed preference without disconnect or restart");
}
void test_switch2_individual_core_start() {
    runtime_configuration.joycon_mode = JoyConMode::kIndividual;
    bluepad32_input_backend_init();
    auto right = switch2_device(0, UNI_SW2_JOYCON_R_PID);
    auto left = switch2_device(1, UNI_SW2_JOYCON_L_PID);
    register_lookup_device(&right);
    register_lookup_device(&left);
    during_uni_init = []() {
        require(installed_platform != nullptr &&
                    installed_platform->on_device_ready(lookup_devices[0]) == UNI_ERROR_SUCCESS &&
                    installed_platform->on_device_ready(lookup_devices[1]) == UNI_ERROR_SUCCESS &&
                    slot_snapshot(0).active && slot_snapshot(1).active &&
                    pair_observation_count == 0,
                "persisted Individual must govern ready callbacks before BTstack initialization completes");
    };
    bool stopped = false;
    try {
        core1_main();
    } catch (const CoreStopped&) {
        stopped = true;
    }
    during_uni_init = nullptr;
    require(stopped && uni_init_calls == 1 && slot_snapshot(0).active && slot_snapshot(1).active,
            "Core1 initialization must retain both solo players without an initial pair race");
}


void test_switch2_individual_boot(bool right_first) {
    runtime_configuration.joycon_mode = JoyConMode::kIndividual;
    start_backend();
    auto left = switch2_device(right_first ? 1 : 0, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(right_first ? 0 : 1, UNI_SW2_JOYCON_R_PID);
    auto& first = right_first ? right : left;
    auto& second = right_first ? left : right;
    remember_switch2(left);
    remember_switch2(right);
    register_lookup_device(&left);
    register_lookup_device(&right);
    ready_switch2(first);
    require(scanning_enabled && background_scan_parameters &&
                !classic_scanning_enabled && !bondable &&
                accepted_stk_methods == 0 && !switch_pico_switch2_pairing_allowed() &&
                platform_on_device_discovered(second.conn.btaddr, nullptr, 0, 0) ==
                    UNI_ERROR_SUCCESS,
            "Individual must still admit a remembered opposite half without BOOTSEL or authentication");
    const auto first_solo = slot_snapshot(0);
    platform_on_device_connected(&second);
    require(!scanning_enabled &&
                platform_on_device_ready(&second) == UNI_ERROR_SUCCESS,
            "remembered Individual mate setup must pause scanning and become ready");
    process_configuration_timer(&g_configuration_timer);
    require(slot_snapshot(0).active && slot_snapshot(1).active &&
                slot_snapshot(0).connection_generation == first_solo.connection_generation &&
                !scanning_enabled && !classic_scanning_enabled && pair_observation_count == 0,
            "persisted Individual must avoid transient boot merging and stop scanning once balanced");
    require(controller_identity_equal(slot_snapshot(left.idx).identity, identity_for_device(&left)) &&
                controller_identity_equal(slot_snapshot(right.idx).identity, identity_for_device(&right)),
            "both Individual players must publish their own solo profile identity");
    uni_controller_t data{};
    data.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    data.gamepad.dpad = DPAD_LEFT;
    data.gamepad.axis_x = -512;
    data.gamepad.axis_y = 100;
    left.switch2_extra_buttons = UNI_SW2_BUTTON_LEFT_SL;
    platform_on_controller_data(&left, &data);
    const auto left_state = slot_snapshot(left.idx).state;
    data.gamepad = {};
    data.gamepad.buttons = BUTTON_B | BUTTON_THUMB_R;
    data.gamepad.axis_rx = 200;
    data.gamepad.axis_ry = -512;
    right.switch2_extra_buttons = UNI_SW2_BUTTON_RIGHT_SR;
    platform_on_controller_data(&right, &data);
    const auto right_state = slot_snapshot(right.idx).state;
    require(left_state.button_south && !left_state.dpad_left &&
                left_state.button_left_shoulder && !left_state.button_right_shoulder &&
                left_state.left_stick_x == scale_axis(100) &&
                left_state.left_stick_y == INT16_MAX &&
                right_state.button_south && right_state.button_left_stick &&
                !right_state.button_right_stick && right_state.button_right_shoulder &&
                !right_state.button_left_shoulder &&
                right_state.left_stick_x == INT16_MAX &&
                right_state.left_stick_y == scale_axis(200) &&
                slot_snapshot(left.idx).state.left_stick_x == left_state.left_stick_x,
            "two Individual halves must keep independent sideways input and rail routing");
    Bluepad32PlaytestSnapshot playtest{};
    bluepad32_input_backend_playtest_snapshot(left.idx, &playtest);
    require(playtest.controller_layout == Bluepad32ControllerLayout::kJoyCon2LeftSolo,
            "Individual left must report actual solo topology");
    bluepad32_input_backend_playtest_snapshot(right.idx, &playtest);
    require(playtest.controller_layout == Bluepad32ControllerLayout::kJoyCon2RightSolo,
            "Individual right must report actual solo topology");
    bluepad32_input_backend_queue_rumble(left.idx, ControllerRumbleOutput{17, 27});
    process_rumble_timer(&g_rumble_timer);
    require(left.last_low == 17 && right.last_low == 0,
            "Individual player rumble must not fan out to its opposite half");
    platform_on_device_disconnected(&second);
    require(scanning_enabled && !classic_scanning_enabled &&
                platform_on_device_discovered(second.conn.btaddr, nullptr, 0, 0) ==
                    UNI_ERROR_SUCCESS,
            "losing an Individual half must resume only remembered opposite-half scanning");
    ready_switch2(second);
    require(!scanning_enabled && !classic_scanning_enabled &&
                slot_snapshot(0).active && slot_snapshot(1).active,
            "remembered Individual reconnection must stop scanning again without merging");
}

void test_switch2_mode_roundtrip(bool right_first) {
    start_backend();
    initialize_runtime_profile_storage();
    auto left = switch2_device(right_first ? 1 : 0, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(right_first ? 0 : 1, UNI_SW2_JOYCON_R_PID);
    const auto left_identity = identity_for_device(&left);
    const auto right_identity = identity_for_device(&right);
    auto left_profile = controller_profile_default(left_identity, 2);
    left_profile.weak_rumble_scale = 37;
    auto right_profile = controller_profile_default(right_identity, 5);
    right_profile.weak_rumble_scale = 63;
    require(runtime_profile_storage.set(left_identity, 2, left_profile) == ProfileStorageResult::kOk &&
                runtime_profile_storage.activate(left_identity, 2) == ProfileStorageResult::kOk &&
                runtime_profile_storage.set(right_identity, 5, right_profile) == ProfileStorageResult::kOk &&
                runtime_profile_storage.activate(right_identity, 5) == ProfileStorageResult::kOk,
            "roundtrip requires independently selected saved solo banks");
    ready_switch2(right_first ? right : left);
    ready_switch2(right_first ? left : right);
    const auto pair_identity = slot_snapshot(0).identity;
    auto pair_profile = controller_profile_default(pair_identity, 7);
    pair_profile.weak_rumble_scale = 95;
    require(runtime_profile_storage.set(pair_identity, 7, pair_profile) == ProfileStorageResult::kOk &&
                runtime_profile_storage.activate(pair_identity, 7) == ProfileStorageResult::kOk,
            "roundtrip requires a distinct saved composite-bank selection");
    auto classic = device(2, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    platform_on_device_connected(&classic);
    require(platform_on_device_ready(&classic) == UNI_ERROR_SUCCESS,
            "Classic must coexist with either Joy-Con player mode");
    uni_controller_t data{};
    data.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    data.gamepad.buttons = BUTTON_Y;
    platform_on_controller_data(&classic, &data);
    data.gamepad = {};
    data.gamepad.dpad = DPAD_UP;
    data.gamepad.axis_x = -512;
    data.gamepad.axis_y = 100;
    left.switch2_extra_buttons = UNI_SW2_BUTTON_GL | UNI_SW2_BUTTON_LEFT_SL;
    platform_on_controller_data(&left, &data);
    data.gamepad = {};
    data.gamepad.buttons = BUTTON_B;
    data.gamepad.axis_rx = 200;
    data.gamepad.axis_ry = -512;
    right.switch2_extra_buttons = UNI_SW2_BUTTON_C | UNI_SW2_BUTTON_RIGHT_SR;
    platform_on_controller_data(&right, &data);
    bluepad32_input_backend_queue_rumble(2, ControllerRumbleOutput{71, 81});
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{31, 41});
    process_rumble_timer(&g_rumble_timer);
    const auto pair_before = slot_snapshot(0);
    const auto spare_before = slot_snapshot(1);
    const auto classic_before = slot_snapshot(2);
    const int classic_calls = classic.rumble_calls;
    const unsigned left_resets = left.switch2_haptics_resets;
    const unsigned right_resets = right.switch2_haptics_resets;
    require(bluepad32_input_backend_capture_start(
                0, pair_before.connection_generation, CaptureOptions{}),
            "pair capture must start before splitting");
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{91, 101});
    bluepad32_input_backend_queue_profile_feedback(
        0, pair_before.connection_generation, 8, ControllerProfileConfirmationPolicy::kRumbleAndLed);
    set_runtime_joycon_mode(JoyConMode::kIndividual);
    const auto left_solo = slot_snapshot(0);
    const auto right_solo = slot_snapshot(1);
    require(left_solo.active && right_solo.active &&
                left_solo.connection_generation != pair_before.connection_generation &&
                right_solo.connection_generation != spare_before.connection_generation &&
                controller_identity_equal(left_solo.identity, left_identity) &&
                controller_identity_equal(right_solo.identity, right_identity) &&
                left_solo.state.button_west && !left_solo.state.dpad_up &&
                left_solo.state.left_stick_x == scale_axis(100) &&
                left_solo.state.left_stick_y == INT16_MAX &&
                right_solo.state.button_south && !right_solo.state.button_east &&
                right_solo.state.left_stick_x == INT16_MAX &&
                right_solo.state.left_stick_y == scale_axis(200) &&
                left_solo.state.motion_sample_count == 0 && right_solo.state.motion_sample_count == 0,
            "split must retain both raw halves and immediately restore rotated solo owners at stable slots");
    require_active_profile(left_solo.identity, 2, 37);
    require_active_profile(right_solo.identity, 5, 63);
    Bluepad32CaptureSnapshot capture{};
    require(bluepad32_input_backend_capture_page(0, 0, &capture) &&
                capture.state == CaptureState::kDisconnected &&
                left.switch2_haptics_resets > left_resets &&
                right.switch2_haptics_resets > right_resets &&
                left.last_rumble_duration_ms == 0 && right.last_rumble_duration_ms == 0,
            "split must cancel recording and both physical haptics epochs");
    const int stops = left.rumble_calls + right.rumble_calls;
    bluepad32_input_backend_queue_profile_feedback(
        0, pair_before.connection_generation, 8, ControllerProfileConfirmationPolicy::kRumbleAndLed);
    process_rumble_timer(&g_rumble_timer);
    require(left.rumble_calls + right.rumble_calls == stops &&
                left.last_low == 0 && right.last_low == 0 &&
                left.player_leds == 1 && right.player_leds == 2 &&
                !bluepad32_input_backend_identify(pair_identity),
            "retired pair feedback and identify must not leak into Individual players");
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{17, 27});
    bluepad32_input_backend_queue_rumble(1, ControllerRumbleOutput{47, 57});
    process_rumble_timer(&g_rumble_timer);
    require(left.last_low == 17 && right.last_low == 47,
            "split outputs must route independent player haptics");
    const uint8_t capture_slot = right_first ? 1 : 0;
    require(bluepad32_input_backend_capture_start(
                capture_slot, slot_snapshot(capture_slot).connection_generation, CaptureOptions{}),
            "either Individual slot must remain recordable before remerging");
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{91, 101});
    bluepad32_input_backend_queue_rumble(1, ControllerRumbleOutput{111, 121});
    bluepad32_input_backend_queue_profile_feedback(
        1, right_solo.connection_generation, 8, ControllerProfileConfirmationPolicy::kRumbleAndLed);
    before_pair_seed = [](const ControllerIdentity&) {
        require(slot_snapshot(0).active && slot_snapshot(1).active &&
                    !controller_identity_is_joycon_pair(slot_snapshot(0).identity) &&
                    !controller_identity_is_joycon_pair(slot_snapshot(1).identity),
                "live pair storage admission must precede publication of either topology change");
    };
    set_runtime_joycon_mode(JoyConMode::kPaired);
    before_pair_seed = nullptr;
    const auto restored = slot_snapshot(0);
    require_pair_owner(restored.identity, left, right);
    require(restored.connection_generation != left_solo.connection_generation &&
                !slot_snapshot(1).active &&
                slot_snapshot(1).connection_generation != right_solo.connection_generation &&
                !slot_snapshot(1).state.button_south &&
                slot_snapshot(1).state.extra_buttons == 0 &&
                restored.state.dpad_up && restored.state.button_east &&
                restored.state.left_stick_x == INT16_MIN &&
                restored.state.right_stick_x == scale_axis(200) &&
                restored.state.motion_sample_count == 0 &&
                bluepad32_input_backend_capture_page(0, 0, &capture) &&
                capture.state == CaptureState::kDisconnected &&
                left.last_rumble_duration_ms == 0 && right.last_rumble_duration_ms == 0,
            "remerge must retain raw pair inputs, cancel either recording, and neutralize the retired solo");
    require_active_profile(restored.identity, 7, 95);
    require_active_profile(left_identity, 2, 37);
    require_active_profile(right_identity, 5, 63);
    process_rumble_timer(&g_rumble_timer);
    require(left.last_low == 0 && right.last_low == 0 &&
                slot_snapshot(2).connection_generation == classic_before.connection_generation &&
                slot_snapshot(2).state.button_north &&
                classic.rumble_calls == classic_calls && classic.last_low == 71 &&
                negotiated_intervals[left.conn.handle] == 24 &&
                negotiated_intervals[right.conn.handle] == 24 &&
                interval_requests[left.conn.handle] == 1 && interval_requests[right.conn.handle] == 1,
            "mode roundtrip must preserve unrelated Classic state, haptics and physical mixed-link intervals");
    ControllerProfile inactive{};
    require(runtime_profile_storage.get(pair_identity, 2, &inactive) == ProfileStorageResult::kOk &&
                inactive.weak_rumble_scale == 37,
            "existing composite profiles, including inactive seeded rows, must survive mode roundtrips");
}

void test_switch2_mode_two_pairs() {
    start_pairing_backend();
    auto right0 = switch2_device(0, UNI_SW2_JOYCON_R_PID);
    auto right1 = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    auto left0 = switch2_device(2, UNI_SW2_JOYCON_L_PID);
    auto left1 = switch2_device(3, UNI_SW2_JOYCON_L_PID);
    ready_switch2(right0);
    ready_switch2(right1);
    ready_switch2(left0);
    ready_switch2(left1);
    const auto pair0 = slot_snapshot(0).identity;
    const auto pair1 = slot_snapshot(1).identity;
    require(runtime_profile_storage.activate(pair0, 6) == ProfileStorageResult::kOk &&
                runtime_profile_storage.activate(pair1, 3) == ProfileStorageResult::kOk,
            "two pairs need distinguishable saved selections");
    for (unsigned roundtrip = 0; roundtrip < 2; ++roundtrip) {
        set_runtime_joycon_mode(JoyConMode::kIndividual);
        for (uint8_t slot = 0; slot < 4; ++slot) {
            require(slot_snapshot(slot).active, "splitting two pairs must expose all four live players");
        }
        require(controller_identity_equal(slot_snapshot(0).identity, identity_for_device(&left0)) &&
                    controller_identity_equal(slot_snapshot(1).identity, identity_for_device(&left1)) &&
                    controller_identity_equal(slot_snapshot(2).identity, identity_for_device(&right0)) &&
                    controller_identity_equal(slot_snapshot(3).identity, identity_for_device(&right1)) &&
                    !scanning_enabled && !incoming_connections,
                "full-capacity split must choose lowest free slots without moving existing left owners");
        set_runtime_joycon_mode(JoyConMode::kPaired);
        require_pair_owner(slot_snapshot(0).identity, left0, right0);
        require_pair_owner(slot_snapshot(1).identity, left1, right1);
        require(!slot_snapshot(2).active && !slot_snapshot(3).active &&
                    !scanning_enabled && !incoming_connections,
                "live pairing hints must restore both exact member combinations and original output slots");
        require_active_profile(pair0, 6, UINT8_MAX);
        require_active_profile(pair1, 3, UINT8_MAX);
        bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{11, 21});
        bluepad32_input_backend_queue_rumble(1, ControllerRumbleOutput{31, 41});
        process_rumble_timer(&g_rumble_timer);
        require(left0.last_low == 11 && right0.last_low == 11 &&
                    left1.last_low == 31 && right1.last_low == 31,
                "roundtripped pair membership must remain visible in physical haptic routing");
    }
}

void test_switch2_live_pair_failure(bool invalid_identity) {
    runtime_configuration.joycon_mode = JoyConMode::kIndividual;
    start_backend();
    initialize_runtime_profile_storage();
    auto left = switch2_device(0, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    ready_switch2(left);
    ready_switch2(right);
    uni_controller_t data{};
    data.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    data.gamepad.dpad = DPAD_LEFT;
    platform_on_controller_data(&left, &data);
    data.gamepad = {};
    data.gamepad.buttons = BUTTON_B;
    platform_on_controller_data(&right, &data);
    const auto left_before = slot_snapshot(0);
    const auto right_before = slot_snapshot(1);
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{17, 27});
    bluepad32_input_backend_queue_rumble(1, ControllerRumbleOutput{47, 57});
    require(bluepad32_input_backend_capture_start(
                1, right_before.connection_generation, CaptureOptions{}),
            "active solo capture must survive a rejected live merge");
    before_pair_seed = [](const ControllerIdentity&) {
        require(slot_snapshot(0).active && slot_snapshot(1).active &&
                    slot_snapshot(0).state.button_south && slot_snapshot(1).state.button_south,
                "both active solos must stay intact while a new composite bank is seeded");
    };
    if (invalid_identity) right.switch2_identity_valid = false;
    else fail_profile_program = true;
    set_runtime_joycon_mode(JoyConMode::kPaired);
    before_pair_seed = nullptr;
    Bluepad32CaptureSnapshot capture{};
    require(slot_snapshot(0).active && slot_snapshot(1).active &&
                slot_snapshot(0).connection_generation == left_before.connection_generation &&
                slot_snapshot(1).connection_generation == right_before.connection_generation &&
                controller_identity_equal(slot_snapshot(0).identity, left_before.identity) &&
                controller_identity_equal(slot_snapshot(1).identity, right_before.identity) &&
                slot_snapshot(0).state.button_south && slot_snapshot(1).state.button_south &&
                bluepad32_input_backend_capture_page(0, 0, &capture) &&
                capture.state == CaptureState::kRecording &&
                left.switch2_haptics_resets == 0 && right.switch2_haptics_resets == 0,
            "failed live pair admission must preserve both solo identities, states, generations and recording");
    const unsigned expected_observations = invalid_identity ? 0 : 1;
    require(pair_observation_count == expected_observations,
            "invalid pair identity must fail before I/O and failed storage admission must be attempted once");
    process_rumble_timer(&g_rumble_timer);
    require(left.last_low == 17 && right.last_low == 47,
            "failed admission must preserve both already-queued solo haptic commands");
    for (unsigned tick = 0; tick < 100; ++tick) {
        now_ms += 50;
        process_configuration_timer(&g_configuration_timer);
    }
    ++runtime_configuration.pairing_window_seconds;
    ++runtime_configuration_generation;
    process_configuration_timer(&g_configuration_timer);
    require(pair_observation_count == expected_observations,
            "pending Paired preference must not create timer or unrelated-configuration retry storms");
    fail_profile_program = false;
    right.switch2_identity_valid = true;
    if (!invalid_identity) {
        require(runtime_profile_storage.initialize(runtime_profile_io()),
                "pair failure recovery must replay intact solo profile banks");
    }
    set_runtime_joycon_mode(JoyConMode::kIndividual);
    set_runtime_joycon_mode(JoyConMode::kPaired);
    require_pair_owner(slot_snapshot(0).identity, left, right);
    require(!slot_snapshot(1).active, "explicit mode retry must recover without Bluetooth reconnect");
}

int live_identity_slot(const ControllerIdentity& identity) {
    for (uint8_t index = 0; index < BLUEPAD32_INPUT_BACKEND_SLOT_COUNT; ++index) {
        const auto snapshot = slot_snapshot(index);
        if (snapshot.active && controller_identity_equal(snapshot.identity, identity)) return index;
    }
    return -1;
}

int live_pair_slot(const uni_hid_device_t& left, const uni_hid_device_t& right) {
    ControllerIdentity pair{};
    require(controller_identity_make_joycon_pair(
                identity_for_device(&left), identity_for_device(&right), &pair),
            "gesture participants must have a valid typed composite identity");
    return live_identity_slot(pair);
}

void require_live_solo(const uni_hid_device_t& controller, const char* message) {
    require(live_identity_slot(identity_for_device(&controller)) >= 0, message);
}

uni_controller_t gesture_input(bool right, bool trigger = true, bool menu = true) {
    uni_controller_t report{};
    report.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    if (right) {
        report.gamepad.buttons = BUTTON_B | (trigger ? BUTTON_TRIGGER_R : 0);
        report.gamepad.misc_buttons = menu ? MISC_BUTTON_START : 0;
        report.gamepad.throttle = trigger ? 1023 : 0;
        report.gamepad.axis_rx = 200;
        report.gamepad.axis_ry = -512;
    } else {
        report.gamepad.buttons = trigger ? BUTTON_TRIGGER_L : 0;
        report.gamepad.misc_buttons = menu ? MISC_BUTTON_SELECT : 0;
        report.gamepad.brake = trigger ? 1023 : 0;
        report.gamepad.dpad = DPAD_UP;
        report.gamepad.axis_x = -512;
        report.gamepad.axis_y = 100;
    }
    return report;
}

bool gesture_profile_consumer_enabled = false;
ControllerState gesture_profile_outputs[BLUEPAD32_INPUT_BACKEND_SLOT_COUNT]{};

void consume_gesture_profiles() {
    if (!gesture_profile_consumer_enabled) return;
    for (uint8_t index = 0; index < BLUEPAD32_INPUT_BACKEND_SLOT_COUNT; ++index) {
        gesture_profile_outputs[index] = controller_profile_runtime_transform(
            index, slot_snapshot(index), now_ms, AdapterUsbMode::kXInput).state;
    }
}

void gesture_report(uni_hid_device_t& controller, bool trigger = true, bool menu = true) {
    auto report = gesture_input(controller.product_id == UNI_SW2_JOYCON_R_PID, trigger, menu);
    platform_on_controller_data(&controller, &report);
    consume_gesture_profiles();
}

void gesture_tick() {
    process_rumble_timer(&g_rumble_timer);
    process_configuration_timer(&g_configuration_timer);
    consume_gesture_profiles();
}

void gesture_reports(uni_hid_device_t& left, uni_hid_device_t& right) {
    gesture_report(left);
    gesture_report(right);
    gesture_tick();
}

void hold_gesture(uni_hid_device_t& left, uni_hid_device_t& right, uint32_t duration_ms) {
    gesture_reports(left, right);
    for (uint32_t elapsed = 0; elapsed < duration_ms;) {
        const uint32_t step = duration_ms - elapsed < 50 ? duration_ms - elapsed : 50;
        elapsed += step;
        now_ms += step;
        gesture_reports(left, right);
    }
}

void release_gesture(uni_hid_device_t& left, uni_hid_device_t& right) {
    ++now_ms;
    gesture_report(left, false, false);
    gesture_report(right, false, false);
    gesture_tick();
}

void require_gesture_masked(const Bluepad32SlotSnapshot& snapshot) {
    const uint16_t menus =
        logical_button_bit(ControllerProfileLogicalButton::kSelect) |
        logical_button_bit(ControllerProfileLogicalButton::kStart);
    require(!snapshot.state.button_select && !snapshot.state.button_start &&
                snapshot.state.left_trigger == 0 && snapshot.state.right_trigger == 0 &&
                (snapshot.pre_hotkey_button_mask & menus) == 0,
            "reserved physical chord must not reach host state or pre-profile hotkeys");
}

void require_gesture_confirmation(size_t first, const uni_hid_device_t& left,
                                  const uni_hid_device_t& right) {
    unsigned left_pulses = 0;
    unsigned right_pulses = 0;
    for (size_t index = first; index < local_rumble_events.size(); ++index) {
        const auto& event = local_rumble_events[index];
        if ((event.high | event.low) == 0 || event.duration_ms == 0) continue;
        require(event.duration_ms <= 150 &&
                    (event.device == &left || event.device == &right),
                "gesture confirmation must be short and restricted to its two physical participants");
        if (event.device == &left) ++left_pulses;
        else ++right_pulses;
    }
    require(left_pulses == 1 && right_pulses == 1,
            "one successful gesture must acknowledge each physical half exactly once");
}

void test_switch2_gesture_timing() {
    runtime_configuration.joycon_mode = JoyConMode::kIndividual;
    start_backend();
    auto left = switch2_device(0, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    ready_switch2(left);
    ready_switch2(right);
    const uint32_t configuration_generation = runtime_configuration_generation;
    gesture_report(left, true, false);
    require(slot_snapshot(0).state.left_trigger == UINT16_MAX,
            "a trigger alone is not a reserved chord");
    for (unsigned elapsed = 0; elapsed <= 2500; elapsed += 50) {
        now_ms = elapsed;
        gesture_report(left);
        gesture_report(right, true, false);
        gesture_tick();
    }
    require_live_solo(left, "one complete half and one partial half must not join");
    require_live_solo(right, "partial participant must retain its solo owner");
    require(pair_observation_count == 0, "a partial gesture must never enroll a composite bank");
    release_gesture(left, right);
    for (unsigned elapsed = 0; elapsed < 500; elapsed += 50) {
        now_ms += 50;
        gesture_report(left);
        gesture_report(right, false, false);
        gesture_tick();
    }
    const uint32_t shared_start = now_ms;
    hold_gesture(left, right, 1999);
    require_live_solo(left, "two seconds must start at concurrent hold, not the earlier left press");
    require(pair_observation_count == 0, "1999ms must not seed or publish a pair");
    now_ms = shared_start + 2000;
    gesture_report(left);
    gesture_tick();
    require_live_solo(right, "a fresh left report must not stand in for the right's 2000ms held report");
    const size_t confirmation_start = local_rumble_events.size();
    gesture_report(right);
    gesture_tick();
    require(live_pair_slot(left, right) >= 0, "both fresh held spans at 2000ms must join Individual halves");
    require_gesture_confirmation(confirmation_start, left, right);
    const uint32_t joined_generation = slot_snapshot(live_pair_slot(left, right)).connection_generation;
    hold_gesture(left, right, 2500);
    gesture_report(left, false, false);
    gesture_report(right, false, true);
    gesture_tick();
    hold_gesture(left, right, 2500);
    require(live_pair_slot(left, right) >= 0 &&
                slot_snapshot(live_pair_slot(left, right)).connection_generation == joined_generation &&
                pair_observation_count == 1,
            "held input and one participant's incomplete release must not toggle or reseed again");
    release_gesture(left, right);
    hold_gesture(left, right, 2000);
    require_live_solo(left, "both participants' full release must permit a second toggle");
    require_live_solo(right, "second toggle must expose the other physical solo");
    require(runtime_configuration.joycon_mode == JoyConMode::kIndividual &&
                runtime_configuration_generation == configuration_generation,
            "connection gestures must never write the saved adapter preference");
}

void test_switch2_gesture_slot_order(bool left_first, uint8_t lower_slot) {
    runtime_configuration.joycon_mode = JoyConMode::kIndividual;
    start_backend();
    auto ordinary = device(0, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    auto left = switch2_device(lower_slot + 1, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(lower_slot, UNI_SW2_JOYCON_R_PID);
    if (lower_slot != 0) ready_switch2(ordinary);
    // Transport indices can put the left half in the higher player slot,
    // even when its ready callback arrives first.
    ready_switch2(left_first ? left : right);
    ready_switch2(left_first ? right : left);
    const auto lower_before = slot_snapshot(lower_slot);
    const auto higher_before = slot_snapshot(lower_slot + 1);
    const auto unrelated_before = slot_snapshot(0);
    hold_gesture(left, right, 2000);
    require(live_pair_slot(left, right) == lower_slot &&
                !slot_snapshot(lower_slot + 1).active,
            "gesture join must retain the lower participating player slot regardless of side or ready order");
    const auto joined = slot_snapshot(lower_slot);
    require_pair_owner(joined.identity, left, right);
    require(joined.connection_generation != lower_before.connection_generation &&
                slot_snapshot(lower_slot + 1).connection_generation != higher_before.connection_generation &&
                slot_snapshot(lower_slot + 1).state.left_stick_x == 0 &&
                slot_snapshot(lower_slot + 1).state.right_stick_x == 0 &&
                left.player_leds == (1u << lower_slot) &&
                right.player_leds == (1u << lower_slot),
            "join must invalidate old player epochs, neutralize the retired slot and light both halves for the retained player");
    if (lower_slot != 0) {
        require(slot_snapshot(0).active &&
                    slot_snapshot(0).connection_generation == unrelated_before.connection_generation &&
                    controller_identity_equal(slot_snapshot(0).identity, unrelated_before.identity),
                "gesture join must not take an earlier slot occupied by an unrelated controller");
    }
}

void test_switch2_gesture_stale() {
    runtime_configuration.joycon_mode = JoyConMode::kIndividual;
    start_backend();
    auto left = switch2_device(0, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    ready_switch2(left);
    ready_switch2(right);
    hold_gesture(left, right, 1000);
    for (unsigned elapsed = 0; elapsed < 2500; elapsed += 50) {
        now_ms += 50;
        gesture_report(left);
        gesture_tick();
    }
    require_live_solo(left, "cached right input must not fire after it goes stale");
    hold_gesture(left, right, 2200);
    require_live_solo(right, "resumed held reports must not rearm a stale attempt without release");
    require(pair_observation_count == 0, "stale attempts must not touch pair storage");
    release_gesture(left, right);
    hold_gesture(left, right, 1000);
    now_ms += 1100;
    gesture_reports(left, right);
    require_live_solo(left, "new reports must not hide a stale inter-report gap from a delayed timer");
    hold_gesture(left, right, 2200);
    require_live_solo(right, "a delayed-timer stale attempt must also require full release before retry");
    release_gesture(left, right);
    hold_gesture(left, right, 1900);
    now_ms += 100;
    gesture_tick();
    require_live_solo(left, "fresh cached reports alone cannot establish two full held spans");
    gesture_report(left);
    gesture_tick();
    require_live_solo(right, "a timer and one participant cannot complete the other's held span");
    gesture_report(right);
    gesture_tick();
    require(live_pair_slot(left, right) >= 0, "release and fresh report spans must recover a stale attempt");
}

void test_switch2_gesture_clock_wrap() {
    runtime_configuration.joycon_mode = JoyConMode::kIndividual;
    start_backend();
    auto left = switch2_device(0, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    ready_switch2(left);
    ready_switch2(right);
    now_ms = UINT32_MAX - 1000u;
    hold_gesture(left, right, 1999);
    require_live_solo(left, "clock wrap must not shorten the hold");
    ++now_ms;
    gesture_reports(left, right);
    require(live_pair_slot(left, right) >= 0, "continuous held reports must complete across millisecond wrap");
}

void configure_gesture_profile(const ControllerIdentity& identity, uint8_t index, uint8_t scale) {
    auto profile = controller_profile_default(identity, index);
    profile.weak_rumble_scale = scale;
    profile.button_map[static_cast<uint8_t>(ControllerProfileLogicalButton::kSelect)] =
        static_cast<uint8_t>(ControllerProfileLogicalButton::kNorth);
    profile.button_map[static_cast<uint8_t>(ControllerProfileLogicalButton::kStart)] =
        static_cast<uint8_t>(ControllerProfileLogicalButton::kCapture);
    profile.switching_chord =
        logical_button_bit(ControllerProfileLogicalButton::kSelect) |
        (1u << CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL);
    profile.motion_toggle_chord =
        logical_button_bit(ControllerProfileLogicalButton::kStart) |
        (1u << CONTROLLER_PROFILE_RIGHT_TRIGGER_CONTROL);
    profile.macros[0].trigger_mask = logical_button_bit(ControllerProfileLogicalButton::kSouth);
    profile.macros[0].first_step = 0;
    profile.macros[0].step_count = 1;
    profile.macros[0].mode = ControllerProfileMacroMode::kToggle;
    profile.macro_step_count = 1;
    for (uint8_t macro = 1; macro < CONTROLLER_PROFILE_MACRO_COUNT; ++macro) {
        profile.macros[macro].first_step = 1;
    }
    profile.macro_steps[0].override_flags = kControllerProfileOverrideButtons;
    profile.macro_steps[0].duration_ms = 10000;
    profile.macro_steps[0].output_button_mask = logical_button_bit(ControllerProfileLogicalButton::kSystem);
    require(runtime_profile_storage.set(identity, index, profile) == ProfileStorageResult::kOk &&
                runtime_profile_storage.activate(identity, index) == ProfileStorageResult::kOk,
            "gesture fixture needs independent profiles with observable macro and reserved-button mappings");
}

void test_switch2_gesture_masking_epochs() {
    start_backend();
    initialize_runtime_profile_storage();
    auto left = switch2_device(0, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    const auto left_identity = identity_for_device(&left);
    const auto right_identity = identity_for_device(&right);
    configure_gesture_profile(left_identity, 2, 37);
    configure_gesture_profile(right_identity, 5, 63);
    ready_switch2(left);
    ready_switch2(right);
    const auto pair_identity = slot_snapshot(0).identity;
    configure_gesture_profile(pair_identity, 7, 95);
    auto ordinary = device(2, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    ready_switch2(ordinary);
    auto ordinary_report = gesture_input(true, false, false);
    ordinary_report.gamepad.buttons = BUTTON_Y;
    platform_on_controller_data(&ordinary, &ordinary_report);
    const auto ordinary_before = slot_snapshot(2);
    bluepad32_input_backend_queue_rumble(2, ControllerRumbleOutput{71, 81});
    gesture_tick();
    const int ordinary_rumble_calls = ordinary.rumble_calls;
    const unsigned writes_before = profile_write_attempts;
    const uint32_t saved_generation = runtime_configuration_generation;
    gesture_profile_consumer_enabled = true;
    controller_profile_runtime_reset();
    release_gesture(left, right);
    auto macro_press = gesture_input(true, false, false);
    macro_press.gamepad.buttons = BUTTON_A;
    ++now_ms;
    platform_on_controller_data(&right, &macro_press);
    consume_gesture_profiles();
    require(gesture_profile_outputs[0].button_system, "a real profile macro must be running before the topology epoch");
    release_gesture(left, right);
    CaptureOptions options{};
    options.channels = 0x1f;
    options.max_events = 32;
    require(bluepad32_input_backend_capture_start(
                0, slot_snapshot(0).connection_generation, options),
            "pair capture must record the arming path before splitting");
    const auto pair_before = slot_snapshot(0);
    ++now_ms;
    gesture_report(left);
    require_gesture_masked(slot_snapshot(0));
    require(slot_snapshot(0).state.dpad_up && slot_snapshot(0).state.button_east &&
                slot_snapshot(0).state.left_stick_x == INT16_MIN &&
                slot_snapshot(0).state.right_stick_x == scale_axis(200),
            "masking a single physical chord must preserve unrelated buttons, axes and companion input");
    gesture_report(right);
    gesture_tick();
    require_gesture_masked(slot_snapshot(0));
    hold_gesture(left, right, 1999);
    require(gesture_profile_outputs[0].button_system &&
                !gesture_profile_outputs[0].button_north &&
                !gesture_profile_outputs[0].button_capture,
            "arming must not leak remapped menus or cancel the preexisting macro");
    Bluepad32CaptureSnapshot capture{};
    require(bluepad32_input_backend_capture_page(0, 0, &capture) &&
                capture.state == CaptureState::kRecording,
            "reserved chord consumption must not interrupt recording before success");
    for (uint8_t index = 0; index < capture.event_count; ++index) {
        require((capture.events[index].buttons &
                    (logical_button_bit(ControllerProfileLogicalButton::kSelect) |
                     logical_button_bit(ControllerProfileLogicalButton::kStart))) == 0 &&
                    capture.events[index].left_trigger == 0 && capture.events[index].right_trigger == 0,
                "recorded macros must never contain reserved gesture components");
    }
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{91, 101});
    bluepad32_input_backend_queue_profile_feedback(
        0, pair_before.connection_generation, 8, ControllerProfileConfirmationPolicy::kRumbleAndLed);
    const size_t host_events_before = switch2_host_events.size();
    const size_t confirmation_start = local_rumble_events.size();
    ++now_ms;
    gesture_reports(left, right);
    require_live_solo(left, "Paired default must allow a gesture split");
    require_live_solo(right, "gesture split must expose both members");
    require_gesture_masked(slot_snapshot(0));
    require_gesture_masked(slot_snapshot(1));
    require(slot_snapshot(0).state.button_west && slot_snapshot(1).state.button_south &&
                slot_snapshot(0).state.left_stick_x == scale_axis(100) &&
                slot_snapshot(1).state.left_stick_x == INT16_MAX &&
                !gesture_profile_outputs[0].button_system && !gesture_profile_outputs[1].button_system &&
                bluepad32_input_backend_capture_page(0, 0, &capture) &&
                capture.state == CaptureState::kDisconnected &&
                switch2_host_events.size() == host_events_before,
            "successful split must rotate retained input, cancel macro/capture epochs and discard queued old rumble");
    require_gesture_confirmation(confirmation_start, left, right);
    bluepad32_input_backend_queue_profile_feedback(
        0, pair_before.connection_generation, 8, ControllerProfileConfirmationPolicy::kRumbleAndLed);
    now_ms += 200;
    gesture_report(left, false, true);
    gesture_report(right, true, false);
    gesture_tick();
    require_gesture_masked(slot_snapshot(0));
    require_gesture_masked(slot_snapshot(1));
    require_gesture_confirmation(confirmation_start, left, right);
    require(slot_snapshot(2).connection_generation == ordinary_before.connection_generation &&
                slot_snapshot(2).state.button_north &&
                ordinary.rumble_calls == ordinary_rumble_calls && ordinary.last_low == 71,
            "gesture confirmation and retired feedback must not disturb unrelated input or rumble");
    release_gesture(left, right);
    gesture_report(left, true, false);
    gesture_report(right, false, true);
    require(slot_snapshot(0).state.left_trigger == UINT16_MAX && slot_snapshot(1).state.button_start,
            "each half must return its component inputs after both components have released");
    release_gesture(left, right);
    auto solo_macro_press = gesture_input(false, false, false);
    solo_macro_press.gamepad.dpad = DPAD_LEFT;
    ++now_ms;
    platform_on_controller_data(&left, &solo_macro_press);
    consume_gesture_profiles();
    require(gesture_profile_outputs[0].button_system,
            "a real solo profile macro must be running before rejoining");
    release_gesture(left, right);
    require(bluepad32_input_backend_capture_start(
                1, slot_snapshot(1).connection_generation, options),
            "the other solo must remain recordable before gesture join");
    hold_gesture(left, right, 1999);
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{111, 121});
    bluepad32_input_backend_queue_rumble(1, ControllerRumbleOutput{131, 141});
    const size_t solo_host_events_before = switch2_host_events.size();
    ++now_ms;
    gesture_reports(left, right);
    require(live_pair_slot(left, right) >= 0 && !gesture_profile_outputs[0].button_system &&
                !gesture_profile_outputs[1].button_system,
            "rejoining must retire both solo macro contexts without reviving the former pair macro");
    require(bluepad32_input_backend_capture_page(0, 0, &capture) &&
                capture.state == CaptureState::kDisconnected &&
                switch2_host_events.size() == solo_host_events_before,
            "gesture join must disconnect either solo recorder and discard both queued solo rumble epochs");
    require_active_profile(pair_identity, 7, 95);
    require_active_profile(left_identity, 2, 37);
    require_active_profile(right_identity, 5, 63);
    require(profile_write_attempts == writes_before &&
                runtime_configuration.joycon_mode == JoyConMode::kPaired &&
                runtime_configuration_generation == saved_generation,
            "gesture roundtrip must preserve all existing profile banks and saved preference without flash writes");
}

void test_switch2_gesture_override_lifetime() {
    runtime_configuration.joycon_mode = JoyConMode::kIndividual;
    start_backend();
    auto left = switch2_device(0, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    remember_switch2(left);
    remember_switch2(right);
    register_lookup_device(&left);
    register_lookup_device(&right);
    ready_switch2(left);
    ready_switch2(right);
    hold_gesture(left, right, 1900);
    platform_on_device_disconnected(&right);
    now_ms += 100;
    gesture_reports(left, right);
    require_live_solo(left, "disconnect during arming must leave the survivor solo");
    require(pair_observation_count == 0, "late detached reports cannot complete a pending gesture");
    ready_switch2(right);
    release_gesture(left, right);
    hold_gesture(left, right, 2000);
    require(live_pair_slot(left, right) >= 0, "reconnected halves must support a new connection-only join");
    auto ordinary = device(2, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    ready_switch2(ordinary);
    ++runtime_configuration.pairing_window_seconds;
    ++runtime_configuration_generation;
    gesture_tick();
    require(live_pair_slot(left, right) >= 0, "unrelated ready and configuration reconciliation must retain a join override");
    platform_on_device_disconnected(&left);
    gesture_tick();
    ready_switch2(left);
    gesture_tick();
    require_live_solo(left, "disconnect must restore Individual for the returning participant");
    require_live_solo(right, "disconnect must restore Individual for its still-connected partner");
    release_gesture(left, right);
    hold_gesture(left, right, 2000);
    set_runtime_joycon_mode(JoyConMode::kPaired);
    release_gesture(left, right);
    hold_gesture(left, right, 2000);
    require_live_solo(left, "a gesture must split while the saved default remains Paired");
    for (unsigned tick = 0; tick < 20; ++tick) {
        now_ms += 50;
        gesture_tick();
        require(!scanning_enabled && !classic_scanning_enabled &&
                    platform_on_device_discovered(right.conn.btaddr, nullptr, 0, 0) ==
                        UNI_ERROR_IGNORE_DEVICE,
                "forced solos under Paired must stop background mate discovery despite free transport capacity");
    }
    require_live_solo(left, "background reconciliation must not rematch a manually split pair");
    require_live_solo(right, "the second forced solo must stay independent beside an unrelated Classic link");
    auto pro = switch2_device(3, UNI_SW2_PRO_PID);
    ready_switch2(pro);
    gesture_tick();
    require_live_solo(right, "new ready events must not immediately undo the forced split");
    platform_on_device_disconnected(&right);
    gesture_tick();
    require(scanning_enabled && background_scan_parameters && !classic_scanning_enabled &&
                !bondable && !switch_pico_switch2_pairing_allowed() &&
                platform_on_device_discovered(right.conn.btaddr, nullptr, 0, 0) == UNI_ERROR_SUCCESS,
            "participant disconnect must clear the override and resume remembered-mate discovery without fresh pairing");
    ready_switch2(right);
    gesture_tick();
    require(live_pair_slot(left, right) >= 0, "either split participant disconnecting must restore Paired for both");
    require(!scanning_enabled && !classic_scanning_enabled,
            "default reconnection must stop background discovery after restoring the pair");
    release_gesture(left, right);
    hold_gesture(left, right, 2000);
    platform_on_device_disconnected(&right);
    auto replacement = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    replacement.conn.btaddr[5] = 0x55;
    ready_switch2(replacement);
    gesture_tick();
    require(live_pair_slot(left, replacement) >= 0 && live_pair_slot(left, right) < 0,
            "reusing the physical index for a different identity must not inherit either member's former override");
    platform_on_device_disconnected(&right);
    gesture_report(right);
    gesture_tick();
    require(live_pair_slot(left, replacement) >= 0,
            "late reports and disconnects from the retired participant must not clear its replacement's topology");
    release_gesture(left, replacement);
    hold_gesture(left, replacement, 2000);
    set_runtime_joycon_mode(JoyConMode::kIndividual);
    set_runtime_joycon_mode(JoyConMode::kPaired);
    require(live_pair_slot(left, replacement) >= 0, "changing the saved default must clear a forced-solo override");
    set_runtime_joycon_mode(JoyConMode::kIndividual);
    release_gesture(left, replacement);
    hold_gesture(left, replacement, 2000);
    require(live_pair_slot(left, replacement) >= 0, "Individual must still permit an explicit current-pair join");
    set_runtime_joycon_mode(JoyConMode::kPaired);
    set_runtime_joycon_mode(JoyConMode::kIndividual);
    require_live_solo(left, "changing the saved default must also clear a forced-pair override");
    require_live_solo(replacement, "clearing a forced-pair override must restore both Individual owners");
}

void test_switch2_gesture_two_pairs() {
    start_backend();
    auto right0 = switch2_device(0, UNI_SW2_JOYCON_R_PID);
    auto right1 = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    auto left0 = switch2_device(2, UNI_SW2_JOYCON_L_PID);
    auto left1 = switch2_device(3, UNI_SW2_JOYCON_L_PID);
    ready_switch2(right0);
    ready_switch2(right1);
    ready_switch2(left0);
    ready_switch2(left1);
    const auto pair0 = slot_snapshot(live_pair_slot(left0, right0));
    const auto pair1 = slot_snapshot(live_pair_slot(left1, right1));
    hold_gesture(left0, right1, 2000);
    require(live_pair_slot(left0, right0) >= 0 && live_pair_slot(left1, right1) >= 0 &&
                slot_snapshot(live_pair_slot(left0, right0)).connection_generation == pair0.connection_generation &&
                slot_snapshot(live_pair_slot(left1, right1)).connection_generation == pair1.connection_generation,
            "chords from different live pairs must never steal or split either pair's members");
    release_gesture(left0, right1);
    const size_t first_confirmation = local_rumble_events.size();
    hold_gesture(left0, right0, 2000);
    require_live_solo(left0, "only the first current pair must split for its own two participants");
    require_live_solo(right0, "first pair's right member must become independent");
    require(live_pair_slot(left1, right1) >= 0 &&
                slot_snapshot(live_pair_slot(left1, right1)).connection_generation == pair1.connection_generation,
            "unjoining one pair must not split or invalidate the unrelated pair");
    require_gesture_confirmation(first_confirmation, left0, right0);
    const size_t second_confirmation = local_rumble_events.size();
    hold_gesture(left1, right1, 2000);
    require_gesture_confirmation(second_confirmation, left1, right1);
    require_live_solo(left1, "second current pair must split independently");
    require_live_solo(right1, "second pair's right member must become independent");
    release_gesture(left0, right0);
    release_gesture(left1, right1);
    const auto former_right = slot_snapshot(live_identity_slot(identity_for_device(&right0)));
    const auto former_left = slot_snapshot(live_identity_slot(identity_for_device(&left1)));
    const size_t confirmation_start = local_rumble_events.size();
    hold_gesture(left0, right1, 2000);
    require(live_pair_slot(left0, right1) >= 0, "explicit solo participants must be allowed to choose a new cross-pair");
    require_gesture_confirmation(confirmation_start, left0, right1);
    gesture_tick();
    require_live_solo(right0, "cross-joining must not regroup the untouched former right partner");
    require_live_solo(left1, "cross-joining must not regroup the untouched former left partner");
    require(slot_snapshot(live_identity_slot(identity_for_device(&right0))).connection_generation ==
                former_right.connection_generation &&
                slot_snapshot(live_identity_slot(identity_for_device(&left1))).connection_generation ==
                    former_left.connection_generation,
            "changing partners must leave former partners' logical epochs intact");
    platform_on_device_disconnected(&right1);
    auto replacement = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    replacement.conn.btaddr[5] = 0x65;
    ready_switch2(replacement);
    gesture_tick();
    require(live_pair_slot(left0, replacement) >= 0,
            "a former pair hint must not pin a cleared survivor to a stolen or disconnected member");
    require_live_solo(right0, "clearing the current override must preserve the untouched former right's solo override");
    require_live_solo(left1, "clearing the current override must preserve the untouched former left's solo override");
    hold_gesture(left1, right0, 2000);
    require(live_pair_slot(left1, right0) >= 0 && live_pair_slot(left0, replacement) >= 0,
            "orphaned former partners must be independently joinable without disturbing the current pair");
}

void test_switch2_gesture_ambiguous() {
    runtime_configuration.joycon_mode = JoyConMode::kIndividual;
    start_backend();
    auto left0 = switch2_device(0, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    auto left1 = switch2_device(2, UNI_SW2_JOYCON_L_PID);
    ready_switch2(left0);
    ready_switch2(right);
    ready_switch2(left1);
    const auto spare_before = slot_snapshot(2);
    for (unsigned elapsed = 0; elapsed <= 2500; elapsed += 50) {
        now_ms = elapsed;
        gesture_report(left0);
        gesture_report(right);
        gesture_report(left1);
        gesture_tick();
    }
    require_live_solo(left0, "two armed left solos must not choose an arbitrary pairing");
    require_live_solo(right, "ambiguous solo arming must preserve the only right player");
    require_live_solo(left1, "ambiguous solo arming must preserve the other left player");
    gesture_report(left1, false, false);
    hold_gesture(left0, right, 2200);
    require(pair_observation_count == 0 && local_rumble_events.empty(),
            "resolving ambiguity without releasing the attempt must not retry storage or acknowledge success");
    release_gesture(left0, right);
    hold_gesture(left0, right, 2000);
    require(live_pair_slot(left0, right) >= 0 &&
                slot_snapshot(2).connection_generation == spare_before.connection_generation &&
                controller_identity_equal(slot_snapshot(2).identity, spare_before.identity),
            "full release must allow one eligible L/R while preserving the unarmed extra solo");
}

void test_switch2_gesture_seed_failure() {
    runtime_configuration.joycon_mode = JoyConMode::kIndividual;
    start_backend();
    initialize_runtime_profile_storage();
    auto left = switch2_device(0, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    const auto left_identity = identity_for_device(&left);
    const auto right_identity = identity_for_device(&right);
    configure_gesture_profile(left_identity, 2, 37);
    configure_gesture_profile(right_identity, 5, 63);
    ready_switch2(left);
    ready_switch2(right);
    // Establish the current USB host mode before testing a gesture failure;
    // the adapter variant legitimately resets its initial haptics epoch.
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{});
    bluepad32_input_backend_queue_rumble(1, ControllerRumbleOutput{});
    release_gesture(left, right);
    const unsigned left_resets_before = left.switch2_haptics_resets;
    const unsigned right_resets_before = right.switch2_haptics_resets;
    const auto left_before = slot_snapshot(0);
    const auto right_before = slot_snapshot(1);
    CaptureOptions options{};
    options.max_duration_ms = 20000;
    require(bluepad32_input_backend_capture_start(1, right_before.connection_generation, options),
            "failed admission must preserve a live solo capture");
    hold_gesture(left, right, 1999);
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{17, 27});
    bluepad32_input_backend_queue_rumble(1, ControllerRumbleOutput{47, 57});
    fail_profile_program = true;
    ++now_ms;
    gesture_reports(left, right);
    Bluepad32CaptureSnapshot capture{};
    require_live_solo(left, "failed seed must not retire the left solo");
    require_live_solo(right, "failed seed must not retire the right solo");
    require(slot_snapshot(0).connection_generation == left_before.connection_generation &&
                slot_snapshot(1).connection_generation == right_before.connection_generation &&
                slot_snapshot(0).state.button_west && slot_snapshot(1).state.button_south &&
                bluepad32_input_backend_capture_page(0, 0, &capture) &&
                capture.state == CaptureState::kRecording &&
                left.switch2_haptics_resets == left_resets_before &&
                right.switch2_haptics_resets == right_resets_before &&
                left.last_low == 17 && right.last_low == 47 && pair_observation_count == 1,
            "seed failure must preserve identities, input, generations, recording and queued independent rumble");
    const unsigned failed_writes = profile_write_attempts;
    hold_gesture(left, right, 5000);
    ++runtime_configuration.pairing_window_seconds;
    ++runtime_configuration_generation;
    gesture_tick();
    require(pair_observation_count == 1 && profile_write_attempts == failed_writes &&
                local_rumble_events.empty(),
            "failed held gesture must not retry flash, churn topology or emit confirmation pulses");
    fail_profile_program = false;
    require(runtime_profile_storage.initialize(runtime_profile_io()),
            "catalog replay must retain intact solo banks after failed pair admission");
    require_active_profile(left_identity, 2, 37);
    require_active_profile(right_identity, 5, 63);
    release_gesture(left, right);
    hold_gesture(left, right, 2000);
    require(live_pair_slot(left, right) >= 0 && pair_observation_count == 2,
            "explicit release and retry must recover once profile storage is available");
    require_active_profile(slot_snapshot(live_pair_slot(left, right)).identity, 2, 37);
    require_active_profile(right_identity, 5, 63);
    require(runtime_configuration.joycon_mode == JoyConMode::kIndividual,
            "failed and successful connection gestures must both leave the saved preference Individual");
}

void test_switch2_gesture_device_scope() {
    start_backend();
    auto original_left = device(0, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    auto original_right = device(1, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    original_left.vendor_id = original_right.vendor_id = UNI_SW2_NINTENDO_VID;
    original_left.product_id = 0x2006;
    original_right.product_id = 0x2007;
    auto pro = switch2_device(2, UNI_SW2_PRO_PID);
    auto other = device(3, true, UNI_BT_CONN_PROTOCOL_BLE);
    uni_hid_device_t* controllers[] = {&original_left, &original_right, &pro, &other};
    for (auto* controller : controllers) ready_switch2(*controller);
    auto report = gesture_input(false);
    report.gamepad.buttons |= BUTTON_TRIGGER_R;
    report.gamepad.misc_buttons |= MISC_BUTTON_START;
    report.gamepad.throttle = 1023;
    for (unsigned elapsed = 0; elapsed <= 2500; elapsed += 50) {
        now_ms = elapsed;
        for (auto* controller : controllers) platform_on_controller_data(controller, &report);
        gesture_tick();
    }
    for (uint8_t index = 0; index < 4; ++index) {
        const auto snapshot = slot_snapshot(index);
        require(snapshot.active && snapshot.state.button_select && snapshot.state.button_start &&
                    snapshot.state.left_trigger == UINT16_MAX && snapshot.state.right_trigger == UINT16_MAX,
                "original JoyCons, Switch2 Pro and unrelated controllers must retain all chord inputs");
    }
    require(pair_observation_count == 0 && local_rumble_events.empty(),
            "the connection gesture must never operate on non-JoyCon2 controllers");
}

ControllerRumbleOutput ordered_switch2_hd(uint8_t frequency = 40) {
    ControllerRumbleOutput rumble{17, 29};
    rumble.hd.actuators[0].sample_count = 3;
    rumble.hd.actuators[1].sample_count = 2;
    rumble.hd.actuators[0].samples[0] = {frequency, 61, 24000, 4000};
    rumble.hd.actuators[0].samples[1] = {41, 62, 5000, 25000};
    rumble.hd.actuators[0].samples[2] = {42, 63, 20000, 14000};
    rumble.hd.actuators[1].samples[0] = {81, 91, 3000, 21000};
    rumble.hd.actuators[1].samples[1] = {82, 92, 23000, 7000};
    return rumble;
}

void require_switch2_sample(const uint8_t encoded[5],
                            const SwitchHapticsSample& source) {
    uint64_t bits = 0;
    for (unsigned index = 0; index < 5; ++index) {
        bits |= uint64_t{encoded[index]} << (index * 8);
    }
    require((bits & 1023u) == 193u + 3u * source.low_frequency_index &&
                ((bits >> 20) & 1023u) == 289u + 3u * source.high_frequency_index &&
                ((bits >> 10) & 1023u) ==
                    ((uint32_t{source.low_amplitude_q15} * 29000u / 32767u) >> 6) &&
                ((bits >> 30) & 1023u) ==
                    ((uint32_t{source.high_amplitude_q15} * 29000u / 32767u) >> 6),
            "native physical sample lost its measured frequency or independent band amplitude");
}

void test_switch2_hd_pro() {
    start_pairing_backend();
    auto pro = switch2_device(0, UNI_SW2_PRO_PID);
    ready_switch2(pro);
    now_ms = 100;
    const auto first = ordered_switch2_hd(40);
    const auto second = ordered_switch2_hd(50);
    bluepad32_input_backend_queue_rumble(0, first);
    now_ms = 101;
    bluepad32_input_backend_queue_rumble(0, second);
    now_ms = 105;
    process_rumble_timer(&g_rumble_timer);
    require(switch2_host_events.size() == 2 && pro.rumble_calls == 0 &&
                switch2_host_events[0].received_ms == 100 &&
                switch2_host_events[1].received_ms == 101,
            "rapid HD commands must bypass the compatibility mailbox in original order and time");
    for (unsigned event = 0; event < 2; ++event) {
        const auto& input = event == 0 ? first : second;
        const auto& output = switch2_host_events[event].frame;
        require(output.sides[0].count == 3 && output.sides[1].count == 2,
                "Pro output must preserve both native side counts");
        for (unsigned side = 0; side < 2; ++side) {
            for (unsigned step = 0; step < output.sides[side].count; ++step) {
                require_switch2_sample(output.sides[side].samples[step],
                                       input.hd.actuators[side].samples[step]);
            }
        }
    }
    ControllerRumbleOutput partial{};
    partial.hd.actuators[0].sample_count = 1;
    bluepad32_input_backend_queue_rumble(0, first);
    bluepad32_input_backend_queue_rumble(0, partial);
    process_rumble_timer(&g_rumble_timer);
    require(switch2_host_events.size() == 4 &&
                switch2_host_events.back().frame.sides[1].count == 0,
            "a silent partial update must neither flush older commands nor invent a right stop");

    pro.switch2_host_blocked = true;
    bluepad32_input_backend_queue_rumble(0, first);
    ControllerRumbleOutput stop{};
    stop.hd.actuators[0].sample_count = 1;
    stop.hd.actuators[1].sample_count = 1;
    bluepad32_input_backend_queue_rumble(0, stop);
    now_ms += 100;
    process_rumble_timer(&g_rumble_timer);
    require(switch2_host_events.size() == 5 &&
                uni_switch2_haptics_is_stop(&switch2_host_events.back().frame),
            "explicit native stop must clear older ingress and bypass age/full barriers");
    pro.switch2_host_blocked = false;
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{19, 23});
    process_rumble_timer(&g_rumble_timer);
    require(!switch2_host_events.back().hd && switch2_host_events.back().strong == 19 &&
                switch2_host_events.back().weak == 23 && pro.rumble_calls == 0,
            "both empty HD sides must use the dedicated conventional host API");
}

void test_switch2_hd_solo(bool right) {
    start_pairing_backend();
    auto solo = switch2_device(0, right ? UNI_SW2_JOYCON_R_PID : UNI_SW2_JOYCON_L_PID);
    ready_switch2(solo);
    const auto input = ordered_switch2_hd();
    bluepad32_input_backend_queue_rumble(0, input);
    process_rumble_timer(&g_rumble_timer);
    require(switch2_host_events.size() == 1 &&
                switch2_host_events[0].frame.sides[0].count == 3 &&
                switch2_host_events[0].frame.sides[1].count == 0,
            "either solo half must receive a mono sequence in physical side zero");
    const SwitchHapticsSample expected[] = {
        {40, 91, 24000, 21000}, {82, 62, 23000, 25000}, {82, 63, 23000, 14000}};
    for (unsigned index = 0; index < 3; ++index) {
        require_switch2_sample(switch2_host_events[0].frame.sides[0].samples[index],
                               expected[index]);
    }
    auto tie = input;
    tie.hd.actuators[0].sample_count = 1;
    tie.hd.actuators[1].sample_count = 1;
    tie.hd.actuators[1].samples[0].low_amplitude_q15 = 24000;
    tie.hd.actuators[1].samples[0].high_amplitude_q15 = 4000;
    bluepad32_input_backend_queue_rumble(0, tie);
    process_rumble_timer(&g_rumble_timer);
    require_switch2_sample(switch2_host_events.back().frame.sides[0].samples[0],
                           tie.hd.actuators[0].samples[0]);
    auto one_side = input;
    one_side.hd.actuators[0].sample_count = 0;
    bluepad32_input_backend_queue_rumble(0, one_side);
    process_rumble_timer(&g_rumble_timer);
    require(switch2_host_events.back().frame.sides[0].count == 2,
            "solo absent source side must not fabricate substeps");
    require_switch2_sample(switch2_host_events.back().frame.sides[0].samples[1],
                           one_side.hd.actuators[1].samples[1]);
}

void test_switch2_hd_pair() {
    start_pairing_backend();
    auto right = switch2_device(0, UNI_SW2_JOYCON_R_PID);
    auto left = switch2_device(1, UNI_SW2_JOYCON_L_PID);
    ready_switch2(right);
    ready_switch2(left);
    right.switch2_host_blocked = true;
    now_ms = 10;
    const auto input = ordered_switch2_hd();
    bluepad32_input_backend_queue_rumble(0, input);
    process_rumble_timer(&g_rumble_timer);
    now_ms = 15;
    process_rumble_timer(&g_rumble_timer);
    require(switch2_host_events.size() == 1 && switch2_host_events[0].device == &left,
            "one blocked half must not duplicate acceptance on its ready partner");
    right.switch2_host_blocked = false;
    now_ms = 20;
    process_rumble_timer(&g_rumble_timer);
    require(switch2_host_events.size() == 2 && switch2_host_events[1].device == &right &&
                switch2_host_events[1].received_ms == 10,
            "paired retry must retain the source timestamp and original missing half");
    for (unsigned half = 0; half < 2; ++half) {
        const auto& output = switch2_host_events[half].frame;
        require(output.sides[0].count == (half == 0 ? 3 : 2) && output.sides[1].count == 0,
                "pair stereo must map selected source side onto each physical side zero");
        for (unsigned step = 0; step < output.sides[0].count; ++step) {
            require_switch2_sample(output.sides[0].samples[step],
                                   input.hd.actuators[half].samples[step]);
        }
    }
    auto right_only = input;
    right_only.hd.actuators[0].sample_count = 0;
    bluepad32_input_backend_queue_rumble(0, right_only);
    process_rumble_timer(&g_rumble_timer);
    require(switch2_host_events.size() == 3 && switch2_host_events.back().device == &right,
            "count zero on one paired half must not submit an invented neutral update");
    right.switch2_host_blocked = true;
    bluepad32_input_backend_queue_rumble(0, input);
    process_rumble_timer(&g_rumble_timer);
    now_ms += 50;
    right.switch2_host_blocked = false;
    process_rumble_timer(&g_rumble_timer);
    Bluepad32BackendDiagnostics diagnostics{};
    bluepad32_input_backend_diagnostics(&diagnostics);
    require(switch2_host_events.size() == 4 && diagnostics.switch2_ingress_drops == 1,
            "an expired partly accepted pair must drop visibly without late or duplicate delivery");
    right.switch2_host_blocked = true;
    bluepad32_input_backend_queue_rumble(0, input);
    process_rumble_timer(&g_rumble_timer);
    ControllerRumbleOutput stop{};
    stop.hd.actuators[0].sample_count = 1;
    stop.hd.actuators[1].sample_count = 1;
    bluepad32_input_backend_queue_rumble(0, stop);
    process_rumble_timer(&g_rumble_timer);
    require(switch2_host_events.size() == 7 &&
                switch2_host_events[5].device == &left &&
                switch2_host_events[6].device == &right &&
                switch2_host_events[5].frame.sides[0].count == 1 &&
                switch2_host_events[6].frame.sides[0].count == 1,
            "paired explicit stop must reach both halves even after partial acceptance and backpressure");
}

void test_switch2_hd_overflow() {
    start_pairing_backend();
    auto pro = switch2_device(0, UNI_SW2_PRO_PID);
    ready_switch2(pro);
    now_ms = 100;
    for (uint8_t command = 0; command < 17; ++command) {
        bluepad32_input_backend_queue_rumble(0, ordered_switch2_hd(40 + command));
    }
    Bluepad32BackendDiagnostics diagnostics{};
    bluepad32_input_backend_diagnostics(&diagnostics);
    require(diagnostics.switch2_ingress_drops == 1 && diagnostics.rumble_pending_slots == 1,
            "bounded ingress overflow must expose one oldest-command drop");
    process_rumble_timer(&g_rumble_timer);
    require(switch2_host_events.size() == 16,
            "one drain must preserve all sixteen surviving commands, not a latest-value mailbox");
    for (uint8_t index = 0; index < 16; ++index) {
        const auto input = ordered_switch2_hd(41 + index);
        require_switch2_sample(switch2_host_events[index].frame.sides[0].samples[0],
                               input.hd.actuators[0].samples[0]);
    }
    pro.switch2_host_blocked = true;
    now_ms = UINT32_MAX - 20;
    bluepad32_input_backend_queue_rumble(0, ordered_switch2_hd());
    now_ms = 28;  // 49ms across wrap.
    process_rumble_timer(&g_rumble_timer);
    bluepad32_input_backend_diagnostics(&diagnostics);
    require(diagnostics.rumble_pending_slots == 1 && diagnostics.switch2_ingress_drops == 1,
            "backpressure must retain an unexpired command across clock wrap");
    now_ms = 29;
    process_rumble_timer(&g_rumble_timer);
    bluepad32_input_backend_diagnostics(&diagnostics);
    require(diagnostics.rumble_pending_slots == 0 && diagnostics.switch2_ingress_drops == 2 &&
                switch2_host_events.size() == 16,
            "original 50ms age must release a backpressured ingress head exactly at expiry");
    pro.switch2_host_blocked = false;
    bluepad32_input_backend_queue_rumble(0, ordered_switch2_hd());
    auto& stale = g_slots[0].switch2_ingress;
    --stale.commands[stale.head].envelope.connection_generation;
    process_rumble_timer(&g_rumble_timer);
    bluepad32_input_backend_diagnostics(&diagnostics);
    require(switch2_host_events.size() == 16 && diagnostics.switch2_ingress_drops == 3,
            "stale logical generation must be rejected before physical submission");
    bluepad32_input_backend_queue_rumble(0, ordered_switch2_hd());
    --stale.commands[stale.head].generation;
    process_rumble_timer(&g_rumble_timer);
    bluepad32_input_backend_diagnostics(&diagnostics);
    require(switch2_host_events.size() == 16 && diagnostics.switch2_ingress_drops == 4,
            "stale haptics epoch must not survive a same-connection mode reset");
    switch2_output_drops = 7;
    bluepad32_input_backend_diagnostics(&diagnostics);
    require(diagnostics.switch2_output_drops == 7,
            "physical output loss must remain visible separately from ingress loss");
}

void test_switch2_hd_epochs() {
    start_pairing_backend();
    auto left = switch2_device(0, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    ready_switch2(left);
    bluepad32_input_backend_queue_rumble(0, ordered_switch2_hd());
    process_rumble_timer(&g_rumble_timer);
    const unsigned left_resets = left.switch2_haptics_resets;
    bluepad32_input_backend_queue_rumble(0, ordered_switch2_hd());
    ready_switch2(right);
    process_rumble_timer(&g_rumble_timer);
    require(switch2_host_events.size() == 1 && left.switch2_haptics_resets > left_resets &&
                right.switch2_haptics_resets != 0,
            "merge must reset accepted physical output and drop queued solo commands");
    bluepad32_input_backend_queue_rumble(0, ordered_switch2_hd());
    process_rumble_timer(&g_rumble_timer);
    const unsigned pair_resets = left.switch2_haptics_resets;
    bluepad32_input_backend_queue_rumble(0, ordered_switch2_hd());
    platform_on_device_disconnected(&right);
    process_rumble_timer(&g_rumble_timer);
    require(switch2_host_events.size() == 3 && left.switch2_haptics_resets > pair_resets,
            "survivor transition must flush both physical and ingress pair state");
    bluepad32_input_backend_queue_rumble(0, ordered_switch2_hd());
    platform_on_device_disconnected(&left);
    auto replacement = switch2_device(0, UNI_SW2_PRO_PID);
    ready_switch2(replacement);
    process_rumble_timer(&g_rumble_timer);
    require(switch2_host_events.size() == 3,
            "reconnect must not inherit a former logical generation's host output");
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{27, 35});
    process_rumble_timer(&g_rumble_timer);
    const unsigned mode_resets = replacement.switch2_haptics_resets;
    bluepad32_input_backend_queue_rumble(0, ordered_switch2_hd());
    test_adapter_mode = AdapterUsbMode::kSwitchProbe;
    process_rumble_timer(&g_rumble_timer);
    require(switch2_host_events.size() == 4 && replacement.switch2_haptics_resets > mode_resets,
            "mode boundary without new input must neutralize held output and queued old-mode HD");
    bluepad32_input_backend_queue_rumble(0, ordered_switch2_hd(50));
    test_adapter_mode = AdapterUsbMode::kXInput;
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{45, 55});
    process_rumble_timer(&g_rumble_timer);
    require(switch2_host_events.size() == 5 && !switch2_host_events.back().hd &&
                switch2_host_events.back().duration_ms == UINT16_MAX &&
                switch2_host_events.back().strong == 45,
            "producer-side mode transition must discard old epoch before accepting new host state");
#endif
}

void test_switch2_hd_feedback() {
    start_pairing_backend();
    auto pro = switch2_device(0, UNI_SW2_PRO_PID);
    ready_switch2(pro);
    bluepad32_input_backend_queue_profile_feedback(
        0, slot_snapshot(0).connection_generation, 2,
        ControllerProfileConfirmationPolicy::kRumble);
    process_rumble_timer(&g_rumble_timer);
    require(pro.rumble_calls == 1, "profile confirmation must retain local-feedback ownership");
    now_ms = 10;
    bluepad32_input_backend_queue_rumble(0, ordered_switch2_hd());
    process_rumble_timer(&g_rumble_timer);
    now_ms = 20;
    bluepad32_input_backend_queue_rumble(0, ordered_switch2_hd(50));
    process_rumble_timer(&g_rumble_timer);
    require(switch2_host_events.size() == 2 && pro.rumble_calls == 1 &&
                switch2_host_events[0].received_ms == 10 &&
                switch2_host_events[1].received_ms == 20,
            "host substeps must advance during feedback without compatibility dispatch or fresh timestamps");
    now_ms = 300;
    process_rumble_timer(&g_rumble_timer);
    require(switch2_host_events.size() == 2,
            "feedback completion must not replay historical host substeps");
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{33, 44});
    process_rumble_timer(&g_rumble_timer);
    require(switch2_host_events.size() == 3 && !switch2_host_events.back().hd &&
                switch2_host_events.back().duration_ms == host_rumble_duration_ms(),
            "conventional host vibration must share the bounded host queue, not local feedback");
    now_ms += 1000;
    process_rumble_timer(&g_rumble_timer);
    require(switch2_host_events.size() == 3,
            "stateful host output must not require backend periodic resubmission");
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{});
    process_rumble_timer(&g_rumble_timer);
    require(switch2_host_events.size() == 4 && switch2_host_events.back().duration_ms == 0 &&
                switch2_host_events.back().weak == 0 && switch2_host_events.back().strong == 0,
            "conventional all-zero host command must explicitly stop retained state");
    pro.switch2_host_blocked = true;
    const uint32_t received_ms = now_ms;
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{65, 75});
    now_ms += 1000;
    process_rumble_timer(&g_rumble_timer);
    pro.switch2_host_blocked = false;
    process_rumble_timer(&g_rumble_timer);
    if (host_rumble_duration_ms() == UINT16_MAX) {
        require(switch2_host_events.size() == 5 &&
                    switch2_host_events.back().received_ms == received_ms &&
                    switch2_host_events.back().strong == 65,
                "held XInput host state must survive backpressure without retimestamping");
    } else {
        Bluepad32BackendDiagnostics diagnostics{};
        bluepad32_input_backend_diagnostics(&diagnostics);
        require(switch2_host_events.size() == 4 && diagnostics.switch2_ingress_drops == 1,
                "finite conventional host state must expire under feedback/backpressure, not revive");
    }
}

void test_switch2_pair_lifecycle(bool right_first) {
    start_pairing_backend();
    uni_hid_device_t left = switch2_device(
        right_first ? 1 : 0, UNI_SW2_JOYCON_L_PID);
    uni_hid_device_t right = switch2_device(
        right_first ? 0 : 1, UNI_SW2_JOYCON_R_PID);
    left.conn.btaddr[0] = 0xc1;
    left.switch2_identity_address_type = BD_ADDR_TYPE_LE_RANDOM;
    initialize_runtime_profile_storage();
    const ControllerIdentity left_identity = identity_for_device(&left);
    const ControllerIdentity right_identity = identity_for_device(&right);
    ControllerProfile left_profile = controller_profile_default(left_identity, 2);
    left_profile.weak_rumble_scale = 37;
    require(runtime_profile_storage.set(left_identity, 2, left_profile) ==
                ProfileStorageResult::kOk &&
                runtime_profile_storage.activate(left_identity, 2) == ProfileStorageResult::kOk &&
                runtime_profile_storage.activate(right_identity, 5) == ProfileStorageResult::kOk,
            "solo owners must have independently selected profiles before pairing");
    uni_hid_device_t& first = right_first ? right : left;
    uni_hid_device_t& second = right_first ? left : right;
    ready_switch2(first);

    uni_controller_t left_data{};
    left_data.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    left_data.gamepad.dpad = DPAD_UP;
    left_data.gamepad.axis_x = -512;
    left_data.gamepad.axis_y = 100;
    left_data.gamepad.buttons = BUTTON_TRIGGER_L;
    left_data.gamepad.accel[0] = 8192;
    left.switch2_extra_buttons =
        UNI_SW2_BUTTON_GL | UNI_SW2_BUTTON_LEFT_SL | UNI_SW2_BUTTON_LEFT_SR;
    uni_controller_t right_data{};
    right_data.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    right_data.gamepad.buttons = BUTTON_B | BUTTON_THUMB_R | BUTTON_TRIGGER_R;
    right_data.gamepad.axis_rx = 200;
    right_data.gamepad.axis_ry = -512;
    right_data.gamepad.accel[2] = 8192;
    right.switch2_extra_buttons =
        UNI_SW2_BUTTON_C | UNI_SW2_BUTTON_GR |
        UNI_SW2_BUTTON_RIGHT_SL | UNI_SW2_BUTTON_RIGHT_SR;
    platform_on_controller_data(&first, right_first ? &right_data : &left_data);
    const Bluepad32SlotSnapshot solo = slot_snapshot(0);
    require(solo.active && solo.state.button_left_shoulder &&
                solo.state.button_right_shoulder && solo.state.button_left_stick == right_first &&
                solo.state.right_stick_x == 0 && solo.state.right_stick_y == 0 &&
                (right_first ? solo.state.button_south : solo.state.button_west),
            "solo JoyCon must rotate face controls, stick click and rail shoulders");
    Bluepad32PlaytestSnapshot playtest{};
    bluepad32_input_backend_playtest_snapshot(0, &playtest);
    require(playtest.controller_layout ==
                (right_first ? Bluepad32ControllerLayout::kJoyCon2RightSolo
                             : Bluepad32ControllerLayout::kJoyCon2LeftSolo),
            "playtest must identify the live rotated solo half");
    require(solo.state.left_stick_x ==
                (right_first ? INT16_MAX : scale_axis(100)) &&
                solo.state.left_stick_y ==
                    (right_first ? scale_axis(200) : INT16_MAX),
            "solo JoyCon stick must rotate with its physical sideways orientation");
    require(bluepad32_input_backend_capture_start(
                0, solo.connection_generation, CaptureOptions{}),
            "solo capture must start on its logical generation");
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{31, 41});
    bluepad32_input_backend_queue_profile_feedback(
        0, solo.connection_generation, 8,
        ControllerProfileConfirmationPolicy::kRumbleAndLed);
    ready_switch2(second);

    Bluepad32SlotSnapshot merged = slot_snapshot(0);
    require(merged.active && !slot_snapshot(1).active &&
                merged.connection_generation != solo.connection_generation,
            "either connection order must merge into the first-ready output");
    require_pair_owner(merged.identity, left, right);
    require_active_profile(merged.identity, 2, 37);
    ControllerProfile pair_profile = controller_profile_default(merged.identity, 7);
    pair_profile.weak_rumble_scale = 95;
    require(runtime_profile_storage.set(merged.identity, 7, pair_profile) ==
                ProfileStorageResult::kOk &&
                runtime_profile_storage.activate(merged.identity, 7) == ProfileStorageResult::kOk,
            "the merged bank must be independently editable");
    require_active_profile(left_identity, 2, 37);
    require_active_profile(right_identity, 5, UINT8_MAX);
    bluepad32_input_backend_playtest_snapshot(0, &playtest);
    require(playtest.controller_layout ==
                Bluepad32ControllerLayout::kJoyCon2MergedPair &&
                playtest.state.motion_sample_count == 0 &&
                controller_identity_equal(playtest.identity, merged.identity),
            "playtest must detect a live companion before any merged motion report");
    Bluepad32CaptureSnapshot capture{};
    require(bluepad32_input_backend_capture_page(0, 0, &capture) &&
                capture.state == CaptureState::kDisconnected,
            "solo recording must not silently cross into the merged generation");
    require(!slot_snapshot(1).state.extra_buttons &&
                !slot_snapshot(1).state.button_south &&
                !slot_snapshot(1).state.button_west &&
                left.last_rumble_duration_ms == 0 && right.last_rumble_duration_ms == 0,
            "pair merge must neutralize ghost output and stop old feedback");
    const int calls_after_merge = left.rumble_calls + right.rumble_calls;
    bluepad32_input_backend_queue_profile_feedback(
        0, solo.connection_generation, 8,
        ControllerProfileConfirmationPolicy::kRumbleAndLed);
    process_rumble_timer(&g_rumble_timer);
    require(left.rumble_calls + right.rumble_calls == calls_after_merge &&
                left.player_leds == 1 && right.player_leds == 1,
            "old-generation solo feedback must not reach the pair");
    require(!bluepad32_input_backend_identify(left_identity) &&
                !bluepad32_input_backend_identify(right_identity) &&
                bluepad32_input_backend_identify(merged.identity),
            "identify must match the pair owner, not either physical solo owner");
    process_rumble_timer(&g_rumble_timer);
    require(left.last_low == UINT8_MAX && right.last_low == UINT8_MAX &&
                left.player_leds == 1 && right.player_leds == 1,
            "identifying the pair owner must reach both physical halves");
    now_ms += 150;
    process_rumble_timer(&g_rumble_timer);
    dispatch_identity_event(SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED, left,
                            left.switch2_identity_address_type, left.conn.btaddr);
    dispatch_identity_event(SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED, right,
                            right.switch2_identity_address_type, right.conn.btaddr);
    require(controller_identity_equal(slot_snapshot(0).identity, merged.identity),
            "physical identity publication must not replace an enrolled composite owner");
    dispatch_identity_event(SM_EVENT_IDENTITY_RESOLVING_FAILED, left,
                            left.switch2_identity_address_type, left.conn.btaddr);
    dispatch_identity_event(SM_EVENT_IDENTITY_RESOLVING_STARTED, right,
                            right.switch2_identity_address_type, right.conn.btaddr);
    require(controller_identity_equal(slot_snapshot(0).identity, merged.identity),
            "physical identity resolution resets must not demote a live pair to global");
    platform_on_controller_data(&left, &left_data);
    platform_on_controller_data(&right, &right_data);
    merged = slot_snapshot(0);
    require(merged.state.dpad_up && merged.state.button_east &&
                merged.state.button_right_stick && !merged.state.button_left_stick &&
                !merged.state.button_left_shoulder && !merged.state.button_right_shoulder &&
                merged.state.left_stick_x == INT16_MIN &&
                merged.state.right_stick_x == scale_axis(200) &&
                merged.state.left_trigger == UINT16_MAX &&
                merged.state.right_trigger == UINT16_MAX &&
                merged.state.extra_buttons == 0x7f &&
                merged.state.motion_samples[0].accel_x == -4096 &&
                merged.state.motion_samples[0].accel_y == 0,
            "merged native controls and extras must combine with right-only aim motion");
    bluepad32_input_backend_report_sent(0);
    platform_on_controller_data(&left, &left_data);
    require(slot_snapshot(0).state.motion_sample_count == 0,
            "left reports must not replay the last right motion sample");

    uni_hid_device_t ordinary = device(2);
    platform_on_device_connected(&ordinary);
    require(platform_on_device_ready(&ordinary) == UNI_ERROR_SUCCESS,
            "ordinary device must coexist with a pair");
    uni_controller_t ordinary_data{};
    ordinary_data.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    ordinary_data.gamepad.buttons = BUTTON_Y;
    platform_on_controller_data(&ordinary, &ordinary_data);
    const Bluepad32SlotSnapshot ordinary_before = slot_snapshot(2);
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{51, 61});
    bluepad32_input_backend_queue_rumble(2, ControllerRumbleOutput{71, 81});
    process_rumble_timer(&g_rumble_timer);
    require(left.last_low == 51 && right.last_low == 51 &&
                left.last_high == 61 && right.last_high == 61 &&
                ordinary.last_low == 71 && ordinary.last_high == 81,
            "pair feedback must fan out without reaching an ordinary player");
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{});
    process_rumble_timer(&g_rumble_timer);
    require(left.last_rumble_duration_ms == 0 &&
                right.last_rumble_duration_ms == 0 && ordinary.last_low == 71,
            "pair stop must stop both physical motors only");
    bluepad32_input_backend_queue_profile_feedback(
        0, merged.connection_generation, 2,
        ControllerProfileConfirmationPolicy::kRumbleAndLed);
    process_rumble_timer(&g_rumble_timer);
    require(left.player_leds == 3 && right.player_leds == 3 &&
                left.last_low == UINT8_MAX && right.last_low == UINT8_MAX,
            "profile feedback must light and rumble both halves");
    now_ms += 300;
    process_rumble_timer(&g_rumble_timer);
    require(left.player_leds == 1 && right.player_leds == 1,
            "profile completion must restore both halves' player indication");

    uni_hid_device_t& lost = right_first ? left : right;
    uni_hid_device_t& survivor = right_first ? right : left;
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{91, 101});
    platform_on_device_disconnected(&lost);
    const Bluepad32SlotSnapshot detached = slot_snapshot(0);
    require(detached.active && !slot_snapshot(1).active &&
                detached.connection_generation != merged.connection_generation &&
                controller_identity_equal(detached.identity, identity_for_device(&survivor)) &&
                detached.state.extra_buttons == survivor.switch2_extra_buttons &&
                detached.state.motion_sample_count == 0 &&
                !detached.state.dpad_up && !detached.state.button_east &&
                (right_first ? detached.state.button_south : detached.state.button_west),
            "either half detach must immediately publish only the rotated survivor and own profile");
    require_active_profile(detached.identity, right_first ? 5 : 2,
                           right_first ? UINT8_MAX : 37);
    require(!bluepad32_input_backend_identify(merged.identity),
            "an offline pair bank must not identify its surviving solo member");
    bluepad32_input_backend_playtest_snapshot(0, &playtest);
    require(playtest.controller_layout ==
                (right_first ? Bluepad32ControllerLayout::kJoyCon2RightSolo
                             : Bluepad32ControllerLayout::kJoyCon2LeftSolo),
            "playtest must stop presenting a pair immediately after companion loss");
    require(survivor.last_rumble_duration_ms == 0 &&
                slot_snapshot(2).connection_generation == ordinary_before.connection_generation &&
                slot_snapshot(2).state.button_north,
            "detach must cancel survivor feedback without changing unrelated player state");
    const int survivor_calls = survivor.rumble_calls;
    platform_on_controller_data(&lost, right_first ? &left_data : &right_data);
    require(platform_on_device_ready(&lost) == UNI_ERROR_NO_SLOTS,
            "late ready for a detached half must not resurrect its old generation");
    bluepad32_input_backend_queue_profile_feedback(
        0, merged.connection_generation, 8,
        ControllerProfileConfirmationPolicy::kRumbleAndLed);
    process_rumble_timer(&g_rumble_timer);
    require(survivor.rumble_calls == survivor_calls &&
                slot_snapshot(0).state.extra_buttons == survivor.switch2_extra_buttons,
            "late detached input and generation-bound feedback must be ignored");
    uni_hid_device_t replacement = switch2_device(lost.idx, lost.product_id);
    memcpy(replacement.conn.btaddr, lost.conn.btaddr, sizeof(bd_addr_t));
    replacement.switch2_identity_address_type = lost.switch2_identity_address_type;
    left_profile.weak_rumble_scale = 61;
    require(runtime_profile_storage.set(left_identity, 2, left_profile) ==
                ProfileStorageResult::kOk,
            "changing the source solo must remain independent while detached");
    ready_switch2(replacement);
    platform_on_device_disconnected(&lost);
    require(slot_snapshot(0).active && !slot_snapshot(1).active &&
                slot_snapshot(0).state.extra_buttons == survivor.switch2_extra_buttons &&
                slot_snapshot(0).connection_generation != detached.connection_generation,
            "replacement must re-pair without stale presses or a late old disconnect");
    require(controller_identity_equal(slot_snapshot(0).identity, merged.identity),
            "rejoining the same typed physical members must restore the same pair owner");
    require_active_profile(slot_snapshot(0).identity, 7, 95);
    const uint32_t clear_token = bluepad32_input_backend_clear_pairings();
    process_rumble_timer(&g_rumble_timer);
    Bluepad32PairingSnapshot cleared{};
    bluepad32_input_backend_pairing_snapshot(&cleared);
    require(bluepad32_input_backend_clear_pairings_completed(cleared, clear_token) &&
                device_disconnect_calls == 3 &&
                !slot_snapshot(0).active && !slot_snapshot(2).active &&
                !switch_pico_switch2_pairing_allowed(),
            "clear must disconnect every physical half and ordinary device, not just outputs");
}

void test_switch2_multiple_pairs() {
    start_pairing_backend();
    uni_hid_device_t right0 = switch2_device(0, UNI_SW2_JOYCON_R_PID);
    uni_hid_device_t right1 = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    uni_hid_device_t left0 = switch2_device(2, UNI_SW2_JOYCON_L_PID);
    uni_hid_device_t left1 = switch2_device(3, UNI_SW2_JOYCON_L_PID);
    ready_switch2(right0);
    ready_switch2(right1);
    ready_switch2(left0);
    ready_switch2(left1);
    require(slot_snapshot(0).active && slot_snapshot(1).active &&
                !slot_snapshot(2).active && !slot_snapshot(3).active &&
                g_connection_policy_state == ConnectionPolicyState::Paused &&
                !scanning_enabled && !incoming_connections,
            "two deterministic pairs must consume four physical resources but only two outputs");
    require_pair_owner(slot_snapshot(0).identity, left0, right0);
    require_pair_owner(slot_snapshot(1).identity, left1, right1);
    uni_controller_t data{};
    data.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    data.gamepad.buttons = BUTTON_A;
    right0.switch2_extra_buttons = UNI_SW2_BUTTON_C;
    platform_on_controller_data(&right0, &data);
    data.gamepad.buttons = BUTTON_Y;
    right1.switch2_extra_buttons = UNI_SW2_BUTTON_GR;
    platform_on_controller_data(&right1, &data);
    require(slot_snapshot(0).state.button_south && !slot_snapshot(0).state.button_north &&
                slot_snapshot(0).state.extra_buttons == UNI_SW2_BUTTON_C &&
                slot_snapshot(1).state.button_north && !slot_snapshot(1).state.button_south &&
                slot_snapshot(1).state.extra_buttons == UNI_SW2_BUTTON_GR,
            "two pairs must not cross-contaminate normal or extra source input");
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{11, 21});
    bluepad32_input_backend_queue_rumble(1, ControllerRumbleOutput{31, 41});
    process_rumble_timer(&g_rumble_timer);
    require(left0.last_low == 11 && right0.last_low == 11 &&
                left1.last_low == 31 && right1.last_low == 31 &&
                left0.player_leds == 1 && right0.player_leds == 1 &&
                left1.player_leds == 2 && right1.player_leds == 2,
            "multiple pairs must retain isolated rumble and player lighting");
    const Bluepad32SlotSnapshot other_pair = slot_snapshot(1);
    platform_on_device_disconnected(&right0);
    uni_hid_device_t ordinary = device(0);
    platform_on_device_connected(&ordinary);
    require(platform_on_device_ready(&ordinary) == UNI_ERROR_SUCCESS &&
                slot_snapshot(0).active && slot_snapshot(2).active &&
                !slot_snapshot(3).active &&
                controller_identity_equal(slot_snapshot(0).identity, identity_for_device(&left0)),
            "reusing freed physical index must allocate a free output, not evict the surviving half");
    data.gamepad.buttons = BUTTON_B;
    platform_on_controller_data(&ordinary, &data);
    bluepad32_input_backend_queue_rumble(2, ControllerRumbleOutput{91, 101});
    process_rumble_timer(&g_rumble_timer);
    require(slot_snapshot(2).state.button_east && !slot_snapshot(0).state.button_east &&
                ordinary.last_low == 91 && left1.last_low == 31 && right1.last_low == 31 &&
                slot_snapshot(1).connection_generation == other_pair.connection_generation &&
                slot_snapshot(1).state.button_north,
            "remapped ordinary physical index must isolate input and feedback from both pairs");
}

void test_switch2_pair_admission_failure(bool right_first) {
    start_pairing_backend();
    initialize_runtime_profile_storage();
    auto left = switch2_device(right_first ? 1 : 0, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(right_first ? 0 : 1, UNI_SW2_JOYCON_R_PID);
    auto& first = right_first ? right : left;
    auto& second = right_first ? left : right;
    ready_switch2(first);
    const ControllerIdentity solo_identity = identity_for_device(&first);
    require(runtime_profile_storage.activate(solo_identity, 3) == ProfileStorageResult::kOk,
            "the first solo must have an existing independent active profile");
    uni_controller_t data{};
    data.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    data.gamepad.buttons = BUTTON_TRIGGER_L | BUTTON_TRIGGER_R;
    platform_on_controller_data(&first, &data);
    const auto solo = slot_snapshot(0);
    require(bluepad32_input_backend_capture_start(
                0, solo.connection_generation, CaptureOptions{}),
            "the first solo must remain recordable during pair admission");
    bluepad32_input_backend_queue_profile_feedback(
        0, solo.connection_generation, 2, ControllerProfileConfirmationPolicy::kRumble);
    const int initial_rumble_calls = first.rumble_calls;
    platform_on_device_connected(&second);
    second.switch2_identity_valid = false;
    require(platform_on_device_ready(&second) == UNI_ERROR_INIT_FAILED &&
                pair_observation_count == 0 &&
                first.rumble_calls == initial_rumble_calls,
            "an unstable member must reject pairing before storage or solo output changes");
    second.switch2_identity_valid = true;
    first.switch2_identity_valid = false;
    require(platform_on_device_ready(&second) == UNI_ERROR_INIT_FAILED &&
                pair_observation_count == 0 &&
                controller_identity_equal(slot_snapshot(0).identity, solo_identity),
            "an unresolved first-ready member must not merge using its stale solo owner");
    first.switch2_identity_valid = true;
    require(runtime_profile_storage.ensure_identity(identity_for_device(&second)) ==
                ProfileStorageResult::kOk,
            "preexisting member rows must allow exercising failure of the actual pair seed");
    before_pair_seed = [](const ControllerIdentity&) {
        const auto first_solo = slot_snapshot(0);
        Bluepad32CaptureSnapshot capture{};
        require(first_solo.active &&
                    first_solo.identity.transport == ControllerTransport::kBle &&
                    !slot_snapshot(1).active &&
                    bluepad32_input_backend_capture_page(0, 0, &capture) &&
                    capture.state == CaptureState::kRecording,
                "persistent pair setup must precede any visible solo topology change");
    };
    fail_profile_program = true;
    require(platform_on_device_ready(&second) == UNI_ERROR_INIT_FAILED &&
                pair_observation_count == 1,
            "pair seed I/O failure must explicitly reject the second ready callback");
    const auto rejected = slot_snapshot(0);
    require(rejected.active && !slot_snapshot(1).active &&
                rejected.connection_generation == solo.connection_generation &&
                controller_identity_equal(rejected.identity, solo_identity) &&
                rejected.state.left_trigger == solo.state.left_trigger &&
                first.rumble_calls == initial_rumble_calls &&
                first.player_leds == 1 && second.player_leds == 0,
            "seed failure must leave the first solo's owner, input, generation and outputs intact");
    ControllerIdentity pair_identity{};
    require(controller_identity_make_joycon_pair(
                identity_for_device(&left), identity_for_device(&right), &pair_identity) &&
                runtime_profile_storage.find(pair_identity) == nullptr,
            "a failed pair seed must not expose a partial persistent owner");
    process_rumble_timer(&g_rumble_timer);
    require(first.last_low == UINT8_MAX && second.rumble_calls == 0,
            "a rejected merge must retain the first solo's queued generation-bound feedback");
    fail_profile_program = false;
    require(platform_on_device_ready(&second) == UNI_ERROR_INIT_FAILED,
            "a retry before catalog replay must not expose a failed seed");
    require(runtime_profile_storage.initialize(runtime_profile_io()),
            "storage replay must recover intact member banks after admission failure");
    require_active_profile(solo_identity, 3, UINT8_MAX);
    require(platform_on_device_ready(&second) == UNI_ERROR_SUCCESS,
            "a replayed catalog must allow the pending physical member to merge on retry");
    before_pair_seed = nullptr;
    require_pair_owner(slot_snapshot(0).identity, left, right);
    require_active_profile(slot_snapshot(0).identity, right_first ? 0 : 3, UINT8_MAX);
}

void test_switch2_pair_member_replacement() {
    start_pairing_backend();
    auto left = switch2_device(0, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    right.conn.btaddr[0] = 0xc2;
    ready_switch2(left);
    ready_switch2(right);
    const ControllerIdentity original = slot_snapshot(0).identity;
    require(runtime_profile_storage.activate(original, 6) == ProfileStorageResult::kOk,
            "the original physical pair must have a distinct saved selection");
    platform_on_device_disconnected(&right);
    auto replacement = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    replacement.conn.btaddr[5] = 0x35;
    ready_switch2(replacement);
    const ControllerIdentity replaced = slot_snapshot(0).identity;
    require_pair_owner(replaced, left, replacement);
    require(!controller_identity_equal(original, replaced) &&
                !bluepad32_input_backend_identify(original),
            "swapping only the right member must select a different pair bank and owner");
    require_active_profile(replaced, 0, UINT8_MAX);
    require(runtime_profile_storage.activate(replaced, 4) == ProfileStorageResult::kOk,
            "the replacement pair bank must be independently selectable");
    platform_on_device_disconnected(&replacement);
    ready_switch2(right);
    require(controller_identity_equal(slot_snapshot(0).identity, original),
            "restoring the original right member must restore the original pair key");
    require_active_profile(slot_snapshot(0).identity, 6, UINT8_MAX);
    require_active_profile(replaced, 4, UINT8_MAX);
    platform_on_device_disconnected(&right);
    auto typed_right = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    memcpy(typed_right.conn.btaddr, right.conn.btaddr, sizeof(bd_addr_t));
    typed_right.switch2_identity_address_type = BD_ADDR_TYPE_LE_RANDOM;
    ready_switch2(typed_right);
    const ControllerIdentity retyped = slot_snapshot(0).identity;
    require_pair_owner(retyped, left, typed_right);
    require(!controller_identity_equal(retyped, original),
            "a member's address type must participate in the pair key even with the same MAC");
    require_active_profile(retyped, 0, UINT8_MAX);
    require_active_profile(original, 6, UINT8_MAX);
}

void test_switch2_admission() {
    start_backend();
    require(!switch_pico_switch2_pairing_allowed(),
            "idle scanning must not grant fresh proprietary pairing");
    bluepad32_input_backend_open_pairing_window();
    require(!switch_pico_switch2_pairing_allowed(),
            "pending Core0 request must not grant pairing before policy consumes it");
    process_rumble_timer(&g_rumble_timer);
    require(switch_pico_switch2_pairing_allowed(),
            "consumed explicit pairing window must permit proprietary pairing");
    now_ms = g_pairing_window_deadline_ms;
    require(!switch_pico_switch2_pairing_allowed(),
            "fresh pairing must close at its deadline even before timer processing");
    uni_hid_device_t pro = switch2_device(0, UNI_SW2_PRO_PID);
    uni_hid_device_t ordinary = device(1, true, UNI_BT_CONN_PROTOCOL_BLE);
    register_lookup_device(&pro);
    register_lookup_device(&ordinary);
    __wrap_sm_request_pairing(pro.conn.handle);
    require(device_disconnect_calls == 1 && last_disconnected_device == &pro &&
                ordinary_smp_requests == 0 && delete_key_calls == 0,
            "Switch2 GATT auth failure must disconnect without SMP or bond deletion");
    __wrap_sm_request_pairing(ordinary.conn.handle);
    __wrap_sm_request_pairing(0x99);
    require(ordinary_smp_requests == 2 && device_disconnect_calls == 1,
            "ordinary and unknown handles must preserve standard SMP behavior");
    require(identity_for_device(&pro).stable &&
                controller_identity_is_global(identity_for_device(&ordinary)),
            "only parser-validated proprietary public identity may bypass SMP resolution");
    pro.switch2_identity_address_type = BD_ADDR_TYPE_LE_RANDOM;
    pro.conn.btaddr[0] = 0xc1;
    require(identity_for_device(&pro).stable &&
                identity_for_device(&pro).address_type == BD_ADDR_TYPE_LE_RANDOM,
            "parser-validated static random identity must retain its address type");
    pro.switch2_identity_valid = false;
    pro.conn.btaddr[0] = 0x41;
    require(controller_identity_is_global(identity_for_device(&pro)),
            "unvalidated proprietary RPA must remain on the global profile");
    pro.switch2_identity_valid = true;
    pro.conn.btaddr[0] = 0xc1;
    ready_switch2(pro);
    uni_controller_t input{};
    input.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    input.gamepad.buttons = BUTTON_B;
    input.gamepad.axis_x = 511;
    input.gamepad.axis_ry = -512;
    pro.switch2_extra_buttons =
        UNI_SW2_BUTTON_C | UNI_SW2_BUTTON_GL | UNI_SW2_BUTTON_GR;
    platform_on_controller_data(&pro, &input);
    require(slot_snapshot(0).state.button_east &&
                slot_snapshot(0).state.left_stick_x == INT16_MAX &&
                slot_snapshot(0).state.right_stick_y == INT16_MIN &&
                slot_snapshot(0).state.extra_buttons == 7,
            "Pro2 must preserve vertical normal controls and ingest remappable extras");
}

void test_switch2_radio_policy() {
    start_backend();
    auto left = switch2_device(0, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    auto classic = device(2, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    auto other_ble = device(3, true, UNI_BT_CONN_PROTOCOL_BLE);
    ready_switch2(left);
    ready_switch2(right);
    platform_on_device_connected(&other_ble);
    require(platform_on_device_ready(&other_ble) == UNI_ERROR_SUCCESS,
            "unrelated BLE controller must join");
    require(negotiated_intervals[left.conn.handle] == 6 &&
                negotiated_intervals[right.conn.handle] == 6,
            "a pair without Classic must retain fast intervals");
    platform_on_device_connected(&classic);
    require(negotiated_intervals[left.conn.handle] == 24 &&
                negotiated_intervals[right.conn.handle] == 24 &&
                negotiated_intervals[other_ble.conn.handle] == 6,
            "Classic setup must relax both physical halves before native attachment, not unrelated BLE");
    require(platform_on_device_ready(&classic) == UNI_ERROR_SUCCESS,
            "Classic controller must complete setup alongside the pair");
    const auto pair = slot_snapshot(0);
    platform_on_device_disconnected(&classic);
    require(negotiated_intervals[left.conn.handle] == 6 &&
                negotiated_intervals[right.conn.handle] == 6 &&
                slot_snapshot(0).connection_generation == pair.connection_generation,
            "Classic departure must restore fast intervals without rebinding the pair");
    platform_on_device_connected(&classic);
    require(platform_on_device_ready(&classic) == UNI_ERROR_SUCCESS,
            "Classic reconnect must complete");
    platform_on_device_disconnected(&left);
    require(negotiated_intervals[right.conn.handle] == 6,
            "a single surviving Switch2 link must return to fast scheduling even with Classic");
    left = switch2_device(0, UNI_SW2_JOYCON_L_PID);
    ready_switch2(left);
    require(negotiated_intervals[left.conn.handle] == 24 &&
                negotiated_intervals[right.conn.handle] == 24,
            "physical index and handle reuse must negotiate mixed intervals on the new connection");
}

void test_switch2_radio_settling() {
    start_backend();
    auto classic = device(2, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    auto left = switch2_device(0, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    platform_on_device_connected(&classic);
    require(platform_on_device_ready(&classic) == UNI_ERROR_SUCCESS,
            "Classic-first connection must become ready");
    ready_switch2(left);
    defer_interval_updates = true;
    now_ms = UINT32_MAX - 10;
    platform_on_device_connected(&right);
    require(interval_requests[right.conn.handle] == 0,
            "pending Switch2 setup must retain ownership of its initial interval");
    require(platform_on_device_ready(&right) == UNI_ERROR_SUCCESS,
            "second Switch2 connection must become ready");
    platform_on_device_disconnected(&classic);
    // The controller completes the old update after the topology reversed.
    for (auto* half : {&left, &right}) {
        negotiated_intervals[half->conn.handle] = pending_intervals[half->conn.handle];
    }
    now_ms += 50;
    process_configuration_timer(&g_configuration_timer);
    for (auto* half : {&left, &right}) {
        negotiated_intervals[half->conn.handle] = pending_intervals[half->conn.handle];
        require(negotiated_intervals[half->conn.handle] == 6,
                "late mixed-mode completion must not strand a pair at slow intervals across clock wrap");
    }
    process_configuration_timer(&g_configuration_timer);
    defer_interval_updates = false;
    reject_interval_updates = true;
    platform_on_device_connected(&classic);
    const unsigned attempts = interval_requests[left.conn.handle];
    now_ms += 999;
    process_configuration_timer(&g_configuration_timer);
    require(interval_requests[left.conn.handle] == attempts,
            "rejected negotiation must not flood the HCI command queue");
    reject_interval_updates = false;
    ++now_ms;
    process_configuration_timer(&g_configuration_timer);
    require(negotiated_intervals[left.conn.handle] == 24 &&
                negotiated_intervals[right.conn.handle] == 24,
            "transiently rejected negotiation must recover without reconnect or pairing");
}

void test_switch2_mate_reconnect() {
    start_backend();
    auto right = switch2_device(0, UNI_SW2_JOYCON_R_PID);
    auto left = switch2_device(1, UNI_SW2_JOYCON_L_PID);
    left.conn.btaddr[0] = 0xc1;
    left.switch2_identity_address_type = BD_ADDR_TYPE_LE_RANDOM;
    remember_switch2(right);
    remember_switch2(left);
    register_lookup_device(&right);
    register_lookup_device(&left);
    ready_switch2(right);
    require(scanning_enabled && background_scan_parameters &&
                !classic_scanning_enabled && incoming_connections &&
                !bondable && accepted_stk_methods == 0 &&
                !switch_pico_switch2_pairing_allowed(),
            "first remembered half must seek its mate with low-duty BLE and authentication closed");
    dispatch_pairing_event(HCI_EVENT_USER_CONFIRMATION_REQUEST);
    dispatch_pairing_event(HCI_EVENT_USER_PASSKEY_REQUEST);
    require(confirmation_accepts == 0 && passkey_accepts == 0 &&
                confirmation_rejections == 1 && passkey_rejections == 1,
            "mate scanning must not authorize new Classic authentication");

    bd_addr_t missing = {1, 2, 3, 4, 5, 6};
    require(platform_on_device_discovered(missing, "Joy-Con 2 (L)", 0, 0) ==
                UNI_ERROR_IGNORE_DEVICE,
            "a device name without a prepared parser candidate cannot authorize passive discovery");
    const auto reject_left = [&]() {
        require(platform_on_device_discovered(left.conn.btaddr, "Joy-Con 2 (L)", 0, 0) ==
                    UNI_ERROR_IGNORE_DEVICE &&
                    scanning_enabled && !classic_scanning_enabled &&
                    !switch_pico_switch2_pairing_allowed(),
                "rejected mate candidates must leave the bounded passive policy unchanged");
    };
    ++left.conn.btaddr[5];
    reject_left();
    --left.conn.btaddr[5];
    left.product_id = UNI_SW2_JOYCON_R_PID;
    reject_left();
    left.product_id = UNI_SW2_PRO_PID;
    reject_left();
    left.product_id = UNI_SW2_JOYCON_L_PID;
    left.conn.protocol = UNI_BT_CONN_PROTOCOL_BR_EDR;
    reject_left();
    left.conn.protocol = UNI_BT_CONN_PROTOCOL_BLE;
    ++left.vendor_id;
    reject_left();
    --left.vendor_id;
    left.switch2_identity_valid = false;
    reject_left();
    left.switch2_identity_valid = true;
    left.switch2_identity_address_type = BD_ADDR_TYPE_LE_PUBLIC;
    reject_left();
    left.switch2_identity_address_type = BD_ADDR_TYPE_LE_RANDOM;
    require(platform_on_device_discovered(left.conn.btaddr, nullptr, 0, 0) ==
                UNI_ERROR_SUCCESS,
            "remembered opposite static-random half must be admitted outside the pairing window");

    // The parser stops GAP scan before it has a backend slot to reserve.
    uni_bt_le_scan_stop();
    platform_on_device_connected(&left);
    require(!scanning_enabled && !classic_scanning_enabled,
            "remembered mate setup must pause scan even when GAP already stopped it");
    platform_on_device_disconnected(&left);
    require(scanning_enabled && background_scan_parameters && !classic_scanning_enabled,
            "failed setup must resume remembered mate scanning");
    uni_bt_le_scan_stop();
    platform_on_device_connected(&left);
    require(!scanning_enabled && !classic_scanning_enabled && incoming_connections,
            "mate setup must pause background scanning while reserving physical capacity");
    require(platform_on_device_ready(&left) == UNI_ERROR_SUCCESS &&
                slot_snapshot(0).active && !slot_snapshot(1).active &&
                !scanning_enabled && !classic_scanning_enabled,
            "ready remembered mate must merge without a pairing window and stop background scanning");
    require_pair_owner(slot_snapshot(0).identity, left, right);
    auto ordinary = device(2, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    platform_on_device_connected(&ordinary);
    require(platform_on_device_ready(&ordinary) == UNI_ERROR_SUCCESS &&
                !scanning_enabled && !classic_scanning_enabled,
            "a complete pair plus incoming Classic controller must not keep LE scanning");
    platform_on_device_disconnected(&left);
    require(scanning_enabled && background_scan_parameters &&
                !classic_scanning_enabled && slot_snapshot(2).active,
            "losing a half must resume its mate scan without disturbing an ordinary controller");
}

void test_switch2_mate_pending() {
    start_backend();
    auto left = switch2_device(0, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(3, UNI_SW2_JOYCON_R_PID);
    remember_switch2(left);
    remember_switch2(right);
    register_lookup_device(&right);
    ready_switch2(left);
    auto first = device(1, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    auto second = device(2, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    auto third = device(3, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    platform_on_device_connected(&first);
    platform_on_device_connected(&second);
    require(!scanning_enabled && !classic_scanning_enabled && incoming_connections &&
                platform_on_device_discovered(right.conn.btaddr, nullptr, 0, 0) ==
                    UNI_ERROR_IGNORE_DEVICE,
            "any pending setup must pause scanning and reject otherwise valid mates");
    require(platform_on_device_ready(&first) == UNI_ERROR_SUCCESS &&
                !scanning_enabled,
            "one resolved setup must not resume mate scanning while another is pending");
    platform_on_device_disconnected(&second);
    require(scanning_enabled && background_scan_parameters &&
                !classic_scanning_enabled &&
                platform_on_device_discovered(right.conn.btaddr, nullptr, 0, 0) ==
                    UNI_ERROR_SUCCESS,
            "last pending setup failure must resume passive mate discovery and admission");
    platform_on_device_connected(&second);
    require(!scanning_enabled &&
                platform_on_device_ready(&second) == UNI_ERROR_SUCCESS &&
                scanning_enabled && !classic_scanning_enabled,
            "last pending setup becoming ready must resume the unmatched half scan");
    platform_on_device_connected(&third);
    require(platform_on_device_ready(&third) == UNI_ERROR_SUCCESS &&
                !scanning_enabled && !classic_scanning_enabled && !incoming_connections &&
                platform_on_device_discovered(right.conn.btaddr, nullptr, 0, 0) ==
                    UNI_ERROR_IGNORE_DEVICE,
            "four physical controllers must stop mate scanning even with an unmatched half");
    platform_on_device_disconnected(&third);
    require(scanning_enabled && background_scan_parameters && incoming_connections,
            "free physical capacity must restore the waiting half scan");

    switch2_clear_succeeds = false;
    bluepad32_input_backend_clear_pairings();
    process_rumble_timer(&g_rumble_timer);
    require(!scanning_enabled && !classic_scanning_enabled && !incoming_connections,
            "failed trust deletion must explicitly stop a direct background LE scan");
    platform_on_device_disconnected(&right);
    platform_on_device_disconnected(&left);
    bluepad32_input_backend_open_pairing_window();
    process_rumble_timer(&g_rumble_timer);
    require(!scanning_enabled && !classic_scanning_enabled && !incoming_connections &&
                !switch_pico_switch2_pairing_allowed() &&
                platform_on_device_discovered(right.conn.btaddr, nullptr, 0, 0) ==
                    UNI_ERROR_IGNORE_DEVICE,
            "late disconnects and pairing requests must not clear the failed-closed latch");
}

void test_switch2_mate_pairing_window() {
    start_backend();
    auto left = switch2_device(0, UNI_SW2_JOYCON_L_PID);
    ready_switch2(left);
    require(scanning_enabled && background_scan_parameters && !classic_scanning_enabled,
            "solo half must enter background discovery before opening pairing");
    bluepad32_input_backend_open_pairing_window();
    process_rumble_timer(&g_rumble_timer);
    require(scanning_enabled && !background_scan_parameters && classic_scanning_enabled &&
                bondable && accepted_stk_methods == kAllBlePairingMethods,
            "explicit pairing must restore full discovery timing and the existing authentication window");
    now_ms = g_pairing_window_deadline_ms;
    process_rumble_timer(&g_rumble_timer);
    require(scanning_enabled && background_scan_parameters && !classic_scanning_enabled &&
                !bondable && accepted_stk_methods == 0 &&
                !switch_pico_switch2_pairing_allowed(),
            "pairing expiry must restore low-duty mate discovery without extending authentication");
    platform_on_device_disconnected(&left);
    require(scanning_enabled && !background_scan_parameters && classic_scanning_enabled &&
                !bondable && accepted_stk_methods == 0,
            "last controller loss must restore idle discovery defaults without opening pairing");
}

void test_switch2_pairing_inventory() {
    start_pairing_backend();
    switch2_pairing_count = 2;
    switch2_pairing_types[0] = BD_ADDR_TYPE_LE_PUBLIC;
    switch2_pairing_types[1] = BD_ADDR_TYPE_LE_RANDOM;
    switch2_pairings[0][5] = 1;
    switch2_pairings[1][0] = 0xc1;
    switch2_pairings[1][5] = 2;
    bluepad32_input_backend_request_pairing_snapshot();
    process_rumble_timer(&g_rumble_timer);
    Bluepad32PairingSnapshot snapshot{};
    bluepad32_input_backend_pairing_snapshot(&snapshot);
    require(snapshot.record_count == 2 && !snapshot.overflow &&
                snapshot.records[0].transport == Bluepad32PairingTransport::kBle &&
                snapshot.records[0].address_type == BD_ADDR_TYPE_LE_PUBLIC &&
                snapshot.records[0].address[5] == 1 &&
                snapshot.records[1].address_type == BD_ADDR_TYPE_LE_RANDOM &&
                snapshot.records[1].address[0] == 0xc1,
            "pairing inventory must include proprietary trust records with real BLE address types");
    switch2_pairing_count = UNI_SWITCH2_PAIRING_CAPACITY;
    classic_bond_count = 1;
    bluepad32_input_backend_request_pairing_snapshot();
    process_rumble_timer(&g_rumble_timer);
    bluepad32_input_backend_pairing_snapshot(&snapshot);
    require(snapshot.record_count == BLUEPAD32_PAIRING_RECORD_CAPACITY &&
                snapshot.overflow,
            "combined ordinary and proprietary inventory must preserve bounded overflow reporting");
    const uint32_t successful_token = bluepad32_input_backend_clear_pairings();
    process_rumble_timer(&g_rumble_timer);
    bluepad32_input_backend_pairing_snapshot(&snapshot);
    require(snapshot.record_count == 0 && !snapshot.overflow &&
                switch2_pairing_count == 0 &&
                bluepad32_input_backend_clear_pairings_completed(snapshot, successful_token),
            "clear completion must follow deletion of proprietary and ordinary pairing records");
    switch2_pairing_count = 1;
    switch2_clear_succeeds = false;
    const uint32_t failed_token = bluepad32_input_backend_clear_pairings();
    process_rumble_timer(&g_rumble_timer);
    bluepad32_input_backend_pairing_snapshot(&snapshot);
    require(snapshot.status == Bluepad32PairingSnapshotStatus::kFailed &&
                snapshot.completed_clear_pairings_token == successful_token &&
                !bluepad32_input_backend_clear_pairings_completed(snapshot, failed_token) &&
                !incoming_connections && !scanning_enabled &&
                !switch_pico_switch2_pairing_allowed(),
            "failed trust deletion must never acknowledge clear or admit reconnects");
    bluepad32_input_backend_open_pairing_window();
    bluepad32_input_backend_request_pairing_snapshot();
    process_rumble_timer(&g_rumble_timer);
    bluepad32_input_backend_pairing_snapshot(&snapshot);
    require(snapshot.status == Bluepad32PairingSnapshotStatus::kFailed &&
                snapshot.completed_clear_pairings_token == successful_token &&
                !switch_pico_switch2_pairing_allowed() &&
                g_connection_policy_state == ConnectionPolicyState::FailedClosed,
            "inventory refresh and pairing requests must not erase failed-clear state");
    switch2_clear_succeeds = true;
    const uint32_t retry_token = bluepad32_input_backend_clear_pairings();
    require(retry_token != failed_token, "explicit retry must receive a fresh clear token");
    process_rumble_timer(&g_rumble_timer);
    bluepad32_input_backend_pairing_snapshot(&snapshot);
    require(snapshot.status == Bluepad32PairingSnapshotStatus::kReady &&
                snapshot.record_count == 0 &&
                bluepad32_input_backend_clear_pairings_completed(snapshot, retry_token) &&
                incoming_connections && scanning_enabled &&
                !switch_pico_switch2_pairing_allowed(),
            "successful explicit retry must restore normal reconnect policy, not fresh pairing");
}

void test_ready_order(bool reverse) {
    start_pairing_backend();
    uni_hid_device_t devices[kSlotCount] = {
        device(0), device(1), device(2), device(3)};
    uni_hid_device_t replacements[kSlotCount] = {
        device(0), device(1), device(2), device(3)};
    const int forward[kSlotCount] = {0, 1, 2, 3};
    const int backward[kSlotCount] = {3, 2, 1, 0};
    const int* order = reverse ? backward : forward;

    platform_on_device_connected(&devices[order[0]]);

    for (int position = 0; position < kSlotCount; ++position) {
        const int slot = order[position];
        require(platform_on_device_ready(&devices[slot]) == UNI_ERROR_SUCCESS,
                "ready device must bind to its Bluepad index");

        for (int candidate = 0; candidate < kSlotCount; ++candidate) {
            ControllerState snapshot{};
            bool expected_active = false;
            for (int ready = 0; ready <= position; ++ready) {
                expected_active = expected_active || order[ready] == candidate;
            }
            require(read_controller_state(candidate, &snapshot) ==
                        expected_active,
                    "only ready indexed slots may become active");
        }

        if (position + 1 < kSlotCount) {
            require(scanning_enabled && classic_scanning_enabled,
                    "pairing-window discovery must continue while a physical slot remains free");
            require(incoming_connections,
                    "incoming connections must remain enabled before all slots are ready");
        }
    }

    require(!scanning_enabled && !classic_scanning_enabled,
            "discovery must stop when all four physical slots are full");
    require(!incoming_connections,
            "incoming connections must be disabled only when all slots are full");

    for (int slot = 0; slot < kSlotCount; ++slot) {
        platform_on_device_disconnected(&devices[slot]);
        require(scanning_enabled && classic_scanning_enabled && incoming_connections,
                "disconnecting any slot must resume connection policy");

        for (int candidate = 0; candidate < kSlotCount; ++candidate) {
            ControllerState snapshot{};
            require(read_controller_state(candidate, &snapshot) ==
                        (candidate != slot),
                    "disconnect must preserve every surviving slot");
        }

        require(platform_on_device_ready(&replacements[slot]) ==
                    UNI_ERROR_SUCCESS,
                "replacement must bind to each freed indexed slot");
        require(!scanning_enabled && !incoming_connections,
                "restoring four ready slots must stop connection policy");
        devices[slot] = replacements[slot];
    }

    bd_addr_t address{};
    require(platform_on_device_discovered(address, "extra", 0, 0) ==
                UNI_ERROR_IGNORE_DEVICE,
            "discovery must reject devices while all four slots are occupied");
}

void test_rejections() {
    start_backend();
    uni_hid_device_t non_gamepad = device(0, false);
    uni_hid_device_t out_of_range = device(4);
    uni_hid_device_t slot_zero = device(0);
    uni_hid_device_t collision = device(0);

    require(platform_on_device_ready(&non_gamepad) ==
                UNI_ERROR_INVALID_CONTROLLER,
            "non-gamepad must be rejected");
    require(platform_on_device_ready(&out_of_range) == UNI_ERROR_NO_SLOTS,
            "Bluepad index 4 must be rejected");
    require(platform_on_device_ready(&slot_zero) == UNI_ERROR_SUCCESS,
            "valid device must occupy its indexed slot");
    require(platform_on_device_ready(&collision) == UNI_ERROR_NO_SLOTS,
            "different device cannot replace an occupied slot");
    g_slots[0].identity.stable = true;
    g_slots[0].identity.transport = ControllerTransport::kClassic;
    g_slots[0].identity.address[5] = 1;
    Bluepad32SlotSnapshot identity_snapshot{};
    bluepad32_input_backend_snapshot(0, &identity_snapshot);
    require(bluepad32_input_backend_identify(
                identity_snapshot.identity) &&
                g_slots[0].pending_profile_feedback_count == 1 &&
                !bluepad32_input_backend_identify(
                    controller_identity_global()),
            "Identify did not target only the selected live controller");
    g_slots[0].pending_profile_feedback_count = 0;
    g_slots[0].pending_profile_feedback[0] = {};

    uni_controller_t collision_data{};
    collision_data.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    collision_data.gamepad.buttons = BUTTON_B;
    platform_on_controller_data(&collision, &collision_data);
    ControllerState snapshot{};
    require(read_controller_state(0, &snapshot),
            "occupied slot must stay active");
    require(!snapshot.button_east,
            "mismatched device input must not enter the occupied slot");

    uni_controller_t slot_zero_data{};
    slot_zero_data.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    slot_zero_data.gamepad.accel[0] = 8192;
    platform_on_controller_data(&slot_zero, &slot_zero_data);
    require(read_controller_state(0, &snapshot) &&
                snapshot.motion_sample_count == 3,
            "valid slot input must remain observable");
    slot_zero.controller.battery = 201;
    g_last_snapshot_generation[0] = 0;
    Bluepad32PlaytestSnapshot playtest{};
    bluepad32_input_backend_playtest_snapshot(0, &playtest);
    require(playtest.active && playtest.state_generation != 0 &&
                playtest.state.motion_sample_count == 3 &&
                playtest.battery == 201 &&
                (playtest.capabilities & 0x08u) != 0,
            "playtest snapshot did not expose input capabilities");
    struct LayoutCase {
        uni_controller_subtype_t subtype;
        Bluepad32ControllerLayout layout;
    };
    const LayoutCase layouts[] = {
        {CONTROLLER_SUBTYPE_WIIMOTE_HORIZONTAL, Bluepad32ControllerLayout::kWiiRemote},
        {CONTROLLER_SUBTYPE_WIIMOTE_VERTICAL, Bluepad32ControllerLayout::kWiiRemote},
        {CONTROLLER_SUBTYPE_WIIMOTE_ACCEL, Bluepad32ControllerLayout::kWiiRemote},
        {CONTROLLER_SUBTYPE_WIIMOTE_NUNCHUK, Bluepad32ControllerLayout::kWiiNunchuk},
        {CONTROLLER_SUBTYPE_WIIMOTE_NUNCHUK_ACCEL, Bluepad32ControllerLayout::kWiiNunchuk},
        {CONTROLLER_SUBTYPE_WII_CLASSIC, Bluepad32ControllerLayout::kUnspecified},
        {CONTROLLER_SUBTYPE_WIIUPRO, Bluepad32ControllerLayout::kUnspecified},
        {CONTROLLER_SUBTYPE_WII_BALANCE_BOARD, Bluepad32ControllerLayout::kUnspecified},
        {CONTROLLER_SUBTYPE_WIIMOTE_UDRAW_TABLET, Bluepad32ControllerLayout::kUnspecified},
        {CONTROLLER_SUBTYPE_NONE, Bluepad32ControllerLayout::kUnspecified},
    };
    for (const LayoutCase& expected : layouts) {
        slot_zero.controller_subtype = expected.subtype;
        bluepad32_input_backend_playtest_snapshot(0, &playtest);
        require(playtest.controller_layout == expected.layout &&
                    playtest.state.motion_sample_count == 3,
                "Wii layout must follow the live subtype, not motion or previous extension");
    }
    bluepad32_input_backend_report_sent(0);
    require(read_controller_state(0, &snapshot) &&
                snapshot.motion_sample_count == 3,
            "playtest snapshot consumed report-path motion");
    bluepad32_input_backend_playtest_snapshot(4, &playtest);
    require(!playtest.active && playtest.state_generation == 0,
            "playtest snapshot accepted slot 4");
    require(!read_controller_state(4, &snapshot),
            "public snapshot must reject slot 4");
    bluepad32_input_backend_report_sent(4);
    require(read_controller_state(0, &snapshot) &&
                snapshot.motion_sample_count == 3,
            "slot 4 acknowledgement must not consume slot 0 IMU");
    bluepad32_input_backend_queue_rumble(4, ControllerRumbleOutput{1, 2});
    process_rumble_timer(&g_rumble_timer);
    require(slot_zero.rumble_calls == 0,
            "slot 4 rumble must not reach a valid controller");
}

void test_independent_lifecycle() {
    test_identity_encoding_contract();
    start_pairing_backend();

    uni_hid_device_t invalid_transport =
        device(6, true, UNI_BT_CONN_PROTOCOL_BLE);
    invalid_transport.conn.handle = 0xffff;
    require(controller_identity_is_global(
                identity_for_device(&invalid_transport)),
            "invalid GAP handles must remain on the global identity");
    uni_hid_device_t sco_transport =
        device(7, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    gap_connection_types[sco_transport.conn.handle] = GAP_CONNECTION_SCO;
    require(controller_identity_is_global(
                identity_for_device(&sco_transport)),
            "SCO links must remain on the global identity");

    uni_hid_device_t aborted = device(0);
    const uint32_t aborted_generation = g_slots[0].connection_generation;
    platform_on_device_connected(&aborted);
    require(g_slots[0].device == &aborted && !g_slots[0].active,
            "connected device must remain identifiable while becoming ready");

    platform_on_device_disconnected(&aborted);
    require(g_slots[0].device == nullptr && !g_slots[0].active,
            "pre-ready disconnect must clear its pending slot identity");
    require(g_slots[0].connection_generation == aborted_generation + 1,
            "pre-ready disconnect must invalidate its connection generation");
    require(g_connection_status == ConnectionStatus::Scanning &&
                scanning_enabled && classic_scanning_enabled &&
                incoming_connections,
            "pre-ready disconnect must restart Classic and BLE scans");

    uni_hid_device_t devices[kSlotCount] = {
        device(0, true, UNI_BT_CONN_PROTOCOL_NONE),
        device(1, true, UNI_BT_CONN_PROTOCOL_BR_EDR),
        device(2, true, UNI_BT_CONN_PROTOCOL_NONE),
        device(3, true, UNI_BT_CONN_PROTOCOL_BR_EDR)};
    gap_connection_types[devices[0].conn.handle] = GAP_CONNECTION_ACL;
    gap_connection_types[devices[1].conn.handle] = GAP_CONNECTION_LE;
    gap_connection_types[devices[2].conn.handle] = GAP_CONNECTION_LE;
    gap_connection_types[devices[3].conn.handle] = GAP_CONNECTION_LE;
    const bd_addr_t classic_address =
        {0x10, 0x11, 0x12, 0x13, 0x14, 0x15};
    const bd_addr_t resolved_connection_address =
        {0x41, 0x21, 0x22, 0x23, 0x24, 0x25};
    const bd_addr_t created_connection_address =
        {0x42, 0x31, 0x32, 0x33, 0x34, 0x35};
    const bd_addr_t reencrypted_connection_address =
        {0x43, 0x41, 0x42, 0x43, 0x44, 0x45};
    memcpy(devices[0].conn.btaddr, classic_address,
           sizeof(classic_address));
    memcpy(devices[1].conn.btaddr, resolved_connection_address,
           sizeof(resolved_connection_address));
    memcpy(devices[2].conn.btaddr, created_connection_address,
           sizeof(created_connection_address));
    memcpy(devices[3].conn.btaddr, reencrypted_connection_address,
           sizeof(reencrypted_connection_address));
    require(
        devices[0].conn.protocol == UNI_BT_CONN_PROTOCOL_NONE &&
            gap_get_connection_type(devices[0].conn.handle) ==
                GAP_CONNECTION_ACL &&
            devices[1].conn.protocol == UNI_BT_CONN_PROTOCOL_BR_EDR &&
            gap_get_connection_type(devices[1].conn.handle) ==
                GAP_CONNECTION_LE &&
            devices[2].conn.protocol == UNI_BT_CONN_PROTOCOL_NONE &&
            gap_get_connection_type(devices[2].conn.handle) ==
                GAP_CONNECTION_LE,
        "identity fixtures must expose authoritative GAP transports over "
        "missing or stale cached protocols");
    const bd_addr_t resolved_address =
        {0x20, 0x21, 0x22, 0x23, 0x24, 0x25};
    const bd_addr_t reencrypted_address =
        {0x30, 0x31, 0x32, 0x33, 0x34, 0x35};
    dispatch_identity_event(
        SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED, devices[1],
        BD_ADDR_TYPE_LE_PUBLIC, resolved_address);
    register_lookup_device(&devices[3]);
    dispatch_identity_event(
        SM_EVENT_REENCRYPTION_STARTED, devices[3],
        BD_ADDR_TYPE_LE_RANDOM, reencrypted_address);
    dispatch_identity_event(
        SM_EVENT_REENCRYPTION_COMPLETE, devices[3],
        BD_ADDR_TYPE_LE_RANDOM, reencrypted_address);
    for (int slot = 0; slot < kSlotCount; ++slot) {
        platform_on_device_connected(&devices[slot]);
        require(g_slots[slot].device == &devices[slot] &&
                    !g_slots[slot].active,
                "each pending device must retain its indexed identity");
    }

    const uint32_t first_pending_generation =
        g_slots[0].connection_generation;
    platform_on_device_disconnected(&devices[0]);
    require(g_slots[0].device == nullptr && !g_slots[0].active,
            "pre-ready disconnect must clear only its own pending identity");
    for (int slot = 1; slot < kSlotCount; ++slot) {
        require(g_slots[slot].device == &devices[slot] &&
                    !g_slots[slot].active,
                "pre-ready disconnect must preserve all pending survivors");
    }
    require(g_slots[0].connection_generation ==
                first_pending_generation + 1,
            "pending disconnect beside peers must invalidate its generation");
    require(g_connection_status == ConnectionStatus::Connecting &&
                scanning_enabled && classic_scanning_enabled &&
                incoming_connections,
            "open pairing slot must preserve pending peers and resume scanning");

    for (int slot = 1; slot < kSlotCount; ++slot) {
        require(platform_on_device_ready(&devices[slot]) == UNI_ERROR_SUCCESS,
                "each surviving pending device must still become ready");
    }
    platform_on_device_connected(&devices[0]);
    require(platform_on_device_ready(&devices[0]) == UNI_ERROR_SUCCESS,
            "reconnected slot 0 device must complete all four slots");
    require(g_connection_status == ConnectionStatus::Ready &&
                !scanning_enabled && !incoming_connections,
            "four ready lifecycle devices must stop connection policy");

    Bluepad32SlotSnapshot lifecycle_snapshots[kSlotCount]{};
    uint32_t baseline_connection_generations[kSlotCount]{};
    for (int slot = 0; slot < kSlotCount; ++slot) {
        bluepad32_input_backend_snapshot(
            static_cast<uint8_t>(slot), &lifecycle_snapshots[slot]);
        require(lifecycle_snapshots[slot].active,
                "ready slot snapshot must publish active state");
        baseline_connection_generations[slot] =
            lifecycle_snapshots[slot].connection_generation;
    }
    require_identity(
        lifecycle_snapshots[0].identity, true,
        ControllerTransport::kClassic, BD_ADDR_TYPE_UNKNOWN,
        devices[0].conn.btaddr, devices[0].vendor_id,
        devices[0].product_id,
        "Classic snapshot identity must use the connected device address");
    require_identity(
        lifecycle_snapshots[1].identity, true,
        ControllerTransport::kBle, BD_ADDR_TYPE_LE_PUBLIC,
        resolved_address, devices[1].vendor_id, devices[1].product_id,
        "resolved BLE snapshot must use the stable identity address");
    require(controller_identity_is_global(
                lifecycle_snapshots[2].identity),
            "unresolved BLE snapshot must use the global unstable identity");
    require_identity(
        lifecycle_snapshots[3].identity, true,
        ControllerTransport::kBle, BD_ADDR_TYPE_LE_RANDOM,
        reencrypted_address, devices[3].vendor_id,
        devices[3].product_id,
        "reencrypted BLE snapshot must use the bonded identity address");
    require(observed_profile_identity_count == 3 &&
                controller_identity_equal(
                    observed_profile_identities[0],
                    lifecycle_snapshots[1].identity) &&
                controller_identity_equal(
                    observed_profile_identities[1],
                    lifecycle_snapshots[3].identity) &&
                controller_identity_equal(
                    observed_profile_identities[2],
                    lifecycle_snapshots[0].identity),
            "only stable ready identities must be enrolled for profiles");
    size_t classic_identity_observations = 0;
    for (size_t index = 0; index < observed_profile_identity_count;
         ++index) {
        if (controller_identity_equal(
                observed_profile_identities[index],
                lifecycle_snapshots[0].identity)) {
            ++classic_identity_observations;
        }
    }
    require(classic_identity_observations == 1,
            "active GAP ACL with no cached protocol must enroll its stable "
            "Classic identity exactly once");
    require(baseline_connection_generations[0] ==
                first_pending_generation + 1 &&
                baseline_connection_generations[1] ==
                    baseline_connection_generations[2] &&
                baseline_connection_generations[2] ==
                    baseline_connection_generations[3],
            "connection generations must isolate each slot lifecycle");

    const bd_addr_t created_address =
        {0x40, 0x41, 0x42, 0x43, 0x44, 0x45};
    register_lookup_device(&devices[2]);
    dispatch_identity_event(
        SM_EVENT_IDENTITY_CREATED, devices[2],
        BD_ADDR_TYPE_LE_RANDOM, created_address);
    bluepad32_input_backend_snapshot(2, &lifecycle_snapshots[2]);
    require_identity(
        lifecycle_snapshots[2].identity, true,
        ControllerTransport::kBle, BD_ADDR_TYPE_LE_RANDOM,
        created_address, devices[2].vendor_id, devices[2].product_id,
        "new BLE identity event must update an active slot snapshot");
    require(observed_profile_identity_count == 4 &&
                controller_identity_equal(
                    observed_profile_identities[3],
                    lifecycle_snapshots[2].identity),
            "late BLE identity creation must enroll the stable identity");

    const uint32_t buttons[kSlotCount] = {
        BUTTON_B, BUTTON_A, BUTTON_X, BUTTON_Y};
    uni_controller_t data[kSlotCount]{};
    for (int slot = 0; slot < kSlotCount; ++slot) {
        data[slot].klass = UNI_CONTROLLER_CLASS_GAMEPAD;
        data[slot].gamepad.buttons = buttons[slot];
        data[slot].gamepad.accel[slot % 3] = 8192 + slot;
        data[slot].gamepad.gyro[(slot + 1) % 3] = 1024 + slot;
        platform_on_controller_data(&devices[slot], &data[slot]);
    }

    ControllerState states[kSlotCount]{};
    for (int slot = 0; slot < kSlotCount; ++slot) {
        require(read_controller_state(slot, &states[slot]) &&
                    states[slot].motion_sample_count == 3,
                "every slot must expose independent input and IMU");
    }
    require(states[0].button_east && !states[0].button_south &&
                !states[0].button_west && !states[0].button_north,
            "slot 0 must contain only slot 0 input");
    require(states[1].button_south && !states[1].button_east &&
                !states[1].button_west && !states[1].button_north,
            "slot 1 must contain only slot 1 input");
    require(states[2].button_west && !states[2].button_east &&
                !states[2].button_south && !states[2].button_north,
            "slot 2 must contain only slot 2 input");
    require(states[3].button_north && !states[3].button_east &&
                !states[3].button_south && !states[3].button_west,
            "slot 3 must contain only slot 3 input");

    bluepad32_input_backend_report_sent(3);
    for (int slot = 0; slot < kSlotCount; ++slot) {
        require(read_controller_state(slot, &states[slot]) &&
                    states[slot].motion_sample_count == (slot == 3 ? 0 : 3),
                "slot 3 acknowledgement must not consume slots 0-2 IMU");
    }
    for (int slot = 0; slot < 3; ++slot) {
        bluepad32_input_backend_report_sent(slot);
        require(read_controller_state(slot, &states[slot]) &&
                    states[slot].motion_sample_count == 0,
                "each slot acknowledgement must consume only its own IMU");
    }

    const ControllerRumbleOutput initial_rumble[kSlotCount] = {
        {11, 21}, {12, 22}, {13, 23}, {14, 24}};
    for (int slot = 0; slot < kSlotCount; ++slot) {
        bluepad32_input_backend_queue_rumble(slot, initial_rumble[slot]);
    }
    process_rumble_timer(&g_rumble_timer);
    for (int slot = 0; slot < kSlotCount; ++slot) {
        require(devices[slot].rumble_calls == 1 &&
                    devices[slot].last_low == 11 + slot &&
                    devices[slot].last_high == 21 + slot &&
                    devices[slot].last_rumble_duration_ms ==
                        host_rumble_duration_ms(),
                "each slot rumble must reach only its indexed controller");
    }

    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{0, 0});
    process_rumble_timer(&g_rumble_timer);
    require(devices[0].rumble_calls == 2 &&
                devices[0].last_rumble_duration_ms == 0,
            "zero XInput magnitude must stop rumble immediately");

    bluepad32_input_backend_queue_rumble(3, ControllerRumbleOutput{55, 66});
    const uint32_t disconnected_generation =
        baseline_connection_generations[3];
    platform_on_device_disconnected(&devices[3]);
    require(scanning_enabled && classic_scanning_enabled && incoming_connections,
            "slot 3 disconnect must resume scanning and incoming connections");
    require(!read_controller_state(3, &states[3]) &&
                !states[3].button_north && states[3].left_stick_x == 0,
            "slot 3 disconnect must publish protocol-neutral state");
    bluepad32_input_backend_snapshot(3, &lifecycle_snapshots[3]);
    require(!lifecycle_snapshots[3].active &&
                lifecycle_snapshots[3].connection_generation ==
                    disconnected_generation + 1 &&
                controller_identity_is_global(
                    lifecycle_snapshots[3].identity) &&
                !lifecycle_snapshots[3].state.button_north &&
                lifecycle_snapshots[3].state.left_stick_x == 0,
            "disconnect snapshot must atomically publish neutral state, "
            "cleared identity, and a new connection generation");
    for (int survivor = 0; survivor < 3; ++survivor) {
        bluepad32_input_backend_snapshot(
            static_cast<uint8_t>(survivor),
            &lifecycle_snapshots[survivor]);
        require(lifecycle_snapshots[survivor].active &&
                    lifecycle_snapshots[survivor].connection_generation ==
                        baseline_connection_generations[survivor],
                "disconnect generation must not leak into surviving slots");
    }
    require(read_controller_state(0, &states[0]) &&
                states[0].button_east &&
                read_controller_state(1, &states[1]) &&
                states[1].button_south &&
                read_controller_state(2, &states[2]) &&
                states[2].button_west,
            "slot 3 disconnect must preserve slots 0-2");
    platform_on_controller_data(&devices[0], &data[0]);
    require(read_controller_state(0, &states[0]) &&
                states[0].button_east,
            "slot 0 input must continue while slot 3 is disconnected");
    const int slot_zero_calls_while_scanning = devices[0].rumble_calls;
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{115, 116});
    tick_backend_timer(99);
    require(devices[0].rumble_calls == slot_zero_calls_while_scanning + 1 &&
                devices[0].last_low == 115 &&
                devices[0].last_high == 116,
            "slot 0 rumble must continue while slot 3 is disconnected");

    uni_hid_device_t slot_three_replacement =
        device(3, true, UNI_BT_CONN_PROTOCOL_BLE);
    memcpy(slot_three_replacement.conn.btaddr,
           devices[3].conn.btaddr, sizeof(devices[3].conn.btaddr));
    require(
        slot_three_replacement.conn.handle == devices[3].conn.handle &&
            gap_get_connection_type(slot_three_replacement.conn.handle) ==
                GAP_CONNECTION_LE &&
            memcmp(slot_three_replacement.conn.btaddr,
                   devices[3].conn.btaddr, sizeof(devices[3].conn.btaddr)) ==
                0,
        "replacement isolation fixture must reuse the active BLE handle "
        "and address");
    require(platform_on_device_ready(&slot_three_replacement) ==
                UNI_ERROR_SUCCESS,
            "slot 3 replacement must bind to the freed indexed slot");
    process_rumble_timer(&g_rumble_timer);
    require(slot_three_replacement.rumble_calls == 0,
            "slot 3 replacement must not receive disconnected device rumble");
    bluepad32_input_backend_snapshot(3, &lifecycle_snapshots[3]);
    require(lifecycle_snapshots[3].active &&
                lifecycle_snapshots[3].connection_generation ==
                    disconnected_generation + 1 &&
                controller_identity_is_global(
                    lifecycle_snapshots[3].identity),
            "replacement must keep the new generation and cannot inherit "
            "the disconnected BLE identity");
    require(observed_profile_identity_count == 4,
            "unstable replacement must not be enrolled for profiles");

    g_slots[3].pending_rumble = {
        3, disconnected_generation, ControllerRumbleOutput{77, 88},
        kXInputHostRumbleDurationMs};
    g_slots[3].rumble_pending = true;
    process_rumble_timer(&g_rumble_timer);
    require(slot_three_replacement.rumble_calls == 0,
            "stale slot 3 connection generation must be rejected");

    uni_controller_t replacement_data{};
    replacement_data.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    replacement_data.gamepad.buttons = BUTTON_Y;
    replacement_data.gamepad.accel[0] = 9000;
    platform_on_controller_data(&slot_three_replacement, &replacement_data);
    require(read_controller_state(3, &states[3]) &&
                states[3].button_north && states[3].motion_sample_count == 3,
            "replacement input and IMU must populate only slot 3");
    require(read_controller_state(0, &states[0]) &&
                states[0].button_east &&
                read_controller_state(1, &states[1]) &&
                states[1].button_south &&
                read_controller_state(2, &states[2]) &&
                states[2].button_west,
            "slot 3 replacement must not disturb slots 0-2");

    const int survivor_calls[kSlotCount - 1] = {
        devices[0].rumble_calls,
        devices[1].rumble_calls,
        devices[2].rumble_calls};
    bluepad32_input_backend_queue_rumble(3, ControllerRumbleOutput{90, 91});
    process_rumble_timer(&g_rumble_timer);
    require(slot_three_replacement.rumble_calls == 1 &&
                slot_three_replacement.last_low == 90 &&
                slot_three_replacement.last_high == 91,
            "new-generation slot 3 rumble must reach its replacement");
    for (int slot = 0; slot < 3; ++slot) {
        require(devices[slot].rumble_calls == survivor_calls[slot],
                "slot 3 rumble must not affect slots 0-2");
    }
    const int all_slot_calls[kSlotCount] = {
        devices[0].rumble_calls,
        devices[1].rumble_calls,
        devices[2].rumble_calls,
        slot_three_replacement.rumble_calls};
    for (int slot = 0; slot < kSlotCount; ++slot) {
        bluepad32_input_backend_queue_rumble(
            slot, ControllerRumbleOutput{static_cast<uint8_t>(100 + slot),
                                         static_cast<uint8_t>(110 + slot)});
    }
    process_rumble_timer(&g_rumble_timer);
    for (int slot = 0; slot < kSlotCount; ++slot) {
        const uni_hid_device_t& target =
            slot == 3 ? slot_three_replacement : devices[slot];
        require(target.rumble_calls == all_slot_calls[slot] + 1 &&
                    target.last_low == 100 + slot &&
                    target.last_high == 110 + slot,
                "survivor rumble must continue after slot 3 replacement");
    }

    uni_hid_device_t replacements[kSlotCount] = {
        device(0), device(1), device(2), device(3)};
    for (int slot = 0; slot < 3; ++slot) {
        platform_on_device_disconnected(&devices[slot]);
        require(scanning_enabled && classic_scanning_enabled && incoming_connections,
                "disconnecting slots 0-2 must resume connection policy");
        require(!read_controller_state(slot, &states[slot]) &&
                    states[slot].left_stick_x == 0,
                "disconnect must publish protocol-neutral state");
        for (int survivor = 0; survivor < kSlotCount; ++survivor) {
            if (survivor == slot) {
                continue;
            }
            require(read_controller_state(survivor, &states[survivor]),
                    "disconnect must preserve all three survivors");
        }
        require(platform_on_device_ready(&replacements[slot]) ==
                    UNI_ERROR_SUCCESS,
                "replacement must bind to each freed slot");
        require(g_connection_status == ConnectionStatus::Ready &&
                    !scanning_enabled && !incoming_connections,
                "replacement must restore the full four-slot policy");
    }

    const int slot_zero_calls_before_mailboxes = replacements[0].rumble_calls;
    const int slot_three_calls_before_mailboxes =
        slot_three_replacement.rumble_calls;
    bluepad32_input_backend_queue_rumble(3, ControllerRumbleOutput{119, 120});
    bluepad32_input_backend_queue_rumble(3, ControllerRumbleOutput{121, 122});
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{123, 124});
    process_rumble_timer(&g_rumble_timer);
    require(slot_three_replacement.rumble_calls ==
                    slot_three_calls_before_mailboxes + 1 &&
                slot_three_replacement.last_low == 121 &&
                slot_three_replacement.last_high == 122,
            "slot 3 mailbox must dispatch only its latest queued value");
    require(replacements[0].rumble_calls ==
                    slot_zero_calls_before_mailboxes + 1 &&
                replacements[0].last_low == 123 &&
                replacements[0].last_high == 124,
            "slot 0 activity must not evict the slot 3 mailbox");
}

void test_pairing_window_policy() {
    bd_addr_t address = {1, 2, 3, 4, 5, 6};

    bluepad32_input_backend_init();
    require(platform_on_device_discovered(address, "controller", 0, 0) ==
                    UNI_ERROR_IGNORE_DEVICE,
            "discovery must remain closed before backend initialization");

    start_backend();
    require(g_connection_policy_state == ConnectionPolicyState::Open &&
                classic_scanning_enabled && scanning_enabled &&
                incoming_connections,
            "boot must allow normal Bluepad32 autoconnect");
    require(platform_on_device_discovered(address, "controller", 0, 0) ==
                    UNI_ERROR_SUCCESS,
            "boot policy must accept a discovered controller");
    dispatch_pairing_event(HCI_EVENT_USER_CONFIRMATION_REQUEST);
    dispatch_pairing_event(HCI_EVENT_USER_PASSKEY_REQUEST);
    require(confirmation_rejections == 1 && passkey_rejections == 1 &&
                confirmation_accepts == 0 && passkey_accepts == 0,
            "closed BOOTSEL window must reject new SSP authentication");


    uni_hid_device_t reconnecting = device(0);
    platform_on_device_connected(&reconnecting);
    require(device_disconnect_calls == 0 &&
                g_slots[0].device == &reconnecting,
            "boot policy must retain a reconnecting controller");
    platform_on_device_disconnected(&reconnecting);

    bluepad32_input_backend_open_pairing_window();
    require(!g_pairing_window_open,
            "Core0 request must wait for Core1 consumption");
    process_rumble_timer(&g_rumble_timer);
    require(g_pairing_window_open && bondable &&
                accepted_stk_methods == kAllBlePairingMethods &&
                g_pairing_window_deadline_ms == 60000 &&
                g_connection_policy_state == ConnectionPolicyState::Open &&
                classic_scanning_enabled && scanning_enabled &&
                incoming_connections,
            "BOOTSEL must enable Classic and BLE pairing without interrupting autoconnect");
    dispatch_pairing_event(HCI_EVENT_USER_CONFIRMATION_REQUEST);
    dispatch_pairing_event(HCI_EVENT_USER_PASSKEY_REQUEST);
    require(confirmation_accepts == 1 && passkey_accepts == 1,
            "open BOOTSEL window must accept new SSP authentication");

    tick_backend_timer(20);
    require(!observed_status_led_on,
            "pairing double blink must finish its first pulse");
    tick_backend_timer(20);
    require(observed_status_led_on,
            "pairing double blink must start its second pulse");
    tick_backend_timer(20);
    require(!observed_status_led_on,
            "pairing double blink must finish its second pulse");

    now_ms = 30000;
    bluepad32_input_backend_open_pairing_window();
    process_rumble_timer(&g_rumble_timer);
    require(g_pairing_window_deadline_ms == 90000,
            "pairing request must extend deadline from current Core1 time");

    uni_hid_device_t devices[kSlotCount] = {
        device(0), device(1), device(2), device(3)};
    for (int slot = 0; slot < kSlotCount; ++slot) {
        require(platform_on_device_ready(&devices[slot]) == UNI_ERROR_SUCCESS,
                "policy test devices must fill all slots");
    }
    require(g_connection_policy_state == ConnectionPolicyState::Paused &&
                g_pairing_window_open && !scanning_enabled &&
                !classic_scanning_enabled && !incoming_connections,
            "full slots must pause scanning without closing the deadline");

    now_ms = 90000;
    process_rumble_timer(&g_rumble_timer);
    require(!g_pairing_window_open && !bondable &&
                accepted_stk_methods == 0 &&
                g_connection_policy_state == ConnectionPolicyState::Paused,
            "Classic and BLE pairing authentication must close at the deadline");
    platform_on_device_disconnected(&devices[3]);
    require(g_connection_policy_state == ConnectionPolicyState::Passive &&
                !classic_scanning_enabled && !scanning_enabled &&
                incoming_connections,
            "a freed slot with active controllers must remain passive");
    require(platform_on_device_discovered(address, "controller", 0, 0) ==
                    UNI_ERROR_IGNORE_DEVICE,
            "passive policy must reject inquiry discoveries");

    bluepad32_input_backend_open_pairing_window();
    process_rumble_timer(&g_rumble_timer);
    require(g_connection_policy_state == ConnectionPolicyState::Open &&
                classic_scanning_enabled && scanning_enabled &&
                incoming_connections,
            "explicit BOOTSEL window must resume active discovery");
    require(platform_on_device_discovered(address, "controller", 0, 0) ==
                    UNI_ERROR_SUCCESS,
            "pairing window discovery must accept a controller");
}


void test_slot_lighting() {
    start_pairing_backend();
    uni_hid_device_t devices[kSlotCount] = {
        device(0), device(1), device(2), device(3)};

    for (uint8_t slot = 0; slot < kSlotCount; ++slot) {
        devices[slot].report_parser.set_lightbar_color = set_lightbar;
        devices[slot].report_parser.set_player_leds = set_player_leds;
        require(platform_on_device_ready(&devices[slot]) == UNI_ERROR_SUCCESS,
                "color-capable controller did not become ready");
        const SwitchRgbColor expected =
            switch_pro_get_slot_light_color(slot);
        require(devices[slot].lightbar_calls == 1 &&
                    devices[slot].lightbar_red == expected.red &&
                    devices[slot].lightbar_green == expected.green &&
                    devices[slot].lightbar_blue == expected.blue &&
                    devices[slot].player_led_calls == 0,
                "slot color did not reach the controller lightbar");
        require(platform_on_device_ready(&devices[slot]) == UNI_ERROR_SUCCESS &&
                    devices[slot].lightbar_calls == 1,
                "duplicate ready event rewrote controller lighting");
    }

    platform_on_device_disconnected(&devices[2]);
    uni_hid_device_t fallback = device(2);
    fallback.report_parser.set_player_leds = set_player_leds;
    require(platform_on_device_ready(&fallback) == UNI_ERROR_SUCCESS &&
                fallback.lightbar_calls == 0 &&
                fallback.player_led_calls == 1 &&
                fallback.player_leds == (1u << 2u),
            "controller without RGB support did not receive its slot LED");
}

void test_profile_chord_remains_raw() {
    start_pairing_backend();
    uni_hid_device_t controller = device(0);
    require(platform_on_device_ready(&controller) == UNI_ERROR_SUCCESS,
            "raw profile chord controller did not become ready");

    uni_controller_t input{};
    input.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    input.gamepad.buttons =
        BUTTON_A | BUTTON_SHOULDER_L | BUTTON_SHOULDER_R;
    input.gamepad.misc_buttons =
        MISC_BUTTON_SELECT | MISC_BUTTON_START;
    platform_on_controller_data(&controller, &input);
    ControllerState snapshot{};
    require(read_controller_state(0, &snapshot) &&
                snapshot.button_south && !snapshot.button_east &&
                snapshot.button_left_shoulder &&
                snapshot.button_right_shoulder &&
                snapshot.button_select && snapshot.button_start,
            "Core 1 suppressed the profile chord or mutated ABXY mapping");
    process_rumble_timer(&g_rumble_timer);
    require(controller.rumble_calls == 0 &&
                !g_slots[0].feedback_pending,
            "legacy ABXY chord still produced local feedback");
}

void test_profile_feedback_scheduler() {
    start_pairing_backend();
    uni_hid_device_t devices[kSlotCount] = {
        device(0), device(1), device(2), device(3)};
    devices[0].report_parser.set_lightbar_color = set_lightbar;
    devices[1].report_parser.set_player_leds = set_player_leds;
    devices[2].report_parser.set_lightbar_color = set_lightbar;
    devices[3].report_parser.set_player_leds = set_player_leds;

    uint32_t generations[kSlotCount]{};
    for (uint8_t slot = 0; slot < kSlotCount; ++slot) {
        require(platform_on_device_ready(&devices[slot]) ==
                    UNI_ERROR_SUCCESS,
                "profile feedback controller did not become ready");
        Bluepad32SlotSnapshot snapshot{};
        bluepad32_input_backend_snapshot(slot, &snapshot);
        generations[slot] = snapshot.connection_generation;
        require(devices[slot].rumble_calls == 0 &&
                    !g_slots[slot].profile_feedback.active &&
                    g_slots[slot].pending_profile_feedback_count == 0,
                "initial controller/profile load scheduled confirmation feedback");
        bluepad32_input_backend_queue_profile_feedback(
            slot, generations[slot], static_cast<uint8_t>(slot + 1u),
            ControllerProfileConfirmationPolicy::kRumble);
        bluepad32_input_backend_queue_rumble(
            slot, ControllerRumbleOutput{
                      static_cast<uint8_t>(0x10u + slot),
                      static_cast<uint8_t>(0x20u + slot)});
    }

    now_ms = 0;
    process_rumble_timer(&g_rumble_timer);
    for (uint8_t slot = 0; slot < kSlotCount; ++slot) {
        require(devices[slot].rumble_calls == 1 &&
                    devices[slot].last_rumble_duration_ms ==
                        kProfileFeedbackPhaseDurationMs &&
                    devices[slot].last_high == UINT8_MAX &&
                    devices[slot].last_low == UINT8_MAX &&
                    g_slots[slot].rumble_pending &&
                    g_slots[slot].profile_feedback.active,
                "profile pulse sequence did not start at full strength");
    }

    now_ms = 74;
    process_rumble_timer(&g_rumble_timer);
    now_ms = 75;
    process_rumble_timer(&g_rumble_timer);
    for (const uni_hid_device_t& controller : devices) {
        require(controller.rumble_calls == 1,
                "profile pulse did not retain a 75 ms on phase");
    }

    now_ms = 149;
    process_rumble_timer(&g_rumble_timer);
    now_ms = 150;
    process_rumble_timer(&g_rumble_timer);
    require(devices[0].rumble_calls == 2 &&
                devices[0].last_high == 0x20 &&
                devices[0].last_low == 0x10 &&
                !g_slots[0].rumble_pending,
            "one-pulse confirmation did not defer host rumble through its off phase");
    for (uint8_t slot = 1; slot < kSlotCount; ++slot) {
        require(devices[slot].rumble_calls == 2 &&
                    devices[slot].last_high == UINT8_MAX &&
                    g_slots[slot].rumble_pending,
                "second profile pulse did not start after 75 ms off");
    }

    now_ms = 225;
    process_rumble_timer(&g_rumble_timer);
    now_ms = 300;
    process_rumble_timer(&g_rumble_timer);
    require(devices[1].rumble_calls == 3 &&
                devices[1].last_high == 0x21 &&
                !g_slots[1].rumble_pending &&
                devices[2].rumble_calls == 3 &&
                devices[3].rumble_calls == 3,
            "two/three/four-pulse sequences diverged at 300 ms");

    now_ms = 375;
    process_rumble_timer(&g_rumble_timer);
    now_ms = 450;
    process_rumble_timer(&g_rumble_timer);
    require(devices[2].rumble_calls == 4 &&
                devices[2].last_high == 0x22 &&
                !g_slots[2].rumble_pending &&
                devices[3].rumble_calls == 4 &&
                devices[3].last_high == UINT8_MAX,
            "three/four-pulse sequences diverged at 450 ms");

    now_ms = 525;
    process_rumble_timer(&g_rumble_timer);
    now_ms = 600;
    process_rumble_timer(&g_rumble_timer);
    require(devices[3].rumble_calls == 5 &&
                devices[3].last_high == 0x23 &&
                !g_slots[3].rumble_pending &&
                devices[0].lightbar_calls == 1 &&
                devices[1].player_led_calls == 1 &&
                devices[2].lightbar_calls == 1 &&
                devices[3].player_led_calls == 1,
            "four-pulse confirmation did not release host rumble at 600 ms");

    const int slot_zero_lightbar_calls = devices[0].lightbar_calls;
    const int slot_one_player_led_calls_before_slot_zero =
        devices[1].player_led_calls;
    const int slot_two_lightbar_calls_before_slot_zero =
        devices[2].lightbar_calls;
    const int slot_three_player_led_calls_before_slot_zero =
        devices[3].player_led_calls;
    bluepad32_input_backend_queue_profile_feedback(
        0, generations[0], 2,
        ControllerProfileConfirmationPolicy::kNone);
    bluepad32_input_backend_queue_rumble(
        0, ControllerRumbleOutput{0x31, 0x41});
    now_ms = 700;
    process_rumble_timer(&g_rumble_timer);
    require(devices[0].rumble_calls == 3 &&
                devices[0].last_high == 0x41 &&
                devices[0].lightbar_calls ==
                    slot_zero_lightbar_calls &&
                !g_slots[0].profile_feedback.active,
            "none policy scheduled profile rumble or lighting");

    bluepad32_input_backend_queue_profile_feedback(
        0, generations[0], 2,
        ControllerProfileConfirmationPolicy::kLed);
    bluepad32_input_backend_queue_rumble(
        0, ControllerRumbleOutput{0x32, 0x42});
    now_ms = 800;
    process_rumble_timer(&g_rumble_timer);
    require(devices[0].rumble_calls == 3 &&
                devices[0].lightbar_calls ==
                    slot_zero_lightbar_calls + 1 &&
                devices[0].lightbar_red ==
                    kProfileLightbarPalette[1].red &&
                devices[0].lightbar_green ==
                    kProfileLightbarPalette[1].green &&
                devices[0].lightbar_blue ==
                    kProfileLightbarPalette[1].blue &&
                observed_status_led_on &&
                g_slots[0].rumble_pending,
            "LED policy did not set transient profile color and first "
            "onboard blink");
    now_ms = 875;
    process_rumble_timer(&g_rumble_timer);
    require(!observed_status_led_on &&
                devices[0].rumble_calls == 3,
            "onboard profile blink did not enter its 75 ms off phase");
    now_ms = 950;
    process_rumble_timer(&g_rumble_timer);
    require(observed_status_led_on,
            "second onboard profile blink did not start");
    now_ms = 1025;
    process_rumble_timer(&g_rumble_timer);
    require(!observed_status_led_on &&
                g_slots[0].rumble_pending,
            "host rumble interrupted the final onboard off phase");
    now_ms = 1100;
    process_rumble_timer(&g_rumble_timer);
    const SwitchRgbColor slot_zero_color =
        switch_pro_get_slot_light_color(0);
    require(devices[0].rumble_calls == 4 &&
                devices[0].last_high == 0x42 &&
                !g_slots[0].rumble_pending &&
                devices[0].lightbar_calls ==
                    slot_zero_lightbar_calls + 2 &&
                devices[0].lightbar_red == slot_zero_color.red &&
                devices[0].lightbar_green == slot_zero_color.green &&
                devices[0].lightbar_blue == slot_zero_color.blue &&
                devices[1].player_led_calls ==
                    slot_one_player_led_calls_before_slot_zero &&
                devices[2].lightbar_calls ==
                    slot_two_lightbar_calls_before_slot_zero &&
                devices[3].player_led_calls ==
                    slot_three_player_led_calls_before_slot_zero,
            "LED-only sequence did not restore its slot color in "
            "isolation after the final gap");

    const int slot_one_player_led_calls =
        devices[1].player_led_calls;
    bluepad32_input_backend_queue_profile_feedback(
        1, generations[1], 3,
        ControllerProfileConfirmationPolicy::kRumbleAndLed);
    now_ms = 1200;
    process_rumble_timer(&g_rumble_timer);
    require(devices[1].rumble_calls == 4 &&
                devices[1].last_high == UINT8_MAX &&
                devices[1].player_led_calls ==
                    slot_one_player_led_calls + 1 &&
                devices[1].player_leds == 0x07 &&
                observed_status_led_on,
            "combined policy did not drive rumble, onboard LED, and player LEDs");
    for (uint32_t deadline = 1275; deadline <= 1650;
         deadline += 75) {
        now_ms = deadline;
        process_rumble_timer(&g_rumble_timer);
    }
    require(devices[1].rumble_calls ==
                    (host_rumble_duration_ms() ==
                             kXInputHostRumbleDurationMs
                         ? 7
                         : 6) &&
                !g_slots[1].profile_feedback.active &&
                devices[1].player_led_calls ==
                    slot_one_player_led_calls + 2 &&
                devices[1].player_leds == (1u << 1u),
            "combined three-pulse sequence did not terminate and "
            "restore slot player lighting");

    const int old_rumble_calls = devices[2].rumble_calls;
    bluepad32_input_backend_queue_profile_feedback(
        2, generations[2], 4,
        ControllerProfileConfirmationPolicy::kRumbleAndLed);
    bluepad32_input_backend_queue_profile_feedback(
        2, generations[2], 1,
        ControllerProfileConfirmationPolicy::kLed);
    require(g_slots[2].pending_profile_feedback_count == 2,
            "two queued profile events did not fill the bounded FIFO");
    platform_on_device_disconnected(&devices[2]);
    uni_hid_device_t replacement = device(2);
    replacement.report_parser.set_lightbar_color = set_lightbar;
    require(platform_on_device_ready(&replacement) ==
                UNI_ERROR_SUCCESS,
            "replacement feedback controller did not become ready");
    now_ms = 1700;
    process_rumble_timer(&g_rumble_timer);
    require(devices[2].rumble_calls == old_rumble_calls &&
                replacement.rumble_calls == 0 &&
                replacement.lightbar_calls == 1 &&
                !g_slots[2].profile_feedback.active &&
                g_slots[2].pending_profile_feedback_count == 0,
            "slot replacement accepted stale queued profile feedback");
    bluepad32_input_backend_queue_profile_feedback(
        2, generations[2], 4,
        ControllerProfileConfirmationPolicy::kRumbleAndLed);
    now_ms = 1705;
    process_rumble_timer(&g_rumble_timer);
    require(replacement.rumble_calls == 0 &&
                replacement.lightbar_calls == 1,
            "stale generation feedback reached a replacement controller");

    const int slot_three_rumble_calls = devices[3].rumble_calls;
    const int slot_three_player_led_calls =
        devices[3].player_led_calls;
    bluepad32_input_backend_queue_profile_feedback(
        3, generations[3], 1,
        ControllerProfileConfirmationPolicy::kLed);
    now_ms = 1800;
    process_rumble_timer(&g_rumble_timer);
    require(observed_status_led_on &&
                g_slots[3].profile_feedback.pulses_started == 1 &&
                devices[3].player_led_calls ==
                    slot_three_player_led_calls + 1 &&
                devices[3].player_leds == 0x01,
            "one-blink LED profile indication did not start");
    now_ms = 1875;
    process_rumble_timer(&g_rumble_timer);
    require(!observed_status_led_on,
            "one-blink LED profile indication did not turn off");
    now_ms = 1950;
    process_rumble_timer(&g_rumble_timer);
    require(!g_slots[3].profile_feedback.active &&
                devices[3].rumble_calls ==
                    slot_three_rumble_calls &&
                devices[3].player_led_calls ==
                    slot_three_player_led_calls + 2 &&
                devices[3].player_leds == (1u << 3u),
            "one-blink LED-only profile indication did not restore "
            "slot lighting cleanly");

    bluepad32_input_backend_queue_profile_feedback(
        3, generations[3], 4,
        ControllerProfileConfirmationPolicy::kLed);
    now_ms = 2000;
    process_rumble_timer(&g_rumble_timer);
    require(devices[3].player_leds == 0x0f,
            "profile 4 player count was not shown during its sequence");
    for (uint8_t pulse = 1; pulse <= 4; ++pulse) {
        require(observed_status_led_on &&
                    g_slots[3].profile_feedback.on &&
                    g_slots[3].profile_feedback.pulses_started ==
                        pulse,
                "four-blink LED sequence missed an on phase");
        now_ms = static_cast<uint32_t>(
            2075u + static_cast<uint32_t>(pulse - 1u) * 150u);
        process_rumble_timer(&g_rumble_timer);
        require(!observed_status_led_on &&
                    !g_slots[3].profile_feedback.on,
                "four-blink LED sequence missed an off phase");
        if (pulse != 4) {
            now_ms += 75;
            process_rumble_timer(&g_rumble_timer);
        }
    }
    now_ms = 2600;
    process_rumble_timer(&g_rumble_timer);
    require(!g_slots[3].profile_feedback.active &&
                devices[3].rumble_calls ==
                    slot_three_rumble_calls &&
                devices[3].player_led_calls ==
                    slot_three_player_led_calls + 4 &&
                devices[3].player_leds == (1u << 3u),
            "four-blink LED-only profile indication did not restore "
            "slot lighting");

    Bluepad32SlotSnapshot replacement_snapshot{};
    bluepad32_input_backend_snapshot(2, &replacement_snapshot);
    const int replacement_lightbar_calls = replacement.lightbar_calls;
    bluepad32_input_backend_queue_profile_feedback(
        2, replacement_snapshot.connection_generation, 1,
        ControllerProfileConfirmationPolicy::kLed);
    now_ms = 2700;
    process_rumble_timer(&g_rumble_timer);
    require(replacement.lightbar_calls ==
                    replacement_lightbar_calls + 1,
            "current-generation profile lighting was not applied");
    now_ms = 2775;
    process_rumble_timer(&g_rumble_timer);
    platform_on_device_disconnected(&replacement);
    uni_hid_device_t second_replacement = device(2);
    second_replacement.report_parser.set_lightbar_color =
        set_lightbar;
    require(platform_on_device_ready(&second_replacement) ==
                UNI_ERROR_SUCCESS &&
                second_replacement.lightbar_calls == 1,
            "second replacement did not receive steady slot lighting");
    now_ms = 2850;
    process_rumble_timer(&g_rumble_timer);
    require(second_replacement.lightbar_calls == 1 &&
                replacement.lightbar_calls ==
                    replacement_lightbar_calls + 1 &&
                !g_slots[2].profile_feedback.active,
            "stale final-gap restore touched a replacement connection");

    const int fifo_lightbar_calls = devices[0].lightbar_calls;
    const int fifo_rumble_calls = devices[0].rumble_calls;
    const int isolated_slot_one_lighting =
        devices[1].player_led_calls;
    const int isolated_slot_two_lighting =
        second_replacement.lightbar_calls;
    const int isolated_slot_three_lighting =
        devices[3].player_led_calls;
    bluepad32_input_backend_queue_profile_feedback(
        0, generations[0], 1,
        ControllerProfileConfirmationPolicy::kLed);
    bluepad32_input_backend_queue_profile_feedback(
        0, generations[0], 2,
        ControllerProfileConfirmationPolicy::kRumbleAndLed);
    require(g_slots[0].pending_profile_feedback_count == 2,
            "initial and switched profile events were not queued");

    now_ms = 3000;
    process_rumble_timer(&g_rumble_timer);
    require(g_slots[0].profile_feedback.active &&
                g_slots[0].profile_feedback.pulse_count == 1 &&
                !g_slots[0].profile_feedback.rumble_enabled &&
                g_slots[0].pending_profile_feedback_count == 1 &&
                devices[0].rumble_calls == fifo_rumble_calls &&
                devices[0].lightbar_calls ==
                    fifo_lightbar_calls + 1 &&
                devices[0].lightbar_red ==
                    kProfileLightbarPalette[0].red,
            "LED-only initial event did not run first from the FIFO");

    now_ms = 3075;
    process_rumble_timer(&g_rumble_timer);
    now_ms = 3150;
    process_rumble_timer(&g_rumble_timer);
    require(g_slots[0].profile_feedback.active &&
                g_slots[0].profile_feedback.pulse_count == 2 &&
                g_slots[0].profile_feedback.rumble_enabled &&
                g_slots[0].pending_profile_feedback_count == 0 &&
                devices[0].rumble_calls == fifo_rumble_calls + 1 &&
                devices[0].lightbar_calls ==
                    fifo_lightbar_calls + 3 &&
                devices[0].lightbar_red ==
                    kProfileLightbarPalette[1].red,
            "switched profile event did not follow initial indication "
            "after its final gap");

    now_ms = 3225;
    process_rumble_timer(&g_rumble_timer);
    now_ms = 3300;
    process_rumble_timer(&g_rumble_timer);
    now_ms = 3375;
    process_rumble_timer(&g_rumble_timer);
    now_ms = 3450;
    process_rumble_timer(&g_rumble_timer);
    const SwitchRgbColor final_slot_zero_color =
        switch_pro_get_slot_light_color(0);
    const bool stateful_host_rumble =
        host_rumble_duration_ms() == kXInputHostRumbleDurationMs;
    require(!g_slots[0].profile_feedback.active &&
                devices[0].rumble_calls ==
                    fifo_rumble_calls +
                        (stateful_host_rumble ? 3 : 2) &&
                (!stateful_host_rumble ||
                 (devices[0].last_high == 0x42 &&
                  devices[0].last_low == 0x32)) &&
                devices[0].lightbar_calls ==
                    fifo_lightbar_calls + 4 &&
                devices[0].lightbar_red ==
                    final_slot_zero_color.red &&
                devices[0].lightbar_green ==
                    final_slot_zero_color.green &&
                devices[0].lightbar_blue ==
                    final_slot_zero_color.blue &&
                devices[1].player_led_calls ==
                    isolated_slot_one_lighting &&
                second_replacement.lightbar_calls ==
                    isolated_slot_two_lighting &&
                devices[3].player_led_calls ==
                    isolated_slot_three_lighting,
            "ordered profile FIFO did not restore or remain slot-local");
}

void test_stateful_host_rumble_restore() {
    start_pairing_backend();
    uni_hid_device_t devices[kSlotCount] = {
        device(0), device(1), device(2), device(3)};
    uint32_t generations[kSlotCount]{};
    for (uint8_t slot = 0; slot < kSlotCount; ++slot) {
        require(platform_on_device_ready(&devices[slot]) ==
                    UNI_ERROR_SUCCESS,
                "rumble restore controller did not become ready");
        Bluepad32SlotSnapshot snapshot{};
        bluepad32_input_backend_snapshot(slot, &snapshot);
        generations[slot] = snapshot.connection_generation;
    }

#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    test_adapter_mode = AdapterUsbMode::kXInput;
    const ControllerRumbleOutput desired[kSlotCount] = {
        {0x11, 0x21}, {0x12, 0x22}, {0x13, 0x23}, {0x14, 0x24}};
    for (uint8_t slot = 0; slot < kSlotCount; ++slot) {
        bluepad32_input_backend_queue_rumble(slot, desired[slot]);
    }
    now_ms = 0;
    process_rumble_timer(&g_rumble_timer);
    for (uint8_t slot = 0; slot < kSlotCount; ++slot) {
        require(devices[slot].rumble_calls == 1 &&
                    devices[slot].last_rumble_duration_ms ==
                        kXInputHostRumbleDurationMs &&
                    g_slots[slot].retained_host_rumble_valid,
                "initial XInput rumble was not dispatched and retained");
        bluepad32_input_backend_queue_profile_feedback(
            slot, generations[slot], static_cast<uint8_t>(slot + 1u),
            ControllerProfileConfirmationPolicy::kRumble);
    }
    now_ms = 10;
    process_rumble_timer(&g_rumble_timer);
    for (now_ms = 85; now_ms <= 610; now_ms += 75) {
        process_rumble_timer(&g_rumble_timer);
    }
    for (uint8_t slot = 0; slot < kSlotCount; ++slot) {
        require(devices[slot].rumble_calls ==
                    static_cast<int>(slot + 3u) &&
                    devices[slot].last_low ==
                        desired[slot].low_frequency_magnitude &&
                    devices[slot].last_high ==
                        desired[slot].high_frequency_magnitude &&
                    devices[slot].last_rumble_duration_ms ==
                        kXInputHostRumbleDurationMs &&
                    !g_slots[slot].profile_feedback.active,
                "XInput rumble did not resume after its profile pulse count");
    }

    const int stop_calls_before = devices[0].rumble_calls;
    bluepad32_input_backend_queue_rumble(
        0, ControllerRumbleOutput{0x51, 0x61});
    now_ms = 700;
    process_rumble_timer(&g_rumble_timer);
    bluepad32_input_backend_queue_profile_feedback(
        0, generations[0], 2,
        ControllerProfileConfirmationPolicy::kRumble);
    now_ms = 705;
    process_rumble_timer(&g_rumble_timer);
    bluepad32_input_backend_queue_rumble(
        0, ControllerRumbleOutput{0, 0});
    for (now_ms = 780; now_ms <= 1005; now_ms += 75) {
        process_rumble_timer(&g_rumble_timer);
    }
    require(devices[0].rumble_calls == stop_calls_before + 4 &&
                devices[0].last_rumble_duration_ms == 0 &&
                devices[0].last_low == 0 &&
                devices[0].last_high == 0 &&
                g_slots[0].retained_host_rumble_valid,
            "newest XInput stop did not win during profile pulses");

    test_adapter_mode = AdapterUsbMode::kSwitchProbe;
    const int switch_calls_before = devices[1].rumble_calls;
    bluepad32_input_backend_queue_rumble(
        1, ControllerRumbleOutput{0x31, 0x41});
    now_ms = 1100;
    process_rumble_timer(&g_rumble_timer);
    require(!g_slots[1].retained_host_rumble_valid,
            "Switch rumble retained stale XInput desired state");
    bluepad32_input_backend_queue_profile_feedback(
        1, generations[1], 1,
        ControllerProfileConfirmationPolicy::kRumble);
    now_ms = 1105;
    process_rumble_timer(&g_rumble_timer);
    now_ms = 1180;
    process_rumble_timer(&g_rumble_timer);
    now_ms = 1255;
    process_rumble_timer(&g_rumble_timer);
    require(devices[1].rumble_calls == switch_calls_before + 2 &&
                devices[1].last_rumble_duration_ms ==
                    kProfileFeedbackPhaseDurationMs,
            "finite Switch rumble resumed after profile feedback");

    test_adapter_mode = AdapterUsbMode::kXInput;
    bluepad32_input_backend_queue_rumble(
        2, ControllerRumbleOutput{0x71, 0x81});
    now_ms = 1300;
    process_rumble_timer(&g_rumble_timer);
    bluepad32_input_backend_queue_profile_feedback(
        2, generations[2], 3,
        ControllerProfileConfirmationPolicy::kRumble);
    now_ms = 1305;
    process_rumble_timer(&g_rumble_timer);
    const int old_device_calls = devices[2].rumble_calls;
    platform_on_device_disconnected(&devices[2]);
    uni_hid_device_t replacement = device(2);
    require(platform_on_device_ready(&replacement) ==
                UNI_ERROR_SUCCESS,
            "rumble restore replacement did not become ready");
    now_ms = 2000;
    process_rumble_timer(&g_rumble_timer);
    require(devices[2].rumble_calls == old_device_calls &&
                replacement.rumble_calls == 0 &&
                !g_slots[2].retained_host_rumble_valid &&
                !g_slots[2].profile_feedback.active,
            "stale XInput rumble resumed on a replacement connection");
#else
    bluepad32_input_backend_queue_rumble(
        0, ControllerRumbleOutput{0x31, 0x41});
    now_ms = 0;
    process_rumble_timer(&g_rumble_timer);
    bluepad32_input_backend_queue_profile_feedback(
        0, generations[0], 1,
        ControllerProfileConfirmationPolicy::kRumble);
    now_ms = 5;
    process_rumble_timer(&g_rumble_timer);
    now_ms = 80;
    process_rumble_timer(&g_rumble_timer);
    now_ms = 155;
    process_rumble_timer(&g_rumble_timer);
    require(devices[0].rumble_calls == 2 &&
                !g_slots[0].retained_host_rumble_valid,
            "bounded Switch rumble resumed after profile feedback");
#endif
}

void test_motion_toggle_action() {
    start_pairing_backend();
    uni_hid_device_t slot_zero = device(0);
    uni_hid_device_t slot_one = device(1);
    require(platform_on_device_ready(&slot_zero) == UNI_ERROR_SUCCESS &&
                platform_on_device_ready(&slot_one) == UNI_ERROR_SUCCESS,
            "motion action test controllers did not become ready");

    Bluepad32SlotSnapshot backend_snapshot{};
    bluepad32_input_backend_snapshot(0, &backend_snapshot);
    require(!bluepad32_input_backend_toggle_motion(
                0, backend_snapshot.connection_generation + 1u),
            "stale connection toggled motion");
    require(bluepad32_input_backend_toggle_motion(
                0, backend_snapshot.connection_generation),
            "live connection did not toggle motion");

    uni_controller_t input{};
    input.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    input.gamepad.accel[0] = 8192;
    platform_on_controller_data(&slot_zero, &input);
    ControllerState snapshot{};
    require(read_controller_state(0, &snapshot) &&
                snapshot.motion_sample_count ==
                    (kDefaultMotionEnabled ? 0 : 3),
            "motion toggle did not change slot motion publication");

    process_rumble_timer(&g_rumble_timer);
    require(slot_zero.rumble_calls == 1 &&
                slot_zero.last_rumble_duration_ms ==
                    (kDefaultMotionEnabled
                         ? kMotionDisabledFeedbackDurationMs
                         : kMotionEnabledFeedbackDurationMs) &&
                slot_zero.last_high ==
                    (kDefaultMotionEnabled
                         ? kMotionDisabledFeedbackWeakMagnitude
                         : kMotionEnabledFeedbackWeakMagnitude) &&
                slot_zero.last_low ==
                    (kDefaultMotionEnabled
                         ? kMotionDisabledFeedbackStrongMagnitude
                         : kMotionEnabledFeedbackStrongMagnitude),
            "motion toggle did not send distinct state feedback");

    uni_controller_t peer_input{};
    peer_input.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    peer_input.gamepad.accel[0] = 8192;
    platform_on_controller_data(&slot_one, &peer_input);
    require(read_controller_state(1, &snapshot) &&
                snapshot.motion_sample_count ==
                    (kDefaultMotionEnabled ? 3 : 0),
            "slot 0 motion action changed slot 1 motion state");

    require(bluepad32_input_backend_toggle_motion(
                0, backend_snapshot.connection_generation),
            "second live motion toggle failed");
    platform_on_controller_data(&slot_zero, &input);
    require(read_controller_state(0, &snapshot) &&
                snapshot.motion_sample_count ==
                    (kDefaultMotionEnabled ? 3 : 0),
            "second motion toggle did not restore configured state");

    platform_on_device_disconnected(&slot_zero);
    require(!bluepad32_input_backend_toggle_motion(
                0, backend_snapshot.connection_generation),
            "disconnected generation toggled motion");
    uni_hid_device_t replacement = device(0);
    require(platform_on_device_ready(&replacement) == UNI_ERROR_SUCCESS &&
                g_slots[0].motion_enabled == kDefaultMotionEnabled &&
                g_slots[0].pre_hotkey_button_mask == 0 &&
                !g_slots[0].feedback_pending,
            "disconnect did not reset slot 0 motion action state");
}

void test_protocol_neutral_analog_state() {
    start_pairing_backend();
    uni_hid_device_t controller = device(0);
    uni_hid_device_t peer = device(1);
    platform_on_device_connected(&controller);
    platform_on_device_connected(&peer);
    require(platform_on_device_ready(&controller) == UNI_ERROR_SUCCESS &&
                platform_on_device_ready(&peer) == UNI_ERROR_SUCCESS,
            "analog-state controllers did not become ready");

    uni_controller_t peer_input{};
    peer_input.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    peer_input.gamepad.brake = 256;
    peer_input.gamepad.throttle = 768;
    peer_input.gamepad.buttons =
        BUTTON_TRIGGER_L | BUTTON_TRIGGER_R;
    platform_on_controller_data(&peer, &peer_input);

    ControllerState peer_state{};
    require(read_controller_state(1, &peer_state) &&
                peer_state.left_trigger == 16399 &&
                peer_state.right_trigger == 49199,
            "peer analog trigger state was not published exactly");

    uni_controller_t input{};
    input.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    input.gamepad.axis_x = -512;
    input.gamepad.axis_y = 0;
    input.gamepad.axis_rx = 511;
    input.gamepad.axis_ry = -256;
    input.gamepad.buttons =
        BUTTON_TRIGGER_L | BUTTON_TRIGGER_R;

    struct TriggerCase {
        int32_t brake;
        int32_t throttle;
        uint16_t expected_left;
        uint16_t expected_right;
    };
    constexpr TriggerCase trigger_cases[] = {
        {4, 512, 256, 32799},
        {512, 1020, 32799, UINT16_MAX},
        {1020, 1016, UINT16_MAX, 65086},
        {1016, 1023, 65086, UINT16_MAX},
        {1023, 4, UINT16_MAX, 256},
    };
    ControllerState state{};
    for (const TriggerCase& trigger_case : trigger_cases) {
        input.gamepad.brake = trigger_case.brake;
        input.gamepad.throttle = trigger_case.throttle;
        platform_on_controller_data(&controller, &input);

        require(read_controller_state(0, &state) &&
                    state.left_trigger == trigger_case.expected_left &&
                    state.right_trigger == trigger_case.expected_right,
                "digital trigger bits flattened analog trigger precision");
        require(read_controller_state(1, &peer_state) &&
                    peer_state.left_trigger == 16399 &&
                    peer_state.right_trigger == 49199,
                "slot 0 trigger update changed slot 1");
    }
    require(state.left_stick_x == INT16_MIN &&
                state.left_stick_y == 0 &&
                state.right_stick_x == INT16_MAX &&
                state.right_stick_y == -16384,
            "stick axes were not normalized to signed full range");

    input.gamepad.brake = 0;
    input.gamepad.throttle = 0;
    input.gamepad.buttons = BUTTON_TRIGGER_L;
    platform_on_controller_data(&controller, &input);
    require(read_controller_state(0, &state) &&
                state.left_trigger == UINT16_MAX &&
                state.right_trigger == 0,
            "left digital-only trigger did not map independently");

    input.gamepad.buttons = BUTTON_TRIGGER_R;
    platform_on_controller_data(&controller, &input);
    require(read_controller_state(0, &state) &&
                state.left_trigger == 0 &&
                state.right_trigger == UINT16_MAX,
            "right digital-only trigger did not map independently");

    input.gamepad.buttons = 0;
    platform_on_controller_data(&controller, &input);
    require(read_controller_state(0, &state) &&
                state.left_trigger == 0 &&
                state.right_trigger == 0,
            "released triggers did not return to zero");
    require(read_controller_state(1, &peer_state) &&
                peer_state.left_trigger == 16399 &&
                peer_state.right_trigger == 49199,
            "slot 0 digital trigger fallback changed slot 1");
    input.gamepad.misc_buttons = MISC_BUTTON_CAPTURE;
    platform_on_controller_data(&controller, &input);
    require(read_controller_state(0, &state) &&
                state.button_capture,
            "touchpad/capture input did not reach the logical capture button");
    input.gamepad.misc_buttons = 0;
    platform_on_controller_data(&controller, &input);
    require(read_controller_state(0, &state) &&
                !state.button_capture,
            "released touchpad/capture input remained pressed");
}

void test_host_rumble_mode_duration() {
    start_pairing_backend();
    uni_hid_device_t controller = device(0);
    platform_on_device_connected(&controller);
    require(platform_on_device_ready(&controller) == UNI_ERROR_SUCCESS,
            "rumble-mode controller did not become ready");

#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    test_adapter_mode = AdapterUsbMode::kSwitchProbe;
#endif
    bluepad32_input_backend_queue_rumble(
        0, ControllerRumbleOutput{100, 101});
    process_rumble_timer(&g_rumble_timer);
    require(controller.last_rumble_duration_ms ==
                kSwitchHostRumbleDurationMs,
            "Switch mode did not use bounded host rumble");

#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    test_adapter_mode = AdapterUsbMode::kXInput;
    bluepad32_input_backend_queue_rumble(
        0, ControllerRumbleOutput{102, 103});
    process_rumble_timer(&g_rumble_timer);
    require(controller.last_rumble_duration_ms ==
                kXInputHostRumbleDurationMs,
            "XInput mode did not retain stateful host rumble");
#endif
}

void test_xbox_trigger_rumble() {
    start_pairing_backend();
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    test_adapter_mode = AdapterUsbMode::kSwitchProbe;
#endif
    auto controller = device(0, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    controller.vendor_id = 0x045e;
    controller.product_id = 0x02e0;
    controller.report_parser.play_dual_rumble =
        uni_hid_parser_xboxone_play_dual_rumble;
    require(platform_on_device_ready(&controller) == UNI_ERROR_SUCCESS,
            "Xbox did not become ready");
    ControllerRumbleOutput hd{80, 100};
    hd.hd.actuators[0].sample_count = 3;
    hd.hd.actuators[1].sample_count = 1;
    hd.hd.actuators[0].samples[1].high_amplitude_q15 = 16384;
    bluepad32_input_backend_queue_rumble(0, hd);
    process_rumble_timer(&g_rumble_timer);
    require(xbox_left_trigger == 63 && xbox_right_trigger == 0 &&
                controller.last_low == 80 && controller.last_high == 100 &&
                controller.last_rumble_duration_ms == 50,
            "left high-band substep must drive only left trigger, preserving grips");

    hd = {};
    hd.hd.actuators[0].sample_count = 1;
    hd.hd.actuators[1].sample_count = 1;
    hd.hd.actuators[0].samples[0].low_amplitude_q15 = 32767;
    hd.hd.actuators[1].samples[0].high_amplitude_q15 = 65535;
    bluepad32_input_backend_queue_rumble(0, hd);
    process_rumble_timer(&g_rumble_timer);
    require(xbox_left_trigger == 0 && xbox_right_trigger == 127 &&
                controller.last_rumble_duration_ms == 50,
            "right trigger-only effect must not stop or overflow after amplification");

    dispatch_rumble(&controller, 75, 100, 100);
    require(xbox_left_trigger == 0 && xbox_right_trigger == 0,
            "local feedback must clear trigger vibration");
#ifdef SWITCH_PICO_USB_OUTPUT_MODES
    test_adapter_mode = AdapterUsbMode::kXInput;
#endif
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{200, 180});
    process_rumble_timer(&g_rumble_timer);
    require(xbox_left_trigger == 90 && xbox_right_trigger == 90 &&
                controller.last_rumble_duration_ms == host_rumble_duration_ms(),
            "conventional high-frequency rumble must feed both triggers");
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{});
    process_rumble_timer(&g_rumble_timer);
    require(xbox_left_trigger == 0 && xbox_right_trigger == 0 &&
                controller.last_high == 0 && controller.last_low == 0 &&
                controller.last_rumble_duration_ms == 0,
            "explicit stop must stop all four motors");

    const unsigned calls = xbox_quad_calls;
    controller.report_parser.play_dual_rumble = play_rumble;
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{45, 67});
    process_rumble_timer(&g_rumble_timer);
    require(xbox_quad_calls == calls && controller.last_low == 45 &&
                controller.last_high == 67,
            "Microsoft VID alone must not route an unrelated parser to Xbox output");
    controller.report_parser.play_dual_rumble =
        uni_hid_parser_xboxone_play_dual_rumble;
    bluepad32_input_backend_queue_rumble(0, hd);
    platform_on_device_disconnected(&controller);
    process_rumble_timer(&g_rumble_timer);
    require(xbox_quad_calls == calls,
            "pending Xbox output must not reach a disconnected controller");
}

void test_clear_pairings() {
    classic_bond_count = 1;
    classic_bonds[0][0] = 0x10;
    ble_bond_count = 1;
    ble_bond_types[0] = BD_ADDR_TYPE_LE_PUBLIC;
    ble_bonds[0][0] = 0x20;
    start_pairing_backend();
    require(g_pairing_snapshot.status ==
                    Bluepad32PairingSnapshotStatus::kReady &&
                g_pairing_snapshot.record_count == 2 &&
                g_pairing_snapshot.records[0].transport ==
                    Bluepad32PairingTransport::kClassic &&
                g_pairing_snapshot.records[1].transport ==
                    Bluepad32PairingTransport::kBle,
            "initial pairing snapshot must enumerate Classic and BLE bonds");
    require(!bluepad32_input_backend_clear_pairings_completed(
                g_pairing_snapshot, 0),
            "zero pairing-clear token must never be a valid completion query");
    const uint32_t snapshot_generation =
        g_pairing_snapshot.generation;
    g_next_clear_pairings_request_token = UINT32_MAX;
    uni_hid_device_t devices[2] = {device(0), device(1)};
    for (uni_hid_device_t& controller : devices) {
        require(platform_on_device_ready(&controller) == UNI_ERROR_SUCCESS,
                "pairing reset controller did not become ready");
    }
    bluepad32_input_backend_queue_rumble(
        0, ControllerRumbleOutput{100, 101});

    const uint32_t clear_token =
        bluepad32_input_backend_clear_pairings();
    const uint32_t repeated_token =
        bluepad32_input_backend_clear_pairings();
    expected_pending_clear_token = clear_token;
    repeat_clear_during_disconnect = true;
    g_connection_policy_state =
        ConnectionPolicyState::Uninitialized;
    require(clear_token == UINT32_MAX &&
                repeated_token == clear_token &&
                g_clear_pairings_requested_token == clear_token &&
                g_pairing_snapshot.completed_clear_pairings_token == 0 &&
                !bluepad32_input_backend_clear_pairings_completed(
                    g_pairing_snapshot, clear_token) &&
                delete_key_calls == 0 && device_disconnect_calls == 0,
            "Core0 repeated pending clear calls must share one nonzero token, "
            "remain incomplete, and defer the operation to Core1");
    process_rumble_timer(&g_rumble_timer);

    require(repeated_in_progress_clear_token == clear_token,
            "clear repeated during Core1 work did not share its token");
    require(delete_key_calls == 1 && device_disconnect_calls == 2,
            "pairing reset must delete bonds and disconnect every session");
    require(g_pairing_snapshot.status ==
                    Bluepad32PairingSnapshotStatus::kReady &&
                g_pairing_snapshot.record_count == 0 &&
                g_pairing_snapshot.generation ==
                    snapshot_generation + 1 &&
                g_pairing_snapshot.completed_clear_pairings_token ==
                    clear_token &&
                bluepad32_input_backend_clear_pairings_completed(
                    g_pairing_snapshot, clear_token),
            "pairing reset must publish its completion token with an empty "
            "refreshed snapshot after policy update");
    expected_pending_clear_token = 0;
    for (const BackendSlot& slot : g_slots) {
        require(slot.device == nullptr && !slot.active &&
                    !slot.rumble_pending && !slot.feedback_pending &&
                    slot.state.left_stick_x == 0 &&
                    slot.state.left_stick_y == 0 &&
                    slot.state.right_stick_x == 0 &&
                    slot.state.right_stick_y == 0 &&
                    slot.state.motion_sample_count == 0,
                "pairing reset must publish neutral empty slots");
    }
    require(!g_pairing_window_open && !bondable &&
                accepted_stk_methods == 0 &&
                g_connection_policy_state == ConnectionPolicyState::Open &&
                scanning_enabled && classic_scanning_enabled &&
                incoming_connections && observed_status_led_on,
            "pairing reset must close authentication and resume autoconnect");

    tick_backend_timer(9);
    require(!observed_status_led_on,
            "pairing reset confirmation must use the rapid blink pattern");
    process_rumble_timer(&g_rumble_timer);
    require(delete_key_calls == 1 && device_disconnect_calls == 2 &&
                g_pairing_snapshot.completed_clear_pairings_token ==
                    clear_token,
            "pairing reset request must execute only once");

    const uint32_t completed_generation =
        g_pairing_snapshot.generation;
    bluepad32_input_backend_request_pairing_snapshot();
    require(g_pairing_snapshot.status ==
                    Bluepad32PairingSnapshotStatus::kPending &&
                bluepad32_input_backend_clear_pairings_completed(
                    g_pairing_snapshot, clear_token),
            "unrelated refresh revoked an already completed clear");
    process_rumble_timer(&g_rumble_timer);
    require(g_pairing_snapshot.generation ==
                    completed_generation + 1 &&
                g_pairing_snapshot.completed_clear_pairings_token ==
                    clear_token &&
                bluepad32_input_backend_clear_pairings_completed(
                    g_pairing_snapshot, clear_token),
            "ordinary pairing refresh fabricated or revoked clear completion");

    const uint32_t refreshed_generation =
        g_pairing_snapshot.generation;
    const uint32_t wrapped_token =
        bluepad32_input_backend_clear_pairings();
    require(wrapped_token == 1 &&
                g_pairing_snapshot.status ==
                    Bluepad32PairingSnapshotStatus::kPending &&
                g_pairing_snapshot.completed_clear_pairings_token ==
                    clear_token &&
                !bluepad32_input_backend_clear_pairings_completed(
                    g_pairing_snapshot, wrapped_token) &&
                bluepad32_input_backend_clear_pairings_completed(
                    g_pairing_snapshot, clear_token),
            "UINT32_MAX-to-1 wrap reused the prior completion or revoked it");
    expected_pending_clear_token = wrapped_token;
    process_rumble_timer(&g_rumble_timer);
    expected_pending_clear_token = 0;
    require(delete_key_calls == 2 && device_disconnect_calls == 2 &&
                g_pairing_snapshot.generation ==
                    refreshed_generation + 1 &&
                g_pairing_snapshot.completed_clear_pairings_token ==
                    wrapped_token &&
                bluepad32_input_backend_clear_pairings_completed(
                    g_pairing_snapshot, wrapped_token) &&
                bluepad32_input_backend_clear_pairings_completed(
                    g_pairing_snapshot, clear_token),
            "wrapped clear completion did not acknowledge both itself and "
            "the earlier pre-wrap request exactly once");
}

void test_configuration_timer_rearms_before_storage_work() {
    const uint32_t adds_before = g_configuration_timer.add_count;
    expected_configuration_timer_add_count = adds_before + 1;
    expect_configuration_timer_prearmed = true;
    process_configuration_timer(&g_configuration_timer);
    expect_configuration_timer_prearmed = false;
    require(g_configuration_timer.add_count ==
                    expected_configuration_timer_add_count &&
                g_configuration_timer.timeout_ms ==
                    kConfigurationPollIntervalMs,
            "configuration timer did not remain recurring");
}

void test_flash_core_start_contract() {
    bluepad32_input_backend_init();
    flash_core_init_result = false;
    bluepad32_input_backend_start();
    require(flash_core_init_calls == 1 && core1_launch_calls == 0 &&
                g_connection_policy_state ==
                    ConnectionPolicyState::FailedClosed,
            "Core0 flash-safe init failure must prevent Core1 launch");

    flash_core_init_result = true;
    bluepad32_input_backend_start();
    require(flash_core_init_calls == 2 && core1_launch_calls == 1,
            "Core0 must register as a flash-safe victim before Core1 launch");
    bluepad32_input_backend_start();
    require(flash_core_init_calls == 2 && core1_launch_calls == 1,
            "backend start must remain idempotent");
}



void test_wake_identity_gates_connections() {
    switch2_connections_ready = false;
    bluepad32_input_backend_init();
    platform_on_init_complete();
    require(switch2_wake_initializations == 1 &&
                g_connection_policy_state ==
                    ConnectionPolicyState::Uninitialized &&
                !scanning_enabled && !classic_scanning_enabled &&
                !incoming_connections,
            "controller discovery started before wake identity was ready");

    switch2_connections_ready = true;
    process_rumble_timer(&g_rumble_timer);
    require(g_connection_policy_state == ConnectionPolicyState::Open &&
                scanning_enabled && classic_scanning_enabled &&
                incoming_connections,
            "controller discovery did not start after wake identity setup");
}


void test_system_button_wake_trigger() {
    start_pairing_backend();
    uni_hid_device_t controller = device(0);
    require(platform_on_device_ready(&controller) == UNI_ERROR_SUCCESS,
            "wake trigger controller did not become ready");

    uni_controller_t input{};
    input.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    platform_on_controller_data(&controller, &input);
    require(switch2_wake_requests == 0,
            "neutral input requested a wake burst");

    input.gamepad.misc_buttons = MISC_BUTTON_SYSTEM;
    platform_on_controller_data(&controller, &input);
    require(switch2_wake_requests == 0,
            "plain system button requested a wake burst");

    input.gamepad.buttons = BUTTON_SHOULDER_L | BUTTON_SHOULDER_R;
    platform_on_controller_data(&controller, &input);
    platform_on_controller_data(&controller, &input);
    require(switch2_wake_requests == 1,
            "held L+R+System chord did not produce exactly one wake request");

    input.gamepad.buttons = 0;
    input.gamepad.misc_buttons = 0;
    platform_on_controller_data(&controller, &input);
    input.gamepad.buttons = BUTTON_SHOULDER_L | BUTTON_SHOULDER_R;
    platform_on_controller_data(&controller, &input);
    require(switch2_wake_requests == 1,
            "L+R without System requested wake");
    input.gamepad.misc_buttons = MISC_BUTTON_SYSTEM;
    platform_on_controller_data(&controller, &input);
    require(switch2_wake_requests == 2,
            "second L+R+System chord edge did not request wake");

    input.gamepad.buttons = 0;
    input.gamepad.misc_buttons = MISC_BUTTON_CAPTURE;
    platform_on_controller_data(&controller, &input);
    require(switch2_wake_requests == 2,
            "non-system misc button requested wake");
}


void test_flash_core_init_fatal() {
    bluepad32_input_backend_init();
    flash_core_init_result = false;
    bool stopped = false;
    try {
        core1_main();
    } catch (const CoreStopped&) {
        stopped = true;
    }
    require(stopped && flash_core_init_calls == 1 &&
                cyw43_init_calls == 0 && uni_init_calls == 0 &&
                g_connection_policy_state ==
                    ConnectionPolicyState::FailedClosed,
            "flash-safe Core1 init failure must halt before CYW43 init");
}

#if defined(SWITCH_PICO_HAPTICS_EXPERIMENT) && defined(SWITCH_PICO_USB_OUTPUT_MODES)
void advance_native_backend(uint32_t duration_ms) {
    const uint32_t end_ms = now_ms + duration_ms;
    while (now_ms < end_ms) {
        ++now_ms;
        for (;;) {
            const auto due = std::find_if(
                native_timers.begin(), native_timers.end(),
                [](const auto* timer) {
                    return timer != &g_rumble_timer &&
                           timer != &g_configuration_timer &&
                           timer->due_ms <= now_ms;
                });
            if (due == native_timers.end()) break;
            auto* timer = *due;
            native_timers.erase(due);
            timer->handler(timer);
        }
        if (now_ms % kRumblePollIntervalMs == 0)
            process_rumble_timer(&g_rumble_timer);
    }
}

void require_native_channels(bool left, bool right) {
    HapticsExperimentDiagnostics status;
    haptics_experiment_snapshot(&status);
    require(status.state == HapticsExperimentState::kRunning &&
                status.mode == 1,
            "stateful host rumble lost native gameplay ownership");
    unsigned active[2]{};
    for (unsigned frame = 0; frame < status.packet_frames; ++frame) {
        active[0] += last_native_packet[10 + frame * 2] != 0;
        active[1] += last_native_packet[11 + frame * 2] != 0;
    }
    require((left ? active[0] > status.packet_frames / 2u : active[0] == 0) &&
                (right ? active[1] > status.packet_frames / 2u : active[1] == 0),
            "native PCM did not preserve the requested stateful channels");
}

void test_native_stateful_routing() {
    start_pairing_backend();
    test_adapter_mode = AdapterUsbMode::kXInput;
    auto selected = device(0, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    selected.vendor_id = 0x054c;
    selected.product_id = 0x0ce6;
    selected.conn.connected = true;
    selected.conn.interrupt_cid = 0x80;
    selected.outgoing_buffer.queued = 1;
    require(platform_on_device_ready(&selected) == UNI_ERROR_SUCCESS,
            "selected DualSense was rejected");
    process_rumble_timer(&g_rumble_timer);
    HapticsExperimentDiagnostics status;
    haptics_experiment_snapshot(&status);
    require(status.mode == 1 && status.state == HapticsExperimentState::kPending,
            "XInput DualSense did not auto-arm into native Prepare");
    const uint32_t generation = g_slots[0].connection_generation;
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{180, 0});
    advance_native_backend(70);
    const int prepare_calls = selected.rumble_calls;
    selected.outgoing_buffer.queued = 0;
    advance_native_backend(350);
    require_native_channels(true, false);
    require(selected.rumble_calls == prepare_calls &&
                last_native_cid == selected.conn.interrupt_cid,
            "XInput interleaved compatibility rumble into native PCM");

    auto unselected = device(1, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    unselected.vendor_id = selected.vendor_id;
    unselected.product_id = selected.product_id;
    unselected.conn.connected = true;
    unselected.conn.interrupt_cid = 0x82;
    require(platform_on_device_ready(&unselected) == UNI_ERROR_SUCCESS,
            "unselected DualSense was rejected");
    bluepad32_input_backend_queue_rumble(1, ControllerRumbleOutput{22, 33});
    advance_native_backend(10);
    require(unselected.rumble_calls == 1 && unselected.last_low == 22 &&
                unselected.last_high == 33,
            "unselected controller lost compatibility fallback");
    require(!haptics_experiment_submit_rumble(1, generation, time_us_64(), 255, 255),
            "native stream accepted an unselected slot");

    bluepad32_input_backend_queue_profile_feedback(
        0, generation, 1, ControllerProfileConfirmationPolicy::kRumble);
    advance_native_backend(40);
    require_native_channels(true, true);
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{0, 170});
    advance_native_backend(220);
    require_native_channels(false, true);
    require(selected.rumble_calls == prepare_calls,
            "profile feedback escaped the native overlay");
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{0, 0});
    advance_native_backend(80);
    require_native_channels(false, false);
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{150, 0});
    advance_native_backend(80);
    require(haptics_experiment_request(0, 0), "native stop was rejected");
    advance_native_backend(20);
    require(!haptics_experiment_owns(&selected) && selected.last_low == 150 &&
                selected.last_high == 0 &&
                selected.last_rumble_duration_ms == kXInputHostRumbleDurationMs,
            "explicit native stop lost the latest compatibility fallback");
    require(haptics_experiment_request(2, 0), "manual gameplay rearm failed");
    advance_native_backend(300);
    require_native_channels(true, false);
    const int rearm_calls = selected.rumble_calls;
    advance_native_backend(300);
    require(selected.rumble_calls == rearm_calls,
            "held XInput state required periodic compatibility refresh");

    require(haptics_experiment_request(0, 0), "second native stop failed");
    advance_native_backend(20);
    require(selected.last_low == 150 && selected.last_high == 0 &&
                selected.last_rumble_duration_ms == kXInputHostRumbleDurationMs,
            "manual rearm discarded held state needed by the next Stop");
    require(haptics_experiment_request(1, 0), "fixture start failed");
    advance_native_backend(10);
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{0, 160});
    advance_native_backend(6200);
    require(haptics_experiment_request(2, 0), "post-fixture gameplay arm failed");
    advance_native_backend(300);
    require_native_channels(false, true);

    // A real HD frame replaces stateful XInput semantics and keeps its watchdog.
    test_adapter_mode = AdapterUsbMode::kSwitchProbe;
    ControllerRumbleOutput hd{};
    hd.hd.actuators[0].sample_count = 1;
    hd.hd.actuators[1].sample_count = 1;
    hd.hd.actuators[0].samples[0].low_amplitude_q15 = 20000;
    hd.hd.actuators[0].samples[0].low_frequency_index = 64;
    bluepad32_input_backend_queue_rumble(0, hd);
    advance_native_backend(40);
    require_native_channels(true, false);
    advance_native_backend(100);
    require_native_channels(false, false);

    test_adapter_mode = AdapterUsbMode::kXInput;
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{255, 0});
    platform_on_device_disconnected(&selected);
    require(!haptics_experiment_submit_rumble(0, generation, time_us_64(), 255, 255),
            "disconnected generation remained eligible for native output");
    require(platform_on_device_ready(&selected) == UNI_ERROR_SUCCESS,
            "replacement DualSense was rejected");
    advance_native_backend(300);
    require_native_channels(false, false);
    require(!haptics_experiment_submit_rumble(0, generation, time_us_64(), 255, 255),
            "old generation reached a replacement native stream");
    require(haptics_experiment_request(0, 0), "replacement stop failed");
    advance_native_backend(20);
    platform_on_device_disconnected(&selected);
    platform_on_device_disconnected(&unselected);
}

void test_native_second_slot_selection() {
    start_pairing_backend();
    test_adapter_mode = AdapterUsbMode::kXInput;
    auto pro = device(0, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    pro.vendor_id = 0x057e;
    pro.product_id = 0x2009;
    pro.conn.connected = true;
    pro.conn.interrupt_cid = 0x80;
    require(platform_on_device_ready(&pro) == UNI_ERROR_SUCCESS,
            "first-slot Switch Pro was rejected");
    auto dualsense = device(1, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    dualsense.vendor_id = 0x054c;
    dualsense.product_id = 0x0ce6;
    dualsense.conn.connected = true;
    dualsense.conn.interrupt_cid = 0x82;
    require(platform_on_device_ready(&dualsense) == UNI_ERROR_SUCCESS,
            "second-slot DualSense was rejected");
    advance_native_backend(30);
    HapticsExperimentDiagnostics status;
    haptics_experiment_snapshot(&status);
    require(status.state == HapticsExperimentState::kRunning && status.slot == 1,
            "native auto-arm ignored the first eligible DualSense outside slot zero");
    bluepad32_input_backend_queue_rumble(1, ControllerRumbleOutput{90, 0});
    bluepad32_input_backend_queue_rumble(0, ControllerRumbleOutput{20, 0});
    advance_native_backend(200);
    require_native_channels(true, false);
    require(pro.rumble_calls == 1 && last_native_cid == 0x82,
            "mixed controller slots lost their independent rumble paths");
    auto later = device(2, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    later.vendor_id = dualsense.vendor_id;
    later.product_id = dualsense.product_id;
    later.conn.connected = true;
    later.conn.interrupt_cid = 0x84;
    require(platform_on_device_ready(&later) == UNI_ERROR_SUCCESS,
            "later DualSense was rejected");
    advance_native_backend(30);
    haptics_experiment_snapshot(&status);
    require(status.slot == 1 && last_native_cid == 0x82,
            "a later DualSense stole the selected native stream");
    require(haptics_experiment_request(0, 1), "selected stream did not stop");
    advance_native_backend(20);
    platform_on_device_disconnected(&dualsense);
    platform_on_device_disconnected(&pro);
    platform_on_device_disconnected(&later);
}
#endif

}  // namespace

int main(int argc, char** argv) {
    require(argc == 2, "scenario argument required");
    const std::string scenario = argv[1];
#if defined(SWITCH_PICO_HAPTICS_EXPERIMENT) && defined(SWITCH_PICO_USB_OUTPUT_MODES)
    if (scenario == "native-stateful") {
        test_native_stateful_routing();
        return 0;
    }
    if (scenario == "native-second-slot") {
        test_native_second_slot_selection();
        return 0;
    }
#endif
    if (scenario == "switch2-hd-pro") {
        test_switch2_hd_pro();
    } else if (scenario == "switch2-gesture-timing") {
        test_switch2_gesture_timing();
    } else if (scenario == "switch2-gesture-slot-left-first") {
        test_switch2_gesture_slot_order(true, 0);
    } else if (scenario == "switch2-gesture-slot-right-first") {
        test_switch2_gesture_slot_order(false, 0);
    } else if (scenario == "switch2-gesture-slot-occupied") {
        test_switch2_gesture_slot_order(true, 1);
    } else if (scenario == "switch2-gesture-stale") {
        test_switch2_gesture_stale();
    } else if (scenario == "switch2-gesture-clock-wrap") {
        test_switch2_gesture_clock_wrap();
    } else if (scenario == "switch2-gesture-masking-epochs") {
        test_switch2_gesture_masking_epochs();
    } else if (scenario == "switch2-gesture-override-lifetime") {
        test_switch2_gesture_override_lifetime();
    } else if (scenario == "switch2-gesture-two-pairs") {
        test_switch2_gesture_two_pairs();
    } else if (scenario == "switch2-gesture-ambiguous") {
        test_switch2_gesture_ambiguous();
    } else if (scenario == "switch2-gesture-seed-failure") {
        test_switch2_gesture_seed_failure();
    } else if (scenario == "switch2-gesture-device-scope") {
        test_switch2_gesture_device_scope();
    } else if (scenario == "switch2-hd-solo-left") {
        test_switch2_hd_solo(false);
    } else if (scenario == "switch2-hd-solo-right") {
        test_switch2_hd_solo(true);
    } else if (scenario == "switch2-hd-pair") {
        test_switch2_hd_pair();
    } else if (scenario == "switch2-hd-overflow") {
        test_switch2_hd_overflow();
    } else if (scenario == "switch2-hd-epochs") {
        test_switch2_hd_epochs();
    } else if (scenario == "switch2-hd-feedback") {
        test_switch2_hd_feedback();
    } else if (scenario == "switch2-individual-core-start") {
        test_switch2_individual_core_start();
    } else if (scenario == "switch2-individual-forward") {
        test_switch2_individual_boot(false);
    } else if (scenario == "switch2-individual-reverse") {
        test_switch2_individual_boot(true);
    } else if (scenario == "switch2-mode-forward") {
        test_switch2_mode_roundtrip(false);
    } else if (scenario == "switch2-mode-reverse") {
        test_switch2_mode_roundtrip(true);
    } else if (scenario == "switch2-mode-two-pairs") {
        test_switch2_mode_two_pairs();
    } else if (scenario == "switch2-mode-seed-failure") {
        test_switch2_live_pair_failure(false);
    } else if (scenario == "switch2-mode-identity-failure") {
        test_switch2_live_pair_failure(true);
    } else if (scenario == "switch2-forward") {
        test_switch2_pair_lifecycle(false);
    } else if (scenario == "switch2-reverse") {
        test_switch2_pair_lifecycle(true);
    } else if (scenario == "switch2-multiple-pairs") {
        test_switch2_multiple_pairs();
    } else if (scenario == "switch2-pair-failure-left") {
        test_switch2_pair_admission_failure(false);
    } else if (scenario == "switch2-pair-failure-right") {
        test_switch2_pair_admission_failure(true);
    } else if (scenario == "switch2-pair-replacement") {
        test_switch2_pair_member_replacement();
    } else if (scenario == "switch2-admission") {
        test_switch2_admission();
    } else if (scenario == "switch2-radio-policy") {
        test_switch2_radio_policy();
    } else if (scenario == "switch2-radio-settling") {
        test_switch2_radio_settling();
    } else if (scenario == "switch2-mate-reconnect") {
        test_switch2_mate_reconnect();
    } else if (scenario == "switch2-mate-pending") {
        test_switch2_mate_pending();
    } else if (scenario == "switch2-mate-pairing-window") {
        test_switch2_mate_pairing_window();
    } else if (scenario == "switch2-pairing-inventory") {
        test_switch2_pairing_inventory();
    } else if (scenario == "ready-forward") {
        test_ready_order(false);
    } else if (scenario == "ready-reverse") {
        test_ready_order(true);
    } else if (scenario == "rejections") {
        test_rejections();
    } else if (scenario == "lifecycle") {
        test_independent_lifecycle();
    } else if (scenario == "pairing-policy") {
        test_pairing_window_policy();
    } else if (scenario == "slot-lighting") {
        test_slot_lighting();
    } else if (scenario == "stateful-rumble") {
        test_stateful_host_rumble_restore();
    } else if (scenario == "profile-chord-raw") {
        test_profile_chord_remains_raw();
    } else if (scenario == "profile-feedback") {
        test_profile_feedback_scheduler();
    } else if (scenario == "motion-hotkey") {
        test_motion_toggle_action();
    } else if (scenario == "analog-state") {
        test_protocol_neutral_analog_state();
    } else if (scenario == "rumble-mode") {
        test_host_rumble_mode_duration();
    } else if (scenario == "xbox-rumble") {
        test_xbox_trigger_rumble();
    } else if (scenario == "clear-pairings") {
        test_clear_pairings();
    } else if (scenario == "configuration-timer") {
        test_configuration_timer_rearms_before_storage_work();
    } else if (scenario == "flash-core-start") {
        test_flash_core_start_contract();
    } else if (scenario == "wake-identity-gate") {
        test_wake_identity_gates_connections();
    } else if (scenario == "system-wake") {
        test_system_button_wake_trigger();
    } else if (scenario == "flash-core-failure") {
        test_flash_core_init_fatal();
    } else {
        require(false, "unknown scenario");
    }
    return 0;
}
