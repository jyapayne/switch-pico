#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include <uni.h>
#include "../controller_color_config.h"

namespace {


bool incoming_connections = false;
int scan_starts = 0;
int scan_stops = 0;
bool scanning_enabled = false;
int classic_scan_starts = 0;
int classic_scan_stops = 0;
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
int cyw43_init_calls = 0;
int uni_init_calls = 0;
int device_disconnect_calls = 0;
uni_hid_device_t* last_disconnected_device = nullptr;

struct CoreStopped {};


void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

void play_rumble(uni_hid_device_t* device, uint16_t,
                 uint16_t duration_ms, uint8_t high, uint8_t low) {
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
    result.conn.btaddr[5] = static_cast<uint8_t>(idx + 1);
    result.report_parser.play_dual_rumble = play_rumble;
    return result;
}

}  // namespace


bool uni_hid_device_is_gamepad(const uni_hid_device_t* device) {
    return device != nullptr && device->gamepad;
}

int uni_hid_device_get_idx_for_instance(const uni_hid_device_t* device) {
    return device == nullptr ? -1 : device->idx;
}

void uni_hid_device_disconnect(uni_hid_device_t* device) {
    ++device_disconnect_calls;
    last_disconnected_device = device;
}

void uni_bt_allow_incoming_connections(bool enabled) {
    incoming_connections = enabled;
}


void uni_bt_bredr_scan_start() {
    ++classic_scan_starts;
    classic_scanning_enabled = true;
}

void uni_bt_bredr_scan_stop() {
    if (classic_scanning_enabled) {
        ++classic_scan_stops;
    }
    classic_scanning_enabled = false;
}

void uni_bt_le_scan_start() {
    ++scan_starts;
    scanning_enabled = true;
}

void uni_bt_le_scan_stop() {
    if (scanning_enabled) {
        ++scan_stops;
    }
    scanning_enabled = false;
}

void uni_bt_start_scanning_and_autoconnect_unsafe() {
    uni_bt_bredr_scan_start();
    uni_bt_le_scan_start();
}

void uni_bt_stop_scanning_unsafe() {
    uni_bt_bredr_scan_stop();
    uni_bt_le_scan_stop();
}
void uni_bt_del_keys_unsafe() {
    ++delete_key_calls;
    classic_bond_count = 0;
    ble_bond_count = 0;
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


void uni_platform_set_custom(uni_platform* platform) {
    installed_platform = platform;
}

int uni_init(int, const char**) {
    ++uni_init_calls;
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

void multicore_launch_core1(void (*)()) {
    ++core1_launch_calls;
}

void tight_loop_contents() {
    throw CoreStopped{};
}

uint32_t btstack_run_loop_get_time_ms() {
    return now_ms;
}


#include "../bluepad32_input_backend.cpp"
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

void start_backend() {
    bluepad32_input_backend_init();
    platform_on_init_complete();
    require(incoming_connections && scanning_enabled &&
                classic_scanning_enabled && scan_starts == 1 &&
                link_supervision_timeout ==
                    kClassicLinkSupervisionTimeout &&
                !bondable && accepted_stk_methods == 0 &&
                !ssp_auto_accept && pairing_event_handler != nullptr,
            "initialization must configure liveness and pairing policy");
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




void tick_backend_timer(int ticks) {
    for (int tick = 0; tick < ticks; ++tick) {
        process_rumble_timer(&g_rumble_timer);
    }
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
            SwitchInputState snapshot{};
            bool expected_active = false;
            for (int ready = 0; ready <= position; ++ready) {
                expected_active = expected_active || order[ready] == candidate;
            }
            require(bluepad32_input_backend_snapshot(candidate, &snapshot) ==
                        expected_active,
                    "only ready indexed slots may become active");
        }

        if (position + 1 < kSlotCount) {
            require(scan_stops == 0,
                    "scanning must continue while any slot remains free");
            require(incoming_connections,
                    "incoming connections must remain enabled before all slots are ready");
        }
    }

    require(scan_stops == 1,
            "scanning must stop exactly when all four slots are ready");
    require(!incoming_connections,
            "incoming connections must be disabled only when all slots are full");

    for (int slot = 0; slot < kSlotCount; ++slot) {
        const int starts_before_disconnect = scan_starts;
        platform_on_device_disconnected(&devices[slot]);
        require(scan_starts == starts_before_disconnect + 1 &&
                    scanning_enabled && incoming_connections,
                "disconnecting any slot must resume connection policy");

        for (int candidate = 0; candidate < kSlotCount; ++candidate) {
            SwitchInputState snapshot{};
            require(bluepad32_input_backend_snapshot(candidate, &snapshot) ==
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

    uni_controller_t collision_data{};
    collision_data.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    collision_data.gamepad.buttons = BUTTON_B;
    platform_on_controller_data(&collision, &collision_data);
    SwitchInputState snapshot{};
    require(bluepad32_input_backend_snapshot(0, &snapshot),
            "occupied slot must stay active");
    require(!snapshot.button_a,
            "mismatched device input must not enter the occupied slot");

    uni_controller_t slot_zero_data{};
    slot_zero_data.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    slot_zero_data.gamepad.accel[0] = 8192;
    platform_on_controller_data(&slot_zero, &slot_zero_data);
    require(bluepad32_input_backend_snapshot(0, &snapshot) &&
                snapshot.imu_sample_count == 3,
            "valid slot input must remain observable");
    require(!bluepad32_input_backend_snapshot(4, &snapshot),
            "public snapshot must reject slot 4");
    bluepad32_input_backend_report_sent(4);
    require(bluepad32_input_backend_snapshot(0, &snapshot) &&
                snapshot.imu_sample_count == 3,
            "slot 4 acknowledgement must not consume slot 0 IMU");
    bluepad32_input_backend_queue_rumble(4, SwitchRumbleOutput{1, 2});
    process_rumble_timer(&g_rumble_timer);
    require(slot_zero.rumble_calls == 0,
            "slot 4 rumble must not reach a valid controller");
}

void test_independent_lifecycle() {
    start_pairing_backend();

    uni_hid_device_t aborted = device(0);
    const uint32_t aborted_generation = g_slots[0].connection_generation;
    platform_on_device_connected(&aborted);
    require(g_slots[0].device == &aborted && !g_slots[0].active,
            "connected device must remain identifiable while becoming ready");

    const int starts_before_aborted_disconnect = scan_starts;
    const int classic_starts_before_aborted_disconnect =
        classic_scan_starts;
    const int stops_before_aborted_disconnect = scan_stops;
    const int classic_stops_before_aborted_disconnect =
        classic_scan_stops;
    platform_on_device_disconnected(&aborted);
    require(g_slots[0].device == nullptr && !g_slots[0].active,
            "pre-ready disconnect must clear its pending slot identity");
    require(g_slots[0].connection_generation == aborted_generation + 1,
            "pre-ready disconnect must invalidate its connection generation");
    require(g_connection_status == ConnectionStatus::Scanning &&
                scanning_enabled && classic_scanning_enabled &&
                incoming_connections &&
                scan_starts == starts_before_aborted_disconnect + 1 &&
                classic_scan_starts ==
                    classic_starts_before_aborted_disconnect + 1 &&
                scan_stops == stops_before_aborted_disconnect + 1 &&
                classic_scan_stops ==
                    classic_stops_before_aborted_disconnect + 1,
            "pre-ready disconnect must restart Classic and BLE scans");

    uni_hid_device_t devices[kSlotCount] = {
        device(0), device(1), device(2), device(3)};
    for (int slot = 0; slot < kSlotCount; ++slot) {
        platform_on_device_connected(&devices[slot]);
        require(g_slots[slot].device == &devices[slot] &&
                    !g_slots[slot].active,
                "each pending device must retain its indexed identity");
    }

    const uint32_t first_pending_generation =
        g_slots[0].connection_generation;
    const int starts_before_first_pending_disconnect = scan_starts;
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
                incoming_connections &&
                scan_starts == starts_before_first_pending_disconnect + 1,
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

    SwitchInputState states[kSlotCount]{};
    for (int slot = 0; slot < kSlotCount; ++slot) {
        require(bluepad32_input_backend_snapshot(slot, &states[slot]) &&
                    states[slot].imu_sample_count == 3,
                "every slot must expose independent input and IMU");
    }
    require(states[0].button_a && !states[0].button_b &&
                !states[0].button_y && !states[0].button_x,
            "slot 0 must contain only slot 0 input");
    require(states[1].button_b && !states[1].button_a &&
                !states[1].button_y && !states[1].button_x,
            "slot 1 must contain only slot 1 input");
    require(states[2].button_y && !states[2].button_a &&
                !states[2].button_b && !states[2].button_x,
            "slot 2 must contain only slot 2 input");
    require(states[3].button_x && !states[3].button_a &&
                !states[3].button_b && !states[3].button_y,
            "slot 3 must contain only slot 3 input");

    bluepad32_input_backend_report_sent(3);
    for (int slot = 0; slot < kSlotCount; ++slot) {
        require(bluepad32_input_backend_snapshot(slot, &states[slot]) &&
                    states[slot].imu_sample_count == (slot == 3 ? 0 : 3),
                "slot 3 acknowledgement must not consume slots 0-2 IMU");
    }
    for (int slot = 0; slot < 3; ++slot) {
        bluepad32_input_backend_report_sent(slot);
        require(bluepad32_input_backend_snapshot(slot, &states[slot]) &&
                    states[slot].imu_sample_count == 0,
                "each slot acknowledgement must consume only its own IMU");
    }

    const SwitchRumbleOutput initial_rumble[kSlotCount] = {
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
                        kRumbleDurationMs,
                "each slot rumble must reach only its indexed controller");
    }

    bluepad32_input_backend_queue_rumble(0, SwitchRumbleOutput{0, 0});
    process_rumble_timer(&g_rumble_timer);
    require(devices[0].rumble_calls == 2 &&
                devices[0].last_rumble_duration_ms == 0,
            "zero XInput magnitude must stop rumble immediately");

    bluepad32_input_backend_queue_rumble(3, SwitchRumbleOutput{55, 66});
    const uint32_t disconnected_generation =
        g_slots[3].connection_generation;
    const int starts_before_slot_three_disconnect = scan_starts;
    platform_on_device_disconnected(&devices[3]);
    require(scan_starts == starts_before_slot_three_disconnect + 1 &&
                scanning_enabled && incoming_connections,
            "slot 3 disconnect must resume scanning and incoming connections");
    require(!bluepad32_input_backend_snapshot(3, &states[3]) &&
                !states[3].button_x && states[3].lx == 32768,
            "slot 3 disconnect must neutralize only slot 3");
    require(bluepad32_input_backend_snapshot(0, &states[0]) &&
                states[0].button_a &&
                bluepad32_input_backend_snapshot(1, &states[1]) &&
                states[1].button_b &&
                bluepad32_input_backend_snapshot(2, &states[2]) &&
                states[2].button_y,
            "slot 3 disconnect must preserve slots 0-2");
    platform_on_controller_data(&devices[0], &data[0]);
    require(bluepad32_input_backend_snapshot(0, &states[0]) &&
                states[0].button_a,
            "slot 0 input must continue while slot 3 is disconnected");
    const int slot_zero_calls_while_scanning = devices[0].rumble_calls;
    bluepad32_input_backend_queue_rumble(0, SwitchRumbleOutput{115, 116});
    tick_backend_timer(99);
    require(devices[0].rumble_calls == slot_zero_calls_while_scanning + 1 &&
                devices[0].last_low == 115 &&
                devices[0].last_high == 116,
            "slot 0 rumble must continue while slot 3 is disconnected");

    uni_hid_device_t slot_three_replacement = device(3);
    require(platform_on_device_ready(&slot_three_replacement) ==
                UNI_ERROR_SUCCESS,
            "slot 3 replacement must bind to the freed indexed slot");
    process_rumble_timer(&g_rumble_timer);
    require(slot_three_replacement.rumble_calls == 0,
            "slot 3 replacement must not receive disconnected device rumble");

    g_slots[3].pending_rumble = {
        3, disconnected_generation, SwitchRumbleOutput{77, 88}};
    g_slots[3].rumble_pending = true;
    process_rumble_timer(&g_rumble_timer);
    require(slot_three_replacement.rumble_calls == 0,
            "stale slot 3 connection generation must be rejected");

    uni_controller_t replacement_data{};
    replacement_data.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    replacement_data.gamepad.buttons = BUTTON_Y;
    replacement_data.gamepad.accel[0] = 9000;
    platform_on_controller_data(&slot_three_replacement, &replacement_data);
    require(bluepad32_input_backend_snapshot(3, &states[3]) &&
                states[3].button_x && states[3].imu_sample_count == 3,
            "replacement input and IMU must populate only slot 3");
    require(bluepad32_input_backend_snapshot(0, &states[0]) &&
                states[0].button_a &&
                bluepad32_input_backend_snapshot(1, &states[1]) &&
                states[1].button_b &&
                bluepad32_input_backend_snapshot(2, &states[2]) &&
                states[2].button_y,
            "slot 3 replacement must not disturb slots 0-2");

    const int survivor_calls[kSlotCount - 1] = {
        devices[0].rumble_calls,
        devices[1].rumble_calls,
        devices[2].rumble_calls};
    bluepad32_input_backend_queue_rumble(3, SwitchRumbleOutput{90, 91});
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
            slot, SwitchRumbleOutput{static_cast<uint8_t>(100 + slot),
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
        const int starts_before_disconnect = scan_starts;
        platform_on_device_disconnected(&devices[slot]);
        require(scan_starts == starts_before_disconnect + 1 &&
                    scanning_enabled && incoming_connections,
                "disconnecting slots 0-2 must resume connection policy");
        require(!bluepad32_input_backend_snapshot(slot, &states[slot]) &&
                    states[slot].lx == 32768,
                "disconnect must neutralize its indexed slot");
        for (int survivor = 0; survivor < kSlotCount; ++survivor) {
            if (survivor == slot) {
                continue;
            }
            require(bluepad32_input_backend_snapshot(survivor,
                                                     &states[survivor]),
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
    bluepad32_input_backend_queue_rumble(3, SwitchRumbleOutput{119, 120});
    bluepad32_input_backend_queue_rumble(3, SwitchRumbleOutput{121, 122});
    bluepad32_input_backend_queue_rumble(0, SwitchRumbleOutput{123, 124});
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

void require_south_button_mapping(const SwitchInputState& state,
                                  bool swapped,
                                  const char* message) {
    require(state.button_a == swapped && state.button_b == !swapped &&
                !state.button_x && !state.button_y,
            message);
}

void test_abxy_hotkey() {
    start_pairing_backend();
    uni_hid_device_t slot_zero = device(0);
    uni_hid_device_t slot_one = device(1);
    require(platform_on_device_ready(&slot_zero) == UNI_ERROR_SUCCESS &&
                platform_on_device_ready(&slot_one) == UNI_ERROR_SUCCESS,
            "ABXY test controllers did not become ready");

    uni_controller_t input{};
    input.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    input.gamepad.buttons = BUTTON_A;
    platform_on_controller_data(&slot_zero, &input);
    SwitchInputState snapshot{};
    require(bluepad32_input_backend_snapshot(0, &snapshot),
            "slot 0 ABXY state was not published");
    require_south_button_mapping(
        snapshot, kDefaultSwapAbxy,
        "slot 0 did not start in the configured ABXY layout");

    input.gamepad.buttons =
        kAbxyHotkeyButtonMask | BUTTON_A;
    input.gamepad.misc_buttons = kAbxyHotkeyMiscMask;
    platform_on_controller_data(&slot_zero, &input);
    require(bluepad32_input_backend_snapshot(0, &snapshot),
            "toggled slot 0 state was not published");
    require_south_button_mapping(
        snapshot, !kDefaultSwapAbxy,
        "hotkey did not toggle slot 0 ABXY mapping");
    require(!snapshot.button_l && !snapshot.button_r &&
                !snapshot.button_minus && !snapshot.button_plus,
            "hotkey chord leaked into the Switch report");

    bluepad32_input_backend_queue_rumble(
        0, SwitchRumbleOutput{0x11, 0x22});
    process_rumble_timer(&g_rumble_timer);
    require(slot_zero.rumble_calls == 1 &&
                slot_zero.last_high == kAbxyFeedbackWeakMagnitude &&
                slot_zero.last_low == kAbxyFeedbackStrongMagnitude &&
                slot_zero.last_high == UINT8_MAX &&
                slot_zero.last_low == UINT8_MAX &&
                g_slots[0].rumble_pending,
            "ABXY confirmation was not full-strength or did not take priority");

    platform_on_controller_data(&slot_zero, &input);
    process_rumble_timer(&g_rumble_timer);
    require(g_slots[0].swap_abxy == !kDefaultSwapAbxy &&
                slot_zero.rumble_calls == 1,
            "held hotkey toggled or rumbled more than once");

    input.gamepad = {};
    platform_on_controller_data(&slot_zero, &input);
    input.gamepad.buttons = kAbxyHotkeyButtonMask | BUTTON_A;
    input.gamepad.misc_buttons = kAbxyHotkeyMiscMask;
    platform_on_controller_data(&slot_zero, &input);
    process_rumble_timer(&g_rumble_timer);
    require(g_slots[0].swap_abxy == kDefaultSwapAbxy &&
                slot_zero.rumble_calls == 2,
            "released hotkey did not re-arm for a second toggle");

    now_ms = kAbxyFeedbackDurationMs - 1;
    process_rumble_timer(&g_rumble_timer);
    require(slot_zero.rumble_calls == 2 && g_slots[0].rumble_pending,
            "host rumble interrupted ABXY confirmation");
    now_ms = kAbxyFeedbackDurationMs;
    process_rumble_timer(&g_rumble_timer);
    require(slot_zero.rumble_calls == 3 &&
                slot_zero.last_high == 0x22 &&
                slot_zero.last_low == 0x11 &&
                !g_slots[0].rumble_pending,
            "deferred host rumble did not resume after confirmation");

    uni_controller_t peer_input{};
    peer_input.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    peer_input.gamepad.buttons = BUTTON_A;
    platform_on_controller_data(&slot_one, &peer_input);
    require(bluepad32_input_backend_snapshot(1, &snapshot),
            "slot 1 ABXY state was not published");
    require_south_button_mapping(
        snapshot, kDefaultSwapAbxy,
        "slot 0 hotkey changed slot 1 layout");

    platform_on_device_disconnected(&slot_zero);
    uni_hid_device_t replacement = device(0);
    require(platform_on_device_ready(&replacement) == UNI_ERROR_SUCCESS &&
                g_slots[0].swap_abxy == kDefaultSwapAbxy &&
                !g_slots[0].abxy_hotkey_latched &&
                !g_slots[0].feedback_pending,
            "disconnect did not reset slot 0 hotkey state");
}

void test_motion_hotkey() {
    start_pairing_backend();
    uni_hid_device_t slot_zero = device(0);
    uni_hid_device_t slot_one = device(1);
    require(platform_on_device_ready(&slot_zero) == UNI_ERROR_SUCCESS &&
                platform_on_device_ready(&slot_one) == UNI_ERROR_SUCCESS,
            "motion hotkey test controllers did not become ready");

    uni_controller_t input{};
    input.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    input.gamepad.accel[0] = 8192;
    platform_on_controller_data(&slot_zero, &input);
    SwitchInputState snapshot{};
    require(bluepad32_input_backend_snapshot(0, &snapshot) &&
                snapshot.imu_sample_count ==
                    (kDefaultMotionEnabled ? 3 : 0),
            "slot 0 did not start with configured motion state");

    input.gamepad.dpad = kMotionHotkeyDpadMask;
    input.gamepad.buttons = kMotionHotkeyButtonMask;
    input.gamepad.misc_buttons = kMotionHotkeyMiscMask;
    platform_on_controller_data(&slot_zero, &input);
    require(bluepad32_input_backend_snapshot(0, &snapshot) &&
                snapshot.imu_sample_count ==
                    (kDefaultMotionEnabled ? 0 : 3) &&
                !snapshot.dpad_up && !snapshot.button_r &&
                !snapshot.button_plus,
            "motion chord did not toggle motion or suppress its inputs");

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

    platform_on_controller_data(&slot_zero, &input);
    process_rumble_timer(&g_rumble_timer);
    require(g_slots[0].motion_enabled == !kDefaultMotionEnabled &&
                slot_zero.rumble_calls == 1,
            "held motion chord toggled or rumbled more than once");

    uni_controller_t peer_input{};
    peer_input.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    peer_input.gamepad.accel[0] = 8192;
    platform_on_controller_data(&slot_one, &peer_input);
    require(bluepad32_input_backend_snapshot(1, &snapshot) &&
                snapshot.imu_sample_count ==
                    (kDefaultMotionEnabled ? 3 : 0),
            "slot 0 motion chord changed slot 1 motion state");

    input.gamepad = {};
    platform_on_controller_data(&slot_zero, &input);
    input.gamepad.accel[0] = 8192;
    input.gamepad.dpad = kMotionHotkeyDpadMask;
    input.gamepad.buttons = kMotionHotkeyButtonMask;
    input.gamepad.misc_buttons = kMotionHotkeyMiscMask;
    platform_on_controller_data(&slot_zero, &input);
    require(bluepad32_input_backend_snapshot(0, &snapshot) &&
                snapshot.imu_sample_count ==
                    (kDefaultMotionEnabled ? 3 : 0),
            "released motion chord did not re-arm or restore motion");
    process_rumble_timer(&g_rumble_timer);
    require(g_slots[0].motion_enabled == kDefaultMotionEnabled &&
                slot_zero.rumble_calls == 2,
            "second motion chord did not restore configured state");

    platform_on_device_disconnected(&slot_zero);
    uni_hid_device_t replacement = device(0);
    require(platform_on_device_ready(&replacement) == UNI_ERROR_SUCCESS &&
                g_slots[0].motion_enabled == kDefaultMotionEnabled &&
                !g_slots[0].motion_hotkey_latched &&
                !g_slots[0].feedback_pending,
            "disconnect did not reset slot 0 motion hotkey state");
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
    const uint32_t snapshot_generation =
        g_pairing_snapshot.generation;
    uni_hid_device_t devices[2] = {device(0), device(1)};
    for (uni_hid_device_t& controller : devices) {
        require(platform_on_device_ready(&controller) == UNI_ERROR_SUCCESS,
                "pairing reset controller did not become ready");
    }
    bluepad32_input_backend_queue_rumble(
        0, SwitchRumbleOutput{100, 101});

    bluepad32_input_backend_clear_pairings();
    require(g_clear_pairings_requested && delete_key_calls == 0 &&
                device_disconnect_calls == 0,
            "Core0 pairing reset request must wait for Core1");
    process_rumble_timer(&g_rumble_timer);

    require(delete_key_calls == 1 && device_disconnect_calls == 2,
            "pairing reset must delete bonds and disconnect every session");
    require(g_pairing_snapshot.status ==
                    Bluepad32PairingSnapshotStatus::kReady &&
                g_pairing_snapshot.record_count == 0 &&
                g_pairing_snapshot.generation ==
                    snapshot_generation + 1,
            "pairing reset must publish an empty refreshed snapshot");
    for (const BackendSlot& slot : g_slots) {
        require(slot.device == nullptr && !slot.active &&
                    !slot.rumble_pending && !slot.feedback_pending &&
                    slot.state.lx == kStickMidpoint &&
                    slot.state.ly == kStickMidpoint &&
                    slot.state.rx == kStickMidpoint &&
                    slot.state.ry == kStickMidpoint &&
                    slot.state.imu_sample_count == 0,
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
    require(delete_key_calls == 1 && device_disconnect_calls == 2,
            "pairing reset request must execute only once");
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

}  // namespace

int main(int argc, char** argv) {
    require(argc == 2, "scenario argument required");
    const std::string scenario = argv[1];
    if (scenario == "ready-forward") {
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
    } else if (scenario == "abxy-hotkey") {
        test_abxy_hotkey();
    } else if (scenario == "motion-hotkey") {
        test_motion_hotkey();
    } else if (scenario == "clear-pairings") {
        test_clear_pairings();
    } else if (scenario == "flash-core-start") {
        test_flash_core_start_contract();
    } else if (scenario == "flash-core-failure") {
        test_flash_core_init_fatal();
    } else {
        require(false, "unknown scenario");
    }
    return 0;
}
