#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include <uni.h>
#include "platform/pico/controller_color_config.h"

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
    result.conn.handle = static_cast<hci_con_handle_t>(0x40 + idx);
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
    require_clear_completion_pending();
    require_clear_snapshot_published();
    uni_bt_bredr_scan_start();
    uni_bt_le_scan_start();
}

void uni_bt_stop_scanning_unsafe() {
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


#include "core/controller_identity.cpp"
#include "input/bluepad32_input_backend.cpp"

namespace {
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
    return true;
}
void configuration_service_snapshot(ConfigurationServiceSnapshot* output) {
    *output = {};
    output->state = ConfigurationServiceState::kReady;
    output->configuration.pairing_window_seconds =
        ADAPTER_PAIRING_WINDOW_SECONDS_DEFAULT;
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
                classic_scanning_enabled && scan_starts == 1 &&
                link_supervision_timeout ==
                    kClassicLinkSupervisionTimeout &&
                !bondable && accepted_stk_methods == 0 &&
                !ssp_auto_accept && pairing_event_handler != nullptr &&
                identity_event_handler != nullptr,
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
    const int starts_before_slot_three_disconnect = scan_starts;
    platform_on_device_disconnected(&devices[3]);
    require(scan_starts == starts_before_slot_three_disconnect + 1 &&
                scanning_enabled && incoming_connections,
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
        const int starts_before_disconnect = scan_starts;
        platform_on_device_disconnected(&devices[slot]);
        require(scan_starts == starts_before_disconnect + 1 &&
                    scanning_enabled && incoming_connections,
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
    ControllerState snapshot{};
    require(read_controller_state(0, &snapshot) &&
                snapshot.motion_sample_count ==
                    (kDefaultMotionEnabled ? 3 : 0),
            "slot 0 did not start with configured motion state");

    input.gamepad.dpad = kMotionHotkeyDpadMask;
    input.gamepad.buttons = kMotionHotkeyButtonMask;
    input.gamepad.misc_buttons = kMotionHotkeyMiscMask;
    platform_on_controller_data(&slot_zero, &input);
    Bluepad32SlotSnapshot backend_snapshot{};
    bluepad32_input_backend_snapshot(0, &backend_snapshot);
    require(read_controller_state(0, &snapshot) &&
                backend_snapshot.pre_hotkey_button_mask ==
                    kMotionHotkeyLogicalButtonMask &&
                snapshot.motion_sample_count ==
                    (kDefaultMotionEnabled ? 0 : 3) &&
                !snapshot.dpad_up && !snapshot.button_right_shoulder &&
                !snapshot.button_start,
            "motion chord was not published pre-hotkey or suppressed "
            "from normal output");

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
    require(read_controller_state(1, &snapshot) &&
                snapshot.motion_sample_count ==
                    (kDefaultMotionEnabled ? 3 : 0),
            "slot 0 motion chord changed slot 1 motion state");

    input.gamepad = {};
    platform_on_controller_data(&slot_zero, &input);
    input.gamepad.accel[0] = 8192;
    input.gamepad.dpad = kMotionHotkeyDpadMask;
    input.gamepad.buttons = kMotionHotkeyButtonMask;
    input.gamepad.misc_buttons = kMotionHotkeyMiscMask;
    platform_on_controller_data(&slot_zero, &input);
    require(read_controller_state(0, &snapshot) &&
                snapshot.motion_sample_count ==
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
                g_slots[0].pre_hotkey_button_mask == 0 &&
                !g_slots[0].feedback_pending,
            "disconnect did not reset slot 0 motion hotkey state");
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
    } else if (scenario == "stateful-rumble") {
        test_stateful_host_rumble_restore();
    } else if (scenario == "profile-chord-raw") {
        test_profile_chord_remains_raw();
    } else if (scenario == "profile-feedback") {
        test_profile_feedback_scheduler();
    } else if (scenario == "motion-hotkey") {
        test_motion_hotkey();
    } else if (scenario == "analog-state") {
        test_protocol_neutral_analog_state();
    } else if (scenario == "rumble-mode") {
        test_host_rumble_mode_duration();
    } else if (scenario == "clear-pairings") {
        test_clear_pairings();
    } else if (scenario == "configuration-timer") {
        test_configuration_timer_rearms_before_storage_work();
    } else if (scenario == "flash-core-start") {
        test_flash_core_start_contract();
    } else if (scenario == "flash-core-failure") {
        test_flash_core_init_fatal();
    } else {
        require(false, "unknown scenario");
    }
    return 0;
}
