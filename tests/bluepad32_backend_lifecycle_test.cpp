#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include <uni.h>

namespace {

bool incoming_connections = false;
int scan_starts = 0;
int scan_stops = 0;
bool scanning_enabled = false;
uni_platform* installed_platform = nullptr;
bool observed_status_led_on = false;
int observed_status_led_writes = 0;

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

void play_rumble(uni_hid_device_t* device, uint16_t, uint16_t,
                 uint8_t high, uint8_t low) {
    ++device->rumble_calls;
    device->last_high = high;
    device->last_low = low;
}

uni_hid_device_t device(int idx, bool gamepad = true) {
    uni_hid_device_t result{};
    result.idx = idx;
    result.gamepad = gamepad;
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

void uni_bt_allow_incoming_connections(bool enabled) {
    incoming_connections = enabled;
}

void uni_bt_start_scanning_and_autoconnect_unsafe() {
    ++scan_starts;
    scanning_enabled = true;
}

void uni_bt_stop_scanning_unsafe() {
    ++scan_stops;
    scanning_enabled = false;
}

void uni_platform_set_custom(uni_platform* platform) {
    installed_platform = platform;
}

int uni_init(int, const char**) {
    return 0;
}

int cyw43_arch_init() {
    return 0;
}

void cyw43_arch_gpio_put(int, bool enabled) {
    observed_status_led_on = enabled;
    ++observed_status_led_writes;
}

void multicore_launch_core1(void (*)()) {}

#include "../bluepad32_input_backend.cpp"

namespace {

void start_backend() {
    bluepad32_input_backend_init();
    platform_on_init_complete();
    require(incoming_connections, "initialization must allow connections");
    require(scan_starts == 1, "initialization must start scanning");
}

void tick_backend_timer(int ticks) {
    for (int tick = 0; tick < ticks; ++tick) {
        process_rumble_timer(&g_rumble_timer);
    }
}

void test_ready_order(bool reverse) {
    start_backend();
    uni_hid_device_t devices[kSlotCount] = {
        device(0), device(1), device(2), device(3)};
    uni_hid_device_t replacements[kSlotCount] = {
        device(0), device(1), device(2), device(3)};
    const int forward[kSlotCount] = {0, 1, 2, 3};
    const int backward[kSlotCount] = {3, 2, 1, 0};
    const int* order = reverse ? backward : forward;

    tick_backend_timer(99);
    require(observed_status_led_on,
            "scanning LED must stay on for the first slow-blink half-cycle");
    tick_backend_timer(1);
    require(!observed_status_led_on,
            "scanning LED must turn off at the slow-blink half-cycle");

    platform_on_device_connected(&devices[order[0]]);
    tick_backend_timer(19);
    require(observed_status_led_on,
            "connecting LED must stay on for the first fast-blink half-cycle");
    tick_backend_timer(1);
    require(!observed_status_led_on,
            "connecting LED must turn off at the fast-blink half-cycle");
    tick_backend_timer(20);
    require(observed_status_led_on,
            "connecting LED must turn on for the next fast-blink cycle");

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
            if (position == 0) {
                tick_backend_timer(99);
                require(observed_status_led_on,
                        "a partially full backend must use the scanning LED on half-cycle");
                tick_backend_timer(1);
                require(!observed_status_led_on,
                        "a partially full backend must use the scanning LED off half-cycle");
            }
        }
    }

    require(scan_stops == 1,
            "scanning must stop exactly when all four slots are ready");
    require(!incoming_connections,
            "incoming connections must be disabled only when all slots are full");
    tick_backend_timer(1);
    require(observed_status_led_on,
            "four ready slots must turn the status LED on");
    const int ready_led_writes = observed_status_led_writes;
    tick_backend_timer(200);
    require(observed_status_led_on &&
                observed_status_led_writes == ready_led_writes,
            "four ready slots must keep the status LED solid");

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
    start_backend();

    uni_hid_device_t aborted = device(0);
    const uint32_t aborted_generation = g_slots[0].connection_generation;
    platform_on_device_connected(&aborted);
    require(g_slots[0].device == &aborted && !g_slots[0].active,
            "connected device must remain identifiable while becoming ready");
    tick_backend_timer(19);
    require(observed_status_led_on,
            "a lone pending connection must use the fast LED on half-cycle");
    tick_backend_timer(1);
    require(!observed_status_led_on,
            "a lone pending connection must use the fast LED off half-cycle");

    const int starts_before_aborted_disconnect = scan_starts;
    platform_on_device_disconnected(&aborted);
    require(g_slots[0].device == nullptr && !g_slots[0].active,
            "pre-ready disconnect must clear its pending slot identity");
    require(g_slots[0].connection_generation == aborted_generation + 1,
            "pre-ready disconnect must invalidate its connection generation");
    require(g_connection_status == ConnectionStatus::Scanning &&
                scanning_enabled && incoming_connections &&
                scan_starts == starts_before_aborted_disconnect + 1,
            "pre-ready disconnect with no peer must resume scanning");
    tick_backend_timer(99);
    require(observed_status_led_on,
            "pre-ready disconnect must restore the slow LED on half-cycle");
    tick_backend_timer(1);
    require(!observed_status_led_on,
            "pre-ready disconnect must restore the slow LED off half-cycle");

    uni_hid_device_t devices[kSlotCount] = {
        device(0), device(1), device(2), device(3)};
    for (int slot = 0; slot < kSlotCount; ++slot) {
        platform_on_device_connected(&devices[slot]);
        require(g_slots[slot].device == &devices[slot] &&
                    !g_slots[slot].active,
                "each pending device must retain its indexed identity");
    }
    tick_backend_timer(19);
    require(observed_status_led_on,
            "concurrent pending devices must use the fast LED on half-cycle");
    tick_backend_timer(1);
    require(!observed_status_led_on,
            "concurrent pending devices must use the fast LED off half-cycle");

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
                scanning_enabled && incoming_connections &&
                scan_starts == starts_before_first_pending_disconnect + 1,
            "open slot must scan while other slots remain connecting");
    tick_backend_timer(19);
    require(observed_status_led_on,
            "surviving pending devices must retain the fast LED on half-cycle");
    tick_backend_timer(1);
    require(!observed_status_led_on,
            "surviving pending devices must retain the fast LED off half-cycle");

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
                    devices[slot].last_high == 21 + slot,
                "each slot rumble must reach only its indexed controller");
    }

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
    require(observed_status_led_on,
            "disconnect scanning must use the slow LED on half-cycle");
    tick_backend_timer(1);
    require(!observed_status_led_on,
            "disconnect scanning must reach the slow LED off half-cycle");

    uni_hid_device_t slot_three_replacement = device(3);
    require(platform_on_device_ready(&slot_three_replacement) ==
                UNI_ERROR_SUCCESS,
            "slot 3 replacement must bind to the freed indexed slot");
    process_rumble_timer(&g_rumble_timer);
    require(slot_three_replacement.rumble_calls == 0,
            "slot 3 replacement must not receive disconnected device rumble");
    require(observed_status_led_on,
            "slot 3 replacement must restore the solid ready LED");
    const int replacement_ready_led_writes = observed_status_led_writes;
    tick_backend_timer(100);
    require(observed_status_led_on &&
                observed_status_led_writes == replacement_ready_led_writes,
            "replacement quartet must keep the ready LED solid");

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
    } else {
        require(false, "unknown scenario");
    }
    return 0;
}
