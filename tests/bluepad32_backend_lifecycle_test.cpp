#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include <uni.h>

namespace {

bool incoming_connections = false;
int scan_starts = 0;
int scan_stops = 0;
uni_platform* installed_platform = nullptr;

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
}

void uni_bt_stop_scanning_unsafe() {
    ++scan_stops;
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

void cyw43_arch_gpio_put(int, bool) {}
void multicore_launch_core1(void (*)()) {}

#include "../bluepad32_input_backend.cpp"

namespace {

void start_backend() {
    bluepad32_input_backend_init();
    platform_on_init_complete();
    require(incoming_connections, "initialization must allow connections");
    require(scan_starts == 1, "initialization must start scanning");
}

void test_ready_order(int first_slot) {
    start_backend();
    uni_hid_device_t devices[2] = {device(0), device(1)};
    const int second_slot = 1 - first_slot;

    require(platform_on_device_ready(&devices[first_slot]) ==
                UNI_ERROR_SUCCESS,
            "first ready device must bind to its Bluepad index");
    require(scan_stops == 0,
            "scanning must continue while one slot remains free");
    require(incoming_connections,
            "incoming connections must remain enabled with one ready slot");

    SwitchInputState first{};
    SwitchInputState second{};
    require(bluepad32_input_backend_snapshot(first_slot, &first),
            "first ready slot must be active");
    require(!bluepad32_input_backend_snapshot(second_slot, &second),
            "other slot must remain independently inactive");

    require(platform_on_device_ready(&devices[second_slot]) ==
                UNI_ERROR_SUCCESS,
            "second ready device must bind to its Bluepad index");
    require(scan_stops == 1,
            "scanning must stop exactly when both slots are ready");
    require(!incoming_connections,
            "incoming connections must be disabled only when full");

    bd_addr_t address{};
    require(platform_on_device_discovered(address, "extra", 0, 0) ==
                UNI_ERROR_IGNORE_DEVICE,
            "discovery must reject devices while both slots are occupied");
}

void test_rejections() {
    start_backend();
    uni_hid_device_t non_gamepad = device(0, false);
    uni_hid_device_t out_of_range = device(2);
    uni_hid_device_t slot_zero = device(0);
    uni_hid_device_t collision = device(0);

    require(platform_on_device_ready(&non_gamepad) ==
                UNI_ERROR_INVALID_CONTROLLER,
            "non-gamepad must be rejected");
    require(platform_on_device_ready(&out_of_range) == UNI_ERROR_NO_SLOTS,
            "out-of-range Bluepad index must be rejected");
    require(platform_on_device_ready(&slot_zero) == UNI_ERROR_SUCCESS,
            "valid device must occupy its indexed slot");
    require(platform_on_device_ready(&collision) == UNI_ERROR_NO_SLOTS,
            "different device cannot replace an occupied slot");

    uni_controller_t data{};
    data.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    data.gamepad.buttons = BUTTON_B;
    platform_on_controller_data(&collision, &data);
    SwitchInputState snapshot{};
    require(bluepad32_input_backend_snapshot(0, &snapshot),
            "occupied slot must stay active");
    require(!snapshot.button_a,
            "mismatched device input must not enter the occupied slot");
}

void test_independent_lifecycle() {
    start_backend();
    uni_hid_device_t first = device(0);
    uni_hid_device_t survivor = device(1);
    require(platform_on_device_ready(&survivor) == UNI_ERROR_SUCCESS,
            "slot 1 must be accepted before slot 0");
    require(platform_on_device_ready(&first) == UNI_ERROR_SUCCESS,
            "slot 0 must complete the pair");

    uni_controller_t data0{};
    data0.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    data0.gamepad.buttons = BUTTON_B;
    data0.gamepad.accel[0] = 8192;
    uni_controller_t data1{};
    data1.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    data1.gamepad.buttons = BUTTON_A;
    data1.gamepad.gyro[1] = 1024;
    platform_on_controller_data(&first, &data0);
    platform_on_controller_data(&survivor, &data1);

    SwitchInputState state0{};
    SwitchInputState state1{};
    require(bluepad32_input_backend_snapshot(0, &state0) && state0.button_a &&
                state0.imu_sample_count == 3,
            "slot 0 input and IMU must map only to slot 0");
    require(bluepad32_input_backend_snapshot(1, &state1) && state1.button_b &&
                state1.imu_sample_count == 3,
            "slot 1 input and IMU must map only to slot 1");

    bluepad32_input_backend_report_sent(0);
    require(bluepad32_input_backend_snapshot(0, &state0) &&
                state0.imu_sample_count == 0,
            "slot 0 report acknowledgement must consume only slot 0 IMU");
    require(bluepad32_input_backend_snapshot(1, &state1) &&
                state1.imu_sample_count == 3,
            "slot 0 acknowledgement must not consume slot 1 IMU");

    const SwitchRumbleOutput rumble0{11, 22};
    const SwitchRumbleOutput rumble1{33, 44};
    bluepad32_input_backend_queue_rumble(0, rumble0);
    bluepad32_input_backend_queue_rumble(1, rumble1);
    process_rumble_timer(&g_rumble_timer);
    require(first.rumble_calls == 1 && first.last_low == 11 &&
                first.last_high == 22,
            "slot 0 rumble must reach only controller 0");
    require(survivor.rumble_calls == 1 && survivor.last_low == 33 &&
                survivor.last_high == 44,
            "slot 1 rumble must reach only controller 1");

    bluepad32_input_backend_queue_rumble(0, SwitchRumbleOutput{55, 66});
    const uint32_t disconnected_generation =
        g_slots[0].connection_generation;
    const int starts_before_disconnect = scan_starts;
    platform_on_device_disconnected(&first);
    require(scan_starts == starts_before_disconnect + 1 &&
                incoming_connections,
            "disconnect must resume scanning and incoming connections");
    require(!bluepad32_input_backend_snapshot(0, &state0) &&
                !state0.button_a && state0.lx == 32768,
            "disconnect must neutralize only its own slot");
    require(bluepad32_input_backend_snapshot(1, &state1) && state1.button_b,
            "disconnect must preserve survivor state and activity");

    uni_hid_device_t replacement = device(0);
    require(platform_on_device_ready(&replacement) == UNI_ERROR_SUCCESS,
            "replacement must bind to the freed indexed slot");
    process_rumble_timer(&g_rumble_timer);
    require(replacement.rumble_calls == 0,
            "replacement must not receive disconnected device rumble");

    g_slots[0].pending_rumble = {
        0, disconnected_generation, SwitchRumbleOutput{77, 88}};
    g_slots[0].rumble_pending = true;
    process_rumble_timer(&g_rumble_timer);
    require(replacement.rumble_calls == 0,
            "stale connection generation must be rejected at dispatch");

    uni_controller_t replacement_data{};
    replacement_data.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    replacement_data.gamepad.buttons = BUTTON_Y;
    platform_on_controller_data(&replacement, &replacement_data);
    require(bluepad32_input_backend_snapshot(0, &state0) && state0.button_x,
            "replacement input must populate only the freed slot");
    require(bluepad32_input_backend_snapshot(1, &state1) && state1.button_b,
            "replacement must not disturb survivor input");

    bluepad32_input_backend_queue_rumble(0, SwitchRumbleOutput{90, 91});
    bluepad32_input_backend_queue_rumble(1, SwitchRumbleOutput{92, 93});
    process_rumble_timer(&g_rumble_timer);
    require(replacement.rumble_calls == 1 && replacement.last_low == 90 &&
                replacement.last_high == 91,
            "replacement must receive only new-generation slot 0 rumble");
    require(survivor.rumble_calls == 2 && survivor.last_low == 92 &&
                survivor.last_high == 93,
            "survivor rumble must continue after peer replacement");
}

}  // namespace

int main(int argc, char** argv) {
    require(argc == 2, "scenario argument required");
    const std::string scenario = argv[1];
    if (scenario == "ready-0-1") {
        test_ready_order(0);
    } else if (scenario == "ready-1-0") {
        test_ready_order(1);
    } else if (scenario == "rejections") {
        test_rejections();
    } else if (scenario == "lifecycle") {
        test_independent_lifecycle();
    } else {
        require(false, "unknown scenario");
    }
    return 0;
}
