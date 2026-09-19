// Reuse the backend's transport/storage fixture; these scenarios exercise only
// the native gamepad contract, not a second implementation of its scheduler.
#include <uni.h>
extern "C" bool uni_hid_parser_wii_rumble_ready(uni_hid_device_t*);
#define profile_service_active_profile_snapshot fixture_active_profile_snapshot
#define main backend_fixture_main
#include "bluepad32_backend_lifecycle_test.cpp"
#undef main
#undef profile_service_active_profile_snapshot

namespace {
void (*during_profile_resolution)() = nullptr;
bool native_wii_ready = true;
}

void profile_service_active_profile_snapshot(
    const ControllerIdentity& identity, ProfileServiceActiveProfileSnapshot* output) {
    require(state_lock_depth == 0, "profile service callbacks must not hold the backend lock");
    if (during_profile_resolution) during_profile_resolution();
    fixture_active_profile_snapshot(identity, output);
}

extern "C" bool uni_hid_parser_wii_rumble_ready(uni_hid_device_t*) {
    require(state_lock_depth == 0, "Wii readiness must not hold the backend lock");
    return native_wii_ready;
}

namespace {
struct SensorFixture {
    uni_hid_device_t* device = nullptr;
    uni_native_motion_snapshot_t metadata{};
    bool valid = false;
};
SensorFixture sensors[4];
void (*during_dualsense_dispatch)() = nullptr;
bool dualsense_transport_available = true;

void observe_dualsense_rumble(uni_hid_device_t* target, uint16_t delay,
                             uint16_t duration, uint8_t right, uint8_t left) {
    require(state_lock_depth == 0, "DS5 driver dispatch must not hold the shared state lock");
    play_rumble(target, delay, duration, right, left);
    if (during_dualsense_dispatch) during_dualsense_dispatch();
}

uni_hid_device_t dualsense(int index) {
    auto result = device(index, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    result.vendor_id = 0x054c;
    result.product_id = index == 0 ? 0x0ce6 : 0x0df2;
    result.controller_type = CONTROLLER_TYPE_PS5Controller;
    result.report_parser.parse_input_report = uni_hid_parser_ds5_parse_input_report;
    result.report_parser.play_dual_rumble = observe_dualsense_rumble;
    return result;
}

void report_dualsense(uni_hid_device_t& pad, bool fresh_motion = true) {
    SensorFixture& sensor = sensors[pad.idx];
    sensor.device = &pad;
    sensor.valid = true;
    sensor.metadata.report_tracked = true;
    sensor.metadata.report_valid = true;
    ++sensor.metadata.report_sequence;
    if (fresh_motion) {
        ++sensor.metadata.accel_sequence;
        ++sensor.metadata.gyro_sequence;
    }
    sensor.metadata.accel_valid = fresh_motion;
    sensor.metadata.gyro_valid = fresh_motion;
    sensor.metadata.accel_q13[1] = 8193;
    sensor.metadata.gyro_q10[2] = -123456;
    pad.controller.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    pad.controller.gamepad.buttons = BUTTON_A | BUTTON_SHOULDER_L;
    // Parser init_report may clear legacy fields; native precision comes from
    // the independent sensor provider rather than cached controller values.
    pad.controller.gamepad.accel[1] = 0;
    pad.controller.gamepad.gyro[2] = 0;
    pad.controller.battery = 176;
    platform_on_controller_data(&pad, &pad.controller);
}

Bluepad32NativeGamepadSnapshot bridge_snapshot(uint8_t pair = 0) {
    Bluepad32NativeGamepadSnapshot result{};
    bluepad32_input_backend_native_snapshot(pair, &result);
    return result;
}

void source_isolation() {
    start_pairing_backend();
    auto ordinary = device(0, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    ordinary.vendor_id = 0x054c;
    ordinary.product_id = 0x0ce6;
#if !SWITCH2_BRIDGE_DUALSENSE_INPUT
    ordinary.gamepad = false;
#endif
    require(platform_on_device_ready(&ordinary) == UNI_ERROR_INVALID_CONTROLLER,
            "an ineligible controller must not enter dedicated output slots");
    bluepad32_input_backend_select_native_source(0, ordinary.conn.btaddr);
    require(!bridge_snapshot().controller.active, "an ineligible device cannot become the native source");
    platform_on_device_disconnected(&ordinary);
    bluepad32_input_backend_select_native_source(0, nullptr);
    auto first = dualsense(0);
    auto second = dualsense(1);
    require(platform_on_device_ready(&first) == UNI_ERROR_SUCCESS, "first DS5 must connect");
    now_ms = 100;
    report_dualsense(first);
    const auto initial = bridge_snapshot();
    require(initial.slot == 0 && initial.controller.active && initial.controller.state.button_south &&
                initial.battery == 176 && initial.accel_valid && initial.gyro_valid &&
                !initial.track_stationary_bias && initial.accel_q13[1] == 8193 &&
                initial.gyro_q10[2] == -123456 && initial.gyro_received_us == 100000,
            "native snapshot must preserve coherent physical controls and calibrated precision");
    now_ms = 120;
    platform_on_controller_data(&first, &first.controller);
    bluepad32_input_backend_report_sent(0);
    auto snapshot = bridge_snapshot();
    require(snapshot.state_generation == initial.state_generation && snapshot.received_us == 100000 &&
                snapshot.accel_sequence == initial.accel_sequence &&
                snapshot.gyro_sequence == initial.gyro_sequence,
            "polling/cached callbacks and USB consumption must not freshen input or motion");
    report_dualsense(first, false);
    snapshot = bridge_snapshot();
    require(snapshot.received_us == 120000 && snapshot.gyro_received_us == 100000 &&
                !snapshot.accel_valid && !snapshot.gyro_valid,
            "a controls-only admission must not refresh a duplicate sensor timestamp");
    sensors[0].valid = false;
    first.controller.gamepad.buttons = 0;
    platform_on_controller_data(&first, &first.controller);
    require(bridge_snapshot().controller.state.button_south,
            "malformed parser input must not publish an invented button release");
    uint64_t old_token;
    require(bluepad32_input_backend_native_sample_request(0, 1, &old_token), "first source cue must queue");
    require(platform_on_device_ready(&second) == UNI_ERROR_SUCCESS, "Edge must connect");
    report_dualsense(second);
    require(!bridge_snapshot().controller.active &&
                bluepad32_input_backend_native_sample_result(0, old_token) == -1,
            "auto ambiguity must fail closed and retire source work immediately");
    platform_on_device_disconnected(&second);
    platform_on_controller_data(&first, &first.controller);
    require(!bridge_snapshot().controller.active, "returning to a source cannot resurrect cached state");
    report_dualsense(first);
    snapshot = bridge_snapshot();
    require(snapshot.controller.active && snapshot.controller.connection_generation != initial.controller.connection_generation,
            "a missed ambiguous interval still needs a new adapter epoch");
    platform_on_device_connected(&second);
    require(platform_on_device_ready(&second) == UNI_ERROR_SUCCESS, "Edge reconnect must succeed");
    bluepad32_input_backend_select_native_source(0, first.conn.btaddr);
    report_dualsense(first);
    report_dualsense(second);
    require(bridge_snapshot().slot == 0, "explicit source must ignore another live PS5");
    platform_on_device_disconnected(&first);
    require(!bridge_snapshot().controller.active, "disconnect must not migrate an explicit source");
}

void stable_logical_slot() {
    start_pairing_backend();
    auto unrelated = device(0, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
#if !SWITCH2_BRIDGE_DUALSENSE_INPUT
    unrelated.gamepad = false;
#endif
    auto pad = dualsense(1);
    pad.report_parser.set_lightbar_color = set_lightbar;
    // An earlier, still-unclassified transport connection must not reserve player 1.
    platform_on_device_connected(&unrelated);
    platform_on_device_connected(&pad);
    require(platform_on_device_ready(&pad) == UNI_ERROR_SUCCESS, "DS5 must complete setup");
    now_ms = 100;
    report_dualsense(pad);
    const auto first = bridge_snapshot();
    const auto color = switch_pro_get_slot_light_color(0);
    require(first.controller.active && first.slot == 0 &&
                pad.lightbar_red == color.red && pad.lightbar_green == color.green &&
                pad.lightbar_blue == color.blue,
            "a transport-index-1 DualSense must own logical slot 0 and its lightbar color");
    require(platform_on_device_ready(&unrelated) == UNI_ERROR_INVALID_CONTROLLER,
            "dedicated DualSense mode must reject an unrelated ready controller");
    platform_on_device_disconnected(&unrelated);
    platform_on_device_connected(&unrelated);
    require(platform_on_device_ready(&unrelated) == UNI_ERROR_INVALID_CONTROLLER,
            "remembered unrelated reconnects must remain outside logical slots");
    platform_on_device_disconnected(&unrelated);
    require(bridge_snapshot().slot == 0 &&
                bridge_snapshot().controller.connection_generation == first.controller.connection_generation &&
                pad.lightbar_calls == 1,
            "unrelated connection churn must not rebind or recolor the active DualSense");

    platform_on_device_disconnected(&pad);
    auto reconnected = dualsense(2);
    memcpy(reconnected.conn.btaddr, pad.conn.btaddr, sizeof(pad.conn.btaddr));
    reconnected.report_parser.set_lightbar_color = set_lightbar;
    platform_on_device_connected(&reconnected);
    require(platform_on_device_ready(&reconnected) == UNI_ERROR_SUCCESS, "DS5 reconnect must succeed");
    now_ms = 200;
    report_dualsense(reconnected);
    const auto next = bridge_snapshot();
    require(next.controller.active && next.slot == 0 &&
                controller_identity_equal(next.controller.identity, first.controller.identity) &&
                next.controller.connection_generation != first.controller.connection_generation &&
                reconnected.lightbar_red == color.red && reconnected.lightbar_green == color.green &&
                reconnected.lightbar_blue == color.blue,
            "reusing another Bluetooth index must preserve identity and the first logical slot");
    auto aborted = dualsense(3);
    platform_on_device_connected(&aborted);
    platform_on_device_disconnected(&aborted);
    require(platform_on_device_ready(&aborted) == UNI_ERROR_NO_SLOTS,
            "a late ready callback must not resurrect an unassigned disconnected transport");
    require(bridge_snapshot().controller.active && bridge_snapshot().slot == 0,
            "an aborted second setup must not disturb the active source");
    auto pending0 = device(0, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    auto pending1 = device(1, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    auto pending3 = device(3, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    platform_on_device_connected(&pending0);
    platform_on_device_connected(&pending1);
    platform_on_device_connected(&pending3);
    require(!incoming_connections && !scanning_enabled,
            "unclassified transports still consume physical connection capacity");
    platform_on_device_disconnected(&pending0);
    require(incoming_connections && bridge_snapshot().slot == 0 &&
                bridge_snapshot().controller.connection_generation == next.controller.connection_generation,
            "freeing pending transport capacity must not move the logical source");
    platform_on_device_disconnected(&pending1);
    platform_on_device_disconnected(&pending3);
}

void cue_lifetime() {
    start_pairing_backend();
    auto pad = dualsense(0);
    require(platform_on_device_ready(&pad) == UNI_ERROR_SUCCESS, "DS5 must connect");
    uint64_t right, left, stop;
    require(bluepad32_input_backend_native_sample_request(0, 6, &right) &&
                bluepad32_input_backend_native_sample_request(1, 1, &left) && right != left &&
                bluepad32_input_backend_native_sample_result(0, right) == 0 && pad.rumble_calls == 0,
            "independent acceptance is not driver completion");
    dualsense_transport_available = false;
    process_rumble_timer(&g_rumble_timer);
    require(bluepad32_input_backend_native_sample_result(0, right) == 0 && pad.rumble_calls == 0,
            "a busy source driver must not count as dispatch completion");
    dualsense_transport_available = true;
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_high == 96 && pad.last_low == 160 && pad.last_rumble_duration_ms == 60 &&
                bluepad32_input_backend_native_sample_result(0, right) == 1 &&
                bluepad32_input_backend_native_sample_result(1, right) == -1 &&
                bluepad32_input_backend_native_sample_result(1, left) == 1,
            "combined output must route R weak/right and L strong/left with the shortest safe timer");
    now_ms = 60;
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_high == 0 && pad.last_low == 160 && pad.last_rumble_duration_ms == 940,
            "ending right must preserve only the left pulse's original remaining lifetime");
    require(bluepad32_input_backend_native_sample_request(0, 3, &right), "right can restart independently");
    process_rumble_timer(&g_rumble_timer);
    now_ms = 85;
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_high == 0 && pad.last_low == 160, "right gap must not stop the left motor");
    now_ms = 175;
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_high == 96 && pad.last_low == 160, "later pulse must resume after its gap");
    require(bluepad32_input_backend_native_sample_request(1, 0, &stop), "left stop must replace only left");
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_high == 96 && pad.last_low == 0 &&
                bluepad32_input_backend_native_sample_result(1, stop) == 1 &&
                bluepad32_input_backend_native_sample_result(1, left) == -1,
            "a side stop needs actual dispatch and cannot stop its sibling");
    now_ms = 200;
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_rumble_duration_ms == 0, "final pulse expiry must release both motors");
    const int stopped = pad.rumble_calls;
    now_ms = 5000;
    process_rumble_timer(&g_rumble_timer);
    require(pad.rumble_calls == stopped, "expired pulses must never replay after a stall");
    require(bluepad32_input_backend_native_sample_request(0, 1, &right), "pending timeout cue must queue");
    now_ms += 2000;
    process_rumble_timer(&g_rumble_timer);
    require(bluepad32_input_backend_native_sample_result(0, right) == -1 && pad.rumble_calls == stopped,
            "an undispatched expired cue must fail without producing a late pulse");
}

void cue_races() {
    start_pairing_backend();
    auto pad = dualsense(0);
    require(platform_on_device_ready(&pad) == UNI_ERROR_SUCCESS, "DS5 must connect");
    uint64_t token;
    require(bluepad32_input_backend_native_sample_request(0, 1, &token), "race cue must queue");
    during_dualsense_dispatch = [] { bluepad32_input_backend_native_sample_cancel(0); };
    process_rumble_timer(&g_rumble_timer);
    during_dualsense_dispatch = nullptr;
    require(bluepad32_input_backend_native_sample_result(0, token) == -1,
            "cancellation during dispatch must defeat a late completion");
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_rumble_duration_ms == 0, "in-flight cancellation must retain a bounded stop obligation");
    require(bluepad32_input_backend_native_sample_request(1, 1, &token), "reselection race must queue");
    during_dualsense_dispatch = [] { bluepad32_input_backend_select_native_source(0, nullptr); };
    process_rumble_timer(&g_rumble_timer);
    during_dualsense_dispatch = nullptr;
    require(bluepad32_input_backend_native_sample_result(1, token) == -1,
            "reselection must retire an in-flight token even for the same physical source");
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_rumble_duration_ms == 0, "reselection cannot orphan the just-dispatched motor");
    require(bluepad32_input_backend_native_sample_request(0, 1, &token), "disconnect race must queue");
    platform_on_device_disconnected(&pad);
    auto replacement = dualsense(0);
    require(platform_on_device_ready(&replacement) == UNI_ERROR_SUCCESS, "replacement must connect");
    process_rumble_timer(&g_rumble_timer);
    require(bluepad32_input_backend_native_sample_result(0, token) == -1 && replacement.rumble_calls == 0,
            "old tokens and deferred stops must never enter a replacement connection");
}
void report_gamepad(uni_hid_device_t& pad) {
    pad.controller.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    platform_on_controller_data(&pad, &pad.controller);
}

SensorFixture& motion_fixture(uni_hid_device_t& pad, bool tracked = true) {
    SensorFixture& fixture = sensors[pad.idx];
    fixture = {};
    fixture.device = &pad;
    fixture.valid = true;
    fixture.metadata.report_tracked = tracked;
    fixture.metadata.report_valid = true;
    fixture.metadata.report_sequence = 1;
    fixture.metadata.accel_sequence = 1;
    fixture.metadata.gyro_sequence = 1;
    fixture.metadata.accel_valid = true;
    fixture.metadata.gyro_valid = true;
    fixture.metadata.accel_q13[1] = 8193;
    fixture.metadata.gyro_q10[2] = -123456;
    return fixture;
}

void sensorless_admission() {
    start_pairing_backend();
    auto xbox = device(2, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    xbox.controller_type = CONTROLLER_TYPE_XBoxOneController;
    xbox.report_parser.play_dual_rumble = nullptr;
    platform_on_device_connected(&xbox);
    require(platform_on_device_ready(&xbox) == UNI_ERROR_SUCCESS,
            "normal gamepad admission must not require a motion parser");
    xbox.controller.gamepad.buttons = BUTTON_A;
    xbox.controller.gamepad.accel[1] = 8192;
    xbox.controller.gamepad.gyro[0] = 65536;
    now_ms = 100;
    report_gamepad(xbox);
    const auto initial = bridge_snapshot();
    uint64_t cue = 99;
    require(initial.controller.active && initial.slot == 0 &&
                initial.controller.state.button_south && !initial.accel_valid &&
                !initial.gyro_valid && !initial.track_stationary_bias &&
                initial.accel_sequence == 0 && initial.gyro_sequence == 0 &&
                !bluepad32_input_backend_native_sample_request(0, 1, &cue) && cue == 0,
            "sensorless controls remain live without invented IMU or rumble capability");
    auto generic = device(0, true, UNI_BT_CONN_PROTOCOL_BLE);
    platform_on_device_connected(&generic);
    require(platform_on_device_ready(&generic) == UNI_ERROR_SUCCESS,
            "unknown-family normal AIO gamepads must not face a native brand whitelist");
    report_gamepad(generic);
    require(!bridge_snapshot().controller.active, "two logical gamepads are ambiguous");
    bluepad32_input_backend_select_native_source(0, xbox.conn.btaddr);
    now_ms = 110;
    report_gamepad(xbox);
    require(bridge_snapshot().controller.active && bridge_snapshot().slot == 0 &&
                controller_identity_equal(bridge_snapshot().controller.identity, initial.controller.identity),
            "explicit sensorless selection retains its real identity and logical slot");
    platform_on_device_disconnected(&xbox);
    report_gamepad(generic);
    require(!bridge_snapshot().controller.active, "explicit selection cannot migrate on disconnect");
    bluepad32_input_backend_select_native_source(0, nullptr);
    generic.controller.gamepad.buttons = BUTTON_B;
    report_gamepad(generic);
    require(bridge_snapshot().controller.active && bridge_snapshot().controller.state.button_east &&
                !bridge_snapshot().controller.state.button_south,
            "unique sensorless reselection must publish only the new source's controls");
}

void independent_motion() {
    start_pairing_backend();
    auto ds4 = device(0, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    ds4.controller_type = CONTROLLER_TYPE_PS4Controller;
    require(platform_on_device_ready(&ds4) == UNI_ERROR_SUCCESS, "DS4 controls must be admitted");
    auto& ds = motion_fixture(ds4).metadata;
    ds4.controller.gamepad.buttons = BUTTON_A;
    now_ms = 100;
    report_gamepad(ds4);
    const auto first = bridge_snapshot();
    require(first.accel_valid && first.gyro_valid && !first.track_stationary_bias &&
                first.accel_q13[1] == 8193 && first.gyro_q10[2] == -123456,
            "calibrated DS4 motion retains precision without Wii background correction");
    now_ms = 110;
    ++ds.report_sequence;
    ++ds.accel_sequence;
    report_gamepad(ds4);
    require(bridge_snapshot().accel_received_us == 110000 &&
                bridge_snapshot().gyro_received_us == 100000 && bridge_snapshot().gyro_valid,
            "acceleration ingress must not refresh an unchanged gyro");
    now_ms = 120;
    ds.report_valid = false;
    ds4.controller.gamepad.buttons = 0;
    report_gamepad(ds4);
    require(bridge_snapshot().controller.state.button_south &&
                bridge_snapshot().received_us == 110000,
            "a tracked partial/malformed report cannot publish phantom releases");
    ds.report_valid = true;
    ++ds.report_sequence;
    ds.gyro_valid = false;
    report_gamepad(ds4);
    require(!bridge_snapshot().gyro_valid && bridge_snapshot().accel_valid,
            "gyro capability loss must not suppress working acceleration or controls");
    bluepad32_input_backend_select_native_source(0, nullptr);
    ++ds.report_sequence;
    ds.gyro_valid = true;
    report_gamepad(ds4);
    require(bridge_snapshot().controller.active && !bridge_snapshot().accel_valid &&
                !bridge_snapshot().gyro_valid,
            "reselection cannot rehabilitate cached sensor sequences");
    ++ds.report_sequence;
    ++ds.gyro_sequence;
    report_gamepad(ds4);
    require(bridge_snapshot().gyro_valid && !bridge_snapshot().accel_valid,
            "each sensor must earn freshness separately in a new source epoch");
    platform_on_device_disconnected(&ds4);

    auto pro = device(1, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    pro.controller_type = CONTROLLER_TYPE_SwitchProController;
    require(platform_on_device_ready(&pro) == UNI_ERROR_SUCCESS, "Switch Pro controls must be admitted");
    auto& sw = motion_fixture(pro).metadata;
    sw.accel_valid = sw.gyro_valid = false;
    pro.controller.gamepad.buttons = BUTTON_B;
    report_gamepad(pro);
    require(bridge_snapshot().controller.state.button_east && !bridge_snapshot().gyro_valid,
            "Switch controls must not wait for motion calibration");
    ++sw.report_sequence;
    ++sw.accel_sequence;
    ++sw.gyro_sequence;
    sw.accel_valid = sw.gyro_valid = true;
    report_gamepad(pro);
    require(bridge_snapshot().accel_valid && bridge_snapshot().gyro_valid &&
                !bridge_snapshot().track_stationary_bias,
            "validated Switch motion does not enable Wii background correction");
    platform_on_device_disconnected(&pro);

    auto remote = wii_device(2);
    require(platform_on_device_ready(&remote) == UNI_ERROR_SUCCESS, "Wii controls must be admitted");
    auto& wii = motion_fixture(remote, false).metadata;
    wii.gyro_valid = false;
    remote.controller.gamepad.buttons = BUTTON_A;
    now_ms = 200;
    report_gamepad(remote);
    require(bridge_snapshot().controller.state.button_south && bridge_snapshot().accel_valid &&
                !bridge_snapshot().gyro_valid && bridge_snapshot().track_stationary_bias,
            "Wii without MotionPlus remains acceleration-capable, not a fabricated full IMU");
    now_ms = 210;
    ++wii.gyro_sequence;
    wii.gyro_valid = true;
    report_gamepad(remote);
    require(bridge_snapshot().gyro_valid && bridge_snapshot().gyro_received_us == 210000 &&
                bridge_snapshot().accel_received_us == 200000 &&
                bridge_snapshot().track_stationary_bias,
            "Wii MotionPlus ingress must retain independent acceleration age and Wii bias policy");
}

void paired_source() {
    start_pairing_backend();
    auto left = switch2_device(2, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    ready_switch2(left);
    ready_switch2(right);
    auto& r = motion_fixture(right).metadata;
    auto& l = motion_fixture(left).metadata;
    right.controller.gamepad.buttons = BUTTON_A;
    left.controller.gamepad.dpad = DPAD_LEFT;
    now_ms = 100;
    report_gamepad(right);
    const auto initial = bridge_snapshot();
    require_pair_owner(initial.controller.identity, left, right);
    require(initial.controller.active && initial.slot == 0 && initial.gyro_valid &&
                !slot_snapshot(1).active && !slot_snapshot(2).active,
            "an existing Joy-Con pair is one logical native source, not auto ambiguity");
    now_ms = 110;
    l.gyro_q10[2] = 654321;
    report_gamepad(left);
    require(bridge_snapshot().controller.state.button_south &&
                bridge_snapshot().controller.state.dpad_left &&
                bridge_snapshot().received_us == 110000 &&
                bridge_snapshot().gyro_received_us == 100000 &&
                bridge_snapshot().gyro_q10[2] == initial.gyro_q10[2],
            "left controls merge without refreshing or replacing the right motion owner");
    bluepad32_input_backend_select_native_source(0, right.conn.btaddr);
    ++l.report_sequence;
    report_gamepad(left);
    require(bridge_snapshot().controller.active && !bridge_snapshot().gyro_valid,
            "either member address selects the logical pair, not that member's cached IMU");
    ++r.report_sequence;
    report_gamepad(right);
    require(!bridge_snapshot().gyro_valid, "reselection cannot freshen the right parser cache");
    ++r.report_sequence;
    ++r.accel_sequence;
    ++r.gyro_sequence;
    report_gamepad(right);
    require(bridge_snapshot().gyro_valid, "new right samples restore paired motion");
    uint64_t rc, lc, stop;
    require(bluepad32_input_backend_native_sample_request(0, 6, &rc) &&
                bluepad32_input_backend_native_sample_request(1, 1, &lc),
            "both real halves expose their own driver capability");
    process_rumble_timer(&g_rumble_timer);
    require(right.last_high == 96 && right.last_low == 96 &&
                left.last_high == 160 && left.last_low == 160 &&
                bluepad32_input_backend_native_sample_result(0, rc) == 1 &&
                bluepad32_input_backend_native_sample_result(1, lc) == 1,
            "paired cues must dispatch only each half's contribution to its real driver");
    now_ms = 120;
    require(bluepad32_input_backend_native_sample_request(0, 0, &stop), "right stop must queue");
    process_rumble_timer(&g_rumble_timer);
    require(right.last_rumble_duration_ms == 0 && left.last_rumble_duration_ms == 990 &&
                bluepad32_input_backend_native_sample_result(0, stop) == 1,
            "stopping one paired side preserves the other side's original finite deadline");
    bluepad32_input_backend_select_native_source(0, nullptr);
    set_runtime_joycon_mode(JoyConMode::kIndividual);
    require(!bridge_snapshot().controller.active &&
                bluepad32_input_backend_native_sample_result(1, lc) == -1,
            "live split retires the pair immediately and fails auto selection closed");
    bluepad32_input_backend_select_native_source(0, right.conn.btaddr);
    ++r.report_sequence;
    ++r.accel_sequence;
    ++r.gyro_sequence;
    report_gamepad(right);
    require(bridge_snapshot().controller.active &&
                controller_identity_equal(bridge_snapshot().controller.identity, identity_for_device(&right)),
            "explicit selection after a split retains the real surviving member identity");
}

void pair_cue_races() {
    start_pairing_backend();
    auto left = switch2_device(0, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    ready_switch2(left);
    ready_switch2(right);
    left.report_parser.play_dual_rumble = nullptr;
    uint64_t rc, lc;
    require(!bluepad32_input_backend_native_sample_request(1, 1, &lc) && lc == 0,
            "a paired half without a rumble driver cannot borrow its sibling's capability");
    left.report_parser.play_dual_rumble = play_rumble;
    right.report_parser.play_dual_rumble = [](
        uni_hid_device_t* pad, uint16_t delay, uint16_t duration, uint8_t weak, uint8_t strong) {
        require(state_lock_depth == 0, "paired driver dispatch must not hold the shared lock");
        play_rumble(pad, delay, duration, weak, strong);
        bluepad32_input_backend_native_sample_cancel(1);
    };
    require(bluepad32_input_backend_native_sample_request(0, 1, &rc) &&
                bluepad32_input_backend_native_sample_request(1, 1, &lc), "paired race cues must queue");
    const int before = left.rumble_calls;
    process_rumble_timer(&g_rumble_timer);
    require(left.rumble_calls == before && right.last_rumble_duration_ms == 1000 &&
                bluepad32_input_backend_native_sample_result(0, rc) == 1 &&
                bluepad32_input_backend_native_sample_result(1, lc) == -1,
            "canceling L during R dispatch must not submit stale L or retire the accepted R cue");
    bluepad32_input_backend_native_sample_cancel(0);
    process_rumble_timer(&g_rumble_timer);
    const int before_stall = left.rumble_calls;
    require(right.last_rumble_duration_ms == 0, "partial pair submission retains a real stop obligation");
    right.report_parser.play_dual_rumble = [](
        uni_hid_device_t* pad, uint16_t delay, uint16_t duration, uint8_t weak, uint8_t strong) {
        play_rumble(pad, delay, duration, weak, strong);
        now_ms += 2000;
        bluepad32_input_backend_select_native_source(0, nullptr);
    };
    require(bluepad32_input_backend_native_sample_request(0, 1, &rc) &&
                bluepad32_input_backend_native_sample_request(1, 1, &lc), "reselection race cues must queue");
    process_rumble_timer(&g_rumble_timer);
    right.report_parser.play_dual_rumble = play_rumble;
    require(left.rumble_calls == before_stall &&
                bluepad32_input_backend_native_sample_result(0, rc) == -1 &&
                bluepad32_input_backend_native_sample_result(1, lc) == -1,
            "reselection and a dispatch stall cannot submit the old epoch's remaining half");
    process_rumble_timer(&g_rumble_timer);
    require(right.last_rumble_duration_ms == 0, "reselection cannot orphan a partial pair output");
}

uint8_t mono_magnitude = 0;
void observe_mono_rumble(uni_hid_device_t* pad, uint16_t delay, uint16_t duration,
                         uint8_t weak, uint8_t strong) {
    require(state_lock_depth == 0 && delay == 0, "mono driver dispatch is immediate and outside the lock");
    mono_magnitude = duration == 0 ? 0 : (weak > strong ? weak : strong);
    play_rumble(pad, delay, duration, mono_magnitude, 0);
}

void mono_rumble() {
    start_pairing_backend();
    auto move = device(0, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    move.controller_type = CONTROLLER_TYPE_PSMoveController;
    move.report_parser.play_dual_rumble = observe_mono_rumble;
    require(platform_on_device_ready(&move) == UNI_ERROR_SUCCESS, "mono-rumble gamepad must connect");
    uint64_t right, left;
    require(bluepad32_input_backend_native_sample_request(0, 1, &right) &&
                bluepad32_input_backend_native_sample_request(1, 7, &left),
            "mono devices accept independent logical contributions, not separate actuators");
    process_rumble_timer(&g_rumble_timer);
    require(mono_magnitude == 220 && move.last_rumble_duration_ms == 120,
            "one mono actuator combines both contributions with the shortest finite boundary");
    bluepad32_input_backend_native_sample_cancel(1);
    now_ms = 40;
    process_rumble_timer(&g_rumble_timer);
    require(mono_magnitude == 160 && move.last_rumble_duration_ms == 960 &&
                bluepad32_input_backend_native_sample_result(1, left) == -1,
            "canceling one mono contribution must preserve the other's remaining pulse");
    now_ms = 1000;
    process_rumble_timer(&g_rumble_timer);
    require(mono_magnitude == 0, "the last mono contribution must stop at its original deadline");
    require(bluepad32_input_backend_native_sample_request(0, 1, &right), "new mono cue must queue");
    const int calls = move.rumble_calls;
    now_ms += 2000;
    process_rumble_timer(&g_rumble_timer);
    require(move.rumble_calls == calls && bluepad32_input_backend_native_sample_result(0, right) == -1,
            "expired undispatched mono work cannot replay after a timer stall");
    require(bluepad32_input_backend_native_sample_request(1, 1, &left), "disconnect cue must queue");
    process_rumble_timer(&g_rumble_timer);
    require(mono_magnitude == 160, "disconnect regression must own a real active driver pulse");
    platform_on_device_disconnected(&move);
    require(mono_magnitude == 0 && move.last_rumble_duration_ms == 0,
            "disconnect must retire the source driver's timer before parser memory reuse");
    auto replacement = device(0, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    replacement.report_parser.play_dual_rumble = observe_mono_rumble;
    require(platform_on_device_ready(&replacement) == UNI_ERROR_SUCCESS, "new mono connection must succeed");
    process_rumble_timer(&g_rumble_timer);
    require(replacement.rumble_calls == 0 &&
                bluepad32_input_backend_native_sample_result(1, left) == -1,
            "retired mono work must never enter a replacement connection");
}

void gameplay_timeline() {
    start_pairing_backend();
    auto pad = dualsense(0);
    require(platform_on_device_ready(&pad) == UNI_ERROR_SUCCESS, "gameplay source must connect");
    uint8_t right[] = {40, 80, 120};
    const uint8_t left[] = {60, 180};
    const uint8_t stop = 0;
    require(bluepad32_input_backend_native_rumble_submit(0, right, 3) &&
                bluepad32_input_backend_native_rumble_submit(1, left, 2),
            "mixed sample counts must be accepted independently");
    right[0] = 255;
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_high == 40 && pad.last_low == 60 && pad.last_rumble_duration_ms == 4,
            "copied right/weak and left/strong samples share the earliest finite boundary");
    now_ms = 5;
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_high == 80 && pad.last_low == 60 && pad.last_rumble_duration_ms == 1,
            "5 ms polling skips elapsed time rather than replaying the first sample");
    now_ms = 10;
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_high == 120 && pad.last_low == 180 && pad.last_rumble_duration_ms == 40,
            "last samples hold only to their original receipt watchdog");
    require(bluepad32_input_backend_native_rumble_submit(0, &stop, 1),
            "explicit zero magnitude must be a valid side stop");
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_high == 0 && pad.last_low == 180 && pad.last_rumble_duration_ms == 40,
            "right stop must preserve the left contribution and deadline");
    now_ms = 49;
    const int calls = pad.rumble_calls;
    process_rumble_timer(&g_rumble_timer);
    require(pad.rumble_calls == calls, "unchanged finite holds must not produce duplicate writes");
    now_ms = 50;
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_rumble_duration_ms == 0, "watchdog expiry must stop without further host packets");
    now_ms = 500;
    process_rumble_timer(&g_rumble_timer);
    const int stopped = pad.rumble_calls;
    const uint8_t delayed[] = {21, 42, 84};
    require(bluepad32_input_backend_native_rumble_submit(0, delayed, 3), "delayed block must queue");
    now_ms = 520;
    process_rumble_timer(&g_rumble_timer);
    require(pad.rumble_calls == stopped + 1 && pad.last_high == 84 &&
                pad.last_rumble_duration_ms == 30,
            "a stalled timer dispatches only the current sample with its remaining lifetime");
    bluepad32_input_backend_native_rumble_cancel(0);
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_rumble_duration_ms == 0, "explicit cancellation must stop its live hold");
    now_ms = UINT32_MAX - 9u;
    const uint8_t pulse = 99;
    require(bluepad32_input_backend_native_rumble_submit(1, &pulse, 1), "pre-wrap request must queue");
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_low == 99 && pad.last_rumble_duration_ms == 50,
            "finite host lifetime must remain valid before clock wrap");
    now_ms = 39;
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_low == 99, "wrap must not cause early watchdog expiry");
    now_ms = 40;
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_rumble_duration_ms == 0, "watchdog must expire exactly across clock wrap");
}

void gameplay_availability() {
    start_pairing_backend();
    auto pad = dualsense(0);
    require(platform_on_device_ready(&pad) == UNI_ERROR_SUCCESS, "busy gameplay source must connect");
    const uint8_t old[] = {10, 20, 30};
    const uint8_t newest[] = {70, 140};
    require(bluepad32_input_backend_native_rumble_submit(0, old, 3), "old block must queue");
    dualsense_transport_available = false;
    process_rumble_timer(&g_rumble_timer);
    now_ms = 5;
    require(bluepad32_input_backend_native_rumble_submit(0, newest, 2), "busy source must retain a new block");
    process_rumble_timer(&g_rumble_timer);
    require(pad.rumble_calls == 0, "driver rejection cannot count as a successful output");
    now_ms = 15;
    dualsense_transport_available = true;
    process_rumble_timer(&g_rumble_timer);
    require(pad.rumble_calls == 1 && pad.last_high == 140 && pad.last_rumble_duration_ms == 40,
            "driver recovery must dispatch only the newest current phase, not old samples");
    const uint8_t next = 210;
    during_dualsense_dispatch = [] {
        const uint8_t replacement = 33;
        require(bluepad32_input_backend_native_rumble_submit(0, &replacement, 1),
                "gameplay must replace work while an earlier driver call is in flight");
    };
    require(bluepad32_input_backend_native_rumble_submit(0, &next, 1), "dispatch race must queue");
    process_rumble_timer(&g_rumble_timer);
    during_dualsense_dispatch = nullptr;
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_high == 33 && pad.last_rumble_duration_ms == 50,
            "older dispatch completion cannot consume a newly accepted revision");
    bluepad32_input_backend_native_rumble_cancel(0);
    process_rumble_timer(&g_rumble_timer);
    const int stopped = pad.rumble_calls;
    dualsense_transport_available = false;
    require(bluepad32_input_backend_native_rumble_submit(0, old, 3), "stale block must queue");
    now_ms += 50;
    process_rumble_timer(&g_rumble_timer);
    dualsense_transport_available = true;
    process_rumble_timer(&g_rumble_timer);
    require(pad.rumble_calls == stopped, "expired rejected work must never replay after driver recovery");
}

void gameplay_priority() {
    start_pairing_backend();
    auto pad = dualsense(0);
    require(platform_on_device_ready(&pad) == UNI_ERROR_SUCCESS, "priority source must connect");
    report_dualsense(pad);
    const uint8_t game = 45;
    uint64_t old_cue, new_cue;
    require(bluepad32_input_backend_native_sample_request(0, 1, &old_cue) &&
                bluepad32_input_backend_native_rumble_submit(0, &game, 1),
            "gameplay must replace a pending cue on the same side");
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_high == 45 &&
                bluepad32_input_backend_native_sample_result(0, old_cue) == -1,
            "replaced cue cannot dispatch or complete after gameplay");
    require(bluepad32_input_backend_native_sample_request(0, 6, &new_cue),
            "a new cue must replace live gameplay");
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_high == 96 && pad.last_rumble_duration_ms == 60 &&
                bluepad32_input_backend_native_sample_result(0, new_cue) == 1,
            "built-in cue completion must retain its driver-dispatch semantics");
    require(bluepad32_input_backend_native_rumble_submit(0, &game, 1) &&
                bluepad32_input_backend_native_rumble_submit(1, &game, 1),
            "fresh gameplay must replace the playing cue");
    process_rumble_timer(&g_rumble_timer);
    const auto source = bridge_snapshot();
    bluepad32_input_backend_queue_profile_feedback(source.slot,
        source.controller.connection_generation, 1, ControllerProfileConfirmationPolicy::kRumble);
    require(!bluepad32_input_backend_native_rumble_submit(0, &game, 1),
            "queued higher-priority feedback must not admit gameplay for later replay");
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_high == UINT8_MAX && pad.last_low == UINT8_MAX &&
                pad.last_rumble_duration_ms == 75, "profile feedback must own the only motor writer");
    now_ms = 150;
    process_rumble_timer(&g_rumble_timer);
    const int after_feedback = pad.rumble_calls;
    now_ms = 155;
    process_rumble_timer(&g_rumble_timer);
    require(pad.rumble_calls == after_feedback, "profile completion must never resurrect interrupted gameplay");
    require(bluepad32_input_backend_native_rumble_submit(0, &game, 1), "fresh post-feedback gameplay must resume");
    process_rumble_timer(&g_rumble_timer);
    require(bluepad32_input_backend_toggle_motion(source.slot, source.controller.connection_generation),
            "motion feedback must queue through the existing local-feedback path");
    process_rumble_timer(&g_rumble_timer);
    const int local_calls = pad.rumble_calls;
    now_ms += 3000;
    process_rumble_timer(&g_rumble_timer);
    require(pad.rumble_calls == local_calls, "local feedback must cancel rather than retain interrupted gameplay");
}

void gameplay_source_epochs() {
    start_pairing_backend();
    auto pad = dualsense(0);
    require(platform_on_device_ready(&pad) == UNI_ERROR_SUCCESS, "epoch source must connect");
    const uint8_t game = 170;
    require(!bluepad32_input_backend_native_rumble_submit(0, nullptr, 1) &&
                !bluepad32_input_backend_native_rumble_submit(0, &game, 0) &&
                !bluepad32_input_backend_native_rumble_submit(0, &game, 4) &&
                !bluepad32_input_backend_native_rumble_submit(PROBE_CONTROLLER_COUNT, &game, 1),
            "invalid gameplay frames must fail before dispatch");
    pad.report_parser.play_dual_rumble = nullptr;
    require(!bluepad32_input_backend_native_rumble_submit(0, &game, 1),
            "source without a rumble driver must reject gameplay");
    pad.report_parser.play_dual_rumble = observe_dualsense_rumble;
    require(bluepad32_input_backend_native_rumble_submit(0, &game, 1), "epoch block must queue");
    during_dualsense_dispatch = [] { bluepad32_input_backend_select_native_source(0, nullptr); };
    process_rumble_timer(&g_rumble_timer);
    during_dualsense_dispatch = nullptr;
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_rumble_duration_ms == 0, "same-source reselection must stop an in-flight retired epoch");
    require(bluepad32_input_backend_native_rumble_submit(1, &game, 1), "disconnect block must queue");
    process_rumble_timer(&g_rumble_timer);
    platform_on_device_disconnected(&pad);
    require(pad.last_rumble_duration_ms == 0 &&
                !bluepad32_input_backend_native_rumble_submit(0, &game, 1),
            "disconnect must retire the driver's finite timer and refuse new work");
    pad = dualsense(0);
    platform_on_device_connected(&pad);
    require(platform_on_device_ready(&pad) == UNI_ERROR_SUCCESS, "same-address slot replacement must connect");
    process_rumble_timer(&g_rumble_timer);
    require(pad.rumble_calls == 0, "slot memory reuse cannot inherit old samples or stop obligations");
    controller_profile_runtime_reset();
    during_profile_resolution = [] { bluepad32_input_backend_select_native_source(0, nullptr); };
    require(!bluepad32_input_backend_native_rumble_submit(0, &game, 1),
            "source generation must be rechecked after unlocked profile callbacks");
    during_profile_resolution = nullptr;
    process_rumble_timer(&g_rumble_timer);
    require(pad.rumble_calls == 0, "a profile-resolution race cannot reach the replacement epoch");
}

void gameplay_profile_gain() {
    start_pairing_backend();
    initialize_runtime_profile_storage();
    auto pad = dualsense(0);
    require(platform_on_device_ready(&pad) == UNI_ERROR_SUCCESS, "profile source must connect");
    const auto identity = identity_for_device(&pad);
    auto profile = controller_profile_default(identity, 0);
    profile.weak_rumble_scale = 128;
    profile.strong_rumble_scale = 64;
    require(runtime_profile_storage.set(identity, 0, profile) == ProfileStorageResult::kOk,
            "profile must configure independent host motor gains");
    const uint8_t maximum = 255;
    require(bluepad32_input_backend_native_rumble_submit(0, &maximum, 1) &&
                bluepad32_input_backend_native_rumble_submit(1, &maximum, 1), "scaled gameplay must queue");
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_high == 128 && pad.last_low == 64,
            "right/weak and left/strong magnitudes must use their matching persisted profile gains");
    profile.weak_rumble_scale = 0;
    profile.strong_rumble_scale = 0;
    require(runtime_profile_storage.set(identity, 0, profile) == ProfileStorageResult::kOk,
            "profile mute must update its generation");
    require(bluepad32_input_backend_native_rumble_submit(0, &maximum, 1) &&
                bluepad32_input_backend_native_rumble_submit(1, &maximum, 1), "muted gameplay must queue");
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_high == 0 && pad.last_low == 0 && pad.last_rumble_duration_ms == 0,
            "muting a profile must stop live host output rather than retaining unscaled samples");
    uint64_t cue;
    require(bluepad32_input_backend_native_sample_request(0, 7, &cue), "muted profile still permits local cues");
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_high == 220 && bluepad32_input_backend_native_sample_result(0, cue) == 1,
            "host gain must not scale built-in local confirmation cues");
}

void gameplay_two_pairs() {
    start_pairing_backend();
    auto first = dualsense(0);
    auto second = dualsense(1);
    require(platform_on_device_ready(&first) == UNI_ERROR_SUCCESS &&
                platform_on_device_ready(&second) == UNI_ERROR_SUCCESS, "both gameplay pairs must connect");
    const uint8_t values[] = {31, 62, 93, 124};
    for (uint8_t instance = 0; instance < 4; ++instance)
        require(bluepad32_input_backend_native_rumble_submit(instance, values + instance, 1),
                "each gameplay child must bind its own pair");
    process_rumble_timer(&g_rumble_timer);
    require(first.last_high == 31 && first.last_low == 62 &&
                second.last_high == 93 && second.last_low == 124,
            "four logical contributions must route to two independent physical sources");
    bluepad32_input_backend_native_rumble_cancel(0);
    now_ms = 10;
    process_rumble_timer(&g_rumble_timer);
    require(first.last_high == 0 && first.last_low == 62 && first.last_rumble_duration_ms == 40 &&
                second.rumble_calls == 1, "side cancellation cannot dispatch into another pair");
    bluepad32_input_backend_select_native_source(0, nullptr);
    process_rumble_timer(&g_rumble_timer);
    require(first.last_rumble_duration_ms == 0 && second.rumble_calls == 1,
            "source reselection must retire only its own pair's output");
    platform_on_device_disconnected(&first);
    now_ms = 50;
    process_rumble_timer(&g_rumble_timer);
    require(second.last_rumble_duration_ms == 0 && second.rumble_calls == 2,
            "the surviving pair must expire on its original independent watchdog");
}

void gameplay_paired_revision() {
    start_pairing_backend();
    auto left = switch2_device(0, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    ready_switch2(left);
    ready_switch2(right);
    right.report_parser.play_dual_rumble = [](
        uni_hid_device_t* pad, uint16_t delay, uint16_t duration, uint8_t weak, uint8_t strong) {
        play_rumble(pad, delay, duration, weak, strong);
        const uint8_t fresh = 150;
        require(bluepad32_input_backend_native_rumble_submit(1, &fresh, 1),
                "right dispatch may accept a newer left revision");
    };
    const uint8_t game = 75;
    require(bluepad32_input_backend_native_rumble_submit(0, &game, 1), "paired gameplay must queue");
    const int previous_left = left.rumble_calls;
    process_rumble_timer(&g_rumble_timer);
    require(right.last_high == 75 && right.last_low == 75 && left.rumble_calls == previous_left,
            "new left work must invalidate even a prepared zero-output command before left dispatch");
    right.report_parser.play_dual_rumble = play_rumble;
    process_rumble_timer(&g_rumble_timer);
    require(left.last_high == 150 && left.last_low == 150,
            "the newer paired-side revision must remain pending until its real driver dispatch");
    const uint8_t stop = 0;
    require(bluepad32_input_backend_native_rumble_submit(0, &stop, 1), "paired right stop must queue");
    process_rumble_timer(&g_rumble_timer);
    require(right.last_rumble_duration_ms == 0 && left.last_high == 150 &&
                left.last_rumble_duration_ms == 50, "paired stop must preserve only the live sibling contribution");
}

void gameplay_wii() {
    start_pairing_backend();
    auto remote = wii_device(0);
    remote.report_parser.play_dual_rumble = observe_mono_rumble;
    require(platform_on_device_ready(&remote) == UNI_ERROR_SUCCESS, "Wii GAMEPAD source must connect");
    const uint8_t right = 70, left = 140;
    require(bluepad32_input_backend_native_rumble_submit(0, &right, 1) &&
                bluepad32_input_backend_native_rumble_submit(1, &left, 1), "Wii contributions must queue");
    native_wii_ready = false;
    process_rumble_timer(&g_rumble_timer);
    require(remote.rumble_calls == 0, "Wii topology setup must not consume pending output");
    native_wii_ready = true;
    now_ms = 10;
    process_rumble_timer(&g_rumble_timer);
    require(mono_magnitude == 140 && remote.last_rumble_duration_ms == 40,
            "Wii compatibility combines both contributions on its finite mono motor");
    bluepad32_input_backend_native_rumble_cancel(1);
    process_rumble_timer(&g_rumble_timer);
    require(mono_magnitude == 70 && remote.last_rumble_duration_ms == 40,
            "Wii side cancellation must not stop its sibling's mono contribution");
    now_ms = 50;
    process_rumble_timer(&g_rumble_timer);
    require(mono_magnitude == 0, "Wii gameplay must stop at its original watchdog");
}

}  // namespace

extern "C" void uni_hid_parser_ds5_parse_input_report(uni_hid_device_t*, const uint8_t*, uint16_t) {}
extern "C" bool uni_hid_parser_ds5_bridge_rumble(
    uni_hid_device_t* pad, uint16_t duration, uint8_t right, uint8_t left) {
    if (!dualsense_transport_available) return false;
    observe_dualsense_rumble(pad, 0, duration, right, left);
    return true;
}
extern "C" bool uni_hid_parser_native_motion_snapshot(
    uni_hid_device_t* pad, uni_native_motion_snapshot_t* out) {
    *out = {};
    for (const auto& fixture : sensors) {
        if (fixture.device != pad) continue;
        *out = fixture.metadata;
        if (!fixture.valid) out->report_valid = false;
        return fixture.valid;
    }
    // DS5 must be guarded even before its first good full report.
    out->report_tracked =
        pad->report_parser.parse_input_report == uni_hid_parser_ds5_parse_input_report;
    return false;
}

namespace {

void two_pair_sources() {
    start_pairing_backend();
    auto first = dualsense(2);
    auto second = dualsense(0);
    require(platform_on_device_ready(&first) == UNI_ERROR_SUCCESS &&
                platform_on_device_ready(&second) == UNI_ERROR_SUCCESS,
            "two independent physical pads must be admitted");
    auto& a = motion_fixture(first).metadata;
    auto& b = motion_fixture(second).metadata;
    first.controller.gamepad.buttons = BUTTON_A | BUTTON_SHOULDER_L;
    second.controller.gamepad.buttons = BUTTON_B | BUTTON_SHOULDER_R;
    a.gyro_q10[2] = 10000;
    b.gyro_q10[2] = -20000;
    now_ms = 100;
    report_gamepad(first);
    now_ms = 110;
    report_gamepad(second);
    const auto initial_a = bridge_snapshot(0);
    const auto initial_b = bridge_snapshot(1);
    require(initial_a.controller.active && initial_b.controller.active &&
                initial_a.slot != initial_b.slot &&
                initial_a.controller.state.button_south && !initial_a.controller.state.button_east &&
                initial_b.controller.state.button_east && !initial_b.controller.state.button_south &&
                initial_a.gyro_q10[2] == 10000 && initial_b.gyro_q10[2] == -20000,
            "each pair must publish only its own controls and calibrated motion");
    initialize_runtime_profile_storage();
    auto profile_a = controller_profile_default(initial_a.controller.identity, 2);
    auto profile_b = controller_profile_default(initial_b.controller.identity, 5);
    profile_a.confirmation_policy = profile_b.confirmation_policy = ControllerProfileConfirmationPolicy::kNone;
    profile_a.button_map[static_cast<uint8_t>(ControllerProfileLogicalButton::kSouth)] =
        static_cast<uint8_t>(ControllerProfileLogicalButton::kNorth);
    profile_b.button_map[static_cast<uint8_t>(ControllerProfileLogicalButton::kEast)] =
        static_cast<uint8_t>(ControllerProfileLogicalButton::kWest);
    require(runtime_profile_storage.set(initial_a.controller.identity, 2, profile_a) == ProfileStorageResult::kOk &&
                runtime_profile_storage.activate(initial_a.controller.identity, 2) == ProfileStorageResult::kOk &&
                runtime_profile_storage.set(initial_b.controller.identity, 5, profile_b) == ProfileStorageResult::kOk &&
                runtime_profile_storage.activate(initial_b.controller.identity, 5) == ProfileStorageResult::kOk,
            "independent identities must retain distinct active mapping banks");
    controller_profile_runtime_reset();
    const auto mapped_a = controller_profile_runtime_transform(
        initial_a.slot, initial_a.controller, now_ms, AdapterUsbMode::kXInput);
    const auto mapped_b = controller_profile_runtime_transform(
        initial_b.slot, initial_b.controller, now_ms, AdapterUsbMode::kXInput);
    require(mapped_a.state.button_north && !mapped_a.state.button_south &&
                mapped_b.state.button_west && !mapped_b.state.button_east,
            "each published source must use its own saved profile mapping");
    now_ms = 120;
    ++a.report_sequence;
    ++a.accel_sequence;
    first.controller.gamepad.buttons = BUTTON_X;
    report_gamepad(first);
    require(bridge_snapshot(0).controller.state.button_west &&
                bridge_snapshot(0).accel_received_us == 120000 &&
                bridge_snapshot(0).gyro_received_us == 100000 &&
                bridge_snapshot(1).controller.state.button_east &&
                bridge_snapshot(1).received_us == 110000 &&
                bridge_snapshot(1).gyro_received_us == 110000,
            "one source's input and independent sensor clocks must not freshen the other source");
    require(bluepad32_input_backend_capture_start(
                initial_b.slot, initial_b.controller.connection_generation, CaptureOptions{}),
            "Pair B must be recordable while Pair A changes connections");
    uint64_t old_a, live_b;
    require(bluepad32_input_backend_native_sample_request(0, 1, &old_a) &&
                bluepad32_input_backend_native_sample_request(3, 1, &live_b),
            "both sources must accept independent pending feedback");
    platform_on_device_disconnected(&first);
    auto extra = dualsense(1);
    require(platform_on_device_ready(&extra) == UNI_ERROR_SUCCESS, "third source may connect without assignment");
    report_dualsense(extra);
    require(!bridge_snapshot(0).controller.active &&
                bridge_snapshot(1).controller.connection_generation == initial_b.controller.connection_generation &&
                bluepad32_input_backend_native_sample_result(0, old_a) == -1 &&
                bluepad32_input_backend_native_sample_result(3, live_b) == 0,
            "a new third pad cannot steal a disconnected reservation or retire the independent pair");
    auto reconnected = dualsense(3);
    memcpy(reconnected.conn.btaddr, first.conn.btaddr, sizeof(first.conn.btaddr));
    reconnected.product_id = first.product_id;
    platform_on_device_connected(&reconnected);
    require(platform_on_device_ready(&reconnected) == UNI_ERROR_SUCCESS, "reserved source must reconnect");
    now_ms = 130;
    report_dualsense(reconnected);
    require(bridge_snapshot(0).controller.active &&
                controller_identity_equal(bridge_snapshot(0).controller.identity, initial_a.controller.identity) &&
                bridge_snapshot(0).slot != initial_a.slot &&
                bridge_snapshot(1).slot == initial_b.slot &&
                bridge_snapshot(1).controller.connection_generation == initial_b.controller.connection_generation,
            "stable reservations must restore Pair A across physical and logical slot changes without moving Pair B");
    ++b.report_sequence;
    second.controller.gamepad.buttons = BUTTON_Y;
    report_gamepad(second);
    Bluepad32CaptureSnapshot capture{};
    require(bluepad32_input_backend_capture_page(0, 0, &capture) &&
                capture.state == CaptureState::kRecording && capture.total_events == 2,
            "Pair B capture must keep recording real changes across Pair A's disconnect and rebind");
    process_rumble_timer(&g_rumble_timer);
    require(extra.rumble_calls == 0 && reconnected.rumble_calls == 0 &&
                second.last_high == 0 && second.last_low == 160 &&
                bluepad32_input_backend_native_sample_result(3, live_b) == 1,
            "pending Pair B work must reach only its original physical source after Pair A reconnects");
    const auto restored = bridge_snapshot(0);
    const auto remapped = controller_profile_runtime_transform(
        restored.slot, restored.controller, now_ms, AdapterUsbMode::kXInput);
    require(remapped.state.button_north && !remapped.state.button_south &&
                runtime_profile_storage.find(initial_b.controller.identity)->active_profile == 5,
            "reconnecting at another logical slot must preserve A's saved mapping and B's active profile");
}

void two_pair_cues() {
    start_pairing_backend();
    auto first = dualsense(0);
    auto second = dualsense(1);
    require(platform_on_device_ready(&first) == UNI_ERROR_SUCCESS &&
                platform_on_device_ready(&second) == UNI_ERROR_SUCCESS, "both cue sources must connect");
    report_dualsense(first);
    report_dualsense(second);
    const auto before_b = bridge_snapshot(1);
    uint64_t ar, al, br, bl;
    require(bluepad32_input_backend_native_sample_request(0, 6, &ar) &&
                bluepad32_input_backend_native_sample_request(1, 7, &al) &&
                bluepad32_input_backend_native_sample_request(2, 3, &br) &&
                bluepad32_input_backend_native_sample_request(3, 1, &bl),
            "all four virtual sides must accept independent cues");
    process_rumble_timer(&g_rumble_timer);
    require(first.last_high == 96 && first.last_low == 220 && first.last_rumble_duration_ms == 60 &&
                second.last_high == 96 && second.last_low == 160 && second.last_rumble_duration_ms == 25 &&
                bluepad32_input_backend_native_sample_result(2, ar) == -1 &&
                bluepad32_input_backend_native_sample_result(0, br) == -1,
            "R/L contributions and completion tokens must be scoped to their physical pair");
    now_ms = 25;
    process_rumble_timer(&g_rumble_timer);
    require(second.last_high == 0 && second.last_low == 160 && second.last_rumble_duration_ms == 975 &&
                first.last_high == 96 && first.last_low == 220,
            "Pair B's pulse boundary must not replace Pair A's independently timed motors");
    const uint8_t absent[6] = {0xee, 0, 0, 0, 0, 1};
    bluepad32_input_backend_select_native_source(0, absent);
    process_rumble_timer(&g_rumble_timer);
    require(first.last_rumble_duration_ms == 0 && second.last_low == 160 &&
                bluepad32_input_backend_native_sample_result(0, ar) == -1 &&
                bluepad32_input_backend_native_sample_result(1, al) == -1 &&
                bluepad32_input_backend_native_sample_result(2, br) == 1 &&
                bluepad32_input_backend_native_sample_result(3, bl) == 1 &&
                bridge_snapshot(1).controller.connection_generation == before_b.controller.connection_generation,
            "disabling Pair A must stop only A and preserve B's accepted cues and input epoch");
    bluepad32_input_backend_native_sample_cancel(2);
    now_ms = 40;
    process_rumble_timer(&g_rumble_timer);
    bluepad32_input_backend_select_native_source(0, nullptr);
    require(bluepad32_input_backend_native_sample_request(0, 1, &ar) &&
                bluepad32_input_backend_native_sample_request(2, 6, &br),
            "retired sides can accept fresh boot-unique work");
    during_dualsense_dispatch = [] {
        during_dualsense_dispatch = nullptr;
        bluepad32_input_backend_select_native_source(0, nullptr);
    };
    process_rumble_timer(&g_rumble_timer);
    require(bluepad32_input_backend_native_sample_result(0, ar) == -1 &&
                bluepad32_input_backend_native_sample_result(2, br) == 1 &&
                second.last_high == 96 && second.last_low == 160 && second.last_rumble_duration_ms == 60,
            "reselection during A's driver call must reject stale A completion without retiring B's next dispatch");
    process_rumble_timer(&g_rumble_timer);
    require(first.last_rumble_duration_ms == 0 && second.last_high == 96,
            "a raced A submission must be stopped without canceling B's physical timer");
    platform_on_device_disconnected(&first);
    now_ms = 100;
    process_rumble_timer(&g_rumble_timer);
    require(second.last_high == 0 && second.last_low == 160 && second.last_rumble_duration_ms == 900,
            "B's remaining left pulse must retain its original deadline after A disconnects");
}

void explicit_precedence() {
    start_pairing_backend();
    auto first = dualsense(0);
    auto second = dualsense(1);
    bluepad32_input_backend_select_native_source(1, second.conn.btaddr);
    require(platform_on_device_ready(&second) == UNI_ERROR_SUCCESS, "explicit Pair B may arrive first");
    report_dualsense(second);
    require(!bridge_snapshot(0).controller.active && bridge_snapshot(1).controller.active,
            "automatic Pair A cannot borrow an explicitly reserved source");
    require(platform_on_device_ready(&first) == UNI_ERROR_SUCCESS, "independent automatic source must connect");
    report_dualsense(first);
    const auto initial_a = bridge_snapshot(0);
    auto duplicate = dualsense(2);
    memcpy(duplicate.conn.btaddr, second.conn.btaddr, sizeof(second.conn.btaddr));
    duplicate.product_id = second.product_id;
    require(platform_on_device_ready(&duplicate) == UNI_ERROR_SUCCESS, "ambiguous-address fixture must connect");
    report_dualsense(duplicate);
    require(!bridge_snapshot(1).controller.active &&
                bridge_snapshot(0).controller.connection_generation == initial_a.controller.connection_generation,
            "an ambiguous explicit address must fail only its affected pair closed");
    platform_on_device_disconnected(&duplicate);
    report_dualsense(second);
    require(bridge_snapshot(1).controller.active, "the unique explicit match must resume after ambiguity clears");
    bluepad32_input_backend_select_native_source(0, second.conn.btaddr);
    report_dualsense(second);
    require(!bridge_snapshot(0).controller.active && !bridge_snapshot(1).controller.active,
            "two explicit selectors matching one logical pad must never broadcast it");
    bluepad32_input_backend_select_native_source(0, nullptr);
    report_dualsense(first);
    report_dualsense(second);
    require(controller_identity_equal(bridge_snapshot(0).controller.identity, identity_for_device(&first)) &&
                controller_identity_equal(bridge_snapshot(1).controller.identity, identity_for_device(&second)),
            "releasing an explicit conflict restores separate automatic and explicit sources");
}

void paired_explicit_conflict() {
    start_pairing_backend();
    auto left = switch2_device(0, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    bluepad32_input_backend_select_native_source(0, left.conn.btaddr);
    bluepad32_input_backend_select_native_source(1, right.conn.btaddr);
    ready_switch2(left);
    uint64_t old;
    require(bluepad32_input_backend_native_sample_request(0, 1, &old), "solo explicit source cue must queue");
    ready_switch2(right);
    motion_fixture(left);
    motion_fixture(right);
    report_gamepad(right);
    uint64_t rejected;
    require(!bridge_snapshot(0).controller.active && !bridge_snapshot(1).controller.active &&
                bluepad32_input_backend_native_sample_result(0, old) == -1 &&
                !bluepad32_input_backend_native_sample_request(2, 1, &rejected),
            "paired physical halves matched by different explicit selectors must retire old work and fail both closed");
    bluepad32_input_backend_select_native_source(1, nullptr);
    ++sensors[right.idx].metadata.report_sequence;
    report_gamepad(right);
    require(bridge_snapshot(0).controller.active && !bridge_snapshot(1).controller.active,
            "an explicit logical pair reserves both halves against automatic assignment");
    auto independent = dualsense(2);
    require(platform_on_device_ready(&independent) == UNI_ERROR_SUCCESS, "independent second source must connect");
    report_dualsense(independent);
    const auto before_b = bridge_snapshot(1);
    uint64_t rc, lc, bc;
    require(bluepad32_input_backend_native_sample_request(0, 6, &rc) &&
                bluepad32_input_backend_native_sample_request(1, 1, &lc) &&
                bluepad32_input_backend_native_sample_request(3, 7, &bc),
            "paired real halves and independent pad must accept separate feedback");
    process_rumble_timer(&g_rumble_timer);
    require(right.last_high == 96 && left.last_low == 160 && independent.last_low == 220,
            "feedback must respect both logical pair and paired physical side");
    set_runtime_joycon_mode(JoyConMode::kIndividual);
    ++sensors[left.idx].metadata.report_sequence;
    ++sensors[right.idx].metadata.report_sequence;
    report_gamepad(left);
    report_gamepad(right);
    require(bridge_snapshot(0).controller.active &&
                controller_identity_equal(bridge_snapshot(0).controller.identity, identity_for_device(&left)) &&
                bridge_snapshot(1).controller.connection_generation == before_b.controller.connection_generation &&
                bluepad32_input_backend_native_sample_result(0, rc) == -1 &&
                bluepad32_input_backend_native_sample_result(1, lc) == -1 &&
                bluepad32_input_backend_native_sample_result(3, bc) == 1,
            "splitting a physical pair retires only its old cues and cannot duplicate its unselected member into Pair B");
}

void topology_reservations() {
    start_pairing_backend();
    auto left = switch2_device(0, UNI_SW2_JOYCON_L_PID);
    auto right = switch2_device(1, UNI_SW2_JOYCON_R_PID);
    ready_switch2(left);
    ready_switch2(right);
    motion_fixture(left);
    motion_fixture(right);
    report_gamepad(right);
    auto independent = dualsense(2);
    require(platform_on_device_ready(&independent) == UNI_ERROR_SUCCESS, "independent automatic source must connect");
    report_dualsense(independent);
    const auto before_b = bridge_snapshot(1);
    const auto pair_identity = bridge_snapshot(0).controller.identity;
    set_runtime_joycon_mode(JoyConMode::kIndividual);
    ++sensors[left.idx].metadata.report_sequence;
    ++sensors[right.idx].metadata.report_sequence;
    report_gamepad(left);
    report_gamepad(right);
    require(!bridge_snapshot(0).controller.active &&
                bridge_snapshot(1).controller.connection_generation == before_b.controller.connection_generation,
            "a split remembered pair is ambiguous without moving the independent pair");
    set_runtime_joycon_mode(JoyConMode::kPaired);
    ++sensors[right.idx].metadata.report_sequence;
    report_gamepad(right);
    require(bridge_snapshot(0).controller.active &&
                controller_identity_equal(bridge_snapshot(0).controller.identity, pair_identity) &&
                bridge_snapshot(1).controller.connection_generation == before_b.controller.connection_generation,
            "remerging the same remembered members must restore only their reserved pair");
    platform_on_device_disconnected(&independent);
    set_runtime_joycon_mode(JoyConMode::kIndividual);
    bluepad32_input_backend_select_native_source(0, left.conn.btaddr);
    bluepad32_input_backend_select_native_source(1, right.conn.btaddr);
    bluepad32_input_backend_select_native_source(0, nullptr);
    bluepad32_input_backend_select_native_source(1, nullptr);
    ++sensors[left.idx].metadata.report_sequence;
    ++sensors[right.idx].metadata.report_sequence;
    report_gamepad(left);
    report_gamepad(right);
    require(bridge_snapshot(0).controller.active && bridge_snapshot(1).controller.active,
            "individually reserved physical halves must first own separate logical streams");
    set_runtime_joycon_mode(JoyConMode::kPaired);
    ++sensors[right.idx].metadata.report_sequence;
    report_gamepad(right);
    require(!bridge_snapshot(0).controller.active && !bridge_snapshot(1).controller.active,
            "merging two independently reserved sources must fail both closed rather than duplicate the merged pair");
    set_runtime_joycon_mode(JoyConMode::kIndividual);
    ++sensors[left.idx].metadata.report_sequence;
    ++sensors[right.idx].metadata.report_sequence;
    report_gamepad(left);
    report_gamepad(right);
    require(bridge_snapshot(0).controller.active && bridge_snapshot(1).controller.active &&
                bridge_snapshot(0).slot != bridge_snapshot(1).slot,
            "splitting conflicting members restores their previous independent reservations");
}

void stable_ble_reservation() {
    start_pairing_backend();
    auto first = device(0, true, UNI_BT_CONN_PROTOCOL_BLE);
    auto independent = dualsense(1);
    bluepad32_input_backend_select_native_source(1, independent.conn.btaddr);
    require(platform_on_device_ready(&first) == UNI_ERROR_SUCCESS &&
                platform_on_device_ready(&independent) == UNI_ERROR_SUCCESS,
            "an unresolved BLE gamepad may connect beside an explicit stable source");
    report_gamepad(first);
    report_dualsense(independent);
    const auto initial_b = bridge_snapshot(1);
    require(!bridge_snapshot(0).controller.active && initial_b.controller.active,
            "automatic reservations must not promote an unresolved BLE connection address to a stable identity");
    const bd_addr_t identity = {0xc2, 0x10, 0x20, 0x30, 0x40, 0x50};
    dispatch_identity_event(SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED, first,
                            BD_ADDR_TYPE_LE_RANDOM, identity);
    first.controller.gamepad.buttons = BUTTON_A;
    report_gamepad(first);
    const auto initial_a = bridge_snapshot(0);
    uint64_t old_a, live_b;
    require(initial_a.controller.active && initial_a.controller.state.button_south &&
                bluepad32_input_backend_native_sample_request(0, 1, &old_a) &&
                bluepad32_input_backend_native_sample_request(3, 1, &live_b),
            "resolved identity publication must activate its own stream and feedback without waiting for another connection");
    dispatch_identity_event(SM_EVENT_IDENTITY_RESOLVING_STARTED, first,
                            BD_ADDR_TYPE_LE_RANDOM, identity);
    require(!bridge_snapshot(0).controller.active &&
                bridge_snapshot(1).controller.connection_generation == initial_b.controller.connection_generation &&
                bluepad32_input_backend_native_sample_result(0, old_a) == -1 &&
                bluepad32_input_backend_native_sample_result(3, live_b) == 0,
            "identity loss must retire only the uncertain source's input and pending work");
    platform_on_device_disconnected(&first);
    auto reconnect = device(2, true, UNI_BT_CONN_PROTOCOL_BLE);
    reconnect.vendor_id = first.vendor_id;
    reconnect.product_id = first.product_id;
    platform_on_device_connected(&reconnect);
    require(platform_on_device_ready(&reconnect) == UNI_ERROR_SUCCESS, "BLE controller must reconnect at a different transport index");
    report_gamepad(reconnect);
    require(!bridge_snapshot(0).controller.active, "a fresh unresolved BLE address must not steal the remembered stable source");
    dispatch_identity_event(SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED, reconnect,
                            BD_ADDR_TYPE_LE_RANDOM, identity);
    reconnect.controller.gamepad.buttons = BUTTON_B;
    report_gamepad(reconnect);
    require(bridge_snapshot(0).controller.active && bridge_snapshot(0).controller.state.button_east &&
                !bridge_snapshot(0).controller.state.button_south &&
                controller_identity_equal(bridge_snapshot(0).controller.identity, initial_a.controller.identity) &&
                bridge_snapshot(1).controller.connection_generation == initial_b.controller.connection_generation,
            "resolving a new BLE connection address must recover the original pair reservation without reviving cached controls");
}

}  // namespace

int main(int argc, char** argv) {
    require(argc == 2, "scenario required");
    const std::string scenario = argv[1];
    if (scenario == "source-isolation") source_isolation();
    else if (scenario == "cue-lifetime") cue_lifetime();
    else if (scenario == "cue-races") cue_races();
    else if (scenario == "gameplay-timeline") gameplay_timeline();
    else if (scenario == "gameplay-availability") gameplay_availability();
    else if (scenario == "gameplay-priority") gameplay_priority();
    else if (scenario == "gameplay-source-epochs") gameplay_source_epochs();
    else if (scenario == "gameplay-profile-gain") gameplay_profile_gain();
    else if (scenario == "gameplay-two-pairs") gameplay_two_pairs();
    else if (scenario == "gameplay-paired-revision") gameplay_paired_revision();
    else if (scenario == "gameplay-wii") gameplay_wii();
    else if (scenario == "stable-logical-slot") stable_logical_slot();
    else if (scenario == "sensorless-admission") sensorless_admission();
    else if (scenario == "independent-motion") independent_motion();
    else if (scenario == "paired-source") paired_source();
    else if (scenario == "pair-cue-races") pair_cue_races();
    else if (scenario == "mono-rumble") mono_rumble();
    else if (scenario == "two-pair-sources") two_pair_sources();
    else if (scenario == "two-pair-cues") two_pair_cues();
    else if (scenario == "explicit-precedence") explicit_precedence();
    else if (scenario == "paired-explicit-conflict") paired_explicit_conflict();
    else if (scenario == "topology-reservations") topology_reservations();
    else if (scenario == "stable-ble-reservation") stable_ble_reservation();
    else require(false, "unknown native gamepad scenario");
    return 0;
}
