// Reuse the backend's transport/storage fixture; these scenarios exercise only
// the native gamepad contract, not a second implementation of its scheduler.
#define main backend_fixture_main
#include "bluepad32_backend_lifecycle_test.cpp"
#undef main

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

Bluepad32NativeGamepadSnapshot bridge_snapshot() {
    Bluepad32NativeGamepadSnapshot result{};
    bluepad32_input_backend_native_snapshot(&result);
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
    bluepad32_input_backend_select_native_source(ordinary.conn.btaddr);
    require(!bridge_snapshot().controller.active, "an ineligible device cannot become the native source");
    platform_on_device_disconnected(&ordinary);
    bluepad32_input_backend_select_native_source(nullptr);
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
    bluepad32_input_backend_select_native_source(first.conn.btaddr);
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
    during_dualsense_dispatch = [] { bluepad32_input_backend_select_native_source(nullptr); };
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
    bluepad32_input_backend_select_native_source(xbox.conn.btaddr);
    now_ms = 110;
    report_gamepad(xbox);
    require(bridge_snapshot().controller.active && bridge_snapshot().slot == 0 &&
                controller_identity_equal(bridge_snapshot().controller.identity, initial.controller.identity),
            "explicit sensorless selection retains its real identity and logical slot");
    platform_on_device_disconnected(&xbox);
    report_gamepad(generic);
    require(!bridge_snapshot().controller.active, "explicit selection cannot migrate on disconnect");
    bluepad32_input_backend_select_native_source(nullptr);
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
    bluepad32_input_backend_select_native_source(nullptr);
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
    bluepad32_input_backend_select_native_source(right.conn.btaddr);
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
    bluepad32_input_backend_select_native_source(nullptr);
    set_runtime_joycon_mode(JoyConMode::kIndividual);
    require(!bridge_snapshot().controller.active &&
                bluepad32_input_backend_native_sample_result(1, lc) == -1,
            "live split retires the pair immediately and fails auto selection closed");
    bluepad32_input_backend_select_native_source(right.conn.btaddr);
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
        bluepad32_input_backend_select_native_source(nullptr);
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

int main(int argc, char** argv) {
    require(argc == 2, "scenario required");
    const std::string scenario = argv[1];
    if (scenario == "source-isolation") source_isolation();
    else if (scenario == "cue-lifetime") cue_lifetime();
    else if (scenario == "cue-races") cue_races();
    else if (scenario == "stable-logical-slot") stable_logical_slot();
    else if (scenario == "sensorless-admission") sensorless_admission();
    else if (scenario == "independent-motion") independent_motion();
    else if (scenario == "paired-source") paired_source();
    else if (scenario == "pair-cue-races") pair_cue_races();
    else if (scenario == "mono-rumble") mono_rumble();
    else require(false, "unknown native gamepad scenario");
    return 0;
}
