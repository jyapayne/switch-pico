// Reuse the backend's transport/storage fixture; these scenarios exercise only
// the native DualSense contract, not a second implementation of its scheduler.
#define main backend_fixture_main
#include "bluepad32_backend_lifecycle_test.cpp"
#undef main

namespace {
struct SensorFixture {
    uni_hid_device_t* device = nullptr;
    uni_ds5_bridge_snapshot_t metadata{};
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
    ++sensor.metadata.report_sequence;
    if (fresh_motion) ++sensor.metadata.motion_sequence;
    sensor.metadata.motion_valid = fresh_motion;
    pad.controller.klass = UNI_CONTROLLER_CLASS_GAMEPAD;
    pad.controller.gamepad.buttons = BUTTON_A | BUTTON_SHOULDER_L;
    pad.controller.gamepad.accel[1] = 8193;
    pad.controller.gamepad.gyro[2] = -123456;
    pad.controller.battery = 176;
    platform_on_controller_data(&pad, &pad.controller);
}

Bluepad32DualSenseBridgeSnapshot bridge_snapshot() {
    Bluepad32DualSenseBridgeSnapshot result{};
    bluepad32_input_backend_dualsense_snapshot(&result);
    return result;
}

void source_isolation() {
    start_pairing_backend();
    auto ordinary = device(0, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
    ordinary.vendor_id = 0x054c;
    ordinary.product_id = 0x0ce6;
    require(platform_on_device_ready(&ordinary) == UNI_ERROR_INVALID_CONTROLLER,
            "an unsupported parser must not enter the dedicated DualSense output slots");
    bluepad32_input_backend_select_dualsense_source(ordinary.conn.btaddr);
    require(!bridge_snapshot().controller.active, "VID/PID/address alone must never select a non-PS5 parser");
    platform_on_device_disconnected(&ordinary);
    bluepad32_input_backend_select_dualsense_source(nullptr);
    auto first = dualsense(0);
    auto second = dualsense(1);
    require(platform_on_device_ready(&first) == UNI_ERROR_SUCCESS, "first DS5 must connect");
    now_ms = 100;
    report_dualsense(first);
    const auto initial = bridge_snapshot();
    require(initial.slot == 0 && initial.controller.active && initial.controller.state.button_south &&
                initial.battery == 176 && initial.motion_valid && initial.accel_q13[1] == 8193 &&
                initial.gyro_q10[2] == -123456 && initial.motion_received_us == 100000,
            "native snapshot must preserve coherent physical controls and calibrated precision");
    now_ms = 120;
    platform_on_controller_data(&first, &first.controller);
    bluepad32_input_backend_report_sent(0);
    auto snapshot = bridge_snapshot();
    require(snapshot.state_generation == initial.state_generation && snapshot.received_us == 100000 &&
                snapshot.motion_sequence == initial.motion_sequence,
            "polling/cached callbacks and USB consumption must not freshen input or motion");
    report_dualsense(first, false);
    snapshot = bridge_snapshot();
    require(snapshot.received_us == 120000 && snapshot.motion_received_us == 100000 && !snapshot.motion_valid,
            "a controls-only admission must not refresh a duplicate sensor timestamp");
    sensors[0].valid = false;
    first.controller.gamepad.buttons = 0;
    platform_on_controller_data(&first, &first.controller);
    require(bridge_snapshot().controller.state.button_south,
            "malformed parser input must not publish an invented button release");
    uint64_t old_token;
    require(bluepad32_input_backend_dualsense_sample_request(0, 1, &old_token), "first source cue must queue");
    require(platform_on_device_ready(&second) == UNI_ERROR_SUCCESS, "Edge must connect");
    report_dualsense(second);
    require(!bridge_snapshot().controller.active &&
                bluepad32_input_backend_dualsense_sample_result(0, old_token) == -1,
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
    bluepad32_input_backend_select_dualsense_source(first.conn.btaddr);
    report_dualsense(first);
    report_dualsense(second);
    require(bridge_snapshot().slot == 0, "explicit source must ignore another live PS5");
    platform_on_device_disconnected(&first);
    require(!bridge_snapshot().controller.active, "disconnect must not migrate an explicit source");
}

void stable_logical_slot() {
    start_pairing_backend();
    auto unrelated = device(0, true, UNI_BT_CONN_PROTOCOL_BR_EDR);
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
    require(bluepad32_input_backend_dualsense_sample_request(0, 6, &right) &&
                bluepad32_input_backend_dualsense_sample_request(1, 1, &left) && right != left &&
                bluepad32_input_backend_dualsense_sample_result(0, right) == 0 && pad.rumble_calls == 0,
            "independent acceptance is not driver completion");
    dualsense_transport_available = false;
    process_rumble_timer(&g_rumble_timer);
    require(bluepad32_input_backend_dualsense_sample_result(0, right) == 0 && pad.rumble_calls == 0,
            "a busy source driver must not count as dispatch completion");
    dualsense_transport_available = true;
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_high == 96 && pad.last_low == 160 && pad.last_rumble_duration_ms == 60 &&
                bluepad32_input_backend_dualsense_sample_result(0, right) == 1 &&
                bluepad32_input_backend_dualsense_sample_result(1, right) == -1 &&
                bluepad32_input_backend_dualsense_sample_result(1, left) == 1,
            "combined output must route R weak/right and L strong/left with the shortest safe timer");
    now_ms = 60;
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_high == 0 && pad.last_low == 160 && pad.last_rumble_duration_ms == 940,
            "ending right must preserve only the left pulse's original remaining lifetime");
    require(bluepad32_input_backend_dualsense_sample_request(0, 3, &right), "right can restart independently");
    process_rumble_timer(&g_rumble_timer);
    now_ms = 85;
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_high == 0 && pad.last_low == 160, "right gap must not stop the left motor");
    now_ms = 175;
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_high == 96 && pad.last_low == 160, "later pulse must resume after its gap");
    require(bluepad32_input_backend_dualsense_sample_request(1, 0, &stop), "left stop must replace only left");
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_high == 96 && pad.last_low == 0 &&
                bluepad32_input_backend_dualsense_sample_result(1, stop) == 1 &&
                bluepad32_input_backend_dualsense_sample_result(1, left) == -1,
            "a side stop needs actual dispatch and cannot stop its sibling");
    now_ms = 200;
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_rumble_duration_ms == 0, "final pulse expiry must release both motors");
    const int stopped = pad.rumble_calls;
    now_ms = 5000;
    process_rumble_timer(&g_rumble_timer);
    require(pad.rumble_calls == stopped, "expired pulses must never replay after a stall");
    require(bluepad32_input_backend_dualsense_sample_request(0, 1, &right), "pending timeout cue must queue");
    now_ms += 2000;
    process_rumble_timer(&g_rumble_timer);
    require(bluepad32_input_backend_dualsense_sample_result(0, right) == -1 && pad.rumble_calls == stopped,
            "an undispatched expired cue must fail without producing a late pulse");
}

void cue_races() {
    start_pairing_backend();
    auto pad = dualsense(0);
    require(platform_on_device_ready(&pad) == UNI_ERROR_SUCCESS, "DS5 must connect");
    uint64_t token;
    require(bluepad32_input_backend_dualsense_sample_request(0, 1, &token), "race cue must queue");
    during_dualsense_dispatch = [] { bluepad32_input_backend_dualsense_sample_cancel(0); };
    process_rumble_timer(&g_rumble_timer);
    during_dualsense_dispatch = nullptr;
    require(bluepad32_input_backend_dualsense_sample_result(0, token) == -1,
            "cancellation during dispatch must defeat a late completion");
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_rumble_duration_ms == 0, "in-flight cancellation must retain a bounded stop obligation");
    require(bluepad32_input_backend_dualsense_sample_request(1, 1, &token), "reselection race must queue");
    during_dualsense_dispatch = [] { bluepad32_input_backend_select_dualsense_source(nullptr); };
    process_rumble_timer(&g_rumble_timer);
    during_dualsense_dispatch = nullptr;
    require(bluepad32_input_backend_dualsense_sample_result(1, token) == -1,
            "reselection must retire an in-flight token even for the same physical source");
    process_rumble_timer(&g_rumble_timer);
    require(pad.last_rumble_duration_ms == 0, "reselection cannot orphan the just-dispatched motor");
    require(bluepad32_input_backend_dualsense_sample_request(0, 1, &token), "disconnect race must queue");
    platform_on_device_disconnected(&pad);
    auto replacement = dualsense(0);
    require(platform_on_device_ready(&replacement) == UNI_ERROR_SUCCESS, "replacement must connect");
    process_rumble_timer(&g_rumble_timer);
    require(bluepad32_input_backend_dualsense_sample_result(0, token) == -1 && replacement.rumble_calls == 0,
            "old tokens and deferred stops must never enter a replacement connection");
}
}  // namespace

extern "C" void uni_hid_parser_ds5_parse_input_report(uni_hid_device_t*, const uint8_t*, uint16_t) {}
extern "C" bool uni_hid_parser_ds5_bridge_rumble(
    uni_hid_device_t* pad, uint16_t duration, uint8_t right, uint8_t left) {
    if (!dualsense_transport_available) return false;
    observe_dualsense_rumble(pad, 0, duration, right, left);
    return true;
}
extern "C" bool uni_hid_parser_ds5_bridge_snapshot(uni_hid_device_t* pad, uni_ds5_bridge_snapshot_t* out) {
    for (const auto& fixture : sensors) {
        if (fixture.device != pad || !fixture.valid) continue;
        *out = fixture.metadata;
        return true;
    }
    return false;
}

int main(int argc, char** argv) {
    require(argc == 2, "scenario required");
    const std::string scenario = argv[1];
    if (scenario == "source-isolation") source_isolation();
    else if (scenario == "cue-lifetime") cue_lifetime();
    else if (scenario == "cue-races") cue_races();
    else if (scenario == "stable-logical-slot") stable_logical_slot();
    else require(false, "unknown DualSense scenario");
    return 0;
}
