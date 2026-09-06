#include "input/switch_native_output.h"

#include <btstack.h>
#include <parser/uni_hid_parser_switch.h>
#include <uni.h>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <initializer_list>
#include <iostream>
#include <map>
#include <vector>

namespace {
// These are rumble payloads at the parser's native-send boundary, not simulated
// HCI packets or evidence of physical playback. Only accepted sends advance the
// independent decoder representing the controller's received command history.
constexpr uint8_t kNeutral[8] = {0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40};
constexpr uint8_t kSeed[8] = {0x00, 0x21, 0x40, 0x48, 0x00, 0x01, 0x40, 0x40};
constexpr uint8_t kThree[8] = {0x78, 0x77, 0x1c, 0xe9, 0x00, 0x01, 0x40, 0x40};
constexpr uint8_t kDifferent[8] = {0xa8, 0x89, 0xe3, 0x4d, 0x80, 0x00, 0x40, 0x52};
constexpr SwitchHapticsSample kSteps[3] = {
    {65, 65, 2139, 2282}, {65, 66, 2139, 2093}, {65, 66, 2093, 2093},
};
constexpr uint32_t kGeneration = 7;
const char* scenario = "startup";
uint64_t now_us = 1000000;
bool poll_requested = false;
bool writable = true;
std::deque<bool> send_results;
std::vector<btstack_data_source_t*> sources;
std::vector<btstack_timer_source_t*> timers;
std::map<uint16_t, uni_hid_device_t*> radio_devices;
std::deque<uint16_t> permission_requests;
uint16_t next_cid = 0x40;
bool credit_event_only = false, in_credit_event = false;

struct WireFrame {
    uni_hid_device_t* device;
    uint64_t submitted_us;
    std::array<uint8_t, 8> bytes;
    ControllerRumbleOutput decoded;
};
struct CompatibilityCall {
    uni_hid_device_t* device;
    uint16_t delay_ms;
    uint16_t duration_ms;
    uint8_t weak;
    uint8_t strong;
    size_t wire_position;
    uint64_t submitted_us;
};
std::vector<WireFrame> wire;
std::vector<CompatibilityCall> compatibility;
std::map<uni_hid_device_t*, SwitchHapticsDecoder> physical;

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << scenario << ": " << message << '\n';
        std::exit(1);
    }
}

// Run the actual registered data-source and timer callbacks. The IRQ wake is
// deliberately deferred: submitting on the producer never calls the owner.
void run_until(uint64_t target_us) {
    require(target_us >= now_us, "fake clock moved backwards");
    for (unsigned dispatches = 0; dispatches < 10000; ++dispatches) {
        if (writable && !credit_event_only && !permission_requests.empty()) {
            const auto cid = permission_requests.front();
            permission_requests.pop_front();
            switch_native_output_on_can_send_now(radio_devices[cid], cid);
            continue;
        }
        if (poll_requested) {
            poll_requested = false;
            const auto ready = sources;
            for (auto* source : ready) {
                if (source->callbacks & DATA_SOURCE_CALLBACK_POLL) {
                    require(source->handler != nullptr, "data source lacks handler");
                    source->handler(source, DATA_SOURCE_CALLBACK_POLL);
                }
            }
            continue;
        }
        const auto next = std::min_element(timers.begin(), timers.end(),
            [](const auto* a, const auto* b) { return a->due_us < b->due_us; });
        if (next == timers.end() || (*next)->due_us > target_us) {
            now_us = target_us;
            return;
        }
        auto* timer = *next;
        timers.erase(next);
        require(timer->due_us >= now_us && timer->handler != nullptr,
                "invalid timer deadline or callback");
        now_us = timer->due_us;
        timer->handler(timer);
    }
    require(false, "runloop did not quiesce within bounded dispatches");
}
void flush() { run_until(now_us); }
void advance_ms(uint32_t milliseconds) { run_until(now_us + uint64_t{milliseconds} * 1000); }

void conventional(uni_hid_device_t* device, uint16_t delay_ms,
                  uint16_t duration_ms, uint8_t weak, uint8_t strong) {
    require(device->connected && !device->native_owned,
            "compatibility output ran before native ownership was released");
    compatibility.push_back({device, delay_ms, duration_ms, weak, strong, wire.size(), now_us});
}
uni_hid_device_t device() {
    uni_hid_device_t result{};
    result.report_parser.play_dual_rumble = conventional;
    return result;
}
ControllerIdentity identity(uint8_t address = 1, uint16_t product = 0x2009) {
    ControllerIdentity result{};
    result.stable = true;
    result.transport = ControllerTransport::kClassic;
    result.address[0] = 0x24;
    result.address[1] = 0x68;
    result.address[5] = address;
    result.vendor_id = 0x057e;
    result.product_id = product;
    return result;
}
AdapterConfiguration persisted(std::initializer_list<ControllerIdentity> identities) {
    auto configuration = adapter_configuration_default();
    for (const auto& approved : identities) {
        configuration.native_switch_controllers[configuration.native_switch_controller_count++] = approved;
    }
    std::array<uint8_t, ADAPTER_CONFIGURATION_ENCODED_SIZE> bytes{};
    require(adapter_configuration_encode(configuration, bytes.data(), bytes.size()),
            "could not persist approval fixture through real configuration codec");
    AdapterConfiguration restored{};
    require(adapter_configuration_decode(ADAPTER_CONFIGURATION_SCHEMA_VERSION,
                                          bytes.data(), bytes.size(), &restored),
            "could not restore persisted approval fixture");
    return restored;
}
void attach_approved(uni_hid_device_t& target) {
    switch_native_output_configure(persisted({identity()}), 1);
    switch_native_output_attach(0, kGeneration, &target, identity());
    require(switch_native_output_owns(&target), "persisted physical approval did not acquire owner");
}
SwitchNativeOutputDiagnostics diagnostics(uint8_t slot = 0) {
    SwitchNativeOutputDiagnostics result{};
    switch_native_output_snapshot(slot, &result);
    return result;
}
void submit(const ControllerRumbleOutput& rumble, bool stateful = false,
            uint8_t slot = 0, uint32_t generation = kGeneration) {
    require(switch_native_output_submit(slot, generation, now_us, rumble, stateful),
            "current approved host command was rejected");
}
ControllerRumbleOutput three_steps() {
    SwitchHapticsDecoder host;
    host.decode(kSeed);
    return host.decode(kThree);
}
const WireFrame& last_frame(uni_hid_device_t& target) {
    const auto found = std::find_if(wire.rbegin(), wire.rend(),
        [&](const auto& frame) { return frame.device == &target; });
    require(found != wire.rend(), "controller has no accepted output");
    return *found;
}
size_t frame_count(uni_hid_device_t& target) {
    return std::count_if(wire.begin(), wire.end(),
        [&](const auto& frame) { return frame.device == &target; });
}
bool is_neutral(const WireFrame& frame) {
    return std::memcmp(frame.bytes.data(), kNeutral, sizeof(kNeutral)) == 0;
}
void expect_bytes(const WireFrame& frame, const uint8_t expected[8], const char* message) {
    require(std::memcmp(frame.bytes.data(), expected, 8) == 0, message);
}
void expect_sample(const SwitchHapticsSample& actual, const SwitchHapticsSample& expected) {
    require(actual.low_frequency_index == expected.low_frequency_index &&
                actual.high_frequency_index == expected.high_frequency_index &&
                actual.low_amplitude_q15 == expected.low_amplitude_q15 &&
                actual.high_amplitude_q15 == expected.high_amplitude_q15,
            "accepted output has wrong actuator band amplitude or frequency");
}
void expect_state(uni_hid_device_t& target, SwitchHapticsSample left,
                  SwitchHapticsSample right = {}) {
    const auto& frame = last_frame(target).decoded.hd;
    require(frame.actuators[0].sample_count > 0 && frame.actuators[1].sample_count > 0,
            "accepted payload has no decoded endpoint");
    expect_sample(frame.actuators[0].samples[frame.actuators[0].sample_count - 1], left);
    expect_sample(frame.actuators[1].samples[frame.actuators[1].sample_count - 1], right);
}
void expect_three_steps(uni_hid_device_t& target) {
    const auto& left = last_frame(target).decoded.hd.actuators[0];
    require(left.sample_count == 3, "bounded schedule collapsed ordered host substeps");
    for (uint8_t i = 0; i < 3; ++i) expect_sample(left.samples[i], kSteps[i]);
    expect_state(target, kSteps[2]);
}
void expect_silence_since(size_t first) {
    for (size_t i = first; i < wire.size(); ++i) {
        require(is_neutral(wire[i]), "stopped or disconnected host vibration reappeared");
    }
}

void test_approval() {
    auto target = device();
    switch_native_output_configure(persisted({}), 1);
    switch_native_output_attach(0, kGeneration, &target, identity());
    require(!switch_native_output_owns(&target) && target.acquisitions == 0,
            "Nintendo VID/PID automatically enabled native output");
    require(!switch_native_output_submit(0, kGeneration, now_us, {255, 255}, true) &&
                !switch_native_output_feedback(&target, 255, 255, 10),
            "unapproved native host or feedback output was accepted");
    flush();
    require(wire.empty() && compatibility.empty(), "unapproved attach disturbed conventional output");

    switch_native_output_configure(persisted({identity(2)}), 2);
    require(!switch_native_output_owns(&target) && wire.empty(),
            "approval leaked to another physical address of the same model");
    switch_native_output_configure(persisted({identity()}), 3);
    require(switch_native_output_owns(&target) && target.acquisitions == 1,
            "persisted matching identity did not acquire native output");
    require(is_neutral(last_frame(target)), "approval handoff did not establish physical neutral");
    SwitchHapticsDecoder host;
    const size_t before = wire.size();
    submit(host.decode(kSeed));
    require(wire.size() == before, "producer submission bypassed deferred runloop wake");
    flush();
    expect_bytes(last_frame(target), kSeed, "approved unity output changed a safe native payload");
}

void test_model_gate() {
    auto mismatch = device();
    mismatch.controller_type = 1;
    switch_native_output_configure(persisted({identity()}), 1);
    switch_native_output_attach(0, kGeneration, &mismatch, identity());
    require(!switch_native_output_owns(&mismatch) && mismatch.acquisitions == 0 && wire.empty(),
            "approved identity overrode mismatched parser controller type");
    auto unavailable = device();
    unavailable.info_ready = false;
    switch_native_output_attach(0, kGeneration, &unavailable, identity());
    require(!switch_native_output_owns(&unavailable) && wire.empty(),
            "missing parser device-info enabled native output");
    auto unstable = device();
    auto transient = identity();
    transient.stable = false;
    switch_native_output_attach(0, kGeneration, &unstable, transient);
    require(!switch_native_output_owns(&unstable) && wire.empty(),
            "unstable identity inherited a persisted approval");
    switch_native_output_detach(&unstable);
    auto refused = device();
    refused.acquire_allowed = false;
    switch_native_output_attach(0, kGeneration, &refused, identity());
    require(!switch_native_output_owns(&refused) &&
                !switch_native_output_submit(0, kGeneration, now_us, {255, 0}, true) && wire.empty(),
            "failed parser acquisition swallowed compatibility host commands");
}

void test_revocation(bool expired) {
    auto target = device();
    attach_approved(target);
    SwitchHapticsDecoder host;
    submit(host.decode(kSeed));
    flush();
    advance_ms(5);
    const auto latest = host.decode(kDifferent);
    const uint64_t receipt = now_us;
    submit(latest);  // Revoke before the queued replacement reaches the owner.
    const size_t before = wire.size();
    writable = false;
    switch_native_output_configure(persisted({}), 2);
    require(switch_native_output_owns(&target) && target.releases == 0 && compatibility.empty(),
            "revocation released owner before congestion allowed neutral");
    require(!switch_native_output_submit(0, kGeneration, now_us, {255, 255}, true),
            "revoked identity continued accepting native host updates");
    advance_ms(expired ? 55 : 10);
    require(wire.size() == before && compatibility.empty(),
            "blocked neutral leaked output or resumed compatibility early");
    writable = true;
    advance_ms(1);
    require(!switch_native_output_owns(&target) && target.releases == 1 && compatibility.size() == 1,
            "neutral completion did not release and resume conventional output");
    const auto& resumed = compatibility.back();
    require(resumed.device == &target && resumed.delay_ms == 0 &&
                resumed.wire_position > before && is_neutral(wire[resumed.wire_position - 1]),
            "compatibility did not follow the accepted neutral barrier");
    if (expired) {
        require(resumed.duration_ms == 0 && resumed.weak == 0 && resumed.strong == 0,
                "revocation resurrected an expired host effect");
    } else {
        const uint16_t remaining = static_cast<uint16_t>((receipt + 50000 - resumed.submitted_us + 999) / 1000);
        require(resumed.duration_ms == remaining &&
                    resumed.weak == latest.high_frequency_magnitude &&
                    resumed.strong == latest.low_frequency_magnitude,
                "compatibility resumed stale magnitudes or restarted the 50ms host lifetime");
    }
    const size_t released = wire.size();
    advance_ms(100);
    require(wire.size() == released && compatibility.size() == 1,
            "retired native timer wrote after compatibility resumed");
}

void test_generation() {
    auto old = device();
    auto other = device();
    auto replacement = device();
    switch_native_output_configure(persisted({identity(), identity(2)}), 1);
    switch_native_output_attach(0, kGeneration, &old, identity());
    switch_native_output_attach(1, 21, &other, identity(2));
    submit({255, 0}, true);
    flush();
    submit(three_steps());
    old.connected = false;
    const size_t old_count = frame_count(old);
    switch_native_output_detach(&old);
    switch_native_output_attach(0, kGeneration + 1, &replacement, identity());
    const size_t replacement_count = frame_count(replacement);
    require(is_neutral(last_frame(replacement)), "reconnect inherited a previous physical baseline");
    require(!switch_native_output_submit(0, kGeneration, now_us, {255, 255}, true),
            "old generation was accepted after same-address reconnect");
    submit({0, 255}, true, 1, 21);
    advance_ms(60);  // Includes the outstanding producer wake and old refresh/expiry deadlines.
    require(frame_count(old) == old_count && frame_count(replacement) == replacement_count,
            "old queued command or timer touched disconnected/replacement device");
    expect_state(other, {}, {64, 64, 0, 17867});
    submit({255, 0}, true, 0, kGeneration + 1);
    flush();
    expect_state(replacement, {64, 64, 17867, 0});
    expect_state(other, {}, {64, 64, 0, 17867});
    require(diagnostics(0).completed_commands == 1 && diagnostics(1).completed_commands == 1,
            "slot or generation completion accounting crossed controllers");
}

void test_overflow() {
    auto target = device();
    attach_approved(target);
    const auto before_diagnostics = diagnostics();
    const size_t before = wire.size();
    SwitchHapticsDecoder host;
    for (unsigned i = 0; i < 39; ++i)
        submit(host.decode((i & 1u) ? kDifferent : kSeed));
    submit(host.decode(kThree));
    require(wire.size() == before && diagnostics().queue_depth <= 16,
            "producer bypassed bounded deferred command queue");
    flush();
    require(wire.size() > before && wire.size() <= before + 3 && is_neutral(wire[before]),
            "queue loss replayed a backlog instead of neutral plus bounded newest schedule");
    expect_three_steps(target);
    const auto after = diagnostics();
    require(after.received_commands == 40 && after.dropped_commands == 39 &&
                after.completed_commands == 1 && after.queue_depth == 0 &&
                after.resynchronizations > before_diagnostics.resynchronizations,
            "queue pressure did not distinguish discarded commands from the completed newest state");
    const size_t drained = wire.size();
    advance_ms(12);
    require(wire.size() == drained, "owner caught up obsolete queued vibrations after draining");
}

void test_retry() {
    auto target = device();
    attach_approved(target);
    const size_t before = wire.size();
    const auto before_diagnostics = diagnostics();
    send_results = {false};
    submit(three_steps());
    flush();
    require(wire.size() == before && diagnostics().completed_commands == 0,
            "failed first schedule packet counted as physical output or completion");
    send_results = {true, false};
    advance_ms(1);
    require(wire.size() == before + 1 && diagnostics().completed_commands == 0,
            "baseline-only partial submission counted as complete host command");
    advance_ms(1);
    expect_three_steps(target);
    const auto after = diagnostics();
    require(wire.size() == before + 2 && after.completed_commands == 1 &&
                after.dropped_commands == 0 && after.congested_attempts == 2 &&
                after.resynchronizations == before_diagnostics.resynchronizations,
            "short congestion retry duplicated baseline, dropped history, or miscounted completion");
}

void test_partial_replacement() {
    auto target = device();
    attach_approved(target);
    send_results = {true, false};
    submit(three_steps());
    flush();
    require(diagnostics().completed_commands == 0, "partial schedule was already complete");
    const size_t before = wire.size();
    SwitchHapticsDecoder latest_host;
    submit(latest_host.decode(kDifferent));
    flush();
    require(wire.size() == before + 2 && is_neutral(wire[before]),
            "replacing prepared/partially sent schedule omitted the physical reset barrier");
    expect_bytes(last_frame(target), kDifferent, "old prepared tail replaced newest host output");
    const auto after = diagnostics();
    require(after.completed_commands == 1 && after.dropped_commands == 1,
            "discarded partial command was reported as completed or vanished from loss accounting");
    const size_t replaced = wire.size();
    advance_ms(12);
    require(wire.size() == replaced, "discarded prepared tail was retried after replacement");
}

void test_stalled_schedule(bool partial) {
    auto target = device();
    attach_approved(target);
    writable = partial;
    if (partial) send_results = {true, false};
    const size_t before = wire.size();
    submit(three_steps());
    flush();
    require(wire.size() == before + (partial ? 1 : 0), "incorrect initial blocked schedule setup");
    writable = false;
    advance_ms(20);
    require(diagnostics().completed_commands == 0, "unsubmitted stalled schedule was counted complete");
    const size_t stalled = wire.size();
    writable = true;
    advance_ms(1);
    require(wire.size() > stalled, "current endpoint was not submitted after congestion");
    if (partial) require(is_neutral(wire[stalled]),
        "partially submitted expired timeline lacked physical resynchronization");
    for (size_t i = stalled; i < wire.size(); ++i) {
        if (is_neutral(wire[i])) continue;
        const auto& left = wire[i].decoded.hd.actuators[0];
        require(left.sample_count == 1, "congestion replayed host substeps whose time had passed");
        expect_sample(left.samples[0], kSteps[2]);
    }
    expect_state(target, kSteps[2]);
    require(diagnostics().completed_commands == 1,
            "resynchronized latest endpoint did not finish its current command");
}

void test_feedback_resume() {
    auto target = device();
    attach_approved(target);
    SwitchHapticsDecoder host;
    submit(host.decode(kSeed));
    flush();
    const uint64_t receipt = now_us;
    require(switch_native_output_feedback(&target, 0, 255, 6), "approved local feedback was rejected");
    submit(host.decode(kThree));
    flush();
    expect_state(target, {}, {64, 64, 0, 17867});
    advance_ms(5);
    expect_state(target, {}, {64, 64, 0, 17867});
    const size_t before_resume = wire.size();
    advance_ms(1);
    require(wire.size() > before_resume && is_neutral(wire[before_resume]),
            "local-feedback handoff omitted host resynchronization");
    require(last_frame(target).decoded.hd.actuators[0].sample_count == 1,
            "feedback resumed host timeline from its beginning instead of current substep");
    expect_state(target, kSteps[2]);
    run_until(receipt + 49000);
    expect_state(target, kSteps[2]);
    run_until(receipt + 50000);
    require(is_neutral(last_frame(target)), "Switch host effect survived its original 50ms deadline");
    const size_t expired = wire.size();
    advance_ms(100);
    expect_silence_since(expired);
}

void test_feedback_outlives_host() {
    auto target = device();
    attach_approved(target);
    require(switch_native_output_feedback(&target, 0, 255, 60), "local feedback was rejected");
    const uint64_t receipt = now_us;
    submit(three_steps());
    flush();
    run_until(receipt + 49000);
    expect_state(target, {}, {64, 64, 0, 17867});
    const size_t before_expiry = wire.size();
    run_until(receipt + 50000);
    require(wire.size() == before_expiry,
            "expiry of suppressed host command interrupted active local feedback");
    run_until(receipt + 59000);
    expect_state(target, {}, {64, 64, 0, 17867});
    run_until(receipt + 60000);
    require(is_neutral(last_frame(target)), "feedback completion resurrected already expired host output");
    const size_t finished = wire.size();
    advance_ms(100);
    expect_silence_since(finished);
}

void test_feedback_congestion() {
    auto target = device();
    attach_approved(target);
    SwitchHapticsDecoder host;
    submit(host.decode(kSeed));
    flush();
    send_results = {true, false};  // Neutral reaches the sink; prepared feedback does not.
    require(switch_native_output_feedback(&target, 0, 255, 30), "local feedback was rejected");
    require(is_neutral(last_frame(target)), "feedback congestion fixture did not accept its neutral barrier");
    submit(host.decode(kThree));
    flush();
    advance_ms(1);
    expect_state(target, {}, {64, 64, 0, 17867});
    advance_ms(28);
    expect_state(target, {}, {64, 64, 0, 17867});
    advance_ms(1);
    expect_state(target, kSteps[2]);
    require(last_frame(target).decoded.hd.actuators[0].sample_count == 1,
            "congested feedback restarted suppressed host substeps on resume");
}

void test_stateful() {
    auto target = device();
    attach_approved(target);
    submit({255, 0}, true);
    flush();
    expect_state(target, {64, 64, 17867, 0});
    advance_ms(120);
    expect_state(target, {64, 64, 17867, 0});
    require(diagnostics().completed_commands == 1,
            "stateful refreshes were counted as additional host commands");
    require(switch_native_output_feedback(&target, 0, 255, 5), "stateful overlay feedback was rejected");
    expect_state(target, {}, {64, 64, 0, 17867});
    advance_ms(5);
    expect_state(target, {64, 64, 17867, 0});
    advance_ms(200);
    expect_state(target, {64, 64, 17867, 0});
    const size_t before_zero = wire.size();
    submit({}, true);
    flush();
    require(wire.size() > before_zero && is_neutral(wire[before_zero]),
            "explicit XInput zero did not immediately stop physical output");
    expect_silence_since(before_zero);
    const size_t stopped = wire.size();
    advance_ms(200);
    require(wire.size() == stopped && diagnostics().completed_commands == 2,
            "stateful zero kept refreshing or changed host completion accounting");
}
void test_credit_driven_delivery() {
    auto target = device();
    attach_approved(target);
    credit_event_only = true;
    SwitchHapticsDecoder host;
    const auto before = wire.size();
    submit(host.decode(kSeed));
    flush();
    advance_ms(5);
    require(wire.size() == before, "output attempted without an available credit window");
    require(!permission_requests.empty(), "native owner did not request credit notification");
    const auto cid = permission_requests.front();
    permission_requests.pop_front();
    in_credit_event = true;
    require(switch_native_output_on_can_send_now(&target, cid), "credit event was ignored");
    in_credit_event = false;
    expect_bytes(last_frame(target), kSeed, "credit window did not submit current native state");
}

void test_held_state_coalescing() {
    auto target = device();
    attach_approved(target);
    SwitchHapticsDecoder host;
    submit(host.decode(kSeed));
    flush();
    const auto started = wire.size();
    for (unsigned i = 0; i < 10; ++i) {
        advance_ms(8);
        submit(host.decode(kSeed));
        flush();
    }
    require(diagnostics().received_commands == 11 &&
            diagnostics().completed_commands == 1 &&
            diagnostics().coalesced_commands == 10 &&
            diagnostics().dropped_commands == 0,
            "held state was lost or incorrectly counted as new radio submissions");
    require(wire.size() <= started + 2, "identical reports caused redundant radio traffic");
    for (size_t i = started; i < wire.size(); ++i)
        expect_bytes(wire[i], kSeed, "refresh changed a held native effect");
    advance_ms(49);
    require(!is_neutral(last_frame(target)), "coalescing failed to extend the host watchdog");
    advance_ms(2);
    require(is_neutral(last_frame(target)), "held-state watchdog did not expire");
}

void test_pending_hold_preserves_initial_latency() {
    auto target = device();
    attach_approved(target);
    writable = false;
    SwitchHapticsDecoder host;
    const uint64_t first = now_us;
    for (unsigned i = 0; i < 3; ++i) {
        submit(host.decode(kSeed));
        flush();
        advance_ms(8);
    }
    writable = true;
    advance_ms(1);
    expect_bytes(last_frame(target), kSeed, "pending coalescence changed the current effect");
    require(diagnostics().completed_commands == 1 &&
            diagnostics().coalesced_commands == 2 &&
            diagnostics().dropped_commands == 0 &&
            diagnostics().max_latency_us == last_frame(target).submitted_us - first,
            "coalescing hid the initial wait or counted redundant commands as loss");
}

}  // namespace

uint64_t time_us_64() { return now_us; }
uint32_t time_us_32() { return static_cast<uint32_t>(now_us); }

void btstack_run_loop_set_data_source_handler(
    btstack_data_source_t* source,
    void (*handler)(btstack_data_source_t*, btstack_data_source_callback_type_t)) {
    source->handler = handler;
}
void btstack_run_loop_enable_data_source_callbacks(btstack_data_source_t* source,
                                                   uint16_t callbacks) {
    source->callbacks |= callbacks;
}
void btstack_run_loop_add_data_source(btstack_data_source_t* source) {
    require(std::find(sources.begin(), sources.end(), source) == sources.end(),
            "data source was registered twice");
    sources.push_back(source);
}
void btstack_run_loop_poll_data_sources_from_irq() { poll_requested = true; }
void btstack_run_loop_set_timer_handler(btstack_timer_source_t* timer,
                                         void (*handler)(btstack_timer_source_t*)) {
    timer->handler = handler;
}
void btstack_run_loop_set_timer(btstack_timer_source_t* timer, uint32_t timeout_ms) {
    // Pico runloop adds a tick; deadlines use its millisecond clock, not a busy
    // callback loop at the current instant when the owner requests timeout zero.
    timer->due_us = (now_us / 1000 + uint64_t{timeout_ms} + 1) * 1000;
}
void btstack_run_loop_add_timer(btstack_timer_source_t* timer) {
    require(std::find(timers.begin(), timers.end(), timer) == timers.end(),
            "timer added while already scheduled");
    timers.push_back(timer);
}
bool btstack_run_loop_remove_timer(btstack_timer_source_t* timer) {
    const auto found = std::find(timers.begin(), timers.end(), timer);
    if (found == timers.end()) return false;
    timers.erase(found);
    return true;
}

uint8_t l2cap_request_can_send_now_event(uint16_t cid) {
    if (writable && !credit_event_only)
        switch_native_output_on_can_send_now(radio_devices[cid], cid);
    else if (std::find(permission_requests.begin(), permission_requests.end(), cid) ==
             permission_requests.end())
        permission_requests.push_back(cid);
    return 0;
}

bool uni_hid_parser_switch_native_info(uni_hid_device_t* target, uint8_t* type,
                                       uint8_t* firmware_hi, uint8_t* firmware_lo) {
    if (!target || !target->info_ready) return false;
    if (!target->conn.interrupt_cid) target->conn.interrupt_cid = next_cid++;
    radio_devices[target->conn.interrupt_cid] = target;
    if (type) *type = target->controller_type;
    if (firmware_hi) *firmware_hi = 5;
    if (firmware_lo) *firmware_lo = 1;
    return true;
}
bool uni_hid_parser_switch_native_acquire(uni_hid_device_t* target) {
    if (!target->connected || !target->info_ready || !target->acquire_allowed) return false;
    require(!target->native_owned, "parser acquired twice without release");
    target->native_owned = true;
    ++target->acquisitions;
    return true;
}
bool uni_hid_parser_switch_native_send(uni_hid_device_t* target, const uint8_t rumble[8]) {
    require(target && target->connected && target->native_owned,
            "native send reached disconnected or unowned parser");
    require(!credit_event_only || in_credit_event,
            "native sender polled outside the notified credit window");
    bool accepted = writable;
    if (!send_results.empty()) {
        accepted = send_results.front();
        send_results.pop_front();
    }
    if (!accepted) return false;
    WireFrame frame{target, now_us, {}, physical[target].decode(rumble)};
    std::memcpy(frame.bytes.data(), rumble, frame.bytes.size());
    wire.push_back(frame);
    return true;
}
void uni_hid_parser_switch_native_release(uni_hid_device_t* target) {
    require(target && target->connected && target->native_owned,
            "parser released while disconnected or already unowned");
    target->native_owned = false;
    ++target->releases;
}

int main(int argc, char** argv) {
    require(argc == 2, "one regression scenario is required");
    scenario = argv[1];
    switch_native_output_prepare();
    if (std::strcmp(scenario, "approval") == 0) test_approval();
    else if (std::strcmp(scenario, "model-gate") == 0) test_model_gate();
    else if (std::strcmp(scenario, "revocation") == 0) test_revocation(false);
    else if (std::strcmp(scenario, "revocation-expired") == 0) test_revocation(true);
    else if (std::strcmp(scenario, "generation") == 0) test_generation();
    else if (std::strcmp(scenario, "overflow") == 0) test_overflow();
    else if (std::strcmp(scenario, "retry") == 0) test_retry();
    else if (std::strcmp(scenario, "partial-replacement") == 0) test_partial_replacement();
    else if (std::strcmp(scenario, "stalled") == 0) test_stalled_schedule(false);
    else if (std::strcmp(scenario, "stalled-partial") == 0) test_stalled_schedule(true);
    else if (std::strcmp(scenario, "feedback-resume") == 0) test_feedback_resume();
    else if (std::strcmp(scenario, "feedback-outlives-host") == 0) test_feedback_outlives_host();
    else if (std::strcmp(scenario, "feedback-congestion") == 0) test_feedback_congestion();
    else if (std::strcmp(scenario, "stateful") == 0) test_stateful();
    else if (std::strcmp(scenario, "credit-driven") == 0) test_credit_driven_delivery();
    else if (std::strcmp(scenario, "held-state") == 0) test_held_state_coalescing();
    else if (std::strcmp(scenario, "pending-hold") == 0) test_pending_hold_preserves_initial_latency();
    else require(false, "unknown regression scenario");
    return 0;
}
