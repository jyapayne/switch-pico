#include "controller_input.h"

#include <string.h>

#include "input/bluepad32_input_backend.h"
#include "input/switch2_mouse_capture.h"
#include "platform/pico/bootsel_pairing_button.h"
#include "platform/pico/system_clock.h"
#include "profile/controller_profile_runtime.h"
#include "pico/stdlib.h"

#if !SWITCH_PICO_SWITCH2_USB_BRIDGE || !SWITCH_PICO_BLUEPAD32 || \
    !SWITCH_PICO_ENABLE_BLE || !SWITCH_PICO_SWITCH2_MOUSE_CAPTURE || \
    !SWITCH_PICO_SWITCH2_MOUSE_CAPTURE_NATIVE
#error "The controller bridge requires Bluepad32 BLE and native Switch 2 capture"
#endif

namespace {
constexpr uint8_t kSourceAddress[] = {SWITCH2_BRIDGE_SOURCE_ADDRESS_BYTES};
static_assert(sizeof(kSourceAddress) == 6, "Select one physical Bluetooth address");
constexpr uint32_t kInputDeadlineMs = 500;
constexpr uint32_t kFlashCoordinationTimeoutMs = 1000;
// The backend publishes stage 2 only after Core 1's flash-safe registration;
// reaching Core 1 already required successful Core 0 registration in start().
constexpr uint32_t kFlashCoordinationStage = 2;
bool g_initialized;
bool g_start_attempted;
bool g_flash_ready;
probe_controller_input g_input;
uint32_t g_received_ms;
}  // namespace

extern "C" void probe_controller_input_clock_init(void) {
    system_clock_initialize();
}

extern "C" void probe_controller_input_init(void) {
    if (g_initialized) return;
    switch2_mouse_capture_init();
    switch2_mouse_capture_select_input(kSourceAddress);
    // Prepare the existing storage services without initializing legacy USB.
    // Core 1 loads their persisted state during the normal backend startup.
    bluepad32_input_backend_init();
    controller_profile_runtime_reset();
    g_initialized = true;
}

extern "C" bool probe_controller_input_start(void) {
    if (!g_initialized) probe_controller_input_init();
    if (g_start_attempted) return g_flash_ready;
    g_start_attempted = true;
    bluepad32_input_backend_start();
    const absolute_time_t deadline = make_timeout_time_ms(kFlashCoordinationTimeoutMs);
    do {
        Bluepad32BackendDiagnostics diagnostics;
        bluepad32_input_backend_diagnostics(&diagnostics);
        if (diagnostics.initialization_stage >= kFlashCoordinationStage) {
            g_flash_ready = true;
            return true;
        }
        sleep_ms(1);
    } while (!time_reached(deadline));
    // Do not reset Core 1 or retry a partially launched backend. It may still
    // be running; a false return keeps USB and its flash writes fail-closed.
    return false;
}

extern "C" bool probe_controller_input_pairing_task(void) {
    if (!g_flash_ready) return false;
    // Use the shared sampler/hold policy, but deliberately do not route its
    // kClearPairings event to recovery or any storage-clearing operation.
    if (bootsel_pairing_button_task() != BootselPairingButtonEvent::kOpenPairing)
        return false;
    bluepad32_input_backend_open_pairing_window();
    return true;
}

extern "C" void probe_controller_input_set_native_stream(bool enabled) {
    switch2_mouse_capture_set_native_stream(g_flash_ready && enabled);
}

extern "C" uint32_t probe_controller_input_peek_native_report(
    uint32_t now_ms, uint8_t report[63]) {
    if (!g_flash_ready) return 0;
    return switch2_mouse_capture_peek_native_report(now_ms, report);
}

extern "C" bool probe_controller_input_commit_native_report(uint32_t serial) {
    if (!g_flash_ready) return false;
    return switch2_mouse_capture_commit_native_report(serial);
}

extern "C" bool probe_controller_input_play_sample(uint8_t sample_id, uint64_t* token) {
    if (!g_flash_ready) {
        if (token != nullptr) *token = 0;
        return false;
    }
    return switch2_mouse_capture_request_sample(
        sample_id, to_ms_since_boot(get_absolute_time()), token);
}

extern "C" int probe_controller_input_sample_result(uint64_t token, uint32_t now_ms) {
    if (!g_flash_ready) return -1;
    return switch2_mouse_capture_sample_result(token, now_ms);
}

extern "C" void probe_controller_input_cancel_sample(void) {
    switch2_mouse_capture_cancel_sample();
}

extern "C" void probe_controller_input_poll(uint32_t now_ms,
                                            probe_controller_input* out) {
    if (out == nullptr) return;
    if (!g_flash_ready) {
        *out = {};
        return;
    }
    Switch2MouseCaptureInput sample;
    if (switch2_mouse_capture_latest_input(g_input.serial, &sample)) {
        g_input.serial = sample.serial;
        g_input.active = sample.active;
        g_received_ms = sample.received_ms;
        // Preserve physical byte meaning: never route through the generic
        // solo Joy-Con rotation/mapping. Teardown fields are already zeroed.
        memcpy(g_input.buttons, sample.buttons, sizeof(g_input.buttons));
        memcpy(g_input.stick, sample.stick, sizeof(g_input.stick));
        g_input.native_status = sample.native_status;
        g_input.mouse_epoch = sample.mouse_epoch;
        g_input.mouse_total_x = sample.mouse_total_x;
        g_input.mouse_total_y = sample.mouse_total_y;
        g_input.mouse_surface = sample.mouse_surface;
    }
    // The producer can be a millisecond ahead of the caller's pre-poll clock.
    // Signed elapsed time tolerates that race and ordinary uint32_t rollover.
    // Expiration latches inactive until a newer capture serial arrives.
    if (g_input.active &&
        static_cast<int32_t>(now_ms - g_received_ms) >=
            static_cast<int32_t>(kInputDeadlineMs)) {
        g_input.active = false;
        memset(g_input.buttons, 0, sizeof(g_input.buttons));
        memset(g_input.stick, 0, sizeof(g_input.stick));
        g_input.native_status = 0;
        g_input.mouse_epoch = 0;
        g_input.mouse_total_x = 0;
        g_input.mouse_total_y = 0;
        g_input.mouse_surface = 0;
    }
    *out = g_input;
}
