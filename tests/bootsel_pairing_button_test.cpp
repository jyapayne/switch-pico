#include "bootsel_pairing_button.h"

#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <vector>

#include "hardware/gpio.h"
#include "hardware/regs/sio.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "pico/flash.h"
#include "pico/time.h"

namespace {

#if PICO_RP2350
constexpr uint32_t kBootselInputMask = SIO_GPIO_HI_IN_QSPI_CSN_BITS;
#else
constexpr uint32_t kBootselInputMask = 1u << 1u;
#endif

struct FlashResponse {
    int result;
    bool pressed;
};

ioqspi_hw_t qspi_registers{};
sio_hw_t sio_registers{};
uint64_t now_ms = 0;
std::vector<FlashResponse> flash_responses;
std::size_t next_flash_response = 0;
std::vector<uint32_t> qspi_override_writes;
int flash_safe_calls = 0;
bool inside_flash_safe_callback = false;

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

std::vector<BootselPairingButtonEvent> apply_pressed(
    BootselPairingButtonHoldFsm& fsm, int count) {
    std::vector<BootselPairingButtonEvent> events;
    for (int sample = 0; sample < count; ++sample) {
        const BootselPairingButtonEvent event =
            fsm.update(BootselPairingButtonSample::kPressed);
        if (event != BootselPairingButtonEvent::kNone) {
            events.push_back(event);
        }
    }
    return events;
}

void test_short_press() {
    BootselPairingButtonHoldFsm fsm;
    require(apply_pressed(fsm, 19).empty(),
            "a 19-sample press must not complete the hold");
    require(fsm.update(BootselPairingButtonSample::kReleased) ==
                BootselPairingButtonEvent::kNone,
            "a short-press release must not report a hold");
    require(apply_pressed(fsm, 19).empty(),
            "a release must discard the previous short press");
}

void test_pairing_and_clear_events_once() {
    BootselPairingButtonHoldFsm fsm;
    require(apply_pressed(fsm, 19).empty(),
            "the pairing hold must not fire before sample 20");
    require(fsm.update(BootselPairingButtonSample::kPressed) ==
                BootselPairingButtonEvent::kOpenPairing,
            "pairing must fire on exactly sample 20");
    require(apply_pressed(fsm, 79).empty(),
            "a long hold must not fire between pairing and clearing");
    require(fsm.update(BootselPairingButtonSample::kPressed) ==
                BootselPairingButtonEvent::kClearPairings,
            "clearing must fire on exactly sample 100");
    require(apply_pressed(fsm, 100).empty(),
            "a continuously held button must not repeat either event");
}

void test_release_and_rearm() {
    BootselPairingButtonHoldFsm fsm;
    const auto first_events = apply_pressed(fsm, 100);
    require(first_events.size() == 2 &&
                first_events[0] ==
                    BootselPairingButtonEvent::kOpenPairing &&
                first_events[1] ==
                    BootselPairingButtonEvent::kClearPairings,
            "the initial long hold must report pairing then clearing");
    require(fsm.update(BootselPairingButtonSample::kReleased) ==
                BootselPairingButtonEvent::kNone,
            "release must rearm without reporting an event");
    const auto second_events = apply_pressed(fsm, 20);
    require(second_events.size() == 1 &&
                second_events[0] ==
                    BootselPairingButtonEvent::kOpenPairing,
            "a valid release must permit a later pairing hold");
}

void test_unread_samples_do_not_transition() {
    BootselPairingButtonHoldFsm fsm;
    require(apply_pressed(fsm, 10).empty(),
            "the first half of a pairing hold must not fire");
    for (int sample = 0; sample < 8; ++sample) {
        require(fsm.update(BootselPairingButtonSample::kUnread) ==
                    BootselPairingButtonEvent::kNone,
                "unread press samples must not report or reset a hold");
    }
    require(apply_pressed(fsm, 9).empty(),
            "valid pressed samples must resume after unread samples");
    require(fsm.update(BootselPairingButtonSample::kPressed) ==
                BootselPairingButtonEvent::kOpenPairing,
            "20 valid pressed samples must fire despite unread samples");

    require(fsm.update(BootselPairingButtonSample::kUnread) ==
                BootselPairingButtonEvent::kNone,
            "an unread release must not rearm a completed hold");
    require(apply_pressed(fsm, 79).empty(),
            "the long hold must continue across an unread sample");
    require(fsm.update(BootselPairingButtonSample::kPressed) ==
                BootselPairingButtonEvent::kClearPairings,
            "100 valid pressed samples must clear despite unread samples");
    require(fsm.update(BootselPairingButtonSample::kReleased) ==
                BootselPairingButtonEvent::kNone,
            "a valid release must only rearm");
    const auto events = apply_pressed(fsm, 20);
    require(events.size() == 1 &&
                events[0] == BootselPairingButtonEvent::kOpenPairing,
            "the FSM must fire after the eventual valid release");
}

BootselPairingButtonEvent run_sample(
    uint64_t sample_time_ms, int result, bool pressed) {
    flash_responses.push_back({result, pressed});
    now_ms = sample_time_ms;
    const std::size_t expected_consumed = flash_responses.size();
    const BootselPairingButtonEvent event = bootsel_pairing_button_task();
    require(next_flash_response == expected_consumed,
            "a due poll must invoke flash_safe_execute exactly once");
    return event;
}

void test_sampler_cadence_and_callback_failure() {
    now_ms = 0;
    require(bootsel_pairing_button_task() ==
                BootselPairingButtonEvent::kNone,
            "the sampler must wait for its first 100 ms cadence");
    now_ms = 99;
    require(bootsel_pairing_button_task() ==
                BootselPairingButtonEvent::kNone,
            "the sampler must not poll before 100 ms");
    require(flash_safe_calls == 0,
            "sub-cadence task calls must not enter flash-safe execution");

    require(run_sample(100, PICO_OK, true) ==
                BootselPairingButtonEvent::kNone,
            "the first valid pressed sample must only start the hold");
    require(flash_safe_calls == 1 && qspi_override_writes.size() == 2,
            "a successful sample must float and restore QSPI CSn once");
    const uint32_t disabled =
        GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB;
    require(qspi_override_writes[0] == disabled,
            "the callback must float QSPI CSn before reading BOOTSEL");
    require(qspi_override_writes[1] == 0,
            "the callback must restore normal QSPI CSn control");

    now_ms = 199;
    require(bootsel_pairing_button_task() ==
                BootselPairingButtonEvent::kNone,
            "the sampler must remain gated between 10 Hz polls");
    require(flash_safe_calls == 1,
            "an early task call must not sample BOOTSEL");

    const std::size_t writes_before_failure = qspi_override_writes.size();
    require(run_sample(200, -1, true) ==
                BootselPairingButtonEvent::kNone,
            "flash-safe failure must be treated as unread");
    require(qspi_override_writes.size() == writes_before_failure,
            "a failed flash-safe entry must not invoke the callback");

    for (uint64_t time = 300; time < 2100; time += 100) {
        require(run_sample(time, PICO_OK, true) ==
                    BootselPairingButtonEvent::kNone,
                "the sampler must wait for 20 valid pressed samples");
    }
    require(run_sample(2100, PICO_OK, true) ==
                BootselPairingButtonEvent::kOpenPairing,
            "a failed sample must not reset the valid pressed count");
    require(run_sample(2200, PICO_OK, true) ==
                BootselPairingButtonEvent::kNone,
            "a held button must not repeat pairing");

    require(run_sample(2300, -1, false) ==
                BootselPairingButtonEvent::kNone,
            "a failed release sample must remain unread");
    require(run_sample(2400, PICO_OK, true) ==
                BootselPairingButtonEvent::kNone,
            "an unread release must not rearm the sampler FSM");
    require(run_sample(2500, PICO_OK, false) ==
                BootselPairingButtonEvent::kNone,
            "a valid release must rearm without firing");

    for (uint64_t time = 2600; time < 4500; time += 100) {
        require(run_sample(time, PICO_OK, true) ==
                    BootselPairingButtonEvent::kNone,
                "the rearmed sampler must count a fresh hold");
    }
    require(run_sample(4500, PICO_OK, true) ==
                BootselPairingButtonEvent::kOpenPairing,
            "a valid release must permit a second pairing hold");
}

}  // namespace

ioqspi_hw_t* ioqspi_hw = &qspi_registers;
sio_hw_t* sio_hw = &sio_registers;

absolute_time_t get_absolute_time() {
    return now_ms;
}

uint64_t to_ms_since_boot(absolute_time_t time) {
    return time;
}

void bootsel_test_masked_write(io_rw_32* address, uint32_t, uint32_t mask) {
    require(inside_flash_safe_callback,
            "QSPI override writes must occur inside flash_safe_execute");
    require(address == &ioqspi_hw->io[1].ctrl,
            "the callback must only override QSPI CSn");
    qspi_override_writes.push_back(*address & mask);
}

int flash_safe_execute(void (*function)(void*), void* parameter,
                       uint32_t enter_exit_timeout_ms) {
    require(enter_exit_timeout_ms == 100,
            "BOOTSEL sampling must use the 100 ms flash-safe timeout");
    require(next_flash_response < flash_responses.size(),
            "flash-safe execution requires a queued test response");
    ++flash_safe_calls;
    const FlashResponse response = flash_responses[next_flash_response++];
    if (response.result != PICO_OK) {
        return response.result;
    }

    sio_hw->gpio_hi_in = response.pressed ? 0 : kBootselInputMask;
    inside_flash_safe_callback = true;
    function(parameter);
    inside_flash_safe_callback = false;
    require((ioqspi_hw->io[1].ctrl &
             IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS) == 0,
            "the callback must restore QSPI CSn before returning");
    return PICO_OK;
}

int main() {
    test_short_press();
    test_pairing_and_clear_events_once();
    test_release_and_rearm();
    test_unread_samples_do_not_transition();
    test_sampler_cadence_and_callback_failure();
    return 0;
}
