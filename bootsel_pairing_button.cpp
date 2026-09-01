#include "bootsel_pairing_button.h"

#include "hardware/gpio.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include "pico/flash.h"
#include "pico/time.h"
#if PICO_RP2350
#include "hardware/regs/sio.h"
#endif

namespace {

constexpr uint32_t kPollIntervalMs = 100;
constexpr uint32_t kFlashSafeTimeoutMs = 100;
constexpr uint32_t kQspiCsPinIndex = 1;

BootselPairingButtonHoldFsm g_hold_fsm;
uint32_t g_last_sample_ms = 0;

// QSPI CSn sampling adapted from awalol/DS5Dongle's button_functions.cpp:
// https://github.com/awalol/DS5Dongle/blob/master/src/button_functions.cpp
// Copyright (c) 2026 awalol; used under the MIT License.
//
// This callback and everything it executes while CSn is floated must remain in
// SRAM or be an inlined hardware-register operation. In particular, do not add
// logging or ordinary flash-backed data access here.
void __no_inline_not_in_flash_func(read_bootsel_callback)(void* parameter) {
    auto* pressed = static_cast<bool*>(parameter);

    hw_write_masked(
        &ioqspi_hw->io[kQspiCsPinIndex].ctrl,
        GPIO_OVERRIDE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
        IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);

    for (volatile uint32_t delay = 0; delay < 1000; ++delay) {
    }

#if PICO_RP2350
    *pressed =
        (sio_hw->gpio_hi_in & SIO_GPIO_HI_IN_QSPI_CSN_BITS) == 0;
#else
    *pressed = (sio_hw->gpio_hi_in & (1u << kQspiCsPinIndex)) == 0;
#endif

    hw_write_masked(
        &ioqspi_hw->io[kQspiCsPinIndex].ctrl,
        GPIO_OVERRIDE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_LSB,
        IO_QSPI_GPIO_QSPI_SS_CTRL_OEOVER_BITS);
}

BootselPairingButtonSample sample_bootsel() {
    bool pressed = false;
    const int result = flash_safe_execute(read_bootsel_callback, &pressed,
                                          kFlashSafeTimeoutMs);
    if (result != PICO_OK) {
        return BootselPairingButtonSample::kUnread;
    }
    return pressed ? BootselPairingButtonSample::kPressed
                   : BootselPairingButtonSample::kReleased;
}

}  // namespace

BootselPairingButtonEvent BootselPairingButtonHoldFsm::update(
    BootselPairingButtonSample sample) {
    if (sample == BootselPairingButtonSample::kUnread) {
        return BootselPairingButtonEvent::kNone;
    }

    if (sample == BootselPairingButtonSample::kReleased) {
        pressed_samples_ = 0;
        pairing_reported_ = false;
        clear_reported_ = false;
        return BootselPairingButtonEvent::kNone;
    }

    if (pressed_samples_ < kClearHoldSamples) {
        ++pressed_samples_;
    }
    if (pressed_samples_ >= kClearHoldSamples && !clear_reported_) {
        clear_reported_ = true;
        return BootselPairingButtonEvent::kClearPairings;
    }
    if (pressed_samples_ >= kPairingHoldSamples && !pairing_reported_) {
        pairing_reported_ = true;
        return BootselPairingButtonEvent::kOpenPairing;
    }
    return BootselPairingButtonEvent::kNone;
}

BootselPairingButtonEvent bootsel_pairing_button_task() {
    const uint32_t now_ms =
        static_cast<uint32_t>(to_ms_since_boot(get_absolute_time()));
    if (now_ms - g_last_sample_ms < kPollIntervalMs) {
        return BootselPairingButtonEvent::kNone;
    }
    g_last_sample_ms = now_ms;

    return g_hold_fsm.update(sample_bootsel());
}
