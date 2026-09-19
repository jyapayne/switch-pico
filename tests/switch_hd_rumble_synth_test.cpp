#include "input/switch_hd_rumble_synth.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

namespace {

constexpr double kTau = 6.2831853071795864769;
int failures = 0;

void expect(bool condition, const char* scenario) {
    if (!condition) {
        std::cerr << scenario << '\n';
        ++failures;
    }
}

int signed_byte(uint8_t value) {
    return value < 128 ? value : static_cast<int>(value) - 256;
}

SwitchHapticsSample state(uint8_t low_index = 64, uint16_t low = 32768,
                          uint8_t high_index = 64, uint16_t high = 0) {
    return SwitchHapticsSample{low_index, high_index, low, high};
}

SwitchHapticsFrame one_side(unsigned side, SwitchHapticsSample sample = state()) {
    SwitchHapticsFrame frame;
    frame.actuators[side].sample_count = 1;
    frame.actuators[side].samples[0] = sample;
    return frame;
}

NativeHapticsSample native_state(uint16_t low_code = 385, uint16_t low = 1023,
                                 uint16_t high_code = 481, uint16_t high = 0) {
    return {low_code, high_code, low, high};
}

NativeHapticsFrame native_side(unsigned side, NativeHapticsSample sample = native_state()) {
    NativeHapticsFrame frame{};
    frame.actuators[side] = {1, {sample}};
    return frame;
}

uint16_t native_q15(uint16_t amplitude) {
    return static_cast<uint16_t>((uint32_t{amplitude} * 32768 + 511) / 1023);
}

std::vector<uint8_t> render(SwitchHdRumbleSynth& synth, uint64_t first,
                            uint32_t frames) {
    std::vector<uint8_t> pcm(static_cast<size_t>(frames) * 2, 0xcc);
    synth.render(first, frames, pcm.data());
    return pcm;
}

double wave(double cycles, uint16_t amplitude = 32768) {
    return 127.0 * std::sin(kTau * cycles) * std::pow(amplitude / 32768.0, 0.8);
}

double feedback_wave(double cycles, uint16_t amplitude = 32768) {
    return 63.5 * std::sin(kTau * cycles) * amplitude / 32768;
}

template <typename Function>
void expect_wave(const std::vector<uint8_t>& pcm, unsigned side,
                  Function expected, const char* scenario) {
    for (size_t sample = 0; sample < pcm.size() / 2; ++sample) {
        const double wanted = expected(sample);
        const int actual = signed_byte(pcm[sample * 2 + side]);
        if (std::abs(actual - wanted) > 0.65) {
            std::cerr << scenario << ": sample " << sample << " side " << side
                      << " expected " << wanted << ", got " << actual << '\n';
            ++failures;
            return;
        }
    }
}

void expect_silent(const std::vector<uint8_t>& pcm, const char* scenario) {
    expect(std::all_of(pcm.begin(), pcm.end(), [](uint8_t v) { return v == 0; }),
           scenario);
}

double spectral_amplitude(const std::vector<uint8_t>& pcm, unsigned side,
                           double frequency) {
    double real = 0;
    double imaginary = 0;
    const size_t frames = pcm.size() / 2;
    for (size_t n = 0; n < frames; ++n) {
        const double angle = kTau * frequency * n / 3000;
        const int value = signed_byte(pcm[2 * n + side]);
        real += value * std::cos(angle);
        imaginary += value * std::sin(angle);
    }
    return 2 * std::hypot(real, imaginary) / frames;
}

void test_physical_frequency_and_channels() {
    for (unsigned side = 0; side < 2; ++side) {
        for (unsigned band = 0; band < 2; ++band) {
            for (uint8_t index : {0, 32, 64, 96, 127}) {
                SwitchHdRumbleSynth synth;
                synth.reset(123456);
                const auto tone = one_side(side, state(index, band ? 0 : 32768,
                                                       index, band ? 32768 : 0));
                std::vector<uint8_t> pcm(2400);
                for (unsigned first = 0; first < 1200; first += 60) {
                    expect(synth.push(tone, 123456 + first * 1000 / 3),
                           "periodic host refresh accepted");
                    synth.render(first, 60, pcm.data() + first * 2);
                }
                const double frequency = (band ? 80 : 40) * std::exp2(index / 32.0);
                expect_wave(pcm, side, [frequency](size_t n) {
                    return wave(frequency * n / 3000);
                }, "physical frequency and free-running phase");
                expect_wave(pcm, 1 - side, [](size_t) { return 0; },
                            "opposite actuator remains silent");
                double peak_frequency = 0;
                double peak_amplitude = 0;
                for (int offset = -12; offset <= 12; ++offset) {
                    const double candidate = frequency + offset * 0.25;
                    const double amplitude = spectral_amplitude(pcm, side, candidate);
                    if (amplitude > peak_amplitude) {
                        peak_amplitude = amplitude;
                        peak_frequency = candidate;
                    }
                }
                expect(std::abs(peak_frequency - frequency) <= 0.5 &&
                           peak_amplitude > 123 && peak_amplitude < 132,
                       "DFT peak matches physical frequency including extreme indices");
            }
        }
    }
}

void test_linear_mix_headroom() {
    SwitchHdRumbleSynth synth;
    synth.reset(0);
    auto frame = one_side(0, state(64, 32768, 32, 32768));  // Both bands 160 Hz.
    frame.actuators[1] = one_side(1, state(64, 32768, 64, 32768)).actuators[1];
    synth.push(frame, 0);
    const auto pcm = render(synth, 0, 150);
    expect_wave(pcm, 0, [](size_t n) { return wave(160.0 * n / 3000); },
                "coherent full-scale bands use headroom without waveform clipping");
    expect_wave(pcm, 1, [](size_t n) {
        return (wave(160.0 * n / 3000) + wave(320.0 * n / 3000)) / 2;
    }, "full-scale two-band balance is preserved by the joint gain ceiling");
    int sum = 0;
    for (size_t n = 0; n < pcm.size() / 2; ++n) {
        sum += signed_byte(pcm[n * 2]);
        expect(signed_byte(pcm[n * 2]) != -128, "PCM never overflows signed headroom");
    }
    expect(std::abs(sum) <= 1, "symmetric rounding does not add DC bias");

    synth.reset(0);
    synth.push(one_side(0, state(64, 32768, 64, 16384)), 0);
    expect_wave(render(synth, 0, 150), 0, [](size_t n) {
        return 127.0 * (2 * std::sin(kTau * 160.0 * n / 3000) +
                        std::sin(kTau * 320.0 * n / 3000)) / 3;
    }, "joint limiting preserves the two-band amplitude ratio");

    synth.reset(0);
    synth.push(one_side(0, state(64, 8192, 64, 4096)), 0);
    expect_wave(render(synth, 0, 150), 0, [](size_t n) {
        const double peak = 127.0 * std::pow(0.375, 0.8);
        return peak * (2 * std::sin(kTau * 160.0 * n / 3000) +
                       std::sin(kTau * 320.0 * n / 3000)) / 3;
    }, "quiet-effect curve preserves band balance instead of independently boosting voices");

    synth.reset(0);
    synth.push(one_side(0, state(127, 0, 127, 0)), 0);
    expect_silent(render(synth, 0, 180), "profile-zero amplitudes are never boosted");
}

void test_substeps_and_preemption() {
    SwitchHdRumbleSynth synth;
    synth.reset(0);
    SwitchHapticsFrame frame;
    frame.actuators[0] = {3, {state(), state(64, 0), state(64, 16384)}};
    frame.actuators[1] = {2, {state(64, 0), state(64, 0, 64, 32768)}};
    synth.push(frame, 0);
    auto pcm = render(synth, 0, 32);
    expect_wave(pcm, 0, [](size_t n) {
        return wave(160.0 * n / 3000, n < 8 ? 32768 : n < 16 ? 0 : 16384);
    }, "three left substeps occupy 8/8/8 samples then hold");
    expect_wave(pcm, 1, [](size_t n) {
        return n < 12 ? 0 : wave(320.0 * n / 3000);
    }, "two right substeps independently occupy 12/12 samples");

    synth.reset(0);
    synth.push(frame, 0);
    synth.push(one_side(0, state(64, 0)), 3000);  // Sample 9 cancels old step 3.
    pcm = render(synth, 0, 32);
    expect_wave(pcm, 0, [](size_t n) {
        return n < 8 ? wave(160.0 * n / 3000) : 0;
    }, "new batch preempts future old substeps, not already elapsed samples");
    expect_wave(pcm, 1, [](size_t n) {
        return n < 12 ? 0 : wave(320.0 * n / 3000);
    }, "zero-count side preserves pending substeps on the other actuator");
}

void test_multiple_usb_updates_and_watchdogs() {
    SwitchHdRumbleSynth synth;
    synth.reset(1000);
    synth.push(one_side(0), 1000);
    synth.push(one_side(0, state(64, 0)), 6001);  // Ceil to sample 16.
    synth.push(one_side(0, state(64, 16384)), 12000);  // Sample 33.
    auto pcm = render(synth, 0, 64);
    expect_wave(pcm, 0, [](size_t n) {
        return wave(160.0 * n / 3000, n < 16 ? 32768 : n < 33 ? 0 : 16384);
    }, "all USB updates within one 21.333 ms PCM interval are rendered");

    synth.reset(0);
    auto both = one_side(0);
    both.actuators[1] = both.actuators[0];
    synth.push(both, 0);
    synth.push(one_side(0), 20000);
    synth.push(SwitchHapticsFrame{}, 40000);  // Must not refresh either side.
    pcm = render(synth, 0, 230);
    expect_wave(pcm, 0, [](size_t n) {
        return n < 210 ? wave(160.0 * n / 3000) : 0;
    }, "left watchdog expires exactly 50 ms after its own update");
    expect_wave(pcm, 1, [](size_t n) {
        return n < 150 ? wave(160.0 * n / 3000) : 0;
    }, "zero-count right side does not refresh its watchdog");
}

void test_phase_continuity_and_partitioning() {
    SwitchHdRumbleSynth whole;
    SwitchHdRumbleSynth partitioned;
    whole.reset(0);
    partitioned.reset(0);
    for (SwitchHdRumbleSynth* synth : {&whole, &partitioned}) {
        synth->push(one_side(0), 0);
        synth->push(one_side(0, state(96)), 5000);  // Change frequency at sample 15.
        synth->push(one_side(0, state(96)), 9000);  // Identical state must not reset phase.
        synth->push(one_side(0, state(96, 8192)), 12000);
    }
    const auto pcm = render(whole, 0, 80);
    expect_wave(pcm, 0, [](size_t n) {
        const double cycles = n < 15 ? n * 160.0 / 3000
                                     : (15 * 160.0 + (n - 15) * 320.0) / 3000;
        return wave(cycles, n < 36 ? 32768 : 8192);
    }, "frequency and amplitude transitions preserve accumulated phase");
    std::vector<uint8_t> split(160);
    for (unsigned n = 0; n < 80; ++n) {
        partitioned.render(n, 1, split.data() + n * 2);
    }
    expect(split == pcm, "PCM is independent of render block partitioning");
}

void test_feedback_returns_to_live_host() {
    SwitchHdRumbleSynth synth;
    synth.reset(0);
    synth.push(one_side(0), 0);
    // Feedback may be delivered before an older USB frame drains on Core 1.
    synth.feedback(5001, 4999, 0, 255);  // Samples [16,30), not a whole PCM block.
    synth.push(one_side(0, state(96, 16384)), 8000);  // Sample 24, underneath overlay.
    auto pcm = render(synth, 0, 64);
    expect_wave(pcm, 0, [](size_t n) {
        if (n >= 16 && n < 30) {
            return feedback_wave(320.0 * n / 3000);
        }
        const double cycles = n < 24 ? n * 160.0 / 3000
                                     : (24 * 160.0 + (n - 24) * 320.0) / 3000;
        return wave(cycles, n < 24 ? 32768 : 16384);
    }, "partial feedback expiry returns to live host state and host phase");
    expect_wave(pcm, 1, [](size_t n) {
        return n >= 16 && n < 30 ? feedback_wave(320.0 * n / 3000) : 0;
    }, "feedback overrides both sides only for its actual duration");

    synth.reset(0);
    synth.push(one_side(0), 0);
    synth.feedback(0, 100000, 128, 0);
    synth.feedback(4000, 100000, 0, 0);
    synth.feedback(8000, 100000, 0, 255);
    synth.feedback(12000, 0, 255, 255);
    pcm = render(synth, 0, 60);
    expect_wave(pcm, 0, [](size_t n) {
        if (n < 12) {
            return feedback_wave(160.0 * n / 3000, static_cast<uint16_t>((128u * 32768 + 127) / 255));
        }
        const bool feedback = n >= 24 && n < 36;
        return feedback ? feedback_wave(320.0 * n / 3000) : wave(160.0 * n / 3000);
    }, "zero magnitudes and zero duration cancel override without cancelling host");

    synth.reset(0);
    synth.push(one_side(0), 0);
    synth.feedback(0, 80000, 0, 255);
    pcm = render(synth, 0, 270);
    expect_wave(pcm, 0, [](size_t n) {
        return n < 240 ? feedback_wave(320.0 * n / 3000) : 0;
    }, "feedback expiry cannot resurrect an expired host effect");
}

void test_late_commands_and_clock_rollover() {
    SwitchHdRumbleSynth synth;
    synth.reset(10000);
    SwitchHapticsFrame steps;
    steps.actuators[0] = {3, {state(64, 32768), state(64, 16384), state(64, 8192)}};
    expect(synth.push(steps, 4000), "recent pre-epoch effect is accepted");
    auto pcm = render(synth, 0, 150);
    expect_wave(pcm, 0, [](size_t n) {
        return n < 132 ? wave(160.0 * n / 3000, 8192) : 0;
    }, "pre-epoch effect starts at current substep and keeps original expiry");
    synth.reset(100000);
    expect(!synth.push(steps, 50000) && synth.dropped_updates() == 1,
           "already expired pre-epoch effect is rejected");
    expect_silent(render(synth, 0, 64), "expired pre-epoch effect never replays");

    synth.reset(0);
    render(synth, 0, 40);
    expect(synth.push(steps, 0), "late but ordered frame is accepted");
    pcm = render(synth, 40, 130);
    expect_wave(pcm, 0, [](size_t n) {
        return n + 40 < 150 ? wave(160.0 * (n + 40) / 3000, 8192) : 0;
    }, "late frame skips old substeps and does not restart watchdog");
    expect(!synth.push(one_side(0), UINT64_MAX), "out-of-order timestamp is rejected");

    const uint64_t epoch = UINT64_MAX - 1000;
    synth.reset(epoch);
    synth.push(one_side(0), epoch);
    synth.push(one_side(0, state(64, 16384)), epoch + 3000);
    synth.feedback(epoch + 4000, 1000, 0, 255);
    pcm = render(synth, 0, 30);
    expect_wave(pcm, 0, [](size_t n) {
        if (n >= 12 && n < 15) {
            return feedback_wave(320.0 * n / 3000);
        }
        return wave(160.0 * n / 3000, n < 9 ? 32768 : 16384);
    }, "64-bit microsecond clock rollover preserves order and duration");
    expect(synth.dropped_updates() == 0, "clock rollover is not an out-of-order update");
}

void test_stall_and_overflow() {
    SwitchHdRumbleSynth skipped;
    SwitchHdRumbleSynth rendered;
    skipped.reset(0);
    rendered.reset(0);
    SwitchHapticsFrame steps;
    steps.actuators[0] = {3, {state(32), state(96), state(127)}};
    for (SwitchHdRumbleSynth* synth : {&skipped, &rendered}) {
        synth->push(steps, 0);
        synth->push(one_side(0, state(64)), 12000);
        synth->push(one_side(0, state(32)), 40000);
    }
    render(rendered, 0, 180);
    expect(render(skipped, 180, 80) == render(rendered, 180, 80),
           "forward gap analytically integrates every queued frequency transition");

    constexpr uint64_t far = 3000000000ull;
    expect_silent(render(skipped, far, 64), "giant stall skips stale sound without a PCM backlog");
    expect(skipped.push(one_side(0), (far + 64) * 1000 / 3),
           "fresh effect after giant stall is accepted");
    const auto fresh = render(skipped, far + 64, 150);
    const double fresh_amplitude = spectral_amplitude(fresh, 0, 160);
    expect(fresh_amplitude > 124 && fresh_amplitude < 131,
           "fresh 160 Hz effect resumes at full band amplitude after giant stall");
    expect_wave(fresh, 1, [](size_t) { return 0; },
                "resuming after stall does not activate the other actuator");
    expect_silent(render(skipped, far, 64), "already consumed PCM is not replayable");

    SwitchHdRumbleSynth overflowing;
    SwitchHdRumbleSynth reference;
    overflowing.reset(0);
    reference.reset(0);
    for (unsigned n = 0; n < 40; ++n) {
        const auto frame = one_side(n % 2, state(static_cast<uint8_t>(32 + n % 4 * 16)));
        overflowing.push(frame, n * 1000);
        reference.push(frame, n * 1000);
        render(reference, n * 3, 3);
    }
    expect(overflowing.dropped_updates() > 0, "bounded command ring reports discarded history");
    expect_silent(render(overflowing, 0, 30), "overflow watermark silences discarded past");
    expect(render(overflowing, 120, 120) == render(reference, 120, 120),
           "overflow preserves complete per-side baseline and accumulated phase");
}

void test_stateful_rumble_hold_channels_and_stop() {
    for (unsigned side = 0; side < 2; ++side) {
        SwitchHdRumbleSynth synth;
        synth.reset(0);
        expect(synth.push_rumble(side == 0 ? 128 : 0, side == 1 ? 128 : 0, 0),
               "single-motor persistent command is accepted");
        const auto pcm = render(synth, 0, 1200);
        expect_wave(pcm, side, [side](size_t n) {
            constexpr uint16_t amplitude = (128u * 32768 + 127) / 255;
            return wave((side == 0 ? 160.0 : 320.0) * n / 3000, amplitude);
        }, "stateful motor keeps its isolated frequency and host gain beyond 50 ms");
        expect_wave(pcm, 1 - side, [](size_t) { return 0; },
                    "zero-scaled opposite motor and both unused bands stay silent");
    }

    SwitchHdRumbleSynth synth;
    synth.reset(0);
    synth.push_rumble(255, 255, 0);
    synth.push_rumble(0, 0, 150001);  // Ceil to sample 451.
    const auto pcm = render(synth, 0, 600);
    for (unsigned side = 0; side < 2; ++side) {
        expect_wave(pcm, side, [side](size_t n) {
            return n < 451 ? wave((side == 0 ? 160.0 : 320.0) * n / 3000) : 0;
        }, "explicit zero stops both held motors at its timestamp without gain lift");
    }
    expect_silent(render(synth, 3000, 150), "stopped persistent state cannot reappear after a gap");
}

void test_stateful_rumble_pre_epoch_and_late() {
    SwitchHdRumbleSynth synth;
    synth.reset(1000000);
    expect(synth.push_rumble(128, 255, 1000),
           "persistent command predating the epoch by more than 50 ms is accepted");
    auto pcm = render(synth, 0, 300);
    expect_wave(pcm, 0, [](size_t n) {
        return wave(160.0 * n / 3000, (128u * 32768 + 127) / 255);
    }, "pre-epoch held low motor starts at stream phase zero without expiry");
    expect_wave(pcm, 1, [](size_t n) { return wave(320.0 * n / 3000); },
                "pre-epoch held high motor retains its independent frequency");

    expect(synth.push_rumble(255, 128, 2000), "late ordered persistent command is accepted");
    pcm = render(synth, 300, 300);
    expect_wave(pcm, 0, [](size_t n) { return wave(160.0 * (n + 300) / 3000); },
                "late held command applies at the cursor without resetting host phase");
    expect_wave(pcm, 1, [](size_t n) {
        return wave(320.0 * (n + 300) / 3000, (128u * 32768 + 127) / 255);
    }, "late held command changes amplitude without replaying elapsed history");
    expect_silent(render(synth, 0, 150), "late persistent updates do not make consumed PCM replayable");

    const uint64_t epoch = UINT64_MAX - 1000;
    synth.reset(epoch);
    synth.push_rumble(255, 0, epoch - 100000);
    synth.push_rumble(0, 255, epoch + 3000);
    pcm = render(synth, 0, 300);
    expect_wave(pcm, 0, [](size_t n) { return n < 9 ? wave(160.0 * n / 3000) : 0; },
                "held low motor stops chronologically across clock rollover");
    expect_wave(pcm, 1, [](size_t n) { return n < 9 ? 0 : wave(320.0 * n / 3000); },
                "held high motor starts across clock rollover without phase reset");
}

void test_stateful_rumble_hd_order_and_watchdogs() {
    SwitchHdRumbleSynth synth;
    synth.reset(0);
    auto initial = one_side(0, state(32));
    initial.actuators[1] = one_side(1, state(64, 0, 32, 32768)).actuators[1];
    synth.push(initial, 0);
    synth.push_rumble(255, 255, 5000);  // Sample 15.
    expect(!synth.push(one_side(0, state(64, 0)), 4999),
           "HD cannot overtake a newer stateful host command");
    synth.push(one_side(0, state(96)), 20000);  // Sample 60; left expires at 210.
    expect(!synth.push_rumble(0, 0, 19999),
           "stateful command cannot overtake a newer HD command");
    synth.push(SwitchHapticsFrame{}, 80000);
    expect(synth.dropped_updates() == 2, "both host APIs share chronological rejection accounting");
    auto pcm = render(synth, 0, 300);
    expect_wave(pcm, 0, [](size_t n) {
        const double cycles = n < 15 ? n * 80.0 / 3000 :
            n < 60 ? (15 * 80.0 + (n - 15) * 160.0) / 3000 :
                     (15 * 80.0 + 45 * 160.0 + (n - 60) * 320.0) / 3000;
        return n < 210 ? wave(cycles) : 0;
    }, "HD-stateful-HD transitions preserve phase and restore the updated side watchdog");
    expect_wave(pcm, 1, [](size_t n) {
        const double cycles = n < 15 ? n * 160.0 / 3000 :
                                      (15 * 160.0 + (n - 15) * 320.0) / 3000;
        return wave(cycles);
    }, "zero-count HD sides leave the other motor persistent with continuous phase");

    synth.reset(0);
    synth.push_rumble(255, 255, 0);
    synth.push(one_side(0, state(64, 0)), 0);
    synth.push(one_side(1, state(64, 0, 64, 0)), 1000);
    synth.push_rumble(64, 128, 1000);
    pcm = render(synth, 0, 300);
    expect_wave(pcm, 0, [](size_t n) {
        return n < 3 ? 0 : wave(160.0 * n / 3000, (64u * 32768 + 127) / 255);
    }, "same-timestamp HD supersedes persistent state in call order");
    expect_wave(pcm, 1, [](size_t n) {
        return wave(320.0 * n / 3000, n < 3 ? 32768 : (128u * 32768 + 127) / 255);
    }, "same-timestamp persistent command supersedes HD and disables its watchdog");
}

void test_stateful_rumble_feedback_resume() {
    SwitchHdRumbleSynth synth;
    synth.reset(0);
    synth.push_rumble(255, 0, 0);
    synth.feedback(5001, 74999, 0, 255);  // Samples [16,240).
    synth.push_rumble(128, 255, 20000);  // Update underneath the overlay.
    auto pcm = render(synth, 0, 300);
    expect_wave(pcm, 0, [](size_t n) {
        if (n >= 16 && n < 240) return feedback_wave(320.0 * n / 3000);
        return wave(160.0 * n / 3000, n < 16 ? 32768 : (128u * 32768 + 127) / 255);
    }, "feedback expiry reveals the current persistent low motor and original host phase");
    expect_wave(pcm, 1, [](size_t n) {
        if (n < 16) return 0.0;
        return n < 240 ? feedback_wave(320.0 * n / 3000) : wave(320.0 * n / 3000);
    }, "feedback remains a both-side overlay and resumes the new persistent high motor");

    synth.reset(0);
    synth.push_rumble(255, 255, 0);
    synth.feedback(10000, 70000, 255, 0);
    synth.push_rumble(0, 0, 40000);
    pcm = render(synth, 0, 300);
    for (unsigned side = 0; side < 2; ++side) {
        expect_wave(pcm, side, [side](size_t n) {
            if (n < 30) return wave((side == 0 ? 160.0 : 320.0) * n / 3000);
            return n < 240 ? feedback_wave(160.0 * n / 3000) : 0;
        }, "host zero cannot cancel priority feedback or resurrect held state after its expiry");
    }
}

void test_stateful_rumble_overflow_and_reset() {
    SwitchHdRumbleSynth overflowing;
    SwitchHdRumbleSynth reference;
    overflowing.reset(0);
    reference.reset(0);
    overflowing.push_rumble(128, 255, 0);
    reference.push_rumble(128, 255, 0);
    render(reference, 0, 3);
    for (unsigned n = 1; n <= 40; ++n) {
        const auto frame = one_side(0, state(static_cast<uint8_t>(32 + n % 4 * 16)));
        overflowing.push(frame, n * 1000);
        reference.push(frame, n * 1000);
        render(reference, n * 3, 3);
    }
    expect(overflowing.dropped_updates() > 0, "mixed host command overflow reports discarded history");
    expect_silent(render(overflowing, 0, 30), "mixed command overflow preserves the discard watermark");
    const auto pcm = render(overflowing, 123, 180);
    expect(pcm == render(reference, 123, 180),
           "evicted stateful command preserves per-side expiry mode and phase through HD updates");
    expect_wave(pcm, 1, [](size_t n) { return wave(320.0 * (n + 123) / 3000); },
                "overflow cannot discard the untouched persistent motor state");

    overflowing.push_rumble(255, 255, 200000);
    overflowing.feedback(200000, 1000000, 255, 255);
    overflowing.reset(1000000);
    expect_silent(render(overflowing, 0, 600),
                  "reset clears live persistence and queued host and feedback commands");
    overflowing.reset(1000000);
    expect(overflowing.push_rumble(0, 0, 0) && overflowing.dropped_updates() == 0,
           "reset clears shared ordering and accepts an old profile-zero state");
    expect_silent(render(overflowing, 0, 300), "profile-zero state is silent even with persistent gain");
    overflowing.reset(1000000);
    overflowing.push_rumble(255, 255, 0);
    const auto restarted = render(overflowing, 0, 300);
    for (unsigned side = 0; side < 2; ++side) {
        expect_wave(restarted, side, [side](size_t n) {
            return wave((side == 0 ? 160.0 : 320.0) * n / 3000);
        }, "reset restores zero source phases for both persistent motors");
    }
}

void test_duplicate_order_and_invalid_frames() {
    SwitchHdRumbleSynth synth;
    synth.reset(0);
    synth.push(one_side(0), 1000);
    synth.push(one_side(0, state(64, 8192)), 1000);
    expect(!synth.push(one_side(0, state(64, 0)), 999),
           "older timestamp cannot override newest accepted state");
    auto invalid = one_side(0);
    invalid.actuators[0].sample_count = 4;
    expect(!synth.push(invalid, 2000), "too many substeps rejects whole batch");
    invalid = one_side(0, state(128));
    expect(!synth.push(invalid, 2000), "out-of-range frequency rejects whole batch");
    invalid = one_side(0, state(64, 32769));
    expect(!synth.push(invalid, 2000), "out-of-range linear amplitude rejects whole batch");
    expect(synth.dropped_updates() == 4, "rejected batches are counted");
    const auto pcm = render(synth, 0, 30);
    expect_wave(pcm, 0, [](size_t n) {
        return n < 3 ? 0 : wave(160.0 * n / 3000, 8192);
    }, "duplicate timestamp last-wins without phase reset or malformed-state mutation");
}

void test_native_precision_and_bands() {
    for (uint16_t code : {1, 193, 385, 481, 482, 483, 670}) {
        for (unsigned band = 0; band < 2; ++band) {
            SwitchHdRumbleSynth synth;
            synth.reset(0);
            const unsigned side = band;
            const auto frame = native_side(side, native_state(code, band ? 0 : 1023,
                                                               code, band ? 1023 : 0));
            std::vector<uint8_t> pcm(12000);
            for (unsigned first = 0; first < 6000; first += 60) {
                expect(synth.push_native(frame, first * 1000 / 3),
                       "native periodic refresh accepted");
                synth.render(first, 60, pcm.data() + first * 2);
            }
            const double hz = 10 * std::exp2((code - 1) / 96.0);
            expect_wave(pcm, side, [hz](size_t n) { return wave(hz * n / 3000); },
                        "native 96-step frequency retains wire precision");
            expect_wave(pcm, 1 - side, [](size_t) { return 0; },
                        "native bands stay on their physical actuator");
            expect(spectral_amplitude(pcm, side, hz) > 125,
                   "native PCM has its expected physical spectral peak");
            if (code >= 481 && code <= 483) {
                const double adjacent = 10 * std::exp2(code / 96.0);
                expect(spectral_amplitude(pcm, side, adjacent) < 15,
                       "adjacent native codes are spectrally distinct, not rounded to Switch indices");
            }
        }
    }

    SwitchHdRumbleSynth synth;
    synth.reset(0);
    synth.push_native(native_side(0, native_state(385, 682, 481, 341)), 0);
    expect_wave(render(synth, 0, 150), 0, [](size_t n) {
        return 127.0 * (2 * std::sin(kTau * 160 * n / 3000) +
                        std::sin(kTau * 320 * n / 3000)) / 3;
    }, "native joint gain preserves independent band mixture");

    std::vector<uint8_t> previous;
    for (uint16_t amplitude : {128, 129}) {
        synth.reset(0);
        synth.push_native(native_side(0, native_state(385, amplitude)), 0);
        auto pcm = render(synth, 0, 150);
        expect_wave(pcm, 0, [amplitude](size_t n) {
            return wave(160.0 * n / 3000, native_q15(amplitude));
        }, "native amplitude normalizes all ten bits before existing gain");
        if (!previous.empty()) expect(previous != pcm, "adjacent ten-bit amplitudes remain distinguishable");
        previous = pcm;
    }
}

void test_native_windows_watchdogs_and_legacy() {
    SwitchHdRumbleSynth synth;
    synth.reset(0);
    NativeHapticsFrame frame{};
    frame.actuators[0] = {3, {native_state(), native_state(385, 0), native_state(385, 512)}};
    frame.actuators[1] = {2, {native_state(385, 0), native_state(385, 0, 481, 1023)}};
    synth.push_native(frame, 0);
    synth.push_native(native_side(0, native_state(385, 512)), 20000);
    synth.push_native(NativeHapticsFrame{}, 40000);
    const auto pcm = render(synth, 0, 230);
    expect_wave(pcm, 0, [](size_t n) {
        return n < 210 ? wave(160.0 * n / 3000,
            n < 16 ? 32768 : n < 32 ? 0 : native_q15(512)) : 0;
    }, "native samples use fixed 16-frame spacing with an independent refreshed watchdog");
    expect_wave(pcm, 1, [](size_t n) {
        return n >= 16 && n < 150 ? wave(320.0 * n / 3000) : 0;
    }, "native two-sample update uses 16-frame spacing and untouched side expires at 50 ms");

    synth.reset(0);
    synth.push_native(frame, 0);
    auto legacy = one_side(0);
    legacy.actuators[0] = {3, {state(), state(64, 0), state(64, 16384)}};
    synth.push(legacy, 0);
    const auto mixed = render(synth, 0, 40);
    expect_wave(mixed, 0, [](size_t n) {
        return wave(160.0 * n / 3000, n < 8 ? 32768 : n < 16 ? 0 : 16384);
    }, "legacy replacement uses its own 8 ms window in a native stream");
    expect_wave(mixed, 1, [](size_t n) { return n < 16 ? 0 : wave(320.0 * n / 3000); },
                "legacy partial update does not shorten the other native window");
    expect(!synth.push_native(frame, UINT64_MAX),
           "native and legacy updates share host timestamp ordering");

    synth.reset(0);
    synth.push_rumble(255, 255, 0);
    synth.push_native(native_side(0, native_state(385, 0)), 1000);
    auto held = render(synth, 0, 300);
    expect_wave(held, 0, [](size_t n) { return n < 3 ? wave(160.0 * n / 3000) : 0; },
                "native zero stops the targeted persistent motor");
    expect_wave(held, 1, [](size_t n) { return wave(320.0 * n / 3000); },
                "native zero-count side preserves stateful XInput output");
}

void test_native_late_overflow_and_cancellation() {
    NativeHapticsFrame steps{};
    steps.actuators[0] = {3, {native_state(), native_state(385, 512), native_state(385, 256)}};
    SwitchHdRumbleSynth synth;
    synth.reset(10000);
    expect(synth.push_native(steps, 4000), "recent pre-epoch native update accepted");
    expect_wave(render(synth, 0, 150), 0, [](size_t n) {
        return n < 132 ? wave(160.0 * n / 3000, native_q15(n < 14 ? 512 : 256)) : 0;
    }, "pre-epoch native update keeps original 16-frame sample positions and expiry");
    synth.reset(0);
    render(synth, 0, 40);
    synth.push_native(steps, 0);
    expect_wave(render(synth, 40, 130), 0, [](size_t n) {
        return n + 40 < 150 ? wave(160.0 * (n + 40) / 3000, native_q15(256)) : 0;
    }, "late native update skips elapsed substeps without refreshing expiry");
    synth.reset(100000);
    expect(!synth.push_native(steps, 50000), "expired native pre-epoch command rejected");

    SwitchHdRumbleSynth reference;
    synth.reset(0);
    reference.reset(0);
    for (unsigned n = 0; n < 40; ++n) {
        const auto frame = native_side(n % 2, native_state(static_cast<uint16_t>(385 + n % 5)));
        synth.push_native(frame, n * 1000);
        reference.push_native(frame, n * 1000);
        render(reference, n * 3, 3);
    }
    expect(synth.dropped_updates() > 0, "native bounded timeline accounts for overflow");
    expect_silent(render(synth, 0, 30), "native overflow never replays discarded history");
    expect(render(synth, 120, 120) == render(reference, 120, 120),
           "native overflow preserves partial sides and full-precision accumulated phases");

    synth.reset(0);
    auto both = native_side(0);
    both.actuators[1] = both.actuators[0];
    synth.push_native(both, 0);
    render(synth, 0, 15);
    synth.push_native(both, 10000); // Queued update must also be canceled.
    synth.feedback(5000, 5000, 0, 255);
    synth.cancel_native(1);
    const auto canceled = render(synth, 15, 90);
    expect_wave(canceled, 0, [](size_t n) {
        return n < 15 ? feedback_wave(320.0 * (n + 15) / 3000) : 0;
    }, "native cancel preserves overlay but removes live and queued left host work");
    expect_wave(canceled, 1, [](size_t n) {
        return n < 15 ? feedback_wave(320.0 * (n + 15) / 3000) :
                        wave(160.0 * (n + 15) / 3000);
    }, "native cancellation leaves the other side and its queued updates intact");
    synth.push_native(native_side(0), 35000);
    expect_wave(render(synth, 105, 30), 0, [](size_t n) {
        return wave(160.0 * (n + 105) / 3000);
    }, "fresh native command after cancellation resumes without oscillator reset");
    synth.cancel_native(3);
    expect_silent(render(synth, 135, 120), "both-side cancellation is a lasting stop");
    synth.push_native(both, 100000);
    synth.reset(100000);
    expect_silent(render(synth, 0, 150), "stream reset discards pending native work");
}

void test_native_validation_is_atomic() {
    SwitchHdRumbleSynth synth;
    synth.reset(0);
    synth.push_native(native_side(0), 0);
    auto invalid = native_side(0, native_state(385, 0));
    invalid.actuators[1] = {1, {native_state(671)}};
    expect(!synth.push_native(invalid, 2000), "unmeasured active native frequency rejects whole frame");
    invalid.actuators[1] = {1, {native_state(0)}};
    expect(!synth.push_native(invalid, 2000), "active code zero cannot generate DC");
    invalid.actuators[1] = {1, {native_state(385, 1024)}};
    expect(!synth.push_native(invalid, 2000), "native amplitude overflow rejected");
    invalid.actuators[1].sample_count = 4;
    expect(!synth.push_native(invalid, 2000), "native count overflow rejected");
    expect(synth.push_native(native_side(1, native_state(0, 0, 1023, 0)), 1000),
           "invalid frames do not advance timestamp ordering; silent bands accept wire range");
    const auto pcm = render(synth, 0, 120);
    expect_wave(pcm, 0, [](size_t n) { return wave(160.0 * n / 3000); },
                "malformed right side cannot partially stop left host state");
    expect_wave(pcm, 1, [](size_t) { return 0; }, "silent arbitrary codes never produce DC");
}

void test_native_side_feedback_preserves_host() {
    SwitchHdRumbleSynth synth;
    synth.reset(0);
    auto both = native_side(0);
    both.actuators[1] = both.actuators[0];
    synth.push_native(both, 0);
    synth.feedback_native(5000, 10000, 0, 255);
    const auto pcm = render(synth, 0, 90);
    expect_wave(pcm, 0, [](size_t n) { return wave(160.0 * n / 3000); },
                "right native cue does not mute or attenuate the untouched left host");
    expect_wave(pcm, 1, [](size_t n) {
        return n >= 15 && n < 45 ? feedback_wave(320.0 * n / 3000) :
                                  wave(160.0 * n / 3000);
    }, "native cue overlays only the requested side and resumes live host on expiry");
}

}  // namespace

int main() {
    test_physical_frequency_and_channels();
    test_linear_mix_headroom();
    test_substeps_and_preemption();
    test_multiple_usb_updates_and_watchdogs();
    test_phase_continuity_and_partitioning();
    test_feedback_returns_to_live_host();
    test_late_commands_and_clock_rollover();
    test_stall_and_overflow();
    test_duplicate_order_and_invalid_frames();
    test_stateful_rumble_hold_channels_and_stop();
    test_stateful_rumble_pre_epoch_and_late();
    test_stateful_rumble_hd_order_and_watchdogs();
    test_stateful_rumble_feedback_resume();
    test_stateful_rumble_overflow_and_reset();
    test_native_precision_and_bands();
    test_native_windows_watchdogs_and_legacy();
    test_native_late_overflow_and_cancellation();
    test_native_validation_is_atomic();
    test_native_side_feedback_preserves_host();
    if (failures) {
        std::cerr << failures << " synthesis scenarios failed\n";
        return 1;
    }
    std::cout << "Switch HD rumble synthesis scenarios passed\n";
    return 0;
}
