#pragma once

#include <stdint.h>

#include "usb/switch/switch_haptics.h"

// Single-core, allocation-free 3 kHz stereo PCM timeline. All times use the same
// 64-bit microsecond clock; unsigned clock rollover is supported for intervals
// shorter than 2^63 us. reset() establishes sample zero and zero oscillator phase.
// Host PCM gets 1.5x gain after profile scaling, jointly limited to available
// mixer headroom so two-band balance is preserved. Feedback gain is unchanged.
class SwitchHdRumbleSynth {
public:
    void reset(uint64_t epoch_us);

    // Zero-count sides are untouched (including their independent watchdog).
    // Duplicate timestamps are accepted in call order; the last update wins.
    // Out-of-order or malformed batches are rejected and counted. Pre-epoch
    // batches retain their original substep position and expiry; already
    // expired pre-epoch batches are rejected. Late ordered batches take effect
    // at the render cursor's current substep, never replaying earlier substeps
    // or extending their original expiry.
    bool push(const SwitchHapticsFrame& frame, uint64_t received_us);

    // Signed int8 PCM encoded in bytes, left then right. Calls normally advance
    // monotonically; forward gaps advance phase analytically, not sample by
    // sample. Already consumed samples are returned as silence, never replayed.
    void render(uint64_t first_sample, uint32_t frames,
                uint8_t* interleaved_stereo);

    // Timestamped conventional override on both sides, at 160/320 Hz. Zero
    // duration or two zero magnitudes cancels only the override at at_us.
    // Host oscillators and updates continue underneath it; expiry reveals the
    // current host state. Override phases also free-run from the stream epoch.
    // Feedback timestamps must be chronological independently of push().
    void feedback(uint64_t at_us, uint32_t duration_us,
                  uint8_t low_magnitude, uint8_t high_magnitude);

    uint32_t dropped_updates() const { return dropped_updates_; }

private:
    static constexpr uint8_t kCapacity = 16;

    struct Command {
        int64_t sample = 0;
        int64_t expires = 0;
        SwitchHapticsFrame frame{};
        uint16_t low = 0;
        uint16_t high = 0;
        bool is_feedback = false;
    };

    struct Side {
        SwitchHapticsActuatorFrame frame{1, {}};
        int64_t start = 0;
        int64_t expires = 0;
    };

    // A fixed ring shared by host and feedback commands, ordered by sample.
    // Overflow consumes the oldest command analytically into the live baseline,
    // advancing a discard watermark. Unrendered PCM before that watermark is
    // silence. Thus missing history cannot replay later, and partial-side state
    // and oscillator phase survive eviction without an additional history ring.
    void enqueue(const Command& command);
    void apply(const Command& command);
    void apply_due(bool discarded = false);
    void advance_to(uint64_t sample, bool discarded = false);
    void advance_phases(uint64_t samples);
    uint64_t next_boundary(uint64_t limit) const;
    const SwitchHapticsSample& host_sample(unsigned side) const;
    bool timestamp_sample(uint64_t timestamp_us, int64_t& sample) const;
    void count_drop();

    uint64_t epoch_us_ = 0;
    uint64_t cursor_ = 0;
    uint64_t last_host_us_ = 0;
    uint64_t last_feedback_us_ = 0;
    bool have_host_ = false;
    bool have_feedback_ = false;
    uint8_t head_ = 0;
    uint8_t count_ = 0;
    uint32_t dropped_updates_ = 0;
    Command commands_[kCapacity]{};
    Side sides_[2]{};
    uint32_t phase_[2][2]{};
    int64_t feedback_expires_ = 0;
    uint16_t feedback_low_ = 0;
    uint16_t feedback_high_ = 0;
};
