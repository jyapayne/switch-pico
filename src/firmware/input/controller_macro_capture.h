#pragma once

#include <stdint.h>
#include "core/controller_state.h"

enum class CaptureState : uint8_t {
    kIdle = 0,
    kRecording = 1,
    kStopped = 2,
    kFull = 3,
    kTimedOut = 4,
    kDisconnected = 5,
};

struct CaptureOptions {
    uint8_t channels = 1;
    uint16_t axis_quantum = 512;
    uint16_t trigger_quantum = 1024;
    uint8_t max_events = 8;
    uint32_t max_duration_ms = 10000;
};

struct CaptureEvent {
    uint32_t at_us;
    uint16_t buttons;
    int16_t left_x;
    int16_t left_y;
    int16_t right_x;
    int16_t right_y;
    uint16_t left_trigger;
    uint16_t right_trigger;
};

// Caller serializes access. Timestamps are a monotonically advancing uint32
// microsecond clock, including wrap; unselected event channels are zero.
class ControllerMacroCapture {
public:
    static constexpr uint16_t kCapacity = 128;

    bool start(uint8_t slot, uint32_t generation,
               const CaptureOptions& options, uint32_t now_us,
               const ControllerState& initial);
    void observe(uint8_t slot, uint32_t generation, uint32_t now_us,
                 const ControllerState& state);
    void stop(uint32_t now_us);
    void disconnect(uint8_t slot, uint32_t generation, uint32_t now_us);
    void tick(uint32_t now_us);

    CaptureState state() const { return state_; }
    const CaptureOptions& options() const { return options_; }
    uint8_t slot() const { return slot_; }
    uint32_t generation() const { return generation_; }
    uint32_t run_id() const { return run_id_; }
    uint16_t event_count() const { return event_count_; }
    uint32_t elapsed_us(uint32_t now_us) const;
    bool event(uint16_t index, CaptureEvent* output) const;

private:
    CaptureEvent snapshot(const ControllerState& state, uint32_t at_us) const;
    bool append(const CaptureEvent& event);
    void advance(uint32_t now_us, bool ending);
    void finish(CaptureState state, uint32_t at_us);

    CaptureOptions options_{};
    CaptureState state_ = CaptureState::kIdle;
    uint8_t slot_ = 0;
    uint32_t generation_ = 0;
    uint32_t run_id_ = 0;
    uint32_t started_us_ = 0;
    uint32_t ended_us_ = 0;
    uint16_t event_count_ = 0;
    CaptureEvent events_[kCapacity]{};
};
