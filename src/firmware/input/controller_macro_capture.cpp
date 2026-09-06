#include "input/controller_macro_capture.h"

namespace {

constexpr uint8_t kButtons = 1u << 0;
constexpr uint8_t kLeftStick = 1u << 1;
constexpr uint8_t kRightStick = 1u << 2;
constexpr uint8_t kLeftTrigger = 1u << 3;
constexpr uint8_t kRightTrigger = 1u << 4;
constexpr uint32_t kMaxHoldUs = 10000000;
constexpr uint32_t kMaxDurationMs = 80000;

uint16_t button_mask(const ControllerState& state) {
    return static_cast<uint16_t>(
        (state.button_south ? 1u << 0 : 0) |
        (state.button_east ? 1u << 1 : 0) |
        (state.button_west ? 1u << 2 : 0) |
        (state.button_north ? 1u << 3 : 0) |
        (state.button_left_shoulder ? 1u << 4 : 0) |
        (state.button_right_shoulder ? 1u << 5 : 0) |
        (state.button_select ? 1u << 6 : 0) |
        (state.button_start ? 1u << 7 : 0) |
        (state.button_system ? 1u << 8 : 0) |
        (state.button_capture ? 1u << 9 : 0) |
        (state.button_left_stick ? 1u << 10 : 0) |
        (state.button_right_stick ? 1u << 11 : 0) |
        (state.dpad_up ? 1u << 12 : 0) |
        (state.dpad_down ? 1u << 13 : 0) |
        (state.dpad_left ? 1u << 14 : 0) |
        (state.dpad_right ? 1u << 15 : 0));
}

// Round magnitudes to nearest, with ties away from zero. Keep full scale
// exact and use the same magnitude on both sides of zero where representable.
int16_t quantize_axis(int16_t value, uint16_t quantum) {
    if (value == INT16_MIN) {
        return value;
    }
    const int32_t magnitude = value < 0 ? -static_cast<int32_t>(value) : value;
    if (magnitude == INT16_MAX) {
        return value;
    }
    int32_t rounded = ((magnitude + quantum / 2) / quantum) * quantum;
    if (rounded > INT16_MAX) {
        rounded = INT16_MAX;
    }
    return static_cast<int16_t>(value < 0 ? -rounded : rounded);
}

uint16_t quantize_trigger(uint16_t value, uint16_t quantum) {
    if (value == UINT16_MAX) {
        return value;
    }
    const uint32_t rounded =
        ((static_cast<uint32_t>(value) + quantum / 2u) / quantum) * quantum;
    return static_cast<uint16_t>(rounded > UINT16_MAX ? UINT16_MAX : rounded);
}

bool same_channels(const CaptureEvent& left, const CaptureEvent& right) {
    return left.buttons == right.buttons && left.left_x == right.left_x &&
           left.left_y == right.left_y && left.right_x == right.right_x &&
           left.right_y == right.right_y &&
           left.left_trigger == right.left_trigger &&
           left.right_trigger == right.right_trigger;
}

}  // namespace

bool ControllerMacroCapture::start(uint8_t slot, uint32_t generation,
                                   const CaptureOptions& options,
                                   uint32_t now_us,
                                   const ControllerState& initial) {
    if (state_ == CaptureState::kRecording || slot >= 4 ||
        options.channels == 0 || options.channels > 31 ||
        options.axis_quantum == 0 || options.axis_quantum > INT16_MAX ||
        options.trigger_quantum == 0 || options.max_events == 0 ||
        options.max_events > kCapacity || options.max_duration_ms == 0 ||
        options.max_duration_ms > kMaxDurationMs) {
        return false;
    }
    options_ = options;
    slot_ = slot;
    generation_ = generation;
    started_us_ = now_us;
    ended_us_ = 0;
    ++run_id_;
    if (run_id_ == 0) {
        ++run_id_;
    }
    events_[0] = snapshot(initial, 0);
    event_count_ = 1;
    state_ = CaptureState::kRecording;
    return true;
}

CaptureEvent ControllerMacroCapture::snapshot(const ControllerState& state,
                                              uint32_t at_us) const {
    CaptureEvent captured{};
    captured.at_us = at_us;
    if ((options_.channels & kButtons) != 0) {
        captured.buttons = button_mask(state);
    }
    if ((options_.channels & kLeftStick) != 0) {
        captured.left_x = quantize_axis(state.left_stick_x, options_.axis_quantum);
        captured.left_y = quantize_axis(state.left_stick_y, options_.axis_quantum);
    }
    if ((options_.channels & kRightStick) != 0) {
        captured.right_x = quantize_axis(state.right_stick_x, options_.axis_quantum);
        captured.right_y = quantize_axis(state.right_stick_y, options_.axis_quantum);
    }
    if ((options_.channels & kLeftTrigger) != 0) {
        captured.left_trigger =
            quantize_trigger(state.left_trigger, options_.trigger_quantum);
    }
    if ((options_.channels & kRightTrigger) != 0) {
        captured.right_trigger =
            quantize_trigger(state.right_trigger, options_.trigger_quantum);
    }
    return captured;
}

void ControllerMacroCapture::finish(CaptureState state, uint32_t at_us) {
    state_ = state;
    ended_us_ = at_us;
}

bool ControllerMacroCapture::append(const CaptureEvent& captured) {
    if (captured.at_us == events_[event_count_ - 1].at_us) {
        events_[event_count_ - 1] = captured;
        return true;
    }
    if (event_count_ >= options_.max_events) {
        finish(CaptureState::kFull, captured.at_us);
        return false;
    }
    events_[event_count_++] = captured;
    return true;
}

void ControllerMacroCapture::advance(uint32_t now_us, bool ending) {
    if (state_ != CaptureState::kRecording) {
        return;
    }
    const uint32_t elapsed = now_us - started_us_;
    const uint32_t duration = options_.max_duration_ms * 1000u;
    const bool timed_out = elapsed >= duration;
    const uint32_t target = timed_out ? duration : elapsed;
    // An ending interval may be exactly 10 seconds; it needs no zero-length
    // trailing event. A live boundary can be replaced by an edge at that time.
    for (uint16_t step = 0; step < kCapacity; ++step) {
        const CaptureEvent& previous = events_[event_count_ - 1];
        const uint32_t boundary = previous.at_us + kMaxHoldUs;
        if (boundary > target ||
            (boundary == target && (ending || timed_out))) {
            break;
        }
        CaptureEvent held = previous;
        held.at_us = boundary;
        if (!append(held)) {
            return;
        }
    }
    if (timed_out) {
        finish(CaptureState::kTimedOut, duration);
    }
}

void ControllerMacroCapture::observe(uint8_t slot, uint32_t generation,
                                     uint32_t now_us,
                                     const ControllerState& state) {
    if (state_ != CaptureState::kRecording || slot != slot_) {
        return;
    }
    if (generation != generation_) {
        advance(now_us, true);
        if (state_ == CaptureState::kRecording) {
            finish(CaptureState::kDisconnected, now_us - started_us_);
        }
        return;
    }
    advance(now_us, false);
    if (state_ != CaptureState::kRecording) {
        return;
    }
    const CaptureEvent captured = snapshot(state, now_us - started_us_);
    if (!same_channels(captured, events_[event_count_ - 1])) {
        append(captured);
    }
}

void ControllerMacroCapture::stop(uint32_t now_us) {
    advance(now_us, true);
    if (state_ == CaptureState::kRecording) {
        finish(CaptureState::kStopped, now_us - started_us_);
    }
}

void ControllerMacroCapture::disconnect(uint8_t slot, uint32_t generation,
                                        uint32_t now_us) {
    if (state_ != CaptureState::kRecording || slot != slot_ ||
        generation != generation_) {
        return;
    }
    advance(now_us, true);
    if (state_ == CaptureState::kRecording) {
        finish(CaptureState::kDisconnected, now_us - started_us_);
    }
}

void ControllerMacroCapture::tick(uint32_t now_us) {
    advance(now_us, false);
}

uint32_t ControllerMacroCapture::elapsed_us(uint32_t now_us) const {
    if (state_ == CaptureState::kIdle) {
        return 0;
    }
    if (state_ != CaptureState::kRecording) {
        return ended_us_;
    }
    const uint32_t elapsed = now_us - started_us_;
    const uint32_t duration = options_.max_duration_ms * 1000u;
    return elapsed < duration ? elapsed : duration;
}

bool ControllerMacroCapture::event(uint16_t index, CaptureEvent* output) const {
    if (output == nullptr || index >= event_count_) {
        return false;
    }
    *output = events_[index];
    return true;
}
