#pragma once

#include "usb/switch/switch_haptics.h"

struct SwitchNativeHapticsPackets {
    uint8_t count = 0;
    uint8_t bytes[2][8]{};
    bool raw = false;
    bool quantized = false;
};

// Stateful, bounded Nintendo wire encoder. No allocation or floating point.
//
// reset() assumes the caller has established neutral on the physical device.
// encode() advances the modeled physical state for ALL returned packets: submit
// them in order. Loss, rejection, partial submission, or an intervening writer
// requires neutral + reset before encoding the current host state again.
//
// Raw unity is accepted only if decoding against that model reproduces every
// input substep with safe amplitudes/frequencies. It is never used for mono.
// Absolute amplitude codes are capped at 100 (internal LUT index 228), per
// dekuNukem's documented safe range; Q15 is decoder-normalized, NOT a wire code.
// Frequencies are bounded to the documented absolute range, indices 1..127.
//
// Quantization policy:
// * Round Q15 to the nearest safe decoder LUT value; ties choose lower amplitude.
// * First try every supported one-word form without dropping any substep. Next
//   try a one-step prefix establishing the first target or a relative predecessor
//   without rounding a known relative amplitude; otherwise use an absolute baseline.
//   Retain the original 1..3 steps in the final word. A prefix/baseline is setup,
//   not an extra source substep; its duration needs hardware qualification.
// * If neither supported exact prefix nor absolute-baseline schedule works,
//   choose a baseline permitting the exact first step when possible, otherwise
//   the nearest absolute first state. For each remaining step choose among all
//   32 legal commands by
//   |Q15 error| + 128*|log-frequency-index error|, lower command wins ties.
//   A requested zero amplitude MUST stay zero. Every time slot is retained;
//   no peak/latest collapse. Per-step errors are bounded by 17867 Q15 and 126
//   frequency indices, not claimed perceptually equivalent. quantized reports
//   amplitude/frequency rounding, safety clamping, or this sequence approximation.
// * Mono chooses the dominant amplitude independently per band, carrying that
//   side's frequency (left wins ties), and duplicates it into both wire words.
//   Unequal side counts use max(counts) slots, index min(slot,count-1) (hold the
//   shorter side's last state). This temporal quantization sets quantized.
//
// Absolute bytes have independent public golden vectors. Compressed semantics
// and same-word suppression come from the existing host decoder and are NOT
// proof of actuator acceptance: enable only on hardware-qualified model/firmware.
class SwitchNativeHapticsEncoder {
public:
    void reset();
    SwitchNativeHapticsPackets encode(const ControllerRumbleOutput& input,
                                      bool mono, bool allow_raw);

private:
    SwitchHapticsDecoder physical_{};
};
