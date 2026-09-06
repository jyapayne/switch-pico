#include "input/switch_hd_rumble_synth.h"
#include "input/switch_hd_rumble_envelope.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

namespace {

constexpr int64_t kWatchdogSamples = 150;  // 50 ms at 3 kHz.
constexpr unsigned kWindowSamples = 24;    // 8 ms, independently split per side.

// round(40 * 2^(index/32) * 2^32 / 3000), index 0..159. High-band
// indices address the same logarithmic table with an offset of 32 (80 Hz base).
constexpr uint32_t kPhaseIncrement[160] = {
    57266231u, 58520198u, 59801623u, 61111108u, 62449267u, 63816728u, 65214133u, 66642136u,
    68101409u, 69592636u, 71116516u, 72673765u, 74265113u, 75891307u, 77553110u, 79251302u,
    80986680u, 82760057u, 84572267u, 86424158u, 88316601u, 90250483u, 92226711u, 94246213u,
    96309936u, 98418849u, 100573941u, 102776224u, 105026730u, 107326516u, 109676661u, 112078267u,
    114532461u, 117040396u, 119603246u, 122222217u, 124898535u, 127633456u, 130428265u, 133284272u,
    136202818u, 139185271u, 142233032u, 145347530u, 148530226u, 151782614u, 155106221u, 158502605u,
    161973360u, 165520115u, 169144533u, 172848316u, 176633202u, 180500965u, 184453421u, 188492425u,
    192619872u, 196837698u, 201147882u, 205552448u, 210053460u, 214653032u, 219353321u, 224156534u,
    229064922u, 234080791u, 239206493u, 244444433u, 249797069u, 255266913u, 260856530u, 266568545u,
    272405636u, 278370542u, 284466063u, 290695059u, 297060452u, 303565229u, 310212442u, 317005210u,
    323946720u, 331040229u, 338289067u, 345696633u, 353266403u, 361001930u, 368906843u, 376984851u,
    385239744u, 393675396u, 402295765u, 411104895u, 420106920u, 429306064u, 438706642u, 448313067u,
    458129845u, 468161582u, 478412986u, 488888866u, 499594138u, 510533826u, 521713061u, 533137089u,
    544811271u, 556741085u, 568932127u, 581390118u, 594120904u, 607130458u, 620424884u, 634010420u,
    647893440u, 662080459u, 676578133u, 691393265u, 706532806u, 722003860u, 737813686u, 753969702u,
    770479489u, 787350793u, 804591530u, 822209790u, 840213840u, 858612128u, 877413285u, 896626134u,
    916259690u, 936323164u, 956825972u, 977777733u, 999188277u, 1021067651u, 1043426121u, 1066274178u,
    1089622542u, 1113482169u, 1137864254u, 1162780236u, 1188241808u, 1214260916u, 1240849767u, 1268020839u,
    1295786880u, 1324160918u, 1353156266u, 1382786530u, 1413065613u, 1444007720u, 1475627372u, 1507939404u,
    1540958977u, 1574701585u, 1609183060u, 1644419580u, 1680427680u, 1717224255u, 1754826570u, 1793252268u,
};

// round(127 * 256 * sin(2*pi*index/256)). Linear interpolation retains
// sub-byte precision until the final two-band mix; opposite phases are exact
// negatives, including rounding, so quantization cannot introduce a DC bias.
constexpr int16_t kSine[256] = {
    0, 798, 1595, 2392, 3187, 3980, 4771, 5558, 6343, 7123, 7900, 8671, 9438, 10198, 10953, 11701,
    12442, 13175, 13901, 14618, 15326, 16025, 16715, 17394, 18063, 18721, 19367, 20002, 20625, 21236, 21834, 22418,
    22989, 23547, 24090, 24618, 25132, 25631, 26114, 26581, 27033, 27468, 27886, 28288, 28673, 29041, 29390, 29723,
    30037, 30333, 30611, 30871, 31112, 31334, 31538, 31722, 31887, 32033, 32160, 32267, 32355, 32424, 32473, 32502,
    32512, 32502, 32473, 32424, 32355, 32267, 32160, 32033, 31887, 31722, 31538, 31334, 31112, 30871, 30611, 30333,
    30037, 29723, 29390, 29041, 28673, 28288, 27886, 27468, 27033, 26581, 26114, 25631, 25132, 24618, 24090, 23547,
    22989, 22418, 21834, 21236, 20625, 20002, 19367, 18721, 18063, 17394, 16715, 16025, 15326, 14618, 13901, 13175,
    12442, 11701, 10953, 10198, 9438, 8671, 7900, 7123, 6343, 5558, 4771, 3980, 3187, 2392, 1595, 798,
    0, -798, -1595, -2392, -3187, -3980, -4771, -5558, -6343, -7123, -7900, -8671, -9438, -10198, -10953, -11701,
    -12442, -13175, -13901, -14618, -15326, -16025, -16715, -17394, -18063, -18721, -19367, -20002, -20625, -21236, -21834, -22418,
    -22989, -23547, -24090, -24618, -25132, -25631, -26114, -26581, -27033, -27468, -27886, -28288, -28673, -29041, -29390, -29723,
    -30037, -30333, -30611, -30871, -31112, -31334, -31538, -31722, -31887, -32033, -32160, -32267, -32355, -32424, -32473, -32502,
    -32512, -32502, -32473, -32424, -32355, -32267, -32160, -32033, -31887, -31722, -31538, -31334, -31112, -30871, -30611, -30333,
    -30037, -29723, -29390, -29041, -28673, -28288, -27886, -27468, -27033, -26581, -26114, -25631, -25132, -24618, -24090, -23547,
    -22989, -22418, -21834, -21236, -20625, -20002, -19367, -18721, -18063, -17394, -16715, -16025, -15326, -14618, -13901, -13175,
    -12442, -11701, -10953, -10198, -9438, -8671, -7900, -7123, -6343, -5558, -4771, -3980, -3187, -2392, -1595, -798,
};

bool due(int64_t sample, uint64_t cursor) {
    return sample <= 0 || static_cast<uint64_t>(sample) <= cursor;
}

bool older(uint64_t timestamp, uint64_t previous) {
    return timestamp - previous > static_cast<uint64_t>(INT64_MAX);
}

int32_t rounded_shift(int32_t value, unsigned shift) {
    const int32_t half = int32_t{1} << (shift - 1);
    return value < 0 ? -((-value + half) >> shift)
                     : (value + half) >> shift;
}

int32_t sine(uint32_t phase) {
    const unsigned index = phase >> 24;
    const int32_t first = kSine[index];
    const int32_t difference = kSine[(index + 1) & 255u] - first;
    const int32_t fraction = static_cast<int32_t>((phase >> 8) & 65535u);
    return first + rounded_shift(difference * fraction, 16);
}

void apply_host_gain(uint16_t& low, uint16_t& high) {
    if ((low | high) == 0) return;
    const uint32_t weighted_low = uint32_t{low} * 2;
    const uint32_t weighted_high = uint32_t{high} * 2;
    const uint32_t total = weighted_low + weighted_high;
    uint32_t target = 65535u;
    if (total < 65535u) {
        // A gentle 0.8-power curve lifts quiet/mid-level effects. Apply it
        // jointly so band balance is unchanged, not separately to each voice.
        const uint32_t index = total >> 8;
        const uint32_t fraction = total & 255u;
        const uint32_t first = SwitchHdRumbleEnvelope::kLevel[index];
        const uint32_t difference = SwitchHdRumbleEnvelope::kLevel[index + 1] - first;
        target = first + ((difference * fraction + 128u) >> 8);
    }
    // Product <= 65536*65535 fits uint32; floor rounding keeps the combined
    // weights <= 65535 and each weight representable in uint16. No clipping.
    low = static_cast<uint16_t>(weighted_low * target / total);
    high = static_cast<uint16_t>(weighted_high * target / total);
}

uint8_t mix(uint32_t low_phase, uint32_t high_phase,
            uint16_t low, uint16_t high) {
    const int32_t sum = (low ? sine(low_phase) * low : 0) +
                        (high ? sine(high_phase) * high : 0);
    // Q8 sine times band weights with a combined ceiling of 65536. The
    // 24-bit shift maps that ceiling to +/-127 without int32 overflow.
    return static_cast<uint8_t>(rounded_shift(sum, 24));
}

}  // namespace

void SwitchHdRumbleSynth::reset(uint64_t epoch_us) {
    epoch_us_ = epoch_us;
    cursor_ = 0;
    last_host_us_ = last_feedback_us_ = 0;
    have_host_ = have_feedback_ = false;
    head_ = count_ = 0;
    dropped_updates_ = 0;
    for (unsigned side = 0; side < 2; ++side) {
        sides_[side] = Side{};
        phase_[side][0] = phase_[side][1] = 0;
    }
    feedback_expires_ = 0;
    feedback_low_ = feedback_high_ = 0;
}

void SwitchHdRumbleSynth::count_drop() {
    if (dropped_updates_ != UINT32_MAX) {
        ++dropped_updates_;
    }
}

bool SwitchHdRumbleSynth::timestamp_sample(uint64_t timestamp_us,
                                           int64_t& sample) const {
    const uint64_t elapsed = timestamp_us - epoch_us_;
    if (elapsed <= static_cast<uint64_t>(INT64_MAX)) {
        // ceil(elapsed * 3 / 1000), without overflowing an intermediate.
        sample = static_cast<int64_t>((elapsed / 1000) * 3 +
                                      ((elapsed % 1000) * 3 + 999) / 1000);
        return true;
    }
    const uint64_t before = epoch_us_ - timestamp_us;
    if (before > static_cast<uint64_t>(INT64_MAX)) {
        return false;  // Exactly half the clock range has ambiguous ordering.
    }
    sample = -static_cast<int64_t>((before / 1000) * 3 +
                                  (before % 1000) * 3 / 1000);
    return true;
}

bool SwitchHdRumbleSynth::push(const SwitchHapticsFrame& frame,
                               uint64_t received_us) {
    Command command;
    if ((have_host_ && older(received_us, last_host_us_)) ||
        !timestamp_sample(received_us, command.sample) ||
        command.sample + kWatchdogSamples <= 0) {
        count_drop();
        return false;
    }
    bool has_update = false;
    for (const auto& actuator : frame.actuators) {
        if (actuator.sample_count > 3) {
            count_drop();
            return false;
        }
        has_update |= actuator.sample_count != 0;
        for (unsigned index = 0; index < actuator.sample_count; ++index) {
            const auto& sample = actuator.samples[index];
            if (sample.low_frequency_index > 127 ||
                sample.high_frequency_index > 127 ||
                sample.low_amplitude_q15 > 32768 ||
                sample.high_amplitude_q15 > 32768) {
                count_drop();
                return false;
            }
        }
    }
    have_host_ = true;
    last_host_us_ = received_us;
    if (has_update) {
        command.frame = frame;
        enqueue(command);
    }
    return true;
}

bool SwitchHdRumbleSynth::push_rumble(uint8_t low_magnitude,
                                      uint8_t high_magnitude,
                                      uint64_t received_us) {
    Command command;
    if ((have_host_ && older(received_us, last_host_us_)) ||
        !timestamp_sample(received_us, command.sample)) {
        count_drop();
        return false;
    }
    have_host_ = true;
    last_host_us_ = received_us;
    command.persistent = true;
    command.frame.actuators[0].sample_count = 1;
    command.frame.actuators[1].sample_count = 1;
    command.frame.actuators[0].samples[0].low_amplitude_q15 =
        (static_cast<uint32_t>(low_magnitude) * 32768u + 127u) / 255u;
    command.frame.actuators[1].samples[0].high_amplitude_q15 =
        (static_cast<uint32_t>(high_magnitude) * 32768u + 127u) / 255u;
    enqueue(command);
    return true;
}

void SwitchHdRumbleSynth::feedback(uint64_t at_us, uint32_t duration_us,
                                  uint8_t low_magnitude,
                                  uint8_t high_magnitude) {
    Command command;
    if ((have_feedback_ && older(at_us, last_feedback_us_)) ||
        !timestamp_sample(at_us, command.sample) ||
        !timestamp_sample(at_us + duration_us, command.expires)) {
        count_drop();
        return;
    }
    have_feedback_ = true;
    last_feedback_us_ = at_us;
    command.is_feedback = true;
    command.low = (static_cast<uint32_t>(low_magnitude) * 32768u + 127u) / 255u;
    command.high = (static_cast<uint32_t>(high_magnitude) * 32768u + 127u) / 255u;
    enqueue(command);
}

void SwitchHdRumbleSynth::enqueue(const Command& command) {
    if (count_ == kCapacity) {
        const int64_t oldest_sample = commands_[head_].sample;
        const int64_t watermark = command.sample < oldest_sample
                                      ? command.sample : oldest_sample;
        advance_to(watermark > 0 ? static_cast<uint64_t>(watermark) : 0, true);
        if (count_ == kCapacity) {
            // An older command from the other producer is itself the oldest.
            // Fold it directly into the baseline without losing either side.
            apply(command);
            count_drop();
            return;
        }
    }
    unsigned position = count_;
    while (position && commands_[(head_ + position - 1) % kCapacity].sample >
                           command.sample) {
        commands_[(head_ + position) % kCapacity] =
            commands_[(head_ + position - 1) % kCapacity];
        --position;
    }
    commands_[(head_ + position) % kCapacity] = command;
    ++count_;
}

void SwitchHdRumbleSynth::apply(const Command& command) {
    if (command.is_feedback) {
        feedback_expires_ = command.expires;
        feedback_low_ = command.low;
        feedback_high_ = command.high;
        return;
    }
    for (unsigned side = 0; side < 2; ++side) {
        if (command.frame.actuators[side].sample_count) {
            sides_[side].frame = command.frame.actuators[side];
            sides_[side].start = command.sample;
            sides_[side].expires = command.sample + kWatchdogSamples;
            sides_[side].persistent = command.persistent;
        }
    }
}

void SwitchHdRumbleSynth::apply_due(bool discarded) {
    while (count_ && due(commands_[head_].sample, cursor_)) {
        apply(commands_[head_]);
        head_ = (head_ + 1) % kCapacity;
        --count_;
        if (discarded) {
            count_drop();
        }
    }
}

const SwitchHapticsSample& SwitchHdRumbleSynth::host_sample(unsigned side) const {
    const Side& state = sides_[side];
    const unsigned spacing = kWindowSamples / state.frame.sample_count;
    unsigned index = 0;
    while (index + 1 < state.frame.sample_count &&
           due(state.start + (index + 1) * spacing, cursor_)) {
        ++index;
    }
    return state.frame.samples[index];
}

uint64_t SwitchHdRumbleSynth::next_boundary(uint64_t limit) const {
    const auto consider = [this, &limit](int64_t boundary) {
        if (!due(boundary, cursor_) && static_cast<uint64_t>(boundary) < limit) {
            limit = static_cast<uint64_t>(boundary);
        }
    };
    if (count_) {
        consider(commands_[head_].sample);
    }
    for (const Side& side : sides_) {
        const unsigned spacing = kWindowSamples / side.frame.sample_count;
        for (unsigned index = 1; index < side.frame.sample_count; ++index) {
            consider(side.start + index * spacing);
        }
        if (!side.persistent) consider(side.expires);
    }
    consider(feedback_expires_);
    return limit;
}

void SwitchHdRumbleSynth::advance_phases(uint64_t samples) {
    // Multiplication modulo 2^32 skips arbitrarily large intervals in O(1).
    const uint32_t count = static_cast<uint32_t>(samples);
    for (unsigned side = 0; side < 2; ++side) {
        const auto& sample = host_sample(side);
        phase_[side][0] += kPhaseIncrement[sample.low_frequency_index] * count;
        phase_[side][1] += kPhaseIncrement[sample.high_frequency_index + 32] * count;
    }
}

void SwitchHdRumbleSynth::advance_to(uint64_t sample, bool discarded) {
    apply_due(discarded);
    while (cursor_ < sample) {
        const uint64_t boundary = next_boundary(sample);
        advance_phases(boundary - cursor_);
        cursor_ = boundary;
        apply_due(discarded);
    }
}

void SwitchHdRumbleSynth::render(uint64_t first_sample, uint32_t frames,
                                uint8_t* interleaved_stereo) {
    if (!frames) {
        advance_to(first_sample);
        return;
    }
    if (!interleaved_stereo) {
        return;
    }
    if (first_sample > UINT64_MAX - frames) {
        memset(interleaved_stereo, 0, static_cast<size_t>(frames) * 2);
        return;
    }
    if (first_sample < cursor_) {
        const uint64_t consumed = cursor_ - first_sample;
        const uint32_t silence = consumed < frames
                                     ? static_cast<uint32_t>(consumed) : frames;
        memset(interleaved_stereo, 0, static_cast<size_t>(silence) * 2);
        interleaved_stereo += static_cast<size_t>(silence) * 2;
        first_sample += silence;
        frames -= silence;
    }
    advance_to(first_sample);
    const uint64_t end = first_sample + frames;
    while (cursor_ < end) {
        const uint64_t boundary = next_boundary(end);
        uint32_t increment[2][2];
        uint16_t amplitude[2][2];
        const bool overlay = !due(feedback_expires_, cursor_) &&
                             (feedback_low_ || feedback_high_);
        for (unsigned side = 0; side < 2; ++side) {
            const auto& sample = host_sample(side);
            increment[side][0] = kPhaseIncrement[sample.low_frequency_index];
            increment[side][1] = kPhaseIncrement[sample.high_frequency_index + 32];
            const bool expired = !sides_[side].persistent &&
                                 due(sides_[side].expires, cursor_);
            amplitude[side][0] = expired ? 0 : sample.low_amplitude_q15;
            amplitude[side][1] = expired ? 0 : sample.high_amplitude_q15;
            if (!overlay) apply_host_gain(amplitude[side][0], amplitude[side][1]);
        }
        uint32_t feedback_low_phase = kPhaseIncrement[64] *
                                      static_cast<uint32_t>(cursor_);
        uint32_t feedback_high_phase = kPhaseIncrement[96] *
                                       static_cast<uint32_t>(cursor_);
        for (; cursor_ < boundary; ++cursor_) {
            if (overlay) {
                const uint8_t value = mix(feedback_low_phase, feedback_high_phase,
                                          feedback_low_, feedback_high_);
                *interleaved_stereo++ = value;
                *interleaved_stereo++ = value;
                feedback_low_phase += kPhaseIncrement[64];
                feedback_high_phase += kPhaseIncrement[96];
            } else {
                for (unsigned side = 0; side < 2; ++side) {
                    *interleaved_stereo++ = mix(phase_[side][0], phase_[side][1],
                                                amplitude[side][0], amplitude[side][1]);
                }
            }
            for (unsigned side = 0; side < 2; ++side) {
                phase_[side][0] += increment[side][0];
                phase_[side][1] += increment[side][1];
            }
        }
        apply_due();
    }
}
