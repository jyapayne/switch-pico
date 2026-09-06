#include "usb/switch/switch_native_haptics.h"
#include "usb/switch/switch_haptics_amplitudes.h"
#include "usb/switch/switch_haptics_commands.h"

#include <cstring>

namespace {
using namespace SwitchHapticsCommands;
constexpr uint8_t kSafeIndex = 228;  // Absolute code 100, not Q15 100.
constexpr uint16_t kSafeQ15 = SwitchHapticsTables::kAmplitudeQ15[kSafeIndex];
constexpr uint32_t kNeutral = 0x40400100u;

struct Band {
    uint8_t amplitude = 0;
    uint8_t frequency = 64;
};
struct Sample {
    Band low;
    Band high;
};
struct Timeline {
    uint8_t count = 1;
    Sample samples[3]{};
};

unsigned difference(unsigned a, unsigned b) { return a > b ? a - b : b - a; }
uint16_t amplitude(Band band) { return SwitchHapticsTables::kAmplitudeQ15[band.amplitude]; }
bool same(Band a, Band b) { return a.frequency == b.frequency && amplitude(a) == amplitude(b); }
bool same(Sample a, Sample b) { return same(a.low, b.low) && same(a.high, b.high); }
bool safe(Band band) { return band.amplitude <= kSafeIndex && band.frequency >= 1 && band.frequency <= 127; }

uint8_t nearest_index(uint16_t q15) {
    if (q15 == 0) return 0;
    unsigned lo = 2, hi = kSafeIndex;
    for (unsigned iteration = 0; iteration < 8 && lo < hi; ++iteration) {
        const unsigned mid = (lo + hi) / 2;
        if (SwitchHapticsTables::kAmplitudeQ15[mid] < q15) lo = mid + 1;
        else hi = mid;
    }
    const unsigned lower = lo == 2 ? 0 : lo - 1;
    return static_cast<uint8_t>(difference(q15, SwitchHapticsTables::kAmplitudeQ15[lower]) <=
                               difference(q15, SwitchHapticsTables::kAmplitudeQ15[lo]) ? lower : lo);
}

uint8_t absolute_code(Band band) {
    const uint16_t q15 = amplitude(band);
    unsigned lo = 0, hi = 100;
    for (unsigned iteration = 0; iteration < 7 && lo < hi; ++iteration) {
        const unsigned mid = (lo + hi) / 2;
        if (SwitchHapticsTables::kAmplitudeQ15[host_amplitude_to_lut_index(mid)] < q15) lo = mid + 1;
        else hi = mid;
    }
    const unsigned lower = lo == 0 ? 0 : lo - 1;
    return static_cast<uint8_t>(difference(q15, SwitchHapticsTables::kAmplitudeQ15[host_amplitude_to_lut_index(lower)]) <=
                               difference(q15, SwitchHapticsTables::kAmplitudeQ15[host_amplitude_to_lut_index(lo)]) ? lower : lo);
}
Band absolute_band(Band band) {
    band.amplitude = host_amplitude_to_lut_index(absolute_code(band));
    return band;
}
bool absolute_exact(Band band) { return same(band, absolute_band(band)); }
uint32_t absolute_word(Sample sample) {
    return 0x40000000u | (uint32_t{sample.high.frequency} << 2u) |
           (uint32_t{absolute_code(sample.high)} << 9u) |
           (uint32_t{sample.low.frequency} << 16u) |
           (uint32_t{absolute_code(sample.low)} << 23u);
}
uint32_t command_word(uint8_t count, const uint8_t high[3], const uint8_t low[3]) {
    uint32_t word = uint32_t{count} << 30u;
    for (uint8_t i = 0; i < count; ++i) {
        const unsigned shift = 20u - 10u * i;
        word |= (uint32_t{high[i]} << shift) | (uint32_t{low[i]} << (shift + 5u));
    }
    return word;
}
Band apply(Band band, uint8_t index) {
    const auto& command = kCommands[index];
    band.amplitude = apply_command(command.amplitude_action, command.amplitude_offset,
                                   band.amplitude, 0, 255);
    band.frequency = apply_command(command.frequency_action, command.frequency_offset,
                                   band.frequency, 64, 127);
    return band;
}

// Only silence has two LUT indices for the same observable state. Deduplicating
// equivalent resulting indices bounds this depth-three search to 32*(1+2+4)
// candidates, rather than re-exploring identical commands.
bool exact_commands(Band state, const Band targets[3], uint8_t count,
                    uint8_t commands[3], uint8_t step = 0) {
    if (step == count) return true;
    uint8_t visited_amplitudes[2]{};
    uint8_t visited_count = 0;
    for (uint8_t command = 0; command < 32; ++command) {
        const Band next = apply(state, command);
        if (!safe(next) || !same(next, targets[step])) continue;
        bool visited = false;
        for (uint8_t i = 0; i < visited_count; ++i) visited |= visited_amplitudes[i] == next.amplitude;
        if (visited) continue;
        visited_amplitudes[visited_count++] = next.amplitude;
        commands[step] = command;
        if (exact_commands(next, targets, count, commands, step + 1)) return true;
    }
    return false;
}

bool inverse_value(CommandAction action, int16_t offset, uint8_t target,
                   uint8_t default_value, uint8_t maximum, uint8_t& baseline) {
    switch (action) {
    case CommandAction::Default:
        baseline = default_value;
        return target == default_value;
    case CommandAction::Substitute:
        baseline = default_value;
        return target == offset;
    case CommandAction::Ignore:
        baseline = target;
        return true;
    case CommandAction::Sum: {
        const int value = static_cast<int>(target) - offset;
        if (value < 0 || value > maximum) return false;
        baseline = static_cast<uint8_t>(value);
        return true;
    }
    }
    return false;
}

bool exact_baseline(const Band targets[3], uint8_t count, Band& baseline,
                    uint8_t commands[3]) {
    // Invert each possible first command, then search the remaining substeps.
    // The second target index handles the silent 0/1 LUT alias explicitly.
    for (uint8_t alias = 0; alias < (targets[0].amplitude == 0 ? 2 : 1); ++alias) {
        for (uint8_t command = 0; command < 32; ++command) {
            const auto& spec = kCommands[command];
            Band candidate{};
            const uint8_t target_amplitude = targets[0].amplitude == 0 ? alias : targets[0].amplitude;
            if (!inverse_value(spec.amplitude_action, spec.amplitude_offset, target_amplitude,
                               0, kSafeIndex, candidate.amplitude) ||
                !inverse_value(spec.frequency_action, spec.frequency_offset, targets[0].frequency,
                               64, 127, candidate.frequency) || !safe(candidate) || !absolute_exact(candidate)) continue;
            candidate = absolute_band(candidate);
            if (exact_commands(candidate, targets, count, commands)) {
                baseline = candidate;
                return true;
            }
        }
    }
    return false;
}

void approximate_commands(Band state, const Band targets[3], uint8_t count,
                          uint8_t commands[3]) {
    for (uint8_t step = 0; step < count; ++step) {
        unsigned best_error = UINT32_MAX;
        uint8_t best = 1;  // Silence is always a legal safe candidate.
        for (uint8_t command = 0; command < 32; ++command) {
            const Band next = apply(state, command);
            if (!safe(next) || (amplitude(targets[step]) == 0 && amplitude(next) != 0)) continue;
            const unsigned error = difference(amplitude(next), amplitude(targets[step])) +
                                   128u * difference(next.frequency, targets[step].frequency);
            if (error < best_error) {
                best_error = error;
                best = command;
            }
        }
        commands[step] = best;
        state = apply(state, best);
    }
}

void split(const Timeline& timeline, Band low[3], Band high[3]) {
    for (uint8_t i = 0; i < timeline.count; ++i) {
        low[i] = timeline.samples[i].low;
        high[i] = timeline.samples[i].high;
    }
}
bool admissible(uint32_t word, uint32_t last, bool have_last,
                Sample start, const Timeline& target) {
    return !have_last || word != last ||
           (target.count == 1 && same(start, target.samples[0]));
}

bool exact_word(Sample start, const Timeline& target, uint32_t last, bool have_last,
                uint32_t& word) {
    const auto accept = [&](uint32_t candidate) {
        if (!admissible(candidate, last, have_last, start, target)) return false;
        word = candidate;
        return true;
    };
    const Sample first = target.samples[0];
    if (target.count == 1 && absolute_exact(first.low) && absolute_exact(first.high) &&
        accept(absolute_word(first))) return true;
    Band low[3]{}, high[3]{};
    split(target, low, high);
    uint8_t low_commands[3]{}, high_commands[3]{};
    if (exact_commands(start.low, low, target.count, low_commands) &&
        exact_commands(start.high, high, target.count, high_commands) &&
        accept(command_word(target.count, high_commands, low_commands))) return true;

    if (target.count == 1) {
        // Type 3 changes one absolute amplitude or frequency coordinate.
        for (uint8_t selected = 0; selected < 2; ++selected) {
            const Band before = selected ? start.high : start.low;
            const Band after = selected ? first.high : first.low;
            if (!same(selected ? start.low : start.high, selected ? first.low : first.high)) continue;
            if (before.frequency == after.frequency && absolute_exact(after) &&
                accept(0x40000002u | selected | (uint32_t{absolute_code(after)} << 23u))) return true;
            if (amplitude(before) == amplitude(after) &&
                accept(0x40000006u | selected | (uint32_t{after.frequency} << 23u))) return true;
        }
    } else if (target.count == 2) {
        // Type 4: one absolute band + opposite-band command, then two commands.
        for (uint8_t selected = 0; selected < 2; ++selected) {
            const Band absolute = selected ? first.high : first.low;
            if (!absolute_exact(absolute)) continue;
            Band selected_targets[3]{selected ? high[1] : low[1], {}, {}};
            uint8_t selected_commands[3]{}, other_commands[3]{};
            if (!exact_commands(absolute_band(absolute), selected_targets, 1, selected_commands) ||
                !exact_commands(selected ? start.low : start.high, selected ? low : high,
                                2, other_commands)) continue;
            const uint8_t high_second = selected ? selected_commands[0] : other_commands[1];
            const uint8_t low_second = selected ? other_commands[1] : selected_commands[0];
            const uint32_t candidate = 0x80000000u | selected |
                (uint32_t{absolute.frequency} << 1u) |
                (uint32_t{high_second} << 8u) | (uint32_t{low_second} << 13u) |
                (uint32_t{other_commands[0]} << 18u) | (uint32_t{absolute_code(absolute)} << 23u);
            if (accept(candidate)) return true;
        }
    }
    return false;
}

void recovery_words(const Timeline& target, uint32_t& baseline_word, uint32_t& steps_word,
                    bool& quantized) {
    Band low[3]{}, high[3]{};
    split(target, low, high);
    uint8_t commands[2][3]{};
    Sample baseline{};
    for (uint8_t band = 0; band < 2; ++band) {
        Band* targets = band ? high : low;
        Band& initial = band ? baseline.high : baseline.low;
        if (exact_baseline(targets, target.count, initial, commands[band])) continue;
        quantized = true;
        if (!exact_baseline(targets, 1, initial, commands[band])) initial = absolute_band(targets[0]);
        approximate_commands(initial, targets, target.count, commands[band]);
    }
    baseline_word = absolute_word(baseline);
    steps_word = command_word(target.count, commands[1], commands[0]);
}

void store_word(uint8_t* bytes, uint32_t word) {
    for (uint8_t i = 0; i < 4; ++i) bytes[i] = static_cast<uint8_t>(word >> (8u * i));
}
uint32_t load_word(const uint8_t* bytes) {
    return uint32_t{bytes[0]} | (uint32_t{bytes[1]} << 8u) |
           (uint32_t{bytes[2]} << 16u) | (uint32_t{bytes[3]} << 24u);
}
bool supported_word(uint32_t word) {
    if (word == 0 || word == kNeutral) return true;
    const unsigned count = word >> 30u;
    if (count == 0) return false;
    return count != 1 || (word & 0x000fffffu) == 0 || (word & 3u) == 0 || (word & 2u) != 0;
}
bool same_frame(const SwitchHapticsFrame& a, const SwitchHapticsFrame& b) {
    for (uint8_t side = 0; side < 2; ++side) {
        const auto& left = a.actuators[side];
        const auto& right = b.actuators[side];
        if (left.sample_count != right.sample_count || left.sample_count > 3) return false;
        for (uint8_t step = 0; step < left.sample_count; ++step) {
            const auto& x = left.samples[step];
            const auto& y = right.samples[step];
            if (x.low_amplitude_q15 != y.low_amplitude_q15 || x.high_amplitude_q15 != y.high_amplitude_q15 ||
                x.low_frequency_index != y.low_frequency_index || x.high_frequency_index != y.high_frequency_index ||
                x.low_amplitude_q15 > kSafeQ15 || x.high_amplitude_q15 > kSafeQ15 ||
                x.low_frequency_index < 1 || x.low_frequency_index > 127 ||
                x.high_frequency_index < 1 || x.high_frequency_index > 127) return false;
        }
    }
    return true;
}
Band normalize_band(uint16_t q15, uint8_t frequency, bool& quantized) {
    Band result{nearest_index(q15), static_cast<uint8_t>(frequency < 1 ? 1 : frequency > 127 ? 127 : frequency)};
    quantized |= amplitude(result) != q15 || result.frequency != frequency;
    return result;
}
}  // namespace

void SwitchNativeHapticsEncoder::reset() { physical_.reset(); }

SwitchNativeHapticsPackets SwitchNativeHapticsEncoder::encode(
    const ControllerRumbleOutput& input, bool mono, bool allow_raw) {
    SwitchNativeHapticsPackets packets{};
    if (allow_raw && !mono && input.raw_valid && input.raw_unmodified &&
        supported_word(load_word(input.raw)) && supported_word(load_word(input.raw + 4))) {
        SwitchHapticsDecoder candidate = physical_;
        const auto decoded = candidate.decode(input.raw);
        if (same_frame(decoded.hd, input.hd)) {
            packets.count = 1;
            packets.raw = true;
            std::memcpy(packets.bytes[0], input.raw, 8);
            physical_ = candidate;
            return packets;
        }
    }

    SwitchHapticsFrame desired{};
    const bool conventional = input.hd.actuators[0].sample_count == 0 &&
                              input.hd.actuators[1].sample_count == 0;
    for (uint8_t side = 0; side < 2; ++side) {
        const auto& source = input.hd.actuators[side];
        auto& frame = desired.actuators[side];
        frame.sample_count = source.sample_count == 0 ? 1 : source.sample_count > 3 ? 3 : source.sample_count;
        packets.quantized |= source.sample_count > 3;
        for (uint8_t step = 0; step < frame.sample_count; ++step) {
            auto& sample = frame.samples[step];
            if (conventional) {
                sample.low_amplitude_q15 = static_cast<uint16_t>((uint32_t{input.low_frequency_magnitude} * kSafeQ15 + 127u) / 255u);
                sample.high_amplitude_q15 = static_cast<uint16_t>((uint32_t{input.high_frequency_magnitude} * kSafeQ15 + 127u) / 255u);
            } else if (source.sample_count != 0) {
                sample = source.samples[step];
            }
        }
    }
    if (mono) {
        SwitchHapticsActuatorFrame mixed{};
        const auto& left = desired.actuators[0];
        const auto& right = desired.actuators[1];
        mixed.sample_count = left.sample_count > right.sample_count ? left.sample_count : right.sample_count;
        packets.quantized |= left.sample_count != right.sample_count;
        for (uint8_t step = 0; step < mixed.sample_count; ++step) {
            const auto& l = left.samples[step < left.sample_count ? step : left.sample_count - 1];
            const auto& r = right.samples[step < right.sample_count ? step : right.sample_count - 1];
            const auto& low = l.low_amplitude_q15 >= r.low_amplitude_q15 ? l : r;
            const auto& high = l.high_amplitude_q15 >= r.high_amplitude_q15 ? l : r;
            mixed.samples[step] = {low.low_frequency_index, high.high_frequency_index,
                                  low.low_amplitude_q15, high.high_amplitude_q15};
        }
        desired.actuators[0] = mixed;
        desired.actuators[1] = mixed;
    }
    Timeline target[2]{};
    for (uint8_t side = 0; side < 2; ++side) {
        target[side].count = desired.actuators[side].sample_count;
        for (uint8_t step = 0; step < target[side].count; ++step) {
            const auto& sample = desired.actuators[side].samples[step];
            target[side].samples[step] = {
                normalize_band(sample.low_amplitude_q15, sample.low_frequency_index, packets.quantized),
                normalize_band(sample.high_amplitude_q15, sample.high_frequency_index, packets.quantized)};
        }
    }

    uint32_t words[2]{};
    bool fits[2]{};
    uint32_t prefixes[2]{};
    bool prefixed[2]{};
    for (uint8_t side = 0; side < 2; ++side) {
        const auto& state = physical_.actuators_[side];
        const Sample start{{state.low_amplitude, state.low_frequency},
                           {state.high_amplitude, state.high_frequency}};
        fits[side] = exact_word(start, target[side], state.last_word, state.have_last_word, words[side]);
        if (!fits[side]) {
            const auto try_prefix = [&](uint32_t prefix) {
                auto after = state;
                SwitchHapticsActuatorFrame ignored{};
                SwitchHapticsDecoder::decode_actuator(after, prefix, ignored);
                const Sample established{{after.low_amplitude, after.low_frequency},
                                         {after.high_amplitude, after.high_frequency}};
                if (!exact_word(established, target[side], after.last_word, after.have_last_word, words[side])) return false;
                prefixes[side] = prefix;
                return true;
            };
            // A one-coordinate prefix can retain a tiny relative amplitude
            // that no absolute code represents, while moving its frequency.
            Timeline first{};
            first.samples[0] = target[side].samples[0];
            uint32_t prefix = 0;
            if (target[side].count > 1 &&
                exact_word(start, first, state.last_word, state.have_last_word, prefix)) {
                prefixed[side] = try_prefix(prefix);
            }
            if (!prefixed[side]) {
                // Also search legal relative predecessors. In particular, the
                // silent LUT index1 can lead to index2 on the next increment;
                // replacing that predecessor by absolute zero would lose it.
                Band low[3]{}, high[3]{};
                split(target[side], low, high);
                uint8_t prefix_commands[2]{};
                bool have_prefix[2]{};
                for (uint8_t band = 0; band < 2; ++band) {
                    for (uint8_t command = 0; command < 32; ++command) {
                        const Band after = apply(band ? start.high : start.low, command);
                        uint8_t suffix[3]{};
                        if (safe(after) && exact_commands(after, band ? high : low, target[side].count, suffix)) {
                            prefix_commands[band] = command;
                            have_prefix[band] = true;
                            break;
                        }
                    }
                }
                if (have_prefix[0] && have_prefix[1]) {
                    uint8_t low_prefix[3]{prefix_commands[0], 0, 0};
                    uint8_t high_prefix[3]{prefix_commands[1], 0, 0};
                    prefixed[side] = try_prefix(command_word(1, high_prefix, low_prefix));
                    // A one-step suffix must not equal its prefix (same-word
                    // suppression). Equivalent reset/hold spellings can avoid
                    // that collision without changing either modeled state.
                    for (uint8_t band = 0; band < 2 && !prefixed[side]; ++band) {
                        const Band before = band ? start.high : start.low;
                        const Band expected = apply(before, prefix_commands[band]);
                        uint8_t* commands = band ? high_prefix : low_prefix;
                        for (uint8_t command = 0; command < 32 && !prefixed[side]; ++command) {
                            const Band after = apply(before, command);
                            if (after.amplitude == expected.amplitude && after.frequency == expected.frequency) {
                                commands[0] = command;
                                prefixed[side] = try_prefix(command_word(1, high_prefix, low_prefix));
                            }
                        }
                        commands[0] = prefix_commands[band];
                    }
                }
            }
        }
    }
    const bool direct = fits[0] && fits[1];
    packets.count = direct ? 1 : 2;
    for (uint8_t side = 0; side < 2; ++side) {
        if (direct) {
            store_word(packets.bytes[0] + 4u * side, words[side]);
        } else if (fits[side]) {
            // Do not quantize a representable side merely because its partner
            // needs a baseline. Repeating its previous word holds its endpoint.
            const auto& state = physical_.actuators_[side];
            store_word(packets.bytes[0] + 4u * side, state.have_last_word ? state.last_word : kNeutral);
            store_word(packets.bytes[1] + 4u * side, words[side]);
        } else if (prefixed[side]) {
            store_word(packets.bytes[0] + 4u * side, prefixes[side]);
            store_word(packets.bytes[1] + 4u * side, words[side]);
        } else {
            uint32_t baseline = 0, steps = 0;
            recovery_words(target[side], baseline, steps, packets.quantized);
            store_word(packets.bytes[0] + 4u * side, baseline);
            store_word(packets.bytes[1] + 4u * side, steps);
        }
    }
    for (uint8_t packet = 0; packet < packets.count; ++packet) physical_.decode(packets.bytes[packet]);
    return packets;
}
