#include "usb/switch/switch_native_haptics.h"
#include "profile/controller_profile.h"
#include "profile/controller_profile_transform.h"

#include <cstdlib>
#include <cstring>
#include <iostream>

namespace {
constexpr uint8_t kNeutral[8] = {0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40};
constexpr uint8_t kSeed[8] = {0x00, 0x21, 0x40, 0x48, 0x00, 0x01, 0x40, 0x40};
constexpr uint8_t kOne[8] = {0x00, 0x00, 0x10, 0x69, 0x00, 0x01, 0x40, 0x40};
constexpr uint8_t kTwo[8] = {0x00, 0x74, 0x1c, 0xa9, 0x00, 0x01, 0x40, 0x40};
constexpr uint8_t kThree[8] = {0x78, 0x77, 0x1c, 0xe9, 0x00, 0x01, 0x40, 0x40};

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}
void expect_packet(const SwitchNativeHapticsPackets& packets, const uint8_t expected[8],
                   const char* message) {
    require(packets.count == 1 && std::memcmp(packets.bytes[0], expected, 8) == 0, message);
}
ControllerRumbleOutput single(SwitchHapticsSample left, SwitchHapticsSample right = {}) {
    ControllerRumbleOutput output{};
    output.hd.actuators[0].sample_count = 1;
    output.hd.actuators[1].sample_count = 1;
    output.hd.actuators[0].samples[0] = left;
    output.hd.actuators[1].samples[0] = right;
    return output;
}
ControllerRumbleOutput play(SwitchHapticsDecoder& device, const SwitchNativeHapticsPackets& packets) {
    require(packets.count >= 1 && packets.count <= 2, "encoder exceeded bounded packet schedule");
    ControllerRumbleOutput result{};
    for (uint8_t i = 0; i < packets.count; ++i) {
        result = device.decode(packets.bytes[i]);
        for (const auto& side : result.hd.actuators) {
            require(side.sample_count >= 1 && side.sample_count <= 3, "invalid generated substep count");
            for (uint8_t step = 0; step < side.sample_count; ++step) {
                const auto& sample = side.samples[step];
                require(sample.low_amplitude_q15 <= 17867 && sample.high_amplitude_q15 <= 17867,
                        "generated an amplitude above documented absolute code 100");
                require(sample.low_frequency_index >= 1 && sample.low_frequency_index <= 127 &&
                            sample.high_frequency_index >= 1 && sample.high_frequency_index <= 127,
                        "generated frequency outside public absolute range");
            }
        }
    }
    return result;
}
void expect_sample(const SwitchHapticsSample& actual, const SwitchHapticsSample& expected) {
    require(actual.low_frequency_index == expected.low_frequency_index &&
                actual.high_frequency_index == expected.high_frequency_index &&
                actual.low_amplitude_q15 == expected.low_amplitude_q15 &&
                actual.high_amplitude_q15 == expected.high_amplitude_q15,
            "native output lost band frequency/amplitude or substep order");
}

void test_public_absolute_goldens() {
    // Independent dekuNukem rumble_data_table.md byte example: HF=0x1a8,
    // HA=0x88, LF=0x63, LA=0x804d => a8 89 e3 4d. The input amplitudes are
    // this project's normalized LUT values, not the public physical amplitudes.
    SwitchNativeHapticsEncoder encoder;
    constexpr uint8_t expected[8] = {0xa8, 0x89, 0xe3, 0x4d, 0x80, 0x00, 0x40, 0x52};
    const auto output = encoder.encode(single({99, 106, 3371, 8933}, {64, 32, 4467, 0}), false, false);
    expect_packet(output, expected, "public absolute vector/band-actuator isolation mismatch");
    require(!output.raw && !output.quantized, "exact absolute vector was changed");
    expect_packet(encoder.encode({}, false, false), kNeutral, "conventional zero was not exact neutral");

    constexpr uint8_t safe_max[8] = {0x00, 0xc9, 0x40, 0x72, 0x00, 0xc9, 0x40, 0x72};
    expect_packet(encoder.encode({255, 255}, false, true), safe_max,
                  "conventional maximum did not map to safe fixed carriers");
    const auto clamped = encoder.encode(single({64, 64, 32767, 32767}, {64, 64, 32767, 32767}), false, true);
    expect_packet(clamped, safe_max, "unsafe HD amplitude escaped wire code100 clamp");
    require(clamped.quantized && !clamped.raw, "safety clamp was not observable");
}

void test_provenance_and_profile_gains() {
    SwitchHapticsDecoder host;
    SwitchNativeHapticsEncoder encoder;
    const ControllerRumbleOutput conventional{33, 71};
    require(!conventional.raw_valid && !conventional.raw_unmodified,
            "conventional rumble acquired Nintendo wire provenance");
    require(!host.decode(nullptr).raw_valid, "missing payload acquired raw provenance");
    constexpr uint8_t max_left[8] = {0x00, 0xc9, 0x40, 0x72, 0x00, 0x01, 0x40, 0x40};
    const auto decoded = host.decode(max_left);
    require(decoded.raw_valid && decoded.raw_unmodified && std::memcmp(decoded.raw, max_left, 8) == 0,
            "Switch decode did not preserve original bytes and provenance");
    ControllerProfile profile{};
    profile.strong_rumble_scale = 255;
    profile.weak_rumble_scale = 255;
    const auto unity = controller_profile_scale_host_rumble(decoded, profile);
    const auto raw = encoder.encode(unity, false, true);
    expect_packet(raw, max_left, "unity did not preserve independently specified wire bytes");
    require(raw.raw, "safe synchronized unity did not use raw path");
    profile.strong_rumble_scale = 64;
    profile.weak_rumble_scale = 128;
    const auto scaled = controller_profile_scale_host_rumble(decoded, profile);
    require(scaled.raw_valid && !scaled.raw_unmodified, "profile gains failed to revoke raw fast-path permission");
    constexpr uint8_t intermediate[8] = {0x00, 0x89, 0x40, 0x52, 0x00, 0x01, 0x40, 0x40};
    const auto intermediate_packets = encoder.encode(scaled, false, true);
    expect_packet(intermediate_packets, intermediate, "intermediate band gains treated Q15 as wire amplitude");
    require(!intermediate_packets.raw && intermediate_packets.quantized,
            "intermediate Q15 rounding was not reported");
    profile.strong_rumble_scale = 255;
    profile.weak_rumble_scale = 255;
    require(!controller_profile_scale_host_rumble(scaled, profile).raw_unmodified,
            "later unity gain restored revoked raw provenance");
    profile.strong_rumble_scale = 0;
    profile.weak_rumble_scale = 0;
    expect_packet(encoder.encode(controller_profile_scale_host_rumble(decoded, profile), false, true),
                  kNeutral, "zero gains did not produce exact silence");
}

ControllerRumbleOutput sequence(uint8_t count) {
    auto output = single({65, 65, 2139, 2282});
    auto& side = output.hd.actuators[0];
    side.sample_count = count;
    side.samples[1] = {65, 66, 2139, 2093};
    side.samples[2] = {65, 66, 2093, 2093};
    return output;
}

void test_compressed_goldens_and_resynchronization() {
    // Independently hand-packed command indices from existing protocol forms:
    // H=[+4/+1Hz, -4/+1Hz, hold], L=[+1/+1Hz, hold, -1/hold].
    // These defend bit placement/order independently of decoder round trips;
    // they do not claim physical-controller acceptance of compressed forms.
    const uint8_t* goldens[3] = {kOne, kTwo, kThree};
    for (uint8_t count = 1; count <= 3; ++count) {
        SwitchNativeHapticsEncoder encoder;
        encoder.encode(single({64, 64, 2093, 2093}), false, false);
        const auto packets = encoder.encode(sequence(count), false, false);
        expect_packet(packets, goldens[count - 1], "compressed one/two/three-step golden mismatch");
        require(!packets.quantized, "representable ordered substeps were quantized");
    }

    SwitchHapticsDecoder host;
    SwitchNativeHapticsEncoder encoder;
    encoder.encode(host.decode(kSeed), false, true);
    const auto commands = host.decode(kThree);
    const auto forwarded = encoder.encode(commands, false, true);
    expect_packet(forwarded, kThree, "synchronized compressed unity was not exact");
    require(forwarded.raw, "safe synchronized compressed unity did not use raw");
    const auto repeated = host.decode(kThree);
    require(repeated.hd.actuators[0].sample_count == 1, "host repeat did not retain endpoint");
    require(encoder.encode(repeated, false, true).raw, "same-word hold unexpectedly lost synchronization");

    encoder.reset();
    SwitchHapticsDecoder device;
    const auto recovery = encoder.encode(commands, false, true);
    require(!recovery.raw && recovery.count == 2 && !recovery.quantized,
            "dropped history did not trigger exact baseline recovery");
    const auto result = play(device, recovery);
    require(result.hd.actuators[0].sample_count == 3, "recovery collapsed three substeps");
    for (uint8_t step = 0; step < 3; ++step) expect_sample(result.hd.actuators[0].samples[step], sequence(3).hd.actuators[0].samples[step]);

    // A stale raw envelope after changed profile/output must be compared with
    // physical state, not trusted solely because its flags still say unity.
    encoder.encode(single({64, 64, 4467, 0}), false, false);
    require(!encoder.encode(repeated, false, true).raw, "raw reuse ignored changed physical state");
}

void test_absolute_plus_commands_and_selected_coordinate() {
    SwitchNativeHapticsEncoder encoder;
    SwitchHapticsDecoder host;
    SwitchHapticsDecoder device;
    play(device, encoder.encode(host.decode(kSeed), false, true));
    // Type 4: H absolute code32/frequency70; L command20, then H24/L17.
    constexpr uint8_t type4[8] = {0x8d, 0x38, 0x52, 0x90, 0x00, 0x01, 0x40, 0x40};
    const auto desired = host.decode(type4);
    const auto encoded = encoder.encode(desired, false, false);
    require(encoded.count == 1, "representable mixed form needed extra packets");
    const auto actual = play(device, encoded);
    require(actual.hd.actuators[0].sample_count == 2, "mixed form lost a substep");
    expect_sample(actual.hd.actuators[0].samples[0], {65, 70, 2139, 4096});
    expect_sample(actual.hd.actuators[0].samples[1], {66, 70, 2332, 4096});
    require(!encoded.quantized, "representable mixed absolute/relative form was quantized");
    // Preserve non-absolute low amplitude index134 while updating only H freq.
    constexpr uint8_t selected[8] = {0x07, 0x00, 0x00, 0x68, 0x00, 0x01, 0x40, 0x40};
    const auto frequency = host.decode(selected);
    const auto selected_packets = encoder.encode(frequency, false, false);
    require(selected_packets.count == 1, "selected coordinate needed extra packets");
    const auto selected_result = play(device, selected_packets);
    expect_sample(selected_result.hd.actuators[0].samples[0], {66, 80, 2332, 4096});
}

void test_relative_only_amplitude_survives_prefixes() {
    SwitchNativeHapticsEncoder encoder;
    SwitchHapticsDecoder host;
    SwitchHapticsDecoder device;
    // Reach internal index2 through two distinct increment words. It is below
    // the first nonzero absolute code (index15), yet is a legal relative state.
    constexpr uint8_t first[8] = {0x00, 0x00, 0x50, 0x6b, 0x00, 0x01, 0x40, 0x40};
    constexpr uint8_t second[8] = {0x00, 0x00, 0x50, 0x69, 0x00, 0x01, 0x40, 0x40};
    play(device, encoder.encode(host.decode(first), false, true));
    play(device, encoder.encode(host.decode(second), false, true));
    auto input = single({80, 64, 134, 134});
    input.hd.actuators[0].sample_count = 3;
    input.hd.actuators[0].samples[1] = {81, 64, 137, 134};
    input.hd.actuators[0].samples[2] = {82, 64, 140, 134};
    const auto packets = encoder.encode(input, false, false);
    constexpr uint8_t frequency_prefix[4] = {0x06, 0x00, 0x00, 0x68};
    require(packets.count == 2 && !packets.quantized &&
                std::memcmp(packets.bytes[0], frequency_prefix, 4) == 0,
            "single-coordinate prefix rounded an existing relative-only amplitude");
    const auto result = play(device, packets);
    require(result.hd.actuators[0].sample_count == 3, "state prefix lost temporal slots");
    for (uint8_t step = 0; step < 3; ++step)
        expect_sample(result.hd.actuators[0].samples[step], input.hd.actuators[0].samples[step]);

    // The right side now needs an absolute baseline; its partner's relative
    // amplitude must not be rounded just to make both setup words absolute.
    input.hd.actuators[0].samples[0] = {82, 64, 140, 134};
    input.hd.actuators[0].samples[1] = {82, 64, 143, 134};
    input.hd.actuators[0].samples[2] = {83, 64, 146, 134};
    input.hd.actuators[1] = sequence(3).hd.actuators[0];
    const auto partner_recovery = encoder.encode(input, false, false);
    const auto partner_result = play(device, partner_recovery);
    require(partner_recovery.count == 2 && !partner_recovery.quantized,
            "partner recovery quantized an independently representable side");
    for (uint8_t side = 0; side < 2; ++side)
        for (uint8_t step = 0; step < 3; ++step)
            expect_sample(partner_result.hd.actuators[side].samples[step], input.hd.actuators[side].samples[step]);

    // Resync from established neutral must distinguish internal silent index1
    // from absolute zero, or the representable first index2 step gets rounded.
    encoder.reset();
    device.reset();
    input = single({64, 64, 134, 0});
    input.hd.actuators[0].sample_count = 3;
    input.hd.actuators[0].samples[1] = {65, 64, 137, 0};
    input.hd.actuators[0].samples[2] = {66, 64, 140, 0};
    const auto relative_recovery = encoder.encode(input, false, false);
    const auto recovered = play(device, relative_recovery);
    require(relative_recovery.count == 2 && !relative_recovery.quantized,
            "relative-only predecessor was lost during recovery");
    for (uint8_t step = 0; step < 3; ++step)
        expect_sample(recovered.hd.actuators[0].samples[step], input.hd.actuators[0].samples[step]);

    encoder.reset();
    device.reset();
    const auto collision = encoder.encode(single({66, 64, 134, 0}), false, false);
    const auto collision_result = play(device, collision);
    require(collision.count == 2 && !collision.quantized &&
                std::memcmp(collision.bytes[0], collision.bytes[1], 4) != 0,
            "same-word prefix collision suppressed a representable relative increment");
    expect_sample(collision_result.hd.actuators[0].samples[0], {66, 64, 134, 0});
}

void test_mono_bands_ties_and_temporal_policy() {
    SwitchNativeHapticsEncoder encoder;
    auto input = single({32, 80, 4467, 2093}, {96, 100, 2093, 8933});
    constexpr uint8_t mono[8] = {0x90, 0x89, 0x20, 0x52, 0x90, 0x89, 0x20, 0x52};
    expect_packet(encoder.encode(input, true, true), mono, "mono did not independently select dominant bands");
    input.hd.actuators[1].samples[0].low_amplitude_q15 = 4467;
    expect_packet(encoder.encode(input, true, true), mono, "mono tie did not retain left band frequency");

    // Dominance is evaluated BEFORE LUT rounding: 4468 > 4467 although both
    // round to the same safe amplitude. The right low-band frequency must win.
    input.hd.actuators[1].samples[0].low_amplitude_q15 = 4468;
    constexpr uint8_t near_tie[8] = {0x90, 0x89, 0x60, 0x52, 0x90, 0x89, 0x60, 0x52};
    expect_packet(encoder.encode(input, true, true), near_tie, "LUT rounding changed mono dominance");

    encoder.reset();
    SwitchHapticsDecoder device;
    input = sequence(3);
    input.hd.actuators[1].samples[0] = {65, 66, 0, 4467};
    const auto packets = encoder.encode(input, true, false);
    const auto result = play(device, packets);
    require(packets.quantized && result.hd.actuators[0].sample_count == 3 &&
                result.hd.actuators[1].sample_count == 3,
            "unequal-side temporal quantization was hidden or dropped slots");
    for (uint8_t step = 0; step < 3; ++step) {
        const auto expected = SwitchHapticsSample{65, 66, sequence(3).hd.actuators[0].samples[step].low_amplitude_q15, 4467};
        expect_sample(result.hd.actuators[0].samples[step], expected);
        expect_sample(result.hd.actuators[1].samples[step], expected);
    }
}

void test_unrepresentable_timeline_and_safety() {
    SwitchNativeHapticsEncoder encoder;
    SwitchHapticsDecoder device;
    auto input = single({64, 64, 17867, 0});
    auto& left = input.hd.actuators[0];
    left.sample_count = 3;
    left.samples[1] = {127, 1, 0, 17867};
    left.samples[2] = {1, 127, 17867, 0};
    const auto packets = encoder.encode(input, false, false);
    const auto result = play(device, packets);
    const auto& actual = result.hd.actuators[0];
    require(packets.count == 2 && packets.quantized && actual.sample_count == 3,
            "unrepresentable sequence did not expose bounded three-slot quantization");
    require(actual.samples[0].low_amplitude_q15 == 17867 && actual.samples[0].high_amplitude_q15 == 0 &&
                actual.samples[1].low_amplitude_q15 == 0 && actual.samples[1].high_amplitude_q15 > 0 &&
                actual.samples[2].low_amplitude_q15 > 0 && actual.samples[2].high_amplitude_q15 == 0,
            "quantization collapsed band transitions or introduced sound into a zero band");

    SwitchHapticsDecoder host;
    // Unsafe host command substitute240 briefly peaks then stops; checking only
    // final endpoint would mistakenly authorize the unsafe raw packet.
    constexpr uint8_t unsafe_steps[8] = {0x21, 0x84, 0x10, 0xc4, 0x00, 0x01, 0x40, 0x40};
    const auto unsafe = encoder.encode(host.decode(unsafe_steps), false, true);
    require(!unsafe.raw && unsafe.quantized, "unsafe intermediate raw amplitude passed through");
    play(device, unsafe);
    constexpr uint8_t reserved[8] = {0x01, 0x00, 0x00, 0x40, 0x00, 0x01, 0x40, 0x40};
    require(!encoder.encode(host.decode(reserved), false, true).raw,
            "reserved discriminator was passed through as a qualified native form");
}
}  // namespace

int main() {
    test_public_absolute_goldens();
    test_provenance_and_profile_gains();
    test_compressed_goldens_and_resynchronization();
    test_absolute_plus_commands_and_selected_coordinate();
    test_relative_only_amplitude_survives_prefixes();
    test_mono_bands_ties_and_temporal_policy();
    test_unrepresentable_timeline_and_safety();
    return 0;
}
