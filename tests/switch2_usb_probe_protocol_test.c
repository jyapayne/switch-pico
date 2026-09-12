#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "protocol.h"
#include "descriptors.h"
#include "memory.h"

static const uint32_t common_button_bits[2][16] = {
    {
        0x000004, 0x000008, 0x000001, 0x000002, 0x000040, 0x000080, 0x000200, 0x000400,
        0x001000, 0, 0, 0, 0x004000, 0, 0x000010, 0x000020,
    },
    {
        0x010000, 0x040000, 0x080000, 0x020000, 0x400000, 0x800000, 0x000100, 0x000800,
        0x002000, 0, 0, 0, 0, 0, 0x100000, 0x200000,
    },
};

static void initialize(probe_protocol_state* state) {
    static const uint8_t command[] = {
        0x03, 0x91, 0, 0x0d, 0, 8, 0, 0, 1, 0, 1, 2, 3, 4, 5, 6,
    };
    uint8_t reply[12];
    assert(probe_protocol_command(state, command, sizeof(command), reply, sizeof(reply), NULL) == 12);
}

static void set_features(probe_protocol_state* state, uint8_t subcommand, uint8_t flags) {
    const uint8_t command[] = {0x0c, 0x91, 0, subcommand, 0, 4, 0, 0, flags, 0, 0, 0};
    uint8_t reply[12];
    assert(probe_protocol_command(state, command, sizeof(command), reply, sizeof(reply), NULL) == 12);
}

static void select_report(probe_protocol_state* state, uint8_t report_id) {
    const uint8_t command[] = {0x03, 0x91, 0, 0x0a, 0, 4, 0, 0, report_id, 0, 0, 0};
    uint8_t reply[8];
    assert(probe_protocol_command(state, command, sizeof(command), reply, sizeof(reply), NULL) == 8);
}

static void test_descriptors(void) {
    const uint16_t product_id = probe_device_descriptor[10] |
        ((uint16_t)probe_device_descriptor[11] << 8);
    assert(product_id == (SWITCH2_PROBE_JOYCON_LEFT ? 0x2067 : 0x2066));
    assert(probe_configuration_descriptor[2] == sizeof(probe_configuration_descriptor));
    assert(probe_configuration_descriptor[4] == 2 * PROBE_CONTROLLER_COUNT);
    unsigned interface_count = 0, endpoint_count = 0;
    unsigned interface = 0, seen_endpoints = 0;
    for (size_t offset = 9; offset < sizeof(probe_configuration_descriptor);) {
        const uint8_t* descriptor = probe_configuration_descriptor + offset;
        assert(descriptor[0] >= 2);
        assert(offset + descriptor[0] <= sizeof(probe_configuration_descriptor));
        if (descriptor[1] == 4) {
            assert(descriptor[0] == 9);
            interface = descriptor[2];
            assert(interface == interface_count++);
            assert(descriptor[4] == 2);
            assert(descriptor[5] == (interface % 2 ? 0xff : 3));
            assert(descriptor[8] == 5 + interface);
        } else if (descriptor[1] == 5) {
            assert(descriptor[0] == 7);
            const unsigned endpoint = descriptor[2] & 0x0f;
            assert(endpoint == interface + 1);
            const unsigned bit = endpoint + ((descriptor[2] & 0x80) ? 8 : 0);
            assert(!(seen_endpoints & (1u << bit)));
            seen_endpoints |= 1u << bit;
            assert(descriptor[3] == (interface % 2 ? 2 : 3));
            ++endpoint_count;
        }
        offset += descriptor[0];
    }
    assert(interface_count == 2 * PROBE_CONTROLLER_COUNT);
    assert(endpoint_count == 4 * PROBE_CONTROLLER_COUNT);
    // Read HID short items as a host would: each function advertises only its
    // own native report plus common 05, with sizes matching report generation.
    for (uint8_t instance = 0; instance < PROBE_CONTROLLER_COUNT; ++instance) {
        const uint8_t* descriptor = probe_hid_report_descriptors[instance];
        unsigned input_bits[256] = {0}, output_bits[256] = {0};
        unsigned report_id = 0, report_size = 0, report_count = 0;
        for (size_t offset = 0; offset < sizeof(probe_hid_report_descriptors[instance]);) {
            const uint8_t prefix = descriptor[offset++];
            assert(prefix != 0xfe);
            const unsigned size = (prefix & 3) == 3 ? 4 : prefix & 3;
            assert(offset + size <= sizeof(probe_hid_report_descriptors[instance]));
            uint32_t value = 0;
            for (unsigned i = 0; i < size; ++i)
                value |= (uint32_t)descriptor[offset++] << (8 * i);
            switch (prefix & 0xfc) {
                case 0x74: report_size = value; break;
                case 0x94: report_count = value; break;
                case 0x84: report_id = value; assert(report_id < 256); break;
                case 0x80: input_bits[report_id] += report_size * report_count; break;
                case 0x90: output_bits[report_id] += report_size * report_count; break;
            }
        }
        const bool is_left = SWITCH2_PROBE_COMPOSITE ? instance == 1 : SWITCH2_PROBE_JOYCON_LEFT;
        probe_protocol_state state;
        probe_protocol_reset(&state, is_left);
        initialize(&state);
        uint8_t report[PROBE_INPUT_SIZE];
        for (unsigned id = 0; id < 256; ++id) {
            const size_t expected = (id == 5 || id == (is_left ? 7u : 8u)) ? sizeof(report) : 0;
            assert(input_bits[id] == expected * 8u);
            assert(probe_protocol_report(&state, (uint8_t)id, report, sizeof(report)) == expected);
            assert(output_bits[id] == (id == 1 ? 63u * 8u : 0));
        }
    }
}

static void test_report_selection_and_reset(bool is_left) {
    const uint8_t native_id = is_left ? 7 : 8, opposite_id = is_left ? 8 : 7;
    probe_protocol_state state;
    probe_protocol_reset(&state, is_left);
    uint8_t report[PROBE_INPUT_SIZE];
    assert(probe_protocol_report(&state, native_id, report, sizeof(report)) == 0);
    initialize(&state);
    assert(state.report_id == native_id);
    assert(probe_protocol_report(&state, state.report_id, report, sizeof(report)) == sizeof(report));
    select_report(&state, 5);
    assert(state.report_id == 5);
    select_report(&state, opposite_id);
    assert(state.report_id == 5); // Unsupported IDs are ACKed but ignored.
    memset(report, 0xa5, sizeof(report));
    assert(probe_protocol_report(&state, opposite_id, report, sizeof(report)) == 0);
    for (size_t i = 0; i < sizeof(report); ++i) assert(report[i] == 0xa5);
    select_report(&state, native_id);
    assert(state.report_id == native_id);
    assert(probe_protocol_report(&state, native_id, report, sizeof(report) - 1) == 0);
    state.report_counter = 0x12345678;
    initialize(&state); // Repeated USB initialization must not rewind a live stream.
    assert(probe_protocol_report(&state, native_id, report, sizeof(report)) == sizeof(report));
    assert(report[0] == 0x78);
    select_report(&state, 5);
    probe_protocol_reset(&state, is_left);
    assert(probe_protocol_report(&state, native_id, report, sizeof(report)) == 0);
    initialize(&state);
    assert(state.report_id == native_id);
    assert(probe_protocol_report(&state, state.report_id, report, sizeof(report)) == sizeof(report));
    assert(report[0] == 0);
}

static void test_buttons_stick_and_feature_control(bool is_left) {
    const uint8_t native_id = is_left ? 7 : 8;
    const unsigned common_stick_offset = is_left ? 10 : 13;
    const unsigned absent_stick_offset = is_left ? 13 : 10;
    const unsigned common_rail_offset = is_left ? 6 : 4;
    probe_protocol_state state;
    probe_protocol_reset(&state, is_left);
    initialize(&state);
    set_features(&state, 2, 3);
    set_features(&state, 4, 3);
    state.controller_active = true;
    const uint8_t stick[] = {0x23, 0x61, 0x45};
    const uint8_t calibrated_center[] = {0xff, 0x47, 0x81};
    const uint8_t neutral[] = {0, 8, 0x80};
    memcpy(state.controller_stick, stick, sizeof(stick));
    memcpy(state.stick_center, calibrated_center, sizeof(calibrated_center));
    uint8_t native[PROBE_INPUT_SIZE], common[PROBE_INPUT_SIZE];
    for (unsigned bit = 0; bit < 16; ++bit) {
        memset(state.controller_buttons, 0, sizeof(state.controller_buttons));
        state.controller_buttons[bit / 8] = (uint8_t)(1u << (bit % 8));
        assert(probe_protocol_report(&state, native_id, native, sizeof(native)) == sizeof(native));
        assert(probe_protocol_report(&state, 5, common, sizeof(common)) == sizeof(common));
        const uint16_t expected_native = common_button_bits[is_left][bit] ? (uint16_t)(1u << bit) : 0;
        assert((uint16_t)(native[2] | ((uint16_t)native[3] << 8)) == expected_native);
        for (unsigned byte = 0; byte < 4; ++byte)
            assert(common[4 + byte] == (uint8_t)(common_button_bits[is_left][bit] >> (8 * byte)));
        assert(memcmp(native + 5, stick, sizeof(stick)) == 0);
        assert(memcmp(common + common_stick_offset, stick, sizeof(stick)) == 0);
        assert(memcmp(common + absent_stick_offset, neutral, sizeof(neutral)) == 0);
    }
    // Host feature disable gates the live controls without losing calibration.
    set_features(&state, 5, 3);
    assert(probe_protocol_report(&state, native_id, native, sizeof(native)) == sizeof(native));
    assert(native[2] == 0 && native[3] == 0);
    assert(memcmp(native + 5, calibrated_center, sizeof(calibrated_center)) == 0);
    assert(probe_protocol_report(&state, 5, common, sizeof(common)) == sizeof(common));
    assert(common[4] == 0 && common[5] == 0 && common[6] == 0 && common[7] == 0);
    assert(memcmp(common + common_stick_offset, calibrated_center, sizeof(calibrated_center)) == 0);
    set_features(&state, 4, 3);
    state.controller_active = false;
    assert(probe_protocol_report(&state, native_id, native, sizeof(native)) == sizeof(native));
    assert(native[2] == 0 && native[3] == 0);
    assert(memcmp(native + 5, calibrated_center, sizeof(calibrated_center)) == 0);
    state.test_rail_buttons = true;
    assert(probe_protocol_report(&state, native_id, native, sizeof(native)) == sizeof(native));
    assert(native[2] == 0 && native[3] == 0xc0);
    assert(probe_protocol_report(&state, 5, common, sizeof(common)) == sizeof(common));
    assert(common[common_rail_offset] == 0x30);
    set_features(&state, 5, 1);
    assert(probe_protocol_report(&state, native_id, native, sizeof(native)) == sizeof(native));
    assert(native[3] == 0);
}

static void test_opaque_native_feature_gates(bool is_left) {
    const unsigned imu_length_offset = is_left ? 14 : 15;
#ifdef SWITCH2_PROBE_ZERO_NATIVE_IMU_PAYLOAD
    const unsigned imu_data_offset = imu_length_offset + 1;
#endif
    probe_protocol_state state;
    probe_protocol_reset(&state, is_left);
    set_features(&state, 2, 0x17);
    set_features(&state, 4, 0x17);
    uint8_t source[PROBE_INPUT_SIZE], actual[PROBE_INPUT_SIZE], expected[PROBE_INPUT_SIZE];
    for (size_t i = 0; i < sizeof(source); ++i) source[i] = (uint8_t)(i * 3 + 1);
    source[imu_length_offset] = 30;
    memcpy(expected, source, sizeof(expected));
#ifdef SWITCH2_PROBE_OMIT_NATIVE_IMU
    memset(expected + imu_length_offset, 0, 41);
#elif defined(SWITCH2_PROBE_ZERO_NATIVE_IMU_PAYLOAD)
    memset(expected + imu_data_offset, 0, 40);
#endif
    memcpy(actual, source, sizeof(actual));
    probe_protocol_gate_native_report(&state, actual);
    assert(memcmp(actual, expected, sizeof(actual)) == 0);
    // IMU disable must leave mouse, NFC (R), and reserved tail bytes untouched.
    set_features(&state, 5, 4);
    memcpy(actual, source, sizeof(actual));
    memset(expected + imu_length_offset, 0, 41);
    probe_protocol_gate_native_report(&state, actual);
    assert(memcmp(actual, expected, sizeof(actual)) == 0);
    set_features(&state, 4, 4);
    set_features(&state, 5, 0x13);
    memcpy(actual, source, sizeof(actual));
    memcpy(expected, source, sizeof(expected));
    memset(expected + 2, 0, 2);
    memcpy(expected + 5, state.stick_center, sizeof(state.stick_center));
    memset(expected + 9, 0, 5);
#ifdef SWITCH2_PROBE_OMIT_NATIVE_IMU
    memset(expected + imu_length_offset, 0, 41);
#elif defined(SWITCH2_PROBE_ZERO_NATIVE_IMU_PAYLOAD)
    memset(expected + imu_data_offset, 0, 40);
#endif
    probe_protocol_gate_native_report(&state, actual);
    assert(memcmp(actual, expected, sizeof(actual)) == 0);
}

static const uint8_t sample_command[] = {
    0x0a, 0x91, 0, 0x02, 0, 4, 0, 0, 3, 0, 0, 0,
};
typedef struct {
    unsigned source_calls;
    uint8_t expected_sample;
    bool source_available;
    uint64_t source_token;
    bool storage_available;
    unsigned saves;
    uint8_t pairing_blob[PROBE_PAIRING_BLOB_SIZE];
} controller_context;

static bool play_sample(void* context, uint8_t sample_id, uint64_t* token) {
    controller_context* controller = context;
    ++controller->source_calls;
    assert(sample_id == controller->expected_sample);
    *token = controller->source_token;
    return controller->source_available;
}

static bool save_pairing(void* context, const uint8_t* blob, size_t size) {
    controller_context* controller = context;
    assert(size == sizeof(controller->pairing_blob));
    if (!controller->storage_available) return false;
    memcpy(controller->pairing_blob, blob, size);
    ++controller->saves;
    return true;
}

static void expect_no_dispatch(probe_protocol_state* state, const uint8_t* command,
                               size_t length, size_t capacity) {
    uint8_t reply[8];
    uint64_t token = UINT64_MAX;
    const controller_context* controller = state->context;
    const unsigned calls_before = controller->source_calls;
    assert(probe_protocol_command(state, command, length, reply, capacity, &token) == 0);
    assert(token == 0);
    assert(controller->source_calls == calls_before);
}

static void test_sample_dispatch(void) {
    controller_context controller = {
        .expected_sample = 3, .source_available = true,
        .source_token = UINT64_C(0x1234567800000001),
    };
    probe_protocol_state state;
    probe_protocol_reset(&state, false);
    state.context = &controller;
    state.play_sample = play_sample;

    // Each transport/header field and reserved payload byte is a dispatch gate.
    const struct { uint8_t offset; uint8_t value; } invalid[] = {
        {0, 0x18}, {1, 0x01}, {2, 0x01}, {3, 0x01}, {4, 1}, {5, 3},
        {6, 1}, {7, 1}, {8, 8}, {9, 1}, {10, 1}, {11, 1},
    };
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        uint8_t command[sizeof(sample_command)];
        memcpy(command, sample_command, sizeof(command));
        command[invalid[i].offset] = invalid[i].value;
        expect_no_dispatch(&state, command, sizeof(command), 8);
    }
    expect_no_dispatch(&state, sample_command, 7, 8);
    uint8_t resized[13] = {0};
    memcpy(resized, sample_command, sizeof(sample_command));
    resized[5] = 3;
    expect_no_dispatch(&state, resized, 11, 8);
    resized[5] = 5;
    expect_no_dispatch(&state, resized, sizeof(resized), 8);
    expect_no_dispatch(&state, sample_command, sizeof(sample_command), 7);

    // Synchronous-only callers cannot accidentally acknowledge a sample.
    uint8_t reply[8];
    const unsigned calls_before = controller.source_calls;
    assert(probe_protocol_command(&state, sample_command, sizeof(sample_command),
                                  reply, sizeof(reply), NULL) == 0);
    assert(controller.source_calls == calls_before);
    state.play_sample = NULL;
    expect_no_dispatch(&state, sample_command, sizeof(sample_command), sizeof(reply));
    state.play_sample = play_sample;

    // A rejected request must not leak even a token written by the source.
    uint64_t token = UINT64_MAX;
    controller.source_available = false;
    assert(probe_protocol_command(&state, sample_command, sizeof(sample_command),
                                  reply, sizeof(reply), &token) == 0);
    assert(token == 0);
    controller.source_available = true;
    controller.source_token = 0;
    token = UINT64_MAX;
    assert(probe_protocol_command(&state, sample_command, sizeof(sample_command),
                                  reply, sizeof(reply), &token) == 0);
    assert(token == 0);

    // The observed sample and range boundaries only produce deferred replies.
    const uint8_t samples[] = {3, 0, 7};
    const uint8_t sample_ack[] = {0x0a, 0x01, 0, 0x02, 0, 0xf8, 0, 0};
    controller.source_token = UINT64_C(0x1234567800000001);
    for (size_t i = 0; i < sizeof(samples); ++i) {
        uint8_t command[sizeof(sample_command)];
        memcpy(command, sample_command, sizeof(command));
        command[8] = controller.expected_sample = samples[i];
        token = 0;
        assert(probe_protocol_command(&state, command, sizeof(command), reply,
                                      sizeof(reply), &token) == sizeof(sample_ack));
        assert(token == controller.source_token);
        assert(memcmp(reply, sample_ack, sizeof(sample_ack)) == 0);
        ++controller.source_token;
    }

    // Ordinary report selection retains its immediate, empty USB ACK.
    const uint8_t select_report[] = {0x03, 0x91, 0, 0x0a, 0, 4, 0, 0, 5, 0, 0, 0};
    const uint8_t select_ack[] = {0x03, 0x01, 0, 0x0a, 0, 0xf8, 0, 0};
    assert(probe_protocol_command(&state, select_report, sizeof(select_report),
                                  reply, sizeof(reply), &token) == sizeof(select_ack));
    assert(token == 0);
    assert(memcmp(reply, select_ack, sizeof(select_ack)) == 0);
}

static void test_interleaved_reports_and_features(void) {
    probe_protocol_state right, left;
    probe_protocol_reset(&right, false);
    probe_protocol_reset(&left, true);
    uint8_t reports[2][PROBE_INPUT_SIZE];
    initialize(&right);
    assert(probe_protocol_report(&right, 8, reports[0], sizeof(reports[0])) == PROBE_INPUT_SIZE);
    assert(probe_protocol_report(&left, 7, reports[1], sizeof(reports[1])) == 0);
    initialize(&left);
    select_report(&right, 5);
    select_report(&left, 8);
    select_report(&right, 7);
    assert(right.report_id == 5 && left.report_id == 7);
    set_features(&right, 2, 0x17);
    set_features(&right, 4, 0x17);
    set_features(&left, 2, 0x03);
    set_features(&left, 4, 0x17);
    right.controller_active = left.controller_active = true;
    right.controller_buttons[0] = 0x84; // A + Plus.
    left.controller_buttons[0] = 0x41; // Down + Minus.
    const uint8_t sticks[2][3] = {{0x11, 0x22, 0x33}, {0x44, 0x55, 0x66}};
    const uint8_t neutral[] = {0, 8, 0x80};
    memcpy(right.controller_stick, sticks[0], 3);
    memcpy(left.controller_stick, sticks[1], 3);
    assert(probe_protocol_report(&right, right.report_id, reports[0], sizeof(reports[0])) == PROBE_INPUT_SIZE);
    assert(probe_protocol_report(&left, left.report_id, reports[1], sizeof(reports[1])) == PROBE_INPUT_SIZE);
    assert(reports[0][4] == 1 && reports[0][5] == 4 && reports[0][6] == 0);
    assert(memcmp(reports[0] + 10, neutral, 3) == 0);
    assert(memcmp(reports[0] + 13, sticks[0], 3) == 0);
    assert(reports[1][2] == 0x41 && reports[1][3] == 0);
    assert(memcmp(reports[1] + 5, sticks[1], 3) == 0);
    select_report(&left, 5);
    assert(probe_protocol_report(&left, left.report_id, reports[1], sizeof(reports[1])) == PROBE_INPUT_SIZE);
    assert(reports[1][4] == 0 && reports[1][5] == 1 && reports[1][6] == 1);
    assert(memcmp(reports[1] + 10, sticks[1], 3) == 0);
    assert(memcmp(reports[1] + 13, neutral, 3) == 0);

    uint8_t source[PROBE_INPUT_SIZE], expected[2][PROBE_INPUT_SIZE];
    for (size_t i = 0; i < sizeof(source); ++i) source[i] = (uint8_t)(i * 3 + 1);
    memcpy(expected[0], source, sizeof(source));
#ifdef SWITCH2_PROBE_OMIT_NATIVE_IMU
    memset(expected[0] + 15, 0, 41);
#elif defined(SWITCH2_PROBE_ZERO_NATIVE_IMU_PAYLOAD)
    memset(expected[0] + 16, 0, 40);
#endif
    memcpy(expected[1], source, sizeof(source));
    memset(expected[1] + 9, 0, 5);
    memset(expected[1] + 14, 0, 41);
    memcpy(reports[0], source, sizeof(source));
    memcpy(reports[1], source, sizeof(source));
    probe_protocol_gate_native_report(&left, reports[1]);
    probe_protocol_gate_native_report(&right, reports[0]);
    assert(memcmp(reports, expected, sizeof(reports)) == 0);

    // Reverse the negotiated gates without changing either donor's opaque bytes.
    set_features(&right, 5, 0x15);
    set_features(&left, 2, 0x17);
    set_features(&left, 4, 0x17);
    memcpy(expected[0], source, sizeof(source));
    memset(expected[0] + 2, 0, 2);
    memset(expected[0] + 9, 0, 5);
    memset(expected[0] + 15, 0, 41);
    memcpy(expected[1], source, sizeof(source));
#ifdef SWITCH2_PROBE_OMIT_NATIVE_IMU
    memset(expected[1] + 14, 0, 41);
#elif defined(SWITCH2_PROBE_ZERO_NATIVE_IMU_PAYLOAD)
    memset(expected[1] + 15, 0, 40);
#endif
    memcpy(reports[0], source, sizeof(source));
    memcpy(reports[1], source, sizeof(source));
    probe_protocol_gate_native_report(&right, reports[0]);
    probe_protocol_gate_native_report(&left, reports[1]);
    assert(memcmp(reports, expected, sizeof(reports)) == 0);
    probe_protocol_reset(&right, false);
    assert(probe_protocol_report(&right, 8, reports[0], sizeof(reports[0])) == 0);
    assert(probe_protocol_report(&left, 5, reports[1], sizeof(reports[1])) == PROBE_INPUT_SIZE);
    assert(reports[1][5] == 1 && reports[1][6] == 1);
    memcpy(reports[1], source, sizeof(source));
    probe_protocol_gate_native_report(&left, reports[1]);
    assert(memcmp(reports[1], expected[1], sizeof(reports[1])) == 0);
}

static void test_interleaved_callbacks_and_pairing(void) {
    const uint8_t addresses[2][6] = {
        {0x64, 0xf9, 0xd8, 0x93, 0x05, 0xa2},
        {0x65, 0xf9, 0xd8, 0x93, 0x05, 0xa2},
    };
    controller_context controllers[2] = {
        {.expected_sample = 3, .source_available = true,
         .source_token = UINT64_C(0x100000001), .storage_available = true},
        {.expected_sample = 3, .source_available = false,
         .source_token = UINT64_C(0x200000001), .storage_available = false},
    };
    probe_protocol_state states[2];
    for (unsigned side = 0; side < 2; ++side) {
        probe_protocol_reset(&states[side], side != 0);
        states[side].context = &controllers[side];
        states[side].play_sample = play_sample;
        states[side].save_pairing = save_pairing;
        memcpy(states[side].controller_address, addresses[side], 6);
    }
    uint8_t reply[PROBE_REPLY_MAX_SIZE];
    uint8_t cue_replies[2][8];
    uint64_t tokens[2] = {0, UINT64_MAX};
    assert(probe_protocol_command(&states[0], sample_command, sizeof(sample_command),
                                  cue_replies[0], 8, &tokens[0]) == 8);
    assert(probe_protocol_command(&states[1], sample_command, sizeof(sample_command),
                                  cue_replies[1], 8, &tokens[1]) == 0);
    assert(tokens[0] == UINT64_C(0x100000001) && tokens[1] == 0);
    controllers[1].source_available = true;
    assert(probe_protocol_command(&states[1], sample_command, sizeof(sample_command),
                                  cue_replies[1], 8, &tokens[1]) == 8);
    assert(tokens[0] == UINT64_C(0x100000001) && tokens[1] == UINT64_C(0x200000001));
    const uint8_t cue_ack[] = {0x0a, 1, 0, 2, 0, 0xf8, 0, 0};
    assert(memcmp(cue_replies[0], cue_ack, 8) == 0);
    assert(memcmp(cue_replies[1], cue_ack, 8) == 0);

    const uint8_t hosts[2][16] = {
        {0x15, 0x91, 0, 1, 0, 8, 0, 0, 0, 1, 1, 2, 3, 4, 5, 6},
        {0x15, 0x91, 0, 1, 0, 8, 0, 0, 0, 1, 7, 8, 9, 10, 11, 12},
    };
    const uint8_t device_component[] = {
        0x5c, 0xf6, 0xee, 0x79, 0x2c, 0xdf, 0x05, 0xe1,
        0xba, 0x2b, 0x63, 0x25, 0xc4, 0x1a, 0x5f, 0x10,
    };
    const uint8_t ciphertexts[2][16] = {
        {0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30,
         0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a},
        {0x66, 0xe9, 0x4b, 0xd4, 0xef, 0x8a, 0x2c, 0x3b,
         0x88, 0x4c, 0xfa, 0x59, 0xca, 0x34, 0x2b, 0x2e},
    };
    uint8_t challenges[2][25] = {
        {0x15, 0x91, 0, 2, 0, 17, 0, 0, 0},
        {0x15, 0x91, 0, 2, 0, 17, 0, 0, 0},
    };
    const uint8_t finalize[] = {0x15, 0x91, 0, 3, 0, 1, 0, 0, 0};
    for (unsigned side = 0; side < 2; ++side) {
        assert(probe_protocol_command(&states[side], hosts[side], sizeof(hosts[side]),
                                      reply, sizeof(reply), NULL) == 17);
        assert(memcmp(reply + 11, addresses[side], 6) == 0);
    }
    for (unsigned side = 0; side < 2; ++side) {
        uint8_t key[] = {0x15, 0x91, 0, 4, 0, 17, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        for (unsigned i = 0; i < 16; ++i) {
            // R uses AES's 000102...0f / 001122...ff vector; L uses all zeros.
            key[9 + i] = device_component[i] ^ (side ? 0 : 15u - i);
            challenges[side][9 + i] = side ? 0 : (uint8_t)((15u - i) * 0x11u);
        }
        assert(probe_protocol_command(&states[side], key, sizeof(key),
                                      reply, sizeof(reply), NULL) == 25);
    }
    assert(probe_protocol_command(&states[0], challenges[0], sizeof(challenges[0]),
                                  reply, sizeof(reply), NULL) == 25);
    assert(memcmp(reply + 9, ciphertexts[0], 16) == 0);
    // Right confirmation cannot authorize the left's finalize.
    assert(probe_protocol_command(&states[1], finalize, sizeof(finalize),
                                  reply, sizeof(reply), NULL) == 0);
    assert(controllers[0].saves == 0 && controllers[1].saves == 0);
    assert(probe_protocol_command(&states[1], challenges[1], sizeof(challenges[1]),
                                  reply, sizeof(reply), NULL) == 25);
    assert(memcmp(reply + 9, ciphertexts[1], 16) == 0);
    assert(probe_protocol_command(&states[0], finalize, sizeof(finalize),
                                  reply, sizeof(reply), NULL) == 9);
    assert(reply[8] == 1);
    assert(probe_protocol_command(&states[1], finalize, sizeof(finalize),
                                  reply, sizeof(reply), NULL) == 0);
    assert(controllers[0].saves == 1 && controllers[1].saves == 0);
    controllers[1].storage_available = true;
    assert(probe_protocol_command(&states[1], finalize, sizeof(finalize),
                                  reply, sizeof(reply), NULL) == 9);
    assert(reply[8] == 1);
    assert(controllers[0].saves == 1 && controllers[1].saves == 1);

    // After independent resets, each durable record must resume only its own
    // challenge association; swapping the two contexts' records is rejected.
    for (unsigned side = 0; side < 2; ++side) {
        probe_protocol_reset(&states[side], side != 0);
        memcpy(states[side].controller_address, addresses[side], 6);
        assert(!probe_protocol_restore_pairing(&states[side], controllers[1 - side].pairing_blob,
                                               PROBE_PAIRING_BLOB_SIZE));
        assert(probe_protocol_restore_pairing(&states[side], controllers[side].pairing_blob,
                                              PROBE_PAIRING_BLOB_SIZE));
    }
    for (unsigned side = 0; side < 2; ++side) {
        assert(probe_protocol_command(&states[side], hosts[1 - side], sizeof(hosts[0]),
                                      reply, sizeof(reply), NULL) == 17);
        assert(probe_protocol_command(&states[side], challenges[side], sizeof(challenges[side]),
                                      reply, sizeof(reply), NULL) == 0);
        assert(probe_protocol_command(&states[side], hosts[side], sizeof(hosts[side]),
                                      reply, sizeof(reply), NULL) == 17);
        assert(probe_protocol_command(&states[side], challenges[side], sizeof(challenges[side]),
                                      reply, sizeof(reply), NULL) == 25);
        assert(memcmp(reply + 9, ciphertexts[side], 16) == 0);
    }
}

static bool read_memory(void* context, uint32_t address, uint8_t* output, size_t length) {
    return probe_memory_read(*(const uint8_t*)context, address, output, length);
}

static void test_indexed_memory(void) {
    probe_protocol_state states[PROBE_CONTROLLER_COUNT];
    uint8_t instances[PROBE_CONTROLLER_COUNT];
    for (uint8_t instance = 0; instance < PROBE_CONTROLLER_COUNT; ++instance) {
        instances[instance] = instance;
        probe_protocol_reset(&states[instance], probe_model_is_left(instance));
        states[instance].context = &instances[instance];
        states[instance].read_memory = read_memory;
    }
    const uint8_t calibrations[2][9] = {
        {0x10, 0x08, 0x81, 0, 3, 0x30, 0, 4, 0x40}, // Valid user override.
        {0, 0x09, 0x90, 0, 3, 0x30, 0, 4, 0x40}, // Invalid user, factory fallback.
    };
    const uint8_t command[] = {
        0x02, 0x91, 0, 4, 0, 8, 0, 0, 9, 0x7e, 0, 0, 0xa8, 0x30, 1, 0,
    };
    for (unsigned remaining = PROBE_CONTROLLER_COUNT; remaining; --remaining) {
        const uint8_t instance = (uint8_t)(remaining - 1);
        const bool is_left = SWITCH2_PROBE_COMPOSITE ? instance == 1 : SWITCH2_PROBE_JOYCON_LEFT;
        uint8_t reply[PROBE_REPLY_MAX_SIZE], calibration[9];
        assert(probe_memory_stick_calibration(instance, calibration));
        assert(memcmp(calibration, calibrations[is_left], sizeof(calibration)) == 0);
        assert(probe_protocol_command(&states[instance], command, sizeof(command),
                                      reply, sizeof(reply), NULL) == 25);
        const uint8_t factory[] = {0, is_left ? 9 : 8, is_left ? 0x90 : 0x80, 0, 3, 0x30, 0, 4, 0x40};
        assert(memcmp(reply + 16, factory, sizeof(factory)) == 0);
        const uint32_t ends[] = {0x14fff, 0x1fcfff};
        for (unsigned region = 0; region < 2; ++region) {
            uint8_t output[2] = {0xa5, 0xa5};
            assert(!probe_memory_read(instance, ends[region], output, sizeof(output)));
            assert(output[0] == 0xa5 && output[1] == 0xa5);
            assert(probe_memory_read(instance, ends[region], output, 1));
            assert(output[0] == (uint8_t)((region ? 0xf1 : 0xe1) + is_left));
            assert(output[1] == 0xa5);
        }
    }
    uint8_t output[9];
    memset(output, 0xa5, sizeof(output));
    const uint8_t invalid[] = {PROBE_CONTROLLER_COUNT, UINT8_MAX};
    for (size_t i = 0; i < sizeof(invalid); ++i) {
        assert(!probe_memory_read(invalid[i], 0x130a8, output, sizeof(output)));
        assert(!probe_memory_stick_calibration(invalid[i], output));
        for (size_t byte = 0; byte < sizeof(output); ++byte) assert(output[byte] == 0xa5);
    }
}

int main(void) {
    test_indexed_memory();
    test_descriptors();
    for (unsigned side = 0; side < 2; ++side) {
        test_report_selection_and_reset(side != 0);
        test_buttons_stick_and_feature_control(side != 0);
        test_opaque_native_feature_gates(side != 0);
    }
    test_sample_dispatch();
    test_interleaved_reports_and_features();
    test_interleaved_callbacks_and_pairing();
    return 0;
}
