#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "protocol.h"

static const uint8_t sample_command[] = {
    0x0a, 0x91, 0, 0x02, 0, 4, 0, 0, 3, 0, 0, 0,
};
static unsigned source_calls;
static uint8_t expected_sample = 3;
static bool source_available = true;
static uint64_t source_token = UINT64_C(0x1234567800000001);

static bool play_sample(uint8_t sample_id, uint64_t* token) {
    ++source_calls;
    assert(sample_id == expected_sample);
    *token = source_token;
    return source_available;
}

static void expect_no_dispatch(probe_protocol_state* state, const uint8_t* command,
                               size_t length, size_t capacity) {
    uint8_t reply[8];
    uint64_t token = UINT64_MAX;
    const unsigned calls_before = source_calls;
    assert(probe_protocol_command(state, command, length, reply, capacity, &token) == 0);
    assert(token == 0);
    assert(source_calls == calls_before);
}

int main(void) {
    probe_protocol_state state;
    probe_protocol_reset(&state);
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
    const unsigned calls_before = source_calls;
    assert(probe_protocol_command(&state, sample_command, sizeof(sample_command),
                                  reply, sizeof(reply), NULL) == 0);
    assert(source_calls == calls_before);
    state.play_sample = NULL;
    expect_no_dispatch(&state, sample_command, sizeof(sample_command), sizeof(reply));
    state.play_sample = play_sample;

    // A rejected request must not leak even a token written by the source.
    uint64_t token = UINT64_MAX;
    source_available = false;
    assert(probe_protocol_command(&state, sample_command, sizeof(sample_command),
                                  reply, sizeof(reply), &token) == 0);
    assert(token == 0);
    source_available = true;
    source_token = 0;
    token = UINT64_MAX;
    assert(probe_protocol_command(&state, sample_command, sizeof(sample_command),
                                  reply, sizeof(reply), &token) == 0);
    assert(token == 0);

    // The observed sample and range boundaries only produce deferred replies.
    const uint8_t samples[] = {3, 0, 7};
    const uint8_t sample_ack[] = {0x0a, 0x01, 0, 0x02, 0, 0xf8, 0, 0};
    source_token = UINT64_C(0x1234567800000001);
    for (size_t i = 0; i < sizeof(samples); ++i) {
        uint8_t command[sizeof(sample_command)];
        memcpy(command, sample_command, sizeof(command));
        command[8] = expected_sample = samples[i];
        token = 0;
        assert(probe_protocol_command(&state, command, sizeof(command), reply,
                                      sizeof(reply), &token) == sizeof(sample_ack));
        assert(token == source_token);
        assert(memcmp(reply, sample_ack, sizeof(sample_ack)) == 0);
        ++source_token;
    }

    // Ordinary report selection retains its immediate, empty USB ACK.
    const uint8_t select_report[] = {0x03, 0x91, 0, 0x0a, 0, 4, 0, 0, 5, 0, 0, 0};
    const uint8_t select_ack[] = {0x03, 0x01, 0, 0x0a, 0, 0xf8, 0, 0};
    assert(probe_protocol_command(&state, select_report, sizeof(select_report),
                                  reply, sizeof(reply), &token) == sizeof(select_ack));
    assert(token == 0);
    assert(memcmp(reply, select_ack, sizeof(select_ack)) == 0);
    return 0;
}
