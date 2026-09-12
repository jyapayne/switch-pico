from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from prepare_bluepad32 import prepare_bluepad32

# Reuse the existing real-BTstack-header radio/run-loop fixture, but drive the
# capture-enabled discovery path and link the actual cross-core capture mailbox.
SOURCE = r"""
#define main parser_fixture_main
#include "switch2_parser_native_test.c"
#undef main
#include "model.h"

void capture_init(void);
void capture_select(uint8_t instance, const uint8_t address[6], uint16_t product_id);
void capture_native_stream(uint8_t instance, bool enabled);
uint32_t capture_native_peek(uint8_t instance, uint8_t report[63]);
bool capture_native_commit(uint8_t instance, uint32_t serial);
bool capture_request(uint8_t instance, uint8_t id, uint64_t* token);
int capture_result(uint8_t instance, uint64_t token);
void capture_cancel(uint8_t instance);
void capture_exhaust_records(void);

#define SECONDARY_HANDLE 0x144
static const uint8_t secondary_uuids[2][16] = {
    {0xd5,0xa9,0xe0,0x1e,0x2f,0xfc,0x4c,0xca,0xb2,0x0c,0x8b,0x67,0x14,0x2b,0xf4,0x42},
    {0xcc,0x1b,0xbb,0xb5,0x73,0x54,0x4d,0x32,0xa7,0x16,0xa8,0x1c,0xb2,0x41,0xa3,0x2a},
};
#define OTHER_PRODUCT_ID (SWITCH2_PROBE_JOYCON_LEFT ? UNI_SW2_JOYCON_R_PID : UNI_SW2_JOYCON_L_PID)
static const uint8_t sample_ack[8] = {0x0a,1,1,2,0x10,0x78,0,0};

static void native_input(struct fixture_peer* peer) {
    uint8_t input[63] = {0};
    notify(peer, SECONDARY_HANDLE, input, sizeof(input));
}

static struct fixture_peer* connect_sample_source(uint8_t instance,
                                                  const uint8_t address[6],
                                                  hci_con_handle_t handle) {
    const bool left = probe_model_is_left(instance);
    const unsigned previous_ready = ready;
    uint8_t advertisement_data[64];
    size_t advertisement_size = advertisement(advertisement_data, probe_model_pid(instance), false);
    reverse_bytes(address, advertisement_data + 4, 6);
    assert(uni_bt_le_switch2_handle_advertisement(advertisement_data, advertisement_size));
    struct fixture_peer* peer = &peers[instance];
    peer->device.conn.handle = handle;
    peer->link_alive = true;
    uni_hid_parser_switch2_on_le_connected(&peer->device);
    uint8_t service[28] = {GATT_EVENT_SERVICE_QUERY_RESULT};
    little_endian_store_16(service, 8, SERVICE_START);
    little_endian_store_16(service, 10, SERVICE_START + 0x60);
    reverse_128(service_uuid, service + 12);
    event(peer, service, sizeof(service));
    query_done(peer, 0);
    const unsigned write = request_writes ? ATT_PROPERTY_WRITE : ATT_PROPERTY_WRITE_WITHOUT_RESPONSE;
    characteristic(peer, INPUT_HANDLE, input_uuid, ATT_PROPERTY_NOTIFY);
    characteristic(peer, RESPONSE_HANDLE, response_uuid, ATT_PROPERTY_NOTIFY);
    characteristic(peer, COMMAND_HANDLE, command_uuid, write);
    characteristic(peer, RUMBLE_HANDLE, rumble_uuids[left ? 1 : 2], write);
    characteristic(peer, SECONDARY_HANDLE, secondary_uuids[left ? 1 : 0], ATT_PROPERTY_NOTIFY);
    query_done(peer, 0);
    descriptor(peer, RESPONSE_HANDLE + 2);
    query_done(peer, 0);
    descriptor(peer, INPUT_HANDLE + 2);
    query_done(peer, 0);
    descriptor(peer, SECONDARY_HANDLE + 2);
    query_done(peer, 0);
    query_done(peer, 0);
    for (unsigned i = 0; i < 24 && ready == previous_ready && !disconnected; ++i) {
        if (peer->query == QUERY_CCCD) {
            query_done(peer, 0);
        } else if (peer->command[0] == 0x10) {
            uint8_t version[20] = {0x10,1,1,1,0x10,0x78,0,0};
            version[11] = left ? 0 : 1;
            if (peer->query == QUERY_WRITE) query_done(peer, 0);
            notify(peer, RESPONSE_HANDLE, version, sizeof(version));
        } else {
            acknowledge(peer, false);
        }
    }
    assert(ready == previous_ready + 1 && !disconnected);
    // Let the unchanged neutral-rumble budget quiesce before requesting a cue.
    for (unsigned i = 0; i < 6; ++i) {
        advance(13);
        if (peer->query == QUERY_WRITE) query_done(peer, 0);
    }
    native_input(peer);
    return peer;
}
static struct fixture_peer* sample_ready_on_handle(bool requests, uint32_t start,
                                                   hci_con_handle_t handle) {
    reset();
    now_ms = start;
    capture_init();
    request_writes = requests;
    return connect_sample_source(0, controller_address, handle);
}


static struct fixture_peer* sample_ready(bool requests, uint32_t start) {
    return sample_ready_on_handle(requests, start, 0);
}

static uint64_t request_sample(struct fixture_peer* peer, uint8_t id) {
    uint64_t token = 0;
    assert(capture_request(0, id, &token) && token);
    assert(capture_result(0, token) == 0);
    unsigned commands = peer->commands;
    advance(13);
    const uint8_t expected[12] = {0x0a,0x91,1,2,0,4,0,0,id,0,0,0};
    assert(peer->commands == commands + 1 && peer->command_length == sizeof(expected));
    assert(memcmp(peer->command, expected, sizeof(expected)) == 0);
    assert(capture_result(0, token) == 0);
    return token;
}

static void successful_ack(struct fixture_peer* peer, uint64_t token) {
    if (peer->query == QUERY_WRITE) query_done(peer, 0);
    assert(capture_result(0, token) == 0);
    notify(peer, RESPONSE_HANDLE, sample_ack, sizeof(sample_ack));
    assert(capture_result(0, token) == 1);
    assert(capture_result(0, token) == -1);
}

static void test_ack_order_and_source_matching(void) {
    struct fixture_peer* peer = sample_ready(true, 0);
    uint64_t token = request_sample(peer, 3), refused = 99;
    assert(!capture_request(0, 4, &refused) && refused == 0);
    uint8_t malformed[9] = {0x0a,1,1,2,0x10,0x78,0,0,0};
    notify(peer, RESPONSE_HANDLE, malformed, sizeof(malformed));
    malformed[3] = 1;
    notify(peer, RESPONSE_HANDLE, malformed, 8);
    assert(capture_result(0, token) == 0);
    // Deliver a genuine-shaped ACK from a different Bluetooth handle.
    uint8_t wrong_peer[20] = {GATT_EVENT_NOTIFICATION,18};
    little_endian_store_16(wrong_peer, 2, 99);
    little_endian_store_16(wrong_peer, 8, RESPONSE_HANDLE);
    little_endian_store_16(wrong_peer, 10, sizeof(sample_ack));
    memcpy(wrong_peer + 12, sample_ack, sizeof(sample_ack));
    peer->callback(HCI_EVENT_PACKET, 0, wrong_peer, sizeof(wrong_peer));
    assert(capture_result(0, token) == 0);
    notify(peer, RESPONSE_HANDLE, sample_ack, sizeof(sample_ack));
    assert(capture_result(0, token) == 0); // Application ACK cannot beat ATT completion.
    query_done(peer, 0);
    assert(capture_result(0, token) == 1 && capture_result(0, token) == -1);
    for (uint8_t id = 0; id < 8; ++id) {
        uint64_t next = request_sample(peer, id);
        assert(next > token);
        token = next;
        successful_ack(peer, token); // ATT success alone is not application success.
    }
    assert(!capture_request(0, 8, &refused) && !capture_request(0, 3, NULL));
}

static void test_native_notification_bounds_and_fidelity(void) {
    struct fixture_peer* peer = sample_ready(false, 0);
    capture_native_stream(0, true);
    uint8_t input[100], actual[63];
    for (unsigned i = 0; i < sizeof(input); ++i)
        input[i] = (uint8_t)(i * 37 + 11);
    input[PROBE_IMU_LENGTH_OFFSET] = 40;
    memset(actual, 0xa5, sizeof(actual));
    notify(peer, SECONDARY_HANDLE, input, 62);
    notify(peer, SECONDARY_HANDLE, input, 64);
    notify(peer, SECONDARY_HANDLE, input, 100);
    assert(capture_native_peek(0, actual) == 0 && actual[0] == 0xa5);
    notify(peer, SECONDARY_HANDLE, input, 63);
    uint32_t serial = capture_native_peek(0, actual);
    assert(serial && memcmp(actual, input, sizeof(actual)) == 0);
    assert(capture_native_peek(0, actual) == serial); // Failed USB submission retries intact.
    assert(capture_native_commit(0, serial) && !capture_native_commit(0, serial));
    assert(capture_native_peek(0, actual) == 0);
}

static void test_selection_cannot_transfer_pending_ack(void) {
    struct fixture_peer* peer = sample_ready(true, 0);
    uint64_t old = request_sample(peer, 3), next = 0;
    uint8_t other[6];
    memcpy(other, controller_address, sizeof(other));
    ++other[5];
    capture_select(0, other, PROBE_JOYCON_PID);
    assert(capture_result(0, old) == -1);
    native_input(peer);
    assert(!capture_request(0, 4, &next)); // Old address cannot activate new source.
    capture_select(0, controller_address, OTHER_PRODUCT_ID);
    native_input(peer);
    assert(!capture_request(0, 4, &next)); // Same address, wrong side still cannot.
    capture_select(0, controller_address, PROBE_JOYCON_PID);
    native_input(peer);
    assert(capture_request(0, 4, &next) && next > old);
    unsigned commands = peer->commands;
    advance(13);
    assert(peer->commands == commands);
    notify(peer, RESPONSE_HANDLE, sample_ack, sizeof(sample_ack));
    query_done(peer, 0); // Retired transaction drains without owning the new cue.
    assert(capture_result(0, old) == -1 && capture_result(0, next) == 0);
    advance(13);
    assert(peer->commands == commands + 1 && peer->command[8] == 4);
    successful_ack(peer, next);
}

static void test_cancel_does_not_transfer_old_ack(void) {
    struct fixture_peer* peer = sample_ready(true, 0);
    uint64_t old = request_sample(peer, 3), next;
    capture_cancel(0);
    assert(capture_result(0, old) == -1);
    assert(capture_request(0, 4, &next) && next > old);
    unsigned commands = peer->commands;
    advance(13);
    assert(peer->commands == commands); // Old untagged ACK must drain first.
    notify(peer, RESPONSE_HANDLE, sample_ack, sizeof(sample_ack));
    query_done(peer, 0);
    assert(capture_result(0, old) == -1 && capture_result(0, next) == 0);
    advance(13);
    assert(peer->commands == commands + 1 && peer->command[8] == 4);
    successful_ack(peer, next);

    peer = sample_ready(false, 0);
    commands = peer->commands;
    assert(capture_request(0, 3, &old));
    next_write_error = GATT_CLIENT_BUSY;
    advance(13);
    assert(peer->commands == commands);
    capture_cancel(0);
    assert(capture_request(0, 4, &next) && next > old);
    advance(13);
    assert(peer->commands == commands + 1 && peer->command[8] == 4);
    successful_ack(peer, next);
}

static void test_rejection_disconnect_and_late_link_events(void) {
    struct fixture_peer* peer = sample_ready(true, 0);
    uint64_t old = request_sample(peer, 3);
    notify(peer, RESPONSE_HANDLE, sample_ack, sizeof(sample_ack));
    query_done(peer, 0x0e);
    assert(disconnected == 1 && capture_result(0, old) == -1);

    peer = sample_ready(false, 0);
    old = request_sample(peer, 3);
    uint8_t rejected[8];
    memcpy(rejected, sample_ack, sizeof(rejected));
    rejected[5] = 0x81;
    notify(peer, RESPONSE_HANDLE, rejected, sizeof(rejected));
    assert(disconnected == 1 && capture_result(0, old) == -1);

    peer = sample_ready(false, 0);
    old = request_sample(peer, 3);
    btstack_packet_handler_t retired_callback = peer->callback;
    uni_hid_device_disconnect(&peer->device);
    assert(capture_result(0, old) == -1);
    uint64_t next = 0;
    assert(!capture_request(0, 3, &next));
    peer = sample_ready_on_handle(false, 0, 1);
    next = request_sample(peer, 4);
    assert(next > old);
    // The retired link's callback still carries its old, now absent handle.
    uint8_t late[20] = {GATT_EVENT_NOTIFICATION,18};
    little_endian_store_16(late, 2, 0);
    little_endian_store_16(late, 8, RESPONSE_HANDLE);
    little_endian_store_16(late, 10, sizeof(sample_ack));
    memcpy(late + 12, sample_ack, sizeof(sample_ack));
    retired_callback(HCI_EVENT_PACKET, 0, late, sizeof(late));
    assert(capture_result(0, next) == 0 && capture_result(0, old) == -1);
    successful_ack(peer, next);

    old = request_sample(peer, 3);
    notify(peer, RESPONSE_HANDLE, sample_ack, sizeof(sample_ack));
    uni_hid_device_disconnect(&peer->device);
    assert(capture_result(0, old) == -1); // Disconnect revokes even unconsumed success.
}

static void test_freshness_timeout_and_clock_wrap(void) {
    capture_init();
    uint64_t token = 0;
    assert(!capture_request(0, 3, &token));
    uint8_t input[63] = {0};
    uint8_t other[6];
    memcpy(other, controller_address, sizeof(other));
    ++other[5];
    switch_pico_switch2_mouse_report(OTHER_PRODUCT_ID, controller_address,
                                     PROBE_NATIVE_REPORT_ID, input, 63, now_ms);
    switch_pico_switch2_mouse_report(PROBE_JOYCON_PID, other,
                                     PROBE_NATIVE_REPORT_ID, input, 63, now_ms);
    assert(!capture_request(0, 3, &token));

    struct fixture_peer* peer = sample_ready(false, 0);
    token = request_sample(peer, 3);
    advance(500);
    assert(capture_result(0, token) == -1 && !capture_request(0, 3, &token));

    peer = sample_ready(false, UINT32_MAX - 200);
    uint32_t started = now_ms;
    token = request_sample(peer, 3);
    while ((uint32_t)(now_ms - started) < 1900) {
        advance(100);
        native_input(peer);
    }
    advance(1999 - (uint32_t)(now_ms - started));
    native_input(peer);
    assert(capture_result(0, token) == 0);
    advance(1);
    assert(capture_result(0, token) == -1);
    notify(peer, RESPONSE_HANDLE, sample_ack, sizeof(sample_ack));
    assert(capture_result(0, token) == -1);

    peer = sample_ready(false, 0);
    token = request_sample(peer, 3);
    for (unsigned i = 0; i < 21 && !disconnected; ++i) {
        advance(100);
        if (!disconnected) native_input(peer);
    }
    assert(disconnected == 1 && capture_result(0, token) == -1);
}

#if SWITCH2_PROBE_COMPOSITE || SWITCH2_PROBE_HUB
static void test_simultaneous_native_sources_and_real_acks(void) {
    const uint8_t left_address[6] = {0xc0,0x22,0x33,0x44,0x55,0x67};
    struct fixture_peer* right = sample_ready(true, 0);
    struct fixture_peer* left = connect_sample_source(1, left_address, 1);
    assert(ready == 2 && right->link_alive && left->link_alive);
    capture_native_stream(0, true);
    capture_native_stream(1, true);
    uint8_t rinput[63], linput[63], actual[63];
    for (unsigned i = 0; i < sizeof(rinput); ++i) {
        rinput[i] = (uint8_t)(i * 17 + 3);
        linput[i] = (uint8_t)(i * 29 + 9);
    }
    rinput[probe_model_imu_length_offset(0)] = 30;
    linput[probe_model_imu_length_offset(1)] = 40;
    notify(right, SECONDARY_HANDLE, rinput, sizeof(rinput));
    uint32_t rserial = capture_native_peek(0, actual);
    assert(rserial && memcmp(actual, rinput, sizeof(actual)) == 0);
    notify(left, SECONDARY_HANDLE, linput, sizeof(linput));
    uint32_t lserial = capture_native_peek(1, actual);
    assert(lserial > rserial && memcmp(actual, linput, sizeof(actual)) == 0);
    assert(!capture_native_commit(0, lserial) && !capture_native_commit(1, rserial));
    assert(capture_native_commit(1, lserial));
    assert(capture_native_peek(1, actual) == 0);
    assert(capture_native_peek(0, actual) == rserial);
    assert(memcmp(actual, rinput, sizeof(actual)) == 0);

    uint64_t rtoken, ltoken;
    assert(capture_request(0, 3, &rtoken));
    assert(capture_request(1, 6, &ltoken) && ltoken > rtoken);
    const unsigned rcommands = right->commands, lcommands = left->commands;
    advance(13);
    assert(right->commands == rcommands + 1 && right->command[0] == 0x0a && right->command[8] == 3);
    assert(left->commands == lcommands + 1 && left->command[0] == 0x0a && left->command[8] == 6);
    assert(capture_result(0, ltoken) == -1 && capture_result(1, rtoken) == -1);
    assert(capture_result(0, rtoken) == 0 && capture_result(1, ltoken) == 0);

    // Opposite ATT/application ordering on live links: neither acknowledges its mate.
    notify(left, RESPONSE_HANDLE, sample_ack, sizeof(sample_ack));
    assert(capture_result(0, rtoken) == 0 && capture_result(1, ltoken) == 0);
    query_done(right, 0);
    assert(capture_result(0, rtoken) == 0 && capture_result(1, ltoken) == 0);
    notify(right, RESPONSE_HANDLE, sample_ack, sizeof(sample_ack));
    assert(capture_result(0, rtoken) == 1 && capture_result(0, rtoken) == -1);
    assert(capture_result(1, ltoken) == 0);
    query_done(left, 0);
    assert(capture_result(1, ltoken) == 1 && capture_result(1, ltoken) == -1);

    assert(capture_request(0, 4, &rtoken));
    assert(capture_request(1, 7, &ltoken));
    advance(13);
    assert(right->command[8] == 4 && left->command[8] == 7);
    notify(left, SECONDARY_HANDLE, linput, sizeof(linput));
    lserial = capture_native_peek(1, actual);
    assert(lserial > rserial);
    uni_hid_device_disconnect(&right->device);
    assert(capture_result(0, rtoken) == -1 && capture_result(1, ltoken) == 0);
    assert(capture_native_peek(0, actual) == 0);
    assert(!capture_native_commit(0, rserial));
    assert(capture_native_peek(1, actual) == lserial);
    assert(memcmp(actual, linput, sizeof(actual)) == 0);
    query_done(left, 0);
    assert(capture_result(1, ltoken) == 0);
    notify(left, RESPONSE_HANDLE, sample_ack, sizeof(sample_ack));
    assert(capture_result(1, ltoken) == 1 && capture_result(1, ltoken) == -1);
    assert(capture_native_commit(1, lserial));
    assert(capture_native_peek(1, actual) == 0);
}
#endif

static void test_teardown_survives_capture_exhaustion(void) {
    struct fixture_peer* peer = sample_ready(false, 0);
    uint64_t token, next;
    assert(capture_request(0, 3, &token));
    // The parser has not taken this request, so only source teardown can fail
    // the mailbox when the serialized capture cannot record another event.
    capture_exhaust_records();
    uni_hid_device_disconnect(&peer->device);
    assert(capture_result(0, token) == -1 && !capture_request(0, 3, &next));
}

int main(void) {
    test_ack_order_and_source_matching();
    test_native_notification_bounds_and_fidelity();
    test_selection_cannot_transfer_pending_ack();
    test_cancel_does_not_transfer_old_ack();
    test_rejection_disconnect_and_late_link_events();
    test_freshness_timeout_and_clock_wrap();
#if SWITCH2_PROBE_COMPOSITE || SWITCH2_PROBE_HUB
    test_simultaneous_native_sources_and_real_acks();
#endif
    test_teardown_survives_capture_exhaustion();
    reset();
    puts("Source sample relay ACK ordering, ownership, rejection and lifetime passed");
    return 0;
}
"""

CAPTURE = r"""
#include "input/switch2_mouse_capture.cpp"
#include "model.h"
extern "C" uint32_t btstack_run_loop_get_time_ms(void);
extern "C" void capture_init(void) {
    const uint8_t address[6] = {0xc0,0x22,0x33,0x44,0x55,0x66};
    switch2_mouse_capture_init();
    switch2_mouse_capture_select_input(0, address, probe_model_pid(0));
#if SWITCH2_PROBE_COMPOSITE || SWITCH2_PROBE_HUB
    const uint8_t second[6] = {0xc0,0x22,0x33,0x44,0x55,0x67};
    switch2_mouse_capture_select_input(1, second, probe_model_pid(1));
#endif
}
extern "C" void capture_select(uint8_t instance, const uint8_t address[6], uint16_t product_id) {
    switch2_mouse_capture_select_input(instance, address, product_id);
}
extern "C" void capture_native_stream(uint8_t instance, bool enabled) {
    switch2_mouse_capture_set_native_stream(instance, enabled);
}
extern "C" uint32_t capture_native_peek(uint8_t instance, uint8_t report[63]) {
    return switch2_mouse_capture_peek_native_report(instance, btstack_run_loop_get_time_ms(), report);
}
extern "C" bool capture_native_commit(uint8_t instance, uint32_t serial) {
    return switch2_mouse_capture_commit_native_report(instance, serial);
}
extern "C" bool capture_request(uint8_t instance, uint8_t id, uint64_t* token) {
    return switch2_mouse_capture_request_sample(instance, id, btstack_run_loop_get_time_ms(), token);
}
extern "C" int capture_result(uint8_t instance, uint64_t token) {
    return switch2_mouse_capture_sample_result(instance, token, btstack_run_loop_get_time_ms());
}
extern "C" void capture_cancel(uint8_t instance) { switch2_mouse_capture_cancel_sample(instance); }
// Simulate the boot-lifetime counter boundary without billions of reports.
extern "C" void capture_exhaust_records(void) { g_total_records = UINT32_MAX; }
"""


@pytest.mark.parametrize(
    ("left", "composite", "hub"),
    [
        (False, False, False),
        (True, False, False),
        (False, True, False),
        (False, False, True),
    ],
    ids=["right", "left", "composite", "hub"],
)
def test_source_sample_requires_its_real_bluetooth_ack(
    tmp_path: Path, left: bool, composite: bool, hub: bool
) -> None:
    root = Path(__file__).resolve().parents[1]
    cc = shutil.which("cc") or shutil.which("gcc")
    cxx = shutil.which("c++") or shutil.which("g++")
    assert cc is not None and cxx is not None, "host C and C++ compilers are required"
    sdks = [root / "build" / "_deps" / "pico_sdk-src", root / "external" / "pico-sdk"]
    if sdk := os.environ.get("PICO_SDK_PATH"):
        sdks.insert(0, Path(sdk))
    btstack = next(
        (
            sdk / "lib" / "btstack" / "src"
            for sdk in sdks
            if (sdk / "lib" / "btstack" / "src" / "ble" / "gatt_client.h").is_file()
        ),
        None,
    )
    if btstack is None:
        pytest.skip(
            "Pico SDK BTstack headers required; configure firmware or set PICO_SDK_PATH"
        )
    prepared = prepare_bluepad32(
        root / "external" / "bluepad32",
        root / "patches" / "bluepad32-sdl3-imu.patch",
        tmp_path / "bluepad32-src",
    )
    common = [
        "-O1",
        "-Wall",
        "-Wextra",
        "-ffunction-sections",
        "-fdata-sections",
        "-DSWITCH_PICO_SWITCH2_USB_BRIDGE=1",
        "-DSWITCH_PICO_SWITCH2_MOUSE_CAPTURE=1",
        "-DSWITCH_PICO_SWITCH2_MOUSE_CAPTURE_NATIVE=1",
        f"-DSWITCH2_PROBE_JOYCON_LEFT={int(left)}",
        f"-DSWITCH2_PROBE_COMPOSITE={int(composite)}",
        f"-DSWITCH2_PROBE_HUB={int(hub)}",
        f"-I{root / 'tools' / 'switch2_usb_probe'}",
        f"-I{root / 'bluepad32_config'}",
    ]
    cflags = [
        *common,
        "-std=gnu11",
        "-DENABLE_BLE",
        "-DENABLE_CLASSIC",
        f"-I{root / 'tests'}",
        f"-I{root / 'tests' / 'switch2_parser_native_stubs'}",
        f"-I{prepared / 'src' / 'components' / 'bluepad32' / 'include'}",
        f"-I{btstack}",
        f"-I{btstack.parent / '3rd-party' / 'bluedroid' / 'encoder' / 'include'}",
        f"-I{btstack.parent / '3rd-party' / 'bluedroid' / 'decoder' / 'include'}",
        f"-I{btstack.parent / '3rd-party' / 'yxml'}",
    ]
    source = tmp_path / "sample_relay.c"
    source.write_text(SOURCE)
    capture = tmp_path / "capture.cpp"
    capture.write_text(CAPTURE)
    objects = []
    for index, path in enumerate(
        (
            source,
            root / "bluepad32_config" / "parser" / "uni_hid_parser_switch2.c",
            root / "bluepad32_config" / "parser" / "uni_switch2_haptics.c",
            btstack / "btstack_util.c",
        )
    ):
        obj = tmp_path / f"source{index}.o"
        subprocess.run(
            [cc, *cflags, "-c", str(path), "-o", str(obj)], check=True, cwd=root
        )
        objects.append(str(obj))
    executable = tmp_path / "sample_relay"
    subprocess.run(
        [
            cxx,
            *common,
            "-std=c++17",
            "-pthread",
            f"-I{root / 'tests' / 'switch2_mouse_bridge_native_stubs'}",
            f"-I{root / 'src' / 'firmware'}",
            str(capture),
            *objects,
            "-Wl,--gc-sections",
            "-o",
            str(executable),
        ],
        check=True,
        cwd=root,
    )
    subprocess.run([str(executable)], check=True, cwd=root)
