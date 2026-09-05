#include "input/haptics_transport_probe.h"

#include <btstack.h>
#include <cyw43.h>
#include <pico/critical_section.h>

#include <cassert>
#include <climits>
#include <cstdio>
#include <cstring>

namespace {

constexpr uint16_t kHandle = 0x123;
uint64_t clock_us = 1000;
hci_connection_t connection{2};
bool connected = true;
int free_slots = 7;
unsigned registrations = 0;
void (*event_handler)(uint8_t, uint16_t, uint8_t*, uint16_t) = nullptr;
unsigned writes = 0, reads = 0, polls = 0;
uint32_t write_delay = 0, read_delay = 0, poll_delay = 0;
int write_result = 0, read_result = 0;
uint32_t read_length = 8;
uint8_t* last_buffer = nullptr;
size_t last_write_length = 0;
uint32_t last_capacity = 0;
uint32_t* last_length = nullptr;
void (*write_action)() = nullptr;
void (*read_action)() = nullptr;
void (*poll_action)() = nullptr;
uint8_t buffer[16]{};
bool poll_requested = false;
unsigned queued_input = 0;
unsigned serviced_input = 0;

HapticsTransportProbe snapshot() {
    HapticsTransportProbe result;
    haptics_transport_probe_snapshot(&result);
    return result;
}

void unchanged(const HapticsTransportProbe& expected) {
    const auto actual = snapshot();
    static_assert(sizeof(actual) == 128);
    assert(std::memcmp(&actual, &expected, sizeof(actual)) == 0);
}

void run_action(void (*&action)()) {
    const auto callback = action;
    action = nullptr;
    if (callback != nullptr) callback();
}

int fake_write(uint8_t* data, size_t length) {
    assert(native_probe_lock_depth == 0);
    (void)snapshot();  // Real calls may synchronously reenter snapshot readers.
    ++writes;
    last_buffer = data;
    last_write_length = length;
    clock_us += write_delay;
    run_action(write_action);
    if (data != nullptr && length != 0) data[0] = 0xa5;
    return write_result;
}

int fake_read(uint8_t* data, uint32_t capacity, uint32_t* length) {
    assert(native_probe_lock_depth == 0);
    (void)snapshot();
    ++reads;
    last_buffer = data;
    last_capacity = capacity;
    last_length = length;
    clock_us += read_delay;
    run_action(read_action);
    if (data != nullptr && capacity != 0) data[0] = 0x5a;
    if (read_result == 0 && length != nullptr) *length = read_length;
    return read_result;
}

void fake_poll() {
    assert(native_probe_lock_depth == 0);
    (void)snapshot();
    ++polls;
    clock_us += poll_delay;
    run_action(poll_action);
}

void begin(uint32_t run = 1, uint16_t handle = kHandle) {
    haptics_transport_probe_end();
    clock_us = 1000;
    connected = true;
    connection.num_packets_sent = 2;
    free_slots = 7;
    writes = reads = polls = 0;
    write_delay = read_delay = poll_delay = 0;
    write_result = read_result = 0;
    read_length = 8;
    write_action = read_action = poll_action = nullptr;
    poll_requested = false;
    haptics_transport_probe_begin(run, 9, handle);
    assert(registrations == 1);
}

void complete(uint16_t count) {
    uint8_t event[] = {0x13, 5, 1, 0x23, 0x01,
                       static_cast<uint8_t>(count), static_cast<uint8_t>(count >> 8)};
    event_handler(HCI_EVENT_PACKET, 0, event, sizeof(event));
}

void test_inactive() {
    haptics_transport_probe_prepare();
    haptics_transport_probe_prepare();
    const auto before = snapshot();
    write_result = -7;
    read_result = 17;
    uint32_t length = 99;
    assert(cyw43_bluetooth_hci_write(buffer, sizeof(buffer)) == -7);
    assert(cyw43_bluetooth_hci_read(buffer, sizeof(buffer), &length) == 17);
    assert(length == 99);
    btstack_run_loop_base_poll_data_sources();
    haptics_transport_probe_timer(123);
    haptics_transport_probe_permission(456);
    haptics_transport_probe_send(789, 1000, true);
    unchanged(before);
    assert(writes == 1 && reads == 1 && polls == 1 && registrations == 0);
}

void test_delay_attribution() {
    begin();
    poll_action = [] {
        clock_us += 50;  // Generation is outside the caller-supplied send time.
        const uint32_t send_start = static_cast<uint32_t>(clock_us);
        clock_us += 7;
        write_delay = 3000;
        write_action = [] { connection.num_packets_sent = 5; free_slots = 1; };
        assert(cyw43_bluetooth_hci_write(buffer, sizeof(buffer)) == 0);
        clock_us += 11;
        haptics_transport_probe_send(static_cast<uint32_t>(clock_us) - send_start,
                                     static_cast<uint32_t>(clock_us), true);
        clock_us += 9;
        uint32_t length = 0;
        read_delay = 80;
        assert(cyw43_bluetooth_hci_read(buffer, sizeof(buffer), &length) == 0);
        read_delay = 90;
        read_length = 0;
        assert(cyw43_bluetooth_hci_read(buffer, sizeof(buffer), &length) == 0);
        read_delay = 70;
        read_result = -9;
        length = 99;  // Stale output length on failure is not a packet.
        assert(cyw43_bluetooth_hci_read(buffer, sizeof(buffer), &length) == -9);
        clock_us += 30;
    };
    btstack_run_loop_base_poll_data_sources();
    auto result = snapshot();
    assert(result.send_calls == 1 && result.total_send_us == 3018);
    assert(result.max_send_us == 3018 && result.first_tone_send_return_us == 4068);
    assert(result.write_calls == 1 && result.total_write_us == 3000);
    assert(result.max_write_us == 3000);
    assert(result.read_calls == 3 && result.read_packets == 1);
    assert(result.total_read_us == 240 && result.max_read_us == 90);
    assert(result.poll_calls == 1 && result.total_poll_us == 3347);
    assert(result.max_poll_us == 3347 && result.max_poll_gap_us == 0);
    assert(result.max_outstanding_acl == 5 && result.min_free_acl == 1);
    clock_us += 600;
    poll_delay = 100;
    btstack_run_loop_base_poll_data_sources();
    result = snapshot();
    assert(result.poll_calls == 2 && result.total_poll_us == 3447);
    assert(result.max_poll_gap_us == 3947);
}

void test_nested_boundaries() {
    begin();
    write_result = -23;
    write_delay = 10;
    write_action = [] {
        write_delay = 30;
        assert(cyw43_bluetooth_hci_write(buffer, sizeof(buffer)) == -23);
        clock_us += 7;
    };
    assert(cyw43_bluetooth_hci_write(buffer, sizeof(buffer)) == -23);
    assert(writes == 2 && snapshot().write_calls == 1);
    assert(snapshot().total_write_us == 47);
    read_delay = 11;
    read_action = [] {
        read_delay = 19;
        uint32_t length = 0;
        assert(cyw43_bluetooth_hci_read(buffer, sizeof(buffer), &length) == 0);
        clock_us += 3;
    };
    uint32_t length = 0;
    assert(cyw43_bluetooth_hci_read(buffer, sizeof(buffer), &length) == 0);
    assert(reads == 2 && snapshot().read_calls == 1);
    assert(snapshot().read_packets == 1 && snapshot().total_read_us == 33);
    poll_delay = 5;
    poll_action = [] {
        poll_delay = 13;
        btstack_run_loop_base_poll_data_sources();
        write_delay = 10;
        assert(cyw43_bluetooth_hci_write(buffer, sizeof(buffer)) == -23);
        read_delay = 7;
        read_length = 0;
        uint32_t count = 0;
        assert(cyw43_bluetooth_hci_read(buffer, sizeof(buffer), &count) == 0);
        clock_us += 2;
    };
    btstack_run_loop_base_poll_data_sources();
    const auto result = snapshot();
    assert(polls == 2 && result.poll_calls == 1 && result.total_poll_us == 37);
    assert(result.write_calls == 2 && result.total_write_us == 57);
    assert(result.read_calls == 2 && result.total_read_us == 40 && result.read_packets == 1);
}

void test_returns_and_outputs() {
    begin();
    write_delay = 2;
    read_delay = 3;
    const int results[] = {0, -5, 17, INT_MIN, INT_MAX};
    for (int status : results) {
        write_result = read_result = status;
        buffer[0] = 0;
        assert(cyw43_bluetooth_hci_write(buffer, SIZE_MAX) == status);
        assert(last_buffer == buffer && last_write_length == SIZE_MAX && buffer[0] == 0xa5);
        uint32_t length = 0xfeed;
        assert(cyw43_bluetooth_hci_read(buffer, UINT32_MAX, &length) == status);
        assert(last_buffer == buffer && last_capacity == UINT32_MAX && last_length == &length);
        assert(buffer[0] == 0x5a && length == (status == 0 ? read_length : 0xfeed));
    }
    auto result = snapshot();
    assert(writes == 5 && reads == 5 && result.write_calls == 5 && result.read_calls == 5);
    assert(result.total_write_us == 10 && result.total_read_us == 15 && result.read_packets == 1);
    haptics_transport_probe_end();
    result = snapshot();
    assert(cyw43_bluetooth_hci_write(nullptr, 0) == INT_MAX);
    assert(cyw43_bluetooth_hci_read(nullptr, 0, nullptr) == INT_MAX);
    btstack_run_loop_base_poll_data_sources();
    complete(8);
    unchanged(result);
}

void test_selected_completions() {
    begin();
    uint8_t event[] = {0x13, 9, 2, 0x22, 0, 4, 0, 0x23, 0x01, 3, 0};
    clock_us = 6000;
    event_handler(HCI_EVENT_PACKET, 0, event, sizeof(event));
    assert(snapshot().completion_events == 1 && snapshot().completed_packets == 3);
    assert(snapshot().max_completion_gap_us == 0);
    clock_us = 16000;
    uint8_t other[] = {0x13, 5, 1, 0x22, 0, 7, 0};
    event_handler(HCI_EVENT_PACKET, 0, other, sizeof(other));
    clock_us = 30000;
    complete(2);
    const auto before = snapshot();
    assert(before.completion_events == 2 && before.completed_packets == 5);
    assert(before.max_completion_gap_us == 24000);
    event_handler(HCI_EVENT_PACKET, 0, nullptr, 7);
    event_handler(HCI_EVENT_PACKET, 0, event, 2);
    event_handler(HCI_EVENT_PACKET, 0, event, sizeof(event) - 1);
    event_handler(2, 0, event, sizeof(event));
    event[1] = 8;  // Inconsistent event parameter length.
    event_handler(HCI_EVENT_PACKET, 0, event, sizeof(event));
    event[1] = 9;
    event[2] = 255;  // Handle count exceeds bounded packet storage.
    event_handler(HCI_EVENT_PACKET, 0, event, sizeof(event));
    unchanged(before);

    begin(2, 0xffff);
    complete(3);
    assert(snapshot().completion_events == 0);
    assert(snapshot().max_outstanding_acl == 0 && snapshot().min_free_acl == 0);
}

void test_delayed_completion_and_disconnect() {
    begin();
    connection.num_packets_sent = 5;
    free_slots = 1;
    write_delay = 7;
    write_action = [] {
        clock_us += 80000;
        connection.num_packets_sent = 0;
        free_slots = 6;
        complete(5);  // Completion delivery reenters an in-flight write.
        connected = false;
        connection.num_packets_sent = 255;  // The former object is now invalid.
    };
    assert(cyw43_bluetooth_hci_write(buffer, sizeof(buffer)) == 0);
    haptics_transport_probe_end();
    const auto result = snapshot();
    assert(result.write_calls == 1 && result.total_write_us == 80007);
    assert(result.completion_events == 1 && result.completed_packets == 5);
    assert(result.max_outstanding_acl == 5 && result.min_free_acl == 1);
    complete(7);
    unchanged(result);
}

void test_run_changes_during_calls() {
    begin(7);
    write_delay = 100;
    write_action = [] { haptics_transport_probe_end(); };
    assert(cyw43_bluetooth_hci_write(buffer, sizeof(buffer)) == 0);
    assert(snapshot().active == 0 && snapshot().write_calls == 0);

    begin(7);
    read_delay = 100;
    read_action = [] {
        haptics_transport_probe_end();
        haptics_transport_probe_begin(7, 9, kHandle);  // Even identical public IDs.
        haptics_transport_probe_permission(23);
    };
    uint32_t length = 0;
    assert(cyw43_bluetooth_hci_read(buffer, sizeof(buffer), &length) == 0);
    auto result = snapshot();
    assert(result.active == 1 && result.run_id == 7 && result.connection_generation == 9);
    assert(result.connection_handle == kHandle && result.read_calls == 0 && result.read_packets == 0);
    assert(result.permission_callbacks == 1 && result.total_permission_wait_us == 23);

    poll_delay = 100;
    poll_action = [] {
        haptics_transport_probe_begin(8, 10, kHandle);
        haptics_transport_probe_timer(17);
    };
    btstack_run_loop_base_poll_data_sources();
    result = snapshot();
    assert(result.run_id == 8 && result.connection_generation == 10 && result.poll_calls == 0);
    assert(result.timer_wakes == 1 && result.max_timer_lateness_us == 17);
    assert(result.permission_callbacks == 0 && result.total_permission_wait_us == 0);
    clock_us += 500;
    poll_delay = 20;
    btstack_run_loop_base_poll_data_sources();
    assert(snapshot().poll_calls == 1 && snapshot().total_poll_us == 20);
    assert(snapshot().max_poll_gap_us == 0);

    haptics_transport_probe_end();
    poll_action = [] { haptics_transport_probe_begin(9, 11, kHandle); };
    btstack_run_loop_base_poll_data_sources();
    assert(snapshot().run_id == 9 && snapshot().poll_calls == 0);
    assert(registrations == 1);
}

void test_saturation_and_clock_wrap() {
    begin();
    clock_us = uint64_t{UINT32_MAX} - 20;
    write_delay = 40;
    assert(cyw43_bluetooth_hci_write(buffer, sizeof(buffer)) == 0);
    assert(snapshot().total_write_us == 40);
    haptics_transport_probe_timer(UINT32_MAX - 2);
    haptics_transport_probe_timer(10);
    haptics_transport_probe_permission(UINT32_MAX - 3);
    haptics_transport_probe_permission(10);
    haptics_transport_probe_send(UINT32_MAX - 5, 0, true);
    haptics_transport_probe_send(10, 123, true);
    const auto result = snapshot();
    assert(result.timer_wakes == 2 && result.total_timer_lateness_us == UINT32_MAX);
    assert(result.max_timer_lateness_us == UINT32_MAX - 2);
    assert(result.permission_callbacks == 2 && result.total_permission_wait_us == UINT32_MAX);
    assert(result.max_permission_wait_us == UINT32_MAX - 3);
    assert(result.send_calls == 2 && result.total_send_us == UINT32_MAX);
    assert(result.max_send_us == UINT32_MAX - 5 && result.first_tone_send_return_us == 0);
    clock_us = uint64_t{UINT32_MAX} - 5;
    complete(1);
    clock_us += 20;
    complete(1);
    assert(snapshot().max_completion_gap_us == 20);
    clock_us = uint64_t{UINT32_MAX} - 5;
    poll_delay = 5;
    btstack_run_loop_base_poll_data_sources();
    clock_us += 40;
    btstack_run_loop_base_poll_data_sources();
    assert(snapshot().max_poll_gap_us == 45);
}

void receive_one() {
    read_length = queued_input != 0 ? 8 : 0;
    uint32_t length = 0;
    assert(cyw43_bluetooth_hci_read(buffer, sizeof(buffer), &length) == 0);
    if (length != 0) {
        --queued_input;
        ++serviced_input;
    }
    poll_action = receive_one;
}

void test_bounded_receive_progress() {
    begin();
    haptics_transport_probe_end();  // Input must progress outside an active run.
    const auto before = snapshot();
    queued_input = 3;
    serviced_input = 0;
    poll_action = receive_one;
    poll_requested = true;
    unsigned timer_opportunities = 0;
    while (poll_requested && timer_opportunities < 10) {
        poll_requested = false;
        const unsigned handled_before = serviced_input;
        btstack_run_loop_base_poll_data_sources();
        assert(serviced_input - handled_before <= 1);
        ++timer_opportunities;  // SDK services timers before the next poll.
    }
    assert(queued_input == 0 && serviced_input == 3);
    assert(timer_opportunities == 4 && !poll_requested);
    unchanged(before);
}

void test_advertised_capacity_capture() {
    begin();
    haptics_transport_probe_end();
    uint8_t response[] = {
        0, 0, 0, HCI_EVENT_PACKET, 0x0e, 11, 1, 0x05, 0x10,
        0, 0xfd, 3, 0, 10, 0, 0, 0,
    };
    const auto before = snapshot();
    uint32_t length = 0;
    read_length = sizeof(response) - 1;
    assert(cyw43_bluetooth_hci_read(response, sizeof(response), &length) == 0);
    unchanged(before);
    read_length = sizeof(response);
    response[9] = 1;  // Failed command must not replace advertised capacity.
    assert(cyw43_bluetooth_hci_read(response, sizeof(response), &length) == 0);
    unchanged(before);
    response[9] = 0;
    assert(cyw43_bluetooth_hci_read(response, sizeof(response), &length) == 0);
    assert(snapshot().controller_acl_packet_bytes == 1021);
    assert(snapshot().controller_acl_packet_count == 10);
    begin(45);
    assert(snapshot().controller_acl_packet_count == 10);
    assert(snapshot().controller_acl_packet_bytes == 1021);
}

}  // namespace

int (*native_write)(uint8_t*, size_t) = fake_write;
int (*native_read)(uint8_t*, uint32_t, uint32_t*) = fake_read;
void (*native_poll)() = fake_poll;

uint64_t time_us_64() { return clock_us; }

extern "C" void btstack_run_loop_poll_data_sources_from_irq() {
    assert(native_probe_lock_depth == 0);
    poll_requested = true;
}

extern "C" void hci_add_event_handler(btstack_packet_callback_registration_t* registration) {
    assert(native_probe_lock_depth == 0 && ++registrations == 1);
    event_handler = registration->callback;
}

extern "C" hci_connection_t* hci_connection_for_handle(hci_con_handle_t handle) {
    assert(native_probe_lock_depth == 0);
    return connected && handle == kHandle ? &connection : nullptr;
}

extern "C" int hci_number_free_acl_slots_for_handle(hci_con_handle_t handle) {
    assert(native_probe_lock_depth == 0 && connected && handle == kHandle);
    return free_slots;
}

int main() {
    test_inactive();
    test_delay_attribution();
    test_nested_boundaries();
    test_returns_and_outputs();
    test_selected_completions();
    test_delayed_completion_and_disconnect();
    test_run_changes_during_calls();
    test_saturation_and_clock_wrap();
    test_bounded_receive_progress();
    test_advertised_capacity_capture();
    std::puts("haptics transport probe tests passed");
}
