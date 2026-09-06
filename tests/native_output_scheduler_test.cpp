#include "input/native_output_scheduler.h"
#include <btstack.h>
#include <uni.h>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>

namespace {
uint64_t now = 1000;
int credits = 0;
bool available = true, synchronous = false, hold = false;
uint8_t request_error = 0;
unsigned resubmit = 0;
std::vector<uint16_t> delivered;
std::vector<btstack_timer_source_t*> timers;
uni_hid_device_t devices[4];
void require(bool condition, const char* message) {
    if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
bool send(uni_hid_device_t* device, uint16_t cid, uint32_t generation) {
    require(native_output_scheduler_granted(device), "sender ran without its grant");
    delivered.push_back(cid);
    --credits;
    if (!hold) native_output_scheduler_complete(device, generation);
    if (resubmit && device == &devices[0]) {
        --resubmit;
        native_output_scheduler_request(device, 1, 1000, false, send);
    }
    return true;
}
bool yield_generic(uni_hid_device_t* device, uint16_t cid, uint32_t generation) {
    if (!uni_circular_buffer_is_empty(&device->outgoing_buffer)) {
        native_output_scheduler_complete(device, generation);
        return false;
    }
    return send(device, cid, generation);
}
void notify(unsigned index) {
    const bool consumed = native_output_scheduler_on_can_send_now(&devices[index], devices[index].conn.interrupt_cid);
    if (!consumed && devices[index].outgoing_buffer.queued) {
        --devices[index].outgoing_buffer.queued;
        --credits;
    }
}
void request(unsigned index, uint64_t deadline, bool urgent = false) {
    require(native_output_scheduler_request(&devices[index], 1, deadline, urgent, send) == 0,
            "valid scheduler request rejected");
}
void edf() {
    request(0, 3000); request(1, 1000); request(2, 2000);
    require(delivered.empty(), "scheduler sent without transport credits");
    credits = 3; notify(0);
    require(delivered == std::vector<uint16_t>({65, 66, 64}), "deadline ordering followed callback/slot order");
}
void urgent() {
    request(0, 1000); request(1, 90000, true);
    credits = 2; notify(0);
    require(delivered == std::vector<uint16_t>({65, 64}), "stop did not precede ordinary overdue work");
}
void reservation() {
    native_output_scheduler_reserve(&devices[1], 1, 10000);
    credits = 1;
    request(0, 20000);
    require(delivered.empty(), "future earlier stream lost the last available credit");
    request(1, 10000);
    require(delivered == std::vector<uint16_t>({65}), "reserved stream could not use its credit");
    native_output_scheduler_reserve(&devices[1], 1, UINT64_MAX);
    credits = 1; notify(0);
    require(delivered == std::vector<uint16_t>({65, 64}), "retired reservation blocked remaining work");
}
void reservation_roll_forward() {
    native_output_scheduler_reserve(&devices[1], 1, 10000);
    request(0, 20000);
    credits = 1;
    // A periodic writer announces its next interval immediately before
    // registering its already-due current packet.
    native_output_scheduler_reserve(&devices[1], 1, 30000);
    request(1, 10000);
    require(delivered == std::vector<uint16_t>({65}),
            "reservation update dispatched later work before the due request was registered");
}

void reservation_tie() {
    native_output_scheduler_reserve(&devices[0], 1, 10000);
    native_output_scheduler_reserve(&devices[1], 1, 10000);
    request(0, 10000); request(1, 10000);
    credits = 1; notify(0);
    require(delivered.size() == 1, "equal periodic reservations deadlocked");
    credits = 1; notify(1);
    require(delivered.size() == 2 && delivered[0] != delivered[1], "reservation ties starved a peer");
}
void grant_lifetime() {
    credits = 2; hold = true;
    request(0, 1000); request(1, 500);
    require(delivered.size() == 1 && native_output_scheduler_granted(&devices[0]), "another client stole a live grant");
    require(native_output_scheduler_on_can_send_now(&devices[0], 90), "reentrant generic event escaped live output ownership");
    hold = false;
    native_output_scheduler_complete(&devices[0], 1);
    require(delivered == std::vector<uint16_t>({64, 65}), "grant completion did not unblock urgent peer");
}
void reuse() {
    request(0, 1000);
    native_output_scheduler_cancel(&devices[0]);
    devices[0].conn.interrupt_cid = 80;
    require(native_output_scheduler_request(&devices[0], 2, 2000, false, send) == 0, "new generation rejected");
    credits = 1;
    native_output_scheduler_on_can_send_now(&devices[0], 64);
    require(delivered == std::vector<uint16_t>({80}), "old generation/CID reached sender");
}
void stale_completion() {
    credits = 2;
    hold = true;
    request(0, 1000);
    native_output_scheduler_cancel(&devices[0]);
    require(native_output_scheduler_request(&devices[0], 2, 2000, false, send) == 0,
            "replacement generation was rejected");
    native_output_scheduler_complete(&devices[0], 1);
    require(native_output_scheduler_granted(&devices[0]),
            "old completion released the replacement generation's grant");
    native_output_scheduler_complete(&devices[0], 2);
    require(!native_output_scheduler_granted(&devices[0]), "current completion did not release its grant");
}

void bounded_reentry() {
    credits = 100; resubmit = 50;
    request(0, 1000);
    require(delivered.size() <= 8 && !timers.empty(), "synchronous producer monopolized event loop");
    const size_t before = delivered.size();
    request(1, 1000);
    require(delivered.size() > before && delivered[before] == 65, "equal deadline tie favored self-resubmitting client");
}
void generic() {
    credits = 1; synchronous = true;
    devices[0].outgoing_buffer.queued = 1;
    require(native_output_scheduler_request(&devices[0], 1, 1000, false, yield_generic) == 0, "generic-yield request rejected");
    require(delivered.empty() && devices[0].outgoing_buffer.queued == 0,
            "generic FIFO yield stalled or emitted native output first");
    credits = 1;
    require(native_output_scheduler_request(&devices[0], 1, 2000, false, yield_generic) == 0, "native retry rejected");
    require(delivered == std::vector<uint16_t>({64}), "native output did not resume after generic FIFO drain");
}
void error() {
    available = false; request_error = 0x44;
    require(native_output_scheduler_request(&devices[0], 1, 1000, false, send) == 0x44,
            "immediate transport request error was hidden");
    require(delivered.empty() && !native_output_scheduler_granted(&devices[0]), "failed request granted output");
}
}
uint64_t time_us_64() { return now; }
bool l2cap_can_send_packet_now(uint16_t) { return available && credits > 0; }
int hci_number_free_acl_slots_for_handle(uint16_t) { return credits; }
uint8_t l2cap_request_can_send_now_event(uint16_t cid) {
    if (request_error) return request_error;
    if (synchronous && available && credits > 0)
        for (unsigned i = 0; i < 4; ++i) if (devices[i].conn.interrupt_cid == cid) notify(i);
    return 0;
}
void btstack_run_loop_set_timer_handler(btstack_timer_source_t* timer, void (*callback)(btstack_timer_source_t*)) { timer->handler = callback; }
void btstack_run_loop_set_timer(btstack_timer_source_t* timer, uint32_t delay) { timer->due_us = (now / 1000 + delay + 1) * 1000; }
void btstack_run_loop_add_timer(btstack_timer_source_t* timer) { timers.push_back(timer); }
bool btstack_run_loop_remove_timer(btstack_timer_source_t* timer) {
    const auto found = std::find(timers.begin(), timers.end(), timer);
    if (found == timers.end()) return false;
    timers.erase(found); return true;
}
int main(int argc, char** argv) {
    require(argc == 2, "scenario required");
    for (uint16_t i = 0; i < 4; ++i) {
        devices[i].conn.handle = 10 + i;
        devices[i].conn.interrupt_cid = 64 + i;
    }
    native_output_scheduler_prepare();
    if (!std::strcmp(argv[1], "edf")) edf();
    else if (!std::strcmp(argv[1], "urgent")) urgent();
    else if (!std::strcmp(argv[1], "reservation")) reservation();
    else if (!std::strcmp(argv[1], "reservation-tie")) reservation_tie();
    else if (!std::strcmp(argv[1], "reservation-roll-forward")) reservation_roll_forward();
    else if (!std::strcmp(argv[1], "grant")) grant_lifetime();
    else if (!std::strcmp(argv[1], "reuse")) reuse();
    else if (!std::strcmp(argv[1], "stale-completion")) stale_completion();
    else if (!std::strcmp(argv[1], "bounded")) bounded_reentry();
    else if (!std::strcmp(argv[1], "generic")) generic();
    else if (!std::strcmp(argv[1], "error")) error();
    else require(false, "unknown scenario");
}
