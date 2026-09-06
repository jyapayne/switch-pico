#include "input/native_output_scheduler.h"

#include <btstack.h>
#include <pico/stdlib.h>
#include <uni.h>

namespace {
constexpr uint8_t kCapacity = 4;
constexpr uint8_t kNoClient = kCapacity;
constexpr uint8_t kNoCapacity = 0x07;
constexpr uint8_t kInvalidConnection = 0x02;

struct Client {
    uni_hid_device_t* device = nullptr;
    uint32_t generation = 0;
    uint16_t cid = 0;
    uint16_t handle = 0;
    uint64_t deadline_us = 0;
    uint64_t periodic_deadline_us = 0;
    NativeOutputGrant callback = nullptr;
    bool pending = false;
    bool waiting = false;
    bool urgent = false;
    bool reserved = false;
    bool generic_deferred = false;
    uint8_t request_status = 0;
};
struct EventContext {
    uni_hid_device_t* device;
    uint16_t cid;
    bool consumed = false;
    EventContext* previous;
};

Client clients[kCapacity];
uint8_t granted = kNoClient;
uint8_t cursor = 0;
bool prepared = false;
bool pumping = false;
bool repump = false;
bool timer_armed = false;
btstack_timer_source_t retry_timer{};
EventContext* current_event = nullptr;

void pump();

bool current(const Client& client) {
    return client.device != nullptr && client.device->conn.connected &&
        client.device->conn.interrupt_cid == client.cid &&
        client.device->conn.handle == client.handle && client.cid != 0;
}

void retire(uint8_t index) {
    if (granted == index) granted = kNoClient;
    clients[index] = {};
}

uint8_t find(uni_hid_device_t* device, uint32_t generation, bool allocate) {
    uint8_t empty = kNoClient;
    for (uint8_t i = 0; i < kCapacity; ++i) {
        Client& client = clients[i];
        if (client.device == device) {
            if (client.generation != generation || !current(client)) retire(i);
            else return i;
        } else if (client.device != nullptr && !current(client)) {
            retire(i);
        }
        if (client.device == nullptr && empty == kNoClient) empty = i;
    }
    if (!allocate || empty == kNoClient) return kNoClient;
    clients[empty].device = device;
    clients[empty].generation = generation;
    clients[empty].cid = device->conn.interrupt_cid;
    clients[empty].handle = device->conn.handle;
    return empty;
}

bool earlier(const Client& first, const Client& second) {
    if (first.urgent != second.urgent) return first.urgent;
    return first.deadline_us < second.deadline_us;
}

bool generic_event_in_progress(const uni_hid_device_t* device) {
    for (auto* event = current_event; event != nullptr; event = event->previous)
        if (event->device == device) return true;
    return false;
}

bool admission_allowed(uint8_t index) {
    const Client& client = clients[index];
    if (client.urgent) return true;
    if (hci_number_free_acl_slots_for_handle(client.handle) != 1) return true;
    for (uint8_t i = 0; i < kCapacity; ++i) {
        if (i == index || !current(clients[i]) || !clients[i].reserved) continue;
        const uint64_t deadline = clients[i].periodic_deadline_us;
        if (deadline < client.deadline_us ||
            (deadline == client.deadline_us &&
             (i + kCapacity - cursor) % kCapacity < (index + kCapacity - cursor) % kCapacity))
            return false;
    }
    return true;
}

void retry(btstack_timer_source_t*) {
    timer_armed = false;
    pump();
}

void arm_retry(uint64_t due_us) {
    if (timer_armed) btstack_run_loop_remove_timer(&retry_timer);
    const uint64_t now_ms = time_us_64() / 1000;
    const uint64_t due_ms = due_us / 1000;
    const uint32_t delay = due_ms > now_ms + 1 ? due_ms - now_ms - 1 : 0;
    btstack_run_loop_set_timer_handler(&retry_timer, retry);
    btstack_run_loop_set_timer(&retry_timer, delay);
    timer_armed = true;
    btstack_run_loop_add_timer(&retry_timer);
}

void pump() {
    if (!prepared) return;
    if (pumping) {
        repump = true;
        return;
    }
    pumping = true;
    if (timer_armed) {
        btstack_run_loop_remove_timer(&retry_timer);
        timer_armed = false;
    }
    uint64_t retry_at = UINT64_MAX;
    bool exhausted = true;
    // Bound synchronous callbacks/self-resubmission. A fresh loop turn services
    // receive processing and timers before another bounded wave of grants.
    for (uint8_t work = 0; work < kCapacity * 2; ++work) {
        repump = false;
        for (uint8_t i = 0; i < kCapacity; ++i)
            if (clients[i].device != nullptr && !current(clients[i])) retire(i);
        if (granted != kNoClient) {
            exhausted = false;
            break;
        }
        uint8_t selected = kNoClient;
        for (uint8_t offset = 0; offset < kCapacity; ++offset) {
            const uint8_t i = (cursor + offset) % kCapacity;
            const Client& client = clients[i];
            if (!client.pending || !current(client) ||
                !l2cap_can_send_packet_now(client.cid) || !admission_allowed(i)) continue;
            if (selected == kNoClient || earlier(client, clients[selected])) selected = i;
        }
        if (selected != kNoClient) {
            Client& client = clients[selected];
            const auto callback = client.callback;
            auto* device = client.device;
            const uint16_t cid = client.cid;
            const uint32_t generation = client.generation;
            client.pending = false;
            client.request_status = 0;
            client.generic_deferred = false;
            granted = selected;
            cursor = (selected + 1) % kCapacity;
            const bool consumed = callback(device, cid, generation);
            for (auto* event = current_event; event != nullptr; event = event->previous)
                if (event->device == device && event->cid == cid) event->consumed |= consumed;
            const bool yield_generic = !consumed && client.device == device &&
                client.generation == generation && client.cid == cid && current(client) &&
                !uni_circular_buffer_is_empty(&device->outgoing_buffer) &&
                !generic_event_in_progress(device);
            if (yield_generic && granted == selected) client.generic_deferred = true;
            if (yield_generic && granted == kNoClient && !client.waiting) {
                client.waiting = true;
                const uint8_t status = l2cap_request_can_send_now_event(cid);
                if (client.device == device && client.generation == generation && status != 0)
                    client.waiting = false;
            }
            continue;
        }
        // No ready client can use the resource. Register one notification per
        // pending CID; ready-state and deadline selection are rechecked when any
        // callback arrives, rather than treating a callback as reserved airtime.
        for (uint8_t offset = 0; offset < kCapacity; ++offset) {
            const uint8_t i = (cursor + offset) % kCapacity;
            Client& client = clients[i];
            if (!client.pending || !current(client)) continue;
            if (!admission_allowed(i)) {
                for (uint8_t other = 0; other < kCapacity; ++other) {
                    if (other == i || !current(clients[other]) || !clients[other].reserved) continue;
                    const uint64_t due = clients[other].periodic_deadline_us;
                    const uint64_t next = due > time_us_64() ? due : time_us_64() + 1000;
                    if (next < retry_at) retry_at = next;
                }
                continue;
            }
            if (client.waiting) continue;
            const uint32_t generation = client.generation;
            auto* device = client.device;
            const uint16_t cid = client.cid;
            client.waiting = true;
            const uint8_t status = l2cap_request_can_send_now_event(cid);
            if (client.device != device || client.generation != generation || client.cid != cid) continue;
            if (status != 0) {
                client.waiting = false;
                client.pending = false;
                client.request_status = status;
            }
        }
        if (!repump) {
            exhausted = false;
            break;
        }
    }
    if (exhausted && granted == kNoClient) retry_at = time_us_64() + 1000;
    if (granted == kNoClient && retry_at != UINT64_MAX) arm_retry(retry_at);
    pumping = false;
}
}  // namespace

void native_output_scheduler_prepare() {
    if (prepared) return;
    prepared = true;
}

uint8_t native_output_scheduler_request(uni_hid_device_t* device,
                                        uint32_t generation,
                                        uint64_t deadline_us,
                                        bool urgent_stop,
                                        NativeOutputGrant callback) {
    if (!prepared || device == nullptr || callback == nullptr || !device->conn.connected ||
        device->conn.interrupt_cid == 0) return kInvalidConnection;
    const uint8_t index = find(device, generation, true);
    if (index == kNoClient) return kNoCapacity;
    Client& client = clients[index];
    client.deadline_us = deadline_us;
    client.urgent = urgent_stop;
    client.callback = callback;
    client.pending = true;
    client.request_status = 0;
    pump();
    return client.device == device && client.generation == generation ? client.request_status : 0;
}

void native_output_scheduler_reserve(uni_hid_device_t* device,
                                     uint32_t generation,
                                     uint64_t deadline_us) {
    if (!prepared || device == nullptr || !device->conn.connected || !device->conn.interrupt_cid) return;
    const uint8_t index = find(device, generation, deadline_us != UINT64_MAX);
    if (index == kNoClient) return;
    clients[index].reserved = deadline_us != UINT64_MAX;
    clients[index].periodic_deadline_us = deadline_us;
    // Writers may announce the following interval immediately before requesting
    // the current due packet. Never dispatch competing work in that gap.
    // A standalone reservation release still wakes pending work next loop turn.
    if (!pumping && granted == kNoClient) {
        for (const auto& client : clients) {
            if (client.pending && current(client) && l2cap_can_send_packet_now(client.cid)) {
                arm_retry(time_us_64() + 1000);
                break;
            }
        }
    }
}

void native_output_scheduler_complete(uni_hid_device_t* device, uint32_t generation) {
    const uint8_t index = granted;
    const bool owned = index != kNoClient && clients[index].device == device &&
                       clients[index].generation == generation;
    const bool wake_generic = owned && clients[index].generic_deferred &&
                              !generic_event_in_progress(device);
    if (owned) clients[index].generic_deferred = false;
    if (owned) granted = kNoClient;
    pump();
    if (wake_generic && clients[index].device == device && clients[index].generation == generation &&
        current(clients[index]) && granted != index && !clients[index].waiting &&
        !uni_circular_buffer_is_empty(&device->outgoing_buffer)) {
        clients[index].waiting = true;
        const uint8_t status = l2cap_request_can_send_now_event(clients[index].cid);
        if (clients[index].device == device && clients[index].generation == generation && status != 0)
            clients[index].waiting = false;
    }
}

void native_output_scheduler_cancel(uni_hid_device_t* device) {
    if (!prepared || device == nullptr) return;
    for (uint8_t i = 0; i < kCapacity; ++i)
        if (clients[i].device == device) retire(i);
    pump();
}

bool native_output_scheduler_granted(const uni_hid_device_t* device) {
    return granted != kNoClient && clients[granted].device == device && current(clients[granted]);
}

bool native_output_scheduler_on_can_send_now(uni_hid_device_t* device, uint16_t cid) {
    if (!prepared || device == nullptr) return false;
    for (auto& client : clients)
        if (client.device == device && client.cid == cid && current(client)) client.waiting = false;
    const bool leased = granted != kNoClient && clients[granted].device == device &&
                        current(clients[granted]);
    EventContext event{device, cid, leased, current_event};
    current_event = &event;
    pump();
    current_event = event.previous;
    return event.consumed;
}
