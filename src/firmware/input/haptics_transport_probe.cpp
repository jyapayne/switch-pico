#include "input/haptics_transport_probe.h"

#include <btstack.h>
#include <pico/critical_section.h>
#include <pico/stdlib.h>

extern "C" {
#include <cyw43.h>

// Derive the declarations from the SDK: in particular write uses size_t,
// whereas read's capacity and output length are uint32_t even on a 64-bit host.
decltype(cyw43_bluetooth_hci_write) __real_cyw43_bluetooth_hci_write;
decltype(cyw43_bluetooth_hci_write) __wrap_cyw43_bluetooth_hci_write;
decltype(cyw43_bluetooth_hci_read) __real_cyw43_bluetooth_hci_read;
decltype(cyw43_bluetooth_hci_read) __wrap_cyw43_bluetooth_hci_read;
decltype(btstack_run_loop_base_poll_data_sources)
    __real_btstack_run_loop_base_poll_data_sources;
decltype(btstack_run_loop_base_poll_data_sources)
    __wrap_btstack_run_loop_base_poll_data_sources;
}

namespace {

critical_section_t g_lock;
HapticsTransportProbe g_probe;
bool g_prepared = false;

// Only the snapshot crosses cores. All other state, including wrapper entry,
// belongs to core 1; the lock never spans SDK calls or event delivery.
uint64_t g_epoch = 0;
bool g_registered = false;
bool g_credit_sampled = false;
bool g_tone_recorded = false;
bool g_have_completion = false;
bool g_have_poll = false;
uint32_t g_last_completion_us = 0;
uint32_t g_last_poll_us = 0;
bool g_in_write = false;
bool g_in_read = false;
bool g_in_poll = false;
bool g_receive_work = false;
uint16_t g_controller_acl_bytes = 0;
uint16_t g_controller_acl_count = 0;
btstack_packet_callback_registration_t g_registration{};

uint32_t now_us() {
    return static_cast<uint32_t>(time_us_64());
}

void add(uint32_t& total, uint32_t value) {
    total = value > UINT32_MAX - total ? UINT32_MAX : total + value;
}

void maximum(uint32_t& peak, uint32_t value) {
    if (value > peak) peak = value;
}

void duration(uint32_t& calls, uint32_t& peak, uint32_t& total,
              uint32_t elapsed) {
    add(calls, 1);
    maximum(peak, elapsed);
    add(total, elapsed);
}

bool current(uint64_t epoch) {
    return g_probe.active != 0 && g_epoch == epoch;
}

void sample_credits(uint64_t epoch) {
    if (!current(epoch) || g_probe.connection_handle > 0x0fff) return;
    const auto handle = static_cast<hci_con_handle_t>(g_probe.connection_handle);
    // SDK lookup is fresh each time: never retain a connection across I/O or
    // callbacks, which can synchronously disconnect and free it.
    const hci_connection_t* connection = hci_connection_for_handle(handle);
    if (connection == nullptr) return;
    const uint32_t outstanding = connection->num_packets_sent;
    const int free_slots = hci_number_free_acl_slots_for_handle(handle);
    if (!current(epoch) || free_slots < 0) return;
    critical_section_enter_blocking(&g_lock);
    maximum(g_probe.max_outstanding_acl, outstanding);
    const auto available = static_cast<uint32_t>(free_slots);
    if (!g_credit_sampled || available < g_probe.min_free_acl) {
        g_probe.min_free_acl = available;
    }
    g_credit_sampled = true;
    critical_section_exit(&g_lock);
}

// Suppress only same-boundary recursive measurement, not its real call. Poll,
// read, write and sender totals are inclusive and deliberately overlap. Epoch
// checks discard a call that spans finish or a new begin, even with reused IDs.
struct Call {
    bool& entered;
    const bool outer;
    const uint64_t epoch;
    const bool measured;
    uint32_t start_us = 0;

    explicit Call(bool& guard)
        : entered(guard), outer(!guard), epoch(g_epoch),
          measured(outer && g_probe.active != 0) {
        entered = true;
        if (measured) {
            sample_credits(epoch);
            start_us = now_us();
        }
    }

    ~Call() {
        if (outer) entered = false;
    }

    bool live() const {
        return measured && current(epoch);
    }
};

uint16_t read_le16(const uint8_t* bytes) {
    return static_cast<uint16_t>(bytes[0] | (uint16_t{bytes[1]} << 8));
}
void capture_controller_capacity(const uint8_t* buffer, uint32_t length) {
    // CYW43's four-byte header precedes HCI Command Complete (Read Buffer Size).
    // Capture the raw response before BTstack applies its software buffer cap.
    if (length < 17 || buffer[3] != HCI_EVENT_PACKET || buffer[4] != 0x0e ||
        buffer[5] != 11 || buffer[7] != 0x05 || buffer[8] != 0x10 ||
        buffer[9] != 0) {
        return;
    }
    g_controller_acl_bytes = read_le16(buffer + 10);
    g_controller_acl_count = read_le16(buffer + 13);
    if (g_prepared) {
        critical_section_enter_blocking(&g_lock);
        g_probe.controller_acl_packet_bytes = g_controller_acl_bytes;
        g_probe.controller_acl_packet_count = g_controller_acl_count;
        critical_section_exit(&g_lock);
    }
}


void handle_event(uint8_t packet_type, uint16_t, uint8_t* packet,
                  uint16_t size) {
    if (!g_probe.active || packet_type != HCI_EVENT_PACKET || packet == nullptr ||
        size < 3 || packet[0] != HCI_EVENT_NUMBER_OF_COMPLETED_PACKETS ||
        size != uint16_t{packet[1]} + 2u ||
        size != 3u + uint16_t{packet[2]} * 4u ||
        g_probe.connection_handle > 0x0fff) {
        return;
    }
    uint32_t completed = 0;
    bool selected = false;
    for (uint16_t offset = 3; offset < size; offset += 4) {
        if ((read_le16(packet + offset) & 0x0fff) == g_probe.connection_handle) {
            selected = true;
            completed += read_le16(packet + offset + 2);
        }
    }
    if (!selected) return;
    const uint64_t epoch = g_epoch;
    const uint32_t timestamp = now_us();
    sample_credits(epoch);
    if (!current(epoch)) return;
    critical_section_enter_blocking(&g_lock);
    add(g_probe.completion_events, 1);
    add(g_probe.completed_packets, completed);
    // Gap between selected-handle events, not time from run start to first.
    if (g_have_completion) {
        maximum(g_probe.max_completion_gap_us, timestamp - g_last_completion_us);
    }
    g_have_completion = true;
    g_last_completion_us = timestamp;
    critical_section_exit(&g_lock);
    // BTstack updates credit accounting before delivering this event. These
    // samples and the surrounding I/O samples are observed extrema only.
    sample_credits(epoch);
}

}  // namespace

void haptics_transport_probe_prepare() {
    if (!g_prepared) {
        critical_section_init(&g_lock);
        g_prepared = true;
    }
}

void haptics_transport_probe_snapshot(HapticsTransportProbe* output) {
    if (output == nullptr) return;
    critical_section_enter_blocking(&g_lock);
    *output = g_probe;
    critical_section_exit(&g_lock);
}

void haptics_transport_probe_begin(uint32_t run_id, uint32_t generation,
                                  uint16_t handle) {
    ++g_epoch;
    critical_section_enter_blocking(&g_lock);
    g_probe = {};
    g_probe.run_id = run_id;
    g_probe.connection_generation = generation;
    g_probe.connection_handle = handle;
    g_probe.controller_acl_packet_bytes = g_controller_acl_bytes;
    g_probe.controller_acl_packet_count = g_controller_acl_count;
    g_probe.active = 1;
    g_credit_sampled = false;
    g_tone_recorded = false;
    g_have_completion = false;
    g_have_poll = false;
    critical_section_exit(&g_lock);
    if (!g_registered) {
        g_registered = true;
        g_registration.callback = handle_event;
        hci_add_event_handler(&g_registration);
    }
    sample_credits(g_epoch);
}

void haptics_transport_probe_end() {
    sample_credits(g_epoch);
    critical_section_enter_blocking(&g_lock);
    g_probe.active = 0;
    critical_section_exit(&g_lock);
}

void haptics_transport_probe_timer(uint32_t lateness_us) {
    if (!g_probe.active) return;
    critical_section_enter_blocking(&g_lock);
    duration(g_probe.timer_wakes, g_probe.max_timer_lateness_us,
             g_probe.total_timer_lateness_us, lateness_us);
    critical_section_exit(&g_lock);
}

void haptics_transport_probe_permission(uint32_t wait_us) {
    if (!g_probe.active) return;
    sample_credits(g_epoch);
    critical_section_enter_blocking(&g_lock);
    duration(g_probe.permission_callbacks, g_probe.max_permission_wait_us,
             g_probe.total_permission_wait_us, wait_us);
    critical_section_exit(&g_lock);
}

void haptics_transport_probe_send(uint32_t duration_us, uint32_t return_us,
                                 bool first_tone_success) {
    if (!g_probe.active) return;
    critical_section_enter_blocking(&g_lock);
    duration(g_probe.send_calls, g_probe.max_send_us, g_probe.total_send_us,
             duration_us);
    if (first_tone_success && !g_tone_recorded) {
        g_probe.first_tone_send_return_us = return_us;
        g_tone_recorded = true;
    }
    critical_section_exit(&g_lock);
    sample_credits(g_epoch);
}

extern "C" int __wrap_cyw43_bluetooth_hci_write(uint8_t* buffer, size_t length) {
    Call call(g_in_write);
    const int result = __real_cyw43_bluetooth_hci_write(buffer, length);
    if (call.live()) {
        const uint32_t elapsed = now_us() - call.start_us;
        sample_credits(call.epoch);
        if (call.live()) {
            critical_section_enter_blocking(&g_lock);
            duration(g_probe.write_calls, g_probe.max_write_us,
                     g_probe.total_write_us, elapsed);
            critical_section_exit(&g_lock);
        }
    }
    return result;
}

extern "C" int __wrap_cyw43_bluetooth_hci_read(uint8_t* buffer, uint32_t capacity,
                                             uint32_t* length) {
    Call call(g_in_read);
    const int result = __real_cyw43_bluetooth_hci_read(buffer, capacity, length);
    if (result == 0 && length != nullptr && *length <= capacity && buffer != nullptr) {
        capture_controller_capacity(buffer, *length);
    }
    // A nonempty read may have left more input queued. The experiment limits
    // each transport poll to one packet so timers can run between packets.
    g_receive_work = result == 0 && length != nullptr && *length > 0;
    if (call.live()) {
        const uint32_t elapsed = now_us() - call.start_us;
        // Match the transport's successful/nonempty read criterion. An error
        // may leave length untouched, so never inspect it on an error return.
        const bool packet = result == 0 && length != nullptr && *length > 0;
        sample_credits(call.epoch);
        if (call.live()) {
            critical_section_enter_blocking(&g_lock);
            duration(g_probe.read_calls, g_probe.max_read_us,
                     g_probe.total_read_us, elapsed);
            if (packet) add(g_probe.read_packets, 1);
            critical_section_exit(&g_lock);
        }
    }
    return result;
}

extern "C" void __wrap_btstack_run_loop_base_poll_data_sources() {
    Call call(g_in_poll);
    const bool have_previous = g_have_poll;
    const uint32_t gap = call.start_us - g_last_poll_us;
    if (call.measured) {
        g_have_poll = true;
        g_last_poll_us = call.start_us;
    }
    if (call.outer) g_receive_work = false;
    __real_btstack_run_loop_base_poll_data_sources();
    if (call.live()) {
        const uint32_t elapsed = now_us() - call.start_us;
        sample_credits(call.epoch);
        if (call.live()) {
            critical_section_enter_blocking(&g_lock);
            duration(g_probe.poll_calls, g_probe.max_poll_us,
                     g_probe.total_poll_us, elapsed);
            if (have_previous) maximum(g_probe.max_poll_gap_us, gap);
            critical_section_exit(&g_lock);
        }
    }
    if (call.outer && g_receive_work) {
        // Mark future work; never recursively drain from the current callback.
        // This also runs while measurements are inactive so input cannot be
        // stranded waiting for a new hardware IRQ edge after a bounded poll.
        btstack_run_loop_poll_data_sources_from_irq();
    }
}
