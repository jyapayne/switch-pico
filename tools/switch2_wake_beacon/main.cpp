#include "beacon_protocol.h"

#include <btstack.h>
#include "device/dcd.h"
#include "hardware/sync.h"
#include "pico/async_context_poll.h"
#include "pico/btstack_hci_transport_cyw43.h"
#include "pico/btstack_run_loop_async_context.h"
#include "pico/cyw43_driver.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "tusb.h"

namespace {

constexpr uint32_t kStartupTimeoutMs = 10000;
constexpr uint32_t kCommandTimeoutMs = 1000;
constexpr uint32_t kBurstWatchdogMs = 8000;
constexpr size_t kUsbBytesPerTurn = 64;
const hci_cmd_t kReadScanEnable{0x0c19, ""};

enum class RadioPhase : uint8_t {
    Starting,
    DisableClassicScan,
    VerifyClassicScan,
    InitializeWake,
    Ready,
    Unconfigured,
    Failed,
};

wake_beacon::Protocol protocol;
async_context_poll_t radio_context;
btstack_packet_callback_registration_t event_registration{};
RadioPhase radio_phase = RadioPhase::Starting;
bool driver_live = false;
bool hci_initialized = false;
bool wake_initialized = false;
bool burst_watchdog_armed = false;
uint16_t pending_opcode = 0;
uint32_t command_deadline = 0;
uint32_t startup_deadline = 0;
uint32_t burst_deadline = 0;
volatile bool usb_session_reset = false;

bool expired(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(now - deadline) >= 0;
}

void fail_radio() {
    // Deinitialization happens in the owner loop, not recursively in HCI events.
    radio_phase = RadioPhase::Failed;
    pending_opcode = 0;
}

void handle_packet(uint8_t packet_type, uint16_t,
                   uint8_t* packet, uint16_t size) {
    if (packet_type != HCI_EVENT_PACKET || size < 2 ||
        radio_phase == RadioPhase::Failed) {
        return;
    }
    const uint8_t event = hci_event_packet_get_type(packet);
    if (event == BTSTACK_EVENT_POWERON_FAILED ||
        event == HCI_EVENT_HARDWARE_ERROR) {
        fail_radio();
        return;
    }
    if (event == BTSTACK_EVENT_STATE && size >= 3) {
        const uint8_t state = btstack_event_state_get_state(packet);
        if (state == HCI_STATE_WORKING && radio_phase == RadioPhase::Starting) {
            radio_phase = RadioPhase::DisableClassicScan;
        } else if (state != HCI_STATE_INITIALIZING && state != HCI_STATE_WORKING) {
            fail_radio();
        }
        return;
    }
    if (event != HCI_EVENT_COMMAND_COMPLETE || size < 5 ||
        pending_opcode == 0 ||
        hci_event_command_complete_get_command_opcode(packet) != pending_opcode) {
        return;
    }
    pending_opcode = 0;
    if (size < 6 ||
        hci_event_command_complete_get_return_parameters(packet)[0] !=
            ERROR_CODE_SUCCESS) {
        fail_radio();
        return;
    }
    if (radio_phase == RadioPhase::DisableClassicScan) {
        radio_phase = RadioPhase::VerifyClassicScan;
    } else if (radio_phase == RadioPhase::VerifyClassicScan) {
        // Both inquiry and page scanning must be off on the dual-mode CYW43.
        if (size < 7 || packet[6] != 0) {
            fail_radio();
        } else {
            radio_phase = RadioPhase::InitializeWake;
        }
    }
}

wake_beacon::RadioStatus radio_status() {
    wake_beacon::RadioStatus status;
    status.ready = radio_phase == RadioPhase::Ready;
    status.failed = radio_phase == RadioPhase::Failed;
    status.initialized = wake_initialized;
    if (wake_initialized) {
        switch2_wake_diagnostics(&status.wake);
    }
    return status;
}

void radio_owner_task() {
    const uint32_t now = to_ms_since_boot(get_absolute_time());
    if (radio_phase != RadioPhase::Ready &&
        radio_phase != RadioPhase::Unconfigured &&
        radio_phase != RadioPhase::Failed && expired(now, startup_deadline)) {
        fail_radio();
    }
    if (pending_opcode != 0 && expired(now, command_deadline)) {
        fail_radio();
    }
    if (radio_phase == RadioPhase::Failed) {
        if (driver_live) {
            // Stop the physical radio too: a controller fault or failed stop
            // must not leave advertising running while USB reports failure.
            if (hci_initialized) {
                hci_close();
            }
            cyw43_driver_deinit(&radio_context.core);
            driver_live = false;
        }
        protocol.observe(radio_status());
        return;
    }
    if (pending_opcode == 0 && hci_can_send_command_packet_now()) {
        if (radio_phase == RadioPhase::DisableClassicScan) {
            pending_opcode = hci_write_scan_enable.opcode;
            command_deadline = now + kCommandTimeoutMs;
            if (hci_send_cmd(&hci_write_scan_enable, 0) != ERROR_CODE_SUCCESS) {
                fail_radio();
            }
        } else if (radio_phase == RadioPhase::VerifyClassicScan) {
            pending_opcode = kReadScanEnable.opcode;
            command_deadline = now + kCommandTimeoutMs;
            if (hci_send_cmd(&kReadScanEnable) != ERROR_CODE_SUCCESS) {
                fail_radio();
            }
        }
    }
    if (radio_phase == RadioPhase::InitializeWake) {
        if (!wake_initialized) {
            switch2_wake_initialize();
            wake_initialized = true;
        }
        const auto status = radio_status();
        if (!status.wake.configured) {
            radio_phase = RadioPhase::Unconfigured;
        } else if (status.wake.failures != 0) {
            fail_radio();
        } else if (switch2_wake_ready_for_connections() && !status.wake.busy) {
            radio_phase = RadioPhase::Ready;
        }
    }
    auto status = radio_status();
    if (burst_watchdog_armed) {
        if (!status.wake.busy) {
            burst_watchdog_armed = false;
        } else if (expired(now, burst_deadline)) {
            fail_radio();
            status = radio_status();
        }
    }
    protocol.observe(status);
    if (protocol.dispatch_pending()) {
        const bool accepted = switch2_wake_request();
        protocol.dispatched(accepted);
        if (accepted) {
            burst_deadline = now + kBurstWatchdogMs;
            burst_watchdog_armed = true;
        }
        protocol.observe(radio_status());
    }
}

void reset_usb_session() {
    protocol.connected(false);
    tud_cdc_read_flush();
    tud_cdc_write_clear();
}

void service_usb() {
    // TinyUSB's SDK version drains its event queue. Mask IRQs for this bounded
    // queue snapshot so continuous host traffic cannot keep refilling it. The
    // PICO OSAL preserves this interrupt mask; callbacks never touch the radio.
    const uint32_t saved = save_and_disable_interrupts();
    if (usb_session_reset) {
        usb_session_reset = false;
        reset_usb_session();
    }
    tud_task_ext(0, false);
    restore_interrupts(saved);

    const bool connected = stdio_usb_connected();
    protocol.connected(connected);
    if (!connected) {
        tud_cdc_read_flush();
        tud_cdc_write_clear();
        return;
    }
    // One retained response applies backpressure before another request can be
    // parsed. Never use printf/stdio flush: they may wait for a disconnected PC.
    size_t count = protocol.output_size();
    const size_t available = tud_cdc_write_available();
    if (count > available) count = available;
    if (count > kUsbBytesPerTurn) count = kUsbBytesPerTurn;
    if (count != 0) {
        protocol.consume_output(tud_cdc_write(protocol.output_data(), count));
    }
    tud_cdc_write_flush();
    for (size_t index = 0; index < kUsbBytesPerTurn && protocol.can_receive();
         ++index) {
        const int byte = tud_cdc_read_char();
        if (byte < 0) break;
        protocol.receive(static_cast<uint8_t>(byte));
    }
}

}  // namespace

extern "C" void tud_cdc_line_state_cb(uint8_t interface, bool dtr, bool) {
    if (interface == 0 && !dtr) {
        reset_usb_session();
    }
}

extern "C" void tud_umount_cb() {
    reset_usb_session();
}

extern "C" void tud_event_hook_cb(uint8_t, uint32_t event, bool) {
    // This hook may run in USB IRQ context. Only invalidate the session here;
    // TinyUSB and protocol work is deferred to service_usb() on Core 0.
    if (event == DCD_EVENT_BUS_RESET || event == DCD_EVENT_UNPLUGGED) {
        usb_session_reset = true;
    }
}

int main() {
    // Keep the SDK's unique-board-ID CDC descriptors, but use its TinyUSB FIFO
    // directly for bounded protocol output. No SDK/radio log belongs on CDC.
    if (!stdio_usb_init()) {
        return 1;
    }
    stdio_set_driver_enabled(&stdio_usb, false);
    startup_deadline = to_ms_since_boot(get_absolute_time()) + kStartupTimeoutMs;
    if (!async_context_poll_init_with_defaults(&radio_context) ||
        !cyw43_driver_init(&radio_context.core)) {
        fail_radio();
    } else {
        driver_live = true;
        // Deliberately bypass btstack_cyw43_init(): it initializes flash TLV.
        btstack_memory_init();
        btstack_run_loop_init(
            btstack_run_loop_async_context_get_instance(&radio_context.core));
        hci_init(hci_transport_cyw43_instance(), nullptr);
        hci_initialized = true;
        event_registration.callback = handle_packet;
        hci_add_event_handler(&event_registration);
        if (hci_power_control(HCI_POWER_ON) != ERROR_CODE_SUCCESS) {
            fail_radio();
        }
    }
    for (;;) {
        if (driver_live && radio_phase != RadioPhase::Failed) {
            async_context_poll(&radio_context.core);
        }
        radio_owner_task();
        service_usb();
        sleep_us(100);
    }
}
