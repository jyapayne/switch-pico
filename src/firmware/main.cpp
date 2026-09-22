#include <stdio.h>
#include <string.h>
#include "bsp/board.h"
#include "pico/stdlib.h"
#include "tusb.h"
#include "usb/switch/switch_pro_driver.h"
#include "usb/usb_output_driver.h"
#ifndef SWITCH_PICO_BLUEPAD32
#include "hardware/uart.h"
#include "pico/bootrom.h"
#ifdef SWITCH_PICO_UART_USB_MANAGEMENT
#include "usb/uart_usb_management.h"
#endif
#else
#include "adapter/adapter_mode_controller.h"
#include "input/bluepad32_input_backend.h"
#include "platform/pico/bootsel_pairing_button.h"
#include "platform/pico/system_clock.h"
#include "profile/controller_profile_runtime.h"
#endif
#ifdef SWITCH_PICO_WII_IR_MOUSE
#include "usb/wii_ir_mouse_usb.h"
#endif
#ifdef SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
#include "input/switch2_mouse_capture.h"
#endif

#ifdef SWITCH_PICO_LOG
#define LOG_PRINTF(...) printf(__VA_ARGS__)
#else
#define LOG_PRINTF(...) ((void)0)
#endif

#ifndef SWITCH_PICO_BLUEPAD32
// UART1 is reserved for external input frames from the host PC.
#define UART_ID uart1
#define BAUD_RATE 921600
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#define UART_RUMBLE_HEADER 0xBB
#define UART_RUMBLE_TYPE_SLOT 0x03
#define UART_STATS_TYPE 0x05
// Host -> Pico command frame: 0xAA 0xFE payload_len payload... checksum.
#define UART_COMMAND_VERSION 0xFE
#define UART_COMMAND_REBOOT_BOOTSEL 0x01
#define UART_COMMAND_STATS 0x02
#endif

#ifdef SWITCH_PICO_BLUEPAD32
static_assert(SWITCH_PICO_HID_INSTANCE_COUNT ==
              BLUEPAD32_INPUT_BACKEND_SLOT_COUNT);
static_assert(CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT ==
              BLUEPAD32_INPUT_BACKEND_SLOT_COUNT);
static_assert(SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD ==
              CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD);
static bool g_last_ready[BLUEPAD32_INPUT_BACKEND_SLOT_COUNT]{};
static ControllerState
    g_user_states[BLUEPAD32_INPUT_BACKEND_SLOT_COUNT]{};
#else
static bool g_last_ready[SWITCH_PICO_HID_INSTANCE_COUNT]{};
static ControllerState g_user_states[SWITCH_PICO_HID_INSTANCE_COUNT]{};
#endif

static bool g_last_mounted = false;

#ifndef SWITCH_PICO_BLUEPAD32
static void init_uart_input() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
}
#endif

static ControllerState neutral_input() {
    return controller_neutral_state();
}

#ifndef SWITCH_PICO_BLUEPAD32
// Pico -> host: 0xBB 0x03 slot low high checksum(sum of first 5 bytes).
static void send_rumble_uart_frame(uint8_t slot,
                                   const ControllerRumbleOutput& rumble) {
    uint8_t frame[6] = {
        UART_RUMBLE_HEADER,
        UART_RUMBLE_TYPE_SLOT,
        slot,
        rumble.low_frequency_magnitude,
        rumble.high_frequency_magnitude,
        0,
    };

    for (uint8_t i = 0; i < 5; ++i) {
        frame[5] = static_cast<uint8_t>(frame[5] + frame[i]);
    }
    uart_write_blocking(UART_ID, frame, sizeof(frame));
}
#endif

static void on_rumble_from_usb(uint8_t instance,
                               const ControllerRumbleOutput& rumble) {
#ifdef SWITCH_PICO_BLUEPAD32
    if (instance >= BLUEPAD32_INPUT_BACKEND_SLOT_COUNT) {
        return;
    }
    Bluepad32SlotSnapshot snapshot{};
    bluepad32_input_backend_snapshot(instance, &snapshot);
    bluepad32_input_backend_queue_rumble(
        instance, controller_profile_runtime_scale_host_rumble(
                      instance, snapshot, rumble));
#else
    if (instance >= SWITCH_PICO_HID_INSTANCE_COUNT) {
        return;
    }
    send_rumble_uart_frame(instance, rumble);
#endif
}

#ifndef SWITCH_PICO_BLUEPAD32
// Link health counters, reported on request so the host can tell frame loss
// from a quiet controller. Overruns count RX FIFO overflows flagged by the UART.
struct UartLinkStats {
    uint32_t frames_ok;
    uint32_t frames_rejected;
    uint32_t bytes_discarded;
    uint32_t overruns;
    uint32_t motion_frames;
    uint32_t motion_samples;
};
static UartLinkStats g_uart_stats{};

static void write_u32_le(uint8_t* dst, uint32_t value) {
    dst[0] = static_cast<uint8_t>(value);
    dst[1] = static_cast<uint8_t>(value >> 8);
    dst[2] = static_cast<uint8_t>(value >> 16);
    dst[3] = static_cast<uint8_t>(value >> 24);
}

// Pico -> host: 0xBB 0x05 then six little-endian u32 counters and checksum.
static void send_stats_uart_frame() {
    uint8_t frame[2 + 6 * 4 + 1] = {UART_RUMBLE_HEADER, UART_STATS_TYPE};
    const uint32_t values[6] = {
        g_uart_stats.frames_ok, g_uart_stats.frames_rejected,
        g_uart_stats.bytes_discarded, g_uart_stats.overruns,
        g_uart_stats.motion_frames, g_uart_stats.motion_samples,
    };
    for (uint8_t i = 0; i < 6; ++i) {
        write_u32_le(&frame[2 + i * 4], values[i]);
    }
    uint8_t sum = 0;
    for (uint8_t i = 0; i + 1 < sizeof(frame); ++i) {
        sum = static_cast<uint8_t>(sum + frame[i]);
    }
    frame[sizeof(frame) - 1] = sum;
    uart_write_blocking(UART_ID, frame, sizeof(frame));
}

// Command frames share the report framing. BOOTSEL must carry the "BOOTSEL"
// magic so line noise or a mis-framed report can never trigger it; STATS
// carries "STATS" for the same reason.
static void handle_uart_command(const uint8_t* frame, uint8_t length) {
    static const uint8_t kBootselMagic[7] = {'B', 'O', 'O', 'T', 'S', 'E', 'L'};
    static const uint8_t kStatsMagic[7] = {'S', 'T', 'A', 'T', 'S', 0, 0};
    uint8_t sum = 0;
    for (uint8_t i = 0; i + 1 < length; ++i) {
        sum = static_cast<uint8_t>(sum + frame[i]);
    }
    if (sum != frame[length - 1]) {
        ++g_uart_stats.frames_rejected;
        return;
    }
    const uint8_t payload_len = frame[2];
    const uint8_t* payload = frame + 3;
    if (payload_len != 8) {
        ++g_uart_stats.frames_rejected;
        return;
    }
    if (payload[0] == UART_COMMAND_REBOOT_BOOTSEL &&
        memcmp(payload + 1, kBootselMagic, sizeof(kBootselMagic)) == 0) {
        LOG_PRINTF("[UART] reboot to BOOTSEL requested\n");
        uart_tx_wait_blocking(UART_ID);
        reset_usb_boot(0, 0);
    } else if (payload[0] == UART_COMMAND_STATS &&
               memcmp(payload + 1, kStatsMagic, sizeof(kStatsMagic)) == 0) {
        ++g_uart_stats.frames_ok;
        send_stats_uart_frame();
    } else {
        ++g_uart_stats.frames_rejected;
    }
}

// Controls always reflect the newest UART packet. Motion samples accumulate
// across packets (newest CONTROLLER_MOTION_SAMPLE_CAPACITY kept) until a USB
// report consumes them, so the bridge can forward each sensor sample exactly
// once and the console never integrates a stale window twice.
static void merge_uart_state(ControllerState& slot_state,
                             const ControllerState& parsed) {
    ControllerMotionSample pending[CONTROLLER_MOTION_SAMPLE_CAPACITY];
    const uint8_t pending_count = slot_state.motion_sample_count;
    memcpy(pending, slot_state.motion_samples, sizeof(pending));

    slot_state = parsed;
    const uint8_t incoming = parsed.motion_sample_count;
    const uint8_t total = static_cast<uint8_t>(pending_count + incoming);
    const uint8_t kept = total > CONTROLLER_MOTION_SAMPLE_CAPACITY
                             ? CONTROLLER_MOTION_SAMPLE_CAPACITY
                             : total;
    const uint8_t kept_pending =
        kept > incoming ? static_cast<uint8_t>(kept - incoming) : 0;
    const uint8_t kept_incoming = static_cast<uint8_t>(kept - kept_pending);

    uint8_t out = 0;
    for (uint8_t i = static_cast<uint8_t>(pending_count - kept_pending);
         i < pending_count; ++i) {
        slot_state.motion_samples[out++] = pending[i];
    }
    for (uint8_t i = static_cast<uint8_t>(incoming - kept_incoming);
         i < incoming; ++i) {
        slot_state.motion_samples[out++] = parsed.motion_samples[i];
    }
    slot_state.motion_sample_count = kept;
}

// Consume UART bytes and forward complete frames to the Switch Pro driver.
static bool poll_uart_frames() {
    static uint8_t buffer[64];
    static uint8_t index = 0;
    static uint8_t expected_len = 0;
    static absolute_time_t last_byte_time = {0};
    static bool has_last_byte = false;
    bool new_data = false;

    // The UART flags RX FIFO overflow in RSR; clear it once counted.
    if (uart_get_hw(UART_ID)->rsr & UART_UARTRSR_OE_BITS) {
        uart_get_hw(UART_ID)->rsr = UART_UARTRSR_BITS;
        ++g_uart_stats.overruns;
    }

    while (uart_is_readable(UART_ID)) {
        uint8_t byte = uart_getc(UART_ID);

        uint64_t now = to_ms_since_boot(get_absolute_time());
        if (has_last_byte && (now - to_ms_since_boot(last_byte_time)) > 20) {
            if (index != 0) {
                g_uart_stats.bytes_discarded += index;
            }
            index = 0; // stale data, restart frame
            expected_len = 0;
        }
        last_byte_time = get_absolute_time();
        has_last_byte = true;

        if (index == 0) {
            if (byte != 0xAA) {
                ++g_uart_stats.bytes_discarded;
                continue; // wait for start-of-frame marker
            }
        }

        if (index >= sizeof(buffer)) {
            g_uart_stats.bytes_discarded += index;
            index = 0;
            expected_len = 0;
        }

        buffer[index++] = byte;
        if (index == 3) {
            // v2: header(3) + payload + checksum; v3 adds a slot byte.
            const uint8_t overhead = buffer[1] == 0x03 ? 5u : 4u;
            expected_len = static_cast<uint8_t>(buffer[2] + overhead);
            if (expected_len < 12 || expected_len > sizeof(buffer)) {
                g_uart_stats.bytes_discarded += index;
                ++g_uart_stats.frames_rejected;
                index = 0;
                expected_len = 0;
                continue;
            }
        }

        if (expected_len > 0 && index >= expected_len) {
            if (buffer[1] == UART_COMMAND_VERSION) {
                handle_uart_command(buffer, expected_len);
                index = 0;
                expected_len = 0;
                continue;
            }
            ControllerState parsed{};
            uint8_t slot = 0;
            if (switch_pro_apply_uart_packet(buffer, expected_len, parsed, slot)) {
                merge_uart_state(g_user_states[slot], parsed);
                new_data = true;
                ++g_uart_stats.frames_ok;
                if (parsed.motion_sample_count != 0) {
                    ++g_uart_stats.motion_frames;
                    g_uart_stats.motion_samples += parsed.motion_sample_count;
                }
                LOG_PRINTF("[UART] slot=%u buttons=0x%04x hat=%u lx=%u ly=%u rx=%u ry=%u\n",
                           slot,
                           (parsed.button_east   ? SWITCH_PRO_MASK_A   : 0) |
                           (parsed.button_south   ? SWITCH_PRO_MASK_B   : 0) |
                           (parsed.button_north   ? SWITCH_PRO_MASK_X   : 0) |
                           (parsed.button_west   ? SWITCH_PRO_MASK_Y   : 0) |
                           (parsed.button_left_shoulder   ? SWITCH_PRO_MASK_L   : 0) |
                           (parsed.button_right_shoulder   ? SWITCH_PRO_MASK_R   : 0) |
                           (parsed.left_trigger  ? SWITCH_PRO_MASK_ZL  : 0) |
                           (parsed.right_trigger  ? SWITCH_PRO_MASK_ZR  : 0) |
                           (parsed.button_start? SWITCH_PRO_MASK_PLUS: 0) |
                           (parsed.button_select?SWITCH_PRO_MASK_MINUS:0) |
                           (parsed.button_system?SWITCH_PRO_MASK_HOME:0) |
                           (parsed.button_capture?SWITCH_PRO_MASK_CAPTURE:0) |
                           (parsed.button_left_stick  ? SWITCH_PRO_MASK_L3  : 0) |
                           (parsed.button_right_stick  ? SWITCH_PRO_MASK_R3  : 0),
                           parsed.dpad_up ? SWITCH_PRO_HAT_UP :
                            parsed.dpad_down ? SWITCH_PRO_HAT_DOWN :
                            parsed.dpad_left ? SWITCH_PRO_HAT_LEFT :
                            parsed.dpad_right ? SWITCH_PRO_HAT_RIGHT : SWITCH_PRO_HAT_NOTHING,
                            controller_axis_to_unsigned(parsed.left_stick_x) >> 8,
                            controller_axis_to_unsigned(parsed.left_stick_y) >> 8,
                            controller_axis_to_unsigned(parsed.right_stick_x) >> 8,
                            controller_axis_to_unsigned(parsed.right_stick_y) >> 8);
            } else {
                ++g_uart_stats.frames_rejected;
            }
            index = 0;
            expected_len = 0;
        }
    }

    return new_data;
}
#endif

static void log_usb_state() {
    bool mounted = tud_mounted();
    if (mounted != g_last_mounted) {
        g_last_mounted = mounted;
        LOG_PRINTF("[USB] %s\n", mounted ? "mounted" : "unmounted");
#ifdef SWITCH_PICO_BLUEPAD32
        if (!mounted) {
            adapter_mode_controller_on_usb_unmounted();
        }
#endif
    }

#ifdef SWITCH_PICO_BLUEPAD32
    for (uint8_t instance = 0;
         instance < BLUEPAD32_INPUT_BACKEND_SLOT_COUNT; ++instance) {
        const bool ready = usb_output_driver_is_ready(instance);
        if (ready != g_last_ready[instance]) {
            g_last_ready[instance] = ready;
            LOG_PRINTF("[%s %u] driver %s\n",
                       usb_output_driver_name(), instance,
                       ready ? "ready" : "not ready");
        }
    }
#else
    for (uint8_t instance = 0;
         instance < SWITCH_PICO_HID_INSTANCE_COUNT; ++instance) {
        const bool ready = usb_output_driver_is_ready(instance);
        if (ready != g_last_ready[instance]) {
            g_last_ready[instance] = ready;
            LOG_PRINTF("[SWITCH %u] driver %s\n", instance,
                       ready ? "ready (handshake OK)" : "not ready");
        }
    }
#endif
}

int main() {
#ifdef SWITCH_PICO_BLUEPAD32
    system_clock_initialize();
#endif
    board_init();
    stdio_init_all();
#ifdef SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
    switch2_mouse_capture_init();
#endif

#ifdef SWITCH_PICO_BLUEPAD32
    bluepad32_input_backend_init();
    controller_profile_runtime_reset();
    adapter_mode_controller_initialize_usb();
#else
    init_uart_input();
    usb_output_driver_init(AdapterUsbMode::kSwitch);
    tusb_init();
#endif
#ifdef SWITCH_PICO_BLUEPAD32
    for (uint8_t instance = 0;
         instance < BLUEPAD32_INPUT_BACKEND_SLOT_COUNT; ++instance) {
        usb_output_driver_set_rumble_callback(instance,
                                              on_rumble_from_usb);
        g_user_states[instance] = neutral_input();
        usb_output_driver_set_input(
            instance, g_user_states[instance],
            CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD,
            CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD);
    }
#else
    for (uint8_t instance = 0;
         instance < SWITCH_PICO_HID_INSTANCE_COUNT; ++instance) {
        usb_output_driver_set_rumble_callback(instance, on_rumble_from_usb);
        g_user_states[instance] = neutral_input();
        usb_output_driver_set_input(instance, g_user_states[instance],
                                    SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD,
                                    SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD);
    }
#endif

#ifdef SWITCH_PICO_BLUEPAD32
    bluepad32_input_backend_start();
    LOG_PRINTF("[BOOT] adapter mode=%s\n",
               usb_output_driver_mode_name());
#else
    LOG_PRINTF("[BOOT] switch-pico starting (UART0 log @ 115200)\n");
    LOG_PRINTF("[INFO] UART1 pins TX=%d RX=%d baud=%d slots=%d\n",
           UART_TX_PIN, UART_RX_PIN, BAUD_RATE, SWITCH_PICO_HID_INSTANCE_COUNT);
#endif

    while (true) {
        tud_task();          // USB device tasks
#ifdef SWITCH_PICO_WII_IR_MOUSE
        usb_wii_ir_mouse_task();
#endif

#ifdef SWITCH_PICO_BLUEPAD32
        switch (bootsel_pairing_button_task()) {
            case BootselPairingButtonEvent::kOpenPairing:
                bluepad32_input_backend_open_pairing_window();
                break;
            case BootselPairingButtonEvent::kClearPairings:
                adapter_mode_controller_begin_recovery();
                break;
            case BootselPairingButtonEvent::kNone:
                break;
        }
        const uint32_t now_ms =
            static_cast<uint32_t>(to_ms_since_boot(get_absolute_time()));
        const AdapterUsbMode output_mode = usb_output_driver_mode();
        for (uint8_t instance = 0;
             instance < BLUEPAD32_INPUT_BACKEND_SLOT_COUNT; ++instance) {
            Bluepad32SlotSnapshot snapshot{};
            bluepad32_input_backend_snapshot(instance, &snapshot);
            adapter_mode_controller_process_input(
                instance, snapshot.active,
                snapshot.connection_generation,
                &snapshot.pre_hotkey_button_mask, now_ms,
                &snapshot.state);
            const ControllerProfileTransformResult transformed =
                controller_profile_runtime_transform(
                    instance, snapshot, now_ms, output_mode);
            ControllerProfileRuntimeProfileChangeEvent
                initial_profile_indication{};
            if (controller_profile_runtime_take_initial_profile_indication(
                    instance, &initial_profile_indication)) {
                bluepad32_input_backend_queue_profile_feedback(
                    instance,
                    initial_profile_indication.connection_generation,
                    initial_profile_indication.active_profile_number,
                    initial_profile_indication.policy);
            } else {
                ControllerProfileRuntimeProfileChangeEvent
                    profile_change{};
                if (controller_profile_runtime_take_profile_change(
                        instance, &profile_change)) {
                    bluepad32_input_backend_queue_profile_feedback(
                        instance, profile_change.connection_generation,
                        profile_change.active_profile_number,
                        profile_change.policy);
                }
            }
            g_user_states[instance] = transformed.state;
            usb_output_driver_set_input(
                instance, g_user_states[instance],
                transformed.left_trigger_digital_threshold,
                transformed.right_trigger_digital_threshold);
            if (usb_output_driver_task(instance)) {
                bluepad32_input_backend_report_sent(instance);
            }
        }
        adapter_mode_controller_task(now_ms);
#else
        bool new_data = poll_uart_frames();  // Pull controller state from UART1
        (void)new_data;
        for (uint8_t instance = 0;
             instance < SWITCH_PICO_HID_INSTANCE_COUNT; ++instance) {
            usb_output_driver_set_input(instance, g_user_states[instance],
                                        SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD,
                                        SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD);
            if (usb_output_driver_task(instance)) {
                // Mirror the AIO backend: motion samples are integrated by
                // exactly one USB report, then retired. Buttons and sticks
                // persist until the next UART packet.
                g_user_states[instance].motion_sample_count = 0;
            }
        }
#ifdef SWITCH_PICO_UART_USB_MANAGEMENT
        uart_usb_management_task();
#endif
#endif
        log_usb_state();
    }
}
