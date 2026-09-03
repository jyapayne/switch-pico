#include <stdio.h>
#include "bsp/board.h"
#include "pico/stdlib.h"
#include "tusb.h"
#include "switch_pro_driver.h"
#include "usb_output_driver.h"
#ifndef SWITCH_PICO_BLUEPAD32
#include "hardware/uart.h"
#else
#include "adapter_mode_controller.h"
#include "bluepad32_input_backend.h"
#include "bootsel_pairing_button.h"
#include "controller_profile_runtime.h"
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
#define UART_RUMBLE_TYPE 0x02
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
static constexpr uint8_t SWITCH_HID_INSTANCE = 0;
static bool g_last_ready = false;
static ControllerState g_user_state;
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
static void send_rumble_uart_frame(const ControllerRumbleOutput& rumble) {
    uint8_t frame[5] = {
        UART_RUMBLE_HEADER,
        UART_RUMBLE_TYPE,
        rumble.low_frequency_magnitude,
        rumble.high_frequency_magnitude,
        0,
    };

    for (uint8_t i = 0; i < 4; ++i) {
        frame[4] = static_cast<uint8_t>(frame[4] + frame[i]);
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
    if (instance != SWITCH_HID_INSTANCE) {
        return;
    }
    send_rumble_uart_frame(rumble);
#endif
}

#ifndef SWITCH_PICO_BLUEPAD32
// Consume UART bytes and forward complete frames to the Switch Pro driver.
static bool poll_uart_frames() {
    static uint8_t buffer[64];
    static uint8_t index = 0;
    static uint8_t expected_len = 0;
    static absolute_time_t last_byte_time = {0};
    static bool has_last_byte = false;
    bool new_data = false;

    while (uart_is_readable(UART_ID)) {
        uint8_t byte = uart_getc(UART_ID);

        uint64_t now = to_ms_since_boot(get_absolute_time());
        if (has_last_byte && (now - to_ms_since_boot(last_byte_time)) > 20) {
            index = 0; // stale data, restart frame
            expected_len = 0;
        }
        last_byte_time = get_absolute_time();
        has_last_byte = true;

        if (index == 0) {
            if (byte != 0xAA) {
                continue; // wait for start-of-frame marker
            }
        }

        if (index >= sizeof(buffer)) {
            index = 0;
            expected_len = 0;
        }

        buffer[index++] = byte;
        if (index == 3) {
            expected_len = static_cast<uint8_t>(buffer[2] + 4u);
            if (expected_len < 12 || expected_len > sizeof(buffer)) {
                index = 0;
                expected_len = 0;
                continue;
            }
        }

        if (expected_len > 0 && index >= expected_len) {
            ControllerState parsed{};
            if (switch_pro_apply_uart_packet(buffer, expected_len, parsed)) {
                g_user_state = parsed;
                new_data = true;
                LOG_PRINTF("[UART] packet buttons=0x%04x hat=%u lx=%u ly=%u rx=%u ry=%u\n",
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
    const bool ready = usb_output_driver_is_ready(SWITCH_HID_INSTANCE);
    if (ready != g_last_ready) {
        g_last_ready = ready;
        LOG_PRINTF("[SWITCH] driver %s\n",
                   ready ? "ready (handshake OK)" : "not ready");
    }
#endif
}

int main() {
    board_init();
    stdio_init_all();

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
    usb_output_driver_set_rumble_callback(SWITCH_HID_INSTANCE,
                                          on_rumble_from_usb);
    g_user_state = neutral_input();
    usb_output_driver_set_input(SWITCH_HID_INSTANCE, g_user_state,
                                SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD,
                                SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD);
#endif

#ifdef SWITCH_PICO_BLUEPAD32
    bluepad32_input_backend_start();
    LOG_PRINTF("[BOOT] adapter mode=%s\n",
               usb_output_driver_mode_name());
#else
    LOG_PRINTF("[BOOT] switch-pico starting (UART0 log @ 115200)\n");
    LOG_PRINTF("[INFO] UART1 pins TX=%d RX=%d baud=%d\n",
           UART_TX_PIN, UART_RX_PIN, BAUD_RATE);
#endif

    while (true) {
        tud_task();          // USB device tasks

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
        ControllerState state = g_user_state;
        usb_output_driver_set_input(SWITCH_HID_INSTANCE, state,
                                    SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD,
                                    SWITCH_PRO_DIGITAL_TRIGGER_THRESHOLD);
        (void)usb_output_driver_task(SWITCH_HID_INSTANCE);
#endif
        log_usb_state();
    }
}
