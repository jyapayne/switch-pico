ginclude <stdio.h>
#include <string.h>
#include "bsp/board.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "tusb.h"
#include "switch_pro_driver.h"

#ifdef SWITCH_PICO_LOG
#define LOG_PRINTF(...) printf(__VA_ARGS__)
#else
#define LOG_PRINTF(...) ((void)0)
#endif

// UART1 is reserved for external input frames from the host PC.
#define UART_ID uart1
#define BAUD_RATE 921600
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#define UART_RUMBLE_HEADER 0xBB
#define UART_RUMBLE_RUMBLE_TYPE 0x01

static bool g_last_mounted = false;
static bool g_last_ready = false;

// Track the latest state provided by UART or the autopilot.
static SwitchInputState g_user_state;

static void init_uart_input() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
}

static SwitchInputState neutral_input() {
    SwitchInputState state{};
    state.lx = SWITCH_PRO_JOYSTICK_MID;
    state.ly = SWITCH_PRO_JOYSTICK_MID;
    state.rx = SWITCH_PRO_JOYSTICK_MID;
    state.ry = SWITCH_PRO_JOYSTICK_MID;
    return state;
}

static void send_rumble_uart_frame(const uint8_t rumble[8]) {
    uint8_t frame[11];
    frame[0] = UART_RUMBLE_HEADER;
    frame[1] = UART_RUMBLE_RUMBLE_TYPE;
    memcpy(&frame[2], rumble, 8);

    uint8_t checksum = 0;
    for (int i = 0; i < 10; ++i) {
        checksum = static_cast<uint8_t>(checksum + frame[i]);
    }
    frame[10] = checksum;
    uart_write_blocking(UART_ID, frame, sizeof(frame));
}

static void on_rumble_from_switch(const uint8_t rumble[8]) {
    send_rumble_uart_frame(rumble);
}

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

        buffer[index++] = byte;
        if (index == 3) {
            // We just stored payload_len; compute expected frame length (payload + header/version/len/checksum).
            expected_len = static_cast<uint8_t>(buffer[2] + 4);
            if (expected_len > sizeof(buffer) || expected_len < 8) {
                index = 0;
                expected_len = 0;
                continue;
            }
        }

        if (expected_len && index >= expected_len) {
            SwitchInputState parsed{};
            if (switch_pro_apply_uart_packet(buffer, expected_len, &parsed)) {
                g_user_state = parsed;
                LOG_PRINTF("[UART] packet buttons=0x%04x hat=%u lx=%u ly=%u rx=%u ry=%u\n",
                           (parsed.button_a   ? SWITCH_PRO_MASK_A   : 0) |
                           (parsed.button_b   ? SWITCH_PRO_MASK_B   : 0) |
                           (parsed.button_x   ? SWITCH_PRO_MASK_X   : 0) |
                           (parsed.button_y   ? SWITCH_PRO_MASK_Y   : 0) |
                           (parsed.button_l   ? SWITCH_PRO_MASK_L   : 0) |
                           (parsed.button_r   ? SWITCH_PRO_MASK_R   : 0) |
                           (parsed.button_zl  ? SWITCH_PRO_MASK_ZL  : 0) |
                           (parsed.button_zr  ? SWITCH_PRO_MASK_ZR  : 0) |
                           (parsed.button_plus? SWITCH_PRO_MASK_PLUS: 0) |
                           (parsed.button_minus?SWITCH_PRO_MASK_MINUS:0) |
                           (parsed.button_home?SWITCH_PRO_MASK_HOME:0) |
                           (parsed.button_capture?SWITCH_PRO_MASK_CAPTURE:0) |
                           (parsed.button_l3  ? SWITCH_PRO_MASK_L3  : 0) |
                           (parsed.button_r3  ? SWITCH_PRO_MASK_R3  : 0),
                           parsed.dpad_up ? SWITCH_PRO_HAT_UP :
                           parsed.dpad_down ? SWITCH_PRO_HAT_DOWN :
                           parsed.dpad_left ? SWITCH_PRO_HAT_LEFT :
                           parsed.dpad_right ? SWITCH_PRO_HAT_RIGHT : SWITCH_PRO_HAT_NOTHING,
                           parsed.lx >> 8, parsed.ly >> 8, parsed.rx >> 8, parsed.ry >> 8);
            }
            index = 0;
            expected_len = 0;
            new_data = true;
        }
    }
    return new_data;
}

static void log_usb_state() {
    bool mounted = tud_mounted();
    bool ready = switch_pro_is_ready();

    if (mounted != g_last_mounted) {
        g_last_mounted = mounted;
        LOG_PRINTF("[USB] %s\n", mounted ? "mounted" : "unmounted");
    }
    if (ready != g_last_ready) {
        g_last_ready = ready;
        LOG_PRINTF("[SWITCH] driver %s\n", ready ? "ready (handshake OK)" : "not ready");
    }
}

int main() {
    board_init();
    stdio_init_all();

    init_uart_input();

    tusb_init();
    switch_pro_init();
    switch_pro_set_rumble_callback(on_rumble_from_switch);
    g_user_state = neutral_input();
    switch_pro_set_input(g_user_state);

    LOG_PRINTF("[BOOT] switch-pico starting (UART0 log @ 115200)\n");
    LOG_PRINTF("[INFO] UART1 pins TX=%d RX=%d baud=%d\n",
           UART_TX_PIN, UART_RX_PIN, BAUD_RATE);

    while (true) {
        tud_task();          // USB device tasks
        bool new_data = poll_uart_frames();  // Pull controller state from UART1
        SwitchInputState state = g_user_state;
        switch_pro_set_input(state);
        bool should_update = new_data;
        if (should_update) {
            switch_pro_set_input(state);
        }
        switch_pro_task();   // Push state to the Switch host
        log_usb_state();
    }
}
