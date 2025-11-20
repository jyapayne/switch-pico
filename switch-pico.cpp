#include <stdio.h>
#include "bsp/board.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "tusb.h"
#include "switch_pro_driver.h"

// UART1 is reserved for external input frames from the host PC.
#define UART_ID uart1
#define BAUD_RATE 115200
#define UART_TX_PIN 4
#define UART_RX_PIN 5

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

// Consume UART bytes and forward complete frames to the Switch Pro driver.
static void poll_uart_frames() {
    static uint8_t buffer[8];
    static uint8_t index = 0;
    static absolute_time_t last_byte_time = {0};
    static bool has_last_byte = false;

    while (uart_is_readable(UART_ID)) {
        uint8_t byte = uart_getc(UART_ID);

        uint64_t now = to_ms_since_boot(get_absolute_time());
        if (has_last_byte && (now - to_ms_since_boot(last_byte_time)) > 20) {
            index = 0; // stale data, restart frame
        }
        last_byte_time = get_absolute_time();
        has_last_byte = true;

        if (index == 0) {
            if (byte != 0xAA) {
                continue; // wait for start-of-frame marker
            }
        }

        buffer[index++] = byte;
        if (index >= sizeof(buffer)) {
            switch_pro_apply_uart_packet(buffer, sizeof(buffer));
            index = 0;
        }
    }
}

int main() {
    board_init();
    stdio_init_all();

    init_uart_input();

    tusb_init();
    switch_pro_init();
    switch_pro_set_input(neutral_input());

    while (true) {
        tud_task();          // USB device tasks
        poll_uart_frames();  // Pull controller state from UART1
        switch_pro_task();   // Push state to the Switch host
    }
}
