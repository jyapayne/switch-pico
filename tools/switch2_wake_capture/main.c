#include <stdio.h>
#include <string.h>

#include "pico/cyw43_arch.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "btstack.h"
#include "nintendo_wake.h"

typedef struct {
    bd_addr_t advertiser;
    uint8_t address_type;
    uint8_t event_type;
    int8_t rssi;
    uint8_t data[31];
} wake_capture_t;

static wake_capture_t capture;
static bool captured;
static bool scan_started;
static bool usb_was_connected;
static uint32_t last_print_ms;
static btstack_packet_callback_registration_t registration;
static btstack_timer_source_t output_timer;

static void to_hex(const uint8_t* bytes, size_t size, char* output) {
    static const char digits[] = "0123456789ABCDEF";
    for (size_t index = 0; index < size; ++index) {
        output[index * 2] = digits[bytes[index] >> 4];
        output[index * 2 + 1] = digits[bytes[index] & 0x0f];
    }
    output[size * 2] = 0;
}

static void print_capture(void) {
    const uint8_t* manufacturer = &capture.data[5];
    char raw_hex[63];
    char payload_hex[49];
    to_hex(capture.data, sizeof(capture.data), raw_hex);
    // ESPHome-style payload excludes the two-byte Nintendo company ID.
    to_hex(&manufacturer[2], 24, payload_hex);
    printf("SWITCH2_WAKE_CAPTURE {\"version\":1,\"advertiser\":"
           "\"%02X:%02X:%02X:%02X:%02X:%02X\",\"address_type\":%u,"
           "\"event_type\":%u,\"rssi\":%d,\"pid\":\"%04X\","
           "\"console\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
           "\"raw_hex\":\"%s\",\"esphome_payload_hex\":\"%s\"}\n",
           capture.advertiser[0], capture.advertiser[1],
           capture.advertiser[2], capture.advertiser[3],
           capture.advertiser[4], capture.advertiser[5],
           capture.address_type, capture.event_type, (int)capture.rssi,
           (unsigned)(manufacturer[7] | (manufacturer[8] << 8)),
           manufacturer[17], manufacturer[16], manufacturer[15],
           manufacturer[14], manufacturer[13], manufacturer[12],
           raw_hex, payload_hex);
}

static void output_task(btstack_timer_source_t* timer) {
    const uint32_t now_ms = btstack_run_loop_get_time_ms();
    const bool usb_connected = stdio_usb_connected();
    if (usb_connected && !usb_was_connected) {
        puts("Switch 2 wake capture ready. Put the console to sleep, then wake it with a paired Joy-Con 2 HOME button.");
        puts("Scanning stops automatically after one valid public ADV_IND wake packet.");
        if (captured) {
            print_capture();
            last_print_ms = now_ms;
        }
    }
    if (usb_connected) {
        int key;
        while ((key = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
            if ((key == 'p' || key == 'P') && captured) {
                print_capture();
                last_print_ms = now_ms;
            }
        }
        // Repeat for a generator attached after capture; scanning remains stopped.
        if (captured && now_ms - last_print_ms >= 1000) {
            print_capture();
            last_print_ms = now_ms;
        }
    }
    usb_was_connected = usb_connected;
    static bool blink;
    blink = !blink;
    cyw43_arch_gpio_put(
        CYW43_WL_GPIO_LED_PIN, captured || (scan_started && blink));
    btstack_run_loop_set_timer(timer, 250);
    btstack_run_loop_add_timer(timer);
}

static void handle_packet(uint8_t packet_type, uint16_t,
                          uint8_t* packet, uint16_t size) {
    if (packet_type != HCI_EVENT_PACKET || size < 2 || captured) {
        return;
    }
    const uint8_t event_type = hci_event_packet_get_type(packet);
    if (event_type == BTSTACK_EVENT_STATE && size >= 3 &&
        btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
        gap_set_scan_params(0, 96, 96, 0);  // Passive, continuous, accept all.
        gap_set_scan_duplicate_filter(false);
        gap_start_scan();
        scan_started = true;
        return;
    }
    if (event_type != GAP_EVENT_ADVERTISING_REPORT || size < 12) {
        return;
    }
    const uint8_t length =
        gap_event_advertising_report_get_data_length(packet);
    if (length != sizeof(capture.data) || size < 12u + length ||
        gap_event_advertising_report_get_address_type(packet) !=
            BD_ADDR_TYPE_LE_PUBLIC ||
        gap_event_advertising_report_get_advertising_event_type(packet) != 0) {
        return;
    }
    const uint8_t* data =
        gap_event_advertising_report_get_data(packet);
    if (switch2_wake_manufacturer(data, length) == NULL) {
        return;
    }
    gap_event_advertising_report_get_address(packet, capture.advertiser);
    capture.address_type = BD_ADDR_TYPE_LE_PUBLIC;
    capture.event_type = 0;
    capture.rssi =
        (int8_t)gap_event_advertising_report_get_rssi(packet);
    memcpy(capture.data, data, sizeof(capture.data));
    captured = true;
    scan_started = false;
    gap_stop_scan();
    if (stdio_usb_connected()) {
        print_capture();
        last_print_ms = btstack_run_loop_get_time_ms();
    }
}

int main(void) {
    stdio_init_all();
    if (cyw43_arch_init() != 0) {
        while (true) {
            puts("ERROR: CYW43 initialization failed");
            sleep_ms(1000);
        }
    }
    registration.callback = handle_packet;
    hci_add_event_handler(&registration);
    btstack_run_loop_set_timer_handler(&output_timer, output_task);
    btstack_run_loop_set_timer(&output_timer, 250);
    btstack_run_loop_add_timer(&output_timer);
    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute();
    return 0;
}
