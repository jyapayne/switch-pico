#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hardware_stub.h"
#include "tusb_config.h"

static unsigned test_core, test_exception;
#define get_core_num() test_core
#define __get_current_exception() test_exception
#define hard_assert(value) assert(value)
#define HID_REPORT_TYPE_INPUT 1
static uint64_t get_absolute_time(void) { return 0; }
static uint32_t to_ms_since_boot(uint64_t value) { return (uint32_t)value; }
static void sleep_us(uint32_t microseconds) { (void)microseconds; }
static void panic(const char* text) { (void)text; abort(); }
#define main unused_probe_firmware_main
#include "../tools/switch2_usb_probe/main.c"
#undef main

uint32_t native_test_interrupt_mask;
static bool pending_completion, inject_completion;
static unsigned completions, missed_tokens;
static uint32_t phase;
static char serial_bytes[2 * LOG_CAPACITY];
static size_t serial_size;

void native_test_service_interrupt(void) {
    if (pending_completion && !native_test_interrupt_mask) {
        pending_completion = false;
        ++completions;
    }
}

uint32_t native_hub_trace_phase(uint32_t next) {
    const uint32_t previous = phase;
    phase = next;
    if (inject_completion && next == NATIVE_HUB_TRACE_PHASE_LOG_COPY) {
        // A completed child packet must be serviced before the next owner's
        // token can be selected. This is the same blocking condition checked
        // by native_hub_select_device; the real logger runs between both.
        pending_completion = true;
        native_test_service_interrupt();
        if (pending_completion) ++missed_tokens;
    }
    return previous;
}

bool uart_is_writable(void* uart) { (void)uart; return serial_size < sizeof(serial_bytes); }
void uart_putc_raw(void* uart, char value) { (void)uart; serial_bytes[serial_size++] = value; }

int main(int argc, char** argv) {
    if (argc == 2) {
        if (strcmp(argv[1], "core") == 0) test_core = 1;
        else if (strcmp(argv[1], "irq") == 0) test_exception = 16;
        else return 2;
        probe_debug_printf("unsafe caller\n");
        return 0;
    }
    // Exercise a wrapped, full-length diagnostic message, not just empty logs.
    char message[480];
    memset(message, 'x', sizeof(message) - 1);
    message[sizeof(message) - 1] = 0;
    log_read = log_written = LOG_CAPACITY - 13;
    inject_completion = true;
    assert(probe_debug_printf("%s", message) == (int)strlen(message));
    assert(missed_tokens == 0 && "logging blocked a USB completion and the next device's token");
    assert(completions == 1 && !pending_completion);
    drain_log();
    assert(serial_size == strlen(message));
    assert(memcmp(serial_bytes, message, serial_size) == 0);

    // Full-ring overflow drops a complete message without corrupting queued data.
    inject_completion = false;
    serial_size = 0;
    log_read = 0; log_written = LOG_CAPACITY;
    memset(log_bytes, 'q', sizeof(log_bytes));
    const uint32_t drops_before = log_dropped;
    assert(probe_debug_printf("discard me") < 0);
    drain_log();
    assert(serial_size == LOG_CAPACITY);
    for (size_t i = 0; i < serial_size; ++i) assert(serial_bytes[i] == 'q');
    assert(log_dropped == drops_before + 10);
    serial_size = 0;
    assert(probe_debug_printf("discard me") == 10);
    drain_log();
    assert(serial_size == 10 && memcmp(serial_bytes, "discard me", 10) == 0);

    // Respect a caller's existing critical section; logging cannot enable IRQs.
    serial_size = 0;
    inject_completion = true;
    native_test_interrupt_mask = 1;
    const unsigned completed_before = completions;
    probe_debug_printf("caller owns mask");
    assert(native_test_interrupt_mask == 1 && completions == completed_before);
    restore_interrupts(0);
    assert(completions == completed_before + 1);
    drain_log();
    assert(serial_size == strlen("caller owns mask"));
    assert(memcmp(serial_bytes, "caller owns mask", serial_size) == 0);
    puts("native logging preserved USB progress, message order and caller IRQ state");
    return 0;
}
