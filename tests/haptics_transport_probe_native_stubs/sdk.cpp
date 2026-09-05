#include <btstack.h>
#include <cyw43.h>

extern int (*native_write)(uint8_t*, size_t);
extern int (*native_read)(uint8_t*, uint32_t, uint32_t*);
extern void (*native_poll)();

// Keep definitions separate from test callers, just like the real SDK. Tests
// exercise GNU ld --wrap, not direct calls to the __wrap_ implementation.
extern "C" int cyw43_bluetooth_hci_write(uint8_t* buffer, size_t length) {
    return native_write(buffer, length);
}
extern "C" int cyw43_bluetooth_hci_read(uint8_t* buffer, uint32_t capacity,
                                       uint32_t* length) {
    return native_read(buffer, capacity, length);
}
extern "C" void btstack_run_loop_base_poll_data_sources() {
    native_poll();
}
