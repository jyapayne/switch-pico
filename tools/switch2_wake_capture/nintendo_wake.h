#ifndef SWITCH2_WAKE_CAPTURE_NINTENDO_WAKE_H
#define SWITCH2_WAKE_CAPTURE_NINTENDO_WAKE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

static inline const uint8_t* switch2_wake_manufacturer(
    const uint8_t* data, size_t size) {
    static const uint8_t prefix[] = {
        0x53, 0x05, 0x01, 0x00, 0x03, 0x7e, 0x05,
    };
    static const uint8_t suffix[] = {
        0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    if (size != 31 || memcmp(data, "\x02\x01\x06\x1b\xff", 5) != 0) {
        return NULL;
    }
    const uint8_t* manufacturer = &data[5];
    if (memcmp(manufacturer, prefix, sizeof(prefix)) != 0 ||
        manufacturer[9] != 0x00 || manufacturer[10] != 0x01 ||
        manufacturer[11] != 0x81 ||
        memcmp(&manufacturer[18], suffix, sizeof(suffix)) != 0) {
        return NULL;
    }
    uint8_t target_any = 0;
    for (size_t index = 12; index < 18; ++index) {
        target_any |= manufacturer[index];
    }
    return target_any == 0 ? NULL : manufacturer;
}

#endif
