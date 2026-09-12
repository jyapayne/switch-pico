#include "memory.h"
#include "probe_memory_data.h"
#include <string.h>

_Static_assert(sizeof(probe_factory_memories) == PROBE_CONTROLLER_COUNT * 8192u,
               "factory capture sizes");
_Static_assert(sizeof(probe_user_calibrations) == PROBE_CONTROLLER_COUNT * 4096u,
               "user calibration capture sizes");

bool probe_memory_read(uint8_t instance, uint32_t address, uint8_t* output, size_t length) {
    if (instance >= PROBE_CONTROLLER_COUNT || !output) return false;
    const uint8_t* source;
    size_t offset, available;
    if (address >= 0x13000 && address < 0x15000) {
        offset = address - 0x13000;
        source = probe_factory_memories[instance];
        available = sizeof(probe_factory_memories[instance]) - offset;
    } else if (address >= 0x1fc000 && address < 0x1fd000) {
        offset = address - 0x1fc000;
        source = probe_user_calibrations[instance];
        available = sizeof(probe_user_calibrations[instance]) - offset;
    } else {
        return false; // No fabricated erased bytes, pairing keys, or firmware reads.
    }
    if (length > available) return false;
    memcpy(output, source + offset, length);
    return true;
}

static void unpack_pair(const uint8_t* data, uint16_t values[2]) {
    values[0] = data[0] | ((uint16_t)(data[1] & 15) << 8);
    values[1] = (data[1] >> 4) | ((uint16_t)data[2] << 4);
}

static bool valid_calibration(const uint8_t* data) {
    uint16_t center[2], positive[2], negative[2];
    unpack_pair(data, center);
    unpack_pair(data + 3, positive);
    unpack_pair(data + 6, negative);
    // Same bounds as the existing Bluepad32 Switch 2 calibration decoder.
    for (unsigned i = 0; i < 2; ++i) {
        if (!center[i] || center[i] == 4095 || !positive[i] || !negative[i] ||
            positive[i] > 4095 - center[i] || negative[i] > center[i]) return false;
    }
    return true;
}

bool probe_memory_stick_calibration(uint8_t instance, uint8_t output[9]) {
    if (instance >= PROBE_CONTROLLER_COUNT || !output) return false;
    // Each Joy-Con's own capture uses the primary calibration record.
    // User magic precedes its 9-byte record; factory has no magic.
    const uint8_t* selected = probe_factory_memories[instance] + 0xa8;
    const uint8_t* user = probe_user_calibrations[instance] + 0x40;
    if (user[0] == 0xb2 && user[1] == 0xa1 && valid_calibration(user + 2))
        selected = user + 2;
    if (!valid_calibration(selected)) return false;
    memcpy(output, selected, 9);
    return true;
}
