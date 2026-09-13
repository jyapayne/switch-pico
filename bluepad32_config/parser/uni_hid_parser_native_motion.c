// SPDX-License-Identifier: Apache-2.0
#include "parser/uni_hid_parser_native_motion.h"

#if SWITCH2_BRIDGE_FULL_INPUT
#include <string.h>

#include "parser/uni_hid_parser_wii.h"
#include "sdkconfig.h"
#include "uni_hid_device.h"

// Parser data is already close to its fixed capacity on some families. Keep
// only native-bridge provenance here, not in every normal AIO parser instance.
typedef struct {
    uni_hid_device_t* device;
    void (*setup)(uni_hid_device_t*);
    uint8_t address[6];
    uni_native_motion_snapshot_t sample;
    uint32_t timestamp;
    bool have_timestamp;
} native_motion_t;
static native_motion_t providers[CONFIG_BLUEPAD32_MAX_DEVICES];
static uint32_t sequence;

uint32_t uni_hid_parser_native_motion_next_sequence(void) {
    if (++sequence == 0)
        ++sequence;
    return sequence;
}

static native_motion_t* provider(uni_hid_device_t* d) {
    if (!d)
        return NULL;
    for (unsigned i = 0; i < CONFIG_BLUEPAD32_MAX_DEVICES; ++i) {
        native_motion_t* p = &providers[i];
        if (p->device == d && p->setup == d->report_parser.setup &&
            memcmp(p->address, d->conn.btaddr, sizeof(p->address)) == 0)
            return p;
    }
    return NULL;
}

void uni_hid_parser_native_motion_forget(uni_hid_device_t* d) {
    for (unsigned i = 0; i < CONFIG_BLUEPAD32_MAX_DEVICES; ++i)
        if (providers[i].device == d)
            memset(&providers[i], 0, sizeof(providers[i]));
}

void uni_hid_parser_native_motion_reset(uni_hid_device_t* d) {
    if (!d)
        return;
    uni_hid_parser_native_motion_forget(d);
    for (unsigned i = 0; i < CONFIG_BLUEPAD32_MAX_DEVICES; ++i) {
        native_motion_t* p = &providers[i];
        if (p->device)
            continue;
        p->device = d;
        p->setup = d->report_parser.setup;
        memcpy(p->address, d->conn.btaddr, sizeof(p->address));
        p->sample.report_tracked = true;
        return;
    }
}

void uni_hid_parser_native_motion_begin(uni_hid_device_t* d) {
    native_motion_t* p = provider(d);
    if (p)
        p->sample.report_valid = false;
}

void uni_hid_parser_native_motion_accept(uni_hid_device_t* d) {
    native_motion_t* p = provider(d);
    if (p) {
        p->sample.report_valid = true;
        p->sample.report_sequence = uni_hid_parser_native_motion_next_sequence();
    }
}

void uni_hid_parser_native_motion_accel(uni_hid_device_t* d, const int32_t* value) {
    native_motion_t* p = provider(d);
    if (!p)
        return;
    p->sample.accel_valid = value != NULL;
    if (value) {
        memcpy(p->sample.accel_q13, value, sizeof(p->sample.accel_q13));
        p->sample.accel_sequence = uni_hid_parser_native_motion_next_sequence();
    }
}

void uni_hid_parser_native_motion_gyro(uni_hid_device_t* d, const int32_t* value) {
    native_motion_t* p = provider(d);
    if (!p)
        return;
    p->sample.gyro_valid = value != NULL;
    if (value) {
        memcpy(p->sample.gyro_q10, value, sizeof(p->sample.gyro_q10));
        p->sample.gyro_sequence = uni_hid_parser_native_motion_next_sequence();
    }
}

bool uni_hid_parser_native_motion_fresh(uni_hid_device_t* d, uint32_t timestamp, uint32_t mask) {
    native_motion_t* p = provider(d);
    if (!p)
        return false;
    uint32_t delta = (timestamp - p->timestamp) & mask;
    if (p->have_timestamp && (delta == 0 || delta > (mask >> 1)))
        return false;
    p->timestamp = timestamp;
    p->have_timestamp = true;
    return true;
}

bool uni_hid_parser_native_motion_snapshot(uni_hid_device_t* d, uni_native_motion_snapshot_t* out) {
    if (!out)
        return false;
    memset(out, 0, sizeof(*out));
    if (!d)
        return false;
    // Wii already owns independent, topology-aware calibration and sample IDs.
    // ACK/status/extension packets must not refresh the Remote or MotionPlus.
    if (d->report_parser.setup == uni_hid_parser_wii_setup) {
        out->accel_valid = uni_hid_parser_wii_accel_snapshot(d, out->accel_q13, &out->accel_sequence);
        out->gyro_valid = uni_hid_parser_wii_gyro_snapshot(d, out->gyro_q10, &out->gyro_sequence);
        return true;
    }
    native_motion_t* p = provider(d);
    if (!p)
        return false;
    *out = p->sample;
    return true;
}
#endif
