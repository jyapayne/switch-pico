#pragma once

#include "model.h"
#include "tusb.h"
#if SWITCH2_PROBE_HUB
#include "usb/native_hub/native_hub.h"
#endif

// Application instances are controllers, never native hub device slots.
// Only control transfers retain the transport's rhport/device-slot argument.
static inline bool probe_transport_mounted(uint8_t instance) {
#if SWITCH2_PROBE_HUB
    return native_hub_mounted(instance);
#else
    (void)instance;
    return tud_mounted();
#endif
}

static inline bool probe_transport_suspended(uint8_t instance) {
#if SWITCH2_PROBE_HUB
    return native_hub_suspended(instance);
#else
    (void)instance;
    return tud_suspended();
#endif
}

static inline bool probe_transport_hid_ready(uint8_t instance) {
#if SWITCH2_PROBE_HUB
    return native_hub_hid_ready(instance);
#else
    return tud_hid_n_ready(instance);
#endif
}

static inline bool probe_transport_hid_report(uint8_t instance, uint8_t report_id,
                                               const void* data, uint16_t length) {
#if SWITCH2_PROBE_HUB
    return native_hub_hid_report(instance, report_id, data, length);
#else
    return tud_hid_n_report(instance, report_id, data, length);
#endif
}

static inline uint32_t probe_transport_vendor_write_available(uint8_t instance) {
#if SWITCH2_PROBE_HUB
    return native_hub_vendor_write_available(instance);
#else
    return tud_vendor_n_write_available(instance);
#endif
}

static inline uint32_t probe_transport_vendor_write(uint8_t instance,
                                                     const void* data, uint32_t length) {
#if SWITCH2_PROBE_HUB
    return native_hub_vendor_write(instance, data, length);
#else
    return tud_vendor_n_write(instance, data, length);
#endif
}

static inline uint32_t probe_transport_vendor_write_flush(uint8_t instance) {
#if SWITCH2_PROBE_HUB
    return native_hub_vendor_write_flush(instance);
#else
    return tud_vendor_n_write_flush(instance);
#endif
}

static inline void probe_transport_vendor_discard_received(uint8_t instance) {
#if SWITCH2_PROBE_HUB
    // Native RX supplies the actual packet once, with no second receive FIFO.
    (void)instance;
#else
    // The application consumes the raw callback packet, not this duplicate.
    uint8_t discarded[64];
    while (tud_vendor_n_available(instance)) {
        if (!tud_vendor_n_read(instance, discarded, sizeof(discarded))) break;
    }
#endif
}

static inline bool probe_transport_control_xfer(uint8_t rhport,
                                                 const tusb_control_request_t* request,
                                                 void* buffer, uint16_t length) {
#if SWITCH2_PROBE_HUB
    return native_hub_control_xfer(rhport, request, buffer, length);
#else
    return tud_control_xfer(rhport, request, buffer, length);
#endif
}

static inline bool probe_transport_control_status(uint8_t rhport,
                                                   const tusb_control_request_t* request) {
#if SWITCH2_PROBE_HUB
    return native_hub_control_status(rhport, request);
#else
    return tud_control_status(rhport, request);
#endif
}
