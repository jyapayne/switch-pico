// RAM-only native-SIE address-retargeting experiment. These are vendor test
// devices, not controllers. No usbd/tud global-device state is linked here.
#include "usb_probe.h"
#include "router.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "device/dcd.h"
#include "hardware/clocks.h"
#include "hardware/structs/usb.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"

#define RHPORT 0u
#define EP0_OUT 0x00u
#define EP0_IN 0x80u
#define HUB_EP 0x81u
#define EP0_SIZE 64u
#define EVENT_CAPACITY 32u
#define PORT_COUNT 2u

#define PORT_CONNECTION 0x0001u
#define PORT_ENABLE 0x0002u
#define PORT_SUSPEND 0x0004u
#define PORT_RESET 0x0010u
#define PORT_POWER 0x0100u
#define C_CONNECTION 0x0001u
#define C_ENABLE 0x0002u
#define C_SUSPEND 0x0004u
#define C_RESET 0x0010u

enum {
    FEATURE_PORT_ENABLE = 1,
    FEATURE_PORT_SUSPEND = 2,
    FEATURE_PORT_RESET = 4,
    FEATURE_PORT_POWER = 8,
    FEATURE_C_CONNECTION = 16,
    FEATURE_C_ENABLE = 17,
    FEATURE_C_SUSPEND = 18,
    FEATURE_C_OVERCURRENT = 19,
    FEATURE_C_RESET = 20,
};

typedef enum {
    CTRL_IDLE,
    CTRL_DATA_IN,
    CTRL_STATUS_IN,
    CTRL_STATUS_OUT,
    CTRL_STALLED,
} control_stage;

typedef enum {
    ACTION_NONE,
    ACTION_ADDRESS,
    ACTION_CONFIGURATION,
    ACTION_INTERFACE,
    ACTION_HALT,
    ACTION_CLEAR_HALT,
    ACTION_PORT_SET,
    ACTION_PORT_CLEAR,
    ACTION_KEEPALIVE,
    ACTION_ARM,
    ACTION_REBOOT,
} control_action;

typedef struct {
    uint16_t status;
    uint16_t change;
    uint32_t reset_deadline;
    uint32_t resume_deadline;
    bool resetting;
    bool resuming;
} hub_port;

typedef struct {
    dcd_event_t event;
    uint32_t generation;
    uint32_t endpoint_epoch;
    uint8_t setup_slot;
} queued_event;

typedef struct {
    tusb_control_request_t request;
    uint32_t generation;
    uint16_t length;
    uint16_t sent;
    uint16_t packet_length;
    uint8_t owner;
    control_stage stage;
    control_action action;
    bool need_zlp;
} control_transfer;

static uint8_t addresses[PROBE_ROUTER_SLOTS];
static uint8_t configurations[PROBE_ROUTER_SLOTS];
static uint8_t default_slot;
static bool routing_enabled;
static hub_port ports[PORT_COUNT];
static control_transfer control;
static uint8_t control_data[128] TU_ATTR_ALIGNED(4);
// Even a malformed nonempty status OUT cannot make the DCD copy into NULL.
static uint8_t control_out[EP0_SIZE] TU_ATTR_ALIGNED(4);
static uint32_t setup_count[PROBE_ROUTER_SLOTS];
static uint32_t bad_setup_owner;
static uint32_t correlated_setups;
static uint32_t system_clock_hz;
static bool interrupt_open;
static bool interrupt_pending;
static bool interrupt_halted;
static uint8_t interrupt_bitmap;
static uint32_t endpoint_epoch;
static bool reboot_pending;
static bool failed;

// Only the DCD IRQ produces; only Core 0's task consumes. All task-side DCD
// operations run with USB IRQ disabled. The IRQ never rearms a transfer: this
// SDK resets its transfer state *after* invoking dcd_event_handler().
static queued_event events[EVENT_CAPACITY];
static volatile uint32_t event_head;
static volatile uint32_t event_tail;
static volatile uint32_t event_generation;
static volatile bool event_overflow;
static uint32_t observed_setup_sequence;

static const uint8_t hub_configuration[] = {
    9, 2, 25, 0, 1, 1, 0, 0x80, 50,
    9, 4, 0, 0, 1, 9, 0, 0, 0,
    7, 5, HUB_EP, 3, 1, 0, 12,
};
static const uint8_t child_configuration[] = {
    9, 2, 18, 0, 1, 1, 0, 0x80, 0,
    9, 4, 0, 0, 0, 0xff, 0, 0, 0,
};
static const uint8_t hub_descriptor[] = {
    // Individual logical port power, no overcurrent sensing, 10ms power-good.
    // Both embedded vendor children are non-removable; USB 1.1 full-speed hub.
    9, 0x29, PORT_COUNT, 0x11, 0, 5, 100, 0x06, 0xff,
};
static const tusb_desc_endpoint_t hub_endpoint = {
    .bLength = 7,
    .bDescriptorType = TUSB_DESC_ENDPOINT,
    .bEndpointAddress = HUB_EP,
    .bmAttributes = { .xfer = TUSB_XFER_INTERRUPT },
    .wMaxPacketSize = 1,
    .bInterval = 12,
};

static void put16(uint8_t* out, uint16_t value) {
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
}

static void put32(uint8_t* out, uint32_t value) {
    put16(out, (uint16_t)value);
    put16(out + 2, (uint16_t)(value >> 16));
}

static void publish_addresses(void) {
    probe_router_publish(addresses, default_slot);
}

static void stall_control(void) {
    control.stage = CTRL_STALLED;
    control.action = ACTION_NONE;
    dcd_edpt_stall(RHPORT, EP0_OUT);
    dcd_edpt_stall(RHPORT, EP0_IN);
}

static bool queue_control(uint8_t endpoint, uint8_t* data, uint16_t length) {
    if (dcd_edpt_xfer(RHPORT, endpoint, data, length)) return true;
    stall_control();
    return false;
}

static void status_in(control_action action) {
    control.action = action;
    control.stage = CTRL_STATUS_IN;
    queue_control(EP0_IN, control_out, 0);
}

static void next_control_packet(void) {
    uint16_t remaining = (uint16_t)(control.length - control.sent);
    control.packet_length = remaining > EP0_SIZE ? EP0_SIZE : remaining;
    if (remaining == 0) control.need_zlp = false;
    control.stage = CTRL_DATA_IN;
    queue_control(EP0_IN, control_data + control.sent, control.packet_length);
}

static void reply_data(uint16_t length) {
    control.length = length < control.request.wLength ? length : control.request.wLength;
    control.sent = 0;
    control.need_zlp = length < control.request.wLength && (length % EP0_SIZE) == 0;
    if (control.request.wLength == 0) {
        control.stage = CTRL_STATUS_OUT;
        queue_control(EP0_OUT, control_out, 0);
    } else {
        next_control_packet();
    }
}

static void reply_copy(const uint8_t* data, uint16_t length) {
    memcpy(control_data, data, length);
    reply_data(length);
}

static void reply_word(uint16_t value, uint16_t length) {
    put16(control_data, value);
    reply_data(length);
}

static uint8_t changed_ports(void) {
    uint8_t bitmap = 0;
    for (unsigned i = 0; i < PORT_COUNT; ++i) {
        if (ports[i].change) bitmap |= (uint8_t)(1u << (i + 1));
    }
    return bitmap;
}

static void arm_interrupt(void) {
    if (!interrupt_open || interrupt_pending || interrupt_halted) return;
    interrupt_bitmap = changed_ports();
    if (!interrupt_bitmap) return; // NAK until a hub/port change exists.
    interrupt_pending = dcd_edpt_xfer(RHPORT, HUB_EP, &interrupt_bitmap, 1);
    if (!interrupt_pending) failed = true;
}

static void close_interrupt(void) {
    ++endpoint_epoch;
    dcd_edpt_close_all(RHPORT);
    interrupt_open = false;
    interrupt_pending = false;
    interrupt_halted = false;
}

static void open_interrupt(void) {
    close_interrupt();
    interrupt_open = dcd_edpt_open(RHPORT, &hub_endpoint);
    if (!interrupt_open) failed = true;
}

static void forget_child(unsigned port) {
    uint8_t slot = (uint8_t)(port + 1);
    addresses[slot] = PROBE_ROUTER_UNASSIGNED;
    configurations[slot] = 0;
    if (default_slot == slot) default_slot = PROBE_ROUTER_UNASSIGNED;
}

static void reset_bus_state(void) {
    probe_router_enable(false);
    routing_enabled = false;
    correlated_setups = 0;
    addresses[0] = 0;
    addresses[1] = PROBE_ROUTER_UNASSIGNED;
    addresses[2] = PROBE_ROUTER_UNASSIGNED;
    default_slot = 0;
    memset(configurations, 0, sizeof(configurations));
    memset(ports, 0, sizeof(ports));
    memset(&control, 0, sizeof(control));
    interrupt_open = false;
    interrupt_pending = false;
    interrupt_halted = false;
    ++endpoint_epoch;
    publish_addresses();
    usb_hw->dev_addr_ctrl = 0;
}

static void fill_stats(void) {
    probe_router_stats router;
    probe_router_snapshot(&router);
    const uint32_t words[32] = {
        0x42554850u, 3u, system_clock_hz, routing_enabled,
        addresses[0], addresses[1], addresses[2], default_slot,
        setup_count[0], setup_count[1], setup_count[2], bad_setup_owner,
        router.ready, router.sops, router.sync_ok, router.valid_tokens,
        router.valid_setups, router.crc_errors, router.late_samples,
        router.retargets, router.address_hits[0], router.address_hits[1],
        router.address_hits[2], router.cycles_per_bit,
        router.last_raw[0], router.last_raw[1], router.last_raw[2],
        router.last_raw_count, router.last_raw_eop, router.last_raw_late,
        usb_hw->phy_direct, correlated_setups,
    };
    for (unsigned i = 0; i < 32; ++i) put32(control_data + 4 * i, words[i]);
}

static bool get_descriptor(void) {
    const tusb_control_request_t* request = &control.request;
    uint8_t type = (uint8_t)(request->wValue >> 8);
    uint8_t index = (uint8_t)request->wValue;
    if (type == TUSB_DESC_DEVICE && index == 0 && request->wIndex == 0) {
        uint8_t descriptor[] = {
            18, 1, 0x10, 0x01, 0, 0, 0, EP0_SIZE,
            0x09, 0x12, 0, 0, 0x00, 0x01, 1, 2, 3, 1,
        };
        descriptor[4] = control.owner == 0 ? 9 : 0;
        descriptor[10] = (uint8_t)(control.owner + 1);
        reply_copy(descriptor, sizeof(descriptor));
        return true;
    }
    if (type == TUSB_DESC_CONFIGURATION && index == 0 && request->wIndex == 0) {
        if (control.owner == 0) reply_copy(hub_configuration, sizeof(hub_configuration));
        else reply_copy(child_configuration, sizeof(child_configuration));
        return true;
    }
    if (type != TUSB_DESC_STRING) return false;
    if (index == 0 && request->wIndex == 0) {
        static const uint8_t languages[] = {4, 3, 0x09, 0x04};
        reply_copy(languages, sizeof(languages));
        return true;
    }
    if (request->wIndex != 0x0409) return false;
    const char* text;
    if (index == 1) text = "Native USB capability probe";
    else if (index == 2) {
        static const char* const products[] = {
            "RP2350 native hub probe",
            "RP2350 vendor probe child 1",
            "RP2350 vendor probe child 2",
        };
        text = products[control.owner];
    } else if (index == 3) {
        static const char* const serials[] = {"PHUB-ROOT", "PHUB-CHILD1", "PHUB-CHILD2"};
        text = serials[control.owner];
    } else return false;
    uint16_t length = (uint16_t)strlen(text);
    control_data[0] = (uint8_t)(2 + 2 * length);
    control_data[1] = TUSB_DESC_STRING;
    for (uint16_t i = 0; i < length; ++i) put16(control_data + 2 + 2 * i, (uint8_t)text[i]);
    reply_data((uint16_t)(2 + 2 * length));
    return true;
}

static bool endpoint_exists(uint16_t index) {
    return index == EP0_OUT || index == EP0_IN ||
        (index == HUB_EP && control.owner == 0 && configurations[0] == 1);
}

static bool standard_request(void) {
    const tusb_control_request_t* request = &control.request;
    uint8_t slot = control.owner;
    switch (request->bRequest) {
        case TUSB_REQ_GET_DESCRIPTOR:
            return request->bmRequestType == 0x80 && get_descriptor();
        case TUSB_REQ_SET_ADDRESS:
            if (request->bmRequestType != 0 || request->wValue > 127 ||
                request->wIndex || request->wLength || configurations[slot]) return false;
            if (request->wValue == 0 && default_slot != PROBE_ROUTER_UNASSIGNED && default_slot != slot)
                return false;
            for (unsigned i = 0; i < PROBE_ROUTER_SLOTS; ++i) {
                if (i != slot && addresses[i] == request->wValue) return false;
            }
            status_in(ACTION_ADDRESS);
            return true;
        case TUSB_REQ_GET_CONFIGURATION:
            if (request->bmRequestType != 0x80 || request->wValue || request->wIndex || request->wLength != 1)
                return false;
            reply_word(configurations[slot], 1);
            return true;
        case TUSB_REQ_SET_CONFIGURATION:
            if (request->bmRequestType != 0 || request->wValue > 1 || request->wIndex || request->wLength ||
                addresses[slot] == 0 || addresses[slot] == PROBE_ROUTER_UNASSIGNED) return false;
            status_in(ACTION_CONFIGURATION);
            return true;
        case TUSB_REQ_GET_STATUS:
            if (request->wValue || request->wLength != 2) return false;
            if (request->bmRequestType == 0x80 && request->wIndex == 0) {
                reply_word(0, 2); // Bus powered; no remote wakeup capability.
                return true;
            }
            if (request->bmRequestType == 0x81 && request->wIndex == 0 && configurations[slot]) {
                reply_word(0, 2);
                return true;
            }
            if (request->bmRequestType == 0x82 && endpoint_exists(request->wIndex)) {
                reply_word(request->wIndex == HUB_EP && interrupt_halted ? 1 : 0, 2);
                return true;
            }
            return false;
        case TUSB_REQ_CLEAR_FEATURE:
        case TUSB_REQ_SET_FEATURE:
            if (request->bmRequestType != 0x02 || request->wValue != 0 || request->wIndex != HUB_EP ||
                request->wLength || slot != 0 || !configurations[0]) return false;
            status_in(request->bRequest == TUSB_REQ_SET_FEATURE ? ACTION_HALT : ACTION_CLEAR_HALT);
            return true;
        case TUSB_REQ_GET_INTERFACE:
            if (request->bmRequestType != 0x81 || request->wValue || request->wIndex ||
                request->wLength != 1 || !configurations[slot]) return false;
            reply_word(0, 1);
            return true;
        case TUSB_REQ_SET_INTERFACE:
            if (request->bmRequestType != 0x01 || request->wValue || request->wIndex ||
                request->wLength || !configurations[slot]) return false;
            status_in(ACTION_INTERFACE);
            return true;
        default:
            return false;
    }
}

static bool hub_request(void) {
    const tusb_control_request_t* request = &control.request;
    if (control.owner != 0) return false;
    if (request->bmRequestType == 0xa0 && request->bRequest == TUSB_REQ_GET_DESCRIPTOR &&
        request->wValue == 0x2900 && request->wIndex == 0) {
        reply_copy(hub_descriptor, sizeof(hub_descriptor));
        return true;
    }
    if (!configurations[0]) return false;
    if (request->bmRequestType == 0xa0 && request->bRequest == TUSB_REQ_GET_STATUS &&
        request->wValue == 0 && request->wIndex == 0 && request->wLength == 4) {
        put32(control_data, 0); // No local-power loss or overcurrent changes.
        reply_data(4);
        return true;
    }
    if (request->bmRequestType == 0x20 && request->bRequest == TUSB_REQ_CLEAR_FEATURE &&
        request->wValue <= 1 && request->wIndex == 0 && request->wLength == 0) {
        status_in(ACTION_NONE); // Both supported hub change flags are already clear.
        return true;
    }
    if (request->wIndex < 1 || request->wIndex > PORT_COUNT) return false;
    hub_port* port = &ports[request->wIndex - 1];
    if (request->bmRequestType == 0xa3 && request->bRequest == TUSB_REQ_GET_STATUS &&
        request->wValue == 0 && request->wLength == 4) {
        put16(control_data, port->status);
        put16(control_data + 2, port->change);
        reply_data(4);
        return true;
    }
    if (request->bmRequestType != 0x23 || request->wLength) return false;
    if (request->bRequest == TUSB_REQ_SET_FEATURE) {
        switch (request->wValue) {
            case FEATURE_PORT_POWER:
                break;
            case FEATURE_PORT_RESET:
                if (!routing_enabled || (port->status & (PORT_CONNECTION | PORT_POWER)) !=
                    (PORT_CONNECTION | PORT_POWER)) return false;
                // The one physical SIE cannot own two simultaneous default addresses.
                if (default_slot != PROBE_ROUTER_UNASSIGNED && default_slot != request->wIndex) return false;
                break;
            case FEATURE_PORT_SUSPEND:
                if ((port->status & (PORT_CONNECTION | PORT_ENABLE | PORT_POWER | PORT_RESET)) !=
                    (PORT_CONNECTION | PORT_ENABLE | PORT_POWER)) return false;
                break;
            default:
                return false;
        }
        status_in(ACTION_PORT_SET);
        return true;
    }
    if (request->bRequest == TUSB_REQ_CLEAR_FEATURE) {
        switch (request->wValue) {
            case FEATURE_PORT_POWER:
            case FEATURE_PORT_ENABLE:
            case FEATURE_PORT_SUSPEND:
            case FEATURE_C_CONNECTION:
            case FEATURE_C_ENABLE:
            case FEATURE_C_SUSPEND:
            case FEATURE_C_OVERCURRENT:
            case FEATURE_C_RESET:
                status_in(ACTION_PORT_CLEAR);
                return true;
            default:
                return false;
        }
    }
    return false;
}

static bool vendor_request(void) {
    const tusb_control_request_t* request = &control.request;
    if (request->wIndex) return false;
    if (request->bmRequestType == 0xc0 && request->bRequest == 0x5a &&
        request->wValue == 0 && request->wLength == 128) {
        fill_stats();
        control.action = ACTION_KEEPALIVE;
        reply_data(128);
        return true;
    }
    if (request->bmRequestType != 0x40 || request->wLength) return false;
    if (request->bRequest == 0x5b && request->wValue == 1 && control.owner == 0) {
        probe_router_stats router;
        probe_router_snapshot(&router);
        if (!router.ready || correlated_setups < 20) return false;
        status_in(ACTION_ARM);
        return true;
    }
    if (request->bRequest == 0x5c && request->wValue == 0) {
        status_in(ACTION_REBOOT);
        return true;
    }
    if (request->bRequest == 0x5d && !routing_enabled &&
        probe_router_set_phase(request->wValue)) {
        status_in(ACTION_NONE);
        return true;
    }
    return false;
}

static void handle_setup(const queued_event* queued) {
    // A newer SETUP has already aborted this one's hardware transfer.
    if (queued->generation != event_generation) return;
    memset(&control, 0, sizeof(control));
    control.request = queued->event.setup_received;
    control.generation = queued->generation;
    control.owner = routing_enabled ? queued->setup_slot : 0;
    if (routing_enabled && (control.owner >= PROBE_ROUTER_SLOTS ||
        (addresses[control.owner] == PROBE_ROUTER_UNASSIGNED && default_slot != control.owner))) {
        ++bad_setup_owner;
        stall_control();
        return;
    }
    ++setup_count[control.owner];
    uint8_t type = control.request.bmRequestType & 0x60;
    bool supported = type == 0 ? standard_request() :
        type == 0x20 ? hub_request() : type == 0x40 ? vendor_request() : false;
    if (!supported) stall_control();
}

static void apply_port_feature(bool set) {
    unsigned index = control.request.wIndex - 1;
    hub_port* port = &ports[index];
    uint16_t feature = control.request.wValue;
    uint32_t now = time_us_32();
    if (set) {
        if (feature == FEATURE_PORT_POWER) {
            port->status |= PORT_POWER;
            if (routing_enabled && !(port->status & PORT_CONNECTION)) {
                port->status |= PORT_CONNECTION;
                port->change |= C_CONNECTION;
            }
        } else if (feature == FEATURE_PORT_RESET) {
            forget_child(index);
            port->status = (uint16_t)((port->status | PORT_RESET) & ~(PORT_ENABLE | PORT_SUSPEND));
            port->resetting = true;
            port->resuming = false;
            port->reset_deadline = now + 10000u;
            publish_addresses();
        } else if (feature == FEATURE_PORT_SUSPEND) {
            port->status |= PORT_SUSPEND;
            port->resuming = false;
        }
        return;
    }
    if (feature >= FEATURE_C_CONNECTION && feature <= FEATURE_C_RESET) {
        port->change &= (uint16_t)~(1u << (feature - FEATURE_C_CONNECTION));
    } else if (feature == FEATURE_PORT_ENABLE) {
        port->status &= (uint16_t)~(PORT_ENABLE | PORT_SUSPEND | PORT_RESET);
        port->resetting = false;
        port->resuming = false;
        forget_child(index);
        publish_addresses();
    } else if (feature == FEATURE_PORT_POWER) {
        if (port->status & PORT_CONNECTION) port->change |= C_CONNECTION;
        port->status = 0;
        port->resetting = false;
        port->resuming = false;
        forget_child(index);
        publish_addresses();
    } else if (feature == FEATURE_PORT_SUSPEND && (port->status & PORT_SUSPEND)) {
        port->resuming = true;
        port->resume_deadline = now + 20000u;
    }
}

static void complete_control(void) {
    uint8_t owner = control.owner;
    control_action action = control.action;
    control.stage = CTRL_IDLE;
    control.action = ACTION_NONE;
    switch (action) {
        case ACTION_ADDRESS:
            addresses[owner] = (uint8_t)control.request.wValue;
            if (addresses[owner] == 0) default_slot = owner;
            else if (default_slot == owner) default_slot = PROBE_ROUTER_UNASSIGNED;
            publish_addresses();
            // Once routing is active, C1 is the sole address-register writer.
            // It selects the logical address from each token, after this ACK.
            // This prevents a C0 SET_ADDRESS completion changing the register
            // between another token's acceptance and its SETUP interrupt.
            if (!routing_enabled)
                dcd_edpt0_status_complete(RHPORT, &control.request);
            break;
        case ACTION_CONFIGURATION:
            configurations[owner] = (uint8_t)control.request.wValue;
            if (owner == 0) {
                if (configurations[0]) open_interrupt();
                else {
                    close_interrupt();
                    probe_router_enable(false);
                    routing_enabled = false;
                    memset(ports, 0, sizeof(ports));
                    forget_child(0);
                    forget_child(1);
                    publish_addresses();
                    usb_hw->dev_addr_ctrl = addresses[0];
                }
            }
            break;
        case ACTION_INTERFACE:
            if (owner == 0) open_interrupt();
            break;
        case ACTION_HALT:
            ++endpoint_epoch;
            interrupt_halted = true;
            interrupt_pending = false;
            dcd_edpt_stall(RHPORT, HUB_EP);
            break;
        case ACTION_CLEAR_HALT:
            ++endpoint_epoch;
            interrupt_pending = false;
            interrupt_halted = false;
            // Reopening also cancels a previously queued interrupt safely and
            // resets DATA0; no child has a noncontrol endpoint to disturb.
            open_interrupt();
            break;
        case ACTION_PORT_SET:
            apply_port_feature(true);
            break;
        case ACTION_PORT_CLEAR:
            apply_port_feature(false);
            break;
        case ACTION_KEEPALIVE:
            watchdog_update();
            break;
        case ACTION_ARM:
            if (!routing_enabled) {
                routing_enabled = true;
                publish_addresses();
                probe_router_enable(true);
                for (unsigned i = 0; i < PORT_COUNT; ++i) {
                    ports[i].status |= PORT_CONNECTION;
                    ports[i].change |= C_CONNECTION;
                }
            }
            break;
        case ACTION_REBOOT:
            reboot_pending = true;
            break;
        case ACTION_NONE:
            break;
    }
}

static void handle_transfer(const queued_event* queued) {
    const dcd_event_t* event = &queued->event;
    uint8_t endpoint = event->xfer_complete.ep_addr;
    if (endpoint == HUB_EP) {
        if (queued->endpoint_epoch != endpoint_epoch) return;
        interrupt_pending = false;
        if (event->xfer_complete.result != XFER_RESULT_SUCCESS || event->xfer_complete.len != 1) failed = true;
        return;
    }
    if ((endpoint != EP0_IN && endpoint != EP0_OUT) || queued->generation != control.generation ||
        control.stage == CTRL_IDLE || control.stage == CTRL_STALLED) return;
    if (event->xfer_complete.result != XFER_RESULT_SUCCESS) {
        stall_control();
        return;
    }
    if ((control.stage == CTRL_STATUS_IN && endpoint == EP0_IN) ||
        (control.stage == CTRL_STATUS_OUT && endpoint == EP0_OUT)) {
        if (event->xfer_complete.len == 0) complete_control();
        else stall_control();
        return;
    }
    if (queued->generation != event_generation) return;
    if (control.stage != CTRL_DATA_IN || endpoint != EP0_IN ||
        event->xfer_complete.len != control.packet_length) {
        stall_control();
        return;
    }
    control.sent = (uint16_t)(control.sent + control.packet_length);
    if (control.sent < control.length || control.need_zlp) next_control_packet();
    else {
        control.stage = CTRL_STATUS_OUT;
        queue_control(EP0_OUT, control_out, 0);
    }
}

void dcd_event_handler(dcd_event_t const* event, bool in_isr) {
    (void)in_isr;
    if (event->rhport != RHPORT) return;
    if (event->event_id != DCD_EVENT_SETUP_RECEIVED && event->event_id != DCD_EVENT_XFER_COMPLETE &&
        event->event_id != DCD_EVENT_BUS_RESET && event->event_id != DCD_EVENT_UNPLUGGED) return;
    if (event->event_id == DCD_EVENT_SETUP_RECEIVED || event->event_id == DCD_EVENT_BUS_RESET ||
        event->event_id == DCD_EVENT_UNPLUGGED) ++event_generation;
    uint32_t head = event_head;
    uint32_t next = (head + 1u) % EVENT_CAPACITY;
    if (next == event_tail) {
        event_overflow = true;
        return;
    }
    queued_event* queued = &events[head];
    queued->event = *event;
    queued->generation = event_generation;
    queued->endpoint_epoch = endpoint_epoch;
    queued->setup_slot = PROBE_ROUTER_UNASSIGNED;
    if (event->event_id == DCD_EVENT_SETUP_RECEIVED) {
        // C1 does not change the address while SETUP_REC is pending; C0 does
        // not write it in routed mode. This is the hardware-accepted address,
        // not a fallback inferred from whichever header we last sampled.
        const uint8_t hw_address = usb_hw->dev_addr_ctrl & 0x7fu;
        if (hw_address == 0) {
            queued->setup_slot = default_slot;
        } else {
            for (uint8_t slot = 0; slot < PROBE_ROUTER_SLOTS; ++slot) {
                if (addresses[slot] == hw_address) {
                    queued->setup_slot = slot;
                    break;
                }
            }
        }
        uint32_t sequence;
        const uint8_t candidate = probe_router_setup_slot(&sequence);
        if (candidate < PROBE_ROUTER_SLOTS && candidate == queued->setup_slot &&
            sequence != observed_setup_sequence) ++correlated_setups;
        observed_setup_sequence = sequence;
    }
    __dmb();
    event_head = next;
}

static void port_task(uint32_t now) {
    for (unsigned i = 0; i < PORT_COUNT; ++i) {
        hub_port* port = &ports[i];
        if (port->resetting && (int32_t)(now - port->reset_deadline) >= 0) {
            port->resetting = false;
            port->status &= (uint16_t)~PORT_RESET;
            if (default_slot != PROBE_ROUTER_UNASSIGNED && default_slot != i + 1) {
                // Concurrent default-address resets cannot be represented honestly.
                failed = true;
                return;
            }
            port->status |= PORT_ENABLE;
            port->change |= C_RESET;
            addresses[i + 1] = 0;
            default_slot = (uint8_t)(i + 1);
            publish_addresses();
        }
        if (port->resuming && (int32_t)(now - port->resume_deadline) >= 0) {
            port->resuming = false;
            port->status &= (uint16_t)~PORT_SUSPEND;
            port->change |= C_SUSPEND;
        }
    }
}

static void diagnostic_task(uint32_t now) {
    static uint32_t last_report;
    static char line[384];
    static uint16_t length;
    static uint16_t sent;
    if ((uint32_t)(now - last_report) >= 1000000u && sent == length) {
        last_report = now;
        probe_router_stats router;
        probe_router_snapshot(&router);
        int count = snprintf(line, sizeof(line),
            "[PHUB] route=%u addr=%u,%u,%u default=%u setup=%" PRIu32 ",%" PRIu32 ",%" PRIu32
            " bad=%" PRIu32 " ready=%" PRIu32 " sop=%" PRIu32 " sync=%" PRIu32
            " token=%" PRIu32 " crc=%" PRIu32 " late=%" PRIu32 " retarget=%" PRIu32
            " hits=%" PRIu32 ",%" PRIu32 ",%" PRIu32 " overflow=%u failed=%u"
            " raw=%08" PRIx32 "/%08" PRIx32 " n=%" PRIu32 " eop=%" PRIu32 "\r\n",
            routing_enabled, addresses[0], addresses[1], addresses[2], default_slot,
            setup_count[0], setup_count[1], setup_count[2], bad_setup_owner,
            router.ready, router.sops, router.sync_ok, router.valid_tokens, router.crc_errors,
            router.late_samples, router.retargets, router.address_hits[0], router.address_hits[1],
            router.address_hits[2], event_overflow, failed,
            router.last_raw[0], router.last_raw[1], router.last_raw_count, router.last_raw_eop);
        length = count < 0 ? 0 : (uint16_t)((unsigned)count < sizeof(line) ? (unsigned)count : sizeof(line) - 1);
        sent = 0;
    }
    // No blocking stdio writes: fill only available UART FIFO positions. USB
    // event service continues while the 115200-baud diagnostic line drains.
    for (unsigned budget = 0; sent < length && budget < 32 && uart_is_writable(uart_default); ++budget)
        uart_get_hw(uart_default)->dr = (uint8_t)line[sent++];
}

void probe_hub_init(void) {
    system_clock_hz = clock_get_hz(clk_sys);
    reset_bus_state();
    // Enabling, bus resets, ordinary enumeration, and UART never feed this.
    watchdog_enable(8000, false);
    const tusb_rhport_init_t init = { .role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL };
    if (!dcd_init(RHPORT, &init)) failed = true;
    dcd_int_enable(RHPORT);
}

void probe_hub_task(void) {
    if (!failed) {
        for (unsigned count = 0; count < EVENT_CAPACITY; ++count) {
            dcd_int_disable(RHPORT);
            if (event_overflow) failed = true;
            if (failed || event_tail == event_head) {
                dcd_int_enable(RHPORT);
                break;
            }
            __dmb();
            queued_event queued = events[event_tail];
            event_tail = (event_tail + 1u) % EVENT_CAPACITY;
            switch (queued.event.event_id) {
                case DCD_EVENT_BUS_RESET:
                case DCD_EVENT_UNPLUGGED:
                    reset_bus_state();
                    break;
                case DCD_EVENT_SETUP_RECEIVED:
                    handle_setup(&queued);
                    break;
                case DCD_EVENT_XFER_COMPLETE:
                    handle_transfer(&queued);
                    break;
                default:
                    break;
            }
            dcd_int_enable(RHPORT);
            if (failed || reboot_pending) break;
        }
    }
    uint32_t now = time_us_32();
    dcd_int_disable(RHPORT);
    if (!failed && !reboot_pending) {
        port_task(now);
        if (!failed) arm_interrupt();
    }
    if (failed) {
        probe_router_enable(false);
        routing_enabled = false;
        dcd_disconnect(RHPORT);
    }
    dcd_int_enable(RHPORT);
    if (reboot_pending) {
        // This is reached only after the REBOOT request's status IN was ACKed.
        watchdog_reboot(0, 0, 10);
        reboot_pending = false;
        failed = true;
    }
    diagnostic_task(now);
}
