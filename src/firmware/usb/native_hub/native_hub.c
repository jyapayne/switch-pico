// Native RP2350 SIE transport for an embedded hub and two Joy-Con devices.
// Core1 selects address, endpoint controls and receive buffers. Core0 publishes
// transmit buffers and owns protocols/IRQ completions; IN endpoints NAK until ready.
#include "native_hub.h"
#include "router.h"
#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/resets.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/usb.h"
#include "hardware/structs/usb_dpram.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"

#ifndef NATIVE_HUB_SAMPLE_PHASE
#define NATIVE_HUB_SAMPLE_PHASE 4u
#endif
#define DEVICES 3u
#define CHANNELS 6u
#define PACKET 64u
#define EVENTS 64u
#define NONE 255u
#define CONNECT 1u
#define ENABLE 2u
#define SUSPEND 4u
#define RESET 16u
#define POWER 256u
#define C_CONNECT 1u
#define C_ENABLE 2u
#define C_SUSPEND 4u
#define C_RESET 16u

typedef enum { IDLE, DATA_IN, DATA_OUT, STATUS_IN, STATUS_OUT, STALLED } stage_t;
typedef enum { NO_ACTION, ADDRESS, CONFIGURE, PORT_SET, PORT_CLEAR, HID_SET_REPORT,
               HID_IDLE, HID_PROTOCOL, ENDPOINT_HALT, ENDPOINT_CLEAR } action_t;
typedef struct {
    uint8_t data[128];
    uint16_t length, sent, packet_length;
    bool busy, flush, zlp;
    uint8_t next_pid;
    bool halted;
} endpoint_t;
typedef struct {
    tusb_control_request_t request;
    uint8_t data[1024];
    uint8_t* external;
    uint16_t length, position, packet_length;
    stage_t stage;
    action_t action;
    bool zlp, vendor;
    uint32_t generation;
} control_t;
typedef struct {
    uint32_t buffers[CHANNELS];
    uint32_t endpoint_controls[4];
    uint32_t ep0_image[16];
    endpoint_t ep[CHANNELS];
    control_t control;
    uint32_t generation;
    uint32_t reset_generation;
    // Control SETUP aborts only EP0, never unrelated HID/vendor completions.
    uint32_t endpoint_generation[CHANNELS];
    uint8_t configuration, idle_rate, protocol;
} device_t;
typedef struct { uint16_t status, change; uint32_t deadline; } port_t;
typedef struct {
    uint8_t device, channel, kind;
    uint16_t length;
    uint32_t generation;
    uint32_t reset_generation;
    uint8_t data[64];
} event_t;

static device_t devices[DEVICES];
static port_t ports[2];
static uint8_t addresses[DEVICES];
static uint8_t default_device;
static volatile uint8_t active_device;
static volatile bool bank_restore_pending;
static spin_lock_t* bank_lock;
static event_t events[EVENTS];
static volatile uint32_t event_head, event_tail;
static volatile bool failed;
static volatile bool bus_suspended;
static uint32_t startup_time;
static bool root_configured_once;
static uint32_t hub_endpoint_control;
static bool started;
static uint32_t setup_count[DEVICES], input_count[DEVICES], output_count[DEVICES];
static uint32_t switches, missed_switches, slow_switches;
static uint32_t minimum_lateness = UINT32_MAX, maximum_lateness;
static uint32_t token_hits[DEVICES], missed_lock, blocked_buffers, blocked_sie, root_naks;
static uint16_t root_string[64];
static char root_serial[48];

static const uint8_t hub_device[] = {
    18,1,0x10,1,9,0,0,64,0x7e,5,0x68,0x20,0,1,1,2,3,1
};
static const uint8_t hub_configuration[] = {
    // Grip-style self-powered topology; the Bluetooth children use their own
    // batteries. Do not advertise unimplemented high-speed TT/remote wake.
    9,2,25,0,1,1,0,0xc0,250, 9,4,0,0,1,9,0,0,0, 7,5,0x8f,3,1,0,12
};
static const uint8_t hub_descriptor[] = {9,0x29,2,0x11,0,5,100,6,255};

static inline volatile uint32_t* buffer_regs(void) {
    return (volatile uint32_t*)&usb_dpram->ep_buf_ctrl[0];
}
static inline volatile uint32_t* endpoint_regs(void) {
    return (volatile uint32_t*)&usb_dpram->ep_ctrl[0];
}
static inline uint32_t data_offset(uint8_t device, uint8_t channel) {
    return 0x180u + ((uint32_t)device * 4u + channel - 2u) * PACKET;
}
static inline unsigned physical_channel(uint8_t slot, uint8_t channel) {
    return slot == 0 && channel == 2 ? 30u : channel;
}
static inline unsigned logical_channel(uint8_t slot, uint16_t endpoint) {
    if (slot == 0 && endpoint == 0x8f) return 2;
    return (endpoint & 15u)*2u + ((endpoint & 0x80u) ? 0u : 1u);
}
static inline uint8_t* packet_buffer(uint8_t device, uint8_t channel) {
    return channel < 2 ? usb_dpram->ep0_buf_a :
        (uint8_t*)USBCTRL_DPRAM_BASE + data_offset(device, channel);
}
static __force_inline void copy_from_usb(uint8_t* to, const volatile uint8_t* from, uint16_t length) {
    // Every DPRAM packet starts on a word boundary. Keep only the tail bytewise:
    // unrestricted memcpy may generate an unaligned access for a short tail.
    const volatile uint32_t* words = (const volatile uint32_t*)from;
    while (length >= 4) {
        uint32_t word = *words++;
        memcpy(to,&word,4);
        to += 4;
        length -= 4;
    }
    from = (const volatile uint8_t*)words;
    while (length--) *to++ = *from++;
}
static __force_inline void copy_to_usb(volatile uint8_t* to, const uint8_t* from, uint16_t length) {
    volatile uint32_t* words = (volatile uint32_t*)to;
    while (length >= 4) {
        uint32_t word;
        memcpy(&word,from,4);
        *words++ = word;
        from += 4;
        length -= 4;
    }
    to = (volatile uint8_t*)words;
    while (length--) *to++ = *from++;
}
static __force_inline void buffer_settle(void) {
    // Same minimum metadata-to-AVAIL interval as the Pico TinyUSB DCD.
    __asm volatile (".rept 12\n nop\n .endr" ::: "memory");
}
static __force_inline void set_buffer(uint8_t device, uint8_t channel, uint32_t value) {
    devices[device].buffers[channel] = value;
    __dmb();
    if ((active_device == device && (!bank_restore_pending || channel != 0)) ||
        (device == 0 && channel == 2)) {
        unsigned physical = physical_channel(device,channel);
        buffer_regs()[physical] = value & ~USB_BUF_CTRL_AVAIL;
        if (value & USB_BUF_CTRL_AVAIL) buffer_settle();
        buffer_regs()[physical] = value;
    }
}

// SRAM receiver only. No bank changes while a hardware completion is pending.
bool __not_in_flash_func(native_hub_select_device)(uint8_t address, uint8_t owner, uint32_t cutoff) {
    if (owner >= DEVICES || bank_lock == NULL) return false;
    ++token_hits[owner];
    if (active_device == owner && usb_hw->dev_addr_ctrl == address) return true;
    if (!spin_try_lock_unsafe(bank_lock)) { ++missed_switches; ++missed_lock; return false; }
    __dmb();
    if ((usb_hw->sie_status & USB_SIE_STATUS_SETUP_REC_BITS) || usb_hw->buf_status ||
        (int32_t)(sio_hw->mtime - cutoff) >= 0) {
        ++missed_switches;
        blocked_buffers = usb_hw->buf_status;
        blocked_sie = usb_hw->sie_status;
        spin_unlock_unsafe(bank_lock);
        return false;
    }
    if (active_device != owner) {
        const device_t* restrict incoming = &devices[owner];
        volatile uint32_t* buffers = (volatile uint32_t*)&usb_dpram->ep_buf_ctrl[0];
        volatile uint32_t* controls = (volatile uint32_t*)&usb_dpram->ep_ctrl[0];
        // Select metadata before accepting a token. Only EP0 IN needs a later
        // shared-buffer copy; other endpoints already have private DPRAM data.
        for (unsigned i = 0; i < CHANNELS; ++i) {
            uint32_t value = owner == 0 && i >= 2 ? 0 : incoming->buffers[i];
            buffers[i] = value & ~USB_BUF_CTRL_AVAIL;
        }
        usb_dpram->ep_ctrl[14].in = owner == 0 ? hub_endpoint_control : 0;
        for (unsigned i = 0; i < 4; ++i) controls[i] = incoming->endpoint_controls[i];
        buffer_settle();
        for (unsigned i = 1; i < CHANNELS; ++i)
            buffers[i] = owner == 0 && i >= 2 ? 0 : incoming->buffers[i];
        __dmb();
        usb_hw->dev_addr_ctrl = address;
        bank_restore_pending = true;
        active_device = owner;
    } else {
        usb_hw->dev_addr_ctrl = address;
    }
    __dmb();
    ++switches;
    int32_t lateness = (int32_t)(sio_hw->mtime - cutoff);
    if (lateness >= 0) {
        ++slow_switches;
        if ((uint32_t)lateness < minimum_lateness) minimum_lateness = (uint32_t)lateness;
        if ((uint32_t)lateness > maximum_lateness) maximum_lateness = (uint32_t)lateness;
    }
    spin_unlock_unsafe(bank_lock);
    return true;
}
// Section placement alone permits inlining into the flash-backed task. Keep
// bank-lock ownership independent of XIP instruction-cache refill latency.
static void __no_inline_not_in_flash_func(restore_selected_bank)(void) {
    if (!bank_restore_pending) return;
    uint32_t flags = spin_lock_blocking(bank_lock);
    if (!bank_restore_pending || (usb_hw->sie_status & USB_SIE_STATUS_SETUP_REC_BITS) ||
        (usb_hw->buf_status & 0x3fu)) {
        spin_unlock(bank_lock,flags);
        return;
    }
    uint8_t owner = active_device;
    const device_t* incoming = &devices[owner];
    volatile uint32_t* buffers = buffer_regs();
    if ((incoming->buffers[0] & USB_BUF_CTRL_FULL) &&
        (incoming->buffers[0] & USB_BUF_CTRL_LEN_MASK)) {
        volatile uint32_t* to = (volatile uint32_t*)usb_dpram->ep0_buf_a;
        const uint32_t* from = incoming->ep0_image;
        unsigned words = ((incoming->buffers[0] & USB_BUF_CTRL_LEN_MASK) + 3u) / 4u;
        for (unsigned i = 0; i < words; ++i) to[i] = from[i];
    }
    buffers[0] = incoming->buffers[0] & ~USB_BUF_CTRL_AVAIL;
    usb_hw->ep_stall_arm = ((incoming->buffers[0] & USB_BUF_CTRL_STALL) ? 1u : 0u) |
        ((incoming->buffers[1] & USB_BUF_CTRL_STALL) ? 2u : 0u);
    buffer_settle();
    buffers[0] = incoming->buffers[0];
    bank_restore_pending = false;
    spin_unlock(bank_lock,flags);
}


static void publish_addresses(void) { probe_router_publish(addresses, default_device); }
static uint8_t hardware_owner(void) {
    uint8_t address = usb_hw->dev_addr_ctrl & 127u;
    if (address == 0) return default_device;
    for (uint8_t i = 0; i < DEVICES; ++i) if (addresses[i] == address) return i;
    return NONE;
}
static __force_inline bool push_event(uint8_t device, uint8_t channel, uint8_t kind, uint16_t length,
                       const uint8_t* data) {
    uint32_t next = (event_head + 1u) % EVENTS;
    if (next == event_tail || length > 64) { failed = true; return false; }
    event_t* event = &events[event_head];
    event->device = device; event->channel = channel; event->kind = kind;
    event->length = length;
    event->generation = device < DEVICES ? (channel < 2 ? devices[device].generation :
        devices[device].endpoint_generation[channel]) : 0;
    event->reset_generation = device < DEVICES ? devices[device].reset_generation : 0;
    if (kind == 2 && (channel & 1u) && channel != 1) copy_from_usb(event->data,data,length);
    else if (length) memcpy(event->data,data,length);
    __dmb(); event_head = next;
    return true;
}

static void __not_in_flash_func(usb_interrupt)(void) {
    uint32_t flags = spin_lock_blocking(bank_lock);
    uint32_t status = usb_hw->ints;
    if (status & USB_INTS_BUS_RESET_BITS) {
        // Reset wins over stale transfers and setup snapshots.
        for (uint8_t i = 0; i < DEVICES; ++i) {
            ++devices[i].generation;
            ++devices[i].reset_generation;
            for (unsigned ch = 2; ch < CHANNELS; ++ch) ++devices[i].endpoint_generation[ch];
        }
        for (unsigned i = 0; i < CHANNELS; ++i) buffer_regs()[i] = 0;
        hw_clear_bits(&usb_hw->buf_status,usb_hw->buf_status);
        hw_clear_bits(&usb_hw->sie_status,USB_SIE_STATUS_BUS_RESET_BITS | USB_SIE_STATUS_SETUP_REC_BITS);
        push_event(0,0,3,0,NULL);
        spin_unlock(bank_lock, flags);
        return;
    }
    uint8_t owner = active_device;
    if (owner >= DEVICES) {
        failed = true;
        usb_hw->inte = 0;
        spin_unlock(bank_lock,flags);
        return;
    }
    uint32_t pending = usb_hw->buf_status;
    while (pending) {
        unsigned physical = (unsigned)__builtin_ctz(pending);
        uint32_t mask = 1u << physical;
        unsigned channel = physical == 30 ? 2u : physical;
        uint8_t completed_owner = physical == 30 ? 0u : owner;
        if (channel >= CHANNELS) { failed = true; hw_clear_bits(&usb_hw->buf_status,mask); pending &= ~mask; continue; }
        uint32_t value = buffer_regs()[physical];
        uint16_t length = value & USB_BUF_CTRL_LEN_MASK;
        if (length > PACKET) { failed = true; length = 0; }
        const uint8_t* data = (channel & 1u) ? packet_buffer(completed_owner,channel) :
            devices[completed_owner].ep[channel].data;
        uint32_t ep0_snapshot[PACKET / sizeof(uint32_t)];
        if (channel == 1 && length) {
            // EP0 storage is shared; other packet buffers belong to one device.
            copy_from_usb((uint8_t*)ep0_snapshot,data,length);
            data = (const uint8_t*)ep0_snapshot;
        }
        devices[completed_owner].ep[channel].next_pid ^= 1u;
        devices[completed_owner].buffers[channel] = 0;
        buffer_regs()[physical] = 0;
        hw_clear_bits(&usb_hw->buf_status,mask);
        pending &= ~mask;
        // Unarmed device-specific buffers and the TX shadow cannot be reused
        // until Core0 consumes this event. Copy them without blocking routing.
        spin_unlock(bank_lock,flags);
        push_event(completed_owner,(uint8_t)channel,2,length,data);
        if (!pending && !(status & (USB_INTS_SETUP_REQ_BITS | USB_INTS_DEV_SUSPEND_BITS |
                                    USB_INTS_DEV_RESUME_FROM_HOST_BITS))) return;
        flags = spin_lock_blocking(bank_lock);
    }
    if (status & USB_INTS_SETUP_REQ_BITS) {
        uint8_t actual_owner = hardware_owner();
        uint8_t setup[8];
        copy_from_usb(setup, usb_dpram->setup_packet, sizeof(setup));
        // A new SETUP revokes the preceding control transfer. Reclaim both
        // physical EP0 buffers through the controller's ownership handshake,
        // as the SDK DCD does, before changing their metadata or PID.
        hw_set_bits(&usb_hw->abort, 3u);
        unsigned abort_wait = 4096u;
        while ((usb_hw->abort_done & 3u) != 3u && --abort_wait) {}
        if ((usb_hw->abort_done & 3u) != 3u) {
            // Never acknowledge new work over a buffer the SIE still owns.
            failed = true;
            usb_hw->inte = 0;
            spin_unlock(bank_lock, flags);
            return;
        }
        if (actual_owner < DEVICES && actual_owner == owner) {
            ++devices[owner].generation;
            devices[owner].buffers[0] = devices[owner].buffers[1] =
                USB_BUF_CTRL_DATA1_PID | USB_BUF_CTRL_SEL;
            buffer_regs()[0] = buffer_regs()[1] = USB_BUF_CTRL_DATA1_PID | USB_BUF_CTRL_SEL;
            devices[owner].ep[0].next_pid = devices[owner].ep[1].next_pid = 1;
            push_event(owner,0,1,sizeof(setup),setup);
        } else {
            // No logical owner: never reinterpret it as another controller.
            hw_set_bits(&usb_hw->ep_stall_arm,3u);
            buffer_regs()[0] = buffer_regs()[1] = USB_BUF_CTRL_STALL;
        }
        hw_clear_bits(&usb_hw->abort_done, 3u);
        hw_clear_bits(&usb_hw->abort, 3u);
        hw_clear_bits(&usb_hw->sie_status,USB_SIE_STATUS_SETUP_REC_BITS);
    }
    if (status & USB_INTS_DEV_SUSPEND_BITS) {
        bus_suspended = true; hw_clear_bits(&usb_hw->sie_status,USB_SIE_STATUS_SUSPENDED_BITS);
    }
    if (status & USB_INTS_DEV_RESUME_FROM_HOST_BITS) {
        bus_suspended = false; hw_clear_bits(&usb_hw->sie_status,USB_SIE_STATUS_RESUME_BITS);
    }
    spin_unlock(bank_lock, flags);
}

static void stall(uint8_t slot) {
    uint32_t flags = spin_lock_blocking(bank_lock);
    if (devices[slot].control.generation != devices[slot].generation) {
        spin_unlock(bank_lock,flags);
        return;
    }
    devices[slot].control.stage = STALLED;
    devices[slot].control.action = NO_ACTION;
    if (active_device == slot) hw_set_bits(&usb_hw->ep_stall_arm,3u);
    set_buffer(slot,0,USB_BUF_CTRL_STALL); set_buffer(slot,1,USB_BUF_CTRL_STALL);
    spin_unlock(bank_lock, flags);
}
static void __no_inline_not_in_flash_func(arm_packet)(uint8_t slot, uint8_t channel, const uint8_t* data, uint16_t length) {
    endpoint_t* ep = &devices[slot].ep[channel];
    uint32_t generation = channel < 2 ? devices[slot].control.generation :
        devices[slot].endpoint_generation[channel];
    if (!(channel & 1u) && length) {
        // Private packet storage is unarmed until the metadata below is published.
        memcpy(ep->data,data,length);
        if (channel == 0) memcpy(devices[slot].ep0_image,data,length);
        else copy_to_usb(packet_buffer(slot,channel),data,length);
    }
    uint32_t flags = spin_lock_blocking(bank_lock);
    if (generation != (channel < 2 ? devices[slot].generation :
                       devices[slot].endpoint_generation[channel])) {
        spin_unlock(bank_lock,flags);
        return;
    }
    ep->packet_length = length;
    uint32_t value = USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_LAST | USB_BUF_CTRL_SEL |
        (ep->next_pid ? USB_BUF_CTRL_DATA1_PID : 0);
    if (!(channel & 1u)) {
        if (channel == 0 && active_device == slot && !bank_restore_pending)
            copy_to_usb(packet_buffer(slot,channel),data,length);
        value |= USB_BUF_CTRL_FULL | length;
    } else if (channel == 1) {
        control_t* c = &devices[slot].control;
        uint16_t remaining = c->stage == DATA_OUT ? c->length - c->position : 0;
        value |= remaining > PACKET ? PACKET : remaining;
    } else {
        value |= PACKET;
    }
    set_buffer(slot,channel,value);
    spin_unlock(bank_lock, flags);
}
static void control_next(uint8_t slot) {
    control_t* c = &devices[slot].control;
    uint16_t left = c->length - c->position;
    c->packet_length = left > PACKET ? PACKET : left;
    if (!left) c->zlp = false;
    c->stage = DATA_IN;
    if (slot == 0) {
        uint8_t preview[4] = {0};
        uint16_t preview_length = c->packet_length < 4 ? c->packet_length : 4;
        if (preview_length) memcpy(preview,c->data+c->position,preview_length);
        probe_debug_printf("[HUB_CTRL] arm g=%" PRIu32 " len=%u pid=%u data=%02x%02x%02x%02x\n",
                           c->generation, c->packet_length, devices[slot].ep[0].next_pid,
                           preview[0],preview[1],preview[2],preview[3]);
    }
    arm_packet(slot,0,c->data + c->position,c->packet_length);
}
static void reply(uint8_t slot, const void* data, uint16_t length) {
    control_t* c = &devices[slot].control;
    if (length > sizeof(c->data)) { stall(slot); return; }
    if (length && data != c->data) memcpy(c->data,data,length);
    c->length = length < c->request.wLength ? length : c->request.wLength;
    c->position = 0;
    c->zlp = length < c->request.wLength && length % PACKET == 0;
    if (!c->request.wLength) { c->stage = STATUS_OUT; arm_packet(slot,1,NULL,0); }
    else control_next(slot);
}
static void status_in(uint8_t slot, action_t action) {
    control_t* c = &devices[slot].control;
    c->action = action; c->stage = STATUS_IN;
    arm_packet(slot,0,NULL,0);
}
bool native_hub_control_xfer(uint8_t slot, const tusb_control_request_t* request,
                             void* buffer, uint16_t length) {
    if (slot >= DEVICES || request == NULL || (length && buffer == NULL)) return false;
    control_t* c = &devices[slot].control;
    if (c->generation != devices[slot].generation ||
        memcmp(request,&c->request,sizeof(*request)) != 0) return false;
    if (request->bmRequestType & 0x80) reply(slot,buffer,length);
    else if (!request->wLength) status_in(slot,NO_ACTION);
    else {
        if (length < request->wLength || request->wLength > sizeof(c->data)) return false;
        c->external = buffer; c->length = request->wLength; c->position = 0;
        c->stage = DATA_OUT; arm_packet(slot,1,NULL,0);
    }
    return c->stage != STALLED;
}
bool native_hub_control_status(uint8_t slot, const tusb_control_request_t* request) {
    if (slot >= DEVICES || request == NULL || request->wLength) return false;
    control_t* c = &devices[slot].control;
    if (c->generation != devices[slot].generation ||
        memcmp(request,&c->request,sizeof(*request)) != 0) return false;
    if (request->bmRequestType & 0x80) {
        devices[slot].control.stage = STATUS_OUT; arm_packet(slot,1,NULL,0);
    } else status_in(slot,NO_ACTION);
    return true;
}

static void reset_device(uint8_t slot) {
    uint32_t flags = spin_lock_blocking(bank_lock);
    ++devices[slot].generation;
    ++devices[slot].reset_generation;
    for (unsigned ch = 2; ch < CHANNELS; ++ch) ++devices[slot].endpoint_generation[ch];
    memset(devices[slot].buffers,0,sizeof(devices[slot].buffers));
    memset(devices[slot].endpoint_controls,0,sizeof(devices[slot].endpoint_controls));
    memset(devices[slot].ep,0,sizeof(devices[slot].ep));
    memset(&devices[slot].control,0,sizeof(devices[slot].control));
    devices[slot].configuration = 0; devices[slot].protocol = 1;
    if (slot == 0) {
        hub_endpoint_control = 0;
        usb_dpram->ep_ctrl[14].in = 0;
        buffer_regs()[30] = 0;
    }
    if (active_device == slot) {
        for (unsigned i = 0; i < CHANNELS; ++i) buffer_regs()[i] = 0;
        for (unsigned i = 0; i < 4; ++i) endpoint_regs()[i] = 0;
    }
    spin_unlock(bank_lock, flags);
    if (slot) native_joycon_usb_reset(slot-1);
}
static void forget_port(unsigned port) {
    uint8_t slot = port + 1;
    addresses[slot] = NONE;
    if (default_device == slot) default_device = NONE;
    reset_device(slot);
    publish_addresses();
}
static void reset_bus(void) {
    probe_router_enable(false);
    uint32_t flags = spin_lock_blocking(bank_lock);
    active_device = 0; usb_hw->dev_addr_ctrl = 0;
    memset(ports,0,sizeof(ports));
    addresses[0] = 0; addresses[1] = addresses[2] = NONE; default_device = 0;
    bus_suspended = false;
    spin_unlock(bank_lock, flags);
    for (uint8_t slot = 0; slot < DEVICES; ++slot) reset_device(slot);
    publish_addresses(); probe_router_enable(true);
}
static void configure_device(uint8_t slot, uint8_t configuration) {
    uint32_t flags = spin_lock_blocking(bank_lock);
    device_t* d = &devices[slot];
    d->configuration = configuration;
    for (unsigned ch = 2; ch < CHANNELS; ++ch) ++d->endpoint_generation[ch];
    if (slot == 0 && configuration != 0) root_configured_once = true;
    for (unsigned ch = 2; ch < CHANNELS; ++ch) {
        bool use = configuration && slot != 0;
        uint8_t type = slot && ch >= 4 ? TUSB_XFER_BULK : TUSB_XFER_INTERRUPT;
        d->endpoint_controls[ch-2] = use ? EP_CTRL_ENABLE_BITS | EP_CTRL_INTERRUPT_PER_BUFFER |
            ((uint32_t)type << EP_CTRL_BUFFER_TYPE_LSB) | data_offset(slot,ch) |
            (ch == 2 ? EP_CTRL_INTERRUPT_ON_NAK : 0) : 0;
        d->buffers[ch] = 0; memset(&d->ep[ch],0,sizeof(d->ep[ch]));
        if (active_device == slot) {
            endpoint_regs()[ch-2] = d->endpoint_controls[ch-2]; buffer_regs()[ch] = 0;
        }
    }
    if (slot == 0) {
        hub_endpoint_control = configuration ? EP_CTRL_ENABLE_BITS | EP_CTRL_INTERRUPT_PER_BUFFER | EP_CTRL_INTERRUPT_ON_NAK |
            ((uint32_t)TUSB_XFER_INTERRUPT << EP_CTRL_BUFFER_TYPE_LSB) | data_offset(0,2) : 0;
        usb_dpram->ep_ctrl[14].in = active_device == 0 ? hub_endpoint_control : 0;
        buffer_regs()[30] = 0;
    }
    spin_unlock(bank_lock, flags);
    if (slot) {
        native_joycon_usb_reset(slot-1);
        if (configuration) { arm_packet(slot,3,NULL,0); arm_packet(slot,5,NULL,0); }
    } else if (!configuration) {
        for (unsigned p = 0; p < 2; ++p) { ports[p].status = ports[p].change = 0; forget_port(p); }
    }
}
static const uint16_t* hub_string(uint8_t index) {
    if (!index) { root_string[0] = 0x0304; root_string[1] = 0x0409; return root_string; }
    const char* text = index == 1 ? "Nintendo Co., Ltd." :
        index == 2 ? "Joy-Con 2 Charging Grip" : index == 3 ? root_serial : NULL;
    if (!text) return NULL;
    size_t size = strlen(text); if (size > 63) size = 63;
    root_string[0] = (uint16_t)(0x0300u | (2u + 2u*size));
    for (size_t n = 0; n < size; ++n) root_string[n+1] = (uint8_t)text[n];
    return root_string;
}
static uint16_t get16(const uint8_t* p) { return (uint16_t)(p[0] | ((uint16_t)p[1]<<8)); }
static void word_reply(uint8_t slot, uint16_t value, uint16_t length) {
    uint8_t data[2] = {(uint8_t)value,(uint8_t)(value>>8)}; reply(slot,data,length);
}
static bool standard_request(uint8_t slot) {
    control_t* c = &devices[slot].control;
    const tusb_control_request_t* r = &c->request;
    uint8_t recipient = r->bmRequestType & 31u;
    if (r->bRequest == TUSB_REQ_GET_DESCRIPTOR && (r->bmRequestType & 0x80)) {
        uint8_t type = r->wValue >> 8, index = r->wValue;
        const uint8_t* data = NULL; uint16_t size = 0;
        if (type == TUSB_DESC_DEVICE && !index && !r->wIndex && recipient == 0) {
            data = slot ? native_joycon_device_descriptor(slot-1) : hub_device; size = 18;
        } else if (type == TUSB_DESC_CONFIGURATION && !index && !r->wIndex && recipient == 0) {
            data = slot ? native_joycon_configuration_descriptor(slot-1) : hub_configuration;
            size = get16(data+2);
        } else if (type == TUSB_DESC_STRING && recipient == 0) {
            const uint16_t* text = slot ? native_joycon_string_descriptor(slot-1,index,r->wIndex) : hub_string(index);
            if (text) { data = (const uint8_t*)text; size = text[0] & 255u; }
        } else if (slot && recipient == 1 && !r->wIndex && !index && type == 0x22) {
            data = tud_hid_descriptor_report_cb(slot-1); size = 100;
        } else if (slot && recipient == 1 && !r->wIndex && !index && type == 0x21) {
            data = native_joycon_configuration_descriptor(slot-1)+26; size = 9;
        }
        if (!data) return false;
        reply(slot,data,size); return true;
    }
    if (recipient == 0) {
        if (r->bRequest == TUSB_REQ_SET_ADDRESS && r->bmRequestType == 0 && !r->wLength &&
            !r->wIndex && r->wValue <= 127 && !devices[slot].configuration) {
            for (unsigned i = 0; i < DEVICES; ++i) if (i != slot && addresses[i] == r->wValue) return false;
            status_in(slot,ADDRESS); return true;
        }
        if (r->bRequest == TUSB_REQ_SET_CONFIGURATION && r->bmRequestType == 0 && !r->wLength &&
            !r->wIndex && r->wValue <= 1) { status_in(slot,CONFIGURE); return true; }
        if (r->bRequest == TUSB_REQ_GET_CONFIGURATION && r->bmRequestType == 0x80 &&
            !r->wValue && !r->wIndex && r->wLength == 1) { word_reply(slot,devices[slot].configuration,1); return true; }
        if (r->bRequest == TUSB_REQ_GET_STATUS && r->bmRequestType == 0x80 &&
            !r->wValue && !r->wIndex && r->wLength == 2) { word_reply(slot,1,2); return true; }
    }
    if (recipient == 1 && r->wIndex < (slot ? 2 : 1)) {
        if (r->bRequest == TUSB_REQ_GET_STATUS && r->bmRequestType == 0x81 && !r->wValue && r->wLength == 2) { word_reply(slot,0,2); return true; }
        if (r->bRequest == TUSB_REQ_GET_INTERFACE && r->bmRequestType == 0x81 && !r->wValue && r->wLength == 1) { word_reply(slot,0,1); return true; }
        if (r->bRequest == TUSB_REQ_SET_INTERFACE && r->bmRequestType == 1 && !r->wValue && !r->wLength) { status_in(slot,NO_ACTION); return true; }
    }
    if (recipient == 2 && !(r->wIndex & 0xff70u)) {
        unsigned ep = r->wIndex & 15u;
        unsigned channel = logical_channel(slot,r->wIndex);
        if (channel >= CHANNELS || (ep && !(slot == 0 && r->wIndex == 0x8f ?
            hub_endpoint_control : devices[slot].endpoint_controls[channel-2]))) return false;
        if (r->bRequest == TUSB_REQ_GET_STATUS && r->bmRequestType == 0x82 && !r->wValue && r->wLength == 2) { word_reply(slot,devices[slot].ep[channel].halted,2); return true; }
        if (ep && !r->wValue && !r->wLength && r->bmRequestType == 2 &&
            (r->bRequest == TUSB_REQ_SET_FEATURE || r->bRequest == TUSB_REQ_CLEAR_FEATURE)) {
            status_in(slot,r->bRequest == TUSB_REQ_SET_FEATURE ? ENDPOINT_HALT : ENDPOINT_CLEAR); return true;
        }
    }
    return false;
}
static bool class_request(uint8_t slot) {
    control_t* c = &devices[slot].control; const tusb_control_request_t* r = &c->request;
    if (!slot) {
        if (r->bmRequestType == 0xa0 && r->bRequest == 6 && r->wValue == 0x2900 && !r->wIndex) { reply(slot,hub_descriptor,sizeof(hub_descriptor)); return true; }
        if (r->bmRequestType == 0xa0 && r->bRequest == 0 && !r->wValue && !r->wIndex && r->wLength == 4) { uint32_t zero=0; reply(slot,&zero,4); return true; }
        if (r->wIndex < 1 || r->wIndex > 2) return false;
        port_t* p = &ports[r->wIndex-1];
        if (r->bmRequestType == 0xa3 && !r->bRequest && !r->wValue && r->wLength == 4) {
            uint8_t status[4]={(uint8_t)p->status,(uint8_t)(p->status>>8),(uint8_t)p->change,(uint8_t)(p->change>>8)};
            reply(slot,status,4); return true;
        }
        if (r->bmRequestType != 0x23 || r->wLength || (r->bRequest != 1 && r->bRequest != 3)) return false;
        bool set = r->bRequest == 3;
        if (set && r->wValue != 8 && r->wValue != 4 && r->wValue != 2) return false;
        if (!set && r->wValue != 8 && r->wValue != 1 && r->wValue != 2 &&
            (r->wValue < 16 || r->wValue > 20)) return false;
        if (set && r->wValue == 4 && (!(p->status & POWER) ||
            (default_device != NONE && default_device != r->wIndex))) return false;
        status_in(slot,set ? PORT_SET : PORT_CLEAR); return true;
    }
    if (r->wIndex != 0) return false;
    if (r->bmRequestType == 0xa1 && r->bRequest == 1) {
        uint8_t id = r->wValue, type = r->wValue >> 8;
        uint16_t limit = r->wLength < 64 ? r->wLength : 64;
        uint16_t prefix = id && limit > 1 ? 1 : 0;
        if (prefix) c->data[0] = id;
        uint16_t size = tud_hid_get_report_cb(slot-1,id,(hid_report_type_t)type,c->data+prefix,limit-prefix);
        if (!size || size > limit-prefix) return false;
        reply(slot,c->data,size+prefix); return true;
    }
    if (r->bmRequestType == 0x21 && r->bRequest == 9 && r->wLength <= 64) {
        c->action = HID_SET_REPORT; c->external = c->data; c->length = r->wLength; c->position = 0;
        if (r->wLength) { c->stage = DATA_OUT; arm_packet(slot,1,NULL,0); }
        else status_in(slot,HID_SET_REPORT);
        return true;
    }
    if (r->bmRequestType == 0x21 && r->bRequest == 10 && !r->wLength) { status_in(slot,HID_IDLE); return true; }
    if (r->bmRequestType == 0xa1 && r->bRequest == 2 && r->wLength == 1) { word_reply(slot,devices[slot].idle_rate,1); return true; }
    if (r->bmRequestType == 0x21 && r->bRequest == 11 && !r->wLength && r->wValue <= 1) { status_in(slot,HID_PROTOCOL); return true; }
    if (r->bmRequestType == 0xa1 && r->bRequest == 3 && !r->wValue && r->wLength == 1) { word_reply(slot,devices[slot].protocol,1); return true; }
    return false;
}
static void setup_request(const event_t* event) {
    uint8_t slot = event->device; device_t* d = &devices[slot];
    if (event->generation != d->generation) return;
    memset(&d->control,0,sizeof(d->control));
    control_t* c = &d->control; memcpy(&c->request,event->data,8);
    c->generation = event->generation;
    if (slot == 0)
        probe_debug_printf("[HUB_CTRL] setup g=%" PRIu32 " req=%02x/%02x v=%04x i=%04x n=%u\n",
                           c->generation, c->request.bmRequestType, c->request.bRequest,
                           c->request.wValue, c->request.wIndex, c->request.wLength);
    ++setup_count[slot];
    uint8_t type = c->request.bmRequestType & 0x60;
    bool supported;
    if (type == 0) supported = standard_request(slot);
    else if (type == 0x20) supported = class_request(slot);
    else if (type == 0x40) {
        c->vendor = true;
        supported = tud_vendor_control_xfer_cb(slot,CONTROL_STAGE_SETUP,&c->request);
    } else supported = false;
    if (!supported) stall(slot);
}
static void control_complete(uint8_t slot) {
    control_t* c = &devices[slot].control;
    c->stage = IDLE;
    if (c->vendor) { tud_vendor_control_xfer_cb(slot,CONTROL_STAGE_ACK,&c->request); return; }
    switch (c->action) {
    case ADDRESS: {
        uint32_t flags = spin_lock_blocking(bank_lock);
        addresses[slot] = (uint8_t)c->request.wValue;
        if (!addresses[slot]) default_device = slot;
        else if (default_device == slot) default_device = NONE;
        if (active_device == slot) usb_hw->dev_addr_ctrl = addresses[slot];
        spin_unlock(bank_lock,flags);
        publish_addresses(); break;
    }
    case CONFIGURE: configure_device(slot,(uint8_t)c->request.wValue); break;
    case HID_IDLE: devices[slot].idle_rate = c->request.wValue >> 8; break;
    case HID_PROTOCOL: devices[slot].protocol = c->request.wValue; break;
    case HID_SET_REPORT: {
        uint8_t id = c->request.wValue; const uint8_t* data = c->data; uint16_t length = c->position;
        if (id && length > 1 && data[0] == id) { ++data; --length; }
        tud_hid_set_report_cb(slot-1,id,(hid_report_type_t)(c->request.wValue>>8),data,length); break;
    }
    case ENDPOINT_HALT:
    case ENDPOINT_CLEAR: {
        uint8_t channel = (uint8_t)logical_channel(slot,c->request.wIndex);
        uint32_t flags = spin_lock_blocking(bank_lock);
        ++devices[slot].endpoint_generation[channel];
        endpoint_t* ep = &devices[slot].ep[channel]; ep->halted = c->action == ENDPOINT_HALT;
        ep->busy = ep->flush = ep->zlp = false; ep->next_pid = 0;
        set_buffer(slot,channel,ep->halted ? USB_BUF_CTRL_STALL : 0);
        spin_unlock(bank_lock,flags);
        if (!ep->halted && (channel & 1u)) arm_packet(slot,channel,NULL,0);
        break;
    }
    case PORT_SET:
    case PORT_CLEAR: {
        unsigned index = c->request.wIndex - 1; port_t* p = &ports[index]; bool set = c->action == PORT_SET;
        switch (c->request.wValue) {
        case 8:
            if (set) { p->status |= POWER | CONNECT; p->change |= C_CONNECT; }
            else { p->status = 0; p->change |= C_CONNECT; forget_port(index); }
            break;
        case 4:
            forget_port(index); p->status = (p->status | RESET) & ~(ENABLE | SUSPEND);
            p->deadline = time_us_32()+10000u; break;
        case 1: p->status &= ~ENABLE; forget_port(index); break;
        case 2:
            if (set) p->status |= SUSPEND;
            else { p->status &= ~SUSPEND; p->change |= C_SUSPEND; }
            break;
        default: p->change &= ~(1u << (c->request.wValue-16)); break;
        }
        break;
    }
    default: break;
    }
}
static void transmit_next(uint8_t slot, uint8_t channel) {
    endpoint_t* ep = &devices[slot].ep[channel];
    uint16_t remaining = ep->length - ep->sent;
    uint16_t size = remaining > 64 ? 64 : remaining;
    if (!remaining) ep->zlp = false;
    // arm_packet snapshots this packet for actual completion callbacks.
    uint8_t packet[64]; if (size) memcpy(packet,ep->data+ep->sent,size);
    arm_packet(slot,channel,packet,size);
}
static void __no_inline_not_in_flash_func(transfer_complete)(const event_t* event) {
    uint8_t slot = event->device, channel = event->channel;
    device_t* d = &devices[slot];
    if (event->reset_generation != d->reset_generation) return;
    if (channel >= 2 && event->generation != d->endpoint_generation[channel]) return;
    if (channel < 2) {
        control_t* c = &d->control;
        if (slot == 0)
            probe_debug_printf("[HUB_CTRL] complete g=%" PRIu32 "/%" PRIu32 " ch=%u len=%u expected=%u state=%u\n",
                               event->generation, c->generation, channel, event->length,
                               c->packet_length, (unsigned)c->stage);
        // Preserve a real status ACK queued before the next SETUP. Unlike
        // SETUP, reset invalidates even these queued completions (above).
        if (event->generation != c->generation) return;
        if ((c->stage == STATUS_IN && channel == 0) || (c->stage == STATUS_OUT && channel == 1)) {
            if (event->length) stall(slot);
            else if (c->stage == STATUS_OUT && (c->request.bmRequestType & 0x80u)) {
                // Claim the completed read before a reset can revoke it, but
                // let USB IRQs run while its read-only ACK callback executes.
                uint32_t flags = save_and_disable_interrupts();
                bool acknowledged = event->reset_generation == d->reset_generation;
                if (acknowledged) c->stage = IDLE;
                restore_interrupts(flags);
                if (acknowledged && c->vendor)
                    tud_vendor_control_xfer_cb(slot,CONTROL_STAGE_ACK,&c->request);
            }
            else {
                // Do not let a reset IRQ revoke ownership between checking it
                // and publishing the acknowledged service transaction.
                uint32_t flags = save_and_disable_interrupts();
                if (event->reset_generation == d->reset_generation)
                    control_complete(slot);
                restore_interrupts(flags);
            }
        } else if (c->stage == DATA_IN && channel == 0) {
            if (event->generation != d->generation) return;
            if (event->length != c->packet_length) { stall(slot); return; }
            c->position += event->length;
            if (c->position < c->length || c->zlp) control_next(slot);
            else {
                if (c->vendor && !tud_vendor_control_xfer_cb(slot,CONTROL_STAGE_DATA,&c->request)) { stall(slot); return; }
                c->stage = STATUS_OUT; arm_packet(slot,1,NULL,0);
            }
        } else if (c->stage == DATA_OUT && channel == 1) {
            if (event->generation != d->generation) return;
            if (event->length > c->length-c->position) { stall(slot); return; }
            if (event->length) memcpy(c->external+c->position,event->data,event->length);
            c->position += event->length;
            if (c->position == c->length || event->length < PACKET) {
                if (c->position != c->length || (c->vendor && !tud_vendor_control_xfer_cb(slot,CONTROL_STAGE_DATA,&c->request))) { stall(slot); return; }
                status_in(slot,c->action);
            } else arm_packet(slot,1,NULL,0);
        }
        return;
    }
    if (!d->configuration) return;
    endpoint_t* ep = &d->ep[channel];
    if (channel & 1u) {
        ++output_count[slot];
        if (slot && channel == 5) tud_vendor_rx_cb(slot-1,event->data,event->length);
        else if (slot && channel == 3) tud_hid_set_report_cb(slot-1,0,HID_REPORT_TYPE_OUTPUT,event->data,event->length);
        if (!ep->halted) arm_packet(slot,channel,NULL,0);
    } else {
        if (!ep->busy || event->length != ep->packet_length) { failed = true; return; }
        ep->sent += event->length;
        bool done = ep->sent == ep->length && !ep->zlp;
        if (done) { ep->busy = false; ep->flush = false; }
        else transmit_next(slot,channel);
        ++input_count[slot];
        if (slot && channel == 2) tud_hid_report_complete_cb(slot-1,event->data,event->length);
        else if (slot && channel == 4) tud_vendor_tx_cb(slot-1,event->length);
    }
}

bool native_hub_mounted(uint8_t instance) { return instance < 2 && devices[instance+1].configuration != 0; }
bool native_hub_suspended(uint8_t instance) { return instance >= 2 || bus_suspended || (ports[instance].status & SUSPEND); }
bool native_hub_hid_ready(uint8_t instance) {
    return native_hub_mounted(instance) && !native_hub_suspended(instance) &&
        !devices[instance+1].ep[2].busy && !devices[instance+1].ep[2].halted;
}
bool native_hub_hid_report(uint8_t instance, uint8_t report_id, const void* data, uint16_t length) {
    if (!native_hub_hid_ready(instance) || length > 63 || (length && !data)) return false;
    endpoint_t* ep = &devices[instance+1].ep[2];
    ep->data[0] = report_id; if (length) memcpy(ep->data+1,data,length);
    ep->length = length+1; ep->sent = 0; ep->busy = ep->flush = true; ep->zlp = false;
    transmit_next(instance+1,2); return true;
}
uint32_t native_hub_vendor_write_available(uint8_t instance) {
    if (!native_hub_mounted(instance) || native_hub_suspended(instance)) return 0;
    endpoint_t* ep = &devices[instance+1].ep[4];
    return ep->busy || ep->halted ? 0 : sizeof(ep->data);
}
uint32_t native_hub_vendor_write(uint8_t instance, const void* data, uint32_t length) {
    if (!length || length > native_hub_vendor_write_available(instance) || !data) return 0;
    endpoint_t* ep = &devices[instance+1].ep[4];
    memcpy(ep->data,data,length); ep->length = length; ep->sent = 0;
    ep->busy = true; ep->flush = false; ep->zlp = length % 64 == 0;
    return length;
}
uint32_t native_hub_vendor_write_flush(uint8_t instance) {
    if (instance >= 2) return 0;
    endpoint_t* ep = &devices[instance+1].ep[4];
    if (!ep->busy || ep->flush) return 0;
    ep->flush = true; transmit_next(instance+1,4); return ep->length;
}

void native_hub_startup_guard(void) {
    if (watchdog_enable_caused_reboot()) {
        stdio_init_all();
        printf("[NATIVE_HUB] watchdog timeout recovery -> BOOTSEL; storage retained\n");
        sleep_ms(20);
        reset_usb_boot(0,0);
    }
    watchdog_enable(8000,false);
}

bool native_hub_init(void) {
    if (started || clock_get_hz(clk_sys) != 240000000u) return false;
    bank_lock = spin_lock_instance(spin_lock_claim_unused(true));
    memset(devices,0,sizeof(devices)); memset(ports,0,sizeof(ports));
    snprintf(root_serial,sizeof(root_serial),"switch-pico-");
    pico_get_unique_board_id_string(root_serial+12,sizeof(root_serial)-12);
    reset_block(RESETS_RESET_USBCTRL_BITS); unreset_block_wait(RESETS_RESET_USBCTRL_BITS);
    memset(usb_dpram,0,USB_DPRAM_SIZE);
    active_device = 0; addresses[0] = 0; addresses[1] = addresses[2] = NONE; default_device = 0;
    usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS | USB_USB_MUXING_USBPHY_AS_GPIO_BITS;
    sio_hw->gpio_hi_oe_clr = SIO_GPIO_HI_IN_USB_DP_BITS | SIO_GPIO_HI_IN_USB_DM_BITS;
    hw_set_bits(&usb_hw->phy_direct,USB_USBPHY_DIRECT_DP_PULLUP_EN_BITS);
    hw_set_bits(&usb_hw->phy_direct_override,USB_USBPHY_DIRECT_OVERRIDE_DP_PULLUP_EN_OVERRIDE_EN_BITS);
    usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;
    usb_hw->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS;
    usb_hw->inte = USB_INTS_BUFF_STATUS_BITS | USB_INTS_BUS_RESET_BITS | USB_INTS_SETUP_REQ_BITS |
        USB_INTS_DEV_SUSPEND_BITS | USB_INTS_DEV_RESUME_FROM_HOST_BITS;
    probe_router_init(clock_get_hz(clk_sys)); probe_router_set_phase(NATIVE_HUB_SAMPLE_PHASE);
    multicore_launch_core1(probe_router_core1);
    uint32_t deadline = time_us_32()+100000;
    probe_router_stats observer;
    do { probe_router_snapshot(&observer); if (observer.ready) break; tight_loop_contents(); }
    while ((int32_t)(time_us_32()-deadline) < 0);
    if (!observer.ready) return false;
    publish_addresses(); probe_router_enable(true);
    irq_set_exclusive_handler(USBCTRL_IRQ,usb_interrupt);
    irq_set_priority(USBCTRL_IRQ,0);
    irq_set_enabled(USBCTRL_IRQ,true);
    hw_set_bits(&usb_hw->sie_ctrl,USB_SIE_CTRL_PULLUP_EN_BITS);
    watchdog_enable(8000,false); started = true; startup_time = time_us_32();
    probe_debug_printf("[NATIVE_HUB] stock USB, SIO phase=%u, 240MHz; hub2068 R2066 L2067; isolated EP0/1/2 banks\n",NATIVE_HUB_SAMPLE_PHASE);
    return true;
}
void native_hub_task(void) {
    if (!started) return;
    if (failed) {
        printf("[NATIVE_HUB] transport failed closed; entering BOOTSEL without erasing storage\n");
        reset_usb_boot(0,0);
        return;
    }
    while (event_tail != event_head) {
        event_t event = events[event_tail];
        __dmb(); event_tail = (event_tail+1u)%EVENTS;
        if (event.kind == 3) reset_bus();
        else if (event.device < DEVICES && event.kind == 1) setup_request(&event);
        else if (event.device < DEVICES && event.kind == 2) transfer_complete(&event);
    }
    restore_selected_bank();
    uint32_t now = time_us_32();
    if (usb_hw->ep_nak_stall_status & (1u << 30)) {
        ++root_naks;
        hw_clear_bits(&usb_hw->ep_nak_stall_status,1u << 30);
    }
    for (unsigned p = 0; p < 2; ++p) {
        if ((ports[p].status & RESET) && (int32_t)(now-ports[p].deadline) >= 0) {
            if (default_device != NONE && default_device != p+1) { failed = true; return; }
            ports[p].status = (ports[p].status & ~RESET) | ENABLE;
            ports[p].change |= C_RESET; addresses[p+1] = 0; default_device = p+1; publish_addresses();
        }
    }
    if (devices[0].configuration && !devices[0].ep[2].busy && !devices[0].ep[2].halted) {
        uint8_t changed = (ports[0].change ? 2u : 0u) | (ports[1].change ? 4u : 0u);
        if (changed) {
            endpoint_t* ep = &devices[0].ep[2]; ep->data[0] = changed;
            ep->length = 1; ep->sent = 0; ep->busy = ep->flush = true; ep->zlp = false;
            transmit_next(0,2);
        }
    }
    static uint32_t last_log;
    if ((uint32_t)(now-last_log) >= 1000000u) {
        last_log = now;
        probe_debug_printf("[NATIVE_HUB] addr=%u/%u/%u cfg=%u/%u/%u setup=%"PRIu32"/%"PRIu32"/%"PRIu32
               " in=%"PRIu32"/%"PRIu32" out=%"PRIu32"/%"PRIu32" switch=%"PRIu32" busy=%"PRIu32" slow=%"PRIu32" late=%"PRIu32"/%"PRIu32"\n",
               addresses[0],addresses[1],addresses[2],devices[0].configuration,devices[1].configuration,devices[2].configuration,
               setup_count[0],setup_count[1],setup_count[2],input_count[1],input_count[2],output_count[1],output_count[2],switches,missed_switches,slow_switches,minimum_lateness,maximum_lateness);
        probe_debug_printf("[HUB_SIE] owner=%u sie=%08"PRIx32" nak=%08"PRIx32
                           " txerr=%08"PRIx32" rxerr=%08"PRIx32" ep1=%08"PRIx32"/%08"PRIx32" hidbusy=%u/%u\n",
                           active_device,usb_hw->sie_status,usb_hw->ep_nak_stall_status,
                           usb_hw->ep_tx_error,usb_hw->ep_rx_error,endpoint_regs()[0],buffer_regs()[2],
                           devices[1].ep[2].busy,devices[2].ep[2].busy);
        probe_debug_printf("[HUB_ROUTE] hits=%"PRIu32"/%"PRIu32"/%"PRIu32
                           " lock=%"PRIu32" blocked=%08"PRIx32"/%08"PRIx32" rootnak=%"PRIu32"\n",
                           token_hits[0],token_hits[1],token_hits[2],missed_lock,
                           blocked_buffers,blocked_sie,root_naks);
#if SWITCH2_PROBE_TRACE_NATIVE_INPUT
        probe_router_stats observer;
        probe_router_snapshot(&observer);
        uint32_t interrupt_mask = save_and_disable_interrupts();
        restore_interrupts(interrupt_mask);
        probe_debug_printf("[HUB_HEALTH] router_ready=%"PRIu32" default=%u irq_mask=%"PRIu32
                           " buf_status=%08"PRIx32" irq_status=%08"PRIx32
                           " hub_ep=%08"PRIx32"/%08"PRIx32" busy=%u\n",
                           observer.ready,default_device,interrupt_mask,
                           usb_hw->buf_status,usb_hw->ints,
                           usb_dpram->ep_ctrl[14].in,buffer_regs()[30],devices[0].ep[2].busy);
        probe_debug_printf("[HUB_RAW] intr=%08"PRIx32" inte=%08"PRIx32" sof=%"PRIu32
                           " mtime=%08"PRIx32" watchdog=%08"PRIx32" nak_poll=%08"PRIx32"\n",
                           usb_hw->intr,usb_hw->inte,usb_hw->sof_rd,sio_hw->mtime,
                           usb_hw->dev_sm_watchdog,usb_hw->nak_poll);
        probe_debug_printf("[HUB_PORTS] status=%04x/%04x change=%04x/%04x\n",
                           ports[0].status,ports[1].status,ports[0].change,ports[1].change);
        for (uint8_t slot = 0; slot < DEVICES; ++slot) {
            const device_t* d = &devices[slot];
            probe_debug_printf("[HUB_EP0] slot=%u stage=%u gen=%"PRIu32"/%"PRIu32
                               " reset=%"PRIu32" request=%02x/%02x pos=%u/%u buffers=%08"PRIx32"/%08"PRIx32"\n",
                               slot,(unsigned)d->control.stage,d->control.generation,d->generation,
                               d->reset_generation,d->control.request.bmRequestType,d->control.request.bRequest,
                               d->control.position,d->control.length,d->buffers[0],d->buffers[1]);
        }
#endif
    }
    watchdog_update();
    // This qualification firmware must remain recoverable if the hub never
    // enumerates. A normal reset after successful enumeration is unaffected.
    if (!root_configured_once && (uint32_t)(time_us_32()-startup_time) >= 15000000u)
        reset_usb_boot(0,0);
}
