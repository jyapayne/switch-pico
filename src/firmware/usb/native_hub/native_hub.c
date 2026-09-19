// Native RP2350 SIE transport for an embedded hub and two or four Joy-Cons.
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
#define NATIVE_HUB_SAMPLE_PHASE PROBE_ROUTER_DEFAULT_PHASE
#endif
_Static_assert(NATIVE_HUB_SAMPLE_PHASE < FS_BIT_CYCLES, "Native SIO sample phase must fit one bit");
#define CHILDREN PROBE_CONTROLLER_COUNT
#define DEVICES (CHILDREN + 1u)
#define CHANNELS 6u
#define PACKET 64u
#define PRIVATE_CHANNELS (CHANNELS - 2u)
#define PRIVATE_DATA_BASE 0x180u
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

_Static_assert(CHILDREN == 2u || CHILDREN == 4u, "Native hub supports one or two pairs");
_Static_assert(DEVICES == PROBE_ROUTER_SLOTS, "Hub and router slot counts must match");
_Static_assert(PRIVATE_DATA_BASE % PACKET == 0u, "Private buffers must be packet aligned");
_Static_assert(offsetof(usb_device_dpram_t, ep0_buf_a) + PACKET <= PRIVATE_DATA_BASE,
               "Private banks must not overlap EP0");
_Static_assert(PRIVATE_DATA_BASE + DEVICES * PRIVATE_CHANNELS * PACKET <= USB_DPRAM_SIZE,
               "Native hub endpoint banks must fit USB DPRAM");

typedef enum { IDLE, DATA_IN, DATA_OUT, STATUS_IN, STATUS_OUT, STALLED } stage_t;
typedef enum { NO_ACTION, ADDRESS, CONFIGURE, PORT_SET, PORT_CLEAR, HID_SET_REPORT,
               HID_IDLE, HID_PROTOCOL, ENDPOINT_HALT, ENDPOINT_CLEAR } action_t;
typedef struct {
    uint8_t data[128];
    uint16_t length, sent, packet_length;
    bool busy, flush, zlp;
    uint8_t next_pid;
    bool halted;
    bool status_out_on_complete;
} endpoint_t;
typedef struct {
    tusb_control_request_t request;
    uint8_t data[1024];
    uint8_t* external;
    uint16_t length, position, packet_length;
    stage_t stage;
    action_t action;
    bool zlp, vendor, read_status_preapproved;
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
    bool status_out_armed;
    uint16_t length;
    uint32_t generation;
    uint32_t reset_generation;
#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
    uint32_t trace_cycle;
    uint32_t status_out_arm_cycle;
#endif
    uint8_t data[64];
} event_t;

static device_t devices[DEVICES];
static port_t ports[CHILDREN];
static uint8_t addresses[DEVICES];
static uint8_t default_device;
static volatile uint8_t active_device;
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

#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
#define OUT_TRACE_SLOTS 65u
#define OUT_TRACE_RETAIN 64u
#define OUT_TRACE_STALL_US 200000u
#define OUT_TRACE_LINE_US 50000u
#define OUT_TRACE_SNAPSHOTS 2u

enum { OUT_TRACE_IDLE, OUT_TRACE_PENDING, OUT_TRACE_SUPERSEDED, OUT_TRACE_HOST };
enum { OUT_TRACE_FIRST_IN_ARMED = 1u, OUT_TRACE_FIRST_IN_COMPLETED = 2u };
enum { OUT_TRACE_STATUS_OUT_ARMED = 1u, OUT_TRACE_STATUS_OUT_COMPLETED = 2u };
enum { OUT_TRACE_TOKEN_IN = 1u, OUT_TRACE_TOKEN_OUT = 2u };

enum {
    OUT_TRACE_LOCK = 1u,
    OUT_TRACE_BUFFERS = 2u,
    OUT_TRACE_SETUP = 4u,
    OUT_TRACE_GUARD = 8u,
    OUT_TRACE_INVALID = 16u,
    OUT_TRACE_AFTER_ARM = 32u, // Device-IN observation after notification, not acceptance.
};
typedef struct {
    uint32_t cursor, cutoff, clock_before, clock_after, sof;
    uint32_t address_before, address_after, in0, out0, buffers, sie, sm, ints;
    uint32_t blocked_buffers, blocked_sie, missed_lock, irq_enter, irq_exit, core0_phase;
    uint32_t ep0_word, address_cycle;
    uint32_t publication_sequence, rx_error;
    uint8_t address, owner, owner_before, owner_after, selected, reason, pid, before_valid;
} out_trace_record_t;
typedef struct {
    uint32_t time_us, quiet_us, cursor, count, input[CHILDREN], cycle, sof;
    uint32_t address, owner, out0, buffers, sie, sm, ints, intr, inte;
    uint32_t tx_error, rx_error, irq_enter, irq_exit, core0_phase;
    uint32_t control_slot, control_generation, control_stage, control_position, control_length;
    uint32_t control_word, in0, ep0_word;
    uint32_t reason, setup_cycle, first_in_arm_cycle, first_in_complete_cycle;
    uint32_t first_in_sequence;
    uint16_t first_in_arm_length, first_in_length;
    uint8_t first_in_flags, first_in_pid;
    uint32_t device_generation, control_shadow_in, control_shadow_out;
    uint32_t status_out_arm_cycle, status_out_complete_cycle;
    uint16_t status_out_length;
    uint8_t status_out_flags;
    tusb_control_request_t control_request;
} out_trace_header_t;

// All producer state is ordinary SRAM/BSS. Only Core1 writes the records.
// Cursor low 7 bits name the NEXT slot (0..64); upper bits count ring laps.
// Skipping unused low-bit values keeps each publication monotonic without a
// division on Core1, and slot adjacency survives uint32_t generation rollover.
// Freeze and cursor accesses below are SC (publication includes release,
// snapshot includes acquire). In their common total order, a producer's final
// freeze=false load precedes Core0's freeze=true store. Every earlier record
// has already been published, and at most that one iteration can still write.
// Core0 therefore reads only the 64 slots BEFORE its acquired cursor, never
// the possible in-flight 65th slot. Ordinary record reads/writes cannot race.
// Core0 copies those records into an immutable snapshot before rearming the
// producer. UART draining never reads the live ring or holds it frozen.
static out_trace_record_t out_trace_records[OUT_TRACE_SLOTS];
static uint32_t out_trace_cursor, out_trace_frozen;
static uint32_t out_trace_irq_enter, out_trace_irq_exit;
static uint32_t out_trace_last_missed_lock; // Core1, updated after every rejected selection.
static uint32_t out_trace_last_switches; // Core1, updated after every successful selection.
static uint8_t out_trace_first_tokens[DEVICES]; // Core1: first IN/OUT after SETUP.
// Core0 releases a notification after each child's successful first EP0 IN arm.
// Core1 alone owns seen values. Equality detects updates across uint32_t wrap;
// sequence zero is valid when OUT_TRACE_AFTER_ARM marks a retained observation.
static uint32_t out_trace_publication_sequence[CHILDREN];
static uint32_t out_trace_seen_publication_sequence[CHILDREN];
static uint32_t out_trace_address_cycle; // Core1: most recent selector address write.
static uint32_t out_trace_core0_phase;

// Core0 owns snapshots, UART draining and control/IN-progress watches. Neither
// IRQ nor Core1 logs, allocates, or waits for diagnostic output.
typedef struct {
    out_trace_header_t header;
    out_trace_record_t records[OUT_TRACE_RETAIN];
} out_trace_snapshot_t;
static out_trace_snapshot_t out_trace_snapshots[OUT_TRACE_SNAPSHOTS];
static uint32_t out_trace_snapshot_head, out_trace_snapshot_count, out_trace_snapshot_drops;
static uint32_t out_trace_input[CHILDREN], out_trace_last_input_us, out_trace_last_line_us;
static uint32_t out_trace_dump_line;
static bool out_trace_seen_input;
typedef struct {
    uint32_t generation, since_us;
    stage_t stage;
    uint16_t position;
    bool captured;
    uint32_t setup_cycle, first_in_arm_cycle, first_in_complete_cycle;
    uint32_t first_in_sequence;
    uint16_t first_in_arm_length, first_in_length;
    uint8_t first_in_flags, first_in_pid;
    uint32_t status_out_arm_cycle, status_out_complete_cycle;
    uint16_t status_out_length;
    uint8_t status_out_flags;
} control_trace_watch_t;
static control_trace_watch_t out_trace_controls[DEVICES];
static const out_trace_header_t* out_trace_snapshot(uint32_t now, uint8_t control_slot, uint32_t reason);
#endif

static const uint8_t hub_device[] = {
    18,1,0x10,1,9,0,0,64,0x7e,5,0x68,0x20,0,1,1,2,3,1
};
static const uint8_t hub_configuration[] = {
    // Grip-style self-powered topology; the Bluetooth children use their own
    // batteries. Do not advertise unimplemented high-speed TT/remote wake.
    9,2,25,0,1,1,0,0xc0,250, 9,4,0,0,1,9,0,0,0, 7,5,0x8f,3,1,0,12
};
// One bitmap byte covers the hub and all ports. Children are non-removable;
// the reserved hub bit is clear, and the legacy power-control mask stays 0xff.
static const uint8_t hub_descriptor[] = {
    9,0x29,CHILDREN,0x11,0,5,100,(1u << DEVICES)-2u,255
};

static inline volatile uint32_t* buffer_regs(void) {
    return (volatile uint32_t*)&usb_dpram->ep_buf_ctrl[0];
}
static inline volatile uint32_t* endpoint_regs(void) {
    return (volatile uint32_t*)&usb_dpram->ep_ctrl[0];
}
static inline uint32_t data_offset(uint8_t device, uint8_t channel) {
    return PRIVATE_DATA_BASE + ((uint32_t)device * PRIVATE_CHANNELS + channel - 2u) * PACKET;
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
    if (active_device == device || (device == 0 && channel == 2)) {
        unsigned physical = physical_channel(device,channel);
        buffer_regs()[physical] = value & ~USB_BUF_CTRL_AVAIL;
        if (value & USB_BUF_CTRL_AVAIL) buffer_settle();
        buffer_regs()[physical] = value;
    }
}

// SRAM receiver only. No bank changes while a hardware completion is pending.
bool __not_in_flash_func(native_hub_select_device)(uint8_t address, uint8_t owner, uint32_t cutoff) {
    if (owner >= DEVICES || bank_lock == NULL) return false;
    if (active_device == owner && usb_hw->dev_addr_ctrl == address) {
        ++token_hits[owner];
        return true;
    }
    if (!spin_try_lock_unsafe(bank_lock)) {
        ++missed_switches; ++missed_lock; ++token_hits[owner];
        return false;
    }
    __dmb();
    if ((usb_hw->sie_status & USB_SIE_STATUS_SETUP_REC_BITS) || usb_hw->buf_status ||
        (int32_t)(sio_hw->mtime - cutoff) >= 0) {
        ++missed_switches;
        blocked_buffers = usb_hw->buf_status;
        blocked_sie = usb_hw->sie_status;
        spin_unlock_unsafe(bank_lock);
        ++token_hits[owner];
        return false;
    }
    if (active_device != owner) {
        volatile uint32_t* buffers = (volatile uint32_t*)&usb_dpram->ep_buf_ctrl[0];
        const device_t* restrict incoming = &devices[owner];
        volatile uint32_t* controls = (volatile uint32_t*)&usb_dpram->ep_ctrl[0];
        // PID/length and endpoint pointers must describe the incoming owner
        // before its address becomes visible. AVAIL stays clear until the bank
        // is settled; EP0 IN also needs its shared payload copied below.
        for (unsigned i = 0; i < CHANNELS; ++i) {
            uint32_t value = owner == 0 && i >= 2 ? 0 : incoming->buffers[i];
            buffers[i] = value & ~USB_BUF_CTRL_AVAIL;
        }
        usb_dpram->ep_ctrl[14].in = owner == 0 ? hub_endpoint_control : 0;
        for (unsigned i = 0; i < 4; ++i) controls[i] = incoming->endpoint_controls[i];
        usb_hw->ep_stall_arm = ((incoming->buffers[0] & USB_BUF_CTRL_STALL) ? 1u : 0u) |
            ((incoming->buffers[1] & USB_BUF_CTRL_STALL) ? 2u : 0u);
        __dmb();
        usb_hw->dev_addr_ctrl = address;
        active_device = owner;
#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
        out_trace_address_cycle = sio_hw->mtime;
#endif
        buffer_settle();
        for (unsigned i = 1; i < CHANNELS; ++i)
            buffers[i] = owner == 0 && i >= 2 ? 0 : incoming->buffers[i];
        // Complete EP0 publication here, not in a later foreground pass:
        // alternating control polls must not depend on which owner Core0 sees.
        const uint32_t ep0 = incoming->buffers[0];
        if ((ep0 & (USB_BUF_CTRL_FULL | USB_BUF_CTRL_AVAIL)) ==
            (USB_BUF_CTRL_FULL | USB_BUF_CTRL_AVAIL)) {
            volatile uint32_t* to = (volatile uint32_t*)usb_dpram->ep0_buf_a;
            const uint32_t* from = incoming->ep0_image;
            unsigned words = ((ep0 & USB_BUF_CTRL_LEN_MASK) + 3u) / 4u;
            // Four aligned words per iteration keep the common 16/64-byte
            // replies bounded without a flash-backed memcpy or jump table.
            for (; words >= 4; words -= 4) {
                to[0] = from[0]; to[1] = from[1];
                to[2] = from[2]; to[3] = from[3];
                to += 4; from += 4;
            }
            while (words) { *to++ = *from++; --words; }
        }
        // Metadata was settled above; publish AVAIL only after the payload.
        __dmb();
        buffers[0] = ep0;
    } else {
        usb_hw->dev_addr_ctrl = address;
#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
        out_trace_address_cycle = sio_hw->mtime;
#endif
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
    // Count every valid attempt, but keep this diagnostic store off the
    // address-critical path on a successful handover.
    ++token_hits[owner];
    return true;
}

#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
static __force_inline out_trace_record_t* out_trace_begin_record(void) {
    if (__atomic_load_n(&out_trace_frozen,__ATOMIC_SEQ_CST)) return NULL;
    const uint32_t cursor = __atomic_load_n(&out_trace_cursor,__ATOMIC_SEQ_CST);
    out_trace_record_t* record = &out_trace_records[cursor & 127u];
    record->cursor = cursor;
    return record;
}

static __force_inline void out_trace_finish_record(out_trace_record_t* record) {
    // Post-decision observations are sequential, NOT an atomic hardware image.
    // Never inspect unlocked software control stages or shadow banks.
    record->address_after = usb_hw->dev_addr_ctrl;
    record->owner_after = active_device;
    record->sof = usb_hw->sof_rd;
    record->in0 = usb_dpram->ep_buf_ctrl[0].in;
    record->out0 = usb_dpram->ep_buf_ctrl[0].out;
    record->buffers = usb_hw->buf_status;
    record->sie = usb_hw->sie_status;
    record->rx_error = usb_hw->ep_rx_error;
    record->sm = usb_hw->sm_state;
    record->ints = usb_hw->ints;
    record->missed_lock = missed_lock;
    record->core0_phase = __atomic_load_n(&out_trace_core0_phase,__ATOMIC_ACQUIRE);
    record->irq_enter = __atomic_load_n(&out_trace_irq_enter,__ATOMIC_ACQUIRE);
    record->irq_exit = __atomic_load_n(&out_trace_irq_exit,__ATOMIC_ACQUIRE);
    record->ep0_word = *(const volatile uint32_t*)usb_dpram->ep0_buf_a;
    // A handover has a fresh commit; same-owner control observations can name
    // an earlier selector write. Failed selections did not commit an address.
    record->address_cycle = record->selected ? out_trace_address_cycle : 0;
    const uint32_t cursor = record->cursor;
    const uint32_t next = (cursor & 127u) == OUT_TRACE_SLOTS-1u ? cursor+64u : cursor+1u;
    __atomic_store_n(&out_trace_cursor,next,__ATOMIC_SEQ_CST);
}

void __no_inline_not_in_flash_func(native_hub_note_selected_token)(
        uint8_t address, uint8_t owner, uint32_t cutoff, uint8_t pid) {
    // Update observation state even while frozen, so the next same-owner poll
    // cannot be mislabelled as a handover which happened during the snapshot.
    bool retain = switches != out_trace_last_switches;
    out_trace_last_switches = switches;
    if (owner >= DEVICES) return;
    if (owner) {
        if (pid == 0x2du) {
            out_trace_first_tokens[owner] = OUT_TRACE_TOKEN_IN | OUT_TRACE_TOKEN_OUT;
            retain = true;
        } else if (pid == 0x69u && (out_trace_first_tokens[owner] & OUT_TRACE_TOKEN_IN)) {
            out_trace_first_tokens[owner] &= (uint8_t)~OUT_TRACE_TOKEN_IN;
            retain = true;
        } else if (pid == 0xe1u && (out_trace_first_tokens[owner] & OUT_TRACE_TOKEN_OUT)) {
            out_trace_first_tokens[owner] &= (uint8_t)~OUT_TRACE_TOKEN_OUT;
            retain = true;
        }
    }
    uint32_t publication_sequence = 0;
    bool after_arm = false;
    if (owner && pid == 0x69u) {
        publication_sequence = __atomic_load_n(&out_trace_publication_sequence[owner-1u],__ATOMIC_ACQUIRE);
        after_arm = publication_sequence != out_trace_seen_publication_sequence[owner-1u];
        out_trace_seen_publication_sequence[owner-1u] = publication_sequence;
        retain |= after_arm;
    }
    if (!retain) return;
    // All tracing follows selection. Endpoint bits are not decoded here;
    // neither a handover nor a first-token record proves SIE/host acceptance.
    out_trace_record_t* record = out_trace_begin_record();
    if (!record) return;
    record->cutoff = cutoff;
    record->clock_before = 0;
    record->clock_after = sio_hw->mtime;
    record->address = address;
    record->owner = owner;
    record->address_before = 0;
    record->owner_before = NONE;
    record->selected = true;
    record->reason = after_arm ? OUT_TRACE_AFTER_ARM : 0;
    record->publication_sequence = after_arm ? publication_sequence : 0;
    record->pid = pid;
    record->before_valid = false;
    record->blocked_buffers = record->blocked_sie = 0;
    out_trace_finish_record(record);
}

void __no_inline_not_in_flash_func(native_hub_note_failed_select)(
        uint8_t address, uint8_t owner, uint32_t cutoff, uint8_t pid) {
    // Every rejection reaches this hook, even while frozen. A changed lock
    // counter therefore identifies THIS attempt, not an older rejected token.
    const bool lock_failed = missed_lock != out_trace_last_missed_lock;
    out_trace_last_missed_lock = missed_lock;
    const uint32_t clock_after = sio_hw->mtime;
    out_trace_record_t* record = out_trace_begin_record();
    if (!record) return;
    record->cutoff = cutoff;
    record->clock_before = 0;
    record->clock_after = clock_after;
    record->address = address;
    record->owner = owner;
    record->address_before = 0;
    record->owner_before = NONE;
    record->selected = false;
    record->pid = pid;
    record->before_valid = false; // No added work before normal IN/SETUP selection.
    record->publication_sequence = 0;
    record->blocked_buffers = record->blocked_sie = 0;
    if (lock_failed) record->reason = OUT_TRACE_LOCK;
    else if (owner >= DEVICES || bank_lock == NULL) record->reason = OUT_TRACE_INVALID;
    else {
        record->reason = OUT_TRACE_GUARD;
        record->blocked_buffers = blocked_buffers;
        record->blocked_sie = blocked_sie;
        if (blocked_buffers) record->reason |= OUT_TRACE_BUFFERS;
        if (blocked_sie & USB_SIE_STATUS_SETUP_REC_BITS) record->reason |= OUT_TRACE_SETUP;
    }
    out_trace_finish_record(record);
}

uint32_t __no_inline_not_in_flash_func(native_hub_trace_phase)(uint32_t phase) {
    // All setters run on Core0; IRQ instrumentation does not modify this tag.
    const uint32_t previous = __atomic_load_n(&out_trace_core0_phase,__ATOMIC_RELAXED);
    __atomic_store_n(&out_trace_core0_phase,phase,__ATOMIC_RELEASE);
    return previous;
}

#endif


static void publish_addresses(void) { probe_router_publish(addresses, default_device); }
static uint8_t hardware_owner(void) {
    uint8_t address = usb_hw->dev_addr_ctrl & 127u;
    if (address == 0) return default_device;
    for (uint8_t i = 0; i < DEVICES; ++i) if (addresses[i] == address) return i;
    return NONE;
}
static __force_inline bool push_event(uint8_t device, uint8_t channel, uint8_t kind, uint16_t length,
                       const uint8_t* data, bool status_out_armed, uint32_t status_arm_cycle) {
    uint32_t next = (event_head + 1u) % EVENTS;
    if (next == event_tail || length > 64) { failed = true; return false; }
    event_t* event = &events[event_head];
    event->device = device; event->channel = channel; event->kind = kind;
    event->length = length;
    event->status_out_armed = status_out_armed;
    event->generation = device < DEVICES ? (channel < 2 ? devices[device].generation :
        devices[device].endpoint_generation[channel]) : 0;
    event->reset_generation = device < DEVICES ? devices[device].reset_generation : 0;
#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
    event->trace_cycle = sio_hw->mtime;
    event->status_out_arm_cycle = status_arm_cycle;
#else
    (void)status_arm_cycle;
#endif
    if (kind == 2 && (channel & 1u) && channel != 1) copy_from_usb(event->data,data,length);
    else {
        // SRAM sources may be unaligned. Volatile byte reads keep this copy
        // inline instead of calling flash-backed memcpy during BOOTSEL sampling.
        const volatile uint8_t* source = data;
        for (uint16_t i = 0; i < length; ++i) event->data[i] = source[i];
    }
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
            devices[i].ep[0].status_out_on_complete = false;
            for (unsigned ch = 2; ch < CHANNELS; ++ch) ++devices[i].endpoint_generation[ch];
            // Selection can now publish EP0 without foreground dispatch.
            // Revoke all shadow readiness here so no old bank can reappear
            // between the reset IRQ and reset_bus. Volatile stores stay in SRAM.
            volatile uint32_t* shadow = devices[i].buffers;
            for (unsigned ch = 0; ch < CHANNELS; ++ch) shadow[ch] = 0;
        }
        for (unsigned i = 0; i < CHANNELS; ++i) buffer_regs()[i] = 0;
        buffer_regs()[30] = 0; // Root interrupt endpoint has its own physical bank.
        hw_clear_bits(&usb_hw->buf_status,usb_hw->buf_status);
        hw_clear_bits(&usb_hw->sie_status,USB_SIE_STATUS_BUS_RESET_BITS | USB_SIE_STATUS_SETUP_REC_BITS);
        push_event(0,0,3,0,NULL,false,0);
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
        bool status_out_armed = false;
        uint32_t status_arm_cycle = 0;
        if (channel == 0) {
            endpoint_t* in = &devices[completed_owner].ep[0];
            // Only pre-approved reads may cross this boundary without a DATA
            // callback. A replacement SETUP owns EP0 and must not be overwritten.
            status_out_armed = in->status_out_on_complete &&
                length == in->packet_length && !failed &&
                !(usb_hw->sie_status & USB_SIE_STATUS_SETUP_REC_BITS) &&
                !(pending & 2u);
            in->status_out_on_complete = false;
            if (status_out_armed) {
                devices[completed_owner].ep[1].next_pid = 1;
                devices[completed_owner].ep[1].packet_length = 0;
#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
                status_arm_cycle = sio_hw->mtime;
#endif
                set_buffer(completed_owner,1,USB_BUF_CTRL_AVAIL |
                    USB_BUF_CTRL_LAST | USB_BUF_CTRL_SEL | USB_BUF_CTRL_DATA1_PID);
            }
        }
        devices[completed_owner].ep[channel].next_pid ^= 1u;
        devices[completed_owner].buffers[channel] = 0;
        buffer_regs()[physical] = 0;
        hw_clear_bits(&usb_hw->buf_status,mask);
        pending &= ~mask;
        // Unarmed device-specific buffers and the TX shadow cannot be reused
        // until Core0 consumes this event. Copy them without blocking routing.
        spin_unlock(bank_lock,flags);
        push_event(completed_owner,(uint8_t)channel,2,length,data,
                   status_out_armed,status_arm_cycle);
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
            devices[owner].ep[0].status_out_on_complete = false;
            devices[owner].buffers[0] = devices[owner].buffers[1] =
                USB_BUF_CTRL_DATA1_PID | USB_BUF_CTRL_SEL;
            buffer_regs()[0] = buffer_regs()[1] = USB_BUF_CTRL_DATA1_PID | USB_BUF_CTRL_SEL;
            devices[owner].ep[0].next_pid = devices[owner].ep[1].next_pid = 1;
            push_event(owner,0,1,sizeof(setup),setup,false,0);
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

#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
static void __no_inline_not_in_flash_func(usb_interrupt_traced)(void) {
    __atomic_store_n(&out_trace_irq_enter,sio_hw->mtime,__ATOMIC_RELEASE);
    usb_interrupt();
    __atomic_store_n(&out_trace_irq_exit,sio_hw->mtime,__ATOMIC_RELEASE);
}
#endif

void __no_inline_not_in_flash_func(native_hub_service_pending_usb)(void) {
    // BOOTSEL sampling keeps IRQs disabled while QSPI CSn is floated. Drain
    // real hardware completions so Core1 can route the next device's token.
    // Do not touch NVIC pending state: a later IRQ safely observes cleared flags.
    if (!started || !usb_hw->ints) return;
#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
    usb_interrupt_traced();
#else
    usb_interrupt();
#endif
}

static void stall(uint8_t slot) {
    uint32_t flags = spin_lock_blocking(bank_lock);
    if (devices[slot].control.generation != devices[slot].generation) {
        spin_unlock(bank_lock,flags);
        return;
    }
    devices[slot].control.stage = STALLED;
    devices[slot].control.action = NO_ACTION;
    devices[slot].ep[0].status_out_on_complete = false;
    if (active_device == slot) hw_set_bits(&usb_hw->ep_stall_arm,3u);
    set_buffer(slot,0,USB_BUF_CTRL_STALL); set_buffer(slot,1,USB_BUF_CTRL_STALL);
    spin_unlock(bank_lock, flags);
}
static void __no_inline_not_in_flash_func(arm_packet)(uint8_t slot, uint8_t channel, const uint8_t* data, uint16_t length) {
    endpoint_t* ep = &devices[slot].ep[channel];
    uint32_t generation = channel < 2 ? devices[slot].control.generation :
        devices[slot].endpoint_generation[channel];
    if (!(channel & 1u) && length) {
        // Images are rewritten only while the per-device buffer is unarmed.
        // The publication lock releases this copy to the Core1 bank selector.
        memcpy(ep->data,data,length);
        if (channel == 0) memcpy(devices[slot].ep0_image,data,length);
        else copy_to_usb(packet_buffer(slot,channel),data,length);
    }
#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
    // Publication attempt, not on-wire readiness: an inactive bank receives
    // a shadow which the next selection copies. Do not extend IRQ masking.
    const bool status_out = channel == 1 && devices[slot].control.stage == STATUS_OUT;
    const uint32_t arm_cycle = (channel == 0 || status_out) ? sio_hw->mtime : 0;
#endif
    uint32_t flags = spin_lock_blocking(bank_lock);
    if (generation != (channel < 2 ? devices[slot].generation :
                       devices[slot].endpoint_generation[channel])) {
        spin_unlock(bank_lock,flags);
        return;
    }
    ep->packet_length = length;
    if (channel == 0) {
        const control_t* c = &devices[slot].control;
        ep->status_out_on_complete = c->stage == DATA_IN &&
            (c->request.bmRequestType & 0x80u) &&
            (!c->vendor || c->read_status_preapproved) &&
            c->position + length == c->length && !c->zlp;
    }
    uint32_t value = USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_LAST | USB_BUF_CTRL_SEL |
        (ep->next_pid ? USB_BUF_CTRL_DATA1_PID : 0);
    if (!(channel & 1u)) {
        if (channel == 0 && active_device == slot)
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
#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
    control_trace_watch_t* watch = &out_trace_controls[slot];
    if (channel == 0) {
        if (!(watch->first_in_flags & OUT_TRACE_FIRST_IN_ARMED)) {
            watch->first_in_arm_cycle = arm_cycle;
            watch->first_in_pid = (value & USB_BUF_CTRL_DATA1_PID) != 0;
            watch->first_in_arm_length = length;
            watch->first_in_flags |= OUT_TRACE_FIRST_IN_ARMED;
            if (slot) {
                // Notify only after the generation-validated publication above.
                // Do not extend the bank lock or IRQ masking for diagnostics.
                const uint32_t sequence = __atomic_load_n(
                    &out_trace_publication_sequence[slot-1u],__ATOMIC_RELAXED) + 1u;
                watch->first_in_sequence = sequence;
                __atomic_store_n(&out_trace_publication_sequence[slot-1u],sequence,__ATOMIC_RELEASE);
            }
        }
    } else if (status_out && !(watch->status_out_flags & OUT_TRACE_STATUS_OUT_ARMED)) {
        watch->status_out_arm_cycle = arm_cycle;
        watch->status_out_flags |= OUT_TRACE_STATUS_OUT_ARMED;
    }
#endif
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
                             void* buffer, uint16_t length, bool read_status_preapproved) {
    if (slot >= DEVICES || request == NULL || (length && buffer == NULL)) return false;
    if (read_status_preapproved && !(request->bmRequestType & 0x80u)) return false;
    control_t* c = &devices[slot].control;
    if (c->generation != devices[slot].generation ||
        memcmp(request,&c->request,sizeof(*request)) != 0) return false;
    c->read_status_preapproved = read_status_preapproved;
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
#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
    // Foreground-only evidence belongs to the cleared control generation.
    // Do not extend the reset critical section or alter queued snapshots.
    memset(&out_trace_controls[slot],0,sizeof(out_trace_controls[slot]));
#endif
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
    memset(addresses,NONE,sizeof(addresses)); addresses[0] = 0; default_device = 0;
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
        for (unsigned p = 0; p < CHILDREN; ++p) { ports[p].status = ports[p].change = 0; forget_port(p); }
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
        if (r->wIndex < 1 || r->wIndex > CHILDREN) return false;
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
        if (set && r->wValue == 4) {
            // Only one child may be resetting toward the shared address zero.
            for (unsigned port = 0; port < CHILDREN; ++port)
                if (port+1 != r->wIndex && (ports[port].status & RESET)) return false;
        }
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
#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
    control_trace_watch_t* watch = &out_trace_controls[slot];
    if (slot && d->control.generation && d->control.stage != IDLE &&
        d->control.stage != STALLED &&
        !(watch->captured && watch->generation == d->control.generation &&
          watch->stage == d->control.stage && watch->position == d->control.position))
        out_trace_snapshot(time_us_32(), slot, OUT_TRACE_SUPERSEDED);
    memset(watch, 0, sizeof(*watch));
    watch->since_us = time_us_32();
    watch->setup_cycle = event->trace_cycle;
#endif
    memset(&d->control,0,sizeof(d->control));
    control_t* c = &d->control; memcpy(&c->request,event->data,8);
    c->generation = event->generation;
    if (slot == 0)
        probe_debug_printf("[HUB_CTRL] setup g=%" PRIu32 " req=%02x/%02x v=%04x i=%04x n=%u\n",
                           c->generation, c->request.bmRequestType, c->request.bRequest,
                           c->request.wValue, c->request.wIndex, c->request.wLength);
    ++setup_count[slot];
#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
    // Root-only diagnostic read. Capture the actual child state before host
    // cleanup can supersede it; never forward this into controller management.
    if (!slot && c->request.bmRequestType == 0xc0 &&
        c->request.bRequest == NATIVE_HUB_TRACE_REQUEST &&
        c->request.wValue == NATIVE_HUB_TRACE_VALUE &&
        c->request.wIndex >= 1 && c->request.wIndex <= CHILDREN &&
        c->request.wLength == NATIVE_HUB_TRACE_REPLY_SIZE) {
        const uint8_t target = (uint8_t)c->request.wIndex;
        const out_trace_header_t* captured = out_trace_snapshot(time_us_32(),target,OUT_TRACE_HOST);
        uint8_t response[NATIVE_HUB_TRACE_REPLY_SIZE] = {'N','H','T','R',1,captured ? 0 : 1,target,0};
        const uint32_t captured_time = captured ? captured->time_us : 0;
        const uint32_t captured_generation = captured ? captured->control_generation : 0;
        for (unsigned byte = 0; byte < 4; ++byte) {
            response[8+byte] = (uint8_t)(captured_time >> (8u*byte));
            response[12+byte] = (uint8_t)(captured_generation >> (8u*byte));
        }
        reply(slot,response,sizeof(response));
        return;
    }
#endif
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
static void complete_port_change(const tusb_control_request_t* request, bool set) {
    const unsigned index = request->wIndex - 1;
    port_t* p = &ports[index];
    switch (request->wValue) {
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
    default: p->change &= ~(1u << (request->wValue-16)); break;
    }
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
#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
        control_trace_watch_t* watch = &out_trace_controls[slot];
        if (event->status_out_armed) {
            watch->status_out_arm_cycle = event->status_out_arm_cycle;
            watch->status_out_flags |= OUT_TRACE_STATUS_OUT_ARMED;
        }
        if (channel == 0 && !(watch->first_in_flags & OUT_TRACE_FIRST_IN_COMPLETED)) {
            watch->first_in_complete_cycle = event->trace_cycle;
            watch->first_in_length = event->length;
            watch->first_in_flags |= OUT_TRACE_FIRST_IN_COMPLETED;
        }
        // For a control read, channel 1 is the status direction even if its
        // completion is observed before foreground DATA_IN bookkeeping.
        if (channel == 1 && (c->request.bmRequestType & 0x80u) &&
            !(watch->status_out_flags & OUT_TRACE_STATUS_OUT_COMPLETED)) {
            watch->status_out_complete_cycle = event->trace_cycle;
            watch->status_out_length = event->length;
            watch->status_out_flags |= OUT_TRACE_STATUS_OUT_COMPLETED;
        }
#endif
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
                uint32_t flags = save_and_disable_interrupts();
                if (event->reset_generation != d->reset_generation) {
                    restore_interrupts(flags);
                    return;
                }
                if (c->action == PORT_SET || c->action == PORT_CLEAR) {
                    // Claim this acknowledged action before allowing another
                    // SETUP/reset IRQ. The immutable request stays valid while
                    // child reset callbacks read storage and publish routing.
                    // A later bus reset is queued and processed after this action.
                    const tusb_control_request_t request = c->request;
                    const bool set = c->action == PORT_SET;
                    c->stage = IDLE;
                    restore_interrupts(flags);
                    complete_port_change(&request,set);
                } else {
                    control_complete(slot);
                    restore_interrupts(flags);
                }
            }
        } else if (c->stage == DATA_IN && channel == 0) {
            // Claim DATA completion before reset can revoke it, as for ACK.
            // A newer SETUP may follow already queued final-IN/status events;
            // unlike reset, it must not discard their foreground callback order.
            uint32_t flags = save_and_disable_interrupts();
            bool valid = event->reset_generation == d->reset_generation &&
                (event->generation == d->generation || event->status_out_armed);
            bool length_valid = event->length == c->packet_length;
            if (valid && length_valid) c->position += event->length;
            bool more = c->position < c->length || c->zlp;
            if (valid && length_valid && !more) c->stage = STATUS_OUT;
            restore_interrupts(flags);
            if (!valid) return;
            if (!length_valid) { stall(slot); return; }
            if (more) control_next(slot);
            else {
                if (c->vendor && !tud_vendor_control_xfer_cb(slot,CONTROL_STAGE_DATA,&c->request)) { stall(slot); return; }
                // IRQ may already have consumed this status OUT. Never rearm it.
                if (!event->status_out_armed) arm_packet(slot,1,NULL,0);
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

bool native_hub_mounted(uint8_t instance) { return instance < CHILDREN && devices[instance+1].configuration != 0; }
bool native_hub_suspended(uint8_t instance) { return instance >= CHILDREN || bus_suspended || (ports[instance].status & SUSPEND); }
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
    if (instance >= CHILDREN) return 0;
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
    if (started || clock_get_hz(clk_sys) != FS_CLOCK_HZ) return false;
    bank_lock = spin_lock_instance(spin_lock_claim_unused(true));
    memset(devices,0,sizeof(devices)); memset(ports,0,sizeof(ports));
    snprintf(root_serial,sizeof(root_serial),"switch-pico-");
    pico_get_unique_board_id_string(root_serial+12,sizeof(root_serial)-12);
    reset_block(RESETS_RESET_USBCTRL_BITS); unreset_block_wait(RESETS_RESET_USBCTRL_BITS);
    memset(usb_dpram,0,USB_DPRAM_SIZE);
    memset(addresses,NONE,sizeof(addresses));
    active_device = 0; addresses[0] = 0; default_device = 0;
    usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS | USB_USB_MUXING_USBPHY_AS_GPIO_BITS;
    sio_hw->gpio_hi_oe_clr = SIO_GPIO_HI_IN_USB_DP_BITS | SIO_GPIO_HI_IN_USB_DM_BITS;
    // GPIO-observer mode needs the physical override, but asserting it here
    // would advertise attachment before the SIE, router and IRQ are ready.
    hw_clear_bits(&usb_hw->phy_direct,USB_USBPHY_DIRECT_DP_PULLUP_EN_BITS);
    hw_set_bits(&usb_hw->phy_direct_override,USB_USBPHY_DIRECT_OVERRIDE_DP_PULLUP_EN_OVERRIDE_EN_BITS);
    usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;
    usb_hw->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS;
    usb_hw->inte = USB_INTS_BUFF_STATUS_BITS | USB_INTS_BUS_RESET_BITS | USB_INTS_SETUP_REQ_BITS |
        USB_INTS_DEV_SUSPEND_BITS | USB_INTS_DEV_RESUME_FROM_HOST_BITS;
    probe_router_init(clock_get_hz(clk_sys));
    if (!probe_router_set_phase(NATIVE_HUB_SAMPLE_PHASE)) return false;
    multicore_launch_core1(probe_router_core1);
    uint32_t deadline = time_us_32()+100000;
    probe_router_stats observer;
    do { probe_router_snapshot(&observer); if (observer.ready) break; tight_loop_contents(); }
    while ((int32_t)(time_us_32()-deadline) < 0);
    if (!observer.ready) return false;
    publish_addresses(); probe_router_enable(true);
#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
    irq_set_exclusive_handler(USBCTRL_IRQ,usb_interrupt_traced);
#else
    irq_set_exclusive_handler(USBCTRL_IRQ,usb_interrupt);
#endif
    irq_set_priority(USBCTRL_IRQ,0);
    irq_set_enabled(USBCTRL_IRQ,true);
    watchdog_enable(8000,false); started = true; startup_time = time_us_32();
    __dmb();
    hw_set_bits(&usb_hw->sie_ctrl,USB_SIE_CTRL_PULLUP_EN_BITS);
    // The forced physical pull-up is the actual attach edge. Publish it last:
    // even an immediate host reset/SETUP now has an initialized receiver.
    hw_set_bits(&usb_hw->phy_direct,USB_USBPHY_DIRECT_DP_PULLUP_EN_BITS);
#if CHILDREN == 2
    probe_debug_printf("[NATIVE_HUB] stock USB, SIO phase=%u, %uMHz; hub2068 R2066 L2067; isolated EP0/1/2 banks\n",NATIVE_HUB_SAMPLE_PHASE,(unsigned)FS_CLOCK_MHZ);
#else
    probe_debug_printf("[NATIVE_HUB] stock USB, SIO phase=%u, %uMHz; hub2068 children=%u order=AR/AL/BR/BL; isolated EP0/1/2 banks\n",NATIVE_HUB_SAMPLE_PHASE,(unsigned)FS_CLOCK_MHZ,CHILDREN);
#endif
    return true;
}

#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
static const out_trace_header_t* out_trace_snapshot(uint32_t now, uint8_t control_slot, uint32_t reason) {
    const bool replace_pending = out_trace_snapshot_count == OUT_TRACE_SNAPSHOTS;
    uint32_t tail = (out_trace_snapshot_head + out_trace_snapshot_count) % OUT_TRACE_SNAPSHOTS;
    if (replace_pending) {
        ++out_trace_snapshot_drops;
        tail = (out_trace_snapshot_head+1u)%OUT_TRACE_SNAPSHOTS;
        if (reason != OUT_TRACE_HOST || out_trace_snapshots[tail].header.reason == OUT_TRACE_HOST)
            return NULL;
        // Only the waiting automatic snapshot can be evicted. The head may
        // already have printed a header; neither it nor another host latch is
        // ever rewritten. The loss counter accounts for the displaced snapshot.
    }
    out_trace_snapshot_t* snapshot = &out_trace_snapshots[tail];
    __atomic_store_n(&out_trace_frozen,1u,__ATOMIC_SEQ_CST);
    out_trace_header_t* h = &snapshot->header;
    memset(h,0,sizeof(*h));
    h->cursor = __atomic_load_n(&out_trace_cursor,__ATOMIC_SEQ_CST);
    h->count = h->cursor < OUT_TRACE_RETAIN ? h->cursor : OUT_TRACE_RETAIN;
    h->time_us = now;
    h->reason = reason;
    h->quiet_us = now - (control_slot < DEVICES
        ? out_trace_controls[control_slot].since_us : out_trace_last_input_us);
    for (unsigned instance = 0; instance < CHILDREN; ++instance)
        h->input[instance] = input_count[instance+1];
    h->cycle = sio_hw->mtime;
    h->sof = usb_hw->sof_rd;
    h->address = usb_hw->dev_addr_ctrl;
    h->owner = active_device;
    h->out0 = usb_dpram->ep_buf_ctrl[0].out;
    h->buffers = usb_hw->buf_status;
    h->sie = usb_hw->sie_status;
    h->sm = usb_hw->sm_state;
    h->ints = usb_hw->ints;
    h->intr = usb_hw->intr;
    h->inte = usb_hw->inte;
    h->tx_error = usb_hw->ep_tx_error;
    h->rx_error = usb_hw->ep_rx_error;
    h->irq_enter = __atomic_load_n(&out_trace_irq_enter,__ATOMIC_ACQUIRE);
    h->irq_exit = __atomic_load_n(&out_trace_irq_exit,__ATOMIC_ACQUIRE);
    h->core0_phase = __atomic_load_n(&out_trace_core0_phase,__ATOMIC_ACQUIRE);
    h->in0 = usb_dpram->ep_buf_ctrl[0].in;
    h->ep0_word = *(const volatile uint32_t*)usb_dpram->ep0_buf_a;
    h->control_slot = control_slot;
    if (control_slot < DEVICES) {
        const control_t* c = &devices[control_slot].control;
        const control_trace_watch_t* watch = &out_trace_controls[control_slot];
        h->control_generation = c->generation;
        h->control_stage = c->stage;
        h->control_position = c->position;
        h->control_length = c->length;
        h->control_request = c->request;
        h->setup_cycle = watch->setup_cycle;
        h->first_in_arm_cycle = watch->first_in_arm_cycle;
        h->first_in_complete_cycle = watch->first_in_complete_cycle;
        h->first_in_sequence = watch->first_in_sequence;
        h->first_in_arm_length = watch->first_in_arm_length;
        h->first_in_length = watch->first_in_length;
        h->first_in_flags = watch->first_in_flags;
        h->first_in_pid = watch->first_in_pid;
        // IRQ can update generation/readiness between these individual reads.
        // They are observations, not an atomic multiword ownership snapshot.
        h->device_generation = *(const volatile uint32_t*)&devices[control_slot].generation;
        const volatile uint32_t* shadow = devices[control_slot].buffers;
        h->control_shadow_in = shadow[0];
        h->control_shadow_out = shadow[1];
        h->status_out_arm_cycle = watch->status_out_arm_cycle;
        h->status_out_complete_cycle = watch->status_out_complete_cycle;
        h->status_out_length = watch->status_out_length;
        h->status_out_flags = watch->status_out_flags;
        if (c->position < c->length) {
            unsigned length = c->length-c->position;
            if (length > sizeof(h->control_word)) length = sizeof(h->control_word);
            memcpy(&h->control_word,c->data+c->position,length);
        }
    }
    // Exclude the acquired cursor's possible in-flight slot. IRQs remain
    // enabled while Core0 copies two contiguous spans; only Core1 recording
    // pauses. The immutable copy, not the live ring, feeds the slow UART.
    unsigned start = (h->cursor & 127u)+OUT_TRACE_SLOTS-h->count;
    if (start >= OUT_TRACE_SLOTS) start -= OUT_TRACE_SLOTS;
    unsigned first = OUT_TRACE_SLOTS-start;
    if (first > h->count) first = h->count;
    memcpy(snapshot->records,out_trace_records+start,first*sizeof(snapshot->records[0]));
    memcpy(snapshot->records+first,out_trace_records,(h->count-first)*sizeof(snapshot->records[0]));
    __atomic_store_n(&out_trace_frozen,0u,__ATOMIC_SEQ_CST);
    if (!out_trace_snapshot_count) {
        out_trace_dump_line = 0;
        out_trace_last_line_us = now-OUT_TRACE_LINE_US;
    }
    if (!replace_pending) ++out_trace_snapshot_count;
    return h;
}

static bool out_trace_task(uint32_t now) {
    bool progress = false;
    for (unsigned instance = 0; instance < CHILDREN; ++instance) {
        progress |= input_count[instance+1] != out_trace_input[instance];
        out_trace_input[instance] = input_count[instance+1];
    }
    if (progress) {
        out_trace_last_input_us = now;
        out_trace_seen_input = true;
    }
    for (uint8_t slot = 0; slot < DEVICES; ++slot) {
        const control_t* c = &devices[slot].control;
        control_trace_watch_t* watch = &out_trace_controls[slot];
        const bool pending = c->stage == DATA_IN || c->stage == DATA_OUT ||
            c->stage == STATUS_IN || c->stage == STATUS_OUT;
        const bool changed = c->generation != watch->generation ||
            c->stage != watch->stage || c->position != watch->position;
        if (changed || !pending) {
            watch->since_us = now;
            watch->generation = c->generation;
            watch->stage = c->stage;
            watch->position = c->position;
            watch->captured = false;
        }
        if (pending && !watch->captured &&
            (uint32_t)(now-watch->since_us) >= OUT_TRACE_STALL_US) {
            out_trace_snapshot(now,slot,OUT_TRACE_PENDING);
            // Count queue-full once for this unchanged state, not every tick.
            watch->captured = true;
        }
    }
    if (out_trace_seen_input && (uint32_t)(now-out_trace_last_input_us) >= OUT_TRACE_STALL_US) {
        out_trace_snapshot(now,NONE,OUT_TRACE_IDLE);
        // Root polling is not new controller input. One quiet episode must not
        // repeatedly snapshot/fill the queue between tests.
        out_trace_seen_input = false;
    }
    if (!out_trace_snapshot_count) return false;

    // One bounded line per 50ms. A full logger retries this same line later;
    // neither producer progress nor queued snapshot contents depend on UART.
    if ((uint32_t)(now-out_trace_last_line_us) < OUT_TRACE_LINE_US) return true;
    out_trace_last_line_us = now;
    const out_trace_snapshot_t* snapshot = &out_trace_snapshots[out_trace_snapshot_head];
    const out_trace_header_t* h = &snapshot->header;
    const unsigned header_lines = 5u + (CHILDREN == 2u ? 0u : CHILDREN);
    int queued;
    if (out_trace_dump_line == 0) {
        queued = probe_debug_printf("[HUB_FLIGHT_FREEZE] us=%"PRIu32" quiet=%"PRIu32
#if CHILDREN == 2
                           " next=%08"PRIx32" n=%"PRIu32" in=%"PRIu32"/%"PRIu32
#else
                           " next=%08"PRIx32" n=%"PRIu32" children=%u"
#endif
                           " cycle=%08"PRIx32" sof=%08"PRIx32" addr=%08"PRIx32" owner=%"PRIu32
                           " out0=%08"PRIx32" bs=%08"PRIx32" sie=%08"PRIx32" sm=%08"PRIx32
                           " ints=%08"PRIx32" intr=%08"PRIx32" inte=%08"PRIx32" reason=%"PRIu32"\n",
                           h->time_us,h->quiet_us,h->cursor,h->count,
#if CHILDREN == 2
                           h->input[0],h->input[1],
#else
                           CHILDREN,
#endif
                           h->cycle,h->sof,h->address,h->owner,h->out0,h->buffers,h->sie,h->sm,
                           h->ints,h->intr,h->inte,h->reason);
    } else if (out_trace_dump_line == 1) {
        queued = probe_debug_printf("[HUB_FLIGHT_CONTEXT] next=%08"PRIx32" irq=%08"PRIx32"/%08"PRIx32
                           " phase=%08"PRIx32" txerr=%08"PRIx32" rxerr=%08"PRIx32"\n",
                           h->cursor,h->irq_enter,h->irq_exit,h->core0_phase,h->tx_error,h->rx_error);
    } else if (out_trace_dump_line == 2) {
        queued = probe_debug_printf("[HUB_FLIGHT_CONTROL] slot=%"PRIu32" gen=%"PRIu32
                           " stage=%"PRIu32" pos=%"PRIu32"/%"PRIu32
                           " setup=%02x/%02x v=%04x i=%04x n=%u"
                           " expected=%08"PRIx32" ep0=%08"PRIx32" in0=%08"PRIx32"\n",
                           h->control_slot,h->control_generation,h->control_stage,
                           h->control_position,h->control_length,
                           h->control_request.bmRequestType,h->control_request.bRequest,
                           h->control_request.wValue,h->control_request.wIndex,h->control_request.wLength,
                           h->control_word,h->ep0_word,h->in0);
    } else if (out_trace_dump_line == 3) {
        queued = probe_debug_printf("[HUB_FLIGHT_CONTROL_CLOCK] slot=%"PRIu32" gen=%"PRIu32
                           " setup=%08"PRIx32" arm=%08"PRIx32" complete=%08"PRIx32
                           " flags=%02x pid=%u arm_len=%u len=%u pub=%08"PRIx32"\n",
                           h->control_slot,h->control_generation,h->setup_cycle,h->first_in_arm_cycle,
                           h->first_in_complete_cycle,h->first_in_flags,h->first_in_pid,
                           h->first_in_arm_length,h->first_in_length,h->first_in_sequence);
    } else if (out_trace_dump_line == 4) {
        queued = probe_debug_printf("[HUB_FLIGHT_STATUS_OUT] slot=%"PRIu32" gen=%"PRIu32
                           " dgen=%"PRIu32" shadow_in=%08"PRIx32" shadow_out=%08"PRIx32
                           " arm=%08"PRIx32" complete=%08"PRIx32" flags=%02x len=%u\n",
                           h->control_slot,h->control_generation,h->device_generation,
                           h->control_shadow_in,h->control_shadow_out,h->status_out_arm_cycle,
                           h->status_out_complete_cycle,h->status_out_flags,h->status_out_length);
    } else if (out_trace_dump_line < header_lines) {
        const unsigned instance = out_trace_dump_line-5u;
        queued = probe_debug_printf("[HUB_FLIGHT_INPUT] slot=%u in=%"PRIu32"\n",instance+1u,h->input[instance]);
    } else if (out_trace_dump_line < h->count+header_lines) {
        const out_trace_record_t* r = &snapshot->records[out_trace_dump_line-header_lines];
        // Hex except owner/ok/lock; req=address/owner, addr/owner=before/after.
        // pre=0 means before-clock/address/owner are unavailable; all HW/IRQ fields are POST.
        // reason bits: 01 lock, 02 saved BUF_STATUS, 04 saved SETUP_REC,
        // 08 original guard, 10 invalid arguments; selected records use 00 or
        // 20 for first device IN after a publication notification. Match pub
        // with a valid CONTROL_CLOCK arm/slot before associating a generation:
        // a notification can outlive its control. Endpoint number is unknown.
        // Guard snapshots can race hardware; absence of 02/04 alone does not
        // prove cutoff was the cause. Keep raw cutoff and clocks for analysis.
        queued = probe_debug_printf("[HUB_FLIGHT] n=%08"PRIx32" pid=%02x pre=%u cutoff=%08"PRIx32
                           " clock=%08"PRIx32"/%08"PRIx32" commit=%08"PRIx32" sof=%08"PRIx32
                           " req=%02x/%u addr=%08"PRIx32"/%08"PRIx32" owner=%u/%u ok=%u why=%02x"
                           " in0=%08"PRIx32" out0=%08"PRIx32" bs=%08"PRIx32" sie=%08"PRIx32" sm=%08"PRIx32
                           " ints=%08"PRIx32" block=%08"PRIx32"/%08"PRIx32
                           " lock=%"PRIu32" irq=%08"PRIx32"/%08"PRIx32" phase=%08"PRIx32
                           " ep0=%08"PRIx32" pub=%08"PRIx32" rxerr=%08"PRIx32"\n",
                           r->cursor,r->pid,r->before_valid,r->cutoff,r->clock_before,r->clock_after,
                           r->address_cycle,r->sof,
                           r->address,r->owner,r->address_before,r->address_after,
                           r->owner_before,r->owner_after,r->selected,r->reason,
                           r->in0,r->out0,r->buffers,r->sie,r->sm,r->ints,r->blocked_buffers,
                           r->blocked_sie,r->missed_lock,r->irq_enter,r->irq_exit,r->core0_phase,r->ep0_word,
                           r->publication_sequence,r->rx_error);
    } else {
        // Live drops stay visible even without another snapshot trigger.
        queued = probe_debug_printf("[HUB_FLIGHT_END] next=%08"PRIx32" n=%"PRIu32" lost=%"PRIu32"\n",
                                    h->cursor,h->count,out_trace_snapshot_drops);
        if (queued < 0) return true;
        out_trace_snapshot_head = (out_trace_snapshot_head+1u)%OUT_TRACE_SNAPSHOTS;
        --out_trace_snapshot_count;
        out_trace_dump_line = 0;
        return true;
    }
    if (queued >= 0) ++out_trace_dump_line;
    return true;
}
#endif

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
    uint32_t now = time_us_32();
    if (usb_hw->ep_nak_stall_status & (1u << 30)) {
        ++root_naks;
        hw_clear_bits(&usb_hw->ep_nak_stall_status,1u << 30);
    }
    for (unsigned p = 0; p < CHILDREN; ++p) {
        if ((ports[p].status & RESET) && (int32_t)(now-ports[p].deadline) >= 0) {
            if (default_device != NONE && default_device != p+1) { failed = true; return; }
            ports[p].status = (ports[p].status & ~RESET) | ENABLE;
            ports[p].change |= C_RESET; addresses[p+1] = 0; default_device = p+1; publish_addresses();
        }
    }
    if (devices[0].configuration && !devices[0].ep[2].busy && !devices[0].ep[2].halted) {
        uint8_t changed = 0;
        for (unsigned p = 0; p < CHILDREN; ++p)
            if (ports[p].change) changed |= 1u << (p+1u);
        if (changed) {
            endpoint_t* ep = &devices[0].ep[2]; ep->data[0] = changed;
            ep->length = 1; ep->sent = 0; ep->busy = ep->flush = true; ep->zlp = false;
            transmit_next(0,2);
        }
    }
    static uint32_t last_log;
#if defined(SWITCH2_PROBE_TRACE_NATIVE_INPUT)
    if (out_trace_task(now)) last_log = now;
#endif
    if ((uint32_t)(now-last_log) >= 1000000u) {
        last_log = now;
#if CHILDREN == 2
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
#else
        probe_debug_printf("[NATIVE_HUB] children=%u switch=%"PRIu32" busy=%"PRIu32
                           " slow=%"PRIu32" late=%"PRIu32"/%"PRIu32"\n",
                           CHILDREN,switches,missed_switches,slow_switches,minimum_lateness,maximum_lateness);
        probe_debug_printf("[HUB_SIE] owner=%u sie=%08"PRIx32" nak=%08"PRIx32
                           " txerr=%08"PRIx32" rxerr=%08"PRIx32" ep1=%08"PRIx32"/%08"PRIx32"\n",
                           active_device,usb_hw->sie_status,usb_hw->ep_nak_stall_status,
                           usb_hw->ep_tx_error,usb_hw->ep_rx_error,endpoint_regs()[0],buffer_regs()[2]);
        probe_debug_printf("[HUB_ROUTE] lock=%"PRIu32" blocked=%08"PRIx32"/%08"PRIx32" rootnak=%"PRIu32"\n",
                           missed_lock,blocked_buffers,blocked_sie,root_naks);
        for (unsigned slot = 0; slot < DEVICES; ++slot) {
            probe_debug_printf("[HUB_SLOT] slot=%u addr=%u cfg=%u setup=%"PRIu32
                               " in=%"PRIu32" out=%"PRIu32" hits=%"PRIu32
                               " hidbusy=%u status=%04x change=%04x\n",
                               slot,addresses[slot],devices[slot].configuration,setup_count[slot],
                               input_count[slot],output_count[slot],token_hits[slot],devices[slot].ep[2].busy,
                               slot ? ports[slot-1].status : 0,slot ? ports[slot-1].change : 0);
        }
#endif
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
        probe_debug_printf("[HUB_OBSERVER] returns=%"PRIu32" discard=%"PRIu32" header=%08"PRIx32
                           " enabled=%"PRIu32" fault=%"PRIu32" generation=%"PRIu32" reader=%"PRIu32
                           " gpio=%08"PRIx32" mux=%08"PRIx32"\n",
                           observer.capture_returns,observer.discarded_headers,observer.last_discarded_header,
                           observer.enabled,observer.fatal_fault,observer.published_generation,observer.reader_index,
                           sio_hw->gpio_hi_in,usb_hw->muxing);
        probe_debug_printf("[HUB_ROOT_REPLY] in=%"PRIu32" cutoff=%08"PRIx32
                           " header=%08"PRIx32" seen=%08"PRIx32" eop=%08"PRIx32
                           " before_in=%"PRIu32" before_cutoff=%08"PRIx32
                           " before_header=%08"PRIx32" before_seen=%08"PRIx32" before_eop=%08"PRIx32"\n",
                           observer.root_in_count,observer.root_in_cutoff,observer.root_header,
                           observer.root_header_cycle,observer.root_eop_cycle,
                           observer.before_setup_in_count,observer.before_setup_in_cutoff,
                           observer.before_setup_header,observer.before_setup_header_cycle,
                           observer.before_setup_eop_cycle);
#if CHILDREN == 2
        probe_debug_printf("[HUB_PORTS] status=%04x/%04x change=%04x/%04x\n",
                           ports[0].status,ports[1].status,ports[0].change,ports[1].change);
#endif
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
