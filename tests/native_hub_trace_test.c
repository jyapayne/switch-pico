#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hardware_stub.h"

static sio_hw_t* trace_test_sio(void);
static bool trace_test_lock_busy;
static bool trace_test_trylock(spin_lock_t* lock) {
    return !trace_test_lock_busy && spin_try_lock_unsafe(lock);
}
#undef sio_hw
#define sio_hw trace_test_sio()
#define spin_try_lock_unsafe trace_test_trylock

#define NATIVE_TEST_EXTERNAL_LOG 1
#include "native_hub_transport_fixture.c"

// Each scenario runs in a fresh process: the real recorder's BSS is its reset.
// Hardware registers model host/IRQ observations, not packets on a USB wire.
#define TEST_STALL_US 200000u
#define TEST_LINE_US 50000u
#define TEST_RETAIN 64u
#define TEST_PID_OUT 0xe1u
#define TEST_PID_IN 0x69u
#define TEST_PID_SETUP 0x2du
#define TEST_LOG_LINES 1024u
#define TEST_LINE_SIZE 512u

static char log_lines[TEST_LOG_LINES][TEST_LINE_SIZE];
static unsigned log_count;
static bool logger_full, retry_each_line;
static unsigned retry_count, rejected_lines;
static char rejected_line[TEST_LINE_SIZE];
static uint8_t vendor_reply[96];
static bool supersede_in_callback;
static uint8_t handover_reply[DEVICES][83];
static unsigned handover_acks[DEVICES];
static bool supersede_status_in_callback;
static bool reject_status_in_callback;
static unsigned handover_data_callbacks[DEVICES];
static uint8_t approved_reply[DEVICES][2u*PACKET];
typedef struct {
    uint8_t slot, stage;
    tusb_control_request_t request;
} approved_callback_t;
static approved_callback_t approved_callbacks[64];
static unsigned approved_callback_count;
static bool reset_on_root_complete;

typedef struct {
    usb_device_dpram_t dpram;
    uint32_t stall;
} hardware_bank_t;
static bool clock_steps, commit_probe, commit_handover;
static uint8_t commit_address, commit_owner;
static uint32_t commit_cycle, commit_buffers[CHANNELS], commit_controls[4];
static uint32_t commit_root_control, commit_stall;
static hardware_bank_t commit_bank, commit_latched_bank;

static hardware_bank_t hardware_bank(void) {
    hardware_bank_t bank;
    memcpy(&bank.dpram,usb_dpram,sizeof(bank.dpram));
    bank.stall = usb_hw->ep_stall_arm;
    return bank;
}

static void expect_bank_unchanged(const hardware_bank_t* bank) {
    assert(memcmp(&bank->dpram,usb_dpram,sizeof(bank->dpram)) == 0);
    assert(usb_hw->ep_stall_arm == bank->stall);
}

static sio_hw_t* trace_test_sio(void) {
    if (clock_steps) ++native_test_sio.mtime;
    if (commit_probe && usb_hw->dev_addr_ctrl == commit_address) {
        commit_probe = false;
        commit_cycle = native_test_sio.mtime;
        // Latch the bank at the first clock access after the address store.
        // Later packet delivery uses this metadata, not a repaired return-time
        // bank. This models the publication contract, NOT physical SIE timing,
        // packet acceptance, or how hardware ACKs a mismatched DATA PID.
        commit_latched_bank = hardware_bank();
        assert(active_device == commit_owner);
        if (commit_handover) {
            const uint32_t* buffers = (const uint32_t*)&commit_latched_bank.dpram.ep_buf_ctrl[0];
            const uint32_t* controls = (const uint32_t*)&commit_latched_bank.dpram.ep_ctrl[0];
            for (unsigned channel = 0; channel < CHANNELS; ++channel) {
                assert(!(buffers[channel] & USB_BUF_CTRL_AVAIL));
                assert(buffers[channel] == commit_buffers[channel] &&
                    "address commit exposed missing or outgoing packet metadata");
            }
            for (unsigned i = 0; i < 4; ++i)
                assert(controls[i] == commit_controls[i]);
            assert(commit_latched_bank.dpram.ep_ctrl[14].in == commit_root_control);
            assert(commit_latched_bank.stall == commit_stall);
            // Root EP15 has independent storage; its buffer is not the child
            // EP1 bank. Visibility follows the incoming root endpoint control.
            assert(commit_latched_bank.dpram.ep_buf_ctrl[15].in ==
                commit_bank.dpram.ep_buf_ctrl[15].in);
        } else expect_bank_unchanged(&commit_bank);
    }
    return &native_test_sio;
}

static uint16_t handover_length(uint8_t slot) {
    return slot == 0 ? 16 : slot % 2u ? PACKET : sizeof(handover_reply[slot]);
}

int probe_debug_printf(const char* format, ...) {
    char line[TEST_LINE_SIZE];
    va_list arguments;
    va_start(arguments,format);
    int length = vsnprintf(line,sizeof(line),format,arguments);
    va_end(arguments);
    assert(length >= 0 && (size_t)length < sizeof(line));
    if (reset_on_root_complete && strstr(line,"[HUB_CTRL] complete ") == line) {
        reset_on_root_complete = false;
        native_test_bus_reset(false);
    }
    if (strncmp(line,"[HUB_FLIGHT",11) != 0) return length;
    if (logger_full) return -1;
    if (retry_each_line) {
        if (!retry_count) memcpy(rejected_line,line,(size_t)length+1u);
        else assert(strcmp(rejected_line,line) == 0 && "a rejected dump line was skipped or changed");
        if (++retry_count <= 2u) { ++rejected_lines; return -1; }
        retry_count = 0;
    }
    assert(log_count < TEST_LOG_LINES);
    memcpy(log_lines[log_count++],line,(size_t)length+1u);
    return length;
}

void reset_usb_boot(uint32_t gpio_mask, uint32_t disable_mask) {
    (void)gpio_mask; (void)disable_mask;
    assert(false && "transport failed closed during trace regression");
    abort();
}

bool tud_vendor_control_xfer_cb(uint8_t slot, uint8_t stage, const tusb_control_request_t* request) {
    if (request->bmRequestType == 0xc0 && request->bRequest == 0x5d) {
        if (stage == CONTROL_STAGE_SETUP) {
            if (!request->wValue || request->wValue > sizeof(approved_reply[slot]) ||
                !request->wLength) return false;
            return native_hub_control_xfer(slot,request,approved_reply[slot],request->wValue,true);
        }
        assert(stage == CONTROL_STAGE_DATA || stage == CONTROL_STAGE_ACK);
        assert(approved_callback_count < sizeof(approved_callbacks)/sizeof(approved_callbacks[0]));
        approved_callbacks[approved_callback_count++] = (approved_callback_t){
            .slot = slot, .stage = stage, .request = *request,
        };
        return true; // SETUP validated the reply; DATA never rejects status.
    }
    if (request->bmRequestType == 0xc0 && request->bRequest == 0x5b) {
        if (stage == CONTROL_STAGE_ACK) ++handover_acks[slot];
        if (stage == CONTROL_STAGE_DATA) {
            ++handover_data_callbacks[slot];
            if (reject_status_in_callback) return false;
        }
        if (stage == CONTROL_STAGE_DATA && supersede_status_in_callback) {
            supersede_status_in_callback = false;
            const tusb_control_request_t replacement = {
                .bmRequestType = 0x80, .bRequest = TUSB_REQ_GET_DESCRIPTOR,
                .wValue = TUSB_DESC_DEVICE << 8, .wLength = 18,
            };
            assert(native_test_setup(slot,&replacement,false));
        }
        return stage != CONTROL_STAGE_SETUP || native_hub_control_xfer(
            slot,request,handover_reply[slot],handover_length(slot),false);
    }
    if (request->bmRequestType != 0xc0 || request->bRequest != 0x5a) return false;
    if (stage != CONTROL_STAGE_SETUP) return true;
    bool accepted = native_hub_control_xfer(slot,request,vendor_reply,sizeof(vendor_reply),false);
    if (supersede_in_callback) {
        supersede_in_callback = false;
        const tusb_control_request_t replacement = {
            .bmRequestType = 0x80, .bRequest = TUSB_REQ_GET_DESCRIPTOR,
            .wValue = TUSB_DESC_DEVICE << 8, .wLength = 18,
        };
        native_test_time_us += 7u;
        assert(native_test_setup(slot,&replacement,false));
    }
    return accepted;
}

static bool tagged(const char* line, const char* tag) {
    return strncmp(line,tag,strlen(tag)) == 0;
}

static unsigned count_tag(const char* tag) {
    unsigned count = 0;
    for (unsigned i = 0; i < log_count; ++i) count += tagged(log_lines[i],tag);
    return count;
}

static const char* snapshot_line(unsigned snapshot, const char* tag) {
    unsigned current = 0;
    for (unsigned i = 0; i < log_count; ++i) {
        if (tagged(log_lines[i],"[HUB_FLIGHT_FREEZE]")) ++current;
        if (current == snapshot+1u && tagged(log_lines[i],tag)) return log_lines[i];
    }
    assert(false && "required snapshot diagnostic was not emitted");
    return NULL;
}

static uint32_t field(const char* line, const char* name, unsigned base) {
    const char* value = strstr(line,name);
    assert(value != NULL && "required diagnostic field is absent");
    value += strlen(name);
    char* end;
    unsigned long result = strtoul(value,&end,(int)base);
    assert(end != value && result <= UINT32_MAX);
    return (uint32_t)result;
}

static const char* record_line(unsigned snapshot, unsigned index) {
    unsigned current = 0, record = 0;
    for (unsigned i = 0; i < log_count; ++i) {
        const char* line = log_lines[i];
        if (tagged(line,"[HUB_FLIGHT_FREEZE]")) ++current;
        if (current == snapshot+1u && tagged(line,"[HUB_FLIGHT]") && record++ == index)
            return line;
    }
    assert(false && "required retained token was not emitted");
    return NULL;
}

static const char* expect_observation(unsigned snapshot, unsigned index, uint8_t address,
                                      uint8_t owner, uint8_t pid, uint32_t cycle,
                                      bool selected) {
    const char* line = record_line(snapshot,index);
    assert(field(line," cutoff=",16) == 0x70000000u+cycle);
    assert(field(line," pid=",16) == pid);
    assert(field(line," pre=",10) == 0 && field(line," ok=",10) == selected);
    if (!selected) assert(field(line," commit=",16) == 0);
    char expected[80];
    snprintf(expected,sizeof(expected)," req=%02x/%u ",(unsigned)address,(unsigned)owner);
    assert(strstr(line,expected));
    assert(strstr(line," addr=00000000/") && strstr(line," owner=255/"));
    snprintf(expected,sizeof(expected)," clock=00000000/%08"PRIx32" ",cycle);
    assert(strstr(line,expected));
    return line;
}

static void expect_lost(unsigned snapshots, uint32_t lost) {
    for (unsigned i = 0; i < snapshots; ++i)
        assert(field(snapshot_line(i,"[HUB_FLIGHT_END]")," lost=",10) == lost);
}

static void poll(unsigned ticks) {
    for (unsigned i = 0; i < ticks; ++i) native_test_advance(TEST_LINE_US);
}

static void expect_dump_order(unsigned snapshots) {
    unsigned line = 0;
    const char* headers[] = {
        "[HUB_FLIGHT_FREEZE]", "[HUB_FLIGHT_CONTEXT]", "[HUB_FLIGHT_CONTROL]",
        "[HUB_FLIGHT_CONTROL_CLOCK]", "[HUB_FLIGHT_STATUS_OUT]",
    };
    for (unsigned snapshot = 0; snapshot < snapshots; ++snapshot) {
        assert(line < log_count);
        unsigned records = field(log_lines[line]," n=",10);
        for (unsigned h = 0; h < sizeof(headers)/sizeof(headers[0]); ++h) {
            assert(line < log_count && tagged(log_lines[line],headers[h]));
            ++line;
        }
        if (CHILDREN != 2u) {
            for (unsigned slot = 1; slot <= CHILDREN; ++slot) {
                assert(line < log_count && tagged(log_lines[line],"[HUB_FLIGHT_INPUT]"));
                assert(field(log_lines[line++]," slot=",10) == slot);
            }
        }
        for (unsigned record = 0; record < records; ++record) {
            assert(line < log_count && tagged(log_lines[line],"[HUB_FLIGHT]"));
            ++line;
        }
        assert(line < log_count && tagged(log_lines[line],"[HUB_FLIGHT_END]"));
        assert(field(log_lines[line++]," n=",10) == records);
    }
}

static void dump_through(unsigned snapshots) {
    unsigned attempts = 0;
    while (count_tag("[HUB_FLIGHT_END]") < snapshots && attempts++ < 1000u)
        native_test_advance(TEST_LINE_US);
    assert(count_tag("[HUB_FLIGHT_END]") == snapshots && "snapshot dump failed to finish");
    expect_dump_order(snapshots);
}

static void configure_child(uint8_t slot) {
    const tusb_control_request_t request = {
        .bRequest = TUSB_REQ_SET_CONFIGURATION, .wValue = 1,
    };
    uint8_t data[PACKET];
    uint16_t length = UINT16_MAX;
    assert(native_test_setup(slot,&request,true));
    assert(native_test_in(slot,data,&length,true) && length == 0);
    assert(native_hub_mounted(slot-1u));
}

static void input_completion(uint8_t slot) {
    const uint8_t payload[] = {0x12,0x34,0x56};
    uint8_t data[PACKET];
    uint16_t length = 0;
    assert(native_hub_hid_report(slot-1u,0x30,payload,sizeof(payload)));
    assert(native_test_private_in(slot,0x81,data,&length));
    assert(length == sizeof(payload)+1u && data[0] == 0x30);
    assert(memcmp(data+1,payload,sizeof(payload)) == 0);
    native_test_drain();
}

static void selections(uint32_t marker, unsigned count) {
    for (unsigned i = 0; i < count; ++i) {
        uint8_t slot = 1u + i % CHILDREN;
        sio_hw->mtime = 1000u+i;
        assert(native_hub_select_device(addresses[slot],slot,marker+i));
        native_hub_note_selected_token(addresses[slot],slot,marker+i,TEST_PID_OUT);
    }
}

static void expect_records(unsigned snapshot, uint32_t marker, unsigned count) {
    unsigned current = 0, found = 0;
    for (unsigned i = 0; i < log_count; ++i) {
        const char* line = log_lines[i];
        if (tagged(line,"[HUB_FLIGHT_FREEZE]")) ++current;
        if (current != snapshot+1u || !tagged(line,"[HUB_FLIGHT]")) continue;
        assert(found < count);
        assert(field(line," cutoff=",16) == marker+found);
        assert(field(line," pid=",16) == TEST_PID_OUT);
        assert(field(line," ok=",10) == 1);
        ++found;
    }
    assert(found == count && "snapshot lost selections or read the overwritten live ring");
    assert(field(snapshot_line(snapshot,"[HUB_FLIGHT_FREEZE]")," n=",10) == count);
    assert(field(snapshot_line(snapshot,"[HUB_FLIGHT_END]")," n=",10) == count);
    assert(field(snapshot_line(snapshot,"[HUB_FLIGHT_FREEZE]")," reason=",10) == 0);
}

static void root_read(void) {
    const tusb_control_request_t request = {
        .bmRequestType = 0x80, .bRequest = TUSB_REQ_GET_STATUS, .wLength = 2,
    };
    uint8_t data[PACKET];
    uint16_t length = 0;
    assert(native_test_setup(0,&request,true));
    assert(native_test_in(0,data,&length,true) && length == 2);
    assert(data[0] == 1 && data[1] == 0);
    assert(native_test_out(0,NULL,0,true));
}

typedef struct {
    uint32_t time_us, generation;
    uint8_t slot;
} marker_receipt_t;

static tusb_control_request_t marker_request(uint16_t slot) {
    const tusb_control_request_t request = {
        .bmRequestType = 0xc0, .bRequest = 0x5e, .wValue = 0x5452,
        .wIndex = slot, .wLength = 16,
    };
    return request;
}

static uint32_t reply_u32(const uint8_t* data) {
    return (uint32_t)data[0] | (uint32_t)data[1] << 8 |
        (uint32_t)data[2] << 16 | (uint32_t)data[3] << 24;
}

static marker_receipt_t capture_marker(uint8_t slot, uint8_t status) {
    const tusb_control_request_t request = marker_request(slot);
    uint8_t data[PACKET];
    memset(data,0xa5,sizeof(data));
    uint16_t length = UINT16_MAX;
    assert(native_test_setup(0,&request,true));
    assert(native_test_in(0,data,&length,true) && length == 16);
    assert(memcmp(data,"NHTR",4) == 0 && data[4] == 1);
    assert(data[5] == status && data[6] == slot && data[7] == 0);
    const marker_receipt_t receipt = {
        .time_us = reply_u32(data+8), .generation = reply_u32(data+12), .slot = data[6],
    };
    if (status == 1) assert(receipt.time_us == 0 && receipt.generation == 0);
    assert(native_test_out(0,NULL,0,true));
    return receipt;
}

static void expect_marker(unsigned snapshot, const marker_receipt_t* receipt) {
    const char* header = snapshot_line(snapshot,"[HUB_FLIGHT_FREEZE]");
    const char* control = snapshot_line(snapshot,"[HUB_FLIGHT_CONTROL]");
    assert(field(header," reason=",10) == 3);
    assert(field(header," us=",10) == receipt->time_us);
    assert(field(control," slot=",10) == receipt->slot);
    assert(field(control," gen=",10) == receipt->generation);
    const char* clock = snapshot_line(snapshot,"[HUB_FLIGHT_CONTROL_CLOCK]");
    assert(field(clock," slot=",10) == receipt->slot);
    assert(field(clock," gen=",10) == receipt->generation);
    const char* status = snapshot_line(snapshot,"[HUB_FLIGHT_STATUS_OUT]");
    assert(field(status," slot=",10) == receipt->slot);
    assert(field(status," gen=",10) == receipt->generation);
}

static void live_wrap(void) {
    configure_child(CHILDREN);
    selections(0x10000000u,TEST_RETAIN);
    input_completion(CHILDREN);
    native_test_advance(TEST_STALL_US);
    assert(count_tag("[HUB_FLIGHT_FREEZE]") == 1);
    assert(count_tag("[HUB_FLIGHT_END]") == 0);

    // The first idle dump is still underway. New successful selections must
    // remain observable in a second snapshot, not disappear until UART drains.
    input_completion(CHILDREN);
    selections(0x20000000u,TEST_RETAIN);
    native_test_advance(TEST_STALL_US);
    selections(0x30000000u,3u*TEST_RETAIN);
    dump_through(2);
    expect_records(0,0x10000000u,TEST_RETAIN);
    expect_records(1,0x20000000u,TEST_RETAIN);
    poll(100);
    assert(count_tag("[HUB_FLIGHT_FREEZE]") == 2);
}

static void root_does_not_rearm(void) {
    configure_child(CHILDREN);
    selections(0x10000000u,4);
    input_completion(CHILDREN);
    native_test_advance(TEST_STALL_US);
    for (unsigned i = 0; i < 40; ++i) {
        root_read();
        native_test_advance(TEST_LINE_US);
    }
    dump_through(1);
    for (unsigned i = 0; i < 40; ++i) {
        root_read();
        native_test_advance(TEST_LINE_US);
    }
    assert(count_tag("[HUB_FLIGHT_FREEZE]") == 1 && "root control traffic rearmed input-idle capture");
    input_completion(CHILDREN);
    native_test_advance(TEST_STALL_US-1u);
    assert(count_tag("[HUB_FLIGHT_FREEZE]") == 1);
    native_test_advance(1);
    dump_through(2);
    assert(count_tag("[HUB_FLIGHT_FREEZE]") == 2);
}

static void queue_pressure(void) {
    configure_child(CHILDREN);
    selections(0x10000000u,TEST_RETAIN);
    input_completion(CHILDREN);
    native_test_advance(TEST_STALL_US);
    assert(count_tag("[HUB_FLIGHT_FREEZE]") == 1);
    logger_full = true;
    for (unsigned request = 2; request <= 3; ++request) {
        input_completion(CHILDREN);
        selections(request*0x10000000u,TEST_RETAIN);
        native_test_advance(TEST_STALL_US);
    }
    const tusb_control_request_t pending = {
        .bmRequestType = 0x80, .bRequest = TUSB_REQ_GET_DESCRIPTOR,
        .wValue = TUSB_DESC_DEVICE << 8, .wLength = 18,
    };
    assert(native_test_setup(CHILDREN,&pending,true));
    native_test_advance(TEST_STALL_US); // One more drop, from pending control.
    selections(0x40000000u,3u*TEST_RETAIN);
    poll(40); // Queue pressure consumes both idle and pending one-shots.
    logger_full = false;
    dump_through(2);
    poll(100); // No later trigger is available to reveal the lost request.
    assert(count_tag("[HUB_FLIGHT_FREEZE]") == 2);
    assert(count_tag("[HUB_FLIGHT_END]") == 2);
    expect_records(0,0x10000000u,TEST_RETAIN);
    expect_records(1,0x20000000u,TEST_RETAIN);
    uint32_t lost = 0;
    for (unsigned i = 0; i < log_count; ++i)
        if (strstr(log_lines[i]," lost=")) {
            uint32_t reported = field(log_lines[i]," lost=",10);
            assert(reported <= 2u && "full queue repeatedly counted an unchanged trigger");
            if (reported > lost) lost = reported;
        }
    assert(lost == 2u && "dropped snapshot requests were never reported");
}

static void backpressure(bool full) {
    configure_child(CHILDREN);
    selections(0x10000000u,TEST_RETAIN);
    input_completion(CHILDREN);
    retry_each_line = full;
    native_test_advance(TEST_STALL_US);
    dump_through(1);
    assert(count_tag("[HUB_FLIGHT_FREEZE]") == 1);
    assert(count_tag("[HUB_FLIGHT_CONTEXT]") == 1);
    assert(count_tag("[HUB_FLIGHT_CONTROL]") == 1);
    assert(count_tag("[HUB_FLIGHT_CONTROL_CLOCK]") == 1);
    assert(count_tag("[HUB_FLIGHT_STATUS_OUT]") == 1);
    expect_records(0,0x10000000u,TEST_RETAIN);
    if (full) assert(rejected_lines == 2u*log_count && retry_count == 0);
    for (unsigned i = 0; i < log_count; ++i) fputs(log_lines[i],stdout);
}

static tusb_control_request_t vendor_request(void) {
    const tusb_control_request_t request = {
        .bmRequestType = 0xc0, .bRequest = 0x5a, .wValue = 0x1122,
        .wIndex = 0x3344, .wLength = sizeof(vendor_reply),
    };
    return request;
}

static tusb_control_request_t descriptor_request(void) {
    const tusb_control_request_t request = {
        .bmRequestType = 0x80, .bRequest = TUSB_REQ_GET_DESCRIPTOR,
        .wValue = TUSB_DESC_DEVICE << 8, .wLength = 18,
    };
    return request;
}

static void expect_clock(unsigned snapshot, uint32_t setup, uint32_t arm,
                         uint32_t complete, uint32_t flags, uint16_t arm_length,
                         uint16_t complete_length) {
    const char* clock = snapshot_line(snapshot,"[HUB_FLIGHT_CONTROL_CLOCK]");
    const char* control = snapshot_line(snapshot,"[HUB_FLIGHT_CONTROL]");
    assert(field(clock," slot=",10) == field(control," slot=",10));
    assert(field(clock," gen=",10) == field(control," gen=",10));
    assert(field(clock," setup=",16) == setup);
    assert(field(clock," arm=",16) == arm);
    assert(field(clock," complete=",16) == complete);
    assert(field(clock," flags=",16) == flags);
    assert(field(clock," pid=",10) == 1); // First EP0 publication is DATA1.
    assert(field(clock," arm_len=",10) == arm_length);
    assert(field(clock," len=",10) == complete_length);
}

static const char* expect_status_out(unsigned snapshot, uint32_t arm, uint32_t complete,
                                     uint8_t flags, uint16_t length) {
    const char* status = snapshot_line(snapshot,"[HUB_FLIGHT_STATUS_OUT]");
    const char* control = snapshot_line(snapshot,"[HUB_FLIGHT_CONTROL]");
    assert(field(status," slot=",10) == field(control," slot=",10));
    assert(field(status," gen=",10) == field(control," gen=",10));
    assert(field(status," arm=",16) == arm);
    assert(field(status," complete=",16) == complete);
    assert(field(status," flags=",16) == flags);
    assert(field(status," len=",10) == length);
    return status;
}

static void pending_one_shot(void) {
    const tusb_control_request_t request = vendor_request();
    assert(native_test_setup(CHILDREN,&request,true));
    native_test_advance(TEST_STALL_US-1u);
    assert(count_tag("[HUB_FLIGHT_FREEZE]") == 0);
    native_test_advance(1);
    dump_through(1);
    poll(20);
    assert(count_tag("[HUB_FLIGHT_FREEZE]") == 1);

    uint8_t data[PACKET];
    uint16_t length = 0;
    assert(native_test_in(CHILDREN,data,&length,true) && length == PACKET);
    assert(memcmp(data,vendor_reply,length) == 0);
    native_test_advance(TEST_STALL_US);
    dump_through(2);
    poll(20);
    assert(count_tag("[HUB_FLIGHT_FREEZE]") == 2);
    assert(native_test_in(CHILDREN,data,&length,true) && length == sizeof(vendor_reply)-PACKET);
    assert(memcmp(data,vendor_reply+PACKET,length) == 0);
    native_test_advance(TEST_STALL_US);
    dump_through(3);
    poll(20);
    assert(count_tag("[HUB_FLIGHT_FREEZE]") == 3);

    const char* first = snapshot_line(0,"[HUB_FLIGHT_CONTROL]");
    const char* second = snapshot_line(1,"[HUB_FLIGHT_CONTROL]");
    const char* third = snapshot_line(2,"[HUB_FLIGHT_CONTROL]");
    assert(strstr(first," pos=0/96 ") && strstr(second," pos=64/96 ") && strstr(third," pos=96/96 "));
    assert(field(first," gen=",10) == field(second," gen=",10));
    assert(field(second," gen=",10) == field(third," gen=",10));
    assert(field(first," stage=",10) == field(second," stage=",10));
    assert(field(second," stage=",10) != field(third," stage=",10));
    for (unsigned i = 0; i < 3; ++i)
        assert(field(snapshot_line(i,"[HUB_FLIGHT_FREEZE]")," reason=",10) == 1);
    // A zero cycle is valid evidence, not a missing-sample sentinel. Later
    // 32-byte completion must not replace the first 64-byte publication.
    expect_clock(0,0,0,0,1,PACKET,0);
    expect_clock(1,0,0,0,3,PACKET,PACKET);
    expect_clock(2,0,0,0,3,PACKET,PACKET);
    expect_status_out(0,0,0,0,0);
    expect_status_out(1,0,0,0,0);
    const char* status = expect_status_out(2,0,0,1,0);
    assert((field(status," shadow_out=",16) &
        (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_DATA1_PID | USB_BUF_CTRL_LEN_MASK)) ==
        (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_DATA1_PID));

    // Superseding a state already captured by the pending-control trigger must
    // not queue the same generation/stage/position for a second time.
    const tusb_control_request_t replacement = descriptor_request();
    assert(native_test_setup(CHILDREN,&replacement,true));
    assert(count_tag("[HUB_FLIGHT_FREEZE]") == 3);
    assert(native_test_in(CHILDREN,data,&length,true) && length == 18);
    assert(native_test_out(CHILDREN,NULL,0,true));
    poll(100);
    assert(count_tag("[HUB_FLIGHT_FREEZE]") == 3);
}


static void expect_control_tokens(unsigned snapshot, const uint32_t* cycles, unsigned count) {
    unsigned current = 0, found = 0;
    for (unsigned i = 0; i < log_count; ++i) {
        const char* line = log_lines[i];
        if (tagged(line,"[HUB_FLIGHT_FREEZE]")) ++current;
        if (current != snapshot+1u || !tagged(line,"[HUB_FLIGHT]")) continue;
        assert(found < count);
        assert(field(line," cutoff=",16) == 0x70000000u+cycles[found]);
        assert(field(line," pid=",16) == (found % 2u ? TEST_PID_IN : TEST_PID_SETUP));
        assert(field(line," pre=",10) == 0 && field(line," ok=",10) == 1);
        char clock[48];
        snprintf(clock,sizeof(clock)," clock=00000000/%08"PRIx32" ",cycles[found]);
        assert(strstr(line,clock));
        ++found;
    }
    assert(found == count && "accepted SETUP/first-IN token coverage is incomplete or duplicated");
}

static void control_token(uint8_t slot, uint8_t pid, uint32_t cycle) {
    sio_hw->mtime = cycle;
    assert(native_test_select(slot));
    native_hub_note_selected_token(addresses[slot],slot,0x70000000u+cycle,pid);
}

static void superseded(void) {
    const uint8_t slot = CHILDREN;
    const tusb_control_request_t old_request = vendor_request();
    const tusb_control_request_t new_request = descriptor_request();
    control_token(slot,TEST_PID_SETUP,100);
    sio_hw->mtime = 200;
    assert(native_test_setup(slot,&old_request,false));
    sio_hw->mtime = 300;
    native_test_drain();
    control_token(slot,TEST_PID_IN,400);
    control_token(slot,TEST_PID_IN,450); // Only the first accepted IN is retained.
    uint8_t data[PACKET];
    uint16_t length = 0;
    sio_hw->mtime = 500;
    assert(native_test_in(slot,data,&length,false) && length == PACKET);
    assert(memcmp(data,vendor_reply,length) == 0);
    control_token(slot,TEST_PID_SETUP,600);
    sio_hw->mtime = 700;
    assert(native_test_setup(slot,&new_request,false));
    // Both events are queued. Hardware completion time must survive delayed
    // foreground processing and the newer device generation's protocol abort.
    sio_hw->mtime = 800;
    native_test_drain();
    control_token(slot,TEST_PID_IN,900);
    control_token(slot,TEST_PID_IN,950);
    sio_hw->mtime = 1000;
    assert(native_test_in(slot,data,&length,false) && length == 18);
    sio_hw->mtime = 1100;
    native_test_drain();
    native_test_advance(TEST_STALL_US);
    dump_through(2);
    poll(100);
    assert(count_tag("[HUB_FLIGHT_FREEZE]") == 2);
    const char* old = snapshot_line(0,"[HUB_FLIGHT_CONTROL]");
    const char* next = snapshot_line(1,"[HUB_FLIGHT_CONTROL]");
    assert(field(old," slot=",10) == slot && field(next," slot=",10) == slot);
    assert(field(next," gen=",10) == field(old," gen=",10)+1u);
    assert(strstr(old," setup=c0/5a v=1122 i=3344 n=96 "));
    assert(strstr(old," pos=0/96 "));
    assert(strstr(next," setup=80/06 v=0100 i=0000 n=18 "));
    assert(strstr(next," pos=18/18 "));
    assert(field(snapshot_line(0,"[HUB_FLIGHT_FREEZE]")," reason=",10) == 2);
    assert(field(snapshot_line(1,"[HUB_FLIGHT_FREEZE]")," reason=",10) == 1);
    expect_clock(0,200,300,500,3,PACKET,PACKET);
    expect_clock(1,700,800,1000,3,18,18);
    expect_status_out(0,0,0,0,0);
    expect_status_out(1,1000,0,1,0); // Standard reads arm at final-IN IRQ, not drain at 1100.
    const uint32_t cycles[] = {100,400,600,900};
    expect_control_tokens(0,cycles,3);
    expect_control_tokens(1,cycles,4);
}

static void immediate_supersession(void) {
    supersede_in_callback = true;
    const tusb_control_request_t request = vendor_request();
    assert(native_test_setup(CHILDREN,&request,true));
    uint8_t data[PACKET];
    uint16_t length = 0;
    assert(native_test_in(CHILDREN,data,&length,true) && length == 18);
    assert(native_test_out(CHILDREN,NULL,0,true));
    dump_through(1);
    const char* header = snapshot_line(0,"[HUB_FLIGHT_FREEZE]");
    assert(field(header," reason=",10) == 2);
    assert(field(header," quiet=",10) == 7u && "immediate supersession reported uptime instead of transfer age");
}

static bool routed_token(uint8_t address, uint8_t owner, uint8_t pid, uint32_t cycle) {
    sio_hw->mtime = cycle;
    const uint32_t cutoff = 0x70000000u+cycle;
    const bool selected = native_hub_select_device(address,owner,cutoff);
    if (selected) native_hub_note_selected_token(address,owner,cutoff,pid);
    else native_hub_note_failed_select(address,owner,cutoff,pid);
    return selected;
}

static const char* expect_publication(unsigned snapshot, unsigned index, uint8_t slot,
                                      uint32_t cycle, uint32_t ticket) {
    const char* line = expect_observation(snapshot,index,addresses[slot],slot,TEST_PID_IN,cycle,true);
    const char* clock = snapshot_line(snapshot,"[HUB_FLIGHT_CONTROL_CLOCK]");
    const char* control = snapshot_line(snapshot,"[HUB_FLIGHT_CONTROL]");
    assert(field(line," why=",16) == 0x20 && field(line," pub=",16) == ticket);
    assert(field(clock," flags=",16) & 1u);
    assert(field(clock," slot=",10) == slot && field(control," slot=",10) == slot);
    assert(field(clock," gen=",10) == field(control," gen=",10));
    assert(field(clock," pub=",16) == ticket);
    return line;
}

static void complete_descriptor(uint8_t slot) {
    uint8_t data[PACKET];
    uint16_t length = 0;
    assert(native_test_in(slot,data,&length,true) && length == 18);
    assert(memcmp(data,hub_device,length) == 0);
    assert(native_test_out(slot,NULL,0,true));
}

static void delayed_publication(void) {
    const uint8_t slot = CHILDREN;
    const tusb_control_request_t request = vendor_request();
    uint8_t data[PACKET];
    uint16_t length = 0;
    control_token(slot,TEST_PID_SETUP,100);
    sio_hw->mtime = 110;
    assert(native_test_setup(slot,&request,false));
    control_token(slot,TEST_PID_IN,120);
    assert(!native_test_in(slot,data,&length,false));
    control_token(slot,TEST_PID_IN,130);
    // Foreground reply preparation lags the first same-owner, unarmed IN.
    // This is the old recorder's blind spot: SETUP's first-IN bit is gone.
    native_test_time_us += 3000u;
    sio_hw->mtime = 200;
    native_test_drain();
    // An address-only attempt exercises the lock guard without changing owner;
    // the identical address/owner fast path deliberately bypasses that lock.
    const uint8_t rejected_address = addresses[slot]+1u;
    trace_test_lock_busy = true;
    assert(!routed_token(rejected_address,slot,TEST_PID_IN,210));
    trace_test_lock_busy = false;
    control_token(slot,TEST_PID_OUT,220);
    usb_hw->ep_rx_error = 2u; // Model a sticky EP0 sequence-error observation.
    control_token(slot,TEST_PID_IN,230);
    control_token(slot,TEST_PID_IN,240);
    assert(usb_hw->ep_rx_error == 2u);
    logger_full = true;
    const marker_receipt_t receipt = capture_marker(slot,0);
    assert(usb_hw->ep_rx_error == 2u);
    // Snapshot bytes must survive later hardware changes and real completion.
    usb_hw->ep_rx_error |= 8u;
    assert(native_test_in(slot,data,&length,true) && length == PACKET);
    assert(memcmp(data,vendor_reply,length) == 0);
    assert(native_test_in(slot,data,&length,true) && length == sizeof(vendor_reply)-PACKET);
    assert(memcmp(data,vendor_reply+PACKET,length) == 0);
    assert(native_test_out(slot,NULL,0,true));
    logger_full = false;
    dump_through(1);
    expect_marker(0,&receipt);
    assert(field(snapshot_line(0,"[HUB_FLIGHT_FREEZE]")," n=",10) == 5);
    expect_observation(0,0,addresses[slot],slot,TEST_PID_SETUP,100,true);
    const char* early = expect_observation(0,1,addresses[slot],slot,TEST_PID_IN,120,true);
    assert(!(field(early," in0=",16) & USB_BUF_CTRL_AVAIL));
    assert(field(early," why=",16) == 0 && field(early," rxerr=",16) == 0);
    const char* rejected = expect_observation(0,2,rejected_address,slot,TEST_PID_IN,210,false);
    assert(field(rejected," why=",16) == 1 && field(rejected," pub=",16) == 0);
    const char* out = expect_observation(0,3,addresses[slot],slot,TEST_PID_OUT,220,true);
    assert(field(out," why=",16) == 0 && field(out," pub=",16) == 0);
    const char* after = expect_publication(0,4,slot,230,1);
    assert(field(after," in0=",16) & USB_BUF_CTRL_AVAIL);
    assert(field(after," rxerr=",16) == 2u);
    assert(field(snapshot_line(0,"[HUB_FLIGHT_CONTEXT]")," rxerr=",16) == 2u);
    assert(usb_hw->ep_rx_error == 10u); // Recorder never clears a hardware error.
    expect_clock(0,110,200,0,1,PACKET,0);
    assert(strstr(snapshot_line(0,"[HUB_FLIGHT_CONTROL]")," pos=0/96 "));
    // The posthook knows device address/PID, not endpoint number. Even the
    // ticket-matched record does not prove an EP0 poll or SIE/host acceptance.
    expect_lost(1,0);
}

static void publication_isolation(void) {
    const tusb_control_request_t request = descriptor_request();
    const uint8_t slots[] = {1,CHILDREN};
    for (unsigned i = 0; i < sizeof(slots); ++i) {
        control_token(slots[i],TEST_PID_SETUP,100u+100u*i);
        assert(native_test_setup(slots[i],&request,false));
        control_token(slots[i],TEST_PID_IN,120u+100u*i);
    }
    sio_hw->mtime = 300;
    native_test_drain(); // Both children publish their independent first ticket.
    __atomic_store_n(&out_trace_frozen,1u,__ATOMIC_SEQ_CST);
    control_token(1,TEST_PID_IN,400);
    __atomic_store_n(&out_trace_frozen,0u,__ATOMIC_SEQ_CST);
    for (unsigned i = 0; i < TEST_RETAIN; ++i)
        control_token(1,TEST_PID_IN,410u+i);
    control_token(CHILDREN,TEST_PID_IN,500);
    control_token(CHILDREN,TEST_PID_IN,510);
    logger_full = true;
    const marker_receipt_t first = capture_marker(1,0);
    const marker_receipt_t second = capture_marker(CHILDREN,0);
    complete_descriptor(1);
    complete_descriptor(CHILDREN);
    logger_full = false;
    dump_through(2);
    expect_marker(0,&first);
    expect_marker(1,&second);
    for (unsigned snapshot = 0; snapshot < 2; ++snapshot) {
        assert(field(snapshot_line(snapshot,"[HUB_FLIGHT_FREEZE]")," n=",10) == 5);
        for (unsigned i = 0; i < sizeof(slots); ++i) {
            expect_observation(snapshot,2u*i,addresses[slots[i]],slots[i],TEST_PID_SETUP,100u+100u*i,true);
            expect_observation(snapshot,2u*i+1u,addresses[slots[i]],slots[i],TEST_PID_IN,120u+100u*i,true);
        }
        expect_observation(snapshot,4,addresses[CHILDREN],CHILDREN,TEST_PID_IN,500,true);
    }
    assert(field(snapshot_line(0,"[HUB_FLIGHT_CONTROL_CLOCK]")," pub=",16) == 1);
    expect_publication(1,4,CHILDREN,500,1);
    // Consuming child 1's ticket while frozen neither consumes child 2's equal
    // numeric ticket nor replays child 1's discarded observation after thaw.
    // A later real publication for child 1 must still become observable.
    control_token(1,TEST_PID_SETUP,700);
    sio_hw->mtime = 710;
    assert(native_test_setup(1,&request,true));
    control_token(1,TEST_PID_IN,720);
    control_token(1,TEST_PID_IN,730);
    const marker_receipt_t next = capture_marker(1,0);
    complete_descriptor(1);
    dump_through(3);
    expect_marker(2,&next);
    assert(field(snapshot_line(2,"[HUB_FLIGHT_FREEZE]")," n=",10) == 7);
    expect_observation(2,5,addresses[1],1,TEST_PID_SETUP,700,true);
    expect_publication(2,6,1,720,2);
    expect_lost(3,0);
}

static void publication_wrap_supersession(void) {
    const uint8_t slot = CHILDREN;
    const tusb_control_request_t request = descriptor_request();
    // Seed only the counter boundary; all publications still come from real
    // control requests, not fabricated watch fields or posthook forwarding.
    __atomic_store_n(&out_trace_publication_sequence[slot-1u],UINT32_MAX-1u,__ATOMIC_RELEASE);
    control_token(slot,TEST_PID_SETUP,100);
    sio_hw->mtime = 110;
    assert(native_test_setup(slot,&request,true)); // First ticket: ffffffff.
    control_token(slot,TEST_PID_SETUP,200);
    sio_hw->mtime = 210;
    assert(native_test_setup(slot,&request,false));
    // SETUP does not consume the old notification. Its replacement IRQ has
    // revoked readiness, but Core0 has not yet superseded the old watch.
    control_token(slot,TEST_PID_IN,220);
    logger_full = true;
    sio_hw->mtime = 300;
    native_test_drain(); // Captures the old watch, then publishes ticket zero.
    control_token(slot,TEST_PID_IN,310);
    control_token(slot,TEST_PID_IN,320);
    const marker_receipt_t receipt = capture_marker(slot,0);
    complete_descriptor(slot);
    logger_full = false;
    dump_through(2);
    const char* old_control = snapshot_line(0,"[HUB_FLIGHT_CONTROL]");
    const char* old_clock = snapshot_line(0,"[HUB_FLIGHT_CONTROL_CLOCK]");
    assert(field(snapshot_line(0,"[HUB_FLIGHT_FREEZE]")," reason=",10) == 2);
    assert(field(snapshot_line(0,"[HUB_FLIGHT_FREEZE]")," n=",10) == 3);
    assert(field(old_clock," flags=",16) == 1 && field(old_clock," pub=",16) == UINT32_MAX);
    assert(field(old_clock," gen=",10) == field(old_control," gen=",10));
    assert(field(snapshot_line(0,"[HUB_FLIGHT_STATUS_OUT]")," dgen=",10) > field(old_clock," gen=",10));
    const char* stale = expect_observation(0,2,addresses[slot],slot,TEST_PID_IN,220,true);
    assert(field(stale," why=",16) == 0x20 && field(stale," pub=",16) == UINT32_MAX);
    assert(!(field(stale," in0=",16) & USB_BUF_CTRL_AVAIL));
    expect_marker(1,&receipt);
    assert(receipt.generation == field(old_clock," gen=",10)+1u);
    assert(field(snapshot_line(1,"[HUB_FLIGHT_FREEZE]")," n=",10) == 4);
    expect_publication(1,3,slot,310,0); // why bit validates zero; zero is not absent.
    assert(field(record_line(1,2)," pub=",16) != field(snapshot_line(1,"[HUB_FLIGHT_CONTROL_CLOCK]")," pub=",16));
    expect_clock(1,210,300,0,1,18,0);
    expect_lost(2,0);
}

static void poll_retention(void) {
    const uint8_t slot = CHILDREN;
    const tusb_control_request_t request = vendor_request();
    uint8_t data[PACKET];
    uint16_t length = 0;
    control_token(slot,TEST_PID_SETUP,100);
    assert(native_test_setup(slot,&request,true));
    control_token(slot,TEST_PID_IN,200);
    assert(native_test_in(slot,data,&length,true) && length == PACKET);
    assert(memcmp(data,vendor_reply,length) == 0);
    control_token(slot,TEST_PID_IN,201);
    assert(native_test_in(slot,data,&length,true) && length == sizeof(vendor_reply)-PACKET);
    assert(memcmp(data,vendor_reply+PACKET,length) == 0);
    control_token(slot,TEST_PID_OUT,300);
    assert(native_test_out(slot,NULL,0,true));
    for (unsigned i = 0; i < 3u*TEST_RETAIN; ++i)
        assert(routed_token(addresses[slot],slot,i % 2u ? TEST_PID_IN : TEST_PID_OUT,400u+i));

    assert(routed_token(addresses[0],0,TEST_PID_IN,1000));
    for (unsigned i = 0; i < 3u*TEST_RETAIN; ++i) {
        assert(routed_token(addresses[0],0,i % 2u ? TEST_PID_IN : TEST_PID_OUT,1100u+i));
        root_read();
    }
    const marker_receipt_t first = capture_marker(slot,0);
    dump_through(1);
    expect_marker(0,&first);
    assert(field(snapshot_line(0,"[HUB_FLIGHT_FREEZE]")," n=",10) == 4);
    expect_observation(0,0,addresses[slot],slot,TEST_PID_SETUP,100,true);
    expect_observation(0,1,addresses[slot],slot,TEST_PID_IN,200,true);
    expect_observation(0,2,addresses[slot],slot,TEST_PID_OUT,300,true);
    const char* root = expect_observation(0,3,addresses[0],0,TEST_PID_IN,1000,true);
    assert(field(root," commit=",16) == 1000);

    // Each child's SETUP rearms both directions independently. OUT observation
    // before IN is also valid recorder evidence, not a physical-acceptance claim.
    const tusb_control_request_t descriptor = descriptor_request();
    control_token(1,TEST_PID_SETUP,2000);
    assert(native_test_setup(1,&descriptor,true));
    control_token(slot,TEST_PID_SETUP,2100);
    assert(native_test_setup(slot,&descriptor,true));
    control_token(1,TEST_PID_OUT,2200);
    control_token(1,TEST_PID_OUT,2250);
    control_token(1,TEST_PID_IN,2300);
    assert(native_test_in(1,data,&length,true) && length == 18);
    control_token(slot,TEST_PID_IN,2400);
    assert(native_test_in(slot,data,&length,true) && length == 18);
    control_token(slot,TEST_PID_OUT,2500);
    assert(native_test_out(slot,NULL,0,true));
    assert(native_test_out(1,NULL,0,true));
    const marker_receipt_t second = capture_marker(slot,0);
    dump_through(2);
    expect_marker(1,&second);
    assert(field(snapshot_line(1,"[HUB_FLIGHT_FREEZE]")," n=",10) == 10);
    expect_observation(1,4,addresses[1],1,TEST_PID_SETUP,2000,true);
    expect_observation(1,5,addresses[slot],slot,TEST_PID_SETUP,2100,true);
    expect_observation(1,6,addresses[1],1,TEST_PID_OUT,2200,true);
    expect_observation(1,7,addresses[1],1,TEST_PID_IN,2300,true);
    expect_observation(1,8,addresses[slot],slot,TEST_PID_IN,2400,true);
    expect_observation(1,9,addresses[slot],slot,TEST_PID_OUT,2500,true);
    expect_lost(2,0);
}

static void selection_history(void) {
    // Synthetic selector inputs separate address-only from owner-only changes.
    // The selector does not claim to validate the router's address mapping.
    assert(routed_token(5,0,TEST_PID_IN,100));
    for (unsigned i = 0; i < 3u*TEST_RETAIN; ++i)
        assert(routed_token(5,0,i % 2u ? TEST_PID_IN : TEST_PID_OUT,1000u+i));
    assert(routed_token(5,CHILDREN,TEST_PID_OUT,200));
    for (unsigned i = 0; i < 3u*TEST_RETAIN; ++i)
        assert(routed_token(5,CHILDREN,i % 2u ? TEST_PID_IN : TEST_PID_OUT,2000u+i));
    assert(routed_token(addresses[CHILDREN],CHILDREN,TEST_PID_IN,300));
    const uint8_t pids[] = {TEST_PID_SETUP,TEST_PID_IN,TEST_PID_OUT};
    for (unsigned i = 0; i < sizeof(pids); ++i)
        assert(!routed_token(0,DEVICES,pids[i],400u+i));
    usb_hw->buf_status = 2;
    for (unsigned i = 0; i < sizeof(pids); ++i)
        assert(!routed_token(addresses[0],0,pids[i],500u+i));
    usb_hw->buf_status = 0;
    root_read();
    const marker_receipt_t receipt = capture_marker(CHILDREN,0);
    dump_through(1);
    expect_marker(0,&receipt);
    assert(field(snapshot_line(0,"[HUB_FLIGHT_FREEZE]")," n=",10) == 9);
    const char* line = expect_observation(0,0,5,0,TEST_PID_IN,100,true);
    assert(strstr(line," addr=00000000/00000005 owner=255/0 "));
    assert(field(line," commit=",16) == 100);
    line = expect_observation(0,1,5,CHILDREN,TEST_PID_OUT,200,true);
    char transition[80];
    snprintf(transition,sizeof(transition)," addr=00000000/00000005 owner=255/%u ",(unsigned)CHILDREN);
    assert(strstr(line,transition));
    assert(field(line," commit=",16) == 200);
    line = expect_observation(0,2,addresses[CHILDREN],CHILDREN,TEST_PID_IN,300,true);
    snprintf(transition,sizeof(transition)," addr=00000000/%08x owner=255/%u ",
             (unsigned)addresses[CHILDREN],(unsigned)CHILDREN);
    assert(strstr(line,transition));
    assert(field(line," commit=",16) == 300);
    for (unsigned i = 0; i < sizeof(pids); ++i) {
        line = expect_observation(0,3u+i,0,DEVICES,pids[i],400u+i,false);
        assert(field(line," why=",16) == 0x10);
        line = expect_observation(0,6u+i,addresses[0],0,pids[i],500u+i,false);
        assert(field(line," why=",16) == 0x0a);
        assert(strstr(line," block=00000002/00000000 "));
    }
    expect_lost(1,0);
}

static void frozen_selection_history(void) {
    assert(routed_token(addresses[1],1,TEST_PID_SETUP,100));
    // Deterministically interleave Core1 tokens with the recorder's Core0
    // snapshot freeze. Assert the eventual log, not its internal cursor state.
    __atomic_store_n(&out_trace_frozen,1u,__ATOMIC_SEQ_CST);
    assert(routed_token(addresses[CHILDREN],CHILDREN,TEST_PID_SETUP,200));
    assert(routed_token(addresses[CHILDREN],CHILDREN,TEST_PID_IN,210));
    assert(routed_token(addresses[CHILDREN],CHILDREN,TEST_PID_OUT,220));
    __atomic_store_n(&out_trace_frozen,0u,__ATOMIC_SEQ_CST);
    for (unsigned i = 0; i < TEST_RETAIN; ++i)
        assert(routed_token(addresses[CHILDREN],CHILDREN,
            i % 2u ? TEST_PID_IN : TEST_PID_OUT,300u+i));

    __atomic_store_n(&out_trace_frozen,1u,__ATOMIC_SEQ_CST);
    assert(routed_token(addresses[0],0,TEST_PID_IN,500));
    __atomic_store_n(&out_trace_frozen,0u,__ATOMIC_SEQ_CST);
    for (unsigned i = 0; i < TEST_RETAIN; ++i)
        assert(routed_token(addresses[0],0,i % 2u ? TEST_PID_IN : TEST_PID_OUT,600u+i));

    // A frozen SETUP still rearms first-IN/OUT retention, but a first token
    // already consumed during the earlier freeze must not reappear afterward.
    __atomic_store_n(&out_trace_frozen,1u,__ATOMIC_SEQ_CST);
    assert(routed_token(addresses[1],1,TEST_PID_SETUP,800));
    __atomic_store_n(&out_trace_frozen,0u,__ATOMIC_SEQ_CST);
    assert(routed_token(addresses[1],1,TEST_PID_IN,900));
    assert(routed_token(addresses[1],1,TEST_PID_OUT,1000));
    for (unsigned i = 0; i < TEST_RETAIN; ++i)
        assert(routed_token(addresses[1],1,i % 2u ? TEST_PID_IN : TEST_PID_OUT,1100u+i));
    const marker_receipt_t receipt = capture_marker(1,0);
    dump_through(1);
    expect_marker(0,&receipt);
    assert(field(snapshot_line(0,"[HUB_FLIGHT_FREEZE]")," n=",10) == 3);
    expect_observation(0,0,addresses[1],1,TEST_PID_SETUP,100,true);
    expect_observation(0,1,addresses[1],1,TEST_PID_IN,900,true);
    expect_observation(0,2,addresses[1],1,TEST_PID_OUT,1000,true);
    expect_lost(1,0);
}

static void marker_priority(bool partial) {
    const uint8_t slot = CHILDREN;
    configure_child(slot);
    selections(0x10000000u,TEST_RETAIN);
    input_completion(slot);
    logger_full = !partial;
    native_test_advance(TEST_STALL_US);
    if (partial) {
        poll(6u+CHILDREN);
        assert(count_tag("[HUB_FLIGHT]") > 0 && count_tag("[HUB_FLIGHT_END]") == 0);
    } else assert(log_count == 0);
    logger_full = true;
    input_completion(slot);
    selections(0x20000000u,TEST_RETAIN);
    native_test_advance(TEST_STALL_US);

    // Both automatic slots are occupied. The marker must replace the waiting
    // automatic snapshot, never the head (even before its first UART line).
    selections(0x30000000u,TEST_RETAIN-3u);
    control_token(slot,TEST_PID_SETUP,100);
    const tusb_control_request_t request = vendor_request();
    sio_hw->mtime = 150;
    assert(native_test_setup(slot,&request,true));
    control_token(slot,TEST_PID_IN,200);
    uint8_t data[PACKET];
    uint16_t length = 0;
    sio_hw->mtime = 250;
    assert(native_test_in(slot,data,&length,true) && length == PACKET);
    assert(memcmp(data,vendor_reply,length) == 0);
    control_token(slot,TEST_PID_OUT,300);
    native_test_time_us += 7u;
    const uint32_t captured_time = native_test_time_us;
    const uint32_t captured_generation = devices[slot].control.generation;
    const marker_receipt_t receipt = capture_marker(slot,0);
    assert(receipt.time_us == captured_time && receipt.generation == captured_generation);
    root_read();
    capture_marker(1,1); // A pending host snapshot cannot be replaced by another host.

    // The root read must not disturb the child's remaining payload or status.
    assert(native_test_in(slot,data,&length,true) && length == sizeof(vendor_reply)-PACKET);
    assert(memcmp(data,vendor_reply+PACKET,length) == 0);
    assert(native_test_out(slot,NULL,0,true));
    const tusb_control_request_t replacement = descriptor_request();
    assert(native_test_setup(slot,&replacement,true));
    assert(devices[slot].control.generation != receipt.generation);
    assert(native_test_in(slot,data,&length,true) && length == 18);
    assert(memcmp(data,hub_device,length) == 0);
    assert(native_test_out(slot,NULL,0,true));
    for (unsigned i = 0; i < 3; ++i) {
        input_completion(slot);
        selections(0x40000000u+i*0x10000000u,TEST_RETAIN);
        native_test_advance(TEST_STALL_US);
    }
    poll(40); // Unchanged automatic triggers must not keep increasing lost.
    logger_full = false;
    dump_through(2);
    poll(100);
    assert(count_tag("[HUB_FLIGHT_FREEZE]") == 2);
    expect_records(0,0x10000000u,TEST_RETAIN);
    expect_marker(1,&receipt);
    const char* control = snapshot_line(1,"[HUB_FLIGHT_CONTROL]");
    assert(strstr(control," setup=c0/5a v=1122 i=3344 n=96 "));
    assert(strstr(control," pos=64/96 "));
    expect_clock(1,150,150,250,3,PACKET,PACKET);
    const char* status = expect_status_out(1,0,0,0,0);
    assert(field(status," dgen=",10) == receipt.generation);
    assert((field(status," shadow_in=",16) &
        (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_FULL | USB_BUF_CTRL_LEN_MASK)) ==
        (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_FULL | (sizeof(vendor_reply)-PACKET)));
    assert(!(field(status," shadow_out=",16) & USB_BUF_CTRL_AVAIL));
    assert(field(snapshot_line(1,"[HUB_FLIGHT_FREEZE]")," n=",10) == TEST_RETAIN);
    for (unsigned i = 0; i < TEST_RETAIN-3u; ++i) {
        const char* line = record_line(1,i);
        assert(field(line," cutoff=",16) == 0x30000000u+i);
        assert(field(line," pid=",16) == TEST_PID_OUT && field(line," ok=",10) == 1);
    }
    expect_observation(1,TEST_RETAIN-3u,addresses[slot],slot,TEST_PID_SETUP,100,true);
    expect_observation(1,TEST_RETAIN-2u,addresses[slot],slot,TEST_PID_IN,200,true);
    expect_observation(1,TEST_RETAIN-1u,addresses[slot],slot,TEST_PID_OUT,300,true);
    expect_lost(2,5); // One replacement, one busy marker, three automatic drops.
}

static void marker_zero_and_capacity(void) {
    configure_child(CHILDREN);
    native_test_time_us = 0;
    logger_full = true;
    const marker_receipt_t receipt = capture_marker(1,0);
    // Both receipt fields may legitimately be zero: this child has not yet
    // received a SETUP. Only status distinguishes capture from a busy reply.
    assert(receipt.time_us == 0 && receipt.generation == 0);
    const marker_receipt_t second = capture_marker(CHILDREN,0);
    assert(second.time_us == 0 && second.generation != 0);
    capture_marker(1,1); // Only a full queue with a waiting host refuses a marker.
    assert(log_count == 0);
    logger_full = false;
    native_test_advance(TEST_LINE_US);
    assert(count_tag("[HUB_FLIGHT_FREEZE]") == 1 && count_tag("[HUB_FLIGHT_END]") == 0);
    logger_full = true;
    input_completion(CHILDREN);
    selections(0x10000000u,TEST_RETAIN);
    native_test_advance(TEST_STALL_US);
    root_read();
    for (unsigned i = 0; i < 2; ++i) {
        input_completion(CHILDREN);
        selections(0x20000000u+i*0x10000000u,TEST_RETAIN);
        native_test_advance(TEST_STALL_US);
    }
    poll(40);
    logger_full = false;
    dump_through(2);
    poll(100);
    assert(count_tag("[HUB_FLIGHT_FREEZE]") == 2);
    expect_marker(0,&receipt);
    assert(field(snapshot_line(0,"[HUB_FLIGHT_FREEZE]")," n=",10) == 0);
    expect_marker(1,&second);
    assert(field(snapshot_line(1,"[HUB_FLIGHT_FREEZE]")," n=",10) == 0);
    expect_lost(2,4); // One busy marker and three automatic drops.
}

static void marker_active_priority(void) {
    configure_child(CHILDREN);
    selections(0x10000000u,TEST_RETAIN);
    const marker_receipt_t first = capture_marker(1,0);
    assert(count_tag("[HUB_FLIGHT_FREEZE]") == 1 && count_tag("[HUB_FLIGHT_END]") == 0);
    logger_full = true;
    input_completion(CHILDREN);
    selections(0x20000000u,TEST_RETAIN);
    native_test_advance(TEST_STALL_US);
    // A host head is protected, but does not prevent replacing the automatic
    // snapshot waiting behind it with another independently protected host.
    const marker_receipt_t second = capture_marker(CHILDREN,0);
    capture_marker(1,1);
    root_read();
    for (unsigned i = 0; i < 2; ++i) {
        input_completion(CHILDREN);
        selections(0x30000000u+i*0x10000000u,TEST_RETAIN);
        native_test_advance(TEST_STALL_US);
    }
    poll(40);
    logger_full = false;
    dump_through(2);
    poll(100);
    assert(count_tag("[HUB_FLIGHT_FREEZE]") == 2);
    expect_marker(0,&first);
    expect_marker(1,&second);
    for (unsigned snapshot = 0; snapshot < 2; ++snapshot) {
        assert(field(snapshot_line(snapshot,"[HUB_FLIGHT_FREEZE]")," n=",10) == TEST_RETAIN);
        for (unsigned i = 0; i < TEST_RETAIN; ++i) {
            const char* line = record_line(snapshot,i);
            assert(field(line," cutoff=",16) == (snapshot+1u)*0x10000000u+i);
            assert(field(line," pid=",16) == TEST_PID_OUT && field(line," ok=",10) == 1);
        }
    }
    expect_lost(2,4); // One automatic replacement, one busy marker, two drops.
}

static void marker_rejected_requests(void) {
    const tusb_control_request_t valid = marker_request(CHILDREN);
    tusb_control_request_t invalid[11];
    for (unsigned i = 0; i < sizeof(invalid)/sizeof(invalid[0]); ++i) invalid[i] = valid;
    invalid[0].bmRequestType = 0x40; // OUT direction.
    invalid[1].bmRequestType = 0xc1; // Interface recipient.
    invalid[2].bmRequestType = 0xa0; // Class request.
    invalid[3].bRequest = 0x7e;
    invalid[4].wValue ^= 1u;
    invalid[5].wIndex = 0;
    invalid[6].wIndex = CHILDREN+1u;
    invalid[7].wIndex = 0x100u+CHILDREN;
    invalid[8].wLength = 0;
    invalid[9].wLength = 15;
    invalid[10].wLength = 17;
    for (unsigned i = 0; i < sizeof(invalid)/sizeof(invalid[0]); ++i) {
        assert(!native_test_setup(0,&invalid[i],true));
        root_read();
        poll(10);
        assert(count_tag("[HUB_FLIGHT_FREEZE]") == 0);
    }
    for (uint8_t slot = 1; slot <= CHILDREN; ++slot) {
        assert(!native_test_setup(slot,&valid,true));
        root_read();
        poll(10);
        assert(count_tag("[HUB_FLIGHT_FREEZE]") == 0);
    }
    const marker_receipt_t receipt = capture_marker(CHILDREN,0);
    dump_through(1);
    expect_marker(0,&receipt);
    expect_lost(1,0);
}

static void expect_handover_packet(uint8_t slot, uint16_t offset, uint16_t expected_length,
                                   bool data1) {
    uint8_t data[PACKET];
    uint16_t length = UINT16_MAX;
    assert(native_test_select(slot));
    const uint32_t packet = buffer_regs()[0];
    assert(((packet & USB_BUF_CTRL_DATA1_PID) != 0) == data1);
    // Selection must expose the packet before the token, with no Core0 task.
    assert(native_test_in(slot,data,&length,false));
    assert(length == expected_length);
    assert(memcmp(data,handover_reply[slot]+offset,length) == 0);
}

static tusb_control_request_t approved_request(uint16_t length, uint16_t host_length,
                                               uint16_t tag) {
    const tusb_control_request_t request = {
        .bmRequestType = 0xc0, .bRequest = 0x5d, .wValue = length,
        .wIndex = tag, .wLength = host_length,
    };
    return request;
}

static void expect_approved_packet(uint8_t slot, uint16_t offset, uint16_t expected_length,
                                    bool data1) {
    uint8_t data[PACKET];
    uint16_t length = UINT16_MAX;
    assert(native_test_select(slot));
    assert(((buffer_regs()[0] & USB_BUF_CTRL_DATA1_PID) != 0) == data1);
    assert(native_test_in(slot,data,&length,false) && length == expected_length);
    assert(memcmp(data,approved_reply[slot]+offset,length) == 0);
}

static void complete_ready_status(uint8_t slot) {
    assert(native_test_select(slot));
    assert((buffer_regs()[1] & (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_DATA1_PID |
        USB_BUF_CTRL_STALL | USB_BUF_CTRL_LEN_MASK)) ==
        (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_DATA1_PID));
    assert(native_test_out(slot,NULL,0,false));
    assert(!native_test_out(slot,NULL,0,false));
}

static void expect_approved_callback(unsigned index, uint8_t slot, uint8_t stage,
                                      const tusb_control_request_t* request) {
    assert(index < approved_callback_count);
    const approved_callback_t* callback = &approved_callbacks[index];
    assert(callback->slot == slot && callback->stage == stage);
    assert(memcmp(&callback->request,request,sizeof(*request)) == 0);
}

static void expect_no_control_packets(void) {
    uint8_t data[PACKET];
    uint16_t length = 0;
    for (uint8_t slot = 0; slot < DEVICES; ++slot) {
        assert(!native_test_in(slot,data,&length,false));
        assert(!native_test_out(slot,NULL,0,false));
    }
}

static void approved_status_handoff(void) {
    unsigned callbacks = 0;
    for (unsigned pass = 0; pass < 3; ++pass) {
        tusb_control_request_t requests[DEVICES];
        bool final_first[DEVICES];
        for (uint8_t slot = 0; slot < DEVICES; ++slot) {
            // Mixed short reply, terminating ZLP, and multi-packet tail;
            // then exact one/two-packet replies with no padding ZLP.
            const uint16_t length = pass == 0 ? (slot == 0 ? 1 : slot % 2u ? PACKET : 83) :
                pass == 1 ? PACKET : 2u*PACKET;
            requests[slot] = approved_request(length,pass == 0 ? 128 : length,
                (uint16_t)(100u*pass+slot));
            final_first[slot] = length < PACKET || (length == PACKET && requests[slot].wLength == PACKET);
            assert(native_test_setup(slot,&requests[slot],true));
            assert(!native_test_out(slot,NULL,0,false));
        }
        for (uint8_t slot = 0; slot < DEVICES; ++slot) {
            const uint16_t length = requests[slot].wValue < PACKET ? requests[slot].wValue : PACKET;
            expect_approved_packet(slot,0,length,true);
            if (final_first[slot]) complete_ready_status(slot);
            else assert(!native_test_out(slot,NULL,0,false));
        }
        assert(approved_callback_count == callbacks); // No callbacks in IRQ.
        native_test_drain();
        for (uint8_t slot = 0; slot < DEVICES; ++slot) {
            if (!final_first[slot]) continue;
            expect_approved_callback(callbacks++,slot,CONTROL_STAGE_DATA,&requests[slot]);
            expect_approved_callback(callbacks++,slot,CONTROL_STAGE_ACK,&requests[slot]);
        }
        assert(approved_callback_count == callbacks);
        for (uint8_t slot = 0; slot < DEVICES; ++slot) {
            assert(!native_test_out(slot,NULL,0,false));
            if (final_first[slot]) continue;
            // A required zero-length IN must complete before OUT can be ready.
            expect_approved_packet(slot,PACKET,requests[slot].wValue-PACKET,false);
            complete_ready_status(slot);
        }
        assert(approved_callback_count == callbacks);
        native_test_drain();
        for (uint8_t slot = 0; slot < DEVICES; ++slot) {
            if (final_first[slot]) continue;
            expect_approved_callback(callbacks++,slot,CONTROL_STAGE_DATA,&requests[slot]);
            expect_approved_callback(callbacks++,slot,CONTROL_STAGE_ACK,&requests[slot]);
        }
        assert(approved_callback_count == callbacks);
        expect_no_control_packets(); // Draining final IN must not rearm consumed OUT.
        native_test_drain();
        assert(approved_callback_count == callbacks);
    }
}

static void approved_status_superseded(void) {
    tusb_control_request_t old[DEVICES], next[DEVICES];
    for (uint8_t slot = 0; slot < DEVICES; ++slot) {
        old[slot] = approved_request(1,1,100u+slot);
        next[slot] = approved_request(7,7,200u+slot);
        assert(native_test_setup(slot,&old[slot],true));
    }
    for (uint8_t slot = 0; slot < DEVICES; ++slot) {
        expect_approved_packet(slot,0,1,true);
        complete_ready_status(slot);
        // SETUP revokes readiness but cannot erase the real, queued DATA/ACK.
        assert(native_test_setup(slot,&next[slot],false));
        assert(!native_test_out(slot,NULL,0,false));
    }
    assert(approved_callback_count == 0);
    native_test_drain();
    for (uint8_t slot = 0; slot < DEVICES; ++slot) {
        expect_approved_callback(2u*slot,slot,CONTROL_STAGE_DATA,&old[slot]);
        expect_approved_callback(2u*slot+1u,slot,CONTROL_STAGE_ACK,&old[slot]);
        assert(!native_test_out(slot,NULL,0,false));
        expect_approved_packet(slot,0,7,true);
        complete_ready_status(slot);
    }
    assert(approved_callback_count == 2u*DEVICES);
    native_test_drain();
    for (uint8_t slot = 0; slot < DEVICES; ++slot) {
        expect_approved_callback(2u*(DEVICES+slot),slot,CONTROL_STAGE_DATA,&next[slot]);
        expect_approved_callback(2u*(DEVICES+slot)+1u,slot,CONTROL_STAGE_ACK,&next[slot]);
    }
    assert(approved_callback_count == 4u*DEVICES);
    expect_no_control_packets();

    // A replacement received before IN completion must revoke the pending
    // eligibility, rather than publish the old status for the new SETUP.
    const tusb_control_request_t descriptor = descriptor_request();
    for (uint8_t slot = 0; slot < DEVICES; ++slot)
        assert(native_test_setup(slot,&old[slot],true));
    for (uint8_t slot = 0; slot < DEVICES; ++slot)
        assert(native_test_setup(slot,&descriptor,false));
    expect_no_control_packets();
    native_test_drain();
    for (uint8_t slot = 0; slot < DEVICES; ++slot) {
        assert(!native_test_out(slot,NULL,0,false));
        complete_descriptor(slot);
    }
    assert(approved_callback_count == 4u*DEVICES);
    expect_no_control_packets();
}

static void approved_status_reset(void) {
    const tusb_control_request_t request = approved_request(1,1,100);
    for (uint8_t slot = 0; slot < DEVICES; ++slot)
        assert(native_test_setup(slot,&request,true));
    event_t final_in = {0}, status = {0};
    for (uint8_t slot = 0; slot < DEVICES; ++slot) {
        expect_approved_packet(slot,0,1,true);
        if (slot == 0) final_in = events[event_tail];
        if (slot % 2u == 0) {
            const unsigned status_index = event_head;
            complete_ready_status(slot);
            if (slot == 0) status = events[status_index];
        }
    }
    // Both already-completed statuses and still-ready inactive statuses lose
    // ownership in reset IRQ, before the queued foreground completions run.
    assert(approved_callback_count == 0);
    native_test_bus_reset(false);
    expect_no_control_packets();
    native_test_drain();
    expect_no_control_packets();
    assert(approved_callback_count == 0);
    // Restore the fixture's synthetic routes after deferred reset processing,
    // just as native_test_bus_reset(true) does after its internal drain.
    for (uint8_t slot = 1; slot < DEVICES; ++slot) addresses[slot] = slot*17u;
    const tusb_control_request_t descriptor = descriptor_request();
    for (uint8_t slot = 0; slot < DEVICES; ++slot)
        assert(native_test_setup(slot,&descriptor,true));
    transfer_complete(&final_in);
    transfer_complete(&status);
    for (uint8_t slot = 0; slot < DEVICES; ++slot) {
        uint8_t data[PACKET];
        uint16_t length = 0;
        assert(!native_test_out(slot,NULL,0,false));
        assert(native_test_in(slot,data,&length,false) && length == 18);
        assert(memcmp(data,hub_device,length) == 0);
        complete_ready_status(slot);
    }
    native_test_drain();
    expect_no_control_packets();
    assert(approved_callback_count == 0);
}

static void approved_status_reset_during_completion(void) {
    const tusb_control_request_t request = approved_request(1,1,100);
    assert(native_test_setup(0,&request,true));
    expect_approved_packet(0,0,1,true);
    assert(approved_callback_count == 0);
    // Interrupt foreground after its first reset-generation check, while its
    // diagnostic is emitted, but before it claims the queued final-IN event.
    reset_on_root_complete = true;
    native_test_drain();
    assert(!reset_on_root_complete);
    assert(approved_callback_count == 0);
    expect_no_control_packets();
    assert(native_test_setup(0,&request,true));
    assert(!native_test_out(0,NULL,0,false));
    expect_approved_packet(0,0,1,true);
    complete_ready_status(0);
    native_test_drain();
    expect_approved_callback(0,0,CONTROL_STAGE_DATA,&request);
    expect_approved_callback(1,0,CONTROL_STAGE_ACK,&request);
    assert(approved_callback_count == 2);
    expect_no_control_packets();
}

static void approved_status_port_reset(bool queued_status) {
    const uint8_t target = CHILDREN;
    uint8_t data[PACKET];
    uint16_t length = 0;
    const tusb_control_request_t preparation[] = {
        {.bRequest = TUSB_REQ_SET_ADDRESS, .wValue = 9},
        {.bmRequestType = 0x23, .bRequest = TUSB_REQ_SET_FEATURE,
         .wValue = 8, .wIndex = target},
    };
    for (unsigned i = 0; i < sizeof(preparation)/sizeof(preparation[0]); ++i) {
        assert(native_test_setup(0,&preparation[i],true));
        assert(native_test_in(0,data,&length,true) && length == 0);
    }
    const tusb_control_request_t reset = {
        .bmRequestType = 0x23, .bRequest = TUSB_REQ_SET_FEATURE,
        .wValue = 4, .wIndex = target,
    };
    const tusb_control_request_t request = approved_request(1,1,100);
    assert(native_test_setup(0,&reset,true));
    for (uint8_t slot = 1; slot < DEVICES; ++slot)
        assert(native_test_setup(slot,&request,true));
    // Queue the real root status-IN action first. Its reset_device invalidates
    // the target's later queued completions, without touching sibling owners.
    assert(native_test_in(0,data,&length,false) && length == 0);
    assert(!native_test_out(0,NULL,0,false)); // Writes never acquire status OUT.
    for (uint8_t slot = 1; slot < DEVICES; ++slot) {
        expect_approved_packet(slot,0,1,true);
        if (queued_status) complete_ready_status(slot);
    }
    assert(approved_callback_count == 0);
    native_test_drain();
    native_test_advance(10000u);
    assert(native_test_select(target)); // Reset has returned this child to address zero.
    assert(!native_test_in(target,data,&length,false));
    assert(!native_test_out(target,NULL,0,false));
    unsigned callbacks = 0;
    for (uint8_t slot = 1; slot < DEVICES; ++slot) {
        if (slot == target) continue;
        expect_approved_callback(callbacks++,slot,CONTROL_STAGE_DATA,&request);
        if (queued_status)
            expect_approved_callback(callbacks++,slot,CONTROL_STAGE_ACK,&request);
        else complete_ready_status(slot);
    }
    assert(approved_callback_count == callbacks);
    native_test_drain();
    if (!queued_status) {
        for (uint8_t slot = 1; slot < DEVICES; ++slot)
            if (slot != target)
                expect_approved_callback(callbacks++,slot,CONTROL_STAGE_ACK,&request);
    }
    assert(approved_callback_count == 2u*(CHILDREN-1u) && approved_callback_count == callbacks);
    expect_no_control_packets();
}

static void approved_status_invalid_length(void) {
    const tusb_control_request_t request = approved_request(16,16,100);
    for (uint8_t slot = 0; slot < DEVICES; ++slot) {
        assert(native_test_setup(slot,&request,true));
        assert(native_test_select(slot));
        // Model an actual completed length that disagrees with the armed IN.
        const uint16_t actual = slot % 2u ? 17 : 15;
        buffer_regs()[0] = (buffer_regs()[0] & ~USB_BUF_CTRL_LEN_MASK) | actual;
        uint8_t data[PACKET];
        uint16_t length = 0;
        assert(native_test_in(slot,data,&length,false) && length == actual);
        assert(!native_test_out(slot,NULL,0,false));
    }
    native_test_drain();
    expect_no_control_packets();
    assert(approved_callback_count == 0);
    for (uint8_t slot = 0; slot < DEVICES; ++slot) {
        assert(native_test_select(slot));
        assert(buffer_regs()[0] & USB_BUF_CTRL_STALL);
        assert(buffer_regs()[1] & USB_BUF_CTRL_STALL);
        assert(native_test_setup(slot,&request,true));
        assert(!native_test_out(slot,NULL,0,false));
        expect_approved_packet(slot,0,16,true);
        complete_ready_status(slot);
        native_test_drain();
        expect_approved_callback(2u*slot,slot,CONTROL_STAGE_DATA,&request);
        expect_approved_callback(2u*slot+1u,slot,CONTROL_STAGE_ACK,&request);
    }
    assert(approved_callback_count == 2u*DEVICES);
    expect_no_control_packets();
}

static void approved_status_watch(void) {
    const uint8_t slot = CHILDREN;
    const tusb_control_request_t request = approved_request(1,1,100);
    sio_hw->mtime = 100;
    assert(native_test_setup(slot,&request,true));
    assert(!native_test_out(slot,NULL,0,false));
    sio_hw->mtime = 200;
    expect_approved_packet(slot,0,1,true);
    assert(approved_callback_count == 0);
    assert(native_test_select(1));
    sio_hw->mtime = 300;
    native_test_drain();
    expect_approved_callback(0,slot,CONTROL_STAGE_DATA,&request);
    assert(approved_callback_count == 1);
    logger_full = true;
    sio_hw->mtime = 400;
    const marker_receipt_t armed = capture_marker(slot,0);
    sio_hw->mtime = 500;
    complete_ready_status(slot);
    assert(approved_callback_count == 1);
    sio_hw->mtime = 600;
    native_test_drain();
    expect_approved_callback(1,slot,CONTROL_STAGE_ACK,&request);
    sio_hw->mtime = 700;
    const marker_receipt_t completed = capture_marker(slot,0);
    assert(completed.generation == armed.generation);
    const tusb_control_request_t replacement = descriptor_request();
    sio_hw->mtime = 800;
    assert(native_test_setup(slot,&replacement,true));
    sio_hw->mtime = 900;
    complete_descriptor(slot);
    logger_full = false;
    dump_through(2);
    expect_marker(0,&armed);
    expect_marker(1,&completed);
    expect_clock(0,100,100,200,3,1,1);
    expect_clock(1,100,100,200,3,1,1);
    const char* first = expect_status_out(0,200,0,1,0);
    const char* second = expect_status_out(1,200,500,3,0);
    assert(field(first," dgen=",10) == armed.generation);
    assert(field(second," dgen=",10) == armed.generation);
    assert((field(first," shadow_out=",16) & (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_DATA1_PID |
        USB_BUF_CTRL_LEN_MASK)) == (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_DATA1_PID));
    assert(field(second," shadow_out=",16) == 0);
    assert(field(snapshot_line(0,"[HUB_FLIGHT_CONTROL]")," stage=",10) == STATUS_OUT);
    assert(field(snapshot_line(1,"[HUB_FLIGHT_CONTROL]")," stage=",10) == IDLE);
    assert(approved_callback_count == 2);
    expect_no_control_packets();
}

static void prepare_status_read(uint8_t slot) {
    const tusb_control_request_t request = {
        .bmRequestType = 0xc0, .bRequest = 0x5b, .wLength = 16,
    };
    for (unsigned i = 0; i < sizeof(handover_reply[slot]); ++i)
        handover_reply[slot][i] = (uint8_t)(slot * 0x31u + i * 3u);
    sio_hw->mtime = 100;
    assert(native_test_setup(slot,&request,true));
    sio_hw->mtime = 200;
    expect_handover_packet(slot,0,16,true);
    // This nonpreapproved vendor still requires the foreground DATA callback.
    assert(!(buffer_regs()[1] & USB_BUF_CTRL_AVAIL));
    assert(!native_test_out(slot,NULL,0,false));
}

static void status_out_rejected_data(void) {
    reject_status_in_callback = true;
    for (uint8_t slot = 0; slot < DEVICES; ++slot) prepare_status_read(slot);
    native_test_drain();
    for (uint8_t slot = 0; slot < DEVICES; ++slot) {
        assert(native_test_select(slot));
        assert(buffer_regs()[1] & USB_BUF_CTRL_STALL);
        assert(handover_data_callbacks[slot] == 1 && handover_acks[slot] == 0);
    }
    expect_no_control_packets();
    native_test_drain();
    for (uint8_t slot = 0; slot < DEVICES; ++slot)
        assert(handover_data_callbacks[slot] == 1 && handover_acks[slot] == 0);
}

static void status_out_lifecycle(void) {
    const uint8_t slot = CHILDREN;
    prepare_status_read(slot);
    assert(native_test_select(1));
    sio_hw->mtime = 300;
    native_test_drain(); // Publish to an inactive child's shadow, not root EP0.
    logger_full = true;
    sio_hw->mtime = 400;
    const marker_receipt_t armed = capture_marker(slot,0);
    assert(native_test_select(slot));
    assert((buffer_regs()[1] & (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_DATA1_PID |
        USB_BUF_CTRL_STALL | USB_BUF_CTRL_LEN_MASK)) ==
        (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_DATA1_PID));
    sio_hw->mtime = 500;
    assert(native_test_out(slot,NULL,0,false));
    assert(handover_acks[slot] == 0);
    assert(native_test_select(1));
    sio_hw->mtime = 600;
    native_test_drain();
    assert(handover_acks[slot] == 1);
    sio_hw->mtime = 700;
    const marker_receipt_t completed = capture_marker(slot,0);
    assert(completed.generation == armed.generation);

    // Replace the live control/watch and finish it before either host snapshot
    // drains. Both headers must retain their own publication/completion image.
    const tusb_control_request_t replacement = descriptor_request();
    uint8_t data[PACKET];
    uint16_t length = 0;
    sio_hw->mtime = 800;
    assert(native_test_setup(slot,&replacement,true));
    sio_hw->mtime = 900;
    assert(native_test_in(slot,data,&length,true) && length == 18);
    sio_hw->mtime = 1000;
    assert(native_test_out(slot,NULL,0,true));
    logger_full = false;
    dump_through(2);
    expect_marker(0,&armed);
    expect_marker(1,&completed);
    expect_clock(0,100,100,200,3,16,16);
    const char* first = expect_status_out(0,300,0,1,0);
    const char* second = expect_status_out(1,300,500,3,0);
    assert(field(first," dgen=",10) == armed.generation);
    assert(field(second," dgen=",10) == armed.generation);
    assert(field(first," shadow_in=",16) == 0 && field(second," shadow_in=",16) == 0);
    assert((field(first," shadow_out=",16) &
        (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_DATA1_PID | USB_BUF_CTRL_LEN_MASK)) ==
        (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_DATA1_PID));
    assert(field(second," shadow_out=",16) == 0);
    assert(field(snapshot_line(0,"[HUB_FLIGHT_CONTROL]")," stage=",10) == STATUS_OUT);
    assert(field(snapshot_line(1,"[HUB_FLIGHT_CONTROL]")," stage=",10) == IDLE);
}

static void status_out_superseded_arm(void) {
    const uint8_t slot = CHILDREN;
    prepare_status_read(slot);
    supersede_status_in_callback = true;
    logger_full = true;
    sio_hw->mtime = 300;
    native_test_drain();
    // The DATA callback injected a replacement SETUP before the old status
    // publication acquired its generation guard. STATUS_OUT alone is no arm.
    sio_hw->mtime = 400;
    const marker_receipt_t replacement = capture_marker(slot,0);
    uint8_t data[PACKET];
    uint16_t length = 0;
    assert(native_test_in(slot,data,&length,true) && length == 18);
    assert(memcmp(data,hub_device,length) == 0);
    assert(native_test_out(slot,NULL,0,true));
    assert(handover_acks[slot] == 0);
    logger_full = false;
    dump_through(2);
    const char* old = expect_status_out(0,0,0,0,0);
    assert(field(snapshot_line(0,"[HUB_FLIGHT_FREEZE]")," reason=",10) == 2);
    assert(field(snapshot_line(0,"[HUB_FLIGHT_CONTROL]")," stage=",10) == STATUS_OUT);
    assert(field(old," gen=",10)+1u == replacement.generation);
    assert(field(old," dgen=",10) == replacement.generation);
    assert(!(field(old," shadow_out=",16) & USB_BUF_CTRL_AVAIL));
    expect_marker(1,&replacement);
    const char* next = expect_status_out(1,0,0,0,0);
    assert(field(next," dgen=",10) == replacement.generation);
    assert((field(next," shadow_in=",16) & (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_LEN_MASK)) ==
        (USB_BUF_CTRL_AVAIL | 18u));
}

static void status_out_stale_completion(bool reset) {
    const uint8_t slot = CHILDREN;
    prepare_status_read(slot);
    sio_hw->mtime = 300;
    native_test_drain();
    sio_hw->mtime = 400;
    assert(native_test_out(slot,NULL,0,false));
    assert(event_tail != event_head);
    const event_t completion = events[event_tail];
    assert(completion.device == slot && completion.channel == 1 && completion.length == 0);
    sio_hw->mtime = 500;
    if (reset) native_test_bus_reset(true);
    else native_test_drain();
    // A queued status ACK is revoked by reset, not merely by delayed Core0.
    assert(handover_acks[slot] == (reset ? 0u : 1u));
    const tusb_control_request_t replacement = descriptor_request();
    sio_hw->mtime = 600;
    assert(native_test_setup(slot,&replacement,true));
    // Replay a captured hardware event, not a fabricated watch update: a late
    // completion of the prior control/reset generation cannot mark this read.
    sio_hw->mtime = 700;
    transfer_complete(&completion);
    assert(handover_acks[slot] == (reset ? 0u : 1u));
    assert(!native_test_out(slot,NULL,0,false));
    logger_full = true;
    const marker_receipt_t receipt = capture_marker(slot,0);
    uint8_t data[PACKET];
    uint16_t length = 0;
    assert(native_test_in(slot,data,&length,true) && length == 18);
    assert(memcmp(data,hub_device,length) == 0);
    assert(native_test_out(slot,NULL,0,true));
    logger_full = false;
    dump_through(1);
    expect_marker(0,&receipt);
    const char* status = expect_status_out(0,0,0,0,0);
    assert(field(status," dgen=",10) == receipt.generation);
    assert((field(status," shadow_in=",16) & (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_LEN_MASK)) ==
        (USB_BUF_CTRL_AVAIL | 18u));
}

static void status_out_reset_watch(bool port_reset) {
    const uint8_t slot = CHILDREN;
    uint8_t data[PACKET];
    uint16_t length = 0;
    if (port_reset) {
        // Release address zero and power the port through ordinary root
        // requests so the subsequent child reset follows its real guards.
        const tusb_control_request_t requests[] = {
            {.bRequest = TUSB_REQ_SET_ADDRESS, .wValue = 9},
            {.bmRequestType = 0x23, .bRequest = TUSB_REQ_SET_FEATURE,
             .wValue = 8, .wIndex = slot},
        };
        for (unsigned i = 0; i < sizeof(requests)/sizeof(requests[0]); ++i) {
            assert(native_test_setup(0,&requests[i],true));
            assert(native_test_in(0,data,&length,true) && length == 0);
        }
    }
    prepare_status_read(slot);
    sio_hw->mtime = 300;
    native_test_drain();
    sio_hw->mtime = 400;
    assert(native_test_out(slot,NULL,0,true));
    assert(handover_acks[slot] == 1);
    logger_full = true;
    sio_hw->mtime = 500;
    const marker_receipt_t completed = capture_marker(slot,0);
    assert(completed.generation != 0);
    sio_hw->mtime = 600;
    if (port_reset) {
        const tusb_control_request_t request = {
            .bmRequestType = 0x23, .bRequest = TUSB_REQ_SET_FEATURE,
            .wValue = 4, .wIndex = slot,
        };
        assert(native_test_setup(0,&request,true));
        assert(native_test_in(0,data,&length,true) && length == 0);
        native_test_advance(10000u);
    } else native_test_bus_reset(true);
    // A released notification can outlive its control. Observe it after reset,
    // before any replacement SETUP: its ticket must not make the empty watch valid.
    control_token(slot,TEST_PID_IN,650);
    control_token(slot,TEST_PID_IN,660);
    assert(!native_test_in(slot,data,&length,false));
    sio_hw->mtime = 700;
    // No child SETUP may clear the old watch on behalf of reset. A root marker
    // must already describe an empty control, while the earlier latch stays intact.
    const marker_receipt_t reset = capture_marker(slot,0);
    assert(reset.generation == 0);
    logger_full = false;
    dump_through(2);
    expect_marker(0,&completed);
    expect_marker(1,&reset);
    expect_clock(0,100,100,200,3,16,16);
    const char* before = expect_status_out(0,300,400,3,0);
    const char* after = expect_status_out(1,0,0,0,0);
    assert(field(before," dgen=",10) == completed.generation);
    assert(field(after," dgen=",10) > field(before," dgen=",10));
    assert(field(after," shadow_in=",16) == 0 && field(after," shadow_out=",16) == 0);
    assert(field(snapshot_line(1,"[HUB_FLIGHT_CONTROL]")," stage=",10) == IDLE);
    const char* clock = snapshot_line(1,"[HUB_FLIGHT_CONTROL_CLOCK]");
    const char* cleared[] = {" setup="," arm="," complete="," flags="," pid="," arm_len="," len="," pub="};
    for (unsigned i = 0; i < sizeof(cleared)/sizeof(cleared[0]); ++i)
        assert(field(clock,cleared[i],16) == 0);
    assert(handover_acks[slot] == 1);
    const uint32_t prior_ticket = field(snapshot_line(0,"[HUB_FLIGHT_CONTROL_CLOCK]")," pub=",16);
    assert(prior_ticket != 0);
    assert(field(snapshot_line(1,"[HUB_FLIGHT_FREEZE]")," n=",10) == 1);
    const char* stale = expect_observation(1,0,addresses[slot],slot,TEST_PID_IN,650,true);
    assert(field(stale," why=",16) == 0x20 && field(stale," pub=",16) == prior_ticket);
    assert(!(field(stale," in0=",16) & USB_BUF_CTRL_AVAIL));
    const tusb_control_request_t replacement = descriptor_request();
    sio_hw->mtime = 800;
    assert(native_test_setup(slot,&replacement,true));
    control_token(slot,TEST_PID_IN,820);
    control_token(slot,TEST_PID_IN,830);
    const marker_receipt_t next = capture_marker(slot,0);
    complete_descriptor(slot);
    dump_through(3);
    expect_marker(2,&next);
    assert(field(snapshot_line(2,"[HUB_FLIGHT_FREEZE]")," n=",10) == 2);
    expect_publication(2,1,slot,820,prior_ticket+1u);
    expect_lost(3,0);
}

static uint32_t observe_commit(uint8_t address, uint8_t owner, bool handover, uint32_t cutoff) {
    assert(usb_hw->dev_addr_ctrl != address);
    assert(handover == (active_device != owner));
    commit_bank = hardware_bank();
    commit_address = address;
    commit_owner = owner;
    commit_handover = handover;
    if (handover) {
        for (unsigned channel = 0; channel < CHANNELS; ++channel)
            commit_buffers[channel] = owner == 0 && channel >= 2 ? 0 :
                devices[owner].buffers[channel] & ~USB_BUF_CTRL_AVAIL;
        for (unsigned i = 0; i < 4; ++i)
            commit_controls[i] = devices[owner].endpoint_controls[i];
        commit_root_control = owner == 0 ? hub_endpoint_control : 0;
        commit_stall = ((commit_buffers[0] & USB_BUF_CTRL_STALL) ? 1u : 0u) |
            ((commit_buffers[1] & USB_BUF_CTRL_STALL) ? 2u : 0u);
    }
    commit_cycle = 0;
    commit_probe = clock_steps = true;
    assert(native_hub_select_device(address,owner,cutoff));
    native_hub_note_selected_token(address,owner,cutoff,TEST_PID_OUT);
    clock_steps = false;
    assert(!commit_probe && commit_cycle != 0);
    assert(usb_hw->dev_addr_ctrl == address && active_device == owner);
    return commit_cycle;
}

static void reject_unchanged(uint8_t address, uint8_t owner, uint32_t cutoff) {
    const hardware_bank_t bank = hardware_bank();
    const uint32_t previous_address = usb_hw->dev_addr_ctrl;
    const uint8_t previous_owner = active_device;
    assert(!native_hub_select_device(address,owner,cutoff));
    native_hub_note_failed_select(address,owner,cutoff,TEST_PID_OUT);
    expect_bank_unchanged(&bank);
    assert(usb_hw->dev_addr_ctrl == previous_address && active_device == previous_owner);
}

static void coherent_publication(void) {
    const tusb_control_request_t configuration = {
        .bRequest = TUSB_REQ_SET_CONFIGURATION, .wValue = 1,
    };
    const tusb_control_request_t request = {
        .bmRequestType = 0xc0, .bRequest = 0x5b, .wLength = 128,
    };
    uint8_t data[PACKET];
    uint16_t length = 0;
    for (uint8_t slot = 1; slot < DEVICES; ++slot) configure_child(slot);
    assert(native_test_setup(0,&configuration,true));
    assert(native_test_in(0,data,&length,true) && length == 0);
    ports[0].change = C_RESET;
    native_test_drain();
    for (uint8_t slot = 0; slot < DEVICES; ++slot) {
        for (unsigned i = 0; i < sizeof(handover_reply[slot]); ++i)
            handover_reply[slot][i] = (uint8_t)(slot * 0x31u + i * 3u);
        assert(native_test_setup(slot,&request,true));
        if (slot) {
            const uint8_t payload[] = {slot,0xa5,0x5a};
            assert(native_hub_hid_report(slot-1u,0x30,payload,sizeof(payload)));
            assert(native_hub_vendor_write(slot-1u,payload,sizeof(payload)) == sizeof(payload));
            assert(native_hub_vendor_write_flush(slot-1u) == sizeof(payload));
        }
    }
    assert(native_test_select(0));
    assert(buffer_regs()[0] & USB_BUF_CTRL_AVAIL);
    assert(buffer_regs()[30] & USB_BUF_CTRL_AVAIL);
    assert(usb_dpram->ep_ctrl[14].in & EP_CTRL_ENABLE_BITS);
    native_test_sio.mtime = 1000;
    const uint32_t first_commit = observe_commit(addresses[1],1,true,0x70000000u);
    assert(buffer_regs()[0] & USB_BUF_CTRL_AVAIL);
    for (unsigned channel = 2; channel < CHANNELS; ++channel)
        assert(buffer_regs()[channel] & USB_BUF_CTRL_AVAIL);

    // A posthook on the same owner refers to the prior selector write, not to
    // its later observation clock. Polls must not manufacture fresh commits.
    native_test_sio.mtime += 10u;
    native_hub_note_selected_token(addresses[1],1,0x70000001u,TEST_PID_SETUP);
    native_test_sio.mtime += 10u;
    assert(native_test_select(1));
    native_hub_note_selected_token(addresses[1],1,0x70000002u,TEST_PID_IN);

    const uint8_t incoming = CHILDREN;
    trace_test_lock_busy = true;
    reject_unchanged(addresses[incoming],incoming,0x70000003u);
    trace_test_lock_busy = false;
    usb_hw->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
    reject_unchanged(addresses[incoming],incoming,0x70000004u);
    usb_hw->sie_status = 0;
    const uint32_t pending[] = {1u,2u,4u,8u,16u,32u,1u << 30};
    for (unsigned i = 0; i < sizeof(pending)/sizeof(pending[0]); ++i) {
        usb_hw->buf_status = pending[i];
        reject_unchanged(addresses[incoming],incoming,0x70000010u+i);
    }
    usb_hw->buf_status = 0;
    reject_unchanged(addresses[incoming],incoming,native_test_sio.mtime);
    reject_unchanged(addresses[incoming],DEVICES,0x70000020u);
    spin_lock_t* lock = bank_lock;
    bank_lock = NULL;
    reject_unchanged(addresses[incoming],incoming,0x70000021u);
    bank_lock = lock;

    // The unchanged-owner/address fast path remains a no-op even when a bank
    // handover would be rejected. An address-only write must not quiesce data.
    const hardware_bank_t ready = hardware_bank();
    trace_test_lock_busy = true;
    usb_hw->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
    usb_hw->buf_status = 1u << 30;
    assert(native_hub_select_device(addresses[1],1,0));
    expect_bank_unchanged(&ready);
    trace_test_lock_busy = false;
    usb_hw->sie_status = usb_hw->buf_status = 0;
    observe_commit(5,1,false,0x70000022u);
    assert(native_test_select(1));
    expect_bank_unchanged(&ready);
    observe_commit(addresses[incoming],incoming,true,0x70000023u);
    observe_commit(addresses[0],0,true,0x70000024u);

    // All replies/private banks were prepared before any of these tokens.
    // No Core0 task may repair a handover between selection and consumption.
    for (uint8_t slot = 0; slot < DEVICES; ++slot) {
        expect_handover_packet(slot,0,slot ? PACKET : 16,true);
        if (!slot) continue;
        const uint8_t payload[] = {slot,0xa5,0x5a};
        assert(!(buffer_regs()[2] & USB_BUF_CTRL_DATA1_PID));
        assert(native_test_private_in(slot,0x81,data,&length));
        assert(length == sizeof(payload)+1u && data[0] == 0x30);
        assert(memcmp(data+1,payload,sizeof(payload)) == 0);
        assert(!(buffer_regs()[4] & USB_BUF_CTRL_DATA1_PID));
        assert(native_test_private_in(slot,0x82,data,&length));
        assert(length == sizeof(payload) && memcmp(data,payload,length) == 0);
        assert(!(buffer_regs()[3] & USB_BUF_CTRL_DATA1_PID));
        assert(native_test_private_out(slot,0x01,payload,sizeof(payload),false));
        assert(!(buffer_regs()[5] & USB_BUF_CTRL_DATA1_PID));
        assert(native_test_private_out(slot,0x02,payload,sizeof(payload),false));
    }
    assert(native_test_private_in(0,0x8f,data,&length));
    assert(length == 1 && data[0] == 2);
    native_test_drain();
    for (unsigned instance = 0; instance < CHILDREN; ++instance) {
        assert(native_test_hid_completions[instance] == 1);
        assert(native_test_bulk_completions[instance] == 1);
        assert(native_test_received_count[instance][0] == 1);
        assert(native_test_received_count[instance][1] == 1);
    }

    // Outgoing root visibility must be gone when a child address commits;
    // selecting the root again must already expose its incoming STALL state.
    tusb_control_request_t rejected = request;
    rejected.bRequest = 0x5c;
    assert(!native_test_setup(0,&rejected,true));
    assert(usb_hw->ep_stall_arm == 3u);
    assert(buffer_regs()[0] & USB_BUF_CTRL_STALL);
    assert(usb_dpram->ep_ctrl[14].in & EP_CTRL_ENABLE_BITS);
    observe_commit(addresses[1],1,true,0x70000025u);
    expect_handover_packet(1,PACKET,0,false);
    observe_commit(addresses[0],0,true,0x70000026u);
    assert(usb_hw->ep_stall_arm == 3u);
    assert(buffer_regs()[0] & USB_BUF_CTRL_STALL);
    assert(buffer_regs()[1] & USB_BUF_CTRL_STALL);
    assert(!native_test_in(0,data,&length,false));
    const marker_receipt_t receipt = capture_marker(CHILDREN,0);
    dump_through(1);
    expect_marker(0,&receipt);
    const char* first = record_line(0,0);
    uint32_t before, after;
    const char* clocks = strstr(first," clock=");
    assert(clocks && sscanf(clocks," clock=%"SCNx32"/%"SCNx32,&before,&after) == 2);
    assert(field(first," pre=",10) == 0 && before == 0 && first_commit < after);
    assert(field(first," commit=",16) == first_commit);
    for (unsigned index = 1; index <= 2; ++index) {
        const char* line = record_line(0,index);
        assert(field(line," pre=",10) == 0);
        assert(field(line," commit=",16) == first_commit);
    }
    unsigned records = field(snapshot_line(0,"[HUB_FLIGHT_FREEZE]")," n=",10);
    for (unsigned index = 0; index < records; ++index) {
        const char* line = record_line(0,index);
        if (!field(line," ok=",10)) assert(field(line," commit=",16) == 0);
    }
}

static uint16_t latched_bulk_packet(uint8_t slot, bool in, bool host_data1,
                                    uint8_t* data, uint16_t length) {
    const unsigned channel = in ? 4u : 5u;
    const uint32_t control = in ? commit_latched_bank.dpram.ep_ctrl[1].in :
        commit_latched_bank.dpram.ep_ctrl[1].out;
    const uint32_t packet = in ? commit_latched_bank.dpram.ep_buf_ctrl[2].in :
        commit_latched_bank.dpram.ep_buf_ctrl[2].out;
    assert(active_device == slot && commit_owner == slot);
    assert(usb_hw->dev_addr_ctrl == addresses[slot]);
    assert(control & EP_CTRL_ENABLE_BITS);
    assert((control & 0xffffu) == data_offset(slot,channel));
    assert(!(packet & (USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_STALL)));
    assert(((packet & USB_BUF_CTRL_DATA1_PID) != 0) == host_data1 &&
        "commit-time DATA PID disagrees with independent host sequence");
    assert(buffer_regs()[channel] == (packet | USB_BUF_CTRL_AVAIL));
    uint8_t* payload = (uint8_t*)USBCTRL_DPRAM_BASE + (control & 0xffffu);
    if (in) {
        assert(packet & USB_BUF_CTRL_FULL);
        assert((packet & USB_BUF_CTRL_LEN_MASK) <= length);
        length = packet & USB_BUF_CTRL_LEN_MASK;
        copy_from_usb(data,payload,length);
        buffer_regs()[channel] = packet;
    } else {
        assert(!(packet & USB_BUF_CTRL_FULL));
        assert(length <= (packet & USB_BUF_CTRL_LEN_MASK));
        copy_to_usb(payload,data,length);
        buffer_regs()[channel] = (packet & ~USB_BUF_CTRL_LEN_MASK) | length;
    }
    // Deliver only a correctly matched transaction. There is deliberately no
    // model of hardware ACK/discard behavior for an incorrect DATA PID.
    usb_hw->buf_status |= 1u << channel;
    usb_hw->ints |= USB_INTS_BUFF_STATUS_BITS;
    native_test_service_interrupt();
    assert(!failed);
    return length;
}

static void bulk_commit_pids(void) {
    uint8_t commands[CHILDREN][PACKET], replies[CHILDREN][PACKET], received[PACKET];
    uint16_t command_lengths[CHILDREN], reply_lengths[CHILDREN];
    for (uint8_t slot = 1; slot <= CHILDREN; ++slot) configure_child(slot);
    native_test_sio.mtime = 2000;
    // Slots name virtual USB child ports, not controller profile identities.
    // The host starts each EP2 direction at DATA0 after configuration, then
    // expects DATA1 after one completion. Never infer this from ep.next_pid.
    for (unsigned round = 0; round < 2; ++round) {
        const bool host_data1 = round != 0;
        for (uint8_t slot = 1; slot <= CHILDREN; ++slot) {
            const unsigned instance = slot-1u;
            command_lengths[instance] = round ? 16u+slot : 16u;
            reply_lengths[instance] = 19u+slot+round*7u;
            for (unsigned i = 0; i < PACKET; ++i) {
                commands[instance][i] = (uint8_t)(slot*0x21u+round*0x43u+i);
                replies[instance][i] = (uint8_t)(slot*0x31u+round*0x57u+i*3u);
            }
            assert(native_hub_vendor_write(instance,replies[instance],reply_lengths[instance]) ==
                reply_lengths[instance]);
            assert(native_hub_vendor_write_flush(instance) == reply_lengths[instance]);
        }
        assert(native_test_select(0));
        for (uint8_t slot = 1; slot <= CHILDREN; ++slot) {
            const unsigned instance = slot-1u;
            observe_commit(addresses[slot],slot,true,0x70001000u+round*CHILDREN+slot);
            assert(latched_bulk_packet(slot,false,host_data1,commands[instance],
                command_lengths[instance]) == command_lengths[instance]);
            const uint16_t length = latched_bulk_packet(slot,true,host_data1,received,sizeof(received));
            assert(length == reply_lengths[instance]);
            assert(memcmp(received,replies[instance],length) == 0);
            // IRQs consumed the bank, but callbacks/rearming await Core0. No
            // foreground pass may repair a bank before its modeled packets.
            assert(native_test_received_count[instance][1] == round);
            assert(native_test_bulk_completions[instance] == round);
            uint16_t unarmed_length = UINT16_MAX;
            assert(!native_test_private_in(slot,0x82,received,&unarmed_length));
            assert(!native_test_private_out(slot,0x02,commands[instance],command_lengths[instance],false));
        }
        native_test_drain();
        for (unsigned instance = 0; instance < CHILDREN; ++instance) {
            assert(native_test_received_count[instance][1] == round+1u);
            assert(native_test_received_length[instance][1] == command_lengths[instance]);
            assert(memcmp(native_test_received_data[instance][1],commands[instance],
                command_lengths[instance]) == 0);
            assert(native_test_bulk_completions[instance] == round+1u);
            assert(native_test_received_count[instance][0] == 0);
            assert(native_test_hid_completions[instance] == 0);
        }
        // Draining and selecting every bank again must not replay a completion.
        for (uint8_t slot = 1; slot <= CHILDREN; ++slot) {
            uint16_t length = UINT16_MAX;
            assert(!native_test_private_in(slot,0x82,received,&length));
        }
        native_test_drain();
        for (unsigned instance = 0; instance < CHILDREN; ++instance) {
            assert(native_test_received_count[instance][1] == round+1u);
            assert(native_test_bulk_completions[instance] == round+1u);
        }
    }
}

static void ep0_handover(void) {
    const tusb_control_request_t request = {
        .bmRequestType = 0xc0, .bRequest = 0x5b, .wLength = 128,
    };
    uint8_t data[PACKET];
    uint16_t length = 0;
    const tusb_control_request_t configuration = {
        .bRequest = TUSB_REQ_SET_CONFIGURATION, .wValue = 1,
    };
    for (uint8_t slot = 1; slot < DEVICES; ++slot)
        assert(native_test_setup(slot,&configuration,true));
    for (uint8_t slot = 1; slot < DEVICES; ++slot)
        expect_handover_packet(slot,0,0,true);
    native_test_drain();
    const uint8_t input[] = {0xa5,0x6b,0xd2};
    assert(native_hub_hid_report(0,0x30,input,sizeof(input)));
    for (uint8_t slot = 0; slot < DEVICES; ++slot) {
        for (unsigned i = 0; i < sizeof(handover_reply[slot]); ++i)
            handover_reply[slot][i] = (uint8_t)(slot * 0x31u + i * 3u);
        assert(native_test_setup(slot,&request,true));
    }
    // A private completion stays queued while root/child and child/child INs
    // consume their already-prepared, distinct EP0 packets without foreground work.
    assert(native_test_private_in(1,0x81,data,&length));
    assert(length == sizeof(input)+1u && data[0] == 0x30);
    assert(memcmp(data+1,input,sizeof(input)) == 0);
    for (uint8_t slot = 0; slot < DEVICES; ++slot)
        expect_handover_packet(slot,0,slot == 0 ? 16 : PACKET,true);
    for (uint8_t slot = 0; slot < DEVICES; ++slot)
        assert(!native_test_in(slot,data,&length,false));
    native_test_drain();
    assert(native_test_hid_completions[0] == 1 && native_hub_hid_ready(0));

    // Full short replies need a DATA0 padding ZLP; longer replies need their
    // exact DATA0 tail. Both are prepared while other owners hold the bank.
    for (uint8_t slot = 1; slot < DEVICES; ++slot)
        expect_handover_packet(slot,PACKET,handover_length(slot)-PACKET,false);
    for (uint8_t slot = 0; slot < DEVICES; ++slot)
        assert(!native_test_in(slot,data,&length,false));
    native_test_drain();
    for (uint8_t slot = 0; slot < DEVICES; ++slot) {
        assert(handover_acks[slot] == 0);
        assert(native_test_select(slot));
        assert(buffer_regs()[1] & USB_BUF_CTRL_DATA1_PID);
        assert(native_test_out(slot,NULL,0,false));
    }
    native_test_drain();
    for (uint8_t slot = 0; slot < DEVICES; ++slot) {
        assert(!native_test_in(slot,data,&length,false));
        assert(!native_test_out(slot,NULL,0,false));
        assert(handover_acks[slot] == 1);
    }
    native_test_drain();
    assert(native_test_hid_completions[0] == 1);
    assert(!native_test_private_in(1,0x81,data,&length));

    // A rejected replacement SETUP must restore STALL, never the old ready
    // root reply. Reset then revokes both queued and still-prepared child data.
    for (uint8_t slot = 0; slot < DEVICES; ++slot)
        assert(native_test_setup(slot,&request,true));
    tusb_control_request_t rejected = request;
    rejected.bRequest = 0x5c;
    assert(!native_test_setup(0,&rejected,true));
    expect_handover_packet(1,0,PACKET,true);
    assert(!native_test_in(0,data,&length,false));
    assert(buffer_regs()[0] & USB_BUF_CTRL_STALL);
    assert(native_hub_hid_report(0,0x30,input,sizeof(input)));
    native_test_bus_reset(false);
    // The reset IRQ must revoke inactive readiness before Core0 consumes reset.
    for (uint8_t slot = 0; slot < DEVICES; ++slot) {
        assert(!native_test_in(slot,data,&length,false));
        assert(!native_test_out(slot,NULL,0,false));
        if (slot) {
            assert(!native_test_private_in(slot,0x81,data,&length));
            assert(!native_test_private_out(slot,0x01,input,sizeof(input),false));
        }
    }
    native_test_drain();
    for (uint8_t slot = 0; slot < DEVICES; ++slot) {
        assert(!native_test_in(slot,data,&length,false));
        assert(!native_test_out(slot,NULL,0,false));
        assert(handover_acks[slot] == 1);
        if (slot) assert(native_test_hid_completions[slot-1] == (slot == 1 ? 1u : 0u));
    }
}

int main(int argc, char** argv) {
    assert(argc >= 2);
    native_test_initialize();
    for (unsigned i = 0; i < sizeof(vendor_reply); ++i) vendor_reply[i] = (uint8_t)(i ^ 0x5a);
    for (uint8_t slot = 0; slot < DEVICES; ++slot)
        for (unsigned i = 0; i < sizeof(approved_reply[slot]); ++i)
            approved_reply[slot][i] = (uint8_t)(slot * 0x31u + i * 3u);
    if (strcmp(argv[1],"live-wrap") == 0) live_wrap();
    else if (strcmp(argv[1],"root-idle") == 0) root_does_not_rearm();
    else if (strcmp(argv[1],"queue-pressure") == 0) queue_pressure();
    else if (strcmp(argv[1],"backpressure") == 0) {
        assert(argc == 3);
        backpressure(strcmp(argv[2],"full") == 0);
    }
    else if (strcmp(argv[1],"pending") == 0) pending_one_shot();
    else if (strcmp(argv[1],"superseded") == 0) superseded();
    else if (strcmp(argv[1],"immediate-supersession") == 0) immediate_supersession();
    else if (strcmp(argv[1],"poll-retention") == 0) poll_retention();
    else if (strcmp(argv[1],"selection-history") == 0) selection_history();
    else if (strcmp(argv[1],"frozen-selection-history") == 0) frozen_selection_history();
    else if (strcmp(argv[1],"delayed-publication") == 0) delayed_publication();
    else if (strcmp(argv[1],"publication-isolation") == 0) publication_isolation();
    else if (strcmp(argv[1],"publication-wrap-supersession") == 0) publication_wrap_supersession();
    else if (strcmp(argv[1],"ep0-handover") == 0) ep0_handover();
    else if (strcmp(argv[1],"coherent-publication") == 0) coherent_publication();
    else if (strcmp(argv[1],"bulk-commit-pids") == 0) bulk_commit_pids();
    else if (strcmp(argv[1],"approved-status-handoff") == 0) approved_status_handoff();
    else if (strcmp(argv[1],"approved-status-superseded") == 0) approved_status_superseded();
    else if (strcmp(argv[1],"approved-status-reset") == 0) approved_status_reset();
    else if (strcmp(argv[1],"approved-status-reset-during-completion") == 0) approved_status_reset_during_completion();
    else if (strcmp(argv[1],"approved-status-port-reset-ready") == 0) approved_status_port_reset(false);
    else if (strcmp(argv[1],"approved-status-port-reset-queued") == 0) approved_status_port_reset(true);
    else if (strcmp(argv[1],"approved-status-invalid-length") == 0) approved_status_invalid_length();
    else if (strcmp(argv[1],"approved-status-watch") == 0) approved_status_watch();
    else if (strcmp(argv[1],"status-out-rejected-data") == 0) status_out_rejected_data();
    else if (strcmp(argv[1],"status-out") == 0) status_out_lifecycle();
    else if (strcmp(argv[1],"status-out-superseded") == 0) status_out_superseded_arm();
    else if (strcmp(argv[1],"status-out-stale") == 0) status_out_stale_completion(false);
    else if (strcmp(argv[1],"status-out-reset") == 0) status_out_stale_completion(true);
    else if (strcmp(argv[1],"status-out-reset-watch") == 0) status_out_reset_watch(false);
    else if (strcmp(argv[1],"status-out-port-reset-watch") == 0) status_out_reset_watch(true);
    else if (strcmp(argv[1],"marker-priority") == 0) {
        assert(argc == 3);
        marker_priority(strcmp(argv[2],"partial") == 0);
    }
    else if (strcmp(argv[1],"marker-zero") == 0) marker_zero_and_capacity();
    else if (strcmp(argv[1],"marker-active") == 0) marker_active_priority();
    else if (strcmp(argv[1],"marker-rejected") == 0) marker_rejected_requests();
    else assert(false && "unknown trace scenario");
    fprintf(stderr,"native hub trace scenario %s passed (%u children)\n",argv[1],(unsigned)CHILDREN);
    return 0;
}
