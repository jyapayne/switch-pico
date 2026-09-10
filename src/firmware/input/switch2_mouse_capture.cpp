#include "input/switch2_mouse_capture.h"

#ifdef SWITCH_PICO_SWITCH2_MOUSE_CAPTURE
#include <string.h>

#include "parser/uni_hid_parser_switch2.h"
#include "pico/critical_section.h"

namespace {
critical_section_t g_lock;
bool g_initialized;
uint8_t g_rows[SWITCH2_MOUSE_CAPTURE_CAPACITY][SWITCH2_MOUSE_CAPTURE_ROW_SIZE];
uint8_t g_next;
uint8_t g_count;
uint32_t g_total_records;
bool g_input_selected;
uint8_t g_input_address[6];
Switch2MouseCaptureInput g_latest_input;
#if SWITCH_PICO_SWITCH2_USB_BRIDGE
constexpr uint32_t kInputDeadlineMs = 500;
constexpr uint32_t kSampleDeadlineMs = 2000;
constexpr uint8_t kNativeReportCapacity = 32;
struct NativeReport {
    uint8_t report[SWITCH2_MOUSE_CAPTURE_NATIVE_INPUT_SIZE];
    uint32_t serial;
    uint32_t received_ms;
};
NativeReport g_native_reports[kNativeReportCapacity];
uint8_t g_native_head;
uint8_t g_native_count;
bool g_native_stream;
uint64_t g_sample_serial;
bool g_source_active;
struct {
    uint64_t token;
    uint32_t started_ms;
    uint32_t mouse_epoch;
    uint8_t sample_id;
    bool taken;
    bool acked;
} g_sample;

void clear_native_reports() {
    g_native_head = 0;
    g_native_count = 0;
}

bool source_fresh(uint32_t now_ms) {
    return g_input_selected && g_source_active && g_latest_input.active &&
           static_cast<int32_t>(now_ms - g_latest_input.received_ms) <
               static_cast<int32_t>(kInputDeadlineMs);
}

bool sample_current(uint32_t now_ms) {
    if (g_sample.token &&
        (!source_fresh(now_ms) || g_sample.mouse_epoch != g_latest_input.mouse_epoch ||
         static_cast<int32_t>(now_ms - g_sample.started_ms) >=
             static_cast<int32_t>(kSampleDeadlineMs))) {
        g_sample = {};
    }
    return g_sample.token != 0;
}
#endif

void write_u16(uint8_t* output, uint16_t value) {
    output[0] = static_cast<uint8_t>(value);
    output[1] = static_cast<uint8_t>(value >> 8);
}

void write_u32(uint8_t* output, uint32_t value) {
    output[0] = static_cast<uint8_t>(value);
    output[1] = static_cast<uint8_t>(value >> 8);
    output[2] = static_cast<uint8_t>(value >> 16);
    output[3] = static_cast<uint8_t>(value >> 24);
}

int32_t read_i16(const uint8_t* input) {
    const int32_t value = input[0] | (static_cast<int32_t>(input[1]) << 8);
    return value >= 0x8000 ? value - 0x10000 : value;
}
}  // namespace

void switch2_mouse_capture_init() {
    if (g_initialized) return;
    // The application and TinyUSB already consume all eight exclusive
    // spinlocks. Capture never nests another striped spinlock, so use the
    // SDK's shared stripe pool rather than exhausting startup allocation.
    critical_section_init_with_lock_num(&g_lock, next_striped_spin_lock_num());
    g_initialized = true;
}

extern "C" void switch_pico_switch2_mouse_report(
    uint16_t product_id, const uint8_t address[6], uint8_t report_id,
    const uint8_t* report, uint16_t length, uint32_t received_ms) {
    if (!g_initialized || address == nullptr ||
        (product_id != UNI_SW2_JOYCON_L_PID &&
         product_id != UNI_SW2_JOYCON_R_PID) ||
        length > SWITCH2_MOUSE_CAPTURE_REPORT_SIZE ||
        (length != 0 && report == nullptr)) {
        return;
    }
    if (report_id == 0) {
        if (length != 0) return;
    } else if (length == 0 ||
               (report_id != 0x05 && report_id != 0x07 &&
                report_id != 0x08 && report_id != 0xc0)) {
        return;
    }

    critical_section_enter_blocking(&g_lock);
#if SWITCH_PICO_SWITCH2_USB_BRIDGE
    // Teardown must invalidate source ownership even after capture serials are
    // exhausted; this is independent of whether a ring event can be recorded.
    if (report_id == 0 && g_input_selected && product_id == UNI_SW2_JOYCON_R_PID &&
        memcmp(address, g_input_address, sizeof(g_input_address)) == 0) {
        g_source_active = false;
        clear_native_reports();
        g_sample = {};
    }
#endif
    // Never reuse a serial within one boot, even after UINT32_MAX records.
    if (g_total_records == UINT32_MAX) {
#if SWITCH_PICO_SWITCH2_USB_BRIDGE
        clear_native_reports();
#endif
        critical_section_exit(&g_lock);
        return;
    }
    uint8_t* row = g_rows[g_next];
    write_u32(row, ++g_total_records);
#if SWITCH_PICO_SWITCH2_USB_BRIDGE
    // Exhaustion is terminal for the relay, including the last recorded event.
    if (g_total_records == UINT32_MAX) clear_native_reports();
#endif
    write_u32(row + 4, received_ms);
    write_u16(row + 8, product_id);
    row[10] = report_id;
    row[11] = static_cast<uint8_t>(length);
    memcpy(row + 12, address, 6);
    row[18] = row[19] = 0;
    if (length != 0) memcpy(row + 20, report, length);
    memset(row + 20 + length, 0, SWITCH2_MOUSE_CAPTURE_REPORT_SIZE - length);
    g_next = static_cast<uint8_t>((g_next + 1) % SWITCH2_MOUSE_CAPTURE_CAPACITY);
    if (g_input_selected && product_id == UNI_SW2_JOYCON_R_PID &&
        memcmp(address, g_input_address, sizeof(g_input_address)) == 0 &&
        (report_id == 0 ||
         (report_id == 0x08 && length == SWITCH2_MOUSE_CAPTURE_NATIVE_INPUT_SIZE))) {
        if (report_id == 0x08) {
            if (!g_latest_input.active) {
                g_latest_input.mouse_epoch = g_total_records;
                g_latest_input.mouse_total_x = 0;
                g_latest_input.mouse_total_y = 0;
            }
            g_latest_input.active = true;
#if SWITCH_PICO_SWITCH2_USB_BRIDGE
            g_source_active = true;
            if (g_native_stream && g_total_records != UINT32_MAX) {
                if (g_native_count == kNativeReportCapacity) clear_native_reports();
                NativeReport& packet =
                    g_native_reports[(g_native_head + g_native_count) % kNativeReportCapacity];
                memcpy(packet.report, report, sizeof(packet.report));
                packet.serial = g_total_records;
                packet.received_ms = received_ms;
                ++g_native_count;
            }
#endif
            memcpy(g_latest_input.buttons, report + 2, sizeof(g_latest_input.buttons));
            memcpy(g_latest_input.stick, report + 5, sizeof(g_latest_input.stick));
            g_latest_input.native_status = report[8];
            // At most UINT32_MAX signed16 additions per boot: magnitude < 2^47.
            // Every packet contributes, even when its delta matches the last.
            g_latest_input.mouse_total_x += read_i16(report + 9);
            g_latest_input.mouse_total_y += read_i16(report + 11);
            g_latest_input.mouse_surface = report[13];
        } else {
            g_latest_input = {};
        }
        g_latest_input.serial = g_total_records;
        g_latest_input.received_ms = received_ms;
    }
    if (g_count < SWITCH2_MOUSE_CAPTURE_CAPACITY) ++g_count;
    critical_section_exit(&g_lock);
}

size_t switch2_mouse_capture_snapshot(uint8_t* output, size_t capacity,
                                     uint32_t* total_records) {
    if (!g_initialized || output == nullptr || total_records == nullptr) return 0;

    critical_section_enter_blocking(&g_lock);
    const size_t required = SWITCH2_MOUSE_CAPTURE_HEADER_SIZE +
                            g_count * SWITCH2_MOUSE_CAPTURE_ROW_SIZE;
    if (capacity < required) {
        critical_section_exit(&g_lock);
        return 0;
    }
    *total_records = g_total_records;
    write_u32(output, g_total_records);
    output[4] = g_count;
    memset(output + 5, 0, SWITCH2_MOUSE_CAPTURE_HEADER_SIZE - 5);
    const size_t oldest = g_count == SWITCH2_MOUSE_CAPTURE_CAPACITY ? g_next : 0;
    const size_t first_count =
        g_count < SWITCH2_MOUSE_CAPTURE_CAPACITY - oldest
            ? g_count : SWITCH2_MOUSE_CAPTURE_CAPACITY - oldest;
    memcpy(output + SWITCH2_MOUSE_CAPTURE_HEADER_SIZE, g_rows[oldest],
           first_count * SWITCH2_MOUSE_CAPTURE_ROW_SIZE);
    if (first_count != g_count) {
        memcpy(output + SWITCH2_MOUSE_CAPTURE_HEADER_SIZE +
                   first_count * SWITCH2_MOUSE_CAPTURE_ROW_SIZE,
               g_rows[0], (g_count - first_count) * SWITCH2_MOUSE_CAPTURE_ROW_SIZE);
    }
    critical_section_exit(&g_lock);
    return required;
}

void switch2_mouse_capture_select_input(const uint8_t address[6]) {
    if (!g_initialized || address == nullptr) return;
    critical_section_enter_blocking(&g_lock);
#if SWITCH_PICO_SWITCH2_USB_BRIDGE
    g_source_active = false;
    g_native_stream = false;
    clear_native_reports();
    g_sample = {};
#endif
    memcpy(g_input_address, address, sizeof(g_input_address));
    g_latest_input = {};
    g_input_selected = true;
    critical_section_exit(&g_lock);
}

bool switch2_mouse_capture_latest_input(uint32_t after_serial,
                                       Switch2MouseCaptureInput* output) {
    if (!g_initialized || output == nullptr) return false;
    critical_section_enter_blocking(&g_lock);
    const bool fresh = g_latest_input.serial > after_serial;
    if (fresh) *output = g_latest_input;
    critical_section_exit(&g_lock);
    return fresh;
}

#if SWITCH_PICO_SWITCH2_USB_BRIDGE
void switch2_mouse_capture_set_native_stream(bool enabled) {
    if (!g_initialized) return;
    critical_section_enter_blocking(&g_lock);
    g_native_stream = enabled;
    if (!enabled) clear_native_reports();
    critical_section_exit(&g_lock);
}

uint32_t switch2_mouse_capture_peek_native_report(
    uint32_t now_ms, uint8_t report[SWITCH2_MOUSE_CAPTURE_NATIVE_INPUT_SIZE]) {
    if (!g_initialized || report == nullptr) return 0;
    critical_section_enter_blocking(&g_lock);
    uint32_t serial = 0;
    if (!g_native_stream || !source_fresh(now_ms) ||
        (g_native_count != 0 &&
         static_cast<int32_t>(now_ms - g_native_reports[g_native_head].received_ms) >=
             static_cast<int32_t>(kInputDeadlineMs))) {
        clear_native_reports();
    } else if (g_native_count != 0) {
        const NativeReport& packet = g_native_reports[g_native_head];
        memcpy(report, packet.report, sizeof(packet.report));
        serial = packet.serial;
    }
    critical_section_exit(&g_lock);
    return serial;
}

bool switch2_mouse_capture_commit_native_report(uint32_t serial) {
    if (!g_initialized || serial == 0) return false;
    critical_section_enter_blocking(&g_lock);
    const bool accepted = g_native_stream && g_native_count != 0 &&
                          g_native_reports[g_native_head].serial == serial;
    if (accepted) {
        g_native_head = static_cast<uint8_t>((g_native_head + 1) % kNativeReportCapacity);
        --g_native_count;
    }
    critical_section_exit(&g_lock);
    return accepted;
}

bool switch2_mouse_capture_request_sample(uint8_t sample_id, uint32_t now_ms,
                                          uint64_t* token) {
    if (token != nullptr) *token = 0;
    if (!g_initialized || token == nullptr || sample_id > 7) return false;
    critical_section_enter_blocking(&g_lock);
    const bool accepted = !sample_current(now_ms) && source_fresh(now_ms) &&
                          g_sample_serial != UINT64_MAX;
    if (accepted) {
        g_sample.token = ++g_sample_serial;
        g_sample.started_ms = now_ms;
        g_sample.mouse_epoch = g_latest_input.mouse_epoch;
        g_sample.sample_id = sample_id;
        *token = g_sample.token;
    }
    critical_section_exit(&g_lock);
    return accepted;
}

int switch2_mouse_capture_sample_result(uint64_t token, uint32_t now_ms) {
    if (!g_initialized || token == 0) return -1;
    critical_section_enter_blocking(&g_lock);
    int result = -1;
    if (sample_current(now_ms) && g_sample.token == token) {
        result = g_sample.acked ? 1 : 0;
        if (result == 1) g_sample = {};
    }
    critical_section_exit(&g_lock);
    return result;
}

void switch2_mouse_capture_cancel_sample() {
    if (!g_initialized) return;
    critical_section_enter_blocking(&g_lock);
    g_sample = {};
    critical_section_exit(&g_lock);
}

extern "C" bool switch_pico_switch2_sample_take(
    uint16_t product_id, const uint8_t address[6], uint32_t now_ms,
    uint8_t* sample_id, uint64_t* token) {
    if (!g_initialized || address == nullptr || sample_id == nullptr || token == nullptr)
        return false;
    critical_section_enter_blocking(&g_lock);
    const bool accepted = sample_current(now_ms) && !g_sample.taken &&
                          product_id == UNI_SW2_JOYCON_R_PID &&
                          memcmp(address, g_input_address, sizeof(g_input_address)) == 0;
    if (accepted) {
        g_sample.taken = true;
        *sample_id = g_sample.sample_id;
        *token = g_sample.token;
    }
    critical_section_exit(&g_lock);
    return accepted;
}

extern "C" bool switch_pico_switch2_sample_result(
    uint16_t product_id, const uint8_t address[6], uint64_t token,
    int result, uint32_t now_ms) {
    if (!g_initialized || address == nullptr || token == 0) return false;
    critical_section_enter_blocking(&g_lock);
    const bool accepted = sample_current(now_ms) && g_sample.taken &&
                          g_sample.token == token && product_id == UNI_SW2_JOYCON_R_PID &&
                          memcmp(address, g_input_address, sizeof(g_input_address)) == 0;
    if (accepted) {
        if (result > 0) g_sample.acked = true;
        else if (result < 0) g_sample = {};
    }
    critical_section_exit(&g_lock);
    return accepted;
}
#endif
#endif
