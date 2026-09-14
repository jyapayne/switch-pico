#include "usb/usb_configuration_management.h"
#include "usb/native_hub/native_hub.h"
#include "platform/pico/pico_profile_storage.h"
#include "adapter/adapter_usb_mode.h"
#include "bootsel.h"
#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>

extern "C" {
void native_test_initialize(void);
void native_test_drain(void);
bool native_test_setup(uint8_t, const tusb_control_request_t*, bool);
bool native_test_out(uint8_t, const uint8_t*, uint16_t, bool);
bool native_test_in(uint8_t, uint8_t*, uint16_t*, bool);
void native_test_bus_reset(bool);
void native_test_hold_abort(bool);
bool native_test_select(uint8_t);
bool native_test_private_in(uint8_t, uint8_t, uint8_t*, uint16_t*);
extern uint32_t native_test_interrupt_mask;
}

namespace {
using namespace UsbConfigurationManagement;
std::array<uint8_t, PROFILE_STORAGE_ARENA_COUNT * PROFILE_STORAGE_ARENA_SIZE> flash;
uint32_t programs = 0;
uint32_t erases = 0;
uint32_t bootsel_calls = 0;
std::array<uint8_t, 64> child_identity[2];
bool interleave_identity_ack = false;

void require(bool condition, const char* message) {
    if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
[[noreturn]] void unexpected_mutation() {
    require(false, "profile editor invoked an unrelated configuration/pairing/reboot mutation");
    std::abort();
}
uint16_t u16(const std::vector<uint8_t>& data, size_t offset) {
    return data.at(offset) | (static_cast<uint16_t>(data.at(offset + 1)) << 8);
}
uint32_t u32(const std::vector<uint8_t>& data, size_t offset) {
    return u16(data, offset) | (static_cast<uint32_t>(u16(data, offset + 2)) << 16);
}
void put16(std::vector<uint8_t>& data, size_t offset, uint16_t value) {
    data.at(offset) = value; data.at(offset + 1) = value >> 8;
}
void put32(std::vector<uint8_t>& data, size_t offset, uint32_t value) {
    put16(data, offset, value); put16(data, offset + 2, value >> 16);
}
tusb_control_request_t request(Operation op, bool input, uint16_t length) {
    tusb_control_request_t setup{};
    setup.bmRequestType = input ? 0xc0 : 0x40;
    setup.bRequest = static_cast<uint8_t>(op);
    setup.wValue = kRequestValue; setup.wIndex = kRequestIndex; setup.wLength = length;
    return setup;
}
std::vector<uint8_t> envelope(Operation op, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> bytes(kRequestHeaderSize + payload.size());
    memcpy(bytes.data(), "SPMG", 4);
    bytes[4] = kProtocolVersion; bytes[5] = static_cast<uint8_t>(op);
    put16(bytes, 8, payload.size());
    put32(bytes, 12, configuration_crc32(payload.data(), payload.size()));
    std::copy(payload.begin(), payload.end(), bytes.begin() + kRequestHeaderSize);
    return bytes;
}
void acknowledge(uint8_t slot = 0, bool drain = true) {
    uint8_t packet[64]; uint16_t length = 0xffff;
    require(native_test_in(slot, packet, &length, drain) && length == 0,
            "OUT transfer did not complete with a real zero-length status packet");
}
std::vector<uint8_t> receive(uint8_t slot = 0) {
    std::vector<uint8_t> bytes;
    for (;;) {
        uint8_t packet[64]; uint16_t length = 0;
        require(native_test_in(slot, packet, &length, true), "control IN packet was not available");
        bytes.insert(bytes.end(), packet, packet + length);
        if (length < 64) break;
    }
    require(native_test_out(slot, nullptr, 0, true), "control IN status OUT was rejected");
    return bytes;
}
std::vector<uint8_t> read_operation(Operation op) {
    const auto setup = request(op, true, kMaximumResponseSize);
    require(native_test_setup(0, &setup, true), "root management read stalled");
    auto bytes = receive();
    require(bytes.size() >= kResponseHeaderSize && memcmp(bytes.data(), "SPMG", 4) == 0 &&
            bytes[5] == static_cast<uint8_t>(op) &&
            bytes.size() == kResponseHeaderSize + u16(bytes, 8), "management read envelope is corrupt");
    require(u32(bytes, 16) == configuration_crc32(bytes.data() + kResponseHeaderSize,
            bytes.size() - kResponseHeaderSize), "multi-packet response CRC is corrupt");
    return bytes;
}
void write_operation(Operation op, const std::vector<uint8_t>& payload, bool ack = true) {
    const auto bytes = envelope(op, payload);
    const auto setup = request(op, false, bytes.size());
    require(native_test_setup(0, &setup, true), "root management write setup stalled");
    for (size_t offset = 0; offset < bytes.size(); offset += 64) {
        require(native_test_out(0, bytes.data() + offset,
                    std::min<size_t>(64, bytes.size() - offset), true), "management OUT packet stalled");
    }
    if (ack) acknowledge();
}
std::vector<uint8_t> identity_payload(uint8_t profile) {
    std::vector<uint8_t> payload(15);
    require(controller_identity_encode(controller_identity_global(), payload.data(), 14), "global identity did not encode");
    payload[14] = profile; return payload;
}
std::vector<uint8_t> encoded_profile(uint8_t destination) {
    ControllerProfile profile = controller_profile_default(controller_identity_global(), 0);
    profile.button_map[0] = destination;
    std::vector<uint8_t> bytes(CONTROLLER_PROFILE_ENCODED_SIZE);
    require(controller_profile_encode(profile, bytes.data(), bytes.size()), "edited profile is invalid");
    return bytes;
}
void begin_profile(uint32_t transaction, const std::vector<uint8_t>& profile) {
    std::vector<uint8_t> payload(28);
    put32(payload, 0, transaction);
    require(controller_identity_encode(controller_identity_global(), payload.data() + 4, 14), "profile owner did not encode");
    put16(payload, 20, CONTROLLER_PROFILE_SCHEMA_VERSION); put16(payload, 22, profile.size());
    put32(payload, 24, configuration_crc32(profile.data(), profile.size()));
    write_operation(Operation::kProfileBegin, payload);
    // Selecting an existing owner is part of normal editor navigation and
    // must not persist anything while a profile is merely being staged.
    write_operation(Operation::kProfileSelect, identity_payload(0));
}
std::vector<uint8_t> chunk_payload(uint32_t transaction, const std::vector<uint8_t>& profile, size_t offset) {
    const size_t count = std::min(kMaximumChunkSize, profile.size() - offset);
    std::vector<uint8_t> payload(8 + count);
    put32(payload, 0, transaction); put16(payload, 4, offset); put16(payload, 6, count);
    std::copy_n(profile.data() + offset, count, payload.data() + 8);
    return payload;
}
void stage_profile(uint32_t transaction, const std::vector<uint8_t>& profile) {
    begin_profile(transaction, profile);
    for (size_t offset = 0; offset < profile.size(); offset += kMaximumChunkSize)
        write_operation(Operation::kProfileChunk, chunk_payload(transaction, profile, offset));
}
std::vector<uint8_t> transaction_payload(uint32_t id) { std::vector<uint8_t> data(4); put32(data, 0, id); return data; }
void require_profile(const std::vector<uint8_t>& expected) {
    auto bytes = read_operation(Operation::kProfileRead);
    require(bytes[6] == static_cast<uint8_t>(Status::kOk) &&
            std::vector<uint8_t>(bytes.begin() + kResponseHeaderSize, bytes.end()) == expected,
            "host readback differs from the durable selected profile");
}
void read_child(uint8_t slot) {
    tusb_control_request_t setup{};
    setup.bmRequestType = 0xc0; setup.bRequest = 3; setup.wLength = 128;
    require(native_test_setup(slot, &setup, true), "native child identity stalled");
    const auto bytes = receive(slot);
    require(bytes == std::vector<uint8_t>(child_identity[slot - 1].begin(), child_identity[slot - 1].end()),
            "native child identity leaked root or sibling vendor bytes");
}

void test_profile_transport() {
    const uint32_t programs_before = programs, erases_before = erases;
    const auto original = encoded_profile(0);
    const auto edited = encoded_profile(4);
    auto info = read_operation(Operation::kInfo);
    require(info.size() == kResponseHeaderSize + 8 &&
            info[kResponseHeaderSize + 4] == 5 && info[kResponseHeaderSize + 5] == 7,
            "native INFO does not describe the fixed output and its capabilities");
    auto list = read_operation(Operation::kProfileList);
    require(list[kResponseHeaderSize] == 1 && list.size() > 64, "root catalog omitted the global profile owner");
    auto playtest = read_operation(Operation::kProfilePlaytest);
    require(playtest[kResponseHeaderSize] == 0 && playtest[kResponseHeaderSize + 1] == 0xff,
            "disconnected playtest fabricated controller input");
    require_profile(original);
    require(programs == programs_before && erases == erases_before, "editor reads wrote saved storage");
    for (uint8_t slot : {1, 2}) {
        const auto management = request(Operation::kProfileList, true, kMaximumResponseSize);
        require(!native_test_setup(slot, &management, true), "native child accepted regular management");
        read_child(slot);
    }
    for (Operation op : {Operation::kModeSet, Operation::kReboot}) {
        const auto setup = request(op, false, kRequestHeaderSize + (op == Operation::kModeSet ? 5 : 4));
        require(!native_test_setup(0, &setup, true), "fixed native image accepted mode switching");
    }

    begin_profile(1, edited);
    const auto chunk = envelope(Operation::kProfileChunk, chunk_payload(1, edited, 0));
    const auto setup = request(Operation::kProfileChunk, false, chunk.size());
    require(native_test_setup(0, &setup, true) && native_test_out(0, chunk.data(), 64, true), "first full OUT packet failed");
    read_child(1); read_child(2);
    const auto child_management = request(Operation::kInfo, true, kMaximumResponseSize);
    require(!native_test_setup(1, &child_management, true), "child INFO was accepted during a root write");
    require(native_test_out(0, chunk.data() + 64, chunk.size() - 64, true), "interleaved child requests corrupted root OUT tail");
    acknowledge();
    for (size_t offset = kMaximumChunkSize; offset < edited.size(); offset += kMaximumChunkSize)
        write_operation(Operation::kProfileChunk, chunk_payload(1, edited, offset));
    write_operation(Operation::kProfileCommit, transaction_payload(1), false);
    profile_service_task_on_storage_core(1000);
    require(programs == programs_before && erases == erases_before, "profile persisted before its status ACK");

    // Host sends its next SETUP before Core0 drains the already completed ACK.
    acknowledge(0, false);
    const auto next = request(Operation::kInfo, true, kMaximumResponseSize);
    require(native_test_setup(0, &next, false), "next root SETUP was rejected");
    native_test_drain(); receive();
    profile_service_task_on_storage_core(1000);
    require_profile(edited);
    auto status = read_operation(Operation::kProfileTransactionStatus);
    require(status[6] == static_cast<uint8_t>(Status::kOk) && u32(status, kResponseHeaderSize) == 1,
            "genuine queued ACK lost its commit when the next SETUP arrived");

    // Reopen the actual storage journal, not the service's published cache.
    ProfileStorage reopened;
    ControllerProfile stored{}, sibling{};
    require(reopened.initialize(pico_profile_storage_io()) &&
            reopened.get(controller_identity_global(), 0, &stored) == ProfileStorageResult::kOk &&
            reopened.get(controller_identity_global(), 1, &sibling) == ProfileStorageResult::kOk &&
            stored.button_map[0] == 4 && sibling.button_map[0] == 0,
            "profile journal lost the edit or modified an unrelated profile");
}

void test_interrupted_transactions() {
    const auto retained = encoded_profile(4), replacement = encoded_profile(5);
    const uint32_t programs_before = programs, erases_before = erases;
    stage_profile(2, replacement);
    write_operation(Operation::kProfileCommit, transaction_payload(2), false);
    // No status token completed: a fresh root SETUP aborts the write.
    read_operation(Operation::kInfo);
    profile_service_task_on_storage_core(2000);
    require_profile(retained);
    require(programs == programs_before && erases == erases_before, "aborted status committed stale profile data");
    // The shared service intentionally retains incomplete receives. An
    // explicit wrong-ID commit ends one with OutOfOrder, never a flash write.
    write_operation(Operation::kProfileCommit, transaction_payload(0));

    begin_profile(3, replacement);
    const auto chunk = envelope(Operation::kProfileChunk, chunk_payload(3, replacement, 0));
    const auto setup = request(Operation::kProfileChunk, false, chunk.size());
    require(native_test_setup(0, &setup, true) && native_test_out(0, chunk.data(), 64, true), "aborted-packet setup failed");
    read_operation(Operation::kInfo);
    require(!native_test_out(0, chunk.data() + 64, chunk.size() - 64, true), "new SETUP left an aborted OUT tail armed");
    require(native_test_setup(0, &setup, true) && native_test_out(0, chunk.data(), 64, true), "short-packet setup failed");
    require(!native_test_out(0, chunk.data() + 64, chunk.size() - 65, true), "short OUT tail was accepted");
    auto status = read_operation(Operation::kProfileTransactionStatus);
    require(u16(status, kResponseHeaderSize + 4) == 0, "short OUT appended stale bytes to a profile");
    auto corrupt = chunk;
    corrupt[12] ^= 1;
    require(native_test_setup(0, &setup, true) && native_test_out(0, corrupt.data(), 64, true), "bad-CRC setup failed");
    require(!native_test_out(0, corrupt.data() + 64, corrupt.size() - 64, true), "bad CRC acquired a status ACK");
    write_operation(Operation::kProfileCommit, transaction_payload(0));

    stage_profile(4, replacement);
    write_operation(Operation::kProfileCommit, transaction_payload(4), false);
    // The SIE captured ACK, but a bus reset revoked the queued transaction
    // before the service consumed it. A SETUP must not be conflated with reset.
    acknowledge(0, false);
    native_test_bus_reset(true);
    profile_service_task_on_storage_core(3000);
    require_profile(retained);
    require(programs == programs_before && erases == erases_before, "bus reset committed a stale queued profile ACK");
    write_operation(Operation::kProfileCommit, transaction_payload(0));

    stage_profile(5, replacement);
    write_operation(Operation::kProfileCommit, transaction_payload(5));
    profile_service_task_on_storage_core(3000);
    require_profile(replacement);
    // A repeated status token is unarmed and cannot dispatch the commit twice.
    uint8_t packet[64]; uint16_t length;
    require(!native_test_in(0, packet, &length, true), "completed request retained a second status ACK");
}

void test_pending_control_buffer_ownership() {
    const auto info = request(Operation::kInfo, true, kMaximumResponseSize);
    require(native_test_setup(0, &info, true), "pending INFO setup failed");
    const unsigned programs_before = programs, erases_before = erases;
    native_test_hold_abort(true);
    const auto replacement = request(Operation::kProfileSelect, false, kRequestHeaderSize + 15);
    require(!native_test_setup(0, &replacement, false),
            "new SETUP replaced an EP0 buffer before the controller released ownership");
    uint8_t packet[64]; uint16_t length;
    require(!native_test_in(0, packet, &length, false),
            "an unquiesced control endpoint acknowledged replacement work");
    profile_service_task_on_storage_core(4000);
    require(programs == programs_before && erases == erases_before,
            "unquiesced replacement changed saved profiles");
    native_test_initialize();
}

void test_read_ack_allows_usb_progress() {
    interleave_identity_ack = true;
    read_child(1);
    const auto next = receive(2);
    require(next == std::vector<uint8_t>(child_identity[1].begin(), child_identity[1].end()),
            "SETUP received during read ACK did not retain the next child's response");
}

void test_private_transmit_survives_round_robin_tokens() {
    native_test_initialize();
    tusb_control_request_t configuration{};
    configuration.bRequest = TUSB_REQ_SET_CONFIGURATION;
    configuration.wValue = 1;
    for (uint8_t slot : {1, 2}) {
        require(native_test_setup(slot, &configuration, true), "child configuration failed");
        acknowledge(slot);
    }
    const uint8_t payloads[2][3] = {{0x11, 0x22, 0x33}, {0x44, 0x55, 0x66}};
    for (uint8_t instance : {0, 1}) {
        require(native_hub_hid_report(instance, instance ? 7 : 8, payloads[instance], 3),
                "could not queue HID packet");
        require(native_hub_vendor_write(instance, payloads[instance], 3) == 3 &&
                native_hub_vendor_write_flush(instance) == 3, "could not queue bulk packet");
    }
    uint8_t packet[64];
    uint16_t length = 0;
    for (uint8_t endpoint : {0x81, 0x82}) {
        for (uint8_t slot : {1, 2}) {
            require(native_test_private_in(slot, endpoint, packet, &length),
                    "queued private IN packet required foreground work after bank selection");
            const unsigned prefix = endpoint == 0x81 ? 1 : 0;
            require(length == 3 + prefix &&
                    (!prefix || packet[0] == (slot == 1 ? 8 : 7)) &&
                    std::memcmp(packet + prefix, payloads[slot - 1], 3) == 0,
                    "round-robin IN token received another endpoint's payload");
        }
    }
    native_test_drain();
    require(native_hub_hid_ready(0) && native_hub_hid_ready(1),
            "acknowledged HID packets did not release their queues");
    require(!native_test_private_in(1, 0x81, packet, &length),
            "acknowledged HID packet was retransmitted");
    // The idle poll selected R without restoring its shared EP0 image.
    require(native_hub_hid_report(0, 8, payloads[0], 3), "could not queue the next HID packet");
    require(native_test_private_in(1, 0x81, packet, &length) && length == 4 &&
            std::memcmp(packet + 1, payloads[0], 3) == 0,
            "pending shared EP0 restoration blocked a newly queued private IN packet");
    native_test_drain();
    native_test_initialize();
}
void test_masked_irq_completion_handoff() {
    native_test_initialize();
    tusb_control_request_t configuration{};
    configuration.bRequest = TUSB_REQ_SET_CONFIGURATION;
    configuration.wValue = 1;
    for (uint8_t slot : {1, 2}) {
        require(native_test_setup(slot, &configuration, true), "child configuration failed");
        acknowledge(slot);
    }
    const uint8_t payloads[2][3] = {{0x12, 0x34, 0x56}, {0x78, 0x9a, 0xbc}};
    for (uint8_t instance : {0, 1})
        require(native_hub_hid_report(instance, 8, payloads[instance], 3),
                "could not queue masked-window HID packet");
    uint8_t packet[64];
    uint16_t length = 0;
    native_test_interrupt_mask = 1;
    require(native_test_private_in(1, 0x81, packet, &length),
            "first controller did not complete during masked window");
    require(!native_test_select(2),
            "pending completion must prevent overwriting the active bank");
    native_hub_service_pending_usb();
    require(native_test_interrupt_mask == 1,
            "SRAM service must preserve the caller's interrupt mask");
    require(native_test_private_in(2, 0x81, packet, &length) && length == 4 &&
                packet[0] == 8 && std::memcmp(packet + 1, payloads[1], 3) == 0,
            "SRAM service did not permit the other controller's real packet");
    native_hub_service_pending_usb();
    require(!native_hub_hid_ready(0) && !native_hub_hid_ready(1),
            "SRAM service must defer protocol callbacks to foreground dispatch");
    native_test_interrupt_mask = 0;
    native_test_drain();
    require(native_hub_hid_ready(0) && native_hub_hid_ready(1),
            "deferred completions did not release both controller queues");
    require(!native_test_private_in(1, 0x81, packet, &length),
            "later IRQ dispatch duplicated a serviced completion");
    native_test_initialize();
}


void test_private_bootsel() {
    const auto bytes = envelope(Operation::kBootselReboot, {});
    const auto setup = request(Operation::kBootselReboot, false, bytes.size());
    for (uint8_t slot : {0, 1, 2}) {
        require(native_test_setup(slot, &setup, true), "private BOOTSEL setup stalled");
        require(!native_test_out(slot, bytes.data(), bytes.size() - 1, true), "short BOOTSEL was accepted");
        probe_bootsel_task(100); probe_bootsel_task(200);
        require(bootsel_calls == 0, "short BOOTSEL rebooted the device");
        require(native_test_setup(slot, &setup, true) && native_test_out(slot, bytes.data(), bytes.size(), true),
                "valid private BOOTSEL envelope failed");
        // An unrelated identity/INFO SETUP cancels an unacknowledged BOOTSEL.
        if (slot) read_child(slot); else read_operation(Operation::kInfo);
        probe_bootsel_task(300); probe_bootsel_task(400);
        require(bootsel_calls == 0, "unacknowledged BOOTSEL rebooted the device");
    }
    require(native_test_setup(2, &setup, true) && native_test_out(2, bytes.data(), bytes.size(), true),
            "validated child BOOTSEL failed");
    acknowledge(2);
    probe_bootsel_task(500); probe_bootsel_task(549);
    require(bootsel_calls == 0, "BOOTSEL did not retain the post-ACK delay");
    probe_bootsel_task(550);
    require(bootsel_calls == 1, "validated child BOOTSEL did not reach ROM after the delay");
}

bool flash_read(void*, uint8_t arena, size_t offset, uint8_t* data, size_t size) {
    if (arena >= PROFILE_STORAGE_ARENA_COUNT || offset > PROFILE_STORAGE_ARENA_SIZE ||
        size > PROFILE_STORAGE_ARENA_SIZE - offset) return false;
    memcpy(data, flash.data() + arena * PROFILE_STORAGE_ARENA_SIZE + offset, size); return true;
}
bool flash_erase(void*, uint8_t arena) {
    if (arena >= PROFILE_STORAGE_ARENA_COUNT) return false;
    ++erases; memset(flash.data() + arena * PROFILE_STORAGE_ARENA_SIZE, 0xff, PROFILE_STORAGE_ARENA_SIZE); return true;
}
bool flash_program(void*, uint8_t arena, size_t offset, const uint8_t* data, size_t size) {
    if (arena >= PROFILE_STORAGE_ARENA_COUNT || size != PROFILE_STORAGE_PAGE_SIZE ||
        offset % PROFILE_STORAGE_PAGE_SIZE || offset + size > PROFILE_STORAGE_ARENA_SIZE) return false;
    ++programs;
    uint8_t* destination = flash.data() + arena * PROFILE_STORAGE_ARENA_SIZE + offset;
    for (size_t i = 0; i < size; ++i) destination[i] &= data[i];
    return memcmp(destination, data, size) == 0;
}
} // namespace

ProfileStorageIo pico_profile_storage_io() {
    return {nullptr, PROFILE_STORAGE_ARENA_SIZE, PROFILE_STORAGE_SECTOR_SIZE,
            PROFILE_STORAGE_PAGE_SIZE, flash_read, flash_erase, flash_program};
}
uint32_t configuration_crc32(const uint8_t* data, size_t size) { return profile_storage_crc32(data, size); }
void configuration_service_snapshot(ConfigurationServiceSnapshot* out) {
    *out = {}; out->state = ConfigurationServiceState::kReady;
    out->configuration = adapter_configuration_default();
}
ConfigurationTransactionStatus configuration_service_begin(uint32_t, uint16_t, size_t, uint32_t) { unexpected_mutation(); }
ConfigurationTransactionStatus configuration_service_append(uint32_t, size_t, const uint8_t*, size_t) { unexpected_mutation(); }
ConfigurationTransactionStatus configuration_service_commit(uint32_t) { unexpected_mutation(); }
ConfigurationTransactionStatus configuration_service_reset(uint32_t) { unexpected_mutation(); }
ConfigurationTransactionStatus configuration_service_set_mode(uint32_t, AdapterRequestedMode, const AdapterModeAvailability&) { unexpected_mutation(); }
const AdapterModeAvailability& adapter_usb_mode_availability() { unexpected_mutation(); }
bool adapter_reboot_for_mode_transaction(uint32_t) { unexpected_mutation(); }
bool adapter_reboot_to_bootsel() { unexpected_mutation(); }
void bluepad32_input_backend_request_pairing_snapshot() { unexpected_mutation(); }
uint32_t bluepad32_input_backend_clear_pairings() { unexpected_mutation(); }
void bluepad32_input_backend_pairing_snapshot(Bluepad32PairingSnapshot* out) { *out = {}; }
void bluepad32_input_backend_playtest_snapshot(uint8_t, Bluepad32PlaytestSnapshot* out) { *out = {}; }
void bluepad32_input_backend_diagnostics(Bluepad32BackendDiagnostics* out) { *out = {}; }
bool bluepad32_input_backend_identify(const ControllerIdentity&) { return false; }
bool bluepad32_input_backend_set_wii_orientation(const ControllerIdentity&, uint32_t, bool) { return false; }
bool bluepad32_input_backend_capture_start(uint8_t, uint32_t, const CaptureOptions&) { return false; }
bool bluepad32_input_backend_capture_stop(uint32_t) { return false; }
bool bluepad32_input_backend_capture_page(uint32_t, uint16_t, Bluepad32CaptureSnapshot*) { return false; }
extern "C" void reset_usb_boot(uint32_t, uint32_t) { ++bootsel_calls; }
extern "C" bool tud_vendor_control_xfer_cb(uint8_t slot, uint8_t stage, const tusb_control_request_t* setup) {
    if (probe_management_vendor_control(slot, stage, setup)) return true;
    if (slot < 1 || slot > 2 || setup->bmRequestType != 0xc0 ||
        setup->bRequest != 3 || setup->wValue || setup->wIndex) return false;
    if (stage == CONTROL_STAGE_ACK && slot == 1 && interleave_identity_ack) {
        interleave_identity_ack = false;
        const auto next = *setup;
        require(native_test_setup(2, &next, false), "next child's SETUP was rejected during read ACK");
        // SETUP must be serviced before another token can change the bank.
        // This observes IRQ progress, rather than inspecting the CPU mask.
        require(native_test_select(0), "read ACK callback blocked servicing the next USB SETUP");
    }
    return stage != CONTROL_STAGE_SETUP || native_hub_control_xfer(slot, setup,
            child_identity[slot - 1].data(), child_identity[slot - 1].size());
}

int main() {
    static_assert(sizeof(tusb_control_request_t) == 8);
    flash.fill(0xff); child_identity[0].fill(0x31); child_identity[1].fill(0x72);
    profile_service_prepare(); profile_service_initialize_on_storage_core();
    native_test_initialize();
    test_profile_transport();
    test_interrupted_transactions();
    test_pending_control_buffer_ownership();
    test_read_ack_allows_usb_progress();
    test_private_transmit_survives_round_robin_tokens();
    test_masked_irq_completion_handoff();
    test_private_bootsel();
    std::cout << "native root management packet and persistence regressions passed\n";
}
