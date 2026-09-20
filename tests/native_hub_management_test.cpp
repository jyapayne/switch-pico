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
bool native_test_startup(void);
void native_test_advance(uint32_t);
bool native_test_setup(uint8_t, const tusb_control_request_t*, bool);
bool native_test_out(uint8_t, const uint8_t*, uint16_t, bool);
bool native_test_in(uint8_t, uint8_t*, uint16_t*, bool);
void native_test_bus_reset(bool);
void native_test_hold_abort(bool);
bool native_test_select(uint8_t);
bool native_test_private_in(uint8_t, uint8_t, uint8_t*, uint16_t*);
bool native_test_private_out(uint8_t, uint8_t, const uint8_t*, uint16_t, bool);
extern uint32_t native_test_hid_completions[PROBE_CONTROLLER_COUNT];
extern uint32_t native_test_bulk_completions[PROBE_CONTROLLER_COUNT];
extern uint32_t native_test_received_count[PROBE_CONTROLLER_COUNT][2];
extern uint16_t native_test_received_length[PROBE_CONTROLLER_COUNT][2];
extern uint8_t native_test_received_data[PROBE_CONTROLLER_COUNT][2][64];
extern uint32_t native_test_interrupt_mask;
}

namespace {
using namespace UsbConfigurationManagement;
std::array<uint8_t, PROFILE_STORAGE_ARENA_COUNT * PROFILE_STORAGE_ARENA_SIZE> flash;
uint32_t programs = 0;
uint32_t erases = 0;
uint32_t bootsel_calls = 0;
std::array<uint8_t, 64> child_identity[PROBE_CONTROLLER_COUNT];
bool interleave_identity_ack = false;
bool synthetic_root_management = false;
uint32_t bootsel_time_ms = 0;
uint32_t wake_request_calls = 0;
uint32_t last_wake_request_id = 0;
bool accept_wake_request = true;
Bluepad32Switch2WakeStatus wake_status{};

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
tusb_control_request_t child_request(Operation op, bool input, uint16_t length) {
    auto setup = request(op, input, length);
    setup.bmRequestType = input ? 0xc1 : 0x41;
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
void test_wake_transport() {
    const uint32_t writes_before = programs;
    const uint32_t erases_before = erases;
    const auto bytes = envelope(Operation::kSwitch2Wake, {1, 0, 0, 0});
    const auto setup = request(Operation::kSwitch2Wake, false, bytes.size());
    require(native_test_setup(0, &setup, true) &&
                native_test_out(0, bytes.data(), bytes.size(), true) &&
                wake_request_calls == 1,
            "native wake must be admitted before its status packet");
    acknowledge();
    require(wake_request_calls == 1, "native status ACK replayed wake work");
    wake_status = {1, Bluepad32Switch2WakeState::kQueued, false, false, 0, 0, 0};
    const auto status = read_operation(Operation::kSwitch2Wake);
    require(status.size() == 40 && status[6] == 0 && u16(status, 10) == 1 &&
                u32(status, 12) == 1 && u32(status, 20) == 1 && status[24] == 1 &&
                wake_request_calls == 1,
            "native root wake read must remain correlated and read-only");
    uint8_t packet[64];
    uint16_t length = 0;
    accept_wake_request = false;
    require(native_test_setup(0, &setup, true) &&
                !native_test_out(0, bytes.data(), bytes.size(), true) &&
                !native_test_in(0, packet, &length, true) && wake_request_calls == 2,
            "native busy wake must stall before its status packet");
    accept_wake_request = true;
    auto corrupt = bytes;
    corrupt[12] ^= 1;
    require(native_test_setup(0, &setup, true) &&
                !native_test_out(0, corrupt.data(), corrupt.size(), true) &&
                !native_test_in(0, packet, &length, true) && wake_request_calls == 2,
            "native malformed wake must not dispatch or obtain status authorization");
    require(read_operation(Operation::kSwitch2Wake) == status &&
                programs == writes_before && erases == erases_before,
            "volatile wake management must not alter status on reads or persist anything");
}

void read_child(uint8_t slot) {
    tusb_control_request_t setup{};
    setup.bmRequestType = 0xc0; setup.bRequest = 3; setup.wLength = 128;
    require(native_test_setup(slot, &setup, true), "native child identity stalled");
    const auto bytes = receive(slot);
    require(bytes == std::vector<uint8_t>(child_identity[slot - 1].begin(), child_identity[slot - 1].end()),
            "native child identity leaked root or sibling vendor bytes");
}
void test_child_wake_transport() {
    const uint32_t programs_before = programs, erases_before = erases;
    const auto info_setup = child_request(Operation::kInfo, true, kMaximumResponseSize);
    const auto status_setup = child_request(Operation::kSwitch2Wake, true, kMaximumResponseSize);
    const auto setup = child_request(Operation::kSwitch2Wake, false, kRequestHeaderSize + 4);
    const auto root_info = read_operation(Operation::kInfo);
    const auto root_status = read_operation(Operation::kSwitch2Wake);
    uint32_t expected_calls = wake_request_calls;
    uint8_t packet[64]; uint16_t length;
    for (uint8_t slot = 1; slot <= PROBE_CONTROLLER_COUNT; ++slot) {
        require(native_test_setup(slot, &info_setup, true) && receive(slot) == root_info &&
                native_test_setup(slot, &status_setup, true) && receive(slot) == root_status &&
                wake_request_calls == expected_calls,
                "child discovery/status must match the read-only root management schema");
        const auto bytes = envelope(Operation::kSwitch2Wake, {slot, 0, 0, 0});
        require(native_test_setup(slot, &setup, true) &&
                !native_test_in(slot, packet, &length, true) &&
                !probe_management_vendor_control(slot, CONTROL_STAGE_ACK, &setup) &&
                wake_request_calls == expected_calls,
                "child wake must not acknowledge or submit before DATA validation");
        require(native_test_out(slot, bytes.data(), bytes.size(), true) &&
                wake_request_calls == ++expected_calls && last_wake_request_id == slot,
                "child wake did not admit the requested ID before status ACK");
        require(probe_management_vendor_control(slot, CONTROL_STAGE_DATA, &setup),
                "repeated validated child DATA changed its admission result");
        acknowledge(slot);
        require(probe_management_vendor_control(slot, CONTROL_STAGE_ACK, &setup) &&
                probe_management_vendor_control(slot, CONTROL_STAGE_DATA, &setup) &&
                wake_request_calls == expected_calls,
                "duplicate child DATA/ACK repeated the wake mutation");
        accept_wake_request = false;
        require(native_test_setup(slot, &setup, true) &&
                !native_test_out(slot, bytes.data(), bytes.size(), true) &&
                !native_test_in(slot, packet, &length, true) &&
                wake_request_calls == ++expected_calls,
                "busy child wake must stall before its status packet");
        require(!probe_management_vendor_control(slot, CONTROL_STAGE_DATA, &setup) &&
                !probe_management_vendor_control(slot, CONTROL_STAGE_ACK, &setup) &&
                wake_request_calls == expected_calls,
                "repeated rejected child stages resubmitted wake");
        accept_wake_request = true;
        for (size_t offset : {0, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}) {
            auto corrupt = bytes;
            corrupt[offset] ^= 1;
            require(native_test_setup(slot, &setup, true) &&
                    !native_test_out(slot, corrupt.data(), corrupt.size(), true) &&
                    !native_test_in(slot, packet, &length, true),
                    "malformed child envelope obtained status authorization");
        }
        for (const auto& invalid : {envelope(Operation::kSwitch2Wake, {0, 0, 0, 0}),
                                   envelope(Operation::kSwitch2Wake, {0, 0, 0, 0x80})}) {
            require(native_test_setup(slot, &setup, true) &&
                    !native_test_out(slot, invalid.data(), invalid.size(), true) &&
                    !native_test_in(slot, packet, &length, true),
                    "invalid child request ID obtained status authorization");
        }
        require(native_test_setup(slot, &setup, true) &&
                !native_test_out(slot, bytes.data(), bytes.size() - 1, true) &&
                !native_test_in(slot, packet, &length, true) &&
                wake_request_calls == expected_calls,
                "short child OUT inherited a previous valid envelope");
        for (uint8_t recipient : {0x40, 0x42, 0x43, 0xc0, 0xc2, 0xc3}) {
            auto invalid = setup;
            invalid.bmRequestType = recipient;
            require(!native_test_setup(slot, &invalid, true),
                    "child management accepted the wrong recipient");
        }
        std::array<tusb_control_request_t, 5> invalid_setups;
        invalid_setups.fill(setup);
        invalid_setups[0].wValue ^= 1;
        invalid_setups[1].wIndex = 0;
        invalid_setups[2].wIndex = 0x101;
        invalid_setups[3].wLength -= 1;
        invalid_setups[4].wLength += 1;
        for (const auto& invalid : invalid_setups)
            require(!native_test_setup(slot, &invalid, true),
                    "child wake accepted an invalid value, interface, or length");
        for (unsigned op = 0; op <= UINT8_MAX; ++op) {
            for (bool input : {false, true}) {
                if (op == static_cast<unsigned>(Operation::kSwitch2Wake) ||
                    (input && op == static_cast<unsigned>(Operation::kInfo))) continue;
                const auto invalid = child_request(static_cast<Operation>(op), input,
                                                   input ? kMaximumResponseSize : kRequestHeaderSize);
                require(!native_test_setup(slot, &invalid, true),
                        "child management exposed an operation outside INFO/WAKE");
            }
        }
        read_child(slot);
    }
    require(!native_test_setup(0, &setup, true) &&
            !native_test_setup(0, &info_setup, true),
            "root management incorrectly accepted interface-recipient requests");
    profile_service_task_on_storage_core(4500);
    require(wake_request_calls == expected_calls &&
            programs == programs_before && erases == erases_before,
            "child discovery/rejection dispatched wake or persisted a change");
}

void test_child_management_interleaving() {
    const uint32_t programs_before = programs, erases_before = erases;
    uint32_t expected_calls = wake_request_calls;
    const auto root_setup = request(Operation::kSwitch2Wake, false, kRequestHeaderSize + 4);
    const auto child_setup = child_request(Operation::kSwitch2Wake, false, kRequestHeaderSize + 4);
    require(native_test_setup(0, &root_setup, true), "root pending wake setup failed");
    for (uint8_t slot = 1; slot <= PROBE_CONTROLLER_COUNT; ++slot)
        require(native_test_setup(slot, &child_setup, true), "concurrent child wake setup failed");
    const auto root_bytes = envelope(Operation::kSwitch2Wake, {0x70, 0, 0, 0});
    require(native_test_out(0, root_bytes.data(), root_bytes.size(), true) &&
            wake_request_calls == ++expected_calls && last_wake_request_id == 0x70,
            "child SETUP canceled or overwrote root pending wake");
    acknowledge();
    for (uint8_t slot = PROBE_CONTROLLER_COUNT; slot; --slot) {
        const auto bytes = envelope(Operation::kSwitch2Wake, {slot, 0, 0, 0});
        read_operation(Operation::kInfo);
        require(native_test_out(slot, bytes.data(), bytes.size(), true) &&
                wake_request_calls == ++expected_calls && last_wake_request_id == slot,
                "root or sibling request canceled or overwrote a child's pending wake");
        acknowledge(slot);
    }
    // Distinct snapshots remain stable until each independent IN is consumed.
    wake_status.request_id = 0x80;
    const auto root_read = request(Operation::kSwitch2Wake, true, kMaximumResponseSize);
    const auto child_read = child_request(Operation::kSwitch2Wake, true, kMaximumResponseSize);
    require(native_test_setup(0, &root_read, true), "root snapshot setup failed");
    for (uint8_t slot = 1; slot <= PROBE_CONTROLLER_COUNT; ++slot) {
        wake_status.request_id = slot;
        require(native_test_setup(slot, &child_read, true), "child snapshot setup failed");
    }
    require(u32(receive(), kResponseHeaderSize) == 0x80,
            "child response overwrote pending root IN");
    for (uint8_t slot = PROBE_CONTROLLER_COUNT; slot; --slot)
        require(u32(receive(slot), kResponseHeaderSize) == slot,
                "root or sibling response overwrote pending child IN");
    profile_service_task_on_storage_core(4600);
    require(wake_request_calls == expected_calls &&
            programs == programs_before && erases == erases_before,
            "interleaved volatile management unexpectedly mutated or persisted state");
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

void assign_address(uint8_t slot, uint8_t address) {
    tusb_control_request_t setup{};
    setup.bRequest = TUSB_REQ_SET_ADDRESS;
    setup.wValue = address;
    require(native_test_setup(slot, &setup, true), "SET_ADDRESS stalled");
    acknowledge(slot);
}

void configure(uint8_t slot, uint8_t value = 1) {
    tusb_control_request_t setup{};
    setup.bRequest = TUSB_REQ_SET_CONFIGURATION;
    setup.wValue = value;
    require(native_test_setup(slot, &setup, true), "SET_CONFIGURATION stalled");
    acknowledge(slot);
}

tusb_control_request_t port_feature(uint8_t port, uint16_t feature, bool set) {
    tusb_control_request_t setup{};
    setup.bmRequestType = 0x23;
    setup.bRequest = set ? TUSB_REQ_SET_FEATURE : TUSB_REQ_CLEAR_FEATURE;
    setup.wIndex = port;
    setup.wValue = feature;
    return setup;
}

void change_port(uint8_t port, uint16_t feature, bool set) {
    const auto setup = port_feature(port, feature, set);
    require(native_test_setup(0, &setup, true), "port feature request stalled");
    acknowledge();
}

std::vector<uint8_t> port_status(uint8_t port) {
    tusb_control_request_t setup{};
    setup.bmRequestType = 0xa3;
    setup.bRequest = TUSB_REQ_GET_STATUS;
    setup.wIndex = port;
    setup.wLength = 4;
    require(native_test_setup(0, &setup, true), "port status request stalled");
    return receive();
}

void require_hub_change(uint8_t expected) {
    uint8_t packet[64]; uint16_t length = 0;
    require(native_test_private_in(0, 0x8f, packet, &length) && length == 1 && packet[0] == expected,
            "root interrupt endpoint omitted or mixed port change bits");
    native_test_drain();
    require(!native_test_private_in(0, 0x8f, packet, &length),
            "cleared hub port changes did not return to NAK");
}

void test_port_enumeration_and_bounds() {
    require(native_test_startup(), "native hub startup failed");
    for (uint8_t slot = 1; slot <= PROBE_CONTROLLER_COUNT; ++slot)
        require(!native_test_select(slot), "startup assigned an address to an unreset child");
    assign_address(0, 9);
    configure(0);
    tusb_control_request_t descriptor{};
    descriptor.bmRequestType = 0xa0;
    descriptor.bRequest = TUSB_REQ_GET_DESCRIPTOR;
    descriptor.wValue = 0x2900;
    descriptor.wLength = 64;
    require(native_test_setup(0, &descriptor, true), "hub descriptor stalled");
    const auto bytes = receive();
    require(bytes.size() == 9 && bytes[0] == 9 && bytes[1] == 0x29 &&
            bytes[2] == PROBE_CONTROLLER_COUNT && bytes[7] == (1u << (PROBE_CONTROLLER_COUNT + 1u)) - 2u &&
            bytes[8] == 0xff, "hub descriptor has incorrect port or non-removable masks");
    for (uint8_t port = 1; port <= PROBE_CONTROLLER_COUNT; ++port) {
        require(u16(port_status(port), 0) == 0, "unpowered port is not disconnected");
        change_port(port, 8, true);
        auto status = port_status(port);
        require(u16(status, 0) == 0x101 && u16(status, 2) == 1, "port power did not signal connection");
        change_port(port, 16, false);
        require_hub_change(1u << port);
        change_port(port, 4, true);
        status = port_status(port);
        require(u16(status, 0) == 0x111 && u16(status, 2) == 0, "port reset completed before its deadline");
        native_test_advance(10000);
        status = port_status(port);
        require(u16(status, 0) == 0x103 && u16(status, 2) == 16, "port reset did not enable its child");
        assign_address(port, 17u * port);
        configure(port);
        read_child(port);
        change_port(port, 20, false);
        require_hub_change(1u << port);
        change_port(port, 2, true);
        require(native_hub_suspended(port - 1) && !native_hub_hid_ready(port - 1),
                "suspended port remained ready for input");
        change_port(port, 2, false);
        change_port(port, 18, false);
        require_hub_change(1u << port);
    }
    for (uint8_t slot = 1; slot <= PROBE_CONTROLLER_COUNT; ++slot) read_child(slot);
    for (uint8_t port : {uint8_t{0}, uint8_t{PROBE_CONTROLLER_COUNT + 1}}) {
        auto setup = port_feature(port, 8, true);
        require(!native_test_setup(0, &setup, true), "out-of-range port feature was accepted");
        setup.bmRequestType = 0xa3; setup.bRequest = 0; setup.wValue = 0; setup.wLength = 4;
        require(!native_test_setup(0, &setup, true), "out-of-range port status was accepted");
    }
    const uint8_t data = 1;
    for (uint8_t instance : {uint8_t{PROBE_CONTROLLER_COUNT}, uint8_t{255}}) {
        require(!native_hub_mounted(instance) && native_hub_suspended(instance) &&
                !native_hub_hid_ready(instance) && !native_hub_hid_report(instance, 1, &data, 1) &&
                native_hub_vendor_write_available(instance) == 0 &&
                native_hub_vendor_write(instance, &data, 1) == 0 && native_hub_vendor_write_flush(instance) == 0,
                "out-of-range controller instance touched a bank");
        require(!native_hub_control_xfer(instance + (instance != 255), &descriptor, nullptr, 0, false) &&
                !native_hub_control_status(instance + (instance != 255), &descriptor),
                "out-of-range control slot was accepted");
    }
    configure(0, 0);
    for (uint8_t slot = 1; slot <= PROBE_CONTROLLER_COUNT; ++slot)
        require(!native_hub_mounted(slot - 1) && !native_test_select(slot),
                "root deconfiguration retained a child bank or address");
    native_test_initialize();
}

void test_child_control_and_receive_isolation() {
    native_test_initialize();
    tusb_control_request_t identity{};
    identity.bmRequestType = 0xc0; identity.bRequest = 3; identity.wLength = 128;
    uint8_t packet[64]; uint16_t length;
    for (uint8_t slot = 1; slot <= PROBE_CONTROLLER_COUNT; ++slot) {
        configure(slot);
        const uint8_t payload = 0x70 + slot;
        require(native_hub_hid_report(slot - 1, 8, &payload, 1) &&
                native_test_private_in(slot, 0x81, packet, &length), "HID completion setup failed");
        require(native_test_setup(slot, &identity, false), "interleaved child control setup failed");
    }
    native_test_drain();
    for (uint8_t slot = PROBE_CONTROLLER_COUNT; slot; --slot) {
        const auto bytes = receive(slot);
        require(bytes == std::vector<uint8_t>(child_identity[slot - 1].begin(), child_identity[slot - 1].end()),
                "concurrent control transfers shared another child's EP0 data");
        require(native_test_hid_completions[slot - 1] == 1 && native_hub_hid_ready(slot - 1),
                "new SETUP invalidated an unrelated HID completion");
    }
    for (uint8_t slot = 1; slot <= PROBE_CONTROLLER_COUNT; ++slot) {
        for (uint8_t endpoint : {1, 2}) {
            const uint8_t payload[] = {slot, endpoint, uint8_t(slot ^ 0x5a)};
            require(native_test_private_out(slot, endpoint, payload, sizeof(payload), false),
                    "private OUT packet was not accepted");
            require(!native_test_private_out(slot, endpoint, payload, sizeof(payload), false),
                    "pending OUT buffer failed to NAK before foreground consumption");
        }
    }
    native_test_drain();
    for (uint8_t slot = 1; slot <= PROBE_CONTROLLER_COUNT; ++slot) {
        for (uint8_t endpoint : {1, 2}) {
            const uint8_t payload[] = {slot, endpoint, uint8_t(slot ^ 0x5a)};
            require(native_test_received_count[slot - 1][endpoint - 1] == 1 &&
                    native_test_received_length[slot - 1][endpoint - 1] == sizeof(payload) &&
                    std::memcmp(native_test_received_data[slot - 1][endpoint - 1], payload, sizeof(payload)) == 0,
                    "OUT callback received another child's endpoint payload");
        }
    }
    native_test_initialize();
}

void test_port_reset_revokes_only_its_child_events() {
    for (uint8_t target = 1; target <= PROBE_CONTROLLER_COUNT; ++target) {
        native_test_initialize();
        assign_address(0, 9);
        tusb_control_request_t identity{};
        identity.bmRequestType = 0xc0; identity.bRequest = 3; identity.wLength = 128;
        for (uint8_t slot = 1; slot <= PROBE_CONTROLLER_COUNT; ++slot) {
            change_port(slot, 8, true);
            configure(slot);
            const uint8_t data = slot;
            require(native_hub_hid_report(slot - 1, 8, &data, 1), "reset isolation HID setup failed");
            require(native_test_setup(slot, &identity, true), "reset isolation control setup failed");
        }
        const auto reset = port_feature(target, 4, true);
        require(native_test_setup(0, &reset, true), "port reset request failed");
        acknowledge(0, false);
        uint8_t packet[64]; uint16_t length;
        require(native_test_in(target, packet, &length, false) && length == 64,
                "could not queue the reset child's old control completion");
        for (uint8_t slot = 1; slot <= PROBE_CONTROLLER_COUNT; ++slot) {
            const uint8_t data = slot;
            require(native_test_private_in(slot, 0x81, packet, &length) &&
                    native_test_private_out(slot, 2, &data, 1, false), "reset isolation completion setup failed");
        }
        native_test_drain();
        for (uint8_t slot = 1; slot <= PROBE_CONTROLLER_COUNT; ++slot) {
            const unsigned expected = slot == target ? 0 : 1;
            require(native_test_hid_completions[slot - 1] == expected &&
                    native_test_received_count[slot - 1][1] == expected,
                    "port reset revoked a sibling event or dispatched a stale child event");
            if (slot != target) {
                const auto bytes = receive(slot);
                require(bytes == std::vector<uint8_t>(child_identity[slot - 1].begin(), child_identity[slot - 1].end()),
                        "port reset corrupted a sibling control transfer");
            }
        }
        const uint8_t other = target == PROBE_CONTROLLER_COUNT ? 1 : target + 1;
        const auto concurrent_reset = port_feature(other, 4, true);
        require(!native_test_setup(0, &concurrent_reset, true),
                "simultaneous port resets created competing address-zero owners");
        native_test_advance(10000);
        require(!native_test_in(target, packet, &length, true) && !native_hub_mounted(target - 1),
                "port reset retained a stale control packet or configuration");
        assign_address(target, 17u * target);
        configure(target);
        read_child(target);
    }
    native_test_initialize();
}

void require_interleaved_profile(const std::vector<uint8_t>& expected) {
    const auto setup = request(Operation::kProfileRead, true, kMaximumResponseSize);
    require(native_test_setup(0, &setup, true), "interleaved profile read setup failed");
    std::vector<uint8_t> bytes;
    const size_t total = kResponseHeaderSize + expected.size();
    for (unsigned index = 0; bytes.size() < total; ++index) {
        // Every root IN follows another owner's tokens, including the first.
        read_child(1u + index % PROBE_CONTROLLER_COUNT);
        uint8_t packet[64]; uint16_t length = 0;
        require(native_test_in(0, packet, &length, false),
                "prepared profile packet required foreground work after selection");
        require(length == std::min<size_t>(64, total - bytes.size()),
                "address alternation changed the profile packet boundary");
        bytes.insert(bytes.end(), packet, packet + length);
        native_test_drain(); // Only the completed packet may prepare its successor.
    }
    read_child(PROBE_CONTROLLER_COUNT);
    require(native_test_out(0, nullptr, 0, true), "interleaved profile status OUT failed");
    require(std::memcmp(bytes.data(), "SPMG", 4) == 0 &&
            bytes[5] == static_cast<uint8_t>(Operation::kProfileRead) &&
            bytes[6] == static_cast<uint8_t>(Status::kOk) && u16(bytes, 8) == expected.size() &&
            u32(bytes, 16) == configuration_crc32(expected.data(), expected.size()) &&
            std::vector<uint8_t>(bytes.begin() + kResponseHeaderSize, bytes.end()) == expected,
            "alternating root and child reads mixed profile or identity bytes");
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
    require_interleaved_profile(original);
    require(programs == programs_before && erases == erases_before, "editor reads wrote saved storage");
    for (uint8_t slot = 1; slot <= PROBE_CONTROLLER_COUNT; ++slot) {
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
    for (uint8_t slot = 1; slot <= PROBE_CONTROLLER_COUNT; ++slot) read_child(slot);
    const auto child_management = request(Operation::kInfo, true, kMaximumResponseSize);
    require(!native_test_setup(1, &child_management, true), "child INFO was accepted during a root write");
    if (!SWITCH2_PROBE_NEUTRAL_INPUT) {
        const auto child_info = child_request(Operation::kInfo, true, kMaximumResponseSize);
        require(native_test_setup(1, &child_info, true) && receive(1) == info,
                "child interface discovery failed during multipart root OUT");
    }
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
    for (uint8_t slot = 1; slot <= PROBE_CONTROLLER_COUNT; ++slot) {
        require(native_test_setup(slot, &configuration, true), "child configuration failed");
        acknowledge(slot);
    }
    uint8_t payloads[PROBE_CONTROLLER_COUNT][3];
    for (uint8_t instance = 0; instance < PROBE_CONTROLLER_COUNT; ++instance) {
        payloads[instance][0] = 0x11u + instance;
        payloads[instance][1] = 0x42u + instance;
        payloads[instance][2] = 0x83u + instance;
        require(native_hub_hid_report(instance, 8u - instance, payloads[instance], 3),
                "could not queue HID packet");
        require(native_hub_vendor_write(instance, payloads[instance], 3) == 3 &&
                native_hub_vendor_write_flush(instance) == 3, "could not queue bulk packet");
    }
    uint8_t packet[64];
    uint16_t length = 0;
    for (uint8_t endpoint : {0x81, 0x82}) {
        for (uint8_t slot = 1; slot <= PROBE_CONTROLLER_COUNT; ++slot) {
            require(native_test_private_in(slot, endpoint, packet, &length),
                    "queued private IN packet required foreground work after bank selection");
            const unsigned prefix = endpoint == 0x81 ? 1 : 0;
            require(length == 3 + prefix &&
                    (!prefix || packet[0] == 9u - slot) &&
                    std::memcmp(packet + prefix, payloads[slot - 1], 3) == 0,
                    "round-robin IN token received another endpoint's payload");
            require(!native_test_private_in(slot, endpoint, packet, &length),
                    "unarmed endpoint reused another child's IN packet instead of NAK");
        }
    }
    native_test_drain();
    for (uint8_t instance = 0; instance < PROBE_CONTROLLER_COUNT; ++instance)
        require(native_hub_hid_ready(instance) && native_test_hid_completions[instance] == 1 &&
                native_test_bulk_completions[instance] == 1,
                "acknowledged packets did not release exactly one completion per endpoint");
    require(!native_test_private_in(1, 0x81, packet, &length),
            "acknowledged HID packet was retransmitted");
    // An idle EP0 bank must not block newly queued private endpoint traffic.
    require(native_hub_hid_report(0, 8, payloads[0], 3), "could not queue the next HID packet");
    require(native_test_private_in(1, 0x81, packet, &length) && length == 4 &&
            std::memcmp(packet + 1, payloads[0], 3) == 0,
            "idle shared EP0 blocked a newly queued private IN packet");
    native_test_drain();
    native_test_initialize();
}
void test_masked_irq_completion_handoff() {
    native_test_initialize();
    tusb_control_request_t configuration{};
    configuration.bRequest = TUSB_REQ_SET_CONFIGURATION;
    configuration.wValue = 1;
    for (uint8_t slot = 1; slot <= PROBE_CONTROLLER_COUNT; ++slot) {
        require(native_test_setup(slot, &configuration, true), "child configuration failed");
        acknowledge(slot);
    }
    uint8_t payloads[PROBE_CONTROLLER_COUNT][3];
    for (uint8_t instance = 0; instance < PROBE_CONTROLLER_COUNT; ++instance) {
        payloads[instance][0] = 0x12u + instance;
        payloads[instance][1] = 0x34u + instance;
        payloads[instance][2] = 0x56u + instance;
        require(native_hub_hid_report(instance, 8, payloads[instance], 3),
                "could not queue masked-window HID packet");
    }
    uint8_t packet[64];
    uint16_t length = 0;
    native_test_interrupt_mask = 1;
    for (uint8_t slot = 1; slot <= PROBE_CONTROLLER_COUNT; ++slot) {
        require(native_test_private_in(slot, 0x81, packet, &length) && length == 4 &&
                packet[0] == 8 && std::memcmp(packet + 1, payloads[slot - 1], 3) == 0,
                "controller did not retain its packet during the masked window");
        const uint8_t next = slot == PROBE_CONTROLLER_COUNT ? 1 : slot + 1;
        require(!native_test_select(next),
                "pending completion must prevent overwriting the active bank");
        native_hub_service_pending_usb();
        require(native_test_interrupt_mask == 1,
                "SRAM service must preserve the caller's interrupt mask");
        require(!native_hub_hid_ready(slot - 1) && native_test_hid_completions[slot - 1] == 0,
                "SRAM service must defer protocol callbacks to foreground dispatch");
    }
    native_test_interrupt_mask = 0;
    native_test_drain();
    for (uint8_t instance = 0; instance < PROBE_CONTROLLER_COUNT; ++instance)
        require(native_hub_hid_ready(instance) && native_test_hid_completions[instance] == 1,
                "deferred completions did not release every controller queue exactly once");
    require(!native_test_private_in(1, 0x81, packet, &length),
            "later IRQ dispatch duplicated a serviced completion");
    native_test_initialize();
}


void require_no_bootsel() {
    bootsel_time_ms += 100;
    probe_bootsel_task(bootsel_time_ms);
    probe_bootsel_task(bootsel_time_ms + 50);
    require(bootsel_calls == 0, "unauthorized or unacknowledged BOOTSEL rebooted the device");
}

void test_neutral_management_surface() {
    require(!synthetic_root_management, "neutral surface must use the production BOOTSEL-only callback");
    const uint32_t programs_before = programs, erases_before = erases;
    struct WriteRequest { Operation operation; uint16_t payload_size; };
    const WriteRequest writes[] = {
        {Operation::kModeSet, 5}, {Operation::kReboot, 4},
        {Operation::kConfigurationBegin, 12}, {Operation::kConfigurationChunk, 9},
        {Operation::kConfigurationCommit, 4}, {Operation::kConfigurationReset, 4},
        {Operation::kProfileSelect, 15}, {Operation::kProfileBegin, 28},
        {Operation::kProfileChunk, 9}, {Operation::kProfileCommit, 4},
        {Operation::kProfileReset, 19}, {Operation::kProfileActivate, 19},
        {Operation::kProfileMetadataSet, 20}, {Operation::kProfileIdentify, 14},
        {Operation::kWiiOrientation, 19}, {Operation::kPairingRefresh, 0},
        {Operation::kPairingClear, 0},
    };
    const uint32_t wake_calls_before = wake_request_calls;
    for (uint8_t slot = 0; slot <= PROBE_CONTROLLER_COUNT; ++slot) {
        for (Operation op : {Operation::kInfo, Operation::kConfigurationRead,
                Operation::kTransactionStatus, Operation::kPairingRead,
                Operation::kRuntimeDiagnostics, Operation::kProfileList,
                Operation::kProfileRead, Operation::kProfilePlaytest,
                Operation::kProfileTransactionStatus, Operation::kProfileMetadataRead}) {
            const auto setup = request(op, true, kMaximumResponseSize);
            require(!native_test_setup(slot, &setup, true), "neutral device exposed full management reads");
        }
        for (const auto& item : writes) {
            const auto setup = request(item.operation, false, kRequestHeaderSize + item.payload_size);
            require(!native_test_setup(slot, &setup, true), "neutral device exposed a management mutation");
        }
        for (bool input : {false, true}) {
            const auto wake = request(Operation::kSwitch2Wake, input,
                                      input ? kMaximumResponseSize : kRequestHeaderSize + 4);
            const auto child_wake = child_request(Operation::kSwitch2Wake, input, wake.wLength);
            require(!native_test_setup(slot, &wake, true) &&
                    !native_test_setup(slot, &child_wake, true),
                    "neutral firmware exposed a wake management route");
        }
        const auto child_info = child_request(Operation::kInfo, true, kMaximumResponseSize);
        require(!native_test_setup(slot, &child_info, true),
                "neutral firmware exposed child management discovery");
        if (slot) read_child(slot);
    }
    profile_service_task_on_storage_core(5000);
    require(programs == programs_before && erases == erases_before &&
            wake_request_calls == wake_calls_before,
            "neutral management rejection changed saved profiles");
    require_no_bootsel();
}

void test_private_bootsel(uint8_t reboot_slot) {
    require(!synthetic_root_management, "BOOTSEL must use the production transport callback");
    const uint32_t programs_before = programs, erases_before = erases;
    const auto bytes = envelope(Operation::kBootselReboot, {});
    const auto setup = request(Operation::kBootselReboot, false, bytes.size());
    tusb_control_request_t replacement{};
    replacement.bmRequestType = 0x80;
    replacement.bRequest = TUSB_REQ_GET_STATUS;
    replacement.wLength = 2;
    uint8_t packet[64]; uint16_t length;
    for (uint8_t slot = 0; slot <= PROBE_CONTROLLER_COUNT; ++slot) {
        for (uint16_t size : {uint16_t{0}, uint16_t{kRequestHeaderSize - 1}}) {
            require(native_test_setup(slot, &setup, true), "private BOOTSEL setup stalled");
            require(!native_test_in(slot, packet, &length, true), "BOOTSEL armed status before receiving its envelope");
            require(!native_test_out(slot, bytes.data(), size, true), "short BOOTSEL was accepted");
            require(!native_test_in(slot, packet, &length, true), "short BOOTSEL armed a status ACK");
            require_no_bootsel();
        }
        // Every reserved field and CRC byte must be checked by the shared decoder.
        for (size_t offset : {0, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}) {
            auto malformed = bytes;
            malformed[offset] ^= 1;
            require(native_test_setup(slot, &setup, true) &&
                    native_test_out(slot, bytes.data(), bytes.size(), true), "superseded BOOTSEL setup failed");
            require(native_test_setup(slot, &setup, true), "malformed BOOTSEL setup stalled");
            require(!native_test_out(slot, malformed.data(), malformed.size(), true), "malformed BOOTSEL was accepted");
            require(!native_test_in(slot, packet, &length, true), "malformed BOOTSEL reused an earlier authorization");
            require_no_bootsel();
        }
        std::array<tusb_control_request_t, 8> wrong_setup;
        wrong_setup.fill(setup);
        wrong_setup[0].bmRequestType = 0x41; // Interface recipient.
        wrong_setup[1].bmRequestType = 0x42; // Endpoint recipient.
        wrong_setup[2].bmRequestType = 0xc0; // Wrong direction.
        wrong_setup[3].wValue ^= 1;
        wrong_setup[4].wIndex ^= 1;
        wrong_setup[5].wLength = 0;
        wrong_setup[6].wLength = kRequestHeaderSize - 1;
        wrong_setup[7].wLength = kRequestHeaderSize + 1;
        for (const auto& invalid : wrong_setup) {
            require(native_test_setup(slot, &setup, true) &&
                    native_test_out(slot, bytes.data(), bytes.size(), true), "interrupted BOOTSEL setup failed");
            require(!native_test_setup(slot, &invalid, true), "wrong BOOTSEL setup was accepted");
            require(!native_test_in(slot, packet, &length, true) &&
                    !native_test_out(slot, bytes.data(), bytes.size(), true), "rejected SETUP retained an old BOOTSEL transfer");
            require_no_bootsel();
        }
        // Standard requests do not call the vendor handler: transport ownership
        // must still revoke both incomplete DATA and unacknowledged status.
        for (bool send_data : {false, true}) {
            require(native_test_setup(slot, &setup, true), "interruptible BOOTSEL setup stalled");
            if (send_data)
                require(native_test_out(slot, bytes.data(), bytes.size(), true), "interruptible BOOTSEL DATA failed");
            require(native_test_setup(slot, &replacement, true), "replacement standard request stalled");
            require(receive(slot).size() == 2, "replacement standard transfer did not complete");
            require(!native_test_in(slot, packet, &length, true) &&
                    !native_test_out(slot, bytes.data(), bytes.size(), true), "superseded BOOTSEL retained a transfer");
            require_no_bootsel();
        }
        // Reset revokes queued DATA, validated DATA, and even a captured status
        // ACK that has not reached the foreground callback yet.
        for (unsigned phase : {0, 1, 2}) {
            require(native_test_setup(slot, &setup, true) &&
                    native_test_out(slot, bytes.data(), bytes.size(), phase != 0), "resettable BOOTSEL setup failed");
            if (phase == 2) acknowledge(slot, false);
            native_test_bus_reset(true);
            require(!native_test_in(slot, packet, &length, true), "bus reset retained BOOTSEL status");
            require_no_bootsel();
        }
    }
    // Concurrent children must not share the valid envelope or authorization.
    for (uint8_t slot : {uint8_t{1}, uint8_t{PROBE_CONTROLLER_COUNT}})
        require(native_test_setup(slot, &setup, true), "concurrent BOOTSEL setup failed");
    require(native_test_out(1, bytes.data(), bytes.size(), true), "first child's BOOTSEL DATA failed");
    auto corrupt = bytes;
    corrupt[12] ^= 1;
    require(!native_test_out(PROBE_CONTROLLER_COUNT, corrupt.data(), corrupt.size(), true),
            "last child inherited its sibling's BOOTSEL authorization");
    native_test_bus_reset(true);
    require_no_bootsel();

    if (reboot_slot == 0) {
        require(native_test_startup(), "root-only BOOTSEL startup failed");
        for (uint8_t slot = 1; slot <= PROBE_CONTROLLER_COUNT; ++slot)
            require(!native_test_select(slot), "root-only recovery unexpectedly requires an enumerated child");
    } else {
        native_test_initialize();
    }
    require(native_test_setup(reboot_slot, &setup, true), "valid BOOTSEL setup failed");
    require_no_bootsel();
    require(native_test_out(reboot_slot, bytes.data(), bytes.size(), true), "valid BOOTSEL DATA failed");
    require_no_bootsel();
    acknowledge(reboot_slot, false);
    require_no_bootsel();
    // Unlike reset, the next SETUP preserves a genuine, already-captured ACK.
    require(native_test_setup(reboot_slot, &replacement, false), "post-ACK SETUP failed");
    native_test_drain();
    require(receive(reboot_slot).size() == 2, "post-ACK standard transfer failed");
    const uint32_t now = bootsel_time_ms + 100;
    probe_bootsel_task(now); probe_bootsel_task(now + 49);
    require(bootsel_calls == 0, "BOOTSEL did not retain the post-ACK 50ms delay");
    probe_bootsel_task(now + 50);
    require(bootsel_calls == 1, "validated BOOTSEL did not reach ROM after the delay");
    probe_bootsel_task(now + 100);
    require(bootsel_calls == 1, "BOOTSEL dispatched more than once");
    profile_service_task_on_storage_core(now + 100);
    require(programs == programs_before && erases == erases_before,
            "private BOOTSEL changed saved profiles");
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
bool bluepad32_input_backend_request_switch2_wake(uint32_t request_id) {
    ++wake_request_calls;
    last_wake_request_id = request_id;
    return accept_wake_request;
}
void bluepad32_input_backend_switch2_wake_snapshot(Bluepad32Switch2WakeStatus* out) {
    *out = wake_status;
}
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
    // Exercise the full root service over a synthetic four-child transport
    // without claiming that the neutral firmware exposes that service.
    if (synthetic_root_management && slot == 0 &&
        usb_configuration_management_vendor_control(slot, stage, setup)) return true;
    if (slot < 1 || slot > PROBE_CONTROLLER_COUNT || setup->bmRequestType != 0xc0 ||
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
            child_identity[slot - 1].data(), child_identity[slot - 1].size(), true);
}

int main(int argc, char** argv) {
    static_assert(sizeof(tusb_control_request_t) == 8);
    require(argc == 2 && (std::strcmp(argv[1], "root") == 0 || std::strcmp(argv[1], "child") == 0),
            "select the root or last-child BOOTSEL completion scenario");
    const uint8_t reboot_slot = std::strcmp(argv[1], "root") == 0 ? 0 : PROBE_CONTROLLER_COUNT;
    flash.fill(0xff);
    for (unsigned instance = 0; instance < PROBE_CONTROLLER_COUNT; ++instance)
        child_identity[instance].fill(0x31u + instance * 0x21u);
    profile_service_prepare(); profile_service_initialize_on_storage_core();
    native_test_initialize();
    synthetic_root_management = SWITCH2_PROBE_NEUTRAL_INPUT;
    test_profile_transport();
    test_interrupted_transactions();
    test_pending_control_buffer_ownership();
    test_wake_transport();
    synthetic_root_management = false;
    if (!SWITCH2_PROBE_NEUTRAL_INPUT) {
        test_child_wake_transport();
        test_child_management_interleaving();
    }
    test_read_ack_allows_usb_progress();
    test_private_transmit_survives_round_robin_tokens();
    test_masked_irq_completion_handoff();
    test_port_enumeration_and_bounds();
    test_child_control_and_receive_isolation();
    test_port_reset_revokes_only_its_child_events();
    if (SWITCH2_PROBE_NEUTRAL_INPUT) test_neutral_management_surface();
    test_private_bootsel(reboot_slot);
    std::cout << "native transport, synthetic root management and private BOOTSEL regressions passed\n";
}
