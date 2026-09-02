#pragma once

#include <stdint.h>

#include "controller_color.h"
#include "controller_state.h"
#include "switch_haptics.h"

constexpr uint8_t BLUEPAD32_INPUT_BACKEND_SLOT_COUNT = 4;
constexpr uint8_t BLUEPAD32_PAIRING_RECORD_CAPACITY = 16;

enum class Bluepad32PairingTransport : uint8_t {
    kClassic = 1,
    kBle = 2,
};

enum class Bluepad32PairingSnapshotStatus : uint8_t {
    kReady = 0,
    kPending = 1,
};

struct Bluepad32PairingRecord {
    Bluepad32PairingTransport transport;
    uint8_t address_type;
    uint8_t address[6];
};

struct Bluepad32PairingSnapshot {
    uint32_t generation;
    Bluepad32PairingSnapshotStatus status;
    uint8_t record_count;
    bool overflow;
    Bluepad32PairingRecord records[BLUEPAD32_PAIRING_RECORD_CAPACITY];
};


void bluepad32_input_backend_init();
void bluepad32_input_backend_start();
void bluepad32_input_backend_open_pairing_window();
void bluepad32_input_backend_clear_pairings();
bool bluepad32_input_backend_snapshot(uint8_t slot, ControllerState* out);
void bluepad32_input_backend_request_pairing_snapshot();
void bluepad32_input_backend_pairing_snapshot(
    Bluepad32PairingSnapshot* out);
void bluepad32_input_backend_report_sent(uint8_t slot);
void bluepad32_input_backend_queue_rumble(
    uint8_t slot, const ControllerRumbleOutput& rumble);
