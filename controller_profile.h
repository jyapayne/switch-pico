#pragma once

#include <stddef.h>
#include <stdint.h>

#include "controller_identity.h"

constexpr uint16_t CONTROLLER_PROFILE_LEGACY_SCHEMA_VERSION = 1;
constexpr uint16_t CONTROLLER_PROFILE_SCHEMA_VERSION = 2;
constexpr size_t CONTROLLER_PROFILE_ENCODED_SIZE = 256;
constexpr uint8_t CONTROLLER_PROFILE_COUNT = 4;
constexpr uint8_t CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT = 16;
constexpr uint8_t CONTROLLER_PROFILE_MACRO_STEP_CAPACITY = 8;
constexpr uint16_t CONTROLLER_PROFILE_MAX_WAIT_MS = 10000;
constexpr uint8_t CONTROLLER_PROFILE_NO_BUTTON = 0xff;
constexpr uint8_t CONTROLLER_PROFILE_ALL = 0xff;
// Exact 16-bit counterpart of the existing 358-of-1023 Switch boundary.
constexpr uint16_t CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD = 22934;

constexpr uint8_t CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY = 16;
constexpr uint16_t CONTROLLER_PROFILE_DATABASE_LEGACY_SCHEMA_VERSION = 1;
constexpr uint16_t CONTROLLER_PROFILE_DATABASE_SCHEMA_VERSION = 2;
constexpr size_t CONTROLLER_PROFILE_DATABASE_HEADER_SIZE = 32;
constexpr size_t CONTROLLER_PROFILE_DATABASE_ENTRY_HEADER_SIZE = 16;
constexpr size_t CONTROLLER_PROFILE_DATABASE_ENTRY_SIZE =
    CONTROLLER_PROFILE_DATABASE_ENTRY_HEADER_SIZE +
    CONTROLLER_PROFILE_COUNT * CONTROLLER_PROFILE_ENCODED_SIZE;
constexpr size_t CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE =
    CONTROLLER_PROFILE_DATABASE_HEADER_SIZE +
    CONTROLLER_PROFILE_COUNT * CONTROLLER_PROFILE_ENCODED_SIZE +
    CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY *
        CONTROLLER_PROFILE_DATABASE_ENTRY_SIZE;

static_assert(CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE == 17696,
              "profile database wire size changed");
enum class ControllerProfileLogicalButton : uint8_t {
    kSouth = 0,
    kEast = 1,
    kWest = 2,
    kNorth = 3,
    kLeftShoulder = 4,
    kRightShoulder = 5,
    kSelect = 6,
    kStart = 7,
    kSystem = 8,
    kCapture = 9,
    kLeftStick = 10,
    kRightStick = 11,
    kDpadUp = 12,
    kDpadDown = 13,
    kDpadLeft = 14,
    kDpadRight = 15,
};
static_assert(
    static_cast<uint8_t>(ControllerProfileLogicalButton::kDpadRight) + 1 ==
    CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT);


enum class ControllerProfileConfirmationPolicy : uint8_t {
    kNone = 0,
    kRumble = 1,
    kLed = 2,
    kRumbleAndLed = 3,
};

enum class ControllerProfileTurboMode : uint8_t {
    kOff = 0,
    kTurbo = 1,
    kAutoBurst = 2,
};

enum class ControllerProfileMacroStepType : uint8_t {
    kState = 0,
    kEnd = 1,
};

enum ControllerProfileMacroOverride : uint8_t {
    kControllerProfileOverrideButtons = 1u << 0,
    kControllerProfileOverrideLeftStick = 1u << 1,
    kControllerProfileOverrideRightStick = 1u << 2,
    kControllerProfileOverrideLeftTrigger = 1u << 3,
    kControllerProfileOverrideRightTrigger = 1u << 4,
};

struct ControllerProfileStickConfiguration {
    int16_t center_x = 0;
    int16_t center_y = 0;
    uint16_t inner_deadzone = 0;
    uint16_t outer_saturation = 32767;
    uint16_t curve_q8_8 = 256;
    bool invert_x = false;
    bool invert_y = false;
};

struct ControllerProfileTriggerConfiguration {
    uint16_t lower_deadzone = 0;
    uint16_t upper_saturation = UINT16_MAX;
    uint16_t curve_q8_8 = 256;
    // Compared against the transformed uint16 trigger output.
    uint16_t digital_threshold =
        CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD;
};

struct ControllerProfileMacroStep {
    ControllerProfileMacroStepType type =
        ControllerProfileMacroStepType::kEnd;
    uint8_t override_flags = 0;
    uint16_t duration_ms = 0;
    uint16_t output_button_mask = 0;
    int16_t left_stick_x = 0;
    int16_t left_stick_y = 0;
    int16_t right_stick_x = 0;
    int16_t right_stick_y = 0;
    uint16_t left_trigger = 0;
    uint16_t right_trigger = 0;
};

struct ControllerProfile {
    uint8_t button_map[CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT]{};
    ControllerProfileStickConfiguration sticks[2]{};
    ControllerProfileTriggerConfiguration triggers[2]{};
    uint8_t weak_rumble_scale = UINT8_MAX;
    uint8_t strong_rumble_scale = UINT8_MAX;
    ControllerProfileConfirmationPolicy confirmation_policy =
        ControllerProfileConfirmationPolicy::kRumbleAndLed;
    uint16_t switching_chord = 0;
    uint8_t macro_trigger = CONTROLLER_PROFILE_NO_BUTTON;
    uint8_t macro_cancel = CONTROLLER_PROFILE_NO_BUTTON;
    uint8_t macro_step_count = 1;
    ControllerProfileTurboMode
        turbo_modes[CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT]{};
    ControllerProfileMacroStep
        macro_steps[CONTROLLER_PROFILE_MACRO_STEP_CAPACITY]{};
};

struct ControllerProfileDatabaseEntry {
    bool used = false;
    ControllerIdentity identity{};
    uint8_t active_profile = 0;
    ControllerProfile profiles[CONTROLLER_PROFILE_COUNT]{};
};

struct ControllerProfileDatabase {
    uint8_t fallback_active_profile = 0;
    ControllerProfile fallback_profiles[CONTROLLER_PROFILE_COUNT]{};
    ControllerProfileDatabaseEntry
        entries[CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY]{};
};

enum class ControllerProfileDatabaseResult : uint8_t {
    kOk = 0,
    kInvalidArgument = 1,
    kFull = 2,
};

using ControllerProfileDatabaseRead = bool (*)(
    void* context, size_t offset, uint8_t* output, size_t size);

ControllerProfile controller_profile_default(const ControllerIdentity& identity,
                                             uint8_t profile_index);
bool controller_profile_validate(const ControllerProfile& profile);
bool controller_profile_encode(const ControllerProfile& profile,
                               uint8_t* output, size_t output_size);
bool controller_profile_decode(const uint8_t* input, size_t input_size,
                               ControllerProfile* output);

void controller_profile_database_default(ControllerProfileDatabase* database);
bool controller_profile_database_validate(
    const ControllerProfileDatabase& database);
bool controller_profile_database_encode_range(
    const ControllerProfileDatabase& database, size_t offset,
    uint8_t* output, size_t size);
bool controller_profile_database_decode(
    ControllerProfileDatabaseRead read, void* context,
    ControllerProfileDatabase* output);

const ControllerProfileDatabaseEntry* controller_profile_database_find(
    const ControllerProfileDatabase& database,
    const ControllerIdentity& identity);
ControllerProfileDatabaseEntry* controller_profile_database_find(
    ControllerProfileDatabase* database,
    const ControllerIdentity& identity);
ControllerProfileDatabaseResult controller_profile_database_ensure(
    ControllerProfileDatabase* database, const ControllerIdentity& identity,
    ControllerProfileDatabaseEntry** output);
const ControllerProfile* controller_profile_database_get(
    const ControllerProfileDatabase& database,
    const ControllerIdentity& identity, uint8_t profile_index);
ControllerProfileDatabaseResult controller_profile_database_set(
    ControllerProfileDatabase* database, const ControllerIdentity& identity,
    uint8_t profile_index, const ControllerProfile& profile);
ControllerProfileDatabaseResult controller_profile_database_reset(
    ControllerProfileDatabase* database, const ControllerIdentity& identity,
    uint8_t profile_index);
ControllerProfileDatabaseResult controller_profile_database_activate(
    ControllerProfileDatabase* database, const ControllerIdentity& identity,
    uint8_t profile_index);
