#include "core/controller_identity.h"
#include "profile/controller_profile.h"
#include "controller_profile_legacy_fixtures.h"

#include <cstdlib>
#include <cstring>
#include <iostream>

namespace {

uint8_t encoded_database[CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE]{};
ControllerProfileDatabase database{};
ControllerProfileDatabase decoded_database{};

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

ControllerIdentity identity(uint8_t suffix) {
    ControllerIdentity value{};
    value.stable = true;
    value.transport = ControllerTransport::kClassic;
    value.address[5] = suffix;
    value.vendor_id = 0x057e;
    value.product_id = static_cast<uint16_t>(0x2000u + suffix);
    return value;
}

bool read_encoded_database(void*, size_t offset, uint8_t* output,
                           size_t size) {
    if (offset > sizeof(encoded_database) ||
        size > sizeof(encoded_database) - offset) {
        return false;
    }
    memcpy(output, &encoded_database[offset], size);
    return true;
}

void test_profile_wire_schema() {
    const ControllerProfile profile =
        controller_profile_default(controller_identity_global(), 0);
    uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    require(controller_profile_encode(profile, encoded, sizeof(encoded)),
            "default profile did not encode");
    require(encoded[0] == 4 && encoded[1] == 0 &&
                encoded[2] == 0 && encoded[3] == 1,
            "profile header is not little-endian v4/256");
    for (uint8_t index = 0;
         index < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT; ++index) {
        require(encoded[4 + index] == index,
                "default direct mapping is not identity");
    }
    require(encoded[26] == 0xff && encoded[27] == 0x7f &&
                encoded[30] == 0,
            "default stick encoding changed");
    require(encoded[54] == 0xff && encoded[55] == 0xff &&
                encoded[58] ==
                    static_cast<uint8_t>(
                        CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD) &&
                encoded[59] ==
                    static_cast<uint8_t>(
                        CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD >> 8),
            "default trigger encoding changed");
    require(encoded[60] == CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL &&
                encoded[61] == 0 &&
                encoded[70] == CONTROLLER_PROFILE_RIGHT_TRIGGER_CONTROL &&
                encoded[71] == 0,
            "default trigger mappings are not identity");
    require(encoded[72] == 0xff && encoded[73] == 0xff &&
                encoded[74] == 3 && encoded[78] == 0 &&
                encoded[79] == 0 && encoded[80] == 1 &&
                encoded[81] == CONTROLLER_PROFILE_NO_BUTTON,
            "default rumble or macro encoding changed");
    for (uint8_t index = 0;
         index < CONTROLLER_PROFILE_MACRO_STEP_CAPACITY; ++index) {
        require(encoded[100 + index * 19] == 1,
                "unused macro step is not canonical end");
    }

    ControllerProfile decoded{};
    require(controller_profile_decode(encoded, sizeof(encoded), &decoded),
            "default profile did not decode");
    ControllerProfile action_profile = profile;
    action_profile.switching_chord =
        (1u << CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL) |
        (1u << static_cast<uint8_t>(
            ControllerProfileLogicalButton::kSouth));
    action_profile.macro_trigger_mask =
        (1u << CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL) |
        (1u << CONTROLLER_PROFILE_RIGHT_TRIGGER_CONTROL);
    action_profile.motion_toggle_chord =
        (1u << CONTROLLER_PROFILE_RIGHT_TRIGGER_CONTROL);
    action_profile.macro_cancel =
        CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL;
    uint8_t action_encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    require(controller_profile_encode(
                action_profile, action_encoded, sizeof(action_encoded)) &&
                action_encoded[75] == 0x2d &&
                action_encoded[81] ==
                    CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL &&
                controller_profile_decode(
                    action_encoded, sizeof(action_encoded), &decoded) &&
                decoded.switching_chord ==
                    action_profile.switching_chord &&
                decoded.motion_toggle_chord ==
                    action_profile.motion_toggle_chord &&
                decoded.macro_trigger_mask ==
                    action_profile.macro_trigger_mask &&
                decoded.macro_cancel == action_profile.macro_cancel,
            "trigger-backed action masks did not round-trip");
    encoded[252] = 1;
    require(!controller_profile_decode(encoded, sizeof(encoded), &decoded),
            "nonzero reserved profile byte was accepted");

    ControllerProfile invalid = profile;
    invalid.button_map[0] = CONTROLLER_PROFILE_LOGICAL_CONTROL_COUNT;
    require(!controller_profile_validate(invalid),
            "invalid direct output was accepted");
    invalid = profile;
    invalid.sticks[0].inner_deadzone =
        invalid.sticks[0].outer_saturation;
    require(!controller_profile_validate(invalid),
            "empty stick range was accepted");
    ControllerProfile boundary = profile;
    boundary.triggers[0].lower_deadzone = 30000;
    boundary.triggers[0].upper_saturation = 40000;
    boundary.triggers[0].digital_threshold = 0;
    uint8_t boundary_encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    require(controller_profile_encode(boundary, boundary_encoded,
                                      sizeof(boundary_encoded)) &&
                controller_profile_decode(boundary_encoded,
                                          sizeof(boundary_encoded),
                                          &decoded) &&
                decoded.triggers[0].digital_threshold == 0,
            "current profile rejected zero transformed trigger threshold");
    boundary.triggers[0].digital_threshold = UINT16_MAX;
    require(controller_profile_encode(boundary, boundary_encoded,
                                      sizeof(boundary_encoded)) &&
                controller_profile_decode(boundary_encoded,
                                          sizeof(boundary_encoded),
                                          &decoded) &&
                decoded.triggers[0].digital_threshold == UINT16_MAX,
            "current profile rejected maximum transformed trigger threshold");
    invalid = profile;
    invalid.triggers[0].lower_deadzone =
        invalid.triggers[0].upper_saturation;
    require(!controller_profile_validate(invalid),
            "empty raw trigger range was accepted");
    invalid.triggers[0].lower_deadzone = UINT16_MAX;
    invalid.triggers[0].upper_saturation = UINT16_MAX - 1;
    require(!controller_profile_validate(invalid),
            "reversed raw trigger range was accepted");
    invalid = profile;
    invalid.turbo_modes[0] =
        static_cast<ControllerProfileTurboMode>(3);
    require(!controller_profile_validate(invalid),
            "invalid Turbo mode was accepted");
    invalid = profile;
    invalid.macro_step_count = 2;
    invalid.macro_steps[0].type =
        ControllerProfileMacroStepType::kState;
    invalid.macro_steps[0].duration_ms =
        CONTROLLER_PROFILE_MAX_WAIT_MS + 1;
    require(!controller_profile_validate(invalid),
            "unbounded macro wait was accepted");
    invalid.macro_steps[0].duration_ms =
        CONTROLLER_PROFILE_MAX_WAIT_MS;
    invalid.macro_steps[1].type =
        ControllerProfileMacroStepType::kState;
    require(!controller_profile_validate(invalid),
            "macro without a final end was accepted");
}

void test_legacy_profile_migration() {
    ControllerProfile migrated{};
    require(controller_profile_decode(
                kLegacyDefaultProfile, sizeof(kLegacyDefaultProfile),
                &migrated),
            "legacy default profile did not decode");
    require(migrated.triggers[0].digital_threshold ==
                    CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD &&
                migrated.triggers[1].digital_threshold ==
                    CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD,
            "legacy inherited thresholds were not migrated");

    uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    require(controller_profile_encode(migrated, encoded, sizeof(encoded)),
            "migrated default profile did not encode");
    for (size_t index = 0; index < sizeof(encoded); ++index) {
        const bool migration_byte =
            index == 0 ||
            (index >= 58 && index < 60) ||
            index == 60 ||
            (index >= 68 && index < 70) ||
            index == 70 ||
            index == 78 || index == 79 || index == 81;
        if (!migration_byte) {
            require(encoded[index] == kLegacyDefaultProfile[index],
                    "legacy default profile changed an unrelated byte");
        }
    }
    require(encoded[0] == CONTROLLER_PROFILE_SCHEMA_VERSION &&
                encoded[58] ==
                    static_cast<uint8_t>(
                        CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD) &&
                encoded[59] ==
                    static_cast<uint8_t>(
                        CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD >> 8) &&
                encoded[68] ==
                    static_cast<uint8_t>(
                        CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD) &&
                encoded[69] ==
                    static_cast<uint8_t>(
                        CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD >> 8) &&
                encoded[60] == CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL &&
                encoded[70] == CONTROLLER_PROFILE_RIGHT_TRIGGER_CONTROL &&
                encoded[78] == 0 && encoded[79] == 0 &&
                encoded[81] == CONTROLLER_PROFILE_NO_BUTTON,
            "migrated default profile did not encode as v4");

    require(controller_profile_decode(
                kLegacyNarrowRawRangeProfile,
                sizeof(kLegacyNarrowRawRangeProfile), &migrated),
            "legacy narrow-raw-range profile did not decode");
    require(migrated.triggers[0].lower_deadzone == 30000 &&
                migrated.triggers[0].upper_saturation == 40000 &&
                migrated.triggers[0].digital_threshold ==
                    CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD &&
                migrated.triggers[1].lower_deadzone == 30000 &&
                migrated.triggers[1].upper_saturation == 40000 &&
                migrated.triggers[1].digital_threshold ==
                    CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD,
            "legacy narrow raw range or inherited threshold was not migrated");
    require(controller_profile_encode(migrated, encoded, sizeof(encoded)),
            "migrated narrow-raw-range profile did not encode");
    for (size_t index = 0; index < sizeof(encoded); ++index) {
        const bool migration_byte =
            index == 0 ||
            (index >= 58 && index < 60) ||
            index == 60 ||
            (index >= 68 && index < 70) ||
            index == 70 ||
            index == 78 || index == 79 || index == 81;
        if (!migration_byte) {
            require(
                encoded[index] == kLegacyNarrowRawRangeProfile[index],
                "narrow-raw-range migration changed unrelated profile data");
        }
    }

    require(controller_profile_decode(
                kLegacyCustomThresholdProfile,
                sizeof(kLegacyCustomThresholdProfile), &migrated),
            "legacy custom-threshold profile did not decode");
    require(migrated.triggers[0].digital_threshold == 0x1234 &&
                migrated.triggers[1].digital_threshold == 0xabcd,
            "legacy custom thresholds were not preserved");
    require(migrated.triggers[0].output ==
                    CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL &&
                migrated.triggers[1].output ==
                    CONTROLLER_PROFILE_RIGHT_TRIGGER_CONTROL,
            "legacy trigger identity mappings were not restored");
    require(controller_profile_encode(migrated, encoded, sizeof(encoded)),
            "legacy custom-threshold profile did not re-encode");
    const uint8_t legacy_macro_trigger =
        kLegacyCustomThresholdProfile[78];
    const uint16_t expected_macro_trigger =
        legacy_macro_trigger < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT
            ? static_cast<uint16_t>(1u << legacy_macro_trigger)
            : 0;
    require(migrated.macro_trigger_mask == expected_macro_trigger &&
                migrated.macro_cancel ==
                    kLegacyCustomThresholdProfile[79],
            "legacy macro trigger was not migrated to a chord");
    for (size_t index = 1; index < sizeof(encoded); ++index) {
        const bool migration_byte =
            index == 60 || index == 70 ||
            index == 78 || index == 79 || index == 81;
        if (!migration_byte) {
            require(encoded[index] ==
                        kLegacyCustomThresholdProfile[index],
                    "legacy custom-threshold profile changed data");
        }
    }

    const ControllerProfile previous_source =
        controller_profile_default(controller_identity_global(), 0);
    uint8_t previous_encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    require(controller_profile_encode(
                previous_source, previous_encoded,
                sizeof(previous_encoded)),
            "previous-schema fixture did not encode");
    previous_encoded[0] = static_cast<uint8_t>(
        CONTROLLER_PROFILE_TRIGGER_THRESHOLD_SCHEMA_VERSION);
    previous_encoded[60] = 0;
    previous_encoded[70] = 0;
    previous_encoded[78] = static_cast<uint8_t>(
        ControllerProfileLogicalButton::kSouth);
    previous_encoded[79] = static_cast<uint8_t>(
        ControllerProfileLogicalButton::kCapture);
    previous_encoded[81] = 0;
    previous_encoded[98] = 0;
    previous_encoded[99] = 0;
    require(controller_profile_decode(
                previous_encoded, sizeof(previous_encoded), &migrated) &&
                migrated.triggers[0].output ==
                    CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL &&
                migrated.triggers[1].output ==
                    CONTROLLER_PROFILE_RIGHT_TRIGGER_CONTROL &&
                migrated.macro_trigger_mask ==
                    static_cast<uint16_t>(
                        1u << static_cast<uint8_t>(
                            ControllerProfileLogicalButton::kSouth)) &&
                migrated.macro_cancel ==
                    static_cast<uint8_t>(
                        ControllerProfileLogicalButton::kCapture) &&
                migrated.motion_toggle_chord == 0,
            "v2 profile controls did not migrate to v4");

    ControllerProfile current =
        controller_profile_default(controller_identity_global(), 0);
    current.triggers[0].digital_threshold = 0x8000;
    require(controller_profile_encode(current, encoded, sizeof(encoded)) &&
                controller_profile_decode(encoded, sizeof(encoded),
                                          &migrated) &&
                migrated.triggers[0].digital_threshold == 0x8000,
            "v2 custom threshold matching the legacy default was migrated");
}

void test_database_round_trip_and_capacity() {
    controller_profile_database_default(&database);
    for (uint8_t index = 0;
         index < CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY; ++index) {
        ControllerProfileDatabaseEntry* entry = nullptr;
        require(controller_profile_database_ensure(
                    &database, identity(static_cast<uint8_t>(index + 1)),
                    &entry) == ControllerProfileDatabaseResult::kOk &&
                    entry != nullptr,
                "stable identity was not added");
        entry->active_profile = index % CONTROLLER_PROFILE_COUNT;
    }
    ControllerProfileDatabaseEntry* rejected = nullptr;
    require(controller_profile_database_ensure(
                &database, identity(99), &rejected) ==
                ControllerProfileDatabaseResult::kFull,
            "seventeenth stable identity was not rejected");
    require(controller_profile_database_find(database, identity(1)) !=
                nullptr,
            "full-table rejection evicted an existing identity");

    for (size_t offset = 0; offset < sizeof(encoded_database);
         offset += CONTROLLER_PROFILE_ENCODED_SIZE) {
        const size_t size = sizeof(encoded_database) - offset <
                                    CONTROLLER_PROFILE_ENCODED_SIZE
                                ? sizeof(encoded_database) - offset
                                : CONTROLLER_PROFILE_ENCODED_SIZE;
        require(controller_profile_database_encode_range(
                    database, offset, &encoded_database[offset], size),
                "database range did not encode");
    }
    require(encoded_database[4] ==
                    CONTROLLER_PROFILE_DATABASE_SCHEMA_VERSION &&
                encoded_database[5] == 0,
            "database encoder did not emit v2");
    require(controller_profile_database_decode(
                read_encoded_database, nullptr, &decoded_database),
            "database did not decode");
    require(controller_profile_database_find(
                decoded_database, identity(16)) != nullptr,
            "last database identity did not round trip");

    encoded_database[12] = 1;
    require(!controller_profile_database_decode(
                read_encoded_database, nullptr, &decoded_database),
            "nonzero database header reservation was accepted");
}

}  // namespace
int main() {
    test_profile_wire_schema();
    test_legacy_profile_migration();
    test_database_round_trip_and_capacity();
    return 0;
}
