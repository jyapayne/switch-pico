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

void require_no_new_swing_actions(const ControllerProfile& profile) {
    require(profile.swing.macro == CONTROLLER_PROFILE_NO_BUTTON &&
                profile.nunchuk_swing.button == CONTROLLER_PROFILE_NO_BUTTON &&
                profile.nunchuk_swing.macro == CONTROLLER_PROFILE_NO_BUTTON &&
                profile.nunchuk_swing.sensitivity == 1 &&
                profile.nunchuk_swing.modifier == CONTROLLER_PROFILE_NO_BUTTON &&
                profile.combined_swing.button == CONTROLLER_PROFILE_NO_BUTTON &&
                profile.combined_swing.macro == CONTROLLER_PROFILE_NO_BUTTON &&
                profile.combined_swing.modifier == CONTROLLER_PROFILE_NO_BUTTON &&
                profile.combination_window_ms == 100,
            "legacy profile enabled a new gesture action or lost its defaults");
}

void test_pair_identity_wire_and_member_validation() {
    ControllerIdentity left = identity(1);
    left.transport = ControllerTransport::kBle;
    left.product_id = 0x2067;
    const uint8_t left_address[6] = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15};
    memcpy(left.address, left_address, sizeof(left_address));
    ControllerIdentity right = left;
    right.product_id = 0x2066;
    right.address_type = 1;
    right.address[0] = 0xc0;
    ControllerIdentity pair{};
    uint8_t encoded[CONTROLLER_IDENTITY_ENCODED_SIZE]{};
    const uint8_t expected[14] = {
        5, 3, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
        0xc0, 0x11, 0x12, 0x13, 0x14, 0x15};
    require(controller_identity_make_joycon_pair(left, right, &pair) &&
                controller_identity_encode(pair, encoded, sizeof(encoded)) &&
                memcmp(encoded, expected, sizeof(expected)) == 0,
            "pair identity did not encode both typed members");
    ControllerIdentity decoded{};
    ControllerIdentity decoded_left{};
    ControllerIdentity decoded_right{};
    require(controller_identity_decode(encoded, sizeof(encoded), &decoded) &&
                controller_identity_equal(pair, decoded) &&
                controller_identity_joycon_pair_members(
                    decoded, &decoded_left, &decoded_right) &&
                controller_identity_equal(left, decoded_left) &&
                controller_identity_equal(right, decoded_right),
            "pair identity round trip lost a member or its model");
    decoded.partner_address[5] ^= 1;
    require(!controller_identity_equal(pair, decoded),
            "pair equality ignored the right address");
    for (uint8_t flags = 0; flags < 16; ++flags) {
        if (flags == 1 || flags == 5) {
            continue;
        }
        memcpy(encoded, expected, sizeof(encoded));
        encoded[0] = flags;
        require(!controller_identity_decode(encoded, sizeof(encoded), &decoded),
                "pair decoder accepted unstable, reserved, or nonstatic flags");
    }
    require(!controller_identity_make_joycon_pair(right, left, &decoded),
            "pair helper accepted reversed models");
    right.product_id = 0x2069;
    require(!controller_identity_make_joycon_pair(left, right, &decoded),
            "pair helper accepted a non-Joy-Con member");
    right.product_id = 0x2066;
    right.address_type = 2;
    require(!controller_identity_make_joycon_pair(left, right, &decoded),
            "pair helper accepted a noncanonical identity address type");
    right.address_type = left.address_type;
    memcpy(right.address, left.address, sizeof(right.address));
    require(!controller_identity_make_joycon_pair(left, right, &decoded),
            "pair helper accepted one physical typed address twice");
    left.address[0] = right.address[0] = 0xc0;
    right.address_type = 1;
    require(controller_identity_make_joycon_pair(left, right, &decoded),
            "pair helper conflated public and static-random address namespaces");
    left.partner_address[0] = 1;
    require(!controller_identity_encode(left, encoded, sizeof(encoded)) &&
                !controller_identity_make_joycon_pair(left, right, &decoded),
            "ordinary member accepted stray partner fields");
    left.partner_address[0] = 0;
    const uint8_t ordinary[14] = {
        1, 2, 0, 0, 0xc0, 0x11, 0x12, 0x13, 0x14, 0x15,
        0x7e, 0x05, 0x67, 0x20};
    require(controller_identity_encode(left, encoded, sizeof(encoded)) &&
                memcmp(encoded, ordinary, sizeof(encoded)) == 0 &&
                controller_identity_decode(encoded, sizeof(encoded), &decoded) &&
                controller_identity_equal(left, decoded),
            "ordinary identity wire encoding changed");
    const uint8_t global[14]{};
    require(controller_identity_encode(controller_identity_global(), encoded,
                                       sizeof(encoded)) &&
                memcmp(encoded, global, sizeof(encoded)) == 0,
            "global identity wire encoding changed");
}

void test_profile_wire_schema() {
    const ControllerProfile profile =
        controller_profile_default(controller_identity_global(), 0);
    uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    require(controller_profile_encode(profile, encoded, sizeof(encoded)),
            "default profile did not encode");
    require(encoded[0] == 9 && encoded[1] == 0 &&
                encoded[2] == 0x80 && encoded[3] == 1,
            "profile header is not little-endian v9/384");
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
                encoded[74] == 3 && encoded[75] == 0 &&
                encoded[78] == 0 && encoded[79] == 0,
            "default rumble or action encoding changed");
    require(encoded[96] == 0 && encoded[97] == 0 &&
                encoded[98] == 0x7c && encoded[99] == 0 &&
                encoded[100] == 0 && encoded[101] == 0,
            "default sparse macro descriptor changed");

    ControllerProfile decoded{};
    require(controller_profile_decode(encoded, sizeof(encoded), &decoded),
            "default profile did not decode");
    ControllerProfile action_profile = profile;
    action_profile.switching_chord =
        (1u << CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL) |
        (1u << static_cast<uint8_t>(
            ControllerProfileLogicalButton::kSouth));
    action_profile.motion_toggle_chord =
        (1u << CONTROLLER_PROFILE_RIGHT_TRIGGER_CONTROL);
    action_profile.macros[0].trigger_mask =
        (1u << CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL) |
        (1u << CONTROLLER_PROFILE_RIGHT_TRIGGER_CONTROL);
    action_profile.macros[0].cancel_control =
        CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL;
    action_profile.macros[0].first_step = 0;
    action_profile.macros[0].step_count = 1;
    action_profile.macro_step_count = 1;
    action_profile.macro_steps[0].override_flags =
        kControllerProfileOverrideButtons;
    for (uint8_t macro_index = 1;
         macro_index < CONTROLLER_PROFILE_MACRO_COUNT; ++macro_index) {
        action_profile.macros[macro_index].first_step = 1;
    }
    action_profile.macro_steps[0].duration_ms = 25;
    action_profile.macro_steps[0].output_button_mask = 1;
    uint8_t action_encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    require(controller_profile_encode(
                action_profile, action_encoded, sizeof(action_encoded)),
            "sparse action profile did not encode");
    require(action_encoded[75] == 0x21,
            "sparse action extension encoding changed");
    require(action_encoded[96] == 0 &&
                action_encoded[97] == 0 &&
                action_encoded[98] == 0x43,
            "sparse macro binding encoding changed");
    require(action_encoded[100] == 1 &&
                action_encoded[101] == 5,
            "sparse macro step bounds changed");
    require(controller_profile_decode(
                action_encoded, sizeof(action_encoded), &decoded),
            "sparse action profile did not decode");
    require(decoded.switching_chord ==
                    action_profile.switching_chord &&
                decoded.motion_toggle_chord ==
                    action_profile.motion_toggle_chord &&
                decoded.macros[0].trigger_mask ==
                    action_profile.macros[0].trigger_mask &&
                decoded.macros[0].cancel_control ==
                    action_profile.macros[0].cancel_control &&
                decoded.macros[0].step_count == 1,
            "sparse trigger-backed macro did not round-trip");
    encoded[252] = 1;
    require(!controller_profile_decode(encoded, sizeof(encoded), &decoded),
            "nonzero reserved profile byte was accepted");

    ControllerProfile invalid = profile;
    invalid.button_map[0] = CONTROLLER_PROFILE_FIRST_EXTRA_CONTROL;
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
        static_cast<ControllerProfileTurboMode>(4);
    require(!controller_profile_validate(invalid),
            "invalid Turbo mode was accepted");
    invalid = profile;
    invalid.macros[0].first_step = 0;
    invalid.macros[0].step_count = 1;
    invalid.macro_step_count = 1;
    invalid.macro_steps[0].duration_ms =
        CONTROLLER_PROFILE_MAX_WAIT_MS + 1;
    require(!controller_profile_validate(invalid),
            "unbounded macro wait was accepted");
    invalid.macro_steps[0].duration_ms =
        CONTROLLER_PROFILE_MAX_WAIT_MS;
    invalid.macros[1].first_step = 0;
    require(!controller_profile_validate(invalid),
            "noncanonical shared macro pool was accepted");
}

void test_legacy_profile_migration() {
    ControllerProfile migrated{};
    require(controller_profile_decode(
                kLegacyDefaultProfile, sizeof(kLegacyDefaultProfile),
                &migrated),
            "legacy default profile did not decode");
    require(migrated.swing.button == CONTROLLER_PROFILE_NO_BUTTON &&
                migrated.swing.sensitivity == 1 &&
                migrated.swing.modifier == CONTROLLER_PROFILE_NO_BUTTON,
            "legacy padding enabled a swing gesture");
    require_no_new_swing_actions(migrated);
    require(migrated.triggers[0].digital_threshold ==
                    CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD &&
                migrated.triggers[1].digital_threshold ==
                    CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD,
            "legacy inherited thresholds were not migrated");

    uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    require(controller_profile_encode(migrated, encoded, sizeof(encoded)),
            "migrated default profile did not encode");
    require(migrated.macros[0].step_count == 0 &&
                migrated.macro_step_count == 0,
            "legacy end marker was not removed during migration");
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
                encoded[70] == CONTROLLER_PROFILE_RIGHT_TRIGGER_CONTROL,
            "migrated default profile did not encode as the current schema");

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
    const uint32_t expected_macro_trigger =
        legacy_macro_trigger < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT
            ? static_cast<uint32_t>(1u << legacy_macro_trigger)
            : 0;
    require(migrated.macros[0].trigger_mask ==
                    expected_macro_trigger &&
                migrated.macros[0].cancel_control ==
                    kLegacyCustomThresholdProfile[79],
            "legacy macro trigger was not migrated to descriptor zero");

    uint8_t previous_encoded[CONTROLLER_PROFILE_LEGACY_ENCODED_SIZE]{};
    memcpy(previous_encoded, kLegacyDefaultProfile,
           sizeof(previous_encoded));
    previous_encoded[0] = static_cast<uint8_t>(
        CONTROLLER_PROFILE_TRIGGER_THRESHOLD_SCHEMA_VERSION);
    previous_encoded[58] = static_cast<uint8_t>(
        CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD);
    previous_encoded[59] = static_cast<uint8_t>(
        CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD >> 8);
    previous_encoded[68] = static_cast<uint8_t>(
        CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD);
    previous_encoded[69] = static_cast<uint8_t>(
        CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD >> 8);
    previous_encoded[78] = static_cast<uint8_t>(
        ControllerProfileLogicalButton::kSouth);
    previous_encoded[79] = static_cast<uint8_t>(
        ControllerProfileLogicalButton::kCapture);
    require(controller_profile_decode(
                previous_encoded, sizeof(previous_encoded), &migrated) &&
                migrated.triggers[0].output ==
                    CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL &&
                migrated.triggers[1].output ==
                    CONTROLLER_PROFILE_RIGHT_TRIGGER_CONTROL &&
                migrated.macros[0].trigger_mask ==
                    static_cast<uint32_t>(
                        1u << static_cast<uint8_t>(
                            ControllerProfileLogicalButton::kSouth)) &&
                migrated.macros[0].cancel_control ==
                    static_cast<uint8_t>(
                        ControllerProfileLogicalButton::kCapture) &&
                migrated.motion_toggle_chord == 0,
            "v2 profile controls did not migrate to the current schema");
    previous_encoded[80] = 2;
    previous_encoded[100] = 0;
    previous_encoded[101] = kControllerProfileOverrideButtons;
    previous_encoded[102] = 25;
    previous_encoded[104] = 1;
    require(controller_profile_decode(
                previous_encoded, sizeof(previous_encoded), &migrated) &&
                migrated.macros[0].step_count == 1 &&
                migrated.macro_steps[0].output_button_mask == 1,
            "legacy nonempty macro did not migrate into shared pool");

    previous_encoded[60] = CONTROLLER_PROFILE_RIGHT_TRIGGER_CONTROL;
    previous_encoded[70] = CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL;
    previous_encoded[76] = 3;
    previous_encoded[78] = 1;
    previous_encoded[79] = 0;
    previous_encoded[81] = 11;
    previous_encoded[98] = 4;
    for (uint8_t version = 3; version <= 4; ++version) {
        previous_encoded[0] = version;
        previous_encoded[75] = version == 4 ? 0x21 : 0;
        migrated.swing = {2, 2, 24};
        migrated.nunchuk_swing = {0, 0, 0};
        migrated.combined_swing = {15, CONTROLLER_PROFILE_NO_BUTTON, 24};
        migrated.combination_window_ms = 30;
        require(controller_profile_decode(
                    previous_encoded, sizeof(previous_encoded), &migrated) &&
                    migrated.triggers[0].output ==
                        CONTROLLER_PROFILE_RIGHT_TRIGGER_CONTROL &&
                    migrated.triggers[1].output ==
                        CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL &&
                    migrated.switching_chord ==
                        (3u | (version == 4 ? 1u << 16 : 0u)) &&
                    migrated.motion_toggle_chord ==
                        (4u | (version == 4 ? 1u << 17 : 0u)) &&
                    migrated.macros[0].trigger_mask == 1 &&
                    migrated.macros[0].cancel_control == 11 &&
                    migrated.macros[0].step_count == 1 &&
                    migrated.macro_steps[0].output_button_mask == 1 &&
                    migrated.swing.button == CONTROLLER_PROFILE_NO_BUTTON &&
                    migrated.swing.sensitivity == 1 &&
                    migrated.swing.modifier == CONTROLLER_PROFILE_NO_BUTTON,
                "legacy control mapping or action migration lost settings");
        require_no_new_swing_actions(migrated);
    }

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
            "database encoder did not emit v3");
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

void test_set_b_sparse_extension_and_migration() {
    ControllerProfile profile =
        controller_profile_default(controller_identity_global(), 0);
    profile.shortcuts.modifier = 16;
    profile.shortcuts.selectors[0] = 0;
    profile.shortcuts.selectors[7] = 15;
    profile.shift.mode = ControllerProfileShiftMode::kToggle;
    profile.shift.modifier = 17;
    profile.shift.button_map[0] = CONTROLLER_PROFILE_NO_BUTTON;
    profile.turbo_modes[15] = ControllerProfileTurboMode::kBurst;
    profile.turbo_defaults = {30, 99, 255};
    profile.turbo_override_mask = (1u << 2) | (1u << 15);
    profile.turbo_overrides[2] = {1, 1, 1};
    profile.turbo_overrides[15] = {23, 37, 17};
    profile.turbo_overrides[0] = {0, 0, 0};
    profile.macros[0] = {1, 0xff, 0, 8, ControllerProfileMacroMode::kRepeat, 255};
    profile.macro_step_count = 8;
    for (uint8_t macro = 1; macro < CONTROLLER_PROFILE_MACRO_COUNT; ++macro) {
        profile.macros[macro].first_step = 8;
        profile.macros[macro].mode = static_cast<ControllerProfileMacroMode>(macro - 1);
    }
    for (uint8_t step = 0; step < 8; ++step) {
        profile.macro_steps[step] = {31, 25, 1, -2, 3, -4, 5, 0x1234, 0xabcd};
    }
    uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    ControllerProfile decoded{};
    require(controller_profile_encode(profile, encoded, sizeof(encoded)) &&
                controller_profile_decode(encoded, sizeof(encoded), &decoded),
            "full sparse stream and Set B extension did not round trip");
    require(encoded[254] == 0xcd && encoded[255] == 0xab &&
                encoded[256] == 16 && encoded[264] == 15 &&
                encoded[265] == 2 && encoded[266] == 17 &&
                encoded[267] == 0xff && encoded[283] == 30 &&
                encoded[286] == 4 && encoded[287] == 0x80 &&
                encoded[288] == 1 && encoded[291] == 23 &&
                encoded[336] == 3 && encoded[337] == 255,
            "Set B fields overlap macro data or use wrong sparse ordering");
    require(decoded.turbo_overrides[15].duty_percent == 37 &&
                decoded.macros[0].repeat_count == 255 &&
                decoded.shortcuts.selectors[7] == 15,
            "Set B extension settings were not decoded");
    for (size_t offset = 294; offset < 336; ++offset) {
        require(encoded[offset] == 0, "absent override was not canonical zero");
    }
    encoded[294] = 1;
    require(!controller_profile_decode(encoded, sizeof(encoded), &decoded),
            "nonzero sparse Turbo padding was accepted");
    encoded[294] = 0;
    encoded[383] = 1;
    require(!controller_profile_decode(encoded, sizeof(encoded), &decoded),
            "nonzero extension reservation was accepted");
    encoded[383] = 0;
    encoded[337] = 0;
    require(!controller_profile_decode(encoded, sizeof(encoded), &decoded),
            "zero macro repeat count was accepted");
    encoded[337] = 255;
    uint8_t legacy[CONTROLLER_PROFILE_LEGACY_ENCODED_SIZE]{};
    memcpy(legacy, encoded, sizeof(legacy));
    legacy[0] = 5;
    legacy[2] = 0;
    legacy[95] = 2;
    require(controller_profile_decode(legacy, sizeof(legacy), &decoded) &&
                decoded.macros[0].step_count == 8 &&
                decoded.macro_steps[7].right_trigger == 0xabcd &&
                decoded.macros[0].mode == ControllerProfileMacroMode::kOnce &&
                decoded.shortcuts.modifier == CONTROLLER_PROFILE_NO_BUTTON &&
                decoded.turbo_override_mask == 0 &&
                decoded.swing.button == CONTROLLER_PROFILE_NO_BUTTON &&
                decoded.swing.sensitivity == 1 &&
                decoded.swing.modifier == CONTROLLER_PROFILE_NO_BUTTON,
            "schema5 full136-byte macro stream did not migrate");
    require_no_new_swing_actions(decoded);
    require(controller_profile_encode(decoded, encoded, sizeof(encoded)) &&
                memcmp(&legacy[4], &encoded[4], sizeof(legacy) - 4) == 0,
            "schema5 migration changed existing profile data");
    legacy[0] = 6;
    require(!controller_profile_decode(legacy, sizeof(legacy), &decoded),
            "schema6 accepted a legacy-sized payload");
    profile.shortcuts.selectors[7] = 0;
    require(!controller_profile_validate(profile), "duplicate shortcut was accepted");
    profile.shortcuts.selectors[7] = 15;
    profile.shift.button_map[0] = 16;
    require(!controller_profile_validate(profile), "analog Shift output was accepted");
    profile.shift.button_map[0] = 0;
    profile.turbo_overrides[2].rate_hz = 31;
    require(!controller_profile_validate(profile), "out-of-range Turbo rate was accepted");
    profile.turbo_overrides[2].rate_hz = 1;
    for (uint8_t step = 0; step < 8; ++step) {
        profile.macro_steps[step].duration_ms = 0;
    }
    require(!controller_profile_validate(profile), "zero-duration looping macro was accepted");
    profile.macros[0].mode = ControllerProfileMacroMode::kOnce;
    require(controller_profile_validate(profile), "zero-duration once macro was rejected");
}

void test_legacy_database_strides() {
    for (uint8_t version = 1; version <= 2; ++version) {
        const uint8_t count = version == 1 ? 4 : 8;
        const size_t entry_size = 16 + count * 256;
        const size_t entries_offset = 32 + count * 256;
        const size_t total_size = entries_offset + 16 * entry_size;
        memset(encoded_database, 0, sizeof(encoded_database));
        memcpy(encoded_database, "SPDB", 4);
        encoded_database[4] = version;
        encoded_database[6] = static_cast<uint8_t>(total_size);
        encoded_database[7] = static_cast<uint8_t>(total_size >> 8);
        encoded_database[8] = 16;
        encoded_database[9] = count;
        encoded_database[10] = count - 1;
        encoded_database[11] = 1;
        for (uint8_t index = 0; index < count; ++index) {
            memcpy(&encoded_database[32 + index * 256], kLegacyDefaultProfile, 256);
        }
        const size_t last_entry = entries_offset + 15 * entry_size;
        require(controller_identity_encode(identity(16), &encoded_database[last_entry], 14),
                "legacy database identity did not encode");
        encoded_database[last_entry + 14] = count - 1;
        encoded_database[last_entry + 15] = 1;
        for (uint8_t index = 0; index < count; ++index) {
            memcpy(&encoded_database[last_entry + 16 + index * 256],
                   kLegacyCustomThresholdProfile, 256);
        }
        require(controller_profile_database_decode(
                    read_encoded_database, nullptr, &decoded_database),
                "legacy database strides were not preserved");
        const ControllerProfileDatabaseEntry* entry =
            controller_profile_database_find(decoded_database, identity(16));
        require(entry != nullptr && entry->active_profile == count - 1 &&
                    entry->profiles[count - 1].triggers[1].digital_threshold == 0xabcd,
                "last legacy bank profile was read from the wrong offset");
        if (count == 4) {
            require(entry->profiles[7].triggers[0].digital_threshold ==
                        CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD,
                    "new profile slots were not defaulted during migration");
        }
        encoded_database[4] = 3;
        require(!controller_profile_database_decode(
                    read_encoded_database, nullptr, &decoded_database),
                "current database version accepted legacy strides");
    }
}

void test_schema6_migration_preserves_every_setting() {
    ControllerProfile profile{};
    uint8_t upgraded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    require(controller_profile_decode(kLegacySchema6Profile,
                                      sizeof(kLegacySchema6Profile), &profile) &&
                controller_profile_encode(profile, upgraded, sizeof(upgraded)),
            "schema6 profile could not upgrade");
    require(memcmp(upgraded + 2, kLegacySchema6Profile + 2, 342) == 0,
            "schema6 migration changed an existing encoded setting");
    require(profile.swing.button == CONTROLLER_PROFILE_NO_BUTTON &&
                profile.swing.sensitivity == 1 &&
                profile.swing.modifier == CONTROLLER_PROFILE_NO_BUTTON,
            "schema6 padding became a swing gesture");
    require_no_new_swing_actions(profile);
    for (uint8_t index = 0; index < CONTROLLER_PROFILE_EXTRA_BUTTON_COUNT; ++index) {
        require(profile.extra_button_map[index] == CONTROLLER_PROFILE_NO_BUTTON &&
                    profile.shift.extra_button_map[index] == CONTROLLER_PROFILE_NO_BUTTON,
                "schema6 padding became an extra-button mapping");
    }
    ControllerProfile reloaded{};
    uint8_t round_trip[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    require(controller_profile_decode(upgraded, sizeof(upgraded), &reloaded) &&
                controller_profile_encode(reloaded, round_trip, sizeof(round_trip)) &&
                memcmp(upgraded, round_trip, sizeof(upgraded)) == 0,
            "upgraded schema6 profile was not stable after reload");

    const uint16_t extra_field_offsets[] = {344, 351, 358, 362, 363};
    for (uint16_t offset : extra_field_offsets) {
        uint8_t invalid[sizeof(kLegacySchema6Profile)]{};
        memcpy(invalid, kLegacySchema6Profile, sizeof(invalid));
        invalid[offset] = 1;
        require(!controller_profile_decode(invalid, sizeof(invalid), &reloaded),
                "schema6 interpreted an extra-control field in reserved padding");
    }
    const uint16_t source_offsets[] = {256, 266};
    for (uint16_t offset : source_offsets) {
        uint8_t invalid[sizeof(kLegacySchema6Profile)]{};
        memcpy(invalid, kLegacySchema6Profile, sizeof(invalid));
        invalid[offset] = CONTROLLER_PROFILE_FIRST_EXTRA_CONTROL;
        require(!controller_profile_decode(invalid, sizeof(invalid), &reloaded),
                "schema6 admitted an extra-control modifier");
    }
    uint8_t invalid[sizeof(kLegacySchema6Profile)]{};
    memcpy(invalid, kLegacySchema6Profile, sizeof(invalid));
    invalid[98] = (CONTROLLER_PROFILE_FIRST_EXTRA_CONTROL << 2) | 1;
    require(!controller_profile_decode(invalid, sizeof(invalid), &reloaded),
            "schema6 admitted an extra-control macro cancellation");
}

void test_extra_control_schema_round_trip_and_output_limits() {
    ControllerProfile profile{};
    require(controller_profile_decode(kLegacySchema6Profile,
                                      sizeof(kLegacySchema6Profile), &profile),
            "extra-control fixture did not decode");
    const uint8_t extra_map[] = {0, 16, 17, 12, 13, 14, 15};
    const uint8_t shifted_map[] = {15, 14, 13, 12, 3, 2, 0xff};
    memcpy(profile.extra_button_map, extra_map, sizeof(extra_map));
    memcpy(profile.shift.extra_button_map, shifted_map, sizeof(shifted_map));
    profile.shortcuts.modifier = 18;
    profile.shift.modifier = 24;
    profile.switching_chord |= 0x55u << 18;
    profile.motion_toggle_chord |= 0x2au << 18;
    for (uint8_t index = 0; index < CONTROLLER_PROFILE_MACRO_COUNT; ++index) {
        profile.macros[index].trigger_mask |= (1u << index) << 18;
        profile.macros[index].cancel_control = 24 - index;
    }
    uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    ControllerProfile decoded{};
    require(controller_profile_encode(profile, encoded, sizeof(encoded)) &&
                controller_profile_decode(encoded, sizeof(encoded), &decoded),
            "extra-control profile did not round-trip");
    require(memcmp(encoded + 344, extra_map, sizeof(extra_map)) == 0 &&
                memcmp(encoded + 351, shifted_map, sizeof(shifted_map)) == 0 &&
                encoded[358] == 1 && encoded[359] == 2 &&
                encoded[360] == 4 && encoded[361] == 8 &&
                encoded[362] == 0x55 && encoded[363] == 0x2a,
            "extra-control fields do not use the schema7 extension layout");
    uint8_t round_trip[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    require(controller_profile_encode(decoded, round_trip, sizeof(round_trip)) &&
                memcmp(encoded, round_trip, sizeof(encoded)) == 0,
            "extra controls lost masks, mappings, or modifiers on reload");
    for (uint16_t offset = 358; offset < 364; ++offset) {
        encoded[offset] |= 0x80;
        require(!controller_profile_decode(encoded, sizeof(encoded), &decoded),
                "out-of-range extra-control mask was accepted");
        encoded[offset] &= 0x7f;
    }
    profile.extra_button_map[0] = 18;
    require(!controller_profile_validate(profile),
            "extra input was accepted as a console output destination");
    profile.extra_button_map[0] = 0;
    profile.triggers[0].output = 18;
    require(!controller_profile_validate(profile),
            "analog trigger was allowed to route into a source-only control");
    profile.triggers[0].output = 0;
    profile.shift.extra_button_map[0] = 16;
    require(!controller_profile_validate(profile),
            "Shift extra mapping admitted an analog destination");
}

void test_schema7_migration_preserves_extra_controls_and_macros() {
    uint8_t legacy[sizeof(kLegacySchema6Profile)]{};
    memcpy(legacy, kLegacySchema6Profile, sizeof(legacy));
    legacy[0] = 7;
    const uint8_t extra_map[] = {0, 16, 17, 12, 13, 14, 15};
    const uint8_t shifted_map[] = {15, 14, 13, 12, 3, 2, 0xff};
    memcpy(legacy + 344, extra_map, sizeof(extra_map));
    memcpy(legacy + 351, shifted_map, sizeof(shifted_map));
    legacy[256] = 18;
    legacy[266] = 24;
    for (uint8_t index = 0; index < CONTROLLER_PROFILE_MACRO_COUNT; ++index) {
        legacy[98 + index * 6] = static_cast<uint8_t>(
            (legacy[98 + index * 6] & 3u) | ((24 - index) << 2));
        legacy[358 + index] = static_cast<uint8_t>(1u << index);
    }
    legacy[362] = 0x55;
    legacy[363] = 0x2a;
    ControllerProfile migrated{};
    migrated.swing = {2, 2, 24};
    uint8_t upgraded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    require(controller_profile_decode(legacy, sizeof(legacy), &migrated) &&
                controller_profile_encode(migrated, upgraded, sizeof(upgraded)) &&
                memcmp(legacy + 2, upgraded + 2, 362) == 0 &&
                migrated.swing.button == CONTROLLER_PROFILE_NO_BUTTON &&
                migrated.swing.sensitivity == 1 &&
                migrated.swing.modifier == CONTROLLER_PROFILE_NO_BUTTON,
            "schema7 migration lost existing settings or enabled swing");
    require_no_new_swing_actions(migrated);
    for (size_t offset = 364; offset < sizeof(legacy); ++offset) {
        legacy[offset] = 1;
        require(!controller_profile_decode(legacy, sizeof(legacy), &migrated),
                "schema7 interpreted reserved padding as swing settings");
        legacy[offset] = 0;
    }
}

void test_schema8_migration_preserves_remote_swing() {
    uint8_t legacy[sizeof(kLegacySchema6Profile)]{};
    memcpy(legacy, kLegacySchema6Profile, sizeof(legacy));
    legacy[0] = 8;
    legacy[344] = 16;
    legacy[351] = 15;
    legacy[358] = 0x40;
    legacy[362] = 0x55;
    legacy[363] = 0x2a;
    legacy[364] = 15;
    legacy[365] = 2;
    legacy[366] = 24;
    ControllerProfile migrated{};
    migrated.swing.macro = 0;
    migrated.nunchuk_swing = {0, 0, 0};
    migrated.combined_swing = {1, CONTROLLER_PROFILE_NO_BUTTON, 16};
    migrated.combination_window_ms = 200;
    uint8_t upgraded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    require(controller_profile_decode(legacy, sizeof(legacy), &migrated) &&
                controller_profile_encode(migrated, upgraded, sizeof(upgraded)) &&
                memcmp(legacy + 2, upgraded + 2, 365) == 0 &&
                migrated.swing.button == 15 &&
                migrated.swing.sensitivity == 2 &&
                migrated.swing.modifier == 24,
            "schema8 migration changed Remote, extras, or macro settings");
    require_no_new_swing_actions(migrated);
    for (size_t offset = 367; offset < sizeof(legacy); ++offset) {
        legacy[offset] = 1;
        require(!controller_profile_decode(legacy, sizeof(legacy), &migrated),
                "schema8 interpreted reserved padding as new swing actions");
        legacy[offset] = 0;
    }
}

void test_swing_wire_settings_and_rejection() {
    ControllerProfile profile =
        controller_profile_default(controller_identity_global(), 0);
    profile.swing = {15, 2, 24};
    profile.nunchuk_swing = {0, 0, 0};
    profile.combined_swing = {4, CONTROLLER_PROFILE_NO_BUTTON, 16};
    profile.combination_window_ms = 30;
    profile.macros[0].step_count = 1;
    profile.macro_step_count = 1;
    profile.macro_steps[0].duration_ms = 1;
    for (uint8_t index = 1; index < CONTROLLER_PROFILE_MACRO_COUNT; ++index) {
        profile.macros[index].first_step = 1;
    }
    uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    ControllerProfile decoded{};
    const uint8_t button_actions[] = {
        15, 2, 24, 255, 0, 0, 0, 255, 4, 255, 16, 30};
    require(controller_profile_encode(profile, encoded, sizeof(encoded)) &&
                memcmp(encoded + 364, button_actions, sizeof(button_actions)) == 0 &&
                controller_profile_decode(encoded, sizeof(encoded), &decoded) &&
                decoded.swing.button == 15 && decoded.swing.sensitivity == 2 &&
                decoded.swing.modifier == 24 &&
                decoded.nunchuk_swing.button == 0 &&
                decoded.nunchuk_swing.sensitivity == 0 &&
                decoded.nunchuk_swing.modifier == 0 &&
                decoded.combined_swing.button == 4 &&
                decoded.combined_swing.modifier == 16 &&
                decoded.combination_window_ms == 30,
            "independent swing button actions did not use the schema9 layout");
    const ControllerProfileSwingConfiguration invalid_settings[] = {
        {16, 2, 24}, {254, 2, 24}, {15, 3, 24}, {15, 255, 24},
        {15, 2, 25}, {15, 2, 254}, {255, 3, 255}, {255, 1, 25},
        {255, 1, 24, 4}, {255, 1, 24, 254}, {0, 1, 24, 0}};
    for (uint8_t sensor = 0; sensor < 2; ++sensor) {
        const size_t offset = 364 + sensor * 4;
        for (const auto& settings : invalid_settings) {
            ControllerProfile invalid = profile;
            (sensor == 0 ? invalid.swing : invalid.nunchuk_swing) = settings;
            uint8_t malformed[sizeof(encoded)]{};
            require(!controller_profile_validate(invalid) &&
                        !controller_profile_encode(invalid, malformed,
                                                   sizeof(malformed)),
                    "invalid single-sensor swing action was accepted for encoding");
            memcpy(malformed, encoded, sizeof(malformed));
            malformed[offset] = settings.button;
            malformed[offset + 1] = settings.sensitivity;
            malformed[offset + 2] = settings.modifier;
            malformed[offset + 3] = settings.macro;
            require(!controller_profile_decode(malformed, sizeof(malformed), &decoded),
                    "invalid single-sensor swing wire action was accepted");
        }
    }
    const ControllerProfileCombinedSwingConfiguration invalid_combined[] = {
        {16, 255, 24}, {254, 255, 24}, {255, 4, 24},
        {255, 254, 24}, {0, 0, 24}, {15, 255, 25}, {255, 255, 254}};
    for (const auto& settings : invalid_combined) {
        ControllerProfile invalid = profile;
        invalid.combined_swing = settings;
        uint8_t malformed[sizeof(encoded)]{};
        require(!controller_profile_validate(invalid) &&
                    !controller_profile_encode(invalid, malformed, sizeof(malformed)),
                "invalid combined swing action was accepted for encoding");
        memcpy(malformed, encoded, sizeof(malformed));
        malformed[372] = settings.button;
        malformed[373] = settings.macro;
        malformed[374] = settings.modifier;
        require(!controller_profile_decode(malformed, sizeof(malformed), &decoded),
                "invalid combined swing wire action was accepted");
    }
    const uint8_t invalid_windows[] = {29, 201};
    for (uint8_t window : invalid_windows) {
        ControllerProfile invalid = profile;
        invalid.combination_window_ms = window;
        uint8_t malformed[sizeof(encoded)]{};
        require(!controller_profile_validate(invalid) &&
                    !controller_profile_encode(invalid, malformed, sizeof(malformed)),
                "out-of-range combination window was accepted for encoding");
        memcpy(malformed, encoded, sizeof(malformed));
        malformed[375] = window;
        require(!controller_profile_decode(malformed, sizeof(malformed), &decoded),
                "out-of-range combination window was accepted from wire");
    }
    for (size_t offset = 376; offset < sizeof(encoded); ++offset) {
        require(encoded[offset] == 0, "schema9 reserved tail was not zero");
        encoded[offset] = 1;
        require(!controller_profile_decode(encoded, sizeof(encoded), &decoded),
                "nonzero swing extension reservation was accepted");
        encoded[offset] = 0;
    }
    profile.swing = {CONTROLLER_PROFILE_NO_BUTTON, 0,
                     CONTROLLER_PROFILE_NO_BUTTON};
    profile.nunchuk_swing = profile.swing;
    profile.combined_swing = {};
    require(controller_profile_encode(profile, encoded, sizeof(encoded)) &&
                controller_profile_decode(encoded, sizeof(encoded), &decoded) &&
                decoded.swing.button == CONTROLLER_PROFILE_NO_BUTTON &&
                decoded.swing.macro == CONTROLLER_PROFILE_NO_BUTTON &&
                decoded.swing.sensitivity == 0 &&
                decoded.nunchuk_swing.button == CONTROLLER_PROFILE_NO_BUTTON &&
                decoded.nunchuk_swing.macro == CONTROLLER_PROFILE_NO_BUTTON &&
                decoded.nunchuk_swing.sensitivity == 0 &&
                decoded.combined_swing.button == CONTROLLER_PROFILE_NO_BUTTON &&
                decoded.combined_swing.macro == CONTROLLER_PROFILE_NO_BUTTON,
            "disabled gestures became actions or lost their sensitivity");
}

void test_swing_macro_targets_require_playable_steps() {
    ControllerProfile profile =
        controller_profile_default(controller_identity_global(), 0);
    profile.macros[0].step_count = 2;
    for (uint8_t index = 1; index < CONTROLLER_PROFILE_MACRO_COUNT; ++index) {
        profile.macros[index].first_step = 2;
    }
    profile.macros[3].step_count = 1;
    profile.macro_step_count = 3;
    profile.macro_steps[1].duration_ms = 1;
    profile.macro_steps[2].duration_ms = 10000;
    profile.swing = {255, 2, 24, 0};
    profile.nunchuk_swing = {255, 0, 255, 3};
    profile.combined_swing = {255, 0, 0};
    profile.combination_window_ms = 200;
    uint8_t encoded[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    ControllerProfile decoded{};
    const uint8_t macro_actions[] = {
        255, 2, 24, 0, 255, 0, 255, 3, 255, 0, 0, 200};
    for (uint8_t mode = 0; mode <= 3; ++mode) {
        profile.macros[0].mode = static_cast<ControllerProfileMacroMode>(mode);
        require(controller_profile_encode(profile, encoded, sizeof(encoded)) &&
                    memcmp(encoded + 364, macro_actions, sizeof(macro_actions)) == 0 &&
                    controller_profile_decode(encoded, sizeof(encoded), &decoded) &&
                    decoded.swing.macro == 0 && decoded.nunchuk_swing.macro == 3 &&
                    decoded.combined_swing.macro == 0 &&
                    decoded.combination_window_ms == 200 &&
                    decoded.macros[0].mode == profile.macros[0].mode &&
                    decoded.macros[0].trigger_mask == 0 &&
                    decoded.macro_steps[0].duration_ms == 0 &&
                    decoded.macro_steps[1].duration_ms == 1 &&
                    decoded.macro_steps[2].duration_ms == 10000,
                "gesture macro targets lost steps, playback mode, or index boundaries");
    }
    profile.swing = {};
    profile.nunchuk_swing = {};
    profile.combined_swing = {};
    profile.macros[0].mode = ControllerProfileMacroMode::kOnce;
    profile.macro_steps[1].duration_ms = 0;
    require(controller_profile_encode(profile, encoded, sizeof(encoded)),
            "unbound zero-duration one-shot macro was rejected");
    const size_t macro_offsets[] = {367, 371, 373};
    for (uint8_t gesture = 0; gesture < 3; ++gesture) {
        for (uint8_t target = 0; target < 2; ++target) {
            ControllerProfile invalid = profile;
            uint8_t* bindings[] = {&invalid.swing.macro,
                                   &invalid.nunchuk_swing.macro,
                                   &invalid.combined_swing.macro};
            *bindings[gesture] = target;
            uint8_t malformed[sizeof(encoded)]{};
            require(!controller_profile_validate(invalid) &&
                        !controller_profile_encode(invalid, malformed, sizeof(malformed)),
                    "gesture accepted a zero-duration or empty macro target");
            memcpy(malformed, encoded, sizeof(malformed));
            malformed[macro_offsets[gesture]] = target;
            require(!controller_profile_decode(malformed, sizeof(malformed), &decoded),
                    "wire gesture accepted a zero-duration or empty macro target");
        }
    }
    profile.macro_steps[1].duration_ms = 1;
    profile.macro_steps[2].duration_ms = 0;
    profile.nunchuk_swing.macro = 3;
    require(!controller_profile_validate(profile),
            "gesture used another macro's duration to validate its target");
}

}  // namespace
int main() {
    test_pair_identity_wire_and_member_validation();
    test_profile_wire_schema();
    test_legacy_profile_migration();
    test_database_round_trip_and_capacity();
    test_set_b_sparse_extension_and_migration();
    test_legacy_database_strides();
    test_schema6_migration_preserves_every_setting();
    test_extra_control_schema_round_trip_and_output_limits();
    test_schema7_migration_preserves_extra_controls_and_macros();
    test_schema8_migration_preserves_remote_swing();
    test_swing_wire_settings_and_rejection();
    test_swing_macro_targets_require_playable_steps();
    return 0;
}
