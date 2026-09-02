#include "controller_identity.h"
#include "controller_profile.h"

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
    require(encoded[0] == 1 && encoded[1] == 0 &&
                encoded[2] == 0 && encoded[3] == 1,
            "profile header is not little-endian v1/256");
    for (uint8_t index = 0;
         index < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT; ++index) {
        require(encoded[4 + index] == index,
                "default direct mapping is not identity");
    }
    require(encoded[26] == 0xff && encoded[27] == 0x7f &&
                encoded[30] == 0,
            "default stick encoding changed");
    require(encoded[54] == 0xff && encoded[55] == 0xff &&
                encoded[58] == 0x00 && encoded[59] == 0x80,
            "default trigger encoding changed");
    require(encoded[72] == 0xff && encoded[73] == 0xff &&
                encoded[74] == 3 && encoded[78] == 0xff &&
                encoded[79] == 0xff && encoded[80] == 1,
            "default rumble or macro encoding changed");
    for (uint8_t index = 0;
         index < CONTROLLER_PROFILE_MACRO_STEP_CAPACITY; ++index) {
        require(encoded[100 + index * 19] == 1,
                "unused macro step is not canonical end");
    }

    ControllerProfile decoded{};
    require(controller_profile_decode(encoded, sizeof(encoded), &decoded),
            "default profile did not decode");
    encoded[252] = 1;
    require(!controller_profile_decode(encoded, sizeof(encoded), &decoded),
            "nonzero reserved profile byte was accepted");

    ControllerProfile invalid = profile;
    invalid.button_map[0] = 16;
    require(!controller_profile_validate(invalid),
            "invalid direct output was accepted");
    invalid = profile;
    invalid.sticks[0].inner_deadzone =
        invalid.sticks[0].outer_saturation;
    require(!controller_profile_validate(invalid),
            "empty stick range was accepted");
    invalid = profile;
    invalid.triggers[0].digital_threshold = 0;
    invalid.triggers[0].lower_deadzone = 1;
    require(!controller_profile_validate(invalid),
            "trigger threshold outside its range was accepted");
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
    test_database_round_trip_and_capacity();
    return 0;
}
