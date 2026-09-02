#include "controller_profile.h"

#include <string.h>

namespace {

constexpr uint8_t kDatabaseMagic[4] = {'S', 'P', 'D', 'B'};
constexpr size_t kFallbackOffset = CONTROLLER_PROFILE_DATABASE_HEADER_SIZE;
constexpr size_t kEntriesOffset =
    kFallbackOffset + CONTROLLER_PROFILE_COUNT *
                          CONTROLLER_PROFILE_ENCODED_SIZE;
constexpr uint8_t kStickInvertX = 1u << 0;
constexpr uint8_t kStickInvertY = 1u << 1;
constexpr uint8_t kMacroOverrideMask =
    kControllerProfileOverrideButtons |
    kControllerProfileOverrideLeftStick |
    kControllerProfileOverrideRightStick |
    kControllerProfileOverrideLeftTrigger |
    kControllerProfileOverrideRightTrigger;

uint16_t profile_read_u16(const uint8_t* input) {
    return static_cast<uint16_t>(input[0]) |
           (static_cast<uint16_t>(input[1]) << 8);
}

int16_t profile_read_i16(const uint8_t* input) {
    return static_cast<int16_t>(profile_read_u16(input));
}

void profile_write_u16(uint8_t* output, uint16_t value) {
    output[0] = static_cast<uint8_t>(value);
    output[1] = static_cast<uint8_t>(value >> 8);
}

void profile_write_i16(uint8_t* output, int16_t value) {
    profile_write_u16(output, static_cast<uint16_t>(value));
}

bool profile_bytes_are_zero(const uint8_t* data, size_t size) {
    for (size_t index = 0; index < size; ++index) {
        if (data[index] != 0) {
            return false;
        }
    }
    return true;
}

bool valid_button(uint8_t button) {
    return button < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT ||
           button == CONTROLLER_PROFILE_NO_BUTTON;
}

bool valid_macro_step(const ControllerProfileMacroStep& step,
                      bool must_end) {
    if (must_end) {
        return step.type == ControllerProfileMacroStepType::kEnd &&
               step.override_flags == 0 && step.duration_ms == 0 &&
               step.output_button_mask == 0 && step.left_stick_x == 0 &&
               step.left_stick_y == 0 && step.right_stick_x == 0 &&
               step.right_stick_y == 0 && step.left_trigger == 0 &&
               step.right_trigger == 0;
    }
    if (step.type != ControllerProfileMacroStepType::kState ||
        (step.override_flags & ~kMacroOverrideMask) != 0 ||
        step.duration_ms > CONTROLLER_PROFILE_MAX_WAIT_MS) {
        return false;
    }
    if ((step.override_flags & kControllerProfileOverrideButtons) == 0 &&
        step.output_button_mask != 0) {
        return false;
    }
    if ((step.override_flags & kControllerProfileOverrideLeftStick) == 0 &&
        (step.left_stick_x != 0 || step.left_stick_y != 0)) {
        return false;
    }
    if ((step.override_flags & kControllerProfileOverrideRightStick) == 0 &&
        (step.right_stick_x != 0 || step.right_stick_y != 0)) {
        return false;
    }
    if ((step.override_flags & kControllerProfileOverrideLeftTrigger) == 0 &&
        step.left_trigger != 0) {
        return false;
    }
    if ((step.override_flags & kControllerProfileOverrideRightTrigger) == 0 &&
        step.right_trigger != 0) {
        return false;
    }
    return true;
}

void copy_overlap(size_t range_offset, uint8_t* output,
                  size_t output_size, size_t field_offset,
                  const uint8_t* field, size_t field_size) {
    const size_t range_end = range_offset + output_size;
    const size_t field_end = field_offset + field_size;
    if (range_offset >= field_end || field_offset >= range_end) {
        return;
    }
    const size_t start = range_offset > field_offset
                             ? range_offset
                             : field_offset;
    const size_t end = range_end < field_end ? range_end : field_end;
    memcpy(&output[start - range_offset], &field[start - field_offset],
           end - start);
}

bool read_zero_region(ControllerProfileDatabaseRead read, void* context,
                      size_t offset, size_t size) {
    uint8_t buffer[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    while (size != 0) {
        const size_t chunk = size < sizeof(buffer) ? size : sizeof(buffer);
        if (!read(context, offset, buffer, chunk) ||
            !profile_bytes_are_zero(buffer, chunk)) {
            return false;
        }
        offset += chunk;
        size -= chunk;
    }
    return true;
}

}  // namespace

ControllerProfile controller_profile_default(const ControllerIdentity& identity,
                                             uint8_t profile_index) {
    (void)identity;
    (void)profile_index;
    ControllerProfile profile{};
    for (uint8_t index = 0;
         index < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT; ++index) {
        profile.button_map[index] = index;
        profile.turbo_modes[index] = ControllerProfileTurboMode::kOff;
    }
    for (ControllerProfileStickConfiguration& stick : profile.sticks) {
        stick.center_x = 0;
        stick.center_y = 0;
        stick.inner_deadzone = 0;
        stick.outer_saturation = 32767;
        stick.curve_q8_8 = 256;
        stick.invert_x = false;
        stick.invert_y = false;
    }
    for (ControllerProfileTriggerConfiguration& trigger : profile.triggers) {
        trigger.lower_deadzone = 0;
        trigger.upper_saturation = UINT16_MAX;
        trigger.curve_q8_8 = 256;
        trigger.digital_threshold = 0x8000;
    }
    profile.weak_rumble_scale = UINT8_MAX;
    profile.strong_rumble_scale = UINT8_MAX;
    profile.confirmation_policy =
        ControllerProfileConfirmationPolicy::kRumbleAndLed;
    profile.switching_chord = 0;
    profile.macro_trigger = CONTROLLER_PROFILE_NO_BUTTON;
    profile.macro_cancel = CONTROLLER_PROFILE_NO_BUTTON;
    profile.macro_step_count = 1;
    for (ControllerProfileMacroStep& step : profile.macro_steps) {
        step = {};
        step.type = ControllerProfileMacroStepType::kEnd;
    }
    return profile;
}

bool controller_profile_validate(const ControllerProfile& profile) {
    for (uint8_t output : profile.button_map) {
        if (!valid_button(output)) {
            return false;
        }
    }
    for (const ControllerProfileStickConfiguration& stick : profile.sticks) {
        if (stick.inner_deadzone >= stick.outer_saturation ||
            stick.outer_saturation > 32767 || stick.curve_q8_8 == 0) {
            return false;
        }
    }
    for (const ControllerProfileTriggerConfiguration& trigger :
         profile.triggers) {
        if (trigger.lower_deadzone >= trigger.upper_saturation ||
            trigger.curve_q8_8 == 0 ||
            trigger.digital_threshold < trigger.lower_deadzone ||
            trigger.digital_threshold > trigger.upper_saturation) {
            return false;
        }
    }
    if (static_cast<uint8_t>(profile.confirmation_policy) >
            static_cast<uint8_t>(
                ControllerProfileConfirmationPolicy::kRumbleAndLed) ||
        !valid_button(profile.macro_trigger) ||
        !valid_button(profile.macro_cancel) ||
        profile.macro_step_count == 0 ||
        profile.macro_step_count > CONTROLLER_PROFILE_MACRO_STEP_CAPACITY) {
        return false;
    }
    for (ControllerProfileTurboMode mode : profile.turbo_modes) {
        if (static_cast<uint8_t>(mode) >
            static_cast<uint8_t>(ControllerProfileTurboMode::kAutoBurst)) {
            return false;
        }
    }
    for (uint8_t index = 0;
         index < CONTROLLER_PROFILE_MACRO_STEP_CAPACITY; ++index) {
        const bool must_end = index >= profile.macro_step_count - 1;
        if (!valid_macro_step(profile.macro_steps[index], must_end)) {
            return false;
        }
    }
    return true;
}

bool controller_profile_encode(const ControllerProfile& profile,
                               uint8_t* output, size_t output_size) {
    if (output == nullptr || output_size != CONTROLLER_PROFILE_ENCODED_SIZE ||
        !controller_profile_validate(profile)) {
        return false;
    }
    memset(output, 0, output_size);
    profile_write_u16(&output[0], CONTROLLER_PROFILE_SCHEMA_VERSION);
    profile_write_u16(&output[2], CONTROLLER_PROFILE_ENCODED_SIZE);
    memcpy(&output[4], profile.button_map,
           CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT);
    for (uint8_t index = 0; index < 2; ++index) {
        const ControllerProfileStickConfiguration& stick =
            profile.sticks[index];
        uint8_t* encoded = &output[20 + index * 16];
        profile_write_i16(&encoded[0], stick.center_x);
        profile_write_i16(&encoded[2], stick.center_y);
        profile_write_u16(&encoded[4], stick.inner_deadzone);
        profile_write_u16(&encoded[6], stick.outer_saturation);
        profile_write_u16(&encoded[8], stick.curve_q8_8);
        encoded[10] = (stick.invert_x ? kStickInvertX : 0) |
                      (stick.invert_y ? kStickInvertY : 0);
    }
    for (uint8_t index = 0; index < 2; ++index) {
        const ControllerProfileTriggerConfiguration& trigger =
            profile.triggers[index];
        uint8_t* encoded = &output[52 + index * 10];
        profile_write_u16(&encoded[0], trigger.lower_deadzone);
        profile_write_u16(&encoded[2], trigger.upper_saturation);
        profile_write_u16(&encoded[4], trigger.curve_q8_8);
        profile_write_u16(&encoded[6], trigger.digital_threshold);
    }
    output[72] = profile.weak_rumble_scale;
    output[73] = profile.strong_rumble_scale;
    output[74] = static_cast<uint8_t>(profile.confirmation_policy);
    profile_write_u16(&output[76], profile.switching_chord);
    output[78] = profile.macro_trigger;
    output[79] = profile.macro_cancel;
    output[80] = profile.macro_step_count;
    for (uint8_t index = 0;
         index < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT; ++index) {
        output[82 + index] =
            static_cast<uint8_t>(profile.turbo_modes[index]);
    }
    for (uint8_t index = 0;
         index < CONTROLLER_PROFILE_MACRO_STEP_CAPACITY; ++index) {
        const ControllerProfileMacroStep& step = profile.macro_steps[index];
        uint8_t* encoded = &output[100 + index * 19];
        encoded[0] = static_cast<uint8_t>(step.type);
        encoded[1] = step.override_flags;
        profile_write_u16(&encoded[2], step.duration_ms);
        profile_write_u16(&encoded[4], step.output_button_mask);
        profile_write_i16(&encoded[6], step.left_stick_x);
        profile_write_i16(&encoded[8], step.left_stick_y);
        profile_write_i16(&encoded[10], step.right_stick_x);
        profile_write_i16(&encoded[12], step.right_stick_y);
        profile_write_u16(&encoded[14], step.left_trigger);
        profile_write_u16(&encoded[16], step.right_trigger);
    }
    return true;
}

bool controller_profile_decode(const uint8_t* input, size_t input_size,
                               ControllerProfile* output) {
    if (input == nullptr || output == nullptr ||
        input_size != CONTROLLER_PROFILE_ENCODED_SIZE ||
        profile_read_u16(&input[0]) != CONTROLLER_PROFILE_SCHEMA_VERSION ||
        profile_read_u16(&input[2]) != CONTROLLER_PROFILE_ENCODED_SIZE ||
        !profile_bytes_are_zero(&input[31], 5) || !profile_bytes_are_zero(&input[47], 5) ||
        !profile_bytes_are_zero(&input[60], 2) || !profile_bytes_are_zero(&input[70], 2) ||
        input[75] != 0 || input[81] != 0 ||
        !profile_bytes_are_zero(&input[98], 2) || !profile_bytes_are_zero(&input[252], 4)) {
        return false;
    }

    ControllerProfile profile{};
    memcpy(profile.button_map, &input[4],
           CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT);
    for (uint8_t index = 0; index < 2; ++index) {
        ControllerProfileStickConfiguration& stick = profile.sticks[index];
        const uint8_t* encoded = &input[20 + index * 16];
        if ((encoded[10] & ~(kStickInvertX | kStickInvertY)) != 0) {
            return false;
        }
        stick.center_x = profile_read_i16(&encoded[0]);
        stick.center_y = profile_read_i16(&encoded[2]);
        stick.inner_deadzone = profile_read_u16(&encoded[4]);
        stick.outer_saturation = profile_read_u16(&encoded[6]);
        stick.curve_q8_8 = profile_read_u16(&encoded[8]);
        stick.invert_x = (encoded[10] & kStickInvertX) != 0;
        stick.invert_y = (encoded[10] & kStickInvertY) != 0;
    }
    for (uint8_t index = 0; index < 2; ++index) {
        ControllerProfileTriggerConfiguration& trigger =
            profile.triggers[index];
        const uint8_t* encoded = &input[52 + index * 10];
        trigger.lower_deadzone = profile_read_u16(&encoded[0]);
        trigger.upper_saturation = profile_read_u16(&encoded[2]);
        trigger.curve_q8_8 = profile_read_u16(&encoded[4]);
        trigger.digital_threshold = profile_read_u16(&encoded[6]);
    }
    profile.weak_rumble_scale = input[72];
    profile.strong_rumble_scale = input[73];
    profile.confirmation_policy =
        static_cast<ControllerProfileConfirmationPolicy>(input[74]);
    profile.switching_chord = profile_read_u16(&input[76]);
    profile.macro_trigger = input[78];
    profile.macro_cancel = input[79];
    profile.macro_step_count = input[80];
    for (uint8_t index = 0;
         index < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT; ++index) {
        profile.turbo_modes[index] =
            static_cast<ControllerProfileTurboMode>(input[82 + index]);
    }
    for (uint8_t index = 0;
         index < CONTROLLER_PROFILE_MACRO_STEP_CAPACITY; ++index) {
        ControllerProfileMacroStep& step = profile.macro_steps[index];
        const uint8_t* encoded = &input[100 + index * 19];
        if (encoded[18] != 0) {
            return false;
        }
        step.type = static_cast<ControllerProfileMacroStepType>(encoded[0]);
        step.override_flags = encoded[1];
        step.duration_ms = profile_read_u16(&encoded[2]);
        step.output_button_mask = profile_read_u16(&encoded[4]);
        step.left_stick_x = profile_read_i16(&encoded[6]);
        step.left_stick_y = profile_read_i16(&encoded[8]);
        step.right_stick_x = profile_read_i16(&encoded[10]);
        step.right_stick_y = profile_read_i16(&encoded[12]);
        step.left_trigger = profile_read_u16(&encoded[14]);
        step.right_trigger = profile_read_u16(&encoded[16]);
    }
    if (!controller_profile_validate(profile)) {
        return false;
    }
    *output = profile;
    return true;
}

void controller_profile_database_default(ControllerProfileDatabase* database) {
    if (database == nullptr) {
        return;
    }
    database->fallback_active_profile = 0;
    const ControllerIdentity global = controller_identity_global();
    for (ControllerProfileDatabaseEntry& entry : database->entries) {
        entry.used = false;
        entry.identity = global;
        entry.active_profile = 0;
    }
    for (uint8_t profile_index = 0;
         profile_index < CONTROLLER_PROFILE_COUNT; ++profile_index) {
        database->fallback_profiles[profile_index] =
            controller_profile_default(global, profile_index);
    }
}

bool controller_profile_database_validate(
    const ControllerProfileDatabase& database) {
    if (database.fallback_active_profile >= CONTROLLER_PROFILE_COUNT) {
        return false;
    }
    for (const ControllerProfile& profile : database.fallback_profiles) {
        if (!controller_profile_validate(profile)) {
            return false;
        }
    }
    for (uint8_t index = 0;
         index < CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY; ++index) {
        const ControllerProfileDatabaseEntry& entry = database.entries[index];
        if (!entry.used) {
            continue;
        }
        if (!entry.identity.stable ||
            controller_identity_is_global(entry.identity) ||
            entry.active_profile >= CONTROLLER_PROFILE_COUNT) {
            return false;
        }
        for (uint8_t prior = 0; prior < index; ++prior) {
            if (database.entries[prior].used &&
                controller_identity_equal(database.entries[prior].identity,
                                          entry.identity)) {
                return false;
            }
        }
        for (const ControllerProfile& profile : entry.profiles) {
            if (!controller_profile_validate(profile)) {
                return false;
            }
        }
    }
    return true;
}

bool controller_profile_database_encode_range(
    const ControllerProfileDatabase& database, size_t offset,
    uint8_t* output, size_t size) {
    if (output == nullptr || offset > CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE ||
        size > CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE - offset ||
        !controller_profile_database_validate(database)) {
        return false;
    }
    memset(output, 0, size);
    uint8_t header[CONTROLLER_PROFILE_DATABASE_HEADER_SIZE]{};
    memcpy(header, kDatabaseMagic, sizeof(kDatabaseMagic));
    profile_write_u16(&header[4],
                      CONTROLLER_PROFILE_DATABASE_SCHEMA_VERSION);
    profile_write_u16(&header[6], CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE);
    header[8] = CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY;
    header[9] = CONTROLLER_PROFILE_COUNT;
    header[10] = database.fallback_active_profile;
    uint8_t used_count = 0;
    for (const ControllerProfileDatabaseEntry& entry : database.entries) {
        used_count += entry.used ? 1 : 0;
    }
    header[11] = used_count;
    copy_overlap(offset, output, size, 0, header, sizeof(header));

    uint8_t encoded_profile[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    for (uint8_t profile_index = 0;
         profile_index < CONTROLLER_PROFILE_COUNT; ++profile_index) {
        const size_t profile_offset =
            kFallbackOffset +
            profile_index * CONTROLLER_PROFILE_ENCODED_SIZE;
        if (offset < profile_offset + CONTROLLER_PROFILE_ENCODED_SIZE &&
            offset + size > profile_offset) {
            if (!controller_profile_encode(
                    database.fallback_profiles[profile_index],
                    encoded_profile, sizeof(encoded_profile))) {
                return false;
            }
            copy_overlap(offset, output, size, profile_offset,
                         encoded_profile, sizeof(encoded_profile));
        }
    }

    for (uint8_t entry_index = 0;
         entry_index < CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY;
         ++entry_index) {
        const ControllerProfileDatabaseEntry& entry =
            database.entries[entry_index];
        if (!entry.used) {
            continue;
        }
        const size_t entry_offset =
            kEntriesOffset + entry_index * CONTROLLER_PROFILE_DATABASE_ENTRY_SIZE;
        uint8_t entry_header[CONTROLLER_PROFILE_DATABASE_ENTRY_HEADER_SIZE]{};
        if (!controller_identity_encode(entry.identity, entry_header,
                                        CONTROLLER_IDENTITY_ENCODED_SIZE)) {
            return false;
        }
        entry_header[14] = entry.active_profile;
        entry_header[15] = 1;
        copy_overlap(offset, output, size, entry_offset, entry_header,
                     sizeof(entry_header));
        for (uint8_t profile_index = 0;
             profile_index < CONTROLLER_PROFILE_COUNT; ++profile_index) {
            const size_t profile_offset =
                entry_offset + CONTROLLER_PROFILE_DATABASE_ENTRY_HEADER_SIZE +
                profile_index * CONTROLLER_PROFILE_ENCODED_SIZE;
            if (offset < profile_offset + CONTROLLER_PROFILE_ENCODED_SIZE &&
                offset + size > profile_offset) {
                if (!controller_profile_encode(entry.profiles[profile_index],
                                               encoded_profile,
                                               sizeof(encoded_profile))) {
                    return false;
                }
                copy_overlap(offset, output, size, profile_offset,
                             encoded_profile, sizeof(encoded_profile));
            }
        }
    }
    return true;
}

bool controller_profile_database_decode(
    ControllerProfileDatabaseRead read, void* context,
    ControllerProfileDatabase* output) {
    if (read == nullptr || output == nullptr) {
        return false;
    }
    uint8_t header[CONTROLLER_PROFILE_DATABASE_HEADER_SIZE]{};
    if (!read(context, 0, header, sizeof(header)) ||
        memcmp(header, kDatabaseMagic, sizeof(kDatabaseMagic)) != 0 ||
        profile_read_u16(&header[4]) !=
            CONTROLLER_PROFILE_DATABASE_SCHEMA_VERSION ||
        profile_read_u16(&header[6]) != CONTROLLER_PROFILE_DATABASE_ENCODED_SIZE ||
        header[8] != CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY ||
        header[9] != CONTROLLER_PROFILE_COUNT ||
        header[10] >= CONTROLLER_PROFILE_COUNT ||
        header[11] > CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY ||
        !profile_bytes_are_zero(&header[12], sizeof(header) - 12)) {
        return false;
    }

    controller_profile_database_default(output);
    output->fallback_active_profile = header[10];
    uint8_t encoded_profile[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    for (uint8_t profile_index = 0;
         profile_index < CONTROLLER_PROFILE_COUNT; ++profile_index) {
        const size_t profile_offset =
            kFallbackOffset +
            profile_index * CONTROLLER_PROFILE_ENCODED_SIZE;
        if (!read(context, profile_offset, encoded_profile,
                  sizeof(encoded_profile)) ||
            !controller_profile_decode(
                encoded_profile, sizeof(encoded_profile),
                &output->fallback_profiles[profile_index])) {
            controller_profile_database_default(output);
            return false;
        }
    }

    uint8_t decoded_used_count = 0;
    for (uint8_t entry_index = 0;
         entry_index < CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY;
         ++entry_index) {
        const size_t entry_offset =
            kEntriesOffset + entry_index * CONTROLLER_PROFILE_DATABASE_ENTRY_SIZE;
        uint8_t entry_header[CONTROLLER_PROFILE_DATABASE_ENTRY_HEADER_SIZE]{};
        if (!read(context, entry_offset, entry_header,
                  sizeof(entry_header))) {
            controller_profile_database_default(output);
            return false;
        }
        if (entry_header[15] == 0) {
            if (!profile_bytes_are_zero(entry_header, sizeof(entry_header)) ||
                !read_zero_region(
                    read, context,
                    entry_offset + CONTROLLER_PROFILE_DATABASE_ENTRY_HEADER_SIZE,
                    CONTROLLER_PROFILE_COUNT *
                        CONTROLLER_PROFILE_ENCODED_SIZE)) {
                controller_profile_database_default(output);
                return false;
            }
            continue;
        }
        if (entry_header[15] != 1 ||
            entry_header[14] >= CONTROLLER_PROFILE_COUNT) {
            controller_profile_database_default(output);
            return false;
        }
        ControllerProfileDatabaseEntry& entry = output->entries[entry_index];
        if (!controller_identity_decode(entry_header,
                                        CONTROLLER_IDENTITY_ENCODED_SIZE,
                                        &entry.identity) ||
            !entry.identity.stable ||
            controller_identity_is_global(entry.identity)) {
            controller_profile_database_default(output);
            return false;
        }
        entry.used = true;
        entry.active_profile = entry_header[14];
        ++decoded_used_count;
        for (uint8_t profile_index = 0;
             profile_index < CONTROLLER_PROFILE_COUNT; ++profile_index) {
            const size_t profile_offset =
                entry_offset + CONTROLLER_PROFILE_DATABASE_ENTRY_HEADER_SIZE +
                profile_index * CONTROLLER_PROFILE_ENCODED_SIZE;
            if (!read(context, profile_offset, encoded_profile,
                      sizeof(encoded_profile)) ||
                !controller_profile_decode(encoded_profile,
                                           sizeof(encoded_profile),
                                           &entry.profiles[profile_index])) {
                controller_profile_database_default(output);
                return false;
            }
        }
    }
    if (decoded_used_count != header[11] ||
        !controller_profile_database_validate(*output)) {
        controller_profile_database_default(output);
        return false;
    }
    return true;
}

const ControllerProfileDatabaseEntry* controller_profile_database_find(
    const ControllerProfileDatabase& database,
    const ControllerIdentity& identity) {
    if (!identity.stable || controller_identity_is_global(identity)) {
        return nullptr;
    }
    for (const ControllerProfileDatabaseEntry& entry : database.entries) {
        if (entry.used &&
            controller_identity_equal(entry.identity, identity)) {
            return &entry;
        }
    }
    return nullptr;
}

ControllerProfileDatabaseEntry* controller_profile_database_find(
    ControllerProfileDatabase* database,
    const ControllerIdentity& identity) {
    if (database == nullptr) {
        return nullptr;
    }
    return const_cast<ControllerProfileDatabaseEntry*>(
        controller_profile_database_find(*database, identity));
}

ControllerProfileDatabaseResult controller_profile_database_ensure(
    ControllerProfileDatabase* database, const ControllerIdentity& identity,
    ControllerProfileDatabaseEntry** output) {
    if (database == nullptr || output == nullptr || !identity.stable ||
        controller_identity_is_global(identity)) {
        return ControllerProfileDatabaseResult::kInvalidArgument;
    }
    if (ControllerProfileDatabaseEntry* found =
            controller_profile_database_find(database, identity)) {
        *output = found;
        return ControllerProfileDatabaseResult::kOk;
    }
    for (ControllerProfileDatabaseEntry& entry : database->entries) {
        if (!entry.used) {
            entry.active_profile = 0;
            entry.used = true;
            entry.identity = identity;
            for (uint8_t profile_index = 0;
                 profile_index < CONTROLLER_PROFILE_COUNT; ++profile_index) {
                entry.profiles[profile_index] =
                    controller_profile_default(identity, profile_index);
            }
            *output = &entry;
            return ControllerProfileDatabaseResult::kOk;
        }
    }
    return ControllerProfileDatabaseResult::kFull;
}

const ControllerProfile* controller_profile_database_get(
    const ControllerProfileDatabase& database,
    const ControllerIdentity& identity, uint8_t profile_index) {
    if (profile_index >= CONTROLLER_PROFILE_COUNT) {
        return nullptr;
    }
    if (controller_identity_is_global(identity)) {
        return &database.fallback_profiles[profile_index];
    }
    const ControllerProfileDatabaseEntry* entry =
        controller_profile_database_find(database, identity);
    return entry == nullptr ? nullptr : &entry->profiles[profile_index];
}

ControllerProfileDatabaseResult controller_profile_database_set(
    ControllerProfileDatabase* database, const ControllerIdentity& identity,
    uint8_t profile_index, const ControllerProfile& profile) {
    if (database == nullptr || profile_index >= CONTROLLER_PROFILE_COUNT ||
        !controller_profile_validate(profile)) {
        return ControllerProfileDatabaseResult::kInvalidArgument;
    }
    if (controller_identity_is_global(identity)) {
        database->fallback_profiles[profile_index] = profile;
        return ControllerProfileDatabaseResult::kOk;
    }
    ControllerProfileDatabaseEntry* entry = nullptr;
    const ControllerProfileDatabaseResult result =
        controller_profile_database_ensure(database, identity, &entry);
    if (result == ControllerProfileDatabaseResult::kOk) {
        entry->profiles[profile_index] = profile;
    }
    return result;
}

ControllerProfileDatabaseResult controller_profile_database_reset(
    ControllerProfileDatabase* database, const ControllerIdentity& identity,
    uint8_t profile_index) {
    if (database == nullptr ||
        (profile_index != CONTROLLER_PROFILE_ALL &&
         profile_index >= CONTROLLER_PROFILE_COUNT)) {
        return ControllerProfileDatabaseResult::kInvalidArgument;
    }
    if (controller_identity_is_global(identity)) {
        for (uint8_t index = 0; index < CONTROLLER_PROFILE_COUNT; ++index) {
            if (profile_index == CONTROLLER_PROFILE_ALL ||
                profile_index == index) {
                database->fallback_profiles[index] =
                    controller_profile_default(identity, index);
            }
        }
        return ControllerProfileDatabaseResult::kOk;
    }
    ControllerProfileDatabaseEntry* entry = nullptr;
    const ControllerProfileDatabaseResult result =
        controller_profile_database_ensure(database, identity, &entry);
    if (result != ControllerProfileDatabaseResult::kOk) {
        return result;
    }
    for (uint8_t index = 0; index < CONTROLLER_PROFILE_COUNT; ++index) {
        if (profile_index == CONTROLLER_PROFILE_ALL ||
            profile_index == index) {
            entry->profiles[index] = controller_profile_default(identity, index);
        }
    }
    return ControllerProfileDatabaseResult::kOk;
}

ControllerProfileDatabaseResult controller_profile_database_activate(
    ControllerProfileDatabase* database, const ControllerIdentity& identity,
    uint8_t profile_index) {
    if (database == nullptr || profile_index >= CONTROLLER_PROFILE_COUNT) {
        return ControllerProfileDatabaseResult::kInvalidArgument;
    }
    if (controller_identity_is_global(identity)) {
        database->fallback_active_profile = profile_index;
        return ControllerProfileDatabaseResult::kOk;
    }
    ControllerProfileDatabaseEntry* entry = nullptr;
    const ControllerProfileDatabaseResult result =
        controller_profile_database_ensure(database, identity, &entry);
    if (result == ControllerProfileDatabaseResult::kOk) {
        entry->active_profile = profile_index;
    }
    return result;
}
