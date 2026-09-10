#include "profile/controller_profile.h"
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
constexpr uint16_t kLegacyDefaultDigitalThreshold = 0x8000;
constexpr uint32_t kLogicalControlMask =
    (1u << CONTROLLER_PROFILE_LOGICAL_CONTROL_COUNT) - 1u;

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

bool valid_control_output(uint8_t output) {
    return output < CONTROLLER_PROFILE_FIRST_EXTRA_CONTROL ||
           output == CONTROLLER_PROFILE_NO_BUTTON;
}

bool valid_source_control(uint8_t control) {
    return control < CONTROLLER_PROFILE_LOGICAL_CONTROL_COUNT ||
           control == CONTROLLER_PROFILE_NO_BUTTON;
}

bool valid_swing_action(uint8_t button, uint8_t macro, uint8_t modifier) {
    return valid_button(button) && valid_source_control(modifier) &&
           (macro == CONTROLLER_PROFILE_NO_BUTTON ||
            (macro < CONTROLLER_PROFILE_MACRO_COUNT &&
             button == CONTROLLER_PROFILE_NO_BUTTON));
}

bool valid_turbo_settings(const ControllerProfileTurboSettings& settings) {
    return settings.rate_hz >= CONTROLLER_PROFILE_TURBO_RATE_MIN &&
           settings.rate_hz <= CONTROLLER_PROFILE_TURBO_RATE_MAX &&
           settings.duty_percent >= CONTROLLER_PROFILE_TURBO_DUTY_MIN &&
           settings.duty_percent <= CONTROLLER_PROFILE_TURBO_DUTY_MAX &&
           settings.burst_count >= CONTROLLER_PROFILE_TURBO_BURST_MIN;
}

bool valid_macro_step(const ControllerProfileMacroStep& step) {
    if ((step.override_flags & ~kMacroOverrideMask) != 0 ||
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

size_t sparse_macro_step_size(const ControllerProfileMacroStep& step) {
    size_t size = 3;
    size += (step.override_flags & kControllerProfileOverrideButtons) != 0
                ? 2
                : 0;
    size += (step.override_flags & kControllerProfileOverrideLeftStick) != 0
                ? 4
                : 0;
    size += (step.override_flags & kControllerProfileOverrideRightStick) != 0
                ? 4
                : 0;
    size += (step.override_flags & kControllerProfileOverrideLeftTrigger) != 0
                ? 2
                : 0;
    size += (step.override_flags & kControllerProfileOverrideRightTrigger) != 0
                ? 2
                : 0;
    return size;
}

bool encode_sparse_macro_step(
    const ControllerProfileMacroStep& step, uint8_t* output,
    size_t output_size, size_t* encoded_size) {
    const size_t required = sparse_macro_step_size(step);
    if (!valid_macro_step(step) || output == nullptr ||
        encoded_size == nullptr || required > output_size) {
        return false;
    }
    size_t offset = 0;
    output[offset++] = step.override_flags;
    profile_write_u16(&output[offset], step.duration_ms);
    offset += 2;
    if ((step.override_flags & kControllerProfileOverrideButtons) != 0) {
        profile_write_u16(&output[offset], step.output_button_mask);
        offset += 2;
    }
    if ((step.override_flags & kControllerProfileOverrideLeftStick) != 0) {
        profile_write_i16(&output[offset], step.left_stick_x);
        profile_write_i16(&output[offset + 2], step.left_stick_y);
        offset += 4;
    }
    if ((step.override_flags & kControllerProfileOverrideRightStick) != 0) {
        profile_write_i16(&output[offset], step.right_stick_x);
        profile_write_i16(&output[offset + 2], step.right_stick_y);
        offset += 4;
    }
    if ((step.override_flags & kControllerProfileOverrideLeftTrigger) != 0) {
        profile_write_u16(&output[offset], step.left_trigger);
        offset += 2;
    }
    if ((step.override_flags & kControllerProfileOverrideRightTrigger) != 0) {
        profile_write_u16(&output[offset], step.right_trigger);
        offset += 2;
    }
    *encoded_size = offset;
    return true;
}

bool decode_sparse_macro_step(
    const uint8_t* input, size_t input_size,
    ControllerProfileMacroStep* output, size_t* decoded_size) {
    if (input == nullptr || output == nullptr || decoded_size == nullptr ||
        input_size < 3) {
        return false;
    }
    ControllerProfileMacroStep step{};
    step.override_flags = input[0];
    step.duration_ms = profile_read_u16(&input[1]);
    if (!valid_macro_step(step)) {
        return false;
    }
    const size_t required = sparse_macro_step_size(step);
    if (required > input_size) {
        return false;
    }
    size_t offset = 3;
    if ((step.override_flags & kControllerProfileOverrideButtons) != 0) {
        step.output_button_mask = profile_read_u16(&input[offset]);
        offset += 2;
    }
    if ((step.override_flags & kControllerProfileOverrideLeftStick) != 0) {
        step.left_stick_x = profile_read_i16(&input[offset]);
        step.left_stick_y = profile_read_i16(&input[offset + 2]);
        offset += 4;
    }
    if ((step.override_flags & kControllerProfileOverrideRightStick) != 0) {
        step.right_stick_x = profile_read_i16(&input[offset]);
        step.right_stick_y = profile_read_i16(&input[offset + 2]);
        offset += 4;
    }
    if ((step.override_flags & kControllerProfileOverrideLeftTrigger) != 0) {
        step.left_trigger = profile_read_u16(&input[offset]);
        offset += 2;
    }
    if ((step.override_flags & kControllerProfileOverrideRightTrigger) != 0) {
        step.right_trigger = profile_read_u16(&input[offset]);
        offset += 2;
    }
    if (!valid_macro_step(step)) {
        return false;
    }
    *output = step;
    *decoded_size = offset;
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
    for (uint8_t index = 0; index < 2; ++index) {
        ControllerProfileTriggerConfiguration& trigger =
            profile.triggers[index];
        trigger.lower_deadzone = 0;
        trigger.upper_saturation = UINT16_MAX;
        trigger.curve_q8_8 = 256;
        trigger.digital_threshold =
            CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD;
        trigger.output = static_cast<uint8_t>(
            CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL + index);
    }
    profile.weak_rumble_scale = UINT8_MAX;
    profile.strong_rumble_scale = UINT8_MAX;
    profile.confirmation_policy =
        ControllerProfileConfirmationPolicy::kRumbleAndLed;
    profile.switching_chord = 0;
    profile.motion_toggle_chord = 0;
    for (ControllerProfileMacro& macro : profile.macros) {
        macro = {};
        macro.cancel_control = CONTROLLER_PROFILE_NO_BUTTON;
    }
    profile.macro_step_count = 0;
    for (ControllerProfileMacroStep& step : profile.macro_steps) {
        step = {};
    }
    return profile;
}

bool controller_profile_validate(const ControllerProfile& profile) {
    for (uint8_t output : profile.button_map) {
        if (!valid_control_output(output)) {
            return false;
        }
    }
    for (uint8_t output : profile.extra_button_map) {
        if (!valid_control_output(output)) {
            return false;
        }
    }
    if (!valid_source_control(profile.shortcuts.modifier) ||
        !valid_swing_action(profile.swing.button, profile.swing.macro,
                            profile.swing.modifier) ||
        !valid_swing_action(profile.nunchuk_swing.button,
                            profile.nunchuk_swing.macro,
                            profile.nunchuk_swing.modifier) ||
        !valid_swing_action(profile.combined_swing.button,
                            profile.combined_swing.macro,
                            profile.combined_swing.modifier) ||
        profile.swing.sensitivity > 2 ||
        profile.nunchuk_swing.sensitivity > 2 ||
        profile.combination_window_ms < 30 ||
        profile.combination_window_ms > 200 ||
        !valid_source_control(profile.shift.modifier) ||
        static_cast<uint8_t>(profile.shift.mode) >
            static_cast<uint8_t>(ControllerProfileShiftMode::kToggle) ||
        (profile.shift.mode != ControllerProfileShiftMode::kOff &&
         profile.shift.modifier == CONTROLLER_PROFILE_NO_BUTTON) ||
        !valid_turbo_settings(profile.turbo_defaults)) {
        return false;
    }
    uint16_t selectors = 0;
    for (uint8_t selector : profile.shortcuts.selectors) {
        if (selector == CONTROLLER_PROFILE_NO_BUTTON) {
            continue;
        }
        if (!(selector < 4 || (selector >= 12 && selector < 16)) ||
            selector == profile.shortcuts.modifier ||
            profile.shortcuts.modifier == CONTROLLER_PROFILE_NO_BUTTON ||
            (selectors & (1u << selector)) != 0) {
            return false;
        }
        selectors |= static_cast<uint16_t>(1u << selector);
    }
    for (uint8_t output : profile.shift.button_map) {
        if (!valid_button(output)) {
            return false;
        }
    }
    for (uint8_t output : profile.shift.extra_button_map) {
        if (!valid_button(output)) {
            return false;
        }
    }
    for (uint8_t button = 0;
         button < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT; ++button) {
        if ((profile.turbo_override_mask & (1u << button)) != 0 &&
            !valid_turbo_settings(profile.turbo_overrides[button])) {
            return false;
        }
    }
    for (const ControllerProfileStickConfiguration& stick : profile.sticks) {
        if (stick.inner_deadzone >= stick.outer_saturation ||
            stick.outer_saturation > 32767 || stick.curve_q8_8 == 0) {
            return false;
        }
    }
    bool routed_triggers[2]{};
    for (const ControllerProfileTriggerConfiguration& trigger :
         profile.triggers) {
        if (trigger.lower_deadzone >= trigger.upper_saturation ||
            trigger.curve_q8_8 == 0 ||
            !valid_control_output(trigger.output)) {
            return false;
        }
        if (trigger.output >= CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL &&
            trigger.output <= CONTROLLER_PROFILE_RIGHT_TRIGGER_CONTROL) {
            const uint8_t target = static_cast<uint8_t>(
                trigger.output - CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL);
            if (routed_triggers[target]) {
                return false;
            }
            routed_triggers[target] = true;
        }
    }
    if (static_cast<uint8_t>(profile.confirmation_policy) >
            static_cast<uint8_t>(
                ControllerProfileConfirmationPolicy::kRumbleAndLed) ||
        (profile.switching_chord & ~kLogicalControlMask) != 0 ||
        (profile.motion_toggle_chord & ~kLogicalControlMask) != 0 ||
        profile.macro_step_count > CONTROLLER_PROFILE_MACRO_STEP_CAPACITY) {
        return false;
    }
    uint8_t expected_first_step = 0;
    size_t encoded_macro_size = 0;
    uint32_t trigger_masks[CONTROLLER_PROFILE_MACRO_COUNT]{};
    for (uint8_t macro_index = 0;
         macro_index < CONTROLLER_PROFILE_MACRO_COUNT; ++macro_index) {
        const ControllerProfileMacro& macro =
            profile.macros[macro_index];
        if ((macro.trigger_mask & ~kLogicalControlMask) != 0 ||
            !valid_source_control(macro.cancel_control) ||
            static_cast<uint8_t>(macro.mode) >
                static_cast<uint8_t>(ControllerProfileMacroMode::kRepeat) ||
            macro.repeat_count == 0 ||
            macro.first_step != expected_first_step ||
            macro.step_count > CONTROLLER_PROFILE_MACRO_STEPS_PER_MACRO ||
            macro.step_count >
                profile.macro_step_count - expected_first_step) {
            return false;
        }
        for (uint8_t previous = 0; previous < macro_index; ++previous) {
            if (macro.trigger_mask != 0 &&
                trigger_masks[previous] == macro.trigger_mask) {
                return false;
            }
        }
        trigger_masks[macro_index] = macro.trigger_mask;
        uint32_t duration_ms = 0;
        for (uint8_t step = 0; step < macro.step_count; ++step) {
            const ControllerProfileMacroStep& value =
                profile.macro_steps[expected_first_step + step];
            if (!valid_macro_step(value)) {
                return false;
            }
            encoded_macro_size += sparse_macro_step_size(value);
            duration_ms += value.duration_ms;
        }
        if ((profile.swing.macro == macro_index ||
             profile.nunchuk_swing.macro == macro_index ||
             profile.combined_swing.macro == macro_index) &&
            (macro.step_count == 0 || duration_ms == 0)) {
            return false;
        }
        if (macro.trigger_mask != 0 && macro.step_count != 0 &&
            macro.mode != ControllerProfileMacroMode::kOnce &&
            duration_ms == 0) {
            return false;
        }
        expected_first_step =
            static_cast<uint8_t>(expected_first_step + macro.step_count);
    }
    if (expected_first_step != profile.macro_step_count ||
        encoded_macro_size > CONTROLLER_PROFILE_MACRO_STREAM_SIZE) {
        return false;
    }
    for (uint8_t index = profile.macro_step_count;
         index < CONTROLLER_PROFILE_MACRO_STEP_CAPACITY; ++index) {
        const ControllerProfileMacroStep& step = profile.macro_steps[index];
        if (step.override_flags != 0 || step.duration_ms != 0 ||
            step.output_button_mask != 0 || step.left_stick_x != 0 ||
            step.left_stick_y != 0 || step.right_stick_x != 0 ||
            step.right_stick_y != 0 || step.left_trigger != 0 ||
            step.right_trigger != 0) {
            return false;
        }
    }
    for (ControllerProfileTurboMode mode : profile.turbo_modes) {
        if (static_cast<uint8_t>(mode) >
            static_cast<uint8_t>(ControllerProfileTurboMode::kBurst)) {
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
        encoded[8] = trigger.output;
    }
    output[72] = profile.weak_rumble_scale;
    output[73] = profile.strong_rumble_scale;
    output[74] = static_cast<uint8_t>(profile.confirmation_policy);
    output[75] = static_cast<uint8_t>(
        ((profile.switching_chord >> 16) & 0x03u) |
        (((profile.motion_toggle_chord >> 16) & 0x03u) << 4));
    profile_write_u16(
        &output[76], static_cast<uint16_t>(profile.switching_chord));
    profile_write_u16(
        &output[78], static_cast<uint16_t>(profile.motion_toggle_chord));
    for (uint8_t index = 0;
         index < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT; ++index) {
        output[80 + index] =
            static_cast<uint8_t>(profile.turbo_modes[index]);
    }

    size_t stream_offset = 0;
    for (uint8_t macro_index = 0;
         macro_index < CONTROLLER_PROFILE_MACRO_COUNT; ++macro_index) {
        const ControllerProfileMacro& macro =
            profile.macros[macro_index];
        uint8_t* descriptor =
            &output[96 + macro_index *
                              CONTROLLER_PROFILE_MACRO_DESCRIPTOR_SIZE];
        profile_write_u16(
            descriptor, static_cast<uint16_t>(macro.trigger_mask));
        const uint8_t cancel =
            macro.cancel_control == CONTROLLER_PROFILE_NO_BUTTON
                ? 0x1fu
                : macro.cancel_control;
        descriptor[2] = static_cast<uint8_t>(
            ((macro.trigger_mask >> 16) & 0x03u) |
            (cancel << 2));
        descriptor[3] = static_cast<uint8_t>(stream_offset);
        descriptor[4] = macro.step_count;
        const size_t macro_start = stream_offset;
        for (uint8_t step_index = 0;
             step_index < macro.step_count; ++step_index) {
            size_t encoded_size = 0;
            if (!encode_sparse_macro_step(
                    profile.macro_steps[macro.first_step + step_index],
                    &output[120 + stream_offset],
                    CONTROLLER_PROFILE_MACRO_STREAM_SIZE - stream_offset,
                    &encoded_size)) {
                return false;
            }
            stream_offset += encoded_size;
        }
        descriptor[5] =
            static_cast<uint8_t>(stream_offset - macro_start);
        output[336 + macro_index * 2] = static_cast<uint8_t>(macro.mode);
        output[337 + macro_index * 2] = macro.repeat_count;
        output[358 + macro_index] = static_cast<uint8_t>(
            macro.trigger_mask >> CONTROLLER_PROFILE_FIRST_EXTRA_CONTROL);
    }
    output[256] = profile.shortcuts.modifier;
    memcpy(&output[257], profile.shortcuts.selectors,
           CONTROLLER_PROFILE_COUNT);
    output[265] = static_cast<uint8_t>(profile.shift.mode);
    output[266] = profile.shift.modifier;
    memcpy(&output[267], profile.shift.button_map,
           CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT);
    output[283] = profile.turbo_defaults.rate_hz;
    output[284] = profile.turbo_defaults.duty_percent;
    output[285] = profile.turbo_defaults.burst_count;
    profile_write_u16(&output[286], profile.turbo_override_mask);
    size_t settings_offset = 288;
    for (uint8_t button = 0;
         button < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT; ++button) {
        if ((profile.turbo_override_mask & (1u << button)) != 0) {
            const ControllerProfileTurboSettings& settings =
                profile.turbo_overrides[button];
            output[settings_offset++] = settings.rate_hz;
            output[settings_offset++] = settings.duty_percent;
            output[settings_offset++] = settings.burst_count;
        }
    }
    memcpy(&output[344], profile.extra_button_map,
           CONTROLLER_PROFILE_EXTRA_BUTTON_COUNT);
    memcpy(&output[351], profile.shift.extra_button_map,
           CONTROLLER_PROFILE_EXTRA_BUTTON_COUNT);
    output[362] = static_cast<uint8_t>(
        profile.switching_chord >> CONTROLLER_PROFILE_FIRST_EXTRA_CONTROL);
    output[363] = static_cast<uint8_t>(
        profile.motion_toggle_chord >> CONTROLLER_PROFILE_FIRST_EXTRA_CONTROL);
    output[364] = profile.swing.button;
    output[365] = profile.swing.sensitivity;
    output[366] = profile.swing.modifier;
    output[367] = profile.swing.macro;
    output[368] = profile.nunchuk_swing.button;
    output[369] = profile.nunchuk_swing.sensitivity;
    output[370] = profile.nunchuk_swing.modifier;
    output[371] = profile.nunchuk_swing.macro;
    output[372] = profile.combined_swing.button;
    output[373] = profile.combined_swing.macro;
    output[374] = profile.combined_swing.modifier;
    output[375] = profile.combination_window_ms;
    return stream_offset <= CONTROLLER_PROFILE_MACRO_STREAM_SIZE;
}

bool controller_profile_decode(const uint8_t* input, size_t input_size,
                               ControllerProfile* output) {
    if (input == nullptr || output == nullptr ||
        (input_size != CONTROLLER_PROFILE_LEGACY_ENCODED_SIZE &&
         input_size != CONTROLLER_PROFILE_ENCODED_SIZE)) {
        return false;
    }
    const uint16_t schema_version = profile_read_u16(&input[0]);
    const size_t expected_size =
        schema_version >= CONTROLLER_PROFILE_EXPANDED_SCHEMA_VERSION
            ? CONTROLLER_PROFILE_ENCODED_SIZE
            : CONTROLLER_PROFILE_LEGACY_ENCODED_SIZE;
    if (schema_version < CONTROLLER_PROFILE_LEGACY_SCHEMA_VERSION ||
        schema_version > CONTROLLER_PROFILE_SCHEMA_VERSION ||
        input_size != expected_size ||
        profile_read_u16(&input[2]) != expected_size ||
        !profile_bytes_are_zero(&input[31], 5) ||
        !profile_bytes_are_zero(&input[47], 5)) {
        return false;
    }
    const bool has_control_mapping =
        schema_version >= CONTROLLER_PROFILE_CONTROL_MAPPING_SCHEMA_VERSION;
    const bool has_action_controls =
        schema_version >= CONTROLLER_PROFILE_ACTION_CONTROL_SCHEMA_VERSION;
    const bool sparse_macros =
        schema_version >= CONTROLLER_PROFILE_SPARSE_MACRO_SCHEMA_VERSION;
    const bool has_expanded_settings =
        schema_version >= CONTROLLER_PROFILE_EXPANDED_SCHEMA_VERSION;
    const bool has_extra_controls =
        schema_version >= CONTROLLER_PROFILE_EXTRA_CONTROL_SCHEMA_VERSION;
    const bool has_swing =
        schema_version >= CONTROLLER_PROFILE_SWING_SCHEMA_VERSION;
    const bool has_combined_swing =
        schema_version >= CONTROLLER_PROFILE_SCHEMA_VERSION;
    if ((has_control_mapping
             ? input[61] != 0 || input[71] != 0
             : !profile_bytes_are_zero(&input[60], 2) ||
                   !profile_bytes_are_zero(&input[70], 2)) ||
        (sparse_macros
             ? (input[75] & 0xccu) != 0
             : (has_action_controls ? (input[75] & 0xc0u) != 0
                                    : input[75] != 0)) ||
        (!has_control_mapping &&
         (input[81] != 0 ||
          !profile_bytes_are_zero(&input[98], 2))) ||
        (!sparse_macros &&
         !profile_bytes_are_zero(&input[252], 4))) {
        return false;
    }

    ControllerProfile profile{};
    memcpy(profile.button_map, &input[4],
           CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT);
    for (uint8_t index = 0; index < 2; ++index) {
        ControllerProfileStickConfiguration& stick = profile.sticks[index];
        const uint8_t* encoded = &input[20 + index * 16];
        if ((encoded[10] & ~(kStickInvertX | kStickInvertY)) != 0 ||
            !profile_bytes_are_zero(&encoded[11], 5)) {
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
        trigger.output =
            has_control_mapping
                ? encoded[8]
                : static_cast<uint8_t>(
                      CONTROLLER_PROFILE_LEFT_TRIGGER_CONTROL + index);
        if (schema_version == CONTROLLER_PROFILE_LEGACY_SCHEMA_VERSION &&
            trigger.digital_threshold == kLegacyDefaultDigitalThreshold) {
            trigger.digital_threshold =
                CONTROLLER_PROFILE_DEFAULT_DIGITAL_THRESHOLD;
        }
    }
    profile.weak_rumble_scale = input[72];
    profile.strong_rumble_scale = input[73];
    profile.confirmation_policy =
        static_cast<ControllerProfileConfirmationPolicy>(input[74]);

    if (sparse_macros) {
        profile.switching_chord =
            profile_read_u16(&input[76]) |
            (static_cast<uint32_t>(input[75] & 0x03u) << 16);
        profile.motion_toggle_chord =
            profile_read_u16(&input[78]) |
            (static_cast<uint32_t>((input[75] >> 4) & 0x03u) << 16);
        for (uint8_t index = 0;
             index < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT; ++index) {
            profile.turbo_modes[index] =
                static_cast<ControllerProfileTurboMode>(input[80 + index]);
        }

        size_t expected_stream_offset = 0;
        uint8_t decoded_step_count = 0;
        for (uint8_t macro_index = 0;
             macro_index < CONTROLLER_PROFILE_MACRO_COUNT; ++macro_index) {
            const uint8_t* descriptor =
                &input[96 + macro_index *
                                 CONTROLLER_PROFILE_MACRO_DESCRIPTOR_SIZE];
            if ((descriptor[2] & 0x80u) != 0 ||
                descriptor[3] != expected_stream_offset ||
                descriptor[4] >
                    CONTROLLER_PROFILE_MACRO_STEPS_PER_MACRO ||
                expected_stream_offset + descriptor[5] >
                    CONTROLLER_PROFILE_MACRO_STREAM_SIZE ||
                decoded_step_count + descriptor[4] >
                    CONTROLLER_PROFILE_MACRO_STEP_CAPACITY) {
                return false;
            }
            ControllerProfileMacro& macro = profile.macros[macro_index];
            macro.trigger_mask =
                profile_read_u16(descriptor) |
                (static_cast<uint32_t>(descriptor[2] & 0x03u) << 16);
            const uint8_t cancel =
                static_cast<uint8_t>((descriptor[2] >> 2) & 0x1fu);
            if (cancel >= (has_extra_controls
                               ? CONTROLLER_PROFILE_LOGICAL_CONTROL_COUNT
                               : CONTROLLER_PROFILE_FIRST_EXTRA_CONTROL) &&
                cancel != 0x1fu) {
                return false;
            }
            macro.cancel_control =
                cancel == 0x1fu ? CONTROLLER_PROFILE_NO_BUTTON : cancel;
            macro.first_step = decoded_step_count;
            macro.step_count = descriptor[4];
            if (has_expanded_settings) {
                macro.mode = static_cast<ControllerProfileMacroMode>(
                    input[336 + macro_index * 2]);
                macro.repeat_count = input[337 + macro_index * 2];
            }

            size_t consumed = 0;
            for (uint8_t step_index = 0;
                 step_index < macro.step_count; ++step_index) {
                size_t step_size = 0;
                if (!decode_sparse_macro_step(
                        &input[120 + expected_stream_offset + consumed],
                        descriptor[5] - consumed,
                        &profile.macro_steps[decoded_step_count],
                        &step_size)) {
                    return false;
                }
                consumed += step_size;
                ++decoded_step_count;
            }
            if (consumed != descriptor[5]) {
                return false;
            }
            expected_stream_offset += consumed;
        }
        if (!profile_bytes_are_zero(
                &input[120 + expected_stream_offset],
                CONTROLLER_PROFILE_MACRO_STREAM_SIZE -
                    expected_stream_offset)) {
            return false;
        }
        profile.macro_step_count = decoded_step_count;
    } else {
        profile.switching_chord = profile_read_u16(&input[76]);
        profile.motion_toggle_chord =
            has_control_mapping ? profile_read_u16(&input[98]) : 0;
        ControllerProfileMacro& macro = profile.macros[0];
        if (has_control_mapping) {
            macro.trigger_mask = profile_read_u16(&input[78]);
            macro.cancel_control = input[81];
        } else {
            if (!valid_button(input[78])) {
                return false;
            }
            macro.trigger_mask =
                input[78] < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT
                    ? static_cast<uint32_t>(1u << input[78])
                    : 0;
            macro.cancel_control = input[79];
        }
        if (has_action_controls) {
            profile.switching_chord |=
                static_cast<uint32_t>(input[75] & 0x03u) << 16;
            macro.trigger_mask |=
                static_cast<uint32_t>((input[75] >> 2) & 0x03u) << 16;
            profile.motion_toggle_chord |=
                static_cast<uint32_t>((input[75] >> 4) & 0x03u) << 16;
        }
        const uint8_t legacy_step_count = input[80];
        if (legacy_step_count == 0 ||
            legacy_step_count >
                CONTROLLER_PROFILE_LEGACY_MACRO_STEP_CAPACITY) {
            return false;
        }
        for (uint8_t index = 0;
             index < CONTROLLER_PROFILE_LEGACY_MACRO_STEP_CAPACITY;
             ++index) {
            const uint8_t* encoded = &input[100 + index * 19];
            const bool must_end = index >= legacy_step_count - 1;
            if (must_end) {
                if (encoded[0] != 1 ||
                    !profile_bytes_are_zero(&encoded[1], 18)) {
                    return false;
                }
                continue;
            }
            if (encoded[0] != 0 || encoded[18] != 0) {
                return false;
            }
            ControllerProfileMacroStep& step = profile.macro_steps[index];
            step.override_flags = encoded[1];
            step.duration_ms = profile_read_u16(&encoded[2]);
            step.output_button_mask = profile_read_u16(&encoded[4]);
            step.left_stick_x = profile_read_i16(&encoded[6]);
            step.left_stick_y = profile_read_i16(&encoded[8]);
            step.right_stick_x = profile_read_i16(&encoded[10]);
            step.right_stick_y = profile_read_i16(&encoded[12]);
            step.left_trigger = profile_read_u16(&encoded[14]);
            step.right_trigger = profile_read_u16(&encoded[16]);
            if (!valid_macro_step(step)) {
                return false;
            }
        }
        macro.first_step = 0;
        macro.step_count = static_cast<uint8_t>(legacy_step_count - 1u);
        profile.macro_step_count = macro.step_count;
        for (uint8_t index = 1; index < CONTROLLER_PROFILE_MACRO_COUNT; ++index) {
            profile.macros[index].first_step = profile.macro_step_count;
        }
        for (uint8_t index = 0;
             index < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT; ++index) {
            profile.turbo_modes[index] =
                static_cast<ControllerProfileTurboMode>(input[82 + index]);
        }
    }
    if (has_expanded_settings) {
        profile.shortcuts.modifier = input[256];
        memcpy(profile.shortcuts.selectors, &input[257],
               CONTROLLER_PROFILE_COUNT);
        profile.shift.mode = static_cast<ControllerProfileShiftMode>(input[265]);
        profile.shift.modifier = input[266];
        memcpy(profile.shift.button_map, &input[267],
               CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT);
        profile.turbo_defaults = {input[283], input[284], input[285]};
        profile.turbo_override_mask = profile_read_u16(&input[286]);
        size_t settings_offset = 288;
        for (uint8_t button = 0;
             button < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT; ++button) {
            if ((profile.turbo_override_mask & (1u << button)) != 0) {
                profile.turbo_overrides[button] = {
                    input[settings_offset], input[settings_offset + 1],
                    input[settings_offset + 2]};
                settings_offset += 3;
            }
        }
        if (!profile_bytes_are_zero(
                &input[settings_offset], 336 - settings_offset)) {
            return false;
        }
        if (!has_extra_controls &&
            (!profile_bytes_are_zero(&input[344], 40) ||
             !valid_control_output(profile.shortcuts.modifier) ||
             !valid_control_output(profile.shift.modifier))) {
            return false;
        }
    } else {
        for (ControllerProfileTurboMode mode : profile.turbo_modes) {
            if (static_cast<uint8_t>(mode) >
                static_cast<uint8_t>(ControllerProfileTurboMode::kAutoBurst)) {
                return false;
            }
        }
    }
    if (has_extra_controls) {
        memcpy(profile.extra_button_map, &input[344],
               CONTROLLER_PROFILE_EXTRA_BUTTON_COUNT);
        memcpy(profile.shift.extra_button_map, &input[351],
               CONTROLLER_PROFILE_EXTRA_BUTTON_COUNT);
        for (size_t index = 358; index < 364; ++index) {
            if ((input[index] & 0x80u) != 0) {
                return false;
            }
        }
        const size_t reserved_offset =
            has_combined_swing ? 376 : (has_swing ? 367 : 364);
        if (!profile_bytes_are_zero(
                &input[reserved_offset], expected_size - reserved_offset)) {
            return false;
        }
        for (uint8_t index = 0; index < CONTROLLER_PROFILE_MACRO_COUNT; ++index) {
            profile.macros[index].trigger_mask |=
                static_cast<uint32_t>(input[358 + index])
                << CONTROLLER_PROFILE_FIRST_EXTRA_CONTROL;
        }
        profile.switching_chord |= static_cast<uint32_t>(input[362])
                                   << CONTROLLER_PROFILE_FIRST_EXTRA_CONTROL;
        profile.motion_toggle_chord |= static_cast<uint32_t>(input[363])
                                       << CONTROLLER_PROFILE_FIRST_EXTRA_CONTROL;
    } else {
        for (const ControllerProfileMacro& macro : profile.macros) {
            if (!valid_control_output(macro.cancel_control)) {
                return false;
            }
        }
    }
    if (has_swing) {
        profile.swing = {input[364], input[365], input[366]};
    }
    if (has_combined_swing) {
        profile.swing.macro = input[367];
        profile.nunchuk_swing = {
            input[368], input[369], input[370], input[371]};
        profile.combined_swing = {input[372], input[373], input[374]};
        profile.combination_window_ms = input[375];
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
    if (!read(context, 0, header, sizeof(header))) {
        return false;
    }
    const uint16_t schema_version = profile_read_u16(&header[4]);
    const bool legacy =
        schema_version < CONTROLLER_PROFILE_DATABASE_SCHEMA_VERSION;
    const uint8_t profile_count = header[9];
    const size_t profile_size = legacy
        ? CONTROLLER_PROFILE_LEGACY_ENCODED_SIZE
        : CONTROLLER_PROFILE_ENCODED_SIZE;
    const size_t entry_size =
        CONTROLLER_PROFILE_DATABASE_ENTRY_HEADER_SIZE +
        profile_count * profile_size;
    const size_t entries_offset =
        kFallbackOffset + profile_count * profile_size;
    const size_t encoded_size =
        entries_offset + CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY * entry_size;
    if (memcmp(header, kDatabaseMagic, sizeof(kDatabaseMagic)) != 0 ||
        schema_version < CONTROLLER_PROFILE_DATABASE_LEGACY_SCHEMA_VERSION ||
        schema_version > CONTROLLER_PROFILE_DATABASE_SCHEMA_VERSION ||
        profile_read_u16(&header[6]) != encoded_size ||
        header[8] != CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY ||
        (profile_count != CONTROLLER_PROFILE_COUNT &&
         !(schema_version == CONTROLLER_PROFILE_DATABASE_LEGACY_SCHEMA_VERSION &&
           profile_count == 4)) ||
        header[10] >= profile_count ||
        header[11] > CONTROLLER_PROFILE_STABLE_IDENTITY_CAPACITY ||
        !profile_bytes_are_zero(&header[12], sizeof(header) - 12)) {
        return false;
    }

    controller_profile_database_default(output);
    output->fallback_active_profile = header[10];
    uint8_t encoded_profile[CONTROLLER_PROFILE_ENCODED_SIZE]{};
    for (uint8_t profile_index = 0;
         profile_index < profile_count; ++profile_index) {
        const size_t profile_offset =
            kFallbackOffset +
            profile_index * profile_size;
        if (!read(context, profile_offset, encoded_profile,
                  profile_size) ||
            !controller_profile_decode(
                encoded_profile, profile_size,
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
            entries_offset + entry_index * entry_size;
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
                    profile_count * profile_size)) {
                controller_profile_database_default(output);
                return false;
            }
            continue;
        }
        if (entry_header[15] != 1 ||
            entry_header[14] >= profile_count) {
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
             profile_index < profile_count; ++profile_index) {
            const size_t profile_offset =
                entry_offset + CONTROLLER_PROFILE_DATABASE_ENTRY_HEADER_SIZE +
                profile_index * profile_size;
            if (!read(context, profile_offset, encoded_profile,
                      profile_size) ||
                !controller_profile_decode(encoded_profile,
                                           profile_size,
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
