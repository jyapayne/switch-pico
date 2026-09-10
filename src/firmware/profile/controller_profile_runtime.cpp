#include "profile/controller_profile_runtime.h"
#include "configuration/configuration_service.h"
#include "core/controller_identity.h"
#include "profile/controller_synthetic_input.h"
#include "profile/profile_service.h"

namespace {

struct ControllerProfileRuntimeContext {
    bool active = false;
    bool connection_seen = false;
    uint32_t connection_generation = 0;
    ControllerIdentity identity{};
    uint32_t database_generation = 0;
    uint8_t active_profile_index = 0;
    bool profile_snapshot_valid = false;
    ControllerProfile profile{};
    ControllerSyntheticInputContext synthetic{};
    WiiSwingGestures swing{};
    bool runtime_generations_initialized = false;
    AdapterUsbMode output_mode = AdapterUsbMode::kSwitchProbe;
    uint32_t configuration_reset_generation = 0;
    uint32_t previous_hotkey_control_mask = 0;
    uint32_t held_hotkey_chord = 0;
    uint32_t held_hotkey_release_mask = 0;
    uint32_t held_shortcut_selectors = 0;
    bool activation_requested = true;
    uint8_t target_profile_index = 0;
    uint32_t activation_transaction_id = 0;
    bool profile_change_pending = false;
    ControllerProfileRuntimeProfileChangeEvent pending_profile_change{};
    bool initial_profile_indication_resolved = false;
    bool initial_profile_indication_pending = false;
    ControllerProfileRuntimeProfileChangeEvent
        pending_initial_profile_indication{};
};

ControllerProfileRuntimeContext
    g_contexts[CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT]{};
ControllerProfile g_default_profile{};
ControllerProfileTransformResult g_neutral_output{};
bool g_initialized = false;
uint32_t g_next_activation_sequence = 1;

uint32_t next_activation_transaction_id() {
    const uint32_t transaction_id =
        0x80000000u | g_next_activation_sequence;
    g_next_activation_sequence =
        g_next_activation_sequence == 0x7fffffffu
            ? 1u
            : g_next_activation_sequence + 1u;
    return transaction_id;
}

uint32_t effective_switching_chord(const ControllerProfile& profile) {
    return profile.switching_chord == 0
               ? CONTROLLER_PROFILE_DEFAULT_SWITCHING_CHORD
               : profile.switching_chord;
}

uint32_t effective_motion_toggle_chord(
    const ControllerProfile& profile) {
    return profile.motion_toggle_chord == 0
               ? CONTROLLER_PROFILE_DEFAULT_MOTION_TOGGLE_CHORD
               : profile.motion_toggle_chord;
}

bool chord_held(uint32_t controls, uint32_t chord) {
    return chord != 0 && (controls & chord) == chord;
}

uint32_t shortcut_selector_mask(const ControllerProfile& profile) {
    uint32_t mask = 0;
    for (const uint8_t selector : profile.shortcuts.selectors) {
        if (selector < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT) {
            mask |= 1u << selector;
        }
    }
    return mask;
}

void initialize_defaults() {
    if (g_initialized) {
        return;
    }
    g_default_profile =
        controller_profile_default(controller_identity_global(), 0);
    g_neutral_output = controller_profile_transform(
        controller_neutral_state(), g_default_profile);
    g_initialized = true;
}

void clear_context(ControllerProfileRuntimeContext* context) {
    if (context == nullptr) {
        return;
    }
    *context = {};
}

void refresh_profile(ControllerProfileRuntimeContext* context,
                     const ControllerIdentity& identity,
                     uint32_t connection_generation,
                     uint32_t observed_database_generation,
                     const Bluepad32SlotSnapshot& input_snapshot) {
    ProfileServiceActiveProfileSnapshot snapshot{};
    profile_service_active_profile_snapshot(identity, &snapshot);

    const bool same_connection =
        context->active &&
        context->connection_generation == connection_generation;
    const bool same_identity =
        same_connection &&
        controller_identity_equal(context->identity, identity);
    const uint8_t previous_profile_index =
        context->active_profile_index;
    const bool previous_profile_valid =
        context->profile_snapshot_valid;
    const bool had_connection = context->connection_seen;
    if (!same_connection) {
        context->runtime_generations_initialized = false;
        context->previous_hotkey_control_mask = 0;
        context->held_hotkey_chord = 0;
        context->held_hotkey_release_mask = 0;
        context->held_shortcut_selectors = 0;
        context->activation_requested = true;
        context->activation_transaction_id = 0;
        context->profile_change_pending = false;
        context->pending_profile_change = {};
        context->initial_profile_indication_resolved = false;
        context->initial_profile_indication_pending = false;
        context->pending_initial_profile_indication = {};
    }

    context->active = true;
    context->connection_seen = true;
    context->connection_generation = connection_generation;
    context->identity = identity;
    context->database_generation = snapshot.valid
                                       ? snapshot.metadata.generation
                                       : observed_database_generation;
    context->active_profile_index = snapshot.valid ? snapshot.profile_index : 0;
    context->profile_snapshot_valid = snapshot.valid;
    context->profile = snapshot.valid ? snapshot.profile : g_default_profile;
    const uint32_t current_input_control_mask =
        controller_profile_extract_control_mask(
            input_snapshot.state, context->profile);
    controller_synthetic_input_cancel(
        &context->synthetic, current_input_control_mask);
    context->swing.reset();
    const uint32_t hotkey_control_mask =
        (current_input_control_mask & ~0xffffu) |
        input_snapshot.pre_hotkey_button_mask;
    // Refreshes must not turn a held input into a fresh command. The first
    // connection retains the existing ability to start a held cycle chord.
    context->previous_hotkey_control_mask =
        had_connection ? hotkey_control_mask :
        hotkey_control_mask & effective_motion_toggle_chord(context->profile);
    if (!context->initial_profile_indication_resolved && snapshot.valid) {
        context->initial_profile_indication_resolved = true;
        const uint8_t policy = static_cast<uint8_t>(
            controller_profile_confirmation_policy(context->profile));
        if ((policy & static_cast<uint8_t>(
                          ControllerProfileConfirmationPolicy::kLed)) != 0) {
            context->pending_initial_profile_indication = {
                connection_generation,
                context->database_generation,
                static_cast<uint8_t>(context->active_profile_index + 1u),
                ControllerProfileConfirmationPolicy::kLed,
            };
            context->initial_profile_indication_pending = true;
        }
    }
    // A pending target and its consuming chord belong to the observation that
    // started them, not to the newly loaded profile's bindings.
    if (same_identity && previous_profile_valid && snapshot.valid &&
        previous_profile_index != context->active_profile_index) {
        context->pending_profile_change = {
            connection_generation,
            context->database_generation,
            static_cast<uint8_t>(context->active_profile_index + 1u),
            controller_profile_confirmation_policy(context->profile),
        };
        context->profile_change_pending = true;
    }
}

ControllerProfileRuntimeContext* update_context(
    uint8_t slot, const Bluepad32SlotSnapshot& snapshot) {
    if (slot >= CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT) {
        return nullptr;
    }

    ControllerProfileRuntimeContext& context = g_contexts[slot];
    if (!snapshot.active) {
        if (context.active) {
            clear_context(&context);
            context.connection_seen = true;
        }
        return nullptr;
    }

    const uint32_t database_generation =
        profile_service_database_generation();
    if (!context.active ||
        context.connection_generation != snapshot.connection_generation ||
        !controller_identity_equal(context.identity, snapshot.identity) ||
        context.database_generation != database_generation) {
        refresh_profile(
            &context, snapshot.identity, snapshot.connection_generation,
            database_generation, snapshot);
    }
    return &context;
}
void request_activation(ControllerProfileRuntimeContext* context,
                        const ControllerIdentity& identity) {
    if (context->activation_requested) {
        return;
    }
    if (context->target_profile_index == context->active_profile_index) {
        context->activation_requested = true;
        return;
    }
    const ConfigurationTransactionStatus status =
        profile_service_activate_internal(
            context->activation_transaction_id, identity,
            context->target_profile_index);
    if (status != ConfigurationTransactionStatus::kBusy) {
        context->activation_requested = true;
    }
}

uint32_t process_hotkeys(ControllerProfileRuntimeContext* context,
                         uint8_t slot, uint32_t controls) {
    const ControllerProfile& profile = context->profile;
    const uint32_t selectors = shortcut_selector_mask(profile);
    const uint32_t modifier =
        profile.shortcuts.modifier < CONTROLLER_PROFILE_LOGICAL_CONTROL_COUNT
            ? 1u << profile.shortcuts.modifier : 0;
    const uint32_t pressed_selectors = controls & selectors;
    const bool direct = (controls & modifier) != 0 && pressed_selectors != 0;
    const uint32_t cycle_chord = effective_switching_chord(profile);
    const uint32_t motion_chord = effective_motion_toggle_chord(profile);
    const bool cycle = chord_held(controls, cycle_chord);
    const bool motion = chord_held(controls, motion_chord);
    uint32_t consumed = (direct ? modifier | pressed_selectors : 0) |
                        (cycle ? cycle_chord : 0) |
                        (motion ? motion_chord : 0);

    if (context->held_hotkey_chord != 0) {
        consumed |= context->held_hotkey_chord |
                    (controls & context->held_shortcut_selectors);
        const bool released = context->held_shortcut_selectors != 0
            ? (controls & context->held_hotkey_release_mask) == 0 ||
              !chord_held(controls, context->held_hotkey_chord &
                                       ~context->held_shortcut_selectors)
            : !chord_held(controls, context->held_hotkey_chord);
        if (released) {
            context->held_hotkey_chord = 0;
            context->held_hotkey_release_mask = 0;
            context->held_shortcut_selectors = 0;
            context->activation_requested = true;
            context->activation_transaction_id = 0;
        } else if (chord_held(controls, context->held_hotkey_chord)) {
            request_activation(context, context->identity);
        } else {
            // An ambiguous direct chord remains rejected until its modifier
            // or all of its original selectors are released.
            context->activation_requested = true;
        }
        if (!released) return consumed;
    }
    if (!direct && !cycle && !motion) {
        return consumed;
    }

    const uint32_t chord = direct ? modifier | pressed_selectors :
                           cycle ? cycle_chord : motion_chord;
    context->held_hotkey_chord = chord;
    context->held_hotkey_release_mask = direct ? pressed_selectors : chord;
    context->held_shortcut_selectors = direct ? selectors : 0;
    context->activation_requested = true;
    if (chord_held(context->previous_hotkey_control_mask, chord)) {
        return consumed;
    }
    if (direct) {
        if ((pressed_selectors & (pressed_selectors - 1u)) != 0) {
            return consumed;
        }
        for (uint8_t index = 0; index < CONTROLLER_PROFILE_COUNT; ++index) {
            const uint8_t selector = profile.shortcuts.selectors[index];
            if (selector < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT &&
                (pressed_selectors & (1u << selector)) != 0) {
                context->target_profile_index = index;
                break;
            }
        }
    } else if (cycle) {
        context->target_profile_index = static_cast<uint8_t>(
            (context->active_profile_index + 1u) % CONTROLLER_PROFILE_COUNT);
    } else {
        bluepad32_input_backend_toggle_motion(
            slot, context->connection_generation);
        return consumed;
    }
    context->activation_requested = false;
    context->activation_transaction_id = next_activation_transaction_id();
    request_activation(context, context->identity);
    return consumed;
}

}  // namespace

void controller_profile_runtime_reset() {
    g_initialized = false;
    initialize_defaults();
    for (ControllerProfileRuntimeContext& context : g_contexts) {
        clear_context(&context);
    }
    g_next_activation_sequence = 1;
}

ControllerProfileTransformResult controller_profile_runtime_transform(
    uint8_t slot, const Bluepad32SlotSnapshot& snapshot, uint32_t now_ms,
    AdapterUsbMode output_mode) {
    initialize_defaults();
    ControllerProfileRuntimeContext* context =
        update_context(slot, snapshot);
    if (context == nullptr) {
        return g_neutral_output;
    }
    const uint32_t reset_generation =
        configuration_service_reset_generation();
    const uint32_t state_control_mask =
        controller_profile_extract_control_mask(
            snapshot.state, context->profile);
    const uint32_t input_control_mask =
        (state_control_mask & ~0xffffu) |
        snapshot.pre_hotkey_button_mask;
    if (context->runtime_generations_initialized &&
        (context->output_mode != output_mode ||
         context->configuration_reset_generation != reset_generation)) {
        controller_synthetic_input_cancel(
            &context->synthetic, state_control_mask);
        context->swing.reset();
        context->previous_hotkey_control_mask = input_control_mask;
        context->activation_requested = true;
        context->activation_transaction_id = 0;
    }
    context->runtime_generations_initialized = true;
    context->output_mode = output_mode;
    context->configuration_reset_generation = reset_generation;
    const uint32_t consumed_controls =
        process_hotkeys(context, slot, input_control_mask);
    context->previous_hotkey_control_mask = input_control_mask;
    const bool macro_was_active = context->synthetic.macro_active;
    const ControllerProfile& profile = context->profile;
    const auto binding_allowed = [&](uint8_t button, uint8_t macro, uint8_t modifier) {
        const bool assigned = button < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT ||
                              macro < CONTROLLER_PROFILE_MACRO_COUNT;
        return assigned && (modifier == CONTROLLER_PROFILE_NO_BUTTON ||
            (modifier < CONTROLLER_PROFILE_LOGICAL_CONTROL_COUNT &&
             (input_control_mask & (1u << modifier)) != 0));
    };
    bool cancel_pressed = false;
    for (const auto& macro : profile.macros) {
        cancel_pressed |= macro.cancel_control < CONTROLLER_PROFILE_LOGICAL_CONTROL_COUNT &&
                          (input_control_mask & (1u << macro.cancel_control)) != 0;
    }
    uint8_t allowed = 0;
    if (consumed_controls == 0 && !cancel_pressed) {
        if (binding_allowed(profile.swing.button, profile.swing.macro, profile.swing.modifier))
            allowed |= 1;
        if (binding_allowed(profile.nunchuk_swing.button, profile.nunchuk_swing.macro,
                            profile.nunchuk_swing.modifier))
            allowed |= 2;
        if (binding_allowed(profile.combined_swing.button, profile.combined_swing.macro,
                            profile.combined_swing.modifier))
            allowed |= 4;
    }
    const WiiSwingGestureResult gestures = context->swing.update(
        snapshot.accelerometer, snapshot.nunchuk_accelerometer, now_ms,
        profile.swing.sensitivity, profile.nunchuk_swing.sensitivity,
        allowed, profile.combination_window_ms);
    const uint8_t macros[3] = {
        profile.swing.macro, profile.nunchuk_swing.macro, profile.combined_swing.macro};
    const uint8_t buttons[3] = {
        profile.swing.button, profile.nunchuk_swing.button, profile.combined_swing.button};
    uint8_t requested_macro = CONTROLLER_PROFILE_NO_BUTTON;
    for (unsigned i = 0; i < 3; ++i)
        if ((gestures.started & (1u << i)) && macros[i] < requested_macro)
            requested_macro = macros[i];
    ControllerProfileTransformResult result = controller_synthetic_input_apply(
        &context->synthetic, snapshot.state, profile, now_ms,
        consumed_controls, consumed_controls != 0, requested_macro);
    if (macro_was_active || context->synthetic.macro_active) {
        context->swing.discard_actions();
    } else {
        uint16_t output_buttons = controller_profile_extract_button_mask(result.state);
        for (unsigned i = 0; i < 3; ++i)
            if ((gestures.active & (1u << i)) && buttons[i] < CONTROLLER_PROFILE_LOGICAL_BUTTON_COUNT)
                output_buttons |= static_cast<uint16_t>(1u << buttons[i]);
        controller_profile_apply_button_mask(output_buttons, &result.state);
    }
    return result;
}

bool controller_profile_runtime_take_initial_profile_indication(
    uint8_t slot, ControllerProfileRuntimeProfileChangeEvent* output) {
    if (output == nullptr) {
        return false;
    }
    *output = {};
    if (slot >= CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT) {
        return false;
    }

    ControllerProfileRuntimeContext& context = g_contexts[slot];
    if (!context.initial_profile_indication_pending) {
        return false;
    }
    *output = context.pending_initial_profile_indication;
    context.initial_profile_indication_pending = false;
    context.pending_initial_profile_indication = {};
    return true;
}

bool controller_profile_runtime_take_profile_change(
    uint8_t slot, ControllerProfileRuntimeProfileChangeEvent* output) {
    if (output == nullptr) {
        return false;
    }
    *output = {};
    if (slot >= CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT) {
        return false;
    }

    ControllerProfileRuntimeContext& context = g_contexts[slot];
    if (!context.profile_change_pending) {
        return false;
    }
    *output = context.pending_profile_change;
    context.profile_change_pending = false;
    context.pending_profile_change = {};
    return true;
}

ControllerRumbleOutput controller_profile_runtime_scale_host_rumble(
    uint8_t slot, const Bluepad32SlotSnapshot& snapshot,
    const ControllerRumbleOutput& rumble) {
    initialize_defaults();
    ControllerProfileRuntimeContext* context =
        update_context(slot, snapshot);
    const ControllerProfile& profile =
        context == nullptr ? g_default_profile : context->profile;
    return controller_profile_scale_host_rumble(rumble, profile);
}

ControllerProfileRuntimeLocalConfirmation
controller_profile_runtime_local_confirmation(
    uint8_t slot, const Bluepad32SlotSnapshot& snapshot,
    const ControllerRumbleOutput& rumble) {
    initialize_defaults();
    ControllerProfileRuntimeContext* context =
        update_context(slot, snapshot);
    const ControllerProfile& profile =
        context == nullptr ? g_default_profile : context->profile;
    return {
        rumble,
        controller_profile_confirmation_policy(profile),
    };
}
