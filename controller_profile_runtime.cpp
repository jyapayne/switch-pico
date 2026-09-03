#include "controller_profile_runtime.h"

#include "configuration_service.h"
#include "controller_identity.h"
#include "controller_synthetic_input.h"
#include "profile_service.h"

namespace {

struct ControllerProfileRuntimeContext {
    bool active = false;
    uint32_t connection_generation = 0;
    ControllerIdentity identity{};
    uint32_t database_generation = 0;
    uint8_t active_profile_index = 0;
    bool profile_snapshot_valid = false;
    ControllerProfile profile{};
    ControllerSyntheticInputContext synthetic{};
    bool runtime_generations_initialized = false;
    AdapterUsbMode output_mode = AdapterUsbMode::kSwitchProbe;
    uint32_t configuration_reset_generation = 0;
    bool switching_chord_held = false;
    bool switching_chord_armed = true;
    bool switching_activation_requested = false;
    uint16_t held_switching_chord = 0;
    uint8_t switching_target_profile_index = 0;
    uint32_t switching_transaction_id = 0;
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

uint16_t effective_switching_chord(const ControllerProfile& profile) {
    return profile.switching_chord == 0
               ? CONTROLLER_PROFILE_DEFAULT_SWITCHING_CHORD
               : profile.switching_chord;
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
                     uint16_t current_input_button_mask) {
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
    if (!same_connection) {
        context->runtime_generations_initialized = false;
        context->switching_chord_held = false;
        context->switching_chord_armed = true;
        context->switching_activation_requested = false;
        context->held_switching_chord = 0;
        context->switching_target_profile_index = 0;
        context->switching_transaction_id = 0;
        context->profile_change_pending = false;
        context->pending_profile_change = {};
        context->initial_profile_indication_resolved = false;
        context->initial_profile_indication_pending = false;
        context->pending_initial_profile_indication = {};
    }

    controller_synthetic_input_cancel(&context->synthetic,
                                      current_input_button_mask);
    context->active = true;
    context->connection_generation = connection_generation;
    context->identity = identity;
    context->database_generation = snapshot.valid
                                       ? snapshot.metadata.generation
                                       : observed_database_generation;
    context->active_profile_index = snapshot.valid ? snapshot.profile_index : 0;
    context->profile_snapshot_valid = snapshot.valid;
    context->profile = snapshot.valid ? snapshot.profile : g_default_profile;
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
    if (context->switching_chord_held &&
        !context->switching_activation_requested) {
        context->switching_target_profile_index =
            static_cast<uint8_t>(
                (context->active_profile_index + 1u) %
                CONTROLLER_PROFILE_COUNT);
    }
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
            database_generation,
            controller_profile_extract_button_mask(snapshot.state));
    }
    return &context;
}
void process_profile_switching(
    ControllerProfileRuntimeContext* context,
    const ControllerIdentity& identity,
    uint16_t switching_input_button_mask,
    uint16_t current_input_button_mask,
    ControllerState* consumed_input) {
    if (context == nullptr || consumed_input == nullptr) {
        return;
    }

    if (context->switching_chord_held) {
        if ((switching_input_button_mask &
             context->held_switching_chord) ==
            context->held_switching_chord) {
            controller_profile_apply_button_mask(
                static_cast<uint16_t>(
                    current_input_button_mask &
                    ~context->held_switching_chord),
                consumed_input);
            if (!context->switching_activation_requested) {
                const ConfigurationTransactionStatus status =
                    profile_service_activate_internal(
                        context->switching_transaction_id, identity,
                        context->switching_target_profile_index);
                if (status != ConfigurationTransactionStatus::kBusy) {
                    context->switching_activation_requested = true;
                }
            }
            return;
        }
        context->switching_chord_held = false;
        context->switching_activation_requested = false;
        context->held_switching_chord = 0;
        context->switching_transaction_id = 0;
    }

    const uint16_t chord =
        effective_switching_chord(context->profile);
    const bool chord_fully_held =
        (switching_input_button_mask & chord) == chord;
    if (!chord_fully_held) {
        context->switching_chord_armed = true;
        return;
    }
    if (!context->switching_chord_armed) {
        return;
    }

    context->switching_chord_armed = false;
    context->switching_chord_held = true;
    context->held_switching_chord = chord;
    context->switching_target_profile_index =
        static_cast<uint8_t>(
            (context->active_profile_index + 1u) %
            CONTROLLER_PROFILE_COUNT);
    context->switching_transaction_id =
        next_activation_transaction_id();
    controller_profile_apply_button_mask(
        static_cast<uint16_t>(
            current_input_button_mask & ~context->held_switching_chord),
        consumed_input);
    const ConfigurationTransactionStatus status =
        profile_service_activate_internal(
            context->switching_transaction_id, identity,
            context->switching_target_profile_index);
    if (status != ConfigurationTransactionStatus::kBusy) {
        context->switching_activation_requested = true;
    }
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
    ControllerState consumed_input = snapshot.state;
    const uint16_t current_input_button_mask =
        controller_profile_extract_button_mask(snapshot.state);
    process_profile_switching(
        context, snapshot.identity, snapshot.pre_hotkey_button_mask,
        current_input_button_mask, &consumed_input);

    const uint32_t reset_generation =
        configuration_service_reset_generation();
    if (!context->runtime_generations_initialized) {
        context->runtime_generations_initialized = true;
        context->output_mode = output_mode;
        context->configuration_reset_generation = reset_generation;
    } else if (context->output_mode != output_mode ||
               context->configuration_reset_generation !=
                   reset_generation) {
        controller_synthetic_input_cancel(
            &context->synthetic,
            controller_profile_extract_button_mask(consumed_input));
        context->output_mode = output_mode;
        context->configuration_reset_generation = reset_generation;
    }
    return controller_synthetic_input_apply(
        &context->synthetic, consumed_input, context->profile, now_ms);
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
