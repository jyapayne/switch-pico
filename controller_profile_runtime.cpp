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
    ControllerProfile profile{};
    ControllerSyntheticInputContext synthetic{};
    bool runtime_generations_initialized = false;
    AdapterUsbMode output_mode = AdapterUsbMode::kSwitchProbe;
    uint32_t configuration_reset_generation = 0;
};

ControllerProfileRuntimeContext
    g_contexts[CONTROLLER_PROFILE_RUNTIME_SLOT_COUNT]{};
ControllerProfile g_default_profile{};
ControllerProfileTransformResult g_neutral_output{};
bool g_initialized = false;

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

    context->active = true;
    context->connection_generation = connection_generation;
    context->identity = identity;
    context->database_generation = snapshot.valid
                                       ? snapshot.metadata.generation
                                       : observed_database_generation;
    context->active_profile_index = snapshot.valid ? snapshot.profile_index : 0;
    context->profile = snapshot.valid ? snapshot.profile : g_default_profile;
    controller_synthetic_input_cancel(&context->synthetic,
                                      current_input_button_mask);
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


}  // namespace

void controller_profile_runtime_reset() {
    g_initialized = false;
    initialize_defaults();
    for (ControllerProfileRuntimeContext& context : g_contexts) {
        clear_context(&context);
    }
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
    if (!context->runtime_generations_initialized) {
        context->runtime_generations_initialized = true;
        context->output_mode = output_mode;
        context->configuration_reset_generation = reset_generation;
    } else if (context->output_mode != output_mode ||
               context->configuration_reset_generation !=
                   reset_generation) {
        controller_synthetic_input_cancel(
            &context->synthetic,
            controller_profile_extract_button_mask(snapshot.state));
        context->output_mode = output_mode;
        context->configuration_reset_generation = reset_generation;
    }
    return controller_synthetic_input_apply(
        &context->synthetic, snapshot.state, context->profile, now_ms);
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
