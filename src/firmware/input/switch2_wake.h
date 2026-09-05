#pragma once

#include <stdint.h>

struct Switch2WakeDiagnostics {
    bool configured;
    bool busy;
    uint32_t accepted_requests;
    uint32_t completed_bursts;
    uint32_t failures;
};

// Installs BTstack callbacks and applies the configured stable public identity.
// Call once from the BTstack core before admitting controller connections.
void switch2_wake_initialize();
bool switch2_wake_ready_for_connections();

// Starts one wake burst when configured and idle. Calls while busy coalesce.
bool switch2_wake_request();

void switch2_wake_diagnostics(Switch2WakeDiagnostics* out);
