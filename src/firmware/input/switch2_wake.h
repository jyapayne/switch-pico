#pragma once

#include <stdint.h>

struct Switch2WakeDiagnostics {
    bool configured;
    bool busy;
    uint32_t accepted_requests;
    uint32_t completed_bursts;
    uint32_t failures;
};

// Installs BTstack callbacks and remembers the Pico's normal public identity.
void switch2_wake_initialize();


// Starts one wake burst when configured and idle. Calls while busy coalesce.
bool switch2_wake_request();

void switch2_wake_diagnostics(Switch2WakeDiagnostics* out);
