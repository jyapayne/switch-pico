#pragma once

// Core 0 only. Main initializes the router/Core 1 before attaching USB here.
void probe_hub_init(void);
void probe_hub_task(void);
