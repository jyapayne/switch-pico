#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Samples BOOTSEL through flash_safe_execute. Returns 1 pressed, 0 released,
// or -1 when sampling was unsafe/unavailable. Does not trigger pairing actions.
int bootsel_button_sample(void);

#ifdef __cplusplus
}
#endif
