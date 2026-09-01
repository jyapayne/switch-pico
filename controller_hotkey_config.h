// Compile-time AIO controller hotkey configuration.

#pragma once

// Bluepad32 button masks. Default chord: L + R + SELECT + START.
#define SWITCH_ABXY_HOTKEY_BUTTON_MASK \
    (BUTTON_SHOULDER_L | BUTTON_SHOULDER_R)
#define SWITCH_ABXY_HOTKEY_MISC_MASK \
    (MISC_BUTTON_SELECT | MISC_BUTTON_START)

// 0 starts each new connection in Nintendo positional layout; 1 starts swapped.
#define SWITCH_ABXY_DEFAULT_SWAPPED 0

// Local confirmation pulse sent only to the controller that toggled.
#define SWITCH_ABXY_FEEDBACK_DURATION_MS 120
#define SWITCH_ABXY_FEEDBACK_WEAK_MAGNITUDE 0x80
#define SWITCH_ABXY_FEEDBACK_STRONG_MAGNITUDE 0x80
