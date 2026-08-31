// Compile-time Switch grip colors. Physical controller lightbar values are
// derived automatically; each value here is an 8-bit RGB component.

#pragma once

// Body shell color
#define SWITCH_COLOR_BODY_R 0x1B
#define SWITCH_COLOR_BODY_G 0x1B
#define SWITCH_COLOR_BODY_B 0x1D

// Face/button cluster color
#define SWITCH_COLOR_BUTTON_R 0xFF
#define SWITCH_COLOR_BUTTON_G 0xFF
#define SWITCH_COLOR_BUTTON_B 0xFF

// Per-slot Switch grip colors: blue, red, yellow, green.
#define SWITCH_COLOR_SLOT_1_R 0x00
#define SWITCH_COLOR_SLOT_1_G 0x89
#define SWITCH_COLOR_SLOT_1_B 0xEB

#define SWITCH_COLOR_SLOT_2_R 0xE6
#define SWITCH_COLOR_SLOT_2_G 0x39
#define SWITCH_COLOR_SLOT_2_B 0x46

#define SWITCH_COLOR_SLOT_3_R 0xF6
#define SWITCH_COLOR_SLOT_3_G 0xC9
#define SWITCH_COLOR_SLOT_3_B 0x45

#define SWITCH_COLOR_SLOT_4_R 0x2E
#define SWITCH_COLOR_SLOT_4_G 0xCC
#define SWITCH_COLOR_SLOT_4_B 0x71
