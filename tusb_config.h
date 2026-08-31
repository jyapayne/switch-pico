// TinyUSB configuration for one to four Switch Pro style HID interfaces.
// Each interface uses independent 64-byte interrupt IN and OUT endpoints.
#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif
#ifndef SWITCH_PICO_HID_INSTANCE_COUNT
#define SWITCH_PICO_HID_INSTANCE_COUNT 1
#endif

#if SWITCH_PICO_HID_INSTANCE_COUNT < 1 || SWITCH_PICO_HID_INSTANCE_COUNT > 4
#error "SWITCH_PICO_HID_INSTANCE_COUNT must be between 1 and 4"
#endif


#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS           OPT_OS_NONE
#endif

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#endif

#define CFG_TUD_ENDPOINT0_SIZE 64

// Device class configuration
#define CFG_TUD_HID SWITCH_PICO_HID_INSTANCE_COUNT
#define CFG_TUD_CDC 0
#define CFG_TUD_MSC 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0
// Always enable TinyUSB debug at level 2; LOG_PRINTF controls user-facing logs.
#ifdef CFG_TUSB_DEBUG
#undef CFG_TUSB_DEBUG
#endif
#define CFG_TUSB_DEBUG 0

#define CFG_TUD_HID_EP_BUFSIZE 64

#ifdef __cplusplus
}
#endif

#endif // _TUSB_CONFIG_H_
