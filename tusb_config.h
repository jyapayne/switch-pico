// TinyUSB configuration tailored for a single Switch Pro style HID interface.
// Data is derived from TinyUSB examples and tuned for a 64-byte HID endpoint.
#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
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
#define CFG_TUD_HID 1
#define CFG_TUD_CDC 0
#define CFG_TUD_MSC 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0
#ifdef CFG_TUSB_DEBUG
#undef CFG_TUSB_DEBUG
#endif
#define CFG_TUSB_DEBUG 2

#define CFG_TUD_HID_EP_BUFSIZE 64

#ifdef __cplusplus
}
#endif

#endif // _TUSB_CONFIG_H_
