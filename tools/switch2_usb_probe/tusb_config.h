#pragma once

#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_NONE
#endif
#define CFG_TUD_ENDPOINT0_SIZE 64
#define CFG_TUD_HID 1
#define CFG_TUD_HID_EP_BUFSIZE 64
#define CFG_TUD_CDC 0
#define CFG_TUD_MSC 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 1
#define CFG_TUD_VENDOR_EPSIZE 64
#define CFG_TUD_VENDOR_RX_BUFSIZE 256
#define CFG_TUD_VENDOR_TX_BUFSIZE 256
#ifdef CFG_TUSB_DEBUG
#undef CFG_TUSB_DEBUG
#endif
// Packet-level SDK logging would overflow 115200 baud while streaming input.
// Requests, command payloads, errors and aggregate counters are logged explicitly.
#define CFG_TUSB_DEBUG 1
#define CFG_TUSB_DEBUG_PRINTF probe_debug_printf

#ifdef __cplusplus
extern "C" {
#endif
int probe_debug_printf(const char* format, ...);
#ifdef __cplusplus
}
#endif
