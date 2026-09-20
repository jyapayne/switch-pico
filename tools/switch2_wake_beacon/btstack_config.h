#ifndef SWITCH2_WAKE_BEACON_BTSTACK_CONFIG_H
#define SWITCH2_WAKE_BEACON_BTSTACK_CONFIG_H

// BTstack requires its peripheral/advertiser fields for BLE advertising.
// There is no central/Classic role, connection pool, GATT/SM/HID stack or scan.
#if defined(ENABLE_CLASSIC) || defined(ENABLE_LE_CENTRAL)
#error "Wake-only firmware must not enable controller discovery/hosting roles"
#endif
#define ENABLE_LE_PERIPHERAL
#define HAVE_EMBEDDED_TIME_MS
// Required by the SDK's compiled dump helper; no logger/dump is initialized.
#define ENABLE_PRINTF_HEXDUMP
#define HAVE_ASSERT
#define HCI_OUTGOING_PRE_BUFFER_SIZE 4
#define HCI_INCOMING_PRE_BUFFER_SIZE 4
#define HCI_ACL_PAYLOAD_SIZE 251
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT 4
#define MAX_NR_HCI_CONNECTIONS 0
#define MAX_NR_L2CAP_CHANNELS 0
#define MAX_NR_L2CAP_SERVICES 0
#define MAX_NR_WHITELIST_ENTRIES 0
#define HCI_RESET_RESEND_TIMEOUT_MS 1000

#endif
