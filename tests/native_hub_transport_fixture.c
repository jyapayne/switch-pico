#include "hardware_stub.h"
#include "usb/native_hub/native_hub.c"

usb_hw_t native_test_usb;
usb_device_dpram_t native_test_dpram;
sio_hw_t native_test_sio;
bool native_test_abort_stuck;
uint32_t native_test_interrupt_mask;
static bool servicing_interrupt;

void native_test_service_interrupt(void) {
    if (native_test_interrupt_mask || servicing_interrupt || !usb_hw->ints) return;
    servicing_interrupt = true;
    usb_interrupt();
    usb_hw->ints = 0;
    servicing_interrupt = false;
}

void probe_router_init(uint32_t hz) { (void)hz; }
void probe_router_core1(void) {}
void probe_router_publish(const uint8_t values[PROBE_ROUTER_SLOTS], uint8_t slot) { (void)values; (void)slot; }
void probe_router_enable(bool enabled) { (void)enabled; }
bool probe_router_set_phase(uint32_t phase) { (void)phase; return true; }
void probe_router_snapshot(probe_router_stats* snapshot) { memset(snapshot,0,sizeof(*snapshot)); snapshot->ready = 1; }
int probe_debug_printf(const char* format, ...) { (void)format; return 0; }

const uint8_t* native_joycon_device_descriptor(uint8_t instance) { (void)instance; return hub_device; }
const uint8_t* native_joycon_configuration_descriptor(uint8_t instance) { (void)instance; return hub_configuration; }
const uint16_t* native_joycon_string_descriptor(uint8_t instance, uint8_t index, uint16_t language) {
    (void)instance; (void)language; return hub_string(index);
}
void native_joycon_usb_reset(uint8_t instance) { (void)instance; }
const uint8_t* tud_hid_descriptor_report_cb(uint8_t instance) { (void)instance; return NULL; }
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t id, hid_report_type_t type, uint8_t* data, uint16_t length) {
    (void)instance; (void)id; (void)type; (void)data; (void)length; return 0;
}
void tud_hid_set_report_cb(uint8_t instance, uint8_t id, hid_report_type_t type, const uint8_t* data, uint16_t length) {
    (void)instance; (void)id; (void)type; (void)data; (void)length;
}
void tud_hid_report_complete_cb(uint8_t instance, const uint8_t* data, uint16_t length) { (void)instance; (void)data; (void)length; }
void tud_vendor_rx_cb(uint8_t instance, const uint8_t* data, uint16_t length) { (void)instance; (void)data; (void)length; }
void tud_vendor_tx_cb(uint8_t instance, uint32_t length) { (void)instance; (void)length; }

void native_test_initialize(void) {
    memset(devices,0,sizeof(devices));
    memset(ports,0,sizeof(ports));
    memset(usb_hw,0,sizeof(*usb_hw));
    memset(usb_dpram,0,sizeof(*usb_dpram));
    event_head = event_tail = 0;
    native_test_abort_stuck = false;
    native_test_interrupt_mask = 0;
    servicing_interrupt = false;
    failed = bus_suspended = bank_restore_pending = false;
    bank_lock = spin_lock_instance(0);
    active_device = default_device = 0;
    addresses[0] = 0; addresses[1] = 1; addresses[2] = 2;
    started = root_configured_once = true;
}

static bool select_slot(uint8_t slot) {
    if (!native_hub_select_device(addresses[slot],slot,UINT32_MAX / 2)) return false;
    restore_selected_bank();
    return true;
}

bool native_test_select(uint8_t slot) { return select_slot(slot); }

void native_test_drain(void) { native_hub_task(); }

bool native_test_setup(uint8_t slot, const tusb_control_request_t* request, bool drain) {
    if (!select_slot(slot)) return false;
    memcpy(usb_dpram->setup_packet,request,sizeof(*request));
    usb_hw->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
    usb_hw->ints = USB_INTS_SETUP_REQ_BITS;
    native_test_service_interrupt();
    if (drain) native_hub_task();
    return !failed && devices[slot].control.stage != STALLED;
}

void native_test_hold_abort(bool hold) { native_test_abort_stuck = hold; }

bool native_test_out(uint8_t slot, const uint8_t* data, uint16_t length, bool drain) {
    if (!select_slot(slot)) return false;
    if (usb_hw->abort & 2u) return false;
    uint32_t value = buffer_regs()[1];
    if (!(value & USB_BUF_CTRL_AVAIL) || (value & USB_BUF_CTRL_STALL) ||
        length > (value & USB_BUF_CTRL_LEN_MASK)) return false;
    if (length) copy_to_usb(usb_dpram->ep0_buf_a,data,length);
    buffer_regs()[1] = (value & ~(USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_LEN_MASK)) | length;
    usb_hw->buf_status = 2;
    usb_hw->ints = USB_INTS_BUFF_STATUS_BITS;
    native_test_service_interrupt();
    if (drain) native_hub_task();
    return !failed && devices[slot].control.stage != STALLED;
}

bool native_test_in(uint8_t slot, uint8_t* data, uint16_t* length, bool drain) {
    if (!select_slot(slot)) return false;
    if (usb_hw->abort & 1u) return false;
    uint32_t value = buffer_regs()[0];
    if (!(value & USB_BUF_CTRL_AVAIL) || !(value & USB_BUF_CTRL_FULL) ||
        (value & USB_BUF_CTRL_STALL)) return false;
    *length = value & USB_BUF_CTRL_LEN_MASK;
    if (*length) copy_from_usb(data,usb_dpram->ep0_buf_a,*length);
    buffer_regs()[0] = value & ~USB_BUF_CTRL_AVAIL;
    usb_hw->buf_status = 1;
    usb_hw->ints = USB_INTS_BUFF_STATUS_BITS;
    native_test_service_interrupt();
    if (drain) native_hub_task();
    return !failed && devices[slot].control.stage != STALLED;
}

bool native_test_private_in(uint8_t slot, uint8_t endpoint, uint8_t* data, uint16_t* length) {
    if (slot < 1 || slot > 2 || (endpoint != 0x81 && endpoint != 0x82)) return false;
    // A host token selects the bank, but cannot wait for a foreground task.
    if (!native_hub_select_device(addresses[slot],slot,UINT32_MAX / 2)) return false;
    unsigned channel = (endpoint & 15u) * 2u;
    uint32_t control = endpoint_regs()[channel - 2u];
    uint32_t value = buffer_regs()[channel];
    if (!(control & EP_CTRL_ENABLE_BITS) || !(value & USB_BUF_CTRL_AVAIL) ||
        !(value & USB_BUF_CTRL_FULL) || (value & USB_BUF_CTRL_STALL)) return false;
    *length = value & USB_BUF_CTRL_LEN_MASK;
    if (*length > PACKET) return false;
    if (*length) copy_from_usb(data,
        (const volatile uint8_t*)USBCTRL_DPRAM_BASE + (control & 0xffffu), *length);
    buffer_regs()[channel] = value & ~USB_BUF_CTRL_AVAIL;
    usb_hw->buf_status |= 1u << channel;
    usb_hw->ints |= USB_INTS_BUFF_STATUS_BITS;
    native_test_service_interrupt();
    return !failed;
}

void native_test_bus_reset(bool drain) {
    usb_hw->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;
    usb_hw->ints = USB_INTS_BUS_RESET_BITS;
    native_test_service_interrupt();
    if (drain) native_hub_task();
    // Assign fixture addresses after reset, independently of EP0 state.
    addresses[0] = 0; addresses[1] = 1; addresses[2] = 2;
}
