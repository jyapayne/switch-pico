#include "hardware_stub.h"
#include <stdlib.h>
static uint32_t native_test_time_us = 1000000u;
#define time_us_32() native_test_time_us
#include "usb/native_hub/native_hub.c"

usb_hw_t native_test_usb;
usb_device_dpram_t native_test_dpram;
sio_hw_t native_test_sio;
bool native_test_abort_stuck;
uint32_t native_test_interrupt_mask;
uint32_t native_test_hid_completions[CHILDREN], native_test_bulk_completions[CHILDREN];
uint32_t native_test_received_count[CHILDREN][2];
uint16_t native_test_received_length[CHILDREN][2];
uint8_t native_test_received_data[CHILDREN][2][PACKET];
static bool servicing_interrupt;
static void (*native_test_reset_hook)(uint8_t instance);

#ifndef NATIVE_TEST_EXTERNAL_IRQ
void native_test_service_interrupt(void) {
    if (native_test_interrupt_mask || servicing_interrupt || !usb_hw->ints) return;
    servicing_interrupt = true;
    usb_interrupt();
    usb_hw->ints = 0;
    servicing_interrupt = false;
}
#endif

#ifndef NATIVE_TEST_EXTERNAL_ROUTER
void probe_router_init(uint32_t hz) { (void)hz; }
void probe_router_core1(void) {}
void probe_router_publish(const uint8_t values[PROBE_ROUTER_SLOTS], uint8_t slot) { (void)values; (void)slot; }
void probe_router_enable(bool enabled) { (void)enabled; }
bool probe_router_set_phase(uint32_t phase) { (void)phase; return true; }
void probe_router_snapshot(probe_router_stats* snapshot) { memset(snapshot,0,sizeof(*snapshot)); snapshot->ready = 1; }
#endif
#ifndef NATIVE_TEST_EXTERNAL_LOG
int probe_debug_printf(const char* format, ...) { (void)format; return 0; }
#endif

const uint8_t* native_joycon_device_descriptor(uint8_t instance) { (void)instance; return hub_device; }
const uint8_t* native_joycon_configuration_descriptor(uint8_t instance) { (void)instance; return hub_configuration; }
const uint16_t* native_joycon_string_descriptor(uint8_t instance, uint8_t index, uint16_t language) {
    (void)instance; (void)language; return hub_string(index);
}
void native_joycon_usb_reset(uint8_t instance) {
    if (native_test_reset_hook) native_test_reset_hook(instance);
}
const uint8_t* tud_hid_descriptor_report_cb(uint8_t instance) { (void)instance; return NULL; }
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t id, hid_report_type_t type, uint8_t* data, uint16_t length) {
    (void)instance; (void)id; (void)type; (void)data; (void)length; return 0;
}
void tud_hid_set_report_cb(uint8_t instance, uint8_t id, hid_report_type_t type, const uint8_t* data, uint16_t length) {
    (void)id; (void)type;
    if (instance >= CHILDREN || length > PACKET) abort();
    ++native_test_received_count[instance][0];
    native_test_received_length[instance][0] = length;
    if (length) memcpy(native_test_received_data[instance][0],data,length);
}
void tud_hid_report_complete_cb(uint8_t instance, const uint8_t* data, uint16_t length) {
    (void)data; (void)length;
    if (instance >= CHILDREN) abort();
    ++native_test_hid_completions[instance];
}
void tud_vendor_rx_cb(uint8_t instance, const uint8_t* data, uint16_t length) {
    if (instance >= CHILDREN || length > PACKET) abort();
    ++native_test_received_count[instance][1];
    native_test_received_length[instance][1] = length;
    if (length) memcpy(native_test_received_data[instance][1],data,length);
}
void tud_vendor_tx_cb(uint8_t instance, uint32_t length) {
    (void)length;
    if (instance >= CHILDREN) abort();
    ++native_test_bulk_completions[instance];
}

void native_test_initialize(void) {
    memset(devices,0,sizeof(devices));
    memset(ports,0,sizeof(ports));
    memset(usb_hw,0,sizeof(*usb_hw));
    memset(usb_dpram,0,sizeof(*usb_dpram));
    memset(native_test_hid_completions,0,sizeof(native_test_hid_completions));
    memset(native_test_bulk_completions,0,sizeof(native_test_bulk_completions));
    memset(native_test_received_count,0,sizeof(native_test_received_count));
    memset(native_test_received_length,0,sizeof(native_test_received_length));
    memset(native_test_received_data,0,sizeof(native_test_received_data));
    native_test_time_us = 1000000u;
    event_head = event_tail = 0;
    native_test_abort_stuck = false;
    native_test_interrupt_mask = 0;
    servicing_interrupt = false;
    native_test_reset_hook = NULL;
    failed = bus_suspended = false;
    bank_lock = spin_lock_instance(0);
    active_device = default_device = 0;
    addresses[0] = 0;
    for (unsigned slot = 1; slot < DEVICES; ++slot) addresses[slot] = slot * 17u;
    started = root_configured_once = true;
}

bool native_test_startup(void) {
    native_test_initialize();
    started = root_configured_once = false;
    return native_hub_init();
}

void native_test_advance(uint32_t microseconds) {
    native_test_time_us += microseconds;
    native_hub_task();
}

static bool select_slot(uint8_t slot) {
    if (slot >= DEVICES || addresses[slot] == NONE) return false;
    return native_hub_select_device(addresses[slot],slot,UINT32_MAX / 2);
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
    if (slot >= DEVICES || addresses[slot] == NONE || (slot == 0 ? endpoint != 0x8f :
        (endpoint != 0x81 && endpoint != 0x82))) return false;
    // A host token selects the bank, but cannot wait for a foreground task.
    if (!native_hub_select_device(addresses[slot],slot,UINT32_MAX / 2)) return false;
    unsigned channel = logical_channel(slot,endpoint);
    unsigned physical = physical_channel(slot,channel);
    uint32_t control = slot == 0 ? usb_dpram->ep_ctrl[14].in : endpoint_regs()[channel - 2u];
    uint32_t value = buffer_regs()[physical];
    if (!(control & EP_CTRL_ENABLE_BITS) || !(value & USB_BUF_CTRL_AVAIL) ||
        !(value & USB_BUF_CTRL_FULL) || (value & USB_BUF_CTRL_STALL)) return false;
    *length = value & USB_BUF_CTRL_LEN_MASK;
    if (*length > PACKET) return false;
    if (*length) copy_from_usb(data,
        (const volatile uint8_t*)USBCTRL_DPRAM_BASE + (control & 0xffffu), *length);
    buffer_regs()[physical] = value & ~USB_BUF_CTRL_AVAIL;
    usb_hw->buf_status |= 1u << physical;
    usb_hw->ints |= USB_INTS_BUFF_STATUS_BITS;
    native_test_service_interrupt();
    return !failed;
}

bool native_test_private_out(uint8_t slot, uint8_t endpoint, const uint8_t* data, uint16_t length, bool drain) {
    if (slot < 1 || slot >= DEVICES || addresses[slot] == NONE ||
        (endpoint != 0x01 && endpoint != 0x02) || length > PACKET) return false;
    if (!native_hub_select_device(addresses[slot],slot,UINT32_MAX / 2)) return false;
    unsigned channel = logical_channel(slot,endpoint);
    uint32_t control = endpoint_regs()[channel - 2u];
    uint32_t value = buffer_regs()[channel];
    if (!(control & EP_CTRL_ENABLE_BITS) || !(value & USB_BUF_CTRL_AVAIL) ||
        (value & USB_BUF_CTRL_STALL)) return false;
    if (length) copy_to_usb((volatile uint8_t*)USBCTRL_DPRAM_BASE + (control & 0xffffu),data,length);
    buffer_regs()[channel] = (value & ~(USB_BUF_CTRL_AVAIL | USB_BUF_CTRL_LEN_MASK)) | length;
    usb_hw->buf_status |= 1u << channel;
    usb_hw->ints |= USB_INTS_BUFF_STATUS_BITS;
    native_test_service_interrupt();
    if (drain) native_hub_task();
    return !failed;
}

void native_test_bus_reset(bool drain) {
    usb_hw->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;
    usb_hw->ints = USB_INTS_BUS_RESET_BITS;
    native_test_service_interrupt();
    if (drain) native_hub_task();
    // Assign fixture addresses after reset, independently of EP0 state.
    addresses[0] = 0;
    for (unsigned slot = 1; slot < DEVICES; ++slot) addresses[slot] = slot * 17u;
}
