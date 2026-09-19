#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "hardware_stub.h"

static void startup_set_bits(volatile uint32_t* address, uint32_t bits);
static void startup_clear_bits(volatile uint32_t* address, uint32_t bits);
static void startup_reset(uint32_t mask);
static void startup_launch(void (*entry)(void));
static void startup_irq_handler(unsigned irq, void (*handler)(void));
static void startup_irq_enable(unsigned irq, bool enabled);
static void startup_wait(void);

#define hw_set_bits(address, bits) startup_set_bits(address, bits)
#define hw_clear_bits(address, bits) startup_clear_bits(address, bits)
#define reset_block(mask) startup_reset(mask)
#define multicore_launch_core1(entry) startup_launch(entry)
#define irq_set_exclusive_handler(irq, handler) startup_irq_handler(irq, handler)
#define irq_set_enabled(irq, enabled) startup_irq_enable(irq, enabled)
#define tight_loop_contents() startup_wait()
#define NATIVE_TEST_EXTERNAL_ROUTER 1
#define NATIVE_TEST_EXTERNAL_IRQ 1
#include "native_hub_transport_fixture.c"

// This is a register/IRQ model, not a physical USB bus or Core1 timing model.
// The physical pull-up follows DIRECT when its override is enabled; otherwise
// it follows SIE_CTRL. In particular, a disabled SIE pull-up cannot mask a
// forced physical pull-up. Register bit writes and startup hardware/router
// boundaries sample this signal. Each scenario starts in fresh process BSS.
static bool attached, observer_initialized, observer_launched, observer_ready;
static bool routing_enabled, addresses_published, irq_enabled, enumeration_done;
static uint8_t routed_addresses[PROBE_ROUTER_SLOTS], routed_default;
static void (*installed_irq)(void);
static uint32_t ready_delay_us, observer_start_us, wait_us, attach_time_us;
static unsigned attach_edges, detach_edges, controller_resets, setup_irqs;
static bool observer_never_ready;

static bool physical_pullup(void) {
    if (usb_hw->phy_direct_override & USB_USBPHY_DIRECT_OVERRIDE_DP_PULLUP_EN_OVERRIDE_EN_BITS)
        return (usb_hw->phy_direct & USB_USBPHY_DIRECT_DP_PULLUP_EN_BITS) != 0;
    return (usb_hw->sie_ctrl & USB_SIE_CTRL_PULLUP_EN_BITS) != 0;
}

static uint8_t route_address(uint8_t address) {
    assert(routing_enabled && addresses_published);
    if (!address) return routed_default;
    for (uint8_t slot = 0; slot < PROBE_ROUTER_SLOTS; ++slot)
        if (routed_addresses[slot] == address) return slot;
    return NONE;
}

void native_test_service_interrupt(void) {
    if (native_test_interrupt_mask || servicing_interrupt || !irq_enabled ||
        !installed_irq || !(usb_hw->ints & usb_hw->inte)) return;
    servicing_interrupt = true;
    if (usb_hw->ints & USB_INTS_SETUP_REQ_BITS) ++setup_irqs;
    installed_irq();
    usb_hw->ints = 0;
    servicing_interrupt = false;
}

static void host_setup(uint8_t address, const tusb_control_request_t* request) {
    assert(attached && physical_pullup());
    uint8_t slot = route_address(address);
    assert(slot == 0 && "root request must route through the published address table");
    assert(native_hub_select_device(address,slot,UINT32_MAX / 2u));
    memcpy(usb_dpram->setup_packet,request,sizeof(*request));
    usb_hw->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
    usb_hw->ints = USB_INTS_SETUP_REQ_BITS;
    unsigned previous = setup_irqs;
    native_test_service_interrupt();
    assert(setup_irqs == previous+1u && "first host SETUP must reach the installed IRQ");
    native_hub_task();
    assert(!failed);
}

static void receive_descriptor(uint8_t type, uint16_t expected_length) {
    uint8_t response[PACKET];
    uint16_t length = 0;
    assert(native_test_in(0,response,&length,true));
    assert(length == expected_length && response[1] == type);
    if (type == TUSB_DESC_DEVICE) {
        assert(response[0] == 18 && response[4] == 9 && response[7] == PACKET);
    } else {
        assert(response[0] == 9 && response[2] == expected_length && response[3] == 0);
        assert(response[4] == 1 && response[5] == 1);
    }
    assert(native_test_out(0,NULL,0,true));
    assert(devices[0].control.stage == IDLE && !failed);
}

static void enumerate_at_attach(void) {
    // Do not reset/reinitialize the transport here: that could repair the very
    // startup state being tested. Deliver the first SETUP inside the physical
    // attach write, before native_hub_init has even returned to its caller.
    const tusb_control_request_t descriptor = {
        .bmRequestType = 0x80, .bRequest = TUSB_REQ_GET_DESCRIPTOR,
        .wValue = TUSB_DESC_DEVICE << 8, .wLength = 18,
    };
    host_setup(0,&descriptor);
    receive_descriptor(TUSB_DESC_DEVICE,18);

    const tusb_control_request_t set_address = {
        .bmRequestType = 0, .bRequest = TUSB_REQ_SET_ADDRESS, .wValue = 9,
    };
    host_setup(0,&set_address);
    assert(route_address(0) == 0 && route_address(9) == NONE);
    uint8_t response[PACKET];
    uint16_t length = 1;
    assert(native_test_in(0,response,&length,true) && length == 0);
    assert(route_address(9) == 0 && route_address(0) == NONE);
    assert(usb_hw->dev_addr_ctrl == 9);

    const tusb_control_request_t configuration = {
        .bmRequestType = 0x80, .bRequest = TUSB_REQ_GET_DESCRIPTOR,
        .wValue = TUSB_DESC_CONFIGURATION << 8, .wLength = 25,
    };
    host_setup(9,&configuration);
    receive_descriptor(TUSB_DESC_CONFIGURATION,25);
    enumeration_done = true;
}

static void observe_pullup(void) {
    bool visible = physical_pullup();
    if (visible == attached) return;
    attached = visible;
    if (!visible) {
        ++detach_edges;
        return;
    }
    ++attach_edges;
    attach_time_us = native_test_time_us;
    assert((usb_hw->main_ctrl & USB_MAIN_CTRL_CONTROLLER_EN_BITS) &&
           "physical attach preceded controller readiness");
    assert((usb_hw->sie_ctrl & USB_SIE_CTRL_EP0_INT_1BUF_BITS) &&
           "physical attach preceded EP0 readiness");
    assert(observer_ready && routing_enabled && addresses_published &&
           "physical attach preceded observer/address routing readiness");
    assert(routed_addresses[0] == 0 && routed_default == 0);
    for (unsigned slot = 1; slot < PROBE_ROUTER_SLOTS; ++slot)
        assert(routed_addresses[slot] == NONE);
    assert(irq_enabled && installed_irq && !native_test_interrupt_mask &&
           "physical attach preceded IRQ readiness");
    assert((usb_hw->inte & (USB_INTS_SETUP_REQ_BITS | USB_INTS_BUFF_STATUS_BITS |
                            USB_INTS_BUS_RESET_BITS)) ==
           (USB_INTS_SETUP_REQ_BITS | USB_INTS_BUFF_STATUS_BITS | USB_INTS_BUS_RESET_BITS));
    assert(started && "physical attach preceded foreground/pending-IRQ readiness");
    enumerate_at_attach();
}

static void startup_set_bits(volatile uint32_t* address, uint32_t bits) {
    (hw_set_bits)(address,bits);
    observe_pullup();
}

static void startup_clear_bits(volatile uint32_t* address, uint32_t bits) {
    (hw_clear_bits)(address,bits);
    observe_pullup();
}

static void startup_reset(uint32_t mask) {
    assert(mask == RESETS_RESET_USBCTRL_BITS);
    ++controller_resets;
    memset(usb_hw,0,sizeof(*usb_hw));
    observe_pullup();
}

static void startup_launch(void (*entry)(void)) {
    observe_pullup();
    assert(observer_initialized && entry == probe_router_core1);
    observer_launched = true;
}

static void startup_irq_handler(unsigned irq, void (*handler)(void)) {
    observe_pullup();
    assert(irq == USBCTRL_IRQ);
    installed_irq = handler;
}

static void startup_irq_enable(unsigned irq, bool enabled) {
    observe_pullup();
    assert(irq == USBCTRL_IRQ);
    irq_enabled = enabled;
    native_test_service_interrupt();
}

static void startup_wait(void) {
    observe_pullup();
    assert(!attached && "host must remain detached throughout observer startup");
    native_test_time_us += 1000u;
    wait_us += 1000u;
    assert(wait_us <= 100000u && "observer startup must have a bounded timeout");
}

void probe_router_init(uint32_t hz) {
    observe_pullup();
    assert(hz == 240000000u);
    observer_initialized = true;
    observer_start_us = native_test_time_us;
    memset(routed_addresses,NONE,sizeof(routed_addresses));
    routed_default = NONE;
}

void probe_router_core1(void) {}

void probe_router_publish(const uint8_t values[PROBE_ROUTER_SLOTS], uint8_t slot) {
    observe_pullup();
    memcpy(routed_addresses,values,sizeof(routed_addresses));
    routed_default = slot;
    addresses_published = true;
}

void probe_router_enable(bool enabled) {
    observe_pullup();
    routing_enabled = enabled;
}

bool probe_router_set_phase(uint32_t phase) {
    (void)phase;
    observe_pullup();
    return true;
}

void probe_router_snapshot(probe_router_stats* snapshot) {
    observe_pullup();
    assert(observer_initialized && observer_launched);
    if (!observer_never_ready && native_test_time_us-observer_start_us >= ready_delay_us)
        observer_ready = true;
    memset(snapshot,0,sizeof(*snapshot));
    snapshot->ready = observer_ready;
}

bool tud_vendor_control_xfer_cb(uint8_t slot, uint8_t stage, const tusb_control_request_t* request) {
    (void)slot; (void)stage; (void)request;
    assert(false && "root standard enumeration must not invoke vendor handling");
    return false;
}

void reset_usb_boot(uint32_t gpio_mask, uint32_t disable_mask) {
    (void)gpio_mask; (void)disable_mask;
    assert(false && "cold startup must not enter BOOTSEL");
    abort();
}

int main(int argc, char** argv) {
    assert(argc == 2);
    if (strcmp(argv[1],"delayed") == 0) ready_delay_us = 75000u;
    else if (strcmp(argv[1],"timeout") == 0) observer_never_ready = true;
    else assert(strcmp(argv[1],"ready") == 0);

    // No native_test_initialize/startup helper: call the actual initializer
    // from cold BSS, rather than inheriting the transport fixture's ready state.
    assert(!physical_pullup());
    bool initialized = native_hub_init();
    observe_pullup();
    assert(controller_resets == 1 && detach_edges == 0);
    if (observer_never_ready) {
        assert(!initialized && !started && !physical_pullup());
        assert(!attach_edges && !enumeration_done && !setup_irqs);
        assert(wait_us == 100000u);
    } else {
        assert(initialized && attached && attach_edges == 1);
        assert(enumeration_done && setup_irqs == 3);
        assert(wait_us == ready_delay_us && attach_time_us-observer_start_us == ready_delay_us);
        assert(startup_time == attach_time_us && !failed);
    }
    printf("native cold startup %s passed for %u children\n",argv[1],CHILDREN);
    return 0;
}
