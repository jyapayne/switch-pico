#include <inttypes.h>
#include <stdio.h>

#include "hardware/clocks.h"
#include "hardware/structs/sio.h"
#include "hardware/structs/usb.h"
#include "hardware/vreg.h"
#include "hardware/watchdog.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "router.h"
#include "usb_probe.h"

#if !PICO_NO_FLASH
#error "The native USB address probe must run from RAM, never replace flash firmware"
#endif
#if !PICO_RP2350
#error "The native USB address probe requires RP2350"
#endif

int main(void) {
    // A failed USB experiment must not strand the board in this RAM program.
    // Explicit host GET_STATS requests are the only keepalive after startup.
    watchdog_enable(8000, false);
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    sleep_ms(10);
    set_sys_clock_khz(240000, true);
    stdio_init_all();
    printf("\n[HUBPROBE] RAM-only built-in USB address experiment, clock=%" PRIu32 " Hz\n",
           clock_get_hz(clk_sys));
    printf("[HUBPROBE] No GPIO data wiring, Bluetooth, or flash writes; watchdog returns to stored firmware\n");

    probe_router_init(clock_get_hz(clk_sys));
    multicore_launch_core1(probe_router_core1);
    const uint32_t start = time_us_32();
    probe_router_stats observer = {0};
    do {
        probe_router_snapshot(&observer);
        if (observer.ready) break;
        sleep_us(10);
    } while ((uint32_t)(time_us_32() - start) < 100000);
    printf("[HUBPROBE] Observer ready=%" PRIu32 " cycles/bit=%" PRIu32 "\n",
           observer.ready, observer.cycles_per_bit);
    probe_hub_init();
#if PROBE_USB_GPIO_MODE
    // With TO_PHY retained and SIO outputs disabled, hardware measurements
    // showed both live SIO inputs and successful native-controller enumeration.
    // Keep only the internal full-speed attachment resistor; do not drive data.
    sio_hw->gpio_hi_oe_clr = SIO_GPIO_HI_IN_USB_DP_BITS | SIO_GPIO_HI_IN_USB_DM_BITS;
    hw_set_bits(&usb_hw->phy_direct, USB_USBPHY_DIRECT_DP_PULLUP_EN_BITS);
    hw_set_bits(&usb_hw->phy_direct_override,
                USB_USBPHY_DIRECT_OVERRIDE_DP_PULLUP_EN_OVERRIDE_EN_BITS);
    hw_set_bits(&usb_hw->muxing, USB_USB_MUXING_USBPHY_AS_GPIO_BITS);
    printf("[HUBPROBE] USBPHY_AS_GPIO plus TO_PHY; SIO outputs disabled, internal pull-up retained\n");
#endif
    printf("[HUBPROBE] RX=SIO GPIO_HI_IN[25:24], mux=%08" PRIx32 "; SIO=%08" PRIx32
           " PHY=%08" PRIx32 "\n", usb_hw->muxing, sio_hw->gpio_hi_in, usb_hw->phy_direct);
    while (true) {
        probe_hub_task();
        sleep_us(100);
    }
}
