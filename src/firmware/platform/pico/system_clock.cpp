#include "platform/pico/system_clock.h"

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/powman.h"
#include "hardware/regs/clocks.h"
#include "hardware/regs/qmi.h"
#include "hardware/structs/qmi.h"
#include "hardware/vreg.h"
#include "pico/stdlib.h"

namespace {
SystemClockStatus g_status{};
}

void system_clock_initialize() {
    static_assert(SWITCH_PICO_SYS_CLOCK_MHZ == 150 ||
                  SWITCH_PICO_SYS_CLOCK_MHZ == 300 ||
                  SWITCH_PICO_SYS_CLOCK_MHZ == 400);
    // Flash timing was established by boot stage 2. Do not raise clk_sys if
    // another boot configuration failed to provide the required divider.
    const uint32_t flash_divider =
        (qmi_hw->m[0].timing & QMI_M0_TIMING_CLKDIV_BITS) >>
        QMI_M0_TIMING_CLKDIV_LSB;
    if (flash_divider < (SWITCH_PICO_SYS_CLOCK_MHZ + 74u) / 75u) {
        panic("unsafe overclock flash divider");
    }
#if SWITCH_PICO_SYS_CLOCK_MHZ == 150
    constexpr auto voltage = VREG_VOLTAGE_1_10;
#elif SWITCH_PICO_OVERCLOCK_MV == 1400
    constexpr auto voltage = VREG_VOLTAGE_1_40;
    vreg_disable_voltage_limit();
#else
    constexpr auto voltage = VREG_VOLTAGE_1_30;
#endif
    if (vreg_get_voltage() != voltage) {
        vreg_set_voltage(voltage);
        sleep_us(1000);
    }
#if SWITCH_PICO_SYS_CLOCK_MHZ != 400 || SWITCH_PICO_OVERCLOCK_MV != 1400
    // A warm reboot from the explicit 1.4 V image must not leave its lifted
    // voltage limit behind. Lower voltage before reinstating the limit.
    powman_clear_bits(&powman_hw->vreg_ctrl,
                      POWMAN_VREG_CTRL_DISABLE_VOLTAGE_LIMIT_BITS);
#endif
#if SWITCH_PICO_SYS_CLOCK_MHZ != 150
    set_sys_clock_khz(SWITCH_PICO_SYS_CLOCK_MHZ * 1000u, true);
#endif
    g_status.requested_sys_khz = SWITCH_PICO_SYS_CLOCK_MHZ * 1000u;
    g_status.measured_sys_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    g_status.measured_usb_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
    // Regulator setting, not an externally calibrated voltage measurement.
    g_status.core_voltage_mv = 550u + 50u * vreg_get_voltage();
    g_status.flash_clock_divider = flash_divider;
    g_status.cyw43_pio_divider256 =
        CYW43_PIO_CLOCK_DIV_INT * 256u + CYW43_PIO_CLOCK_DIV_FRAC8;
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);
}

SystemClockStatus system_clock_status() {
    uint32_t sum = 0;
    for (unsigned sample = 0; sample < 8; ++sample) sum += adc_read();
    // Datasheet nominal transfer: 27 C - (Vadc - 0.706 V) / 0.001721.
    // The 3.3 V reference and sensor offset make this an estimate, not a
    // calibrated thermal qualification. Use signed integer microvolts.
    const int32_t microvolts = static_cast<int32_t>(
        (uint64_t{sum} * 3300000u + 16384u) / 32768u);
    g_status.temperature_millicelsius =
        27000 - static_cast<int32_t>((int64_t{microvolts - 706000} * 1000) / 1721);
    return g_status;
}
