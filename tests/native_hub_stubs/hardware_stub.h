#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define __not_in_flash_func(name) name
#define __no_inline_not_in_flash_func(name) __attribute__((noinline)) name
#define __force_inline inline __attribute__((always_inline))
#define __dmb() ((void)0)

typedef struct { unsigned unused; } spin_lock_t;
extern uint32_t native_test_interrupt_mask;
void native_test_service_interrupt(void);
static inline uint32_t save_and_disable_interrupts(void) {
    uint32_t flags = native_test_interrupt_mask;
    native_test_interrupt_mask = 1;
    return flags;
}
static inline void restore_interrupts(uint32_t flags) {
    native_test_interrupt_mask = flags;
    native_test_service_interrupt();
}
static inline uint32_t spin_lock_blocking(spin_lock_t* lock) {
    (void)lock; return save_and_disable_interrupts();
}
static inline void spin_unlock(spin_lock_t* lock, uint32_t flags) {
    (void)lock; restore_interrupts(flags);
}
static inline bool spin_try_lock_unsafe(spin_lock_t* lock) { (void)lock; return true; }
static inline void spin_unlock_unsafe(spin_lock_t* lock) { (void)lock; }
static inline int spin_lock_claim_unused(bool required) { (void)required; return 0; }
static inline spin_lock_t* spin_lock_instance(unsigned index) {
    static spin_lock_t lock; (void)index; return &lock;
}
static inline void hw_clear_bits(volatile uint32_t* address, uint32_t bits) { *address &= ~bits; }

typedef struct {
    volatile uint32_t ints, sie_status, buf_status, dev_addr_ctrl, inte;
    volatile uint32_t ep_stall_arm, muxing, phy_direct, phy_direct_override;
    volatile uint32_t pwr, main_ctrl, sie_ctrl, ep_nak_stall_status;
    volatile uint32_t ep_tx_error, ep_rx_error;
    volatile uint32_t abort, abort_done;
} usb_hw_t;
typedef struct { volatile uint32_t in, out; } usb_pair_t;
typedef struct {
    uint8_t setup_packet[8];
    usb_pair_t ep_ctrl[15];
    usb_pair_t ep_buf_ctrl[16];
    uint8_t ep0_buf_a[64];
    uint8_t padding[3776];
} usb_device_dpram_t;
typedef struct { volatile uint32_t mtime, gpio_hi_oe_clr; } sio_hw_t;
extern usb_hw_t native_test_usb;
extern usb_device_dpram_t native_test_dpram;
extern sio_hw_t native_test_sio;
#define usb_hw (&native_test_usb)
#define usb_dpram (&native_test_dpram)
#define sio_hw (&native_test_sio)
#define USBCTRL_DPRAM_BASE ((uintptr_t)usb_dpram)
#define USB_DPRAM_SIZE sizeof(*usb_dpram)

extern bool native_test_abort_stuck;
static inline void hw_set_bits(volatile uint32_t* address, uint32_t bits) {
    *address |= bits;
    if (address == &usb_hw->abort && !native_test_abort_stuck)
        usb_hw->abort_done |= bits;
}

#define USB_BUF_CTRL_LEN_MASK 0x3ffu
#define USB_BUF_CTRL_AVAIL (1u << 10)
#define USB_BUF_CTRL_STALL (1u << 11)
#define USB_BUF_CTRL_SEL (1u << 12)
#define USB_BUF_CTRL_DATA1_PID (1u << 13)
#define USB_BUF_CTRL_LAST (1u << 14)
#define USB_BUF_CTRL_FULL (1u << 15)
#define EP_CTRL_ENABLE_BITS (1u << 31)
#define EP_CTRL_INTERRUPT_PER_BUFFER (1u << 29)
#define EP_CTRL_INTERRUPT_ON_NAK (1u << 16)
#define EP_CTRL_BUFFER_TYPE_LSB 26
#define USB_INTS_BUS_RESET_BITS (1u << 0)
#define USB_INTS_SETUP_REQ_BITS (1u << 1)
#define USB_INTS_DEV_SUSPEND_BITS (1u << 2)
#define USB_INTS_DEV_RESUME_FROM_HOST_BITS (1u << 3)
#define USB_INTS_BUFF_STATUS_BITS (1u << 4)
#define USB_SIE_STATUS_SETUP_REC_BITS (1u << 0)
#define USB_SIE_STATUS_BUS_RESET_BITS (1u << 1)
#define USB_SIE_STATUS_SUSPENDED_BITS (1u << 2)
#define USB_SIE_STATUS_RESUME_BITS (1u << 3)
#define USB_USB_MUXING_TO_PHY_BITS 1u
#define USB_USB_MUXING_SOFTCON_BITS 2u
#define USB_USB_MUXING_USBPHY_AS_GPIO_BITS 4u
#define SIO_GPIO_HI_IN_USB_DP_BITS 1u
#define SIO_GPIO_HI_IN_USB_DM_BITS 2u
#define USB_USBPHY_DIRECT_DP_PULLUP_EN_BITS 1u
#define USB_USBPHY_DIRECT_OVERRIDE_DP_PULLUP_EN_OVERRIDE_EN_BITS 1u
#define USB_USB_PWR_VBUS_DETECT_BITS 1u
#define USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS 2u
#define USB_MAIN_CTRL_CONTROLLER_EN_BITS 1u
#define USB_SIE_CTRL_EP0_INT_1BUF_BITS 1u
#define USB_SIE_CTRL_PULLUP_EN_BITS 2u
#define RESETS_RESET_USBCTRL_BITS 1u
#define USBCTRL_IRQ 0u
#define clk_sys 0u

static inline uint32_t clock_get_hz(unsigned clock) { (void)clock; return 240000000u; }
static inline void reset_block(uint32_t mask) { (void)mask; }
static inline void unreset_block_wait(uint32_t mask) { (void)mask; }
static inline void multicore_launch_core1(void (*entry)(void)) { (void)entry; }
static inline void irq_set_exclusive_handler(unsigned irq, void (*fn)(void)) { (void)irq; (void)fn; }
static inline void irq_set_priority(unsigned irq, unsigned priority) { (void)irq; (void)priority; }
static inline void irq_set_enabled(unsigned irq, bool enabled) { (void)irq; (void)enabled; }
static inline uint32_t time_us_32(void) { return 1000000u; }
static inline bool watchdog_enable_caused_reboot(void) { return false; }
static inline void watchdog_enable(uint32_t ms, bool pause) { (void)ms; (void)pause; }
static inline void watchdog_update(void) {}
static inline void stdio_init_all(void) {}
static inline void sleep_ms(uint32_t ms) { (void)ms; }
static inline void tight_loop_contents(void) {}
static inline void pico_get_unique_board_id_string(char* buffer, size_t size) {
    if (size) buffer[0] = '\0';
}
#ifdef __cplusplus
extern "C" {
#endif
void reset_usb_boot(uint32_t gpio_mask, uint32_t disable_mask);
int probe_debug_printf(const char* format, ...);
#ifdef __cplusplus
}
#endif
