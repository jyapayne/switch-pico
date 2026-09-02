#pragma once

#include <stdint.h>

class AdapterHostProbeState {
  public:
    static constexpr uint32_t kRebootDelayMs = 100;

    void note_ms_os_string() { saw_ms_os_string_ = true; }

    void note_ms_compat_id_request(uint32_t now_ms) {
        if (!saw_ms_os_string_) {
            return;
        }
        confirmed_windows_ = true;
        reboot_deadline_ms_ = now_ms + kRebootDelayMs;
    }

    bool windows_confirmed() const { return confirmed_windows_; }

    bool should_reboot(uint32_t now_ms) const {
        return confirmed_windows_ &&
               static_cast<int32_t>(now_ms - reboot_deadline_ms_) >= 0;
    }

  private:
    uint32_t reboot_deadline_ms_ = 0;
    bool saw_ms_os_string_ = false;
    bool confirmed_windows_ = false;
};
