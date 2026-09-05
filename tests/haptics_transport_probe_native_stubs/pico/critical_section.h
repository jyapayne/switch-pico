#pragma once

#include <cassert>

struct critical_section_t {
    bool initialized = false;
};
inline unsigned native_probe_lock_depth = 0;
inline void critical_section_init(critical_section_t* section) {
    assert(!section->initialized);
    section->initialized = true;
}
inline void critical_section_enter_blocking(critical_section_t* section) {
    assert(section->initialized && native_probe_lock_depth == 0);
    ++native_probe_lock_depth;
}
inline void critical_section_exit(critical_section_t*) {
    assert(native_probe_lock_depth == 1);
    --native_probe_lock_depth;
}
