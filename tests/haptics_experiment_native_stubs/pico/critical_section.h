#pragma once

#include <cassert>

struct critical_section_t {
    bool initialized = false;
};

inline unsigned native_haptics_lock_depth = 0;

inline void critical_section_init(critical_section_t* section) {
    assert(!section->initialized);
    section->initialized = true;
}
inline void critical_section_enter_blocking(critical_section_t* section) {
    assert(section->initialized);
    assert(native_haptics_lock_depth == 0);
    ++native_haptics_lock_depth;
}
inline void critical_section_exit(critical_section_t*) {
    assert(native_haptics_lock_depth == 1);
    --native_haptics_lock_depth;
}
