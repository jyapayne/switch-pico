#pragma once
struct critical_section_t {};
inline unsigned next_striped_spin_lock_num() { return 16; }
inline void critical_section_init_with_lock_num(critical_section_t*, unsigned) {}
inline void critical_section_enter_blocking(critical_section_t*) {}
inline void critical_section_exit(critical_section_t*) {}
