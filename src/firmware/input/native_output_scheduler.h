#pragma once

#include <stdint.h>

struct uni_hid_device_s;
using NativeOutputGrant = bool (*)(uni_hid_device_s*, uint16_t, uint32_t);

// Fixed capacity: one native output client per physical controller, four total.
// prepare() precedes Core 1 launch; all other calls belong to BTstack/Core 1.
// Callbacks run without locks, at most one grant at a time. A callback's bool
// retains the existing generic-FIFO consumption convention (Sony true, Nintendo
// false). The grantee MUST complete() after its single send attempt or an abort;
// a grant must never be held across a timer/event-loop wait.
void native_output_scheduler_prepare();
uint8_t native_output_scheduler_request(uni_hid_device_s* device,
                                        uint32_t generation,
                                        uint64_t deadline_us,
                                        bool urgent_stop,
                                        NativeOutputGrant grant);
// Announce the next periodic deadline without requesting an early send. The
// arbiter leaves the last free controller ACL credit for an earlier announced
// deadline, with rotating ties, except urgent stops. UINT64_MAX clears this
// reservation without canceling a pending request. This is host admission, not
// an on-air reservation.
// This update never invokes a grant callback synchronously.
void native_output_scheduler_reserve(uni_hid_device_s* device,
                                     uint32_t generation,
                                     uint64_t deadline_us);
void native_output_scheduler_complete(uni_hid_device_s* device, uint32_t generation);
void native_output_scheduler_cancel(uni_hid_device_s* device);
bool native_output_scheduler_granted(const uni_hid_device_s* device);
bool native_output_scheduler_on_can_send_now(uni_hid_device_s* device,
                                            uint16_t cid);
