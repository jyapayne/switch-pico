#pragma once

#include <stddef.h>
#include <stdint.h>

#include "input/switch2_wake.h"

namespace wake_beacon {

struct RadioStatus {
    bool ready = false;
    bool failed = false;
    bool initialized = false;
    Switch2WakeDiagnostics wake{};
};

// Owned entirely by Core 0. Parsing only queues a request; the radio owner calls
// dispatch_pending()/dispatched() separately, outside USB callbacks and IRQs.
class Protocol {
public:
    static constexpr size_t kMaxLineBytes = 64;  // Excludes LF, includes optional CR.
    static constexpr size_t kResponseBytes = 512;

    void connected(bool connected);
    bool can_receive() const;
    bool receive(uint8_t byte);
    const char* output_data() const;
    size_t output_size() const;
    void consume_output(size_t count);

    void observe(const RadioStatus& radio);
    bool dispatch_pending() const;
    void dispatched(bool accepted);

private:
    enum class State : uint8_t {
        Idle, Queued, Broadcasting, Complete, Unconfigured, Failed,
    };

    bool active() const;
    bool busy() const;
    const char* state_name() const;
    void command();
    void respond(const char* error);

    RadioStatus radio_{};
    State state_ = State::Idle;
    uint32_t request_id_ = 0;
    uint32_t accepted_requests_ = 0;
    uint32_t local_failures_ = 0;
    uint32_t start_completed_ = 0;
    uint32_t start_failures_ = 0;
    bool connected_ = false;
    bool invalid_line_ = false;
    char line_[kMaxLineBytes]{};
    size_t line_size_ = 0;
    char response_[kResponseBytes]{};
    size_t response_size_ = 0;
    size_t response_offset_ = 0;
};

}  // namespace wake_beacon
