#include "beacon_protocol.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

namespace wake_beacon {

void Protocol::connected(bool connected) {
    if (!connected) {
        // An interrupted line must never be completed by a later USB session.
        line_size_ = 0;
        invalid_line_ = false;
        response_size_ = 0;
        response_offset_ = 0;
    }
    connected_ = connected;
}

bool Protocol::can_receive() const {
    return connected_ && output_size() == 0;
}

bool Protocol::receive(uint8_t byte) {
    if (!can_receive()) {
        return false;
    }
    if (byte == '\n') {
        if (invalid_line_) {
            respond("malformed");
        } else {
            command();
        }
        line_size_ = 0;
        invalid_line_ = false;
    } else if (line_size_ == kMaxLineBytes ||
               (byte != '\r' && (byte < 0x20 || byte > 0x7e))) {
        // Discard through LF, never parse an overflowing suffix as a command.
        invalid_line_ = true;
    } else if (!invalid_line_) {
        line_[line_size_++] = static_cast<char>(byte);
    }
    return true;
}

const char* Protocol::output_data() const {
    return response_ + response_offset_;
}

size_t Protocol::output_size() const {
    return response_size_ - response_offset_;
}

void Protocol::consume_output(size_t count) {
    if (count > output_size()) {
        count = output_size();
    }
    response_offset_ += count;
}

bool Protocol::active() const {
    return state_ == State::Queued || state_ == State::Broadcasting;
}

bool Protocol::busy() const {
    return active() || (!radio_.failed && radio_.wake.busy);
}

const char* Protocol::state_name() const {
    switch (state_) {
        case State::Idle: return "idle";
        case State::Queued: return "queued";
        case State::Broadcasting: return "broadcasting";
        case State::Complete: return "complete";
        case State::Unconfigured: return "unconfigured";
        case State::Failed: return "failed";
    }
    return "failed";
}

void Protocol::observe(const RadioStatus& radio) {
    if (radio.failed && !radio_.failed) {
        ++local_failures_;
    }
    radio_ = radio;
    if (active()) {
        if (radio_.failed) {
            state_ = State::Failed;
        } else if (radio_.initialized && !radio_.wake.configured) {
            state_ = State::Unconfigured;
        } else if (state_ == State::Broadcasting) {
            // A failed stop may still complete during cleanup. Failure wins.
            if (radio_.wake.failures != start_failures_) {
                state_ = State::Failed;
            } else if (radio_.wake.completed_bursts != start_completed_ &&
                       !radio_.wake.busy) {
                state_ = State::Complete;
            }
        }
    } else if (request_id_ == 0) {
        if (radio_.failed) {
            state_ = State::Failed;
        } else if (radio_.initialized && !radio_.wake.configured) {
            state_ = State::Unconfigured;
        }
    }
}

bool Protocol::dispatch_pending() const {
    return state_ == State::Queued && radio_.ready && !radio_.failed &&
           radio_.wake.configured && !radio_.wake.busy;
}

void Protocol::dispatched(bool accepted) {
    if (!dispatch_pending()) {
        return;
    }
    if (accepted) {
        start_completed_ = radio_.wake.completed_bursts;
        start_failures_ = radio_.wake.failures;
        state_ = State::Broadcasting;
    } else {
        ++local_failures_;
        state_ = State::Failed;
    }
}

void Protocol::command() {
    size_t length = line_size_;
    if (length != 0 && line_[length - 1] == '\r') {
        --length;
    }
    constexpr char status[] = "SPWB1 STATUS";
    constexpr char wake[] = "SPWB1 WAKE ";
    if (length == sizeof(status) - 1 &&
        memcmp(line_, status, sizeof(status) - 1) == 0) {
        respond("");
        return;
    }
    if (length <= sizeof(wake) - 1 ||
        memcmp(line_, wake, sizeof(wake) - 1) != 0) {
        respond("malformed");
        return;
    }
    uint32_t id = 0;
    constexpr uint32_t max_id = 0x7fffffff;
    for (size_t index = sizeof(wake) - 1; index < length; ++index) {
        const char digit = line_[index];
        if (digit < '0' || digit > '9' ||
            id > (max_id - static_cast<uint32_t>(digit - '0')) / 10) {
            respond("malformed");
            return;
        }
        id = id * 10 + static_cast<uint32_t>(digit - '0');
    }
    if (id == 0) {
        respond("malformed");
    } else if (id == request_id_) {
        // Retain idempotency across disconnects, failures and completion.
        respond("");
    } else if (busy()) {
        respond("busy");
    } else if (radio_.failed) {
        respond("radio_init_failed");
    } else {
        request_id_ = id;
        state_ = State::Queued;
        ++accepted_requests_;
        respond("");
    }
}

void Protocol::respond(const char* error) {
    const int length = snprintf(
        response_, sizeof(response_),
        "SPWB1 {\"protocol\":1,\"role\":\"wake-only\",\"firmware\":\"1.0.0\","
        "\"radio_ready\":%s,\"controller_hosting\":false,"
        "\"request_id\":%" PRIu32 ",\"state\":\"%s\",\"configured\":%s,"
        "\"busy\":%s,\"accepted_requests\":%" PRIu32 ","
        "\"completed_bursts\":%" PRIu32 ",\"failures\":%" PRIu32 ","
        "\"error\":\"%s\"}\n",
        radio_.ready ? "true" : "false", request_id_, state_name(),
        radio_.wake.configured ? "true" : "false", busy() ? "true" : "false",
        accepted_requests_, radio_.wake.completed_bursts,
        radio_.wake.failures + local_failures_, error);
    // All strings are fixed literals and even maximum counters fit in 512 bytes.
    response_size_ = length > 0 && static_cast<size_t>(length) < sizeof(response_)
                         ? static_cast<size_t>(length)
                         : 0;
    response_offset_ = 0;
}

}  // namespace wake_beacon
