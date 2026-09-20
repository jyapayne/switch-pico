#include <cstdlib>
#include <iostream>
#include <string>

#include "beacon_protocol.h"

namespace {

using wake_beacon::Protocol;
using wake_beacon::RadioStatus;

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

void field(const std::string& response, const std::string& key,
           const std::string& value) {
    const auto token = "\"" + key + "\":" + value;
    require(response.find(token + ",") != std::string::npos ||
                response.find(token + "}") != std::string::npos,
            ("unexpected " + key + " in " + response).c_str());
}

void feed(Protocol& protocol, const std::string& input) {
    for (unsigned char byte : input) {
        require(protocol.receive(byte), "unexpected receive backpressure");
    }
}

std::string drain(Protocol& protocol) {
    const std::string response(protocol.output_data(), protocol.output_size());
    protocol.consume_output(response.size());
    require(response.rfind("SPWB1 {", 0) == 0 && response.back() == '\n',
            "missing complete framed JSON response");
    require(response.find('\n') == response.size() - 1,
            "multiple or unsolicited response lines");
    return response;
}

std::string request(Protocol& protocol, const std::string& line) {
    feed(protocol, line);
    return drain(protocol);
}

RadioStatus ready() {
    RadioStatus radio;
    radio.ready = true;
    radio.initialized = true;
    radio.wake.configured = true;
    return radio;
}

void malformed_and_framing() {
    Protocol protocol;
    protocol.observe(ready());
    protocol.connected(true);
    const std::string invalid[] = {
        "\n", "SPWB1 WAKE 0\n", "SPWB1 WAKE -1\n",
        "SPWB1 WAKE +1\n", "SPWB1 WAKE 2147483648\n",
        "SPWB1 WAKE 999999999999999999999\n", "SPWB1 WAKE 1x\n",
        "SPWB1 WAKE 1 \n", "SPWB1 WAKE 1\r\r\n",
        "SPWB1 WAKE 1\rSTATUS\n", "SPWB2 WAKE 1\n",
        std::string("SPWB1 WAKE 1\0", 13) + "\n",
        "SPWB1 WAKE " + std::string(53, '0') + "1\n",
        std::string(65, 'x') + "SPWB1 WAKE 1\n",
    };
    for (const auto& line : invalid) {
        const auto response = request(protocol, line);
        field(response, "error", "\"malformed\"");
        field(response, "accepted_requests", "0");
        field(response, "request_id", "0");
        require(!protocol.dispatch_pending(), "malformed request queued wake");
    }
    feed(protocol, "SPWB1 WAKE 2147483647\r");
    require(!protocol.dispatch_pending() && protocol.output_size() == 0,
            "request acted before newline");
    feed(protocol, "\n");
    const auto response = drain(protocol);
    field(response, "request_id", "2147483647");
    field(response, "state", "\"queued\"");
    require(protocol.dispatch_pending(), "valid boundary ID not queued");
}

void exact_line_boundary() {
    Protocol protocol;
    protocol.observe(ready());
    protocol.connected(true);
    const auto response = request(
        protocol, "SPWB1 WAKE " + std::string(52, '0') + "1\n");
    field(response, "error", "\"\"");
    field(response, "request_id", "1");
    require(protocol.dispatch_pending(), "64-byte line rejected");
}

void readonly_and_readiness() {
    Protocol protocol;
    protocol.connected(true);
    require(protocol.output_size() == 0 && !protocol.dispatch_pending(),
            "connection caused output or wake");
    field(request(protocol, "SPWB1 STATUS\r\n"), "state", "\"idle\"");
    require(!protocol.dispatch_pending(), "STATUS caused wake");
    auto response = request(protocol, "SPWB1 WAKE 19\n");
    field(response, "state", "\"queued\"");
    field(response, "radio_ready", "false");
    require(!protocol.dispatch_pending(), "wake dispatched before readiness");
    protocol.observe(ready());
    require(protocol.dispatch_pending(), "queued wake lost during startup");
    protocol.dispatched(true);
    require(!protocol.dispatch_pending(), "accepted wake dispatched twice");
    response = request(protocol, "SPWB1 STATUS\n");
    field(response, "state", "\"broadcasting\"");
    field(response, "accepted_requests", "1");
}

void replay_and_busy() {
    Protocol protocol;
    auto radio = ready();
    protocol.observe(radio);
    protocol.connected(true);
    request(protocol, "SPWB1 WAKE 23\n");
    auto response = request(protocol, "SPWB1 WAKE 24\n");
    field(response, "error", "\"busy\"");
    field(response, "request_id", "23");
    field(response, "state", "\"queued\"");
    response = request(protocol, "SPWB1 WAKE 00023\n");
    field(response, "error", "\"\"");
    field(response, "accepted_requests", "1");
    protocol.dispatched(true);
    radio.wake.busy = true;
    protocol.observe(radio);
    protocol.connected(false);
    protocol.connected(true);
    response = request(protocol, "SPWB1 WAKE 23\n");
    field(response, "state", "\"broadcasting\"");
    field(response, "accepted_requests", "1");
    require(!protocol.dispatch_pending(), "active replay restarted wake");
    radio.wake.busy = false;
    radio.wake.completed_bursts = 1;
    protocol.observe(radio);
    response = request(protocol, "SPWB1 WAKE 23\n");
    field(response, "state", "\"complete\"");
    field(response, "accepted_requests", "1");
    require(!protocol.dispatch_pending(), "terminal replay restarted wake");
    field(request(protocol, "SPWB1 WAKE 24\n"), "state", "\"queued\"");
    require(protocol.dispatch_pending(), "new ID did not start next wake");
}

void disconnect_and_backpressure() {
    Protocol protocol;
    protocol.observe(ready());
    protocol.connected(true);
    feed(protocol, "SPWB1 WAKE ");
    protocol.connected(false);
    require(!protocol.receive('7'), "disconnected input accepted");
    protocol.connected(true);
    field(request(protocol, "7\n"), "error", "\"malformed\"");
    require(!protocol.dispatch_pending(), "disconnect joined partial request");

    feed(protocol, "SPWB1 WAKE 7\n");
    const std::string queued(protocol.output_data(), protocol.output_size());
    require(!protocol.receive('S'), "backpressured request accepted");
    protocol.consume_output(7);
    require(!protocol.receive('\n'), "partial TX released backpressure");
    protocol.dispatched(true);
    auto radio = ready();
    radio.wake.completed_bursts = 1;
    protocol.observe(radio);
    require(std::string(protocol.output_data(), protocol.output_size()) ==
                queued.substr(7),
            "radio completion corrupted in-flight queued response");
    protocol.connected(false);
    require(protocol.output_size() == 0, "disconnect retained partial TX");
    protocol.connected(true);
    auto response = request(protocol, "SPWB1 STATUS\n");
    field(response, "state", "\"complete\"");
    field(response, "request_id", "7");
    require(!protocol.dispatch_pending(), "reconnect caused another wake");
}

void failed_cleanup_is_not_success() {
    Protocol protocol;
    auto radio = ready();
    // Counter wrap is also a legitimate change, not an ordering comparison.
    radio.wake.failures = UINT32_MAX;
    radio.wake.completed_bursts = UINT32_MAX;
    protocol.observe(radio);
    protocol.connected(true);
    request(protocol, "SPWB1 WAKE 31\n");
    protocol.dispatched(true);
    radio.wake.failures = 0;
    radio.wake.completed_bursts = 0;
    radio.wake.busy = true;
    protocol.observe(radio);
    auto response = request(protocol, "SPWB1 STATUS\n");
    field(response, "state", "\"failed\"");
    field(response, "error", "\"\"");
    field(response, "busy", "true");
    field(request(protocol, "SPWB1 WAKE 32\n"), "error", "\"busy\"");
    radio.wake.busy = false;
    protocol.observe(radio);
    response = request(protocol, "SPWB1 WAKE 31\n");
    field(response, "state", "\"failed\"");
    require(!protocol.dispatch_pending(), "failed replay retried burst");
    field(request(protocol, "SPWB1 WAKE 32\n"), "state", "\"queued\"");
    protocol.dispatched(false);
    field(request(protocol, "SPWB1 STATUS\n"), "state", "\"failed\"");
    require(!protocol.dispatch_pending(), "dispatch refusal retried burst");
}

void startup_failure_and_missing_configuration() {
    Protocol protocol;
    protocol.connected(true);
    request(protocol, "SPWB1 WAKE 41\n");
    RadioStatus radio;
    radio.failed = true;
    radio.wake.busy = true;  // Software may still report busy after radio shutdown.
    protocol.observe(radio);
    auto response = request(protocol, "SPWB1 STATUS\n");
    field(response, "request_id", "41");
    field(response, "state", "\"failed\"");
    field(response, "error", "\"\"");
    field(response, "busy", "false");
    field(response, "failures", "1");
    protocol.observe(radio);
    response = request(protocol, "SPWB1 WAKE 42\n");
    field(response, "error", "\"radio_init_failed\"");
    field(response, "failures", "1");
    field(response, "request_id", "41");
    require(!protocol.dispatch_pending(), "failed radio dispatched wake");

    Protocol unconfigured;
    unconfigured.connected(true);
    request(unconfigured, "SPWB1 WAKE 1\n");
    radio = {};
    radio.initialized = true;
    unconfigured.observe(radio);
    response = request(unconfigured, "SPWB1 STATUS\n");
    field(response, "state", "\"unconfigured\"");
    field(response, "configured", "false");
    require(!unconfigured.dispatch_pending(), "unconfigured wake dispatched");
}

}  // namespace

int main() {
    malformed_and_framing();
    exact_line_boundary();
    readonly_and_readiness();
    replay_and_busy();
    disconnect_and_backpressure();
    failed_cleanup_is_not_success();
    startup_failure_and_missing_configuration();
    return 0;
}
