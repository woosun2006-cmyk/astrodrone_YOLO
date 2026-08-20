#pragma once

#include <string>

#include "runtime_transport.hpp"

namespace autopilot {

struct RealFlightOptions {
    std::string router_serial;
    std::string command_endpoint;
    std::string telemetry_endpoint;
    bool allow_arm = false;
    bool confirm_real_flight = false;
    bool commands_enabled = false;
    bool router_ownership_confirmed = false;
};

struct RealFlightDecision {
    bool allowed = false;
    std::string reason;
};

// Validates the explicit real-flight boundary without opening serial or a
// socket. The launcher performs the existence check; this function validates
// the policy and endpoint shape used by C++ tools and tests.
RealFlightDecision validate_real_flight(const RuntimeTransportConfig& config,
                                        const RealFlightOptions& options);

}  // namespace autopilot
