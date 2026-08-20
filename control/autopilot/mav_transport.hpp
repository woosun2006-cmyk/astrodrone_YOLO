#pragma once

// Compatibility boundary. The implementation remains in the existing root
// source during this migration step; new code should include this header.
#include "../mav_transport.hpp"

namespace autopilot {
using Transport = ::Transport;
using ::open_transport;
}  // namespace autopilot
