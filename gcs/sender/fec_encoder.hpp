#pragma once

// Interleaved single-parity XOR FEC encoder.
//
// Each call to submit() assigns the payload to the next lane round-robin
// (lane = seq % lanes), so a run of consecutive losses on the wire lands in
// different lanes' groups instead of wiping one group out entirely. The
// DATA packet for a payload is emitted immediately -- it never waits on its
// group to fill, so latency is unaffected by FEC. Once a lane collects
// group_size DATA payloads, a single PARITY packet (XOR of the group's
// payloads, zero-padded to the longest one) is emitted and the lane resets
// for its next group.
//
// Only group_size data packets + 1 parity packet per group is implemented
// (recovers exactly one loss per group). parity_count is carried in every
// header for forward compatibility but must be 1 for now -- see
// fec_packet.hpp.

#include <cstdint>
#include <functional>
#include <vector>

#include "fec_packet.hpp"

namespace gcs {

class FecEncoder {
public:
    using SendFn = std::function<void(const uint8_t* data, size_t len)>;

    // lanes: interleave depth (how many groups are in flight at once).
    // group_size: data packets per group before a parity packet fires.
    FecEncoder(uint8_t lanes, uint8_t group_size, SendFn send);

    // payload/len must fit in kMaxPayload. Emits one DATA packet now, and a
    // PARITY packet whenever this call completes a group.
    void submit(const uint8_t* payload, uint16_t len);

private:
    struct Lane {
        uint16_t group_index = 0;
        std::vector<std::vector<uint8_t>> pending;
        uint16_t max_len = 0;
    };

    void emit(uint8_t type, uint8_t lane, uint16_t lane_group, uint8_t group_seq,
              const uint8_t* payload, uint16_t len);

    std::vector<Lane> lanes_;
    uint8_t group_size_;
    uint32_t next_seq_ = 0;
    uint8_t next_lane_ = 0;
    SendFn send_;
};

}  // namespace gcs
