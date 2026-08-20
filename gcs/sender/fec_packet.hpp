#pragma once

// Wire format shared by the Jetson sender and any decoder (reference Python
// decoder under gcs/tools/, later the laptop GCS). A telemetry message is
// split across `lanes` interleaved FEC groups round-robin by sequence
// number, so a burst loss that hits several consecutive packets lands in
// different groups instead of wiping one group out. Each group is
// group_size DATA packets + parity_count PARITY packets (XOR of the
// group's payloads, zero-padded to the longest payload in the group).
//
// Only parity_count == 1 (single XOR parity, recovers exactly one loss per
// group) is implemented right now -- the header carries parity_count so a
// second parity scheme can be added later without changing the wire format.

#include <cstddef>
#include <cstdint>

namespace gcs {

constexpr uint32_t kMagic = 0x31544741;  // "AGT1" little-endian on disk
constexpr uint8_t kTypeData = 0;
constexpr uint8_t kTypeParity = 1;
constexpr size_t kMaxPayload = 1200;

#pragma pack(push, 1)
struct PacketHeader {
    uint32_t magic;         // kMagic
    uint8_t type;           // kTypeData / kTypeParity
    uint32_t seq;           // sender-global monotonic counter, all packets
    uint8_t lane;           // 0..lanes-1, which interleave lane this belongs to
    uint16_t lane_group;    // monotonic group-instance counter within the lane
    uint8_t group_seq;      // DATA: index within group (0..group_size-1)
                             // PARITY: parity index (0..parity_count-1)
    uint8_t group_size;     // G, echoed so a decoder can be config-free
    uint8_t parity_count;   // M, echoed likewise
    uint16_t payload_len;   // bytes of payload that follow the header
    uint32_t crc32;         // CRC32 of payload only (corruption, not loss)
};
#pragma pack(pop)

static_assert(sizeof(PacketHeader) == 21, "PacketHeader layout changed - update decoders too");

}  // namespace gcs
