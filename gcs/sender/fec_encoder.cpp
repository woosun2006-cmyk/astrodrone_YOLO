#include "fec_encoder.hpp"

#include <algorithm>
#include <cassert>
#include <cstring>

#include "crc32.hpp"

namespace gcs {

FecEncoder::FecEncoder(uint8_t lanes, uint8_t group_size, SendFn send)
    : lanes_(lanes), group_size_(group_size), send_(std::move(send)) {
    assert(lanes >= 1 && group_size >= 1);
}

void FecEncoder::submit(const uint8_t* payload, uint16_t len) {
    assert(len <= kMaxPayload);

    // Flush a parity queued by an earlier group *before* this call's DATA
    // packet, so it's never physically adjacent to the data packet that
    // triggered it (see the class comment for why that matters).
    emit_one_pending_parity();

    const uint8_t lane_idx = next_lane_;
    next_lane_ = static_cast<uint8_t>((next_lane_ + 1) % lanes_.size());
    Lane& lane = lanes_[lane_idx];

    const uint8_t group_seq = static_cast<uint8_t>(lane.pending.size());
    emit(kTypeData, lane_idx, lane.group_index, group_seq, payload, len);

    lane.pending.emplace_back(payload, payload + len);
    lane.max_len = std::max(lane.max_len, len);

    if (lane.pending.size() == group_size_) {
        std::vector<uint8_t> parity(lane.max_len, 0);
        for (const auto& p : lane.pending) {
            for (size_t i = 0; i < p.size(); ++i) {
                parity[i] ^= p[i];
            }
        }
        pending_parity_.push_back({lane_idx, lane.group_index, std::move(parity)});

        lane.pending.clear();
        lane.max_len = 0;
        lane.group_index++;
    }
}

void FecEncoder::flush() { emit_one_pending_parity(); }

void FecEncoder::emit_one_pending_parity() {
    if (pending_parity_.empty()) {
        return;
    }
    PendingParity p = std::move(pending_parity_.front());
    pending_parity_.pop_front();
    emit(kTypeParity, p.lane, p.lane_group, 0, p.payload.data(),
         static_cast<uint16_t>(p.payload.size()));
}

void FecEncoder::emit(uint8_t type, uint8_t lane, uint16_t lane_group, uint8_t group_seq,
                       const uint8_t* payload, uint16_t len) {
    std::vector<uint8_t> buf(sizeof(PacketHeader) + len);
    PacketHeader hdr{};
    hdr.magic = kMagic;
    hdr.type = type;
    hdr.seq = next_seq_++;
    hdr.lane = lane;
    hdr.lane_group = lane_group;
    hdr.group_seq = group_seq;
    hdr.group_size = group_size_;
    hdr.parity_count = 1;
    hdr.payload_len = len;
    hdr.crc32 = crc32(payload, len);

    std::memcpy(buf.data(), &hdr, sizeof(hdr));
    std::memcpy(buf.data() + sizeof(hdr), payload, len);
    send_(buf.data(), buf.size());
}

}  // namespace gcs
