#pragma once
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

namespace dai {

/**
 * Represents a frame split into smaller packets, sent incrementally.
 */
struct PacketizedData : public ADatatype {
    PacketizedData(std::uint32_t numPackets) : numPackets(numPackets) {}
    const std::uint32_t numPackets;
};

}  // namespace dai
