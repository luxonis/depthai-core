#pragma once
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * Represents a frame split into smaller packets, sent incrementally.
 */
struct PacketizedData : public Buffer {
   public:
    PacketizedData() = default;
    PacketizedData(std::uint32_t numPackets, std::uint32_t totalSize) : numPackets(numPackets), totalSize(totalSize) {}
    virtual ~PacketizedData();

    std::uint32_t numPackets;
    std::uint32_t totalSize;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const;

    DEPTHAI_SERIALIZE(PacketizedData, numPackets, totalSize);
};

}  // namespace dai
