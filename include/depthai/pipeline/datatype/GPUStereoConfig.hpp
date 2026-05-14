#pragma once

#include <cstdint>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * @brief Configuration for the GPUStereo node.
 *
 * All other algorithm parameters (pyramid levels, block size, etc.) are
 * hardcoded on the device side.
 */
class GPUStereoConfig : public Buffer {
   public:
    /**
     * @brief Confidence filter threshold.
     *
     * Pixels whose matching cost exceeds this value are invalidated (disparity set to 0).
     * Range [0, 255]. A value of 0 disables the confidence filter entirely.
     * Lower values are more aggressive (reject more pixels).
     */
    std::uint8_t confidenceThreshold = 10;

    GPUStereoConfig() = default;
    virtual ~GPUStereoConfig();

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DEPTHAI_SERIALIZE(GPUStereoConfig, confidenceThreshold);
};

}  // namespace dai
