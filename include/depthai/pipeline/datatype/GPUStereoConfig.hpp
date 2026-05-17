#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * @brief Configuration for the GPUStereo node.
 */
class GPUStereoConfig : public Buffer {
   public:
    /**
     * @brief Confidence threshold for disparity filtering.
     *
     * Value in range [0, 255]. 0 disables the filter.
     */
    int confidenceThreshold = 10;

    GPUStereoConfig() = default;
    virtual ~GPUStereoConfig();

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DEPTHAI_SERIALIZE(GPUStereoConfig, confidenceThreshold);
};

}  // namespace dai
