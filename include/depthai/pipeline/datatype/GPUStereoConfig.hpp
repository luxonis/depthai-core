#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * @brief Internal configuration for the GPUStereo node.
 *
 * Only `confidence_threshold` is user-facing (via GPUStereo::setConfidenceThreshold).
 * All other algorithm parameters are hardcoded on the device side.
 */
class GPUStereoConfig : public Buffer {
   public:
    int confidence_threshold = 10;

    GPUStereoConfig() = default;
    virtual ~GPUStereoConfig();

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DEPTHAI_SERIALIZE(GPUStereoConfig, confidence_threshold);
};

}  // namespace dai
