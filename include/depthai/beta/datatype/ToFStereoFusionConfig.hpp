#pragma once

#include <stdexcept>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai::beta {

/** Runtime confidence threshold for ToFStereoFusion. */
struct ToFStereoFusionConfig : Buffer {
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::ToFStereoFusionConfig;
    }

    /** Fused depth pixels with confidence below this value are invalidated. Range: [0, 1]. */
    float confidenceThreshold = 0.5f;

    ToFStereoFusionConfig& setConfidenceThreshold(float threshold) {
        if(!(threshold >= 0.0f && threshold <= 1.0f)) throw std::invalid_argument("confidence threshold must be between 0 and 1");
        confidenceThreshold = threshold;
        return *this;
    }

    DEPTHAI_SERIALIZE(ToFStereoFusionConfig, confidenceThreshold);
};

}  // namespace dai::beta
