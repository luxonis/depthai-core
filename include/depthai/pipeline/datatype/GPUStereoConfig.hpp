#pragma once

#include <cstdint>

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
    std::uint8_t confidenceThreshold = 10;

    GPUStereoConfig() = default;
    virtual ~GPUStereoConfig();

    /**
     * @brief Set the confidence threshold for disparity filtering.
     *
     * Pixels with a matching cost above this threshold are invalidated.
     * @param threshold Value in range [0, 255]. 0 disables the filter. Values outside the range are clamped.
     */
    GPUStereoConfig& setConfidenceThreshold(int threshold);

    /**
     * @brief Get the confidence threshold for disparity filtering.
     */
    int getConfidenceThreshold() const;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DEPTHAI_SERIALIZE(GPUStereoConfig, confidenceThreshold);
};

}  // namespace dai
