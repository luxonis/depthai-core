#pragma once

#include <unordered_map>
#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * EdgeDetectorConfig message. Carries sobel edge filter config.
 */
class EdgeDetectorConfig : public Buffer {
   public:
    EdgeDetectorConfig() = default;
    virtual ~EdgeDetectorConfig() = default;

    struct EdgeDetectorConfigData {
        /**
         * Used for horizontal gradient computation in 3x3 Sobel filter
         * Format - 3x3 matrix, 2nd column must be 0
         * Default - +1 0 -1; +2 0 -2; +1 0 -1
         */
        std::vector<std::vector<int>> sobelFilterHorizontalKernel;
        /**
         * Used for vertical gradient computation in 3x3 Sobel filter
         * Format - 3x3 matrix, 2nd row must be 0
         * Default - +1 +2 +1; 0 0 0; -1 -2 -1
         */
        std::vector<std::vector<int>> sobelFilterVerticalKernel;

        DEPTHAI_SERIALIZE(EdgeDetectorConfigData, sobelFilterHorizontalKernel, sobelFilterVerticalKernel);
    };

    /**
     * Set sobel filter horizontal and vertical 3x3 kernels
     * @param horizontalKernel Used for horizontal gradient computation in 3x3 Sobel filter
     * @param verticalKernel Used for vertical gradient computation in 3x3 Sobel filter
     */
    void setSobelFilterKernels(const std::vector<std::vector<int>>& horizontalKernel, const std::vector<std::vector<int>>& verticalKernel);

    /**
     * Retrieve configuration data for EdgeDetector
     * @returns EdgeDetectorConfigData: sobel filter horizontal and vertical 3x3 kernels
     */
    EdgeDetectorConfigData getConfigData() const;

    EdgeDetectorConfigData config;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::EdgeDetectorConfig;
    };

    DEPTHAI_SERIALIZE(EdgeDetectorConfig, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, config);
};

}  // namespace dai
