#pragma once

#include "depthai-shared/datatype/RawImageAlignConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * ImageAlignConfig message
 */
class ImageAlignConfig : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawImageAlignConfig& cfg;

   public:
    ImageAlignConfig();
    explicit ImageAlignConfig(std::shared_ptr<RawImageAlignConfig> ptr);
    virtual ~ImageAlignConfig() = default;

    /**
     * Set explicit configuration.
     * @param config Explicit configuration
     */
    ImageAlignConfig& set(dai::RawImageAlignConfig config);

    /**
     * Retrieve configuration data for SpatialLocationCalculator.
     * @returns config for SpatialLocationCalculator
     */
    dai::RawImageAlignConfig get() const;
};

}  // namespace dai