#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawDepthAlignConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * DepthAlignConfig message
 */
class DepthAlignConfig : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawDepthAlignConfig& cfg;

   public:
    DepthAlignConfig();
    explicit DepthAlignConfig(std::shared_ptr<RawDepthAlignConfig> ptr);
    virtual ~DepthAlignConfig() = default;

    /**
     * Set explicit configuration.
     * @param config Explicit configuration
     */
    DepthAlignConfig& set(dai::RawDepthAlignConfig config);

    /**
     * Retrieve configuration data for SpatialLocationCalculator.
     * @returns config for SpatialLocationCalculator
     */
    dai::RawDepthAlignConfig get() const;
};

}  // namespace dai