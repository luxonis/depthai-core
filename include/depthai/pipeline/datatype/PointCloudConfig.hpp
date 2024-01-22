#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawPointCloudConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * PointCloudConfig message. Carries ROI (region of interest) and threshold for depth calculation
 */
class PointCloudConfig : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawPointCloudConfig& cfg;

   public:
    /**
     * Construct PointCloudConfig message.
     */
    PointCloudConfig();
    explicit PointCloudConfig(std::shared_ptr<RawPointCloudConfig> ptr);
    virtual ~PointCloudConfig() = default;

    /**
     * Set explicit configuration.
     * @param config Explicit configuration
     */
    PointCloudConfig& set(dai::RawPointCloudConfig config);

    /**
     * Retrieve configuration data for SpatialLocationCalculator.
     * @returns config for SpatialLocationCalculator
     */
    dai::RawPointCloudConfig get() const;

    /**
     * Enable or disable sparse point cloud calculation.
     * @param enable
     */
    PointCloudConfig& setSparse(bool enable);
};

}  // namespace dai
