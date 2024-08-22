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
     * Retrieve sparse point cloud calculation status.
     * @returns true if sparse point cloud calculation is enabled, false otherwise
     */
    bool getSparse() const;

    /**
     * Retrieve transformation matrix for point cloud calculation.
     * @returns 4x4 transformation matrix
     */
    std::array<std::array<float, 4>, 4> getTransformationMatrix() const;

    /**
     * Enable or disable sparse point cloud calculation.
     * @param enable
     */
    PointCloudConfig& setSparse(bool enable);

    /**
     * Set 4x4 transformation matrix for point cloud calculation. Default is an identity matrix.
     * @param transformationMatrix
     */
    PointCloudConfig& setTransformationMatrix(const std::array<std::array<float, 4>, 4>& transformationMatrix);

    /**
     * Set 3x3 transformation matrix for point cloud calculation. Default is an identity matrix.
     * @param transformationMatrix
     */
    PointCloudConfig& setTransformationMatrix(const std::array<std::array<float, 3>, 3>& transformationMatrix);
};

}  // namespace dai
