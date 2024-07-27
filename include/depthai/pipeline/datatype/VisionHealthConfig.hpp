#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawVisionHealthConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * VisionHealthConfig message.
 */
class VisionHealthConfig : public Buffer {
    Serialized serialize() const override;
    RawVisionHealthConfig& cfg;

   public:
    /**
     * Construct VisionHealthConfig message.
     */
    VisionHealthConfig();
    explicit VisionHealthConfig(std::shared_ptr<RawVisionHealthConfig> ptr);
    virtual ~VisionHealthConfig() = default;

    VisionHealthConfig& addRelativeVisionHealthConfig(VisionHealthMetricTypes metric, std::string oper, tl::optional<float> threshold, float sigmas);

    VisionHealthConfig& addAbsoluteVisionHealthConfig(VisionHealthMetricTypes metric, std::string oper, float threshold);

    /**
     * Set explicit configuration.
     * @param config Explicit configuration
     */
    VisionHealthConfig& set(dai::RawVisionHealthConfig config);

    /**
     * Retrieve configuration data for SpatialLocationCalculator.
     * @returns config for SpatialLocationCalculator
     */
    dai::RawVisionHealthConfig get() const;
};

}  // namespace dai
