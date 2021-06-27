#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawAprilTagConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * AprilTagConfig message. Carries ROI (region of interest) and threshold for depth calculation
 */
class AprilTagConfig : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawAprilTagConfig& cfg;

   public:
    /**
     * Construct AprilTagConfig message.
     */
    AprilTagConfig();
    explicit AprilTagConfig(std::shared_ptr<RawAprilTagConfig> ptr);
    virtual ~AprilTagConfig() = default;

    /**
     * Set a vector of ROIs as configuration data.
     * @param ROIs Vector of configuration parameters for ROIs (region of interests)
     */
    void setType(AprilTagConfigData::AprilTagType type);

    /**
     * Retrieve configuration data for AprilTag
     * @returns Vector of configuration parameters for ROIs (region of interests)
     */
    AprilTagConfigData getConfigData() const;
};

}  // namespace dai
