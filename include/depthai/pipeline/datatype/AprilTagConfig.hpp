#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawAprilTagConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * AprilTagConfig message.
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
     * @param t AprilTag type
     */
    void setType(AprilTagType::Type t);

    /**
     * Retrieve configuration data for AprilTag
     * @returns Vector of configuration parameters for type (region of interests)
     */
    AprilTagType getConfigData() const;
};

}  // namespace dai
