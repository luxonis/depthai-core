#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawTofCameraConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * TofCameraConfig message. Carries config for feature tracking algorithm
 */
class TofCameraConfig : public Buffer {
    Serialized serialize() const override;
    RawTofCameraConfig& cfg;

   public:

    /**
     * Construct TofCameraConfig message.
     */
    TofCameraConfig();
    explicit TofCameraConfig(std::shared_ptr<RawTofCameraConfig> ptr);
    virtual ~TofCameraConfig() = default;

    /**
     * Set explicit configuration.
     * @param config Explicit configuration
     */
    TofCameraConfig& set(dai::RawTofCameraConfig config);

    /**
     * Retrieve configuration data for TofCamera.
     * @returns config for feature tracking algorithm
     */
    dai::RawTofCameraConfig get() const;
};

}  // namespace dai
