#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawStereoDepthConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * StereoDepthConfig message.
 */
class StereoDepthConfig : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawStereoDepthConfig& cfg;

   public:
    /**
     * Construct StereoDepthConfig message.
     */
    StereoDepthConfig();
    explicit StereoDepthConfig(std::shared_ptr<RawStereoDepthConfig> ptr);
    virtual ~StereoDepthConfig() = default;

    void setConfidenceThreshold(int confThr);
    void setMedianFilter(StereoDepthConfigData::MedianFilter median);
    void setBilateralFilterSigma(uint16_t sigma);
};

}  // namespace dai
