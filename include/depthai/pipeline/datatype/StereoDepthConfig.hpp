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

    /**
     * Confidence threshold for disparity calculation
     * @param confThr Confidence threshold value 0..255
     */
    void setConfidenceThreshold(int confThr);
    /**
     * Get confidence threshold for disparity calculation
     */
    int getConfidenceThreshold() const;

    /**
     * @param median Set kernel size for disparity/depth median filtering, or disable
     */
    void setMedianFilter(dai::MedianFilter median);
    /**
     * Get median filter setting
     */
    dai::MedianFilter getMedianFilter() const;

    /**
     * A larger value of the parameter means that farther colors within the pixel neighborhood will be mixed together,
     * resulting in larger areas of semi-equal color.
     * @param sigma Set sigma value for 5x5 bilateral filter. 0..65535
     */
    void setBilateralFilterSigma(uint16_t sigma);
    /**
     * Get sigma value for 5x5 bilateral filter
     */
    uint16_t getBilateralFilterSigma() const;

    /**
     * @param threshold Set threshold for left-right, right-left disparity map combine, 0..255
     */
    void setLeftRightCheckThreshold(int threshold);
    /**
     * Get threshold for left-right check combine
     */
    int getLeftRightCheckThreshold() const;

    /**
     * Computes and combines disparities in both L-R and R-L directions, and combine them.
     *
     * For better occlusion handling, discarding invalid disparity values
     */
    void setLeftRightCheck(bool enable);

    /**
     * Disparity range increased from 95 to 190, combined from full resolution and downscaled images.
     * Suitable for short range objects
     */
    void setExtendedDisparity(bool enable);

    /**
     * Computes disparity with sub-pixel interpolation (5 fractional bits).
     *
     * Suitable for long range. Currently incompatible with extended disparity
     */
    void setSubpixel(bool enable);

    /**
     * Useful for normalization of the disparity map.
     * @returns Maximum disparity value that the node can return
     */
    float getMaxDisparity() const;

    /**
     * Set explicit configuration.
     * @param config Explicit configuration
     */
    void set(dai::RawStereoDepthConfig config);

    /**
     * Retrieve configuration data for StereoDepth.
     * @returns config for stereo depth algorithm
     */
    dai::RawStereoDepthConfig get() const;
};

}  // namespace dai
