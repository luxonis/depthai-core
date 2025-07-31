#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <memory>

// shared
#include "depthai/pipeline/datatype/VppConfig.hpp"
#include "depthai/properties/VppProperties.hpp"

namespace dai {
namespace node {

/**
 * @brief Vpp node. Apply Virtual Projection Pattern algorithm to stereo images based on disparity.
 */
class Vpp : public DeviceNodeCRTP<DeviceNode, Vpp, VppProperties> {
   public:
    constexpr static const char* NAME = "Vpp";
    using DeviceNodeCRTP::DeviceNodeCRTP;

   protected:
    Properties& getProperties();
    Vpp() = default;
    Vpp(std::unique_ptr<Properties> props);

   public:
    /**
     * Initial config to use for VPP.
     */
    std::shared_ptr<VppConfig> initialConfig = std::make_shared<VppConfig>();

    /**
     * Input VppConfig message with ability to modify parameters in runtime.
     */
    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::VppConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input for left ImgFrame
     */
    Input left{*this, {"left", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input for right ImgFrame
     */
    Input right{*this, {"right", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input for disparity ImgFrame (RAW16 or float32)
     */
    Input disparity{*this, {"disparity", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Output ImgFrame message that carries the processed left image with virtual projection pattern applied.
     */
    Output leftOut{*this, {"leftOut", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Output ImgFrame message that carries the processed right image with virtual projection pattern applied.
     */
    Output rightOut{*this, {"rightOut", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Set the pattern generation method
     * @param method Method to use (RANDOM or MAX_DISTANCE)
     */
    void setMethod(VppConfig::Method method);

    /**
     * Set patch size for projection
     * @param size Patch size (must be odd number, default: 5)
     */
    void setPatchSize(int size);

    /**
     * Set aggregation window size for color computation
     * @param sizeX Window size in X direction (default: 64)
     * @param sizeY Window size in Y direction (default: 3)
     */
    void setAggregationWindowSize(int sizeX, int sizeY);

    /**
     * Set projection direction
     * @param left2right True for left to right projection (default: true)
     */
    void setProjectionDirection(bool left2right);

    /**
     * Set blending factors
     * @param blending Alpha blending factor (default: 0.4)
     * @param blendingOcclusion Alpha blending factor for occluded areas (default: 0.0)
     */
    void setBlending(float blending, float blendingOcclusion = 0.0f);

    /**
     * Enable/disable distance-based patch sizing
     * @param enable Enable distance patch (default: false)
     * @param gamma Distance gamma parameter (default: 0.3)
     */
    void setDistancePatch(bool enable, float gamma = 0.3f);

    /**
     * Enable/disable bilateral patch filtering
     * @param enable Enable bilateral patch (default: false)
     * @param spatialSigma Spatial sigma parameter (default: 2.0)
     * @param intensitySigma Intensity sigma parameter (default: 1.0)
     * @param threshold Bilateral threshold (default: 0.001)
     */
    void setBilateralPatch(bool enable, float spatialSigma = 2.0f, float intensitySigma = 1.0f, float threshold = 0.001f);

    /**
     * Set disparity processing range
     * @param minThreshold Minimum disparity threshold (default: 0.0)
     * @param maxThreshold Maximum disparity threshold (default: 255.0)
     */
    void setDisparityRange(float minThreshold, float maxThreshold);

    /**
     * Set whether to use uniform color for pattern generation
     * @param uniform True for uniform color, false for varying colors (default: false)
     */
    void setUniformColor(bool uniform);

    /**
     * Set whether to discard occluded points
     * @param discard True to discard occluded points (default: false)
     */
    void setDiscardOcclusion(bool discard);

    /**
     * Set whether to use weighted splatting of patterns in target view
     * @param interpolate True to enable interpolation (default: true)
     */
    void setInterpolate(bool interpolate);

    /**
     * Specify number of frames in pool.
     * @param numFramesPool How many frames should the pool have
     */
    void setNumFramesPool(int numFramesPool);
};

}  // namespace node
}  // namespace dai