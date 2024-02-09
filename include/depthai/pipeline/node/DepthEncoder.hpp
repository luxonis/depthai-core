#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <tuple>

// shared
#include "depthai-shared/properties/DepthEncoderProperties.hpp"

namespace dai {
namespace node {

/**
 * @brief DepthEncoder node. Encodes disparity/depth frames into RGB/NV12 format.
 */
class DepthEncoder : public NodeCRTP<DeviceNode, DepthEncoder, DepthEncoderProperties> {
   public:
    constexpr static const char* NAME = "DepthEncoder";
    using NodeCRTP::NodeCRTP;
    void build();

   protected:
    Properties& getProperties();

   public:
    DepthEncoder();
    DepthEncoder(std::unique_ptr<Properties> props);

    /**
     * Input for encoder
     *
     * Default queue is non-blocking with size 8
     */
    Input input{true, *this, "left", Input::Type::SReceiver, false, 8, true, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs decoded image.
     */
    Output output{true, *this, "depth", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Passthrough input message.
     */
    Output passthroughInput{true, *this, "passthroughInput", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    void setLut(std::vector<uint8_t> lutR, std::vector<uint8_t> lutG, std::vector<uint8_t> lutB);

    void setOutputType(RawImgFrame::Type outputType);

    void setNumFramesPool(int numFramesPool);
    /**
     * @brief Set the LUT for depth to hue mapping, allowing for external customization of disparity and hue calculations.
     * @param minDepthIn Minimum input value for depth.
     * @param maxDepthIn Maximum input value for depth.
     * @param bufferAmount Amount (as a fraction of 1) of the colorspace to buffer at both ends of the LUT to avoid
     *                     inversion of low and high values due to the Hue LUT being cyclic. Must be between 0 and 0.5.
     * @param getMinDisparity Function to calculate minimum disparity from minIn and maxIn depths.
     * @param getMaxDisparity Function to calculate maximum disparity from minIn, maxIn depths, and a value for scaling.
     * @param getHueValueFromDisparity Function to calculate hue value given a disparity, minIn, maxIn depths and maximum hue value.
     */
    void setHueLutGeneric(uint16_t minDepthIn,
                          uint16_t maxDepthIn,
                          float bufferAmount,
                          const std::function<uint16_t(uint16_t, uint16_t)>& getMinDisparity,
                          const std::function<uint16_t(uint16_t, uint16_t, uint16_t)>& getMaxDisparity,
                          const std::function<uint16_t(uint16_t, uint16_t, uint16_t, uint16_t)>& getHueValueFromDisparity);
    /**
     * @brief Set the LUT for disparity to hue mapping, where HUE values mapped are linearly proportional to disparity.
     * @param minInDepth Minimum input value for depth
     * @param maxInDepth Maximum input value for depth
     * @param scale Scale factor to transform depth to disparity. Applied as disparity = scale / depth
     * @param bufferAmount Amount (as a fraction of 1) of the colorspace to buffer at both ends of the LUT to avoid
     *                     inversion of low and high values due to the Hue LUT being cyclic. Must be between 0 and 0.5.
     */
    void setHueLutDisparity(uint16_t minInDepth, uint16_t maxInDepth, float scale, float bufferAmount);

    /**
     * @brief Set the LUT for depth to hue mapping, where HUE values mapped are linearly proportional to depth.
     * @param minInDepth Minimum input value for depth
     * @param maxInDepth Maximum input value for depth
     * @param scale Scale factor to transform depth to disparity. Applied as disparity = scale / depth
     * @param bufferAmount Amount (as a fraction of 1) of the colorspace to buffer at both ends of the LUT to avoid
     *                     inversion of low and high values due to the Hue LUT being cyclic. Must be between 0 and 0.5.
     */
    void setHueLutDepth(uint16_t minInDepth, uint16_t maxInDepth, float scale, float bufferAmount);

    /**
     * @brief Set the LUT for depth to hue mapping, where HUE values are mapped proportionally to the normalized depth.
     * @param minInDepth Minimum input value for depth
     * @param maxInDepth Maximum input value for depth
     * @param scale Scale factor to transform depth to disparity. Applied as disparity = scale / depth
     * @param bufferAmount Amount (as a fraction of 1) of the colorspace to buffer at both ends of the LUT to avoid
     *                     inversion of low and high values due to the Hue LUT being cyclic. Must be between 0 and 0.5.
     * Internally the function calculates the HUE values based on a logarithmic scale.
     * This is useful if we want the decoded depth to have a constant relative
     *  error.
     * The function calculates the hue values based on this formula:
     * hue = a * log(depth) + b
     * where a and b are the coefficients returned by this function.

      * @return Tuple containing the <a, d> coefficients for the logarithmic mapping of depth to hue.
     */
    std::tuple<double, double> setHueLutDepthNormalized(uint16_t minInDepth, uint16_t maxInDepth, float scale, float bufferAmount);

    void setNumShaves(int numShaves);
};

}  // namespace node
}  // namespace dai
