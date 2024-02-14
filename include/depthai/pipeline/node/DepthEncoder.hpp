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
        * @brief Set the hue encoding type and the range of disparity/depth values to encode.
        * @param hueEncodingType Type of encoding to use for hue values.
        * @param minDepth Minimum input value for depth.
        * @param maxDepth Maximum input value for depth.
        The function leaves 5% of the colorspace at both ends of the LUT to avoid
        inversion of low and high values due to the Hue LUT being cyclic.

        In case this function is called, the `setLut` function will be ignored.
    */
    void setHueEncoding(HueEncodingType hueEncodingType, int32_t minDepth, int32_t maxDepth);

    void setNumShaves(int numShaves);
};

}  // namespace node
}  // namespace dai
