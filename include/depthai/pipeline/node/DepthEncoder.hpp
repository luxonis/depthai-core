#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

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

   private:

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

    /**
     * @brief Set the LUT for depth mapping
        * @param minIn Minimum input value
        * @param maxIn Maximum input value
        * @param scaleFactor Scale factor for input values in case of depth encoding. The scale should work as follows: disparity  = scaleFactor / depth
        * @param bufferPercentage Percentage of the colorspace to be left out of the LUT to avoid inversion of the low and high values due to the Hue LUT being cyclic
     */
    void setHueLut(uint16_t minIn, uint16_t maxIn, float scaleFactor = 1, float bufferPercentage = 0.0f);


    void setOutputType(RawImgFrame::Type outputType);

    void setNumFramesPool(int numFramesPool);

   void setNumShaves(int numShaves);

};

}  // namespace node
}  // namespace dai
