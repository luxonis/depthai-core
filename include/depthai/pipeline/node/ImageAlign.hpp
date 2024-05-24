#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/properties/ImageAlignProperties.hpp>

#include "depthai/pipeline/datatype/ImageAlignConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief ImageAlign node. Calculates spatial location data on a set of ROIs on depth map.
 */
class ImageAlign : public NodeCRTP<Node, ImageAlign, ImageAlignProperties> {
   public:
    constexpr static const char* NAME = "ImageAlign";

   protected:
    Properties& getProperties();

   private:
    std::shared_ptr<RawImageAlignConfig> rawConfig;

   public:
    ImageAlign(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    ImageAlign(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

    /**
     * Initial config to use when calculating spatial location data.
     */
    ImageAlignConfig initialConfig;

    /**
     * Input message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, false, 4, {{DatatypeEnum::ImageAlignConfig, false}}};

    /**
     * Input message.
     * Default queue is non-blocking with size 4.
     */
    Input input{*this, "input", Input::Type::SReceiver, false, 4, true, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Input align to message.
     * Default queue is non-blocking with size 1.
     */
    Input inputAlignTo{*this, "inputAlignTo", Input::Type::SReceiver, false, 1, true, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs ImgFrame message that is aligned to inputAlignTo.
     */
    Output outputAligned{*this, "outputAligned", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughInput{*this, "passthroughInput", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Specify the output size of the aligned image
     */
    ImageAlign& setOutputSize(int alignWidth, int alignHeight);

    /**
     * Specify whether to keep aspect ratio when resizing
     */
    ImageAlign& setOutKeepAspectRatio(bool keep);

    /**
     * Specify interpolation method to use when resizing
     */
    ImageAlign& setInterpolation(Interpolation interp);

    /**
     * Specify number of shaves to use for this node
     */
    ImageAlign& setNumShaves(int numShaves);

    /**
     * Specify number of frames in the pool
     */
    ImageAlign& setNumFramesPool(int numFramesPool);
};

}  // namespace node
}  // namespace dai