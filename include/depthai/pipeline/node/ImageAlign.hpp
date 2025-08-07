#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/ImageAlignProperties.hpp>

#include "depthai/pipeline/datatype/ImageAlignConfig.hpp"

namespace dai {
namespace node {

/**
 * @brief ImageAlign node. Calculates spatial location data on a set of ROIs on depth map.
 */
class ImageAlign : public DeviceNodeCRTP<DeviceNode, ImageAlign, ImageAlignProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "ImageAlign";
    using DeviceNodeCRTP::DeviceNodeCRTP;

   protected:
    Properties& getProperties() override;

   public:
    /**
     * Initial config to use when calculating spatial location data.
     */
    std::shared_ptr<ImageAlignConfig> initialConfig = std::make_shared<ImageAlignConfig>();

    /**
     * Input message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, false, 4, {{DatatypeEnum::ImageAlignConfig, false}}}};

    /**
     * Input message.
     * Default queue is non-blocking with size 4.
     */
    Input input{*this, {"input", DEFAULT_GROUP, false, 4, {{DatatypeEnum::ImgFrame, false}}}};

    /**
     * Input align to message.
     * Default queue is non-blocking with size 1.
     */
    Input inputAlignTo{*this, {"inputAlignTo", DEFAULT_GROUP, false, 1, {{DatatypeEnum::ImgFrame, false}}, true}};

    /**
     * Outputs ImgFrame message that is aligned to inputAlignTo.
     */
    Output outputAligned{*this, {"outputAligned", DEFAULT_GROUP, {{DatatypeEnum::ImgFrame, false}}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughInput{*this, {"passthroughInput", DEFAULT_GROUP, {{DatatypeEnum::ImgFrame, false}}}};

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

    /**
     * Specify whether to run on host or device
     * By default, the node will run on device.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    void run() override;

   private:
    bool runOnHostVar = false;
};

}  // namespace node
}  // namespace dai