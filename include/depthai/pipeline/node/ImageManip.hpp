#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/datatype/ImageManipConfig.hpp>

// shared
#include <depthai/properties/ImageManipProperties.hpp>
#include <functional>

namespace dai {
namespace node {

/**
 * @brief ImageManip node. Capability to crop, resize, warp, ... incoming image frames
 */
class ImageManip : public DeviceNodeCRTP<DeviceNode, ImageManip, ImageManipProperties>, public HostRunnable {
   private:
    bool runOnHostVar = false;

   protected:
    Properties& getProperties() override;

   public:
    constexpr static const char* NAME = "ImageManip";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    using Backend = ImageManipProperties::Backend;
    using PerformanceMode = ImageManipProperties::PerformanceMode;

    ImageManip() = default;
    /**
     * Construct an ImageManip node with properties.
     */
    ImageManip(std::unique_ptr<Properties> props);

    /**
     * Build the node and return a shared pointer to it.
     */
    std::shared_ptr<ImageManip> build() {
        return std::static_pointer_cast<ImageManip>(shared_from_this());
    }
    /**
     * Initial config to use when manipulating frames
     */
    std::shared_ptr<ImageManipConfig> initialConfig = std::make_shared<ImageManipConfig>();

    /**
     * Input ImageManipConfig message with ability to modify parameters in runtime
     */
    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImageManipConfig, true}}}, false}};

    /**
     * Input image to be modified
     */
    Input inputImage{*this, {"inputImage", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs ImgFrame message that carries modified image.
     */
    // Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::ImgFrame, true}}};
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};

    /**
     * Specify number of frames in pool.
     * @param numFramesPool How many frames should the pool have
     */
    void setNumFramesPool(int numFramesPool);

    /**
     * Specify maximum size of output image.
     * @param maxFrameSize Maximum frame size in bytes
     */
    void setMaxOutputFrameSize(int maxFrameSize);

    /**
     * Specify whether to run on host or device
     * @param runOnHost Run node on host
     */
    ImageManip& setRunOnHost(bool runOnHost = true);

    /**
     * Set CPU as backend preference
     * @param backend Backend preference
     */
    ImageManip& setBackend(Backend backend);

    /**
     * Set performance mode
     * @param performanceMode Performance mode
     */
    ImageManip& setPerformanceMode(PerformanceMode performanceMode);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    void run() override;
};

}  // namespace node
}  // namespace dai
