#pragma once


#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/datatype/ImageManipConfigV2.hpp>

// shared
#include <depthai/properties/ImageManipPropertiesV2.hpp>
#include <functional>

namespace dai {
namespace node {

/**
 * @brief ImageManip node. Capability to crop, resize, warp, ... incoming image frames
 */
class ImageManipV2 : public DeviceNodeCRTP<DeviceNode, ImageManipV2, ImageManipPropertiesV2>, public HostRunnable {
   private:
    bool runOnHostVar = false;

   protected:
    Properties& getProperties() override;

   public:
    constexpr static const char* NAME = "ImageManipV2";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    using Backend = ImageManipPropertiesV2::Backend;
    using PerformanceMode = ImageManipPropertiesV2::PerformanceMode;

    ImageManipV2() = default;
    ImageManipV2(std::unique_ptr<Properties> props);

    std::shared_ptr<ImageManipV2> build() {
        return std::static_pointer_cast<ImageManipV2>(shared_from_this());
    }
    /**
     * Initial config to use when manipulating frames
     */
    ImageManipConfigV2 initialConfig;

    /**
     * Input ImageManipConfigV2 message with ability to modify parameters in runtime
     */
    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImageManipConfigV2, true}}}, false}};

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
    ImageManipV2& setRunOnHost(bool runOnHost = true);

    /**
     * Set CPU as backend preference
     * @param backend Backend preference
     */
    ImageManipV2& setBackend(Backend backend);

    /**
     * Set performance mode
     * @param performanceMode Performance mode
     */
    ImageManipV2& setPerformanceMode(PerformanceMode performanceMode);

    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    void run() override;

};

}  // namespace node
}  // namespace dai
