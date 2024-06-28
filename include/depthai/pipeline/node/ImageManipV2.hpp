#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/datatype/ImageManipConfigV2.hpp>

// shared
#include <depthai/properties/ImageManipPropertiesV2.hpp>

namespace dai {
namespace node {

/**
 * @brief ImageManip node. Capability to crop, resize, warp, ... incoming image frames
 */
class ImageManipV2 : public DeviceNodeCRTP<DeviceNode, ImageManipV2, ImageManipPropertiesV2> {
   public:
    constexpr static const char* NAME = "ImageManipV2";
    using DeviceNodeCRTP::DeviceNodeCRTP;

   protected:
    Properties& getProperties();

    void setWarpMesh(const float* meshData, int numMeshPoints, int width, int height);

   public:
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
     * Input ImageManipConfig message with ability to modify parameters in runtime
     * Default queue is blocking with size 8
     */
    // Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, true, 8, {{DatatypeEnum::ImageManipConfig, true}}};
    Input inputConfig{*this, {.name = "inputConfig", .types = {{DatatypeEnum::ImageManipConfigV2, true}}, .waitForMessage = false}};

    /**
     * Input image to be modified
     * Default queue is blocking with size 8
     */
    // Input inputImage{*this, "inputImage", Input::Type::SReceiver, true, 8, {{DatatypeEnum::ImgFrame, true}}, true};
    Input inputImage{*this, {.name = "inputImage", .types = {{DatatypeEnum::ImgFrame, true}}}};

    /**
     * Outputs ImgFrame message that carries modified image.
     */
    // Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::ImgFrame, true}}};
    Output out{*this, {.name = "out", .types = {{DatatypeEnum::ImgFrame, true}}}};

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
};

}  // namespace node
}  // namespace dai
