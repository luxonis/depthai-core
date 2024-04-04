#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <depthai/pipeline/datatype/ImageManipConfig.hpp>

// shared
#include <depthai/properties/ImageManipProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief ImageManip node. Capability to crop, resize, warp, ... incoming image frames
 */
class ImageManip : public DeviceNodeCRTP<DeviceNode, ImageManip, ImageManipProperties> {
   public:
    constexpr static const char* NAME = "ImageManip";
    using DeviceNodeCRTP::DeviceNodeCRTP;

   protected:
    Properties& getProperties();

    void setWarpMesh(const float* meshData, int numMeshPoints, int width, int height);

   public:
    ImageManip() = default;
    ImageManip(std::unique_ptr<Properties> props);

    /**
     * Initial config to use when manipulating frames
     */
    ImageManipConfig initialConfig;

    /**
     * Input ImageManipConfig message with ability to modify parameters in runtime
     * Default queue is blocking with size 8
     */
    // Input inputConfig{*this, "inputConfig", Input::Type::SReceiver, true, 8, {{DatatypeEnum::ImageManipConfig, true}}};
    Input inputConfig{*this, {.name = "inputConfig", .types = {{DatatypeEnum::ImageManipConfig, true}} , .waitForMessage = false}};

    /**
     * Input image to be modified
     * Default queue is blocking with size 8
     */
    // Input inputImage{*this, "inputImage", Input::Type::SReceiver, true, 8, {{DatatypeEnum::ImgFrame, true}}, true};
    Input inputImage{*this, {.name = "inputImage", .types = {{DatatypeEnum::ImgFrame, true}}}};

    /**
     * Outputs ImgFrame message that carries modified image.
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::ImgFrame, true}}};

    // Functions to set ImageManipConfig - deprecated
    [[deprecated("Use 'initialConfig.setCropRect()' instead")]] void setCropRect(float xmin, float ymin, float xmax, float ymax);
    [[deprecated("Use 'initialConfig.setCenterCrop()' instead")]] void setCenterCrop(float ratio, float whRatio = 1.0f);
    [[deprecated("Use 'initialConfig.setResize()' instead")]] void setResize(int w, int h);
    [[deprecated("Use 'initialConfig.setResizeThumbnail()' instead")]] void setResizeThumbnail(int w, int h, int bgRed = 0, int bgGreen = 0, int bgBlue = 0);
    [[deprecated("Use 'initialConfig.setFrameType()' instead")]] void setFrameType(ImgFrame::Type name);
    [[deprecated("Use 'initialConfig.setHorizontalFlip()' instead")]] void setHorizontalFlip(bool flip);
    void setKeepAspectRatio(bool keep);

    // Functions to set properties
    /**
     * Specify whether or not wait until configuration message arrives to inputConfig Input.
     * @param wait True to wait for configuration message, false otherwise.
     */
    [[deprecated("Use 'inputConfig.setWaitForMessage()' instead")]] void setWaitForConfigInput(bool wait);

    /**
     * @see setWaitForConfigInput
     * @returns True if wait for inputConfig message, false otherwise
     */
    [[deprecated("Use 'inputConfig.setWaitForMessage()' instead")]] bool getWaitForConfigInput() const;

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
     * Set a custom warp mesh
     * @param meshData 2D plane of mesh points, starting from top left to bottom right
     * @param width Width of mesh
     * @param height Height of mesh
     */
    void setWarpMesh(const std::vector<Point2f>& meshData, int width, int height);
    void setWarpMesh(const std::vector<std::pair<float, float>>& meshData, int width, int height);
};

}  // namespace node
}  // namespace dai
