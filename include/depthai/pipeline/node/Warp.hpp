#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
// #include <depthai/pipeline/datatype/WarpConfig.hpp>

// shared
#include <depthai/common/Point2f.hpp>
#include <depthai/properties/WarpProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief Warp node. Capability to crop, resize, warp, ... incoming image frames
 */
class Warp : public DeviceNodeCRTP<DeviceNode, Warp, WarpProperties> {
   public:
    constexpr static const char* NAME = "Warp";
    using DeviceNodeCRTP::DeviceNodeCRTP;

   private:
    void setWarpMesh(const float* meshData, int numMeshPoints, int width, int height);

   public:
    /**
     * Input image to be modified
     * Default queue is blocking with size 8
     */
    Input inputImage{*this, {"inputImage", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{DatatypeEnum::ImgFrame, true}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Outputs ImgFrame message that carries warped image.
     */
    Output out{*this, {"out", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};

    /**
     * Sets output frame size in pixels
     *
     * @param size width and height in pixels
     */
    void setOutputSize(std::tuple<int, int> size);
    void setOutputSize(int width, int height);

    /**
     * Set a custom warp mesh
     * @param meshData 2D plane of mesh points, starting from top left to bottom right
     * @param width Width of mesh
     * @param height Height of mesh
     */
    void setWarpMesh(const std::vector<Point2f>& meshData, int width, int height);
    void setWarpMesh(const std::vector<std::pair<float, float>>& meshData, int width, int height);

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
     * Specify which hardware warp engines to use
     * @param ids Which warp engines to use (0, 1, 2)
     */
    void setHwIds(std::vector<int> ids);
    /// Retrieve which hardware warp engines to use
    std::vector<int> getHwIds() const;

    /**
     * Specify which interpolation method to use
     * @param interpolation type of interpolation
     */
    void setInterpolation(dai::Interpolation interpolation);
    /// Retrieve which interpolation method to use
    dai::Interpolation getInterpolation() const;
};

}  // namespace node
}  // namespace dai
