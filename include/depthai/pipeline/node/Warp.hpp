#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
// #include <depthai/pipeline/datatype/WarpConfig.hpp>

// shared
#include <depthai-shared/common/Point2f.hpp>
#include <depthai-shared/properties/WarpProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief Warp node. Capability to crop, resize, warp, ... incoming image frames
 */
class Warp : public NodeCRTP<DeviceNode, Warp, WarpProperties> {
   public:
    constexpr static const char* NAME = "Warp";
    using NodeCRTP::NodeCRTP;

   private:
    void setWarpMesh(const float* meshData, int numMeshPoints, int width, int height);

   public:
    // /**
    //  * Initial config to use when manipulating frames
    //  */
    // WarpConfig initialConfig;

    // /**
    //  * Input WarpConfig message with ability to modify parameters in runtime
    //  * Default queue is blocking with size 8
    //  */
    // Input inputConfig{true, *this, "inputConfig", Input::Type::SReceiver, true, 8, {{DatatypeEnum::WarpConfig, true}}};

    /**
     * Input image to be modified
     * Default queue is blocking with size 8
     */
    Input inputImage{true, *this, "inputImage", Input::Type::SReceiver, true, 8, true, {{DatatypeEnum::ImgFrame, true}}};

    /**
     * Outputs ImgFrame message that carries warped image.
     */
    Output out{true, *this, "out", Output::Type::MSender, {{DatatypeEnum::ImgFrame, true}}};

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
     * @param ids Maximum frame size in bytes
     */
    void setHwIds(std::vector<int> ids);
    /// Retrieve which hardware warp engines to use
    std::vector<int> getHwIds() const;

    /**
     * Specify which interpolation method to use
     * @param interpolation type of interpolation
     */
    void setInterpolation(Properties::Interpolation interpolation);
    /// Retrieve which interpolation method to use
    Properties::Interpolation getInterpolation() const;
};

}  // namespace node
}  // namespace dai
