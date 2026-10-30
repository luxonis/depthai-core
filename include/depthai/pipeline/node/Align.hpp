#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/AlignProperties.hpp>

#include "depthai/pipeline/datatype/AlignConfig.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"

namespace dai {
namespace node {

/**
 * @brief Align node. Aligns ImgFrame, Transformable and depth/color MessageGroup messages using ImgTransformation metadata.
 */
class Align : public DeviceNodeCRTP<DeviceNode, Align, AlignProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "Align";
    using DeviceNodeCRTP::DeviceNodeCRTP;

   protected:
    Properties& getProperties() override;

   public:
    /**
     * Initial config to use when aligning messages.
     */
    std::shared_ptr<AlignConfig> initialConfig = std::make_shared<AlignConfig>();

    /**
     * Input message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, false, 4, {{DatatypeEnum::AlignConfig, false}}}};

    /**
     * Input message to be aligned. Can be either ImgFrame or any message that implements Transformable interface.
     * On host, also accepts a MessageGroup containing a RAW16 "depth" frame in millimeters and any number of
     * coloring ImgFrames registered to the same pixel grid and transformation as depth. All entries are warped
     * together using depth, with nearest-surface occlusion handling. Names and group metadata are preserved.
     * Use setRunOnHost(true) for MessageGroup inputs. Output images are undistorted; uncovered pixels are zero.
     * Group alignment always uses measured depth, independently of staticDepthPlane.
     * Default queue is non-blocking with size 4.
     */
    Input input{
        *this, {"input", DEFAULT_GROUP, false, 4, {{DatatypeEnum::ImgFrame, false}, {DatatypeEnum::Transformable, true}, {DatatypeEnum::MessageGroup, false}}}};

    /**
     * Input align to message.
     * Default queue is non-blocking with size 1.
     */
    Input inputAlignTo{*this, {"inputAlignTo", DEFAULT_GROUP, false, 1, {{DatatypeEnum::ImgFrame, false}, {DatatypeEnum::Transformable, true}}, true}};

    /**
     * Outputs the input message aligned to the inputAlignTo message. Output message will be of the same type as input message.
     */
    Output outputAligned{
        *this, {"outputAligned", DEFAULT_GROUP, {{DatatypeEnum::ImgFrame, false}, {DatatypeEnum::Transformable, true}, {DatatypeEnum::MessageGroup, false}}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughInput{
        *this,
        {"passthroughInput", DEFAULT_GROUP, {{DatatypeEnum::ImgFrame, false}, {DatatypeEnum::Transformable, true}, {DatatypeEnum::MessageGroup, false}}}};

    /**
     * Specify number of frames in the pool
     */
    Align& setNumFramesPool(int numFramesPool);

    /**
     * Set interpolation for both image remap stages before starting the pipeline.
     * AUTO preserves the backend default: nearest-neighbor on device; on host,
     * bilinear for NV12/YUV420p and nearest-neighbor for other image formats.
     * RAW16 depth and typed segmentation masks always use nearest-neighbor.
     * Plain ImgFrame masks must explicitly select NEAREST_NEIGHBOR.
     * @param interpolation AUTO, NEAREST_NEIGHBOR, BILINEAR, or BICUBIC.
     * @return This node for chaining.
     */
    Align& setInterpolation(Interpolation interpolation);

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

    void buildStage1() override;

   private:
    bool runOnHostVar = false;
};

}  // namespace node
}  // namespace dai
