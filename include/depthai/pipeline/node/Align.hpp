#pragma once

#include <array>
#include <cstdint>
#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/ImageAlignProperties.hpp>

#include "depthai/pipeline/datatype/ImageAlignConfig.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"

namespace dai {
namespace node {

/**
 * @brief Align node. Aligns ImgFrame and Transformable messages using ImgTransformation metadata.
 */
class Align : public DeviceNodeCRTP<DeviceNode, Align, ImageAlignProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "Align";
    using DeviceNodeCRTP::DeviceNodeCRTP;

   protected:
    Properties& getProperties() override;

   public:
    /**
     * Initial config to use when aligning messages.
     */
    std::shared_ptr<ImageAlignConfig> initialConfig = std::make_shared<ImageAlignConfig>();

    /**
     * Input message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, false, 4, {{DatatypeEnum::ImageAlignConfig, false}}}};

    /**
     * Input message to be aligned. Can be either ImgFrame or any message that implements Transformable interface.
     * Default queue is non-blocking with size 4.
     */
    Input input{*this, {"input", DEFAULT_GROUP, false, 4, {{DatatypeEnum::ImgFrame, false}, {DatatypeEnum::Transformable, true}}}};

    /**
     * Input align to message.
     * Default queue is non-blocking with size 1.
     */
    Input inputAlignTo{*this, {"inputAlignTo", DEFAULT_GROUP, false, 1, {{DatatypeEnum::ImgFrame, false}, {DatatypeEnum::Transformable, true}}, true}};

    /**
     * Outputs the input message aligned to the inputAlignTo message. Output message will be of the same type as input message.
     */
    Output outputAligned{*this, {"outputAligned", DEFAULT_GROUP, {{DatatypeEnum::ImgFrame, false}, {DatatypeEnum::Transformable, true}}}};

    /**
     * Passthrough message on which the calculation was performed.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughInput{*this, {"passthroughInput", DEFAULT_GROUP, {{DatatypeEnum::ImgFrame, false}, {DatatypeEnum::Transformable, true}}}};

    /**
     * Specify the output size of the aligned image
     */
    Align& setOutputSize(int alignWidth, int alignHeight);

    /**
     * Specify interpolation method to use when resizing
     */
    Align& setInterpolation(Interpolation interp);

    /**
     * Specify number of shaves to use for this node
     */
    Align& setNumShaves(int numShaves);

    /**
     * Specify number of frames in the pool
     */
    Align& setNumFramesPool(int numFramesPool);

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

#if defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
    struct ImgFrameRunState;

    ImgFrameRunState prepareRectificationMatrices(const ImgTransformation& inputTransform, const ImgTransformation& alignToTransform);
    static void updateShiftFactor(ImgFrameRunState& state, uint16_t staticDepthPlane);
    ImgTransformation extractTransformationFromBuffer(const std::shared_ptr<Buffer>& buffer, DatatypeEnum datatype);

    std::shared_ptr<ImgFrame> alignImgFrame(ImgFrame inputImg, const ImgFrameRunState& state, std::array<uint8_t, 3> bgColor = {0, 0, 0});

    std::shared_ptr<Buffer> buildAlignedOutputMessage(const std::shared_ptr<Buffer>& inputMsg,
                                                      DatatypeEnum inputType,
                                                      const ImgTransformation& targetTransform,
                                                      const ImgFrameRunState& runState,
                                                      bool& warnedAboutDistortion);
#endif
};

}  // namespace node
}  // namespace dai
