#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/ImageAlignProperties.hpp>

#if defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
    #include <opencv2/core/types.hpp>
#endif

#include "depthai/pipeline/datatype/ImageAlignConfig.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"

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

#if defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
    struct ImgFrameRunState;

    ImgFrameRunState prepareRectificationMatrices(const ImgTransformation& inputTransform, const ImgTransformation& alignToTransform);
    static void updateShiftFactor(ImgFrameRunState& state, uint16_t staticDepthPlane);
    ImgTransformation extractTransformationFromBuffer(const std::shared_ptr<Buffer>& buffer, DatatypeEnum datatype);

    void legacyRun(std::shared_ptr<ImgFrame> firstInputImg,
                   std::shared_ptr<ImgFrame> inputAlignToMsg);  // lagacy ImgFrame to ImgFrame alignment
    void genericAlignRun(std::shared_ptr<Buffer> firstInput,
                         std::shared_ptr<Buffer> inputAlignToMsg);  // if one of the inputs is transformable buffer

    std::shared_ptr<ImgFrame> alignImgFrame(ImgFrame inputImg, const ImgFrameRunState& state, cv::Scalar bgColor = cv::Scalar(0, 0, 0));

    std::shared_ptr<Buffer> buildAlignedOutputMessage(const std::shared_ptr<Buffer>& inputMsg,
                                                      DatatypeEnum inputType,
                                                      const ImgTransformation& targetTransform,
                                                      const ImgFrameRunState& runState,
                                                      bool& warnedAboutDistortion);
#endif
};

}  // namespace node
}  // namespace dai
