#pragma once

#include <depthai/pipeline/DeviceNode.hpp>
#include <utility>

// shared
#include <depthai/properties/ImageAlignProperties.hpp>

#include "depthai/pipeline/datatype/ImageAlignConfig.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai {
namespace node {

/**
 * @brief Align an image or depth map to another camera's image geometry.
 */
class ImageAlign : public DeviceNodeCRTP<DeviceNode, ImageAlign, ImageAlignProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "ImageAlign";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    using Backend = dai::Backend;

   protected:
    struct ImageAlignInputState {
        ImgTransformation depthTransformation;
        ImgTransformation ontoTransformation;
        std::pair<unsigned int, unsigned int> depthResolution{};
        std::pair<unsigned int, unsigned int> ontoResolution{};
        ImgFrame::Type depthType = ImgFrame::Type::RAW16;

        bool differsFrom(const ImageAlignInputState& previous) const {
            return depthResolution != previous.depthResolution || ontoResolution != previous.ontoResolution || depthType != previous.depthType
                   || !depthTransformation.isEqualTransformation(previous.depthTransformation)
                   || !ontoTransformation.isEqualTransformation(previous.ontoTransformation);
        }
    };

    Properties& getProperties() override;

   public:
    /**
     * Initial alignment configuration.
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
     * For RAW16 depth, values represent axial Z in the target camera coordinate system; zero remains invalid.
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
     * Select the execution backend before starting the pipeline.
     * CPU uses the host or RVC4 CPU; GPU prefers the RVC4 GPU for supported RAW16
     * frames and falls back to CPU otherwise. Host execution always uses CPU.
     * AUTO is the default and preserves the platform's automatic selection.
     * RVC2 device execution supports only AUTO (hardware remapping and SHAVEs).
     * This does not change host/device placement; use setRunOnHost() for that.
     * @param backend CPU, GPU or AUTO
     * @return This node for chaining
     * @throws std::invalid_argument for HW or an unknown backend
     */
    ImageAlign& setBackend(Backend backend);

    /** Get the configured backend preference; runtime fallback does not change it. */
    Backend getBackend() const;

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
