#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

#include "depthai/pipeline/Subnode.hpp"
#include "depthai/pipeline/node/Sync.hpp"

// shared
#include <depthai/properties/RectificationProperties.hpp>

namespace cv {
class Mat;
}

namespace dai {
namespace node {

class Rectification : public DeviceNodeCRTP<DeviceNode, Rectification, RectificationProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "Rectification";
    using DeviceNodeCRTP::DeviceNodeCRTP;
    using Backend = RectificationProperties::Backend;

    /**
     * Input images to be rectified
     */
    Input input1{*this, {"input1", DEFAULT_GROUP, true, 4, {{DatatypeEnum::ImgFrame, true}}}};
    Input input2{*this, {"input2", DEFAULT_GROUP, true, 4, {{DatatypeEnum::ImgFrame, true}}}};

    /**
     * Passthrough for input messages (so the node can be placed between other nodes)
     */
    Output passthrough1{*this, {"passthrough1", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};
    Output passthrough2{*this, {"passthrough2", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, true}}}}};

    /**
     * Rectified outputs. On host and RVC4, color inputs retain their color format, including RGB/BGR channel order and planar layout.
     */
    Output output1{*this, {"output1", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};
    Output output2{*this, {"output2", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Specify whether to run on host or device
     * By default, the node will run on device.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * RVC2 supports AUTO and HW using the same on-device warp engine as StereoDepth.
     * RVC4: by default, AUTO prefers GPU on device and
     * falls back to CPU when unavailable. GPU uses CPU-compatible dense OpenCL
     * remapping on RVC4 and throws if unavailable. HW prefers EVA with CPU fallback
     * for unsupported layouts. Host execution supports CPU/AUTO only.
     */
    Rectification& setBackend(Backend backend);

    /**
     * Set output size. On RVC2 this must match the input size; use ImageManip to resize.
     */
    Rectification& setOutputSize(uint32_t width, uint32_t height);

    /**
     * Set output size. On RVC2 this must match the input size; use ImageManip to resize.
     */
    Rectification& setOutputSize(std::pair<uint32_t, uint32_t> size) {
        return setOutputSize(size.first, size.second);
    }

    /**
     * Enable or disable rectification (useful for minimal changes during debugging)
     */
    Rectification& enableRectification(bool enable) {
        properties.enableRectification = enable;
        return *this;
    }
    /**
     * Check if the node is set to run on host
     */
    bool runOnHost() const override;

    void run() override;

   protected:
    struct RemapFrame {
        std::shared_ptr<ImgFrame> input;
        std::shared_ptr<ImgFrame> output;
        const cv::Mat& mapX;
        const cv::Mat& mapY;
    };

    // Both outputs must be complete when this method returns.
    virtual void remapPair(const RemapFrame& left, const RemapFrame& right, bool mapsChanged);
    void remapFrame(const RemapFrame& frame);

   private:
    bool runOnHostVar = false;
};

}  // namespace node
}  // namespace dai
