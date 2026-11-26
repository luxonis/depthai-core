#pragma once

#include <array>
#include <cstdint>
#include <depthai/beta/node/ToFStereoFusion.hpp>
#include <depthai/pipeline/datatype/MessageGroup.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <vector>

#include "pipeline/Pool.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai::gate {

class ToFStereoFusion : public dai::beta::node::ToFStereoFusion {
   public:
    constexpr static const char* NAME = beta::node::ToFStereoFusion::NAME;

    struct FusionOutputs {
        std::shared_ptr<ImgFrame> depth;
        std::shared_ptr<ImgFrame> tofNeuralConfidence;
    };

    ToFStereoFusion() : ToFStereoFusion(std::make_unique<Properties>()) {}
    explicit ToFStereoFusion(std::unique_ptr<Properties> props) : dai::beta::node::ToFStereoFusion(std::move(props)) {}

    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;

    bool runOnHost() const override {
        return true;
    }
    void run() override;
    FusionOutputs alphaNetwork(MessageGroup& group, bool confidenceConnected);

   private:
    void ensureBuffers(uint32_t width, uint32_t height);
    void alignNeural(const ImgFrame& tofImage, const ImgFrame& depthNeural, const ImgFrame& confidenceNeural);

    uint32_t bufferWidth = 0;
    uint32_t bufferHeight = 0;
    std::shared_ptr<NNData> nnData;
    std::shared_ptr<Pool<ImgFrame>> depthFrames;
    std::shared_ptr<Pool<ImgFrame>> confidenceFrames;
    cv::Mat neuralRectificationMapX, neuralRectificationMapY, tofRectificationMapX, tofRectificationMapY;
    std::array<std::array<float, 3>, 3> neuralToTofRotation{};
    std::array<float, 3> neuralToTofTranslation{};
    cv::Size previousTofSize, previousNeuralSize;
    cv::Rect overlapCrop;
    ImgTransformation croppedTransformation;
    bool alignmentInitialized = false;
    ImgTransformation previousTofTransformation;
    ImgTransformation previousNeuralTransformation;
    std::vector<uint16_t> alignedNeuralDepth;
    std::vector<uint8_t> alignedNeuralConfidence;
};

}  // namespace dai::gate
