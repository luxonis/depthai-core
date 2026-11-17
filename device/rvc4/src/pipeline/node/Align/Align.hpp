#pragma once

// libraries
#include <array>
#include <cstdint>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <utility>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/datatype/AlignConfig.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Align.hpp"
#include "hexagon/ImageAlignDsp/cpu/ImageAlignDspHandler.hpp"
#include "pipeline/Pool.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utilities/EvaDataMemory.hpp"

namespace dai {
namespace gate {

class Align : public ::dai::node::Align {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;

    constexpr static const char* NAME = node::Align::NAME;
    Align() : Align(std::make_unique<dai::node::Align::Properties>()) {}
    Align(std::unique_ptr<Properties> props) : dai::node::Align(std::move(props)) {
        // Preserve deserialized settings when SDK getProperties() serializes
        // initialConfig again during device-side pipeline construction.
        initialConfig = std::make_shared<AlignConfig>(properties.initialConfig);
    }
    bool runOnHost() const override {
        return true;
    }
    void run() override;
    void buildStage1() override;

   private:
    static const bool PRINT_DEBUG;

    std::shared_ptr<ImgFrame> alignImgFrame(std::shared_ptr<ImgFrame> inputImg,
                                            const cv::Scalar borderColor = cv::Scalar(0, 0, 0),
                                            const cv::InterpolationFlags interpolation = cv::INTER_NEAREST);

    std::pair<bool, std::string> allocatePools(int width, int height, int alignWidth, int alignHeight, ImgFrame::Type inputFrameType);
    void extractCalibrationData(const dai::ImgTransformation& inputTransformation,
                                const dai::ImgTransformation& alignToTransformation,
                                int inputWidth,
                                int inputHeight);
    ImgTransformation extractTransformationFromBuffer(const std::shared_ptr<Buffer>& buffer);

    std::shared_ptr<Buffer> alignInputMsg(const std::shared_ptr<Buffer>& inputMsg, bool& warnedAboutDistortion);

    std::pair<int, int> alignOutputSize() const;
    void updateMaps();
    void setupRemappingParameters(std::shared_ptr<Buffer>& inputMsg);

    void refreshTransformations(std::shared_ptr<Buffer> inputMsg, std::shared_ptr<Buffer> alignToMsg);

    bool calibrationSet = false;

    std::array<std::array<float, 3>, 3> depthSourceIntrinsics;
    std::array<std::array<float, 3>, 3> alignSourceIntrinsics;
    std::array<std::array<float, 4>, 4> depthToAlignExtrinsics;

    dai::ImgTransformation currentInputTransformation;
    dai::ImgTransformation currentAlignToTransformation;

    bool allocated = false;
    uint32_t allocatedIntermediateFrameSize = 0;
    uint32_t allocatedOutputFrameSize = 0;

    ImgFrame::Type inputFrameType = ImgFrame::Type::NONE;

    std::shared_ptr<Pool<ImgFrame>> up_plgPoolAlign, up_plgWarpPool1, up_plgWarpPool2;

    cv::Mat map_x_1, map_y_1;
    cv::Mat map_x_2, map_y_2;

    uint16_t staticDepthPlane = 0;
    ImageAlignDspHandler* dspHandler = nullptr;
    std::shared_ptr<EvaDataMemory> dspConfigData;
};

}  // namespace gate
}  // namespace dai
