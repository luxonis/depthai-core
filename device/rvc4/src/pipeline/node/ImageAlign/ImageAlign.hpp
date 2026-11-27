#pragma once

#include <array>
#include <vector>

// libraries
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/datatype/ImageAlignConfig.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/ImageAlign.hpp"
#include "pipeline/Pool.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/node/Warp/rvc4/WarpDispatcherRvc4.hpp"

namespace dai {
namespace gate {

class ImageAlign : public ::dai::node::ImageAlign {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;

    constexpr static const char* NAME = node::ImageAlign::NAME;
    ImageAlign() : ImageAlign(std::make_unique<dai::node::ImageAlign::Properties>()) {}
    ImageAlign(std::unique_ptr<Properties> props) : dai::node::ImageAlign(std::move(props)) {
        // override that this is a host node
    }
    bool runOnHost() const override {
        return true;
    }
    void run() override;
    void buildStage1() override;

   private:
    static const bool PRINT_DEBUG;

    std::pair<bool, std::string> allocatePools(
        int width, int height, int alignWidth, int alignHeight, ImgFrame::Type inputFrameType, uint32_t warp1MinSize, uint32_t warp2MinSize);
    void initialize(const std::shared_ptr<ImgFrame>& inputImg,
                    const std::shared_ptr<ImgFrame>& inputAlignToImg,
                    ImgTransformation& inputAlignToTransform,
                    bool& hwSupportedFrameType,
                    bool& warpInited);
    void extractCalibrationData(int depthWidth, int depthHeight);
    void shiftMesh(cv::Mat& meshX, int shiftX);

    CalibrationHandler calibHandler;
    std::array<std::array<float, 3>, 3> depthSourceIntrinsics;
    std::array<std::array<float, 3>, 3> alignSourceIntrinsics;
    std::array<std::array<float, 4>, 4> depthToAlignExtrinsics;
    std::vector<float> depthDistortionCoefficients;
    ImgTransformation depthSourceTransformation;
    ImgTransformation alignToTransformation;

    bool keepAspectRatio = true;

    bool allocated = false;

    dai::CameraBoardSocket alignFrom;
    dai::CameraBoardSocket alignTo;
    ImgFrame::Type inputFrameType;

    int alignWidth;
    int alignHeight;

    std::shared_ptr<Pool<ImgFrame>> up_plgPoolImageAlign, up_plgWarpPool1, up_plgWarpPool2;

    cv::Mat map_x_1, map_y_1;
    cv::Mat map_x_2, map_y_2;

    WarpDispatcher warp1, warp2;
};

}  // namespace gate
}  // namespace dai
