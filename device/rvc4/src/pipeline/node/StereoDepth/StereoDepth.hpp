#pragma once
#include <stdint.h>

// libraries
#include "depthai/pipeline/node/StereoDepth.hpp"

// shared
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/Timestamp.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/properties/Properties.hpp"
#include "depthai/properties/StereoDepthProperties.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/node/StereoDepth/PyrImgDispatcher.hpp"
#include "pipeline/node/StereoDepth/rvc4/StereoDepthDispatcherRvc4.hpp"
#include "pipeline/node/Warp/rvc4/WarpDispatcherRvc4.hpp"
#include "utilities/AcceleratorMemory.hpp"

namespace dai {
namespace gate {

class StereoDepth : public dai::node::StereoDepth {
    // private:
    //     std::shared_ptr<GstStereoDepth> webcamPipeline;
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;

    constexpr static const char* NAME = dai::node::StereoDepth::NAME;

   public:
    StereoDepth() : StereoDepth(std::make_unique<dai::node::StereoDepth::Properties>()) {}
    StereoDepth(std::unique_ptr<Properties> props) : dai::node::StereoDepth(std::move(props)) {
        // override that this is a host node
    }
    bool runOnHost() const override {
        return true;
    }

    void buildStage1() override;
    void run() override;
    void stop() override;

   private:
    AtomicBool threadsRunning{true};
    StereoDepthDispatcher stereoDepth;
    // We use other StereoDepthDispatcher instance for downscaled image (extended mode)
    StereoDepthDispatcher stereoDepthDs;
    PyrImgDispatcher pyrImg;
    WarpDispatcher warp;
    bool warpInitialized = false;

    std::vector<std::shared_ptr<dai::ImgFrame> > outLeftPyrFrames;
    std::vector<std::shared_ptr<dai::ImgFrame> > outRightPyrFrames;
    //
    std::shared_ptr<StereoDepthConfig> latestConfig;

    bool customInputPixelDescriptors;

    bool disparityConnected;
    bool depthConnected;
    bool syncedLeftConnected;
    bool syncedRightConnected;
    bool rectifiedLeftConnected;
    bool rectifiedRightConnected;

    // unsupported
    bool outConfigConnected;
    bool debugDispLrCheckIt1Connected;
    bool debugDispLrCheckIt2Connected;
    bool debugExtDispLrCheckIt1Connected;
    bool debugExtDispLrCheckIt2Connected;
    bool debugDispCostDumpConnected;
    bool confidenceMapConnected;

    //
    bool enableRectification = false;
    dai::CameraBoardSocket leftSocket, rightSocket, alignedSocket;

    uint32_t outDisparityFrameSize;
    uint32_t outConfidenceFrameSize;

    uint32_t outRectifiedFrameSize;
    bool lazyLoad = true;
    std::shared_ptr<dai::ImgFrame> _leftIn, _rightIn;

    uint32_t inWarpWidth;
    uint32_t inWarpHeight;
    uint32_t outFrameWidth;
    uint32_t outFrameHeight;

    std::thread threadWarp;
    void _ThreadWarp();
    std::thread threadDescriptor;
    void _ThreadDescriptor();
    std::thread threadStereo;
    void _ThreadStereo();
    std::thread threadPostProcessing;
    void _ThreadPostProcessing();

    void initCalib(const std::shared_ptr<const dai::ImgFrame>& leftIn, const std::shared_ptr<const dai::ImgFrame>& rightIn);
    void setFixedNumerator(dai::ImgTransformation& alignedTransformation, dai::ImgTransformation& nonAlignedTransformation);
    void setRectifiedTransformations(dai::ImgTransformation& transformation, const cv::Mat& cv_R, const cv::Mat& cv_KRect, dai::CameraBoardSocket socket) const;

    std::vector<double> leftRawGridX, leftRawGridY;
    std::vector<double> rightRawGridX, rightRawGridY;
    int stepWidth, stepHeight, meshWidth, meshHeight, meshElemSize, meshSize, meshStride, meshStrideElems;

    struct PerformanceMetrics {
        std::chrono::time_point<std::chrono::steady_clock> tStartSync, tEndSync;
        std::chrono::time_point<std::chrono::steady_clock> tWarpAllocationStart, tWarpAllocationEnd;
        std::chrono::time_point<std::chrono::steady_clock> tStartWarp, tEndWarp;
        std::chrono::time_point<std::chrono::steady_clock> tWarpPushStart;
        std::chrono::time_point<std::chrono::steady_clock> tStereoPop;
        std::chrono::time_point<std::chrono::steady_clock> tStereoAllocationStart, tStereoAllocationEnd;
        std::chrono::time_point<std::chrono::steady_clock> tStartStereo, tEndStereo;
        std::chrono::time_point<std::chrono::steady_clock> tStereoPushStart;

        std::chrono::time_point<std::chrono::steady_clock> tPostProcessingPop;
        std::chrono::time_point<std::chrono::steady_clock> tPostProcessingAllocationStart, tPostProcessingAllocationEnd;
        std::chrono::time_point<std::chrono::steady_clock> tStartPostProcessing, tEndPostProcessing;
    };

    struct WarpOut {
        std::shared_ptr<const dai::ImgFrame> leftIn;
        std::shared_ptr<const dai::ImgFrame> rightIn;
        std::shared_ptr<const dai::ImgFrame> rectifiedLeftFrame;
        std::shared_ptr<const dai::ImgFrame> rectifiedRightFrame;

        PerformanceMetrics performanceMetrics;
    };
    struct DescriptorOut {
        WarpOut warpPassthrough;
        std::shared_ptr<dai::ImgFrame> stereoDescriptorFrameLeft;
        std::shared_ptr<dai::ImgFrame> stereoDescriptorFrameRight;
    };

    struct StereoOut {
        WarpOut warpPassthrough;
        std::shared_ptr<dai::ImgFrame> disparityFrame;
        std::shared_ptr<dai::ImgFrame> confidenceFrame;
        std::shared_ptr<dai::ImgFrame> depthFrame;

        std::shared_ptr<StereoDepthConfig> stereoConfig;
        bool newConfig = false;
        int maxDisparity = 63;
        bool runPostProcessingOnCPU = true;
    };

    dai::LockingQueue<WarpOut> warpOut;
    dai::LockingQueue<DescriptorOut> descriptorOut;
    dai::LockingQueue<StereoOut> stereoOut;

    std::optional<dai::BsplineStereoCalibration> activeSplineCalibration;
    template <typename T>
    void combineLevels(T* reference, T* downScaled, T* out, uint32_t referenceWidth, uint32_t numDisparities = 64);
    void combineDispMaps(std::shared_ptr<dai::ImgFrame>& disparityFrame,
                         std::shared_ptr<dai::ImgFrame>& disparityFrameDs,
                         std::shared_ptr<dai::ImgFrame>& out,
                         uint16_t maxDisp,
                         int downscaled);

    void disparityToDepth(std::shared_ptr<dai::ImgFrame>& disparityFrame, std::shared_ptr<dai::ImgFrame>& depthFrame, int scaleFactor);

    void leftRightCheck(
        uint16_t* disp_LR,
        uint16_t* disp_RL,
        uint8_t* _conf_LR,
        uint8_t* _conf_RL,
        uint16_t* out,
        uint32_t width,
        uint32_t height,
        uint32_t threshold,
        bool primarySearchR2L);
    void confidenceInvalidation(uint16_t* inDisp, uint8_t* inConf, uint16_t* outDisp, uint32_t width, uint32_t height, uint32_t threshold);

    cv::Mat map_x_l, map_y_l;
    cv::Mat map_x_r, map_y_r;
    cv::Mat map_x_reverse, map_y_reverse;
    bool rectifiedTransformValid = false;
    cv::Mat cv_R_left, cv_R_right;          // rectified extrinsic rotations
    cv::Mat cv_KRect_left, cv_KRect_right;  // rectified intrinsics
    float fixedNumerator = 0;
    bool extendedMode = false;
    dai::StereoDepthConfig::AlgorithmControl::DepthAlign autoAlignment =
        dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_LEFT;
};

}  // namespace gate
}  // namespace dai
