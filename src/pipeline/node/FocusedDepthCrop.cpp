#include "depthai/pipeline/node/FocusedDepthCrop.hpp"

#include <algorithm>
#include <cmath>

#include "depthai/pipeline/datatype/FocusedDepthRoi.hpp"
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/utility/ImageManipImpl.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
namespace dai {
namespace node {

FocusedDepthCrop::FocusedDepthCrop()
    : DeviceNodeCRTP<DeviceNode, FocusedDepthCrop, FocusedDepthCropProperties>() {
    properties.imageManip.outputFrameSize = 16 * 1024 * 1024;
}

namespace {

inline std::array<float, 9> flatten(std::array<std::array<float, 3>, 3> mat) {
    return {mat[0][0], mat[0][1], mat[0][2], mat[1][0], mat[1][1], mat[1][2], mat[2][0], mat[2][1], mat[2][2]};
}

bool runImageManip(const ImgFrame& srcFrame,
                   const ImageManipConfig& config,
                   ImageManipProperties& manipProps,
                   std::shared_ptr<ImgFrame>& outImage,
                   std::shared_ptr<spdlog::async_logger> logger) {
    if(!srcFrame.data) {
        return false;
    }
    impl::ImageManipOperations<impl::_ImageManipBuffer, impl::_ImageManipMemory, impl::WarpH> manip(manipProps, std::move(logger));
    auto srcFrameSpecs = impl::getSrcFrameSpecs(srcFrame.fb);
    manip.build(config.base, config.outputFrameType, srcFrameSpecs, srcFrame.getType());
    auto newCameraMatrix = impl::matmul(manip.getMatrix(), srcFrame.transformation.getIntrinsicMatrix());
    manip.buildUndistort(config.base.undistort,
                         flatten(srcFrame.transformation.getIntrinsicMatrix()),
                         flatten(newCameraMatrix),
                         srcFrame.transformation.getDistortionCoefficients(),
                         srcFrame.getType(),
                         srcFrame.getWidth(),
                         srcFrame.getHeight(),
                         manip.getOutputWidth(),
                         manip.getOutputHeight());
    const auto outputSize = manip.getOutputSize();
    if(outputSize == 0) {
        return false;
    }
    if(static_cast<long>(outputSize) > static_cast<long>(manipProps.outputFrameSize)) {
        return false;
    }
    outImage = std::make_shared<ImgFrame>();
    auto outImageData = std::make_shared<impl::_ImageManipMemory>(manipProps.outputFrameSize);
    outImage->data = outImageData;
    auto srcMem = std::make_shared<impl::_ImageManipMemory>(srcFrame.data->getData());
    if(!manip.apply(srcMem, outImageData)) {
        return false;
    }
    const auto outType = manip.getOutputFrameType();
    const auto dstSpecs = manip.getOutputFrameSpecs(outType);
    outImage->sourceFb = srcFrame.sourceFb;
    outImage->cam = srcFrame.cam;
    outImage->instanceNum = srcFrame.instanceNum;
    outImage->sequenceNum = srcFrame.sequenceNum;
    outImage->ts = srcFrame.ts;
    outImage->tsDevice = srcFrame.tsDevice;
    outImage->category = srcFrame.category;
    outImage->event = srcFrame.event;
    outImage->fb.height = dstSpecs.height;
    outImage->fb.width = dstSpecs.width;
    outImage->fb.stride = dstSpecs.p1Stride;
    outImage->fb.p1Offset = dstSpecs.p1Offset;
    outImage->fb.p2Offset = dstSpecs.p2Offset;
    outImage->fb.p3Offset = dstSpecs.p3Offset;
    outImage->setType(outType);

    outImage->transformation = srcFrame.transformation;
    if(manip.undistortEnabled()) {
        outImage->transformation.setDistortionCoefficients({});
    }
    auto srcCrops = manip.getSrcCrops();
    outImage->transformation.addSrcCrops(srcCrops);
    outImage->transformation.addTransformation(manip.getMatrix());
    outImage->transformation.setSize(dstSpecs.width, dstSpecs.height);
    return true;
}

}  // namespace

FocusedDepthCrop::FocusedDepthCrop(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, FocusedDepthCrop, FocusedDepthCropProperties>(std::move(props)) {}

FocusedDepthCrop::Properties& FocusedDepthCrop::getProperties() {
    return properties;
}

FocusedDepthCrop& FocusedDepthCrop::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
    return *this;
}

bool FocusedDepthCrop::runOnHost() const {
    return runOnHostVar;
}

void FocusedDepthCrop::run() {
#ifndef DEPTHAI_HAVE_OPENCV_SUPPORT
    throw std::runtime_error("FocusedDepthCrop requires OpenCV (DEPTHAI_HAVE_OPENCV_SUPPORT).");
#else
    auto& logger = pimpl->logger;
    while(mainLoop()) {
        std::shared_ptr<MessageGroup> group;
        {
            auto blockEvent = inputBlockEvent();
            group = input.get<MessageGroup>();
        }
        if(!group) {
            logger->warn("FocusedDepthCrop: null MessageGroup");
            continue;
        }
        auto left = group->get<ImgFrame>("left");
        auto right = group->get<ImgFrame>("right");
        auto roiMsg = std::dynamic_pointer_cast<FocusedDepthRoi>(group->get("roi"));
        if(!left || !right || !roiMsg) {
            logger->error("FocusedDepthCrop: MessageGroup must contain left, right, and roi (FocusedDepthRoi)");
            continue;
        }

        Rect r = roiMsg->roi;
        if(roiMsg->normalizedCoords) {
            r.hasNormalized = true;
            r.normalized = true;
        }
        r = r.denormalize(static_cast<int>(left->getWidth()), static_cast<int>(left->getHeight()));
        if(r.empty()) {
            logger->warn("FocusedDepthCrop: empty ROI");
            continue;
        }

        int rx = static_cast<int>(std::lround(r.x));
        int ry = static_cast<int>(std::lround(r.y));
        int rw = static_cast<int>(std::lround(r.width));
        int rh = static_cast<int>(std::lround(r.height));
        const int LW = static_cast<int>(left->getWidth());
        const int LH = static_cast<int>(left->getHeight());
        rx = std::clamp(rx, 0, std::max(0, LW - 1));
        ry = std::clamp(ry, 0, std::max(0, LH - 1));
        rw = std::clamp(rw, 1, std::max(1, LW - rx));
        rh = std::clamp(rh, 1, std::max(1, LH - ry));

        const float D = properties.maxDisparityPixels;
        const int cropX = static_cast<int>(std::max(0.F, std::floor(static_cast<float>(rx) - D)));
        const int unionRight = rx + rw;
        int cropW = std::min(LW - cropX, unionRight - cropX);
        int cropY = ry;
        int cropH = std::min(rh, LH - cropY);

        if(cropW < static_cast<int>(properties.minCropWidth) || cropH < static_cast<int>(properties.minCropHeight)) {
            logger->warn("FocusedDepthCrop: ROI too small after clamp ({}x{})", cropW, cropH);
            continue;
        }

        ImageManipConfig cfgL;
        cfgL.addCrop(static_cast<uint32_t>(cropX), static_cast<uint32_t>(cropY), static_cast<uint32_t>(cropW), static_cast<uint32_t>(cropH));
        if(properties.neuralResizeWidth > 0 && properties.neuralResizeHeight > 0) {
            cfgL.setOutputSize(properties.neuralResizeWidth, properties.neuralResizeHeight, ImageManipConfig::ResizeMode::LETTERBOX);
        }
        cfgL.setFrameType(left->getType());

        ImageManipConfig cfgR;
        cfgR.addCrop(static_cast<uint32_t>(cropX), static_cast<uint32_t>(cropY), static_cast<uint32_t>(cropW), static_cast<uint32_t>(cropH));
        if(properties.neuralResizeWidth > 0 && properties.neuralResizeHeight > 0) {
            cfgR.setOutputSize(properties.neuralResizeWidth, properties.neuralResizeHeight, ImageManipConfig::ResizeMode::LETTERBOX);
        }
        cfgR.setFrameType(right->getType());

        std::shared_ptr<ImgFrame> outL;
        std::shared_ptr<ImgFrame> outR;
        if(!runImageManip(*left, cfgL, properties.imageManip, outL, logger)) {
            logger->error("FocusedDepthCrop: left manip failed");
            continue;
        }
        if(!runImageManip(*right, cfgR, properties.imageManip, outR, logger)) {
            logger->error("FocusedDepthCrop: right manip failed");
            continue;
        }

        {
            auto blockEvent = outputBlockEvent();
            leftOut.send(outL);
            rightOut.send(outR);
        }
    }
#endif
}

}  // namespace node
}  // namespace dai
