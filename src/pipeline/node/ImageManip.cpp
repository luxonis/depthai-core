#include "depthai/pipeline/node/ImageManip.hpp"

#include "depthai/utility/ImageManipImpl.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {

namespace node {

inline std::array<float, 9> flatten(std::array<std::array<float, 3>, 3> mat) {
    return {mat[0][0], mat[0][1], mat[0][2], mat[1][0], mat[1][1], mat[1][2], mat[2][0], mat[2][1], mat[2][2]};
}

ImageManip::ImageManip(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, ImageManip, ImageManipProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

void ImageManip::run() {
    impl::ImageManipOperations<impl::_ImageManipBuffer, impl::_ImageManipMemory, impl::WarpH> manip(properties, pimpl->logger);
    auto iConf = runOnHost() ? *initialConfig : properties.initialConfig;
    impl::loop<ImageManip, impl::_ImageManipBuffer, impl::_ImageManipMemory>(
        *this,
        iConf,
        pimpl->logger,
        [&](const ImageManipConfig& config, const ImgFrame& frame) {
            auto srcFrameSpecs = impl::getSrcFrameSpecs(frame.fb);
            manip.build(config.base, config.outputFrameType, srcFrameSpecs, frame.getType());
            auto newCameraMatrix = impl::matmul(manip.getMatrix(), frame.transformation.getIntrinsicMatrix());
            manip.buildUndistort(config.base.undistort,
                                 flatten(frame.transformation.getIntrinsicMatrix()),
                                 flatten(newCameraMatrix),
                                 frame.transformation.getDistortionCoefficients(),
                                 frame.getType(),
                                 frame.getWidth(),
                                 frame.getHeight(),
                                 manip.getOutputWidth(),
                                 manip.getOutputHeight());
            return manip.getOutputSize();
        },
        [&](std::shared_ptr<Memory>& src, std::shared_ptr<impl::_ImageManipMemory> dst) {
            auto srcMem = std::make_shared<impl::_ImageManipMemory>(src->getData());
            return manip.apply(srcMem, dst);
        },
        [&](const ImgFrame& srcFrame, ImgFrame& dstFrame) {
            auto outType = manip.getOutputFrameType();
            auto dstSpecs = manip.getOutputFrameSpecs(outType);
            dstFrame.sourceFb = srcFrame.sourceFb;
            dstFrame.cam = srcFrame.cam;
            dstFrame.instanceNum = srcFrame.instanceNum;
            dstFrame.sequenceNum = srcFrame.sequenceNum;
            dstFrame.tsDevice = srcFrame.tsDevice;
            dstFrame.ts = srcFrame.ts;
            dstFrame.category = srcFrame.category;
            dstFrame.event = srcFrame.event;
            dstFrame.fb.height = dstSpecs.height;
            dstFrame.fb.width = dstSpecs.width;
            dstFrame.fb.stride = dstSpecs.p1Stride;
            dstFrame.fb.p1Offset = dstSpecs.p1Offset;
            dstFrame.fb.p2Offset = dstSpecs.p2Offset;
            dstFrame.fb.p3Offset = dstSpecs.p3Offset;
            dstFrame.setType(outType);

            // Transformations
            dstFrame.transformation = srcFrame.transformation;
            if(manip.undistortEnabled()) {
                dstFrame.transformation.setDistortionCoefficients({});
            }
            auto srcCrops = manip.getSrcCrops();
            dstFrame.transformation.addSrcCrops(srcCrops);
            dstFrame.transformation.addTransformation(manip.getMatrix());
            dstFrame.transformation.setSize(dstSpecs.width, dstSpecs.height);
        });
}

void ImageManip::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

void ImageManip::setMaxOutputFrameSize(int maxFrameSize) {
    properties.outputFrameSize = maxFrameSize;
}

ImageManip::Properties& ImageManip::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}
ImageManip& ImageManip::setRunOnHost(bool _runOnHost) {
    runOnHostVar = _runOnHost;
    return *this;
}
ImageManip& ImageManip::setBackend(Backend backend) {
    properties.backend = backend;
    return *this;
}
ImageManip& ImageManip::setPerformanceMode(ImageManip::PerformanceMode performanceMode) {
    properties.performanceMode = performanceMode;
    return *this;
}

/**
 * Check if the node is set to run on host
 */
bool ImageManip::runOnHost() const {
    return runOnHostVar;
}

}  // namespace node
}  // namespace dai
