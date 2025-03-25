#include "depthai/pipeline/node/ImageManipV2.hpp"

#include "depthai/utility/ImageManipV2Impl.hpp"

namespace dai {

namespace node {

ImageManipV2::ImageManipV2(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, ImageManipV2, ImageManipPropertiesV2>(std::move(props)), initialConfig(properties.initialConfig) {}

void ImageManipV2::run() {
    impl::ImageManipOperations<impl::_ImageManipBuffer, impl::_ImageManipMemory, impl::WarpH> manip(logger);
    manip.init();
    auto iConf = runOnHost() ? initialConfig : properties.initialConfig;
    loop<ImageManipV2, impl::_ImageManipBuffer, impl::_ImageManipMemory>(
        *this,
        iConf,
        logger,
        [&](const ImageManipConfigV2& config, const ImgFrame& frame) {
            auto srcFrameSpecs = impl::getSrcFrameSpecs(frame.fb);
            manip.build(config.base, config.outputFrameType, srcFrameSpecs, frame.getType());
            return manip.getOutputSize();
        },
        [&](std::shared_ptr<Memory>& src, std::shared_ptr<impl::_ImageManipMemory> dst) {
            auto srcMem = std::make_shared<impl::_ImageManipMemory>(src->getData());
            return manip.apply(srcMem, dst);
        },
        [&](const ImageManipConfigV2& config, const ImgFrame& srcFrame, ImgFrame& dstFrame) {
            auto outType = config.outputFrameType == ImgFrame::Type::NONE ? srcFrame.getType() : config.outputFrameType;
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
            auto srcCrops = manip.getSrcCrops();
            dstFrame.transformation.addSrcCrops(srcCrops);
            dstFrame.transformation.addTransformation(manip.getMatrix());
            dstFrame.transformation.setSize(dstSpecs.width, dstSpecs.height);
        });
}

void ImageManipV2::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

void ImageManipV2::setMaxOutputFrameSize(int maxFrameSize) {
    properties.outputFrameSize = maxFrameSize;
}

ImageManipV2::Properties& ImageManipV2::getProperties() {
    properties.initialConfig = initialConfig;
    return properties;
}
ImageManipV2& ImageManipV2::setRunOnHost(bool _runOnHost) {
    runOnHostVar = _runOnHost;
    return *this;
}

/**
 * Check if the node is set to run on host
 */
bool ImageManipV2::runOnHost() const {
    return runOnHostVar;
}

}  // namespace node
}  // namespace dai
