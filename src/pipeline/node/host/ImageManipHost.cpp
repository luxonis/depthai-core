#include "depthai/pipeline/node/host/ImageManipHost.hpp"

#include "depthai/pipeline/node/ImageManipV2.hpp"
#include "depthai/utility/ImageManipV2Impl.hpp"

namespace dai {
namespace node {

void ImageManipHost::run() {
    impl::ImageManipOperations<impl::_ImageManipBuffer, impl::_ImageManipMemory> manip(logger);
    manip.init();
    ImageManipV2::loop<ImageManipHost, impl::_ImageManipBuffer, impl::_ImageManipMemory>(
        *this,
        initialConfig,
        logger,
        [&](const ImageManipConfigV2& config, const ImgFrame& frame) {
            auto srcFrameSpecs = impl::getSrcFrameSpecs(frame.fb);
            manip.build(config.base, config.outputFrameType, srcFrameSpecs, frame.getType());
            return manip.getOutputSize();
        },
        [&](std::shared_ptr<Memory>& src, span<uint8_t> dst) { return manip.apply(src, dst); },
        [&](const ImageManipConfigV2& config, const ImgFrame& srcFrame, ImgFrame& dstFrame) {
            auto outType = config.outputFrameType == ImgFrame::Type::NONE ? srcFrame.getType() : config.outputFrameType;
            auto dstSpecs = manip.getOutputFrameSpecs(outType);
            dstFrame.sourceFb = srcFrame.sourceFb;
            dstFrame.cam = srcFrame.cam;
            dstFrame.instanceNum = srcFrame.instanceNum;
            dstFrame.sequenceNum = srcFrame.sequenceNum;
            dstFrame.tsDevice = srcFrame.tsDevice;
            dstFrame.ts = srcFrame.ts;
            dstFrame.HFovDegrees = srcFrame.HFovDegrees;
            dstFrame.category = srcFrame.category;
            dstFrame.event = srcFrame.event;
            dstFrame.fb.height = dstSpecs.height;
            dstFrame.fb.width = dstSpecs.width;
            dstFrame.fb.stride = dstSpecs.p1Stride;
            dstFrame.fb.p1Offset = dstSpecs.p1Offset;
            dstFrame.fb.p2Offset = dstSpecs.p2Offset;
            dstFrame.fb.p3Offset = dstSpecs.p3Offset;
            dstFrame.setType(outType);
        });
}

}  // namespace node
}  // namespace dai
