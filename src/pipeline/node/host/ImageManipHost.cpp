#include "depthai/pipeline/node/host/ImageManipHost.hpp"

namespace dai {
namespace node {

using dai::impl::FrameSpecs;
FrameSpecs getSrcFrameSpecs(dai::ImgFrame::Specs srcSpecs) {
    FrameSpecs specs;
    specs.width = srcSpecs.width;
    specs.height = srcSpecs.height;
    specs.p1Offset = srcSpecs.p1Offset;
    switch(srcSpecs.type) {
        case dai::ImgFrame::Type::RGB888p:
        case dai::ImgFrame::Type::BGR888p:
            specs.p1Stride = srcSpecs.stride >= specs.width ? srcSpecs.stride : specs.width;
            specs.p2Stride = specs.p1Stride;
            specs.p3Stride = specs.p1Stride;
            specs.p2Offset = srcSpecs.p2Offset > 0 ? srcSpecs.p2Offset : specs.p1Stride * specs.height;
            specs.p3Offset = srcSpecs.p3Offset > 0 ? srcSpecs.p3Offset : 2 * specs.p1Stride * specs.height;
            break;
        case dai::ImgFrame::Type::RGB888i:
        case dai::ImgFrame::Type::BGR888i:
            specs.p1Stride = srcSpecs.stride >= 3 * specs.width ? srcSpecs.stride : 3 * specs.width;
            specs.p2Stride = specs.p1Stride;
            specs.p3Stride = specs.p1Stride;
            specs.p2Offset = specs.p1Offset;
            specs.p3Offset = specs.p1Offset;
            break;
        case dai::ImgFrame::Type::NV12:
            specs.p2Offset = srcSpecs.p2Offset > 0 ? srcSpecs.p2Offset : specs.width * specs.height;
            specs.p3Offset = specs.p2Offset;
            specs.p1Stride = srcSpecs.stride >= specs.width ? srcSpecs.stride : (specs.p2Offset - specs.p1Offset) / specs.height;
            specs.p2Stride = specs.p1Stride;
            specs.p3Stride = 0;
            break;
        case dai::ImgFrame::Type::YUV420p:
            specs.p2Offset = srcSpecs.p2Offset > 0 ? srcSpecs.p2Offset : specs.width * specs.height;
            specs.p3Offset = srcSpecs.p3Offset > srcSpecs.p2Offset ? srcSpecs.p3Offset : specs.p2Offset + (specs.width * specs.height) / 4;
            specs.p1Stride = srcSpecs.stride >= specs.width ? srcSpecs.stride : (specs.p2Offset - specs.p1Offset) / specs.height;
            specs.p2Stride = (specs.p3Offset - specs.p2Offset) / (specs.height / 2);
            specs.p3Stride = specs.p2Stride;
            break;
        case dai::ImgFrame::Type::RAW8:
        case dai::ImgFrame::Type::GRAY8:
            specs.p1Stride = srcSpecs.stride >= specs.width ? srcSpecs.stride : specs.width;
            break;
        case ImgFrame::Type::YUV422i:
        case ImgFrame::Type::YUV444p:
        case ImgFrame::Type::YUV422p:
        case ImgFrame::Type::YUV400p:
        case ImgFrame::Type::RGBA8888:
        case ImgFrame::Type::RGB161616:
        case ImgFrame::Type::LUT2:
        case ImgFrame::Type::LUT4:
        case ImgFrame::Type::LUT16:
        case ImgFrame::Type::RAW16:
        case ImgFrame::Type::RAW14:
        case ImgFrame::Type::RAW12:
        case ImgFrame::Type::RAW10:
        case ImgFrame::Type::PACK10:
        case ImgFrame::Type::PACK12:
        case ImgFrame::Type::YUV444i:
        case ImgFrame::Type::NV21:
        case ImgFrame::Type::BITSTREAM:
        case ImgFrame::Type::HDR:
        case ImgFrame::Type::RGBF16F16F16p:
        case ImgFrame::Type::BGRF16F16F16p:
        case ImgFrame::Type::RGBF16F16F16i:
        case ImgFrame::Type::BGRF16F16F16i:
        case ImgFrame::Type::GRAYF16:
        case ImgFrame::Type::RAW32:
        case ImgFrame::Type::NONE:
            throw std::runtime_error("Frame type " + std::to_string((int)srcSpecs.type) + " not supported");
            break;
    }
    return specs;
}

void ImageManipHost::run() {
    using namespace std::chrono;
    impl::ImageManipOperations manip(logger);
    manip.init();

    auto config = initialConfig;

    std::shared_ptr<ImgFrame> inImage;

    while(isRunning()) {
        std::shared_ptr<ImageManipConfigV2> pConfig;
        std::shared_ptr<Memory> inImageData;
        bool hasConfig = false;
        bool needsImage = true;
        bool skipImage = false;
        if(inputConfig.getWaitForMessage()) {
            std::cout << "trying to receive config..." << std::endl;

            pConfig = inputConfig.get<ImageManipConfigV2>();
            hasConfig = true;
            if(inImage != nullptr && hasConfig && pConfig->getReusePreviousImage()) {
                needsImage = false;
            }
            skipImage = pConfig->getSkipCurrentImage();
        } else {
            pConfig = inputConfig.tryGet<ImageManipConfigV2>();
            if(pConfig != nullptr) {
                hasConfig = true;
            }
        }

        if(needsImage) {
            inImage = inputImage.get<ImgFrame>();
#ifdef DEPTHAI_HAVE_FASTCV_SUPPORT
            {
                // Converts or just returns the same pointer if already EvaData
                inImageData = EvaDataMemory::convert(inImage->data);
                auto t2 = steady_clock::now();
                wastedTime += duration_cast<microseconds>(t2 - t1);
                logger->info("Input image to EvaData convert time: {}us", duration_cast<microseconds>(t2 - t1).count());
            }
#else
            inImageData = inImage->data;
#endif
            if(!hasConfig) {
                auto _pConfig = inputConfig.tryGet<ImageManipConfigV2>();
                if(_pConfig != nullptr) {
                    pConfig = _pConfig;
                    hasConfig = true;
                }
            }

            if(skipImage) {
                continue;
            }
        }

        // if has new config, parse and check if any changes
        if(hasConfig) {
            config = *pConfig;
        }

        auto startP = std::chrono::steady_clock::now();

        auto srcFrameSpecs = getSrcFrameSpecs(inImage->fb);
        auto t1 = steady_clock::now();
        manip.build(config.base, config.outputFrameType, srcFrameSpecs, inImage->getType());
        auto t2 = steady_clock::now();

        // Check the output image size requirements, and check whether pool has the size required
        if(manip.getOutputSize() == 0) {
            out.send(inImage);
        } else if((long)manip.getOutputSize() <= (long)properties.outputFrameSize) {
            auto outImage = std::make_shared<ImgFrame>();
            outImage->setData(std::vector<uint8_t>(properties.outputFrameSize));

            bool success = true;
            {
                auto outType = config.outputFrameType == ImgFrame::Type::NONE ? inImage->getType() : config.outputFrameType;
                auto dstSpecs = manip.getOutputFrameSpecs(outType);
                outImage->sourceFb = inImage->sourceFb;
                outImage->cam = inImage->cam;
                outImage->instanceNum = inImage->instanceNum;
                outImage->sequenceNum = inImage->sequenceNum;
                outImage->tsDevice = inImage->tsDevice;
                outImage->ts = inImage->ts;
                outImage->HFovDegrees = inImage->HFovDegrees;
                outImage->category = inImage->category;
                outImage->event = inImage->event;
                outImage->fb.height = dstSpecs.height;
                outImage->fb.width = dstSpecs.width;
                outImage->fb.stride = dstSpecs.p1Stride;
                outImage->fb.p1Offset = dstSpecs.p1Offset;
                outImage->fb.p2Offset = dstSpecs.p2Offset;
                outImage->fb.p3Offset = dstSpecs.p3Offset;
                outImage->setType(outType);

                auto t3 = steady_clock::now();
                success = manip.apply(inImage, outImage->getData());
                auto t4 = steady_clock::now();

                logger->trace("Build time: {}us, Process time: {}us, Total time: {}us, image manip id: {}",
                              duration_cast<microseconds>(t2 - t1).count(),
                              duration_cast<microseconds>(t4 - t3).count(),
                              duration_cast<microseconds>(t4 - t1).count(),
                              id);
            }
            if(!success) {
                logger->error("Processing failed, potentially unsupported config");
            }
            out.send(outImage);
        } else {
            logger->error(
                "Output image is bigger ({}B) than maximum frame size specified in properties ({}B) - skipping frame.\nPlease use the setMaxOutputFrameSize "
                "API to explicitly config the [maximum] output size.",
                manip.getOutputSize(),
                properties.outputFrameSize);
        }

        // Update previousConfig of preprocessor, to be able to check if it needs to be updated
        auto loopNanos = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - startP).count();
        logger->trace("ImageManip | total process took {}ns ({}ms)", loopNanos, (double)loopNanos / 1e6);
    }
}
}  // namespace node
}  // namespace dai
