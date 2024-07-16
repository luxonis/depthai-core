#include "depthai/pipeline/node/ImageManipV2.hpp"

#define PLANE_ALIGNMENT 128

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #define USE_OPENCV 1
    #include <opencv2/opencv.hpp>
#endif

namespace dai {

constexpr ImgFrame::Type VALID_TYPE_COLOR = ImgFrame::Type::RGB888i;
constexpr ImgFrame::Type VALID_TYPE_GRAY = ImgFrame::Type::GRAY8;

#ifndef ALIGN_UP
template <typename T>
constexpr T ALIGN_UP(T value, std::size_t alignment) {
    return (value + (alignment - 1)) & ~(alignment - 1);
}
#endif
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
FrameSpecs getCcDstFrameSpecs(FrameSpecs srcSpecs, dai::ImgFrame::Type from, dai::ImgFrame::Type to) {
    FrameSpecs specs;
    if(from == to) return srcSpecs;
    specs.width = srcSpecs.width;
    specs.height = srcSpecs.height;
    specs.p1Offset = 0;
    switch(to) {
        case dai::ImgFrame::Type::RGB888p:
        case dai::ImgFrame::Type::BGR888p:
            specs.p1Stride = ALIGN_UP(specs.width, 8);
            specs.p2Stride = specs.p1Stride;
            specs.p3Stride = specs.p1Stride;
            specs.p2Offset = specs.p1Offset + ALIGN_UP(specs.p1Stride * specs.height, PLANE_ALIGNMENT);
            specs.p3Offset = specs.p2Offset + ALIGN_UP(specs.p1Stride * specs.height, PLANE_ALIGNMENT);
            break;
        case dai::ImgFrame::Type::RGB888i:
        case dai::ImgFrame::Type::BGR888i:
            specs.p1Stride = ALIGN_UP(specs.width * 3, 8);
            specs.p2Stride = specs.p1Stride;
            specs.p3Stride = specs.p1Stride;
            specs.p2Offset = specs.p1Offset;
            specs.p3Offset = specs.p1Offset;
            break;
        case dai::ImgFrame::Type::NV12:
            specs.p1Stride = ALIGN_UP(specs.width, 8);
            specs.p2Stride = specs.p1Stride;
            specs.p2Offset = specs.p1Offset + ALIGN_UP(specs.p1Stride * specs.height, PLANE_ALIGNMENT);
            specs.p3Offset = specs.p2Offset;
            specs.p3Stride = 0;
            break;
        case dai::ImgFrame::Type::YUV420p:
            specs.p1Stride = ALIGN_UP(specs.width, 8);
            specs.p2Stride = ALIGN_UP(specs.width / 2, 8);
            specs.p3Stride = ALIGN_UP(specs.width / 2, 8);
            specs.p2Offset = specs.p1Offset + ALIGN_UP(specs.p1Stride * specs.height, PLANE_ALIGNMENT);
            specs.p3Offset = specs.p2Offset + ALIGN_UP(specs.p2Stride * (specs.height / 2), PLANE_ALIGNMENT);
            break;
        case dai::ImgFrame::Type::RAW8:
        case dai::ImgFrame::Type::GRAY8:
            specs.p1Stride = ALIGN_UP(specs.width, 8);
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
            throw std::runtime_error("Frame type not supported");
            break;
    }
    return specs;
}

namespace node {

ImageManipV2::ImageManipV2(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, ImageManipV2, ImageManipPropertiesV2>(std::move(props)) {}

void ImageManipV2::run() {
    using namespace std::chrono;
    impl::ImageManipOperations manip(logger);
    manip.init();

    auto config = properties.initialConfig;

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

namespace impl {

static inline int clampi(int val, int minv, int maxv) {
    return val < minv ? minv : (val > maxv ? maxv : val);
}
static inline float clampf(float val, float minv, float maxv) {
    return val < minv ? minv : (val > maxv ? maxv : val);
}

bool isTypeSupportedL(dai::ImgFrame::Type type) {
    using ImgType = dai::ImgFrame::Type;
    return type == ImgType::GRAY8 || type == ImgType::RAW8 || type == ImgType::RGB888i || type == ImgType::BGR888i;
}
bool isTypeSupportedC(dai::ImgFrame::Type type) {
    using ImgType = dai::ImgFrame::Type;
    return type == ImgType::GRAY8 || type == ImgType::RAW8 || type == ImgType::RGB888i || type == ImgType::BGR888i || type == ImgType::RGB888p
           || type == ImgType::BGR888p || type == ImgType::YUV420p || type == ImgType::NV12;
}

bool getFrameTypeInfo(dai::ImgFrame::Type outFrameType, int& outNumPlanes, float& outBpp) {
    // Set output Bpp and planes by PixelFormat and interleaved options
    outNumPlanes = 3;

    switch(outFrameType) {
        case dai::ImgFrame::Type::RGB888p:
        case dai::ImgFrame::Type::BGR888p:
            outBpp = 1;
            outNumPlanes = 3;
            break;

        case dai::ImgFrame::Type::RGB888i:
        case dai::ImgFrame::Type::BGR888i:
            outBpp = 3;
            outNumPlanes = 1;
            break;

        case dai::ImgFrame::Type::RAW8:
        case dai::ImgFrame::Type::YUV400p:
        case dai::ImgFrame::Type::GRAY8:
            outBpp = 1;
            outNumPlanes = 1;
            break;

        case dai::ImgFrame::Type::RAW16:
            outBpp = 2;
            outNumPlanes = 1;
            break;

        case dai::ImgFrame::Type::YUV420p:
        case dai::ImgFrame::Type::NV12:
            outBpp = 1.5;
            outNumPlanes = 1;
            break;

        case ImgFrame::Type::YUV422i:
        case ImgFrame::Type::YUV444p:
        case ImgFrame::Type::YUV422p:
        case ImgFrame::Type::RGBA8888:
        case ImgFrame::Type::RGB161616:
        case ImgFrame::Type::LUT2:
        case ImgFrame::Type::LUT4:
        case ImgFrame::Type::LUT16:
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
            return false;
            break;
    }

    return true;
}

//-----------------------------
//--- Frame type conversion ---
//-----------------------------

static inline void YUVfromRGB(float& Y, float& U, float& V, const float R, const float G, const float B) {
    Y = 0.257f * R + 0.504f * G + 0.098f * B + 16;
    U = -0.148f * R - 0.291f * G + 0.439f * B + 128;
    V = 0.439f * R - 0.368f * G - 0.071f * B + 128;
}
static inline void RGBfromYUV(float& R, float& G, float& B, float Y, float U, float V) {
    Y -= 16;
    U -= 128;
    V -= 128;
    R = 1.164f * Y + 1.596f * V;
    G = 1.164f * Y - 0.392f * U - 0.813f * V;
    B = 1.164f * Y + 2.017f * U;
}

bool ColorChange::colorConvertToRGB888p(
    const dai::span<const uint8_t> inputFrame, dai::span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, dai::ImgFrame::Type from) {
    dai::ImgFrame::Type to = dai::ImgFrame::Type::RGB888p;

    auto src = inputFrame.data();
    auto inputSize = inputFrame.size();
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, 8);

    bool done = false;
    switch(from) {
        case dai::ImgFrame::Type::RGB888p:
            std::copy(src, src + inputSize, outputFrame.data());
            done = true;
            break;
        case dai::ImgFrame::Type::BGR888p:
            std::copy(src + srcSpecs.p1Offset, src + srcSpecs.p2Offset, outputFrame.data() + dstSpecs.p3Offset);
            std::copy(src + srcSpecs.p2Offset, src + srcSpecs.p3Offset, outputFrame.data() + dstSpecs.p2Offset);
            std::copy(src + srcSpecs.p3Offset, src + inputSize, outputFrame.data() + dstSpecs.p1Offset);
            done = true;
            break;
        case dai::ImgFrame::Type::RGB888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_0,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_1,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_2,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            cv::split(img, channels);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t srcPos = lineStart + j * 3;
                    uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    uint32_t p2Pos = dstSpecs.p2Offset + i * dstSpecs.p2Stride + j;
                    uint32_t p3Pos = dstSpecs.p3Offset + i * dstSpecs.p3Stride + j;
                    outputFrame[p1Pos] = src[srcPos + 0];
                    outputFrame[p2Pos] = src[srcPos + 1];
                    outputFrame[p3Pos] = src[srcPos + 2];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_2,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_1,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_0,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::split(img, channels);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t srcPos = lineStart + j * 3;
                    uint32_t p1Pos = dstSpecs.p3Offset + i * dstSpecs.p3Stride + j;
                    uint32_t p2Pos = dstSpecs.p2Offset + i * dstSpecs.p2Stride + j;
                    uint32_t p3Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    outputFrame[p1Pos] = src[srcPos + 0];
                    outputFrame[p2Pos] = src[srcPos + 1];
                    outputFrame[p3Pos] = src[srcPos + 2];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::NV12: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvColorYCbCr420PseudoPlanarToRGB888u8(src + srcSpecs.p1Offset,
                                                   src + srcSpecs.p2Offset,
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   srcSpecs.p1Stride,
                                                   srcSpecs.p2Stride,
                                                   ccAuxFrame->data(),
                                                   auxStride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_0,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_1,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_2,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
            cv::Mat frameY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            cv::Mat auxBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->data(), auxStride);
            cv::cvtColorTwoPlane(frameY, frameUV, auxBGR, cv::COLOR_YUV2BGR_NV12);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::split(auxBGR, channels);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::YUV420p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvColorYCbCr420PlanarToRGB888u8(src + srcSpecs.p1Offset,
                                             src + srcSpecs.p2Offset,
                                             src + srcSpecs.p3Offset,
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             srcSpecs.p1Stride,
                                             srcSpecs.p2Stride,
                                             srcSpecs.p3Stride,
                                             ccAuxFrame->data(),
                                             auxStride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_0,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_1,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_2,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStartY = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                const uint32_t lineStartU = srcSpecs.p2Offset + (i / 2) * srcSpecs.p2Stride;
                const uint32_t lineStartV = srcSpecs.p3Offset + (i / 2) * srcSpecs.p3Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    const uint32_t p2Pos = dstSpecs.p2Offset + i * dstSpecs.p2Stride + j;
                    const uint32_t p3Pos = dstSpecs.p3Offset + i * dstSpecs.p3Stride + j;
                    float Y = src[lineStartY + j];
                    float U = src[lineStartU + j / 2];
                    float V = src[lineStartV + j / 2];
                    float R, G, B;
                    RGBfromYUV(R, G, B, Y, U, V);
                    outputFrame[p1Pos] = static_cast<uint8_t>(clampi(roundf(R), 0, 255));
                    outputFrame[p2Pos] = static_cast<uint8_t>(clampi(roundf(G), 0, 255));
                    outputFrame[p3Pos] = static_cast<uint8_t>(clampi(roundf(B), 0, 255));
                }
            }
#endif
            done = true;
            break;
        }
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
        case ImgFrame::Type::RAW8:
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
        case ImgFrame::Type::GRAY8:
        case ImgFrame::Type::GRAYF16:
        case ImgFrame::Type::RAW32:
        case ImgFrame::Type::NONE:
            break;
    }

    return done;
}

bool ColorChange::colorConvertToBGR888p(
    const dai::span<const uint8_t> inputFrame, dai::span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, dai::ImgFrame::Type from) {
    dai::ImgFrame::Type to = dai::ImgFrame::Type::BGR888p;

    auto src = inputFrame.data();
    auto inputSize = inputFrame.size();
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, 8);

    bool done = false;
    switch(from) {
        case dai::ImgFrame::Type::RGB888p:
            std::copy(src + srcSpecs.p1Offset, src + srcSpecs.p2Offset, outputFrame.data() + dstSpecs.p3Offset);
            std::copy(src + srcSpecs.p2Offset, src + srcSpecs.p3Offset, outputFrame.data() + dstSpecs.p2Offset);
            std::copy(src + srcSpecs.p3Offset, src + inputSize, outputFrame.data() + dstSpecs.p1Offset);
            done = true;
            break;
        case dai::ImgFrame::Type::BGR888p:
            std::copy(src, src + inputSize, outputFrame.data());
            done = true;
            break;
        case dai::ImgFrame::Type::RGB888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_2,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_1,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_0,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::split(img, channels);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t srcPos = lineStart + j * 3;
                    uint32_t p1Pos = dstSpecs.p3Offset + i * dstSpecs.p3Stride + j;
                    uint32_t p2Pos = dstSpecs.p2Offset + i * dstSpecs.p2Stride + j;
                    uint32_t p3Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    outputFrame[p1Pos] = src[srcPos + 0];
                    outputFrame[p2Pos] = src[srcPos + 1];
                    outputFrame[p3Pos] = src[srcPos + 2];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_0,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_1,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_2,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            cv::split(img, channels);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t srcPos = lineStart + j * 3;
                    uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    uint32_t p2Pos = dstSpecs.p2Offset + i * dstSpecs.p2Stride + j;
                    uint32_t p3Pos = dstSpecs.p3Offset + i * dstSpecs.p3Stride + j;
                    outputFrame[p1Pos] = src[srcPos + 0];
                    outputFrame[p2Pos] = src[srcPos + 1];
                    outputFrame[p3Pos] = src[srcPos + 2];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::NV12: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvColorYCbCr420PseudoPlanarToRGB888u8(src + srcSpecs.p1Offset,
                                                   src + srcSpecs.p2Offset,
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   srcSpecs.p1Stride,
                                                   srcSpecs.p2Stride,
                                                   ccAuxFrame->data(),
                                                   auxStride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_2,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_1,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_0,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
            cv::Mat frameY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            cv::Mat auxBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->data(), auxStride);
            cv::cvtColorTwoPlane(frameY, frameUV, auxBGR, cv::COLOR_YUV2BGR_NV12);
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            cv::split(auxBGR, channels);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::YUV420p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvColorYCbCr420PlanarToRGB888u8(src + srcSpecs.p1Offset,
                                             src + srcSpecs.p2Offset,
                                             src + srcSpecs.p3Offset,
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             srcSpecs.p1Stride,
                                             srcSpecs.p2Stride,
                                             srcSpecs.p3Stride,
                                             ccAuxFrame->data(),
                                             auxStride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_2,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_1,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(ccAuxFrame->data(),
                                srcSpecs.width,
                                srcSpecs.height,
                                auxStride,
                                0,
                                0,
                                0,
                                0,
                                FASTCV_CHANNEL_0,
                                FASTCV_RGB,
                                outputFrame.data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStartY = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                const uint32_t lineStartU = srcSpecs.p2Offset + (i / 2) * srcSpecs.p2Stride;
                const uint32_t lineStartV = srcSpecs.p3Offset + (i / 2) * srcSpecs.p3Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    const uint32_t p2Pos = dstSpecs.p2Offset + i * dstSpecs.p2Stride + j;
                    const uint32_t p3Pos = dstSpecs.p3Offset + i * dstSpecs.p3Stride + j;
                    float Y = src[lineStartY + j];
                    float U = src[lineStartU + j / 2];
                    float V = src[lineStartV + j / 2];
                    float R, G, B;
                    RGBfromYUV(R, G, B, Y, U, V);
                    outputFrame[p1Pos] = static_cast<uint8_t>(clampi(roundf(B), 0, 255));
                    outputFrame[p2Pos] = static_cast<uint8_t>(clampi(roundf(G), 0, 255));
                    outputFrame[p3Pos] = static_cast<uint8_t>(clampi(roundf(R), 0, 255));
                }
            }
#endif
            done = true;
            break;
        }
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
        case ImgFrame::Type::RAW8:
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
        case ImgFrame::Type::GRAY8:
        case ImgFrame::Type::GRAYF16:
        case ImgFrame::Type::RAW32:
        case ImgFrame::Type::NONE:
            break;
    }

    return done;
}

bool ColorChange::colorConvertToRGB888i(
    const dai::span<const uint8_t> inputFrame, dai::span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, dai::ImgFrame::Type from) {
    dai::ImgFrame::Type to = dai::ImgFrame::Type::RGB888i;

    auto src = inputFrame.data();
    auto inputSize = inputFrame.size();
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, 8);

    bool done = false;
    switch(from) {
        case dai::ImgFrame::Type::RGB888p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvChannelCombine3Planesu8(src + srcSpecs.p1Offset,
                                       srcSpecs.width,
                                       srcSpecs.height,
                                       srcSpecs.p1Stride,
                                       src + srcSpecs.p2Offset,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p3Offset,
                                       srcSpecs.p3Stride,
                                       outputFrame.data() + dstSpecs.p1Offset,
                                       dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::merge(channels, img);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStart + j * 3;
                    uint32_t p1Pos = srcSpecs.p1Offset + i * srcSpecs.p1Stride + j;
                    uint32_t p2Pos = srcSpecs.p2Offset + i * srcSpecs.p2Stride + j;
                    uint32_t p3Pos = srcSpecs.p3Offset + i * srcSpecs.p3Stride + j;
                    outputFrame[dstPos + 0] = src[p1Pos];
                    outputFrame[dstPos + 1] = src[p2Pos];
                    outputFrame[dstPos + 2] = src[p3Pos];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvChannelCombine3Planesu8(src + srcSpecs.p3Offset,
                                       srcSpecs.width,
                                       srcSpecs.height,
                                       srcSpecs.p3Stride,
                                       src + srcSpecs.p2Offset,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p1Offset,
                                       srcSpecs.p1Stride,
                                       outputFrame.data() + dstSpecs.p1Offset,
                                       dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::merge(channels, img);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStart + j * 3;
                    uint32_t p1Pos = srcSpecs.p3Offset + i * srcSpecs.p3Stride + j;
                    uint32_t p2Pos = srcSpecs.p2Offset + i * srcSpecs.p2Stride + j;
                    uint32_t p3Pos = srcSpecs.p1Offset + i * srcSpecs.p1Stride + j;
                    outputFrame[dstPos + 0] = src[p1Pos];
                    outputFrame[dstPos + 1] = src[p2Pos];
                    outputFrame[dstPos + 2] = src[p3Pos];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::RGB888i:
            std::copy(src, src + inputSize, outputFrame.data());
            done = true;
            break;
        case dai::ImgFrame::Type::BGR888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvColorRGB888ToBGR888u8(
                src + srcSpecs.p1Offset, srcSpecs.width, srcSpecs.height, srcSpecs.p1Stride, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat imgBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(img, imgBGR, cv::COLOR_RGB2BGR);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStartSrc = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                uint32_t lineStartDst = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStartDst + j * 3;
                    uint32_t srcPos = lineStartSrc + j * 3;
                    outputFrame[dstPos + 0] = src[srcPos + 2];
                    outputFrame[dstPos + 1] = src[srcPos + 1];
                    outputFrame[dstPos + 2] = src[srcPos + 0];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::NV12: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvColorYCbCr420PseudoPlanarToRGB888u8(src + srcSpecs.p1Offset,
                                                   src + srcSpecs.p2Offset,
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   srcSpecs.p1Stride,
                                                   srcSpecs.p2Stride,
                                                   outputFrame.data() + dstSpecs.p1Offset,
                                                   dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
            cv::Mat frameY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            cv::Mat auxBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->data(), auxStride);
            cv::cvtColorTwoPlane(frameY, frameUV, auxBGR, cv::COLOR_YUV2BGR_NV12);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(auxBGR, img, cv::COLOR_RGB2BGR);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::YUV420p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvColorYCbCr420PlanarToRGB888u8(src + srcSpecs.p1Offset,
                                             src + srcSpecs.p2Offset,
                                             src + srcSpecs.p3Offset,
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             srcSpecs.p1Stride,
                                             srcSpecs.p2Stride,
                                             srcSpecs.p3Stride,
                                             outputFrame.data() + dstSpecs.p1Offset,
                                             dstSpecs.p1Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStartY = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                const uint32_t lineStartU = srcSpecs.p2Offset + (i / 2) * srcSpecs.p2Stride;
                const uint32_t lineStartV = srcSpecs.p3Offset + (i / 2) * srcSpecs.p3Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + 3 * j;
                    float Y = src[lineStartY + j];
                    float U = src[lineStartU + j / 2];
                    float V = src[lineStartV + j / 2];
                    float R, G, B;
                    RGBfromYUV(R, G, B, Y, U, V);
                    outputFrame[pos + 0] = static_cast<uint8_t>(clampi(roundf(R), 0, 255.0f));
                    outputFrame[pos + 1] = static_cast<uint8_t>(clampi(roundf(G), 0, 255.0f));
                    outputFrame[pos + 2] = static_cast<uint8_t>(clampi(roundf(B), 0, 255.0f));
                }
            }
#endif
            done = true;
            break;
        }
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
        case ImgFrame::Type::RAW8:
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
        case ImgFrame::Type::GRAY8:
        case ImgFrame::Type::GRAYF16:
        case ImgFrame::Type::RAW32:
        case ImgFrame::Type::NONE:
            break;
    }

    return done;
}

bool ColorChange::colorConvertToBGR888i(
    const dai::span<const uint8_t> inputFrame, dai::span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, dai::ImgFrame::Type from) {
    dai::ImgFrame::Type to = dai::ImgFrame::Type::BGR888i;

    auto src = inputFrame.data();
    auto inputSize = inputFrame.size();
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, 8);

    bool done = false;
    switch(from) {
        case dai::ImgFrame::Type::RGB888p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvChannelCombine3Planesu8(src + srcSpecs.p3Offset,
                                       srcSpecs.width,
                                       srcSpecs.height,
                                       srcSpecs.p3Stride,
                                       src + srcSpecs.p2Offset,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p1Offset,
                                       srcSpecs.p1Stride,
                                       outputFrame.data() + dstSpecs.p1Offset,
                                       dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::merge(channels, img);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStart + j * 3;
                    uint32_t p1Pos = srcSpecs.p3Offset + i * srcSpecs.p3Stride + j;
                    uint32_t p2Pos = srcSpecs.p2Offset + i * srcSpecs.p2Stride + j;
                    uint32_t p3Pos = srcSpecs.p1Offset + i * srcSpecs.p1Stride + j;
                    outputFrame[dstPos + 0] = src[p1Pos];
                    outputFrame[dstPos + 1] = src[p2Pos];
                    outputFrame[dstPos + 2] = src[p3Pos];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvChannelCombine3Planesu8(src + srcSpecs.p1Offset,
                                       srcSpecs.width,
                                       srcSpecs.height,
                                       srcSpecs.p1Stride,
                                       src + srcSpecs.p2Offset,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p3Offset,
                                       srcSpecs.p3Stride,
                                       outputFrame.data() + dstSpecs.p1Offset,
                                       dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::merge(channels, img);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStart = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStart + j * 3;
                    uint32_t p1Pos = srcSpecs.p1Offset + i * srcSpecs.p1Stride + j;
                    uint32_t p2Pos = srcSpecs.p2Offset + i * srcSpecs.p2Stride + j;
                    uint32_t p3Pos = srcSpecs.p3Offset + i * srcSpecs.p3Stride + j;
                    outputFrame[dstPos + 0] = src[p1Pos];
                    outputFrame[dstPos + 1] = src[p2Pos];
                    outputFrame[dstPos + 2] = src[p3Pos];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::RGB888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvColorRGB888ToBGR888u8(
                src + srcSpecs.p1Offset, srcSpecs.width, srcSpecs.height, srcSpecs.p1Stride, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
            cv::Mat img(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat imgBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(img, imgBGR, cv::COLOR_RGB2BGR);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                uint32_t lineStartSrc = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                uint32_t lineStartDst = dstSpecs.p1Offset + i * dstSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    uint32_t dstPos = lineStartDst + j * 3;
                    uint32_t srcPos = lineStartSrc + j * 3;
                    outputFrame[dstPos + 0] = src[srcPos + 2];
                    outputFrame[dstPos + 1] = src[srcPos + 1];
                    outputFrame[dstPos + 2] = src[srcPos + 0];
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888i:
            std::copy(src, src + inputSize, outputFrame.data());
            done = true;
            break;
        case dai::ImgFrame::Type::NV12: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvColorYCbCr420PseudoPlanarToRGB888u8(src + srcSpecs.p1Offset,
                                                   src + srcSpecs.p2Offset,
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   srcSpecs.p1Stride,
                                                   srcSpecs.p2Stride,
                                                   ccAuxFrame->data(),
                                                   auxStride);
            fcvColorRGB888ToBGR888u8(ccAuxFrame->data(), srcSpecs.width, srcSpecs.height, auxStride, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
            cv::Mat frameY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            cv::Mat img(dstSpecs.height, dstSpecs.width, CV_8UC3, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColorTwoPlane(frameY, frameUV, img, cv::COLOR_YUV2BGR_NV12);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::YUV420p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvColorYCbCr420PlanarToRGB888u8(src + srcSpecs.p1Offset,
                                             src + srcSpecs.p2Offset,
                                             src + srcSpecs.p3Offset,
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             srcSpecs.p1Stride,
                                             srcSpecs.p2Stride,
                                             srcSpecs.p3Stride,
                                             ccAuxFrame->data(),
                                             auxStride);
            fcvColorRGB888ToBGR888u8(ccAuxFrame->data(), srcSpecs.width, srcSpecs.height, auxStride, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStartY = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                const uint32_t lineStartU = srcSpecs.p2Offset + (i / 2) * srcSpecs.p2Stride;
                const uint32_t lineStartV = srcSpecs.p3Offset + (i / 2) * srcSpecs.p3Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + 3 * j;
                    float Y = src[lineStartY + j];
                    float U = src[lineStartU + j / 2];
                    float V = src[lineStartV + j / 2];
                    float R, G, B;
                    RGBfromYUV(R, G, B, Y, U, V);
                    outputFrame[pos + 0] = static_cast<uint8_t>(clampi(roundf(B), 0, 255.0f));
                    outputFrame[pos + 1] = static_cast<uint8_t>(clampi(roundf(G), 0, 255.0f));
                    outputFrame[pos + 2] = static_cast<uint8_t>(clampi(roundf(R), 0, 255.0f));
                }
            }
#endif
            done = true;
            break;
        }
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
        case ImgFrame::Type::RAW8:
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
        case ImgFrame::Type::GRAY8:
        case ImgFrame::Type::GRAYF16:
        case ImgFrame::Type::RAW32:
        case ImgFrame::Type::NONE:
            break;
    }

    return done;
}

bool ColorChange::colorConvertToNV12(
    const dai::span<const uint8_t> inputFrame, dai::span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, dai::ImgFrame::Type from) {
    dai::ImgFrame::Type to = dai::ImgFrame::Type::NV12;

    auto src = inputFrame.data();
    auto inputSize = inputFrame.size();
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, 8);

    bool done = false;
    switch(from) {
        case dai::ImgFrame::Type::RGB888p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvChannelCombine3Planesu8(src + srcSpecs.p3Offset,
                                       srcSpecs.width,
                                       srcSpecs.height,
                                       srcSpecs.p3Stride,
                                       src + srcSpecs.p2Offset,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p1Offset,
                                       srcSpecs.p1Stride,
                                       ccAuxFrame->data(),
                                       auxStride);
            fcvColorRGB888ToYCbCr420PseudoPlanaru8(ccAuxFrame->data(),
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   auxStride,
                                                   outputFrame.data() + dstSpecs.p1Offset,
                                                   outputFrame.data() + dstSpecs.p2Offset,
                                                   dstSpecs.p1Stride,
                                                   dstSpecs.p2Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStartR = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                const uint32_t lineStartG = srcSpecs.p2Offset + i * srcSpecs.p2Stride;
                const uint32_t lineStartB = srcSpecs.p3Offset + i * srcSpecs.p3Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    const uint32_t p2Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2) * 2;
                    const uint32_t p3Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2) * 2 + 1;
                    float R = src[lineStartR + j];
                    float G = src[lineStartG + j];
                    float B = src[lineStartB + j];
                    float Y, U, V;
                    YUVfromRGB(Y, U, V, R, G, B);
                    outputFrame[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame[p3Pos] = static_cast<uint8_t>(V);
                    }
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvChannelCombine3Planesu8(src + srcSpecs.p1Offset,
                                       srcSpecs.width,
                                       srcSpecs.height,
                                       srcSpecs.p1Stride,
                                       src + srcSpecs.p2Offset,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p3Offset,
                                       srcSpecs.p3Stride,
                                       ccAuxFrame->data(),
                                       auxStride);
            fcvColorRGB888ToYCbCr420PseudoPlanaru8(ccAuxFrame->data(),
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   auxStride,
                                                   outputFrame.data() + dstSpecs.p1Offset,
                                                   outputFrame.data() + dstSpecs.p2Offset,
                                                   dstSpecs.p1Stride,
                                                   dstSpecs.p2Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStartB = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                const uint32_t lineStartG = srcSpecs.p2Offset + i * srcSpecs.p2Stride;
                const uint32_t lineStartR = srcSpecs.p3Offset + i * srcSpecs.p3Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    const uint32_t p2Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2) * 2;
                    const uint32_t p3Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2) * 2 + 1;
                    float R = src[lineStartR + j];
                    float G = src[lineStartG + j];
                    float B = src[lineStartB + j];
                    float Y, U, V;
                    YUVfromRGB(Y, U, V, R, G, B);
                    outputFrame[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame[p3Pos] = static_cast<uint8_t>(V);
                    }
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::RGB888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvColorRGB888ToBGR888u8(src + srcSpecs.p1Offset, srcSpecs.width, srcSpecs.height, srcSpecs.p1Stride, ccAuxFrame->data(), auxStride);
            fcvColorRGB888ToYCbCr420PseudoPlanaru8(ccAuxFrame->data(),
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   auxStride,
                                                   outputFrame.data() + dstSpecs.p1Offset,
                                                   outputFrame.data() + dstSpecs.p2Offset,
                                                   dstSpecs.p1Stride,
                                                   dstSpecs.p2Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t pos = lineStart + j * 3;
                    const uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    const uint32_t p2Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2) * 2;
                    const uint32_t p3Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2) * 2 + 1;
                    float R = src[pos + 0];
                    float G = src[pos + 1];
                    float B = src[pos + 2];
                    float Y, U, V;
                    YUVfromRGB(Y, U, V, R, G, B);
                    outputFrame[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame[p3Pos] = static_cast<uint8_t>(V);
                    }
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888i:
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvColorRGB888ToYCbCr420PseudoPlanaru8(src + srcSpecs.p1Offset,
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   srcSpecs.p1Stride,
                                                   outputFrame.data() + dstSpecs.p1Offset,
                                                   outputFrame.data() + dstSpecs.p2Offset,
                                                   dstSpecs.p1Stride,
                                                   dstSpecs.p2Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t pos = lineStart + j * 3;
                    const uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    const uint32_t p2Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2) * 2;
                    const uint32_t p3Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2) * 2 + 1;
                    float B = src[pos + 0];
                    float G = src[pos + 1];
                    float R = src[pos + 2];
                    float Y, U, V;
                    YUVfromRGB(Y, U, V, R, G, B);
                    outputFrame[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame[p3Pos] = static_cast<uint8_t>(V);
                    }
                }
            }
#endif
            done = true;
            break;
        case dai::ImgFrame::Type::NV12:
            std::copy(src, src + inputSize, outputFrame.data());
            done = true;
            break;
        case dai::ImgFrame::Type::YUV420p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                src + srcSpecs.p2Offset,
                                srcSpecs.p2Stride,
                                src + srcSpecs.p3Offset,
                                srcSpecs.p3Stride,
                                FASTCV_CHANNEL_0,
                                FASTCV_IYUV,
                                outputFrame.data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelCombine2Planesu8(src + srcSpecs.p2Offset,
                                       srcSpecs.width / 2,
                                       srcSpecs.height / 2,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p3Offset,
                                       srcSpecs.p3Stride,
                                       outputFrame.data() + dstSpecs.p2Offset,
                                       dstSpecs.p2Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
            std::vector<cv::Mat> channels;
            channels.reserve(2);
            channels.emplace_back(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            cv::Mat frameUV(dstSpecs.height / 2, dstSpecs.width / 2, CV_8UC2, outputFrame.data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            cv::merge(channels, frameUV);
            cv::Mat srcY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat dstY(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            srcY.copyTo(dstY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
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
        case ImgFrame::Type::RAW8:
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
        case ImgFrame::Type::GRAY8:
        case ImgFrame::Type::GRAYF16:
        case ImgFrame::Type::RAW32:
        case ImgFrame::Type::NONE:
            break;
    }

    return done;
}

bool ColorChange::colorConvertToYUV420p(
    const dai::span<const uint8_t> inputFrame, dai::span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, dai::ImgFrame::Type from) {
    dai::ImgFrame::Type to = dai::ImgFrame::Type::YUV420p;

    auto src = inputFrame.data();
    auto inputSize = inputFrame.size();
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, 8);

    bool done = false;
    switch(from) {
        case dai::ImgFrame::Type::RGB888p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvChannelCombine3Planesu8(src + srcSpecs.p3Offset,
                                       srcSpecs.width,
                                       srcSpecs.height,
                                       srcSpecs.p3Stride,
                                       src + srcSpecs.p2Offset,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p1Offset,
                                       srcSpecs.p1Stride,
                                       ccAuxFrame->data(),
                                       auxStride);
            fcvColorRGB888ToYCbCr420Planaru8(ccAuxFrame->data(),
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             auxStride,
                                             outputFrame.data() + dstSpecs.p1Offset,
                                             outputFrame.data() + dstSpecs.p2Offset,
                                             outputFrame.data() + dstSpecs.p3Offset,
                                             dstSpecs.p1Stride,
                                             dstSpecs.p2Stride,
                                             dstSpecs.p3Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStartR = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                const uint32_t lineStartG = srcSpecs.p2Offset + i * srcSpecs.p2Stride;
                const uint32_t lineStartB = srcSpecs.p3Offset + i * srcSpecs.p3Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    const uint32_t p2Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2);
                    const uint32_t p3Pos = dstSpecs.p3Offset + (i / 2) * dstSpecs.p3Stride + (j / 2);
                    float R = src[lineStartR + j];
                    float G = src[lineStartG + j];
                    float B = src[lineStartB + j];
                    float Y, U, V;
                    YUVfromRGB(Y, U, V, R, G, B);
                    outputFrame[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame[p3Pos] = static_cast<uint8_t>(V);
                    }
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvChannelCombine3Planesu8(src + srcSpecs.p1Offset,
                                       srcSpecs.width,
                                       srcSpecs.height,
                                       srcSpecs.p1Stride,
                                       src + srcSpecs.p2Offset,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p3Offset,
                                       srcSpecs.p3Stride,
                                       ccAuxFrame->data(),
                                       auxStride);
            fcvColorRGB888ToYCbCr420Planaru8(ccAuxFrame->data(),
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             auxStride,
                                             outputFrame.data() + dstSpecs.p1Offset,
                                             outputFrame.data() + dstSpecs.p2Offset,
                                             outputFrame.data() + dstSpecs.p3Offset,
                                             dstSpecs.p1Stride,
                                             dstSpecs.p2Stride,
                                             dstSpecs.p3Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStartR = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                const uint32_t lineStartG = srcSpecs.p2Offset + i * srcSpecs.p2Stride;
                const uint32_t lineStartB = srcSpecs.p3Offset + i * srcSpecs.p3Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    const uint32_t p2Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2);
                    const uint32_t p3Pos = dstSpecs.p3Offset + (i / 2) * dstSpecs.p3Stride + (j / 2);
                    float B = src[lineStartR + j];
                    float G = src[lineStartG + j];
                    float R = src[lineStartB + j];
                    float Y, U, V;
                    YUVfromRGB(Y, U, V, R, G, B);
                    outputFrame[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame[p3Pos] = static_cast<uint8_t>(V);
                    }
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::RGB888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvColorRGB888ToBGR888u8(src + srcSpecs.p1Offset, srcSpecs.width, srcSpecs.height, srcSpecs.p1Stride, ccAuxFrame->data(), auxStride);
            fcvColorRGB888ToYCbCr420Planaru8(ccAuxFrame->data(),
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             auxStride,
                                             outputFrame.data() + dstSpecs.p1Offset,
                                             outputFrame.data() + dstSpecs.p2Offset,
                                             outputFrame.data() + dstSpecs.p3Offset,
                                             dstSpecs.p1Stride,
                                             dstSpecs.p2Stride,
                                             dstSpecs.p3Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t pos = lineStart + j * 3;
                    const uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    const uint32_t p2Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2);
                    const uint32_t p3Pos = dstSpecs.p3Offset + (i / 2) * dstSpecs.p3Stride + (j / 2);
                    float R = src[pos + 0];
                    float G = src[pos + 1];
                    float B = src[pos + 2];
                    float Y, U, V;
                    YUVfromRGB(Y, U, V, R, G, B);
                    outputFrame[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame[p3Pos] = static_cast<uint8_t>(V);
                    }
                }
            }
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888i:
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvColorRGB888ToYCbCr420Planaru8(src + srcSpecs.p1Offset,
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             srcSpecs.p1Stride,
                                             outputFrame.data() + dstSpecs.p1Offset,
                                             outputFrame.data() + dstSpecs.p2Offset,
                                             outputFrame.data() + dstSpecs.p3Offset,
                                             dstSpecs.p1Stride,
                                             dstSpecs.p2Stride,
                                             dstSpecs.p3Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStart = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t pos = lineStart + j * 3;
                    const uint32_t p1Pos = dstSpecs.p1Offset + i * dstSpecs.p1Stride + j;
                    const uint32_t p2Pos = dstSpecs.p2Offset + (i / 2) * dstSpecs.p2Stride + (j / 2);
                    const uint32_t p3Pos = dstSpecs.p3Offset + (i / 2) * dstSpecs.p3Stride + (j / 2);
                    float B = src[pos + 0];
                    float G = src[pos + 1];
                    float R = src[pos + 2];
                    float Y, U, V;
                    YUVfromRGB(Y, U, V, R, G, B);
                    outputFrame[p1Pos] = static_cast<uint8_t>(Y);
                    if(i % 2 == 0 && j % 2 == 0) {
                        outputFrame[p2Pos] = static_cast<uint8_t>(U);
                        outputFrame[p3Pos] = static_cast<uint8_t>(V);
                    }
                }
            }
#endif
            done = true;
            break;
        case dai::ImgFrame::Type::NV12: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                src + srcSpecs.p2Offset,
                                srcSpecs.p2Stride,
                                0,
                                0,
                                FASTCV_CHANNEL_Y,
                                FASTCV_NV12,
                                outputFrame.data() + dstSpecs.p1Offset,
                                dstSpecs.p1Stride);
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                src + srcSpecs.p2Offset,
                                srcSpecs.p2Stride,
                                0,
                                0,
                                FASTCV_CHANNEL_U,
                                FASTCV_NV12,
                                outputFrame.data() + dstSpecs.p2Offset,
                                dstSpecs.p2Stride);
            fcvChannelExtractu8(src + srcSpecs.p1Offset,
                                srcSpecs.width,
                                srcSpecs.height,
                                srcSpecs.p1Stride,
                                src + srcSpecs.p2Offset,
                                srcSpecs.p2Stride,
                                0,
                                0,
                                FASTCV_CHANNEL_V,
                                FASTCV_NV12,
                                outputFrame.data() + dstSpecs.p3Offset,
                                dstSpecs.p3Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            std::vector<cv::Mat> channels;
            channels.reserve(2);
            channels.emplace_back(dstSpecs.height / 2, dstSpecs.width / 2, CV_8UC1, outputFrame.data() + dstSpecs.p2Offset, dstSpecs.p2Stride);
            channels.emplace_back(dstSpecs.height / 2, dstSpecs.width / 2, CV_8UC1, outputFrame.data() + dstSpecs.p3Offset, dstSpecs.p3Stride);
            cv::split(frameUV, channels);
            cv::Mat srcY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat dstY(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            srcY.copyTo(dstY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::YUV420p:
            std::copy(src, src + inputSize, outputFrame.data());
            done = true;
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
        case ImgFrame::Type::RAW8:
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
        case ImgFrame::Type::GRAY8:
        case ImgFrame::Type::GRAYF16:
        case ImgFrame::Type::RAW32:
        case ImgFrame::Type::NONE:
            break;
    }

    return done;
}

bool ColorChange::colorConvertToGRAY8(
    const dai::span<const uint8_t> inputFrame, dai::span<uint8_t> outputFrame, FrameSpecs srcSpecs, FrameSpecs dstSpecs, dai::ImgFrame::Type from) {
    dai::ImgFrame::Type to = dai::ImgFrame::Type::GRAY8;

    auto src = inputFrame.data();
    auto inputSize = inputFrame.size();
    uint32_t auxStride = ALIGN_UP(3 * srcSpecs.width, 8);

    bool done = false;
    switch(from) {
        case dai::ImgFrame::Type::RGB888p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvChannelCombine3Planesu8(src + srcSpecs.p1Offset,
                                       srcSpecs.width,
                                       srcSpecs.height,
                                       srcSpecs.p1Stride,
                                       src + srcSpecs.p2Offset,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p3Offset,
                                       srcSpecs.p3Stride,
                                       ccAuxFrame->data(),
                                       auxStride);
            fcvColorRGB888ToGrayu8(ccAuxFrame->data(), srcSpecs.width, srcSpecs.height, auxStride, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            cv::Mat auxRGB(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->data(), auxStride);
            cv::merge(channels, auxRGB);
            // Convert to grayscale
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(auxRGB, gray, cv::COLOR_RGB2GRAY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvChannelCombine3Planesu8(src + srcSpecs.p3Offset,
                                       srcSpecs.width,
                                       srcSpecs.height,
                                       srcSpecs.p3Stride,
                                       src + srcSpecs.p2Offset,
                                       srcSpecs.p2Stride,
                                       src + srcSpecs.p1Offset,
                                       srcSpecs.p1Stride,
                                       ccAuxFrame->data(),
                                       auxStride);
            fcvColorRGB888ToGrayu8(ccAuxFrame->data(), srcSpecs.width, srcSpecs.height, auxStride, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
            std::vector<cv::Mat> channels;
            channels.reserve(3);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            channels.emplace_back(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p3Offset), srcSpecs.p3Stride);
            cv::Mat auxRGB(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->data(), auxStride);
            cv::merge(channels, auxRGB);
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(auxRGB, gray, cv::COLOR_BGR2GRAY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::RGB888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvColorRGB888ToGrayu8(
                src + srcSpecs.p1Offset, srcSpecs.width, srcSpecs.height, srcSpecs.p1Stride, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
            cv::Mat frameRGB(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(frameRGB, gray, cv::COLOR_RGB2GRAY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::BGR888i: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvColorRGB888ToBGR888u8(src + srcSpecs.p1Offset, srcSpecs.width, srcSpecs.height, srcSpecs.p1Stride, ccAuxFrame->data(), auxStride);
            fcvColorRGB888ToGrayu8(ccAuxFrame->data(), srcSpecs.width, srcSpecs.height, auxStride, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
            cv::Mat frameBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(frameBGR, gray, cv::COLOR_BGR2GRAY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::NV12: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvColorYCbCr420PseudoPlanarToRGB888u8(src + srcSpecs.p1Offset,
                                                   src + srcSpecs.p2Offset,
                                                   srcSpecs.width,
                                                   srcSpecs.height,
                                                   srcSpecs.p1Stride,
                                                   srcSpecs.p2Stride,
                                                   ccAuxFrame->data(),
                                                   auxStride);
            fcvColorRGB888ToGrayu8(ccAuxFrame->data(), srcSpecs.width, srcSpecs.height, auxStride, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
            cv::Mat frameY(srcSpecs.height, srcSpecs.width, CV_8UC1, const_cast<uint8_t*>(src + srcSpecs.p1Offset), srcSpecs.p1Stride);
            cv::Mat frameUV(srcSpecs.height / 2, srcSpecs.width / 2, CV_8UC2, const_cast<uint8_t*>(src + srcSpecs.p2Offset), srcSpecs.p2Stride);
            cv::Mat auxBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->data(), auxStride);
            cv::cvtColorTwoPlane(frameY, frameUV, auxBGR, cv::COLOR_YUV2BGR_NV12);
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(auxBGR, gray, cv::COLOR_BGR2GRAY);
#else
            throw std::runtime_error("FastCV or OpenCV support required for this conversion");
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::YUV420p: {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
            fcvColorYCbCr420PlanarToRGB888u8(src + srcSpecs.p1Offset,
                                             src + srcSpecs.p2Offset,
                                             src + srcSpecs.p3Offset,
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             srcSpecs.p1Stride,
                                             srcSpecs.p2Stride,
                                             srcSpecs.p3Stride,
                                             ccAuxFrame->data(),
                                             auxStride);
            fcvColorRGB888ToGrayu8(ccAuxFrame->data(), srcSpecs.width, srcSpecs.height, auxStride, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
#else
            for(uint32_t i = 0; i < srcSpecs.height; ++i) {
                const uint32_t lineStartY = srcSpecs.p1Offset + i * srcSpecs.p1Stride;
                const uint32_t lineStartU = srcSpecs.p2Offset + (i / 2) * srcSpecs.p2Stride;
                const uint32_t lineStartV = srcSpecs.p3Offset + (i / 2) * srcSpecs.p3Stride;
                for(uint32_t j = 0; j < srcSpecs.width; ++j) {
                    const uint32_t pos = srcSpecs.p1Offset + i * auxStride + 3 * j;
                    float Y = src[lineStartY + j];
                    float U = src[lineStartU + j / 2];
                    float V = src[lineStartV + j / 2];
                    float R, G, B;
                    RGBfromYUV(R, G, B, Y, U, V);
                    ccAuxFrame->data()[pos + 0] = static_cast<uint8_t>(clampi(roundf(B), 0, 255.0f));
                    ccAuxFrame->data()[pos + 1] = static_cast<uint8_t>(clampi(roundf(G), 0, 255.0f));
                    ccAuxFrame->data()[pos + 2] = static_cast<uint8_t>(clampi(roundf(R), 0, 255.0f));
                }
            }
            cv::Mat auxBGR(srcSpecs.height, srcSpecs.width, CV_8UC3, ccAuxFrame->data(), auxStride);
            cv::Mat gray(dstSpecs.height, dstSpecs.width, CV_8UC1, outputFrame.data() + dstSpecs.p1Offset, dstSpecs.p1Stride);
            cv::cvtColor(auxBGR, gray, cv::COLOR_BGR2GRAY);
#endif
            done = true;
            break;
        }
        case dai::ImgFrame::Type::RAW8:
        case dai::ImgFrame::Type::GRAY8:
            std::copy(src, src + inputSize, outputFrame.data());
            done = true;
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
            break;
    }

    return done;
}

void ColorChange::build(const FrameSpecs srcFrameSpecs, const FrameSpecs dstFrameSpecs, const ImgFrame::Type typeFrom, const ImgFrame::Type typeTo) {
    from = typeFrom;
    to = typeTo;
    srcSpecs = srcFrameSpecs;
    dstSpecs = dstFrameSpecs;
    size_t newAuxFrameSize = srcSpecs.height * ALIGN_UP(3 * srcSpecs.width, 8);
    if(!ccAuxFrame || ccAuxFrame->size() < newAuxFrameSize) ccAuxFrame = std::make_shared<dai::impl::ImageManipMemory>(newAuxFrameSize);
}

void ColorChange::apply(span<const uint8_t> src, span<uint8_t> dst) {
    float bpp;
    int numPlanes;
    getFrameTypeInfo(to, numPlanes, bpp);

    logger->debug("From {} ({}):\n\t{}x{}\n\tstride {} {} {}\n\toffset {} {} {}\n\ttotal size {}\n",
                  (int)from,
                  (void*)src.data(),
                  (int)srcSpecs.width,
                  (int)srcSpecs.height,
                  (int)srcSpecs.p1Stride,
                  (int)srcSpecs.p2Stride,
                  (int)srcSpecs.p3Stride,
                  (int)srcSpecs.p1Offset,
                  (int)srcSpecs.p2Offset,
                  (int)srcSpecs.p3Offset,
                  (int)src.size());
    logger->debug("To {} ({}):\n\t{}x{}\n\tstride {} {} {}\n\toffset {} {} {}\n\ttotal size {}\n",
                  (int)to,
                  (void*)dst.data(),
                  (int)dstSpecs.width,
                  (int)dstSpecs.height,
                  (int)dstSpecs.p1Stride,
                  (int)dstSpecs.p2Stride,
                  (int)dstSpecs.p3Stride,
                  (int)dstSpecs.p1Offset,
                  (int)dstSpecs.p2Offset,
                  (int)dstSpecs.p3Offset,
                  (int)dst.size());

    bool done = false;
    auto start = std::chrono::steady_clock::now();
    switch(to) {
        case dai::ImgFrame::Type::RGB888p:
            done = colorConvertToRGB888p(src, dst, srcSpecs, dstSpecs, from);
            break;
        case dai::ImgFrame::Type::BGR888p:
            done = colorConvertToBGR888p(src, dst, srcSpecs, dstSpecs, from);
            break;
        case dai::ImgFrame::Type::RGB888i:
            done = colorConvertToRGB888i(src, dst, srcSpecs, dstSpecs, from);
            break;
        case dai::ImgFrame::Type::BGR888i:
            done = colorConvertToBGR888i(src, dst, srcSpecs, dstSpecs, from);
            break;
        case dai::ImgFrame::Type::NV12:
            done = colorConvertToNV12(src, dst, srcSpecs, dstSpecs, from);
            break;
        case dai::ImgFrame::Type::YUV420p:
            done = colorConvertToYUV420p(src, dst, srcSpecs, dstSpecs, from);
            break;
        case dai::ImgFrame::Type::GRAY8:
        case dai::ImgFrame::Type::RAW8:
            done = colorConvertToGRAY8(src, dst, srcSpecs, dstSpecs, from);
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
            break;
    }
    auto diff = std::chrono::steady_clock::now() - start;
    if(logger) logger->trace("ImageManip | colorConvert took {}ns", std::chrono::duration_cast<std::chrono::nanoseconds>(diff).count());

    if(!done) {
        if(logger) logger->error("Convert color from {} to {} not supported or failed.", (int)from, (int)to);
        std::copy(src.data(), src.data() + (src.size() <= dst.size() ? src.size() : dst.size()), dst.data());
    }
}

//------------
//--- Warp ---
//------------

// helper type for the visitor #4
template <class... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};
// explicit deduction guide (not needed as of C++20)
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

bool float_eq(float a, float b) {
    return fabs(a - b) <= 1e-6f;
}

bool isSingleChannelu8(const std::shared_ptr<dai::ImgFrame> img) {
    return img->getType() == dai::ImgFrame::Type::GRAY8 || img->getType() == dai::ImgFrame::Type::RAW8;
}
bool isSingleChannelu8(const dai::ImgFrame::Type type) {
    return type == dai::ImgFrame::Type::GRAY8 || type == dai::ImgFrame::Type::RAW8;
}

template <typename T>
std::string getOpStr(const T& op) {
    return op.toStr();
}

std::string getConfigString(const dai::ImageManipOpsBase& ops) {
    std::stringstream configSS;
    const auto operations = ops.getOperations();
    for(auto i = 0U; i < operations.size(); ++i) {
        configSS << std::visit([](auto&& op) { return getOpStr(op); }, operations[i].op);
        if(i != operations.size() - 1) configSS << " ";
    }
    return configSS.str();
}

std::array<std::array<float, 3>, 3> matmul(std::array<std::array<float, 3>, 3> A, std::array<std::array<float, 3>, 3> B) {
    return {{{A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0],
              A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1],
              A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2]},
             {A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0],
              A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1],
              A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2]},
             {A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0],
              A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1],
              A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2]}}};
}

std::array<float, 2> matvecmul(std::array<std::array<float, 3>, 3> M, std::array<float, 2> vec) {
    auto x = M[0][0] * vec[0] + M[0][1] * vec[1] + M[0][2];
    auto y = M[1][0] * vec[0] + M[1][1] * vec[1] + M[1][2];
    auto z = M[2][0] * vec[0] + M[2][1] * vec[1] + M[2][2];
    return {x / z, y / z};
}

std::tuple<float, float, float, float> getOuterRect(const std::vector<std::array<float, 2>> points) {
    float minx = points[0][0];
    float maxx = points[0][0];
    float miny = points[0][1];
    float maxy = points[0][1];

    for(auto i = 0U; i < points.size(); ++i) {
        minx = std::min(points[i][0], minx);
        maxx = std::max(points[i][0], maxx);
        miny = std::min(points[i][1], miny);
        maxy = std::max(points[i][1], maxy);
    }
    return {minx, maxx, miny, maxy};
}

std::vector<std::array<float, 2>> getHull(const std::vector<std::array<float, 2>> points) {
    std::vector<std::array<float, 2>> remaining(points.rbegin(), points.rend() - 1);
    std::vector<std::array<float, 2>> hull{points.front()};
    hull.reserve(points.size());
    while(remaining.size() > 0) {
        auto pt = remaining.back();
        remaining.pop_back();
        while(hull.size() >= 2) {
            auto last1 = hull.size() - 1;
            auto last2 = hull.size() - 2;
            std::array<float, 2> v1 = {hull[last1][0] - hull[last2][0], hull[last1][1] - hull[last2][1]};
            std::array<float, 2> v2 = {pt[0] - hull[last1][0], pt[1] - hull[last1][1]};
            std::array<float, 2> v3 = {hull[0][0] - pt[0], hull[0][1] - pt[1]};
            auto cross1 = v1[0] * v2[1] - v1[1] * v2[0];
            auto cross2 = v2[0] * v3[1] - v2[1] * v3[0];
            if(cross1 < 0 || cross2 < 0) {
                remaining.push_back(hull.back());
                hull.pop_back();
            } else if(cross1 == 0 || cross2 == 0) {
                throw std::runtime_error("Colinear points");
            } else {
                break;
            }
        }
        hull.push_back(pt);
    }
    return hull;
}

std::array<std::array<float, 2>, 2> getInverse(const std::array<std::array<float, 2>, 2> mat) {
    auto det = mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
    if(det == 0) {
        throw std::runtime_error("Determinant is zero");
    }
    return {{{mat[1][1] / det, -mat[0][1] / det}, {-mat[1][0] / det, mat[0][0] / det}}};
}

std::array<std::array<float, 3>, 3> getInverse(const std::array<std::array<float, 3>, 3>& matrix) {
    std::array<std::array<float, 3>, 3> inv;
    float det = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
                - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
                + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);

    if(det == 0) {
        throw std::runtime_error("Matrix is singular and cannot be inverted.");
    }

    std::array<std::array<float, 3>, 3> adj;

    adj[0][0] = (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]);
    adj[0][1] = -(matrix[0][1] * matrix[2][2] - matrix[0][2] * matrix[2][1]);
    adj[0][2] = (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]);

    adj[1][0] = -(matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]);
    adj[1][1] = (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]);
    adj[1][2] = -(matrix[0][0] * matrix[1][2] - matrix[0][2] * matrix[1][0]);

    adj[2][0] = (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
    adj[2][1] = -(matrix[0][0] * matrix[2][1] - matrix[0][1] * matrix[2][0]);
    adj[2][2] = (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]);

    float invDet = 1.0f / det;

    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            inv[i][j] = adj[i][j] * invDet;
        }
    }

    return inv;
}

std::array<std::array<float, 2>, 4> getOuterRotatedRect(const std::vector<std::array<float, 2>>& points) {
    auto hull = getHull(points);
    float minArea = std::numeric_limits<float>::max();
    std::array<std::array<float, 2>, 4> minAreaPoints;

    for(size_t i = 1; i < hull.size(); ++i) {
        std::array<float, 2> vec = {hull[i][0] - hull[i - 1][0], hull[i][1] - hull[i - 1][1]};
        std::array<float, 2> vecOrth = {-vec[1], vec[0]};
        std::array<std::array<float, 2>, 2> mat = {{{vec[0], vecOrth[0]}, {vec[1], vecOrth[1]}}};
        std::array<std::array<float, 2>, 2> matInv = getInverse(mat);

        std::vector<std::array<float, 2>> rotatedHull;
        for(const auto& pt : hull) {
            float newX = matInv[0][0] * pt[0] + matInv[0][1] * pt[1];
            float newY = matInv[1][0] * pt[0] + matInv[1][1] * pt[1];
            rotatedHull.push_back({newX, newY});
        }

        const auto [minx, maxx, miny, maxy] = getOuterRect(rotatedHull);
        float area = (maxx - minx) * (maxy - miny);

        if(area < minArea) {
            minArea = area;
            std::array<std::array<float, 2>, 4> rectPoints = {{{minx, miny}, {maxx, miny}, {maxx, maxy}, {minx, maxy}}};
            for(auto i = 0U; i < rectPoints.size(); ++i) {
                auto& pt = rectPoints[i];
                float origX = mat[0][0] * pt[0] + mat[0][1] * pt[1];
                float origY = mat[1][0] * pt[0] + mat[1][1] * pt[1];
                minAreaPoints[i] = {origX, origY};
            }
        }
    }

    return minAreaPoints;
}

std::array<std::array<float, 3>, 3> getResizeMat(Resize o, float width, float height, uint32_t outputWidth, uint32_t outputHeight) {
    if(o.mode == Resize::VALUE) {
        if(o.width > 0 && o.height <= 0) {
            o.height = o.normalized ? o.width : height * (o.width / width);
        } else if(o.width <= 0 && o.height > 0) {
            o.width = o.normalized ? o.height : width * (o.height / height);
        }
    } else if(o.mode == Resize::FIT || o.mode == Resize::FILL) {
        if(outputWidth > 0 && outputHeight > 0) {
            float ratio = width / height;
            if((float)outputWidth / (float)outputHeight > ratio) {
                if(o.mode == Resize::FIT) {
                    o.width = outputHeight * ratio;
                    o.height = outputHeight;
                } else {
                    o.width = outputWidth;
                    o.height = outputWidth / ratio;
                }
            } else {
                if(o.mode == Resize::FIT) {
                    o.width = outputWidth;
                    o.height = outputWidth / ratio;
                } else {
                    o.width = outputHeight * ratio;
                    o.height = outputHeight;
                }
            }
        } else {
            throw std::runtime_error("Neither output size nor resize dimensions are set");
        }
    }
    if(!o.normalized) {
        o.width /= width;
        o.height /= height;
    }
    return {{{o.width, 0, 0}, {0, o.height, 0}, {0, 0, 1}}};
}

std::tuple<std::array<std::array<float, 3>, 3>, std::array<std::array<float, 2>, 4>, std::vector<std::array<std::array<float, 2>, 4>>> getTransform(
    const std::vector<ManipOp>& ops, uint32_t inputWidth, uint32_t inputHeight, uint32_t outputWidth, uint32_t outputHeight) {
    std::array<std::array<float, 3>, 3> transform{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    std::array<std::array<float, 2>, 4> imageCorners{{{0, 0}, {(float)inputWidth, 0}, {(float)inputWidth, (float)inputHeight}, {0, (float)inputHeight}}};
    std::vector<std::array<std::array<float, 2>, 4>> srcCorners;
    for(const auto& op : ops) {
        std::array<std::array<float, 3>, 3> mat = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

        float centerX = (imageCorners[0][0] + imageCorners[1][0] + imageCorners[2][0] + imageCorners[3][0]) / 4;
        float centerY = (imageCorners[0][1] + imageCorners[1][1] + imageCorners[2][1] + imageCorners[3][1]) / 4;

        const auto [_minx, _maxx, _miny, _maxy] = getOuterRect(std::vector(imageCorners.begin(), imageCorners.end()));
        float minx = _minx;
        float maxx = _maxx;
        float miny = _miny;
        float maxy = _maxy;
        float width = maxx - minx;
        float height = maxy - miny;

        std::visit(
            overloaded{[](auto _) {},
                       [&](Translate o) {
                           if(o.normalized) {
                               o.offsetX *= width;
                               o.offsetY *= height;
                           }
                           mat = {{{1, 0, o.offsetX}, {0, 1, o.offsetY}, {0, 0, 1}}};
                       },
                       [&](Rotate o) {
                           float cos = std::cos(o.angle);
                           float sin = std::sin(o.angle);
                           if(o.normalized) {
                               o.offsetX *= width;
                               o.offsetY *= height;
                           }
                           float moveX = centerX + o.offsetX;
                           float moveY = centerY + o.offsetY;
                           if(o.center) {
                               mat = {{{1, 0, -moveX}, {0, 1, -moveY}, {0, 0, 1}}};
                           }
                           mat = matmul({{{cos, -sin, 0}, {sin, cos, 0}, {0, 0, 1}}}, mat);
                           if(o.center) {
                               mat = matmul({{{1, 0, moveX}, {0, 1, moveY}, {0, 0, 1}}}, mat);
                           }
                       },
                       [&](Resize o) { mat = getResizeMat(o, width, height, outputWidth, outputHeight); },
                       [&](Flip o) {
                           float moveX = centerX;
                           float moveY = centerY;
                           switch(o.direction) {
                               case Flip::HORIZONTAL: {
                                   if(o.center) {
                                       mat = {{{1, 0, -moveX}, {0, 1, -moveY}, {0, 0, 1}}};
                                   }
                                   mat = matmul({{{-1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}, mat);
                                   if(o.center) {
                                       mat = matmul({{{1, 0, moveX}, {0, 1, moveY}, {0, 0, 1}}}, mat);
                                   }
                                   break;
                               }
                               case Flip::VERTICAL: {
                                   if(o.center) {
                                       mat = {{{1, 0, -moveX}, {0, 1, -moveY}, {0, 0, 1}}};
                                   }
                                   mat = matmul({{{1, 0, 0}, {0, -1, 0}, {0, 0, 1}}}, mat);
                                   if(o.center) {
                                       mat = matmul({{{1, 0, moveX}, {0, 1, moveY}, {0, 0, 1}}}, mat);
                                   }
                                   break;
                               }
                           }
                       },
                       [&](FourPoints o) {
                           if(o.normalized) {
                               for(auto i = 0; i < 4; ++i) {
                                   o.src[i].x *= width;
                                   o.src[i].y *= height;
                                   o.dst[i].x *= width;
                                   o.dst[i].y *= height;
                               }
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
                               std::array<float, 9> coeff = {};
                               std::array<float, 8> srcData = {o.src[0].x, o.src[0].y, o.src[1].x, o.src[1].y, o.src[2].x, o.src[2].y, o.src[3].x, o.src[3].y};
                               std::array<float, 8> dstData = {o.dst[0].x, o.dst[0].y, o.dst[1].x, o.dst[1].y, o.dst[2].x, o.dst[2].y, o.dst[3].x, o.dst[3].y};
                               fcvGetPerspectiveTransformf32(srcData.data(), dstData.data(), coeff.data());
                               mat = {{{coeff[0], coeff[1], coeff[2]}, {coeff[3], coeff[4], coeff[5]}, {coeff[6], coeff[7], coeff[8]}}};
#elif defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
                               cv::Point2f srcPoints[4] = {cv::Point2f(o.src[0].x, o.src[0].y),
                                                           cv::Point2f(o.src[1].x, o.src[1].y),
                                                           cv::Point2f(o.src[2].x, o.src[2].y),
                                                           cv::Point2f(o.src[3].x, o.src[3].y)};
                               cv::Point2f dstPoints[4] = {cv::Point2f(o.dst[0].x, o.dst[0].y),
                                                           cv::Point2f(o.dst[1].x, o.dst[1].y),
                                                           cv::Point2f(o.dst[2].x, o.dst[2].y),
                                                           cv::Point2f(o.dst[3].x, o.dst[3].y)};
                               cv::Mat cvMat = cv::getPerspectiveTransform(srcPoints, dstPoints);
                               mat = {{{(float)cvMat.at<double>(0, 0), (float)cvMat.at<double>(0, 1), (float)cvMat.at<double>(0, 2)},
                                       {(float)cvMat.at<double>(1, 0), (float)cvMat.at<double>(1, 1), (float)cvMat.at<double>(1, 2)},
                                       {(float)cvMat.at<double>(2, 0), (float)cvMat.at<double>(2, 1), (float)cvMat.at<double>(2, 2)}}};
#else
                               throw std::runtime_error("FourPoints not supported without OpenCV or FastCV enabled");
#endif
                           }
                       },
                       [&](Affine o) { mat = {{{o.matrix[0], o.matrix[1], 0}, {o.matrix[2], o.matrix[3], 0}, {0, 0, 1}}}; },
                       [&](Perspective o) {
                           mat = {{{o.matrix[0], o.matrix[1], o.matrix[2]}, {o.matrix[3], o.matrix[4], o.matrix[5]}, {o.matrix[6], o.matrix[7], o.matrix[8]}}};
                       },
                       [&](Crop o) {
                           if(o.normalized) {
                               o.width *= width;
                               o.height *= height;
                           }
                           if(o.width > 0 && o.height <= 0)
                               o.height = roundf(height * ((float)o.width / width));
                           else if(o.height > 0 && o.width <= 0)
                               o.width = roundf(width * (o.height / height));
                           else if(o.height <= 0 && o.width <= 0) {
                               o.width = roundf(maxx);
                               o.height = roundf(maxy);
                           }

                           outputWidth = o.width;
                           outputHeight = o.height;

                           if(o.center) {
                               std::array<std::array<float, 3>, 3> _mat = {
                                   {{1, 0, -minx + (outputWidth - (maxx - minx)) / 2}, {0, 1, -miny + (outputHeight - (maxy - miny)) / 2}, {0, 0, 1}}};
                               transform = matmul(_mat, transform);
                           }

                           imageCorners = {{{0, 0}, {(float)outputWidth, 0}, {(float)outputWidth, (float)outputHeight}, {0, (float)outputHeight}}};
                           auto transformInv = getInverse(transform);
                           srcCorners.push_back({matvecmul(transformInv, imageCorners[0]),
                                                 matvecmul(transformInv, imageCorners[1]),
                                                 matvecmul(transformInv, imageCorners[2]),
                                                 matvecmul(transformInv, imageCorners[3])});
                       }},
            op.op);
        /*printf("Mat: %f %f %f %f %f %f %f %f %f\n", mat[0][0], mat[0][1], mat[0][2], mat[1][0], mat[1][1], mat[1][2], mat[2][0], mat[2][1], mat[2][2]);*/
        imageCorners = getOuterRotatedRect(
            {matvecmul(mat, imageCorners[0]), matvecmul(mat, imageCorners[1]), matvecmul(mat, imageCorners[2]), matvecmul(mat, imageCorners[3])});
        transform = matmul(mat, transform);
    }
    /*printf("Transform: %f %f %f %f %f %f %f %f %f\n", transform[0][0], transform[0][1], transform[0][2], transform[1][0], transform[1][1], transform[1][2],
     * transform[2][0], transform[2][1], transform[2][2]);*/
    return {transform, imageCorners, srcCorners};
}

dai::ImgFrame::Type getValidType(dai::ImgFrame::Type type) {
    return isSingleChannelu8(type) ? VALID_TYPE_GRAY : VALID_TYPE_COLOR;
}

void dai::impl::ImageManipOperations::init() {
#ifdef DEPTHAI_HAVE_FASTCV_SUPPORT
    #ifndef FORCE_FASTCV_ON_SPECIFIC_CORE
    const fcvOperationMode operMode = FASTCV_OP_PERFORMANCE;  // FASTCV_OP_CPU_OFFLOAD;
    // DON'T CALL fcvSetOperationMode(operMode) BEFORE zdl::SNPE::SNPEBuilder::build() !!!
    // else zdl::SNPE::SNPEBuilder::build() will SIGSEGV !!!
    int setOperMode = fcvSetOperationMode(operMode);
    FCV_IMG_DEBUG("Set operation mode to: %d with result: %d\n", operMode, setOperMode);
    #else
    ForceExecutionOnSpecificCore();
    #endif
#endif
}

ImageManipOperations& ImageManipOperations::build(const ImageManipOpsBase& newBase,
                                                  ImgFrame::Type outType,
                                                  FrameSpecs srcFrameSpecs,
                                                  ImgFrame::Type inFrameType) {
    auto newCfgStr = getConfigString(newBase);
    if(newCfgStr == prevConfig && outType == outputFrameType && srcFrameSpecs.width == srcSpecs.width && srcFrameSpecs.height == srcSpecs.height
       && inFrameType == inType)
        return *this;
    prevConfig = newCfgStr;

    if(newBase.hasWarp(srcFrameSpecs.width, srcFrameSpecs.height)) mode = mode | MODE_WARP;
    if(newBase.colormap != Colormap::NONE && isSingleChannelu8(inFrameType)) mode = mode | MODE_COLORMAP;
    if(outType != ImgFrame::Type::NONE && outType != inFrameType) mode = mode | MODE_CONVERT;

    assert(inFrameType != ImgFrame::Type::NONE);
    base = newBase;
    outputFrameType = outType == ImgFrame::Type::NONE ? inFrameType : outType;
    inType = inFrameType;
    type = inType;
    srcSpecs = srcFrameSpecs;

    if(!isSingleChannelu8(inType) && base.colormap != Colormap::NONE) {
        if(logger) logger->warn("ImageManip | Colormap can only be applied to single channel images, ignoring colormap");
        base.colormap = Colormap::NONE;
    }

    if(mode == 0) {
        return *this;
    } else if(mode == MODE_CONVERT) {
        auto ccDstSpecs = getCcDstFrameSpecs(srcSpecs, inType, outputFrameType);
        preprocCc.build(srcSpecs, ccDstSpecs, inType, outputFrameType);
    } else {
#if defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV || defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
        if(!isTypeSupportedL(inType)) {
#else
        if(!isTypeSupportedC(inType)) {
#endif
            auto color = getValidType(inType);
            auto ccDstSpecs = getCcDstFrameSpecs(srcSpecs, inType, color);
            preprocCc.build(srcSpecs, ccDstSpecs, inType, color);
            srcSpecs = ccDstSpecs;
            type = color;
            convertInput = true;
            if(outputFrameType != color)
                mode = mode & ~MODE_CONVERT;
            else
                mode = mode | MODE_CONVERT;
        }
    }

    const uint32_t inputWidth = srcSpecs.width;
    const uint32_t inputHeight = srcSpecs.height;
    assert(inputWidth > 0 && inputHeight > 0);
    matrix = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

    if(mode & MODE_WARP) {
        auto operations = base.getOperations();

        auto [transformMat, imageCorners, srcCorners] = getTransform(operations, inputWidth, inputHeight, base.outputWidth, base.outputHeight);
        matrix = transformMat;

        {
            auto [minx, maxx, miny, maxy] = getOuterRect(std::vector(imageCorners.begin(), imageCorners.end()));
            if(base.outputWidth == 0) base.outputWidth = maxx;
            if(base.outputHeight == 0) base.outputHeight = maxy;
        }

        if(base.resizeMode != ImageManipOpsBase::ResizeMode::NONE) {
            Resize res;
            switch(base.resizeMode) {
                case ImageManipOpsBase::ResizeMode::NONE:
                    break;
                case ImageManipOpsBase::ResizeMode::STRETCH:
                    res = Resize(base.outputWidth, base.outputHeight);
                    break;
                case ImageManipOpsBase::ResizeMode::LETTERBOX:
                    res = Resize::fit();
                    break;
                case ImageManipOpsBase::ResizeMode::CENTER_CROP:
                    res = Resize::fill();
                    break;
            }
            auto [minx, maxx, miny, maxy] = getOuterRect(std::vector(imageCorners.begin(), imageCorners.end()));
            auto mat = getResizeMat(res, maxx - minx, maxy - miny, base.outputWidth, base.outputHeight);
            imageCorners = {
                {{matvecmul(mat, imageCorners[0])}, {matvecmul(mat, imageCorners[1])}, {matvecmul(mat, imageCorners[2])}, {matvecmul(mat, imageCorners[2])}}};
            matrix = matmul(mat, matrix);
        }

        if(base.center) {
            float width = base.outputWidth;
            float height = base.outputHeight;
            auto [minx, maxx, miny, maxy] = getOuterRect(std::vector(imageCorners.begin(), imageCorners.end()));
            std::array<std::array<float, 3>, 3> mat = {{{1, 0, -minx + (width - (maxx - minx)) / 2}, {0, 1, -miny + (height - (maxy - miny)) / 2}, {0, 0, 1}}};
            imageCorners = {
                {{matvecmul(mat, imageCorners[0])}, {matvecmul(mat, imageCorners[1])}, {matvecmul(mat, imageCorners[2])}, {matvecmul(mat, imageCorners[2])}}};
            matrix = matmul(mat, matrix);
        }

        matrixInv = getInverse(matrix);
        srcCorners.push_back({matvecmul(matrixInv, {0, 0}),
                              matvecmul(matrixInv, {(float)base.outputWidth, 0}),
                              matvecmul(matrixInv, {(float)base.outputWidth, (float)base.outputHeight}),
                              matvecmul(matrixInv, {0, (float)base.outputHeight})});

        if(logger) {
            logger->trace("Image corners: ");
            logger->trace("|{} {}|{} {}|", imageCorners[0][0], imageCorners[0][1], imageCorners[1][0], imageCorners[1][1]);
            logger->trace("-------------");
            logger->trace("|{} {}|{} {}|", imageCorners[3][0], imageCorners[3][1], imageCorners[2][0], imageCorners[2][1]);
            logger->trace("Transformation matrix: ");
            logger->trace("|{} {} {}|", matrix[0][0], matrix[0][1], matrix[0][2]);
            logger->trace("-------------");
            logger->trace("|{} {} {}|", matrix[1][0], matrix[1][1], matrix[1][2]);
            logger->trace("-------------");
            logger->trace("|{} {} {}|", matrix[2][0], matrix[2][1], matrix[2][2]);
            logger->trace("Transformation matrix inverse: ");
            logger->trace("|{} {} {}|", matrixInv[0][0], matrixInv[0][1], matrixInv[0][2]);
            logger->trace("-------------");
            logger->trace("|{} {} {}|", matrixInv[1][0], matrixInv[1][1], matrixInv[1][2]);
            logger->trace("-------------");
            logger->trace("|{} {} {}|", matrixInv[2][0], matrixInv[2][1], matrixInv[2][2]);
        }

        warpEngine.build(srcSpecs, getOutputFrameSpecs(type), type, matrix, srcCorners);
        clrChange.build(getOutputFrameSpecs(type), getOutputFrameSpecs(outputFrameType), type, outputFrameType);
    } else {
        base.outputWidth = inputWidth;
        base.outputHeight = inputHeight;
    }
    float bppPre, bppPost;
    int numPlanesPre, numPlanesPost;
    getFrameTypeInfo(getValidType(type), numPlanesPre, bppPre);
    getFrameTypeInfo(isSingleChannelu8(type) && base.colormap != Colormap::NONE ? VALID_TYPE_COLOR : type, numPlanesPost, bppPost);
    size_t newConvertedSize = ALIGN_UP(ALIGN_UP((size_t)((float)inputWidth * bppPre), 8) * inputHeight, PLANE_ALIGNMENT) * numPlanesPre;
    size_t newColormapSize = ALIGN_UP(ALIGN_UP((size_t)((float)base.outputWidth), 8) * base.outputHeight, PLANE_ALIGNMENT);
    size_t newWarpedSize = ALIGN_UP(ALIGN_UP((size_t)((float)base.outputWidth * bppPost), 8) * base.outputHeight, PLANE_ALIGNMENT) * numPlanesPost;

    if(!convertedFrame || convertedFrame->size() < newConvertedSize) convertedFrame = std::make_shared<ImageManipMemory>(newConvertedSize);
    if(!colormapFrame || colormapFrame->size() < newColormapSize) colormapFrame = std::make_shared<ImageManipMemory>(newColormapSize);
    if(!warpedFrame || warpedFrame->size() < newWarpedSize) warpedFrame = std::make_shared<ImageManipMemory>(newWarpedSize);

    return *this;
}

bool ImageManipOperations::apply(const std::shared_ptr<ImgFrame> src, span<uint8_t> dst) {
    if(src->getType() != inType || src->getWidth() != srcSpecs.width || src->getHeight() != srcSpecs.height)
        throw std::runtime_error("ImageManip not built for the source image specs. Consider rebuilding with the new configuration.");
    if(mode == 0) {
        std::copy(src->getData().begin(), src->getData().end(), dst.begin());
        return true;
    }

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    if(convertInput || mode == MODE_CONVERT) preprocCc.apply(src->getData(), mode == MODE_CONVERT ? dst : convertedFrame->getData());
    if(mode != MODE_CONVERT) {
        if(mode & MODE_WARP) {
            warpEngine.apply(convertInput ? convertedFrame->getData() : src->getData(),
                             base.colormap != Colormap::NONE ? colormapFrame->getData() : (type == outputFrameType ? dst : warpedFrame->getData()));
        }
        if(mode & MODE_COLORMAP) {
            uint8_t* colormapDst = outputFrameType == VALID_TYPE_COLOR ? dst.data() : warpedFrame->data();
            cv::Mat gray(base.outputWidth,
                         base.outputHeight,
                         CV_8UC1,
                         mode & MODE_WARP ? colormapFrame->data() : (convertInput ? convertedFrame->data() : src->getData().data()),
                         ALIGN_UP(base.outputWidth, 8));
            cv::Mat color(base.outputWidth, base.outputHeight, CV_8UC3, colormapDst);
            cv::applyColorMap(gray, color, cv::COLORMAP_JET);  // TODO(asahtik) set colormap from `Colormap`
        }
        // Change color(format) if outputFrameType is not None / the same as the current frame type and not (frame type is single channel + colormap is
        // applied
        // + output frame type is RGBi)
        if(type != outputFrameType && !(isSingleChannelu8(type) && base.colormap != Colormap::NONE && outputFrameType == VALID_TYPE_COLOR)) {
            clrChange.apply(warpedFrame->getData(), dst);
        }
    }
    return true;  // TODO(asahtik): Handle failed transformation
#else
    return false;
#endif
}

size_t ImageManipOperations::getOutputWidth() const {
    return base.outputWidth;
}

size_t ImageManipOperations::getOutputHeight() const {
    return base.outputHeight;
}

size_t ImageManipOperations::getOutputStride(uint8_t plane) const {
    if(mode == 0) return plane == 0 ? srcSpecs.p1Stride : (plane == 1 ? srcSpecs.p2Stride : (plane == 2 ? srcSpecs.p3Stride : 0));
    switch(outputFrameType) {
        case ImgFrame::Type::RGB888p:
        case ImgFrame::Type::BGR888p:
            return plane < 3 ? ALIGN_UP(base.outputWidth, 8) : 0;
        case ImgFrame::Type::RGB888i:
        case ImgFrame::Type::BGR888i:
            return plane == 0 ? ALIGN_UP(base.outputWidth * 3, 8) : 0;
        case ImgFrame::Type::NV12:
            return plane < 2 ? ALIGN_UP(base.outputWidth, 8) : 0;
        case ImgFrame::Type::YUV420p:
            return plane == 0 ? ALIGN_UP(base.outputWidth, 8) : (plane < 3 ? base.outputWidth / 2 : 0);
        case ImgFrame::Type::GRAY8:
        case ImgFrame::Type::RAW8:
            return plane == 0 ? ALIGN_UP(base.outputWidth, 8) : 0;
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
            return 0;
    }
    return 0;
}

size_t ImageManipOperations::getOutputSize() const {
    if(mode == 0) return 0;
    size_t size = 0;
    switch(outputFrameType) {
        case ImgFrame::Type::RGB888p:
        case ImgFrame::Type::BGR888p:
            size = ALIGN_UP(getOutputStride() * getOutputHeight(), PLANE_ALIGNMENT) * 3;
            break;
        case ImgFrame::Type::RGB888i:
        case ImgFrame::Type::BGR888i:
        case ImgFrame::Type::GRAY8:
        case ImgFrame::Type::RAW8:
            size = getOutputStride() * getOutputHeight();
            break;
        case ImgFrame::Type::NV12:
            size = ALIGN_UP(getOutputStride(0) * getOutputHeight(), PLANE_ALIGNMENT) + ALIGN_UP(getOutputStride(1) * getOutputHeight() / 2, PLANE_ALIGNMENT);
            break;
        case ImgFrame::Type::YUV420p:
            size =
                ALIGN_UP(getOutputStride(0) * getOutputHeight(), PLANE_ALIGNMENT) + ALIGN_UP(getOutputStride(1) * getOutputHeight() / 2, PLANE_ALIGNMENT) * 2;
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
            throw std::runtime_error("Output frame type not supported");
    }
    if(size == 0) throw std::runtime_error("Output size is 0");
    return size;
}

FrameSpecs ImageManipOperations::getOutputFrameSpecs(ImgFrame::Type type) const {
    if(mode == 0) return srcSpecs;
    FrameSpecs specs;
    specs.width = base.outputWidth;
    specs.height = base.outputHeight;
    specs.p1Offset = 0;
    switch(type) {
        case dai::ImgFrame::Type::RGB888p:
        case dai::ImgFrame::Type::BGR888p:
            specs.p1Stride = ALIGN_UP(specs.width, 8);
            specs.p2Stride = specs.p1Stride;
            specs.p3Stride = specs.p1Stride;
            specs.p2Offset = specs.p1Offset + ALIGN_UP(specs.p1Stride * specs.height, PLANE_ALIGNMENT);
            specs.p3Offset = specs.p2Offset + ALIGN_UP(specs.p1Stride * specs.height, PLANE_ALIGNMENT);
            break;
        case dai::ImgFrame::Type::RGB888i:
        case dai::ImgFrame::Type::BGR888i:
            specs.p1Stride = ALIGN_UP(specs.width * 3, 8);
            specs.p2Stride = specs.p1Stride;
            specs.p3Stride = specs.p1Stride;
            specs.p2Offset = specs.p1Offset;
            specs.p3Offset = specs.p1Offset;
            break;
        case dai::ImgFrame::Type::NV12:
            specs.p1Stride = ALIGN_UP(specs.width, 8);
            specs.p2Stride = specs.p1Stride;
            specs.p2Offset = specs.p1Offset + ALIGN_UP(specs.p1Stride * specs.height, PLANE_ALIGNMENT);
            specs.p3Offset = specs.p2Offset;
            specs.p3Stride = 0;
            break;
        case dai::ImgFrame::Type::YUV420p:
            specs.p1Stride = ALIGN_UP(specs.width, 8);
            specs.p2Stride = ALIGN_UP(specs.width / 2, 8);
            specs.p3Stride = ALIGN_UP(specs.width / 2, 8);
            specs.p2Offset = specs.p1Offset + ALIGN_UP(specs.p1Stride * specs.height, PLANE_ALIGNMENT);
            specs.p3Offset = specs.p2Offset + ALIGN_UP(specs.p2Stride * (specs.height / 2), PLANE_ALIGNMENT);
            break;
        case dai::ImgFrame::Type::RAW8:
        case dai::ImgFrame::Type::GRAY8:
            specs.p1Stride = ALIGN_UP(specs.width, 8);
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
            throw std::runtime_error("Frame type not supported");
            break;
    }
    return specs;
}

typedef enum AEEResult { AEE_SUCCESS, AEE_EBADPARM, AEE_ERROR } AEEResult;

AEEResult DaiDsp_manipGetSrcMask(
    const uint32_t width, const uint32_t height, const float* corners, const uint32_t cornersLen, bool init0, uint8_t* mask, const uint32_t maskLen) {
    if(maskLen < height * width) return AEE_EBADPARM;
    const uint32_t numBoxes = cornersLen / (4 * 2);
    const float p11x = corners[0];
    const float p11y = corners[1];
    const float p12x = corners[2];
    const float p12y = corners[3];
    /*const float p13x = corners[4];*/
    /*const float p13y = corners[5];*/
    const float p14x = corners[6];
    const float p14y = corners[7];

    float minx = 0;
    float maxx = 0;
    float miny = 0;
    float maxy = 0;

    {
        const uint32_t iminx = floorf(minx);
        const uint32_t imaxx = ceilf(maxx);
        const uint32_t iminy = floorf(miny);
        const uint32_t imaxy = ceilf(maxy);

        if(init0) {
            memset(mask, 0, height * width * sizeof(uint8_t));
        }
        for(uint32_t i = iminy; i < imaxy; ++i) {
            const uint32_t lineStart = i * width;
#pragma clang loop vectorize(enable) interleave(enable) unroll_count(8) distribute(enable)
            for(uint32_t j = lineStart + iminx; j < lineStart + imaxx; ++j) {
                const float x = j - lineStart;
                const float y = i;
                const float apx = x - p11x;
                const float apy = y - p11y;
                const float abx = p12x - p11x;
                const float aby = p12y - p11y;
                const float adx = p14x - p11x;
                const float ady = p14y - p11y;
                // (0 < AP . AB < AB . AB) and (0 < AP . AD < AD . AD)
                mask[j] = (bool)(0 <= apx * abx + apy * aby && apx * abx + apy * aby < abx * abx + aby * aby && 0 <= apx * adx + apy * ady
                                 && apx * adx + apy * ady < adx * adx + ady * ady);
            }
        }
    }
    for(uint32_t b = 1; b < numBoxes; ++b) {
        const float pi1x = corners[0];
        const float pi1y = corners[1];
        const float pi2x = corners[2];
        const float pi2y = corners[3];
        /*const float pi3x = corners[4];*/
        /*const float pi3y = corners[5];*/
        const float pi4x = corners[6];
        const float pi4y = corners[7];

        minx = 0;
        maxx = 0;
        miny = 0;
        maxy = 0;

        const uint32_t iminx = floorf(minx);
        const uint32_t imaxx = ceilf(maxx);
        const uint32_t iminy = floorf(miny);
        const uint32_t imaxy = ceilf(maxy);

        for(uint32_t i = iminy; i < imaxy; ++i) {
            const uint32_t lineStart = i * width;
#pragma clang loop vectorize(enable) interleave(enable) unroll_count(8) distribute(enable)
            for(uint32_t j = lineStart + iminx; j < lineStart + imaxx; ++j) {
                const float x = j - lineStart;
                const float y = i;
                const float apx = x - pi1x;
                const float apy = y - pi1y;
                const float abx = pi2x - pi1x;
                const float aby = pi2y - pi1y;
                const float adx = pi4x - pi1x;
                const float ady = pi4y - pi1y;
                // (0 < AP . AB < AB . AB) and (0 < AP . AD < AD . AD)
                mask[j] &= (bool)(0 < apx * abx + apy * aby && apx * abx + apy * aby < abx * abx + aby * aby && 0 < apx * adx + apy * ady
                                  && apx * adx + apy * ady < adx * adx + ady * ady);
            }
        }
    }

    return AEE_SUCCESS;
}

AEEResult DaiDsp_manipGetRemap3x3(const uint32_t inWidth,
                                  const uint32_t inHeight,
                                  const uint32_t outWidth,
                                  const uint32_t outHeight,
                                  const float* matrix,
                                  const uint32_t matrixLen,
                                  const uint8_t* __restrict__ srcMask,
                                  const uint32_t srcMaskLen,
                                  const uint32_t minx,
                                  const uint32_t maxx,
                                  const uint32_t miny,
                                  const uint32_t maxy,
                                  float* __restrict__ mapX,
                                  const uint32_t mapXLen,
                                  float* __restrict__ mapY,
                                  const uint32_t mapYLen,
                                  uint8_t* __restrict__ dstMask,
                                  const uint32_t dstMaskLen) {
    if(mapXLen != inWidth * inHeight || mapYLen != inWidth * inHeight || matrixLen != 9 || srcMaskLen != inWidth * inHeight
       || dstMaskLen != outWidth * outHeight) {
        return AEE_EBADPARM;
    }
/**
 * 0 1 2   0
 * 3 4 5 x 1
 * 6 7 8   2
 */
#pragma clang loop vectorize(enable) interleave(enable) unroll_count(8) distribute(enable)
    for(uint32_t i = 0; i < outWidth * outHeight; ++i) {
        uint32_t x = i % outWidth;
        uint32_t y = i / outWidth;

        float tX = matrix[0] * x + matrix[1] * y + matrix[2];
        float tY = matrix[3] * x + matrix[4] * y + matrix[5];
        float q = matrix[6] * x + matrix[7] * y + matrix[8];

        mapX[i] = tX / q;
        mapY[i] = tY / q;

        // Remap if outside of the mask
        mapX[i] = mapX[i] < minx ? minx : mapX[i];
        mapX[i] = mapX[i] >= maxx ? maxx - 1 : mapX[i];
        mapY[i] = mapY[i] < miny ? miny : mapY[i];
        mapY[i] = mapY[i] >= maxy ? maxy - 1 : mapY[i];

        const uint32_t idx = (uint32_t)roundf(mapY[i]) * inWidth + (uint32_t)roundf(mapX[i]);
        dstMask[i] = 0x1 & srcMask[idx];
    }
    return AEE_SUCCESS;
}

AEEResult DaiDsp_subsampleMap2x2(const uint32_t width,
                                 const uint32_t height,
                                 const float* __restrict__ mapX,
                                 const uint32_t mapXLen,
                                 const float* __restrict__ mapY,
                                 const uint32_t mapYLen,
                                 const uint8_t* __restrict__ dstMask,
                                 const uint32_t dstMaskLen,
                                 float* __restrict__ mapXss,
                                 const uint32_t mapXssLen,
                                 float* __restrict__ mapYss,
                                 const uint32_t mapYssLen,
                                 uint8_t* __restrict__ dstMaskss,
                                 const uint32_t dstMaskssLen) {
    if(mapXLen != width * height || mapYLen != width * height || dstMaskLen != width * height || mapXssLen != width * height / 4
       || mapYssLen != width * height / 4 || dstMaskssLen != width * height / 4 || width % 2 != 0 || height % 2 != 0) {
        return AEE_EBADPARM;
    }
#pragma clang loop vectorize(enable) interleave(enable) unroll_count(8) distribute(enable)
    for(uint32_t idx = 0; idx < width * height / 4; ++idx) {
        const uint32_t i = idx / (width / 2);
        const uint32_t j = idx % (width / 2);
        const uint32_t idx1 = 2 * i + 2 * j;
        const uint32_t idx2 = 2 * i + 2 * j + 1;
        const uint32_t idx3 = 2 * i + 1 + 2 * j;
        const uint32_t idx4 = 2 * i + 1 + 2 * j + 1;
        mapXss[idx] = (mapX[idx1] + mapX[idx2] + mapX[idx3] + mapX[idx4]) / (4.0f * 2.0f);
        mapYss[idx] = (mapY[idx1] + mapY[idx2] + mapY[idx3] + mapY[idx4]) / (4.0f * 2.0f);
        dstMaskss[idx] = dstMask[idx1] | dstMask[idx2] | dstMask[idx3] | dstMask[idx4];
    }
    return AEE_SUCCESS;
}

static inline float linInterp(const float x,
                              const float y,
                              const uint32_t channel,
                              const uint32_t width,
                              const uint32_t height,
                              const uint32_t stride,
                              const uint32_t numChannels,
                              const uint8_t* data) {
    const int x1 = floorf(x);
    const int y1 = floorf(y);
    const int x2 = clampi(x1 + 1, 0, width - 1);
    const int y2 = clampi(y1 + 1, 0, height - 1);
    const float fx = x - x1;
    const float fy = y - y1;
    const uint32_t q11i = y1 * stride + numChannels * x1;
    const uint32_t q21i = y1 * stride + numChannels * x2;
    const uint32_t q12i = y2 * stride + numChannels * x1;
    const uint32_t q22i = y2 * stride + numChannels * x2;

    const float r1 = data[q11i + channel] * (1 - fx) + data[q21i + channel] * fx;
    const float r2 = data[q12i + channel] * (1 - fx) + data[q22i + channel] * fx;
    const float p = r1 * (1 - fy) + r2 * fy;

    return roundf(clampf(p, 0.0f, 255.1f));
}

AEEResult DaiDsp_remapImage(const uint8_t* __restrict__ inData,
                            const uint32_t inDataLen,
                            const float* __restrict__ mapX,
                            const uint32_t mapXLen,
                            const float* __restrict__ mapY,
                            const uint32_t mapYLen,
                            const uint8_t* __restrict__ dstMask,
                            const uint32_t dstMaskLen,
                            const uint16_t numChannels,
                            const uint32_t inWidth,
                            const uint32_t inHeight,
                            const uint32_t inStride,
                            const uint32_t outWidth,
                            const uint32_t outHeight,
                            const uint32_t outStride,
                            uint8_t* __restrict__ outData,
                            const uint32_t outDataLen) {
    if(mapXLen != mapYLen || mapXLen != outWidth * outHeight || inDataLen != inHeight * inStride || outDataLen != outHeight * outStride
       || dstMaskLen != outWidth * outHeight || numChannels == 0 || numChannels > 3)
        return AEE_EBADPARM;

    for(uint32_t i = 0; i < outHeight; ++i) {
        const uint32_t lineStart = i * outStride;
        switch(numChannels) {
            case 1:
#pragma clang loop vectorize(enable) interleave(enable) unroll_count(8) distribute(enable)
                for(uint32_t j = 0; j < outWidth; ++j) {
                    const uint32_t idx = lineStart + j * numChannels;
                    const uint32_t mapIdx = i * outWidth + j;
                    const float x = clampf(mapX[mapIdx], 0, inWidth - 1);
                    const float y = clampf(mapY[mapIdx], 0, inHeight - 1);

                    const float l1 = linInterp(x, y, 0, inWidth, inHeight, inStride, numChannels, inData);
                    outData[idx + 0] = dstMask[mapIdx] * l1;
                }
                break;
            case 2:
#pragma clang loop vectorize(enable) interleave(enable) unroll_count(8) distribute(enable)
                for(uint32_t j = 0; j < outWidth; ++j) {
                    const uint32_t idx = lineStart + j * numChannels;
                    const uint32_t mapIdx = i * outWidth + j;
                    const float x = clampf(mapX[mapIdx], 0, inWidth - 1);
                    const float y = clampf(mapY[mapIdx], 0, inHeight - 1);

                    const float l1 = linInterp(x, y, 0, inWidth, inHeight, inStride, numChannels, inData);
                    const float l2 = linInterp(x, y, 1, inWidth, inHeight, inStride, numChannels, inData);
                    outData[idx + 0] = dstMask[mapIdx] * l1;
                    outData[idx + 1] = dstMask[mapIdx] * l2;
                }
                break;
            case 3:
#pragma clang loop vectorize(enable) interleave(enable) unroll_count(8) distribute(enable)
                for(uint32_t j = 0; j < outWidth; ++j) {
                    const uint32_t idx = lineStart + j * numChannels;
                    const uint32_t mapIdx = i * outWidth + j;
                    const float x = clampf(mapX[mapIdx], 0, inWidth - 1);
                    const float y = clampf(mapY[mapIdx], 0, inHeight - 1);

                    const float l1 = linInterp(x, y, 0, inWidth, inHeight, inStride, numChannels, inData);
                    const float l2 = linInterp(x, y, 1, inWidth, inHeight, inStride, numChannels, inData);
                    const float l3 = linInterp(x, y, 2, inWidth, inHeight, inStride, numChannels, inData);
                    outData[idx + 0] = dstMask[mapIdx] * l1;
                    outData[idx + 1] = dstMask[mapIdx] * l2;
                    outData[idx + 2] = dstMask[mapIdx] * l3;
                }
                break;
            default:
                return AEE_EBADPARM;
        }
    }
    return AEE_SUCCESS;
}

void WarpEngine::build(const FrameSpecs srcFrameSpecs,
                       const FrameSpecs dstFrameSpecs,
                       const ImgFrame::Type type,
                       const std::array<std::array<float, 3>, 3> matrix,
                       std::vector<std::array<std::array<float, 2>, 4>> srcCorners) {
    this->matrix = matrix;
    this->type = type;
    srcSpecs = srcFrameSpecs;
    dstSpecs = dstFrameSpecs;

    if(!fastCvBorder || fastCvBorder->size() < dstSpecs.height * 2) fastCvBorder = std::make_shared<ImageManipBuffer<uint32_t>>(dstSpecs.height * 2);
#if !USE_OPENCV && !USE_FASTCV || !defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && !defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
    const uint32_t inWidth = srcFrameSpecs.width;
    const uint32_t inHeight = srcFrameSpecs.height;
    const uint32_t outWidth = dstFrameSpecs.width;
    const uint32_t outHeight = dstFrameSpecs.height;
    const auto matrixInv = getInverse(matrix);
    if(!srcMask || srcMask->size() < inWidth * inHeight) srcMask = std::make_shared<ImageManipBuffer<uint8_t>>(inWidth * inHeight);
    if(!dstMask || dstMask->size() < outWidth * outHeight) dstMask = std::make_shared<ImageManipBuffer<uint8_t>>(outWidth * outHeight);
    if(!mapX || mapX->size() < outWidth * outHeight) mapX = std::make_shared<ImageManipBuffer<float>>(outWidth * outHeight);
    if(!mapY || mapY->size() < outWidth * outHeight) mapY = std::make_shared<ImageManipBuffer<float>>(outWidth * outHeight);
    if(type == ImgFrame::Type::YUV420p || type == ImgFrame::Type::NV12) {
        if(!srcMaskss || srcMaskss->size() < inWidth * inHeight) srcMaskss = std::make_shared<ImageManipBuffer<uint8_t>>(inWidth * inHeight);
        if(!dstMaskss || dstMaskss->size() < outWidth * outHeight) dstMaskss = std::make_shared<ImageManipBuffer<uint8_t>>(outWidth * outHeight);
        if(!mapXss || mapXss->size() < outWidth * outHeight) mapXss = std::make_shared<ImageManipBuffer<float>>(outWidth * outHeight);
        if(!mapYss || mapYss->size() < outWidth * outHeight) mapYss = std::make_shared<ImageManipBuffer<float>>(outWidth * outHeight);
    }

    std::vector<float> cornersArr;
    cornersArr.reserve(srcCorners.size() * 4 * 2);
    float minx = inWidth, maxx = 0, miny = inHeight, maxy = 0;
    for(const auto& corners : srcCorners)
        for(const auto& corner : corners) {
            cornersArr.push_back(corner[0]);
            cornersArr.push_back(corner[1]);
            minx = std::min(minx, corner[0]);
            maxx = std::max(maxx, corner[0]);
            miny = std::min(miny, corner[1]);
            maxy = std::max(maxy, corner[1]);
        }
    std::array matrixInv1D = {matrixInv[0][0],
                              matrixInv[0][1],
                              matrixInv[0][2],
                              matrixInv[1][0],
                              matrixInv[1][1],
                              matrixInv[1][2],
                              matrixInv[2][0],
                              matrixInv[2][1],
                              matrixInv[2][2]};

    // 1. Get src mask
    AEEResult err1 = DaiDsp_manipGetSrcMask(inWidth, inHeight, cornersArr.data(), cornersArr.size(), true, srcMask->data(), srcMask->size());
    if(err1 != AEE_SUCCESS) throw std::runtime_error("Failed to get src mask");
    // 2. Get transform + dst mask
    AEEResult err2 = DaiDsp_manipGetRemap3x3(inWidth,
                                             inHeight,
                                             outWidth,
                                             outHeight,
                                             matrixInv1D.data(),
                                             9,
                                             srcMask->data(),
                                             srcMask->size(),
                                             minx,
                                             maxx,
                                             miny,
                                             maxy,
                                             mapX->data(),
                                             mapX->size(),
                                             mapY->data(),
                                             mapY->size(),
                                             dstMask->data(),
                                             dstMask->size());
    if(err2 != AEE_SUCCESS) throw std::runtime_error("Failed to get remap map");
    if(type == ImgFrame::Type::YUV420p || type == ImgFrame::Type::NV12) {
        AEEResult err3 = DaiDsp_subsampleMap2x2(outWidth,
                                                outHeight,
                                                mapX->data(),
                                                mapX->size(),
                                                mapY->data(),
                                                mapY->size(),
                                                dstMask->data(),
                                                dstMask->size(),
                                                mapXss->data(),
                                                mapXss->size(),
                                                mapYss->data(),
                                                mapYss->size(),
                                                dstMaskss->data(),
                                                dstMaskss->size());
        if(err3 != AEE_SUCCESS) throw std::runtime_error("Failed to subsample map");
    }
#endif
}

void WarpEngine::apply(const span<const uint8_t> src, span<uint8_t> dst) {
    if(float_eq(matrix[2][0], 0) && float_eq(matrix[2][1], 0) && float_eq(matrix[2][2], 1)) {
        // Do affine
#if defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
        int numPlanes = isSingleChannelu8(type) ? 1 : 3;
        float affine[6] = {matrix[0][0], matrix[0][1], matrix[0][2], matrix[1][0], matrix[1][1], matrix[1][2]};
        const cv::Mat cvSrc(srcSpecs.height, srcSpecs.width, isSingleChannelu8(type) ? CV_8UC1 : CV_8UC3, const_cast<uint8_t*>(src.data()), srcSpecs.p1Stride);
        cv::Mat cvDst(dstSpecs.height, dstSpecs.width, numPlanes == 1 ? CV_8UC1 : CV_8UC3, dst.data(), dstSpecs.p1Stride);
        if(!(dst.size() >= dstSpecs.height * dstSpecs.width * 3)) throw std::runtime_error("Destination buffer too small");
        if(!(src.size() >= srcSpecs.height * srcSpecs.width * 3)) throw std::runtime_error("Source buffer too small");
        cv::Mat cvAffine(2, 3, CV_32F, affine);

        cv::warpAffine(cvSrc, cvDst, cvAffine, cv::Size(dstSpecs.width, dstSpecs.height));
#elif defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_FASTCV
        float affine[6] = {matrixInv[0][0], matrixInv[0][1], matrixInv[0][2], matrixInv[1][0], matrixInv[1][1], matrixInv[1][2]};
        if(!((ptrdiff_t)src.data() % 128 == 0 && (ptrdiff_t)dst.data() % 128 == 0 && (ptrdiff_t)fastCvBorder->data() % 128 == 0 && srcSpecs.p1Stride % 8 == 0
             && srcSpecs.p1Stride > 0)) {
            throw std::runtime_error("Assumptions not taken into account");
        }
        if(isSingleChannelu8(src)) {
            fcvTransformAffineClippedu8_v3(src.data(),
                                           srcSpecs.width,
                                           srcSpecs.height,
                                           srcSpecs.p1Stride,
                                           affine,
                                           dst.data(),
                                           dstSpecs.width,
                                           dstSpecs.height,
                                           dstSpecs.p1Stride,
                                           nullptr,
                                           FASTCV_INTERPOLATION_TYPE_BILINEAR,
                                           FASTCV_BORDER_CONSTANT,
                                           0);
        } else {
            fcv3ChannelTransformAffineClippedBCu8(src.data()),
                                                  srcSpecs.width,
                                                  srcSpecs.height,
                                                  srcSpecs.p1Stride,
                                                  affine,
                                                  dst.data(),
                                                  dstSpecs.width,
                                                  dstSpecs.height,
                                                  dstSpecs.p1Stride,
                                                  fastCvBorder->data());
        }
#endif
    } else {
        // Do perspective
#if defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && USE_OPENCV
        int numPlanes = isSingleChannelu8(type) ? 1 : 3;
        float projection[9] = {matrix[0][0], matrix[0][1], matrix[0][2], matrix[1][0], matrix[1][1], matrix[1][2], matrix[2][0], matrix[2][1], matrix[2][2]};
        cv::Mat cvSrc(srcSpecs.height, srcSpecs.width, isSingleChannelu8(type) ? CV_8UC1 : CV_8UC3, const_cast<uint8_t*>(src.data()), srcSpecs.p1Stride);
        cv::Mat cvDst(dstSpecs.height, dstSpecs.width, numPlanes == 1 ? CV_8UC1 : CV_8UC3, dst.data(), dstSpecs.p1Stride);
        assert(dst.size() >= dstSpecs.height * dstSpecs.p1Stride);
        cv::Mat cvProjection(3, 3, CV_32F, projection);
        cv::warpPerspective(cvSrc, cvDst, cvProjection, cv::Size(dstSpecs.width, dstSpecs.height));
#elif defined(DEPTHAI_HAVE_FASTCV_SUPPORT) && USE_OPENCV
        float projection[9] = {matrixInv[0][0],
                               matrixInv[0][1],
                               matrixInv[0][2],
                               matrixInv[1][0],
                               matrixInv[1][1],
                               matrixInv[1][2],
                               matrixInv[2][0],
                               matrixInv[2][1],
                               matrixInv[2][2]};
        if(!((ptrdiff_t)src.data() % 128 == 0 && (ptrdiff_t)dst.data() % 128 == 0 && (ptrdiff_t)fastCvBorder->data() % 128 == 0 && srcSpecs.p1Stride % 8 == 0
             && srcSpecs.p1Stride > 0)) {
            throw std::runtime_error("Assumptions not taken into account");
        }
        fcvStatus status = fcvStatus::FASTCV_SUCCESS;
        if(isSingleChannelu8(src))
            status = fcvWarpPerspectiveu8_v4(src.data(),
                                             srcSpecs.width,
                                             srcSpecs.height,
                                             srcSpecs.p1Stride,
                                             dst.data(),
                                             dstSpecs.width,
                                             dstSpecs.height,
                                             dstSpecs.p1Stride,
                                             projection,
                                             FASTCV_INTERPOLATION_TYPE_BILINEAR,
                                             FASTCV_BORDER_CONSTANT,
                                             0);
        else
            fcv3ChannelWarpPerspectiveu8_v2(
                src.data(), srcSpecs.width, srcSpecs.height, srcSpecs.p1Stride, dst.data(), dstSpecs.width, dstSpecs.height, dstSpecs.p1Stride, projection);
        if(status != fcvStatus::FASTCV_SUCCESS) {
            if(logger) logger->error("FastCV operation failed with error code {}", status);
            return false;
        }
#endif
    }
#if !USE_OPENCV && !USE_FASTCV || !defined(DEPTHAI_HAVE_OPENCV_SUPPORT) && !defined(DEPTHAI_HAVE_FASTCV_SUPPORT)
    assert(mapX && mapY && dstMask && srcMask);
    // Apply transformation multiple times depending on the image format
    switch(type) {
        case ImgFrame::Type::RGB888i:
        case ImgFrame::Type::BGR888i:
            assert(dstSpecs.width * dstSpecs.height * 3 >= dst.size());
            DaiDsp_remapImage(src.data() + srcSpecs.p1Offset,
                              src.size() - srcSpecs.p1Offset,
                              mapX->data(),
                              mapX->size(),
                              mapY->data(),
                              mapY->size(),
                              dstMask->data(),
                              dstMask->size(),
                              3,
                              srcSpecs.width,
                              srcSpecs.height,
                              srcSpecs.p1Stride,
                              dstSpecs.width,
                              dstSpecs.height,
                              dstSpecs.p1Stride,
                              dst.data() + dstSpecs.p1Offset,
                              dst.size() - dstSpecs.p1Offset);
            break;
        case ImgFrame::Type::RGB888p:
        case ImgFrame::Type::BGR888p:
            assert(dstSpecs.width * dstSpecs.height * 3 >= dst.size());
            DaiDsp_remapImage(src.data() + srcSpecs.p1Offset,
                              src.size() - srcSpecs.p1Offset,
                              mapX->data(),
                              mapX->size(),
                              mapY->data(),
                              mapY->size(),
                              dstMask->data(),
                              dstMask->size(),
                              1,
                              srcSpecs.width,
                              srcSpecs.height,
                              srcSpecs.p1Stride,
                              dstSpecs.width,
                              dstSpecs.height,
                              dstSpecs.p1Stride,
                              dst.data() + dstSpecs.p1Offset,
                              dst.size() - dstSpecs.p1Offset);
            DaiDsp_remapImage(src.data() + srcSpecs.p2Offset,
                              src.size() - srcSpecs.p2Offset,
                              mapX->data(),
                              mapX->size(),
                              mapY->data(),
                              mapY->size(),
                              dstMask->data(),
                              dstMask->size(),
                              1,
                              srcSpecs.width,
                              srcSpecs.height,
                              srcSpecs.p2Stride,
                              dstSpecs.width,
                              dstSpecs.height,
                              dstSpecs.p2Stride,
                              dst.data() + dstSpecs.p2Offset,
                              dst.size() - dstSpecs.p2Offset);
            DaiDsp_remapImage(src.data() + srcSpecs.p3Offset,
                              src.size() - srcSpecs.p3Offset,
                              mapX->data(),
                              mapX->size(),
                              mapY->data(),
                              mapY->size(),
                              dstMask->data(),
                              dstMask->size(),
                              1,
                              srcSpecs.width,
                              srcSpecs.height,
                              srcSpecs.p3Stride,
                              dstSpecs.width,
                              dstSpecs.height,
                              dstSpecs.p3Stride,
                              dst.data() + dstSpecs.p3Offset,
                              dst.size() - dstSpecs.p3Offset);
            break;
        case ImgFrame::Type::YUV420p:
            assert(dstSpecs.width * dstSpecs.height * 3 / 2 >= dst.size());
            assert(dstSpecs.width % 2 == 0 && dstSpecs.height % 2 == 0);
            DaiDsp_remapImage(src.data() + srcSpecs.p1Offset,
                              src.size() - srcSpecs.p1Offset,
                              mapX->data(),
                              mapX->size(),
                              mapY->data(),
                              mapY->size(),
                              dstMask->data(),
                              dstMask->size(),
                              1,
                              srcSpecs.width,
                              srcSpecs.height,
                              srcSpecs.p1Stride,
                              dstSpecs.width,
                              dstSpecs.height,
                              dstSpecs.p1Stride,
                              dst.data() + dstSpecs.p1Offset,
                              dst.size() - dstSpecs.p1Offset);
            DaiDsp_remapImage(src.data() + srcSpecs.p2Offset,
                              src.size() - srcSpecs.p2Offset,
                              mapXss->data(),
                              mapXss->size(),
                              mapYss->data(),
                              mapYss->size(),
                              dstMaskss->data(),
                              dstMaskss->size(),
                              1,
                              srcSpecs.width / 2,
                              srcSpecs.height / 2,
                              srcSpecs.p2Stride,
                              dstSpecs.width / 2,
                              dstSpecs.height / 2,
                              dstSpecs.p2Stride,
                              dst.data() + dstSpecs.p2Offset,
                              dst.size() - dstSpecs.p2Offset);
            DaiDsp_remapImage(src.data() + srcSpecs.p3Offset,
                              src.size() - srcSpecs.p3Offset,
                              mapXss->data(),
                              mapXss->size(),
                              mapYss->data(),
                              mapYss->size(),
                              dstMaskss->data(),
                              dstMaskss->size(),
                              1,
                              srcSpecs.width / 2,
                              srcSpecs.height / 2,
                              srcSpecs.p3Stride,
                              dstSpecs.width / 2,
                              dstSpecs.height / 2,
                              dstSpecs.p2Stride,
                              dst.data() + dstSpecs.p3Offset,
                              dst.size() - dstSpecs.p3Offset);
            break;
        case ImgFrame::Type::NV12:
            assert(dstSpecs.width * dstSpecs.height * 3 / 2 >= dst.size());
            assert(dstSpecs.width % 2 == 0 && dstSpecs.height % 2 == 0);
            DaiDsp_remapImage(src.data() + srcSpecs.p1Offset,
                              src.size() - srcSpecs.p1Offset,
                              mapX->data(),
                              mapX->size(),
                              mapY->data(),
                              mapY->size(),
                              dstMask->data(),
                              dstMask->size(),
                              1,
                              srcSpecs.width,
                              srcSpecs.height,
                              srcSpecs.p1Stride,
                              dstSpecs.width,
                              dstSpecs.height,
                              dstSpecs.p1Stride,
                              dst.data() + dstSpecs.p1Offset,
                              dst.size() - dstSpecs.p1Offset);
            DaiDsp_remapImage(src.data() + srcSpecs.p2Offset,
                              src.size() - srcSpecs.p2Offset,
                              mapXss->data(),
                              mapXss->size(),
                              mapYss->data(),
                              mapYss->size(),
                              dstMaskss->data(),
                              dstMaskss->size(),
                              2,
                              srcSpecs.width / 2,
                              srcSpecs.height / 2,
                              srcSpecs.p2Stride,
                              dstSpecs.width / 2,
                              dstSpecs.height / 2,
                              dstSpecs.p2Stride,
                              dst.data() + dstSpecs.p2Offset,
                              dst.size() - dstSpecs.p2Offset);
            break;
        case ImgFrame::Type::RAW8:
        case ImgFrame::Type::GRAY8:
            assert(dstSpecs.width * dstSpecs.height >= dst.size());
            DaiDsp_remapImage(src.data() + srcSpecs.p1Offset,
                              src.size() - srcSpecs.p1Offset,
                              mapX->data(),
                              mapX->size(),
                              mapY->data(),
                              mapY->size(),
                              dstMask->data(),
                              dstMask->size(),
                              1,
                              srcSpecs.width,
                              srcSpecs.height,
                              srcSpecs.p1Stride,
                              dstSpecs.width,
                              dstSpecs.height,
                              dstSpecs.p1Stride,
                              dst.data() + dstSpecs.p1Offset,
                              dst.size() - dstSpecs.p1Offset);
            break;
        default:
            throw std::runtime_error("Unsupported image format. Only YUV420p, RGB888p, BGR888p, RGB888i, BGR888i, RAW8, NV12, GRAY8 are supported");
            break;
    }
#endif
}

}  // namespace impl
}  // namespace dai
