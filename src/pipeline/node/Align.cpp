#include "depthai/pipeline/node/Align.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "common/ImgTransformations.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/datatype/AprilTags.hpp"
#include "pipeline/datatype/DatatypeEnum.hpp"
#include "pipeline/datatype/ImgDetections.hpp"
#include "pipeline/datatype/ImgFrame.hpp"
#include "pipeline/datatype/PointCloudData.hpp"
#include "pipeline/datatype/SegmentationMask.hpp"
#include "pipeline/datatype/SpatialImgDetections.hpp"
#include "pipeline/datatype/Tracklets.hpp"

#if defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
    #include <opencv2/calib3d.hpp>
    #include <opencv2/core/types.hpp>
    #include <opencv2/imgproc/imgproc.hpp>
#endif

namespace dai {
namespace node {

static constexpr const bool PRINT_DEBUG = false;

ImageAlignProperties& Align::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

Align& Align::setOutputSize(int alignWidth, int alignHeight) {
    properties.alignWidth = alignWidth;
    properties.alignHeight = alignHeight;
    return *this;
}
Align& Align::setOutKeepAspectRatio(bool keep) {
    properties.outKeepAspectRatio = keep;
    return *this;
}

Align& Align::setInterpolation(Interpolation interp) {
    properties.interpolation = interp;
    return *this;
}

Align& Align::setNumShaves(int numShaves) {
    properties.numShaves = numShaves;
    return *this;
}

Align& Align::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
    return *this;
}

void Align::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

/**
 * Check if the node is set to run on host
 */
bool Align::runOnHost() const {
    return runOnHostVar;
}

#if !defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
void Align::run() {
    throw std::runtime_error("Align node requires OpenCV support to run. Please enable OpenCV support in your build configuration.");
}
#else  // DEPTHAI_HAVE_OPENCV_SUPPORT

namespace {

template <typename T>
std::vector<T> flatten(const std::vector<std::vector<T> >& orig) {
    std::vector<T> ret;
    for(const auto& v : orig) ret.insert(ret.end(), v.begin(), v.end());
    return ret;
}

bool isTransformableDatatype(DatatypeEnum datatype) {
    return datatype == DatatypeEnum::Transformable || isDatatypeSubclassOf(DatatypeEnum::Transformable, datatype);
}

template <typename Message>
std::shared_ptr<Message> transformMessage(const std::shared_ptr<Buffer>& inputMsg, const ImgTransformation& targetTransform, const char* messageName) {
    auto typedInput = std::dynamic_pointer_cast<Message>(inputMsg);
    if(!typedInput) {
        throw std::logic_error(fmt::format("Failed to cast transformable {} input", messageName));
    }
    return std::make_shared<Message>(typedInput->transformTo(targetTransform));
}

cv::Mat vecToCvMat(int rows, int cols, int type, const std::vector<std::vector<float> >& orig) {
    std::vector<float> flat = flatten(orig);
    cv::Mat cvMat = cv::Mat(rows, cols, type);
    memcpy(cvMat.data, flat.data(), flat.size() * sizeof(flat[0]));
    return cvMat;
}

cv::Mat vecToCvMat(int rows, int cols, int type, const std::vector<float>& orig) {
    cv::Mat cvMat = cv::Mat(rows, cols, type);
    memcpy(cvMat.data, orig.data(), orig.size() * sizeof(orig[0]));
    return cvMat;
}

cv::Mat arrayToCvMat(int rows, int cols, int type, const std::array<std::array<float, 3>, 3>& orig) {
    cv::Mat cvMat = cv::Mat(rows, cols, type);
    memcpy(cvMat.data, orig.data(), orig.size() * sizeof(orig[0]));
    return cvMat;
}

std::pair<cv::Mat, cv::Mat> computeRectificationMatrices(cv::Mat M1, cv::Mat d1, cv::Mat M2, cv::Mat d2, cv::Size imageSize, cv::Mat R, cv::Mat T) {
    cv::Mat R1, R2, P1, P2, Q;
    cv::Mat _cv_M1, _cv_M2, _cv_d1, _cv_d2, _cv_R, _cv_T;
    M1.convertTo(_cv_M1, CV_64FC1);
    M2.convertTo(_cv_M2, CV_64FC1);
    d1.convertTo(_cv_d1, CV_64FC1);
    d2.convertTo(_cv_d2, CV_64FC1);
    R.convertTo(_cv_R, CV_64FC1);
    T.convertTo(_cv_T, CV_64FC1);
    _cv_T = _cv_T.t();

    cv::stereoRectify(
        _cv_M1, _cv_d1, _cv_M2, _cv_d2, imageSize, _cv_R, _cv_T, R1, R2, P1, P2, Q);  // todo expose alpha param to user for increased fov, maybe flags?

    cv::Mat cv_R1, cv_R2;
    R1.convertTo(cv_R1, CV_32FC1);
    R2.convertTo(cv_R2, CV_32FC1);

    return std::make_pair(cv_R1, cv_R2);
}

std::string matToString(const cv::Mat& mat) {
    std::ostringstream oss;
    oss << mat;
    return oss.str();
}

int shiftDepthImg(const std::shared_ptr<dai::ImgFrame>& inVec,
                  const std::shared_ptr<dai::ImgFrame>& outVec,
                  const std::array<std::array<float, 3>, 3>& depthSourceIntrinsics,
                  const std::array<std::array<float, 4>, 4>& depthToAlignExtrinsics) {
    int depthLut[65536];

    auto& depthIntrinsics = depthSourceIntrinsics;
    auto& cameraExtrinsics = depthToAlignExtrinsics;

    int width = inVec->getWidth();
    int height = inVec->getHeight();
    int bpp = 2;  // todo

    const uint16_t* plane = reinterpret_cast<const uint16_t*>(inVec->getData().data());
    uint16_t* alignedPlane = reinterpret_cast<uint16_t*>(outVec->getData().data());

    // assert(!(bpp != sizeof(*plane)) && "Wrong in/out size");
    // assert(!(bpp != sizeof(*alignedPlane)) && "Wrong in/out size");

    const int lineLength = width * bpp;

    float shiftX = cameraExtrinsics[0][3];

    float depthFx = depthIntrinsics[0][0];
    // float depthCx = depthIntrinsics[0][2];

    float shiftXPreComputed = shiftX * depthFx;

    depthLut[0] = shiftX > 0 ? width : -width;
    for(int i = 1; i < 65536; i++) {
        depthLut[i] = shiftXPreComputed / (float)i + 0.5f;
    }

    for(int i = 0; i < height; i++) {
        const uint16_t* currentLine = plane + width * i;
        uint16_t* alignedLine = alignedPlane + width * i;

        memset(alignedLine, 0, lineLength);

        if(shiftX > 0) {
            for(int j = width - 1; j >= 0; j--) {
                uint16_t depth = currentLine[j];

                int int_u = j + depthLut[depth];

                if(int_u < width - 1) {
                    alignedLine[int_u] = depth;
                    alignedLine[int_u + 1] = depth;
                }
            }

        } else {
            for(int j = 0; j < width; j++) {
                uint16_t depth = currentLine[j];

                int int_u = j + depthLut[depth];

                if(int_u > 0) {
                    alignedLine[int_u] = depth;
                    alignedLine[int_u - 1] = depth;
                }
            }
        }
    }

    return 0;
}

}  // namespace

struct Align::ImgFrameRunState {
    bool degenerateStereoTransform = false;
    bool rotationOnlyDegenerateTransform = false;
    std::array<std::array<float, 3>, 3> depthSourceIntrinsics = {};
    std::array<std::array<float, 4>, 4> depthToAlignExtrinsics = {};
    uint16_t staticDepthPlane = 0;
    int shiftFactor = 0;
    int alignWidth = 0;
    int alignHeight = 0;

    cv::Mat map_x_1;
    cv::Mat map_y_1;
    cv::Mat map_x_2;
    cv::Mat map_y_2;
};

DatatypeEnum classifyInputDatatype(const std::shared_ptr<Buffer>& buffer) {
    if(buffer == nullptr) {
        throw std::runtime_error("Message is nullptr, cannot align");
    }

    const auto datatype = buffer->getDatatype();
    if(datatype == DatatypeEnum::ImgFrame || isTransformableDatatype(datatype)) {
        return datatype;
    }

    throw std::runtime_error("Unsupported datatype on Align input");
}

ImgTransformation adjustScale(ImgTransformation t, int w, int h) {
    auto [currentW, currentH] = t.getSize();

    if(static_cast<int>(currentW) != w || static_cast<int>(currentH) != h) {
        float scaleX = static_cast<float>(w) / static_cast<float>(currentW);
        float scaleY = static_cast<float>(h) / static_cast<float>(currentH);
        t.addScale(scaleX, scaleY);
        t.setSize(w, h);
    }
    return t;
}

void Align::run() {
    auto& logger = pimpl->logger;

    auto firstInput = input.get<Buffer>();
    auto firstAlignTo = inputAlignTo.get<Buffer>();

    std::shared_ptr<Buffer> alignToMsg = firstAlignTo;
    DatatypeEnum alignToDatatype = classifyInputDatatype(alignToMsg);
    DatatypeEnum inputDatatype = classifyInputDatatype(firstInput);

    bool warnedAboutDistortion = false;

    auto latestConfig = initialConfig;
    const auto adjustAlignToTransform = [&](ImgTransformation transform) {
        if(properties.alignWidth > 0 && properties.alignHeight > 0) {
            return adjustScale(std::move(transform), properties.alignWidth, properties.alignHeight);
        }
        return transform;
    };
    const auto isFrameLike = [](DatatypeEnum datatype) {
        return datatype == DatatypeEnum::ImgFrame || datatype == DatatypeEnum::ImgDetections || datatype == DatatypeEnum::SpatialImgDetections
               || datatype == DatatypeEnum::SegmentationMask;
    };

    ImgTransformation alignToTransform = adjustAlignToTransform(extractTransformationFromBuffer(alignToMsg, alignToDatatype));
    dai::ImgTransformation newAlignToTransform = alignToTransform;
    ImgTransformation inputTransform = extractTransformationFromBuffer(firstInput, inputDatatype);

    ImgFrameRunState runState = prepareRectificationMatrices(inputTransform, alignToTransform);
    updateShiftFactor(runState, latestConfig->staticDepthPlane);

    auto nextInputMsg = [&]() -> std::shared_ptr<Buffer> {
        if(firstInput) return std::exchange(firstInput, nullptr);
        return input.get<Buffer>();
    };

    auto nextAlignToMsg = [&]() -> std::shared_ptr<Buffer> {
        if(firstAlignTo) return std::exchange(firstAlignTo, nullptr);
        return inputAlignTo.tryGet<Buffer>();
    };

    while(mainLoop()) {
        std::shared_ptr<Buffer> inputMsg = nullptr;
        std::shared_ptr<ImageAlignConfig> inConfig = nullptr;

        {
            auto blockEvent = this->inputBlockEvent();

            inputMsg = nextInputMsg();
            alignToMsg = nextAlignToMsg();

            inConfig = inputConfig.getWaitForMessage() ? inputConfig.get<ImageAlignConfig>() : inputConfig.tryGet<ImageAlignConfig>();
        }
        auto tStart = std::chrono::steady_clock::now();

        if(inConfig) latestConfig = inConfig;

        if(alignToMsg) {
            alignToDatatype = classifyInputDatatype(alignToMsg);
            newAlignToTransform = adjustAlignToTransform(extractTransformationFromBuffer(alignToMsg, alignToDatatype));
        }
        inputDatatype = classifyInputDatatype(inputMsg);
        auto newInputTransform = extractTransformationFromBuffer(inputMsg, inputDatatype);
        DatatypeEnum inputType = inputDatatype;

        const bool transformChanged = !alignToTransform.isEqualTransformation(newAlignToTransform) || !inputTransform.isEqualTransformation(newInputTransform);

        if(isFrameLike(inputType) && transformChanged) {
            logger->info("Input or alignTo transformation changed, updating rectification maps.");
            runState = prepareRectificationMatrices(newInputTransform, newAlignToTransform);
            warnedAboutDistortion = false;
        }
        if(isFrameLike(inputType)) {
            updateShiftFactor(runState, latestConfig->staticDepthPlane);
        }

        alignToTransform = newAlignToTransform;
        inputTransform = newInputTransform;

        std::shared_ptr<Buffer> alignedInputMsg = buildAlignedOutputMessage(inputMsg, inputType, alignToTransform, runState, warnedAboutDistortion);

        auto tStop = std::chrono::steady_clock::now();
        auto runtime = std::chrono::duration_cast<std::chrono::milliseconds>(tStop - tStart).count();
        logger->trace("Generic align step took {} ms", runtime);

        {
            auto blockEvent = this->outputBlockEvent();
            outputAligned.send(alignedInputMsg);
            passthroughInput.send(inputMsg);
        }
    }
}

ImgTransformation Align::extractTransformationFromBuffer(const std::shared_ptr<Buffer>& buffer, DatatypeEnum datatype) {
    if(buffer == nullptr) {
        throw std::runtime_error("Message is nullptr, cannot extract transformation");
    }
    auto logger = pimpl->logger;
    ImgTransformation transform;
    if(isTransformableDatatype(datatype)) {
        auto transformable = std::dynamic_pointer_cast<Transformable>(buffer);
        if(!transformable || !transformable->getTransformation().has_value()) {
            logger->error("Input message does not have an ImgTransformation set, cannot align it.");
            throw std::runtime_error("Missing ImgTransformation on Align input");
        }

        transform = *transformable->getTransformation();
    } else if(datatype == DatatypeEnum::ImgFrame) {
        auto alignToImg = std::dynamic_pointer_cast<ImgFrame>(buffer);
        transform = alignToImg->transformation;
    } else {
        logger->error("Unsupported datatype on Align input: {}", (int)datatype);
        throw std::runtime_error("Unsupported datatype on Align input");
    }
    return transform;
}

std::shared_ptr<ImgFrame> Align::alignImgFrame(ImgFrame inputImg, const Align::ImgFrameRunState& state, std::array<uint8_t, 3> bgColorRgb) {
    using namespace std::chrono;
    auto& logger = pimpl->logger;
    const cv::Scalar bgColor(bgColorRgb[0], bgColorRgb[1], bgColorRgb[2]);

    ImgFrame::Type inputFrameType = inputImg.getType();

    bool allocated = false;
    uint32_t frameSize = 0;
    uint32_t outFrameSize = 0;

    std::unordered_set<ImgFrame::Type> supportedFrameTypes = {ImgFrame::Type::YUV420p,
                                                              ImgFrame::Type::NV12,
                                                              ImgFrame::Type::GRAY8,
                                                              ImgFrame::Type::RAW8,
                                                              ImgFrame::Type::RAW16,
                                                              ImgFrame::Type::RGB888i,
                                                              ImgFrame::Type::BGR888i,
                                                              ImgFrame::Type::RGB888p,
                                                              ImgFrame::Type::BGR888p};

    std::unordered_map<ImgFrame::Type, float> frameTypeToBpp = {
        {ImgFrame::Type::YUV420p, 1.5f},
        {ImgFrame::Type::NV12, 1.5f},
        {ImgFrame::Type::GRAY8, 1.0f},
        {ImgFrame::Type::RAW8, 1.0f},
        {ImgFrame::Type::RAW16, 2.0f},
        {ImgFrame::Type::RGB888i, 3.0f},
        {ImgFrame::Type::BGR888i, 3.0f},
        {ImgFrame::Type::RGB888p, 3.0f},
        {ImgFrame::Type::BGR888p, 3.0f},
    };

    auto allocatePools = [&](int width, int height, int alignWidth, int alignHeight, float inputFrameBpp) -> std::pair<bool, std::string> {
        if(allocated) return {true, ""};

        float bpp = inputFrameBpp;

        frameSize = roundf(width * height * bpp);

        outFrameSize = roundf(alignWidth * alignHeight * bpp);

        allocated = true;
        return {true, ""};
    };

    int alignWidth = state.alignWidth;
    int alignHeight = state.alignHeight;

    auto remapNv12 = [&](cv::Mat& inputNV12, cv::Mat& outputNV12, cv::Mat& map_x, cv::Mat& map_y) {
        cv::Mat bgrFrame;
        cv::cvtColor(inputNV12, bgrFrame, cv::COLOR_YUV2BGR_NV12);

        cv::Mat remappedBGR;
        cv::remap(bgrFrame, remappedBGR, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, bgColor);

        CV_Assert((remappedBGR.cols % 2) == 0 && (remappedBGR.rows % 2) == 0);

        cv::Mat yuv420;
        cv::cvtColor(remappedBGR, yuv420, cv::COLOR_BGR2YUV_I420);
        CV_Assert(yuv420.isContinuous());

        const int w = remappedBGR.cols;
        const int h = remappedBGR.rows;

        if(h % 2 != 0 || w % 2 != 0) {
            throw std::runtime_error("Remapped image has odd width or height, cannot convert to NV12");
        }
        const int cw = w / 2;
        const int ch = h / 2;

        cv::Mat yDst(h, w, CV_8UC1, outputNV12.data, outputNV12.step[0]);
        cv::Mat uvDst(ch, cw, CV_8UC2, outputNV12.data + outputNV12.step[0] * h, outputNV12.step[0]);

        const uint8_t* srcY = yuv420.ptr<uint8_t>();
        const uint8_t* srcU = srcY + static_cast<size_t>(w) * h;
        const uint8_t* srcV = srcU + static_cast<size_t>(cw) * ch;

        cv::Mat ySrc(h, w, CV_8UC1, const_cast<uint8_t*>(srcY));
        cv::Mat uSrc(ch, cw, CV_8UC1, const_cast<uint8_t*>(srcU));
        cv::Mat vSrc(ch, cw, CV_8UC1, const_cast<uint8_t*>(srcV));

        ySrc.copyTo(yDst);
        cv::merge(std::vector<cv::Mat>{uSrc, vSrc}, uvDst);
    };

    auto remapYuv420 = [&](cv::Mat& inputYUV420, cv::Mat& outputYUV420, cv::Mat& map_x, cv::Mat& map_y) {
        cv::Mat bgrFrame;
        cv::cvtColor(inputYUV420, bgrFrame, cv::COLOR_YUV2BGR_IYUV);

        cv::Mat remappedBGR;
        cv::remap(bgrFrame, remappedBGR, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, bgColor);

        cv::cvtColor(remappedBGR, outputYUV420, cv::COLOR_BGR2YUV_I420);
    };

    bool inputIsDepth = inputImg.getType() == ImgFrame::Type::RAW16;

    uint32_t width = inputImg.getWidth();
    uint32_t height = inputImg.getHeight();

    inputFrameType = inputImg.getType();

    if(!supportedFrameTypes.count(inputFrameType)) {
        throw std::runtime_error("Unsupported frame type in Align");
    }
    float inputFrameBpp = frameTypeToBpp[inputFrameType];

    auto [success, msg] = allocatePools(width, height, alignWidth, alignHeight, inputFrameBpp);
    if(!success) {
        // logger->error(msg);
        throw std::runtime_error(msg);
    }

    decltype(steady_clock::now()) t1, t2, tStart, tStop;
    tStart = steady_clock::now();
    if(PRINT_DEBUG) {
        t1 = steady_clock::now();
    }

    // warp1
    auto depthImgRectified = std::make_shared<ImgFrame>();
    depthImgRectified->setData(std::vector<uint8_t>(frameSize));
    depthImgRectified->setMetadata(inputImg);
    depthImgRectified->setWidth(inputImg.getWidth());
    depthImgRectified->setHeight(inputImg.getHeight());
    depthImgRectified->setType(inputImg.getType());
    depthImgRectified->fb.stride = depthImgRectified->fb.width * depthImgRectified->getBytesPerPixel();

    auto inputFrame = inputImg.getFrame();
    auto depthImgRectifiedFrame = depthImgRectified->getFrame();

    cv::Mat map_x_1 = state.map_x_1;
    cv::Mat map_y_1 = state.map_y_1;

    cv::Mat map_x_2 = state.map_x_2;
    cv::Mat map_y_2 = state.map_y_2;

    if(inputFrameBpp == 1.5f) {
        auto inputFrameCopy = inputFrame.clone();
        if(depthImgRectified->getType() == ImgFrame::Type::NV12) {
            remapNv12(inputFrameCopy, depthImgRectifiedFrame, map_x_1, map_y_1);
        } else if(depthImgRectified->getType() == ImgFrame::Type::YUV420p) {
            remapYuv420(inputFrameCopy, depthImgRectifiedFrame, map_x_1, map_y_1);
        } else {
            logger->error("Unsupported frame type for NV12/YUV420 remapping: {}", (int)depthImgRectified->getType());
        }
    } else if(depthImgRectified->getType() == ImgFrame::Type::RGB888p || depthImgRectified->getType() == ImgFrame::Type::BGR888p) {
        cv::Mat inputCvFrame = inputImg.getCvFrame();
        cv::Mat remappedCvFrame;
        cv::remap(inputCvFrame, remappedCvFrame, map_x_1, map_y_1, cv::INTER_NEAREST, cv::BORDER_CONSTANT, bgColor);
        depthImgRectified->setCvFrame(remappedCvFrame, depthImgRectified->getType());
    } else {
        cv::remap(inputFrame, depthImgRectifiedFrame, map_x_1, map_y_1, cv::INTER_NEAREST, cv::BORDER_CONSTANT, bgColor);
    }

    if(PRINT_DEBUG) {
        t2 = steady_clock::now();
        auto elapsed = duration_cast<microseconds>(t2 - t1).count() / 1000.f;
        logger->warn("Align step1 took '{}' ms.", elapsed);
    }

    auto shiftedOutput = std::make_shared<ImgFrame>();
    shiftedOutput->setData(std::vector<uint8_t>(frameSize));

    if(PRINT_DEBUG) {
        t1 = steady_clock::now();
    }

    auto warp2Input = depthImgRectified;

    if(inputIsDepth && state.staticDepthPlane == 0 && !state.degenerateStereoTransform) {
        shiftedOutput->setMetadata(inputImg);
        shiftedOutput->setWidth(inputImg.getWidth());
        shiftedOutput->setHeight(inputImg.getHeight());
        shiftedOutput->setType(inputImg.getType());
        shiftedOutput->fb.stride = shiftedOutput->fb.width * shiftedOutput->getBytesPerPixel();

        auto startProcessing = high_resolution_clock::now();

        int nErr = 0;
        nErr = shiftDepthImg(depthImgRectified, shiftedOutput, state.depthSourceIntrinsics, state.depthToAlignExtrinsics);

        if(nErr != 0) {
            logger->error("alignDepthImg failed with code {}", nErr);
        }

        auto stopProcessing = high_resolution_clock::now();

        auto durationProcessing = duration_cast<microseconds>(stopProcessing - startProcessing);
        logger->debug("Processing time: {} ms", durationProcessing.count() / 1000.0f);

        warp2Input = shiftedOutput;
    }

    if(PRINT_DEBUG) {
        t2 = steady_clock::now();
        auto elapsed = duration_cast<microseconds>(t2 - t1).count() / 1000.f;
        logger->warn("Align step2 took '{}' ms.", elapsed);
    }

    if(PRINT_DEBUG) {
        t1 = steady_clock::now();
    }

    // warp2
    auto alignedImg = std::make_shared<ImgFrame>();
    alignedImg->setData(std::vector<uint8_t>(outFrameSize));
    if(PRINT_DEBUG) {
        t2 = steady_clock::now();
        auto elapsed = duration_cast<microseconds>(t2 - t1).count() / 1000.f;
        logger->warn("Align output pool took '{}' ms.", elapsed);
        t1 = steady_clock::now();
    }

    alignedImg->setMetadata(inputImg);
    alignedImg->setWidth(alignWidth);
    alignedImg->setHeight(alignHeight);
    alignedImg->setType(inputImg.getType());
    alignedImg->fb.stride = alignedImg->fb.width * alignedImg->getBytesPerPixel();

    auto warp2InputFrame = warp2Input->getFrame();
    auto alignedImgFrame = alignedImg->getFrame();
    if(inputFrameBpp == 1.5f) {
        if(alignedImg->getType() == ImgFrame::Type::NV12) {
            remapNv12(warp2InputFrame, alignedImgFrame, map_x_2, map_y_2);
        } else if(alignedImg->getType() == ImgFrame::Type::YUV420p) {
            remapYuv420(warp2InputFrame, alignedImgFrame, map_x_2, map_y_2);
        } else {
            logger->error("Unsupported frame type for NV12/YUV420 remapping: {}", (int)alignedImg->getType());
        }
    } else if(alignedImg->getType() == ImgFrame::Type::RGB888p || alignedImg->getType() == ImgFrame::Type::BGR888p) {
        cv::Mat warp2InputCvFrame = warp2Input->getCvFrame();
        cv::Mat remappedCvFrame;
        cv::remap(warp2InputCvFrame, remappedCvFrame, map_x_2, map_y_2, cv::INTER_NEAREST, cv::BORDER_CONSTANT, bgColor);
        alignedImg->setCvFrame(remappedCvFrame, alignedImg->getType());
    } else {
        cv::remap(warp2InputFrame, alignedImgFrame, map_x_2, map_y_2, cv::INTER_NEAREST, cv::BORDER_CONSTANT, bgColor);
    }
    if(PRINT_DEBUG) {
        t2 = steady_clock::now();
        auto elapsed = duration_cast<microseconds>(t2 - t1).count() / 1000.f;
        logger->warn("Align step3 took '{}' ms.", elapsed);
    }

    tStop = steady_clock::now();
    auto runtime = duration_cast<milliseconds>(tStop - tStart).count();
    logger->trace("Align took {} ms", runtime);

    return alignedImg;
}

Align::ImgFrameRunState Align::prepareRectificationMatrices(const ImgTransformation& inputTransform, const ImgTransformation& alignToTransform) {
    ImgFrameRunState state;
    try {
        state.depthSourceIntrinsics = inputTransform.getIntrinsicMatrix();
        std::array<std::array<float, 3>, 3> alignSourceIntrinsics = alignToTransform.getIntrinsicMatrix();
        state.depthToAlignExtrinsics = inputTransform.getExtrinsics().getExtrinsicsTransformationTo(alignToTransform.getExtrinsics());
        std::vector<float> depthDistortionCoefficients = inputTransform.getDistortionCoefficients();
        std::vector<float> alignDistortionCoefficients = alignToTransform.getDistortionCoefficients();

        int alignWidth = static_cast<int>(alignToTransform.getSize().first);
        int alignHeight = static_cast<int>(alignToTransform.getSize().second);

        std::vector<std::vector<float> > depthToAlignRotation = dai::matrix::matrix3x3ToVectorMatrix(inputTransform.getRotationMatrixTo(alignToTransform));
        auto translation = inputTransform.getTranslationVectorTo(alignToTransform, false, LengthUnit::MILLIMETER);
        std::vector<float> depthToAlignTranslation = {translation[0], translation[1], translation[2]};

        if(depthDistortionCoefficients.empty()) {
            depthDistortionCoefficients.assign(14, 0.0f);
        }
        int depthWidth = static_cast<int>(inputTransform.getSize().first);
        int depthHeight = static_cast<int>(inputTransform.getSize().second);

        auto cv_M1 = arrayToCvMat(3, 3, CV_32FC1, state.depthSourceIntrinsics);
        auto cv_M2 = arrayToCvMat(3, 3, CV_32FC1, alignSourceIntrinsics);

        auto cv_d1 = vecToCvMat(1, depthDistortionCoefficients.size(), CV_32FC1, depthDistortionCoefficients);
        auto cv_dNone = cv::Mat::zeros(1, static_cast<int>(alignDistortionCoefficients.size()), CV_32FC1);  // No distortion for aligned frame

        auto cv_R = vecToCvMat(3, 3, CV_32FC1, depthToAlignRotation);
        auto cv_T = vecToCvMat(1, 3, CV_32FC1, depthToAlignTranslation);

        const float translationNorm =
            std::sqrt(depthToAlignTranslation[0] * depthToAlignTranslation[0] + depthToAlignTranslation[1] * depthToAlignTranslation[1]
                      + depthToAlignTranslation[2] * depthToAlignTranslation[2]);
        state.degenerateStereoTransform = translationNorm <= 1e-6f;
        const std::array<std::array<float, 3>, 3> identityRotation = {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
        state.rotationOnlyDegenerateTransform =
            state.degenerateStereoTransform && !dai::matrix::mateq(dai::matrix::vectorMatrixToMatrix3x3(depthToAlignRotation), identityRotation, 1e-4f);

        cv::Mat cv_R1;
        cv::Mat cv_R2;
        if(state.degenerateStereoTransform) {
            cv_R1 = cv::Mat::eye(3, 3, CV_32FC1);
            cv_R2 = cv::Mat::eye(3, 3, CV_32FC1);
        } else {
            cv::Size imageSize = cv::Size(depthWidth, depthHeight);
            std::tie(cv_R1, cv_R2) = computeRectificationMatrices(cv_M1, cv_d1, cv_M2, cv_dNone, imageSize, cv_R, cv_T);
        }

        // dai::CameraModel cameraModel = dai::CameraModel::Perspective;  // todo

        auto cv_targetCamMatrix = cv_M1.clone();
        auto cv_meshSize = cv::Size(depthWidth, depthHeight);

        cv::initUndistortRectifyMap(cv_M1, cv_d1, cv_R1, cv_targetCamMatrix, cv_meshSize, CV_32FC1, state.map_x_1, state.map_y_1);

        cv::Mat cv_newR;
        cv::Mat cv_newT = cv::Mat::zeros(3, 1, CV_32FC1);
        if(state.rotationOnlyDegenerateTransform) {
            cv_newR = cv_R.clone();
        } else if(state.degenerateStereoTransform) {
            cv_newR = cv::Mat::eye(3, 3, CV_32FC1);
        } else {
            cv_newR = cv_R2 * (cv_R * cv_R1.t());
            cv_newT = cv_R2 * cv_T.t();
        }

        for(size_t i = 0; i < 3; i++) {
            for(size_t j = 0; j < 3; j++) {
                state.depthToAlignExtrinsics[i][j] = cv_newR.at<float>(i, j);
            }
        }
        for(size_t i = 0; i < 3; i++) {
            state.depthToAlignExtrinsics[i][3] = cv_newT.at<float>(i);
        }

        // Rotate the depth to the RGB frame
        cv::Mat cv_R_back;
        if(state.rotationOnlyDegenerateTransform) {
            cv_R_back = cv_R.clone();
        } else {
            cv_R_back = cv_R2.t();
        }

        cv_meshSize = cv::Size(alignWidth, alignHeight);

        cv::initUndistortRectifyMap(cv_targetCamMatrix, cv_dNone, cv_R_back, cv_M2, cv_meshSize, CV_32FC1, state.map_x_2, state.map_y_2);

        state.alignWidth = alignWidth;
        state.alignHeight = alignHeight;

    } catch(const std::exception& e) {
        throw std::runtime_error(fmt::format("Failed to prepare rectification maps: {}", e.what()));
    }

    return state;
}

void Align::updateShiftFactor(ImgFrameRunState& state, uint16_t staticDepthPlane) {
    int nextShiftFactor = 0;
    if(staticDepthPlane != 0) {
        nextShiftFactor = roundf((state.depthToAlignExtrinsics[0][3] * state.depthSourceIntrinsics[0][0]) / static_cast<float>(staticDepthPlane));
    }

    const int shiftDelta = nextShiftFactor - state.shiftFactor;
    if(shiftDelta != 0) {
        state.map_x_1 = state.map_x_1 + cv::Scalar(-shiftDelta);
    }

    state.staticDepthPlane = staticDepthPlane;
    state.shiftFactor = nextShiftFactor;
}

std::shared_ptr<Buffer> Align::buildAlignedOutputMessage(const std::shared_ptr<Buffer>& inputMsg,
                                                         DatatypeEnum inputType,
                                                         const ImgTransformation& targetTransform,
                                                         const ImgFrameRunState& runState,
                                                         bool& warnedAboutDistortion) {
    auto& logger = pimpl->logger;

    const auto alignToDistortion = targetTransform.getDistortionCoefficients();
    bool hasDistortion = std::any_of(alignToDistortion.begin(), alignToDistortion.end(), [](float value) { return std::abs(value) > 0.0f; });

    auto warnAboutDistortion = [&]() {
        if(!warnedAboutDistortion && hasDistortion) {
            logger->warn(
                "The input connected to inputAlignTo is distorted. The aligned image / segmentation mask will still be undistorted, meaning it won't be "
                "perfectly aligned.");
            warnedAboutDistortion = true;
        };
    };

    if(inputType == DatatypeEnum::ImgFrame) {
        auto imgFrameInput = std::dynamic_pointer_cast<ImgFrame>(inputMsg);

        warnAboutDistortion();

        auto alignedImg = alignImgFrame(*imgFrameInput, runState, {0, 0, 0});
        alignedImg->setTransformation(targetTransform);
        return alignedImg;
    }

    if(inputType == DatatypeEnum::ImgDetections) {
        std::shared_ptr<ImgDetections> imgDetectionsInput = std::dynamic_pointer_cast<ImgDetections>(inputMsg);
        std::optional<ImgFrame> segMask = imgDetectionsInput->getSegmentationMask();

        auto alignedImg = transformMessage<ImgDetections>(inputMsg, targetTransform, "ImgDetections");

        if(segMask) {
            warnAboutDistortion();

            auto alignedSegMask = alignImgFrame(*segMask, runState, {255, 255, 255});
            alignedImg->setSegmentationMask(*alignedSegMask);
        }

        return alignedImg;
    }

    if(inputType == DatatypeEnum::SpatialImgDetections) {
        auto spatialImgDetectionsInput = std::dynamic_pointer_cast<SpatialImgDetections>(inputMsg);
        auto segMask = spatialImgDetectionsInput->getSegmentationMask();

        auto alignedSpatialImgDetections = transformMessage<SpatialImgDetections>(inputMsg, targetTransform, "SpatialImgDetections");

        if(segMask) {
            warnAboutDistortion();
            auto alignedSegMask = alignImgFrame(*segMask, runState, {255, 255, 255});
            alignedSpatialImgDetections->setSegmentationMask(*alignedSegMask);
        }

        return alignedSpatialImgDetections;
    }

    if(inputType == DatatypeEnum::SegmentationMask) {
        auto segMaskInput = std::dynamic_pointer_cast<SegmentationMask>(inputMsg);
        auto segMaskFrame = segMaskInput->getFrame();

        warnAboutDistortion();

        auto alignedSegMaskFrame = alignImgFrame(segMaskFrame, runState, {255, 255, 255});

        std::shared_ptr<SegmentationMask> alignedSegMask = std::make_shared<SegmentationMask>();
        alignedSegMask->setMask(*alignedSegMaskFrame);
        alignedSegMask->setTimestamp(segMaskInput->getTimestamp());
        alignedSegMask->setTimestampDevice(segMaskInput->getTimestampDevice());
        alignedSegMask->setSequenceNum(segMaskInput->getSequenceNum());
        alignedSegMask->setTransformation(targetTransform);
        alignedSegMask->setLabels(segMaskInput->getLabels());
        return alignedSegMask;
    }

    if(inputType == DatatypeEnum::AprilTags) {
        return transformMessage<AprilTags>(inputMsg, targetTransform, "AprilTags");
    }

    if(inputType == DatatypeEnum::Tracklets) {
        return transformMessage<Tracklets>(inputMsg, targetTransform, "Tracklets");
    }

    if(inputType == DatatypeEnum::PointCloudData) {
        logger->warn("Alignment of PointCloudData should be handled before PointCloud node creates the message. Returning identity transformation");
        return transformMessage<PointCloudData>(inputMsg, targetTransform, "PointCloudData");
    }

    if(inputType == DatatypeEnum::Transformable) {  // custom python messages
        auto transformableInput = std::dynamic_pointer_cast<TransformableBuffer>(inputMsg);
        if(transformableInput) {
            return transformableInput->transformTo(targetTransform);
        }
    }

    throw std::runtime_error(fmt::format("Unsupported datatype for alignment: {}", static_cast<int>(inputType)));
}

#endif  // DEPTHAI_HAVE_OPENCV_SUPPORT

}  // namespace node
}  // namespace dai
