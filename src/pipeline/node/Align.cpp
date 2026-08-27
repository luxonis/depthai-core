#include "depthai/pipeline/node/Align.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <optional>
#include <stdexcept>
#include <tuple>
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

AlignProperties& Align::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

Align& Align::setOutputSize(int alignWidth, int alignHeight) {
    properties.alignWidth = alignWidth;
    properties.alignHeight = alignHeight;
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

void Align::buildStage1() {
    if(!runOnHostVar && device != nullptr && device->getPlatform() == Platform::RVC2) {
        pimpl->logger->info("Align is not supported on the RVC2 device, running on host instead.");
        runOnHostVar = true;
    }
}

#if !defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
void Align::run() {
    throw std::runtime_error("Align node requires OpenCV support to run. Please enable OpenCV support in your build configuration.");
}
#else  // DEPTHAI_HAVE_OPENCV_SUPPORT

namespace {

bool isTransformableDatatype(DatatypeEnum datatype) {
    return datatype == DatatypeEnum::Transformable || isDatatypeSubclassOf(DatatypeEnum::Transformable, datatype);
}

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

bool isFrameLikeDatatype(DatatypeEnum datatype) {
    return datatype == DatatypeEnum::ImgFrame || datatype == DatatypeEnum::ImgDetections || datatype == DatatypeEnum::SpatialImgDetections
           || datatype == DatatypeEnum::SegmentationMask;
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

template <typename Message>
std::shared_ptr<Message> castInput(const std::shared_ptr<Buffer>& inputMsg) {
    return std::static_pointer_cast<Message>(inputMsg);
}

template <typename Message>
std::shared_ptr<Message> transformMessage(const std::shared_ptr<Buffer>& inputMsg, const ImgTransformation& targetTransform) {
    return std::make_shared<Message>(castInput<Message>(inputMsg)->transformTo(targetTransform));
}

ImgTransformation extractTransformationFromBuffer(const std::shared_ptr<Buffer>& buffer, DatatypeEnum datatype, spdlog::async_logger& logger) {
    if(buffer == nullptr) {
        throw std::runtime_error("Message is nullptr, cannot extract transformation");
    }
    ImgTransformation transform;
    if(isTransformableDatatype(datatype)) {
        auto transformable = std::dynamic_pointer_cast<Transformable>(buffer);
        if(!transformable || !transformable->getTransformation().has_value()) {
            logger.error("Input message does not have an ImgTransformation set, cannot align it.");
            throw std::runtime_error("Missing ImgTransformation on Align input");
        }

        transform = *transformable->getTransformation();
    } else if(datatype == DatatypeEnum::ImgFrame) {
        auto alignToImg = castInput<ImgFrame>(buffer);
        transform = alignToImg->transformation;
    } else {
        logger.error("Unsupported datatype on Align input: {}", (int)datatype);
        throw std::runtime_error("Unsupported datatype on Align input");
    }
    return transform;
}

struct ImgFrameRunState {
    bool degenerateStereoTransform = false;
    bool rotationOnlyDegenerateTransform = false;
    std::array<std::array<float, 3>, 3> inputIntrinsics = {};
    std::array<std::array<float, 4>, 4> inputToAlignExtrinsics = {};
    uint16_t staticDepthPlane = 0;
    int shiftFactor = 0;
    int alignWidth = 0;
    int alignHeight = 0;

    cv::Mat mapX1;
    cv::Mat mapY1;
    cv::Mat mapX2;
    cv::Mat mapY2;
};

template <typename T>
std::vector<T> flatten(const std::vector<std::vector<T> >& orig) {
    std::vector<T> ret;
    for(const auto& v : orig) ret.insert(ret.end(), v.begin(), v.end());
    return ret;
}

float alignFrameTypeBytesPerPixel(ImgFrame::Type type) {
    if(type == ImgFrame::Type::YUV420p || type == ImgFrame::Type::NV12 || type == ImgFrame::Type::NV21) {
        return 1.5f;
    }
    if(type == ImgFrame::Type::GRAY8 || type == ImgFrame::Type::RAW8) {
        return 1.0f;
    }
    if(type == ImgFrame::Type::RAW16) {
        return 2.0f;
    }
    if(type == ImgFrame::Type::RGB888i || type == ImgFrame::Type::BGR888i || type == ImgFrame::Type::RGB888p || type == ImgFrame::Type::BGR888p) {
        return 3.0f;
    }
    return 0.0f;
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

cv::Mat arrayToCvMat(const std::array<std::array<float, 3>, 3>& orig) {
    cv::Mat cvMat = cv::Mat(3, 3, CV_32FC1);
    memcpy(cvMat.data, orig.data(), orig.size() * sizeof(orig[0]));
    return cvMat;
}

std::pair<cv::Mat, cv::Mat> computeRectificationMatrices(const cv::Mat& inputIntrinsics,
                                                         const cv::Mat& inputDistortion,
                                                         const cv::Mat& alignIntrinsics,
                                                         const cv::Mat& alignDistortion,
                                                         cv::Size imageSize,
                                                         const cv::Mat& rotation,
                                                         const cv::Mat& translation) {
    cv::Mat m1, m2, d1, d2, r, t;
    inputIntrinsics.convertTo(m1, CV_64FC1);
    alignIntrinsics.convertTo(m2, CV_64FC1);
    inputDistortion.convertTo(d1, CV_64FC1);
    alignDistortion.convertTo(d2, CV_64FC1);
    rotation.convertTo(r, CV_64FC1);
    translation.convertTo(t, CV_64FC1);
    t = t.t();

    cv::Mat r1, r2, p1, p2, q;
    cv::stereoRectify(m1, d1, m2, d2, imageSize, r, t, r1, r2, p1, p2, q);  // todo expose alpha param to user for increased fov, maybe flags?

    cv::Mat rectificationR1, rectificationR2;
    r1.convertTo(rectificationR1, CV_32FC1);
    r2.convertTo(rectificationR2, CV_32FC1);

    return std::make_pair(rectificationR1, rectificationR2);
}

void updateShiftFactor(ImgFrameRunState& state, uint16_t staticDepthPlane) {
    int nextShiftFactor = 0;
    if(staticDepthPlane != 0) {
        nextShiftFactor = roundf((state.inputToAlignExtrinsics[0][3] * state.inputIntrinsics[0][0]) / static_cast<float>(staticDepthPlane));
    }

    const int shiftDelta = nextShiftFactor - state.shiftFactor;
    if(shiftDelta != 0) {
        state.mapX1 = state.mapX1 + cv::Scalar(-shiftDelta);
    }

    state.staticDepthPlane = staticDepthPlane;
    state.shiftFactor = nextShiftFactor;
}

void shiftDepthImg(const ImgFrame& input,
                   ImgFrame& output,
                   const std::array<std::array<float, 3>, 3>& inputIntrinsics,
                   const std::array<std::array<float, 4>, 4>& inputToAlignExtrinsics) {
    int depthLut[65536];

    const int width = input.getWidth();
    const int height = input.getHeight();

    const uint16_t* plane = reinterpret_cast<const uint16_t*>(input.getData().data());
    uint16_t* alignedPlane = reinterpret_cast<uint16_t*>(output.getData().data());

    const size_t lineLength = static_cast<size_t>(width) * sizeof(uint16_t);

    const float shiftX = inputToAlignExtrinsics[0][3];
    const float inputFx = inputIntrinsics[0][0];
    const float shiftXPreComputed = shiftX * inputFx;

    depthLut[0] = shiftX > 0 ? width : -width;
    for(int i = 1; i < 65536; i++) {
        depthLut[i] = shiftXPreComputed / (float)i + 0.5f;
    }

    for(int i = 0; i < height; i++) {
        const uint16_t* currentLine = plane + static_cast<size_t>(width) * i;
        uint16_t* alignedLine = alignedPlane + static_cast<size_t>(width) * i;

        memset(alignedLine, 0, lineLength);

        if(shiftX > 0) {
            for(int j = width - 1; j >= 0; j--) {
                uint16_t depth = currentLine[j];

                int shifted = j + depthLut[depth];

                if(shifted < width - 1) {
                    alignedLine[shifted] = depth;
                    alignedLine[shifted + 1] = depth;
                }
            }
        } else {
            for(int j = 0; j < width; j++) {
                uint16_t depth = currentLine[j];

                int shifted = j + depthLut[depth];

                if(shifted > 0) {
                    alignedLine[shifted] = depth;
                    alignedLine[shifted - 1] = depth;
                }
            }
        }
    }
}

void remapNv12(const cv::Mat& inputNv12, cv::Mat& outputNv12, const cv::Mat& mapX, const cv::Mat& mapY, int interpolation, const cv::Scalar& bgColor) {
    cv::Mat bgrFrame;
    cv::cvtColor(inputNv12, bgrFrame, cv::COLOR_YUV2BGR_NV12);

    cv::Mat remappedBgr;
    cv::remap(bgrFrame, remappedBgr, mapX, mapY, interpolation, cv::BORDER_CONSTANT, bgColor);

    const int w = remappedBgr.cols;
    const int h = remappedBgr.rows;
    if(h % 2 != 0 || w % 2 != 0) {
        throw std::runtime_error("Remapped image has odd width or height, cannot convert to NV12");
    }

    cv::Mat yuv420;
    cv::cvtColor(remappedBgr, yuv420, cv::COLOR_BGR2YUV_I420);
    CV_Assert(yuv420.isContinuous());

    const int cw = w / 2;
    const int ch = h / 2;

    cv::Mat yDst(h, w, CV_8UC1, outputNv12.data, outputNv12.step[0]);
    cv::Mat uvDst(ch, cw, CV_8UC2, outputNv12.data + outputNv12.step[0] * h, outputNv12.step[0]);

    uint8_t* srcY = yuv420.ptr<uint8_t>();
    uint8_t* srcU = srcY + static_cast<size_t>(w) * h;
    uint8_t* srcV = srcU + static_cast<size_t>(cw) * ch;

    cv::Mat ySrc(h, w, CV_8UC1, srcY);
    cv::Mat uSrc(ch, cw, CV_8UC1, srcU);
    cv::Mat vSrc(ch, cw, CV_8UC1, srcV);

    ySrc.copyTo(yDst);
    cv::merge(std::vector<cv::Mat>{uSrc, vSrc}, uvDst);
}

void remapYuv420(const cv::Mat& inputYuv420, cv::Mat& outputYuv420, const cv::Mat& mapX, const cv::Mat& mapY, int interpolation, const cv::Scalar& bgColor) {
    cv::Mat bgrFrame;
    cv::cvtColor(inputYuv420, bgrFrame, cv::COLOR_YUV2BGR_IYUV);

    cv::Mat remappedBgr;
    cv::remap(bgrFrame, remappedBgr, mapX, mapY, interpolation, cv::BORDER_CONSTANT, bgColor);

    cv::cvtColor(remappedBgr, outputYuv420, cv::COLOR_BGR2YUV_I420);
}

std::shared_ptr<ImgFrame> makeRemapTarget(const ImgFrame& source, int width, int height, uint32_t dataSize) {
    auto frame = std::make_shared<ImgFrame>();
    frame->setData(std::vector<uint8_t>(dataSize));

    frame->fb.type = source.fb.type;
    frame->fb.width = static_cast<unsigned int>(width);
    frame->fb.height = static_cast<unsigned int>(height);
    frame->fb.bytesPP = static_cast<unsigned int>(ImgFrame::typeToBpp(source.fb.type));
    frame->fb.stride = frame->fb.width * frame->fb.bytesPP;
    frame->fb.p1Offset = 0;
    frame->fb.p2Offset = 0;
    frame->fb.p3Offset = 0;

    frame->sourceFb = source.sourceFb;
    frame->transformation = source.transformation;
    frame->cam = source.cam;
    frame->category = source.category;
    frame->instanceNum = source.instanceNum;
    frame->event = source.event;

    frame->setBufferMetadataFrom(&source);
    frame->setSize(width, height);
    frame->setSequenceNum(source.getSequenceNum());
    frame->setInstanceNum(source.instanceNum);
    return frame;
}

std::shared_ptr<ImgFrame> alignImgFrame(ImgFrame& inputImg, const ImgFrameRunState& state, bool isSegmentationMask, std::array<uint8_t, 3> bgColorRgb) {
    const cv::Scalar bgColor(bgColorRgb[0], bgColorRgb[1], bgColorRgb[2]);

    const ImgFrame::Type inputFrameType = inputImg.getType();
    const float inputFrameBpp = alignFrameTypeBytesPerPixel(inputFrameType);
    if(inputFrameBpp <= 0.0f) {
        throw std::runtime_error(fmt::format("Frame type '{}' is not supported in Align", static_cast<int>(inputFrameType)));
    }

    const int width = static_cast<int>(inputImg.getWidth());
    const int height = static_cast<int>(inputImg.getHeight());
    if(width != state.mapX1.cols || height != state.mapX1.rows) {
        throw std::runtime_error(fmt::format("Input frame size {}x{} does not match the size {}x{} the rectification maps were computed for",
                                             width,
                                             height,
                                             state.mapX1.cols,
                                             state.mapX1.rows));
    }

    const auto frameSize = static_cast<uint32_t>(roundf(width * height * inputFrameBpp));
    const auto outFrameSize = static_cast<uint32_t>(roundf(state.alignWidth * state.alignHeight * inputFrameBpp));

    const int interp = cv::INTER_NEAREST;
    const int chromaInterp = isSegmentationMask ? cv::INTER_NEAREST : cv::INTER_LINEAR;

    const auto remapFrame = [&](ImgFrame& src, ImgFrame& dst, const cv::Mat& mapX, const cv::Mat& mapY) {
        auto srcFrame = src.getFrame();
        auto dstFrame = dst.getFrame();
        if(inputFrameBpp == 1.5f) {
            if(dst.getType() == ImgFrame::Type::NV12) {
                remapNv12(srcFrame, dstFrame, mapX, mapY, chromaInterp, bgColor);
            } else {  // YUV420p; no other frame type has 1.5 bytes per pixel
                remapYuv420(srcFrame, dstFrame, mapX, mapY, chromaInterp, bgColor);
            }
        } else if(dst.getType() == ImgFrame::Type::RGB888p || dst.getType() == ImgFrame::Type::BGR888p) {
            cv::Mat remapped;
            cv::remap(src.getCvFrame(), remapped, mapX, mapY, interp, cv::BORDER_CONSTANT, bgColor);
            dst.setCvFrame(remapped, dst.getType());
        } else {
            cv::remap(srcFrame, dstFrame, mapX, mapY, interp, cv::BORDER_CONSTANT, bgColor);
        }
    };

    // warp1: undistort + rectify in the input camera frame
    auto rectified = makeRemapTarget(inputImg, width, height, frameSize);
    remapFrame(inputImg, *rectified, state.mapX1, state.mapY1);

    auto warp2Input = rectified;
    if(inputFrameType == ImgFrame::Type::RAW16 && state.staticDepthPlane == 0 && !state.degenerateStereoTransform) {
        auto shifted = makeRemapTarget(inputImg, width, height, frameSize);
        shiftDepthImg(*rectified, *shifted, state.inputIntrinsics, state.inputToAlignExtrinsics);
        warp2Input = shifted;
    }

    // warp2: rectify into the align camera frame
    auto alignedImg = makeRemapTarget(inputImg, state.alignWidth, state.alignHeight, outFrameSize);
    remapFrame(*warp2Input, *alignedImg, state.mapX2, state.mapY2);

    return alignedImg;
}

ImgFrameRunState prepareRectificationMatrices(const ImgTransformation& inputTransform, const ImgTransformation& alignToTransform) {
    ImgFrameRunState state;
    try {
        state.inputIntrinsics = inputTransform.getIntrinsicMatrix();
        std::array<std::array<float, 3>, 3> alignSourceIntrinsics = alignToTransform.getIntrinsicMatrix();
        state.inputToAlignExtrinsics = inputTransform.getExtrinsics().getExtrinsicsTransformationTo(alignToTransform.getExtrinsics());
        std::vector<float> inputDistortionCoefficients = inputTransform.getDistortionCoefficients();
        std::vector<float> alignDistortionCoefficients = alignToTransform.getDistortionCoefficients();

        int alignWidth = static_cast<int>(alignToTransform.getSize().first);
        int alignHeight = static_cast<int>(alignToTransform.getSize().second);

        std::vector<std::vector<float> > inputToAlignRotation = dai::matrix::matrix3x3ToVectorMatrix(inputTransform.getRotationMatrixTo(alignToTransform));
        auto translation = inputTransform.getTranslationVectorTo(alignToTransform, false, LengthUnit::MILLIMETER);
        std::vector<float> inputToAlignTranslation = {translation[0], translation[1], translation[2]};

        if(inputDistortionCoefficients.empty()) {
            inputDistortionCoefficients.assign(14, 0.0f);
        }
        int inputWidth = static_cast<int>(inputTransform.getSize().first);
        int inputHeight = static_cast<int>(inputTransform.getSize().second);

        auto inputIntrinsicsMat = arrayToCvMat(state.inputIntrinsics);
        auto alignIntrinsicsMat = arrayToCvMat(alignSourceIntrinsics);

        auto inputDistortion = vecToCvMat(1, inputDistortionCoefficients.size(), CV_32FC1, inputDistortionCoefficients);
        cv::Mat noDistortion = cv::Mat::zeros(1, static_cast<int>(alignDistortionCoefficients.size()), CV_32FC1);  // No distortion for aligned frame

        auto rotation = vecToCvMat(3, 3, CV_32FC1, inputToAlignRotation);
        auto translationMat = vecToCvMat(1, 3, CV_32FC1, inputToAlignTranslation);

        const float translationNorm =
            std::sqrt(inputToAlignTranslation[0] * inputToAlignTranslation[0] + inputToAlignTranslation[1] * inputToAlignTranslation[1]
                      + inputToAlignTranslation[2] * inputToAlignTranslation[2]);
        state.degenerateStereoTransform = translationNorm <= 1e-6f;
        const std::array<std::array<float, 3>, 3> identityRotation = {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
        state.rotationOnlyDegenerateTransform =
            state.degenerateStereoTransform && !dai::matrix::mateq(dai::matrix::vectorMatrixToMatrix3x3(inputToAlignRotation), identityRotation, 1e-4f);

        cv::Mat rectificationR1;
        cv::Mat rectificationR2;
        if(state.degenerateStereoTransform) {
            rectificationR1 = cv::Mat::eye(3, 3, CV_32FC1);
            rectificationR2 = cv::Mat::eye(3, 3, CV_32FC1);
        } else {
            cv::Size imageSize = cv::Size(inputWidth, inputHeight);
            std::tie(rectificationR1, rectificationR2) =
                computeRectificationMatrices(inputIntrinsicsMat, inputDistortion, alignIntrinsicsMat, noDistortion, imageSize, rotation, translationMat);
        }

        auto targetCamMatrix = inputIntrinsicsMat.clone();
        auto meshSize = cv::Size(inputWidth, inputHeight);

        cv::initUndistortRectifyMap(inputIntrinsicsMat, inputDistortion, rectificationR1, targetCamMatrix, meshSize, CV_32FC1, state.mapX1, state.mapY1);

        cv::Mat newRotation;
        cv::Mat newTranslation = cv::Mat::zeros(3, 1, CV_32FC1);
        if(state.rotationOnlyDegenerateTransform) {
            newRotation = rotation.clone();
        } else if(state.degenerateStereoTransform) {
            newRotation = cv::Mat::eye(3, 3, CV_32FC1);
        } else {
            newRotation = rectificationR2 * (rotation * rectificationR1.t());
            newTranslation = rectificationR2 * translationMat.t();
        }

        for(size_t i = 0; i < 3; i++) {
            for(size_t j = 0; j < 3; j++) {
                state.inputToAlignExtrinsics[i][j] = newRotation.at<float>(i, j);
            }
        }
        for(size_t i = 0; i < 3; i++) {
            state.inputToAlignExtrinsics[i][3] = newTranslation.at<float>(i);
        }

        cv::Mat backRotation;
        if(state.rotationOnlyDegenerateTransform) {
            backRotation = rotation.clone();
        } else {
            backRotation = rectificationR2.t();
        }

        meshSize = cv::Size(alignWidth, alignHeight);

        cv::initUndistortRectifyMap(targetCamMatrix, noDistortion, backRotation, alignIntrinsicsMat, meshSize, CV_32FC1, state.mapX2, state.mapY2);

        state.alignWidth = alignWidth;
        state.alignHeight = alignHeight;

    } catch(const std::exception& e) {
        throw std::runtime_error(fmt::format("Failed to prepare rectification maps: {}", e.what()));
    }

    return state;
}

std::shared_ptr<Buffer> buildAlignedOutputMessage(const std::shared_ptr<Buffer>& inputMsg,
                                                  DatatypeEnum inputType,
                                                  const ImgTransformation& targetTransform,
                                                  const ImgFrameRunState& runState,
                                                  bool& warnedAboutDistortion,
                                                  spdlog::async_logger& logger) {
    const auto alignToDistortion = targetTransform.getDistortionCoefficients();
    bool hasDistortion = std::any_of(alignToDistortion.begin(), alignToDistortion.end(), [](float value) { return std::abs(value) > 0.0f; });
    ImgTransformation outputTransform = targetTransform;
    if(!alignToDistortion.empty()) {
        outputTransform.setDistortionCoefficients(std::vector<float>(alignToDistortion.size(), 0.0f));
    }

    auto warnAboutDistortion = [&]() {
        if(!warnedAboutDistortion && hasDistortion) {
            logger.warn(
                "The input connected to inputAlignTo is distorted. The aligned image / segmentation mask will still be undistorted, meaning it won't be "
                "perfectly aligned.");
            warnedAboutDistortion = true;
        };
    };

    if(inputType == DatatypeEnum::ImgFrame) {
        auto imgFrameInput = castInput<ImgFrame>(inputMsg);

        warnAboutDistortion();

        auto alignedImg = alignImgFrame(*imgFrameInput, runState, false, {0, 0, 0});
        const auto [sourceWidth, sourceHeight] = outputTransform.getSourceSize();
        alignedImg->setSourceSize(static_cast<unsigned int>(sourceWidth), static_cast<unsigned int>(sourceHeight));
        alignedImg->setTransformation(outputTransform);
        return alignedImg;
    }

    if(inputType == DatatypeEnum::ImgDetections) {
        auto imgDetectionsInput = castInput<ImgDetections>(inputMsg);
        std::optional<ImgFrame> segMask = imgDetectionsInput->getSegmentationMask();

        auto alignedImg = transformMessage<ImgDetections>(inputMsg, outputTransform);

        if(segMask) {
            warnAboutDistortion();

            auto alignedSegMask = alignImgFrame(*segMask, runState, true, {255, 255, 255});
            alignedImg->setSegmentationMask(*alignedSegMask);
        }

        return alignedImg;
    }

    if(inputType == DatatypeEnum::SpatialImgDetections) {
        auto spatialImgDetectionsInput = castInput<SpatialImgDetections>(inputMsg);
        auto segMask = spatialImgDetectionsInput->getSegmentationMask();

        auto alignedSpatialImgDetections = transformMessage<SpatialImgDetections>(inputMsg, outputTransform);

        if(segMask) {
            warnAboutDistortion();
            auto alignedSegMask = alignImgFrame(*segMask, runState, true, {255, 255, 255});
            alignedSpatialImgDetections->setSegmentationMask(*alignedSegMask);
        }

        return alignedSpatialImgDetections;
    }

    if(inputType == DatatypeEnum::SegmentationMask) {
        auto segMaskInput = castInput<SegmentationMask>(inputMsg);
        auto segMaskFrame = segMaskInput->getFrame();

        warnAboutDistortion();

        auto alignedSegMaskFrame = alignImgFrame(segMaskFrame, runState, true, {255, 255, 255});

        std::shared_ptr<SegmentationMask> alignedSegMask = std::make_shared<SegmentationMask>();
        alignedSegMask->setMask(*alignedSegMaskFrame);
        alignedSegMask->setTimestamp(segMaskInput->getTimestamp());
        alignedSegMask->setTimestampDevice(segMaskInput->getTimestampDevice());
        alignedSegMask->setSequenceNum(segMaskInput->getSequenceNum());
        alignedSegMask->setTransformation(outputTransform);
        alignedSegMask->setLabels(segMaskInput->getLabels());
        return alignedSegMask;
    }

    if(inputType == DatatypeEnum::AprilTags) {
        return transformMessage<AprilTags>(inputMsg, outputTransform);
    }

    if(inputType == DatatypeEnum::Tracklets) {
        return transformMessage<Tracklets>(inputMsg, outputTransform);
    }

    if(inputType == DatatypeEnum::PointCloudData) {
        logger.warn("Alignment of PointCloudData should be handled before PointCloud node creates the message. Returning identity transformation");
        return transformMessage<PointCloudData>(inputMsg, outputTransform);
    }

    if(inputType == DatatypeEnum::Transformable) {  // custom python messages
        // The Transformable tag covers the whole subtree of custom types, so this cast stays runtime-checked.
        auto transformableInput = std::dynamic_pointer_cast<TransformableBuffer>(inputMsg);
        if(transformableInput) {
            return transformableInput->transformTo(outputTransform);
        }
    }

    throw std::runtime_error(fmt::format("Unsupported datatype for alignment: {}", static_cast<int>(inputType)));
}

}  // namespace

void Align::run() {
    auto& logger = pimpl->logger;

    auto latestConfig = initialConfig;
    bool warnedAboutDistortion = false;

    const auto adjustAlignToTransform = [&](ImgTransformation transform) {
        if(properties.alignWidth > 0 && properties.alignHeight > 0) {
            return adjustScale(std::move(transform), properties.alignWidth, properties.alignHeight);
        }
        return transform;
    };

    ImgTransformation inputTransform;
    ImgTransformation alignToTransform;
    ImgFrameRunState runState;
    bool initialized = false;
    bool runStateValid = false;

    while(mainLoop()) {
        std::shared_ptr<Buffer> inputMsg = nullptr;
        std::shared_ptr<Buffer> alignToMsg = nullptr;
        std::shared_ptr<AlignConfig> inConfig = nullptr;

        {
            auto blockEvent = this->inputBlockEvent();

            inputMsg = input.get<Buffer>();
            if(!initialized) {
                alignToMsg = inputAlignTo.get<Buffer>();
            } else {
                alignToMsg = inputAlignTo.getWaitForMessage() ? inputAlignTo.get<Buffer>() : inputAlignTo.tryGet<Buffer>();
            }

            inConfig = inputConfig.getWaitForMessage() ? inputConfig.get<AlignConfig>() : inputConfig.tryGet<AlignConfig>();
        }
        auto tStart = std::chrono::steady_clock::now();

        if(inConfig) latestConfig = inConfig;

        ImgTransformation newAlignToTransform = alignToTransform;
        if(alignToMsg) {
            const DatatypeEnum alignToDatatype = classifyInputDatatype(alignToMsg);
            newAlignToTransform = adjustAlignToTransform(extractTransformationFromBuffer(alignToMsg, alignToDatatype, *logger));
        }
        const DatatypeEnum inputDatatype = classifyInputDatatype(inputMsg);
        ImgTransformation newInputTransform = extractTransformationFromBuffer(inputMsg, inputDatatype, *logger);

        const bool transformChanged =
            !initialized || !alignToTransform.isEqualTransformation(newAlignToTransform) || !inputTransform.isEqualTransformation(newInputTransform);

        if(isFrameLikeDatatype(inputDatatype)) {
            if(!runStateValid || transformChanged) {
                if(runStateValid) {
                    logger->info("Input or alignTo transformation changed, updating rectification maps.");
                }
                runState = prepareRectificationMatrices(newInputTransform, newAlignToTransform);
                runStateValid = true;
                warnedAboutDistortion = false;
            }
            updateShiftFactor(runState, latestConfig->staticDepthPlane);
        }

        alignToTransform = newAlignToTransform;
        inputTransform = newInputTransform;
        initialized = true;

        std::shared_ptr<Buffer> alignedInputMsg =
            buildAlignedOutputMessage(inputMsg, inputDatatype, alignToTransform, runState, warnedAboutDistortion, *logger);

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

#endif  // DEPTHAI_HAVE_OPENCV_SUPPORT

}  // namespace node
}  // namespace dai
