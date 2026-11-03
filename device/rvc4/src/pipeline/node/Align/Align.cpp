#include "Align.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include "depthai/common/DepthUnit.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/pipeline/datatype/AlignConfig.hpp"
#include "depthai/pipeline/datatype/AprilTags.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/SegmentationMask.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai/pipeline/datatype/Tracklets.hpp"
#include "depthai/pipeline/datatype/Transformable.hpp"
#include "depthai/utility/matrixOps.hpp"
#include "hexagon/ImageAlignDsp/shared/ImageAlignDspConfig.h"
#include "pipeline/utilities/Alignment/RectifiedBounds.hpp"
#include "utilities/ImageRemap.hpp"
#include "vendor/DeviceError.hpp"
#include "vendor/dai_device_dsp.h"

namespace dai {
namespace gate {

namespace {

template <typename T>
std::vector<T> flatten(const std::vector<std::vector<T>>& orig) {
    std::vector<T> ret;
    for(const auto& v : orig) ret.insert(ret.end(), v.begin(), v.end());
    return ret;
}

cv::Mat vecToCvMat(int rows, int cols, int type, const std::vector<std::vector<float>>& orig) {
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
    memcpy(cvMat.data, orig.data(), sizeof(orig));
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

bool isTransformableDatatype(DatatypeEnum datatype) {
    return datatype == DatatypeEnum::Transformable || isDatatypeSubclassOf(DatatypeEnum::Transformable, datatype);
}

cv::Mat frameForRemap(ImgFrame& frame) {
    int type = CV_8UC1;
    if(frame.getType() == ImgFrame::Type::RAW16)
        type = CV_16UC1;
    else if(frame.getType() == ImgFrame::Type::GRAYF16)
        type = CV_16FC1;
    else if(frame.getType() == ImgFrame::Type::RGB888i || frame.getType() == ImgFrame::Type::BGR888i)
        type = CV_8UC3;
    return planeView(frame, frame.fb.p1Offset, frame.getHeight(), frame.getWidth(), type, frame.getStride());
}

void remapPlanarRgb(const std::shared_ptr<ImgFrame>& input,
                    const std::shared_ptr<ImgFrame>& output,
                    const cv::Mat& mapX,
                    const cv::Mat& mapY,
                    const cv::InterpolationFlags interpolation,
                    const cv::Scalar& borderColor) {
    const size_t inputPlaneSize = input->getStride() * input->getHeight();
    const size_t outputPlaneSize = output->getStride() * output->getHeight();
    const std::array<size_t, 3> inputOffsets = {input->fb.p1Offset,
                                                input->fb.p2Offset > input->fb.p1Offset ? input->fb.p2Offset : input->fb.p1Offset + inputPlaneSize,
                                                input->fb.p3Offset > input->fb.p2Offset ? input->fb.p3Offset : input->fb.p1Offset + 2 * inputPlaneSize};
    const std::array<size_t, 3> outputOffsets = {output->fb.p1Offset,
                                                 output->fb.p2Offset > output->fb.p1Offset ? output->fb.p2Offset : output->fb.p1Offset + outputPlaneSize,
                                                 output->fb.p3Offset > output->fb.p2Offset ? output->fb.p3Offset : output->fb.p1Offset + 2 * outputPlaneSize};

    for(size_t plane = 0; plane < 3; ++plane) {
        const auto inputPlane = planeView(*input, inputOffsets[plane], input->getHeight(), input->getWidth(), CV_8UC1, input->getStride());
        auto outputPlane = planeView(*output, outputOffsets[plane], output->getHeight(), output->getWidth(), CV_8UC1, output->getStride());
        cv::remap(inputPlane, outputPlane, mapX, mapY, interpolation, cv::BORDER_CONSTANT, cv::Scalar(borderColor[plane]));
    }
}

DatatypeEnum classifyInputDatatype(const std::shared_ptr<Buffer>& buffer) {
    const auto datatype = buffer->getDatatype();
    if(datatype == DatatypeEnum::PointCloudData) {
        throw std::runtime_error("Align node does not support PointCloudData. Align the depth frame before you calculate the point cloud.");
    }
    if(datatype == DatatypeEnum::ImgFrame || isTransformableDatatype(datatype)) {
        return datatype;
    }

    throw std::runtime_error("Unsupported datatype on input, cannot initialize calibration!");
}

std::shared_ptr<ImgFrame> extractFrameForWarp(const std::shared_ptr<Buffer>& buffer) {
    DatatypeEnum datatype = classifyInputDatatype(buffer);
    switch(datatype) {
        case DatatypeEnum::ImgFrame:
            return std::dynamic_pointer_cast<ImgFrame>(buffer);
        case DatatypeEnum::ImgDetections: {
            auto imgDetections = std::dynamic_pointer_cast<ImgDetections>(buffer);
            if(auto segMask = imgDetections->getSegmentationMask()) {
                return std::make_shared<ImgFrame>(*segMask);
            }
            break;
        }
        case DatatypeEnum::SpatialImgDetections: {
            auto spatialImgDetections = std::dynamic_pointer_cast<SpatialImgDetections>(buffer);
            if(auto segMask = spatialImgDetections->getSegmentationMask()) {
                return std::make_shared<ImgFrame>(*segMask);
            }
            break;
        }
        case DatatypeEnum::SegmentationMask: {
            auto segmentationMask = std::dynamic_pointer_cast<SegmentationMask>(buffer);
            if(segmentationMask->data->getSize() != 0) {
                return std::make_shared<ImgFrame>(segmentationMask->getFrame());
            }
            break;
        }
        default:
            break;
    }

    return nullptr;
}

ImgFrame::Type extractFrameTypeForWarp(const std::shared_ptr<Buffer>& buffer) {
    switch(classifyInputDatatype(buffer)) {
        case DatatypeEnum::ImgFrame: {
            auto imgFrame = std::dynamic_pointer_cast<ImgFrame>(buffer);
            return imgFrame ? imgFrame->getType() : ImgFrame::Type::NONE;
        }
        case DatatypeEnum::ImgDetections:
        case DatatypeEnum::SpatialImgDetections:
        case DatatypeEnum::SegmentationMask:
            return buffer->data->getSize() != 0 ? ImgFrame::Type::GRAY8 : ImgFrame::Type::NONE;
        default:
            return ImgFrame::Type::NONE;
    }
}

std::string frameTypeToString(ImgFrame::Type type) {  // todo (aljazkonec): make a helper header or use magic_enum
    switch(type) {
        case ImgFrame::Type::YUV422i:
            return "YUV422i";
        case ImgFrame::Type::YUV444p:
            return "YUV444p";
        case ImgFrame::Type::YUV420p:
            return "YUV420p";
        case ImgFrame::Type::YUV422p:
            return "YUV422p";
        case ImgFrame::Type::YUV400p:
            return "YUV400p";
        case ImgFrame::Type::RGBA8888:
            return "RGBA8888";
        case ImgFrame::Type::RGB161616:
            return "RGB161616";
        case ImgFrame::Type::RGB888p:
            return "RGB888p";
        case ImgFrame::Type::BGR888p:
            return "BGR888p";
        case ImgFrame::Type::RGB888i:
            return "RGB888i";
        case ImgFrame::Type::BGR888i:
            return "BGR888i";
        case ImgFrame::Type::LUT2:
            return "LUT2";
        case ImgFrame::Type::LUT4:
            return "LUT4";
        case ImgFrame::Type::LUT16:
            return "LUT16";
        case ImgFrame::Type::RAW16:
            return "RAW16";
        case ImgFrame::Type::RAW14:
            return "RAW14";
        case ImgFrame::Type::RAW12:
            return "RAW12";
        case ImgFrame::Type::RAW10:
            return "RAW10";
        case ImgFrame::Type::RAW8:
            return "RAW8";
        case ImgFrame::Type::PACK10:
            return "PACK10";
        case ImgFrame::Type::PACK12:
            return "PACK12";
        case ImgFrame::Type::YUV444i:
            return "YUV444i";
        case ImgFrame::Type::NV12:
            return "NV12";
        case ImgFrame::Type::NV21:
            return "NV21";
        case ImgFrame::Type::BITSTREAM:
            return "BITSTREAM";
        case ImgFrame::Type::HDR:
            return "HDR";
        case ImgFrame::Type::RGBF16F16F16p:
            return "RGBF16F16F16p";
        case ImgFrame::Type::BGRF16F16F16p:
            return "BGRF16F16F16p";
        case ImgFrame::Type::RGBF16F16F16i:
            return "RGBF16F16F16i";
        case ImgFrame::Type::BGRF16F16F16i:
            return "BGRF16F16F16i";
        case ImgFrame::Type::GRAY8:
            return "GRAY8";
        case ImgFrame::Type::GRAYF16:
            return "GRAYF16";
        case ImgFrame::Type::RAW32:
            return "RAW32";
        case ImgFrame::Type::NONE:
            return "NONE";
    }
    return fmt::format("<unknown {}>", static_cast<int>(type));
}

}  // namespace

const bool Align::PRINT_DEBUG = false;

std::pair<bool, std::string> Align::allocatePools(int width, int height, int alignWidth, int alignHeight, ImgFrame::Type inputFrameType) {
    uint32_t poolFrames = properties.numFramesPool;

    float bpp = 1.0f;
    switch(inputFrameType) {
        case ImgFrame::Type::YUV420p:
        case ImgFrame::Type::NV12:
            bpp = 1.5f;
            break;
        case ImgFrame::Type::RGB888p:
        case ImgFrame::Type::BGR888p:
        case ImgFrame::Type::RGB888i:
        case ImgFrame::Type::BGR888i:
            bpp = 3.0f;
            break;
        case ImgFrame::Type::RAW8:
        case ImgFrame::Type::GRAY8:
            bpp = 1.0f;
            break;
        case ImgFrame::Type::RAW16:
        case ImgFrame::Type::GRAYF16:
            bpp = 2.0f;
            break;
        default:
            return {false,
                    fmt::format("Align does not support input frame type {}. Supported types are: NV12, YUV420p, RGB888p, BGR888p, RGB888i, BGR888i, "
                                "GRAY8, RAW8, RAW16 and GRAYF16.",
                                frameTypeToString(inputFrameType))};
    }

    uint32_t frameSize = roundf(width * height * bpp);
    uint32_t outFrameSize = roundf(alignWidth * alignHeight * bpp);
    if(allocated && allocatedIntermediateFrameSize == frameSize && allocatedOutputFrameSize == outFrameSize) return {true, ""};

    up_plgPoolAlign = std::make_shared<Pool<ImgFrame>>(poolFrames, frameSize);
    up_plgWarpPool1 = std::make_shared<Pool<ImgFrame>>(poolFrames, frameSize);
    up_plgWarpPool2 = std::make_shared<Pool<ImgFrame>>(poolFrames, outFrameSize);

    allocatedIntermediateFrameSize = frameSize;
    allocatedOutputFrameSize = outFrameSize;
    allocated = true;
    return {true, ""};
}

void Align::extractCalibrationData(const dai::ImgTransformation& inputTransformation,
                                   const dai::ImgTransformation& alignToTransformation,
                                   int inputWidth,
                                   int inputHeight) {
    if(calibrationSet) return;

    auto inputDistortionCoefficients = inputTransformation.getDistortionCoefficients();
    auto alignDistortionCoefficients = alignToTransformation.getDistortionCoefficients();

    depthSourceIntrinsics = inputTransformation.getIntrinsicMatrix();
    alignSourceIntrinsics = alignToTransformation.getIntrinsicMatrix();

    std::array<std::array<float, 3>, 3> inputToAlignRotation;
    std::array<float, 3> translationVec;
    try {
        inputToAlignRotation = inputTransformation.getRotationMatrixTo(alignToTransformation);
        translationVec = inputTransformation.getTranslationVectorTo(alignToTransformation, false, LengthUnit::MILLIMETER);
    } catch(const std::exception& e) {
        throw std::runtime_error(
            fmt::format("Target camera sockets are not set for extrinsics. Align node requires input and alignTo messages to carry complete transformation "
                        "extrinsics. Input message toCameraSocket: {}, alignTo message toCameraSocket: {}. Underlying error: {}",
                        toString(inputTransformation.getExtrinsics().toCameraSocket),
                        toString(alignToTransformation.getExtrinsics().toCameraSocket),
                        e.what()));
    }

    auto cv_M1 = arrayToCvMat(depthSourceIntrinsics);
    auto cv_M2 = arrayToCvMat(alignSourceIntrinsics);

    cv::Mat cv_d1;
    if(inputDistortionCoefficients.size() != 0) {
        cv_d1 = vecToCvMat(1, inputDistortionCoefficients.size(), CV_32FC1, inputDistortionCoefficients);
    }
    cv::Mat cv_dNone;
    if(alignDistortionCoefficients.size() != 0) {
        cv_dNone = vecToCvMat(1, alignDistortionCoefficients.size(), CV_32FC1, std::vector<float>(alignDistortionCoefficients.size(), 0.0f));
    }

    auto cv_R = arrayToCvMat(inputToAlignRotation);
    auto cv_T = vecToCvMat(1, 3, CV_32FC1, {translationVec[0], translationVec[1], translationVec[2]});

    const float translationNorm =
        std::sqrt(translationVec[0] * translationVec[0] + translationVec[1] * translationVec[1] + translationVec[2] * translationVec[2]);
    const bool degenerateStereoTransform = translationNorm <= 1e-6f;
    const std::array<std::array<float, 3>, 3> identityRotation = {{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}};
    const bool rotationOnlyDegenerateTransform = degenerateStereoTransform && !matrix::mateq(inputToAlignRotation, identityRotation, 1e-4f);

    cv::Mat cv_R1, cv_R2;
    if(degenerateStereoTransform) {
        cv_R1 = cv::Mat::eye(3, 3, CV_32FC1);
        cv_R2 = cv::Mat::eye(3, 3, CV_32FC1);
    } else {
        cv::Size imageSize = cv::Size(inputWidth, inputHeight);
        std::tie(cv_R1, cv_R2) = computeRectificationMatrices(cv_M1, cv_d1, cv_M2, cv_dNone, imageSize, cv_R, cv_T);
    }

    cv::Mat cv_newR;
    cv::Mat cv_newT = cv::Mat::zeros(3, 1, CV_32FC1);
    if(rotationOnlyDegenerateTransform) {
        cv_newR = cv_R.clone();
    } else if(degenerateStereoTransform) {
        cv_newR = cv::Mat::eye(3, 3, CV_32FC1);
    } else {
        cv_newR = cv_R2 * (cv_R * cv_R1.t());
        cv_newT = cv_R2 * cv_T.t();
    }

    for(size_t i = 0; i < 3; i++) {
        for(size_t j = 0; j < 3; j++) {
            this->depthToAlignExtrinsics[i][j] = cv_newR.at<float>(i, j);
        }
    }
    for(size_t i = 0; i < 3; i++) {
        this->depthToAlignExtrinsics[i][3] = cv_newT.at<float>(i);
    }

    cv::Mat cv_combinedExtrinsics = cv::Mat::eye(4, 4, CV_32F);
    cv_newR.copyTo(cv_combinedExtrinsics(cv::Rect(0, 0, 3, 3)));
    cv_newT.copyTo(cv_combinedExtrinsics(cv::Rect(3, 0, 1, 3)));

    // Rotate the depth to the RGB frame
    cv::Mat cv_R_back;
    if(rotationOnlyDegenerateTransform) {
        cv_R_back = cv_R.clone();
    } else {
        cv_R_back = cv_R2.t();
    }

    const auto [outputWidth, outputHeight] = alignOutputSize();
    const auto bounds =
        detail::rectifiedBounds({inputWidth, inputHeight}, cv_M1, cv_d1, cv_R1, {outputWidth, outputHeight}, cv_M2, cv_R_back.t());
    auto cv_meshSize = bounds.size();
    if(inputFrameType == ImgFrame::Type::NV12 || inputFrameType == ImgFrame::Type::YUV420p) {
        cv_meshSize.width += cv_meshSize.width % 2;
        cv_meshSize.height += cv_meshSize.height % 2;
    }
    auto cv_targetCamMatrix = cv_M1.clone();
    cv_targetCamMatrix.at<float>(0, 2) -= bounds.x;
    cv_targetCamMatrix.at<float>(1, 2) -= bounds.y;
    // Shift the rectified projection, not the source-image coordinates in
    // map_x_1. Rectified X/Y can correspond to both source axes after rotation.
    auto shiftedProjection = cv_targetCamMatrix.clone();
    if(staticDepthPlane != 0) {
        // Preserve subpixel translation; rounding here is magnified at the output resolution.
        shiftedProjection.at<float>(0, 2) += depthToAlignExtrinsics[0][3] * depthSourceIntrinsics[0][0] / staticDepthPlane;
        shiftedProjection.at<float>(1, 2) += depthToAlignExtrinsics[1][3] * depthSourceIntrinsics[1][1] / staticDepthPlane;
    }
    cv::initUndistortRectifyMap(cv_M1, cv_d1, cv_R1, shiftedProjection, cv_meshSize, CV_32FC1, map_x_1, map_y_1);

    cv_meshSize = cv::Size(outputWidth, outputHeight);

    cv::initUndistortRectifyMap(cv_targetCamMatrix, cv_dNone, cv_R_back, cv_M2, cv_meshSize, CV_32FC1, map_x_2, map_y_2);

    logger->debug("R = {}", matToString(cv_R));
    logger->debug("T = {}", matToString(cv_T));
    logger->debug("M1 = {}", matToString(cv_M1));
    logger->debug("M2 = {}", matToString(cv_M2));
    logger->debug("R1 = {}", matToString(cv_R1));
    logger->debug("R2 = {}", matToString(cv_R2));
    logger->debug("TARGET_MATRIX = {}", matToString(cv_targetCamMatrix));
    logger->debug("newT = {}", matToString(cv_newT));
    logger->debug("newR = {}", matToString(cv_newR));
    logger->debug("combinedExtrinsics = {}", matToString(cv_combinedExtrinsics));
    logger->debug("R_back = {}", matToString(cv_R_back));

    calibrationSet = true;
}

void Align::buildStage1() {
    // This warms up the dsp so it works afterwards when called.
    // Without this it fails in some cases with:
    // Remote_session_control failed
    // as it has some race conditions with some unknown stuff.
    // Main suspects are the hardware accelerator and fastCV. fastCV and the accelerator already had a race condition,
    // where fastCV had to be configured before the accelerator or the accelerator init would fail.
    //
    // THEORY:
    // As the accelerator init also uses CDSP this is probably the same s**t.
    // fastCV maybe reconfigures / overrides some global CDSP state
    // and that doesn't play nice with other CDSP stuff.
    dai::device::openDspSession(dai_device_dsp_open_align, "dai_device_dsp_open_align");
}

std::shared_ptr<ImgFrame> Align::alignImgFrame(std::shared_ptr<ImgFrame> inputImg, const cv::Scalar borderColor, const cv::InterpolationFlags interpolation) {
    using namespace std::chrono;

    // check maps are not empty
    if(map_x_1.empty() || map_y_1.empty()) {
        throw std::runtime_error("Maps for remap are empty, cannot align image!");
    }

    const ImgFrame::Type frameType = inputImg->getType();
    const bool inputIsDepth = frameType == ImgFrame::Type::RAW16;
    const bool inputIsPlanarRgb = frameType == ImgFrame::Type::RGB888p || frameType == ImgFrame::Type::BGR888p;
    const uint32_t width = inputImg->getWidth();
    const uint32_t height = inputImg->getHeight();
    const auto [transformWidth, transformHeight] = currentInputTransformation.getSize();
    if(width != transformWidth || height != transformHeight) {
        throw std::runtime_error(
            fmt::format("Input frame size {}x{} does not match its transformation size {}x{}", width, height, transformWidth, transformHeight));
    }

    decltype(steady_clock::now()) t1, t2;
    if(PRINT_DEBUG) {
        t1 = steady_clock::now();
    }

    auto depthImgRectified = up_plgWarpPool1->get();
    depthImgRectified->setWidth(map_x_1.cols);
    depthImgRectified->setHeight(map_x_1.rows);
    depthImgRectified->setType(frameType);
    depthImgRectified->fb.stride = depthImgRectified->fb.width * depthImgRectified->getBytesPerPixel();

    depthImgRectified->setBufferMetadataFrom(inputImg);
    depthImgRectified->setInstanceNum(inputImg->getInstanceNum());

    depthImgRectified->fb.p1Offset = 0;
    depthImgRectified->fb.p2Offset = depthImgRectified->fb.stride * depthImgRectified->getHeight();
    depthImgRectified->fb.p3Offset = inputIsPlanarRgb ? 2 * depthImgRectified->fb.p2Offset : depthImgRectified->fb.p2Offset;
    if(frameType == ImgFrame::Type::YUV420p) {
        depthImgRectified->fb.p3Offset += (depthImgRectified->getStride() / 2) * (depthImgRectified->getHeight() / 2);
    }

    auto depthImgRectifiedFrame = depthImgRectified->getFrame();

    // TODO (aljazkonec): Move the remap to GPU for depth
    if(frameType == ImgFrame::Type::NV12 || frameType == ImgFrame::Type::YUV420p) {
        remapFrame(*inputImg, *depthImgRectified, map_x_1, map_y_1, interpolation);
    } else if(inputIsPlanarRgb) {
        remapPlanarRgb(inputImg, depthImgRectified, map_x_1, map_y_1, interpolation, borderColor);
    } else {
        cv::remap(frameForRemap(*inputImg), depthImgRectifiedFrame, map_x_1, map_y_1, interpolation, cv::BORDER_CONSTANT, borderColor);
    }

    if(PRINT_DEBUG) {
        t2 = steady_clock::now();
        auto elapsed = duration_cast<microseconds>(t2 - t1).count() / 1000.f;
        logger->warn("Align step1 took '{}' ms.", elapsed);
        t1 = steady_clock::now();
    }

    auto shiftedOutput = up_plgPoolAlign->get();

    if(PRINT_DEBUG) {
        t1 = steady_clock::now();
    }

    auto warp2Input = depthImgRectified;
    if(inputIsDepth && staticDepthPlane == 0) {
        if(!dspOpened || !dspConfigData) {
            throw std::logic_error("Align DSP must be initialized before aligning depth frames");
        }

        shiftedOutput->setBufferMetadataFrom(inputImg);
        shiftedOutput->setWidth(map_x_1.cols);
        shiftedOutput->setHeight(map_x_1.rows);
        shiftedOutput->setType(frameType);
        shiftedOutput->fb.stride = shiftedOutput->fb.width * shiftedOutput->getBytesPerPixel();
        shiftedOutput->fb.p1Offset = 0;
        shiftedOutput->fb.p2Offset = shiftedOutput->fb.stride * shiftedOutput->getHeight();
        shiftedOutput->fb.p3Offset = shiftedOutput->fb.p2Offset;

        std::shared_ptr<AcceleratorMemory> inDepthImgRectified = AcceleratorMemory::convert(depthImgRectified->data);
        std::shared_ptr<AcceleratorMemory> outShiftedOutput = AcceleratorMemory::convert(shiftedOutput->data);

        auto startDSP = high_resolution_clock::now();
        auto& config = *reinterpret_cast<ImageAlignDspConfig*>(dspConfigData->data());
        config.depthSourceIntrinsics = depthSourceIntrinsics;
        config.alignSourceIntrinsics = alignSourceIntrinsics;
        config.depthToAlignExtrinsics = depthToAlignExtrinsics;
        config.width = shiftedOutput->fb.width;
        config.height = shiftedOutput->fb.height;
        config.stride = shiftedOutput->fb.stride;
        config.bpp = shiftedOutput->getBytesPerPixel();

        const dai_device_buf dspIn0 = dai::device::toDeviceBuf(*inDepthImgRectified), dspConfig = dai::device::toDeviceBuf(*dspConfigData),
                             dspOut = dai::device::toDeviceBuf(*outShiftedOutput);
        int nErr = dai_device_dsp_shift_depth(&dspIn0, &dspConfig, &dspOut);
        if(nErr != 0) {
            logger->error("alignDepthImg failed with code {}", nErr);
        }

        shiftedOutput->data = AcceleratorMemory::convert(outShiftedOutput);
        auto stopDSP = high_resolution_clock::now();
        auto durationDSP = duration_cast<microseconds>(stopDSP - startDSP);
        logger->debug("DSP time: {} ms", durationDSP.count() / 1000.0f);

        warp2Input = shiftedOutput;
    }

    if(PRINT_DEBUG) {
        t2 = steady_clock::now();
        auto elapsed = duration_cast<microseconds>(t2 - t1).count() / 1000.f;
        logger->warn("Align step2 took '{}' ms.", elapsed);
        t1 = steady_clock::now();
    }

    if(PRINT_DEBUG) {
        t1 = steady_clock::now();
    }

    // Warp 2

    if(PRINT_DEBUG) {
        t2 = steady_clock::now();
        auto elapsed = duration_cast<microseconds>(t2 - t1).count() / 1000.f;
        logger->warn("Align output pool took '{}' ms.", elapsed);
        t1 = steady_clock::now();
    }

    auto alignedImg = up_plgWarpPool2->get();

    // Copy timing and identity fields.
    alignedImg->setBufferMetadataFrom(inputImg);
    alignedImg->setInstanceNum(inputImg->getInstanceNum());

    // Set the new frame layout explicitly
    const auto [alignedWidth, alignedHeight] = alignOutputSize();
    alignedImg->setWidth(alignedWidth);
    alignedImg->setHeight(alignedHeight);
    alignedImg->setType(frameType);
    alignedImg->fb.stride = alignedImg->getWidth() * alignedImg->getBytesPerPixel();

    // Reset/recompute plane offsets for the new layout
    alignedImg->fb.p1Offset = 0;
    alignedImg->fb.p2Offset = alignedImg->fb.stride * alignedImg->getHeight();
    alignedImg->fb.p3Offset = inputIsPlanarRgb ? 2 * alignedImg->fb.p2Offset : alignedImg->fb.p2Offset;
    if(frameType == ImgFrame::Type::YUV420p) {
        alignedImg->fb.p3Offset += (alignedImg->getStride() / 2) * (alignedImg->getHeight() / 2);
    }

    // Geometry belongs to the target coordinate system; capture identity above
    // remains with the source. setSourceSize resets the transformation, so set
    // it before restoring the target calibration and output geometry.
    const auto [targetSourceWidth, targetSourceHeight] = currentAlignToTransformation.getSourceSize();
    alignedImg->setSourceSize(targetSourceWidth, targetSourceHeight);
    alignedImg->setTransformation(this->currentAlignToTransformation);
    alignedImg->transformation.setSize(alignedWidth, alignedHeight);
    auto alignedDistortion = alignedImg->transformation.getDistortionCoefficients();
    alignedImg->transformation.setDistortionCoefficients(std::vector<float>(alignedDistortion.size(), 0.0f));

    auto alignedImgFrame = alignedImg->getFrame();

    // TODO (aljazkonec): Move this warp to the GPU, as for Warp 1 above. The maps are static between calibration changes,
    // thus the GPU mesh buffers can stay allocated and only the source and destination frames change for each frame.
    if(frameType == ImgFrame::Type::NV12 || frameType == ImgFrame::Type::YUV420p) {
        remapFrame(*warp2Input, *alignedImg, map_x_2, map_y_2, interpolation);
    } else if(inputIsPlanarRgb) {
        remapPlanarRgb(warp2Input, alignedImg, map_x_2, map_y_2, interpolation, borderColor);
    } else {
        cv::remap(frameForRemap(*warp2Input), alignedImgFrame, map_x_2, map_y_2, interpolation, cv::BORDER_CONSTANT, borderColor);
    }

    if(PRINT_DEBUG) {
        t2 = steady_clock::now();
        auto elapsed = duration_cast<microseconds>(t2 - t1).count() / 1000.f;
        logger->warn("Align step3 took '{}' ms.", elapsed);
    }

    return alignedImg;
}

ImgTransformation Align::extractTransformationFromBuffer(const std::shared_ptr<Buffer>& buffer) {
    DatatypeEnum datatype = classifyInputDatatype(buffer);

    ImgTransformation transform;
    if(isTransformableDatatype(datatype)) {
        auto transformable = std::dynamic_pointer_cast<Transformable>(buffer);
        if(!transformable || !transformable->getTransformation().has_value()) {
            logger->error("Input connected to inputAlignTo does not have a ImgTransformation set, cannot use it as alignTo target!");
            throw std::runtime_error("Missing ImgTransformation on inputAlignTo");
        }

        transform = *transformable->getTransformation();
    } else if(datatype == DatatypeEnum::ImgFrame) {
        auto alignToImg = std::dynamic_pointer_cast<ImgFrame>(buffer);
        transform = alignToImg->transformation;
    } else {
        logger->error("Unsupported datatype on inputAlignTo: {}", (int)datatype);
        throw std::runtime_error("Unsupported datatype on inputAlignTo");
    }
    return transform;
}

std::pair<int, int> Align::alignOutputSize() const {
    const auto [width, height] = currentAlignToTransformation.getSize();
    return {static_cast<int>(width), static_cast<int>(height)};
}

void Align::updateMaps() {
    auto inputWidth = static_cast<int>(this->currentInputTransformation.getSize().first);
    auto inputHeight = static_cast<int>(this->currentInputTransformation.getSize().second);

    try {
        extractCalibrationData(currentInputTransformation, currentAlignToTransformation, inputWidth, inputHeight);
    } catch(const std::exception& e) {
        throw std::runtime_error(e.what());
    }
}

std::shared_ptr<Buffer> Align::alignInputMsg(const std::shared_ptr<Buffer>& inputMsg, bool& warnedAboutDistortion) {
    DatatypeEnum inputType = classifyInputDatatype(inputMsg);

    const auto alignToDistortion = this->currentAlignToTransformation.getDistortionCoefficients();
    bool hasDistortion = std::any_of(alignToDistortion.begin(), alignToDistortion.end(), [](float value) { return std::abs(value) > 0.0f; });

    auto warnAboutDistortion = [&]() {
        if(!warnedAboutDistortion && hasDistortion) {
            logger->warn(
                "The input connected to inputAlignTo is distorted. The aligned image / segmentation mask will still be undistorted, meaning it won't be "
                "perfectly aligned.");
            warnedAboutDistortion = true;
        }
    };

    ImgTransformation undistortedTransform = this->currentAlignToTransformation;
    if(!alignToDistortion.empty()) {
        undistortedTransform.setDistortionCoefficients(std::vector<float>(alignToDistortion.size(), 0.0f));
    }

    switch(inputType) {
        case DatatypeEnum::ImgFrame: {
            auto imgFrameInput = std::dynamic_pointer_cast<ImgFrame>(inputMsg);

            warnAboutDistortion();

            auto interpolation = cv::INTER_NEAREST;
            switch(properties.interpolation) {
                case Interpolation::AUTO:
                case Interpolation::NEAREST_NEIGHBOR:
                    break;
                case Interpolation::BILINEAR:
                    interpolation = cv::INTER_LINEAR;
                    break;
                case Interpolation::BICUBIC:
                    interpolation = cv::INTER_CUBIC;
                    break;
                default:
                    throw std::invalid_argument("Unsupported Align interpolation");
            }
            // Do not mix depth values across discontinuities.
            if(imgFrameInput->getType() == ImgFrame::Type::RAW16) interpolation = cv::INTER_NEAREST;
            auto alignedImg = alignImgFrame(imgFrameInput, cv::Scalar(0, 0, 0), interpolation);
            alignedImg->setBufferMetadataFrom(imgFrameInput);
            alignedImg->setInstanceNum(imgFrameInput->getInstanceNum());
            const auto [outWidth, outHeight] = alignOutputSize();
            alignedImg->setSize(outWidth, outHeight);
            return alignedImg;
        }
        case DatatypeEnum::ImgDetections: {
            std::shared_ptr<ImgDetections> imgDetectionsInput = std::dynamic_pointer_cast<ImgDetections>(inputMsg);
            std::optional<ImgFrame> segMask = imgDetectionsInput->getSegmentationMask();

            // With a mask the detections have to share the undistorted frame of the mask pixels, otherwise the two would not overlap.
            const ImgTransformation& detectionsTransform = segMask ? undistortedTransform : this->currentAlignToTransformation;
            auto alignedImg = std::make_shared<ImgDetections>(imgDetectionsInput->transformTo(detectionsTransform));

            if(segMask) {
                warnAboutDistortion();

                auto alignedSegMask = alignImgFrame(std::make_shared<ImgFrame>(*segMask), cv::Scalar(255, 255, 255), cv::INTER_NEAREST);
                alignedImg->setSegmentationMask(*alignedSegMask);
            }

            return alignedImg;
        }
        case DatatypeEnum::SpatialImgDetections: {
            auto spatialImgDetectionsInput = std::dynamic_pointer_cast<SpatialImgDetections>(inputMsg);
            auto segMask = spatialImgDetectionsInput->getSegmentationMask();

            const ImgTransformation& detectionsTransform = segMask ? undistortedTransform : this->currentAlignToTransformation;
            auto alignedSpatialImgDetections = std::make_shared<SpatialImgDetections>(spatialImgDetectionsInput->transformTo(detectionsTransform));

            if(segMask) {
                warnAboutDistortion();

                auto alignedSegMask = alignImgFrame(std::make_shared<ImgFrame>(*segMask), cv::Scalar(255, 255, 255), cv::INTER_NEAREST);
                alignedSpatialImgDetections->setSegmentationMask(*alignedSegMask);
            }

            return alignedSpatialImgDetections;
        }
        case DatatypeEnum::SegmentationMask: {
            auto segMaskInput = std::dynamic_pointer_cast<SegmentationMask>(inputMsg);
            auto segMaskFrame = segMaskInput->getFrame();

            warnAboutDistortion();

            auto alignedSegMaskFrame = alignImgFrame(std::make_shared<ImgFrame>(segMaskFrame), cv::Scalar(255, 255, 255), cv::INTER_NEAREST);

            std::shared_ptr<SegmentationMask> alignedSegMask = std::make_shared<SegmentationMask>();
            alignedSegMask->setMask(*alignedSegMaskFrame);
            alignedSegMask->setBufferMetadataFrom(segMaskInput);
            alignedSegMask->setTransformation(undistortedTransform);
            alignedSegMask->setLabels(segMaskInput->getLabels());
            return alignedSegMask;
        }
        case DatatypeEnum::AprilTags: {
            auto aprilTagsInput = std::dynamic_pointer_cast<AprilTags>(inputMsg);
            return std::make_shared<AprilTags>(aprilTagsInput->transformTo(this->currentAlignToTransformation));
        }
        case DatatypeEnum::Tracklets: {
            auto trackletsInput = std::dynamic_pointer_cast<Tracklets>(inputMsg);
            return std::make_shared<Tracklets>(trackletsInput->transformTo(this->currentAlignToTransformation));
        }
        case DatatypeEnum::Transformable: {
            auto transformableInput = std::dynamic_pointer_cast<TransformableBuffer>(inputMsg);
            if(transformableInput) {
                return transformableInput->transformTo(this->currentAlignToTransformation);
            }
            break;
        }
        default:
            break;
    }

    throw std::runtime_error(fmt::format("Unsupported datatype for alignment: {}", static_cast<int>(inputType)));
}

void Align::refreshTransformations(std::shared_ptr<Buffer> inputMsg, std::shared_ptr<Buffer> alignToMsg) {
    ImgTransformation newInputTransform = extractTransformationFromBuffer(inputMsg);

    ImgTransformation newAlignToTransform = this->currentAlignToTransformation;
    if(alignToMsg != nullptr) {
        newAlignToTransform = extractTransformationFromBuffer(alignToMsg);
    }

    bool transformationChanged =
        !this->currentAlignToTransformation.isEqualTransformation(newAlignToTransform)
        || !this->currentInputTransformation.isEqualTransformation(newInputTransform);  // possible issue: if new transforms are also default constructed

    const ImgFrame::Type newInputFrameType = extractFrameTypeForWarp(inputMsg);
    const bool frameTypeChanged = newInputFrameType != ImgFrame::Type::NONE && newInputFrameType != this->inputFrameType;

    if(transformationChanged || frameTypeChanged) {
        this->calibrationSet = false;
        this->currentAlignToTransformation = newAlignToTransform;
        this->currentInputTransformation = newInputTransform;
    }
}

void Align::setupRemappingParameters(std::shared_ptr<Buffer>& inputMsg) {
    auto frameForWarp = extractFrameForWarp(inputMsg);

    if(frameForWarp) {
        this->inputFrameType = frameForWarp->getType();
        updateMaps();

        auto [success, msg] = allocatePools(
            map_x_1.cols, map_x_1.rows, currentAlignToTransformation.getSize().first, currentAlignToTransformation.getSize().second, inputFrameType);
        if(!success) {
            throw std::runtime_error(msg);
        }
    }

    calibrationSet = true;
}

void Align::run() {
    auto firstInput = input.get<Buffer>();
    auto firstAlignTo = inputAlignTo.get<Buffer>();
    std::shared_ptr<Buffer> alignToMsg = firstAlignTo;
    dai::device::openDspSession(dai_device_dsp_open_align, "dai_device_dsp_open_align");
    dspOpened = true;
    dspConfigData = AcceleratorMemory::create(sizeof(ImageAlignDspConfig));

    staticDepthPlane = properties.initialConfig.staticDepthPlane;

    auto nextInputMsg = [&]() -> std::shared_ptr<Buffer> {
        if(firstInput) return std::exchange(firstInput, nullptr);
        return input.get<Buffer>();
    };

    auto nextAlignToMsg = [&]() -> std::shared_ptr<Buffer> {
        if(firstAlignTo) return std::exchange(firstAlignTo, nullptr);
        return inputAlignTo.getWaitForMessage() ? inputAlignTo.get<Buffer>() : inputAlignTo.tryGet<Buffer>();
    };

    refreshTransformations(firstInput, alignToMsg);
    setupRemappingParameters(firstInput);

    bool warnedAboutDistortion = false;

    while(mainLoop()) {
        std::shared_ptr<Buffer> inputMsg = nullptr;
        std::shared_ptr<AlignConfig> inConfig = nullptr;

        {
            auto blockEvent = this->inputBlockEvent();

            inputMsg = nextInputMsg();
            alignToMsg = nextAlignToMsg();

            inConfig = inputConfig.getWaitForMessage() ? inputConfig.get<AlignConfig>() : inputConfig.tryGet<AlignConfig>();
        }
        auto tStart = std::chrono::steady_clock::now();

        if(inConfig && inConfig->staticDepthPlane != staticDepthPlane) {
            staticDepthPlane = inConfig->staticDepthPlane;
            calibrationSet = false;
        }

        refreshTransformations(inputMsg, alignToMsg);
        if(!calibrationSet) {
            setupRemappingParameters(inputMsg);
        }

        std::shared_ptr<Buffer> alignedInputMsg = alignInputMsg(inputMsg, warnedAboutDistortion);

        auto tStop = std::chrono::steady_clock::now();
        auto runtime = std::chrono::duration_cast<std::chrono::milliseconds>(tStop - tStart).count();
        logger->trace("Align step took {} ms", runtime);

        {
            auto blockEvent = this->outputBlockEvent();
            outputAligned.send(alignedInputMsg);
            passthroughInput.send(inputMsg);
        }
    }

    dspConfigData.reset();
    dspOpened = false;
}

}  // namespace gate
}  // namespace dai
