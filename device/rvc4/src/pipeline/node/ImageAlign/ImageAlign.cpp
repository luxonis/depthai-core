#include "ImageAlign.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <unordered_set>
#include <vector>

#include "depthai/common/DepthUnit.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/pipeline/datatype/ImageAlignConfig.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/utility/matrixOps.hpp"
#include "hexagon/ImageAlignDsp/shared/ImageAlignDspConfig.h"
#include "pipeline/NodeTiming.hpp"
#include "pipeline/utilities/Alignment/RectifiedBounds.hpp"
#include "utilities/ImageRemap.hpp"
#include "vendor/DeviceError.hpp"
#include "vendor/dai_device_dsp.h"

namespace dai {
namespace gate {

namespace {

bool isDsxSizeSupported(int inputWidth, int inputHeight, int outputWidth, int outputHeight) {
    const auto supportedAxis = [](std::int64_t input, std::int64_t output) {
        // DSX accepts identity or a reduction between 1.25x and 4x.
        return input > 0 && output > 0 && (input == output || (input * 4 >= output * 5 && input <= output * 4));
    };
    return supportedAxis(inputWidth, outputWidth) && supportedAxis(inputHeight, outputHeight);
}

bool isIdentityIntrinsics(const std::array<std::array<float, 3>, 3>& matrix, float eps = 1e-6f) {
    for(size_t row = 0; row < 3; ++row) {
        for(size_t col = 0; col < 3; ++col) {
            const float expected = (row == col) ? 1.0f : 0.0f;
            if(std::abs(matrix[row][col] - expected) > eps) {
                return false;
            }
        }
    }
    return true;
}

// The focal lengths are negative in frames that the camera flips or mirrors (Camera image orientation)
bool hasValidIntrinsics(const std::array<std::array<float, 3>, 3>& matrix) {
    return !isIdentityIntrinsics(matrix) && std::isfinite(matrix[0][0]) && std::isfinite(matrix[1][1]) && matrix[0][0] != 0.0f && matrix[1][1] != 0.0f
           && std::isfinite(matrix[2][2]) && matrix[2][2] != 0.0f;
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

}  // namespace

const bool ImageAlign::PRINT_DEBUG = false;

std::pair<bool, std::string> ImageAlign::allocatePools(
    int width, int height, int alignWidth, int alignHeight, ImgFrame::Type inputFrameType, uint32_t warp1MinSize, uint32_t warp2MinSize) {
    if(allocated) return {true, ""};

    uint32_t poolFrames = properties.numFramesPool;

    float bpp = 1.0f;
    switch(inputFrameType) {
        case ImgFrame::Type::YUV420p:
        case ImgFrame::Type::NV12:
            bpp = 1.5f;
            break;
        case ImgFrame::Type::RGB888p:
        case ImgFrame::Type::BGR888p:
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
            return {false, "Unsupported frame type"};
    }

    uint32_t frameSize = std::max(static_cast<uint32_t>(roundf(width * height * bpp)), warp1MinSize);
    up_plgPoolImageAlign = std::make_shared<Pool<ImgFrame>>(poolFrames, frameSize);
    up_plgWarpPool1 = std::make_shared<Pool<ImgFrame>>(poolFrames, frameSize);

    uint32_t outFrameSize = std::max(static_cast<uint32_t>(roundf(alignWidth * alignHeight * bpp)), warp2MinSize);
    up_plgWarpPool2 = std::make_shared<Pool<ImgFrame>>(poolFrames, outFrameSize);

    allocated = true;
    return {true, ""};
}

void ImageAlign::extractCalibrationData(int depthWidth, int depthHeight) {
    if(depthDistortionCoefficients.empty()) {
        depthDistortionCoefficients.assign(14, 0.0f);
    }
    const auto& inputTransformation = depthSourceTransformation;
    const auto alignDistortionCoefficients = alignToTransformation.getDistortionCoefficients();

    if(!inputTransformation.getExtrinsics().hasCompatibleCoordinateSystem(alignToTransformation.getExtrinsics())) {
        throw std::runtime_error("Cannot align ImgTransformations expressed relative to different target coordinate systems");
    }

    std::array<std::array<float, 3>, 3> inputToAlignRotation;
    std::array<float, 3> translationVec;

    bool fallbackToCalibHandler = false;
    if(inputTransformation.getExtrinsics().toCameraSocket == dai::CameraBoardSocket::AUTO
       || alignToTransformation.getExtrinsics().toCameraSocket == dai::CameraBoardSocket::AUTO) {
        fallbackToCalibHandler = true;
    } else {
        try {
            inputToAlignRotation = inputTransformation.getRotationMatrixTo(alignToTransformation);
            translationVec = inputTransformation.getTranslationVectorTo(alignToTransformation, false, LengthUnit::MILLIMETER);
        } catch(const std::runtime_error& ex) {
            logger->error("Error extracting calibration data: {}. Falling back to default calibration data.", ex.what());
            fallbackToCalibHandler = true;
        }
    }

    if(fallbackToCalibHandler) {
        calibHandler = getParentPipeline().getCalibrationData();
        inputToAlignRotation = matrix::vectorMatrixToMatrix3x3(calibHandler.getCameraRotationMatrix(alignFrom, alignTo));
        auto tempVector = calibHandler.getCameraTranslationVector(alignFrom, alignTo, false);
        translationVec = {tempVector[0] * 10, tempVector[1] * 10, tempVector[2] * 10};  // convert from cm to mm
    }

    const std::vector<float> depthToAlignTranslation(translationVec.begin(), translationVec.end());

    auto cv_M1 = arrayToCvMat(3, 3, CV_32FC1, depthSourceIntrinsics);
    auto cv_M2 = arrayToCvMat(3, 3, CV_32FC1, alignSourceIntrinsics);

    cv::Mat cv_d1;
    if(depthDistortionCoefficients.size() != 0) {
        cv_d1 = vecToCvMat(1, depthDistortionCoefficients.size(), CV_32FC1, depthDistortionCoefficients);
    }
    cv::Mat cv_dNone;
    if(alignDistortionCoefficients.size() != 0) {
        cv_dNone = vecToCvMat(1, alignDistortionCoefficients.size(), CV_32FC1, std::vector<float>(alignDistortionCoefficients.size(), 0.0f));
    }

    auto cv_R = arrayToCvMat(3, 3, CV_32FC1, inputToAlignRotation);
    auto cv_T = vecToCvMat(1, 3, CV_32FC1, depthToAlignTranslation);
    cv::Mat cv_R1, cv_R2;
    cv::Size imageSize = cv::Size(depthWidth, depthHeight);
    std::tie(cv_R1, cv_R2) = computeRectificationMatrices(cv_M1, cv_d1, cv_M2, cv_dNone, imageSize, cv_R, cv_T);

    dai::CameraModel cameraModel = dai::CameraModel::Perspective;  // todo

    auto cv_targetCamMatrix = cv_M1.clone();
    const auto bounds = detail::rectifiedBounds(imageSize, cv_M1, cv_d1, cv_R1, {alignWidth, alignHeight}, cv_M2, cv_R2, logger.get());
    auto rectifiedSize = bounds.size();
    if(inputFrameType == ImgFrame::Type::NV12 || inputFrameType == ImgFrame::Type::YUV420p) {
        rectifiedSize.width += rectifiedSize.width % 2;
        rectifiedSize.height += rectifiedSize.height % 2;
    }
    cv_targetCamMatrix.at<float>(0, 2) -= bounds.x;
    cv_targetCamMatrix.at<float>(1, 2) -= bounds.y;
    auto cv_meshSize = rectifiedSize;

    cv::initUndistortRectifyMap(cv_M1, cv_d1, cv_R1, cv_targetCamMatrix, cv_meshSize, CV_32FC1, map_x_1, map_y_1);

    cv::Mat cv_newR = cv_R2 * (cv_R * cv_R1.t());
    cv::Mat cv_newT = cv_R2 * cv_T.t();

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
    cv::Mat cv_R_back = cv_R2.t();

    cv_meshSize = cv::Size(alignWidth, alignHeight);

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
}

void ImageAlign::shiftMesh(cv::Mat& meshX, int shiftX) {
    meshX = meshX + cv::Scalar(shiftX);
}

void ImageAlign::initialize(const std::shared_ptr<ImgFrame>& inputImg,
                            const std::shared_ptr<ImgFrame>& inputAlignToImg,
                            ImgTransformation& inputAlignToTransform,
                            bool& hwSupportedFrameType,
                            bool& warpInited) {
    alignToTransformation = inputAlignToImg->transformation;
    inputAlignToTransform = alignToTransformation;
    const auto alignToDistortion = inputAlignToTransform.getDistortionCoefficients();
    const bool hasDistortion = std::any_of(alignToDistortion.begin(), alignToDistortion.end(), [](float value) { return std::abs(value) > 0.0f; });
    if(hasDistortion) {
        logger->warn("The input connected to inputAlignTo is distorted. The aligned image will still be undistorted, meaning it won't be perfectly aligned.");
    }

    alignTo = static_cast<CameraBoardSocket>(inputAlignToImg->getInstanceNum());
    const int previousAlignWidth = alignWidth;
    const int previousAlignHeight = alignHeight;
    alignWidth = properties.alignWidth;
    alignHeight = properties.alignHeight;
    if(alignWidth == 0 || alignHeight == 0) {
        alignWidth = inputAlignToImg->getWidth();
        alignHeight = inputAlignToImg->getHeight();
    }
    if(allocated && (alignWidth != previousAlignWidth || alignHeight != previousAlignHeight)) {
        allocated = false;
        if(warpInited) {
            warp1.Close();
            warp2.Close();
            warpInited = false;
        }
    }

    auto alignTransformForIntrinsics = inputAlignToTransform;
    auto [alignTransformWidth, alignTransformHeight] = alignTransformForIntrinsics.getSize();
    if(static_cast<int>(alignTransformWidth) != alignWidth || static_cast<int>(alignTransformHeight) != alignHeight) {
        float scaleX = static_cast<float>(alignWidth) / static_cast<float>(alignTransformWidth);
        float scaleY = static_cast<float>(alignHeight) / static_cast<float>(alignTransformHeight);
        alignTransformForIntrinsics.addScale(scaleX, scaleY);
        alignTransformForIntrinsics.setSize(alignWidth, alignHeight);
    }

    alignSourceIntrinsics = alignTransformForIntrinsics.getIntrinsicMatrix();
    inputAlignToTransform = alignTransformForIntrinsics;

    const auto previousRectifiedSize = map_x_1.size();
    extractCalibrationData(inputImg->getWidth(), inputImg->getHeight());
    if(allocated && map_x_1.size() != previousRectifiedSize) {
        allocated = false;
        if(warpInited) {
            warp1.Close();
            warp2.Close();
            warpInited = false;
        }
    }

    if(hwSupportedFrameType
       && (!isDsxSizeSupported(inputImg->getWidth(), inputImg->getHeight(), map_x_1.cols, map_x_1.rows)
           || !isDsxSizeSupported(map_x_1.cols, map_x_1.rows, alignWidth, alignHeight))) {
        hwSupportedFrameType = false;
        if(warpInited) {
            warp1.Close();
            warp2.Close();
            warpInited = false;
        }
        allocated = false;
        logger->debug("ImageAlign dimensions are unsupported by the hardware accelerator; using CPU remapping.");
    }

    if(warpInited && hwSupportedFrameType) {
        warp1.SetWarpMesh(map_x_1, map_y_1);
        warp2.SetWarpMesh(map_x_2, map_y_2);
    }

    if(!warpInited && hwSupportedFrameType) {
        allocated = false;  // Rebuild pools with the accelerator's buffer requirements when switching back from CPU.
        warpInited = true;
        dai::WarpProperties warp1Config;
        warp1Config.outputWidth = map_x_1.cols;
        warp1Config.outputHeight = map_x_1.rows;
        warp1Config.meshWidth = map_x_1.cols;
        warp1Config.meshHeight = map_x_1.rows;
        warp1Config.interpolation = dai::Interpolation::BILINEAR;

        int warpErr = warp1.Init(warp1Config, inputImg->getWidth(), inputImg->getHeight(), inputFrameType, inputFrameType);
        if(warpErr) {
            throw std::runtime_error(fmt::format("Failed initializing left warp node: {}!", warpErr));
        }
        warp1.SetWarpMesh(map_x_1, map_y_1);

        dai::WarpProperties warp2Config;
        warp2Config.outputWidth = alignWidth;
        warp2Config.outputHeight = alignHeight;
        warp2Config.meshWidth = alignWidth;
        warp2Config.meshHeight = alignHeight;
        warp2Config.interpolation = dai::Interpolation::BILINEAR;

        warpErr = warp2.Init(warp2Config, map_x_1.cols, map_x_1.rows, inputFrameType, inputFrameType);
        if(warpErr) {
            throw std::runtime_error(fmt::format("Failed initializing right warp node: {}!", warpErr));
        }
        warp2.SetWarpMesh(map_x_2, map_y_2);
    }

    auto [success, msg] = allocatePools(map_x_1.cols,
                                        map_x_1.rows,
                                        alignWidth,
                                        alignHeight,
                                        inputFrameType,
                                        hwSupportedFrameType ? warp1.getMinOutImgSize() : 0,
                                        hwSupportedFrameType ? warp2.getMinOutImgSize() : 0);
    if(!success) {
        logger->error(msg);
        throw std::runtime_error(msg);
    }
}

void ImageAlign::buildStage1() {
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

void ImageAlign::run() {
    using namespace std::chrono;
    logger->info("{} running on {}.", this->getName(), runOnHost() ? "host" : "device");

    auto pipeline = getParentPipeline();

    if(pipeline.isCalibrationDataAvailable()) {
        logger->debug("Calibration data available. Using it as an extrinsics fallback.");
        calibHandler = pipeline.getCalibrationData();
    } else {
        throw std::runtime_error("No calibration data available.");
    }

    this->alignWidth = properties.alignWidth;
    this->alignHeight = properties.alignHeight;
    this->keepAspectRatio = properties.outKeepAspectRatio;

    std::unordered_set<ImgFrame::Type> hwSupportedFrameTypes = {
        ImgFrame::Type::YUV420p, ImgFrame::Type::NV12, ImgFrame::Type::GRAY8, ImgFrame::Type::RAW8};  // TODO (aljaz): not true
    std::unordered_set<ImgFrame::Type> supportedFrameTypes = hwSupportedFrameTypes;
    supportedFrameTypes.insert(ImgFrame::Type::RAW16);

    auto latestConfig = this->initialConfig;

    int previousShiftFactor = 0;

    dai::device::openDspSession(dai_device_dsp_open_align, "dai_device_dsp_open_align");

    std::shared_ptr<AcceleratorMemory> inConfigData = AcceleratorMemory::create(sizeof(ImageAlignDspConfig));

    bool warpInited = false;
    bool hwSupportedFrameType = false;
    ImgTransformation inputAlignToTransform;
    ImageAlignInputState previousInputs;
    std::shared_ptr<ImgFrame> inputAlignToImg;

    while(mainLoop()) {
        std::shared_ptr<ImgFrame> inputImg = nullptr;
        std::shared_ptr<ImageAlignConfig> inConfig = nullptr;
        bool hasConfig = false;
        {
            auto blockEvent = this->inputBlockEvent();

            inputImg = input.get<ImgFrame>();
            auto newInputAlignToImg = inputAlignTo.tryGet<ImgFrame>();
            if(newInputAlignToImg) {
                inputAlignToImg = newInputAlignToImg;
            } else if(!inputAlignToImg) {
                inputAlignToImg = inputAlignTo.get<ImgFrame>();
            }

            if(!hasValidIntrinsics(inputImg->transformation.getIntrinsicMatrix())
               || !hasValidIntrinsics(inputAlignToImg->transformation.getIntrinsicMatrix())) {
                throw std::runtime_error("ImageAlign requires valid intrinsics on both input and inputAlignTo frames.");
            }

            hasConfig = false;

            if(inputConfig.getWaitForMessage()) {
                logger->trace("Receiving ImageAlign config message!");
                inConfig = inputConfig.get<ImageAlignConfig>();
                hasConfig = true;
            } else {
                inConfig = inputConfig.tryGet<ImageAlignConfig>();
                if(inConfig != nullptr) {
                    hasConfig = true;
                }
            }
        }
        if(hasConfig) {
            latestConfig = inConfig;
        }

        bool inputIsDepth = inputImg->getType() == ImgFrame::Type::RAW16;

        alignFrom = (dai::CameraBoardSocket)inputImg->getInstanceNum();
        inputFrameType = inputImg->getType();

        depthSourceIntrinsics = inputImg->transformation.getIntrinsicMatrix();
        depthDistortionCoefficients = inputImg->transformation.getDistortionCoefficients();
        depthSourceTransformation = inputImg->transformation;

        const ImageAlignInputState currentInputs{inputImg->transformation,
                                                 inputAlignToImg->transformation,
                                                 {inputImg->getWidth(), inputImg->getHeight()},
                                                 {inputAlignToImg->getWidth(), inputAlignToImg->getHeight()},
                                                 inputFrameType};
        const bool inputConfigurationChanged =
            previousInputs.depthResolution != currentInputs.depthResolution || previousInputs.depthType != currentInputs.depthType;
        if(inputConfigurationChanged && allocated) {
            allocated = false;
            if(warpInited) {
                warp1.Close();
                warp2.Close();
                warpInited = false;
            }
        }

        if(currentInputs.differsFrom(previousInputs)) {
            hwSupportedFrameType = hwSupportedFrameTypes.count(inputFrameType);
            initialize(inputImg, inputAlignToImg, inputAlignToTransform, hwSupportedFrameType, warpInited);
            previousInputs = currentInputs;
            previousShiftFactor = 0;
        }

        if(alignFrom == static_cast<CameraBoardSocket>(inputAlignToImg->getInstanceNum())) {
            logger->error("Cannot align image to itself (camera socket {}), possible misconfiguration, skipping frame!", (int)alignFrom);
            continue;
        }

        if(!supportedFrameTypes.count(inputFrameType)) {
            logger->error("Frame type '{}' is not supported in ImageAlign.", (int)inputFrameType);  // todo toStr
            continue;
        }

        auto staticDepthPlane = latestConfig->staticDepthPlane;
        int constantShiftFactor = 0;
        if(staticDepthPlane != 0) {
            constantShiftFactor = roundf((depthToAlignExtrinsics[0][3] * depthSourceIntrinsics[0][0]) / (float)staticDepthPlane);
        }

        int currentShiftFactor = constantShiftFactor - previousShiftFactor;

        if(currentShiftFactor != 0) {
            shiftMesh(map_x_1, -currentShiftFactor);
            if(warpInited && hwSupportedFrameType) {
                warp1.SetWarpMesh(map_x_1, map_y_1);
            }
        }

        previousShiftFactor = constantShiftFactor;

        auto tAbsoluteBeginning = steady_clock::now();
        auto tGotInput = tAbsoluteBeginning;
        decltype(steady_clock::now()) t1, t2;
        if(PRINT_DEBUG) {
            t1 = steady_clock::now();
        }

        // warp1
        auto depthImgRectified = up_plgWarpPool1->get();

        depthImgRectified->setMetadata(*inputImg);
        depthImgRectified->setWidth(map_x_1.cols);
        depthImgRectified->setHeight(map_x_1.rows);
        depthImgRectified->setType(inputImg->getType());
        depthImgRectified->fb.stride = depthImgRectified->fb.width * depthImgRectified->getBytesPerPixel();
        depthImgRectified->fb.p1Offset = 0;
        depthImgRectified->fb.p2Offset = 0;
        depthImgRectified->fb.p3Offset = 0;
        if(inputFrameType == ImgFrame::Type::NV12 || inputFrameType == ImgFrame::Type::YUV420p) {
            depthImgRectified->fb.p2Offset = depthImgRectified->fb.stride * depthImgRectified->fb.height;
            depthImgRectified->fb.p3Offset = depthImgRectified->fb.p2Offset;
            if(inputFrameType == ImgFrame::Type::YUV420p) {
                depthImgRectified->fb.p3Offset += (depthImgRectified->fb.stride / 2) * (depthImgRectified->fb.height / 2);
            }
        }

        // todo each step could run on its own thread for max throughput
        if(hwSupportedFrameType) {
            warp1.Run(inputImg, depthImgRectified);
        } else {
            remapFrame(*inputImg, *depthImgRectified, map_x_1, map_y_1);
        }

        if(PRINT_DEBUG) {
            t2 = steady_clock::now();
            auto elapsed = duration_cast<microseconds>(t2 - t1).count() / 1000.f;
            logger->warn("Align step1 took '{}' ms.", elapsed);
        }

        auto shiftedOutput = up_plgPoolImageAlign->get();

        if(PRINT_DEBUG) {
            t1 = steady_clock::now();
        }

        auto warp2Input = depthImgRectified;

        if(inputIsDepth && staticDepthPlane == 0) {
            shiftedOutput->setMetadata(*inputImg);
            shiftedOutput->setWidth(map_x_1.cols);
            shiftedOutput->setHeight(map_x_1.rows);
            shiftedOutput->setType(inputImg->getType());
            shiftedOutput->fb.stride = shiftedOutput->fb.width * shiftedOutput->getBytesPerPixel();

            std::shared_ptr<AcceleratorMemory> inDepthImgRectified = AcceleratorMemory::convert(depthImgRectified->data);
            std::shared_ptr<AcceleratorMemory> outShiftedOutput = AcceleratorMemory::convert(shiftedOutput->data);

            auto startDSP = high_resolution_clock::now();

            auto& config = *reinterpret_cast<ImageAlignDspConfig*>(inConfigData->data());

            config.depthSourceIntrinsics = depthSourceIntrinsics;
            config.alignSourceIntrinsics = alignSourceIntrinsics;
            config.depthToAlignExtrinsics = depthToAlignExtrinsics;
            config.width = shiftedOutput->fb.width;
            config.height = shiftedOutput->fb.height;
            config.stride = shiftedOutput->fb.stride;
            config.bpp = shiftedOutput->getBytesPerPixel();

            int nErr = 0;
            const dai_device_buf dspIn0 = dai::device::toDeviceBuf(*inDepthImgRectified), dspConfig = dai::device::toDeviceBuf(*inConfigData),
                                 dspOut = dai::device::toDeviceBuf(*outShiftedOutput);
            nErr = dai_device_dsp_shift_depth(&dspIn0, &dspConfig, &dspOut);

            if(nErr != 0) {
                logger->error("alignDepthImg failed with code {}", nErr);
            }

            shiftedOutput->data = AcceleratorMemory::convert(outShiftedOutput);

            auto stopDSP = high_resolution_clock::now();

            auto durationDSP = duration_cast<microseconds>(stopDSP - startDSP);
            logger->debug("ImageAlign depth shift took {} ms.", durationDSP.count() / 1000.0f);

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
        auto alignedImg = up_plgWarpPool2->get();
        if(PRINT_DEBUG) {
            t2 = steady_clock::now();
            auto elapsed = duration_cast<microseconds>(t2 - t1).count() / 1000.f;
            logger->warn("Align output pool took '{}' ms.", elapsed);
            t1 = steady_clock::now();
        }

        // manually set metadata as the true metadata is a combination of inputImg and inputAlignToImg
        alignedImg->cam = inputImg->cam;
        alignedImg->category = inputImg->category;
        alignedImg->event = inputImg->event;
        alignedImg->sourceFb = inputAlignToImg->sourceFb;
        alignedImg->setWidth(alignWidth);
        alignedImg->setHeight(alignHeight);
        alignedImg->setType(inputImg->getType());
        alignedImg->fb.stride = alignedImg->fb.width * alignedImg->getBytesPerPixel();
        alignedImg->fb.p1Offset = 0;
        alignedImg->fb.p2Offset = 0;
        alignedImg->fb.p3Offset = 0;
        if(alignedImg->getType() == ImgFrame::Type::NV12) {
            alignedImg->fb.p2Offset = alignedImg->fb.stride * alignedImg->fb.height;
            alignedImg->fb.p3Offset = alignedImg->fb.p2Offset;
        } else if(alignedImg->getType() == ImgFrame::Type::YUV420p) {
            alignedImg->fb.p2Offset = alignedImg->fb.stride * alignedImg->fb.height;
            alignedImg->fb.p3Offset = alignedImg->fb.p2Offset + (alignedImg->fb.stride / 2) * (alignedImg->fb.height / 2);
        }

        if(hwSupportedFrameType) {
            warp2.Run(warp2Input, alignedImg);
        } else {
            remapFrame(*warp2Input, *alignedImg, map_x_2, map_y_2);
        }

        if(PRINT_DEBUG) {
            t2 = steady_clock::now();
            auto elapsed = duration_cast<microseconds>(t2 - t1).count() / 1000.f;
            logger->warn("Align step3 took '{}' ms.", elapsed);
        }

        alignedImg->setInstanceNum((uint32_t)alignTo);

        alignedImg->setBufferMetadataFrom(inputImg);

        alignedImg->transformation = inputAlignToTransform;

        auto tProcessed = steady_clock::now();
        auto alignedDistortion = alignedImg->transformation.getDistortionCoefficients();
        alignedImg->transformation.setDistortionCoefficients(std::vector<float>(alignedDistortion.size(), 0.0f));
        {
            auto blockEvent = this->outputBlockEvent();
            outputAligned.send(alignedImg);
            passthroughInput.send(inputImg);
        }
        auto tAbsoluteEnd = steady_clock::now();
        dai::gate::logTiming(logger, this->getName(), tAbsoluteBeginning, tGotInput, tProcessed, tAbsoluteEnd);
    }
    if(warpInited) {
        warp1.Close();
        warp2.Close();
    }
}

}  // namespace gate
}  // namespace dai
