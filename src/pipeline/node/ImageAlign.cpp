#include "depthai/pipeline/node/ImageAlign.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <unordered_set>

#include "pipeline/ThreadedNodeImpl.hpp"

#if defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
    #include <opencv2/calib3d.hpp>
    #include <opencv2/imgproc/imgproc.hpp>
#endif

namespace dai {
namespace node {

static constexpr const bool PRINT_DEBUG = false;

ImageAlignProperties& ImageAlign::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

ImageAlign& ImageAlign::setOutputSize(int alignWidth, int alignHeight) {
    properties.alignWidth = alignWidth;
    properties.alignHeight = alignHeight;
    return *this;
}
ImageAlign& ImageAlign::setOutKeepAspectRatio(bool keep) {
    properties.outKeepAspectRatio = keep;
    return *this;
}

ImageAlign& ImageAlign::setInterpolation(Interpolation interp) {
    properties.interpolation = interp;
    return *this;
}

ImageAlign& ImageAlign::setNumShaves(int numShaves) {
    properties.numShaves = numShaves;
    return *this;
}

ImageAlign& ImageAlign::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
    return *this;
}

void ImageAlign::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

/**
 * Check if the node is set to run on host
 */
bool ImageAlign::runOnHost() const {
    return runOnHostVar;
}

#if !defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
void ImageAlign::run() {
    throw std::runtime_error("ImageAlign node requires OpenCV support to run. Please enable OpenCV support in your build configuration.");
}
#else  // DEPTHAI_HAVE_OPENCV_SUPPORT

namespace {

cv::Mat vecToCvMat(int rows, int cols, int type, const std::vector<float>& orig) {
    cv::Mat cvMat = cv::Mat(rows, cols, type);
    // An empty vector (e.g. no distortion coefficients) gives an empty cv::Mat with null data
    if(!orig.empty()) {
        memcpy(cvMat.data, orig.data(), orig.size() * sizeof(orig[0]));
    }
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

// Keep both viewing cones in the intermediate image before applying the depth-dependent shift.
cv::Rect rectifiedBounds(cv::Size sourceSize,
                         const cv::Mat& sourceK,
                         const cv::Mat& sourceDistortion,
                         const cv::Mat& sourceRotation,
                         cv::Size targetSize,
                         const cv::Mat& targetK,
                         const cv::Mat& targetRotation) {
    cv::Rect bounds(0, 0, sourceSize.width, sourceSize.height);
    const auto includeBorder = [&](cv::Size size, const cv::Mat& intrinsics, const cv::Mat& distortion, const cv::Mat& rotation) {
        std::vector<cv::Point2f> border, rectified;
        border.reserve(2 * (size.width + size.height + 2));
        for(int x = 0; x <= size.width; ++x) {
            border.emplace_back(x, 0);
            border.emplace_back(x, size.height);
        }
        for(int y = 0; y <= size.height; ++y) {
            border.emplace_back(0, y);
            border.emplace_back(size.width, y);
        }
        cv::undistortPoints(border, rectified, intrinsics, distortion, rotation, sourceK);
        for(const auto& point : rectified) {
            if(!std::isfinite(point.x) || !std::isfinite(point.y)) throw std::runtime_error("ImageAlign: invalid rectified image bounds");
        }
        bounds |= cv::boundingRect(rectified);
    };
    includeBorder(sourceSize, sourceK, sourceDistortion, sourceRotation);
    includeBorder(targetSize, targetK, cv::Mat{}, targetRotation);
    return bounds;
}

std::string matToString(const cv::Mat& mat) {
    std::ostringstream oss;
    oss << mat;
    return oss.str();
}

void shiftDepthImg(const cv::Mat& input, cv::Mat& output, const std::array<int, 65536>& depthLut, bool shiftPositive) {
    const int width = input.cols;
    const int height = input.rows;
    output.create(input.size(), input.type());

    for(int i = 0; i < height; i++) {
        const uint16_t* currentLine = input.ptr<uint16_t>(i);
        uint16_t* alignedLine = output.ptr<uint16_t>(i);

        memset(alignedLine, 0, width * sizeof(uint16_t));

        for(int j = shiftPositive ? width - 1 : 0; j >= 0 && j < width; j += shiftPositive ? -1 : 1) {
            const uint16_t depth = currentLine[j];
            if(depth == 0) continue;
            const int u = j + depthLut[depth];
            for(const int x : {u, u + (shiftPositive ? 1 : -1)}) {
                if(x >= 0 && x < width) alignedLine[x] = depth;
            }
        }
    }
}

}  // namespace

void ImageAlign::run() {
    using namespace std::chrono;
    auto& logger = pimpl->logger;
    logger->info("{} running on {}.", this->getName(), runOnHostVar ? "host" : "device");

    bool calibrationSet = false;
    std::array<std::array<float, 3>, 3> depthSourceIntrinsics;
    std::array<std::array<float, 3>, 3> alignSourceIntrinsics;
    std::array<std::array<float, 4>, 4> depthToAlignExtrinsics;
    std::vector<float> depthDistortionCoefficients;
    ImgTransformation depthSourceTransformation;
    ImgTransformation alignToTransformation;

    dai::CameraBoardSocket alignFrom;
    dai::CameraBoardSocket alignTo;
    ImgFrame::Type inputFrameType;

    int alignWidth;
    int alignHeight;

    cv::Mat map_x_1, map_y_1;
    cv::Mat map_x_2, map_y_2;

    cv::Mat depthImgRectifiedFrame, shiftedOutput;
    std::array<int, 65536> depthShiftLut{};

    std::unordered_set<ImgFrame::Type> hwSupportedFrameTypes = {ImgFrame::Type::YUV420p, ImgFrame::Type::NV12, ImgFrame::Type::GRAY8, ImgFrame::Type::RAW8};
    std::unordered_set<ImgFrame::Type> supportedFrameTypes = hwSupportedFrameTypes;
    supportedFrameTypes.insert(ImgFrame::Type::RAW16);

    std::unordered_map<ImgFrame::Type, float> frameTypeToBpp = {
        {ImgFrame::Type::YUV420p, 1.5f},
        {ImgFrame::Type::NV12, 1.5f},
        {ImgFrame::Type::GRAY8, 1.0f},
        {ImgFrame::Type::RAW8, 1.0f},
        {ImgFrame::Type::RAW16, 2.0f},
    };

    auto extractCalibrationData = [&](int depthWidth, int depthHeight, int alignWidth, int alignHeight) {
        if(calibrationSet) return;

        if(depthDistortionCoefficients.empty()) {
            depthDistortionCoefficients.assign(14, 0.0f);
        }
        const auto alignDistortionCoefficients = alignToTransformation.getDistortionCoefficients();
        const auto depthToAlignRotation = depthSourceTransformation.getRotationMatrixTo(alignToTransformation);
        const auto depthToAlignTranslationArray = depthSourceTransformation.getTranslationVectorTo(alignToTransformation, false, LengthUnit::MILLIMETER);
        const std::vector<float> depthToAlignTranslation(depthToAlignTranslationArray.begin(), depthToAlignTranslationArray.end());

        auto cv_M1 = arrayToCvMat(3, 3, CV_32FC1, depthSourceIntrinsics);
        auto cv_M2 = arrayToCvMat(3, 3, CV_32FC1, alignSourceIntrinsics);

        auto cv_d1 = vecToCvMat(1, depthDistortionCoefficients.size(), CV_32FC1, depthDistortionCoefficients);
        auto cv_dNone = vecToCvMat(
            1, alignDistortionCoefficients.size(), CV_32FC1, std::vector<float>(alignDistortionCoefficients.size(), 0.0f));  // No distortion for aligned frame
        auto cv_R = arrayToCvMat(3, 3, CV_32FC1, depthToAlignRotation);
        auto cv_T = vecToCvMat(1, 3, CV_32FC1, depthToAlignTranslation);

        cv::Mat cv_R1, cv_R2;
        cv::Size imageSize = cv::Size(depthWidth, depthHeight);
        std::tie(cv_R1, cv_R2) = computeRectificationMatrices(cv_M1, cv_d1, cv_M2, cv_dNone, imageSize, cv_R, cv_T);

        // dai::CameraModel cameraModel = dai::CameraModel::Perspective;  // todo

        auto cv_targetCamMatrix = cv_M1.clone();
        const auto bounds = rectifiedBounds(imageSize, cv_M1, cv_d1, cv_R1, {alignWidth, alignHeight}, cv_M2, cv_R2);
        auto rectifiedSize = bounds.size();
        // Chroma planes require even intermediate dimensions for NV12 and YUV420p.
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
                depthToAlignExtrinsics[i][j] = cv_newR.at<float>(i, j);
            }
        }
        for(size_t i = 0; i < 3; i++) {
            depthToAlignExtrinsics[i][3] = cv_newT.at<float>(i);
        }

        const float shift = depthToAlignExtrinsics[0][3] * depthSourceIntrinsics[0][0];
        for(size_t i = 1; i < depthShiftLut.size(); ++i) depthShiftLut[i] = static_cast<int>(shift / static_cast<float>(i) + 0.5f);

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

        calibrationSet = true;
    };

    auto remapNv12 = [&](cv::Mat& inputNV12, cv::Mat& outputNV12, cv::Mat& map_x, cv::Mat& map_y) {
        cv::Mat bgrFrame;
        cv::cvtColor(inputNV12, bgrFrame, cv::COLOR_YUV2BGR_NV12);

        cv::Mat remappedBGR;
        cv::remap(bgrFrame, remappedBGR, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        cv::cvtColor(remappedBGR, outputNV12, cv::COLOR_BGR2YUV_YV12);
    };

    auto remapYuv420 = [&](cv::Mat& inputYUV420, cv::Mat& outputYUV420, cv::Mat& map_x, cv::Mat& map_y) {
        cv::Mat bgrFrame;
        cv::cvtColor(inputYUV420, bgrFrame, cv::COLOR_YUV2BGR_IYUV);

        cv::Mat remappedBGR;
        cv::remap(bgrFrame, remappedBGR, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        cv::cvtColor(remappedBGR, outputYUV420, cv::COLOR_BGR2YUV_I420);
    };

    auto shiftMesh = [](cv::Mat& meshX, int shiftX) { meshX = meshX + cv::Scalar(shiftX); };

    alignWidth = properties.alignWidth;
    alignHeight = properties.alignHeight;
    // bool keepAspectRatio = properties.outKeepAspectRatio;

    bool initialized = false;

    auto latestConfig = initialConfig;

    int previousShiftFactor = 0;

    ImgTransformation inputAlignToTransform;
    ImageAlignInputState previousInputs;
    std::shared_ptr<ImgFrame> inputAlignToImg = nullptr;
    while(mainLoop()) {
        auto tAbsoluteBeginning = steady_clock::now();
        std::shared_ptr<ImgFrame> inputImg = nullptr;
        std::shared_ptr<ImageAlignConfig> inConfig = nullptr;
        bool hasConfig = false;
        {
            auto blockEvent = this->inputBlockEvent();

            inputImg = input.get<ImgFrame>();
            // Non-blocking, so that a lower inputAlignTo frequency doesn't stall the main input. The queue holds a single,
            // non-blocking slot, so this always yields the most recent alignTo frame. Only the very first frame is awaited.
            auto newInputAlignToImg = inputAlignTo.tryGet<ImgFrame>();
            if(newInputAlignToImg) {
                inputAlignToImg = newInputAlignToImg;
            } else if(inputAlignToImg == nullptr) {
                inputAlignToImg = inputAlignTo.get<ImgFrame>();
            }

            const ImageAlignInputState currentInputs{inputImg->transformation,
                                                     inputAlignToImg->transformation,
                                                     {inputImg->getWidth(), inputImg->getHeight()},
                                                     {inputAlignToImg->getWidth(), inputAlignToImg->getHeight()},
                                                     inputImg->getType()};
            if(currentInputs.differsFrom(previousInputs)) {
                initialized = false;
                calibrationSet = false;
                previousShiftFactor = 0;
            }

            if(!initialized) {
                alignToTransformation = inputAlignToImg->transformation;
                inputAlignToTransform = alignToTransformation;
                const auto alignToDistortion = inputAlignToTransform.getDistortionCoefficients();
                const bool hasDistortion = std::any_of(alignToDistortion.begin(), alignToDistortion.end(), [](float value) { return std::abs(value) > 0.0f; });
                if(hasDistortion) {
                    logger->warn(
                        "The input connected to inputAlignTo is distorted. The aligned image will still be undistorted, meaning it won't be perfectly "
                        "aligned.");
                }

                alignTo = static_cast<CameraBoardSocket>(inputAlignToImg->getInstanceNum());
                // Re-derive on every (re)initialization, the alignTo frame may have changed resolution.
                alignWidth = properties.alignWidth;
                alignHeight = properties.alignHeight;
                if(alignWidth == 0 || alignHeight == 0) {
                    alignWidth = inputAlignToImg->getWidth();
                    alignHeight = inputAlignToImg->getHeight();
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

                previousInputs = currentInputs;
                initialized = true;
            }

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
        auto tGotInput = steady_clock::now();

        if(hasConfig) {
            latestConfig = inConfig;
        }

        bool inputIsDepth = inputImg->getType() == ImgFrame::Type::RAW16;

        uint32_t width = inputImg->getWidth();
        uint32_t height = inputImg->getHeight();

        alignFrom = (dai::CameraBoardSocket)inputImg->getInstanceNum();
        inputFrameType = inputImg->getType();

        depthSourceIntrinsics = inputImg->transformation.getIntrinsicMatrix();
        depthDistortionCoefficients = inputImg->transformation.getDistortionCoefficients();

        if(alignFrom == alignTo) {
            logger->error("Cannot align image to itself (camera socket {}), possible misconfiguration, skipping frame!", (int)alignFrom);
            continue;
        }

        if(!supportedFrameTypes.count(inputFrameType)) {
            logger->error("Frame type '{}' is not supported in ImageAlign.", (int)inputFrameType);  // todo toStr
            continue;
        }
        float inputFrameBpp = frameTypeToBpp[inputFrameType];

        depthSourceTransformation = inputImg->transformation;

        try {
            extractCalibrationData(width, height, alignWidth, alignHeight);
        } catch(const std::exception& e) {
            throw std::runtime_error(e.what());
        }

        auto staticDepthPlane = latestConfig->staticDepthPlane;
        int constantShiftFactor = 0;
        if(staticDepthPlane != 0) {
            constantShiftFactor = roundf((depthToAlignExtrinsics[0][3] * depthSourceIntrinsics[0][0]) / (float)staticDepthPlane);
        }

        int currentShiftFactor = constantShiftFactor - previousShiftFactor;

        if(currentShiftFactor != 0) {
            shiftMesh(map_x_1, -currentShiftFactor);
        }

        previousShiftFactor = constantShiftFactor;

        decltype(steady_clock::now()) t1, t2;
        if(PRINT_DEBUG) {
            t1 = steady_clock::now();
        }

        // warp1
        auto inputFrame = inputImg->getFrame();

        if(inputFrameBpp == 1.5f) {
            if(inputImg->getType() == ImgFrame::Type::NV12) {
                remapNv12(inputFrame, depthImgRectifiedFrame, map_x_1, map_y_1);
            } else if(inputImg->getType() == ImgFrame::Type::YUV420p) {
                remapYuv420(inputFrame, depthImgRectifiedFrame, map_x_1, map_y_1);
            } else {
                logger->error("Unsupported frame type for NV12/YUV420 remapping: {}", (int)inputImg->getType());
            }
        } else {
            cv::remap(inputFrame, depthImgRectifiedFrame, map_x_1, map_y_1, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        }

        if(PRINT_DEBUG) {
            t2 = steady_clock::now();
            auto elapsed = duration_cast<microseconds>(t2 - t1).count() / 1000.f;
            logger->warn("Align step1 took '{}' ms.", elapsed);
        }

        if(PRINT_DEBUG) {
            t1 = steady_clock::now();
        }

        auto warp2InputFrame = depthImgRectifiedFrame;

        if(inputIsDepth && staticDepthPlane == 0) {
            auto startProcessing = high_resolution_clock::now();
            shiftDepthImg(depthImgRectifiedFrame, shiftedOutput, depthShiftLut, depthToAlignExtrinsics[0][3] > 0);

            auto stopProcessing = high_resolution_clock::now();

            auto durationProcessing = duration_cast<microseconds>(stopProcessing - startProcessing);
            logger->debug("ImageAlign depth shift took {} ms.", durationProcessing.count() / 1000.0f);

            warp2InputFrame = shiftedOutput;
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
        alignedImg->setData(std::vector<uint8_t>(static_cast<size_t>(alignWidth * alignHeight * inputFrameBpp)));
        if(PRINT_DEBUG) {
            t2 = steady_clock::now();
            auto elapsed = duration_cast<microseconds>(t2 - t1).count() / 1000.f;
            logger->warn("Align output pool took '{}' ms.", elapsed);
            t1 = steady_clock::now();
        }

        // manually set metadata
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

        auto alignedImgFrame = alignedImg->getFrame();
        if(inputFrameBpp == 1.5f) {
            if(alignedImg->getType() == ImgFrame::Type::NV12) {
                remapNv12(warp2InputFrame, alignedImgFrame, map_x_2, map_y_2);
            } else if(alignedImg->getType() == ImgFrame::Type::YUV420p) {
                remapYuv420(warp2InputFrame, alignedImgFrame, map_x_2, map_y_2);
            } else {
                logger->error("Unsupported frame type for NV12/YUV420 remapping: {}", (int)alignedImg->getType());
            }
        } else {
            cv::remap(warp2InputFrame, alignedImgFrame, map_x_2, map_y_2, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        }
        if(PRINT_DEBUG) {
            t2 = steady_clock::now();
            auto elapsed = duration_cast<microseconds>(t2 - t1).count() / 1000.f;
            logger->warn("Align step3 took '{}' ms.", elapsed);
        }

        alignedImg->setInstanceNum((uint32_t)alignTo);
        alignedImg->setBufferMetadataFrom(inputImg);
        alignedImg->transformation = inputAlignToTransform;
        const auto alignToDistortion = inputAlignToTransform.getDistortionCoefficients();
        alignedImg->transformation.setDistortionCoefficients(std::vector<float>(alignToDistortion.size(), 0.0f));

        auto tProcessed = steady_clock::now();
        {
            auto blockEvent = this->outputBlockEvent();
            outputAligned.send(alignedImg);
            passthroughInput.send(inputImg);
        }
        auto tAbsoluteEnd = steady_clock::now();
        this->logTiming(logger, tAbsoluteBeginning, tGotInput, tProcessed, tAbsoluteEnd);
    }
}

#endif  // DEPTHAI_HAVE_OPENCV_SUPPORT

}  // namespace node
}  // namespace dai
