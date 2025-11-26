#include "depthai/pipeline/node/ImageAlign.hpp"

#include <sstream>
#include <unordered_set>

#include "depthai/pipeline/Pipeline.hpp"
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

template <typename T>
std::vector<T> flatten(const std::vector<std::vector<T> >& orig) {
    std::vector<T> ret;
    for(const auto& v : orig) ret.insert(ret.end(), v.begin(), v.end());
    return ret;
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

int shiftDepthImg(std::shared_ptr<dai::ImgFrame> inVec,
                  std::shared_ptr<dai::ImgFrame> outVec,
                  std::array<std::array<float, 3>, 3> depthSourceIntrinsics,
                  std::array<std::array<float, 4>, 4> depthToAlignExtrinsics) {
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

void ImageAlign::run() {
    using namespace std::chrono;
    auto& logger = pimpl->logger;

    dai::CalibrationHandler calibHandler;

    bool calibrationSet = false;
    std::array<std::array<float, 3>, 3> depthSourceIntrinsics;
    std::array<std::array<float, 3>, 3> alignSourceIntrinsics;
    std::array<std::array<float, 4>, 4> depthToAlignExtrinsics;

    dai::CameraBoardSocket alignFrom;
    dai::CameraBoardSocket alignTo;
    ImgFrame::Type inputFrameType;

    int alignWidth;
    int alignHeight;

    cv::Mat map_x_1, map_y_1;
    cv::Mat map_x_2, map_y_2;

    bool allocated = false;
    uint32_t frameSize = 0;
    uint32_t outFrameSize = 0;

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

    auto allocatePools = [&](int width, int height, int alignWidth, int alignHeight, float inputFrameBpp) -> std::pair<bool, std::string> {
        if(allocated) return {true, ""};

        float bpp = inputFrameBpp;

        frameSize = roundf(width * height * bpp);

        outFrameSize = roundf(alignWidth * alignHeight * bpp);

        allocated = true;
        return {true, ""};
    };

    auto extractCalibrationData = [&](int depthWidth, int depthHeight, int alignWidth, int alignHeight) {
        if(calibrationSet) return;

        auto depthDistortionCoefficients = std::vector<float>(14, 0.0f);
        auto alignDistortionCoefficients = calibHandler.getDistortionCoefficients(alignTo);

        auto depthToAlignRotation = calibHandler.getCameraRotationMatrix(alignFrom, alignTo);
        auto depthToAlignTranslation = calibHandler.getCameraTranslationVector(alignFrom, alignTo, false);

        for(auto& t : depthToAlignTranslation) {
            t *= 10;  // convert to mm
        }

        auto cv_M1 = arrayToCvMat(3, 3, CV_32FC1, depthSourceIntrinsics);
        auto cv_M2 = arrayToCvMat(3, 3, CV_32FC1, alignSourceIntrinsics);

        auto cv_dNone = vecToCvMat(1, depthDistortionCoefficients.size(), CV_32FC1, depthDistortionCoefficients);
        auto cv_d2 = vecToCvMat(1, alignDistortionCoefficients.size(), CV_32FC1, alignDistortionCoefficients);

        auto cv_R = vecToCvMat(3, 3, CV_32FC1, depthToAlignRotation);
        auto cv_T = vecToCvMat(1, 3, CV_32FC1, depthToAlignTranslation);

        cv::Mat cv_R1, cv_R2;
        cv::Size imageSize = cv::Size(depthWidth, depthHeight);
        std::tie(cv_R1, cv_R2) = computeRectificationMatrices(cv_M1, cv_dNone, cv_M2, cv_d2, imageSize, cv_R, cv_T);

        // dai::CameraModel cameraModel = dai::CameraModel::Perspective;  // todo

        auto cv_targetCamMatrix = cv_M1.clone();
        auto cv_meshSize = cv::Size(depthWidth, depthHeight);

        cv::initUndistortRectifyMap(cv_M1, cv_dNone, cv_R1, cv_targetCamMatrix, cv_meshSize, CV_32FC1, map_x_1, map_y_1);

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

    auto pipeline = getParentPipeline();

    try {
        calibHandler = pipeline.getDefaultDevice()->getCalibration();
    } catch(const std::exception& e) {
        logger->error("Failed to get calibration data: {}", e.what());
    }

    alignWidth = properties.alignWidth;
    alignHeight = properties.alignHeight;
    // bool keepAspectRatio = properties.outKeepAspectRatio;

    bool initialized = false;

    auto latestConfig = initialConfig;

    int previousShiftFactor = 0;

    ImgTransformation inputAlignToTransform;
    ImgFrame inputAlignToImgFrame;
    uint32_t currentEepromId = getParentPipeline().getEepromId();

    while(mainLoop()) {
        std::shared_ptr<ImgFrame> inputImg = nullptr;
        std::shared_ptr<ImageAlignConfig> inConfig = nullptr;
        bool hasConfig = false;
        {
            auto blockEvent = this->inputBlockEvent();

            inputImg = input.get<ImgFrame>();

            if(!initialized) {
                initialized = true;

                auto inputAlignToImg = inputAlignTo.get<ImgFrame>();

                inputAlignToImgFrame = *inputAlignToImg;

                inputAlignToTransform = inputAlignToImg->transformation;

                alignSourceIntrinsics = inputAlignToImg->transformation.getIntrinsicMatrix();

                alignTo = static_cast<CameraBoardSocket>(inputAlignToImg->getInstanceNum());
                if(alignWidth == 0 || alignHeight == 0) {
                    alignWidth = inputAlignToImg->getWidth();
                    alignHeight = inputAlignToImg->getHeight();
                }
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

        if(hasConfig) {
            latestConfig = inConfig;
        }

        bool inputIsDepth = inputImg->getType() == ImgFrame::Type::RAW16;

        uint32_t width = inputImg->getWidth();
        uint32_t height = inputImg->getHeight();

        alignFrom = (dai::CameraBoardSocket)inputImg->getInstanceNum();
        inputFrameType = inputImg->getType();

        depthSourceIntrinsics = inputImg->transformation.getIntrinsicMatrix();

        if(alignFrom == alignTo) {
            logger->error("Cannot align image to itself (camera socket {}), possible misconfiguration, skipping frame!", (int)alignFrom);
            continue;
        }

        if(!supportedFrameTypes.count(inputFrameType)) {
            logger->error("Frame type '{}' is not supported in ImageAlign.", (int)inputFrameType);  // todo toStr
            continue;
        }
        float inputFrameBpp = frameTypeToBpp[inputFrameType];

        auto [success, msg] = allocatePools(width, height, alignWidth, alignHeight, inputFrameBpp);
        if(!success) {
            logger->error(msg);
            throw std::runtime_error(msg);
        }

        uint32_t latestEepromId = getParentPipeline().getEepromId();

        if(latestEepromId > currentEepromId) {
            logger->debug("EEPROM data changed (ID: {} -> {}), reconfiguring ...", currentEepromId, latestEepromId);
            calibrationSet = false;
            calibHandler = pipeline.getCalibrationData();
            currentEepromId = latestEepromId;
        }

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

        decltype(steady_clock::now()) t1, t2, tStart, tStop;
        tStart = steady_clock::now();
        if(PRINT_DEBUG) {
            t1 = steady_clock::now();
        }

        // warp1
        auto depthImgRectified = std::make_shared<ImgFrame>();
        depthImgRectified->setData(std::vector<uint8_t>(frameSize));

        depthImgRectified->setMetadata(*inputImg);
        depthImgRectified->setWidth(inputImg->getWidth());
        depthImgRectified->setHeight(inputImg->getHeight());
        depthImgRectified->setType(inputImg->getType());
        depthImgRectified->fb.stride = depthImgRectified->fb.width * depthImgRectified->getBytesPerPixel();

        auto inputFrame = inputImg->getFrame();
        auto depthImgRectifiedFrame = depthImgRectified->getFrame();

        if(inputFrameBpp == 1.5f) {
            auto inputFrameCopy = inputFrame.clone();
            if(depthImgRectified->getType() == ImgFrame::Type::NV12) {
                remapNv12(inputFrameCopy, depthImgRectifiedFrame, map_x_1, map_y_1);
            } else if(depthImgRectified->getType() == ImgFrame::Type::YUV420p) {
                remapYuv420(inputFrameCopy, depthImgRectifiedFrame, map_x_1, map_y_1);
            } else {
                logger->error("Unsupported frame type for NV12/YUV420 remapping: {}", (int)depthImgRectified->getType());
            }
        } else {
            cv::remap(inputFrame, depthImgRectifiedFrame, map_x_1, map_y_1, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
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

        if(inputIsDepth && staticDepthPlane == 0) {
            shiftedOutput->setMetadata(*inputImg);
            shiftedOutput->setWidth(inputImg->getWidth());
            shiftedOutput->setHeight(inputImg->getHeight());
            shiftedOutput->setType(inputImg->getType());
            shiftedOutput->fb.stride = shiftedOutput->fb.width * shiftedOutput->getBytesPerPixel();

            auto startProcessing = high_resolution_clock::now();

            int nErr = 0;
            nErr = shiftDepthImg(depthImgRectified, shiftedOutput, depthSourceIntrinsics, depthToAlignExtrinsics);

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

        alignedImg->setMetadata(inputAlignToImgFrame);
        alignedImg->setWidth(alignWidth);
        alignedImg->setHeight(alignHeight);
        alignedImg->setType(inputImg->getType());
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
        } else {
            cv::remap(warp2InputFrame, alignedImgFrame, map_x_2, map_y_2, cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        }
        if(PRINT_DEBUG) {
            t2 = steady_clock::now();
            auto elapsed = duration_cast<microseconds>(t2 - t1).count() / 1000.f;
            logger->warn("Align step3 took '{}' ms.", elapsed);
        }

        alignedImg->setInstanceNum((uint32_t)alignTo);

        alignedImg->setTimestamp(inputImg->getTimestamp());
        alignedImg->setTimestampDevice(inputImg->getTimestampDevice());
        alignedImg->setTimestampSystem(inputImg->getTimestampSystem());
        alignedImg->setSequenceNum(inputImg->getSequenceNum());

        alignedImg->transformation = inputAlignToTransform;

        tStop = steady_clock::now();
        auto runtime = duration_cast<milliseconds>(tStop - tStart).count();

        logger->trace("ImageAlign took {} ms", runtime);

        alignedImg->transformation.setDistortionCoefficients({});
        {
            auto blockEvent = this->outputBlockEvent();
            outputAligned.send(alignedImg);
            passthroughInput.send(inputImg);
        }
    }
}

#endif  // DEPTHAI_HAVE_OPENCV_SUPPORT

}  // namespace node
}  // namespace dai
