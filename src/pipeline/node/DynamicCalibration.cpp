#include "depthai/pipeline/node/DynamicCalibration.hpp"

#include <opencv2/opencv.hpp>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/utility/matrixOps.hpp"
#include "spdlog/spdlog.h"
#include "utility/Logging.hpp"

namespace dai {
namespace node {

DynamicCalibration::Properties& DynamicCalibration::getProperties() {
    return properties;
}

void DynamicCalibration::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}
/**
 * Check if the node is set to run on host
 */
bool DynamicCalibration::runOnHost() const {
    return runOnHostVar;
}

void DynamicCalibration::buildInternal() {
    sync->out.link(syncInput);
    sync->setRunOnHost(true);
}

std::pair<std::shared_ptr<dcl::CameraCalibrationHandle>, std::shared_ptr<dcl::CameraCalibrationHandle>> DclUtils::convertDaiCalibrationToDcl(
    const CalibrationHandler& currentCalibration,
    const CameraBoardSocket boardSocketA,
    const CameraBoardSocket boardSocketB,
    const int width,
    const int height) {
    // clang-format off
    std::shared_ptr<dcl::CameraCalibrationHandle> calibA = DclUtils::createDclCalibration(
        currentCalibration.getCameraIntrinsics(boardSocketA, width, height),
        currentCalibration.getDistortionCoefficients(boardSocketA),
	{
	    {1.0f, 0.0f, 0.0f},
	    {0.0f, 1.0f, 0.0f},
	    {0.0f, 0.0f, 1.0f}
	},
	{0.0f, 0.0f, 0.0f}
    );
    std::shared_ptr<dcl::CameraCalibrationHandle> calibB = DclUtils::createDclCalibration(
        currentCalibration.getCameraIntrinsics(boardSocketB, width, height),
        currentCalibration.getDistortionCoefficients(boardSocketB),
	currentCalibration.getCameraRotationMatrix(boardSocketA, boardSocketB),
	currentCalibration.getCameraTranslationVector(boardSocketA, boardSocketB, false)
    );
    // clang-format on
    return std::make_pair(calibA, calibB);
}

std::shared_ptr<dcl::CameraCalibrationHandle> DclUtils::createDclCalibration(const std::vector<std::vector<float>> cameraMatrix,
                                                                             const std::vector<float> distortionCoefficients,
                                                                             const std::vector<std::vector<float>> rotationMatrix,
                                                                             const std::vector<float> translationVector) {
    dcl::scalar_t cameraMatrixArr[9];
    dcl::scalar_t distortion[14] = {0};
    dcl::scalar_t rvec[3];
    dcl::scalar_t tvec[3];

    // Convert cameraMatrix
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            cameraMatrixArr[i * 3 + j] = static_cast<dcl::scalar_t>(cameraMatrix[i][j]);
        }
    }

    // Convert distortion
    for(size_t i = 0; i < distortionCoefficients.size() && i < 14; ++i) {
        distortion[i] = static_cast<dcl::scalar_t>(distortionCoefficients[i]);
    }

    // Convert rotation to vector
    std::vector<float> rvecVec = matrix::rotationMatrixToVector(rotationMatrix);
    for(int i = 0; i < 3; ++i) {
        rvec[i] = static_cast<dcl::scalar_t>(rvecVec[i]);
    }

    for(int i = 0; i < 3; ++i) {
        tvec[i] = static_cast<dcl::scalar_t>(translationVector[i] / 100.0f);  // Convert to m
    }

    return std::make_shared<dcl::CameraCalibrationHandle>(rvec, tvec, cameraMatrixArr, distortion);
}

void DclUtils::convertDclCalibrationToDai(CalibrationHandler& calibHandler,
                                          const std::shared_ptr<const dcl::CameraCalibrationHandle> daiCalibrationA,
                                          const std::shared_ptr<const dcl::CameraCalibrationHandle> daiCalibrationB,
                                          const CameraBoardSocket socketSrc,
                                          const CameraBoardSocket socketDest,
                                          const int width,
                                          const int height) {
    dcl::scalar_t tvecA[3];
    daiCalibrationA->getTvec(tvecA);
    dcl::scalar_t rvecA[3];
    daiCalibrationA->getRvec(rvecA);

    constexpr dcl::scalar_t threshold = 1e-10;

    // clang-format off
    auto isNonZero = [&](const auto& vec) {
      return std::abs(vec[0]) > threshold || std::abs(vec[1]) > threshold || std::abs(vec[2]) > threshold;
    };
    // clang-format on

    if(isNonZero(tvecA) || isNonZero(rvecA)) {
        throw std::runtime_error("Extrinsics of the left camera are not zero within the allowed threshold.");
    }

    dcl::scalar_t distortionA[14];
    daiCalibrationA->getDistortion(distortionA);

    dcl::scalar_t cameraMatrixA[9];
    daiCalibrationA->getCameraMatrix(cameraMatrixA);
    // clang-format off
    std::vector<std::vector<float>> matA = {
        {static_cast<float>(cameraMatrixA[0]), static_cast<float>(cameraMatrixA[1]), static_cast<float>(cameraMatrixA[2])},
        {static_cast<float>(cameraMatrixA[3]), static_cast<float>(cameraMatrixA[4]), static_cast<float>(cameraMatrixA[5])},
        {static_cast<float>(cameraMatrixA[6]), static_cast<float>(cameraMatrixA[7]), static_cast<float>(cameraMatrixA[8])}
    };
    // clang-format on

    dcl::scalar_t distortionB[14];
    daiCalibrationB->getDistortion(distortionB);

    dcl::scalar_t cameraMatrixB[9];
    daiCalibrationB->getCameraMatrix(cameraMatrixB);
    // clang-format off
    std::vector<std::vector<float>> matB = {
        {static_cast<float>(cameraMatrixB[0]), static_cast<float>(cameraMatrixB[1]), static_cast<float>(cameraMatrixB[2])},
        {static_cast<float>(cameraMatrixB[3]), static_cast<float>(cameraMatrixB[4]), static_cast<float>(cameraMatrixB[5])},
        {static_cast<float>(cameraMatrixB[6]), static_cast<float>(cameraMatrixB[7]), static_cast<float>(cameraMatrixB[8])}
    };
    // clang-format on

    dcl::scalar_t tvecB[3];
    daiCalibrationB->getTvec(tvecB);
    auto translation = std::vector<float>(tvecB, tvecB + 3);
    for(auto& val : translation) {
        val *= 100.0f;
    }
    dcl::scalar_t rvecB[3];
    daiCalibrationB->getRvec(rvecB);
    auto rotationMatrix = matrix::rvecToRotationMatrix(rvecB);

    calibHandler.setCameraIntrinsics(socketSrc, matA, width, height);
    calibHandler.setCameraIntrinsics(socketDest, matB, width, height);
    calibHandler.setDistortionCoefficients(socketSrc, std::vector<float>(distortionA, distortionA + 14));
    calibHandler.setDistortionCoefficients(socketDest, std::vector<float>(distortionB, distortionB + 14));
    auto specTranslation = calibHandler.getCameraTranslationVector(socketSrc, socketDest, true);
    calibHandler.setCameraExtrinsics(socketSrc, socketDest, rotationMatrix, translation, specTranslation);
}

dcl::ImageData DclUtils::cvMatToImageData(const cv::Mat& mat) {
    if(mat.empty()) {
        throw std::runtime_error("cv::Mat is empty");
    }

    dcl::ImageData img;
    img.width = static_cast<unsigned int>(mat.cols);
    img.height = static_cast<unsigned int>(mat.rows);
    img.data.assign(mat.data, mat.data + mat.total() * mat.elemSize());

    int type = mat.type();
    switch(type) {
        case CV_8UC1:
            img.format = dcl::DCL_8UC1;
            break;
        case CV_8UC3:
            img.format = dcl::DCL_8UC3;
            break;
        default:
            throw std::runtime_error("Unsupported cv::Mat type: " + std::to_string(type));
    }

    return img;
}

dai::CalibrationQuality DynamicCalibration::calibQualityfromDCL(const dcl::CalibrationQuality& src) {
    dai::CalibrationQuality quality;

    if(src.calibrationQuality.has_value()) {
        CalibrationQuality::Data data = {
                {
                    src.calibrationQuality->rotationChange[0],
                    src.calibrationQuality->rotationChange[1],
                    src.calibrationQuality->rotationChange[2],
                },
            src.calibrationQuality->epipolarErrorChange,
            src.calibrationQuality->depthDistanceDifference,
        };
        quality.data = std::optional<CalibrationQuality::Data>(data);
    } else {
        quality.data = std::nullopt;
    }
    return quality;
}

void DynamicCalibration::setCalibration(CalibrationHandler& handler) {
    device->setCalibration(handler);
    auto [calibA, calibB] = DclUtils::convertDaiCalibrationToDcl(handler, daiSocketA, daiSocketB, width, height);
    dynCalibImpl->setNewCalibration(deviceName, socketA, calibA->getCalibration());
    dynCalibImpl->setNewCalibration(deviceName, socketB, calibB->getCalibration());
}

DynamicCalibration::ErrorCode DynamicCalibration::runQualityCheck(const bool force) {
    dcl::PerformanceMode performanceMode = force ? dcl::PerformanceMode::SKIP_CHECKS : properties.initialConfig.performanceMode;
    auto dclResult = dynCalibImpl->checkCalibration(dcDevice, socketA, socketB, performanceMode);

    if(!dclResult.passed()) {
        auto result = std::make_shared<CalibrationQuality>();
        qualityOutput.send(result);
        return DynamicCalibration::ErrorCode::QUALITY_CHECK_FAILED;
    }

    auto result = std::make_shared<CalibrationQuality>(calibQualityfromDCL(dclResult.value));

    qualityOutput.send(result);

    return DynamicCalibration::ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::runCalibration(const dai::CalibrationHandler& currentHandler, const bool force) {
    dcl::PerformanceMode performanceMode = force ? dcl::PerformanceMode::SKIP_CHECKS : properties.initialConfig.performanceMode;

    auto dclResult = dynCalibImpl->findNewCalibration(dcDevice, socketA, socketB, performanceMode);
    if(!dclResult.passed()) {
        auto result = std::make_shared<DynamicCalibrationResult>(std::nullopt, std::nullopt, dclResult.errorMessage());

        calibrationOutput.send(result);
        return DynamicCalibration::ErrorCode::CALIBRATION_FAILED;
    }

    auto daiCalibrationA = dclResult.value.first;
    auto daiCalibrationB = dclResult.value.second;
    // clang-format off
    auto newCalibrationHandler = currentHandler;
    dai::node::DclUtils::convertDclCalibrationToDai(
	newCalibrationHandler, daiCalibrationA, daiCalibrationB, daiSocketA, daiSocketB, width, height);

    auto result = std::make_shared<DynamicCalibrationResult>(
        std::optional<CalibrationHandler>(newCalibrationHandler),
	std::nullopt,
	dclResult.errorMessage());
    // clang-format on

    calibrationOutput.send(result);

    return DynamicCalibration::ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::runLoadImage(const bool blocking) {
    std::shared_ptr<dai::MessageGroup> inSyncGroup;
    if(!blocking) {
        inSyncGroup = syncInput.tryGet<dai::MessageGroup>();
    } else {
        inSyncGroup = syncInput.get<dai::MessageGroup>();
    }
    if(!inSyncGroup) {
        return DynamicCalibration::ErrorCode::EMPTY_IMAGE_QUEUE;
    }
    auto leftFrame = inSyncGroup->get<dai::ImgFrame>(leftInputName);
    auto rightFrame = inSyncGroup->get<dai::ImgFrame>(rightInputName);

    if(!leftFrame || !rightFrame) {
        return DynamicCalibration::ErrorCode::MISSING_IMAGE;
    }

    dcl::timestamp_t timestamp = leftFrame->getTimestamp().time_since_epoch().count();
    auto leftCvFrame = leftFrame->getCvFrame();
    auto rightCvFrame = rightFrame->getCvFrame();

    dynCalibImpl->loadStereoImagePair(
        DclUtils::cvMatToImageData(leftCvFrame), DclUtils::cvMatToImageData(rightCvFrame), deviceName, socketA, socketB, timestamp);

    return DynamicCalibration::ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::computeCoverage() {
    auto resultCoverage = dynCalibImpl->computeCoverage(sensorA, sensorB, properties.initialConfig.performanceMode);

    if(!resultCoverage.passed()) {
        throw std::runtime_error("Coverage check failed!");
    }

    auto& coverage = resultCoverage.value;

    auto coverageResult = std::make_shared<CoverageData>(coverage);
    coverageOutput.send(coverageResult);

    return DynamicCalibration::ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::initializePipeline(const std::shared_ptr<dai::Device> daiDevice) {
    auto initialConfig = configInput.tryGet<dai::DynamicCalibrationConfig>();
    if(initialConfig) {
        properties.initialConfig = *initialConfig;
    }

    auto inSyncGroup = syncInput.get<dai::MessageGroup>();
    if(!inSyncGroup) {
        return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
    }
    auto leftFrame = inSyncGroup->get<dai::ImgFrame>(leftInputName);
    auto rightFrame = inSyncGroup->get<dai::ImgFrame>(rightInputName);
    if(!leftFrame || !rightFrame) {
        return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
    }

    width = leftFrame->getWidth();
    height = rightFrame->getHeight();

    daiSocketA = static_cast<CameraBoardSocket>(leftFrame->instanceNum);
    daiSocketB = static_cast<CameraBoardSocket>(rightFrame->instanceNum);

    socketA = static_cast<dcl::socket_t>(daiSocketA);
    socketB = static_cast<dcl::socket_t>(daiSocketB);

    calibrationHandler = daiDevice->getCalibration();

    auto [calibA, calibB] = DclUtils::convertDaiCalibrationToDcl(calibrationHandler, daiSocketA, daiSocketB, width, height);

    // set up the dynamic calibration
    deviceName = daiDevice->getDeviceId();
    dcDevice = dynCalibImpl->addDevice(deviceName);
    const dcl::resolution_t resolution = {static_cast<unsigned>(width), static_cast<unsigned>(height)};
    sensorA = std::make_shared<dcl::CameraSensorHandle>(calibA, resolution);
    sensorB = std::make_shared<dcl::CameraSensorHandle>(calibB, resolution);
    dynCalibImpl->addSensor(deviceName, sensorA, socketA);
    dynCalibImpl->addSensor(deviceName, sensorB, socketB);

    return DynamicCalibration::ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::evaluateCommand(const std::shared_ptr<DynamicCalibrationCommand> command) {
    if(auto recalibrateCommand = std::dynamic_pointer_cast<RecalibrateCommand>(command)) {
        return runCalibration(calibrationHandler, recalibrateCommand->force);
    }
    if(auto calibrationQualityCommand = std::dynamic_pointer_cast<CalibrationQualityCommand>(command)) {
        return runQualityCheck(calibrationQualityCommand->force);
    }
    if(auto startCalibrationCommand = std::dynamic_pointer_cast<StartRecalibrationCommand>(command)) {
        recalibrationRunning = true;
        return ErrorCode::OK;
    }
    if(auto loadImageCommand = std::dynamic_pointer_cast<LoadImageCommand>(command)) {
        auto error = runLoadImage(true);
        computeCoverage();
        return error;
    }
    if(auto applyCalibrationCommand = std::dynamic_pointer_cast<ApplyCalibrationCommand>(command)) {
        calibrationHandler = applyCalibrationCommand->calibration;
        setCalibration(calibrationHandler);
        return ErrorCode::OK;
    }
    return ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::doWork(std::chrono::steady_clock::time_point& previousLoadingTime) {
    ErrorCode errorCode = ErrorCode::OK;
    auto calibrationCommand = commandInput.tryGet<DynamicCalibrationCommand>();
    if(calibrationCommand) {
        errorCode = evaluateCommand(calibrationCommand);
    }

    if(!recalibrationRunning) {
        return errorCode;
    }

    auto loadError = ErrorCode::OK;
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<float> elapsed = now - previousLoadingTime;
    if(elapsed.count() > properties.initialConfig.loadImagePeriod) {
        loadError = runLoadImage();
        computeCoverage();
        if(loadError == ErrorCode::OK) {
            previousLoadingTime = std::chrono::steady_clock::now();
            auto reacalibrationResult = runCalibration(calibrationHandler);
            if(reacalibrationResult == DynamicCalibration::ErrorCode::OK) {
                recalibrationRunning = false;
            } else {
                return reacalibrationResult;
            }
        }
    }
    if(errorCode == ErrorCode::OK) {
        return loadError;
    }
    return errorCode;
}

// clang-format off
DynamicCalibration::ErrorCode DynamicCalibration::doWorkContinuous(
    std::chrono::steady_clock::time_point& previousCalibrationTime,
    std::chrono::steady_clock::time_point& previousLoadingTime)
{
    auto now = std::chrono::steady_clock::now();

    std::chrono::duration<float> elapsedCalibration = now - previousCalibrationTime;
    if(elapsedCalibration.count() > properties.initialConfig.calibrationPeriod) {
        auto calibrationError = runCalibration(calibrationHandler);
	previousCalibrationTime = std::chrono::steady_clock::now();
	return calibrationError;
    }
    std::chrono::duration<float> elapsedLoading = now - previousLoadingTime;
    if (elapsedLoading.count() > properties.initialConfig.loadImagePeriod) {
        auto loadImageError = runLoadImage();
	// do we want to return the coverage in the continuous mode? -> computeCoverage();
	previousLoadingTime = std::chrono::steady_clock::now();
	return loadImageError;
    }
    return ErrorCode::OK;
}
// clang-format on

void DynamicCalibration::run() {
    if(!device) {
        logger::error("Dynamic calibration node does not have access to any device.");
        return;
    }

    logger::info("DynamicCalibration node is running");
    std::runtime_error("DynamicCalibration node is running!");
    auto previousLoadingTimeFloat = std::chrono::steady_clock::now() + std::chrono::duration<float>(properties.initialConfig.calibrationPeriod);
    auto previousLoadingTime = std::chrono::time_point_cast<std::chrono::steady_clock::duration>(previousLoadingTimeFloat);
    initializePipeline(device);
    if(!(properties.initialConfig.recalibrationMode == dai::DynamicCalibrationConfig::RecalibrationMode::CONTINUOUS)) {
        // non-Continuous mode
        while(isRunning()) {
            doWork(previousLoadingTime);
        }
    } else {
        // Continuous mode
        auto previousCalibrationTime = std::chrono::steady_clock::now();
        while(isRunning()) {
            doWorkContinuous(previousCalibrationTime, previousLoadingTime);
        }
    }
}

}  // namespace node
}  // namespace dai
