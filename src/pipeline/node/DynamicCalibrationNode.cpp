#include "depthai/pipeline/node/DynamicCalibrationNode.hpp"

#include <opencv2/opencv.hpp>
#include <pipeline/ThreadedNodeImpl.hpp>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/utility/matrixOps.hpp"
#include "pipeline/node/DynamicCalibrationUtils.hpp"
#include "spdlog/async_logger.h"
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
    logger = pimpl->logger;
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

#define DCL_DISTORTION_SIZE (14)
std::shared_ptr<dcl::CameraCalibrationHandle> DclUtils::createDclCalibration(const std::vector<std::vector<float>> cameraMatrix,
                                                                             const std::vector<float> distortionCoefficients,
                                                                             const std::vector<std::vector<float>> rotationMatrix,
                                                                             const std::vector<float> translationVector) {
    dcl::scalar_t cameraMatrixArr[9];
    dcl::scalar_t distortion[DCL_DISTORTION_SIZE] = {0};
    dcl::scalar_t rvec[3];
    dcl::scalar_t tvec[3];

    // Convert cameraMatrix
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            cameraMatrixArr[i * 3 + j] = static_cast<dcl::scalar_t>(cameraMatrix[i][j]);
        }
    }

    // Convert distortion
    for(size_t i = 0; i < DCL_DISTORTION_SIZE; ++i) {
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
                                          const std::shared_ptr<const dcl::CameraCalibrationHandle> dclCalibrationA,
                                          const std::shared_ptr<const dcl::CameraCalibrationHandle> dclCalibrationB,
                                          const CameraBoardSocket socketSrc,
                                          const CameraBoardSocket socketDest,
                                          const int width,
                                          const int height) {
    dcl::scalar_t tvecA[3];
    dclCalibrationA->getTvec(tvecA);
    dcl::scalar_t rvecA[3];
    dclCalibrationA->getRvec(rvecA);

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
    dclCalibrationA->getDistortion(distortionA);

    dcl::scalar_t cameraMatrixA[9];
    dclCalibrationA->getCameraMatrix(cameraMatrixA);
    // clang-format off
    std::vector<std::vector<float>> matA = {
        {static_cast<float>(cameraMatrixA[0]), static_cast<float>(cameraMatrixA[1]), static_cast<float>(cameraMatrixA[2])},
        {static_cast<float>(cameraMatrixA[3]), static_cast<float>(cameraMatrixA[4]), static_cast<float>(cameraMatrixA[5])},
        {static_cast<float>(cameraMatrixA[6]), static_cast<float>(cameraMatrixA[7]), static_cast<float>(cameraMatrixA[8])}
    };
    // clang-format on

    dcl::scalar_t distortionB[14];
    dclCalibrationB->getDistortion(distortionB);

    dcl::scalar_t cameraMatrixB[9];
    dclCalibrationB->getCameraMatrix(cameraMatrixB);
    // clang-format off
    std::vector<std::vector<float>> matB = {
        {static_cast<float>(cameraMatrixB[0]), static_cast<float>(cameraMatrixB[1]), static_cast<float>(cameraMatrixB[2])},
        {static_cast<float>(cameraMatrixB[3]), static_cast<float>(cameraMatrixB[4]), static_cast<float>(cameraMatrixB[5])},
        {static_cast<float>(cameraMatrixB[6]), static_cast<float>(cameraMatrixB[7]), static_cast<float>(cameraMatrixB[8])}
    };
    // clang-format on

    dcl::scalar_t tvecB[3];
    dclCalibrationB->getTvec(tvecB);
    auto translation = std::vector<float>(tvecB, tvecB + 3);
    for(auto& val : translation) {
        val *= 100.0f;
    }
    dcl::scalar_t rvecB[3];
    dclCalibrationB->getRvec(rvecB);
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

dai::CalibrationQuality DynamicCalibration::calibQualityfromDCL(const dcl::CalibrationDifference& src) {
    dai::CalibrationQuality quality;

    CalibrationQuality::Data data{};
    data.rotationChange[0] = src.rotationChange[0];
    data.rotationChange[1] = src.rotationChange[1];
    data.rotationChange[2] = src.rotationChange[2];
    data.depthErrorDifference = src.depthDistanceDifference;
    data.sampsonErrorCurrent = src.sampsonErrorCurrent;
    data.sampsonErrorNew = src.sampsonErrorNew;
    quality.qualityData = data;  // optional constructed from value
    return quality;
}

void DynamicCalibration::setCalibration(CalibrationHandler& handler) {
    logger->info("Applying calibration to device: {}", deviceName);
    device->setCalibration(handler);
    auto [calibA, calibB] = DclUtils::convertDaiCalibrationToDcl(handler, daiSocketA, daiSocketB, width, height);
    dynCalibImpl->setNewCalibration(deviceName, socketA, calibA->getCalibration());
    dynCalibImpl->setNewCalibration(deviceName, socketB, calibB->getCalibration());
}

DynamicCalibration::ErrorCode DynamicCalibration::runQualityCheck(const bool force) {
    dcl::PerformanceMode pm = force ? dcl::PerformanceMode::SKIP_CHECKS : performanceMode;
    logger->info("Running calibration quality check (force={} mode={})", force, static_cast<int>(pm));

    auto dclResult = dynCalibImpl->checkCalibration(dcDevice, socketA, socketB, pm);

    if(!dclResult.passed()) {
        auto result = std::make_shared<CalibrationQuality>();
        result->info = dclResult.errorMessage();
        logger->info("WARNING: Quality check failed: {}", dclResult.errorMessage());

        qualityOutput.send(result);
        return DynamicCalibration::ErrorCode::QUALITY_CHECK_FAILED;
    }

    auto result = std::make_shared<CalibrationQuality>(calibQualityfromDCL(dclResult.value));
    result->info = dclResult.errorMessage();
    logger->info("Quality check passed.");

    qualityOutput.send(result);

    return DynamicCalibration::ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::runCalibration(const dai::CalibrationHandler& currentHandler, const bool force) {
    dcl::PerformanceMode pm = force ? dcl::PerformanceMode::SKIP_CHECKS : performanceMode;
    logger->info("Running calibration (force={} mode={})", force, static_cast<int>(pm));
    auto dclResult = dynCalibImpl->findNewCalibration(dcDevice, socketA, socketB, pm);
    if(!dclResult.passed()) {
        auto result = std::make_shared<DynamicCalibrationResult>(dclResult.errorMessage());
        logger->info("WARNING: Calibration failed: {}", dclResult.errorMessage());

        calibrationOutput.send(result);
        return DynamicCalibration::ErrorCode::CALIBRATION_FAILED;
    }

    auto dclCalibrationA = dclResult.value.newCalibration.first;
    auto dclCalibrationB = dclResult.value.newCalibration.second;
    // clang-format off
    auto newCalibrationHandler = currentHandler;

    dai::node::DclUtils::convertDclCalibrationToDai(
	newCalibrationHandler, dclCalibrationA, dclCalibrationB, daiSocketA, daiSocketB, width, height);

    CalibrationQuality::Data qualityData{};
    qualityData.rotationChange[0] = dclResult.value.calibrationDifference->rotationChange[0];
    qualityData.rotationChange[1] = dclResult.value.calibrationDifference->rotationChange[1];
    qualityData.rotationChange[2] = dclResult.value.calibrationDifference->rotationChange[2];
    qualityData.depthErrorDifference = dclResult.value.calibrationDifference->depthDistanceDifference;
    qualityData.sampsonErrorCurrent  = dclResult.value.calibrationDifference->sampsonErrorCurrent;
    qualityData.sampsonErrorNew = dclResult.value.calibrationDifference->sampsonErrorNew;

    DynamicCalibrationResult::Data resultData{};
    resultData.newCalibration       = newCalibrationHandler;
    resultData.currentCalibration   = currentHandler;
    resultData.calibrationDifference = qualityData;

    auto result = std::make_shared<DynamicCalibrationResult>(resultData, dclResult.errorMessage());
    // clang-format on
    logger->info(
        "Calibration successful. Rotation Δ=({}, {}, {})", qualityData.rotationChange[0], qualityData.rotationChange[1], qualityData.rotationChange[2]);
    calibrationOutput.send(result);

    return DynamicCalibration::ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::runLoadImage(const bool blocking) {
    std::shared_ptr<dai::MessageGroup> inSyncGroup;
    logger->info("Attempting to load stereo image pair (blocking={})", blocking);
    if(!blocking) {
        inSyncGroup = syncInput.tryGet<dai::MessageGroup>();
    } else {
        slept = true;
        inSyncGroup = syncInput.get<dai::MessageGroup>();
    }
    if(!inSyncGroup) {
        return DynamicCalibration::ErrorCode::EMPTY_IMAGE_QUEUE;
    }
    auto leftFrame = inSyncGroup->get<dai::ImgFrame>(leftInputName);
    auto rightFrame = inSyncGroup->get<dai::ImgFrame>(rightInputName);

    if(!leftFrame || !rightFrame) {
        logger->info("WARNING: Missing image(s) in MessageGroup (left={}, right={})", leftFrame ? "ok" : "missing", rightFrame ? "ok" : "missing");
        return DynamicCalibration::ErrorCode::MISSING_IMAGE;
    }

    dcl::timestamp_t timestamp = leftFrame->getTimestamp().time_since_epoch().count();
    auto leftCvFrame = leftFrame->getCvFrame();
    auto rightCvFrame = rightFrame->getCvFrame();

    logger->info("Loaded stereo image pair: {}x{} @ timestamp={}", leftFrame->getWidth(), leftFrame->getHeight(), timestamp);

    dynCalibImpl->loadStereoImagePair(
        DclUtils::cvMatToImageData(leftCvFrame), DclUtils::cvMatToImageData(rightCvFrame), deviceName, socketA, socketB, timestamp);

    return DynamicCalibration::ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::computeCoverage() {
    auto resultCoverage = dynCalibImpl->computeCoverage(sensorA, sensorB, performanceMode);

    if(!resultCoverage.passed()) {
        throw std::runtime_error("Coverage check failed!");
    }

    auto& coverage = resultCoverage.value;

    auto coverageResult = std::make_shared<CoverageData>(coverage);
    logger->info("Computing coverage for sockets A={} and B={}", static_cast<int>(socketA), static_cast<int>(socketB));

    coverageOutput.send(coverageResult);

    return DynamicCalibration::ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::initializePipeline(const std::shared_ptr<dai::Device> daiDevice) {
    logger->info("Initializing DynamicCalibration pipeline for device: {}", daiDevice->getDeviceId());

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
    if(daiSocketA == daiSocketB) {
        logger->error("Both input images are from the same socket: {}", static_cast<int>(daiSocketA));
        return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
    }

    logger->info("Detected sockets: A={} B={}, resolution={}x{}", static_cast<int>(daiSocketA), static_cast<int>(daiSocketB), width, height);

    socketA = static_cast<dcl::socket_t>(daiSocketA);
    socketB = static_cast<dcl::socket_t>(daiSocketB);

    logger->info("Converting dai calibration to dcl for sockets A={} B={}", static_cast<int>(daiSocketA), static_cast<int>(daiSocketB));

    calibrationHandler = daiDevice->getCalibration();

    auto [calibA, calibB] = DclUtils::convertDaiCalibrationToDcl(calibrationHandler, daiSocketA, daiSocketB, width, height);

    // set up the dynamic calibration
    deviceName = daiDevice->getDeviceId();
    dcDevice = dynCalibImpl->addDevice(deviceName);
    dcl::resolution_t resolution{static_cast<unsigned>(width), static_cast<unsigned>(height)};

    sensorA = std::make_shared<dcl::CameraSensorHandle>(calibA, resolution);
    sensorB = std::make_shared<dcl::CameraSensorHandle>(calibB, resolution);
    logger->info("Added sensors for sockets A={} and B={} to dynCalibImpl", static_cast<int>(socketA), static_cast<int>(socketB));

    dynCalibImpl->addSensor(deviceName, sensorA, socketA);
    dynCalibImpl->addSensor(deviceName, sensorB, socketB);

    return DynamicCalibration::ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::evaluateCommand(const std::shared_ptr<DynamicCalibrationControl>& control) {
    using DC = DynamicCalibrationControl;

    const auto& cmd = control->command;

    // Calibrate
    if(std::holds_alternative<DC::Commands::Calibrate>(cmd)) {
        const auto& c = std::get<DC::Commands::Calibrate>(cmd);
        logger->info("Received Calibrate Command: force={}", c.force);
        calibrationShouldRun = false;  // stop the calibration if it is running
        return runCalibration(calibrationHandler, c.force);
    }
    // Quality check
    else if(std::holds_alternative<DC::Commands::CalibrationQuality>(cmd)) {
        const auto& c = std::get<DC::Commands::CalibrationQuality>(cmd);
        logger->info("Received CalibrationQuality Command: force={}", c.force);
        return runQualityCheck(c.force);
    }
    // Start calibration loop
    else if(std::holds_alternative<DC::Commands::StartCalibration>(cmd)) {
        const auto& c = std::get<DC::Commands::StartCalibration>(cmd);
        logger->info("Received StartCalibration Command");
        calibrationShouldRun = true;
        loadImagePeriod = c.loadImagePeriod;
        calibrationPeriod = c.calibrationPeriod;
        return ErrorCode::OK;
    }
    // Load a single image
    else if(std::holds_alternative<DC::Commands::LoadImage>(cmd)) {
        logger->info("Received LoadImage Command: blocking load with coverage computation");
        auto error = runLoadImage(true);
        computeCoverage();
        return error;
    }
    // Apply calibration
    else if(std::holds_alternative<DC::Commands::ApplyCalibration>(cmd)) {
        const auto& c = std::get<DC::Commands::ApplyCalibration>(cmd);
        logger->info("Received ApplyCalibrationCommand: applying new calibration to device {}", deviceName);
        calibrationHandler = c.calibration;
        setCalibration(calibrationHandler);
        return ErrorCode::OK;
    }
    // Stop calibration loop
    else if(std::holds_alternative<DC::Commands::StopCalibration>(cmd)) {
        logger->info("Received StopCalibrationCommand: stopping calibration");
        calibrationShouldRun = false;
        return ErrorCode::OK;
    }
    // Reset/remove accumulated data
    else if(std::holds_alternative<DC::Commands::ResetData>(cmd)) {
        logger->info("Received RemoveDataCommand: removing the data");
        dynCalibImpl->removeAllData(sensorA, sensorB);
        return ErrorCode::OK;
    }
    // Set performance mode
    else if(std::holds_alternative<DC::Commands::SetPerformanceMode>(cmd)) {
        const auto& c = std::get<DC::Commands::SetPerformanceMode>(cmd);
        logger->info("Received SetPerformanceModeCommand: changing performance mode to {}", static_cast<int>(c.performanceMode));
        performanceMode = c.performanceMode;
        return ErrorCode::OK;
    }

    // Fallback
    logger->info("WARNING: evaluateCommand: Received unknown/unhandled command type");
    return ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::doWork(std::chrono::steady_clock::time_point& previousLoadingAndCalibrationTime) {
    auto error = ErrorCode::OK;  // Expect everything is ok
    auto calibrationCommand = inputControl.tryGet<DynamicCalibrationControl>();
    if(calibrationCommand) {
        error = evaluateCommand(calibrationCommand);
    }
    if(error != ErrorCode::OK) {  // test progress so far
        return error;
    }
    if(!calibrationShouldRun) {
        return error;
    }
    // Rate limit of the image loading
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<float> elapsed = now - previousLoadingAndCalibrationTime;
    bool loadingAndCalibrationRequired = elapsed.count() > loadImagePeriod;
    if(loadingAndCalibrationRequired) {
        logger->info("doWork() called. CalibrationRunning={}, elapsed={}s", calibrationShouldRun, elapsed.count());
        error = runLoadImage(true);
    }

    if(error != ErrorCode::OK) {  // test progress so far
        return error;
    }
    if(loadingAndCalibrationRequired) {
        computeCoverage();
        previousLoadingAndCalibrationTime = std::chrono::steady_clock::now();
        error = runCalibration(calibrationHandler);
        if(error == DynamicCalibration::ErrorCode::OK) {
            calibrationShouldRun = false;
        }
    }

    return error;
}

void DynamicCalibration::run() {
    if(!device) {
        logger->error("Dynamic calibration node does not have access to any device.");
        return;
    }

    logger->info("DynamicCalibration node started ");

    auto previousLoadingTimeFloat = std::chrono::steady_clock::now() + std::chrono::duration<float>(calibrationPeriod);
    auto previousLoadingTime = std::chrono::time_point_cast<std::chrono::steady_clock::duration>(previousLoadingTimeFloat);
    initializePipeline(device);
    while(isRunning()) {
        slept = false;
        doWork(previousLoadingTime);
        if(!slept) {
            // sleep to prevent 100% CPU utilization
            std::this_thread::sleep_for(sleepingTime);
        }
    }
}

}  // namespace node
}  // namespace dai
