#include "depthai/pipeline/node/DynamicCalibration.hpp"

#include <DynamicCalibration.hpp>
#include <common_world_types.hpp>
#include <opencv2/opencv.hpp>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/utility/matrixOps.hpp"
#include "spdlog/spdlog.h"
#include "utility/Logging.hpp"

namespace {

dcl::ImageData cvMatToImageData(const cv::Mat& mat) {
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

}  // namespace

namespace dai {
namespace node {

DynamicCalibration::Properties& DynamicCalibration::getProperties() {
    return properties;
}

void DynamicCalibration::setPerformanceMode(dai::DynamicCalibrationProperties::PerformanceMode mode) {
    properties.performanceMode = mode;
}

void DynamicCalibration::setContinousMode() {
    properties.recalibrationMode = dai::DynamicCalibrationProperties::RecalibrationMode::CONTINUOUS;
}

void DynamicCalibration::setTimeFrequency(int time) {
    properties.timeFrequency = time;
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
    sync->out.link(inSync);
    sync->setRunOnHost(true);
}

dai::DynamicCalibrationResults::CalibrationQualityResult DynamicCalibration::calibQualityfromDCL(const dcl::CalibrationQuality& src) {
    dai::DynamicCalibrationResults::CalibrationQualityResult out_report;
    dai::DynamicCalibrationResults::CalibrationQuality out;
    out.dataAcquired = src.dataAquired;
    if(src.coverageQuality.has_value()) {
        dai::DynamicCalibrationResults::CalibrationQuality::CoverageData cov;
        cov.coveragePerCellA = src.coverageQuality->coveragePerCellA;
        cov.coveragePerCellB = src.coverageQuality->coveragePerCellB;
        cov.meanCoverage = src.coverageQuality->meanCoverage;
        out.coverageQuality = cov;
    }

    if(src.calibrationQuality.has_value()) {
        dai::DynamicCalibrationResults::CalibrationQuality ::CalibrationData cal;
        for(int i = 0; i < 3; ++i) {
            cal.rotationChange[i] = src.calibrationQuality->rotationChange[i];
        }
        cal.epipolarErrorChange = src.calibrationQuality->epipolarErrorChange;
        cal.depthErrorDifference = src.calibrationQuality->depthDistanceDifference;
        out.calibrationQuality = cal;
    }
    out_report.report = out;
    return out_report;
}

std::shared_ptr<dcl::CameraCalibrationHandle> DynamicCalibration::createDCLCameraCalibration(const std::vector<std::vector<float>> cameraMatrix,
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
        tvec[i] = static_cast<dcl::scalar_t>(translationVector[i] / 100);  // Convert to m
    }

    return std::make_shared<dcl::CameraCalibrationHandle>(rvec, tvec, cameraMatrixArr, distortion);
}

CalibrationHandler DynamicCalibration::convertDCLtoDAI(CalibrationHandler calibHandler,
                                                       const std::shared_ptr<const dcl::CameraCalibrationHandle> daiCalibration,
                                                       const CameraBoardSocket socketSrc,
                                                       const CameraBoardSocket socketDest,
                                                       const int width,
                                                       const int height) {
    dcl::scalar_t distortion[14];
    daiCalibration->getDistortion(distortion);
    calibHandler.setDistortionCoefficients(socketDest, std::vector<float>(distortion, distortion + 14));

    dcl::scalar_t cameraMatrix[9];
    daiCalibration->getCameraMatrix(cameraMatrix);
    std::vector<std::vector<float>> mat = {{static_cast<float>(cameraMatrix[0]), static_cast<float>(cameraMatrix[1]), static_cast<float>(cameraMatrix[2])},
                                           {static_cast<float>(cameraMatrix[3]), static_cast<float>(cameraMatrix[4]), static_cast<float>(cameraMatrix[5])},
                                           {static_cast<float>(cameraMatrix[6]), static_cast<float>(cameraMatrix[7]), static_cast<float>(cameraMatrix[8])}};
    calibHandler.setCameraIntrinsics(socketDest, mat, width, height);

    dcl::scalar_t tvec[3];
    daiCalibration->getTvec(tvec);
    auto translation = std::vector<float>(tvec, tvec + 3);
    for(auto& val : translation) {
        val *= 100.0f;
    }
    dcl::scalar_t rvec[3];
    daiCalibration->getRvec(rvec);
    auto rotationMatrix = matrix::rvecToRotationMatrix(rvec);
    auto specTranslation = calibHandler.getCameraTranslationVector(socketSrc, socketDest, true);

    calibHandler.setCameraExtrinsics(socketSrc, socketDest, rotationMatrix, translation, specTranslation);
    return calibHandler;
}

DynamicCalibration::CalibData DynamicCalibration::getDataFromDAIHandler(
    CalibrationHandler currentCalibration, const CameraBoardSocket boardSocketA, const CameraBoardSocket boardSocketB, const int width, const int height) {
    DynamicCalibration::CalibData data;

    data.translationVectorA = {0.0f, 0.0f, 0.0f};
    data.rotationMatrixA = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

    data.translationVectorB = currentCalibration.getCameraTranslationVector(boardSocketA, boardSocketB, false);
    data.rotationMatrixB = currentCalibration.getCameraRotationMatrix(boardSocketA, boardSocketB);

    data.leftCameraMatrix = currentCalibration.getCameraIntrinsics(boardSocketA, width, height);
    data.rightCameraMatrix = currentCalibration.getCameraIntrinsics(boardSocketB, width, height);

    data.leftDistortionCoefficients = currentCalibration.getDistortionCoefficients(boardSocketA);
    data.rightDistortionCoefficients = currentCalibration.getDistortionCoefficients(boardSocketB);

    return data;
}

void DynamicCalibration::setInternalCalibration(
    std::shared_ptr<Device> device, const CameraBoardSocket boardSocketA, const CameraBoardSocket boardSocketB, const int width, const int height) {
    CalibrationHandler currentCalibration = device->getCalibration();
    CalibData calibData = getDataFromDAIHandler(currentCalibration, boardSocketA, boardSocketB, width, height);

    std::shared_ptr<dcl::CameraCalibrationHandle> calibA =
        createDCLCameraCalibration(calibData.leftCameraMatrix, calibData.leftDistortionCoefficients, calibData.rotationMatrixA, calibData.translationVectorA);
    std::shared_ptr<dcl::CameraCalibrationHandle> calibB =
        createDCLCameraCalibration(calibData.rightCameraMatrix, calibData.rightDistortionCoefficients, calibData.rotationMatrixB, calibData.translationVectorB);
    dynCalibImpl->setNewCalibration(deviceName, socketA, calibA->getCalibration());
    dynCalibImpl->setNewCalibration(deviceName, socketB, calibB->getCalibration());
}

void DynamicCalibration::pipelineSetup(std::shared_ptr<Device> device, CameraBoardSocket boardSocketA, CameraBoardSocket boardSocketB, int width, int height) {
    CalibrationHandler currentCalibration = device->getCalibration();
    deviceName = device->getDeviceId();

    dynCalibImpl = std::make_unique<dcl::DynamicCalibration>();
    dcDevice = dynCalibImpl->addDevice(deviceName);

    DynamicCalibration::CalibData calibData = getDataFromDAIHandler(currentCalibration, boardSocketA, boardSocketB, width, height);

    std::shared_ptr<dcl::CameraCalibrationHandle> calibA =
        createDCLCameraCalibration(calibData.leftCameraMatrix, calibData.leftDistortionCoefficients, calibData.rotationMatrixA, calibData.translationVectorA);
    std::shared_ptr<dcl::CameraCalibrationHandle> calibB =
        createDCLCameraCalibration(calibData.rightCameraMatrix, calibData.rightDistortionCoefficients, calibData.rotationMatrixB, calibData.translationVectorB);

    const dcl::resolution_t resolution = {.width = static_cast<unsigned int>(width), .height = static_cast<unsigned int>(height)};

    sensorA = std::make_shared<dcl::CameraSensorHandle>(calibA, resolution);
    sensorB = std::make_shared<dcl::CameraSensorHandle>(calibB, resolution);

    socketA = static_cast<dcl::socket_t>(boardSocketA);
    socketB = static_cast<dcl::socket_t>(boardSocketB);

    dynCalibImpl->addSensor(deviceName, sensorA, socketA);
    dynCalibImpl->addSensor(deviceName, sensorB, socketB);
}

void DynamicCalibration::resetResults() {
    calibQuality.calibrationQuality = dcl::CalibrationData{};
    calibQuality.coverageQuality = dcl::CoverageData{};
    calibQuality.dataAquired = 0.f;
    dynResult.calibOverallQuality = calibQualityfromDCL(calibQuality);
    dynResult.newCalibration = DynamicCalibrationResults::CalibrationResult::Invalid();
    dynResult.calibOverallQuality = DynamicCalibrationResults::CalibrationQualityResult::Invalid();
    dynResult.newCalibration = DynamicCalibrationResults::CalibrationResult::Invalid();
    dynResult.info = "";
};

bool isAlmostBlackOrWhite(const cv::Mat& frame, double tolerance = 25.5, double maxStdDev = 10.0) {
    if(frame.empty()) return true;

    cv::Mat gray;
    if(frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = frame;
    }

    cv::Scalar mean, stddev;
    cv::meanStdDev(gray, mean, stddev);

    double meanVal = mean[0];
    double stdVal = stddev[0];
    bool isAlmostBlack = (meanVal < tolerance && stdVal < maxStdDev);
    bool isAlmostWhite = (meanVal > (255 - tolerance) && stdVal < maxStdDev);

    return isAlmostBlack || isAlmostWhite;
}

void DynamicCalibration::run() {
    if(!device) {
        logger::error("Dynamic calibration node does not have access to any device.");
        return;
    }

    logger::info("DynamicCalibration node is running");
    auto lastAutoTrigger = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    auto continousTrigger = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    resetResults();
    while(isRunning()) {
        // auto leftFrame = left.get<dai::ImgFrame>();
        // auto rightFrame = right.get<dai::ImgFrame>();
        auto inSyncGroup = inSync.tryGet<dai::MessageGroup>();
        if(!inSyncGroup) continue;
        auto leftFrame = inSyncGroup->get<dai::ImgFrame>(leftInputName);
        auto rightFrame = inSyncGroup->get<dai::ImgFrame>(rightInputName);
        if(!leftFrame || !rightFrame) continue;  // todo calib team sync should be checked?

        if(inputConfig.getWaitForMessage()) {  // blocking
            calibrationConfig = inputConfig.get<DynamicCalibrationConfig>();
        } else {  // non-blocking
            calibrationConfig = inputConfig.tryGet<DynamicCalibrationConfig>();
        }

        if(calibrationConfig) {
            auto calibrationCommand = properties.calibrationCommand;
            if(calibrationCommand) {
                switch(*calibrationCommand) {
                    case DynamicCalibrationProperties::CalibrationCommand::START_CALIBRATION_QUALITY_CHECK:
                        logger::info("[DynamicCalibration] StartCalibrationQualityMessageRecieved.");
                        calibrationSM.startQualityCheck();
                        dynResult.info = "Start Calibration Quality Check";
                        break;
                    case DynamicCalibrationProperties::CalibrationCommand::START_RECALIBRATION:
                        logger::info("[DynamicCalibration] RecalibrationQualityMessageReceived.");
                        calibrationSM.startRecalibration();
                        dynResult.info = "Start Recalibration";
                        break;
                    case DynamicCalibrationProperties::CalibrationCommand::START_FORCE_RECALIBRATION:
                        logger::info("[DynamicCalibration] RecalibrationQualityMessageReceived.");
                        calibrationSM.startRecalibration();
                        forceTrigger = true;
                        properties.performanceMode = DynamicCalibrationProperties::PerformanceMode::SKIP_CHECKS;
                        dynResult.info = "Start Recalibration";
                        break;
                    case DynamicCalibrationProperties::CalibrationCommand::START_FORCE_CALIBRATION_QUALITY_CHECK:
                        logger::info("[DynamicCalibration] StartCalibrationQualityMessageRecieved.");
                        calibrationSM.startQualityCheck();
                        forceTrigger = true;
                        properties.performanceMode = DynamicCalibrationProperties::PerformanceMode::SKIP_CHECKS;
                        dynResult.info = "Start Calibration Quality Check";
                        break;
                    default:
                        logger::warn("[DynamicCalibration] Unknown calibrationCommand: {}", static_cast<int>(*calibrationCommand));
                }
            }
        }

        // === STATE MACHINE START ===
        auto now = std::chrono::steady_clock::now();

        if(properties.recalibrationMode == dai::DynamicCalibrationProperties::RecalibrationMode::CONTINUOUS && calibrationSM.isIdle()
           && calibrationSM.pipelineReady && std::chrono::duration_cast<std::chrono::seconds>(now - continousTrigger).count() > properties.timeFrequency) {
            lastAutoTrigger = now;
            calibrationSM.startRecalibration();
        }
        if(!calibrationSM.pipelineReady) {
            calibrationSM.state = CalibrationStateMachine::CalibrationState::InitializingPipeline;
        }

        switch(calibrationSM.state) {
            case CalibrationStateMachine::CalibrationState::InitializingPipeline: {
                if(!calibrationSM.pipelineReady) {
                    widthDefault = leftFrame->getWidth();
                    heightDefault = rightFrame->getHeight();
                    daiSocketA = static_cast<CameraBoardSocket>(leftFrame->instanceNum);
                    daiSocketB = static_cast<CameraBoardSocket>(rightFrame->instanceNum);

                    pipelineSetup(device, daiSocketA, daiSocketB, widthDefault, heightDefault);

                    calibrationSM.markPipelineReady();
                    logger::info("[DynamicCalibration] Pipeline initialized.");
                }
                break;
            }

            case CalibrationStateMachine::CalibrationState::ResetDynamicRecalibration: {
                logger::info("[DynamicCalibration] Resseting dynamic recalibration values");
                resetResults();
                dynCalibImpl->removeDeviceMeasurements(dcDevice);
                calibrationSM.finish();
                break;
            }

            case CalibrationStateMachine::CalibrationState::Idle:
                if(calibrationSM.pipelineReady) {
                    widthDefault = leftFrame->getWidth();
                    heightDefault = rightFrame->getHeight();
                    daiSocketA = static_cast<CameraBoardSocket>(leftFrame->instanceNum);
                    daiSocketB = static_cast<CameraBoardSocket>(rightFrame->instanceNum);
                    setInternalCalibration(device, daiSocketA, daiSocketB, widthDefault, heightDefault);
                }
                break;

            case CalibrationStateMachine::CalibrationState::LoadingImages: {
                if(std::chrono::duration_cast<std::chrono::seconds>(now - lastAutoTrigger).count() > 0.5) {
                    dcl::timestamp_t timestamp = leftFrame->getTimestamp().time_since_epoch().count();
                    auto imageA = leftFrame->getCvFrame();
                    auto imageB = rightFrame->getCvFrame();
                    if(imageA.empty() || imageB.empty()) continue;
                    if(isAlmostBlackOrWhite(leftFrame->getCvFrame())) {
                        dynResult.info = "Frame is nearly black or white — skipping data loading.";
                        break;
                    }
                    dcl::ImageData imgA = cvMatToImageData(imageA);
                    dcl::ImageData imgB = cvMatToImageData(imageB);
                    dynCalibImpl->loadStereoImagePair(imgA, imgB, deviceName, socketA, socketB, timestamp);

                    logger::info("[DynamicCalibration] Loaded image in DCL");

                    calibrationSM.AdvanceAfterLoading();
                    lastAutoTrigger = now;
                    break;
                } else {
                    break;
                }
            }

            case CalibrationStateMachine::CalibrationState::ProcessingQuality: {
                logger::info("[DynamicCalibration] Running quality check...");
                auto result = forceTrigger
                                  ? dynCalibImpl->checkCalibration(
                                      dcDevice, socketA, socketB, static_cast<dcl::PerformanceMode>(DynamicCalibrationProperties::PerformanceMode::SKIP_CHECKS))
                                  : dynCalibImpl->checkCalibration(dcDevice, socketA, socketB, static_cast<dcl::PerformanceMode>(properties.performanceMode));
                calibQuality = result.value;
                dynResult.info = result.errorMessage();
                dynResult.calibOverallQuality = calibQualityfromDCL(calibQuality);
                if(forceTrigger) {
                    forceTrigger = false;
                }
                const auto& rot = dynResult.calibOverallQuality->report->calibrationQuality->rotationChange;
                bool isMissing = std::any_of(
                    rot.begin(), rot.end(), [](float v) { return std::abs(v) > 1000.0f || std::abs(v) < 1e-14f || std::isnan(v) || std::isinf(v); });
                if(isMissing) {
                    logger::info("[DynamicCalibration] results does not have value!");
                    calibrationSM.finish();
                    calibrationSM.startQualityCheck();
                    outputCalibrationResults.send(std::make_shared<DynamicCalibrationResults>(dynResult));
                    break;
                } else {
                    calibrationSM.deleteAllData();
                }
                outputCalibrationResults.send(std::make_shared<DynamicCalibrationResults>(dynResult));
                break;
            }

            case CalibrationStateMachine::CalibrationState::Recalibrating: {
                logger::info("[DynamicCalibration] Running full recalibration...");
                auto result = forceTrigger
                                  ? dynCalibImpl->checkCalibration(
                                      dcDevice, socketA, socketB, static_cast<dcl::PerformanceMode>(DynamicCalibrationProperties::PerformanceMode::SKIP_CHECKS))
                                  : dynCalibImpl->checkCalibration(dcDevice, socketA, socketB, static_cast<dcl::PerformanceMode>(properties.performanceMode));
                if(forceTrigger) {
                    forceTrigger = false;
                }
                calibQuality = result.value;
                dynResult.calibOverallQuality = calibQualityfromDCL(calibQuality);
                dynResult.info = result.errorMessage();
                const auto& rot = dynResult.calibOverallQuality->report->calibrationQuality->rotationChange;
                bool isMissing = std::any_of(
                    rot.begin(), rot.end(), [](float v) { return std::abs(v) > 1000.0f || std::abs(v) < 1e-14f || std::isnan(v) || std::isinf(v); });
                if(isMissing) {
                    logger::info("[DynamicCalibration] results does not have value!");
                    calibrationSM.finish();
                    calibrationSM.startRecalibration();
                    outputCalibrationResults.send(std::make_shared<DynamicCalibrationResults>(dynResult));
                    break;
                }
                auto resultCalib =
                    forceTrigger ? dynCalibImpl->findNewCalibration(
                        dcDevice, socketA, socketB, static_cast<dcl::PerformanceMode>(DynamicCalibrationProperties::PerformanceMode::SKIP_CHECKS))
                                 : dynCalibImpl->findNewCalibration(dcDevice, socketA, socketB, static_cast<dcl::PerformanceMode>(properties.performanceMode));
                dynResult.info = resultCalib.errorMessage();
                if(!resultCalib.value.second) {
                    logger::info("[DynamicCalibration] resultCalib returned null CalibrationHandler!");
                    outputCalibrationResults.send(std::make_shared<DynamicCalibrationResults>(dynResult));
                    calibrationSM.finish();
                    break;
                }
                auto calibrationHandle = resultCalib.value.second;

                if(calibrationHandle->getCameraCalibration()) {
                    CalibrationHandler calibHandler = device->getCalibration();
                    dynResult.newCalibration->calibHandler =
                        convertDCLtoDAI(calibHandler, calibrationHandle, daiSocketA, daiSocketB, widthDefault, heightDefault);
                    calibrationSM.deleteAllData();
                    logger::info("[DynamicCalibration] Got new calibrationHandler.");
                    if(properties.recalibrationMode == dai::DynamicCalibrationProperties::RecalibrationMode::CONTINUOUS) {
                        device->setCalibration(dynResult.newCalibration->calibHandler.value());
                        logger::info("[DynamicCalibration] Applied new calibration in continious mode.");
                    }
                } else if(resultCalib.errorCode == FINDNEWCALIBRATION_NOT_SIGNIFICANT_CHANGE) {
                    calibrationSM.deleteAllData();
                    logger::info("[DynamicCalibration] Find new calibration with no difference");
                } else {
                    calibrationSM.finish();
                }
                logger::info("[DynamicCalibration] Recalibration complete.");
                outputCalibrationResults.send(std::make_shared<DynamicCalibrationResults>(dynResult));
                break;
            }
                // TODO, ADD A METHOD, WHICH SETS NEW CALIBRATION ON DEVICE, DCL AND AS WELL RESETS THE RESULTS IN CALIBRATION STATE
        }
        // === STATE MACHINE END ===

        // send results
    }
}

void DynamicCalibration::CalibrationStateMachine::startQualityCheck() {
    if(isIdle()) {
        state = CalibrationState::LoadingImages;
        mode = CalibrationMode::QualityCheck;
    }
}

void DynamicCalibration::CalibrationStateMachine::startRecalibration() {
    if(isIdle()) {
        state = CalibrationState::LoadingImages;
        mode = CalibrationMode::Recalibration;
    }
}

void DynamicCalibration::CalibrationStateMachine::markPipelineReady() {
    pipelineReady = true;
    state = CalibrationState::Idle;
}

bool DynamicCalibration::CalibrationStateMachine::isIdle() const {
    return state == CalibrationState::Idle;
}

void DynamicCalibration::CalibrationStateMachine::AdvanceAfterLoading() {
    if(mode == CalibrationMode::QualityCheck) {
        state = CalibrationState::ProcessingQuality;
    } else if(mode == CalibrationMode::Recalibration) {
        state = CalibrationState::Recalibrating;
    }
}

void DynamicCalibration::CalibrationStateMachine::deleteAllData() {
    state = CalibrationState::ResetDynamicRecalibration;
}

void DynamicCalibration::CalibrationStateMachine::finish() {
    state = CalibrationState::Idle;
    mode = CalibrationMode::None;
}

std::string DynamicCalibration::CalibrationStateMachine::stateToString() const {
    switch(state) {
        case CalibrationState::Idle:
            return "Idle";
        case CalibrationState::InitializingPipeline:
            return "InitializingPipeline";
        case CalibrationState::LoadingImages:
            return "LoadingImages";
        case CalibrationState::ProcessingQuality:
            return "ProcessingQuality";
        case CalibrationState::Recalibrating:
            return "Recalibrating";
        case CalibrationState::ResetDynamicRecalibration:
            return "ResetingCalibration";
        default:
            return "Unknown";
    }
}

}  // namespace node
}  // namespace dai
