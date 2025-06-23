#include "depthai/pipeline/node/DynamicCalibration.hpp"

#include <DynamicCalibration.hpp>
#include <common_world_types.hpp>
#include <opencv2/opencv.hpp>

#include "common/CameraBoardSocket.hpp"
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

DynamicCalibration::~DynamicCalibration() = default;

DynamicCalibration::Properties& DynamicCalibration::getProperties() {
    properties.initialConfig = *initialConfig;
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


std::vector<float> DynamicCalibration::rotationMatrixToVector(const std::vector<std::vector<float>>& R) {
    if(R.size() != 3 || R[0].size() != 3 || R[1].size() != 3 || R[2].size() != 3) {
        throw std::invalid_argument("Expected a 3x3 rotation matrix.");
    }

    float angle, x, y, z;

    float trace = R[0][0] + R[1][1] + R[2][2];
    float cos_angle = (trace - 1.0f) * 0.5f;

    // Clamp cos_angle to [-1, 1] to avoid NaN due to float precision
    cos_angle = std::fmax(-1.0f, std::fmin(1.0f, cos_angle));
    angle = std::acos(cos_angle);

    if(std::fabs(angle) < 1e-6f) {
        // Angle is ~0 → zero rotation vector
        return {0.0f, 0.0f, 0.0f};
    }

    float rx = R[2][1] - R[1][2];
    float ry = R[0][2] - R[2][0];
    float rz = R[1][0] - R[0][1];

    float sin_angle = std::sqrt(rx * rx + ry * ry + rz * rz) * 0.5f;

    // Normalize axis
    float k = 1.0f / (2.0f * sin_angle);
    x = k * rx;
    y = k * ry;
    z = k * rz;

    // Rotation vector = axis * angle
    return {x * angle, y * angle, z * angle};
}

std::vector<std::vector<float>> rvecToRotationMatrix(const double rvec[3]) {
    // Convert input array to cv::Mat
    cv::Mat rvecMat(3, 1, CV_64F);
    for(int i = 0; i < 3; ++i) {
        rvecMat.at<double>(i, 0) = rvec[i];
    }

    // Convert Rodrigues vector to rotation matrix
    cv::Mat R;
    cv::Rodrigues(rvecMat, R);

    // Convert cv::Mat (CV_64F) to std::vector<std::vector<float>>
    std::vector<std::vector<float>> rotMatrix(3, std::vector<float>(3));
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            rotMatrix[i][j] = static_cast<float>(R.at<double>(i, j));
        }
    }

    return rotMatrix;
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
    std::vector<float> rvecVec = rotationMatrixToVector(rotationMatrix);
    for(int i = 0; i < 3; ++i) {
        rvec[i] = static_cast<dcl::scalar_t>(rvecVec[i]);
    }

    for(int i = 0; i < 3; ++i) {
        tvec[i] = static_cast<dcl::scalar_t>(translationVector[i] * 10.0f);  // Convert to mm
    }

    return std::make_shared<dcl::CameraCalibrationHandle>(rvec, tvec, cameraMatrixArr, distortion);
}

void DynamicCalibration::startCalibQualityCheck() {
    if(!calibrationSM.isIdle()) {
        logger::debug("[DynamicCalibration] Cannot start quality check: state = {}", calibrationSM.stateToString());

        return;
    }
    calibrationSM.startQualityCheck();
    logger::info("[DynamicCalibration] Quality check started.");
}

void DynamicCalibration::startRecalibration() {
    if(!calibrationSM.isIdle()) {
        logger::debug("[DynamicCalibration] Cannot start recalibration: state = {}", calibrationSM.stateToString());
        return;
    }
    calibrationSM.startRecalibration();
    logger::info("[DynamicCalibration] Recalibration started.");
}

void DynamicCalibration::setNewCalibration(CalibrationHandler currentCalibration) {
    CameraBoardSocket boardSocketA = static_cast<CameraBoardSocket>(socketA);
    CameraBoardSocket boardSocketB = static_cast<CameraBoardSocket>(socketB);

    const std::vector<float> translationVectorA = {0.0f, 0.0f, 0.0f};
    const std::vector<std::vector<float>> rotationMatrixA = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

    auto translationVectorB = currentCalibration.getCameraTranslationVector(boardSocketA, boardSocketB);
    auto rotationMatrixB = currentCalibration.getCameraRotationMatrix(boardSocketA, boardSocketB);

    auto leftCameraMatrix = currentCalibration.getCameraIntrinsics(boardSocketA, widthDefault, heightDefault);
    auto rightCameraMatrix = currentCalibration.getCameraIntrinsics(boardSocketB, widthDefault, heightDefault);

    auto leftDistortionCoefficients = currentCalibration.getDistortionCoefficients(boardSocketA);
    auto rightDistortionCoefficients = currentCalibration.getDistortionCoefficients(boardSocketB);
    std::shared_ptr<dcl::CameraCalibrationHandle> calibA =
        createDCLCameraCalibration(leftCameraMatrix, leftDistortionCoefficients, rotationMatrixA, translationVectorA);
    std::shared_ptr<dcl::CameraCalibrationHandle> calibB =
        createDCLCameraCalibration(rightCameraMatrix, rightDistortionCoefficients, rotationMatrixB, translationVectorB);

    dynCalibImpl->setNewCalibration(deviceName, socketA, calibA->getCalibration());
    dynCalibImpl->setNewCalibration(deviceName, socketB, calibB->getCalibration());
    dynResult.newCalibration = DynamicCalibrationResults::CalibrationResult::Invalid();
    dynResult.calibOverallQuality = DynamicCalibrationResults::CalibrationQualityResult::Invalid();
}

CalibrationHandler DynamicCalibration::convertDCLtoDAI(CalibrationHandler calibHandler,
                                                       const std::shared_ptr<const dcl::CameraCalibrationHandle> daiCalibration,
                                                       const CameraBoardSocket socketSrc,
                                                       const CameraBoardSocket socketDest,
                                                       const int width,
                                                       const int height) {
    // set distortion
    dcl::scalar_t distortion[14];
    daiCalibration->getDistortion(distortion);
    calibHandler.setDistortionCoefficients(socketDest, std::vector<float>(distortion, distortion + 14));
    // set camera matrix

    dcl::scalar_t cameraMatrix[9];
    daiCalibration->getCameraMatrix(cameraMatrix);
    std::vector<std::vector<float>> mat = {{static_cast<float>(cameraMatrix[0]), static_cast<float>(cameraMatrix[1]), static_cast<float>(cameraMatrix[2])},
                                           {static_cast<float>(cameraMatrix[3]), static_cast<float>(cameraMatrix[4]), static_cast<float>(cameraMatrix[5])},
                                           {static_cast<float>(cameraMatrix[6]), static_cast<float>(cameraMatrix[7]), static_cast<float>(cameraMatrix[8])}};
    calibHandler.setCameraIntrinsics(socketDest, mat, width, height);

    // tvec
    dcl::scalar_t tvec[3];
    daiCalibration->getTvec(tvec);
    auto translation = std::vector<float>(tvec, tvec + 3);
    for (auto& val : translation) {
        val /= 10.0f;
    }
    // get rvec
    dcl::scalar_t rvec[3];
    daiCalibration->getRvec(rvec);
    auto rotationMatrix = rvecToRotationMatrix(rvec);
    auto specTranslation = calibHandler.getCameraTranslationVector(socketSrc, socketDest, true);

    calibHandler.setCameraExtrinsics(socketSrc, socketDest, rotationMatrix, translation, specTranslation);
    return calibHandler;
}

void DynamicCalibration::setInternalCalibration(std::shared_ptr<Device> device,
                                                const CameraBoardSocket boardSocketA,
                                                const CameraBoardSocket boardSocketB,
                                                const int width,
                                                const int height) {
    CalibrationHandler currentCalibration = device->getCalibration();
    const std::vector<float> translationVectorA = {0.0f, 0.0f, 0.0f};
    const std::vector<std::vector<float>> rotationMatrixA = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

    auto translationVectorB = currentCalibration.getCameraTranslationVector(boardSocketA, boardSocketB);
    auto rotationMatrixB = currentCalibration.getCameraRotationMatrix(boardSocketA, boardSocketB);

    auto leftCameraMatrix = currentCalibration.getCameraIntrinsics(boardSocketA, width, height);
    auto rightCameraMatrix = currentCalibration.getCameraIntrinsics(boardSocketB, width, height);

    auto leftDistortionCoefficients = currentCalibration.getDistortionCoefficients(boardSocketA);
    auto rightDistortionCoefficients = currentCalibration.getDistortionCoefficients(boardSocketB);

    std::shared_ptr<dcl::CameraCalibrationHandle> calibA =
        createDCLCameraCalibration(leftCameraMatrix, leftDistortionCoefficients, rotationMatrixA, translationVectorA);
    std::shared_ptr<dcl::CameraCalibrationHandle> calibB =
        createDCLCameraCalibration(rightCameraMatrix, rightDistortionCoefficients, rotationMatrixB, translationVectorB);
    dynCalibImpl->setNewCalibration(deviceName, socketA, calibA->getCalibration());
    dynCalibImpl->setNewCalibration(deviceName, socketB, calibB->getCalibration());
}

void DynamicCalibration::pipelineSetup(std::shared_ptr<Device> device, CameraBoardSocket boardSocketA, CameraBoardSocket boardSocketB, int width, int height) {
    CalibrationHandler currentCalibration = device->readCalibration();
    deviceName = device->getDeviceId();

    dynCalibImpl = std::make_unique<dcl::DynamicCalibration>();
    dcDevice = dynCalibImpl->addDevice(deviceName);

    const std::vector<float> translationVectorA = {0.0f, 0.0f, 0.0f};
    const std::vector<std::vector<float>> rotationMatrixA = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

    auto translationVectorB = currentCalibration.getCameraTranslationVector(boardSocketA, boardSocketB);
    auto rotationMatrixB = currentCalibration.getCameraRotationMatrix(boardSocketA, boardSocketB);

    auto leftCameraMatrix = currentCalibration.getCameraIntrinsics(boardSocketA, width, height);
    auto rightCameraMatrix = currentCalibration.getCameraIntrinsics(boardSocketB, width, height);

    auto leftDistortionCoefficients = currentCalibration.getDistortionCoefficients(boardSocketA);
    auto rightDistortionCoefficients = currentCalibration.getDistortionCoefficients(boardSocketB);

    std::shared_ptr<dcl::CameraCalibrationHandle> calibA =
        createDCLCameraCalibration(leftCameraMatrix, leftDistortionCoefficients, rotationMatrixA, translationVectorA);
    std::shared_ptr<dcl::CameraCalibrationHandle> calibB =
        createDCLCameraCalibration(rightCameraMatrix, rightDistortionCoefficients, rotationMatrixB, translationVectorB);

    const dcl::resolution_t resolution = {
        .width = static_cast<unsigned int>(width),
        .height = static_cast<unsigned int>(height)
    };

    sensorA = std::make_shared<dcl::CameraSensorHandle>(calibA, resolution);
    sensorB = std::make_shared<dcl::CameraSensorHandle>(calibB, resolution);

    socketA = static_cast<dcl::socket_t>(boardSocketA);
    socketB = static_cast<dcl::socket_t>(boardSocketB);

    dynCalibImpl->addSensor(deviceName, sensorA, socketA);
    dynCalibImpl->addSensor(deviceName, sensorB, socketB);
}

void DynamicCalibration::run() {
    if(!device) {
        logger::error("Dynamic calibration node has to have access to a device!");
        return;
    }

    logger::info("DynamicCalibration node is running");
    auto lastAutoTrigger = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    auto recalibrationMode = DynamicCalibrationConfig::AlgorithmControl::RecalibrationMode::DEFAULT;
    dynResult.newCalibration = DynamicCalibrationResults::CalibrationResult::Invalid();
    dynResult.calibOverallQuality = DynamicCalibrationResults::CalibrationQualityResult::Invalid();
    std::shared_ptr<DynamicCalibrationConfig> calibrationConfig;
    while(isRunning()) {
        auto leftFrame = left.get<dai::ImgFrame>();
        auto rightFrame = right.get<dai::ImgFrame>();

        if(!leftFrame || !rightFrame) continue; //todo calib team sync should be checked?


        if(inputConfig.getWaitForMessage()) { //blocking
            calibrationConfig = inputConfig.get<DynamicCalibrationConfig>();
        } else { //non-blocking
            calibrationConfig = inputConfig.tryGet<DynamicCalibrationConfig>();
        }

        if(calibrationConfig) {
            auto calibrationCommand = calibrationConfig->calibrationCommand;
            if(calibrationCommand) {
                switch(*calibrationCommand) {
                    case DynamicCalibrationConfig::CalibrationCommand::START_CALIBRATION_QUALITY_CHECK:
                        startCalibQualityCheck();
                        logger::info("[DynamicCalibration] StartCalibrationQualityMessageRecieved.");
                        break;
                    case DynamicCalibrationConfig::CalibrationCommand::START_RECALIBRATION:
                        startRecalibration();
                        logger::info("[DynamicCalibration] RecalibratioQualityMessageRecieved.");
                        break;
                    case DynamicCalibrationConfig::CalibrationCommand::START_FORCE_RECALIBRATION:
                        startRecalibration();
                        forceTrigger = true;
                        break;
                    case DynamicCalibrationConfig::CalibrationCommand::START_FORCE_CALIBRATION_QUALITY_CHECK:
                        startCalibQualityCheck();
                        forceTrigger = true;
                        break;
                    default:
                        logger::warn("[DynamicCalibration] Unknown calibrationCommand: {}", static_cast<int>(*calibrationCommand));
                }
            }

        }

        // === STATE MACHINE START ===
        auto now = std::chrono::steady_clock::now();

        if (!calibrationSM.pipelineReady){
            calibrationSM.state = CalibrationStateMachine::CalibrationState::InitializingPipeline;
        }

        if(recalibrationMode == dai::DynamicCalibrationConfig::AlgorithmControl::RecalibrationMode::CONTINUOUS
           && calibrationSM.isIdle() && std::chrono::duration_cast<std::chrono::seconds>(now - lastAutoTrigger).count() > 5) {
            lastAutoTrigger = now;
            // Placeholder for continious function
        }
    
        switch(calibrationSM.state) {
            case CalibrationStateMachine::CalibrationState::InitializingPipeline: {
                if(!calibrationSM.pipelineReady) {
                    widthDefault = leftFrame->getWidth();
                    heightDefault = rightFrame->getHeight();
                    CameraBoardSocket leftSocket = static_cast<CameraBoardSocket>(leftFrame->instanceNum);
                    CameraBoardSocket rightSocket = static_cast<CameraBoardSocket>(rightFrame->instanceNum);

                    pipelineSetup(device, leftSocket, rightSocket, widthDefault, heightDefault);

                    calibrationSM.markPipelineReady();
                    logger::info("[DynamicCalibration] Pipeline initialized.");
                }
                break;
            }

            case CalibrationStateMachine::CalibrationState::ResetDynamicRecalibration: {
                logger::info("[DynamicCalibration] Resseting dynamic recalibration values");
                dynResult.newCalibration = DynamicCalibrationResults::CalibrationResult::Invalid();
                dynResult.calibOverallQuality = DynamicCalibrationResults::CalibrationQualityResult::Invalid();
                dynCalibImpl->resetDeviceMeasurements(dcDevice);
                calibrationSM.finish();
                break;
            }

            case CalibrationStateMachine::CalibrationState::Idle:
                if(calibrationSM.pipelineReady) {
                    widthDefault = leftFrame->getWidth();
                    heightDefault = rightFrame->getHeight();
                    CameraBoardSocket leftSocket = static_cast<CameraBoardSocket>(leftFrame->instanceNum);
                    CameraBoardSocket rightSocket = static_cast<CameraBoardSocket>(rightFrame->instanceNum);
                    setInternalCalibration(device, leftSocket, rightSocket, widthDefault, heightDefault);
                }
                break;

            case CalibrationStateMachine::CalibrationState::CollectingFeatures: {
                if (std::chrono::duration_cast<std::chrono::seconds>(now - lastAutoTrigger).count() > 0.01) {
                    dcl::timestamp_t timestamp = leftFrame->getTimestamp().time_since_epoch().count();
                    auto imageA = leftFrame->getCvFrame();
                    auto imageB = rightFrame->getCvFrame();
                    dcl::ImageData imgA = cvMatToImageData(imageA);
                    dcl::ImageData imgB = cvMatToImageData(imageB);
                    dynCalibImpl->loadStereoImagePair(imgA, imgB, deviceName, socketA, socketB, timestamp);

                    logger::info("[DynamicCalibration] Collected {}{}",
                             (calibrationSM.mode == CalibrationStateMachine::CalibrationMode::QualityCheck ? " (QC)" : " (Recalibration)"));

                    calibrationSM.maybeAdvanceAfterCollection();
                    lastAutoTrigger = now;
                }
                break;
            }

            case CalibrationStateMachine::CalibrationState::ProcessingQuality: {
                logger::info("[DynamicCalibration] Running quality check...");
                auto result = dynCalibImpl->checkCalibrationQuality(dcDevice, socketA, socketB, forceTrigger);
                auto calibQuality = result.value;
                dynResult.calibOverallQuality = dai::DynamicCalibrationResults::CalibrationQualityResult::fromDCL(calibQuality);
                forceTrigger = false;
                auto& report = dynResult.calibOverallQuality->report;
                if (report.has_value() && report->calibrationQuality.has_value()) {
                    calibrationSM.deleteAllData();
                }
                else {
                    calibrationSM.finish();
                }
                break;
            }

            case CalibrationStateMachine::CalibrationState::Recalibrating: {
                logger::info("[DynamicCalibration] Running full recalibration...");
                auto resultCalib = dynCalibImpl->findNewCalibration(dcDevice, socketA, socketB);
                auto calibrationHandle = std::make_shared<dcl::CameraCalibrationHandle>(resultCalib.value.second);
                int width = leftFrame->getWidth();
                int height = rightFrame->getHeight();

                CameraBoardSocket leftSocket = static_cast<CameraBoardSocket>(leftFrame->instanceNum);
                CameraBoardSocket rightSocket = static_cast<CameraBoardSocket>(rightFrame->instanceNum);

                if(calibrationHandle) {
                    CalibrationHandler calibHandler = device->readCalibration();
                    dynResult.newCalibration->calibHandler = convertDCLtoDAI(calibHandler, calibrationHandle, leftSocket, rightSocket, width, height);
                    calibrationSM.deleteAllData();
                }
                else {
                    calibrationSM.finish();
                }
                logger::info("[DynamicCalibration] Recalibration complete.");
                break;
            }
                // TODO, ADD A METHOD, WHICH SETS NEW CALIBRATION ON DEVICE, DCL AND AS WELL RESETS THE RESULTS IN CALIBRATION STATE
        }
        // === STATE MACHINE END ===

        //send results
        outputCalibrationResults.send(std::make_shared<DynamicCalibrationResults>(dynResult));

    }
}
}  // namespace node
}  // namespace dai
