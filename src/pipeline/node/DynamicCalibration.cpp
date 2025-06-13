#include "depthai/pipeline/node/DynamicCalibration.hpp"
#include "common/CameraBoardSocket.hpp"
#include <DynamicCalibration.hpp>
#include <opencv2/opencv.hpp>

dcl::ImageData
cvMatToImageData(const cv::Mat& mat)
{
    if (mat.empty()) {
        throw std::runtime_error("cv::Mat is empty");
    }

    dcl::ImageData img;
    img.width = static_cast<unsigned int>(mat.cols);
    img.height = static_cast<unsigned int>(mat.rows);
    img.data.assign(mat.data, mat.data + mat.total() * mat.elemSize());

    int type = mat.type();
    switch (type) {
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

namespace dai {
namespace node {

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

    if (std::fabs(angle) < 1e-6f) {
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

std::unique_ptr<dcl::CameraSensorHandle> DynamicCalibration::createDCLCameraCalibration(
    const std::vector<std::vector<float>> cameraMatrix,
    const std::vector<float> distortionCoefficients,
    const std::vector<std::vector<float>> rotationMatrix,
    const std::vector<float> translationVector,
    int widthDefault, 
    int heightDefault) {

    dcl::scalar_t cameraMatrixArr[9];
    dcl::scalar_t distortion[14] = {0};
    dcl::scalar_t rvec[3];
    dcl::scalar_t tvec[3];
    const dcl::resolution_t resolution = { .width = static_cast<unsigned int>(widthDefault), .height = static_cast<unsigned int>(heightDefault) };

    // Convert cameraMatrix
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
            cameraMatrixArr[i * 3 + j] = cameraMatrix[i][j];

    // Convert distortion
    for(size_t i = 0; i < distortionCoefficients.size() && i < 14; ++i)
        distortion[i] = distortionCoefficients[i];

    // Convert rotation to vector
    std::vector<float> rvecVec = rotationMatrixToVector(rotationMatrix);
    for(int i = 0; i < 3; ++i) rvec[i] = rvecVec[i];

    for(int i = 0; i < 3; ++i) tvec[i] = translationVector[i] * 10.0f;

    std::unique_ptr<dcl::CameraSensorHandle> handler;
    auto calibrationHandle = std::make_shared<dcl::CameraCalibrationHandle>(rvec, tvec, cameraMatrixArr, distortion);
    handler = std::make_unique<dcl::CameraSensorHandle>(calibrationHandle, resolution);
    return handler;
}


void DynamicCalibration::startCalibQualityCheck() {
    if(!calibrationSM.isIdle()) {
        std::cout << "[DynamicCalibration] Cannot start quality check: state = " << calibrationSM.stateToString() << "\n";
        return;
    }
    calibrationSM.startQualityCheck();
    std::cout << "[DynamicCalibration] Quality check started.\n";
}

void DynamicCalibration::startRecalibration() {
    if(!calibrationSM.isIdle()) {
        std::cout << "[DynamicCalibration] Cannot start recalibration: state = " << calibrationSM.stateToString() << "\n";
        return;
    }
    calibrationSM.startRecalibration();
    std::cout << "[DynamicCalibration] Recalibration started.\n";
}

QualityResult DynamicCalibration::getCalibQuality() const {
    return results.quality;
}

CalibrationResult DynamicCalibration::getNewCalibration() const {
    return results.calibration;
}

void DynamicCalibration::pipelineSetup(std::shared_ptr<Device> device, CameraBoardSocket leftSocket, CameraBoardSocket rightSocket, int widthDefault, int heightDefault){
    CalibrationHandler currentCalibration = device->readCalibration();
    deviceName = device->getDeviceId();

    socketA = static_cast<int>(leftSocket);
    socketB = static_cast<int>(rightSocket);

    dynCalibImpl = std::make_unique<dcl::DynamicCalibration>();
    dcDevice = dynCalibImpl->addDevice(deviceName);

    const std::vector<float> translationVectorA = {0.0f, 0.0f, 0.0f};
    const std::vector<std::vector<float>> rotationMatrixA = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };

    auto translationVectorB = currentCalibration.getCameraTranslationVector(static_cast<dai::CameraBoardSocket>(socketA),
                                                                             static_cast<dai::CameraBoardSocket>(socketB));
    
    auto rotationMatrixB = currentCalibration.getCameraRotationMatrix(static_cast<dai::CameraBoardSocket>(socketA),
                                                                             static_cast<dai::CameraBoardSocket>(socketB));

    auto leftCameraMatrix = currentCalibration.getCameraIntrinsics(static_cast<dai::CameraBoardSocket>(socketA), widthDefault, heightDefault);
    auto rightCameraMatrix = currentCalibration.getCameraIntrinsics(static_cast<dai::CameraBoardSocket>(socketB), widthDefault, heightDefault);

    auto leftDistortionCoefficients = currentCalibration.getDistortionCoefficients(static_cast<dai::CameraBoardSocket>(socketA));
    auto rightDistortionCoefficients = currentCalibration.getDistortionCoefficients(static_cast<dai::CameraBoardSocket>(socketB));

    std::unique_ptr<dcl::CameraSensorHandle> handleA = createDCLCameraCalibration(leftCameraMatrix, leftDistortionCoefficients, rotationMatrixA, translationVectorA, widthDefault, heightDefault);
    std::unique_ptr<dcl::CameraSensorHandle> handleB = createDCLCameraCalibration(rightCameraMatrix, rightDistortionCoefficients, rotationMatrixB, translationVectorB, widthDefault, heightDefault);

    std::shared_ptr<dcl::Sensor> sensorA = handleA->getSensor();
    std::shared_ptr<dcl::Sensor> sensorB = handleB->getSensor();
    dynCalibImpl->addSensor(deviceName, sensorA, socketA);
    dynCalibImpl->addSensor(deviceName, sensorB, socketB);
}

void DynamicCalibration::run() {

    if(!device) {
        std::cout << "Dynamic calibration node has to have access to a device!" << std::endl;
        return;
    }

    std::cout << "DynamicCalibration node is running" << std::endl;
    while(isRunning()) {
        auto leftFrame = left.get<dai::ImgFrame>();
        auto rightFrame = right.get<dai::ImgFrame>();

        if(!leftFrame || !rightFrame) continue;

        // Get and convert frames
        auto imageA = leftFrame->getCvFrame();
        auto imageB = rightFrame->getCvFrame();
        cv::Mat imgColorA, imgColorB;

        if(imageA.channels() != 3) { // REMOVE DEPENDENCY OF THE 3 CHANNEL IMAGE
            cv::cvtColor(imageA, imgColorA, cv::COLOR_GRAY2BGR);
            cv::cvtColor(imageB, imgColorB, cv::COLOR_GRAY2BGR);
        } else {
            imgColorA = imageA;
            imgColorB = imageB;
        }

        // === STATE MACHINE ===
        switch(calibrationSM.state) {

            case CalibrationState::InitializingPipeline: {
                if(!calibrationSM.pipelineReady) {
                    int width = leftFrame->getWidth();
                    int height = rightFrame->getHeight();
                    CameraBoardSocket leftSocket = static_cast<CameraBoardSocket>(leftFrame->instanceNum);
                    CameraBoardSocket rightSocket = static_cast<CameraBoardSocket>(rightFrame->instanceNum);
                    pipelineSetup(device, leftSocket, rightSocket, width, height);

                    calibrationSM.markPipelineReady();
                    std::cout << "[DynamicCalibration] Pipeline initialized.\n";
                }
                break;
            }

            case CalibrationState::Idle:
                // Do nothing
                break;

            case CalibrationState::CollectingFeatures: { // TODO, BETTER HANDLING OF HOW MANY FPS WE DANNA COLLECT FRAMES
                dcl::timestamp_t timestamp = leftFrame->getTimestamp().time_since_epoch().count();
                dynCalibImpl->loadStereoImagePair(
                    cvMatToImageData(imgColorA),
                    cvMatToImageData(imgColorB),
                    deviceName, socketA, socketB, timestamp
                );

                ++calibrationSM.collectedFrames;
                std::cout << "[DynamicCalibration] Collected " 
                          << calibrationSM.collectedFrames 
                          << (calibrationSM.mode == CalibrationMode::QualityCheck ? " (QC)" : " (Recalibration)")
                          << "\n";

                calibrationSM.maybeAdvanceAfterCollection();
                break;
            }

            case CalibrationState::ProcessingQuality: {
                std::cout << "[DynamicCalibration] Running quality check...\n";
                dcl::Result<double> result = dynCalibImpl->checkCalibration(dcDevice, socketA, socketB);

                results.quality.value = result.value;
                results.quality.valid = result.errorCode == 0 ? true : false;
                results.quality.info = result.errorCode == 0 ? "Calib Quality check complete" // TODO, REPLACE WITH ACTUAL REPORTS ON ERRORS, WHAT CAUSED IT
                                                : "Calib Quality check failed with error code " + std::to_string(result.errorCode);

                std::cout << "[DynamicCalibration] Quality result = " << result.value  << "\n";
                calibrationSM.finish();
                break;
            }

            case CalibrationState::Recalibrating: {
                std::cout << "[DynamicCalibration] Running full recalibration...\n";
                dcl::Result<std::pair<std::shared_ptr<dcl::CameraCalibration>, std::shared_ptr<dcl::CameraCalibration>>> calib = dynCalibImpl->recalibrateDevice(dcDevice, socketA, socketB);
                CalibrationHandler calib12 = device->readCalibration(); // TODO, REPLACE WITH FUNCTION WHICH DOES THE SWITCH BETWEEN CALIB and HANDLER

                results.calibration.valid = calib.errorCode == 0 ? true : false; // TODO, REPLACE WITH ACTUAL REPORTS ON ERRORS, WHAT CAUSED IT
                results.calibration.info = calib.errorCode == 0 ? "Recalibration successful"
                                                : "Recalibration failed with error code " + std::to_string(calib.errorCode);
                results.calibration.calibration = calib12;                       
                std::cout << "[DynamicCalibration] Recalibration complete.\n";
                calibrationSM.finish();
                break;
            }

            // TODO, ADD A METHOD, WHICH SETS NEW CALIBRATION ON DEVICE, DCL AND AS WELL RESETS THE RESULTS IN CALIBRATION STATE
        }
    }
}
}  // namespace node
}  // namespace dai        auto extrinsicsLeftToRight = currentCalibration.getCameraExtrinsics(static_cast<dai::CameraBoardSocket>(leftFrame->instanceNum)
