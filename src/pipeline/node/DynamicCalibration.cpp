#include "depthai/pipeline/node/DynamicCalibration.hpp"
#include "common/CameraBoardSocket.hpp"
#include <DynamicCalibration.hpp>

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

float DynamicCalibration::getCalibQuality() const {
    if(!dynCalibImpl) {
        std::cerr << "Dynamic calibration has not been initialized yet!" << std::endl;
        return -1.0f;
    }
    auto result = dynCalibImpl->checkCalibration();
    float qualityCheck = result.value;
    auto error = result.errorCode;
    std::cout << qualityCheck << "    " << error << std::endl;
    return qualityCheck;
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

auto DynamicCalibration::createDCLCameraCalibration(
    const std::vector<std::vector<float>> cameraMatrix,
    const std::vector<float> distortionCoefficients,
    const std::vector<std::vector<float>> rotationMatrix,
    const std::vector<float> translationVector) {

    dcl::extrinsics_t extrinsics;
    dcl::intrinsics_t intrinsics;

    // --- Convert translation vector (3x1)
    if(translationVector.size() != 3)
        throw std::invalid_argument("translationVector must have 3 elements");
    for(int i = 0; i < 3; ++i)
        extrinsics.tvec(i) = translationVector[i] * 10.0f;

    // --- Convert rotation matrix (3x3) → rotation vector (3x1)
    if(rotationMatrix.size() != 3 || rotationMatrix[0].size() != 3)
        throw std::invalid_argument("rotationMatrix must be 3x3");

    std::vector<float> rvec = rotationMatrixToVector(rotationMatrix); 
    for(int i = 0; i < 3; ++i)
        extrinsics.rvec(i) = rvec[i];

    // --- Convert cameraMatrix (3x3) → flat 9-element matrix
    if(cameraMatrix.size() != 3 || cameraMatrix[0].size() != 3)
        throw std::invalid_argument("cameraMatrix must be 3x3");

    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
            intrinsics.cameraMatrix(i, j)= cameraMatrix[i][j];

    // --- Copy distortionCoefficients (14 elements)
    if(distortionCoefficients.size() != 14)
        throw std::invalid_argument("distortionCoefficients must have 14 elements");
    for(int i = 0; i < 14; ++i)
        intrinsics.distortion(i) = distortionCoefficients[i];

    // Create and return calibration object
    auto camera_calib = std::make_shared<dcl::CameraCalibration>(extrinsics, intrinsics);
    return camera_calib;
}

void DynamicCalibration::pipelineSetup(std::shared_ptr<Device> device, CameraBoardSocket leftSocket, CameraBoardSocket rightSocket, int widthDefault, int heightDefault){
    auto currentCalibration = device->readCalibration();
    deviceName = device->getDeviceId();

    socketA = static_cast<int>(leftSocket);
    socketB = static_cast<int>(rightSocket);

    dynCalibImpl = std::make_unique<dcl::DynamicCalibration>();
    std::shared_ptr<dcl::Device> dcDevice = dynCalibImpl->addDevice(deviceName);

    const dcl::resolution_t resolutionA = { .width = static_cast<unsigned int>(widthDefault), .height = static_cast<unsigned int>(heightDefault) };
    const dcl::resolution_t resolutionB = { .width = static_cast<unsigned int>(widthDefault), .height = static_cast<unsigned int>(heightDefault) };

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

    auto camera_calibA = createDCLCameraCalibration(leftCameraMatrix, leftDistortionCoefficients, rotationMatrixA, translationVectorA);
    auto camera_calibB = createDCLCameraCalibration(rightCameraMatrix, rightDistortionCoefficients, rotationMatrixB, translationVectorB);

    auto cam_sensorA = std::make_shared<dcl::CameraSensor>(camera_calibA, resolutionA);
    auto cam_sensorB = std::make_shared<dcl::CameraSensor>(camera_calibB, resolutionB);

    dynCalibImpl->addSensor(deviceName, cam_sensorA, socketA);
    dynCalibImpl->addSensor(deviceName, cam_sensorB, socketB);
}

void DynamicCalibration::run() {

    if(!device) {
        std::cout << "Dynamic calibration node has to have access to a device!" << std::endl;
        return;
    }

    std::cout << "DynamicCalibration node is running" << std::endl; 
    int frameCount = 0;
    bool pipelineCreated = false;
    int width = 0;
    int height = 0;
    while(isRunning()) {
        auto leftFrame = left.get<dai::ImgFrame>();
        auto rightFrame = right.get<dai::ImgFrame>();

        if (!leftFrame || !rightFrame) continue;

        if (!pipelineCreated){
            width = leftFrame->getWidth();
            height = leftFrame->getHeight();
            CameraBoardSocket leftSocket =static_cast<dai::CameraBoardSocket>(leftFrame->instanceNum);
            CameraBoardSocket rightSocket =static_cast<dai::CameraBoardSocket>(rightFrame->instanceNum);
            pipelineSetup(device, leftSocket, rightSocket, width, height);
            pipelineCreated = true;
        }

        if (!dynCalibImpl) continue;

        auto imageA = leftFrame->getCvFrame();
        auto imageB = rightFrame->getCvFrame();
        cv::Mat imgColorA, imgColorB;
        if (imageA.channels() != 3) {
            cv::cvtColor(imageA, imgColorA, cv::COLOR_GRAY2BGR);
            cv::cvtColor(imageB, imgColorB, cv::COLOR_GRAY2BGR);
        }
        else {
            imgColorA = imageA;
            imgColorB = imageB;
        }

        ++frameCount;
        if(frameCount > initialSkipFrames && (frameCount - initialSkipFrames) % processEveryNFrames == 0) {
            dcl::timestamp_t timestamp = leftFrame->getTimestamp().time_since_epoch().count();
            dynCalibImpl->loadStereoImagePair(imgColorA, imgColorB, deviceName, socketA, socketB, timestamp);
        }
    }
}
}  // namespace node
}  // namespace dai        auto extrinsicsLeftToRight = currentCalibration.getCameraExtrinsics(static_cast<dai::CameraBoardSocket>(leftFrame->instanceNum)
