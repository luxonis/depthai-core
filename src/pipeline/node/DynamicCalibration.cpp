#include "depthai/pipeline/node/DynamicCalibration.hpp"
#include "common/CameraBoardSocket.hpp"
#include <DynamicCalibration.hpp>
#include <CalibrationHandle.hpp>

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

float DynamicCalibration::getCalibQuality() const {
    if(!dynCalibImpl) {
        std::cerr << "Dynamic calibration has not been initialized yet!" << std::endl;
        return -1.0f;
    }
    //auto result = dynCalibImpl->checkCalibration();
    //float qualityCheck = result.value;
    //auto error = result.errorCode;
    float qualityCheck = 1;
    //std::cout << qualityCheck << "    " << error << std::endl;
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

    dcl::scalar_t cameraMatrixArr[9];
    dcl::scalar_t distortion[14] = {0};
    dcl::scalar_t rvec[3];
    dcl::scalar_t tvec[3];

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

    auto camera_calib = std::make_shared<dcl::CameraCalibrationHandle>(
        rvec, tvec, cameraMatrixArr, distortion);

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

    auto sensorA = camera_calibA->createCameraSensor(camera_calibA->getCameraCalibration(), resolutionA);
    auto sensorB = camera_calibB->createCameraSensor(camera_calibB->getCameraCalibration(), resolutionB);
    dynCalibImpl->addSensor(deviceName, sensorA, socketA);
    dynCalibImpl->addSensor(deviceName, sensorB, socketB);
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
            dynCalibImpl->loadStereoImagePair(cvMatToImageData(imgColorA), cvMatToImageData(imgColorB), deviceName, socketA, socketB, timestamp);
        }
    }
}
}  // namespace node
}  // namespace dai        auto extrinsicsLeftToRight = currentCalibration.getCameraExtrinsics(static_cast<dai::CameraBoardSocket>(leftFrame->instanceNum)
