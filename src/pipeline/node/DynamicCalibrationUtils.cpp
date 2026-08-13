#include "pipeline/node/DynamicCalibrationUtils.hpp"

#include <DynamicCalibration.hpp>

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/core.hpp>
#endif

#include <algorithm>
#include <stdexcept>
#include <string>

#include "depthai/utility/matrixOps.hpp"

namespace dai {
namespace node {

std::vector<std::vector<float>> DclUtils::calibrationHandleToTransform(const std::shared_ptr<const dcl::CameraCalibrationHandle>& calibration) {
    dcl::scalar_t rvec[3];
    calibration->getRvec(rvec);
    const double rvecDouble[3] = {static_cast<double>(rvec[0]), static_cast<double>(rvec[1]), static_cast<double>(rvec[2])};

    dcl::scalar_t tvec[3];
    calibration->getTvec(tvec);
    return matrix::toVecMatrix4x4(matrix::createTransformationMatrix(
        matrix::rvecToRotationMatrix(rvecDouble), dai::Point3f{static_cast<float>(tvec[0]), static_cast<float>(tvec[1]), static_cast<float>(tvec[2])}));
}

std::vector<CameraBoardSocket> DclUtils::buildSocketConnection(const EepromData& eepromData) {
    std::vector<CameraBoardSocket> referencedSockets;
    referencedSockets.reserve(eepromData.cameraData.size());
    for(const auto& [socket, info] : eepromData.cameraData) {
        (void)socket;
        if(info.extrinsics.toCameraSocket != CameraBoardSocket::AUTO) {
            referencedSockets.push_back(info.extrinsics.toCameraSocket);
        }
    }

    std::vector<CameraBoardSocket> headCandidates;
    headCandidates.reserve(eepromData.cameraData.size());
    for(const auto& [socket, info] : eepromData.cameraData) {
        (void)info;
        if(std::find(referencedSockets.begin(), referencedSockets.end(), socket) == referencedSockets.end()) {
            headCandidates.push_back(socket);
        }
    }

    if(headCandidates.size() != 1) {
        throw std::runtime_error("Calibration must contain a single socketConnection head.");
    }

    std::vector<CameraBoardSocket> connection;
    connection.reserve(eepromData.cameraData.size());

    auto currentSocket = headCandidates.front();
    while(currentSocket != CameraBoardSocket::AUTO) {
        if(std::find(connection.begin(), connection.end(), currentSocket) != connection.end()) {
            throw std::runtime_error("Cyclic socketConnection detected in calibration.");
        }

        const auto currentIt = eepromData.cameraData.find(currentSocket);
        if(currentIt == eepromData.cameraData.end()) {
            throw std::runtime_error("socketConnection references a socket missing from calibration.");
        }

        connection.push_back(currentSocket);
        currentSocket = currentIt->second.extrinsics.toCameraSocket;
    }

    if(connection.size() != eepromData.cameraData.size()) {
        throw std::runtime_error("Calibration socketConnection does not cover all calibrated sockets.");
    }

    return connection;
}

std::vector<std::vector<float>> DclUtils::computeBaseToSocketTransform(const CalibrationHandler& currentCalibration,
                                                                       const std::variant<CameraBoardSocket, HousingCoordinateSystem>& boardSocketBase,
                                                                       CameraBoardSocket boardSocket) {
    if(const auto* cameraBase = std::get_if<CameraBoardSocket>(&boardSocketBase)) {
        if(*cameraBase == boardSocket) {
            return {{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};
        }
        return currentCalibration.getCameraExtrinsics(*cameraBase, boardSocket, false, LengthUnit::METER);
    }

    auto socketToHousingTransform = currentCalibration.getHousingCalibration(boardSocket, HousingCoordinateSystem::AUTO, false, LengthUnit::METER);
    matrix::invertSe3Matrix4x4InPlace(socketToHousingTransform);
    return socketToHousingTransform;
}

std::shared_ptr<dcl::CameraCalibrationHandle> DclUtils::convertDaiCalibrationToDcl(const CalibrationHandler& currentCalibration,
                                                                                   std::variant<CameraBoardSocket, HousingCoordinateSystem> boardSocketBase,
                                                                                   CameraBoardSocket boardSocket,
                                                                                   const std::vector<std::vector<float>>& intrinsicsOverride,
                                                                                   const std::vector<float>& distortionOverride,
                                                                                   const CameraModel distortionModelOverride) {
    const auto baseToSocketTransform = DclUtils::computeBaseToSocketTransform(currentCalibration, boardSocketBase, boardSocket);

    return DclUtils::createDclCalibration(matrix::vectorMatrixToMatrix3x3(intrinsicsOverride),
                                          distortionOverride,
                                          matrix::extractRotationMatrix(baseToSocketTransform),
                                          matrix::extractTranslationVector(baseToSocketTransform),
                                          distortionModelOverride);
}

void DclUtils::setHousingToDai(CalibrationHandler& calibHandler, const std::vector<std::vector<float>>& transformHousingToHousingOrigin) {
    auto translationHousingToHousingOrigin = matrix::extractTranslationVector(transformHousingToHousingOrigin);
    calibHandler.setHousingToHousingOriginExtrinsics(
        matrix::extractRotationMatrix(transformHousingToHousingOrigin), translationHousingToHousingOrigin, LengthUnit::METER);
}

dcl::PerformanceMode DclUtils::daiPerformanceModeToDclPerformanceMode(const dai::DynamicCalibrationControl::PerformanceMode mode) {
    switch(mode) {
        case DynamicCalibrationControl::PerformanceMode::DEFAULT:
            return dcl::PerformanceMode::DEFAULT;
        case DynamicCalibrationControl::PerformanceMode::STATIC_SCENERY:
            return dcl::PerformanceMode::STATIC_SCENERY;
        case DynamicCalibrationControl::PerformanceMode::OPTIMIZE_SPEED:
            return dcl::PerformanceMode::OPTIMIZE_SPEED;
        case DynamicCalibrationControl::PerformanceMode::OPTIMIZE_PERFORMANCE:
            return dcl::PerformanceMode::OPTIMIZE_PERFORMANCE;
        case DynamicCalibrationControl::PerformanceMode::SKIP_CHECKS:
            return dcl::PerformanceMode::SKIP_CHECKS;
        case DynamicCalibrationControl::PerformanceMode::RELAXED_COVERAGE:
            return dcl::PerformanceMode::RELAXED_COVERAGE;
        default:
            throw std::invalid_argument("Unknown PerformanceMode");
    }
}

std::shared_ptr<dcl::CameraCalibrationHandle> DclUtils::createDclCalibration(const std::array<std::array<float, 3>, 3>& cameraMatrix,
                                                                             const std::vector<float>& distortionCoefficients,
                                                                             const std::vector<std::vector<float>>& rotationMatrix,
                                                                             const std::vector<float>& translationVector,
                                                                             const CameraModel distortionModel) {
    dcl::scalar_t cameraMatrixArr[9];
    dcl::scalar_t rvec[3];
    dcl::scalar_t tvec[3];

    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            cameraMatrixArr[i * 3 + j] = static_cast<dcl::scalar_t>(cameraMatrix[i][j]);
        }
    }

    const std::vector<float> rvecVec = matrix::rotationMatrixToVector(rotationMatrix);
    for(int i = 0; i < 3; ++i) {
        rvec[i] = static_cast<dcl::scalar_t>(rvecVec[i]);
        tvec[i] = static_cast<dcl::scalar_t>(translationVector[i]);
    }

    dcl::distortion_t dclDistortion;
    if(distortionModel == dai::CameraModel::Perspective) {
        constexpr std::size_t perspectiveDistortionSize = 14;
        dcl::scalar_t distortion[perspectiveDistortionSize] = {0};
        for(std::size_t i = 0; i < perspectiveDistortionSize; ++i) {
            distortion[i] = static_cast<dcl::scalar_t>(distortionCoefficients[i]);
        }
        dclDistortion = dcl::PerspectiveDistortion::fromArray(distortion);
    } else if(distortionModel == dai::CameraModel::Fisheye) {
        constexpr std::size_t fisheyeDistortionSize = 4;
        dcl::scalar_t distortion[fisheyeDistortionSize] = {0};
        for(std::size_t i = 0; i < fisheyeDistortionSize; ++i) {
            distortion[i] = static_cast<dcl::scalar_t>(distortionCoefficients[i]);
        }
        dclDistortion = dcl::FisheyeDistortion::fromArray(distortion);
    } else {
        throw std::runtime_error("Unsupported distortion model");
    }
    return std::make_shared<dcl::CameraCalibrationHandle>(rvec, tvec, cameraMatrixArr, dclDistortion);
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
dcl::ImageData DclUtils::cvMatToImageData(const cv::Mat& mat) {
    if(mat.empty()) {
        throw std::runtime_error("cv::Mat is empty");
    }

    dcl::ImageData img;
    img.width = static_cast<unsigned int>(mat.cols);
    img.height = static_cast<unsigned int>(mat.rows);

    switch(mat.type()) {
        case CV_8UC1:
            img.format = dcl::DCL_8UC1;
            break;
        case CV_8UC3:
            img.format = dcl::DCL_8UC3;
            break;
        default:
            throw std::runtime_error("Unsupported cv::Mat type: " + std::to_string(mat.type()));
    }
    img.data.assign(mat.data, mat.data + mat.total() * mat.elemSize());
    return img;
}
#endif

}  // namespace node
}  // namespace dai
