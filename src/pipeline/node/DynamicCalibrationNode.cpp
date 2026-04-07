#include "depthai/pipeline/node/DynamicCalibrationNode.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/opencv.hpp>
#endif
#include <algorithm>
#include <iomanip>
#include <pipeline/ThreadedNodeImpl.hpp>
#include <sstream>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationControl.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationResults.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/utility/matrixOps.hpp"
#include "depthai/utility/spimpl.h"
#include "pipeline/node/DynamicCalibrationUtils.hpp"

namespace dai {
namespace node {

namespace {

std::vector<std::vector<float>> extractRotationMatrix(const std::vector<std::vector<float>>& transform) {
    std::vector<std::vector<float>> rotation(3, std::vector<float>(3, 0.0f));
    for(int row = 0; row < 3; ++row) {
        for(int col = 0; col < 3; ++col) {
            rotation[row][col] = transform[row][col];
        }
    }
    return rotation;
}

std::vector<float> extractTranslationVector(const std::vector<std::vector<float>>& transform) {
    std::vector<float> translation(3, 0.0f);
    for(int row = 0; row < 3; ++row) {
        translation[row] = transform[row][3];
    }
    return translation;
}

std::vector<std::vector<float>> intrinsicArrayToMatrix(const std::array<std::array<float, 3>, 3>& intrinsicMatrix) {
    std::vector<std::vector<float>> matrix(3, std::vector<float>(3, 0.0f));
    for(int row = 0; row < 3; ++row) {
        for(int col = 0; col < 3; ++col) {
            matrix[row][col] = intrinsicMatrix[row][col];
        }
    }
    return matrix;
}

std::vector<std::vector<float>> subtractMatrices(const std::vector<std::vector<float>>& lhs, const std::vector<std::vector<float>>& rhs) {
    std::vector<std::vector<float>> delta(3, std::vector<float>(3, 0.0f));
    for(int row = 0; row < 3; ++row) {
        for(int col = 0; col < 3; ++col) {
            delta[row][col] = lhs[row][col] - rhs[row][col];
        }
    }
    return delta;
}

std::string formatMatrix3x3(const std::vector<std::vector<float>>& matrix) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(4) << "[";
    for(size_t row = 0; row < matrix.size(); ++row) {
        if(row != 0) stream << " ";
        stream << "[";
        for(size_t col = 0; col < matrix[row].size(); ++col) {
            if(col != 0) stream << ", ";
            stream << matrix[row][col];
        }
        stream << "]";
        if(row + 1 != matrix.size()) stream << ",";
    }
    stream << "]";
    return stream.str();
}

std::vector<std::vector<float>> calibrationHandleToTransform(const std::shared_ptr<const dcl::CameraCalibrationHandle>& calibration) {
    dcl::scalar_t rvec[3];
    calibration->getRvec(rvec);
    const double rvecDouble[3] = {static_cast<double>(rvec[0]), static_cast<double>(rvec[1]), static_cast<double>(rvec[2])};

    auto transform = matrix::rvecToRotationMatrix(rvecDouble);
    for(auto& row : transform) {
        row.push_back(0.0f);
    }

    dcl::scalar_t tvec[3];
    calibration->getTvec(tvec);
    for(int row = 0; row < 3; ++row) {
        transform[row][3] = static_cast<float>(tvec[row]);
    }
    transform.push_back({0.0f, 0.0f, 0.0f, 1.0f});
    return transform;
}

std::vector<std::vector<float>> invertSe3Transform(const std::vector<std::vector<float>>& transform) {
    auto inverted = transform;

    std::swap(inverted[0][1], inverted[1][0]);
    std::swap(inverted[0][2], inverted[2][0]);
    std::swap(inverted[1][2], inverted[2][1]);

    float translated[3] = {0.0f, 0.0f, 0.0f};
    for(int row = 0; row < 3; ++row) {
        for(int col = 0; col < 3; ++col) {
            translated[row] -= inverted[row][col] * transform[col][3];
        }
        inverted[row][3] = translated[row];
    }

    return inverted;
}

std::vector<CameraBoardSocket> buildSocketConnection(const EepromData& eepromData) {
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

}  // namespace

class DynamicCalibration::Impl {
   public:
    /**
     * DCL held properties
     */
    std::shared_ptr<dcl::CameraSensorHandle> sensorLeft;
    std::shared_ptr<dcl::CameraSensorHandle> sensorRight;
    std::vector<std::shared_ptr<dcl::CameraSensorHandle>> sensors;
    std::shared_ptr<dcl::Device> device;

    dcl::DynamicCalibration dynCalibImpl;
};

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
    pimplDCL = spimpl::make_impl<Impl>();
    logger = pimpl->logger;
    sync->out.link(syncInput);
    sync->setRunOnHost(true);
}

std::shared_ptr<dcl::CameraCalibrationHandle> DclUtils::convertDaiCalibrationToDcl(const CalibrationHandler& currentCalibration,
                                                                                   std::variant<CameraBoardSocket, HousingCoordinateSystem> boardSocketBase,
                                                                                   CameraBoardSocket boardSocket,
                                                                                   const std::pair<int, int>& resolution,
                                                                                   const std::vector<std::vector<float>>* intrinsicsOverride,
                                                                                   std::vector<std::vector<float>>* transformBaseToSocket) {
    std::vector<std::vector<float>> baseToSocketTransform;

    if(const auto* cameraBase = std::get_if<CameraBoardSocket>(&boardSocketBase)) {
        if(*cameraBase == boardSocket) {
            baseToSocketTransform = {{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};
        } else {
            baseToSocketTransform = currentCalibration.getCameraExtrinsics(*cameraBase, boardSocket, false, LengthUnit::METER);
        }
    } else {
        const auto socketToHousingTransform = currentCalibration.getHousingCalibration(boardSocket, HousingCoordinateSystem::AUTO, false, LengthUnit::METER);
        baseToSocketTransform = invertSe3Transform(socketToHousingTransform);
    }

    if(transformBaseToSocket != nullptr) {
        *transformBaseToSocket = baseToSocketTransform;
    }

    const auto intrinsics = intrinsicsOverride != nullptr
                                ? *intrinsicsOverride
                                : currentCalibration.getCameraIntrinsics(boardSocket, resolution.first, resolution.second, Point2f(), Point2f(), false);

    return DclUtils::createDclCalibration(intrinsics,
                                          currentCalibration.getDistortionCoefficients(boardSocket),
                                          extractRotationMatrix(baseToSocketTransform),
                                          extractTranslationVector(baseToSocketTransform),
                                          currentCalibration.getDistortionModel(boardSocket));
}

std::pair<std::shared_ptr<dcl::CameraCalibrationHandle>, std::shared_ptr<dcl::CameraCalibrationHandle>> DclUtils::convertDaiCalibrationToDcl(
    const CalibrationHandler& currentCalibration,
    std::variant<CameraBoardSocket, HousingCoordinateSystem> boardSocketBase,
    CameraBoardSocket boardSocketLeft,
    CameraBoardSocket boardSocketRight,
    const std::pair<int, int>& resolutionLeft,
    const std::pair<int, int>& resolutionRight) {
    std::shared_ptr<dcl::CameraCalibrationHandle> calibLeft =
        DclUtils::convertDaiCalibrationToDcl(currentCalibration, boardSocketBase, boardSocketLeft, resolutionLeft);
    std::shared_ptr<dcl::CameraCalibrationHandle> calibRight =
        DclUtils::convertDaiCalibrationToDcl(currentCalibration, boardSocketBase, boardSocketRight, resolutionRight);
    return std::make_pair(calibLeft, calibRight);
}

void DclUtils::setHousingToDai(CalibrationHandler& calibHandler, const std::vector<std::vector<float>>& transformHousingToHousingOrigin) {
    auto translationHousingToHousingOrigin = extractTranslationVector(transformHousingToHousingOrigin);
    for(auto& val : translationHousingToHousingOrigin) {
        val *= 100.0f;
    }
    calibHandler.setHousingToHousingOriginExtrinsics(extractRotationMatrix(transformHousingToHousingOrigin), translationHousingToHousingOrigin);
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
        default:
            throw std::invalid_argument("Unknown PerformanceMode");
    }
}

#define DCL_PERSPECTIVE_DISTORTION_SIZE (14)
#define DCL_FISHEYE_DISTORTION_SIZE (4)
std::shared_ptr<dcl::CameraCalibrationHandle> DclUtils::createDclCalibration(const std::vector<std::vector<float>>& cameraMatrix,
                                                                             const std::vector<float>& distortionCoefficients,
                                                                             const std::vector<std::vector<float>>& rotationMatrix,
                                                                             const std::vector<float>& translationVector,
                                                                             const CameraModel distortionModel) {
    dcl::scalar_t cameraMatrixArr[9];
    dcl::scalar_t rvec[3];
    dcl::scalar_t tvec[3];

    // Convert cameraMatrix
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            cameraMatrixArr[i * 3 + j] = static_cast<dcl::scalar_t>(cameraMatrix[i][j]);
        }
    }

    // Convert rotation to vector
    std::vector<float> rvecVec = matrix::rotationMatrixToVector(rotationMatrix);
    for(int i = 0; i < 3; ++i) {
        rvec[i] = static_cast<dcl::scalar_t>(rvecVec[i]);
    }

    for(int i = 0; i < 3; ++i) {
        tvec[i] = static_cast<dcl::scalar_t>(translationVector[i]);
    }

    dcl::distortion_t dclDistortion;
    if(distortionModel == dai::CameraModel::Perspective) {
        // Convert distortion
        dcl::scalar_t distortion[DCL_PERSPECTIVE_DISTORTION_SIZE] = {0};
        for(size_t i = 0; i < DCL_PERSPECTIVE_DISTORTION_SIZE; ++i) {
            distortion[i] = static_cast<dcl::scalar_t>(distortionCoefficients[i]);
        }
        dclDistortion = dcl::PerspectiveDistortion::fromArray(distortion);
    } else if(distortionModel == dai::CameraModel::Fisheye) {
        dcl::scalar_t distortion[DCL_FISHEYE_DISTORTION_SIZE] = {0};
        for(size_t i = 0; i < DCL_FISHEYE_DISTORTION_SIZE; ++i) {
            distortion[i] = static_cast<dcl::scalar_t>(distortionCoefficients[i]);
        }
        dclDistortion = dcl::FisheyeDistortion::fromArray(distortion);
    } else {
        throw std::runtime_error("Unsupported distortion model");
    }
    return std::make_shared<dcl::CameraCalibrationHandle>(rvec, tvec, cameraMatrixArr, dclDistortion);
}

std::vector<float> distortionToVector(const dcl::distortion_t& dist) {
    return std::visit(
        [](auto&& d) -> std::vector<float> {
            using T = std::decay_t<decltype(d)>;
            if constexpr(std::is_same_v<T, dcl::FisheyeDistortion>) {
                return std::vector<float>(std::begin(d.data), std::end(d.data));
            } else if constexpr(std::is_same_v<T, dcl::PerspectiveDistortion>) {
                return std::vector<float>(std::begin(d.data), std::end(d.data));
            }
        },
        dist);
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
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
#endif

dai::CalibrationQuality calibQualityfromDCL(const dcl::CalibrationDifference& src) {
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

void DynamicCalibration::setCalibration(CalibrationHandler& handler, bool flash) {
    logger->trace("Applying calibration to device");
    device->setCalibration(handler);
    if(flash) {
        device->flashCalibration(handler);
    }
    if(pimplDCL->sensors.size() == socketConnection.size() && !socketConnection.empty()) {
        socketToSensorExtrinsics.clear();
        socketToSensorExtrinsics.reserve(socketConnection.size());

        for(size_t idx = 0; idx < socketConnection.size(); ++idx) {
            std::vector<std::vector<float>> baseToSocketTransform;
            const auto intrinsicsIt = socketToFrameIntrinsics.find(socketConnection[idx]);
            const auto* intrinsicsOverride = intrinsicsIt != socketToFrameIntrinsics.end() ? &intrinsicsIt->second : nullptr;
            auto calibration = DclUtils::convertDaiCalibrationToDcl(
                handler, daiSocketBase, socketConnection[idx], socketConnectionResolutions[idx], intrinsicsOverride, &baseToSocketTransform);
            socketToSensorExtrinsics.push_back(baseToSocketTransform);
            pimplDCL->dynCalibImpl.setCalibration(pimplDCL->sensors[idx], calibration);
        }
    } else {
        const auto leftIntrinsicsIt = socketToFrameIntrinsics.find(daiSocketLeft);
        const auto rightIntrinsicsIt = socketToFrameIntrinsics.find(daiSocketRight);
        auto calibLeft = DclUtils::convertDaiCalibrationToDcl(handler, daiSocketBase, daiSocketLeft, resolutionLeft, &leftIntrinsicsIt->second);
        auto calibRight = DclUtils::convertDaiCalibrationToDcl(handler, daiSocketBase, daiSocketRight, resolutionRight, &rightIntrinsicsIt->second);
        pimplDCL->dynCalibImpl.setCalibration(pimplDCL->sensorLeft, calibLeft);
        pimplDCL->dynCalibImpl.setCalibration(pimplDCL->sensorRight, calibRight);
    }
}

void DynamicCalibration::computeMetrics(const CalibrationHandler& handler) {
    const auto leftIntrinsicsIt = socketToFrameIntrinsics.find(daiSocketLeft);
    const auto rightIntrinsicsIt = socketToFrameIntrinsics.find(daiSocketRight);
    auto calibLeft = DclUtils::convertDaiCalibrationToDcl(handler, daiSocketBase, daiSocketLeft, resolutionLeft, &leftIntrinsicsIt->second);
    auto calibRight = DclUtils::convertDaiCalibrationToDcl(handler, daiSocketBase, daiSocketRight, resolutionRight, &rightIntrinsicsIt->second);
    auto dataConfidence = pimplDCL->dynCalibImpl.computeDataConfidence(pimplDCL->sensorLeft, pimplDCL->sensorRight);
    auto calibrationConfidence = pimplDCL->dynCalibImpl.computeCalibrationConfidence(calibLeft, calibRight, pimplDCL->sensorLeft, pimplDCL->sensorRight);
    auto metrics = std::make_shared<CalibrationMetrics>();
    if(!dataConfidence.passed()) {
        metrics->dataConfidence = 0.;
    } else {
        metrics->dataConfidence = dataConfidence.value;
    }
    if(!calibrationConfidence.passed()) {
        metrics->calibrationConfidence = 0.;
    } else {
        metrics->calibrationConfidence = calibrationConfidence.value;
    }
    metricsOutput.send(metrics);
}

DynamicCalibration::ErrorCode DynamicCalibration::runQualityCheck(const bool force) {
    dcl::PerformanceMode pm = force ? dcl::PerformanceMode::SKIP_CHECKS : DclUtils::daiPerformanceModeToDclPerformanceMode(performanceMode);
    logger->trace("Running calibration quality check (force={} mode={})", force, static_cast<int>(pm));

    auto dclResult = pimplDCL->dynCalibImpl.checkCalibration(pimplDCL->sensorLeft, pimplDCL->sensorRight, pm);

    if(!dclResult.passed()) {
        auto result = std::make_shared<CalibrationQuality>();
        result->info = dclResult.errorMessage();
        logger->trace("Quality check failed: {}", dclResult.errorMessage());

        qualityOutput.send(result);
        return DynamicCalibration::ErrorCode::QUALITY_CHECK_FAILED;
    }

    auto result = std::make_shared<CalibrationQuality>(calibQualityfromDCL(dclResult.value));
    result->info = dclResult.errorMessage();
    logger->trace("Quality check passed.");

    qualityOutput.send(result);

    return DynamicCalibration::ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::runCalibration(const dai::CalibrationHandler& currentHandler, const bool force) {
    dcl::PerformanceMode pm = force ? dcl::PerformanceMode::SKIP_CHECKS : DclUtils::daiPerformanceModeToDclPerformanceMode(performanceMode);
    logger->trace("Running calibration (force={} mode={})", force, static_cast<int>(pm));

    std::vector<std::shared_ptr<const dcl::CameraSensorHandle>> syncedSensors;
    syncedSensors.reserve(sockets.size());
    for(const auto& socket : sockets) {
        syncedSensors.push_back(pimplDCL->sensors.at(socketToIdx.at(socket)));
    }

    auto dclResult = pimplDCL->dynCalibImpl.findNewCalibration(syncedSensors, pm);
    if(!dclResult.passed()) {
        auto result = std::make_shared<DynamicCalibrationResult>(dclResult.errorMessage());
        logger->warn("Calibration failed: {}", dclResult.errorMessage());

        calibrationOutput.send(result);
        return DynamicCalibration::ErrorCode::CALIBRATION_FAILED;
    }

    if(dclResult.value.size() != syncedSensors.size()) {
        auto result = std::make_shared<DynamicCalibrationResult>("DCL returned calibration count mismatch.");
        calibrationOutput.send(result);
        return DynamicCalibration::ErrorCode::CALIBRATION_FAILED;
    }

    auto newCalibrationHandler = currentHandler;

    for(size_t idx = 0; idx < sockets.size(); ++idx) {
        const auto& calibration = dclResult.value[idx];
        const auto socketIdx = socketToIdx.at(sockets[idx]);
        socketToSensorExtrinsics[socketIdx] = calibrationHandleToTransform(calibration);
        pimplDCL->dynCalibImpl.setCalibration(pimplDCL->sensors.at(socketIdx), calibration);

        // dcl::distortion_t distortion;
        // calibration->getDistortion(distortion);

        // dcl::scalar_t cameraMatrix[9];
        // calibration->getCameraMatrix(cameraMatrix);
        // std::vector<std::vector<float>> mat = {{static_cast<float>(cameraMatrix[0]), static_cast<float>(cameraMatrix[1]),
        // static_cast<float>(cameraMatrix[2])},
        //                                        {static_cast<float>(cameraMatrix[3]), static_cast<float>(cameraMatrix[4]),
        //                                        static_cast<float>(cameraMatrix[5])}, {static_cast<float>(cameraMatrix[6]),
        //                                        static_cast<float>(cameraMatrix[7]), static_cast<float>(cameraMatrix[8])}};

        // newCalibrationHandler.setCameraIntrinsics(sockets[idx], mat, resolutions[idx].first, resolutions[idx].second);
        // newCalibrationHandler.setDistortionCoefficients(sockets[idx], distortionToVector(distortion));
    }

    for(size_t idx = 0; idx + 1 < socketConnection.size(); ++idx) {
        auto transformCurrentToBase = invertSe3Transform(socketToSensorExtrinsics[idx]);
        const auto transformCurrentToNext = matrix::matMul(socketToSensorExtrinsics[idx + 1], transformCurrentToBase);
        auto translationCurrentToNext = extractTranslationVector(transformCurrentToNext);
        for(auto& val : translationCurrentToNext) {
            val *= 100.0f;
        }

        std::vector<float> specTranslationCurrentToNext;
        try {
            specTranslationCurrentToNext =
                currentHandler.getCameraTranslationVector(socketConnection[idx], socketConnection[idx + 1], true, LengthUnit::CENTIMETER);
        } catch(const std::exception&) {
            specTranslationCurrentToNext = translationCurrentToNext;
        }

        newCalibrationHandler.overwriteCameraExtrinsics(
            socketConnection[idx], socketConnection[idx + 1], extractRotationMatrix(transformCurrentToNext), translationCurrentToNext);
    }

    if(std::holds_alternative<HousingCoordinateSystem>(daiSocketBase)) {
        const auto housingOriginSocket = currentHandler.getEepromData().housingExtrinsics.toCameraSocket;
        const auto housingOriginIt = std::find(socketConnection.begin(), socketConnection.end(), housingOriginSocket);
        if(housingOriginIt == socketConnection.end()) {
            auto result = std::make_shared<DynamicCalibrationResult>("Housing origin socket is missing from socketConnection.");
            calibrationOutput.send(result);
            return DynamicCalibration::ErrorCode::CALIBRATION_FAILED;
        }

        const auto transformHousingToHousingOrigin = socketToSensorExtrinsics.at(static_cast<size_t>(std::distance(socketConnection.begin(), housingOriginIt)));
        DclUtils::setHousingToDai(newCalibrationHandler, transformHousingToHousingOrigin);
    }

    CalibrationQuality::Data qualityData{};
    DynamicCalibrationResult::Data resultData{};
    resultData.newCalibration = newCalibrationHandler;
    resultData.currentCalibration = currentHandler;
    resultData.calibrationDifference = qualityData;

    auto dataConfidence = pimplDCL->dynCalibImpl.computeDataConfidence(pimplDCL->sensorLeft, pimplDCL->sensorRight);
    resultData.dataConfidence = dataConfidence.passed() ? dataConfidence.value : 0.0;

    auto result = std::make_shared<DynamicCalibrationResult>(resultData, dclResult.errorMessage());
    calibrationOutput.send(result);

    return DynamicCalibration::ErrorCode::OK;
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
DynamicCalibration::ErrorCode DynamicCalibration::runLoadImage(const bool blocking) {
    std::shared_ptr<dai::MessageGroup> inSyncGroup;
    logger->trace("Attempting to load synced image set (blocking={})", blocking);
    if(!blocking) {
        inSyncGroup = syncInput.tryGet<dai::MessageGroup>();
    } else {
        slept = true;
        inSyncGroup = syncInput.get<dai::MessageGroup>();
    }
    if(!inSyncGroup) {
        return DynamicCalibration::ErrorCode::EMPTY_IMAGE_QUEUE;
    }

    dcl::DeviceImageList images;
    images.reserve(names.size());

    dcl::timestamp_t timestamp = 0;
    bool timestampInitialized = false;

    for(size_t idx = 0; idx < names.size(); ++idx) {
        const auto frame = inSyncGroup->get<dai::ImgFrame>(names[idx]);
        if(!frame) {
            logger->trace("Missing image '{}' in MessageGroup", names[idx]);
            return DynamicCalibration::ErrorCode::MISSING_IMAGE;
        }

        if(!timestampInitialized) {
            timestamp = static_cast<dcl::timestamp_t>(frame->getTimestamp().time_since_epoch().count());
            timestampInitialized = true;
        }

        cv::Mat cvFrame = frame->getCvFrame();
        images.emplace_back(pimplDCL->sensors.at(socketToIdx.at(sockets[idx])), DclUtils::cvMatToImageData(cvFrame));
    }

    auto loadResult = pimplDCL->dynCalibImpl.loadImages(images, timestamp);
    if(!loadResult.passed()) {
        logger->trace("Failed to load synced image set: {}", loadResult.errorMessage());
        return DynamicCalibration::ErrorCode::CALIBRATION_FAILED;
    }

    return DynamicCalibration::ErrorCode::OK;
}
#endif

DynamicCalibration::ErrorCode DynamicCalibration::computeCoverage() {
    auto resultCoverage =
        pimplDCL->dynCalibImpl.computeCoverage(pimplDCL->sensorLeft, pimplDCL->sensorRight, DclUtils::daiPerformanceModeToDclPerformanceMode(performanceMode));

    if(!resultCoverage.passed()) {
        throw std::runtime_error("Coverage check failed!");
    }

    auto& coverage = resultCoverage.value;
    auto coverageResult = std::make_shared<CoverageData>();

    coverageResult->coveragePerCellA = coverage.coveragePerCellA;
    coverageResult->coveragePerCellB = coverage.coveragePerCellB;
    coverageResult->meanCoverage = coverage.meanCoverage;
    coverageResult->coverageAcquired = coverage.coverageAcquired;
    coverageResult->dataAcquired = coverage.dataAcquired;

    logger->trace("Computing coverage");

    coverageOutput.send(coverageResult);

    return DynamicCalibration::ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::initializePipeline(const std::shared_ptr<dai::Device> daiDevice) {
    logger->trace("Initializing DynamicCalibration pipeline for device: {}", daiDevice->getDeviceId());

    auto inSyncGroup = syncInput.get<dai::MessageGroup>();
    if(!inSyncGroup) {
        return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
    }

    names.clear();
    sockets.clear();
    socketConnection.clear();
    socketToIdx.clear();
    resolutions.clear();
    socketToFrameIntrinsics.clear();
    socketConnectionResolutions.clear();
    socketToSensorExtrinsics.clear();

    names = inSyncGroup->getMessageNames();
    sockets.reserve(names.size());
    resolutions.reserve(names.size());

    if(names.empty()) {
        logger->error("DynamicCalibration Sync node has no connected image inputs.");
        return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
    }

    calibrationHandler = daiDevice->getCalibration();
    auto eepromData = calibrationHandler.getEepromData();

    try {
        socketConnection = buildSocketConnection(eepromData);
    } catch(const std::exception& ex) {
        logger->error("Failed to build calibration socketConnection: {}", ex.what());
        return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
    }

    for(const auto& name : names) {
        const auto frame = inSyncGroup->get<dai::ImgFrame>(name);
        if(!frame) {
            logger->error("Missing synced image '{}' during DynamicCalibration initialization.", name);
            return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
        }

        auto resolution = std::make_pair(static_cast<int>(frame->getWidth()), static_cast<int>(frame->getHeight()));
        resolutions.push_back(resolution);

        auto socket = static_cast<CameraBoardSocket>(frame->getInstanceNum());
        sockets.push_back(socket);

        if(eepromData.cameraData.count(socket) == 0) {
            logger->error("Missing calibration data for socket: {}", toString(socket));
            return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
        }

        const auto frameIntrinsics = intrinsicArrayToMatrix(frame->getTransformation().getIntrinsicMatrix());
        const auto calibrationIntrinsics = calibrationHandler.getCameraIntrinsics(socket, resolution.first, resolution.second, Point2f(), Point2f(), false);
        const auto intrinsicsDelta = subtractMatrices(frameIntrinsics, calibrationIntrinsics);
        socketToFrameIntrinsics[socket] = frameIntrinsics;
        logger->warn("DynamicCalibration K compare \n stream='{}'\n socket={} \n size={}x{} frameK={} \n calibrationHandlerK={} \ndelta={}",
                     name,
                     toString(socket),
                     resolution.first,
                     resolution.second,
                     formatMatrix3x3(frameIntrinsics),
                     formatMatrix3x3(calibrationIntrinsics),
                     formatMatrix3x3(intrinsicsDelta));

        const auto socketIt = std::find(socketConnection.begin(), socketConnection.end(), socket);
        if(socketIt == socketConnection.end()) {
            logger->error("Connected socket {} is not present in calibration socketConnection.", toString(socket));
            return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
        }
        socketToIdx[socket] = static_cast<size_t>(std::distance(socketConnection.begin(), socketIt));
    }

    socketConnectionResolutions.resize(socketConnection.size());
    for(size_t idx = 0; idx < socketConnection.size(); ++idx) {
        const auto& cameraInfo = eepromData.cameraData.at(socketConnection[idx]);
        socketConnectionResolutions[idx] = {cameraInfo.width, cameraInfo.height};
    }
    for(size_t idx = 0; idx < sockets.size(); ++idx) {
        socketConnectionResolutions[socketToIdx.at(sockets[idx])] = resolutions[idx];
    }

    bool leftFound = false;
    bool rightFound = false;
    for(size_t idx = 0; idx < names.size(); ++idx) {
        if(names[idx] == leftInputName) {
            daiSocketLeft = sockets[idx];
            resolutionLeft = resolutions[idx];
            leftFound = true;
        } else if(names[idx] == rightInputName) {
            daiSocketRight = sockets[idx];
            resolutionRight = resolutions[idx];
            rightFound = true;
        }
    }

    if(!leftFound || !rightFound) {
        logger->error("DynamicCalibration requires '{}' and '{}' to be connected to Sync. Found {} stream(s).", leftInputName, rightInputName, names.size());
        return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
    }

    if(eepromData.housingExtrinsics.toCameraSocket != CameraBoardSocket::AUTO) {
        daiSocketBase = HousingCoordinateSystem::AUTO;
    } else {
        daiSocketBase = socketConnection.back();
    }

    if(daiSocketLeft == daiSocketRight) {
        logger->error("Both input images are from the same socket: {}", static_cast<int>(daiSocketLeft));
        return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
    }

    logger->trace("Converting dai calibration to dcl");

    auto platform = daiDevice->getPlatform();
    if(platform == dai::Platform::RVC2 && !eepromData.stereoEnableDistortionCorrection && !oldCalibrationWarningIssued) {
        logger->trace("The calibration on the device is old (distortion correction is disabled), for optimal performance full re-calibration is recommended!");
        oldCalibrationWarningIssued = true;
    }
    // set up the dynamic calibration
    pimplDCL->device = pimplDCL->dynCalibImpl.addDevice();
    pimplDCL->sensors.clear();
    pimplDCL->sensors.reserve(socketConnection.size());
    socketToSensorExtrinsics.clear();
    socketToSensorExtrinsics.reserve(socketConnection.size());

    for(size_t idx = 0; idx < socketConnection.size(); ++idx) {
        std::vector<std::vector<float>> baseToSocketTransform;
        const auto intrinsicsIt = socketToFrameIntrinsics.find(socketConnection[idx]);
        const auto* intrinsicsOverride = intrinsicsIt != socketToFrameIntrinsics.end() ? &intrinsicsIt->second : nullptr;
        auto calibration = DclUtils::convertDaiCalibrationToDcl(
            calibrationHandler, daiSocketBase, socketConnection[idx], socketConnectionResolutions[idx], intrinsicsOverride, &baseToSocketTransform);
        socketToSensorExtrinsics.push_back(baseToSocketTransform);

        dcl::resolution_t resolutionDcl{static_cast<unsigned>(socketConnectionResolutions[idx].first),
                                        static_cast<unsigned>(socketConnectionResolutions[idx].second)};
        pimplDCL->sensors.push_back(pimplDCL->dynCalibImpl.addSensor(pimplDCL->device, calibration, resolutionDcl));
    }

    logger->trace("Added {} sensors to dynCalibImpl", pimplDCL->sensors.size());

    pimplDCL->sensorLeft = pimplDCL->sensors.at(socketToIdx.at(daiSocketLeft));
    pimplDCL->sensorRight = pimplDCL->sensors.at(socketToIdx.at(daiSocketRight));

    return DynamicCalibration::ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::evaluateCommand(const std::shared_ptr<DynamicCalibrationControl>& control) {
    using DC = DynamicCalibrationControl;

    const auto& cmd = control->command;

    // Early exit if command is not set
    if(std::holds_alternative<std::monostate>(cmd)) {
        logger->trace("Received unset command");
        return ErrorCode::INVALID_COMMAND;
    }
    // Calibrate
    if(std::holds_alternative<DC::Commands::Calibrate>(cmd)) {
        const auto& c = std::get<DC::Commands::Calibrate>(cmd);
        logger->trace("Received Calibrate Command: force={}", c.force);
        calibrationShouldRun = false;  // stop the calibration if it is running
        return runCalibration(calibrationHandler, c.force);
    }
    // Quality check
    else if(std::holds_alternative<DC::Commands::CalibrationQuality>(cmd)) {
        const auto& c = std::get<DC::Commands::CalibrationQuality>(cmd);
        logger->trace("Received CalibrationQuality Command: force={}", c.force);
        return runQualityCheck(c.force);
    }
    // Start calibration loop
    else if(std::holds_alternative<DC::Commands::StartCalibration>(cmd)) {
        const auto& c = std::get<DC::Commands::StartCalibration>(cmd);
        logger->trace("Received StartCalibration Command");
        calibrationShouldRun = true;
        loadImagePeriod = c.loadImagePeriod;
        calibrationPeriod = c.calibrationPeriod;
        return ErrorCode::OK;
    }
    // Load a single image
    else if(std::holds_alternative<DC::Commands::LoadImage>(cmd)) {
        logger->trace("Received LoadImage Command: blocking load with coverage computation");
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
        auto error = runLoadImage(true);
        computeCoverage();
        return error;
#else
        throw std::runtime_error("DynamicCalibrationNode was built without OpenCV support, cannot load images.");
#endif
    }
    // Apply calibration
    else if(std::holds_alternative<DC::Commands::ApplyCalibration>(cmd)) {
        const auto& c = std::get<DC::Commands::ApplyCalibration>(cmd);
        logger->trace("Received ApplyCalibrationCommand: applying new calibration to device");
        calibrationHandler = c.calibration;
        setCalibration(calibrationHandler, c.flash);
        return ErrorCode::OK;
    }
    // Stop calibration loop
    else if(std::holds_alternative<DC::Commands::StopCalibration>(cmd)) {
        logger->trace("Received StopCalibrationCommand: stopping calibration");
        calibrationShouldRun = false;
        return ErrorCode::OK;
    }
    // Reset/remove accumulated data
    else if(std::holds_alternative<DC::Commands::ResetData>(cmd)) {
        logger->trace("Received RemoveDataCommand: removing the data");
        pimplDCL->dynCalibImpl.removeAllData(pimplDCL->sensorLeft, pimplDCL->sensorRight);
        return ErrorCode::OK;
    }
    // Set performance mode
    else if(std::holds_alternative<DC::Commands::SetPerformanceMode>(cmd)) {
        const auto& c = std::get<DC::Commands::SetPerformanceMode>(cmd);
        logger->trace("Received SetPerformanceModeCommand: changing performance mode to {}", static_cast<int>(c.performanceMode));
        performanceMode = c.performanceMode;
        return ErrorCode::OK;
    } else if(std::holds_alternative<DC::Commands::ComputeCalibrationMetrics>(cmd)) {
        logger->trace("Received ComputeCalibrationMetrics command");
        const auto& c = std::get<DC::Commands::ComputeCalibrationMetrics>(cmd);
        computeMetrics(c.calibration);
        return ErrorCode::OK;
    }

    // Fallback
    logger->trace("Received unknown/unhandled command type");
    return ErrorCode::INVALID_COMMAND;
}

DynamicCalibration::ErrorCode DynamicCalibration::doWork(std::chrono::steady_clock::time_point& previousLoadingAndCalibrationTime) {
    auto error = ErrorCode::OK;  // Expect everything is ok
    std::shared_ptr<DynamicCalibrationControl> calibrationCommand = nullptr;
    {
        auto blockEvent = this->inputBlockEvent();
        calibrationCommand = inputControl.tryGet<DynamicCalibrationControl>();
    }
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
        logger->trace("doWork() called. CalibrationRunning={}, elapsed={}s", calibrationShouldRun, elapsed.count());
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
        error = runLoadImage(true);
#else
        throw std::runtime_error("DynamicCalibrationNode was built without OpenCV support, cannot load images.");
#endif
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

    logger->trace("DynamicCalibration node started ");

    auto previousLoadingTimeFloat = std::chrono::steady_clock::now() + std::chrono::duration<float>(calibrationPeriod);
    auto previousLoadingTime = std::chrono::time_point_cast<std::chrono::steady_clock::duration>(previousLoadingTimeFloat);
    auto initResult = initializePipeline(device);
    if(initResult != DynamicCalibration::ErrorCode::OK) {
        logger->error("DynamicCalibration initialization failed with error code {}", static_cast<int>(initResult));
        return;
    }
    while(mainLoop()) {
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
