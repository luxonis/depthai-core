#include "depthai/pipeline/node/DynamicCalibrationNode.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/opencv.hpp>
#endif
#include <pipeline/ThreadedNodeImpl.hpp>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationControl.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationResults.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/utility/matrixOps.hpp"
#include "depthai/utility/spimpl.h"
#include "pipeline/node/DynamicCalibrationUtils.hpp"
#include <algorithm>
#include <set>

namespace dai {
namespace node {

#define DCL_PERSPECTIVE_DISTORTION_SIZE (14)
#define DCL_FISHEYE_DISTORTION_SIZE (4)

namespace {

std::vector<std::vector<float>> extractRotation3x3(const std::vector<std::vector<float>>& transform) {
    std::vector<std::vector<float>> rotation(3, std::vector<float>(3, 0.0f));
    for(int row = 0; row < 3; ++row) {
        for(int col = 0; col < 3; ++col) {
            rotation[row][col] = transform[row][col];
        }
    }
    return rotation;
}

std::vector<float> extractTranslation3(const std::vector<std::vector<float>>& transform) {
    return {transform[0][3], transform[1][3], transform[2][3]};
}

std::vector<CameraBoardSocket> buildCalibrationSocketChain(const CalibrationHandler& calibrationHandler,
                                                           const std::vector<CameraBoardSocket>& syncedSockets) {
    const auto& cameraData = calibrationHandler.getEepromData().cameraData;
    std::set<CameraBoardSocket> syncedSocketSet(syncedSockets.begin(), syncedSockets.end());
    std::set<CameraBoardSocket> destinationSockets;

    for(const auto socket : syncedSockets) {
        auto it = cameraData.find(socket);
        if(it == cameraData.end()) {
            continue;
        }
        const auto dst = it->second.extrinsics.toCameraSocket;
        if(syncedSocketSet.count(dst) > 0) {
            destinationSockets.insert(dst);
        }
    }

    std::vector<CameraBoardSocket> chain;
    std::set<CameraBoardSocket> visited;
    auto appendFrom = [&](CameraBoardSocket socket) {
        while(syncedSocketSet.count(socket) > 0 && visited.count(socket) == 0) {
            chain.push_back(socket);
            visited.insert(socket);
            auto it = cameraData.find(socket);
            if(it == cameraData.end()) {
                break;
            }
            socket = it->second.extrinsics.toCameraSocket;
        }
    };

    for(const auto socket : syncedSockets) {
        if(destinationSockets.count(socket) == 0) {
            appendFrom(socket);
        }
    }

    for(const auto socket : syncedSockets) {
        appendFrom(socket);
    }

    return chain;
}

}  // namespace

class DynamicCalibration::Impl {
   public:
    /**
     * DCL held properties
     */
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

std::pair<std::shared_ptr<dcl::CameraCalibrationHandle>, std::shared_ptr<dcl::CameraCalibrationHandle>> DclUtils::convertDaiCalibrationToDcl(
    const CalibrationHandler& currentCalibration,
    const CameraBoardSocket boardSocketA,
    const CameraBoardSocket boardSocketB,
    const std::pair<int, int>& resolutionA,
    const std::pair<int, int>& resolutionB) {
    auto housingToCameraA = currentCalibration.getHousingCalibration(boardSocketA, HousingCoordinateSystem::AUTO, false, LengthUnit::METER);
    matrix::invertSe3Matrix4x4InPlace(housingToCameraA);

    auto housingToCameraB = currentCalibration.getHousingCalibration(boardSocketB, HousingCoordinateSystem::AUTO, false, LengthUnit::METER);
    matrix::invertSe3Matrix4x4InPlace(housingToCameraB);

    return std::make_pair(
        DclUtils::createDclCalibration(currentCalibration.getCameraIntrinsics(boardSocketA, resolutionA.first, resolutionA.second, Point2f(), Point2f(), false),
                                       currentCalibration.getDistortionCoefficients(boardSocketA),
                                       extractRotation3x3(housingToCameraA),
                                       extractTranslation3(housingToCameraA),
                                       currentCalibration.getDistortionModel(boardSocketA)),
        DclUtils::createDclCalibration(currentCalibration.getCameraIntrinsics(boardSocketB, resolutionB.first, resolutionB.second, Point2f(), Point2f(), false),
                                       currentCalibration.getDistortionCoefficients(boardSocketB),
                                       extractRotation3x3(housingToCameraB),
                                       extractTranslation3(housingToCameraB),
                                       currentCalibration.getDistortionModel(boardSocketB)));
}

std::shared_ptr<dcl::CameraCalibrationHandle> DclUtils::convertDaiCalibrationToDcl(const CalibrationHandler& currentCalibration,
                                                                                    const CameraBoardSocket boardSocket,
                                                                                    const ImgTransformation& transformation) {
    auto intrinsicsArray = transformation.getIntrinsicMatrix();
    std::vector<std::vector<float>> intrinsics = {
        {intrinsicsArray[0][0], intrinsicsArray[0][1], intrinsicsArray[0][2]},
        {intrinsicsArray[1][0], intrinsicsArray[1][1], intrinsicsArray[1][2]},
        {intrinsicsArray[2][0], intrinsicsArray[2][1], intrinsicsArray[2][2]},
    };

    // Force the RGB principal point to the optical center requested by calibration flow.
    if(boardSocket == CameraBoardSocket::CAM_A) {
        intrinsics[0][2] = 2028.0f;
        intrinsics[1][2] = 1520.0f;
    }

    auto distortion = transformation.getDistortionCoefficients();
    if(distortion.empty() && transformation.getDistortionModel() == dai::CameraModel::Perspective) {
        distortion.assign(DCL_PERSPECTIVE_DISTORTION_SIZE, 0.0f);
    } else if(distortion.empty() && transformation.getDistortionModel() == dai::CameraModel::Fisheye) {
        distortion.assign(DCL_FISHEYE_DISTORTION_SIZE, 0.0f);
    }

    auto housingToCamera = currentCalibration.getHousingCalibration(boardSocket, HousingCoordinateSystem::AUTO, false, LengthUnit::METER);
    matrix::invertSe3Matrix4x4InPlace(housingToCamera);

    return DclUtils::createDclCalibration(intrinsics,
                                          distortion,
                                          extractRotation3x3(housingToCamera),
                                          extractTranslation3(housingToCamera),
                                          transformation.getDistortionModel());
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

void enforceRgbPrincipalPoint(CalibrationHandler& calibHandler,
                              const CameraBoardSocket socket,
                              const std::pair<int, int>& resolution) {
    if(socket != CameraBoardSocket::CAM_A) {
        return;
    }

    auto intrinsics = calibHandler.getCameraIntrinsics(socket, resolution.first, resolution.second, Point2f(), Point2f(), false);
    intrinsics[0][2] = 2028.0f;
    intrinsics[1][2] = 1520.0f;
    calibHandler.setCameraIntrinsics(socket, intrinsics, resolution.first, resolution.second);
}

void DclUtils::convertDclCalibrationToDai(CalibrationHandler& calibHandler,
                                          const std::shared_ptr<const dcl::CameraCalibrationHandle>& dclCalibrationA,
                                          const std::shared_ptr<const dcl::CameraCalibrationHandle>& dclCalibrationB,
                                          const CameraBoardSocket socketSrc,
                                          const CameraBoardSocket socketDest,
                                          const std::pair<int, int>& resolutionA,
                                          const std::pair<int, int>& resolutionB) {
    // Get extrinsics for both cameras (T_left_to_origin and T_right_to_origin)
    dcl::scalar_t tvecA[3];
    dclCalibrationA->getTvec(tvecA);
    dcl::scalar_t rvecA[3];
    dclCalibrationA->getRvec(rvecA);

    dcl::scalar_t tvecB[3];
    dclCalibrationB->getTvec(tvecB);
    dcl::scalar_t rvecB[3];
    dclCalibrationB->getRvec(rvecB);

    // Convert rvecs to rotation matrices
    // DCL uses "origin to camera" convention: p_cam = R_cam * p_origin + t_cam
    auto R_A = matrix::rvecToRotationMatrix(rvecA);
    auto R_B = matrix::rvecToRotationMatrix(rvecB);

    // Compute T_A_to_B (left to right) from per-camera poses:
    //   p_A = R_A * p_ref + t_A  =>  p_ref = R_A^T * (p_A - t_A)
    //   p_B = R_B * p_ref + t_B  =>  p_B = R_B * R_A^T * p_A + t_B - R_B * R_A^T * t_A
    // R_rel = R_B * R_A^T
    // t_rel = t_B - R_rel * t_A
    std::vector<std::vector<float>> R_A_inv(3, std::vector<float>(3, 0.0f));
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            R_A_inv[i][j] = R_A[j][i];  // transpose of rotation = inverse
        }
    }

    auto rotationMatrix = matrix::matMul(R_B, R_A_inv);

    // t_rel = t_B - R_rel * t_A, converted from meters to cm
    std::vector<float> translation(3, 0.0f);
    for(int i = 0; i < 3; ++i) {
        float R_rel_dot_tA = 0.0f;
        for(int j = 0; j < 3; ++j) {
            R_rel_dot_tA += rotationMatrix[i][j] * static_cast<float>(tvecA[j]);
        }
        translation[i] = (static_cast<float>(tvecB[i]) - R_rel_dot_tA) * 100.0f;  // meters to cm
    }

    dcl::distortion_t distortionA;
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

    dcl::distortion_t distortionB;
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

    calibHandler.setCameraIntrinsics(socketSrc, matA, resolutionA.first, resolutionA.second);
    calibHandler.setCameraIntrinsics(socketDest, matB, resolutionB.first, resolutionB.second);
    calibHandler.setDistortionCoefficients(socketSrc, distortionToVector(distortionA));
    calibHandler.setDistortionCoefficients(socketDest, distortionToVector(distortionB));
    auto specTranslation = calibHandler.getCameraTranslationVector(socketSrc, socketDest, true);
    calibHandler.setCameraExtrinsics(socketSrc, socketDest, rotationMatrix, translation, specTranslation);
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
    for(std::size_t index = 0; index < syncedSockets.size(); ++index) {
        enforceRgbPrincipalPoint(handler, syncedSockets.at(index), syncedResolutions.at(index));
    }
    logger->trace("Applying calibration to device");
    device->setCalibration(handler);
    if(flash) {
        device->flashCalibration(handler);
    }
    for(std::size_t index = 0; index < pimplDCL->sensors.size(); ++index) {
        auto calibration = DclUtils::convertDaiCalibrationToDcl(handler, syncedSockets.at(index), syncedTransformations.at(index));
        pimplDCL->dynCalibImpl.setCalibration(pimplDCL->sensors.at(index), calibration);
    }
}

void DynamicCalibration::computeMetrics(const CalibrationHandler& handler) {
    auto calibA = DclUtils::convertDaiCalibrationToDcl(handler, daiSocketA, syncedTransformations.at(0));
    auto calibB = DclUtils::convertDaiCalibrationToDcl(handler, daiSocketB, syncedTransformations.at(1));
    auto dataConfidence = pimplDCL->dynCalibImpl.computeDataConfidence(pimplDCL->sensors.at(0), pimplDCL->sensors.at(1));
    auto calibrationConfidence = pimplDCL->dynCalibImpl.computeCalibrationConfidence(calibA, calibB, pimplDCL->sensors.at(0), pimplDCL->sensors.at(1));
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

    auto dclResult = pimplDCL->dynCalibImpl.checkCalibration(pimplDCL->sensors.at(0), pimplDCL->sensors.at(1), pm);

    if(!dclResult.passed()) {
        auto result = std::make_shared<CalibrationQuality>();
        result->info = dclResult.errorMessage();
        logger->trace("WARNING: Quality check failed: {}", dclResult.errorMessage());

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
    std::vector<std::shared_ptr<const dcl::CameraSensorHandle>> sensorsForCalibration;
    sensorsForCalibration.reserve(pimplDCL->sensors.size());
    for(const auto& sensor : pimplDCL->sensors) {
        sensorsForCalibration.push_back(sensor);
    }
    auto dclResult = pimplDCL->dynCalibImpl.findNewCalibration(sensorsForCalibration, pm);
    if(!dclResult.passed()) {
        auto result = std::make_shared<DynamicCalibrationResult>(dclResult.errorMessage());
        logger->trace("WARNING: Calibration failed: {}", dclResult.errorMessage());

        calibrationOutput.send(result);
        return DynamicCalibration::ErrorCode::CALIBRATION_FAILED;
    }

    auto newCalibrationHandler = currentHandler;

    for(std::size_t chainIndex = 1; chainIndex < calibrationSocketChain.size(); ++chainIndex) {
        const auto previousSocket = calibrationSocketChain.at(chainIndex - 1);
        const auto currentSocket = calibrationSocketChain.at(chainIndex);
        const auto previousCalibration = dclResult.value.at(socketToSensorIndex.at(previousSocket));
        const auto currentCalibration = dclResult.value.at(socketToSensorIndex.at(currentSocket));
        const auto& previousResolution = syncedResolutions.at(socketToSensorIndex.at(previousSocket));
        const auto& currentResolution = syncedResolutions.at(socketToSensorIndex.at(currentSocket));

        dai::node::DclUtils::convertDclCalibrationToDai(
            newCalibrationHandler, previousCalibration, currentCalibration, previousSocket, currentSocket, previousResolution, currentResolution);
    }
    for(std::size_t index = 0; index < syncedSockets.size(); ++index) {
        enforceRgbPrincipalPoint(newCalibrationHandler, syncedSockets.at(index), syncedResolutions.at(index));
    }

    CalibrationQuality::Data qualityData{};
    auto qualityDifference = pimplDCL->dynCalibImpl.checkCalibration(pimplDCL->sensors.at(0), pimplDCL->sensors.at(1), pm);
    if(qualityDifference.passed()) {
        qualityData.rotationChange[0] = qualityDifference.value.rotationChange[0];
        qualityData.rotationChange[1] = qualityDifference.value.rotationChange[1];
        qualityData.rotationChange[2] = qualityDifference.value.rotationChange[2];
        qualityData.depthErrorDifference = qualityDifference.value.depthDistanceDifference;
        qualityData.sampsonErrorCurrent = qualityDifference.value.sampsonErrorCurrent;
        qualityData.sampsonErrorNew = qualityDifference.value.sampsonErrorNew;
    }

    DynamicCalibrationResult::Data resultData{};
    resultData.newCalibration = newCalibrationHandler;
    resultData.currentCalibration = currentHandler;
    resultData.calibrationDifference = qualityData;
    auto dataConfidence = pimplDCL->dynCalibImpl.computeDataConfidence(pimplDCL->sensors.at(0), pimplDCL->sensors.at(1));
    resultData.dataConfidence = dataConfidence.passed() ? dataConfidence.value : 0.0;

    auto result = std::make_shared<DynamicCalibrationResult>(resultData, dclResult.errorMessage());
    logger->trace(
        "Calibration successful. Rotation Δ=({}, {}, {})", qualityData.rotationChange[0], qualityData.rotationChange[1], qualityData.rotationChange[2]);
    calibrationOutput.send(result);

    return DynamicCalibration::ErrorCode::OK;
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
DynamicCalibration::ErrorCode DynamicCalibration::runLoadImage(const bool blocking) {
    std::shared_ptr<dai::MessageGroup> inSyncGroup;
    logger->trace("Attempting to load synchronized image set (blocking={})", blocking);
    if(!blocking) {
        inSyncGroup = syncInput.tryGet<dai::MessageGroup>();
    } else {
        slept = true;
        inSyncGroup = syncInput.get<dai::MessageGroup>();
    }
    if(!inSyncGroup) {
        return DynamicCalibration::ErrorCode::EMPTY_IMAGE_QUEUE;
    }
    dcl::DeviceImageList deviceImages;
    deviceImages.reserve(syncedInputNames.size());

    std::shared_ptr<dai::ImgFrame> referenceFrame;
    for(std::size_t index = 0; index < syncedInputNames.size(); ++index) {
        auto frame = inSyncGroup->get<dai::ImgFrame>(syncedInputNames.at(index));
        if(!frame) {
            logger->trace("WARNING: Missing image '{}' in MessageGroup", syncedInputNames.at(index));
            return DynamicCalibration::ErrorCode::MISSING_IMAGE;
        }
        if(!referenceFrame) {
            referenceFrame = frame;
        }
        syncedTransformations.at(index) = frame->getTransformation();
        deviceImages.emplace_back(pimplDCL->sensors.at(index), DclUtils::cvMatToImageData(frame->getCvFrame()));
    }

    dcl::timestamp_t timestamp = referenceFrame->getTimestamp().time_since_epoch().count();
    logger->trace("Loaded synchronized image set with {} frames @ timestamp={}", deviceImages.size(), timestamp);

    auto loadResult = pimplDCL->dynCalibImpl.loadImages(deviceImages, timestamp);
    if(!loadResult.passed()) {
        logger->trace("WARNING: Failed to load synchronized image set: {}", loadResult.errorMessage());
        return DynamicCalibration::ErrorCode::MISSING_IMAGE;
    }

    return DynamicCalibration::ErrorCode::OK;
}
#endif

DynamicCalibration::ErrorCode DynamicCalibration::computeCoverage() {
    auto resultCoverage =
        pimplDCL->dynCalibImpl.computeCoverage(pimplDCL->sensors.at(0), pimplDCL->sensors.at(1), DclUtils::daiPerformanceModeToDclPerformanceMode(performanceMode));

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
    syncedInputNames = inSyncGroup->getMessageNames();
    if(syncedInputNames.size() < 2) {
        return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
    }
    std::sort(syncedInputNames.begin(), syncedInputNames.end());
    if(auto leftIt = std::find(syncedInputNames.begin(), syncedInputNames.end(), leftInputName); leftIt != syncedInputNames.end()) {
        std::iter_swap(syncedInputNames.begin(), leftIt);
    }
    if(syncedInputNames.size() > 1) {
        if(auto rightIt = std::find(syncedInputNames.begin() + 1, syncedInputNames.end(), rightInputName); rightIt != syncedInputNames.end()) {
            std::iter_swap(syncedInputNames.begin() + 1, rightIt);
        }
    }

    syncedSockets.clear();
    syncedResolutions.clear();
    syncedTransformations.clear();
    inputNameToSensorIndex.clear();
    socketToSensorIndex.clear();

    std::set<CameraBoardSocket> uniqueSockets;
    for(const auto& inputName : syncedInputNames) {
        auto frame = inSyncGroup->get<dai::ImgFrame>(inputName);
        if(!frame) {
            return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
        }
        const auto socket = static_cast<CameraBoardSocket>(frame->instanceNum);
        if(!uniqueSockets.insert(socket).second) {
            logger->error("Multiple synchronized inputs use the same socket: {}", static_cast<int>(socket));
            return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
        }
        inputNameToSensorIndex.emplace(inputName, syncedSockets.size());
        socketToSensorIndex.emplace(socket, syncedSockets.size());
        syncedSockets.push_back(socket);
        syncedResolutions.emplace_back(frame->getWidth(), frame->getHeight());
        syncedTransformations.push_back(frame->getTransformation());
    }

    daiSocketA = syncedSockets.at(0);
    daiSocketB = syncedSockets.at(1);
    resolutionA = syncedResolutions.at(0);
    resolutionB = syncedResolutions.at(1);

    logger->trace("Converting dai calibration to dcl");

    calibrationHandler = daiDevice->getCalibration();
    for(std::size_t index = 0; index < syncedSockets.size(); ++index) {
        enforceRgbPrincipalPoint(calibrationHandler, syncedSockets.at(index), syncedResolutions.at(index));
    }
    auto eepromData = calibrationHandler.getEepromData();
    auto platform = daiDevice->getPlatform();
    if(platform == dai::Platform::RVC2 && !eepromData.stereoEnableDistortionCorrection && !oldCalibrationWarningIssued) {
        logger->trace("The calibration on the device is old (distortion correction is disabled), for optimal performance full re-calibration is recommended!");
        oldCalibrationWarningIssued = true;
    }
    // set up the dynamic calibration
    pimplDCL->device = pimplDCL->dynCalibImpl.addDevice();
    pimplDCL->sensors.clear();
    pimplDCL->sensors.reserve(syncedSockets.size());
    calibrationSocketChain = buildCalibrationSocketChain(calibrationHandler, syncedSockets);

    for(std::size_t index = 0; index < syncedSockets.size(); ++index) {
        auto calibration = DclUtils::convertDaiCalibrationToDcl(calibrationHandler, syncedSockets.at(index), syncedTransformations.at(index));
        dcl::resolution_t resolutionDcl{static_cast<unsigned>(syncedResolutions.at(index).first), static_cast<unsigned>(syncedResolutions.at(index).second)};
        pimplDCL->sensors.push_back(
            pimplDCL->dynCalibImpl.addSensor(pimplDCL->device, calibration, resolutionDcl, static_cast<dcl::socket_t>(syncedSockets.at(index))));
    }

    logger->trace("Added {} sensors to dynCalibImpl", pimplDCL->sensors.size());

    return DynamicCalibration::ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::evaluateCommand(const std::shared_ptr<DynamicCalibrationControl>& control) {
    using DC = DynamicCalibrationControl;

    const auto& cmd = control->command;

    // Early exit if command is not set
    if(std::holds_alternative<std::monostate>(cmd)) {
        logger->warn("Recived UNSET Command");
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
        pimplDCL->dynCalibImpl.removeDeviceMeasurements(pimplDCL->device);
        return ErrorCode::OK;
    }
    // Set performance mode
    else if(std::holds_alternative<DC::Commands::SetPerformanceMode>(cmd)) {
        const auto& c = std::get<DC::Commands::SetPerformanceMode>(cmd);
        logger->trace("Received SetPerformanceModeCommand: changing performance mode to {}", static_cast<int>(c.performanceMode));
        performanceMode = c.performanceMode;
        return ErrorCode::OK;
    } else if(std::holds_alternative<DC::Commands::ComputeCalibrationMetrics>(cmd)) {
        logger->trace("Received ComputerCalibrationMetrics: calculation metricis");
        const auto& c = std::get<DC::Commands::ComputeCalibrationMetrics>(cmd);
        computeMetrics(c.calibration);
        return ErrorCode::OK;
    }

    // Fallback
    logger->trace("WARNING: evaluateCommand: Received unknown/unhandled command type");
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
    initializePipeline(device);
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
