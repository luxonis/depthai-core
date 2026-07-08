#include "depthai/pipeline/node/DynamicCalibrationNode.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/opencv.hpp>
#endif
#include <algorithm>
#include <array>
#include <map>
#include <mutex>
#include <optional>
#include <pipeline/ThreadedNodeImpl.hpp>
#include <string>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationControl.hpp"
#include "depthai/pipeline/datatype/DynamicCalibrationResults.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/utility/CompilerWarnings.hpp"
#include "depthai/utility/matrixOps.hpp"
#include "depthai/utility/spimpl.h"
#include "pipeline/node/DynamicCalibrationUtils.hpp"
#include "utility/Telemetry.hpp"

namespace dai {
namespace node {

namespace {

struct DynamicCalibrationTelemetryAggregateState {
    std::mutex mutex;
    std::string telemetryDeviceId;
    std::uint64_t totalCommands = 0;
    std::uint64_t calibrateCommands = 0;
    std::uint64_t calibrationQualityCommands = 0;
    std::uint64_t startCalibrationCommands = 0;
    std::uint64_t loadImageCommands = 0;
    std::uint64_t applyCalibrationCommands = 0;
    std::uint64_t stopCalibrationCommands = 0;
    std::uint64_t resetDataCommands = 0;
    std::uint64_t setPerformanceModeCommands = 0;
    std::uint64_t computeCalibrationMetricsCommands = 0;
    std::uint64_t calibrationResultCallbacks = 0;
    std::uint64_t metricsResultCallbacks = 0;
    std::uint64_t qualityResultCallbacks = 0;
    std::uint64_t applyResultCallbacks = 0;
    std::uint64_t successfulCalibrations = 0;
    std::uint64_t failedCalibrations = 0;
    double sampsonErrorCurrentSum = 0.0;
    double sampsonErrorNewSum = 0.0;
    std::uint64_t sampsonErrorSamples = 0;
    std::map<std::pair<CameraBoardSocket, CameraBoardSocket>, std::array<double, 3>> pairwiseRotationDifferenceSums;
    std::map<std::pair<CameraBoardSocket, CameraBoardSocket>, std::uint64_t> pairwiseRotationDifferenceSamples;
};

void addPairwiseRotationDifferenceAverages(
    nlohmann::json& properties,
    const std::map<std::pair<CameraBoardSocket, CameraBoardSocket>, std::array<double, 3>>& pairwiseRotationDifferenceSums,
    const std::map<std::pair<CameraBoardSocket, CameraBoardSocket>, std::uint64_t>& pairwiseRotationDifferenceSamples) {
    auto pairwiseProperties = nlohmann::json::array();
    for(const auto& [socketPair, rotationSum] : pairwiseRotationDifferenceSums) {
        const auto sampleCountIt = pairwiseRotationDifferenceSamples.find(socketPair);
        const auto sampleCount = sampleCountIt == pairwiseRotationDifferenceSamples.end() ? std::uint64_t{0} : sampleCountIt->second;
        if(sampleCount == 0) {
            continue;
        }

        const auto socketA = dai::toString(socketPair.first);
        const auto socketB = dai::toString(socketPair.second);
        pairwiseProperties.push_back({{"socket_a", socketA},
                                      {"socket_b", socketB},
                                      {"rotation_difference_avg_deg",
                                       {rotationSum[0] / static_cast<double>(sampleCount),
                                        rotationSum[1] / static_cast<double>(sampleCount),
                                        rotationSum[2] / static_cast<double>(sampleCount)}},
                                      {"samples", sampleCount}});
    }

    properties["dynamic_calibration_pairwise_rotation_difference"] = std::move(pairwiseProperties);
}

void addDynamicCalibrationTelemetryProperties(nlohmann::json& properties, const DynamicCalibrationTelemetryAggregateState& state) {
    properties["telemetry_device_id"] = state.telemetryDeviceId;
    properties["dynamic_calibration_total_commands"] = properties.value("dynamic_calibration_total_commands", std::uint64_t{0}) + state.totalCommands;

    auto& commandCounts = properties["dynamic_calibration_command_counts"];
    if(!commandCounts.is_object()) {
        commandCounts = nlohmann::json::object();
    }
    commandCounts["calibrate"] = commandCounts.value("calibrate", std::uint64_t{0}) + state.calibrateCommands;
    commandCounts["calibration_quality"] = commandCounts.value("calibration_quality", std::uint64_t{0}) + state.calibrationQualityCommands;
    commandCounts["start_calibration"] = commandCounts.value("start_calibration", std::uint64_t{0}) + state.startCalibrationCommands;
    commandCounts["load_image"] = commandCounts.value("load_image", std::uint64_t{0}) + state.loadImageCommands;
    commandCounts["apply_calibration"] = commandCounts.value("apply_calibration", std::uint64_t{0}) + state.applyCalibrationCommands;
    commandCounts["stop_calibration"] = commandCounts.value("stop_calibration", std::uint64_t{0}) + state.stopCalibrationCommands;
    commandCounts["reset_data"] = commandCounts.value("reset_data", std::uint64_t{0}) + state.resetDataCommands;
    commandCounts["set_performance_mode"] = commandCounts.value("set_performance_mode", std::uint64_t{0}) + state.setPerformanceModeCommands;
    commandCounts["compute_calibration_metrics"] =
        commandCounts.value("compute_calibration_metrics", std::uint64_t{0}) + state.computeCalibrationMetricsCommands;

    auto& callbackCounts = properties["dynamic_calibration_callback_counts"];
    if(!callbackCounts.is_object()) {
        callbackCounts = nlohmann::json::object();
    }
    callbackCounts["calibration_result"] = callbackCounts.value("calibration_result", std::uint64_t{0}) + state.calibrationResultCallbacks;
    callbackCounts["metrics_result"] = callbackCounts.value("metrics_result", std::uint64_t{0}) + state.metricsResultCallbacks;
    callbackCounts["quality_result"] = callbackCounts.value("quality_result", std::uint64_t{0}) + state.qualityResultCallbacks;
    callbackCounts["apply_result"] = callbackCounts.value("apply_result", std::uint64_t{0}) + state.applyResultCallbacks;

    properties["dynamic_calibration_successful_calibrations"] =
        properties.value("dynamic_calibration_successful_calibrations", std::uint64_t{0}) + state.successfulCalibrations;
    properties["dynamic_calibration_failed_calibrations"] =
        properties.value("dynamic_calibration_failed_calibrations", std::uint64_t{0}) + state.failedCalibrations;
    properties["dynamic_calibration_sampson_error_samples"] =
        properties.value("dynamic_calibration_sampson_error_samples", std::uint64_t{0}) + state.sampsonErrorSamples;
    properties["dynamic_calibration_sampson_error_current_sum"] =
        properties.value("dynamic_calibration_sampson_error_current_sum", 0.0) + state.sampsonErrorCurrentSum;
    properties["dynamic_calibration_sampson_error_new_sum"] = properties.value("dynamic_calibration_sampson_error_new_sum", 0.0) + state.sampsonErrorNewSum;

    const auto sampsonErrorSamples = properties.value("dynamic_calibration_sampson_error_samples", std::uint64_t{0});
    const auto sampsonErrorCurrentSum = properties.value("dynamic_calibration_sampson_error_current_sum", 0.0);
    const auto sampsonErrorNewSum = properties.value("dynamic_calibration_sampson_error_new_sum", 0.0);
    properties["dynamic_calibration_avg_sampson_error_current"] =
        sampsonErrorSamples == 0 ? 0.0 : sampsonErrorCurrentSum / static_cast<double>(sampsonErrorSamples);
    properties["dynamic_calibration_avg_sampson_error_new"] = sampsonErrorSamples == 0 ? 0.0 : sampsonErrorNewSum / static_cast<double>(sampsonErrorSamples);

    addPairwiseRotationDifferenceAverages(properties, state.pairwiseRotationDifferenceSums, state.pairwiseRotationDifferenceSamples);
}

void addDynamicCalibrationResultTelemetry(DynamicCalibrationTelemetryAggregateState& state, const CalibrationQuality::Data& qualityData) {
    std::lock_guard<std::mutex> lock(state.mutex);
    state.successfulCalibrations += 1;
    state.sampsonErrorCurrentSum += static_cast<double>(qualityData.sampsonErrorCurrent);
    state.sampsonErrorNewSum += static_cast<double>(qualityData.sampsonErrorNew);
    state.sampsonErrorSamples += 1;
    for(const auto& [socketPair, rotationChange] : qualityData.pairwiseRotationDifference) {
        auto& rotationSum = state.pairwiseRotationDifferenceSums[socketPair];
        const auto copyCount = std::min<std::size_t>(3, rotationChange.size());
        for(std::size_t axis = 0; axis < copyCount; ++axis) {
            rotationSum[axis] += static_cast<double>(rotationChange[axis]);
        }
        state.pairwiseRotationDifferenceSamples[socketPair] += 1;
    }
}

const char* telemetryDynamicCalibrationPerformanceModeName(const DynamicCalibrationControl::PerformanceMode mode) {
    switch(mode) {
        case DynamicCalibrationControl::PerformanceMode::DEFAULT:
            return "DEFAULT";
        case DynamicCalibrationControl::PerformanceMode::STATIC_SCENERY:
            return "STATIC_SCENERY";
        case DynamicCalibrationControl::PerformanceMode::OPTIMIZE_SPEED:
            return "OPTIMIZE_SPEED";
        case DynamicCalibrationControl::PerformanceMode::OPTIMIZE_PERFORMANCE:
            return "OPTIMIZE_PERFORMANCE";
        case DynamicCalibrationControl::PerformanceMode::SKIP_CHECKS:
            return "SKIP_CHECKS";
        case DynamicCalibrationControl::PerformanceMode::RELAXED_COVERAGE:
            return "RELAXED_COVERAGE";
    }
    return "UNKNOWN";
}

}  // namespace

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

class DynamicCalibration::Impl {
   public:
    /**
     * DCL held properties
     */
    std::shared_ptr<dcl::Device> device;
    dcl::DynamicCalibration dynCalibImpl;
    std::shared_ptr<DynamicCalibrationTelemetryAggregateState> telemetryAggregateState = std::make_shared<DynamicCalibrationTelemetryAggregateState>();
    utility::Telemetry::AggregateMetricsHandle telemetryAggregateMetricsHandle = 0;
};

DynamicCalibration::~DynamicCalibration() {
    if(pimplDCL && pimplDCL->telemetryAggregateMetricsHandle != 0) {
        utility::Telemetry::getInstance().removeAggregateMetrics(pimplDCL->telemetryAggregateMetricsHandle);
        pimplDCL->telemetryAggregateMetricsHandle = 0;
    }
}

DynamicCalibration::Properties& DynamicCalibration::getProperties() {
    return properties;
}

const DynamicCalibration::ConnectedSensor* DynamicCalibration::findConnectedSensor(CameraBoardSocket socket) const {
    const auto it = std::find_if(connectedSensors.begin(), connectedSensors.end(), [socket](const ConnectedSensor& sensor) { return sensor.socket == socket; });
    return it != connectedSensors.end() ? &(*it) : nullptr;
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
    auto telemetryState = pimplDCL->telemetryAggregateState;
    pimplDCL->telemetryAggregateMetricsHandle = utility::Telemetry::getInstance().addAggregateMetrics([telemetryState](nlohmann::json& properties) {
        std::lock_guard<std::mutex> stateLock(telemetryState->mutex);
        if(telemetryState->telemetryDeviceId.empty()) {
            return;
        }

        auto& byDevice = properties["dynamic_calibration_by_device"];
        if(!byDevice.is_object()) {
            byDevice = nlohmann::json::object();
        }

        auto& deviceProperties = byDevice[telemetryState->telemetryDeviceId];
        if(!deviceProperties.is_object()) {
            deviceProperties = nlohmann::json::object();
        }

        addDynamicCalibrationTelemetryProperties(deviceProperties, *telemetryState);
    });
    sync->out.link(syncInput);
    sync->setRunOnHost(true);
}

void DynamicCalibration::postBuildStage() {
    auto eraseIfUnused = [&](const std::string& name) {
        auto it = inputs.find({inputs.name, name});
        if(it != inputs.end() && !it->second.isConnected() && inputs.size() > 1) {
            logger->trace("Ignoring unconnected DynamicCalibration input '{}'", name);
            inputs.erase(it);
        }
    };

    eraseIfUnused(leftInputName);
    eraseIfUnused(rightInputName);
    eraseIfUnused(rgbInputName);
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

#define DCL_PERSPECTIVE_DISTORTION_SIZE (14)
#define DCL_FISHEYE_DISTORTION_SIZE (4)

std::shared_ptr<dcl::CameraCalibrationHandle> DclUtils::createDclCalibration(const std::array<std::array<float, 3>, 3>& cameraMatrix,
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
    img.data.assign(mat.data, mat.data + mat.total() * mat.elemSize());

    return img;
}
#endif

float sampsonErrorScalePx(const CalibrationHandler& calibrationHandler, CameraBoardSocket socket, const std::pair<int, int>& resolution) {
    const auto intrinsics = calibrationHandler.getCameraIntrinsics(socket, resolution.first, resolution.second);
    return 0.5f * (intrinsics.at(0).at(0) + intrinsics.at(1).at(1));
}

float sampsonErrorToPixels(float sampsonErrorNormalized, float sampsonErrorScalePixels) {
    return sampsonErrorNormalized * sampsonErrorScalePixels;
}

void DynamicCalibration::setCalibration(CalibrationHandler& handler, bool flash) {
    logger->trace("Applying calibration to device");
    device->setCalibration(handler);
    if(flash) {
        device->flashCalibration(handler);
    }
    calibrationHandler = handler;

    socketToSensorExtrinsics.clear();
    socketToSensorExtrinsics.reserve(socketsInHandler.size());
    for(const auto& socket : socketsInHandler) {
        socketToSensorExtrinsics.push_back(DclUtils::computeBaseToSocketTransform(handler, daiSocketBase, socket));
    }

    for(const auto& sensor : connectedSensors) {
        auto calibration =
            DclUtils::convertDaiCalibrationToDcl(handler, daiSocketBase, sensor.socket, sensor.intrinsics, sensor.distortion, sensor.distortionModel);
        pimplDCL->dynCalibImpl.setCalibration(sensor.sensorDcl, calibration);
    }

    auto& telemetryState = *pimplDCL->telemetryAggregateState;
    std::lock_guard<std::mutex> lock(telemetryState.mutex);
    telemetryState.applyResultCallbacks += 1;
}

void DynamicCalibration::computeMetrics(const CalibrationHandler& handler) {
    auto metrics = std::make_shared<CalibrationMetrics>();
    std::vector<std::shared_ptr<const dcl::CameraSensorHandle>> sensors;
    sensors.reserve(connectedSensors.size());
    for(const auto& sensor : connectedSensors) {
        sensors.push_back(sensor.sensorDcl);
    }

    std::vector<std::shared_ptr<const dcl::CameraCalibrationHandle>> calibrations;
    calibrations.reserve(connectedSensors.size());
    for(const auto& sensor : connectedSensors) {
        calibrations.push_back(
            DclUtils::convertDaiCalibrationToDcl(handler, daiSocketBase, sensor.socket, sensor.intrinsics, sensor.distortion, sensor.distortionModel));
    }

    auto dataConfidence = pimplDCL->dynCalibImpl.computeDataConfidence(sensors);
    metrics->dataConfidence = dataConfidence.passed() ? dataConfidence.value : 0.0;

    auto calibrationConfidence = pimplDCL->dynCalibImpl.computeCalibrationConfidence(calibrations, sensors);
    metrics->calibrationConfidence = calibrationConfidence.passed() ? calibrationConfidence.value : 0.0;
    metricsOutput.send(metrics);

    auto& telemetryState = *pimplDCL->telemetryAggregateState;
    std::lock_guard<std::mutex> lock(telemetryState.mutex);
    telemetryState.metricsResultCallbacks += 1;
}

DynamicCalibration::ErrorCode DynamicCalibration::runCalibration(const dai::CalibrationHandler& currentHandler,
                                                                 const bool force,
                                                                 const bool keepCameraCenters) {
    dcl::PerformanceMode pm = force ? dcl::PerformanceMode::SKIP_CHECKS : DclUtils::daiPerformanceModeToDclPerformanceMode(performanceMode);
    logger->trace("Running calibration (force={} mode={})", force, static_cast<int>(pm));

    std::vector<std::shared_ptr<const dcl::CameraSensorHandle>> syncedSensors;
    syncedSensors.reserve(connectedSensors.size());
    for(const auto& sensor : connectedSensors) {
        syncedSensors.push_back(sensor.sensorDcl);
    }

    std::vector<dcl::EdgeBaseline> keptBaselineEdges;
    if(!keepCameraCenters) {
        if(connectedSensors.size() > 2) {
            keptBaselineEdges.reserve(socketsInHandler.size() > 0 ? socketsInHandler.size() - 1 : 0);
            for(size_t idx = 0; idx + 1 < socketsInHandler.size(); ++idx) {
                const auto* sensorAInChain = findConnectedSensor(socketsInHandler[idx]);
                const auto* sensorBInChain = findConnectedSensor(socketsInHandler[idx + 1]);
                if(sensorAInChain == nullptr || sensorBInChain == nullptr) {
                    continue;
                }
                if(!sensorAInChain->sensorDcl || !sensorBInChain->sensorDcl) {
                    auto result = std::make_shared<DynamicCalibrationResult>("DynamicCalibration sensors were not initialized.");
                    calibrationOutput.send(result);
                    return DynamicCalibration::ErrorCode::CALIBRATION_FAILED;
                }

                const auto* connectedSensorsBegin = connectedSensors.data();
                const auto sensorIndexA = static_cast<std::size_t>(sensorAInChain - connectedSensorsBegin);
                const auto sensorIndexB = static_cast<std::size_t>(sensorBInChain - connectedSensorsBegin);
                const auto baselineLength =
                    static_cast<double>(currentHandler.getBaselineDistance(sensorAInChain->socket, sensorBInChain->socket, true, LengthUnit::METER));
                keptBaselineEdges.push_back({sensorIndexA, sensorIndexB, baselineLength});
            }
        }
    }
    auto dclResult = pimplDCL->dynCalibImpl.findNewCalibration(syncedSensors, pm, keepCameraCenters, keptBaselineEdges);
    if(!dclResult.passed()) {
        auto result = std::make_shared<DynamicCalibrationResult>(dclResult.errorMessage());
        logger->warn("Calibration failed: {}", dclResult.errorMessage());

        calibrationOutput.send(result);
        auto& telemetryState = *pimplDCL->telemetryAggregateState;
        std::lock_guard<std::mutex> lock(telemetryState.mutex);
        telemetryState.calibrationResultCallbacks += 1;
        telemetryState.failedCalibrations += 1;
        return DynamicCalibration::ErrorCode::CALIBRATION_FAILED;
    }

    if(dclResult.value.calibrations.size() != syncedSensors.size()) {
        auto result = std::make_shared<DynamicCalibrationResult>("DCL returned calibration count mismatch.");
        calibrationOutput.send(result);
        return DynamicCalibration::ErrorCode::CALIBRATION_FAILED;
    }

    auto newCalibrationHandler = currentHandler;
    auto candidateSocketToSensorExtrinsics = socketToSensorExtrinsics;

    for(size_t idx = 0; idx < connectedSensors.size(); ++idx) {
        const auto& calibration = dclResult.value.calibrations[idx];
        const auto& sensor = connectedSensors[idx];
        candidateSocketToSensorExtrinsics[sensor.connectionOrder] = DclUtils::calibrationHandleToTransform(calibration);
    }

    for(size_t idx = 0; idx + 1 < socketsInHandler.size(); ++idx) {
        auto transformCurrentToBase = candidateSocketToSensorExtrinsics[idx];
        matrix::invertSe3Matrix4x4InPlace(transformCurrentToBase);
        const auto transformCurrentToNext = matrix::matMul(candidateSocketToSensorExtrinsics[idx + 1], transformCurrentToBase);
        auto translationCurrentToNext = matrix::extractTranslationVector(transformCurrentToNext);
        for(auto& val : translationCurrentToNext) {
            val *= 100.0f;
        }
        newCalibrationHandler.updateCameraExtrinsics(
            socketsInHandler[idx], socketsInHandler[idx + 1], matrix::extractRotationMatrix(transformCurrentToNext), translationCurrentToNext);
    }

    if(std::holds_alternative<HousingCoordinateSystem>(daiSocketBase)) {
        const auto housingOriginSocket = currentHandler.getEepromData().housingExtrinsics.toCameraSocket;
        const auto housingOriginIt = std::find(socketsInHandler.begin(), socketsInHandler.end(), housingOriginSocket);
        if(housingOriginIt != socketsInHandler.end()) {
            const auto housingOriginIndex = static_cast<size_t>(std::distance(socketsInHandler.begin(), housingOriginIt));
            DclUtils::setHousingToDai(newCalibrationHandler, candidateSocketToSensorExtrinsics[housingOriginIndex]);
        }
    }

    CalibrationQuality::Data qualityData{};
    if(!connectedSensors.empty()) {
        const auto& referenceSensor = connectedSensors.front();
        const auto sampsonErrorScalePixels = sampsonErrorScalePx(currentHandler, referenceSensor.socket, referenceSensor.resolution);
        qualityData.sampsonErrorCurrent = sampsonErrorToPixels(static_cast<float>(dclResult.value.sampsonErrorCurrent), sampsonErrorScalePixels);
        qualityData.sampsonErrorNew = sampsonErrorToPixels(static_cast<float>(dclResult.value.sampsonErrorNew), sampsonErrorScalePixels);
    }

    std::map<const dcl::CameraSensorHandle*, CameraBoardSocket> sensorHandleToSocket;
    for(const auto& sensor : connectedSensors) {
        if(sensor.sensorDcl) {
            sensorHandleToSocket.emplace(sensor.sensorDcl.get(), sensor.socket);
        }
    }

    for(const auto& [sensorPair, rotationChange] : dclResult.value.pairwiseRotationDifference) {
        if(!sensorPair.first || !sensorPair.second) continue;

        const auto socket1It = sensorHandleToSocket.find(sensorPair.first.get());
        const auto socket2It = sensorHandleToSocket.find(sensorPair.second.get());
        if(socket1It == sensorHandleToSocket.end() || socket2It == sensorHandleToSocket.end()) continue;

        const auto socketPair = std::make_pair(socket1It->second, socket2It->second);
        qualityData.pairwiseRotationDifference[socketPair] = rotationChange;

        if(leftQueueSocket && rightQueueSocket
           && ((socketPair.first == *leftQueueSocket && socketPair.second == *rightQueueSocket)
               || (socketPair.first == *rightQueueSocket && socketPair.second == *leftQueueSocket))) {
            const auto copyCount = std::min(qualityData.rotationChange.size(), rotationChange.size());
            std::copy_n(rotationChange.begin(), copyCount, qualityData.rotationChange.begin());
        }
    }

    DynamicCalibrationResult::Data resultData{};
    resultData.newCalibration = newCalibrationHandler;
    resultData.currentCalibration = currentHandler;
    resultData.calibrationDifference = qualityData;
    resultData.dataConfidence = dclResult.value.dataConfidence;

    auto result = std::make_shared<DynamicCalibrationResult>(resultData, dclResult.errorMessage());
    calibrationOutput.send(result);
    auto& telemetryState = *pimplDCL->telemetryAggregateState;
    {
        std::lock_guard<std::mutex> lock(telemetryState.mutex);
        telemetryState.calibrationResultCallbacks += 1;
    }
    addDynamicCalibrationResultTelemetry(telemetryState, qualityData);
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
        const auto& sensor = connectedSensors[idx];
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
        images.emplace_back(sensor.sensorDcl, DclUtils::cvMatToImageData(cvFrame));
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
    if(connectedSensors.size() < 2) {
        logger->error("DynamicCalibration requires at least 2 connected sensors.");
        return DynamicCalibration::ErrorCode::CALIBRATION_FAILED;
    }

    std::vector<std::shared_ptr<const dcl::CameraSensorHandle>> sensors;
    sensors.reserve(connectedSensors.size());
    for(const auto& sensor : connectedSensors) {
        sensors.push_back(sensor.sensorDcl);
    }
    auto resultCoverage = pimplDCL->dynCalibImpl.computeCoverage(sensors, DclUtils::daiPerformanceModeToDclPerformanceMode(performanceMode));

    if(!resultCoverage.passed()) {
        throw std::runtime_error("Coverage check failed!");
    }

    auto& coverage = resultCoverage.value;
    auto coverageResult = std::make_shared<CoverageData>();

    const auto coverageCount = std::min(connectedSensors.size(), coverage.coveragesPerCell.size());
    for(size_t idx = 0; idx < coverageCount; ++idx) {
        coverageResult->coveragePerCell[connectedSensors[idx].socket] = coverage.coveragesPerCell[idx];
    }
    if(leftQueueSocket) {
        const auto leftCoverageIt = coverageResult->coveragePerCell.find(*leftQueueSocket);
        if(leftCoverageIt != coverageResult->coveragePerCell.end()) {
            coverageResult->coveragePerCellA = leftCoverageIt->second;
        }
    }
    if(rightQueueSocket) {
        const auto rightCoverageIt = coverageResult->coveragePerCell.find(*rightQueueSocket);
        if(rightCoverageIt != coverageResult->coveragePerCell.end()) {
            coverageResult->coveragePerCellB = rightCoverageIt->second;
        }
    }
    coverageResult->meanCoverage = coverage.meanCoverage;
    coverageResult->coverageAcquired = coverage.coverageAcquired;
    coverageResult->dataAcquired = coverage.dataAcquired;

    logger->trace("Computing coverage");

    coverageOutput.send(coverageResult);

    return DynamicCalibration::ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::initializePipeline(const std::shared_ptr<dai::Device>& daiDevice) {
    logger->trace("Initializing DynamicCalibration pipeline for device: {}", daiDevice->getDeviceId());

    auto inSyncGroup = syncInput.get<dai::MessageGroup>();
    if(!inSyncGroup) {
        return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
    }

    names.clear();
    names = inSyncGroup->getMessageNames();

    connectedSensors.clear();
    connectedSensors.reserve(names.size());

    socketsInHandler.clear();
    socketToSensorExtrinsics.clear();
    leftQueueSocket.reset();
    rightQueueSocket.reset();

    if(names.empty()) {
        logger->error("DynamicCalibration Sync node has no connected image inputs.");
        return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
    }

    if(names.size() < 2) {
        logger->error("DynamicCalibration requires at least 2 connected image inputs.");
        return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
    }

    calibrationHandler = daiDevice->getCalibration();
    auto eepromData = calibrationHandler.getEepromData();

    try {
        socketsInHandler = DclUtils::buildSocketConnection(eepromData);
    } catch(const std::exception& ex) {
        logger->error("Failed to build calibration socketConnection: {}", ex.what());
        return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
    }

    if(eepromData.housingExtrinsics.toCameraSocket != CameraBoardSocket::AUTO) {
        daiSocketBase = HousingCoordinateSystem::AUTO;
    } else {
        daiSocketBase = socketsInHandler.back();
    }

    for(const auto& name : names) {
        const auto frame = inSyncGroup->get<dai::ImgFrame>(name);

        if(!frame) {
            logger->error("Missing synced image '{}' during DynamicCalibration initialization.", name);
            return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
        }

        auto resolution = std::make_pair(static_cast<int>(frame->getWidth()), static_cast<int>(frame->getHeight()));

        const auto frameIntrinsics = matrix::matrix3x3ToVectorMatrix(frame->getTransformation().getIntrinsicMatrix());
        const auto frameDistortion = frame->getTransformation().getDistortionCoefficients();
        const auto frameDistortionModel = frame->getTransformation().getDistortionModel();

        auto socket = static_cast<CameraBoardSocket>(frame->getInstanceNum());

        if(eepromData.cameraData.count(socket) == 0) {
            logger->error("Missing calibration data for socket: {}", toString(socket));
            return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
        }

        const auto socketIt = std::find(socketsInHandler.begin(), socketsInHandler.end(), socket);
        if(socketIt == socketsInHandler.end()) {
            logger->error("Socket {} not found in calibration handler.", toString(socket));
            return DynamicCalibration::ErrorCode::PIPELINE_INITIALIZATION_FAILED;
        }

        connectedSensors.push_back({socket,
                                    frameIntrinsics,
                                    frameDistortion,
                                    frameDistortionModel,
                                    static_cast<size_t>(std::distance(socketsInHandler.begin(), socketIt)),
                                    resolution,
                                    nullptr});

        if(name == leftInputName) {
            leftQueueSocket = socket;
        } else if(name == rightInputName) {
            rightQueueSocket = socket;
        }
    }

    logger->trace("Converting dai calibration to dcl");

    auto platform = daiDevice->getPlatform();
    if(platform == dai::Platform::RVC2 && !eepromData.stereoEnableDistortionCorrection && !oldCalibrationWarningIssued) {
        logger->trace("The calibration on the device is old (distortion correction is disabled), for optimal performance full re-calibration is recommended!");
        oldCalibrationWarningIssued = true;
    }
    // set up the dynamic calibration
    pimplDCL->device = pimplDCL->dynCalibImpl.addDevice();
    socketToSensorExtrinsics.clear();
    socketToSensorExtrinsics.reserve(socketsInHandler.size());

    for(const auto& socket : socketsInHandler) {
        const auto baseToSocketTransform = DclUtils::computeBaseToSocketTransform(calibrationHandler, daiSocketBase, socket);
        socketToSensorExtrinsics.push_back(baseToSocketTransform);
    }

    for(auto& sensor : connectedSensors) {
        dcl::resolution_t resolutionDcl{static_cast<unsigned>(sensor.resolution.first), static_cast<unsigned>(sensor.resolution.second)};
        auto calibration = DclUtils::convertDaiCalibrationToDcl(
            calibrationHandler, daiSocketBase, sensor.socket, sensor.intrinsics, sensor.distortion, sensor.distortionModel);
        sensor.sensorDcl = pimplDCL->dynCalibImpl.addSensor(pimplDCL->device, calibration, resolutionDcl);
    }

    logger->trace("Added {} sensors to dynCalibImpl", connectedSensors.size());

    return DynamicCalibration::ErrorCode::OK;
}

DynamicCalibration::ErrorCode DynamicCalibration::evaluateCommand(const std::shared_ptr<DynamicCalibrationControl>& control) {
    using DC = DynamicCalibrationControl;

    const auto& cmd = control->command;

    // Early exit if command is not set
    if(std::holds_alternative<std::monostate>(cmd)) {
        logger->trace("Received UNSET Command");
        return ErrorCode::INVALID_COMMAND;
    }
    // Calibrate
    if(std::holds_alternative<DC::Commands::Calibrate>(cmd)) {
        const auto& c = std::get<DC::Commands::Calibrate>(cmd);
        logger->trace("Received Calibrate Command: force={}", c.force);
        {
            std::lock_guard<std::mutex> lock(pimplDCL->telemetryAggregateState->mutex);
            pimplDCL->telemetryAggregateState->totalCommands += 1;
            pimplDCL->telemetryAggregateState->calibrateCommands += 1;
        }
        startCalibrationCommand.reset();  // stop the calibration if it is running
        return runCalibration(calibrationHandler, c.force, c.keepCameraCenters);
    }
    // Quality check
    else if(std::holds_alternative<DC::Commands::CalibrationQuality>(cmd)) {
        const auto& c = std::get<DC::Commands::CalibrationQuality>(cmd);
        logger->trace("Received CalibrationQuality Command: force={}", c.force);
        {
            std::lock_guard<std::mutex> lock(pimplDCL->telemetryAggregateState->mutex);
            pimplDCL->telemetryAggregateState->totalCommands += 1;
            pimplDCL->telemetryAggregateState->calibrationQualityCommands += 1;
        }
        logger->warn("CalibrationQuality command is deprecated and returns an empty result");
        qualityOutput.send(std::make_shared<CalibrationQuality>());
        return ErrorCode::OK;
    }
    // Start calibration loop
    else if(std::holds_alternative<DC::Commands::StartCalibration>(cmd)) {
        const auto& c = std::get<DC::Commands::StartCalibration>(cmd);
        logger->trace("Received StartCalibration Command");
        {
            std::lock_guard<std::mutex> lock(pimplDCL->telemetryAggregateState->mutex);
            pimplDCL->telemetryAggregateState->totalCommands += 1;
            pimplDCL->telemetryAggregateState->startCalibrationCommands += 1;
        }
        startCalibrationCommand = c;
        return ErrorCode::OK;
    }
    // Load a single image
    else if(std::holds_alternative<DC::Commands::LoadImage>(cmd)) {
        logger->trace("Received LoadImage Command: blocking load with coverage computation");
        {
            std::lock_guard<std::mutex> lock(pimplDCL->telemetryAggregateState->mutex);
            pimplDCL->telemetryAggregateState->totalCommands += 1;
            pimplDCL->telemetryAggregateState->loadImageCommands += 1;
        }
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
        {
            std::lock_guard<std::mutex> lock(pimplDCL->telemetryAggregateState->mutex);
            pimplDCL->telemetryAggregateState->totalCommands += 1;
            pimplDCL->telemetryAggregateState->applyCalibrationCommands += 1;
        }
        calibrationHandler = c.calibration;
        setCalibration(calibrationHandler, c.flash);
        return ErrorCode::OK;
    }
    // Stop calibration loop
    else if(std::holds_alternative<DC::Commands::StopCalibration>(cmd)) {
        logger->trace("Received StopCalibrationCommand: stopping calibration");
        {
            std::lock_guard<std::mutex> lock(pimplDCL->telemetryAggregateState->mutex);
            pimplDCL->telemetryAggregateState->totalCommands += 1;
            pimplDCL->telemetryAggregateState->stopCalibrationCommands += 1;
        }
        startCalibrationCommand.reset();
        return ErrorCode::OK;
    }
    // Reset/remove accumulated data
    else if(std::holds_alternative<DC::Commands::ResetData>(cmd)) {
        logger->trace("Received RemoveDataCommand: removing the data");
        {
            std::lock_guard<std::mutex> lock(pimplDCL->telemetryAggregateState->mutex);
            pimplDCL->telemetryAggregateState->totalCommands += 1;
            pimplDCL->telemetryAggregateState->resetDataCommands += 1;
        }
        for(size_t i = 0; i < connectedSensors.size(); ++i) {
            for(size_t j = i + 1; j < connectedSensors.size(); ++j) {
                pimplDCL->dynCalibImpl.removeAllData(connectedSensors[i].sensorDcl, connectedSensors[j].sensorDcl);
            }
        }
        return ErrorCode::OK;
    }
    // Set performance mode
    else if(std::holds_alternative<DC::Commands::SetPerformanceMode>(cmd)) {
        const auto& c = std::get<DC::Commands::SetPerformanceMode>(cmd);
        logger->trace("Received SetPerformanceModeCommand: changing performance mode to {}", static_cast<int>(c.performanceMode));
        {
            std::lock_guard<std::mutex> lock(pimplDCL->telemetryAggregateState->mutex);
            pimplDCL->telemetryAggregateState->totalCommands += 1;
            pimplDCL->telemetryAggregateState->setPerformanceModeCommands += 1;
        }
        performanceMode = c.performanceMode;
        return ErrorCode::OK;
    } else if(std::holds_alternative<DC::Commands::ComputeCalibrationMetrics>(cmd)) {
        logger->trace("Received ComputeCalibrationMetrics command");
        const auto& c = std::get<DC::Commands::ComputeCalibrationMetrics>(cmd);
        {
            std::lock_guard<std::mutex> lock(pimplDCL->telemetryAggregateState->mutex);
            pimplDCL->telemetryAggregateState->totalCommands += 1;
            pimplDCL->telemetryAggregateState->computeCalibrationMetricsCommands += 1;
        }
        computeMetrics(c.calibration);
        return ErrorCode::OK;
    }

    // Fallback
    logger->trace("evaluateCommand: Received unknown/unhandled command type");
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
    if(!startCalibrationCommand) {
        return error;
    }
    // Rate limit of the image loading
    auto now = std::chrono::steady_clock::now();

    std::chrono::duration<float> elapsed = now - previousLoadingAndCalibrationTime;
    bool loadingAndCalibrationRequired = elapsed.count() > startCalibrationCommand->loadImagePeriod;
    if(loadingAndCalibrationRequired) {
        logger->trace("doWork() called. CalibrationRunning={}, elapsed={}s", true, elapsed.count());
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
        error = runCalibration(calibrationHandler, false, startCalibrationCommand->keepCameraCenters);
        if(error == DynamicCalibration::ErrorCode::OK) {
            startCalibrationCommand.reset();
        }
    }

    return error;
}

void DynamicCalibration::run() {
    if(!device) {
        logger->error("Dynamic calibration node does not have access to any device.");
        return;
    }

    auto& telemetryState = *pimplDCL->telemetryAggregateState;
    {
        std::lock_guard<std::mutex> lock(telemetryState.mutex);
        telemetryState.telemetryDeviceId = device->getTemporaryTelemetryDeviceId();
    }

    logger->trace("DynamicCalibration node started ");

    auto previousLoadingTimeFloat =
        std::chrono::steady_clock::now() + std::chrono::duration<float>(DynamicCalibrationControl::Commands::StartCalibration{}.calibrationPeriod);
    auto previousLoadingTime = std::chrono::time_point_cast<std::chrono::steady_clock::duration>(previousLoadingTimeFloat);
    auto initResult = initializePipeline(device);
    if(initResult != DynamicCalibration::ErrorCode::OK) {
        logger->error("DynamicCalibration initialization failed with error code {}", static_cast<int>(initResult));
        return;
    }
    utility::Telemetry::getInstance().event(getParentPipeline(),
                                            "depthai_dynamic_calibration_node_started",
                                            nlohmann::json{{"run_on_host", runOnHost()},
                                                           {"performance_mode", telemetryDynamicCalibrationPerformanceModeName(performanceMode)},
                                                           {"connected_sensor_count", connectedSensors.size()}});
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
