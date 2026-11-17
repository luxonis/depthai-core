#include "Camera.hpp"

#include <spdlog/async_logger.h>

#include "depthai/common/CameraModel.hpp"
#include "depthai/common/DepthUnit.hpp"
#include "depthai/common/Extrinsics.hpp"
#include "depthai/common/Timestamp.hpp"
#include "depthai/utility/matrixOps.hpp"

extern "C" {
#include <fastcv/fastcv.h>
#include <fastcv/fastcvExt.h>
}

#include <camera/CameraMetadata.h>

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <exception>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "../ImgTransformationUtils.hpp"
#include "Timesync.hpp"
#include "camera/CameraSingleton.hpp"
#include "camera/LuxonisSidechannel.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraFeatures.hpp"
#include "depthai/common/CameraSensorType.hpp"
#include "depthai/common/ExternalFrameSyncRoles.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/ToF.hpp"
#include "depthai/properties/CameraProperties.hpp"
#include "luxonis_sidechannel_data.h"
#include "pipeline/NodeTiming.hpp"
#include "pipeline/MultiDeviceCalibrationStore.hpp"
#include "utilities/CameraDiscovery.hpp"
#include "utilities/ClockOffset.hpp"
#include "utilities/DeviceModel.hpp"
#include "utilities/DeviceTreeDiscovery.hpp"
#include "utilities/ErrorMacros.hpp"
#include "utilities/EvaDataMemory.hpp"
#include "utilities/FastCvUtils.hpp"
#include "utilities/FsyncController.hpp"
#include "utilities/OSUtils.hpp"
#include "utilities/StereoPairs.hpp"

// #define ENABLE_TIMING_DEBUG

// #define DISABLE_TRY_SEND

#ifndef ALIGN_UP
template <typename T>
constexpr T ALIGN_UP(T value, std::size_t alignment) {
    return (value + (alignment - 1)) & ~(alignment - 1);
}
#endif

namespace dai {
namespace gate {

constexpr float F_EPSILON{1e-6};
namespace {

void logMissingCameraIntrinsics(const std::string& productName,
                               CameraBoardSocket boardSocket,
                               const std::shared_ptr<spdlog::async_logger>& logger) {
    // CS lenses are interchangeable, so factory intrinsics are not expected.
    const bool isOak4Cs = productName == "OAK-4-CS" || productName.rfind("OAK-4-CS-", 0) == 0 || productName == "OAK-4-S-CS"
                          || productName.rfind("OAK-4-S-CS-", 0) == 0;
    if(isOak4Cs) {
        logger->info("No camera intrinsics available for OAK-4-CS. Calibrate with the fitted lens if intrinsics are required.");
    } else {
        logger->warn("No intrinsics data found for camera on socket {}", static_cast<int>(boardSocket));
    }
}

dai::ImgTransformation getCameraTransformation(const CalibrationHandler& calibHandler,
                                               const CameraBoardSocket boardSocket,
                                               const std::string& cameraName,
                                               const std::shared_ptr<spdlog::async_logger>& logger) {
    size_t sourceWidth = calibHandler.getSourceWidth(boardSocket);
    size_t sourceHeight = calibHandler.getSourceHeight(boardSocket);

    const auto cameraFeatures = CameraDiscovery::getInstance().getCameraFeatures(cameraName, boardSocket);
    // Handle the case where the camera was not calibrated with the maximum resolution
    // NOTE: This assumes that the calibration was done with full FoV.
    if(sourceWidth < cameraFeatures.width || sourceHeight < cameraFeatures.height) {
        logger->info("Calibration data for camera on socket {} is not for the maximum resolution. Using the maximum resolution for undistortion.",
                     static_cast<int>(boardSocket));
        sourceWidth = cameraFeatures.width;
        sourceHeight = cameraFeatures.height;
    }

    const auto intrinsics =
        matrix::vectorMatrixToMatrix3x3(calibHandler.getCameraIntrinsics(boardSocket, static_cast<int>(sourceWidth), static_cast<int>(sourceHeight)));

    std::vector<float> distortionCoefficients = calibHandler.getDistortionCoefficients(boardSocket);
    dai::CameraModel distortionModel = calibHandler.getDistortionModel(boardSocket);
    // getExtrinsicsToOrigin always writes a real socket into originSocket or throws.
    dai::CameraBoardSocket originSocket = dai::CameraBoardSocket::AUTO;
    const auto cameraToOrigin = calibHandler.getExtrinsicsToOrigin(boardSocket, false, originSocket);

    dai::Extrinsics extrinsics(cameraToOrigin, originSocket, dai::LengthUnit::CENTIMETER);
    try {
        // Same target as the measured pose above so the struct stays consistent.
        const auto specTranslation = calibHandler.getCameraTranslationVector(boardSocket, originSocket, true, dai::LengthUnit::CENTIMETER);
        extrinsics.specTranslation = {specTranslation[0], specTranslation[1], specTranslation[2]};
    } catch(const std::exception& e) {
        logger->debug("No spec translation available for camera on socket {} to origin socket {}: {}",
                      static_cast<int>(boardSocket),
                      static_cast<int>(originSocket),
                      e.what());
    }

    dai::ImgTransformation transformation(sourceWidth, sourceHeight, intrinsics, distortionModel, distortionCoefficients, extrinsics);

    return transformation;
}

dai::ImgTransformation adjustTransformationSize(dai::ImgTransformation transformation,
                                                uint32_t resolutionWidth,
                                                uint32_t resolutionHeight,
                                                bool& warnedForMissingCalibration,
                                                const std::shared_ptr<spdlog::async_logger>& logger) {
    if(transformation.isValid()) {
        const auto [baseWidth, baseHeight] = transformation.getSize();
        const auto scaleX = static_cast<float>(resolutionWidth) / baseWidth;
        const auto scaleY = static_cast<float>(resolutionHeight) / baseHeight;
        transformation.addScale(scaleX, scaleY);
        if(std::abs(scaleX - scaleY) > F_EPSILON && !warnedForMissingCalibration) {
            warnedForMissingCalibration = true;
            logger->warn("Scaling X and Y by different factors in camera node should not happen, please report to developers. X: {}, Y: {}", scaleX, scaleY);
        }
    } else {
        transformation = ImgTransformation(resolutionWidth, resolutionHeight);
    }

    return transformation;
}

bool trySendSyncedOutput(Node::Output& output, const std::shared_ptr<ADatatype>& msg) {
    for(const auto& connection : output.getConnections()) {
        if(connection.in != nullptr && connection.in->isFull()) {
            return false;
        }
    }

    for(const auto& queueConnection : output.getQueueConnections()) {
        if(queueConnection.queue != nullptr && queueConnection.queue->isFull()) {
            return false;
        }
    }

    return output.trySend(msg);
}

lux_sc_camera_fsync_t fromImgFrameFsync(dai::ImgFrame::Fsync fsyncMode) {
    switch(fsyncMode) {
        case dai::ImgFrame::Fsync::INPUT:
            return LUX_SC_CAMERA_FSYNC_SLAVE;
        case dai::ImgFrame::Fsync::OUTPUT:
            return LUX_SC_CAMERA_FSYNC_MASTER;
        case dai::ImgFrame::Fsync::PTP:
            return LUX_SC_CAMERA_FSYNC_TIME;
        case dai::ImgFrame::Fsync::NONE:
        default:
            return LUX_SC_CAMERA_FSYNC_NONE;
    }
}
}  // namespace

std::vector<CameraBoardSocket> Camera::camerasInThePipeline = {};

void Camera::setDeviceId(const std::string& deviceId) {
    this->deviceId = deviceId;
}

namespace {

std::map<dai::gate::DeviceModel::Model const, std::vector<dai::CameraBoardSocket> const> const exposureMasterCameraPriority = {
    {dai::gate::DeviceModel::Model::OAK4_D, {dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, dai::CameraBoardSocket::CAM_A}},
    {dai::gate::DeviceModel::Model::OAK4_LITE, {dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, dai::CameraBoardSocket::CAM_A}},
    {dai::gate::DeviceModel::Model::OAK4_S, {dai::CameraBoardSocket::CAM_A}},
    {dai::gate::DeviceModel::Model::OAK4_MAX, {dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, dai::CameraBoardSocket::CAM_A}}};

struct HighSpeedVideoConfig {
    int32_t width;
    int32_t height;
    int32_t minFps;
    int32_t maxFps;
    int32_t batchSize;
};

std::pair<std::optional<HighSpeedVideoConfig>, std::set<float>> getClosestHighSpeedVideoConfig(
    const std::shared_ptr<spdlog::async_logger>& logger, dai::CameraBoardSocket socket, int32_t sensorModeWidth, int32_t sensorModeHeight, float targetFps) {
    constexpr uint32_t kHighSpeedVideoConfigEntrySize = 5;
    std::set<float> possibleFpsValues;
    DAI_CHECK_IN(logger);

    const auto cameraId = dai::gate::CameraDiscovery::getInstance().getCameraId(socket);
    DAI_CHECK_IN(cameraId >= 0);

    const auto& cameraInfos = CameraSingleton::getInstance().cameraInfos;
    if(static_cast<size_t>(cameraId) >= cameraInfos.size()) {
        logger->trace("Couldn't query HFR metadata for socket {} and sensor mode {}x{} at target fps {}: camera info missing",
                      static_cast<int>(socket),
                      sensorModeWidth,
                      sensorModeHeight,
                      targetFps);
        return {std::nullopt, possibleFpsValues};
    }

    using CameraMetadata = android::hardware::camera::common::V1_0::helper::CameraMetadata;
    CameraMetadata staticMeta;
    staticMeta = cameraInfos[cameraId].static_camera_characteristics;
    if(!staticMeta.exists(ANDROID_CONTROL_AVAILABLE_HIGH_SPEED_VIDEO_CONFIGURATIONS)) {
        logger->trace("No ANDROID_CONTROL_AVAILABLE_HIGH_SPEED_VIDEO_CONFIGURATIONS for socket {} and sensor mode {}x{} at target fps {}",
                      static_cast<int>(socket),
                      sensorModeWidth,
                      sensorModeHeight,
                      targetFps);
        return {std::nullopt, possibleFpsValues};
    }

    const auto entry = staticMeta.find(ANDROID_CONTROL_AVAILABLE_HIGH_SPEED_VIDEO_CONFIGURATIONS);
    std::optional<HighSpeedVideoConfig> bestAtOrAboveTargetConfig;
    std::optional<HighSpeedVideoConfig> bestBelowTargetConfig;
    bool foundMatchingConfig = false;
    logger->trace(
        "Checking HFR metadata for socket {} sensor mode {}x{} target fps {}", static_cast<int>(socket), sensorModeWidth, sensorModeHeight, targetFps);
    for(uint32_t i = 0; i + (kHighSpeedVideoConfigEntrySize - 1) < entry.count; i += kHighSpeedVideoConfigEntrySize) {
        const int32_t width = entry.data.i32[i + 0];
        const int32_t height = entry.data.i32[i + 1];
        const int32_t minFps = entry.data.i32[i + 2];
        const int32_t maxFps = entry.data.i32[i + 3];
        const int32_t batchSize = entry.data.i32[i + 4];
        if(width != sensorModeWidth || height != sensorModeHeight || minFps <= 0 || maxFps <= 0) {
            continue;
        }

        foundMatchingConfig = true;
        const HighSpeedVideoConfig config{width, height, minFps, maxFps, batchSize};
        logger->trace("Available HFR mode for socket {}: {}x{} minFPS {} maxFPS {} batchSize {}",
                      static_cast<int>(socket),
                      config.width,
                      config.height,
                      config.minFps,
                      config.maxFps,
                      config.batchSize);

        possibleFpsValues.insert(static_cast<float>(maxFps));
        if(maxFps + F_EPSILON >= targetFps) {
            if(!bestAtOrAboveTargetConfig || maxFps < bestAtOrAboveTargetConfig->maxFps) {
                bestAtOrAboveTargetConfig = config;
            }
        } else if(!bestBelowTargetConfig || maxFps > bestBelowTargetConfig->maxFps) {
            bestBelowTargetConfig = config;
        }
    }

    if(bestAtOrAboveTargetConfig) {
        logger->trace("Selected HFR mode for socket {}: {}x{} minFPS {} maxFPS {} batchSize {} for target fps {}",
                      static_cast<int>(socket),
                      bestAtOrAboveTargetConfig->width,
                      bestAtOrAboveTargetConfig->height,
                      bestAtOrAboveTargetConfig->minFps,
                      bestAtOrAboveTargetConfig->maxFps,
                      bestAtOrAboveTargetConfig->batchSize,
                      targetFps);
        return {bestAtOrAboveTargetConfig, possibleFpsValues};
    }
    if(bestBelowTargetConfig) {
        logger->trace("Selected fallback HFR mode for socket {}: {}x{} minFPS {} maxFPS {} batchSize {} for target fps {}",
                      static_cast<int>(socket),
                      bestBelowTargetConfig->width,
                      bestBelowTargetConfig->height,
                      bestBelowTargetConfig->minFps,
                      bestBelowTargetConfig->maxFps,
                      bestBelowTargetConfig->batchSize,
                      targetFps);
        return {bestBelowTargetConfig, possibleFpsValues};
    }

    if(!foundMatchingConfig) {
        logger->trace("No matching HFR modes in metadata for socket {} sensor mode {}x{} and target fps {}",
                      static_cast<int>(socket),
                      sensorModeWidth,
                      sensorModeHeight,
                      targetFps);
    }
    return {std::nullopt, possibleFpsValues};
}

std::string vectorToString(const std::set<float>& vec) {
    std::ostringstream oss;
    bool first = true;

    for(float val : vec) {
        if(!first) oss << ", ";
        oss << val;
        first = false;
    }

    return oss.str();
}

}  // namespace

static std::shared_ptr<ImgFrame> convertToFormat(ImgFrame::Type type, std::shared_ptr<ImgFrame> inFrame) {
    DAI_CHECK_IN(inFrame);
    DAI_CHECK_IN(inFrame->getType() == ImgFrame::Type::NV12);
    DAI_CHECK(type == ImgFrame::Type::RGB888i || type == ImgFrame::Type::BGR888i || type == ImgFrame::Type::GRAY8 || type == ImgFrame::Type::RAW8,
              "Only supported Camera node output formats are: NV12 (most performant), GRAY8 (very performant), RAW8 (very performant), "
              "RGB888i (middle performant), BGR888i (worst performant)");

    if(type == ImgFrame::Type::GRAY8 || type == ImgFrame::Type::RAW8) {
        // If, for example, we have a 2x2 image, NV12 is a 2x2x1.5 = 6 bytes in memory
        // whereas GRAY8/RAW8 occupies just 2x2x1 = 4 bytes in memory. Moreover, we can view the luminance plane of NV12 as a GRAY8/RAW8 image.
        // Therefore, we just take the first N bytes of the NV12 image to form a GRAY8/RAW8 image.
        // This is done by resizing the underlying memory of the ImgFrame to the size of the GRAY8/RAW8 image.
        inFrame->data->setSize(inFrame->getStride() * ALIGN_UP(inFrame->getHeight(), 32));

        // Set other metadata to match the image
        inFrame->setType(type);
        // inFrame->fb.stride = inFrame->getWidth();
        inFrame->fb.p1Offset = 0;
        inFrame->fb.p2Offset = 0;
        inFrame->fb.p3Offset = 0;
        return inFrame;
    }

    std::shared_ptr<dai::ImgFrame> outFrame = std::make_shared<dai::ImgFrame>();
    outFrame->setMetadata(*inFrame);
    outFrame->setSize(inFrame->getWidth(), inFrame->getHeight());
    outFrame->setType(type);
    outFrame->fb.stride = 3 * outFrame->getWidth();
    outFrame->fb.p1Offset = 0;
    outFrame->fb.p2Offset = 0;
    outFrame->fb.p3Offset = 0;
    outFrame->setBufferMetadataFrom(inFrame);
    outFrame->cam = inFrame->cam;
    size_t outputSize = outFrame->getStride() * outFrame->getHeight();
    outFrame->data = dai::EvaDataMemory::create(outputSize);
    DAI_CHECK_IN(outFrame->data);
    const uint8_t* srcPtr = inFrame->getData().data();
    uint8_t* dstPtr = outFrame->getData().data();
    fcvRun([&]() {
        fcvColorYCbCr420PseudoPlanarToRGB888u8(srcPtr,
                                               srcPtr + inFrame->fb.p2Offset,
                                               inFrame->getWidth(),
                                               inFrame->getHeight(),
                                               inFrame->getStride(),
                                               inFrame->getStride(),
                                               dstPtr,
                                               outFrame->getStride());
    });
    if(type == ImgFrame::Type::BGR888i) {
        auto tmpData = dai::EvaDataMemory::create(outputSize);
        DAI_CHECK_IN(dstPtr && tmpData && tmpData->data());
        fcvRun([&]() { fcvColorRGB888ToBGR888u8(dstPtr, inFrame->getWidth(), inFrame->getHeight(), 0, tmpData->data(), 0); });
        outFrame->data = tmpData;
    }
    return outFrame;
}

std::shared_ptr<ImgFrame> Camera::toCorrectFormat(ImgFrameCapability cap, std::shared_ptr<ImgFrame> inFrame) {
    if(cap.type && *cap.type != inFrame->getType()) {
        auto tStart = std::chrono::steady_clock::now();
        auto outFrame = convertToFormat(*cap.type, inFrame);
        logger->debug("Camera timings: color_convert: {}ms",
                      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - tStart).count() / 1000);
        return outFrame;
    } else {
        return inFrame;
    }
}

float Camera::getFps() {
    if(FsyncController::getInstance().supportsExternalFrameSyncMode() && FsyncController::getInstance().isOk()
       && FsyncController::getInstance().getExternalFrameSyncRole() == dai::ExternalFrameSyncRole::SLAVE && properties.fps != -1
       && (properties.initialControl.frameSyncMode == dai::CameraControl::FrameSyncMode::AUTO
           || properties.initialControl.frameSyncMode == dai::CameraControl::FrameSyncMode::INPUT)) {
        throw std::runtime_error("Cannot override fps while using external FSYNC slave mode");
    }

    if(FsyncController::getInstance().supportsExternalFrameSyncMode() && FsyncController::getInstance().isOk()
       && FsyncController::getInstance().getExternalFrameSyncRole() == dai::ExternalFrameSyncRole::SLAVE
       && (properties.initialControl.frameSyncMode == dai::CameraControl::FrameSyncMode::AUTO
           || properties.initialControl.frameSyncMode == dai::CameraControl::FrameSyncMode::INPUT)) {
        return FsyncController::getInstance().slaveMeasureFrequency();
    }

    if(properties.fps > 0.0f) {
        return properties.fps;
    }
    float maxSelectedFps{-1.0f};
    for(const auto& output : properties.outputRequests) {
        if(output.fps.value) {
            if(const auto* fps = std::get_if<float>(&(*output.fps.value))) {
                maxSelectedFps = std::max(maxSelectedFps, *fps);
            } else if(const auto* fps = std::get_if<std::pair<float, float>>(&(*output.fps.value))) {
                maxSelectedFps = std::max(maxSelectedFps, std::get<0>(*fps));
            } else if(const auto* fps = std::get_if<std::vector<float>>(&(*output.fps.value))) {
                DAI_CHECK(fps->size() > 0, "When passing a vector to ImgFrameCapability->fps, please pass a non empty vector!");
                maxSelectedFps = std::max(maxSelectedFps, *std::max_element(fps->begin(), fps->end()));
            } else {
                DAI_CHECK_IN(false);
            }
        }
    }
    return maxSelectedFps;
}

size_t Camera::getMaxFpsIndex() const {
    size_t maxFpsIndex = 0;
    float maxFps = -1.0f;
    size_t i = 0;

    for(const auto& output : properties.outputRequests) {
        if(output.fps.value) {
            if(const auto* fps = std::get_if<float>(&(*output.fps.value))) {
                if(*fps > maxFps) {
                    maxFps = *fps;
                    maxFpsIndex = i;
                }
            } else if(const auto* fps = std::get_if<std::pair<float, float>>(&(*output.fps.value))) {
                if(std::get<0>(*fps) > maxFps) {
                    maxFps = std::get<0>(*fps);
                    maxFpsIndex = i;
                }
            } else if(const auto* fps = std::get_if<std::vector<float>>(&(*output.fps.value))) {
                DAI_CHECK(fps->size() > 0, "When passing a vector to ImgFrameCapability->fps, please pass a non empty vector!");
                const auto candidate = *std::max_element(fps->begin(), fps->end());
                if(candidate > maxFps) {
                    maxFps = candidate;
                    maxFpsIndex = i;
                }
            } else {
                DAI_CHECK_IN(false);
            }
        }
        ++i;
    }
    return maxFpsIndex;
}

std::vector<std::pair<int, dai::CameraSensorConfig>> Camera::filterByHDR(const std::vector<std::pair<int, dai::CameraSensorConfig>>& modes) const {
    std::vector<std::pair<int, dai::CameraSensorConfig>> out;
    const bool hdrRequested = properties.initialControl.getHdr();
    for(const auto& m : modes) {
        if(hdrRequested == m.second.hdr) {
            out.push_back(m);
        }
    }
    return out;
}

std::vector<std::pair<int, dai::CameraSensorConfig>> Camera::filterByFps(const std::vector<std::pair<int, dai::CameraSensorConfig>>& modes,
                                                                         float requestedFps) {
    std::vector<std::pair<int, dai::CameraSensorConfig>> out;
    out.reserve(modes.size());
    for(const auto& mode : modes) {
        const auto& cfg = mode.second;
        /*
         * the modes are sorted lowest to highest and we have a rule on the OS side:
         * any lower resolution mode should not have lower fps than the higher resolution modes for that sensor (separate by HDR / non-HDR category)
         */
        if(/*requestedFps >= cfg.minFps && TODO(jakgra) reenable this after adding < 10fps support to imx586 driver in the OS */ requestedFps <= cfg.maxFps) {
            out.push_back(mode);
        }
    }
    return out;
}

std::pair<uint32_t, uint32_t> Camera::getLowestSatisfyingSensorSize() const {
    uint32_t maxW = 0;
    uint32_t maxH = 0;
    for(const auto& spec : properties.outputRequests) {
        if(!spec.size.value) {
            continue;
        }
        const auto* size = std::get_if<std::pair<uint32_t, uint32_t>>(&(*spec.size.value));
        maxW = std::max(maxW, size->first);
        maxH = std::max(maxH, size->second);
    }
    return {maxW, maxH};
}

Camera::SensorConfig Camera::returnSensorConfig(const std::pair<int, dai::CameraSensorConfig>& mode, float requestedFps) const {
    float selectedFps{-1.0f};
    if(mode.second.hfr && requestedFps > 120.999999f) {
        const float targetFps = requestedFps > 0.0f ? requestedFps : mode.second.maxFps;
        const auto selectedHighSpeedConfig = getClosestHighSpeedVideoConfig(logger, properties.boardSocket, mode.second.width, mode.second.height, targetFps);
        if(selectedHighSpeedConfig.first) {
            selectedFps = static_cast<float>(selectedHighSpeedConfig.first->maxFps);
            logger->trace("Chosen HFR fps for socket {} sensor mode {}x{}: requested {}, selected {}",
                          static_cast<int>(properties.boardSocket),
                          mode.second.width,
                          mode.second.height,
                          targetFps,
                          selectedFps);
        } else {
            selectedFps = mode.second.maxFps;
            logger->trace("Chosen HFR fps for socket {} sensor mode {}x{}: requested {}, falling back to sensor mode max fps {}",
                          static_cast<int>(properties.boardSocket),
                          mode.second.width,
                          mode.second.height,
                          targetFps,
                          selectedFps);
        }
        if(properties.fps > 0.0f) {
            DAI_CHECK_V(std::fabs(selectedFps - std::round(requestedFps)) < F_EPSILON,
                        "Can't set camera {} sensor fps to {} in {}x{} sensor mode. Only supported fps are: 0-120 and discrete HFR values: {}",
                        static_cast<int>(properties.boardSocket),
                        requestedFps,
                        mode.second.width,
                        mode.second.height,
                        vectorToString(selectedHighSpeedConfig.second));
        }
        return {mode.first, mode.second, selectedFps};
    }
    if(requestedFps < 0.0f) {
        if(mode.second.minFps <= 30.00001f && mode.second.maxFps >= 29.99999f) {
            selectedFps = 30.0f;
        } else {
            selectedFps = mode.second.maxFps;
        }
    } else {
        selectedFps = requestedFps;
    }
    DAI_CHECK_IN(properties.fps);
    return {mode.first, mode.second, selectedFps};
}

static bool isFullFOV(dai::CameraSensorConfig mode) {
    return std::fabs(mode.fov.topLeft().x - 0) < F_EPSILON && std::fabs(mode.fov.topLeft().y - 0) < F_EPSILON
           && std::fabs(mode.fov.bottomRight().x - 0) < F_EPSILON && std::fabs(mode.fov.bottomRight().y - 0) < F_EPSILON;
}

Camera::SensorConfig Camera::decideSensorConfig() {
    auto cameraFeatures = CameraDiscovery::getInstance().getCameraFeatures(properties.cameraName, properties.boardSocket);
    std::vector<std::pair<int, dai::CameraSensorConfig>> modes;
    modes.reserve(cameraFeatures.configs.size());
    for(int idx = 0; idx < cameraFeatures.configs.size(); ++idx) {
        modes.emplace_back(idx, cameraFeatures.configs[idx]);
    }
    // Filter out HDR or non-HDR sensor modes based on what user requested.
    modes = filterByHDR(modes);
    // Sort by area so below logic works.
    std::sort(modes.begin(), modes.end(), [](const auto& a, const auto& b) { return a.second.width * a.second.height > b.second.width * b.second.height; });
    const float requestedFps = getFps();
    // If the user overrode the sensor mode use that or error out.
    const bool overrideRes = properties.resolutionWidth != dai::CameraProperties::AUTO || properties.resolutionHeight != dai::CameraProperties::AUTO;
    if(overrideRes) {
        DAI_CHECK(properties.resolutionWidth != dai::CameraProperties::AUTO && properties.resolutionHeight != dai::CameraProperties::AUTO,
                  "Both resolutionWidth and resolutionHeight must be set when overriding sensor resolution");
        for(const auto& mode : modes) {
            const auto& cfg = mode.second;
            if(std::abs(cfg.width - properties.resolutionWidth) < F_EPSILON && std::abs(cfg.height - properties.resolutionHeight) < F_EPSILON) {
                DAI_CHECK_V(requestedFps < 0.0f || (requestedFps >= cfg.minFps && requestedFps <= cfg.maxFps),
                            "Requested fps is not supported by the overridden sensor resolution (requested: {}, supported: [{}, {}]). HDR: {}",
                            requestedFps,
                            cfg.minFps,
                            cfg.maxFps,
                            properties.initialControl.getHdr());
                return returnSensorConfig(mode, requestedFps);
            }
        }
        DAI_CHECK_V(false, "Requested sensor resolution {}x{} not found", properties.resolutionWidth, properties.resolutionHeight);
    }
    // If the user requested specific FPS settings filter out all the sensor modes that can't satisfy it.
    if(requestedFps > 0.0f) {
        modes = filterByFps(modes, requestedFps);
    }
    // Find which sensor mode can satisfy us. We prefer bigger because sensor scaling is worse then IPE scaling - That is why we sorted modes by area.
    // Our other preferences by priority are:
    // - never upscale !!!
    // - fullFOV, so we see it all
    // - cheap on IPE instances (area <= 5000 * 4000) // TODO(jakgra) get IPE (spectra / titan HW) cost per sensor mode to be more exact here
    // - fallback on non fullFOV is allowed
    auto lowestSatisfyingSensorSize = getLowestSatisfyingSensorSize();
    enum Condition {
        FULL_FOV,
        CHEAP_IPE,
    };
    for(auto& conditions : std::vector<std::vector<Condition>>({{FULL_FOV, CHEAP_IPE}, {FULL_FOV}, {CHEAP_IPE}, {}})) {
        for(auto& mode : modes) {
            bool modeValid{true};
            for(auto& condition : conditions) {
                if(condition == FULL_FOV && !isFullFOV(mode.second)) {
                    modeValid = false;
                    continue;
                } else if(condition == CHEAP_IPE && mode.second.width * mode.second.height > 5000 * 4000) {
                    modeValid = false;
                    continue;
                }
            }
            if(modeValid && mode.second.width >= std::get<0>(lowestSatisfyingSensorSize) && mode.second.height >= std::get<1>(lowestSatisfyingSensorSize)) {
                return returnSensorConfig(mode, requestedFps);
            }
        }
    }
    uint32_t maxW = 0;
    uint32_t maxH = 0;
    for(const auto& mode : modes) {
        if(requestedFps >= mode.second.minFps && requestedFps <= mode.second.maxFps) {
            const auto area = static_cast<uint64_t>(mode.second.width) * static_cast<uint64_t>(mode.second.height);
            const auto bestArea = static_cast<uint64_t>(maxW) * static_cast<uint64_t>(maxH);
            if(area > bestArea) {
                maxW = mode.second.width;
                maxH = mode.second.height;
            }
        }
    }
    DAI_CHECK_V(false,
                "Couldn't satisfy size {}x{}@{} for sensor {}{}. At {} fps max supported resolution is {}x{}.",
                std::get<0>(lowestSatisfyingSensorSize),
                std::get<1>(lowestSatisfyingSensorSize),
                requestedFps,
                cameraFeatures.sensorName,
                properties.initialControl.getHdr() ? ", with HDR enabled" : "",
                requestedFps,
                maxW,
                maxH);
}

void Camera::refreshMultiDeviceCalibration() {
    auto& store = MultiDeviceCalibrationStore::getInstance();
    if(store.getGeneration() == multiDeviceCalibrationId) {
        return;
    }
    const auto snapshot = store.get();
    logger->debug("Multi-device calibration changed (generation {} -> {}), {}",
                  multiDeviceCalibrationId,
                  snapshot.generation,
                  snapshot.handler ? "rebasing to the graph origin" : "publishing local extrinsics");
    multiDeviceCalibration = snapshot.handler;
    multiDeviceCalibrationId = snapshot.generation;
    warnedForMultiDeviceMismatch = false;
}

void Camera::sendBuffer(CameraSensor::FrameCallbackArgs args) {
    auto mainBlockEvent = this->blockEvent(PipelineEvent::Type::LOOP, "_mainLoop");
    refreshMultiDeviceCalibration();
    dai::CameraBoardSocket socket = properties.boardSocket;
    using namespace std::chrono;

#ifdef ENABLE_TIMING_DEBUG
    const auto startTime = high_resolution_clock::now();
#endif
    logger->trace("Received frame for '{}': exposure={}, seqNum={}\n", args.output.c_str(), args.cam.exposureTimeUs, args.sequenceNumber);
    std::chrono::nanoseconds steadyClockDiff;
    std::chrono::nanoseconds systemClockDiff;
    std::optional<dai::Timestamp> tsSystem = std::nullopt;
    if(getOSVersion() >= dai::Version(1, 21, 0)) {
        steadyClockDiff = SteadyToQTimerOffset::instance().offset();
        systemClockDiff = SystemToQTimerOffset::instance().offset();

        // Convert the system clock difference to seconds and nanoseconds
        auto systemClockSec = duration_cast<seconds>(systemClockDiff).count();
        auto systemClockNsec = duration_cast<nanoseconds>(systemClockDiff - seconds(systemClockSec)).count();
        tsSystem.emplace();
        auto nsec = args.ts.nsec + systemClockNsec;
        tsSystem->nsec = nsec % 1000000000;
        tsSystem->sec = args.ts.sec + systemClockSec + nsec / 1000000000;
    } else {
        steadyClockDiff = SteadyToRawOffset::instance().offset();
    }

    // Convert the steady clock difference to seconds and nanoseconds
    auto steadyClockSec = duration_cast<seconds>(steadyClockDiff).count();
    auto steadyClockNsec = duration_cast<nanoseconds>(steadyClockDiff - seconds(steadyClockSec)).count();
    args.ts.sec += steadyClockSec;
    args.ts.nsec += steadyClockNsec;

    const std::string dynOut{"dynOut"};
    if(args.output.rfind(dynOut, 0) == 0) {
        const auto outKey = args.output.substr(dynOut.size());
        const size_t outIndex = std::stoul(outKey);
        DAI_CHECK_IN(outIndex < currentOutputs.size() && outIndex < properties.outputRequests.size());
        const auto& req = properties.outputRequests[outIndex];

        uint32_t& currentEepromId = currentOutputs[outIndex].eepromId;
        uint32_t latestEepromId = getParentPipeline().getEepromId();
        // TODO(aljazkonec): move to a funciton to avoid code duplication with setupResizers()
        if(latestEepromId > currentEepromId) {
            logger->debug("EEPROM data changed (ID: {} -> {}), reconfiguring ...", currentEepromId, latestEepromId);

            auto calibHandler = getParentPipeline().getCalibrationData();

            if(calibHandler.getEepromData().cameraData.count(properties.boardSocket) > 0) {
                dai::ImgTransformation outputTransformation = getCameraTransformation(calibHandler, properties.boardSocket, properties.cameraName, logger);
                calibrationWidth = outputTransformation.getSourceSize().first;
                calibrationHeight = outputTransformation.getSourceSize().second;
                currentOutputs[outIndex].transformation = adjustTransformationSize(outputTransformation,
                                                                                   currentOutputs[outIndex].resolutionWidth,
                                                                                   currentOutputs[outIndex].resolutionHeight,
                                                                                   warnedForMissingCalibration,
                                                                                   logger);
            } else if(!warnedForMissingCalibration) {
                logMissingCameraIntrinsics(calibHandler.getEepromData().productName, properties.boardSocket, logger);
            }

            currentEepromId = latestEepromId;
        }

        // TODO(jakgra) get format and other things from specs array augmented size
        int size = args.wstride * args.hstride * 3 / 2;
        std::shared_ptr<dai::ImgFrame> NV12frame = makeFrame(args, tsSystem, dai::ImgFrame::Type::NV12, false, socket, currentOutputs[outIndex].transformation);

        // Route the main sensor output to ISP-only consumers as well.
        if(outIndex == maxResolutionIndex) {
            for(size_t i = 0; i < currentOutputs.size(); ++i) {
                if(!currentOutputs[i].ispOutput) {
                    continue;
                }
                if(currentOutputs[i].fpsRegulator != nullptr) {
                    currentOutputs[i].fpsRegulator->inQueue.trySend(NV12frame);
                } else {
                    const auto& ispReq = properties.outputRequests[i];
                    trySendSyncedOutput(dynamicOutputs[std::to_string(i)], toCorrectFormat(ispReq, NV12frame));
                }
            }
        }

        if(currentOutputs[outIndex].fpsRegulator != nullptr) {
            logger->trace("cam-{}out-{}-frame-{} HERE START, size: {}x{} to fps regulator 0 : date: {}\n",
                          static_cast<int>(properties.boardSocket),
                          outKey.c_str(),
                          args.sequenceNumber,
                          args.width,
                          args.height,
                          std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
            {
                auto blockEvent = this->outputBlockEvent();
#ifdef DISABLE_TRY_SEND
                currentOutputs[outIndex].fpsRegulator->inQueue.send(NV12frame);
#else
                const auto res = currentOutputs[outIndex].fpsRegulator->inQueue.trySend(NV12frame);
                if(!res) {
                    logger->info("Dropping frame in fps regulator {}: {}x{} on cam {}, because queue full", outKey, args.width, args.height, toString(socket));
                }
#endif
            }
        } else if(currentOutputs[outIndex].resizer != nullptr) {
            logger->trace("cam-%dout-%s-frame-%lu HERE START, size: %dx%d to resizer 0 : date: %lu\n",
                          static_cast<int>(properties.boardSocket),
                          outKey.c_str(),
                          args.sequenceNumber,
                          args.width,
                          args.height,
                          std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
            static int frameCounter{0};
            {
                auto blockEvent = this->outputBlockEvent();
#ifdef DISABLE_TRY_SEND
                currentOutputs[outIndex].resizer->inQ.send(NV12frame);
#else
                const auto res = currentOutputs[outIndex].resizer->inQ.trySend(NV12frame);
                if(!res) {
                    logger->info("Dropping frame in resizer {}: {}x{} on cam {}, because queue full", outKey, args.width, args.height, toString(socket));
                }
#endif
            }
        } else {
            logger->trace("cam-%dout-%s-frame-%lu HERE START, size: %dx%d DIRECT RETURN\n",
                          static_cast<int>(properties.boardSocket),
                          outKey.c_str(),
                          args.sequenceNumber,
                          args.width,
                          args.height);
            static int frameCounter{0};
            {
                auto blockEvent = this->outputBlockEvent();
#ifdef DISABLE_TRY_SEND
                dynamicOutputs[outKey].send(toCorrectFormat(req, NV12frame));
#else
                const auto res = trySendSyncedOutput(dynamicOutputs[outKey], toCorrectFormat(req, NV12frame));
                if(!res) {
                    logger->info("Dropping frame in output {}: {}x{} on cam {}, because queue full", outKey, args.width, args.height, toString(socket));
                }
#endif
            }
        }
        logger->trace(
            "sent dynamic output CAM[%d] SPECS[%s] frame..., size: %dx%d\n", static_cast<int>(properties.boardSocket), outKey.c_str(), args.width, args.height);
    } else if(args.output == "raw") {
        dai::ImgFrame::Type type = dai::ImgFrame::Type::RAW10;  // TODO(Morato) make this configurable
        ImgTransformation rawTransformation = baseTransformation;
        if(rawTransformation.isValid() && (args.width != calibrationWidth || args.height != calibrationHeight)) {
            rawTransformation.addScale((float)args.width / (float)calibrationWidth, (float)args.height / (float)calibrationHeight);
        } else if(!rawTransformation.isValid()) {
            rawTransformation = ImgTransformation(args.width, args.height);
        }
        std::shared_ptr<dai::ImgFrame> rawFrame = makeFrame(args, tsSystem, type, false, socket, rawTransformation);
        {
            auto blockEvent = this->outputBlockEvent();
#ifdef DISABLE_TRY_SEND
            raw.send(rawFrame);
#else
            const auto res = trySendSyncedOutput(raw, rawFrame);
            if(!res) {
                logger->info("Dropping frame in raw output: {}x{} on cam {}, because queue full", args.width, args.height, toString(socket));
            }
#endif
        }
        logger->trace("sent raw frame...\n");
    } else {
        logger->error("Unknown output: {}", args.output);
    }
#ifdef ENABLE_TIMING_DEBUG
    const auto endTime = high_resolution_clock::now();
    std::cout << "sendBuffer() function took: " << duration_cast<microseconds>(endTime - startTime).count() / 1000.0f << "ms\n" << std::flush;
#endif
}

std::shared_ptr<dai::ImgFrame> Camera::makeFrame(CameraSensor::FrameCallbackArgs args,
                                                 std::optional<dai::Timestamp> tsSystem,
                                                 dai::ImgFrame::Type type,
                                                 bool interleaved,
                                                 dai::CameraBoardSocket socket,
                                                 ImgTransformation transformation) {
    std::shared_ptr<dai::ImgFrame> frame = std::make_shared<dai::ImgFrame>();
    frame->setWidth(args.width);
    frame->setHeight(args.height);
    // frame->getData().resize(size);
    frame->setType(type);
    frame->setInstanceNum(static_cast<int>(socket));
    frame->cam = args.cam;
    // Set stride
    frame->fb.stride = args.wstride;
    // set planes
    frame->fb.p1Offset = args.wstride * args.hstride * 0;
    frame->fb.p2Offset = args.wstride * args.hstride * 1;
    frame->fb.p3Offset = args.wstride * args.hstride * 2;

    using namespace std::chrono;
    auto offset = Timesync::getDefaultSynchronizedOffset();
    auto tpHost = time_point<steady_clock, steady_clock::duration>{seconds(args.ts.sec + offset.tv_sec) + nanoseconds(args.ts.nsec + offset.tv_nsec)};
    auto tpDevice = time_point<steady_clock, steady_clock::duration>{seconds(args.ts.sec) + nanoseconds(args.ts.nsec)};

    if(tsSystem.has_value()) {
        auto tpSystem = time_point<system_clock, system_clock::duration>{seconds(tsSystem->sec) + nanoseconds(tsSystem->nsec)};
        frame->setTimestampSystem(tpSystem);
    }
    frame->setTimestamp(tpHost);
    frame->setTimestampDevice(tpDevice);
    frame->setSequenceNum(args.sequenceNumber);
    frame->setWidth(args.width);
    frame->setHeight(args.height);
    frame->setSourceSize(calibrationWidth, calibrationHeight);
    transformation = prepareMultiDeviceTransformation(std::move(transformation), deviceId, multiDeviceCalibration.get(), warnedForMultiDeviceMismatch, logger);
    frame->transformation = std::move(transformation);

    if(!frame->validateTransformations() && !warnedForMissingCalibration) {
        warnedForMissingCalibration = true;
        logger->warn(
            "Transformation does not match the frame metadata, please report to developers. Transformation size: {}x{}, frame size: {}x{}; transformation "
            "source size: {}x{}, frame source size: {}x{}",
            frame->transformation.getSize().first,
            frame->transformation.getSize().second,
            args.width,
            args.height,
            frame->transformation.getSourceSize().first,
            frame->transformation.getSourceSize().second,
            calibrationWidth,
            calibrationHeight);
    }

    if(interleaved) {
        throw std::runtime_error("Interleaved mode not supported yet");
        // // TODO switch to ImageManip instead, with a CSC only path, and profile
        // frame->data = std::make_shared<VpuDataMemory>(size);
        // toPlanar(frame->data->getData(), data->getData().data(), width, height);
    } else {
        // zerocopy
        frame->data = args.data;
    }

    return frame;
}

void Camera::buildStage1() {
    mockIspEnabled = mockIsp.isConnected();
    if(mockIspEnabled && (properties.mockIspWidth <= 0 || properties.mockIspHeight <= 0)) {
        throw std::runtime_error("Mock ISP enabled, but mockIspWidth and mockIspHeight not set");
    }
    logger->debug("Camera: mock isp enabled {}", mockIspEnabled);

    // Register the camera to the static vector
    if(std::find(camerasInThePipeline.begin(), camerasInThePipeline.end(), properties.boardSocket) != camerasInThePipeline.end()) {
        throw std::runtime_error("Camera on socket " + std::to_string(static_cast<int>(properties.boardSocket)) + " already exists in the pipeline");
    }
    camerasInThePipeline.push_back(properties.boardSocket);  // No need for lock, since buildStages are called sequentially
    const size_t propertiesSize = properties.outputRequests.size();
    const bool onlyIspIndices = propertiesSize > 0 && getIspIndices().size() == propertiesSize;
    const size_t maxFpsIndex = getMaxFpsIndex();
    if(onlyIspIndices) {
        properties.outputRequests[maxFpsIndex].ispOutput = false;
    }
    sensorConfig = decideSensorConfig();
    if(onlyIspIndices) {
        DAI_CHECK_IN(sensorConfig);
        properties.outputRequests[maxFpsIndex].size.value =
            std::make_pair<uint32_t, uint32_t>(static_cast<uint32_t>((*sensorConfig).mode.width), static_cast<uint32_t>((*sensorConfig).mode.height));
    }
    if(!mockIspEnabled) {
        DAI_CHECK_IN(sensorConfig);
        CameraSingleton::getInstance().registerSensorConfig(properties.boardSocket,
                                                            properties.initialControl.frameSyncMode,
                                                            CameraSingleton::SensorConfigInfo{
                                                                .modeIndex = (*sensorConfig).modeIndex,
                                                                .mode = (*sensorConfig).mode,
                                                                .fps = (*sensorConfig).fps,
                                                            });
    }
    camSyncType = ImgFrame::Fsync::NONE;

    // Set the calibration data
    auto pipeline = getParentPipeline();
    if(pipeline.isCalibrationDataAvailable()) {
        auto calib = pipeline.getCalibrationData();
        try {
            sourceHFovDegree = calib.getFov(properties.boardSocket, false);
        } catch(const std::exception& e) {
            logger->info("No FOV data found for camera on socket.");
        }
        logger->info("Setting FOV of the camera to {} degrees", sourceHFovDegree);
    } else {
        logger->info("No calibration data found, Camera won't add FOVs frames!");
    }
}

size_t Camera::findMaxResolutionIndex() const {
    size_t bestIdx = 0;
    uint64_t maxArea = 0;

    size_t i = 0;
    for(const auto& spec : properties.outputRequests) {
        if(spec.ispOutput || !spec.size.value) {
            ++i;
            continue;
        }

        const auto* size = std::get_if<std::pair<uint32_t, uint32_t>>(&(*spec.size.value));
        if(size == nullptr) {
            ++i;
            continue;
        }

        const uint64_t area = static_cast<uint64_t>(size->first) * static_cast<uint64_t>(size->second);
        if(area > maxArea) {
            maxArea = area;
            bestIdx = i;
        }
        ++i;
    }
    return bestIdx;
}

std::vector<size_t> Camera::getIspIndices() const {
    std::vector<size_t> ispIndices;
    size_t i = 0;
    for(const auto& output : properties.outputRequests) {
        if(output.ispOutput) {
            ispIndices.push_back(i);
        }
        ++i;
    }
    return ispIndices;
}

void Camera::setupResizers(const SensorConfig& config) {
    std::vector<std::tuple<uint32_t, uint32_t, float, size_t>> outSizes;
    size_t specIndex = 0;
    auto cameraFeautures = CameraDiscovery::getInstance().getCameraFeatures(properties.cameraName, properties.boardSocket);
    auto sensorName = cameraFeautures.sensorName;
    const auto ispScalers = CameraDiscovery::getInstance().getAvailableISPScalers(properties.boardSocket);
    std::unique_ptr<CalibrationHandler> calib = nullptr;
    uint32_t eepromId = 0;
    {
        auto pipeline = getParentPipeline();
        bool transformationSet = false;
        if(pipeline.isCalibrationDataAvailable()) {
            try {
                calib = std::make_unique<CalibrationHandler>(pipeline.getCalibrationData());
                eepromId = pipeline.getEepromId();
                baseTransformation = getCameraTransformation(*calib, properties.boardSocket, properties.cameraName, logger);
                calibrationWidth = baseTransformation.getSourceSize().first;
                calibrationHeight = baseTransformation.getSourceSize().second;
                transformationSet = true;
            } catch(const std::exception& e) {
                warnedForMissingCalibration = true;
                logMissingCameraIntrinsics(calib ? calib->getEepromData().productName : "", properties.boardSocket, logger);
            }
        } else {
            warnedForMissingCalibration = true;
            logger->warn("No calibration data found, ImgTransformations will not be valid!");
        }
        if(!transformationSet) {
            auto cameraFeatures = CameraDiscovery::getInstance().getCameraFeatures(properties.cameraName, properties.boardSocket);
            baseTransformation = ImgTransformation(cameraFeatures.width, cameraFeatures.height);
            calibrationWidth = cameraFeatures.width;
            calibrationHeight = cameraFeatures.height;
        }
    }
    if(!isFullFOV(config.mode)) {
        auto fov = config.mode.fov;
        if(fov.isNormalized()) {
            fov = fov.denormalize(cameraFeautures.width, cameraFeautures.height);
        }
        baseTransformation.addCrop(static_cast<int>(std::round(fov.x)),
                                   static_cast<int>(std::round(fov.y)),
                                   static_cast<int>(std::round(fov.width)),
                                   static_cast<int>(std::round(fov.height)));
    }
    for(const auto spec : properties.outputRequests) {
        std::optional<float> parsedFps;
        if(spec.fps.value) {
            if(const auto* fps = std::get_if<float>(&(*spec.fps.value))) {
                parsedFps = *fps;
            } else if(const auto* fps = std::get_if<std::pair<float, float>>(&(*spec.fps.value))) {
                // TODO(jakgra) handle ranges better (merge multiple requests into same fps)?
                parsedFps = std::get<0>(*fps);
            } else if(const auto* fps = std::get_if<std::vector<float>>(&(*spec.fps.value))) {
                DAI_CHECK(fps->size() > 0, "When passing a vector to ImgFrameCapability->fps, please pass a non empty vector!");
                parsedFps = (*fps)[0];
            } else {
                DAI_CHECK_IN(false);
            }
        } else {
            DAI_CHECK_IN(sensorConfig);
            parsedFps = (*sensorConfig).fps;
        }
        DAI_CHECK_IN(parsedFps);
        auto wantedFps = *parsedFps;
        DAI_CHECK(wantedFps > 0, "Please don't set minimum ImgFrameCapability->fps to 0 but to 1");
        auto& cameraOutput = currentOutputs.emplace_back();
        cameraOutput.index = specIndex;
        cameraOutput.ispOutput = spec.ispOutput;
        bool found = false;
        uint32_t width = 0;
        uint32_t height = 0;
        if(!spec.ispOutput) {
            if(spec.size.value) {
                if(const auto* size = std::get_if<std::pair<uint32_t, uint32_t>>(&(*spec.size.value))) {
                    width = size->first;
                    height = size->second;
                } else {
                    // TODO(jakgra) add support for other logic here
                    DAI_CHECK_IN(false);
                }
            } else {
                // TODO(jakgra) add support for other logic here
                DAI_CHECK_IN(false);
            }
        } else {
            width = static_cast<uint32_t>(config.mode.width);
            height = static_cast<uint32_t>(config.mode.height);
        }
        size_t resolutionWidth = 0;
        size_t resolutionHeight = 0;
        float sensorFps = config.fps;
        if(!mockIspEnabled) {
            const auto& resolutions = ispScalers;
            DAI_CHECK_IN(sensorConfig);
            const auto sensorMode = (*sensorConfig).mode;
            if(sensorMode.hfr) {
                for(const auto& res : resolutions) {
                    if(res.first == sensorMode.width && res.second == sensorMode.height) {
                        resolutionWidth = res.first;
                        resolutionHeight = res.second;
                        found = true;
                        break;
                    }
                }
                DAI_CHECK_V(found, "Couldn't find ISP scaler resolution {}x{} for HFR sensor mode", sensorMode.width, sensorMode.height);
            } else {
                uint64_t bestArea = 0;
                for(const auto& res : resolutions) {
                    logger->debug("COMP: ({}/{} == {}/{}) = {}",
                                  res.first,
                                  res.second,
                                  sensorMode.width,
                                  sensorMode.height,
                                  static_cast<uint64_t>(res.first) * static_cast<uint64_t>(sensorMode.height)
                                      == static_cast<uint64_t>(res.second) * static_cast<uint64_t>(sensorMode.width));
                    if(static_cast<uint64_t>(res.first) * static_cast<uint64_t>(sensorMode.height)
                       != static_cast<uint64_t>(res.second) * static_cast<uint64_t>(sensorMode.width)) {
                        continue;
                    }
                    if(res.first >= width && res.second >= height) {
                        const uint64_t area = static_cast<uint64_t>(res.first) * static_cast<uint64_t>(res.second);
                        if(!found || area < bestArea) {
                            resolutionWidth = res.first;
                            resolutionHeight = res.second;
                            bestArea = area;
                        }
                        found = true;
                    }
                }
                DAI_CHECK_V(found, "Couldn't find ISP scaler resolution for sensor mode {}x{}", sensorMode.width, sensorMode.height);
            }
            if(found) {
                logger->info("Chose ISP scaler resolution: {}x{}", resolutionWidth, resolutionHeight);
                if(!cameraOutput.ispOutput) {
                    outSizes.emplace_back(static_cast<uint32_t>(resolutionWidth), static_cast<uint32_t>(resolutionHeight), wantedFps, specIndex);
                }
            }

            cameraOutput.eepromId = eepromId;
            cameraOutput.resolutionWidth = resolutionWidth;
            cameraOutput.resolutionHeight = resolutionHeight;
            cameraOutput.transformation = adjustTransformationSize(baseTransformation, resolutionWidth, resolutionHeight, warnedForMissingCalibration, logger);
            if((cameraOutput.transformation.getSize().first != resolutionWidth || cameraOutput.transformation.getSize().second != resolutionHeight)
               && !warnedForMissingCalibration) {
                warnedForMissingCalibration = true;
                logger->warn("Frame size does not equal transformation size, please report to developers. Frame size: {}x{}, transformation size: {}x{}",
                             cameraOutput.transformation.getSize().first,
                             cameraOutput.transformation.getSize().second,
                             resolutionWidth,
                             resolutionHeight);
            }
        } else {
            resolutionWidth = properties.mockIspWidth;
            resolutionHeight = properties.mockIspHeight;
            if(properties.mockIspFps > 0) {
                sensorFps = properties.mockIspFps;
            }
            found = true;
        }
        uint32_t maxWidth = 0;
        uint32_t maxHeight = 0;
        uint64_t maxArea = 0;
        for(const auto& res : ispScalers) {
            const uint64_t area = static_cast<uint64_t>(res.first) * static_cast<uint64_t>(res.second);
            if(area > maxArea) {
                maxArea = area;
                maxWidth = res.first;
                maxHeight = res.second;
            }
        }
        DAI_CHECK_V(found,
                    "Sensor {} doesn't support requested resolution: {}x{} . Maximum supported resolution for this sensor is {}x{}",
                    sensorName,
                    width,
                    height,
                    maxWidth,
                    maxHeight);
        if(!spec.ispOutput) {
            if(resolutionWidth != width || resolutionHeight != height || (spec.enableUndistortion && *spec.enableUndistortion)) {
                DAI_CHECK_V(width % 2 == 0 && height % 2 == 0, "We only support even numbers for resolution sizes. Got {}x{}.", width, height);
                ImageManipConfig config;
                // Undistortion alone must preserve the requested dimensions too;
                // automatic ImageManip sizing is capped at 4000x3000.
                config.setOutputSize(width, height, ImageManipConfig::ResizeMode::STRETCH);
                if(resolutionWidth != width || resolutionHeight != height) {
                    switch(spec.resizeMode) {
                        using E = ImgResizeMode;
                        case E::CROP: {
                            config.setOutputSize(width, height, ImageManipConfig::ResizeMode::CENTER_CROP);
                            break;
                        }
                        case E::STRETCH:
                            config.setOutputSize(width, height, ImageManipConfig::ResizeMode::STRETCH);
                            break;
                        case E::LETTERBOX:
                            config.setOutputSize(width, height, ImageManipConfig::ResizeMode::LETTERBOX);
                            break;
                        default:
                            DAI_CHECK_IN(false);
                            break;
                    }
                }
                config.setUndistort(spec.enableUndistortion && *spec.enableUndistortion);
                cameraOutput.resizer = std::make_unique<ManipResizer>(config, spec.alphaScaling, dynamicOutputs[std::to_string(specIndex)]);
            }
        }
        if(std::abs(wantedFps - sensorFps) > 0.0001) {
            if(cameraOutput.resizer == nullptr) {
                cameraOutput.fpsRegulator = std::make_unique<FpsRegulator>(sensorFps, wantedFps, &dynamicOutputs[std::to_string(specIndex)], logger);
            } else {
                cameraOutput.fpsRegulator = std::make_unique<FpsRegulator>(sensorFps, wantedFps, &cameraOutput.resizer->inQ, logger);
            }
        }
        ++specIndex;
    }
    if(!mockIspEnabled) {
        // Some outputs can be served by routing the main ISP frame and therefore don't need their own HAL stream.
        cameraSensor->setupDynamicOutputs(outSizes);
    }
    maxResolutionIndex = findMaxResolutionIndex();
    return;
}

void Camera::buildStage2() {
    logger->trace("started camera\n");
    selectedSensorType = dai::CameraSensorType::AUTO;
    if(!mockIspEnabled) {
        int numPools = properties.numFramesPoolIsp;

        auto boardSocket = properties.boardSocket;
        // Image orientation, default to DTS, allow host to override
        auto orientation = properties.imageOrientation;
        // if(orientation == CameraImageOrientation::AUTO) orientation = cam.camera.orientation;

        // Query camera id
        auto cam = CameraDiscovery::getInstance().getCameraDescription(properties.cameraName, properties.boardSocket);
        auto features = CameraDiscovery::getInstance().getCameraFeatures(properties.cameraName, properties.boardSocket);

        // Helper function to format supported types for logging
        // Outputs a string like "COLOR, MONO, TOF, THERMAL"
        auto formatSupportedTypes = [](const std::vector<dai::CameraSensorType>& supportedTypes) {
            return std::accumulate(
                std::next(supportedTypes.begin()), supportedTypes.end(), dai::toString(supportedTypes.front()), [](std::string a, dai::CameraSensorType b) {
                    return a + ", " + dai::toString(b);
                });
        };

        selectedSensorType = properties.sensorType;
        constexpr bool performSensorTypeValidation = true;

        // In case of any non-AUTO sensor type, check if it is supported by
        // the sensor
        if(performSensorTypeValidation && selectedSensorType != dai::CameraSensorType::AUTO
           && std::find(features.supportedTypes.begin(), features.supportedTypes.end(), selectedSensorType) == features.supportedTypes.end()) {
            throw std::invalid_argument(
                fmt::format("Desired sensor type {} is not supported by the "
                            "sensor on socket {}. Officially supported sensor types: [{}]",
                            dai::toString(selectedSensorType),
                            dai::toString(boardSocket),
                            formatSupportedTypes(features.supportedTypes)));
        }

        // Now, the sensor type is either AUTO or something that we already
        // know is supported by the underlying sensor
        if(selectedSensorType == dai::CameraSensorType::AUTO) {
            if(features.supportedTypes.size() >= 1) {
                selectedSensorType = features.supportedTypes[0];
            } else {
                throw std::invalid_argument(fmt::format("No supported sensor types found for camera on socket {}", dai::toString(boardSocket)));
            }
            logger->debug(
                "Sensor type {} selected automatically for "
                "camera on socket {}. Officially supported sensor types: [{}]",
                dai::toString(selectedSensorType),
                dai::toString(boardSocket),
                formatSupportedTypes(features.supportedTypes));
        } else {
            logger->debug("Sensor type {} selected by user for camera on socket {}. Officially supported sensor types: [{}]",
                          dai::toString(selectedSensorType),
                          dai::toString(boardSocket),
                          formatSupportedTypes(features.supportedTypes));
        }
        if(cam.id == -1) {
            throw std::invalid_argument(fmt::format("Cannot find camera given socket: {}", (int)properties.boardSocket));
            // throw std::invalid_argument(fmt::format("Cannot find camera given socket: {} and name: '{}'", properties.boardSocket, properties.cameraName));
            // throw std::invalid_argument(fmt::format("Cannot find camera given name: '{}'", properties.cameraName));
        }

        if(properties.initialControl.frameSyncMode != dai::CameraControl::FrameSyncMode::TIME_PTP) {
            const auto requestedFsyncEnable = CameraSingleton::getInstance().isFsyncEnabled(properties.boardSocket);
            if(FsyncController::getInstance().isOk()) {
                // On old OS version the STM firmware only supports up to 90.0f fps
                if(FsyncController::getInstance().supportsExternalFrameSyncMode()) {
                    camSyncType = requestedFsyncEnable ? ImgFrame::Fsync::INPUT : ImgFrame::Fsync::NONE;
                } else {
                    DAI_CHECK_IN(sensorConfig);
                    if(sensorConfig->fps <= 60.0) {
                        camSyncType = requestedFsyncEnable ? ImgFrame::Fsync::INPUT : ImgFrame::Fsync::NONE;
                    } else {
                        if(properties.initialControl.frameSyncMode == dai::CameraControl::FrameSyncMode::INPUT) {
                            throw std::runtime_error("Frame sync INPUT mode requested for fps > 60 on old OS version. Please update your OS.");
                        }
                        camSyncType = ImgFrame::Fsync::NONE;
                    }
                }
            } else {
                if(requestedFsyncEnable && properties.initialControl.frameSyncMode == dai::CameraControl::FrameSyncMode::INPUT) {
                    throw std::runtime_error("Frame sync INPUT mode requested, but the fsync controller is not available.");
                }
                if(requestedFsyncEnable) {
                    logger->warn("Fsync was requested, but the fsync controller is not available. Disabling fsync for camera socket {}.",
                                 toString(properties.boardSocket));
                } else {
                    logger->warn("Fsync not OK, not using it.");
                }
                camSyncType = ImgFrame::Fsync::NONE;
            }
        } else {
            camSyncType = ImgFrame::Fsync::PTP;
        }

        // Sanity check
        if(FsyncController::getInstance().supportsExternalFrameSyncMode() && FsyncController::getInstance().isOk()
           && FsyncController::getInstance().getExternalFrameSyncRole() == dai::ExternalFrameSyncRole::SLAVE && properties.fps != -1
           && camSyncType == ImgFrame::Fsync::INPUT) {
            throw std::runtime_error("Cannot override fps while using external FSYNC slave mode");
        }

        if(properties.initialControl.frameSyncMode == dai::CameraControl::FrameSyncMode::TIME_PTP) {
            if(dai::gate::getOSVersion() < dai::Version(1, 25, 3)) {
                logger->error("OS version 1.25.3 or higher required to enable PTP feature");
                throw std::runtime_error("OS version 1.25.3 or higher required to enable PTP feature");
            }
        }

        if(camSyncType == ImgFrame::Fsync::INPUT) {
            DAI_CHECK_IN(sensorConfig);

            int fsyncId = -1;
            if(0 == static_cast<int>(properties.boardSocket)) {
                fsyncId = 2;  // FSYNC PIN index 2 == PA1
            } else if(1 == static_cast<int>(properties.boardSocket) || 2 == static_cast<int>(properties.boardSocket)) {
                fsyncId = 4;  // FSYNC PIN index 4 == PA8
            }
            auto ret = dai::DeviceTreeDiscovery::getInstance().getFsyncPin(static_cast<uint32_t>(properties.boardSocket));
            if(ret.has_value()) {
                fsyncId = static_cast<int>(*ret);
            } else {
                if(!dai::gate::FsyncController::getInstance().supportsExternalFrameSyncMode()) {
                    logger->info("Could not get the FSYNC pin for socket " + toString(properties.boardSocket)
                                 + ". OS probably does not support dynamic fsync pin assignment.");
                    logger->info("Falling back to hardcoded FSYNC pin.");
                } else {
                    logger->error("Could not get the FSYNC pin for socket " + toString(properties.boardSocket)
                                  + ". Check camera device tree and camx camera configuration.");
                    logger->warn("Falling back to hardcoded FSYNC pin.");
                }
            }

            if(fsyncId == -1) {
                throw std::runtime_error("Could not determine FSYNC pin for camera socket " + toString(properties.boardSocket) + ".");
            }

            const float wantedStmFps = (*sensorConfig).fps + ((getOSVersion() < dai::Version(0, 23, 0)) ? 0.02f : 0.0f);
            (*sensorConfig).fps = wantedStmFps;
            logger->trace("Camera socket: {}, setting explicit fsync id: {}, fps: {}\n", static_cast<int>(properties.boardSocket), fsyncId, wantedStmFps);
#ifndef FSYNC_DONT_WRITE
            FsyncController::getInstance().setFps(fsyncId, wantedStmFps);
            FsyncController::getInstance().addFsyncConsumer(properties.boardSocket);
#endif

            const auto fsyncRole = FsyncController::getInstance().getExternalFrameSyncRole();
            if(fsyncRole == dai::ExternalFrameSyncRole::MASTER) {
                int fsyncExtOut = 11;
                auto extOutPin = dai::DeviceTreeDiscovery::getInstance().getExternalFrameSyncPin();
                if(extOutPin.has_value()) {
                    fsyncExtOut = static_cast<int>(*extOutPin);
                } else {
                    if(!dai::gate::FsyncController::getInstance().supportsExternalFrameSyncMode()) {
                        logger->info("Could not get the external FSYNC pin. OS probably does not support dynamic fsync pin assignment.");
                        logger->info("Falling back to hardcoded external FSYNC pin.");
                    } else {
                        logger->error("Could not get the external FSYNC pin. Check STM device tree properties.");
                        logger->warn("Falling back to hardcoded external FSYNC pin.");
                    }
                }
#ifndef FSYNC_DONT_WRITE
                FsyncController::getInstance().setFps(fsyncExtOut, wantedStmFps);
                FsyncController::getInstance().registerChannelForExposureUpdates(fsyncExtOut);
#endif
                logger->info("This device is a FSync master!");
            } else if(fsyncRole == dai::ExternalFrameSyncRole::SLAVE) {
                logger->info("This device is a FSync slave!");
            } else {
                throw std::runtime_error("Illegal fsync role detected!");
            }
        }

        // Create camera
        cameraSensor = std::make_unique<CameraSensor>(
            cam,
            (*sensorConfig).modeIndex,
            (*sensorConfig).fps,
            fromImgFrameFsync(camSyncType),
            orientation,
            properties,
            getParentPipeline().getEepromData(),
            [this](CameraSensor::FrameCallbackArgs args) { sendBuffer(args); },
            logger);
        numRaw = raw.getConnections().size();
        if(numRaw > 0) {
            cameraSensor->setupRawOutput();
        }

        // Apply a custom camera tuning blob if one was set on the pipeline
        if(auto deviceProps = getParentPipeline().getDefaultDeviceProperties()) {
            // Match RVC2 semantics: a global tuning blob applies to all cameras,
            // per-socket blobs are only used when no global blob is set.
            std::string tuningUri;
            if(deviceProps->cameraTuningBlobSize.value_or(0) > 0 && !deviceProps->cameraTuningBlobUri.empty()) {
                tuningUri = deviceProps->cameraTuningBlobUri;
            } else {
                auto sizeIt = deviceProps->cameraSocketTuningBlobSize.find(boardSocket);
                auto uriIt = deviceProps->cameraSocketTuningBlobUri.find(boardSocket);
                if(sizeIt != deviceProps->cameraSocketTuningBlobSize.end() && sizeIt->second > 0 && uriIt != deviceProps->cameraSocketTuningBlobUri.end()
                   && !uriIt->second.empty()) {
                    tuningUri = uriIt->second;
                }
            }
            if(!tuningUri.empty()) {
                // URI has the form "asset:<key>" - assets are stored under their key in the pipeline asset manager
                const auto scheme = tuningUri.find(':');
                const auto assetKey = scheme != std::string::npos ? tuningUri.substr(scheme + 1) : tuningUri;
                auto asset = getParentPipeline().getAssetManager().get(assetKey);
                DAI_CHECK_V(asset != nullptr, "Camera tuning blob asset not found for URI {}", tuningUri);
                const auto& blob = asset->getData();
                const auto tuningPath = fmt::format("/tmp/depthai_camera_tuning_{}.bin", static_cast<int>(boardSocket));
                {
                    std::ofstream tuningFile(tuningPath, std::ios::binary | std::ios::trunc);
                    DAI_CHECK_V(tuningFile.is_open(), "Could not open {} for writing the camera tuning blob", tuningPath);
                    tuningFile.write(reinterpret_cast<const char*>(blob.data()), static_cast<std::streamsize>(blob.size()));
                    DAI_CHECK_V(tuningFile.good(), "Failed to write the camera tuning blob to {}", tuningPath);
                }
                logger->info("Camera socket {}: applying custom tuning blob from {} ({} bytes)", dai::toString(boardSocket), tuningUri, blob.size());
                cameraSensor->setTuningOverride(tuningPath);
            }
        }
    }

    DAI_CHECK_IN(sensorConfig);
    setupResizers(*sensorConfig);

    // validateSettings(); // TODO(jakgra) reenable if needed
}

void Camera::buildStage3() {
    if(!mockIspEnabled) {
        if(camSyncType == ImgFrame::Fsync::INPUT) {
            setDefaultExposureMaster();
        }

        DAI_CHECK_IN(cameraSensor);
        cameraSensor->build(selectedSensorType);
    }

    // Start FSync only after all camera nodes finished buildStage2.
    if(camSyncType == ImgFrame::Fsync::INPUT && !mockIspEnabled && FsyncController::getInstance().isOk()) {
        FsyncController::getInstance().start();
    }
}

void Camera::setDefaultExposureMaster() {
    using Model = dai::gate::DeviceModel::Model;
    Model const model = dai::gate::DeviceModel::get();
    auto& fsync = FsyncController::getInstance();

    // stobe enabled and exposure master is not registered fsync consumer
    // This is to check if user specified correct socket
    if(fsync.getExternalStrobeEnable() && fsync.getExternalStrobeExposureMasterCamera().has_value()
       && !fsync.isFsyncConsumer(fsync.getExternalStrobeExposureMasterCamera().value())) {
        throw std::runtime_error("Specified exposure master either not in pipeline or not using fsync");
    }

    // Try to find default exposure master candidates
    auto itr = exposureMasterCameraPriority.find(model);
    if(itr == exposureMasterCameraPriority.cend()) {
        // If strobe is enabled and no exposure master is defined by the user, we throw
        if(fsync.getExternalStrobeEnable() && !fsync.getExternalStrobeExposureMasterCamera().has_value()) {
            throw std::runtime_error("No default exposure master found. You must specify which camera to use to control exposure");
        }
        // user specified the master
        return;
    }

    auto const candidates = itr->second;
    for(auto const& camCandidate : candidates) {
        for(auto& cam : camerasInThePipeline) {
            if(cam == camCandidate && fsync.isFsyncConsumer(cam)) {
                if(cam == properties.boardSocket) {
                    // We assume that led exposure will always be set automatically,
                    // because there is no interface just yet, to allow user to set it
                    // This means that all devices with IR DOT and FLOOD must be defined in exposureMasterCameraPriority
                    fsync.setLedExposureMasterCamera(cam);
                    // do not override master if set by user
                    if(!fsync.getExternalStrobeExposureMasterCamera().has_value()) {
                        fsync.setExternalStrobeExposureMasterCamera(cam);
                    }
                    return;
                }
                return;
            }
        }
    }
}

void* Camera::sensorErrorCbAdd(SensorErrorStateCallback cb) {
    if(!cameraSensor) {
        return nullptr;
    }
    return cameraSensor->sensorErrorCbAdd(std::move(cb));
}

void Camera::sensorErrorCbRemove(void* opaqueHandle) {
    if(cameraSensor) {
        cameraSensor->sensorErrorCbRemove(opaqueHandle);
    }
}

void Camera::validateSettings() {
    /*
      auto sensorConfig = getSensorConfig();
      // TODO(Jakob) - this is not robust, we should rather pass the limitations over from lower stack
      if(sensorConfig.size.width == 8000 || sensorConfig.size.height == 6000) {
          // A few limitations apply when a camera is running at 8k
          // 1. Only one additional camera can be run at the same time
          // 2. Only one output can be connected at a time, even if it's the raw output

          // Use only warnings, since this might get fixed with an OS update in the future

          // Check if there are any other cameras running
          if(camerasInThePipeline.size() > 2) {
              logger->warn(
                  "When running a camera at 8000x6000 resolution, only one additional camera can be run at the same time, while {} are currently running. Please
      " "consider running the camera at a lower resolution.", camerasInThePipeline.size());
          }

          auto numOutputs = numRaw + dynamicOutputs.size();
          if(numOutputs > 1) {
              logger->warn(
                  "When running a camera at 8000x6000 resolution, only one output can be connected at a time. Consider running the camera at a lower resolution
      " "or only using one output.");
          }
      }

      if(numRaw) {
          if(dynamicOutputs.size() > 1) {
              logger->warn(
                  "When running a camera with raw output, only one additional output can be connected at a time, while {} are currently connected. Please "
                  "consider running the camera with only one output.",
                  dynamicOutputs.size());
          }
      }
      */
}

void Camera::setToFExposure(uint32_t exposureTimeUs, uint32_t sensitivityIso) {
    std::lock_guard lock(tofExposureMutex);
    if(cameraSensor && isRunning() && tofExposureEnabled && !tofExposureLocked) {
        CameraControl control;
        if(tofExposureLimitUs) exposureTimeUs = std::min(exposureTimeUs, *tofExposureLimitUs);
        cameraSensor->applyCameraControl(control.setManualExposure(exposureTimeUs, sensitivityIso));
    }
}

void Camera::run() {
    logger->info("{} running on {}.", this->getName(), runOnHost() ? "host" : "device");
    const auto rawConnections = raw.getConnections();
    const bool tofConnected = std::any_of(rawConnections.begin(), rawConnections.end(), [](const auto& connection) {
        return std::dynamic_pointer_cast<node::ToFBase>(connection.inputNode.lock()) != nullptr;
    });
    const auto warnIfToFAutoExposureUnavailable = [&]() {
        if(selectedSensorType == CameraSensorType::TOF && !tofConnected) {
            logger->warn("ToF camera auto exposure requires the raw output to be connected to a ToF node");
        }
    };
    tofExposureEnabled = !initialControl.getCommand(CameraControl::Command::AE_MANUAL) || initialControl.getCommand(CameraControl::Command::AE_AUTO);
    tofExposureLocked = initialControl.getCommand(CameraControl::Command::AE_LOCK) && initialControl.aeLockMode;
    if(tofExposureEnabled) warnIfToFAutoExposureUnavailable();
    if(initialControl.getCommand(CameraControl::Command::AE_TARGET_FPS_RANGE) && initialControl.aeMaxExposureTimeUs > 0) {
        tofExposureLimitUs = initialControl.aeMaxExposureTimeUs;
    }
    const auto serialNumber = getSerialNumber();
    if(!serialNumber.empty() && serialNumber != "xlink-server") {
        setDeviceId(serialNumber);
    }
    // PipelineBuilder installed and validated the graph; runtime updates arrive through the store.
    refreshMultiDeviceCalibration();

    // There is a race condition and we can't initialize FastCV before initializing SNPE (we won't be reporting to qualcomm (for now))
    // manip.Init() needs to be in the run function so SNPE has time to initialize before FastCV (called via manip.Init())
    for(auto& cameraOutput : currentOutputs) {
        if(cameraOutput.resizer != nullptr) {
            std::optional<ImgFrame::Type> type = properties.outputRequests[cameraOutput.index].type;
            cameraOutput.resizer->runThreadedResizer(
                properties.numFramesPoolVideo,
                [type](std::shared_ptr<ImgFrame> frame) {
                    if(type && *type != frame->getType()) {
                        return convertToFormat(*type, frame);
                    } else {
                        return frame;
                    }
                },
                mockIspEnabled,
                logger);
        }
        if(cameraOutput.fpsRegulator != nullptr) {
            cameraOutput.fpsRegulator->run();
        }
    }

    // Start the pipeline, run in separate thread
    if(!mockIspEnabled) {
        cameraSensor->run();
    }
    try {
        while(isRunning()) {
            if(!mockIspEnabled) {
                // Time out every 200ms to check if isRunning() is true and if cameraSensor->run() thread is still alive or has failed
                std::shared_ptr<CameraControl> pControl;
                bool timedOut{false};
                pControl = inputControl.get<CameraControl>(std::chrono::milliseconds(200), timedOut);
                if(!timedOut) {
                    auto control = *pControl;  // get raw
                    std::lock_guard lock(tofExposureMutex);
                    if(control.getCommand(CameraControl::Command::AE_MANUAL)) tofExposureEnabled = false;
                    if(control.getCommand(CameraControl::Command::AE_AUTO)) {
                        tofExposureEnabled = true;
                        warnIfToFAutoExposureUnavailable();
                    }
                    if(control.getCommand(CameraControl::Command::AE_LOCK)) tofExposureLocked = control.aeLockMode;
                    if(control.getCommand(CameraControl::Command::AE_TARGET_FPS_RANGE)) {
                        tofExposureLimitUs = control.aeMaxExposureTimeUs > 0 ? std::make_optional(control.aeMaxExposureTimeUs) : std::nullopt;
                    }
                    cameraSensor->applyCameraControl(control);
                }
            } else {
                this->pipelineEventDispatcher->pingMainLoopEvent();
                auto tAbsoluteBeginning = std::chrono::steady_clock::now();
                std::shared_ptr<dai::ImgFrame> frame = nullptr;
                {
                    auto blockEvent = this->inputBlockEvent();
                    frame = mockIsp.get<dai::ImgFrame>();
                }
                auto tGotInput = std::chrono::steady_clock::now();
                DAI_CHECK(frame != nullptr, "Received message is not ImgFrame");
                DAI_CHECK(frame->getType() == dai::ImgFrame::Type::NV12, "mockIsp only supports NV12 input frames");
                auto tProcessed = std::chrono::steady_clock::now();
                for(auto i = 0U; i < currentOutputs.size(); ++i) {
                    const auto& req = properties.outputRequests[i];
                    {
                        auto blockEvent = this->outputBlockEvent();
                        if(currentOutputs[i].fpsRegulator != nullptr) {
                            currentOutputs[i].fpsRegulator->inQueue.send(frame);
                        } else if(currentOutputs[i].resizer != nullptr) {
                            currentOutputs[i].resizer->inQ.send(frame);
                        } else {
                            dynamicOutputs[std::to_string(i)].send(toCorrectFormat(req, frame));
                        }
                    }
                }
                auto tAbsoluteEnd = std::chrono::steady_clock::now();
                dai::gate::logTiming(logger, this->getName(), tAbsoluteBeginning, tGotInput, tProcessed, tAbsoluteEnd);
                std::this_thread::yield();
            }
        }
        for(auto& cameraOutput : currentOutputs) {
            if(cameraOutput.resizer != nullptr) {
                cameraOutput.resizer->stop();
            }
            if(cameraOutput.fpsRegulator != nullptr) {
                cameraOutput.fpsRegulator->stop();
            }
        }
        for(auto& cameraOutput : currentOutputs) {
            if(cameraOutput.resizer != nullptr) {
                cameraOutput.resizer->join();
            }
        }
    } catch(...) {
        for(auto& cameraOutput : currentOutputs) {
            if(cameraOutput.resizer != nullptr) {
                cameraOutput.resizer->stop();
            }
            if(cameraOutput.fpsRegulator != nullptr) {
                cameraOutput.fpsRegulator->stop();
            }
        }
        for(auto& cameraOutput : currentOutputs) {
            if(cameraOutput.resizer != nullptr) {
                cameraOutput.resizer->join();
            }
        }
        throw;
    }
}

void Camera::stop() {
    for(auto& cameraOutput : currentOutputs) {
        if(cameraOutput.resizer != nullptr) {
            cameraOutput.resizer->stop();
        }
    }
    for(auto& cameraOutput : currentOutputs) {
        if(cameraOutput.resizer != nullptr) {
            cameraOutput.resizer->join();
        }
    }
    ThreadedNode::stop();
    if(cameraSensor) cameraSensor->stop();
}

}  // namespace gate
}  // namespace dai
