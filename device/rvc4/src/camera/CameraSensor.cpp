#include "CameraSensor.hpp"

#include <cerrno>
#include <camera/VendorTagDescriptor.h>
#include <fcntl.h>
#include <gbm.h>
#include <gbm_priv.h>
#include <gpiod.h>
#include <linux/videodev2.h>
#include <luxonis_sidechannel_data.h>
#include <spdlog/spdlog.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "camera/CameraSingleton.hpp"
#include "camera/HalBufferMemory.hpp"
#include "camera/HalUtil.hpp"
#include "camera/IspRouteAllocator.hpp"
#include "camera/LuxonisSidechannel.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraSensorType.hpp"
#include "depthai/common/Timestamp.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "luxonis_sidechannel_data.h"
#include "utilities/DeviceTelemetry.hpp"
#include "utilities/DeviceTreeDiscovery.hpp"
#include "utilities/ErrorMacros.hpp"
#include "utilities/FsyncController.hpp"
#include "utilities/OSUtils.hpp"
#include "utilities/PtpUtils.hpp"
#include "utilities/StereoPairs.hpp"
#include "utilities/String.hpp"

#define COUNT_OF(arr) (sizeof(arr) / sizeof(arr[0]))

#define WSTRIDE_ALIGN 128
#define WSTRIDE_ALIGN_RAW 64
#define HSTRIDE_ALIGN 32

namespace dai {
namespace gate {

#ifndef ALIGN_UP
template <typename T>
constexpr T ALIGN_UP(T value, std::size_t alignment) {
    return (value + (alignment - 1)) & ~(alignment - 1);
}
#endif

#ifdef ENABLE_TIMING_DEBUG
    #include <chrono>
#endif

namespace {

bool readRawExposureTimeNs(
    const uint8_t* data, size_t size, uint32_t width, uint32_t height, uint32_t strideBytes, uint64_t& exposureNs) {
    exposureNs = 0;
    const bool supported = (width == 1344 && (height == 2420 || height == 4832 || height == 7244))
                           || (width == 672 && (height == 2420 || height == 3626));
    if(!data || !supported || strideBytes < width * 2 || size / strideBytes < height) return false;

    const auto readStatus = [&](uint32_t row, size_t offset, unsigned bytes, uint32_t& value) {  // VD55H1 status: 1 byte per 2 RAW16 px, after a 12-byte header
        const auto* packet = data + size_t(row) * strideBytes + 12 + 4 * offset;
        value = 0;
        for(unsigned i = 0; i < bytes; ++i, packet += 4) {
            const uint8_t tag = (packet[3] << 4) | (packet[2] >> 4);
            if(tag != 0x5A) return false;
            const uint8_t byte = (packet[1] << 4) | (packet[0] >> 4);
            value |= uint32_t(byte) << (8 * i);
        }
        return true;
    };
    uint32_t pixelClock = 0, lineLength = 0, exposure = 0;
    if(!readStatus(0, 0x78, 4, pixelClock)              // Status register 0x078: pixel clock in Hz.
       || !readStatus(2, 0x58, 2, lineLength)          // Status register 0x158: clocks per line.
       || !readStatus(height - 4, 0x3c, 4, exposure)   // Status register 0x23c: context 0 exposure.
       || pixelClock == 0 || lineLength == 0) return false;

    const uint64_t ticks = uint64_t(exposure >> 16) * lineLength + (exposure & 0xfff);
    exposureNs = (ticks * 1000000000ULL + pixelClock / 2) / pixelClock;
    return exposureNs > 0 && exposureNs <= 1000000000ULL;
}

constexpr int kDefaultStreamBufferCount = 14;
constexpr int kHfrStreamBufferCount = 48;
constexpr unsigned int SENSOR_ERROR_GPIO_CHIP = 0;

std::optional<int32_t> encodeExplicitIspRoute(const dai::CameraProperties& properties) {  // Keys match the strings of the ISP route allocator
    const std::string* fullIfeValue = nullptr;
    const std::string* sharedIspValue = nullptr;
    for(const auto& pairKV : properties.initialControl.miscControls) {
        if(std::get<0>(pairKV) == "beta-full-ife") {
            fullIfeValue = &std::get<1>(pairKV);
        } else if(std::get<0>(pairKV) == "beta-shared-isp") {
            sharedIspValue = &std::get<1>(pairKV);
        }
    }

    if(fullIfeValue != nullptr) {
        return dai::encodeFullIfeRoute(*fullIfeValue);
    }
    if(sharedIspValue != nullptr) {
        return dai::encodeSharedIspRoute(*sharedIspValue);
    }
    return std::nullopt;
}

struct SensorTelemetryMetrics {
    std::string name;
    double exposureTimeUsSum{0.0};
    double sensitivityIsoSum{0.0};
    double lensPositionSum{0.0};
    std::uint64_t count{0};
};

std::mutex sensorTelemetryMetricsMtx;
std::map<int, SensorTelemetryMetrics> sensorTelemetryMetrics;
std::once_flag sensorTelemetryMetricsRegistered;

void appendSensorTelemetryMetrics(nlohmann::json& properties) {
    std::map<int, SensorTelemetryMetrics> metricsBySocket;
    {
        std::lock_guard<std::mutex> lock(sensorTelemetryMetricsMtx);
        metricsBySocket.swap(sensorTelemetryMetrics);
    }

    if(metricsBySocket.empty()) {
        return;
    }

    if(!properties.contains("sensor_metrics") || !properties["sensor_metrics"].is_array()) {
        properties["sensor_metrics"] = nlohmann::json::array();
    }

    auto& sensorMetrics = properties["sensor_metrics"];
    for(const auto& [socket, metrics] : metricsBySocket) {
        if(metrics.count == 0) {
            continue;
        }

        const auto count = static_cast<double>(metrics.count);
        sensorMetrics.push_back({
            {"socket", socket},
            {"name", metrics.name},
            {"lens", metrics.lensPositionSum / count},
            {"iso", metrics.sensitivityIsoSum / count},
            {"exposure", metrics.exposureTimeUsSum / count},
        });
    }
}

void ensureSensorTelemetryMetricsRegistered() {
    std::call_once(sensorTelemetryMetricsRegistered, []() { DeviceTelemetry::getInstance().addAggregateMetrics(appendSensorTelemetryMetrics); });
}

void recordSensorTelemetryMetrics(int socket, const std::string& sensorName, double exposureTimeUs, double sensitivityIso, double lensPosition) {
    ensureSensorTelemetryMetricsRegistered();

    std::lock_guard<std::mutex> lock(sensorTelemetryMetricsMtx);
    auto& metrics = sensorTelemetryMetrics[socket];
    metrics.name = toLowerCase(sensorName);
    metrics.exposureTimeUsSum += exposureTimeUs;
    metrics.sensitivityIsoSum += sensitivityIso;
    metrics.lensPositionSum += lensPosition;
    ++metrics.count;
}

dai::ImgFrame::Fsync toImgFrameFsync(lux_sc_camera_fsync_t fsyncMode) {
    switch(fsyncMode) {
        case LUX_SC_CAMERA_FSYNC_SLAVE:
            return dai::ImgFrame::Fsync::INPUT;
        case LUX_SC_CAMERA_FSYNC_MASTER:
            return dai::ImgFrame::Fsync::OUTPUT;
        case LUX_SC_CAMERA_FSYNC_TIME:
            return dai::ImgFrame::Fsync::PTP;
        case LUX_SC_CAMERA_FSYNC_NONE:
        default:
            return dai::ImgFrame::Fsync::NONE;
    }
}

std::string telemetryFsyncModeName(lux_sc_camera_fsync_t fsyncMode) {
    switch(fsyncMode) {
        case LUX_SC_CAMERA_FSYNC_SLAVE:
            return "input";
        case LUX_SC_CAMERA_FSYNC_MASTER:
            return "output";
        case LUX_SC_CAMERA_FSYNC_TIME:
            return "ptp";
        case LUX_SC_CAMERA_FSYNC_NONE:
        default:
            return "none";
    }
}
}  // namespace

constexpr float NEUTRAL_KELVINS = 6500.0f;  // D65

std::mutex CameraSensor::buildMtx;
std::optional<CameraSensor::MetaCacheItem> CameraSensor::popMetaCache(uint64_t sequenceNumber, const std::string& output) {
    std::lock_guard lock(metaCacheMutex);
    metaCacheLast[output] = sequenceNumber;
    uint64_t smallestSeqNum = std::numeric_limits<uint64_t>::max();
    bool firstRuns = false;
    DAI_CHECK_IN(metaCacheLast.size() > 0);
    for(auto& item : metaCacheLast) {
        if(item.second == std::numeric_limits<uint64_t>::max()) {  // max = output has not popped yet (no cleanup); each output sets it before camera start
            firstRuns = true;
            break;
        }
        if(item.second < smallestSeqNum) {
            smallestSeqNum = item.second;
        }
    }
    const auto resultMaybe =
        std::find_if(metaCache.begin(), metaCache.end(), [sequenceNumber](const auto iter) { return iter.sequenceNumber == sequenceNumber; });
    if(resultMaybe == metaCache.end()) {
        if(sensorErrorGpioState.load() == 1) {  // Legal: the request errored (HAL ERROR_RESULT) or metadata is late; drop the frame, do not abort
            return std::nullopt;
        }
        uint64_t cacheMin = std::numeric_limits<uint64_t>::max();
        uint64_t cacheMax = 0;
        for(const auto& it : metaCache) {
            cacheMin = std::min(cacheMin, it.sequenceNumber);
            cacheMax = std::max(cacheMax, it.sequenceNumber);
        }
        logger->warn("No cached metadata for seq {} (output '{}'), dropping frame. cache size {}, seq range [{}, {}]",
                     sequenceNumber,
                     output,
                     metaCache.size(),
                     cacheMin,
                     cacheMax);
        return std::nullopt;
    }
    auto result = *resultMaybe;
    if(!firstRuns) {
        ++resultMaybe->triggeredCount;
        if(resultMaybe->triggeredCount == curStreams.size()) {
            metaCache.erase(
                std::remove_if(metaCache.begin(), metaCache.end(), [smallestSeqNum](const auto iter) { return iter.sequenceNumber <= smallestSeqNum; }),
                metaCache.end());
        }
    }
    return result;
}

void CameraSensor::setupRawOutput() {
    hasRaw = true;
    queues.push_back("raw");
}

void CameraSensor::setupDynamicOutputs(std::vector<std::tuple<uint32_t, uint32_t, float, size_t>>& specs) {
    outputSpecs = specs;
}

CameraSensor::CameraSensor(const CameraDescription cam,
                           const size_t sensorModeIndex,
                           float wantedFps,
                           lux_sc_camera_fsync_t fsyncMode,
                           dai::CameraImageOrientation orientation,
                           dai::CameraProperties properties,
                           std::optional<EepromData> eepromData,
                           FrameCallback frameCb,
                           std::shared_ptr<spdlog::async_logger> pLogger) {
    logger = pLogger;
    cameraDescription = cam;
    ensureSensorTelemetryMetricsRegistered();
    numFramesPoolOutputs = properties.numFramesPoolOutputs;
    maxSizePoolOutputs = properties.maxSizePoolOutputs;
    explicitIspRoute = encodeExplicitIspRoute(properties);
    {  // All camera nodes are constructed before any opens, so run() sees the whole fleet
        IspSensorRequest allocReq;
        allocReq.socket = static_cast<int>(cam.camera.socket);
        allocReq.socketName = toString(cam.camera.socket);
        allocReq.fps = wantedFps;
        const auto allocFeatures = CameraDiscovery::getInstance().getCameraFeatures("", cam.camera.socket);
        if(sensorModeIndex < allocFeatures.configs.size()) {
            allocReq.width = allocFeatures.configs[sensorModeIndex].width;
            allocReq.height = allocFeatures.configs[sensorModeIndex].height;
        }
        for(const auto& pairKV : properties.initialControl.miscControls) {
            if(std::get<0>(pairKV) == "beta-full-ife") {
                allocReq.manualFullIfe = std::get<1>(pairKV);
            } else if(std::get<0>(pairKV) == "beta-shared-isp") {
                allocReq.manualSharedIsp = std::get<1>(pairKV);
            }
        }
        IspRouteAllocator::getInstance().registerSensor(allocReq);
    }
    bool stereoSensor = false;
    if(eepromData.has_value()) {
        auto stereoPairs = getStereoPairs(eepromData.value());
        for(const auto& pair : stereoPairs) {
            if(pair.left == cam.camera.socket || pair.right == cam.camera.socket) {
                stereoSensor = true;
                break;
            }
        }
    } else {
        logger->error("No EEPROM data found for device. Can't check if is stereo sensor.");
    }
    this->eepromData = eepromData;
    curFps = wantedFps;
    cameraId = cam.id;
    this->fsyncMode = fsyncMode;
    logger->info("Camera node on socket {} is a stereo sensor: {}", cameraId, stereoSensor);
    sensorMode = sensorModeIndex;
    process_capture_result = processCaptureResultCb;
    notify = notifyCb;
    request_stream_buffers = requestStreamBuffersCb;
    return_stream_buffers = returnStreamBuffersCb;
    initialControl = properties.initialControl;
    isStereoSensor = stereoSensor;
    handleBuffer = frameCb;
    CameraSingleton::getInstance().setCameraActive(cam.camera.socket, true);
}

CameraSensor::~CameraSensor() {
    IspRouteAllocator::getInstance().unregisterSensor(static_cast<int>(cameraDescription.camera.socket));
    logger->trace("CameraSensor DESTRUCTOR called");
    unregisterSensorErrorCallback();
    stopSensorErrorMonitor();
}

void* CameraSensor::sensorErrorCbAdd(SensorErrorStateCallback cb) {
    if(!cb) {
        return nullptr;
    }

    auto entry = std::make_shared<SensorErrorCallbackEntry>();
    entry->cb = std::move(cb);

    std::lock_guard<std::mutex> lock(sensorErrorCallbacksLock);
    sensorErrorCallbacks.push_back(entry);
    return entry.get();
}

void CameraSensor::sensorErrorCbRemove(void* opaqueHandle) {
    if(opaqueHandle == nullptr) {
        return;
    }

    std::lock_guard<std::mutex> lock(sensorErrorCallbacksLock);
    sensorErrorCallbacks.erase(
        std::remove_if(sensorErrorCallbacks.begin(),
                       sensorErrorCallbacks.end(),
                       [opaqueHandle](const std::shared_ptr<SensorErrorCallbackEntry>& entry) { return entry.get() == opaqueHandle; }),
        sensorErrorCallbacks.end());
}

void CameraSensor::startSensorErrorMonitor() {
    stopSensorErrorMonitor();
    sensorErrorMonitorRunning = true;
    sensorErrorMonitorThread = std::thread(&CameraSensor::sensorErrorMonitorLoop, this);
}

void CameraSensor::stopSensorErrorMonitor() {
    sensorErrorMonitorRunning = false;
    if(sensorErrorMonitorThread.joinable()) {
        sensorErrorMonitorThread.join();
    }
}

int CameraSensor::shouldSigAbort(int32_t depthaiSocket, void* userData) {
    auto* sensor = static_cast<CameraSensor*>(userData);
    if(sensor == nullptr) {
        return 1;
    }

    if(static_cast<int32_t>(sensor->cameraDescription.camera.socket) != depthaiSocket) {
        return 1;
    }

    const int gpioState = sensor->sensorErrorGpioState.load();
    return (gpioState == 1) ? 0 : 1;
}

void CameraSensor::registerSensorErrorCallback() {
    unregisterSensorErrorCallback();

    auto* sidechannel = LuxonisSidechannel::tryGetInstance();
    if(sidechannel == nullptr || sidechannel->ops.sensor_err_cb_add == nullptr || sidechannel->ops.sensor_err_cb_rm == nullptr) {
        return;
    }

    const auto depthaiSocket = static_cast<int32_t>(cameraDescription.camera.socket);
    sensorErrorCbHandle = sidechannel->ops.sensor_err_cb_add(depthaiSocket, &CameraSensor::shouldSigAbort, this);
    if(sensorErrorCbHandle == nullptr) {
        logger->warn("Failed to register sensor error callback for socket {}", static_cast<int>(cameraDescription.camera.socket));
    }
}

void CameraSensor::unregisterSensorErrorCallback() {
    auto* sidechannel = LuxonisSidechannel::tryGetInstance();
    if((sensorErrorCbHandle != nullptr) && (sidechannel != nullptr) && (sidechannel->ops.sensor_err_cb_rm != nullptr)) {
        sidechannel->ops.sensor_err_cb_rm(sensorErrorCbHandle);
    }
    sensorErrorCbHandle = nullptr;
}

void CameraSensor::notifySensorErrorStateChanged(int state) {
    std::vector<std::shared_ptr<SensorErrorCallbackEntry>> callbacks;
    {
        std::lock_guard<std::mutex> lock(sensorErrorCallbacksLock);
        callbacks = sensorErrorCallbacks;
    }

    for(const auto& entry : callbacks) {
        if(entry && entry->cb) {
            entry->cb(state);
        }
    }
}

void CameraSensor::sensorErrorMonitorLoop() {
    auto* sidechannel = LuxonisSidechannel::tryGetInstance();
    if(sidechannel == nullptr || sidechannel->ops.sensor_get_error_gpio == nullptr) {
        DAI_LOG("No sensor error GPIO callback available for socket {}, disabling monitor", static_cast<int>(cameraDescription.camera.socket));
        return;
    }

    const auto cfgGpio = sidechannel->ops.sensor_get_error_gpio(static_cast<uint32_t>(cameraDescription.camera.socket));
    if(cfgGpio < 0) {
        DAI_LOG("No sensor error GPIO configured for socket {}, disabling monitor", static_cast<int>(cameraDescription.camera.socket));
        return;
    }

    const auto gpioLine = static_cast<unsigned int>(cfgGpio);
    gpiod_chip* chip = gpiod_chip_open_by_number(SENSOR_ERROR_GPIO_CHIP);
    if(chip == nullptr) {
        DAI_WARN("Failed to open /dev/gpiochip{}: {}", SENSOR_ERROR_GPIO_CHIP, std::strerror(errno));
        return;
    }

    gpiod_line* line = gpiod_chip_get_line(chip, gpioLine);
    if(line == nullptr) {
        DAI_WARN("Failed to get gpiochip{} line {}: {}", SENSOR_ERROR_GPIO_CHIP, gpioLine, std::strerror(errno));
        gpiod_chip_close(chip);
        return;
    }

    const auto consumerName = std::string("depthai-device-gpio") + std::to_string(gpioLine) + "-monitor";
    if(gpiod_line_request_both_edges_events(line, consumerName.c_str()) < 0) {
        DAI_WARN("Failed to request sensor error GPIO {} edge events: {}", gpioLine, std::strerror(errno));
        gpiod_chip_close(chip);
        return;
    }

    const int initialState = gpiod_line_get_value(line);
    if(initialState < 0) {
        DAI_WARN("Failed to read initial sensor error GPIO {} state: {}", gpioLine, std::strerror(errno));
    } else {
        sensorErrorGpioState.store(initialState);
    }

    DAI_INFO("Monitoring /dev/gpiochip{} line {} for sensor error", SENSOR_ERROR_GPIO_CHIP, gpioLine);

    const timespec timeout{1, 0};
    while(sensorErrorMonitorRunning.load()) {
        const int waitResult = gpiod_line_event_wait(line, &timeout);
        if(waitResult < 0) {
            DAI_WARN("Sensor error GPIO {} event wait failed: {}", gpioLine, std::strerror(errno));
            break;
        }
        if(waitResult == 0) {
            continue;
        }

        gpiod_line_event event{};
        if(gpiod_line_event_read(line, &event) < 0) {
            DAI_WARN("Sensor error GPIO {} event read failed: {}", gpioLine, std::strerror(errno));
            break;
        }

        const int state = event.event_type == GPIOD_LINE_EVENT_RISING_EDGE ? 1 : 0;
        sensorErrorGpioState.store(state);
        DAI_INFO("Sensor error GPIO {} changed to {}", gpioLine, state);
        notifySensorErrorStateChanged(state);
    }

    gpiod_line_release(line);
    gpiod_chip_close(chip);
}

void CameraSensor::build(dai::CameraSensorType type) {
    std::unique_lock<std::mutex> lock(buildMtx);
    int fsyncId = 0;
    if(0 == (int)cameraId) {
        fsyncId = 2;  // FSYNC PIN index 2 == PA1
    } else if(1 == (int)cameraId || 2 == (int)cameraId) {
        fsyncId = 4;  // FSYNC PIN index 4 == PA8
    } else {
        // DAI_CHECK_IN(false); // TODO(Morato) fix this once the inforomation is available from the OS
    }
    auto ret = dai::DeviceTreeDiscovery::getInstance().getFsyncPin((uint32_t)cameraId);
    if(ret.has_value()) {
        fsyncId = static_cast<int>(*ret);
    } else {
        if(!dai::gate::FsyncController::getInstance().supportsExternalFrameSyncMode()) {
            logger->info("Could not get the FSYNC pin for socket " + std::to_string(cameraId) + ". OS probably does not support dynamic fsync pin assignment.");
            logger->info("Falling back to hardcoded FSYNC pin.");
        } else {
            logger->error("Could not get the FSYNC pin for socket " + std::to_string(cameraId) + ". Check camera device tree and camx camera configuration.");
            logger->warn("Falling back to hardcoded FSYNC pin.");
        }
    }
    isToFSensor = (type == dai::CameraSensorType::TOF);
    auto* sidechannel = LuxonisSidechannel::tryGetInstance();
    if(sidechannel == nullptr || sidechannel->ops.sensor_get_error_gpio == nullptr) {
        logger->info("Skipping sensor error GPIO monitor for socket {}: callback not available",
                     static_cast<int>(cameraDescription.camera.socket));
    } else {
        const auto cfgGpio = sidechannel->ops.sensor_get_error_gpio(static_cast<uint32_t>(cameraDescription.camera.socket));
        if(cfgGpio < 0) {
            logger->info("Skipping sensor error GPIO monitor for socket {}: GPIO not configured",
                         static_cast<int>(cameraDescription.camera.socket));
        } else {
            registerSensorErrorCallback();
            startSensorErrorMonitor();
        }
    }
    if(dai::gate::getOSVersion() >= dai::Version(0, 25, 0)) {
        if(fsyncMode == LUX_SC_CAMERA_FSYNC_SLAVE) {
            printf("Turning fsync slave mode on\n");
        }

#ifndef DISABLE_FSYNC
        if(fsyncMode == LUX_SC_CAMERA_FSYNC_SLAVE && dai::gate::FsyncController::getInstance().isOk()) {
    #ifndef FSYNC_DONT_WRITE
            auto oldFps{curFps};
            curFps = dai::gate::FsyncController::getInstance().getClosestFps(curFps);
            DAI_CHECK_V(
                std::abs(oldFps - curFps) < 0.5,
                "Fsync driver returned wrong closest fps {} -> {}",
                oldFps,
                curFps);
    #endif
        }
#endif

        lux_sc_camera_mode_set_t mode;
        mode.camera_id = cameraId;
#ifdef DISABLE_FSYNC
        mode.fsync_mode = LUX_SC_CAMERA_FSYNC_NONE;
#else
        mode.fsync_mode = fsyncMode;
#endif
        mode.frame_duration = std::lround(1.0 / (double)curFps * 1.0e9);
        LuxonisSidechannel::getInstance().setCameraMode(mode);
    } else {
        logger->info("NOT turning fsync slave mode on as OS is old");
    }
    if(hasRaw + outputSpecs.size() > 3) {
        throw std::runtime_error("rvc4 currently supports max. 3 outputs per Camera sensor. Add an ImageManip node or contact support if you need more.");
    } else if(!hasRaw && outputSpecs.empty()) {
        throw std::runtime_error("CameraSensor must have at least one output. Use requestOutput() or create a link from raw output.");
    }
}

void CameraSensor::cameraLoop() {
    int frameNumber{0};
    while(true) {
        if(frameNumber == 1) {  // On RVC4 the move lens command must also be applied after stream start
            const bool hasMoveLens = initialControl.getCommand(dai::CameraControl::Command::MOVE_LENS);
            const bool hasMoveLensRaw = initialControl.getCommand(dai::CameraControl::Command::MOVE_LENS_RAW);
            if(hasMoveLens || hasMoveLensRaw) {
                dai::CameraControl control;
                if(hasMoveLens) {
                    control.setManualFocus(initialControl.lensPosition);
                }
                if(hasMoveLensRaw) {
                    control.setManualFocusRaw(initialControl.lensPositionRaw);
                }
                applyCameraControl(control);
            }
        }
        std::vector<camera3_stream_buffer_t> outputBuffers;
        for(auto& stream : curStreams) {
            outputBuffers.push_back(cameraPool->getFreeBuffer(&stream));
        }
        curRequest.frame_number = frameNumber;
        {
            std::lock_guard lock(curSettingsNextLock);
            if(curSettingsNextChanged) {
                curSettingsNextChanged = false;
                logger->trace("Applying new request settings");
                curSettings.unlock(curSettingsLastLocked);  // Must be unlocked for any write, also for the copy below
                curSettings = curSettingsNext;
                curSettingsLastLocked = curSettings.getAndLock();
                auto afTrigger = curSettingsNext.find(ANDROID_CONTROL_AF_TRIGGER);
                if(afTrigger.count == 1 && afTrigger.data.u8[0] == ANDROID_CONTROL_AF_TRIGGER_START) {
                    logger->debug("Camera[{}] AF trigger sent with request {}", cameraId, frameNumber);
                    const uint8_t idle = ANDROID_CONTROL_AF_TRIGGER_IDLE;
                    auto res = curSettingsNext.update(ANDROID_CONTROL_AF_TRIGGER, &idle, 1);
                    DAI_CHECK_IN(res == 0);
                    curSettingsNextChanged = true;
                }
            }
        }
        curRequest.settings = curSettingsLastLocked;
        curRequest.num_output_buffers = outputBuffers.size();
        curRequest.output_buffers = outputBuffers.data();
        auto res = mDevice->ops->process_capture_request(mDevice, &curRequest);
        DAI_CHECK_IN(res == 0);
        ++frameNumber;
    }
}

void CameraSensor::addStream(std::vector<camera3_stream_t*>& curStreamsPtrs, const std::string& outputName, int format, uint32_t width, uint32_t height) {
    DAI_CHECK_IN(curStreams.capacity() >= curStreams.size() + 1 && curStreamsPtrs.capacity() >= curStreamsPtrs.size() + 1);  // Growing would move the pointers
    metaCacheLast[outputName] = std::numeric_limits<uint64_t>::max();
    curStreams.push_back({});
    auto* newStream = &curStreams.back();
    logger->trace("new stream: {:p}, {:p}", static_cast<void*>(newStream), static_cast<void*>(&curStreams[curStreams.size() - 1]));
    newStream->stream_type = CAMERA3_STREAM_OUTPUT;
    newStream->width = width;
    newStream->height = height;
    newStream->format = format;
    newStream->data_space = static_cast<android_dataspace>(0x0);
    newStream->rotation = CAMERA3_STREAM_ROTATION_0;
    newStream->usage = GRALLOC_USAGE_HW_VIDEO_ENCODER | GRALLOC_USAGE_SW_READ_OFTEN | GRALLOC_USAGE_SW_WRITE_OFTEN;
    newStream->max_buffers =
        highFpsMode ? kHfrStreamBufferCount : kDefaultStreamBufferCount;  // Use higher buffer count for HFR (qmmf_camera_adaptor_gtest.cc).
    newStream->group_id = -1;
    newStream->priv = nullptr;
    curStreamsPtrs.push_back(newStream);
    {
        std::lock_guard lock(streamsToCallbacksLock);
        logger->trace("streamsToCallback[{:p}] = XXX", static_cast<void*>(newStream));
        streamsToCallbacks[newStream] = [this, outputName](const camera3_stream_buffer_t* buffer, const camera3_capture_result_t* result) {
#ifdef ENABLE_TIMING_DEBUG
            uint64_t nekStartTime;
            asm volatile("mrs %0, cntvct_el0" : "=r"(nekStartTime));
            const auto startTime = std::chrono::high_resolution_clock::now();
#endif
            try {
                auto halData = std::make_shared<dai::gate::HalBufferMemory>(buffer, cameraPool);
                std::shared_ptr<dai::Memory> data = halData;
#define INT1E9 1000000000
                auto seqNum = result->frame_number;
                dai::ImgFrame::CameraSettings cam{};
                cam.fsync = toImgFrameFsync(fsyncMode);
                cam.sensorMode = static_cast<int32_t>(sensorMode);
                cam.fps = curFps;
                logger->trace("before pop: {}", result->frame_number);
                struct timespec timestamp = {};
#ifdef DEBUG_DISABLE_META
                cam.exposureTimeUs = 1000;
                cam.lensPositionRaw = 0.5;
                cam.lensPosition = 128;
#else
                const auto metaMaybe = popMetaCache(seqNum, outputName);
                if(!metaMaybe.has_value()) {
                    return;
                }
                const auto& meta = *metaMaybe;
                if(!streamEnabled.load()) {
                    return;
                }
                logger->trace("after pop: {}", result->frame_number);
                cam.exposureTimeUs = meta.exposureTime / 1000;
                cam.lensPositionRaw = meta.lensPosition;
                cam.lensPosition = std::round(meta.lensPosition * 255.0f);
                cam.wbColorTemp = meta.correlatedColorTemperature;
                cam.sensitivityIso = meta.iso;
                cam.sensorTemperatureC = meta.sensorTemperatureC;
                logger->trace("metadata for frame {}: exposure: {}, timestamp: {}, lens position: {}\n",
                              meta.sequenceNumber,
                              meta.exposureTime,
                              meta.timestamp,
                              meta.lensPosition);
                GST_TIME_TO_TIMESPEC(meta.timestamp + meta.exposureTime, timestamp);  // meta.timestamp is start of exposure; depthai-core wants end of exposure
#endif
                dai::Timestamp ts = {.sec = timestamp.tv_sec, .nsec = timestamp.tv_nsec};
                auto* bo = cameraPool->getBo(buffer);
                if(!bo) {
                    logger->error("getBo returned null for frame {}", result->frame_number);
                    cameraPool->returnBuffer(buffer);
                    return;
                }
                auto stride = gbm_bo_get_stride(bo);
                if(buffer->stream->format == HAL_PIXEL_FORMAT_RAW16) {
                    if(cameraDescription.camera.sensorName == "VD55H1") {
                        uint64_t exposureNs = 0;
                        if(readRawExposureTimeNs(halData->getData().data(),
                                                 halData->getSize(),
                                                 halData->privHnd->width,
                                                 halData->privHnd->height,
                                                 stride,
                                                 exposureNs)) {
                            cam.exposureTimeUs = static_cast<int32_t>((exposureNs + 500) / 1000);
                        } else {
                            logger->warn("Could not parse VD55H1 exposure status for raw frame {}", seqNum);
                        }
                    }
                    logger->trace("RAW16 dims: {}x{}, stride: {}, size: {}",
                                  halData->privHnd->width,
                                  halData->privHnd->height,
                                  stride,
                                  halData->getSize());
                    handleBuffer({data,
                                  halData->privHnd->width,
                                  halData->privHnd->height,
                                  static_cast<int>(halData->getSize()),
                                  static_cast<int>(stride),
                                  halData->privHnd->height,
                                  std::string("GRAY16"),
                                  ts,
                                  outputName,
                                  cam,
                                  seqNum});
                } else if(buffer->stream->format == HAL_PIXEL_FORMAT_RAW10) {
                    logger->trace("RAW dims: {}x{}, dims_unaligned: {}x{}, size: {}",
                                  halData->privHnd->width,
                                  halData->privHnd->height,
                                  ALIGN_UP(halData->privHnd->width, WSTRIDE_ALIGN_RAW) * 10 / 8,  // stride,
                                  ALIGN_UP(halData->privHnd->height, HSTRIDE_ALIGN),              // halData->getSize() / stride,
                                  halData->getSize());
                    handleBuffer({data,
                                  halData->privHnd->width,
                                  halData->privHnd->height,
                                  static_cast<int>(halData->getSize()),
                                  ALIGN_UP(halData->privHnd->width, WSTRIDE_ALIGN_RAW) * 10 / 8,  // stride,
                                  ALIGN_UP(halData->privHnd->height, HSTRIDE_ALIGN),              // halData->getSize() / stride,
                                  std::string("RAW10"),
                                  ts,
                                  outputName,
                                  cam,
                                  seqNum});
                } else {
                    logger->trace("dims: {}x{}, dims_unaligned: {}x{}, size: {}",
                                  halData->privHnd->width,
                                  halData->privHnd->height,
                                  stride,
                                  gbm_bo_get_offset(bo, 1) / stride,
                                  halData->getSize());
                    handleBuffer({data,
                                  halData->privHnd->width,
                                  halData->privHnd->height,
                                  static_cast<int>(halData->getSize()),
                                  static_cast<int>(stride),
                                  static_cast<int>(gbm_bo_get_offset(bo, 1) / stride),
                                  std::string("NV12"),
                                  ts,
                                  outputName,
                                  cam,
                                  seqNum});
                }
            } catch(const std::exception& exception) {
                logger->error("Exception in send_frame(): {}", exception.what());
            } catch(const std::string& exception) {
                logger->error("Exception in send_frame(): {}", exception);
            } catch(...) {
                logger->error("Exception in send_frame(): Unknown type of exception");
            }
#ifdef ENABLE_TIMING_DEBUG
            const auto endTime = std::chrono::high_resolution_clock::now();
            logger->trace("stream callback took: {}ms", std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() / 1000.0f);
            uint64_t nekEndTime;
            asm volatile("mrs %0, cntvct_el0" : "=r"(nekEndTime));
            logger->trace("stream callback took: {} cpu cycles", nekEndTime - nekStartTime);
#endif
        };
    }  // namespace gate
}  // namespace dai

namespace {
void PrintAfAvailableModes(const android::hardware::camera::common::V1_0::helper::CameraMetadata& meta,
                           int cameraId,
                           const std::shared_ptr<spdlog::async_logger>& logger) {
    auto AfModeToStr = [](uint8_t mode) -> const char* {
        switch(mode) {
            case ANDROID_CONTROL_AF_MODE_OFF:
                return "OFF";
            case ANDROID_CONTROL_AF_MODE_AUTO:
                return "AUTO";
            case ANDROID_CONTROL_AF_MODE_MACRO:
                return "MACRO";
            case ANDROID_CONTROL_AF_MODE_CONTINUOUS_VIDEO:
                return "CONTINUOUS_VIDEO";
            case ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE:
                return "CONTINUOUS_PICTURE";
            case ANDROID_CONTROL_AF_MODE_EDOF:
                return "EDOF";
            default:
                return "UNKNOWN";
        }
    };
    if(!logger) {
        return;
    }
    if(!meta.exists(ANDROID_CONTROL_AF_AVAILABLE_MODES)) {
        logger->warn("cameraId {}: ANDROID_CONTROL_AF_AVAILABLE_MODES not available", cameraId);
        return;
    }
    auto entry = meta.find(ANDROID_CONTROL_AF_AVAILABLE_MODES);
    std::string modes;
    modes.reserve(entry.count * 16);
    for(uint32_t i = 0; i < entry.count; ++i) {
        if(i != 0) {
            modes += ",";
        }
        modes += AfModeToStr(entry.data.u8[i]);
    }
    logger->warn("cameraId {}: ANDROID_CONTROL_AF_AVAILABLE_MODES = [{}]", cameraId, modes);
}
}  // namespace

void CameraSensor::run() {
    std::unique_lock<std::mutex> lock(buildMtx);
    DAI_CHECK_IN(curFps > 0.0f);
    if(!onlyOnce) {
        printf("ERROR: Why was \"void CameraSensor::run(bool runInThread)\" called more than once??? Ignoring second call.\n");
        return;
    }
    onlyOnce = false;
    {  // The IspAlloc log line is stable: tests parse it
        const auto decision = IspRouteAllocator::getInstance().resolve(static_cast<int>(cameraDescription.camera.socket));
        DAI_CHECK_V(decision.error.empty(),
                    "ISP route allocation failed for socket {}: {}",
                    toString(cameraDescription.camera.socket),
                    decision.error);
        if(!decision.fullIfe.empty()) {
            explicitIspRoute = dai::encodeFullIfeRoute(decision.fullIfe);
        } else if(!decision.sharedIsp.empty()) {
            explicitIspRoute = dai::encodeSharedIspRoute(decision.sharedIsp);
        } else {
            explicitIspRoute = std::nullopt;
        }
        const auto features = CameraDiscovery::getInstance().getCameraFeatures("", cameraDescription.camera.socket);
        uint32_t modeWidth = 0;
        uint32_t modeHeight = 0;
        if(sensorMode < features.configs.size()) {
            modeWidth = features.configs[sensorMode].width;
            modeHeight = features.configs[sensorMode].height;
        }
        logger->info("IspAlloc socket={} route={} source={} mode={}x{}@{:.2f}",
                     toString(cameraDescription.camera.socket),
                     decision.canonical,
                     decision.automatic ? "auto" : "manual",
                     modeWidth,
                     modeHeight,
                     curFps);
    }
    auto camM = CameraSingleton::getInstance().cameraModule;
    DAI_CHECK_IN(camM);
    loadCorrectTuning();
    auto res = camM->common.methods->open(&camM->common, std::to_string(cameraId).c_str(), (hw_device_t**)(&mDevice));
    DAI_CHECK_V(0 == res && mDevice != nullptr, "Opening the camera {} failed", cameraId);
    res = mDevice->ops->initialize(mDevice, this);
    int specIndex{0};
    float fps{0.0f};
    std::vector<camera3_stream_t*> curStreamsPtrs;
    int allStreamsSize = outputSpecs.size() + hasRaw;
    curStreams.reserve(allStreamsSize);
    curStreamsPtrs.reserve(allStreamsSize);
    highFpsMode = curFps > 120.999999f;
    const bool sharedIspRoute = explicitIspRoute.has_value() && !dai::ispRouteUsesBps(*explicitIspRoute)  // Shared ISP needs NORMAL mode: HFR ignores the route
                                && !dai::ispRouteHasDedicatedFullIfe(*explicitIspRoute);
    hfrModeEnabled = highFpsMode && !sharedIspRoute;  // sharedIspRoute keeps kHfrStreamBufferCount: a shallow pool shows as frozen frames
    cameraPool = std::make_shared<CameraPool>((highFpsMode || sharedIspRoute) ? kHfrStreamBufferCount : numFramesPoolOutputs, maxSizePoolOutputs);
    if(hasRaw) {
        auto features = CameraDiscovery::getInstance().getCameraFeatures("", cameraDescription.camera.socket);
        DAI_CHECK_IN(sensorMode < features.configs.size());
        auto config = features.configs[sensorMode];
        int rawFormat = isToFSensor ? HAL_PIXEL_FORMAT_RAW16 : HAL_PIXEL_FORMAT_RAW10;
        addStream(curStreamsPtrs, "raw", rawFormat, config.width, config.height);
    }
    for(const auto& spec : outputSpecs) {
        addStream(curStreamsPtrs,
                  std::string("dynOut") + std::to_string(std::get<3>(spec)),
                  HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED,
                  std::get<0>(spec),
                  std::get<1>(spec));
    }
    auto* config = &curStreamsConfig;
    const auto selectedSensorFeatures = CameraDiscovery::getInstance().getCameraFeatures("", cameraDescription.camera.socket);
    DAI_CHECK_IN(sensorMode < selectedSensorFeatures.configs.size());
    const auto& selectedSensorConfig = selectedSensorFeatures.configs[sensorMode];
    config->operation_mode = hfrModeEnabled ? CAMERA3_STREAM_CONFIGURATION_CONSTRAINED_HIGH_SPEED_MODE : CAMERA3_STREAM_CONFIGURATION_NORMAL_MODE;
#define QCAMERA3_SENSORMODE_ZZHDR_OPMODE (0xF002)
    if(initialControl.enableHdr) {
        config->operation_mode |= QCAMERA3_SENSORMODE_ZZHDR_OPMODE;
    }

    if(initialControl.frameSyncMode == CameraControl::FrameSyncMode::TIME_PTP && !getPtpEnabled()) {
        logger->error("PTP frame sync requested but not enabled (or supported) in the OS. Please enable it first in the OS (see documentation) and then use it in DAI.");
        throw std::runtime_error("PTP frame sync requested but not enabled (supported) in the OS. Please enable it first in the OS (see documentation) and then use it in DAI.");
    }
#define FORCE_SENSOR_MODE_MASK (0x00F00000)
#define FORCE_SENSOR_MODE_DATA(idx) ((idx + 1) << 20)
#define FORCE_SENSORMODE_ENABLE (1 << 24)
    uint32_t forceSensorMode = 1 << 4;
    const bool fsyncEnabled = (fsyncMode == LUX_SC_CAMERA_FSYNC_SLAVE);
    const bool ptpEnabled = (fsyncMode == LUX_SC_CAMERA_FSYNC_TIME);
    DAI_INFO("Forcing sensor mode: {} ({}x{}) with fps: {}, HFR: {}, fsync enabled: {}, ptp enabled: {}",
             sensorMode,
             selectedSensorConfig.width,
             selectedSensorConfig.height,
             curFps,
             hfrModeEnabled,
             fsyncEnabled,
             ptpEnabled);
    DeviceTelemetry::getInstance().event("depthai_camera_sensor_mode_started",
                                         nlohmann::json{
                                             {"socket", toString(cameraDescription.camera.socket)},
                                             {"sensor_name", toLowerCase(cameraDescription.camera.sensorName)},
                                             {"width", selectedSensorConfig.width},
                                             {"height", selectedSensorConfig.height},
                                             {"fps", curFps},
                                             {"fsync_mode", telemetryFsyncModeName(fsyncMode)},
                                             {"hdr_enabled", initialControl.enableHdr},
                                         });
    activeArraySize = readActiveArraySize();
    hfrBatchSize = hfrModeEnabled ? readHfrBatchSize(curStreams.front().width, curStreams.front().height) : 1;
    int cam_feature_flags = (FORCE_SENSOR_MODE_DATA(sensorMode) | forceSensorMode);
    config->operation_mode |= ((FORCE_SENSOR_MODE_MASK & cam_feature_flags) | FORCE_SENSORMODE_ENABLE);
    config->num_streams = curStreamsPtrs.size();
    config->streams = curStreamsPtrs.data();
    int32_t fpsRange[2] = {static_cast<int32_t>(std::lround(curFps)), static_cast<int32_t>(std::lround(curFps))};  // Without it, 3A runs at the mode's max fps
    res = sessionParameters.update(ANDROID_CONTROL_AE_TARGET_FPS_RANGE, fpsRange, 2);
    DAI_CHECK_IN(res == 0);
    const auto socket = cameraDescription.camera.socket;
    const bool useRealtimeBPS = explicitIspRoute.has_value() && dai::ispRouteUsesBps(*explicitIspRoute);
    const bool useOfflineIFE = explicitIspRoute.has_value() && !useRealtimeBPS && !dai::ispRouteHasDedicatedFullIfe(*explicitIspRoute);
    int32_t enableOfflineIFE{explicitIspRoute.value_or(useOfflineIFE ? 1 : 0)};
    if(explicitIspRoute.has_value()) {
        logger->info("Explicit ISP route for socket {}: 0x{:08x}, offline {}, BPS {}",
                     static_cast<int>(socket),
                     static_cast<uint32_t>(*explicitIspRoute),
                     useOfflineIFE,
                     useRealtimeBPS);
    }
    res = sessionParameters.update(HalUtil::getVendorTag("org.codeaurora.qcamera3.sessionParameters.EnableOfflineIFE"), &enableOfflineIFE, 1);
    DAI_CHECK_IN(res == 0);
    uint8_t cameraRunningOnBPS[sizeof(int32_t)] = {static_cast<uint8_t>(useRealtimeBPS)};  // HAL tag is TYPE_BYTE with sizeof(BOOL) bytes
    res = sessionParameters.update(HalUtil::getVendorTag("com.qti.chi.bpsrealtimecam.cameraRunningOnBPS"), cameraRunningOnBPS, sizeof(cameraRunningOnBPS));
    DAI_CHECK_IN(res == 0);
    config->session_parameters = sessionParameters.getAndLock();
    res = mDevice->ops->configure_streams(mDevice, config);
    DAI_CHECK_IN(res == 0);
    const camera_metadata_t* rawRequest =
        mDevice->ops->construct_default_request_settings(mDevice, hfrModeEnabled ? CAMERA3_TEMPLATE_VIDEO_RECORD : CAMERA3_TEMPLATE_PREVIEW);
    curRequest.frame_number = 0;
    curSettings = rawRequest;
    int64_t frameDuration = std::lround(1.0 / curFps * 1.0e9);  // Override set explicitly: else AE_TARGET_FPS_RANGE rounds fps
    res = curSettings.update(HalUtil::getVendorTag("org.quic.camera.manualExposure.overrideSensorFrameDuration"), &frameDuration, 1);
    DAI_CHECK_IN(res == 0);
    uint8_t disable{1};
    res = curSettings.update(HalUtil::getVendorTag("org.quic.camera.manualExposure.disableFPSLimits"), &disable, 1);
    DAI_CHECK_IN(res == 0);
    if(isStereoSensor) {
        uint8_t mode = ANDROID_NOISE_REDUCTION_MODE_HIGH_QUALITY;
        auto res = curSettings.update(ANDROID_NOISE_REDUCTION_MODE, &mode, 1);
        DAI_CHECK_IN(res == 0);
        int32_t sharpness = 1;
        res = curSettings.update(HalUtil::getVendorTag("org.codeaurora.qcamera3.sharpness.strength"), &sharpness, 1);
        DAI_CHECK_IN(res == 0);
    }
    applyCameraControl(initialControl);
    curRequest.num_output_buffers = 0;
    curRequest.output_buffers = nullptr;
    camThread = std::thread(&CameraSensor::cameraLoop, this);
}

void CameraSensor::stop() {
    logger->trace("CameraSensor stop called");
    CameraSingleton::getInstance().setCameraActive(cameraDescription.camera.socket, false);
    unregisterSensorErrorCallback();
    stopSensorErrorMonitor();
}

enum sensor_register_control_command {
    HDR_EXPOSURE_RATIO_COMMAND = 1,
    HDR_LOCAL_TONE_WEIGHT_COMMAND,
};

#define MAP_VALUE(x, in_min, in_max, out_min, out_max) ((((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min))) + (out_min))

std::optional<std::pair<int32_t, int32_t>> CameraSensor::readActiveArraySize() const {  // AF/AE regions are in active-array pixels, for any sensor mode
    using CameraMetadata = android::hardware::camera::common::V1_0::helper::CameraMetadata;
    CameraMetadata staticMeta;
    staticMeta = CameraSingleton::getInstance().cameraInfos[cameraId].static_camera_characteristics;
    if(!staticMeta.exists(ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE)) {
        logger->warn("Camera[{}] has no active array size, AF and AE regions are passed to the camera stack unclamped", cameraId);
        return std::nullopt;
    }
    const auto activeArray = staticMeta.find(ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE);  // xmin, ymin, width, height
    logger->info("Camera[{}] AF/AE regions are in active-array pixels, {}x{}", cameraId, activeArray.data.i32[2], activeArray.data.i32[3]);
    return std::make_pair(activeArray.data.i32[2], activeArray.data.i32[3]);
}

uint32_t CameraSensor::readHfrBatchSize(uint32_t width, uint32_t height) const {  // As the HAL picks it; 1 when none matches
    using CameraMetadata = android::hardware::camera::common::V1_0::helper::CameraMetadata;
    CameraMetadata staticMeta;
    staticMeta = CameraSingleton::getInstance().cameraInfos[cameraId].static_camera_characteristics;
    const auto entry = staticMeta.find(ANDROID_CONTROL_AVAILABLE_HIGH_SPEED_VIDEO_CONFIGURATIONS);  // width, height, min fps, max fps, max batch size
    const auto fps = static_cast<int32_t>(std::lround(curFps));
    for(size_t i = 0; i + 4 < entry.count; i += 5) {
        const int32_t* config = entry.data.i32 + i;
        if(config[0] == static_cast<int32_t>(width) && config[1] == static_cast<int32_t>(height) && config[3] == fps) {
            return static_cast<uint32_t>(std::max(config[4], 1));
        }
    }
    return 1;
}

std::array<int32_t, 5> CameraSensor::toCamxRegion(const dai::CameraControl::RegionParams& region, const char* name) const {  // [xmin, ymin, xmax, ymax, weight]
    constexpr int32_t weight = 1000;  // Max priority; CameraControl does not expose the region priority.
    const int32_t right = region.x + region.width, bottom = region.y + region.height;
    if(!activeArraySize) {
        return {region.x, region.y, right, bottom, weight};
    }
    const auto [width, height] = *activeArraySize;
    if((right > width || bottom > height) && !warnedRegionClamped) {
        warnedRegionClamped = true;
        logger->warn(
            "Camera[{}] {} region {},{} {}x{} reaches past the active array {}x{} and is clamped to it. Regions are in pixels of the "
            "frames' transformation source, see CameraControl::set{}Region(Rect, ImgTransformation)",
            cameraId,
            name,
            region.x,
            region.y,
            region.width,
            region.height,
            width,
            height,
            name);
    }
    return {std::min<int32_t>(region.x, width), std::min<int32_t>(region.y, height), std::min(right, width), std::min(bottom, height), weight};
}

void CameraSensor::applyCameraControl(dai::CameraControl control) {
    logger->trace("applyCameraControl() called");
    std::lock_guard lock(curSettingsNextLock);
    if(!curSettingsNextChanged) {
        curSettingsNextChanged = true;
        curSettingsNext = curSettings;
    }
    if(control.getCommand(dai::CameraControl::Command::AF_TRIGGER) && !control.getCommand(dai::CameraControl::Command::AF_MODE)) {  // HAL3 ignores it in AF OFF
        const auto afMode = curSettingsNext.find(ANDROID_CONTROL_AF_MODE);
        if(afMode.count == 1 && afMode.data.u8[0] != ANDROID_CONTROL_AF_MODE_AUTO && afMode.data.u8[0] != ANDROID_CONTROL_AF_MODE_MACRO) {
            control.setAutoFocusMode(dai::CameraControl::AutoFocusMode::AUTO);
        }
    }
    auto cmdMask = control.cmdMask;
    for(int idx = 0; cmdMask != 0; cmdMask >>= 1, idx++) {
        if(cmdMask & 1) {
            try {
                dai::CameraControl::Command cmd = (dai::CameraControl::Command)idx;
                switch(cmd) {
                    case dai::CameraControl::Command::AE_MANUAL: {
                        if(isToFSensor) {
                            logger->trace("Camera[{}] manual exp: {}, iso {}", cameraId, control.expManual.exposureTimeUs, control.expManual.sensitivityIso);
                        } else {
                            logger->info("Camera[{}] manual exp: {}, iso {}", cameraId, control.expManual.exposureTimeUs, control.expManual.sensitivityIso);
                        }
                        const uint8_t aeMode = ANDROID_CONTROL_AE_MODE_OFF;
                        auto res = curSettingsNext.update(ANDROID_CONTROL_AE_MODE, &aeMode, 1);
                        DAI_CHECK_IN(res == 0);
                        const int32_t priority = 0;
                        res = curSettingsNext.update(HalUtil::getVendorTag("org.codeaurora.qcamera3.iso_exp_priority.select_priority"), &priority, 1);
                        DAI_CHECK_IN(res == 0);
                        const int32_t isoValue = control.expManual.sensitivityIso;
                        res = curSettingsNext.update(HalUtil::getVendorTag("org.codeaurora.qcamera3.iso_exp_priority.use_iso_value"), &isoValue, 1);
                        DAI_CHECK_IN(res == 0);
                        const int64_t isoMode{8};  // manual iso mode
                        res = curSettingsNext.update(HalUtil::getVendorTag("org.codeaurora.qcamera3.iso_exp_priority.use_iso_exp_priority"), &isoMode, 1);
                        DAI_CHECK_IN(res == 0);
                        const int64_t exposureTime = control.expManual.exposureTimeUs * 1000ull;
                        res = curSettingsNext.update(ANDROID_SENSOR_EXPOSURE_TIME, &exposureTime, 1);
                        DAI_CHECK_IN(res == 0);
                    } break;
                    case dai::CameraControl::Command::AE_AUTO: {
                        const uint8_t aeMode = ANDROID_CONTROL_AE_MODE_ON;
                        auto res = curSettingsNext.update(ANDROID_CONTROL_AE_MODE, &aeMode, 1);
                        DAI_CHECK_IN(res == 0);
                        const int32_t priority = 0;
                        res = curSettingsNext.update(HalUtil::getVendorTag("org.codeaurora.qcamera3.iso_exp_priority.select_priority"), &priority, 1);
                        DAI_CHECK_IN(res == 0);
                        const int64_t isoMode{0};  // auto iso mode
                        res = curSettingsNext.update(HalUtil::getVendorTag("org.codeaurora.qcamera3.iso_exp_priority.use_iso_exp_priority"), &isoMode, 1);
                        DAI_CHECK_IN(res == 0);
                    } break;
                    case dai::CameraControl::Command::AE_LOCK: {
                        uint8_t aeLockMode = control.aeLockMode;
                        auto res = curSettingsNext.update(ANDROID_CONTROL_AE_LOCK, &aeLockMode, 1);
                        DAI_CHECK_IN(res == 0);
                    } break;
                    case dai::CameraControl::Command::AE_REGION: {
                        int32_t exposureMeteringMode{2};  // 2 = spot metering
                        auto res = curSettingsNext.update(
                            HalUtil::getVendorTag("org.codeaurora.qcamera3.exposure_metering.exposure_metering_mode"), &exposureMeteringMode, 1);

                        const auto controlAeRegions = toCamxRegion(control.aeRegion, "AutoExposure");
                        auto status = curSettingsNext.update(ANDROID_CONTROL_AE_REGIONS, controlAeRegions.data(), controlAeRegions.size());
                        if(status != 0) {
                            logger->error("Camera[{}] could not set ANDROID_CONTROL_AE_REGIONS", cameraId);
                        }
                    } break;
                    case dai::CameraControl::Command::EXPOSURE_COMPENSATION: {
                        int32_t compensation = MAP_VALUE(control.expCompensation, -9, 9, -12, 12);
                        auto res = curSettingsNext.update(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION, &compensation, 1);
                        DAI_CHECK_IN(res == 0);
                    } break;
                    case dai::CameraControl::Command::ANTIBANDING_MODE: {
                        std::vector<uint8_t> antibandingModes{ANDROID_CONTROL_AE_ANTIBANDING_MODE_OFF,
                                                              ANDROID_CONTROL_AE_ANTIBANDING_MODE_50HZ,
                                                              ANDROID_CONTROL_AE_ANTIBANDING_MODE_60HZ,
                                                              ANDROID_CONTROL_AE_ANTIBANDING_MODE_AUTO};
                        int idx = (int)control.antiBandingMode;
                        DAI_CHECK(idx >= 0 && idx < antibandingModes.size(), "Wrong ANTIBANDING_MODE");
                        uint8_t mode = antibandingModes[idx];
                        auto res = curSettingsNext.update(ANDROID_CONTROL_AE_ANTIBANDING_MODE, &mode, 1);
                        DAI_CHECK_IN(res == 0);
                    } break;
                    case dai::CameraControl::Command::WB_COLOR_TEMP: {
                        if(control.wbColorTemp <= 0) {
                            logger->error("Got CameraControl::WB_COLOR_TEMP with wbColorTemp {}. Ignoring as color temp must be more than 0.",
                                          control.wbColorTemp);
                            continue;
                        }
                        int32_t awbMode = 1;  // HAL manual WB mode: 0 disable, 1 CCT, 2 gains
                        curSettingsNext.update(HalUtil::getVendorTag("org.codeaurora.qcamera3.manualWB.partial_mwb_mode"), &awbMode, 1);
                        uint8_t awbOff = ANDROID_CONTROL_AWB_MODE_OFF;
                        curSettingsNext.update(ANDROID_CONTROL_AWB_MODE, &awbOff, 1);
                        const int32_t correlatedColorTemperature = control.wbColorTemp;
                        curSettingsNext.update(HalUtil::getVendorTag("org.codeaurora.qcamera3.manualWB.color_temperature"), &correlatedColorTemperature, 1);
                    } break;
                    case dai::CameraControl::Command::AWB_MODE: {
                        static const std::vector<uint8_t> whiteBalanceModes = {
                            ANDROID_CONTROL_AWB_MODE_OFF,
                            ANDROID_CONTROL_AWB_MODE_AUTO,
                            ANDROID_CONTROL_AWB_MODE_INCANDESCENT,
                            ANDROID_CONTROL_AWB_MODE_FLUORESCENT,
                            ANDROID_CONTROL_AWB_MODE_WARM_FLUORESCENT,
                            ANDROID_CONTROL_AWB_MODE_DAYLIGHT,
                            ANDROID_CONTROL_AWB_MODE_CLOUDY_DAYLIGHT,
                            ANDROID_CONTROL_AWB_MODE_TWILIGHT,
                            ANDROID_CONTROL_AWB_MODE_SHADE,
                        };
                        int idx = (int)control.awbMode;
                        DAI_CHECK(idx >= 0 && idx < whiteBalanceModes.size(), "Wrong AWB_MODE");
                        int32_t awbMode = 0;
                        curSettingsNext.update(HalUtil::getVendorTag("org.codeaurora.qcamera3.manualWB.partial_mwb_mode"), &awbMode, 1);
                        uint8_t mode = whiteBalanceModes[idx];
                        auto res = curSettingsNext.update(ANDROID_CONTROL_AWB_MODE, &mode, 1);
                        DAI_CHECK_IN(res == 0);
                    } break;
                    case dai::CameraControl::Command::AWB_LOCK: {
                        uint8_t awbLock = control.awbLockMode;
                        auto res = curSettingsNext.update(ANDROID_CONTROL_AWB_LOCK, &awbLock, 1);
                        DAI_CHECK_IN(res == 0);
                    } break;
                    case dai::CameraControl::Command::EFFECT_MODE: {
                        static const std::vector<uint8_t> effects = {ANDROID_CONTROL_EFFECT_MODE_OFF,
                                                                     ANDROID_CONTROL_EFFECT_MODE_MONO,
                                                                     ANDROID_CONTROL_EFFECT_MODE_NEGATIVE,
                                                                     ANDROID_CONTROL_EFFECT_MODE_SOLARIZE,
                                                                     ANDROID_CONTROL_EFFECT_MODE_SEPIA,
                                                                     ANDROID_CONTROL_EFFECT_MODE_POSTERIZE,
                                                                     ANDROID_CONTROL_EFFECT_MODE_WHITEBOARD,
                                                                     ANDROID_CONTROL_EFFECT_MODE_BLACKBOARD,
                                                                     ANDROID_CONTROL_EFFECT_MODE_AQUA};
                        int idx = (int)control.effectMode;
                        DAI_CHECK(idx >= 0 && idx < effects.size(), "Wrong EFFECT_MODE");
                        uint8_t effect = effects[idx];
                        auto res = curSettingsNext.update(ANDROID_CONTROL_EFFECT_MODE, &effect, 1);
                        DAI_CHECK_IN(res == 0);
                    } break;
                    case dai::CameraControl::Command::CONTRAST: {
                        const int32_t contrast = MAP_VALUE(control.contrast, -10, 10, 1, 10);
                        auto res = curSettingsNext.update(HalUtil::getVendorTag("org.codeaurora.qcamera3.contrast.level"), &contrast, 1);
                        DAI_CHECK_IN(res == 0);
                    } break;
                    case dai::CameraControl::Command::AF_TRIGGER: {
                        logger->info("Camera[{}] AF trigger", cameraId);
                        const uint8_t afTrigger = ANDROID_CONTROL_AF_TRIGGER_START;
                        auto res = curSettingsNext.update(ANDROID_CONTROL_AF_TRIGGER, &afTrigger, 1);
                        DAI_CHECK_IN(res == 0);
                    } break;
                    case dai::CameraControl::Command::AF_REGION: {
                        const auto controlAfRegions = toCamxRegion(control.afRegion, "AutoFocus");  // Leave AF mode: regions apply in all modes but OFF
                        auto res = curSettingsNext.update(ANDROID_CONTROL_AF_REGIONS, controlAfRegions.data(), controlAfRegions.size());
                        DAI_CHECK_IN(res == 0);
                    } break;
                    case dai::CameraControl::Command::MOVE_LENS:
                    case dai::CameraControl::Command::MOVE_LENS_RAW: {
                        float rawLensPosition{0.0};
                        if(cmd == dai::CameraControl::Command::MOVE_LENS) {
                            rawLensPosition = (float)control.lensPosition / 255.0f;
                            printf("=== ColorCamera: manual focus: %d -> raw: %f\n", control.lensPosition, rawLensPosition);
                        } else {
                            printf("=== ColorCamera: manual focus RAW variant: %f\n", control.lensPositionRaw);
                            rawLensPosition = control.lensPositionRaw;
                        }
                        lux_sc_camera_lens_pos_set_t msg;
                        msg.camera_id = cameraId;
                        msg.lens_pos = rawLensPosition;
                        auto cb = LuxonisSidechannel::getInstance().ops.camera_lens_pos_set;
                        if(cb) {
                            cb(msg);
                        } else {
                            LuxonisSidechannel::getInstance().sendMessage(msg);
                        }
                        float tag_value = isEvenFocusApply ? 0.4 : 0.6;  // Alternating 0.4/0.6 forces AF to write the actuator, which we intercept
                        auto res = curSettingsNext.update(ANDROID_LENS_FOCUS_DISTANCE, &tag_value, 1);
                        DAI_CHECK_IN(res == 0);
                        uint8_t focusMode = ANDROID_CONTROL_AF_MODE_OFF;
                        res = curSettingsNext.update(ANDROID_CONTROL_AF_MODE, &focusMode, 1);
                        DAI_CHECK_IN(res == 0);
                        isEvenFocusApply = !isEvenFocusApply;
                    } break;
                    
                    case dai::CameraControl::Command::FRAME_SYNC: {
                        if(!firstControlApply) {
                            logger->warn("Runtime PTP Frame sync switching is not supported, please use Camera::initialControl to change frame sync settings");
                        }
                    } break;

                    case dai::CameraControl::Command::START_STREAM: {
                        streamEnabled.store(true);
                    } break;

                    case dai::CameraControl::Command::STOP_STREAM: {
                        streamEnabled.store(false);
                    } break;

                    case dai::CameraControl::Command::SHARPNESS: {
                        int32_t sharpness = MAP_VALUE(control.sharpness, 0, 4, 0, 6);
                        auto res = curSettingsNext.update(HalUtil::getVendorTag("org.codeaurora.qcamera3.sharpness.strength"), &sharpness, 1);
                        DAI_CHECK_IN(res == 0);
                    } break;

                    case dai::CameraControl::Command::SATURATION: {
                        int32_t saturation = MAP_VALUE(control.saturation, -10, 10, 0, 10);
                        auto res = curSettingsNext.update(HalUtil::getVendorTag("org.codeaurora.qcamera3.saturation.use_saturation"), &saturation, 1);
                        DAI_CHECK_IN(res == 0);
                    } break;

                    case dai::CameraControl::Command::AF_MODE: {
                        const int continuousMode = hfrModeEnabled ? ANDROID_CONTROL_AF_MODE_CONTINUOUS_VIDEO : ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE;
                        int af_mode_lookup[] = {ANDROID_CONTROL_AF_MODE_OFF,
                                                ANDROID_CONTROL_AF_MODE_AUTO,
                                                ANDROID_CONTROL_AF_MODE_MACRO,
                                                continuousMode,
                                                continuousMode,
                                                ANDROID_CONTROL_AF_MODE_EDOF};
                        if((int)control.autoFocusMode != 0) {
                            lux_sc_camera_lens_pos_set_t msg;
                            msg.camera_id = cameraId;
                            msg.lens_pos = -1.0f;
                            auto cb = LuxonisSidechannel::getInstance().ops.camera_lens_pos_set;
                            if(cb) {
                                cb(msg);
                            } else {
                                LuxonisSidechannel::getInstance().sendMessage(msg);
                            }
                        }
                        uint8_t mode = af_mode_lookup[(int)control.autoFocusMode % 6];
                        auto res = curSettingsNext.update(ANDROID_CONTROL_AF_MODE, &mode, 1);
                        DAI_CHECK_IN(res == 0);
                    } break;

                    case dai::CameraControl::Command::HDR: {
                        if(!firstControlApply) {  // Reconfigure with HDR on/off randomly crashes ov9282 (fsync during reconfigure)
                            logger->warn("Runtime HDR switching is not supported, please use Camera::initialControl to enable HDR");
                        }
                    } break;

                    case dai::CameraControl::Command::SCENE_MODE: {
                        int scene_lookup[] = {
                            ANDROID_CONTROL_SCENE_MODE_DISABLED,
                            ANDROID_CONTROL_SCENE_MODE_FACE_PRIORITY,
                            ANDROID_CONTROL_SCENE_MODE_ACTION,
                            ANDROID_CONTROL_SCENE_MODE_PORTRAIT,
                            ANDROID_CONTROL_SCENE_MODE_LANDSCAPE,
                            ANDROID_CONTROL_SCENE_MODE_NIGHT,
                            ANDROID_CONTROL_SCENE_MODE_NIGHT_PORTRAIT,
                            ANDROID_CONTROL_SCENE_MODE_THEATRE,
                            ANDROID_CONTROL_SCENE_MODE_BEACH,
                            ANDROID_CONTROL_SCENE_MODE_SNOW,
                            ANDROID_CONTROL_SCENE_MODE_SUNSET,
                            ANDROID_CONTROL_SCENE_MODE_STEADYPHOTO,
                            ANDROID_CONTROL_SCENE_MODE_FIREWORKS,
                            ANDROID_CONTROL_SCENE_MODE_SPORTS,
                            ANDROID_CONTROL_SCENE_MODE_PARTY,
                            ANDROID_CONTROL_SCENE_MODE_CANDLELIGHT,
                            ANDROID_CONTROL_SCENE_MODE_HDR,
                        };
                        uint8_t mode = scene_lookup[(int)control.sceneMode % 17];
                        auto res = curSettingsNext.update(ANDROID_CONTROL_SCENE_MODE, &mode, 1);
                        DAI_CHECK_IN(res == 0);
                    } break;

                    case dai::CameraControl::Command::LUMA_DENOISE:
                    case dai::CameraControl::Command::CHROMA_DENOISE: {
                        int noiseLookup[] = {ANDROID_NOISE_REDUCTION_MODE_OFF, ANDROID_NOISE_REDUCTION_MODE_FAST, ANDROID_NOISE_REDUCTION_MODE_HIGH_QUALITY};
                        uint8_t mode = noiseLookup[(int)MAP_VALUE(MAX(control.chromaDenoise, control.lumaDenoise), 0, 4, 0, 2)];
                        auto res = curSettingsNext.update(ANDROID_NOISE_REDUCTION_MODE, &mode, 1);
                        DAI_CHECK_IN(res == 0);
                    } break;

                    case dai::CameraControl::Command::AE_TARGET_FPS_RANGE: {
                        int64_t exposureTimeLimit = control.aeMaxExposureTimeUs * 1000;
                        auto res = curSettingsNext.update(
                            HalUtil::getVendorTag("org.codeaurora.qcamera3.exposureEVSimulation.previewExposureTimeLimit"), &exposureTimeLimit, 1);
                        DAI_CHECK_IN(res == 0);
                    } break;
                    
                    case dai::CameraControl::Command::AE_MAX_ISO: {
                        int32_t previewISOLimit[2] = {0, control.aeMaxISO};
                        auto res = curSettingsNext.update(
                            HalUtil::getVendorTag("org.codeaurora.qcamera3.exposureEVSimulation.previewISOLimit"), previewISOLimit, 2);
                        DAI_CHECK_IN(res == 0);
                    } break;

                    // TODO handle more controls, check MX processCameraControls
                    default:
                        std::cout << "Apply camera failed unknown control type" << std::endl;
                        break;
                }
            } catch(const std::exception& exception) {
                logger->error("Exception in applyCameraControl(): {}", exception.what());
            } catch(const std::string& exception) {
                logger->error("Exception in applyCameraControl(): {}", exception);
            } catch(...) {
                logger->error("Exception in applyCameraControl(): Unknown type of exception");
            }
        }
    }

    for(const auto& pairKV : control.miscControls) {
        const std::string name = std::get<0>(pairKV).c_str();
        const std::string value = std::get<1>(pairKV).c_str();
        int32_t sensor_register_control[32] = {0};

        if(name == "beta-full-ife" || name == "beta-shared-isp") {
            continue;  // Consumed at camera open for explicit ISP routing
        }

        std::cout << "Misc Control: [" << name << " : " << value << "]" << std::endl;
        if(dai::gate::getOSVersion() < dai::Version(1, 19, 0)) {
            std::cout << "Unsupported misc control command: " << name << std::endl
                      << "Please update your operating system to version 1.19.0 or newer for HDR control" << std::endl
                      << std::flush;

            break;
        }

        if(name == "hdr-exposure-ratio") {
            float v = std::stof(value);
            if(v <= 1) {
                printf("ERROR: hdr-exposure-ratio should be set higher than 1 to enable HDR\n");
            }
            sensor_register_control[0] = HDR_EXPOSURE_RATIO_COMMAND;
            sensor_register_control[1] = static_cast<int32_t>((v * 65536));
        } else if(name == "hdr-local-tone-weight") {
            float v = std::stof(value);
            sensor_register_control[0] = HDR_LOCAL_TONE_WEIGHT_COMMAND;
            sensor_register_control[1] = static_cast<int32_t>((v * 65536));
        } else {
            std::cout << "Unknown misc control command: " << name << std::endl << std::flush;
            break;
        }

        lux_sc_sensor_register_control_t msg;
        msg.camera_id = cameraId;
        std::memcpy(msg.sensor_register_control, sensor_register_control, sizeof(msg.sensor_register_control));
        auto cb = LuxonisSidechannel::getInstance().ops.sensor_register_control;
        if(cb) {
            cb(msg);
        } else {
            LuxonisSidechannel::getInstance().sendMessage(msg);
        }
    }
    firstControlApply = false;
}

void CameraSensor::processCaptureResultCb(const struct camera3_callback_ops* ops, const camera3_capture_result_t* result) {
#ifdef ENABLE_TIMING_DEBUG
    auto startTime = std::chrono::high_resolution_clock::now();
#endif
    auto* thisPtr = const_cast<CameraSensor*>(static_cast<const CameraSensor*>(ops));
    thisPtr->processCaptureResult(result);
#ifdef ENABLE_TIMING_DEBUG
    auto endTime = std::chrono::high_resolution_clock::now();
    logger->trace("process capture took: {}ms", std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() / 1000.0f);
#endif
}  // Must take < 5 ms: the HAL aborts at 4+ frame intervals (camera3.h process_capture_result)

void CameraSensor::notifyCb(const struct camera3_callback_ops* ops, const camera3_notify_msg_t* msg) {
    auto* thisPtr = const_cast<CameraSensor*>(static_cast<const CameraSensor*>(ops));
    thisPtr->handleNotify(msg);
}

camera3_buffer_request_status_t CameraSensor::requestStreamBuffersCb(const struct camera3_callback_ops* ops,
                                                                     uint32_t num_buffer_reqs,
                                                                     const camera3_buffer_request_t* buffer_reqs,
                                                                     uint32_t* num_returned_buf_reqs,
                                                                     camera3_stream_buffer_ret_t* returned_buf_reqs) {
    printf("ERROR: HAL request_stream_buffers() called!!!\n");
    return CAMERA3_BUF_REQ_FAILED_UNKNOWN;
}

void CameraSensor::returnStreamBuffersCb(const struct camera3_callback_ops* ops, uint32_t num_buffers, const camera3_stream_buffer_t* const* buffers) {
    printf("ERROR: HAL return_stream_buffers() called!!!\n");
}

void CameraSensor::processCaptureResult(const camera3_capture_result_t* result) {
#ifdef ENABLE_TIMING_DEBUG
    const auto startTime = std::chrono::high_resolution_clock::now();
#endif
    logger->trace("Process capture result: camid: {}, frame: {}, num_out_buffers: {}, partial_result: {} START",
                  cameraId,
                  result->frame_number,
                  result->num_output_buffers,
                  result->partial_result);
    try {
        if(result->partial_result == 2 || (result->partial_result == 0 && result->num_output_buffers > 0 && result->result)) {
            bool warnMissing = !hfrModeEnabled && !hasRaw;
            int64_t timestamp{0};  // Some meta is only in partial_result == 1; take it from there when needed
            if(!HalUtil::getMetaInt64(result->result, ANDROID_SENSOR_TIMESTAMP, timestamp)) {
                logger->warn("seqNum: {}, Couldn't find ANDROID_SENSOR_TIMESTAMP meta on frame meta. Setting frame timestamp to 0.", result->frame_number);
            }
            int64_t exposure{0};
            const bool hasExposure = HalUtil::getMetaInt64(result->result, ANDROID_SENSOR_EXPOSURE_TIME, exposure);
            if(!hasExposure) {
                if(warnMissing) {
                    logger->warn("seqNum: {}, Couldn't find ANDROID_SENSOR_EXPOSURE_TIME meta on frame meta. Setting exposure time to 0.",
                                 result->frame_number);
                }
            }
            int32_t iso{0};
            if(!HalUtil::getMetaInt32(result->result, ANDROID_SENSOR_SENSITIVITY, iso)) {
                if(warnMissing) {
                    logger->warn("seqNum: {}, Couldn't find ANDROID_SENSOR_SENSITIVITY meta on frame meta. Setting iso to 0.", result->frame_number);
                }
            }
            float lensPosition{0.0f};
            if(!HalUtil::getMetaFloat(result->result, ANDROID_LENS_FOCUS_DISTANCE, lensPosition)) {
                if(warnMissing) {
                    logger->warn("seqNum: {}, Couldn't find ANDROID_LENS_FOCUS_DISTANCE meta on frame meta. Setting lens distance time to 0.",
                                 result->frame_number);
                }
            }
            int32_t correlatedColorTemperature{0};
            if(!HalUtil::getMetaInt32(result->result, HalUtil::getVendorTag("org.quic.camera2.statsconfigs.AWBFrameControlCCT"), correlatedColorTemperature)) {
                correlatedColorTemperature = static_cast<int32_t>(std::round(NEUTRAL_KELVINS));
                if(warnMissing) {
                    logger->trace(
                        "seqNum: {}, Couldn't find org.quic.camera2.statsconfigs.AWBFrameControlCCT meta on frame meta. "
                        "Setting correlatedColorTemperature to {}.",
                        result->frame_number,
                        correlatedColorTemperature);
                }
            }
            std::optional<float> sensorTemperatureC = std::nullopt;
            static const auto sensorTemperatureTag =
                HalUtil::tryGetVendorTag("org.codeaurora.qcamera3.sensorreadoutput.SensorReadOutput");
            if(sensorTemperatureTag.has_value()) {
                float sensorTemperature = 0.0f;
                if(HalUtil::getMetaSensorTemperature(result->result, *sensorTemperatureTag, sensorTemperature)) {
                    sensorTemperatureC = sensorTemperature;
                } else if(warnMissing) {
                    logger->trace(
                        "seqNum: {}, Couldn't find org.codeaurora.qcamera3.sensorreadoutput.SensorReadOutput meta on frame meta.",
                        result->frame_number);
                }
            }
#ifndef DEBUG_DISABLE_META
            MetaCacheItem item{};
            item.exposureTime = exposure;
            item.sequenceNumber = result->frame_number;
            item.timestamp = timestamp;
            item.lensPosition = lensPosition;
            item.correlatedColorTemperature = correlatedColorTemperature;
            item.sensorTemperatureC = sensorTemperatureC;
            item.iso = iso;
            item.hasSettings = hasExposure;
            recordSensorTelemetryMetrics(static_cast<int>(cameraDescription.camera.socket),
                                         cameraDescription.camera.sensorName,
                                         static_cast<double>(exposure) / 1000.0,
                                         static_cast<double>(iso),
                                         static_cast<double>(std::round(lensPosition * 255.0f)));
            {
                std::lock_guard lock(metaCacheMutex);
                if(item.hasSettings) {  // HFR: the HAL sends a batch's settings only with its last frame; earlier frames take them
                    for(uint64_t back = 1; back < hfrBatchSize && back <= item.sequenceNumber; ++back) {
                        const auto earlier =
                            std::find_if(metaCache.begin(), metaCache.end(), [sequenceNumber = item.sequenceNumber - back](const auto& cached) {
                                return cached.sequenceNumber == sequenceNumber;
                            });
                        if(earlier == metaCache.end() || earlier->hasSettings) {
                            break;
                        }
                        earlier->exposureTime = item.exposureTime;
                        earlier->iso = item.iso;
                        earlier->lensPosition = item.lensPosition;
                        earlier->correlatedColorTemperature = item.correlatedColorTemperature;
                        earlier->hasSettings = true;
                    }
                }
                metaCache.emplace_back(item);
            }
#endif
        }
    } catch(const std::exception& exception) {
        logger->error("Exception in processCaptureResult(): {}", exception.what());
        for(int idx = 0; idx < result->num_output_buffers; ++idx) {
            auto* outBuf = &result->output_buffers[idx];
            cameraPool->returnBuffer(outBuf);
        }
        return;
    } catch(const std::string& exception) {
        logger->error("Exception in processCaptureResult(): {}", exception);
        for(int idx = 0; idx < result->num_output_buffers; ++idx) {
            auto* outBuf = &result->output_buffers[idx];
            cameraPool->returnBuffer(outBuf);
        }
        return;
    } catch(...) {
        logger->error("Exception in processCaptureResult(): Unknown type of exception");
        for(int idx = 0; idx < result->num_output_buffers; ++idx) {
            auto* outBuf = &result->output_buffers[idx];
            cameraPool->returnBuffer(outBuf);
        }
        return;
    }
    if(result->num_output_buffers > 0) {
        for(int idx = 0; idx < result->num_output_buffers; ++idx) {
            auto* outBuf = &result->output_buffers[idx];
            logger->trace("Fence was: acquire: {}, release: {}", outBuf->acquire_fence, outBuf->release_fence);
            if(outBuf->status != CAMERA3_BUFFER_STATUS_OK) {
                logger->warn("Buffer for frame {} returned with error status {}, dropping", result->frame_number, static_cast<int>(outBuf->status));
                cameraPool->returnBuffer(outBuf);
                continue;
            }
            std::function<void(const camera3_stream_buffer_t*, const camera3_capture_result_t*)> cb;
            {
                std::lock_guard lock(streamsToCallbacksLock);
                auto it = streamsToCallbacks.find(outBuf->stream);
                DAI_CHECK_IN(it != streamsToCallbacks.end());
                cb = it->second;
            }
            cb(outBuf, result);
        }
    }
    logger->trace("Process capture result: camid: {}, frame: {}, num_out_buffers: {}, partial_result: {} END",
                  cameraId,
                  result->frame_number,
                  result->num_output_buffers,
                  result->partial_result);
#ifdef ENABLE_TIMING_DEBUG
    const auto endTime = std::chrono::high_resolution_clock::now();
    logger->trace("CameraSensor::processCaptureResult() took: %fms",
                  std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() / 1000.0f);
#endif
}

void CameraSensor::handleNotify(const camera3_notify_msg_t* msg) {
    logger->trace("Notify called");
    if(msg->type == CAMERA3_MSG_SHUTTER) {
        const auto shut = msg->message.shutter;
        logger->trace("Notify SHUTTER: frame_number: {}, timestamp: {}, readoutTimestamp: {}", shut.frame_number, shut.timestamp, shut.readoutTimestamp);
    } else if(msg->type == CAMERA3_MSG_ERROR) {
        const auto err = msg->message.error;
        logger->warn("Notify ERROR: code={} ({}), stream={:p}, frame_number={}",
                     err.error_code,
                     err.error_code == CAMERA3_MSG_ERROR_DEVICE    ? "DEVICE"
                     : err.error_code == CAMERA3_MSG_ERROR_REQUEST ? "REQUEST"
                     : err.error_code == CAMERA3_MSG_ERROR_RESULT  ? "RESULT"
                     : err.error_code == CAMERA3_MSG_ERROR_BUFFER  ? "BUFFER"
                                                                   : "?",
                     static_cast<void*>(err.error_stream),
                     err.frame_number);
    } else {
        printf("Notify OTHER");
    }
}

void CameraSensor::setTuningOverride(const std::string& tuningPath) {
    tuningOverride = tuningPath;
}

void CameraSensor::applyTuningFile(const std::string& tuningPath) {
    lux_sc_camera_tuning_update_t msg{};
    logger->info("Applying tuning file for: {} -> {}", cameraDescription.camera.sensorName, tuningPath);
    msg.camera_id = cameraId;
    DAI_CHECK_IN(tuningPath.size() < FILENAME_MAX);
    memcpy(msg.path, tuningPath.c_str(), tuningPath.size() + 1);
    auto cb = LuxonisSidechannel::getInstance().ops.camera_tuning_update;
    if(cb) {
        cb(msg);
    } else {
        LuxonisSidechannel::getInstance().sendMessage(msg);
    }
}

void CameraSensor::loadCorrectTuning() {
    if(tuningOverride.has_value()) {
        applyTuningFile(*tuningOverride);
        return;
    }
    if(!eepromData.has_value()) {
        logger->warn("Couldn't find EEPROM. Tuning might be off.");
        return;
    }
    if(dai::gate::getOSVersion() < dai::Version(1, 20, 4)) {
        logger->info("Old OS version doesn't override tuning");
    } else {
        if(eepromData->hardwareConf.size() < 7) {
            logger->error("Wrong HW config in eeprom: \"{}\"", eepromData->hardwareConf);
            return;
        }
        auto fovStr = eepromData->hardwareConf.substr(3, 4);
        std::string tuningPath{"/usr/lib/camera/com.qti.tuned.luxonis_" + toLowerCase(cameraDescription.camera.sensorName) + "_" + fovStr + ".bin"};
        if(access(tuningPath.c_str(), F_OK) == 0) {
            logger->info("Found tuning override for: {} -> {}", cameraDescription.camera.sensorName, tuningPath);
            applyTuningFile(tuningPath);
        } else {
            logger->info("No tuning override found for: {} -> {}", cameraDescription.camera.sensorName, tuningPath);
        }
    }
}

}  // namespace gate
}  // namespace dai

#undef DEBUG_DISABLE_META
