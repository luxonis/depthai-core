#include "depthai/device/HealthCheck.hpp"

#include <XLink/XLinkPublicDefines.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "common/UsbSpeed.hpp"
#include "depthai/common/StereoPair.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/datatype/BenchmarkReport.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/BenchmarkIn.hpp"
#include "depthai/pipeline/node/BenchmarkOut.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "pipeline/Pipeline.hpp"
#include "utility/Logging.hpp"

namespace dai {

constexpr std::uint32_t BANDWIDTH_TEST_WIDTH = 1024;
constexpr std::uint32_t BANDWIDTH_TEST_HEIGHT = 1024;
constexpr std::uint32_t BANDWIDTH_TEST_BYTES = BANDWIDTH_TEST_WIDTH * BANDWIDTH_TEST_HEIGHT;
constexpr std::uint32_t BANDWIDTH_REPORT_MESSAGES = 128;
constexpr std::uint32_t IMU_REPORT_RATE = 400;
constexpr std::uint32_t IMU_BATCH_REPORT_THRESHOLD = 10;
constexpr std::uint32_t IMU_MAX_BATCH_REPORTS = 10;
constexpr float MAX_POWER_IR_INTENSITY = 1.0f;
constexpr auto DIAGNOSTIC_OUTPUT_TIMEOUT = std::chrono::seconds(10);
constexpr auto POWER_SUPPLY_POLL_INTERVAL = std::chrono::milliseconds(100);
constexpr auto BANDWIDTH_RECONNECT_TIMEOUT = std::chrono::seconds(10);
constexpr auto BANDWIDTH_RECONNECT_POLL_INTERVAL = std::chrono::milliseconds(100);

template <typename T>
std::shared_ptr<T> getUntil(const std::shared_ptr<MessageQueue>& queue, std::chrono::steady_clock::time_point deadline, bool& timedOut) {
    const auto now = std::chrono::steady_clock::now();
    if(deadline <= now) {
        timedOut = true;
        return nullptr;
    }
    return queue->get<T>(std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now), timedOut);
}

std::string socketToString(CameraBoardSocket socket) {
    return std::to_string(static_cast<int>(socket));
}

bool isFiniteMatrix(const std::vector<std::vector<float>>& matrix, std::size_t rows, std::size_t cols) {
    if(matrix.size() != rows) return false;
    return std::all_of(matrix.begin(), matrix.end(), [cols](const std::vector<float>& row) {
        return row.size() == cols && std::all_of(row.begin(), row.end(), [](float value) { return std::isfinite(value); });
    });
}

UsbGeneration usbSpeedToGeneration(UsbSpeed speed) {
    switch(speed) {
        case UsbSpeed::LOW:
            return UsbGeneration::USB_1_0;
        case UsbSpeed::FULL:
            return UsbGeneration::USB_1_1;
        case UsbSpeed::HIGH:
            return UsbGeneration::USB_2_0;
        case UsbSpeed::SUPER:
            return UsbGeneration::USB_3_0;
        case UsbSpeed::SUPER_PLUS:
            return UsbGeneration::USB_3_1;
        case UsbSpeed::UNKNOWN:
        default:
            return UsbGeneration::UNKNOWN;
    }
}

std::string usbGenerationToString(UsbGeneration generation) {
    switch(generation) {
        case UsbGeneration::USB_1_0:
            return "USB 1.0";
        case UsbGeneration::USB_1_1:
            return "USB 1.1";
        case UsbGeneration::USB_2_0:
            return "USB 2.0";
        case UsbGeneration::USB_3_0:
            return "USB 3.0";
        case UsbGeneration::USB_3_1:
            return "USB 3.1";
        case UsbGeneration::UNKNOWN:
        default:
            return "UNKNOWN";
    }
}

std::string healthCheckResultToString(HealthCheckResult result) {
    switch(result) {
        case HealthCheckResult::PASS:
            return "pass";
        case HealthCheckResult::FAIL:
            return "fail";
        case HealthCheckResult::NOT_RUN:
        default:
            return "not run";
    }
}

std::string healthCheckIssueTypeToString(HealthCheckIssueType type) {
    switch(type) {
        case HealthCheckIssueType::Warning:
            return "warning";
        case HealthCheckIssueType::Error:
            return "error";
        default:
            return "unknown";
    }
}

std::string healthCheckIssueStageToString(HealthCheckIssueStage stage) {
    switch(stage) {
        case HealthCheckIssueStage::Connection:
            return "connection";
        case HealthCheckIssueStage::DeviceAvailability:
            return "deviceAvailability";
        case HealthCheckIssueStage::UsbGeneration:
            return "usbGeneration";
        case HealthCheckIssueStage::Bandwidth:
            return "bandwidth";
        case HealthCheckIssueStage::CameraFunctionality:
            return "cameraFunctionality";
        case HealthCheckIssueStage::CameraCalibration:
            return "cameraCalibration";
        case HealthCheckIssueStage::ImuFunctionality:
            return "imuFunctionality";
        case HealthCheckIssueStage::ImuCalibration:
            return "imuCalibration";
        case HealthCheckIssueStage::PowerSupply:
            return "powerSupply";
        default:
            return "unknown";
    }
}

std::string serializeIssues(const std::vector<HealthCheckIssue>& issues) {
    std::ostringstream stream;
    stream << "[";
    for(std::size_t i = 0; i < issues.size(); ++i) {
        if(i != 0) {
            stream << ",\n";
        }
        stream << "{type=" << healthCheckIssueTypeToString(issues[i].type) << ", stage=" << healthCheckIssueStageToString(issues[i].stage)
               << ", message=" << std::quoted(issues[i].message) << "}";
    }
    stream << "]";
    return stream.str();
}

struct IrPowerGuard {
    std::shared_ptr<Device> device;
    bool enabled = false;

    bool disable() {
        if(!enabled || !device) return true;

        bool success = true;

        try {
            device->setIrLaserDotProjectorIntensity(0.0f);
        } catch(const std::exception& ex) {
            logger::warn("Health check powerSupply warning: failed to disable IR laser dot projector: {}", ex.what());
            success = false;
        }

        try {
            device->setIrFloodLightIntensity(0.0f);
        } catch(const std::exception& ex) {
            logger::warn("Health check powerSupply warning: failed to disable IR flood light: {}", ex.what());
            success = false;
        }

        enabled = false;
        return success;
    }

    ~IrPowerGuard() {
        disable();
    }
};

void enablePowerSupplyLoad(const std::shared_ptr<Device>& device, HealthCheckMetrics& metrics, IrPowerGuard& guard) {
    try {
        const auto irDrivers = device->getIrDrivers();
        if(irDrivers.empty()) {
            metrics.issues.emplace_back(
                HealthCheckIssueType::Warning, HealthCheckIssueStage::PowerSupply, "No IR drivers detected; power supply check ran without IR load.");
            return;
        }

        bool laserEnabled = false;
        bool floodEnabled = false;
        try {
            laserEnabled = device->setIrLaserDotProjectorIntensity(MAX_POWER_IR_INTENSITY);
        } catch(const std::exception& ex) {
            metrics.issues.emplace_back(
                HealthCheckIssueType::Warning, HealthCheckIssueStage::PowerSupply, std::string("Failed to enable IR laser dot projector: ") + ex.what());
        }
        try {
            floodEnabled = device->setIrFloodLightIntensity(MAX_POWER_IR_INTENSITY);
        } catch(const std::exception& ex) {
            metrics.issues.emplace_back(
                HealthCheckIssueType::Warning, HealthCheckIssueStage::PowerSupply, std::string("Failed to enable IR flood light: ") + ex.what());
        }
        guard.enabled = laserEnabled || floodEnabled;

        if(!guard.enabled) {
            metrics.powerSupplyFunctionality = HealthCheckResult::FAIL;
            metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::PowerSupply, "Failed to enable IR laser/flood drivers.");
        } else if(!laserEnabled || !floodEnabled) {
            metrics.issues.emplace_back(HealthCheckIssueType::Warning, HealthCheckIssueStage::PowerSupply, "Only one IR driver accepted maximum intensity.");
        }
    } catch(const std::exception& ex) {
        metrics.powerSupplyFunctionality = HealthCheckResult::FAIL;
        metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::PowerSupply, ex.what());
    }
}

bool waitForPowerSupplyLoad(const std::shared_ptr<Device>& device, std::chrono::steady_clock::time_point deadline, HealthCheckMetrics& metrics) {
    const auto isDeviceAvailable = [&]() {
        try {
            if(device->hasCrashed()) {
                metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::PowerSupply, "Device crashed during power supply load.");
                return false;
            }
            if(device->isClosed()) {
                metrics.issues.emplace_back(
                    HealthCheckIssueType::Error, HealthCheckIssueStage::PowerSupply, "Device connection closed during power supply load.");
                return false;
            }
        } catch(const std::exception& ex) {
            metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::PowerSupply, ex.what());
            return false;
        }
        return true;
    };

    while(std::chrono::steady_clock::now() < deadline) {
        if(!isDeviceAvailable()) {
            return false;
        }

        const auto now = std::chrono::steady_clock::now();
        if(now >= deadline) break;
        std::this_thread::sleep_for(std::min(POWER_SUPPLY_POLL_INTERVAL, std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)));
    }

    return isDeviceAvailable();
}

void setRequestedChecksFailed(HealthCheckMetrics& metrics, const HealthCheckConfig& config) {
    if(metrics.issues.empty()) {
        logger::warn("Requested health check diagnostics will be marked failed because a prerequisite failed.");
    } else {
        logger::warn("Requested health check diagnostics will be marked failed because a prerequisite failed: {}", serializeIssues(metrics.issues));
    }

    if(config.checkUsbGeneration) {
        metrics.usbGeneration = UsbGeneration::UNKNOWN;
    }
    if(config.measureBandwidth) {
        metrics.bandwidthMbps = 0.0f;
    }
    if(config.verifyCameraFunctionality) {
        metrics.cameraFunctionality = HealthCheckResult::FAIL;
    }
    if(config.verifyCameraCalibration) {
        metrics.cameraCalibration = HealthCheckResult::FAIL;
    }
    if(config.verifyImuFunctionality) {
        metrics.imuFunctionality = HealthCheckResult::FAIL;
    }
    if(config.verifyImuCalibration) {
        metrics.imuCalibration = HealthCheckResult::FAIL;
    }
    if(config.verifyPowerSupply) {
        metrics.powerSupplyFunctionality = HealthCheckResult::FAIL;
    }
}

bool performDeviceAvailabilityCheck(const std::shared_ptr<Device>& device, const HealthCheckConfig& config, HealthCheckMetrics& metrics) {
    try {
        if(device->isPipelineRunning()) {
            metrics.deviceInUse = true;
            metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::DeviceAvailability, "Pipeline is already running on the device.");
            setRequestedChecksFailed(metrics, config);
            return false;
        }
    } catch(const std::exception& ex) {
        metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::DeviceAvailability, ex.what());
        setRequestedChecksFailed(metrics, config);
        return false;
    }
    return true;
}

bool hasAvailableUserCalibratedStereoPair(const std::vector<CameraBoardSocket>& connectedCameras,
                                          const std::vector<StereoPair>& deviceStereoPairs,
                                          const CalibrationHandler& calibration) {
    return std::any_of(deviceStereoPairs.begin(), deviceStereoPairs.end(), [&](const StereoPair& pair) {
        const auto containsSocket = [&](CameraBoardSocket socket) {
            return std::find(connectedCameras.begin(), connectedCameras.end(), socket) != connectedCameras.end();
        };

        if(!containsSocket(pair.left) || !containsSocket(pair.right)) {
            return false;
        }

        try {
            return isFiniteMatrix(calibration.getCameraExtrinsics(pair.left, pair.right), 4, 4);
        } catch(const std::exception&) {
            return false;
        }
    });
}

bool verifyCameraCalibrationMetadata(const std::vector<CameraBoardSocket>& connectedCameras,
                                     const std::vector<StereoPair>& deviceStereoPairs,
                                     const std::optional<CalibrationHandler>& calibration,
                                     HealthCheckMetrics& metrics) {
    if(connectedCameras.empty()) {
        metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::CameraCalibration, "No connected cameras detected.");
        return false;
    }

    if(!calibration) {
        metrics.issues.emplace_back(
            HealthCheckIssueType::Error, HealthCheckIssueStage::CameraCalibration, "No readable calibration containing camera parameters.");
        return false;
    }

    try {
        CalibrationHandler validatedCalibration(calibration->getEepromData(), true);
        for(auto socket : connectedCameras) {
            if(!validatedCalibration.hasCameraCalibration(socket)) {
                metrics.issues.emplace_back(HealthCheckIssueType::Error,
                                            HealthCheckIssueStage::CameraCalibration,
                                            "Missing calibration for camera socket " + socketToString(socket) + ".");
                return false;
            }

            const auto intrinsics = validatedCalibration.getDefaultIntrinsics(socket);
            const auto& matrix = std::get<0>(intrinsics);
            if(!isFiniteMatrix(matrix, 3, 3)) {
                metrics.issues.emplace_back(HealthCheckIssueType::Error,
                                            HealthCheckIssueStage::CameraCalibration,
                                            "Invalid intrinsics for camera socket " + socketToString(socket) + ".");
                return false;
            }
        }

        if(!deviceStereoPairs.empty() && !hasAvailableUserCalibratedStereoPair(connectedCameras, deviceStereoPairs, validatedCalibration)) {
            metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::CameraCalibration, "No calibrated stereo pair is available.");
            return false;
        }

        return true;
    } catch(const std::exception& ex) {
        metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::CameraCalibration, ex.what());
        return false;
    }
}

bool verifyImuCalibrationMetadata(const std::string& imuName, const std::optional<CalibrationHandler>& calibration, HealthCheckMetrics& metrics) {
    if(imuName.empty()) {
        metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::ImuCalibration, "No connected IMU detected.");
        return false;
    }

    if(!calibration) {
        metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::ImuCalibration, "No readable calibration containing IMU parameters.");
        return false;
    }

    try {
        const auto params = calibration->getImuParameters();
        if(!isFiniteMatrix(params.accelerometer, 3, 4)) {
            metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::ImuCalibration, "Invalid accelerometer calibration matrix.");
            return false;
        }
        if(!isFiniteMatrix(params.gyroscope, 3, 4)) {
            metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::ImuCalibration, "Invalid gyroscope calibration matrix.");
            return false;
        }
        return true;
    } catch(const std::exception& ex) {
        metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::ImuCalibration, ex.what());
        return false;
    }
}

void runDiagnosticPipeline(const std::shared_ptr<Device>& device,
                           const HealthCheckConfig& config,
                           const std::vector<CameraBoardSocket>& connectedCameras,
                           const std::string& imuName,
                           HealthCheckMetrics& metrics) {
    Pipeline pipeline(device);
    std::vector<std::pair<CameraBoardSocket, std::shared_ptr<MessageQueue>>> cameraQueues;
    std::shared_ptr<MessageQueue> imuQueue;

    const bool shouldCreateCameraStreams = (config.verifyCameraFunctionality && metrics.cameraFunctionality == HealthCheckResult::NOT_RUN)
                                           || (config.verifyPowerSupply && metrics.powerSupplyFunctionality == HealthCheckResult::NOT_RUN);

    // Prepare camera streams for camera verification and power-supply load
    if(shouldCreateCameraStreams) {
        for(auto socket : connectedCameras) {
            auto camera = pipeline.create<node::Camera>()->build(socket);
            auto* output = camera->requestFullResolutionOutput();
            if(output == nullptr) {
                if(config.verifyCameraFunctionality && metrics.cameraFunctionality == HealthCheckResult::NOT_RUN) {
                    metrics.cameraFunctionality = HealthCheckResult::FAIL;
                    metrics.issues.emplace_back(HealthCheckIssueType::Error,
                                                HealthCheckIssueStage::CameraFunctionality,
                                                "Failed to create diagnostic output for camera socket " + socketToString(socket) + ".");
                }
                if(config.verifyPowerSupply && metrics.powerSupplyFunctionality == HealthCheckResult::NOT_RUN) {
                    metrics.powerSupplyFunctionality = HealthCheckResult::FAIL;
                    metrics.issues.emplace_back(HealthCheckIssueType::Error,
                                                HealthCheckIssueStage::PowerSupply,
                                                "Failed to create camera load for socket " + socketToString(socket) + ".");
                }
                break;
            }
            cameraQueues.emplace_back(socket, output->createOutputQueue(4, false));
        }
    }

    // Prepare IMU verification
    if(config.verifyImuFunctionality && !imuName.empty()) {
        auto imu = pipeline.create<node::IMU>();
        imu->enableIMUSensor(IMUSensor::ACCELEROMETER_RAW, IMU_REPORT_RATE);
        imu->enableIMUSensor(IMUSensor::GYROSCOPE_RAW, IMU_REPORT_RATE);
        imu->setBatchReportThreshold(IMU_BATCH_REPORT_THRESHOLD);
        imu->setMaxBatchReports(IMU_MAX_BATCH_REPORTS);
        imuQueue = imu->out.createOutputQueue(10, false);
    }

    // Prepare power supply verification
    if(config.verifyPowerSupply && metrics.powerSupplyFunctionality == HealthCheckResult::NOT_RUN && cameraQueues.empty()) {
        metrics.powerSupplyFunctionality = HealthCheckResult::FAIL;
        metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::PowerSupply, "No camera streams were created for power supply load.");
    }
    IrPowerGuard irGuard{device, false};

    try {
        pipeline.start();

        // Verify camera streaming and keep the same streams active for power-supply validation
        if(!cameraQueues.empty() && shouldCreateCameraStreams) {
            bool allCamerasStreaming = true;
            for(auto& socketAndQueue : cameraQueues) {
                bool timedOut = false;
                auto frame = getUntil<ImgFrame>(socketAndQueue.second, std::chrono::steady_clock::now() + DIAGNOSTIC_OUTPUT_TIMEOUT, timedOut);
                if(timedOut || frame == nullptr || frame->getData().empty()) {
                    allCamerasStreaming = false;
                    if(config.verifyCameraFunctionality && metrics.cameraFunctionality == HealthCheckResult::NOT_RUN) {
                        metrics.issues.emplace_back(HealthCheckIssueType::Error,
                                                    HealthCheckIssueStage::CameraFunctionality,
                                                    "Camera socket " + socketToString(socketAndQueue.first) + " did not stream a frame.");
                    }
                    if(config.verifyPowerSupply && metrics.powerSupplyFunctionality == HealthCheckResult::NOT_RUN) {
                        metrics.powerSupplyFunctionality = HealthCheckResult::FAIL;
                        metrics.issues.emplace_back(HealthCheckIssueType::Error,
                                                    HealthCheckIssueStage::PowerSupply,
                                                    "Camera socket " + socketToString(socketAndQueue.first) + " did not stream before power supply load.");
                    }
                    break;
                }
            }
            if(config.verifyCameraFunctionality && metrics.cameraFunctionality == HealthCheckResult::NOT_RUN) {
                metrics.cameraFunctionality = allCamerasStreaming ? HealthCheckResult::PASS : HealthCheckResult::FAIL;
            }
        }

        // Verify IMU
        if(imuQueue) {
            bool timedOut = false;
            auto imuData = getUntil<IMUData>(imuQueue, std::chrono::steady_clock::now() + DIAGNOSTIC_OUTPUT_TIMEOUT, timedOut);
            if(timedOut || imuData == nullptr || imuData->packets.empty()) {
                metrics.imuFunctionality = HealthCheckResult::FAIL;
                metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::ImuFunctionality, "IMU did not stream packets.");
            } else {
                metrics.imuFunctionality = HealthCheckResult::PASS;
            }
        }

        // Verify power supply
        if(config.verifyPowerSupply && metrics.powerSupplyFunctionality == HealthCheckResult::NOT_RUN) {
            enablePowerSupplyLoad(device, metrics, irGuard);
            bool sufficientPower = waitForPowerSupplyLoad(device, std::chrono::steady_clock::now() + config.powerSupplyCheckDuration, metrics);
            if(!sufficientPower) {
                metrics.powerSupplyFunctionality = HealthCheckResult::FAIL;
            }
        }

        const bool irDisableSucceeded = irGuard.disable();
        if(config.verifyPowerSupply && metrics.powerSupplyFunctionality == HealthCheckResult::NOT_RUN && !irDisableSucceeded) {
            metrics.powerSupplyFunctionality = HealthCheckResult::FAIL;
            metrics.issues.emplace_back(HealthCheckIssueType::Error,
                                        HealthCheckIssueStage::PowerSupply,
                                        "Failed to disable IR load after validation; device disconnected during cleanup.");
        }
        if(config.verifyPowerSupply && metrics.powerSupplyFunctionality == HealthCheckResult::NOT_RUN) {
            metrics.powerSupplyFunctionality = HealthCheckResult::PASS;
        }

        pipeline.stop();
        pipeline.wait();

    } catch(const std::exception& ex) {
        if(config.verifyCameraFunctionality && metrics.cameraFunctionality == HealthCheckResult::NOT_RUN) {
            metrics.cameraFunctionality = HealthCheckResult::FAIL;
            metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::CameraFunctionality, ex.what());
        }
        if(config.verifyImuFunctionality && metrics.imuFunctionality == HealthCheckResult::NOT_RUN) {
            metrics.imuFunctionality = HealthCheckResult::FAIL;
            metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::ImuFunctionality, ex.what());
        }
        if(config.verifyPowerSupply && metrics.powerSupplyFunctionality == HealthCheckResult::NOT_RUN) {
            metrics.powerSupplyFunctionality = HealthCheckResult::FAIL;
            metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::PowerSupply, ex.what());
        }

        irGuard.disable();
        pipeline.stop();
        pipeline.wait();
    }
}

void measureBandwidth(const std::shared_ptr<Device>& device, HealthCheckMetrics& metrics) {
    Pipeline pipeline(device);

    auto benchmarkOut = pipeline.create<node::BenchmarkOut>();
    benchmarkOut->setRunOnHost(false);
    benchmarkOut->setFps(0.0f);

    auto benchmarkIn = pipeline.create<node::BenchmarkIn>();
    benchmarkIn->setRunOnHost(true);
    benchmarkIn->sendReportEveryNMessages(BANDWIDTH_REPORT_MESSAGES);
    benchmarkIn->logReportsAsWarnings(false);

    benchmarkOut->out.link(benchmarkIn->input);
    auto bandwidthInputQueue = benchmarkOut->input.createInputQueue();
    auto bandwidthReportQueue = benchmarkIn->report.createOutputQueue(2, false);

    try {
        pipeline.start();

        auto frame = std::make_shared<ImgFrame>();
        frame->setType(ImgFrame::Type::RAW8);
        frame->setSize(BANDWIDTH_TEST_WIDTH, BANDWIDTH_TEST_HEIGHT);
        frame->setData(std::vector<std::uint8_t>(BANDWIDTH_TEST_BYTES, 0xAA));
        bandwidthInputQueue->send(frame);

        bool timedOut = false;
        auto report = getUntil<BenchmarkReport>(bandwidthReportQueue, std::chrono::steady_clock::now() + DIAGNOSTIC_OUTPUT_TIMEOUT, timedOut);
        if(timedOut || report == nullptr || report->fps <= 0.0f) {
            metrics.bandwidthMbps = 0.0f;
            metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::Bandwidth, "Timed out waiting for bandwidth benchmark report.");
        } else {
            metrics.bandwidthMbps = report->fps * static_cast<float>(BANDWIDTH_TEST_BYTES) * 8.0f / 1000000.0f;
        }

        pipeline.stop();
        pipeline.wait();
    } catch(const std::exception& ex) {
        metrics.bandwidthMbps = 0.0f;
        metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::Bandwidth, ex.what());

        pipeline.stop();
        pipeline.wait();
    }
}

HealthCheckMetrics DeviceHealthCheck::run(const DeviceInfo& devInfo, const HealthCheckConfig& config) {
    HealthCheckMetrics metrics;

    // Create a device object for a health check, check its availability
    logger::info("Health check: Connecting to device and checking availability");
    std::shared_ptr<Device> device;
    try {
        device = std::make_shared<Device>(devInfo);
    } catch(const std::exception& ex) {
        const std::string errorMessage = ex.what();
        metrics.deviceInUse = errorMessage.find("X_LINK_DEVICE_ALREADY_IN_USE") != std::string::npos;
        metrics.missingUdevRules = errorMessage.find("X_LINK_INSUFFICIENT_PERMISSIONS") != std::string::npos;
        metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::Connection, errorMessage);
        setRequestedChecksFailed(metrics, config);

        return metrics;
    }

    // Perform basic device info check
    metrics.deviceInUse = device->getDeviceInfo().status == X_LINK_DEVICE_ALREADY_IN_USE;
    metrics.deviceInSetupMode = device->getDeviceInfo().state == X_LINK_GATE_SETUP;
    metrics.missingUdevRules = device->getDeviceInfo().status == X_LINK_INSUFFICIENT_PERMISSIONS;

    if(!performDeviceAvailabilityCheck(device, config, metrics)) {
        return metrics;
    }

    // USB speed and generation check
    if(config.checkUsbGeneration) {
        logger::info("Health check: Checking USB generation");
        if(device->getDeviceInfo().protocol == X_LINK_TCP_IP) {
            metrics.usbGeneration = UsbGeneration::UNKNOWN;
        } else {
            const auto usbSpeed = device->getUsbSpeed();
            metrics.usbGeneration = usbSpeedToGeneration(usbSpeed);
            if(metrics.usbGeneration == UsbGeneration::UNKNOWN) {
                metrics.issues.emplace_back(HealthCheckIssueType::Warning, HealthCheckIssueStage::UsbGeneration, "Connected USB generation is unknown.");
            }
        }
    }

    // Read the calibration
    std::optional<CalibrationHandler> calibration;
    if(config.verifyCameraCalibration || config.verifyImuCalibration) {
        logger::info("Health check: Reading calibration");
        try {
            calibration = device->getCalibration();
        } catch(const std::exception& ex) {
            if(config.verifyCameraCalibration) {
                metrics.cameraCalibration = HealthCheckResult::FAIL;
                metrics.issues.emplace_back(HealthCheckIssueType::Error,
                                            HealthCheckIssueStage::CameraCalibration,
                                            std::string("No readable user calibration. User calibration error: ") + ex.what());
            }
            if(config.verifyImuCalibration) {
                metrics.imuCalibration = HealthCheckResult::FAIL;
                metrics.issues.emplace_back(HealthCheckIssueType::Error,
                                            HealthCheckIssueStage::ImuCalibration,
                                            std::string("No readable user calibration. User calibration error: ") + ex.what());
            }
        }
    }

    // Query cameras for camera verification and power-supply load
    std::vector<CameraBoardSocket> connectedCameras;
    std::vector<StereoPair> deviceStereoPairs;
    if(config.verifyCameraFunctionality || config.verifyCameraCalibration || config.verifyPowerSupply) {
        logger::info("Health check: Querying connected cameras");
        try {
            connectedCameras = device->getConnectedCameras();
            if(connectedCameras.empty()) {
                if(config.verifyCameraFunctionality) {
                    metrics.cameraFunctionality = HealthCheckResult::FAIL;
                    metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::CameraFunctionality, "No connected cameras detected.");
                }
                if(config.verifyPowerSupply) {
                    metrics.powerSupplyFunctionality = HealthCheckResult::FAIL;
                    metrics.issues.emplace_back(
                        HealthCheckIssueType::Error, HealthCheckIssueStage::PowerSupply, "No connected cameras detected for power supply load.");
                }
            }
        } catch(const std::exception& ex) {
            if(config.verifyCameraFunctionality) {
                metrics.cameraFunctionality = HealthCheckResult::FAIL;
                metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::CameraFunctionality, ex.what());
            }
            if(config.verifyPowerSupply) {
                metrics.powerSupplyFunctionality = HealthCheckResult::FAIL;
                metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::PowerSupply, ex.what());
            }
        }
    }

    // Verify camera calibration metadata
    if(config.verifyCameraCalibration) {
        logger::info("Health check: Verifying camera calibration");
        try {
            deviceStereoPairs = device->getStereoPairs();
        } catch(const std::exception& ex) {
            metrics.cameraCalibration = HealthCheckResult::FAIL;
            metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::CameraCalibration, ex.what());
        }

        if(metrics.cameraCalibration == HealthCheckResult::NOT_RUN) {
            metrics.cameraCalibration =
                verifyCameraCalibrationMetadata(connectedCameras, deviceStereoPairs, calibration, metrics) ? HealthCheckResult::PASS : HealthCheckResult::FAIL;
        }
    }

    // Verify IMU functionality and calibration metadata
    std::string imuName;
    if(config.verifyImuFunctionality || config.verifyImuCalibration) {
        logger::info("Health check: Verifying IMU");
        try {
            imuName = device->getConnectedIMU();
            if(config.verifyImuFunctionality && imuName.empty()) {
                metrics.imuFunctionality = HealthCheckResult::FAIL;
                metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::ImuFunctionality, "No connected IMU detected.");
            }
        } catch(const std::exception& ex) {
            if(config.verifyImuFunctionality) {
                metrics.imuFunctionality = HealthCheckResult::FAIL;
                metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::ImuFunctionality, ex.what());
            }
        }

        if(config.verifyImuCalibration && metrics.imuCalibration == HealthCheckResult::NOT_RUN) {
            metrics.imuCalibration = verifyImuCalibrationMetadata(imuName, calibration, metrics) ? HealthCheckResult::PASS : HealthCheckResult::FAIL;
        }
    }

    if(config.verifyCameraFunctionality || config.verifyImuFunctionality || config.verifyPowerSupply) {
        logger::info("Health check: Running diagnostic pipeline");
        runDiagnosticPipeline(device, config, connectedCameras, imuName, metrics);
    }

    // Measure the bandwidth
    if(config.measureBandwidth) {
        logger::info("Health check: Measuring bandwidth");
        device.reset();

        std::string lastErrorMessage;
        auto deadline = std::chrono::steady_clock::now() + BANDWIDTH_RECONNECT_TIMEOUT;
        while(std::chrono::steady_clock::now() < deadline) {
            try {
                device = std::make_shared<Device>(devInfo);
                break;
            } catch(const std::exception& ex) {
                lastErrorMessage = ex.what();
                std::this_thread::sleep_for(BANDWIDTH_RECONNECT_POLL_INTERVAL);
            }
        }

        if(!device) {
            metrics.bandwidthMbps = 0.0f;
            metrics.issues.emplace_back(HealthCheckIssueType::Error, HealthCheckIssueStage::Bandwidth, lastErrorMessage);
            return metrics;
        }
        measureBandwidth(device, metrics);
    }

    return metrics;
}

std::string HealthCheckMetrics::toString() const {
    std::ostringstream stream;
    stream << std::boolalpha << "HealthCheckMetrics(\n"
           << "  usbGeneration=" << usbGenerationToString(usbGeneration) << ",\n"
           << "  bandwidthMbps=" << bandwidthMbps << ",\n"
           << "  cameraFunctionality=" << healthCheckResultToString(cameraFunctionality) << ",\n"
           << "  cameraCalibration=" << healthCheckResultToString(cameraCalibration) << ",\n"
           << "  imuFunctionality=" << healthCheckResultToString(imuFunctionality) << ",\n"
           << "  imuCalibration=" << healthCheckResultToString(imuCalibration) << ",\n"
           << "  powerSupplyFunctionality=" << healthCheckResultToString(powerSupplyFunctionality) << ",\n"
           << "  deviceInUse=" << deviceInUse << ",\n"
           << "  deviceInSetupMode=" << deviceInSetupMode << ",\n"
           << "  missingUdevRules=" << missingUdevRules << ",\n"
           << "  issues=" << serializeIssues(issues) << "\n"
           << ")";
    return stream.str();
}

}  // namespace dai
