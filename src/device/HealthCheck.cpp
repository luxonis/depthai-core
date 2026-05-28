#include "depthai/device/HealthCheck.hpp"

#include <XLink/XLinkPublicDefines.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include "common/UsbSpeed.hpp"
#include "depthai/common/StereoPair.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/InputQueue.hpp"
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

constexpr std::uint32_t BANDWIDTH_TEST_BYTES = 1024 * 1024;
constexpr std::uint32_t BANDWIDTH_REPORT_MESSAGES = 128;
constexpr std::uint32_t IMU_REPORT_RATE = 400;
constexpr std::uint32_t IMU_BATCH_REPORT_THRESHOLD = 10;
constexpr std::uint32_t IMU_MAX_BATCH_REPORTS = 10;
constexpr float MAX_POWER_IR_INTENSITY = 1.0f;
constexpr auto POWER_SUPPLY_POLL_INTERVAL = std::chrono::milliseconds(100);

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

void setWarning(HealthCheckMetrics& metrics, const std::string& key, const std::string& message) {
    logger::warn("Health check warning at {}: {}", key, message);
    metrics.warnings[key] = message;
}

void setError(HealthCheckMetrics& metrics, const std::string& key, const std::string& message) {
    logger::warn("Health check failed at {}: {}", key, message);
    metrics.errors[key] = message;
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
            setWarning(metrics, "powerSupply", "No IR drivers detected; power supply check ran without IR load.");
            return;
        }

        bool laserEnabled = false;
        bool floodEnabled = false;
        try {
            laserEnabled = device->setIrLaserDotProjectorIntensity(MAX_POWER_IR_INTENSITY);
        } catch(const std::exception& ex) {
            setWarning(metrics, "powerSupplyIrLaser", std::string("Failed to enable IR laser dot projector: ") + ex.what());
        }
        try {
            floodEnabled = device->setIrFloodLightIntensity(MAX_POWER_IR_INTENSITY);
        } catch(const std::exception& ex) {
            setWarning(metrics, "powerSupplyIrFlood", std::string("Failed to enable IR flood light: ") + ex.what());
        }
        guard.enabled = laserEnabled || floodEnabled;

        if(!guard.enabled) {
            metrics.powerSupplyFunctionality = false;
            setError(metrics, "powerSupply", "Failed to enable IR laser/flood drivers.");
        } else if(!laserEnabled || !floodEnabled) {
            setWarning(metrics, "powerSupply", "Only one IR driver accepted maximum intensity.");
        }
    } catch(const std::exception& ex) {
        metrics.powerSupplyFunctionality = false;
        setError(metrics, "powerSupply", ex.what());
    }
}

bool waitForPowerSupplyLoad(const std::shared_ptr<Device>& device, std::chrono::steady_clock::time_point deadline, HealthCheckMetrics& metrics) {
    while(std::chrono::steady_clock::now() < deadline) {
        try {
            if(device->hasCrashed()) {
                setError(metrics, "powerSupply", "Device crashed during power supply load.");
                return false;
            }
            if(device->isClosed()) {
                setError(metrics, "powerSupply", "Device connection closed during power supply load.");
                return false;
            }
        } catch(const std::exception& ex) {
            setError(metrics, "powerSupply", ex.what());
            return false;
        }

        const auto now = std::chrono::steady_clock::now();
        if(now >= deadline) break;
        std::this_thread::sleep_for(std::min(POWER_SUPPLY_POLL_INTERVAL, std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)));
    }

    try {
        if(device->hasCrashed()) {
            setError(metrics, "powerSupply", "Device crashed during power supply load.");
            return false;
        }
        if(device->isClosed()) {
            setError(metrics, "powerSupply", "Device connection closed during power supply load.");
            return false;
        }
    } catch(const std::exception& ex) {
        setError(metrics, "powerSupply", ex.what());
        return false;
    }

    return true;
}

void setRequestedChecksFailed(HealthCheckMetrics& metrics, const HealthCheckConfig& config) {
    logger::warn("Requested health check diagnostics will be marked failed because a prerequisite failed.");

    if(config.checkUsbSpeed) {
        metrics.usbSpeed = UsbSpeed::UNKNOWN;
        metrics.usbGeneration = UsbGeneration::UNKNOWN;
    }
    if(config.measureBandwidth) {
        metrics.bandwidthMbps = 0.0f;
    }
    if(config.verifyCameras) {
        metrics.cameraFunctionality = false;
        metrics.cameraCalibration = false;
    }
    if(config.verifyIMU) {
        metrics.imuFunctionality = false;
        metrics.imuCalibration = false;
    }
    if(config.verifyPowerSupply) {
        metrics.powerSupplyFunctionality = false;
    }
}

bool performDeviceAvailabilityCheck(const std::shared_ptr<Device>& device, const HealthCheckConfig& config, HealthCheckMetrics& metrics) {
    try {
        if(device->isPipelineRunning()) {
            metrics.appRunningOnDevice = true;
            setError(metrics, "deviceAvailability", "Pipeline is already running on the device.");
            setRequestedChecksFailed(metrics, config);
            return false;
        }
    } catch(const std::exception& ex) {
        setError(metrics, "deviceAvailability", ex.what());
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
        setError(metrics, "cameraCalibration", "No connected cameras detected.");
        return false;
    }

    if(!calibration) {
        setError(metrics, "cameraCalibration", "No readable calibration containing camera parameters.");
        return false;
    }

    try {
        CalibrationHandler validatedCalibration(calibration->getEepromData(), true);
        for(auto socket : connectedCameras) {
            if(!validatedCalibration.hasCameraCalibration(socket)) {
                setError(metrics, "cameraCalibration", "Missing calibration for camera socket " + socketToString(socket) + ".");
                return false;
            }

            const auto intrinsics = validatedCalibration.getDefaultIntrinsics(socket);
            const auto& matrix = std::get<0>(intrinsics);
            if(!isFiniteMatrix(matrix, 3, 3)) {
                setError(metrics, "cameraCalibration", "Invalid intrinsics for camera socket " + socketToString(socket) + ".");
                return false;
            }
        }

        if(!deviceStereoPairs.empty() && !hasAvailableUserCalibratedStereoPair(connectedCameras, deviceStereoPairs, validatedCalibration)) {
            setError(metrics, "cameraCalibration", "No calibrated stereo pair is available.");
            return false;
        }

        return true;
    } catch(const std::exception& ex) {
        setError(metrics, "cameraCalibration", ex.what());
        return false;
    }
}

bool verifyImuCalibrationMetadata(const std::string& imuName, const std::optional<CalibrationHandler>& calibration, HealthCheckMetrics& metrics) {
    if(imuName.empty()) {
        setError(metrics, "imuCalibration", "No connected IMU detected.");
        return false;
    }

    if(!calibration) {
        setError(metrics, "imuCalibration", "No readable calibration containing IMU parameters.");
        return false;
    }

    try {
        const auto params = calibration->getImuParameters();
        if(!isFiniteMatrix(params.accelerometer, 3, 4)) {
            setError(metrics, "imuCalibration", "Invalid accelerometer calibration matrix.");
            return false;
        }
        if(!isFiniteMatrix(params.gyroscope, 3, 4)) {
            setError(metrics, "imuCalibration", "Invalid gyroscope calibration matrix.");
            return false;
        }
        return true;
    } catch(const std::exception& ex) {
        setError(metrics, "imuCalibration", ex.what());
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

    const bool shouldCreateCameraStreams =
        (config.verifyCameras && !metrics.cameraFunctionality.has_value()) || (config.verifyPowerSupply && !metrics.powerSupplyFunctionality.has_value());

    // Prepare camera streams for camera verification and power-supply load
    if(shouldCreateCameraStreams) {
        for(auto socket : connectedCameras) {
            auto camera = pipeline.create<node::Camera>()->build(socket);
            auto* output = camera->requestFullResolutionOutput();
            if(output == nullptr) {
                if(config.verifyCameras && !metrics.cameraFunctionality.has_value()) {
                    metrics.cameraFunctionality = false;
                    setError(metrics, "cameraFunctionality", "Failed to create diagnostic output for camera socket " + socketToString(socket) + ".");
                }
                if(config.verifyPowerSupply && !metrics.powerSupplyFunctionality.has_value()) {
                    metrics.powerSupplyFunctionality = false;
                    setError(metrics, "powerSupply", "Failed to create camera load for socket " + socketToString(socket) + ".");
                }
                break;
            }
            cameraQueues.emplace_back(socket, output->createOutputQueue(4, false));
        }
    }

    // Prepare IMU verification
    if(config.verifyIMU && !imuName.empty()) {
        auto imu = pipeline.create<node::IMU>();
        imu->enableIMUSensor(IMUSensor::ACCELEROMETER_RAW, IMU_REPORT_RATE);
        imu->enableIMUSensor(IMUSensor::GYROSCOPE_RAW, IMU_REPORT_RATE);
        imu->setBatchReportThreshold(IMU_BATCH_REPORT_THRESHOLD);
        imu->setMaxBatchReports(IMU_MAX_BATCH_REPORTS);
        imuQueue = imu->out.createOutputQueue(10, false);
    }

    // Prepare power supply verification
    if(config.verifyPowerSupply && !metrics.powerSupplyFunctionality.has_value() && cameraQueues.empty()) {
        metrics.powerSupplyFunctionality = false;
        setError(metrics, "powerSupply", "No camera streams were created for power supply load.");
    }
    IrPowerGuard irGuard{device, false};

    try {
        if(config.verifyPowerSupply && !metrics.powerSupplyFunctionality.has_value()) {
            enablePowerSupplyLoad(device, metrics, irGuard);
        }

        pipeline.start();

        // Verify camera streaming and keep the same streams active for power-supply validation
        if(!cameraQueues.empty() && shouldCreateCameraStreams) {
            bool allCamerasStreaming = true;
            for(auto& socketAndQueue : cameraQueues) {
                bool timedOut = false;
                auto frame = getUntil<ImgFrame>(socketAndQueue.second, std::chrono::steady_clock::now() + config.timeout, timedOut);
                if(timedOut || frame == nullptr || frame->getData().empty()) {
                    allCamerasStreaming = false;
                    if(config.verifyCameras && !metrics.cameraFunctionality.has_value()) {
                        setError(metrics, "cameraFunctionality", "Camera socket " + socketToString(socketAndQueue.first) + " did not stream a frame.");
                    }
                    if(config.verifyPowerSupply && !metrics.powerSupplyFunctionality.has_value()) {
                        metrics.powerSupplyFunctionality = false;
                        setError(metrics, "powerSupply", "Camera socket " + socketToString(socketAndQueue.first) + " did not stream during power supply load.");
                    }
                    break;
                }
            }
            if(config.verifyCameras && !metrics.cameraFunctionality.has_value()) {
                metrics.cameraFunctionality = allCamerasStreaming;
            }
        }

        // Verify IMU
        if(imuQueue) {
            bool timedOut = false;
            auto imuData = getUntil<IMUData>(imuQueue, std::chrono::steady_clock::now() + config.timeout, timedOut);
            if(timedOut || imuData == nullptr || imuData->packets.empty()) {
                metrics.imuFunctionality = false;
                setError(metrics, "imuFunctionality", "IMU did not stream packets.");
            } else {
                metrics.imuFunctionality = true;
            }
        }

        // Verify power supply
        bool sufficientPower = true;
        if(config.verifyPowerSupply && !metrics.powerSupplyFunctionality.has_value()) {
            sufficientPower = waitForPowerSupplyLoad(device, std::chrono::steady_clock::now() + config.timeout, metrics);
            if(!sufficientPower) {
                metrics.powerSupplyFunctionality = false;
            }
        }

        const bool irDisableSucceeded = irGuard.disable();
        if(config.verifyPowerSupply && !metrics.powerSupplyFunctionality.has_value() && !irDisableSucceeded) {
            metrics.powerSupplyFunctionality = false;
            setError(metrics, "powerSupply", "Failed to disable IR load after validation; device disconnected during cleanup.");
        }

        pipeline.stop();
        pipeline.wait();

        if(config.verifyPowerSupply && !metrics.powerSupplyFunctionality.has_value()) {
            metrics.powerSupplyFunctionality = true;
        }

    } catch(const std::exception& ex) {
        if(config.verifyCameras && !metrics.cameraFunctionality.has_value()) {
            metrics.cameraFunctionality = false;
            setError(metrics, "cameraFunctionality", ex.what());
        }
        if(config.verifyIMU && !metrics.imuFunctionality.has_value()) {
            metrics.imuFunctionality = false;
            setError(metrics, "imuFunctionality", ex.what());
        }
        if(config.verifyPowerSupply && !metrics.powerSupplyFunctionality.has_value()) {
            metrics.powerSupplyFunctionality = false;
            setError(metrics, "powerSupply", ex.what());
        }

        try {
            irGuard.disable();
            pipeline.stop();
            pipeline.wait();
        } catch(const std::exception& cleanupError) {
            logger::warn("Health check diagnostic pipeline cleanup failed after exception: {}", cleanupError.what());
        }
    }
}

void measureBandwidth(const std::shared_ptr<Device>& device, const HealthCheckConfig& config, HealthCheckMetrics& metrics) {
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
        frame->setSize(BANDWIDTH_TEST_BYTES, 1);
        frame->setData(std::vector<std::uint8_t>(BANDWIDTH_TEST_BYTES, 0xAA));
        bandwidthInputQueue->send(frame);

        bool timedOut = false;
        auto report = getUntil<BenchmarkReport>(bandwidthReportQueue, std::chrono::steady_clock::now() + config.timeout, timedOut);
        if(timedOut || report == nullptr || report->fps <= 0.0f) {
            metrics.bandwidthMbps = 0.0f;
            setError(metrics, "bandwidth", "Timed out waiting for bandwidth benchmark report.");
        } else {
            metrics.bandwidthMbps = report->fps * static_cast<float>(BANDWIDTH_TEST_BYTES) * 8.0f / 1000000.0f;
        }

        pipeline.stop();
        pipeline.wait();
    } catch(const std::exception& ex) {
        metrics.bandwidthMbps = 0.0f;
        setError(metrics, "bandwidth", ex.what());

        try {
            pipeline.stop();
            pipeline.wait();
        } catch(const std::exception& cleanupError) {
            logger::warn("Health check bandwidth pipeline cleanup failed after exception: {}", cleanupError.what());
        }
    }
}

HealthCheckMetrics DeviceHealthCheck::run(const DeviceInfo& devInfo, const HealthCheckConfig& config) {
    HealthCheckMetrics metrics;

    // Perform basic device info check
    metrics.appRunningOnDevice = devInfo.status == X_LINK_DEVICE_ALREADY_IN_USE;
    metrics.inSetupMode = devInfo.state == X_LINK_GATE_SETUP;
    metrics.udevRulesSet = devInfo.status != X_LINK_INSUFFICIENT_PERMISSIONS;

    // Create a device object for a health check, check its availability
    std::shared_ptr<Device> device;
    try {
        device = std::make_shared<Device>(devInfo);
    } catch(const std::exception& ex) {
        setRequestedChecksFailed(metrics, config);
        setError(metrics, "connection", ex.what());

        return metrics;
    }

    if(!performDeviceAvailabilityCheck(device, config, metrics)) {
        return metrics;
    }

    // USB speed and generation check
    if(config.checkUsbSpeed) {
        if(device->getDeviceInfo().protocol == X_LINK_TCP_IP) {
            metrics.usbSpeed = UsbSpeed::UNKNOWN;
            metrics.usbGeneration = UsbGeneration::UNKNOWN;
        } else {
            metrics.usbSpeed = device->getUsbSpeed();
            metrics.usbGeneration = usbSpeedToGeneration(*metrics.usbSpeed);
            if(metrics.usbGeneration == UsbGeneration::UNKNOWN) {
                setWarning(metrics, "usbGeneration", "Connected USB generation is unknown.");
            }
        }
    }

    // Measure the bandwidth
    if(config.measureBandwidth) {
        measureBandwidth(device, config, metrics);
        device.reset();
        try {
            device = std::make_shared<Device>(devInfo);
        } catch(const std::exception& ex) {
            metrics.bandwidthMbps = 0.0f;
            setError(metrics, "bandwidth", ex.what());
            return metrics;
        }
    }

    // Read the calibration
    std::optional<CalibrationHandler> calibration;
    if(config.verifyCameras || config.verifyIMU) {
        try {
            calibration = device->readCalibration2();
        } catch(const std::exception& ex) {
            if(config.verifyCameras) {
                metrics.cameraCalibration = false;
                setError(metrics, "cameraCalibration", std::string("No readable user calibration. User calibration error: ") + ex.what());
            }
            if(config.verifyIMU) {
                metrics.imuCalibration = false;
                setError(metrics, "imuCalibration", std::string("No readable user calibration. User calibration error: ") + ex.what());
            }
        }
    }

    // Query cameras for camera verification and power-supply load
    std::vector<CameraBoardSocket> connectedCameras;
    std::vector<StereoPair> deviceStereoPairs;
    if(config.verifyCameras || config.verifyPowerSupply) {
        try {
            connectedCameras = device->getConnectedCameras();
            if(connectedCameras.empty()) {
                if(config.verifyCameras) {
                    metrics.cameraFunctionality = false;
                    setError(metrics, "cameraFunctionality", "No connected cameras detected.");
                }
                if(config.verifyPowerSupply) {
                    metrics.powerSupplyFunctionality = false;
                    setError(metrics, "powerSupply", "No connected cameras detected for power supply load.");
                }
            }
        } catch(const std::exception& ex) {
            if(config.verifyCameras) {
                metrics.cameraFunctionality = false;
                setError(metrics, "cameraFunctionality", ex.what());
            }
            if(config.verifyPowerSupply) {
                metrics.powerSupplyFunctionality = false;
                setError(metrics, "powerSupply", ex.what());
            }
        }
    }

    // Verify camera calibration metadata
    if(config.verifyCameras) {
        try {
            deviceStereoPairs = device->getStereoPairs();
        } catch(const std::exception& ex) {
            metrics.cameraCalibration = false;
            setError(metrics, "cameraCalibration", ex.what());
        }

        if(!metrics.cameraCalibration.has_value()) {
            metrics.cameraCalibration = verifyCameraCalibrationMetadata(connectedCameras, deviceStereoPairs, calibration, metrics);
        }
    }

    // Verify IMU functionality and calibration metadata
    std::string imuName;
    if(config.verifyIMU) {
        try {
            imuName = device->getConnectedIMU();
            if(imuName.empty()) {
                metrics.imuFunctionality = false;
                setError(metrics, "imuFunctionality", "No connected IMU detected.");
            }
        } catch(const std::exception& ex) {
            metrics.imuFunctionality = false;
            setError(metrics, "imuFunctionality", ex.what());
        }

        if(!metrics.imuCalibration.has_value()) {
            metrics.imuCalibration = verifyImuCalibrationMetadata(imuName, calibration, metrics);
        }
    }

    if(config.verifyCameras || config.verifyIMU || config.verifyPowerSupply) {
        runDiagnosticPipeline(device, config, connectedCameras, imuName, metrics);
    }

    return metrics;
}

}  // namespace dai
