#include "fsync_ptp_test_utils.hpp"

#include <algorithm>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/ExternalFrameSyncRoles.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/properties/SyncProperties.hpp"
#include "depthai/xlink/XLinkConnection.hpp"

#define REQUIRE_MSG(x, msg)                                         \
    if(!(x)) {                                                      \
        std::cout << "\x1B[1;31m" << msg << "\x1B[0m" << std::endl; \
        REQUIRE((x));                                               \
    }

namespace {

    struct Delta {
        std::uint64_t delta_us;
        std::string name;
    };

    double calculate_mean(std::vector<Delta> values) {
        std::uint64_t sum = std::accumulate(values.begin(), values.end(), static_cast<std::uint64_t>(0),
        [](std::uint64_t const &sum, Delta const &delta) -> std::uint64_t {
            return sum + delta.delta_us;
        });
        return double(sum) / double(values.size());
    }

    double percentile_linear(std::vector<Delta> values, double q)
    {
        if (values.empty()) {
            throw std::invalid_argument("percentile_linear: input vector must not be empty");
        }

        if (!std::isfinite(q) || q < 0.0 || q > 100.0) {
            throw std::invalid_argument("percentile_linear: q must be a finite value in [0, 100]");
        }

        std::sort(values.begin(), values.end(),
        [](Delta const &a, Delta const &b) -> bool {
            return a.delta_us < b.delta_us;
        });

        if (values.size() == 1) {
            return static_cast<double>(values[0].delta_us);
        }

        const double pos = (q / 100.0) * static_cast<double>(values.size() - 1);
        const std::size_t lower = static_cast<std::size_t>(std::floor(pos));
        const std::size_t upper = static_cast<std::size_t>(std::ceil(pos));
        const double fraction = pos - static_cast<double>(lower);

        const double lower_value = static_cast<double>(values[lower].delta_us);
        const double upper_value = static_cast<double>(values[upper].delta_us);

        return lower_value + (upper_value - lower_value) * fraction;
    }

    Delta calculate_max_outlier(std::vector<Delta>& values) {
        auto max_itr = std::max_element(values.begin(), values.end(),
        [](Delta const &a, Delta const &b) -> bool {
            return a.delta_us < b.delta_us;
        });
        
        REQUIRE_MSG(max_itr != values.end(), "Outlier not found");
        return *max_itr;
    }

    void expect_percentile(
        const std::vector<Delta>& values,
        double q,
        double expected)
    {
        using Catch::Matchers::WithinAbs;
        REQUIRE_THAT(percentile_linear(values, q), WithinAbs(expected, 1e-12));
    }

    std::string toString(dai::ImgFrame::Fsync fsync) {
        switch(fsync) {
            case dai::ImgFrame::Fsync::NONE:
                return "NONE";
            case dai::ImgFrame::Fsync::INPUT:
                return "INPUT";
            case dai::ImgFrame::Fsync::OUTPUT:
                return "OUTPUT";
            case dai::ImgFrame::Fsync::PTP:
                return "PTP";
        }
        return "UNKNOWN";
    }

    dai::ImgFrame::Fsync convertSyncType(SyncType syncType) {
        if(syncType == SyncType::EXTERNAL) {
            return dai::ImgFrame::Fsync::INPUT;
        } else if(syncType == SyncType::PTP) {
            return dai::ImgFrame::Fsync::PTP;
        } else {
            throw std::runtime_error("Unknown sync type");
        }
    }

    std::optional<std::string> getCameraSensorName(std::shared_ptr<dai::Device> device, dai::CameraBoardSocket socket) {
        auto cameraNames = device->getCameraSensorNames();
        for(auto& name : cameraNames) {
            if(name.first == socket) {
                return name.second;
            }
        }
        return std::nullopt;
    }
}

std::string toString(SyncType syncType) {
    if(syncType == SyncType::EXTERNAL) {
        return "external";
    } else if(syncType == SyncType::PTP) {
        return "PTP";
    } else {
        throw std::runtime_error("Unknown sync type");
    }
}

void setUpCameraSocket(dai::Pipeline& pipeline,
                       std::shared_ptr<dai::Device> device,
                       std::shared_ptr<dai::node::Sync> syncNode,
                       dai::CameraBoardSocket socket,
                       std::string& deviceName,
                       float targetFps,
                       SyncType syncType,
                       std::optional<dai::ExternalFrameSyncRole> role,
                       std::vector<std::string>& inputNames) {
    std::shared_ptr<dai::node::Camera> cam;
    if(syncType == SyncType::PTP || role == dai::ExternalFrameSyncRole::MASTER) {
        cam = pipeline.create<dai::node::Camera>(device)->build(socket, std::nullopt, targetFps);
    } else if(role == dai::ExternalFrameSyncRole::SLAVE) {
        cam = pipeline.create<dai::node::Camera>(device)->build(socket, std::nullopt);
    } else {
        throw std::runtime_error("Don't know how to handle role");
    }

    if(syncType == SyncType::PTP) {
        cam->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::TIME_PTP);
        std::cout << "Setting PTP for " << dai::toString(socket) << std::endl;
    }

    auto ccmName = getCameraSensorName(device, socket);
    REQUIRE_MSG(ccmName.has_value(), "Camera sensor name not found for socket " + dai::toString(socket));
    std::string fullSocketName = deviceName + "_" +dai::toString(socket) + "[" + ccmName.value() + "]";

    int width = 320;
    int height = 240;
    cam->requestOutput(std::make_pair(width, height), dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP)->link(syncNode->inputs[fullSocketName]);

    inputNames.push_back(fullSocketName);
}

void setUpIrLeds(std::shared_ptr<dai::Device> device) {
    auto drivers = device->getIrDrivers();
    bool found = false;

    for(auto& driver : drivers) {
        std::string name;
        int bus;
        int addr;
        std::tie(name, bus, addr) = driver;

        if(name == "stm-dot") {
            device->setIrLaserDotProjectorIntensity(0.1);
            found = true;
        } else if(name == "stm-flood") {
            device->setIrFloodLightIntensity(0.1);
            found = true;
        }
    }

    if(!found) {
        std::cout << "No IR drivers found, skipping IR intensity setting" << std::endl;
    } else {
        std::cout << "IR intensity set" << std::endl;
    }
}

void setupDevice(dai::DeviceInfo& deviceInfo,
                 dai::Pipeline& pipeline,
                 std::shared_ptr<dai::node::Sync> syncNode,
                 uint32_t &numMasters,
                 uint32_t &numSlaves,
                 std::vector<std::string>& inputNames,
                 float targetFps,
                 SyncType syncType,
                 std::optional<std::set<std::string>> &allowedSensors) {
    auto device = pipeline.addDevice(deviceInfo);

    if(device->getPlatform() != dai::Platform::RVC4) {
        throw std::runtime_error("This test supports only RVC4 platform!");
    }

    std::string name = deviceInfo.getXLinkDeviceDesc().name;
    std::optional<dai::ExternalFrameSyncRole> role = std::nullopt;
    if(syncType == SyncType::EXTERNAL) {
        role = device->getExternalFrameSyncRole();
    }

    std::cout << "=== Connected to " << deviceInfo.getDeviceId() << std::endl;
    std::cout << "    Device ID: " << device->getDeviceId() << std::endl;
    std::cout << "    Num of cameras: " << device->getConnectedCameras().size() << std::endl;

    auto isSensorAllowed = [&](dai::CameraBoardSocket socket) -> bool {
        if (!allowedSensors.has_value()) {
            return true;
        }

        auto sensorNames = device->getCameraSensorNames();
        if(sensorNames.find(socket) == sensorNames.end()) {
            std::cout << "Skipping socket " << dai::toString(socket) << " because it does not have associated sensor name!" << std::endl;
            return false;
        }

        auto sensorName = sensorNames.at(socket);
        for (const auto& allowedSensor : allowedSensors.value()) {
            if (sensorName == allowedSensor) {
                return true;
            }
        }
        return false;
    };

    for(auto socket : device->getConnectedCameras()) {
        if(!isSensorAllowed(socket)) {
            std::cout << "Skipping socket " << dai::toString(socket) << std::endl;
            continue;
        }
        std::cout << "Setting up socker " << dai::toString(socket) << std::endl;
        setUpCameraSocket(pipeline, device, syncNode, socket, name, targetFps, syncType, role, inputNames);
    }

    setUpIrLeds(device);

    if(syncType == SyncType::EXTERNAL) {
        if(role == dai::ExternalFrameSyncRole::MASTER) {
            numMasters++;
            device->setExternalStrobeEnable(true);
            std::cout << device->getDeviceId() << " is master" << std::endl;

            if(numMasters > 1) {
                throw std::runtime_error("Only one master pipeline is supported");
            }
        } else if(role == dai::ExternalFrameSyncRole::SLAVE) {
            numSlaves++;
            std::cout << device->getDeviceId() << " is slave" << std::endl;
        } else {
            throw std::runtime_error("Don't know how to handle role");
        }
    }
}

int testFsync(float targetFps, struct FsyncTestParameters parameters) {

    std::cout << "=================================\x1B[1;32mTest started\x1B[0m================================" << std::endl;
    std::cout << "Sync type: " << toString(parameters.syncType) << std::endl;
    std::cout << "FPS: " << targetFps << std::endl;
    std::cout << "SYNC_THRESHOLD_SEC: " << parameters.syncThresholdSec << std::endl;
    std::cout << "RECV_ALL_TIMEOUT_SEC: " << parameters.firstGroupTimeoutSec << std::endl;
    std::cout << "INITIAL_SYNC_TIMEOUT_SEC: " << parameters.syncAcquisitionTimeoutSec << std::endl;

    if (parameters.allowedSensors.has_value()) {
        std::cout << "ALLOWED_SENSORS: " << std::endl;
        for (const auto& sensor : parameters.allowedSensors.value()) {
            std::cout << "\t" << sensor << std::endl;
        }
    }

    std::vector<dai::DeviceInfo> deviceInfos = dai::Device::getAllAvailableDevices();

    // REQUIRE_MSG(deviceInfos.size() >= 2, "At least two devices are required for this test.");
    REQUIRE_MSG(deviceInfos.size() == parameters.expectedDevices, "Expected exactly " << parameters.expectedDevices << " devices, got " << deviceInfos.size());

    uint32_t numMasters = 0;
    uint32_t numSlaves = 0;
    dai::Pipeline pipeline(false);

    auto sync = pipeline.create<dai::node::Sync>();
    sync->setRunOnHost(true);
    sync->setSyncThreshold(std::chrono::nanoseconds(long(round(1e9 * 0.5f / targetFps))));
    sync->setTimestampSource(dai::SyncProperties::TimestampSource::SYSTEM);

    std::vector<std::string> inputNames;

    for(auto deviceInfo : deviceInfos) {
        setupDevice(deviceInfo, pipeline, sync, numMasters, numSlaves, inputNames, targetFps, parameters.syncType, parameters.allowedSensors);
    }

    if (parameters.syncType == SyncType::EXTERNAL) {
        if(numMasters == 0) {
            throw std::runtime_error("No master detected!");
        }
        if(numSlaves == 0) {
            throw std::runtime_error("No slaves detected!");
        }
    }
    auto queue = sync->out.createOutputQueue();

    pipeline.start();

    std::optional<std::shared_ptr<dai::MessageGroup>> latestFrameGroup;
    bool firstReceived = false;
    auto startTime = std::chrono::steady_clock::now();
    auto prevReceived = std::chrono::steady_clock::now();

    std::optional<std::chrono::time_point<std::chrono::steady_clock>> initialSyncTime;

    std::vector<Delta> deltas;

    bool waitingForInitialSync = true;
    bool waitingForWarmup = true;
    if (parameters.warmupDurationSec == 0) {
        waitingForWarmup = false;
    }

    while(true) {
        while(queue->has()) {
            auto syncData = queue->get();
            REQUIRE_MSG(syncData != nullptr, "Sync node failed to receive message");
            latestFrameGroup = std::dynamic_pointer_cast<dai::MessageGroup>(syncData);
            if(!firstReceived) {
                firstReceived = true;
                initialSyncTime = std::chrono::steady_clock::now();
            }
            prevReceived = std::chrono::steady_clock::now();
        }

        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - prevReceived).count() > 5) {
            REQUIRE_MSG(false, "Timeout: No frame groups received for 5 seconds");
        }

        if(!firstReceived) {
            auto endTime = std::chrono::steady_clock::now();
            auto elapsedSec = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
            REQUIRE_MSG(elapsedSec < parameters.firstGroupTimeoutSec, "Timeout: Didn't receive first group on time");
        }

        auto totalElapsedSec = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - startTime).count();

        if(totalElapsedSec >= parameters.totalRunDurationSec) {
            std::cout << "Timeout: Test finished after " << totalElapsedSec << " sec" << std::endl;
            break;
        }

        if(latestFrameGroup.has_value()) {
            REQUIRE_MSG(size_t(latestFrameGroup.value()->getNumMessages()) == inputNames.size(),
                        "Number of messages received doesn't match number of outputs");

            using ts_type = std::chrono::time_point<std::chrono::system_clock>;
            std::map<std::string, ts_type> tsValues;
            for(auto name : inputNames) {
                auto frame = latestFrameGroup.value()->get<dai::ImgFrame>(name);
                REQUIRE_MSG(frame != nullptr, "Frame pointer is null");
                REQUIRE_MSG(frame->getFsync() == convertSyncType(parameters.syncType),
                    "Frame sync type doesn't match: expected " << toString(convertSyncType(parameters.syncType)) << ", got " << toString(frame->getFsync()));
                tsValues.emplace(name, frame->getTimestampSystem(dai::CameraExposureOffset::END).value());
            }

            auto compFunct = [](const std::pair<std::string, ts_type>& p1, const std::pair<std::string, ts_type>& p2) -> bool { return p1.second < p2.second; };

            auto maxElement = std::max_element(tsValues.begin(), tsValues.end(), compFunct);
            auto minElement = std::min_element(tsValues.begin(), tsValues.end(), compFunct);

            auto delta = maxElement->second - minElement->second;
            auto deltaUs = std::chrono::duration_cast<std::chrono::microseconds>(delta).count();

            bool syncStatus = abs(deltaUs) < parameters.syncThresholdSec * 1e6;

            if (waitingForWarmup) {
                auto endTime = std::chrono::steady_clock::now();
                auto elapsedSec = std::chrono::duration_cast<std::chrono::seconds>(endTime - initialSyncTime.value()).count();
                if (elapsedSec >= parameters.warmupDurationSec) {
                    waitingForWarmup = false;
                }
            }

            if (syncStatus && !waitingForInitialSync && !waitingForWarmup) {
                Delta deltaStruct;
                deltaStruct.delta_us = deltaUs;
                deltaStruct.name = "[MIN=" + minElement->first + ", MAX=" + maxElement->first + "]";
                deltas.emplace_back(deltaStruct);
            }

            if(!syncStatus && waitingForInitialSync) {
                auto endTime = std::chrono::steady_clock::now();
                auto elapsedSec = std::chrono::duration_cast<std::chrono::seconds>(endTime - initialSyncTime.value()).count();
                REQUIRE_MSG(elapsedSec < parameters.syncAcquisitionTimeoutSec, "Timeout: Didn't sync frames in time");
            }

            if(syncStatus && waitingForInitialSync) {
                std::cout << "Sync status: in sync" << std::endl;
                waitingForInitialSync = false;
            }

            // Enable this once we have better accuracy for timestamps
            // if (thresholds.syncType == SyncType::EXTERNAL) {
            //     REQUIRE_MSG(waitingForInitialSync || syncStatus, "Sync error: Sync lost, threshold exceeded: " << deltaUs << " us");
            // }

            latestFrameGroup.reset();
        }
    }

    REQUIRE_MSG(deltas.size() > 100, "[FPS=" << targetFps << "] Not enough frames left after stabilization period (expected at least 100, got " << deltas.size() << ").");

    double meanDelta_us = calculate_mean(deltas);
    double p99Delta_us = percentile_linear(deltas, 99.0);
    Delta maxDelta = calculate_max_outlier(deltas);

    std::cout << "=== Stats" << std::endl;
    std::cout << "   [FPS=" << targetFps << "] # of frames used for stats caluculation: " << deltas.size() << std::endl;
    std::cout << "   [FPS=" << targetFps << "] Mean frame delta: " << meanDelta_us/1e3 << " ms" << std::endl;
    std::cout << "   [FPS=" << targetFps << "] p99 frame delta: " << p99Delta_us/1e3 << " ms" << std::endl;
    std::cout << "   [FPS=" << targetFps << "] Max outlier frame delta: " << maxDelta.delta_us/1e3 << " ms, between " << maxDelta.name << std::endl;

    REQUIRE_MSG(meanDelta_us/1e6 < parameters.deltaMeanThreshold, "[FPS=" << targetFps << "] Mean value of frame deltas above " << parameters.deltaMeanThreshold*1e3 << " ms (" << meanDelta_us/1e3 << " ms)");
    REQUIRE_MSG(p99Delta_us/1e6 < parameters.deltaP99Threshold, "[FPS=" << targetFps << "] p99 metric does not meet " << parameters.deltaP99Threshold*1e3 << " ms (" << p99Delta_us/1e3 << " ms)");

    return 0;
}
