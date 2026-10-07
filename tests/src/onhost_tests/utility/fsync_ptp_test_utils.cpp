#include "fsync_ptp_test_utils.hpp"

#include <algorithm>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <tuple>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/ExternalFrameSyncRoles.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
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

    double calculate_mean(std::vector<Delta> &values) {
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

    std::chrono::seconds toPhaseDuration(std::uint64_t seconds) {
        const auto remaining = std::chrono::steady_clock::time_point::max() - std::chrono::steady_clock::now();
        const auto maxSeconds = std::chrono::duration_cast<std::chrono::seconds>(remaining).count();
        INFO("Phase duration must fit a steady-clock deadline");
        CAPTURE(seconds, maxSeconds);
        REQUIRE(seconds < static_cast<std::uint64_t>(maxSeconds));
        return std::chrono::seconds(static_cast<std::chrono::seconds::rep>(seconds));
    }

    void printTestParameters(float targetFps, const FsyncTestParameters& parameters) {
        std::cout << "=================================\x1B[1;32mTest started\x1B[0m================================\n";
        std::cout << "Sync type: " << ::toString(parameters.syncType) << '\n';
        std::cout << "FPS: " << targetFps << '\n';
        std::cout << "SYNC_THRESHOLD_SEC: " << parameters.syncThresholdSec << '\n';
        std::cout << "FIRST_GROUP_TIMEOUT_SEC: " << parameters.firstGroupTimeoutSec << '\n';
        std::cout << "WARMUP_DURATION_SEC: " << parameters.warmupDurationSec << '\n';
        std::cout << "CONVERGENCE_TIMEOUT_SEC: " << parameters.syncAcquisitionTimeoutSec << '\n';
        std::cout << "MEASUREMENT_DURATION_SEC: " << parameters.measurementDurationSec << '\n';
        std::cout << "MEASUREMENT_NO_PROGRESS_TIMEOUT_SEC: " << parameters.measurementNoProgressTimeoutSec << '\n';
        if(parameters.allowedSensors.has_value()) {
            std::cout << "ALLOWED_SENSORS:\n";
            for(const auto& sensor : parameters.allowedSensors.value()) {
                std::cout << '\t' << sensor << '\n';
            }
        }
    }
}

GroupReader::GroupReader(dai::MessageQueue& queue, std::vector<std::string> inputNames, SyncType syncType)
    : queue(queue), inputNames(std::move(inputNames)), expectedFsync(convertSyncType(syncType)) {
    std::sort(this->inputNames.begin(), this->inputNames.end());
    INFO("GroupReader requires nonempty, unique expected stream names");
    CAPTURE(this->inputNames);
    REQUIRE_FALSE(this->inputNames.empty());
    REQUIRE_FALSE(this->inputNames.front().empty());
    REQUIRE(std::adjacent_find(this->inputNames.begin(), this->inputNames.end()) == this->inputNames.end());
}

std::optional<GroupReadResult> GroupReader::read(std::chrono::steady_clock::time_point deadline) const {
    const auto now = std::chrono::steady_clock::now();
    if(now >= deadline) return std::nullopt;

    bool hasTimedOut = false;
    const auto group = queue.get<dai::MessageGroup>(deadline - now, hasTimedOut);
    if(hasTimedOut) return std::nullopt;
    INFO("GroupReader expected a non-null MessageGroup");
    REQUIRE(group != nullptr);
    return analyze(*group);
}

std::chrono::system_clock::time_point GroupReader::readTimestamp(const dai::MessageGroup& group, const std::string& name) const {
    CAPTURE(name);
    // MessageGroup::get inserts missing names; use a const lookup for validation.
    const auto entry = group.group.find(name);
    INFO("GroupReader requires each expected stream to contain an ImgFrame with matching fsync metadata and a system timestamp");
    REQUIRE(entry != group.group.end());
    const auto frame = std::dynamic_pointer_cast<const dai::ImgFrame>(entry->second);
    REQUIRE(frame != nullptr);
    CAPTURE(toString(expectedFsync), toString(frame->getFsync()));
    REQUIRE(frame->getFsync() == expectedFsync);
    const auto timestamp = frame->getTimestampSystem(dai::CameraExposureOffset::END);
    REQUIRE(timestamp.has_value());
    return *timestamp;
}

GroupReadResult GroupReader::analyze(const dai::MessageGroup& group) const {
    INFO("GroupReader message count must match the expected stream count");
    REQUIRE(group.group.size() == inputNames.size());
    std::vector<std::chrono::system_clock::time_point> timestamps;
    timestamps.reserve(inputNames.size());
    for(const auto& name : inputNames) {
        timestamps.push_back(readTimestamp(group, name));
    }

    // Both extrema choose the lexicographically first stream when timestamps tie.
    const auto minIt = std::min_element(timestamps.begin(), timestamps.end());
    const auto maxIt = std::max_element(timestamps.begin(), timestamps.end());
    const auto spread = *maxIt - *minIt;
    const auto minName = inputNames[static_cast<std::size_t>(minIt - timestamps.begin())];
    const auto maxName = inputNames[static_cast<std::size_t>(maxIt - timestamps.begin())];

    std::sort(timestamps.begin(), timestamps.end());
    const auto middle = timestamps.size() / 2;
    const auto upper = timestamps[middle];
    const auto lower = timestamps[(timestamps.size() - 1) / 2];
    // Keep native clock precision; an even-sized median rounds down to a clock tick.
    const auto median = lower + (upper - lower) / 2;
    return {spread, minName, maxName, median};
}

GroupReadResult waitForFirstGroup(const GroupReader& reader, std::chrono::seconds timeout) {
    const auto start = std::chrono::steady_clock::now();
    INFO("Startup phase: timeout " << timeout.count() << " seconds to receive the first validated group");
    REQUIRE(timeout > std::chrono::seconds::zero());

    const auto deadline = start + timeout;
    std::cout << "Startup started: " << timeout.count() << " sec\n";
    auto group = reader.read(deadline);
    {
        INFO("Startup timed out without receiving a validated group");
        REQUIRE(group.has_value());
    }

    std::cout << "Startup finished: first validated group received\n";
    return std::move(*group);
}

void consumeWarmup(const GroupReader& reader, std::chrono::seconds duration) {
    const auto start = std::chrono::steady_clock::now();
    INFO("Warmup phase: consuming validated groups for " << duration.count() << " seconds");
    REQUIRE(duration >= std::chrono::seconds::zero());
    if(duration == std::chrono::seconds::zero()) return;

    const auto deadline = start + duration;
    std::cout << "Warmup started: " << duration.count() << " sec\n";
    while(reader.read(deadline)) {
        // Discard each validated group; a timed read ends warmup when the budget expires.
    }
    std::cout << "Warmup finished\n";
}

GroupReadResult waitForConvergence(const GroupReader& reader,
                                  std::chrono::seconds timeout,
                                  std::chrono::system_clock::duration syncThreshold,
                                  std::optional<GroupReadResult> initialGroup) {
    const auto start = std::chrono::steady_clock::now();
    INFO("Convergence phase: timeout " << timeout.count() << " seconds, spread must be strictly below "
                                      << std::chrono::duration_cast<std::chrono::microseconds>(syncThreshold).count() << " us");
    REQUIRE(timeout > std::chrono::seconds::zero());
    REQUIRE(syncThreshold > std::chrono::system_clock::duration::zero());

    const auto deadline = start + timeout;
    std::cout << "Convergence started: " << timeout.count() << " sec\n";
    auto group = std::move(initialGroup);
    if(!group) group = reader.read(deadline);
    {
        INFO("Convergence timed out without receiving a validated group");
        REQUIRE(group.has_value());
    }

    while(group->timestampSpread >= syncThreshold) {
        INFO("Convergence timed out; last observed spread: "
             << std::chrono::duration_cast<std::chrono::microseconds>(group->timestampSpread).count() << " us");
        CAPTURE(group->minStreamName, group->maxStreamName);
        group = reader.read(deadline);
        REQUIRE(group.has_value());
    }

    std::cout << "Convergence finished: in sync\n";
    return std::move(*group);
}

std::vector<GroupReadResult> collectMeasurements(const GroupReader& reader,
                                               GroupReadResult firstGroup,
                                               std::chrono::seconds duration,
                                               float targetFps,
                                               std::chrono::seconds noProgressTimeout) {
    const auto start = std::chrono::steady_clock::now();
    INFO("Measurement phase: duration must be positive, got " << duration.count() << " seconds");
    REQUIRE(duration > std::chrono::seconds::zero());
    REQUIRE(noProgressTimeout > std::chrono::seconds::zero());
    CAPTURE(targetFps);
    INFO("Maximum measurement delivery silence: " << noProgressTimeout.count() << " seconds");
    {
        INFO("Measurement target FPS must be finite and positive");
        REQUIRE(std::isfinite(targetFps));
        REQUIRE(targetFps > 0.0f);
    }
    const auto maxGap = std::chrono::duration<double>(1.5 / targetFps);

    const auto deadline = start + duration;
    std::cout << "Measurement started: " << duration.count() << " sec\n";
    std::vector<GroupReadResult> samples;
    samples.push_back(std::move(firstGroup));
    auto lastProgressTime = start;
    while(std::chrono::steady_clock::now() < deadline) {
        const auto progressDeadline = lastProgressTime + noProgressTimeout;
        auto group = reader.read(std::min(deadline, progressDeadline));
        if(!group) {
            INFO("Synchronized groups stopped arriving during measurement");
            REQUIRE(deadline < progressDeadline);
            break;
        }
        const auto receivedAt = std::chrono::steady_clock::now();
        {
            INFO("Measurement delivery silence exceeded the no-progress timeout");
            CAPTURE(std::chrono::duration<double>(receivedAt - lastProgressTime).count());
            REQUIRE(receivedAt < progressDeadline);
        }
        const auto previousTimestamp = samples.back().medianTimestamp;
        const auto gap = group->medianTimestamp - previousTimestamp;
        INFO("Measurement continuity: sample index " << samples.size() << " (zero-based; convergence sample is 0), observed gap "
                                                     << std::chrono::duration<double>(gap).count() << " seconds, allowed gap " << maxGap.count()
                                                     << " seconds (1.5 frame periods)");
        {
            INFO("Nonincreasing measurement timestamps are not allowed");
            REQUIRE(gap > std::chrono::system_clock::duration::zero());
        }
        {
            INFO("Measurement frame interval exceeds the allowed gap");
            REQUIRE(gap <= maxGap);
        }
        samples.push_back(std::move(*group));
        lastProgressTime = receivedAt;
    }
    INFO("Trailing measurement interval must remain within the delivery-silence allowance");
    CAPTURE(std::chrono::duration<double>(deadline - lastProgressTime).count());
    REQUIRE(deadline - lastProgressTime < noProgressTimeout);
    std::cout << "Measurement finished: " << samples.size() << " samples\n";
    return samples;
}

void reportAndCheckStatistics(const std::vector<GroupReadResult>& samples, float targetFps, const FsyncTestParameters& parameters) {
    CAPTURE(targetFps);
    {
        INFO("Not enough measurement samples (expected at least 101, got " << samples.size() << ").");
        REQUIRE(samples.size() > 100);
    }

    std::vector<Delta> deltas;
    deltas.reserve(samples.size());
    for(const auto& sample : samples) {
        const auto spreadUs = std::chrono::duration_cast<std::chrono::microseconds>(sample.timestampSpread).count();
        deltas.push_back({static_cast<std::uint64_t>(spreadUs), "[MIN=" + sample.minStreamName + ", MAX=" + sample.maxStreamName + "]"});
    }

    const double meanDeltaUs = calculate_mean(deltas);
    const double p99DeltaUs = percentile_linear(deltas, 99.0);
    const Delta maxDelta = calculate_max_outlier(deltas);

    std::cout << "=== Stats\n";
    std::cout << "   [FPS=" << targetFps << "] # of frames used for stats calculation: " << deltas.size() << '\n';
    std::cout << "   [FPS=" << targetFps << "] Mean frame delta: " << meanDeltaUs / 1e3 << " ms\n";
    std::cout << "   [FPS=" << targetFps << "] p99 frame delta: " << p99DeltaUs / 1e3 << " ms\n";
    std::cout << "   [FPS=" << targetFps << "] Max outlier frame delta: " << static_cast<double>(maxDelta.delta_us) / 1e3 << " ms, between "
              << maxDelta.name << '\n';

    const double meanDeltaSec = meanDeltaUs / 1e6;
    const double p99DeltaSec = p99DeltaUs / 1e6;
    INFO("Arithmetic mean and p99 frame deltas must be strictly below their thresholds (all values in seconds)");
    CAPTURE(meanDeltaSec, p99DeltaSec, parameters.deltaMeanThreshold, parameters.deltaP99Threshold);
    REQUIRE(meanDeltaSec < parameters.deltaMeanThreshold);
    REQUIRE(p99DeltaSec < parameters.deltaP99Threshold);
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

            REQUIRE_MSG(numMasters <= 1, "Only one master pipeline is supported");
        } else if(role == dai::ExternalFrameSyncRole::SLAVE) {
            numSlaves++;
            std::cout << device->getDeviceId() << " is slave" << std::endl;
        } else {
            throw std::runtime_error("Don't know how to handle role");
        }
    }
}

std::tuple<dai::Pipeline, std::shared_ptr<dai::node::Sync>, std::vector<std::string>> setupPipeline(
                   float targetFps,
                   struct FsyncTestParameters parameters)
{
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
    std::set<std::string> contributingDeviceIds;
    std::vector<std::string> devicesWithoutStreams;

    for(auto deviceInfo : deviceInfos) {
        const auto previousInputCount = inputNames.size();
        setupDevice(deviceInfo, pipeline, sync, numMasters, numSlaves, inputNames, targetFps, parameters.syncType, parameters.allowedSensors);
        if(inputNames.size() > previousInputCount) {
            contributingDeviceIds.insert(deviceInfo.getDeviceId());
        } else {
            devicesWithoutStreams.push_back(deviceInfo.getDeviceId());
            std::cout << "Device " << deviceInfo.getDeviceId() << " contributes no camera streams after sensor filtering\n";
        }
    }

    {
        INFO("Multi-device synchronization requires camera streams from at least two distinct devices");
        CAPTURE(contributingDeviceIds, devicesWithoutStreams, parameters.allowedSensors);
        REQUIRE(contributingDeviceIds.size() >= 2);
        if(!parameters.allowedSensors.has_value()) {
            INFO("Without a sensor filter, every discovered device must contribute camera streams");
            REQUIRE(contributingDeviceIds.size() == deviceInfos.size());
        }
    }

    if (parameters.syncType == SyncType::EXTERNAL) {
        REQUIRE_MSG(numMasters == 1, "Number of masters detected is not 1");
        REQUIRE_MSG(numSlaves > 0, "Number of slaves detected is not > 0");
    }

    return std::make_tuple(std::move(pipeline), sync, std::move(inputNames));
}

int testSync(float targetFps, struct FsyncTestParameters parameters) {
    printTestParameters(targetFps, parameters);
    REQUIRE(std::isfinite(targetFps));
    REQUIRE(targetFps > 0.0f);
    REQUIRE(std::isfinite(parameters.syncThresholdSec));
    REQUIRE(parameters.syncThresholdSec > 0.0);
    REQUIRE(parameters.syncThresholdSec < std::chrono::duration<double>(std::chrono::system_clock::duration::max()).count());

    const auto firstGroupTimeout = toPhaseDuration(parameters.firstGroupTimeoutSec);
    const auto warmupDuration = toPhaseDuration(parameters.warmupDurationSec);
    const auto convergenceTimeout = toPhaseDuration(parameters.syncAcquisitionTimeoutSec);
    const auto measurementDuration = toPhaseDuration(parameters.measurementDurationSec);
    const auto noProgressTimeout = toPhaseDuration(parameters.measurementNoProgressTimeoutSec);
    const auto syncThreshold = std::chrono::duration_cast<std::chrono::system_clock::duration>(std::chrono::duration<double>(parameters.syncThresholdSec));
    REQUIRE(firstGroupTimeout > std::chrono::seconds::zero());
    REQUIRE(convergenceTimeout > std::chrono::seconds::zero());
    REQUIRE(measurementDuration > std::chrono::seconds::zero());
    REQUIRE(noProgressTimeout > std::chrono::seconds::zero());
    REQUIRE(syncThreshold > std::chrono::system_clock::duration::zero());

    auto [pipeline, sync, inputNames] = setupPipeline(targetFps, parameters);
    auto queue = sync->out.createOutputQueue();
    const GroupReader reader(*queue, std::move(inputNames), parameters.syncType);

    // Pipeline destruction stops and joins it if a phase fails a fatal Catch2 assertion.
    pipeline.start();
    std::optional<GroupReadResult> initialGroup = waitForFirstGroup(reader, firstGroupTimeout);

    if(warmupDuration > std::chrono::seconds::zero()) {
        initialGroup.reset();
        consumeWarmup(reader, warmupDuration);
    }

    auto convergedGroup = waitForConvergence(reader, convergenceTimeout, syncThreshold, std::move(initialGroup));
    const auto samples = collectMeasurements(reader, std::move(convergedGroup), measurementDuration, targetFps, noProgressTimeout);
    pipeline.stop();

    reportAndCheckStatistics(samples, targetFps, parameters);
    return 0;
}
