#pragma once

#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <map>
#include <chrono>
#include <cstdint>
#include <set>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/ExternalFrameSyncRoles.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/node/Sync.hpp"

enum class SyncType {
    EXTERNAL,
    PTP,
};

std::string toString(SyncType syncType);

struct FsyncTestParameters {
    double syncThresholdSec;
    uint64_t totalRunDurationSec;
    int firstGroupTimeoutSec;
    int syncAcquisitionTimeoutSec;
    int warmupDurationSec;
    double deltaMeanThreshold;
    double deltaP99Threshold;
    SyncType syncType;
    std::optional<std::set<std::string>> allowedSensors;
    int expectedDevices;
};

void setUpCameraSocket(dai::Pipeline& pipeline,
                       std::shared_ptr<dai::Device> device,
                       std::shared_ptr<dai::node::Sync> syncNode,
                       dai::CameraBoardSocket socket,
                       std::string& deviceName,
                       float targetFps,
                       SyncType syncType,
                       std::optional<dai::ExternalFrameSyncRole> role,
                       std::vector<std::string>& inputNames);

void setUpIrLeds(std::shared_ptr<dai::Device> device);

void setupDevice(dai::DeviceInfo& deviceInfo,
                 dai::Pipeline& pipeline,
                 std::shared_ptr<dai::node::Sync> syncNode,
                 uint32_t &numMasters,
                 uint32_t &numSlaves,
                 std::vector<std::string>& inputNames,
                 float targetFps,
                 SyncType syncType,
                 std::optional<std::set<std::string>> &allowedSensors);

int testFsync(float targetFps, struct FsyncTestParameters parameters);