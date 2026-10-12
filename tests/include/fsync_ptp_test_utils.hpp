#pragma once

#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <map>
#include <chrono>
#include <cstdint>

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
    uint64_t testDurationSec;
    int recvAllTimeoutSec;
    int initialSyncTimeoutSec;
    int initialTimeoutSec;
    double deltaMeanThreshold;
    double deltaP99Threshold;
    SyncType syncType;
};

dai::Node::Output* createPipeline(std::shared_ptr<dai::Pipeline> pipeline,
                                  dai::CameraBoardSocket socket,
                                  float sensorFps,
                                  SyncType syncType,
                                  std::optional<dai::ExternalFrameSyncRole> role);

std::shared_ptr<dai::node::Sync> createSyncNode(std::shared_ptr<dai::Pipeline>& masterPipeline,
                                                std::map<std::string, dai::Node::Output*>& masterNode,
                                                const std::string& masterName,
                                                std::chrono::nanoseconds syncThreshold,
                                                std::vector<std::string>& outputNames,
                                                std::map<std::string, std::map<std::string, std::shared_ptr<dai::MessageQueue>>>& slaveQueues,
                                                std::map<std::string, std::shared_ptr<dai::InputQueue>>& inputQueues);

void setUpCameraSocket(std::shared_ptr<dai::Pipeline>& pipeline,
                       dai::CameraBoardSocket socket,
                       std::string& name,
                       float targetFps,
                       SyncType syncType,
                       std::optional<dai::ExternalFrameSyncRole> role,
                       std::optional<std::map<std::string, dai::Node::Output*>>& masterNode,
                       std::map<std::string, std::map<std::string, std::shared_ptr<dai::MessageQueue>>>& slaveQueues,
                       std::vector<std::string>& camSockets);

void setUpIrLeds(std::shared_ptr<dai::Device> device);

void setupDevice(dai::DeviceInfo& deviceInfo,
                 std::shared_ptr<dai::Pipeline>& masterPipeline,
                 std::optional<std::map<std::string, dai::Node::Output*>>& masterNode,
                 std::optional<std::string>& masterName,
                 std::map<std::string, std::shared_ptr<dai::Pipeline>>& slavePipelines,
                 std::map<std::string, std::map<std::string, std::shared_ptr<dai::MessageQueue>>>& slaveQueues,
                 std::vector<std::string>& camSockets,
                 float targetFps,
                 SyncType syncType);

int testFsync(float targetFps, struct FsyncTestParameters parameters);