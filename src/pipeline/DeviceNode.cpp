#include "depthai/pipeline/DeviceNode.hpp"

#include <spdlog/async_logger.h>

#include <ratio>

#include "depthai/pipeline/Pipeline.hpp"
#include "spdlog/fmt/fmt.h"

namespace dai {

DeviceNode::DeviceNode(std::unique_ptr<Properties> props, bool conf) : propertiesHolder(std::move(props)) {
    configureMode = conf;
}

DeviceNode::DeviceNode(const std::shared_ptr<Device>& device, std::unique_ptr<Properties> props, bool conf)
    : device(device), propertiesHolder(std::move(props)) {
    configureMode = conf;
}

void DeviceNode::setDevice(const std::shared_ptr<Device>& device) {
    this->device = device;
}

const std::shared_ptr<Device> DeviceNode::getDevice() const {
    return device;
}

Properties& DeviceNode::getProperties() {
    return *propertiesHolder;
}

void DeviceNode::run() {}

void DeviceNode::setLogLevel(dai::LogLevel level) {
    const bool isHostNode = runOnHost();
    if(isHostNode) {
        ThreadedNode::setLogLevel(level);
    } else {
        int64_t myid = id;
        device->setNodeLogLevel(myid, level);
    }
}

void DeviceNode::logTiming(const std::shared_ptr<spdlog::async_logger>& logger,
                           std::chrono::steady_clock::time_point tAbsoluteBeginning,
                           std::chrono::steady_clock::time_point tGotInput,
                           std::chrono::steady_clock::time_point tProcessed,
                           std::chrono::steady_clock::time_point tAbsoluteEnd) {
    logger->trace("{} took {}ms, getting input {}ms, processing {}ms,  sending output {}ms",
                  this->getName(),
                  std::chrono::duration_cast<std::chrono::microseconds>(tAbsoluteEnd - tAbsoluteBeginning).count() / 1000,
                  std::chrono::duration_cast<std::chrono::microseconds>(tGotInput - tAbsoluteBeginning).count() / 1000,
                  std::chrono::duration_cast<std::chrono::microseconds>(tProcessed - tGotInput).count() / 1000,
                  std::chrono::duration_cast<std::chrono::microseconds>(tAbsoluteEnd - tProcessed).count() / 1000);
}

dai::LogLevel DeviceNode::getLogLevel() const {
    const bool isHostNode = runOnHost();
    if(isHostNode) {
        return ThreadedNode::getLogLevel();
    } else {
        int64_t myid = id;
        return device->getNodeLogLevel(myid);
    }
}

HostRunnable::~HostRunnable() = default;

}  // namespace dai
