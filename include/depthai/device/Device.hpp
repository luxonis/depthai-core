// IWYU pragma: private, include "depthai/depthai.hpp"
#pragma once

// std
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

// project
#include "depthai/device/DeviceBase.hpp"
#include "depthai/device/Platform.hpp"
#include "depthai/utility/RecordReplay.hpp"

namespace dai {

/**
 * Represents the DepthAI device with the methods to interact with it.
 * Implements the host-side queues to connect with XLinkIn and XLinkOut nodes
 */
class Device : public DeviceBase {
   public:
    using DeviceBase::DeviceBase;  // inherit the ctors
    using DeviceBase::ReconnectionStatus;

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * Uses OpenVINO version OpenVINO::VERSION_UNIVERSAL
     */
    Device();

    /**
     * @brief dtor to close the device
     */
    ~Device() override;


   private:
    void closeImpl() override;
};

}  // namespace dai
