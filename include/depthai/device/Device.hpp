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
#include "depthai/utility/RecordReplay.hpp"

namespace dai {

/**
 * @brief Hardware platform type
 *
 */
enum class Platform { RVC2, RVC3, RVC4 };

/**
 * @brief Convert Platform enum to string
 *
 * @param platform Platform enum
 * @return std::string String representation of Platform
 */
std::string platform2string(Platform platform);

/**
 * @brief Convert string to Platform enum
 *
 * @param platform String representation of Platform
 * @return Platform Platform enum
 */
Platform string2platform(const std::string& platform);

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

    /**
     * @brief Get the platform of the connected device
     * @return Platform Platform enum
     */
    Platform getPlatform() const;

    /**
     * @brief Get the platform of the connected device as string
     * @return std::string String representation of Platform
     */
    std::string getPlatformAsString() const;

   private:
    void closeImpl() override;
};

}  // namespace dai
