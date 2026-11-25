#pragma once
#include <cstdint>
#include <optional>
#include <string>

#include "depthai/common/UsbSpeed.hpp"
#include "depthai/device/Version.hpp"

namespace dai {
namespace gate {

/**
 * @brief Get the OS version from /etc/os-release
 * @return dai::Version object representing the OS version
 * @throws std::runtime_error if the file cannot be found or parsed
 */
dai::Version getOSVersion();

/**
 * @brief Get the OS version string from the VERSION field in /etc/os-release
 * @return OS version string, for example "1.32.0"
 * @throws std::runtime_error if the file cannot be found or parsed
 */
std::string getOSVersionString();


enum class DeviceState {
    FACTORY,     // Factory mode, DepthAI can run in this mode
    SETUP,       // Device is in setup mode, DepthAI can not run in this mode
    CONFIGURED,  // Device is configured, DepthAI can run in this mode
    ERROR,       // Device is in error state, DepthAI can not run in this mode
};

/**
 * @brief Get the current state of the device
 * @return DeviceState enum representing the current state of the device
 */
DeviceState getDeviceState();

/**
 * @brief Get the name of the active power profile from /run/agentconfd/power_profile
 * @return Power profile name, for example "15w_sustained", or std::nullopt if agentconfd has not published one
 */
std::optional<std::string> getPowerProfile();

/**
 * @brief Get the OS brightness ceiling for an fsync-stm LED, such as "ir_dot" or "ir_flood"
 * @return Brightness ceiling in the range 0-255, or std::nullopt if unavailable
 */
std::optional<uint32_t> getLedBrightnessCeiling(const std::string& ledName);

enum M8ConnectorState {
    DISCONNECTED,
    CONNECTED,
};

/**
 * @brief Get the M8 connector state published by agentconfd
 * @return Connector state, or std::nullopt if unavailable
 */
std::optional<M8ConnectorState> getM8State();

/**
 * @brief Get the device serial number from sysfs
 * @return Serial number, or "xlink-server" if unavailable
 */
std::string getSerialNumber();

/**
 * @brief Get the current USB connection speed
 * @return USB speed, or dai::UsbSpeed::UNKNOWN if unavailable
 */
dai::UsbSpeed getUsbSpeed();

/**
 * @brief Get the Ethernet link speed for the interface with the given IPv4 address
 * @return Link speed in Mbps, or 0 if unavailable
 */
int getEthernetLinkSpeed(const std::string& address);

/**
 * @brief Get the resident memory usage of the current process
 * @return RSS in KiB, or std::nullopt if unavailable
 */
std::optional<int64_t> getProcessMemoryUsage();

/**
 * @brief Enable or disable core dumps for the current process and configure the dump filter
 */
void enableCrashDump(bool enable);

}  // namespace gate
}  // namespace dai
