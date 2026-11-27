#pragma once
#include <cstdint>
#include <optional>
#include <string>

#include "depthai/common/UsbSpeed.hpp"
#include "depthai/device/Version.hpp"

namespace dai {
namespace gate {

dai::Version getOSVersion();  // From /etc/os-release; throws std::runtime_error if missing or unparsable

std::string getOSVersionString();  // VERSION of /etc/os-release, e.g. "1.32.0"; throws std::runtime_error if missing or unparsable

enum class DeviceState {
    FACTORY,     // Factory mode, DepthAI can run in this mode
    SETUP,       // Device is in setup mode, DepthAI can not run in this mode
    CONFIGURED,  // Device is configured, DepthAI can run in this mode
    ERROR,       // Device is in error state, DepthAI can not run in this mode
};

DeviceState getDeviceState();

std::optional<std::string> getPowerProfile();

std::optional<uint32_t> getLedBrightnessCeiling(const std::string& ledName);  // Ceiling in the range 0-255; std::nullopt if unavailable

enum M8ConnectorState {
    DISCONNECTED,
    CONNECTED,
};

std::optional<M8ConnectorState> getM8State();

std::string getSerialNumber();  // "xlink-server" if unavailable

dai::UsbSpeed getUsbSpeed();

int getEthernetLinkSpeed(const std::string& address);  // Link speed in Mbps, 0 if unavailable

std::optional<int64_t> getProcessMemoryUsage();  // RSS in KiB

void enableCrashDump(bool enable);

}  // namespace gate
}  // namespace dai
