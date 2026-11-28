#include "OSUtils.hpp"

#include <ifaddrs.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/resource.h>

#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;

namespace dai {
namespace gate {

std::string unquote(std::string value) {
    if(value.size() >= 2 && ((value.front() == '"' && value.back() == '"') || (value.front() == '\'' && value.back() == '\''))) {
        return value.substr(1, value.size() - 2);
    }
    return value;
}

std::string getOSVersionString() {
    const fs::path osVersionFile("/etc/os-release");
    if(!fs::exists(osVersionFile)) {
        throw std::runtime_error("Could not find /etc/os-release file");
    }

    std::ifstream osVersionFileStream(osVersionFile);
    std::string line;

    while(std::getline(osVersionFileStream, line)) {
        const auto separator = line.find('=');
        if(separator == std::string::npos) {
            continue;
        }
        if(line.substr(0, separator) == "VERSION") {
            return unquote(line.substr(separator + 1));
        }
    }

    throw std::runtime_error("Could not parse VERSION from /etc/os-release");
}

dai::Version getOSVersion() {
    const auto osVersion = getOSVersionString();
    std::regex versionRegex(R"((\d+)\.(\d+)(?:\.(\d+))?)");
    std::smatch match;

    if(std::regex_search(osVersion, match, versionRegex)) {
        int major = std::stoi(match[1]);
        int minor = std::stoi(match[2]);
        int patch = match[3].matched ? std::stoi(match[3]) : 0;
        return dai::Version(major, minor, patch);
    }

    throw std::runtime_error("Could not parse OS version from /etc/os-release");
}

DeviceState getDeviceState() {
    bool factoryFileExists = fs::exists("/persist/factory/enabled");
    bool deviceIsConfigured = fs::exists("/etc/agentconfd/.setup_complete");

    if(factoryFileExists) {
        return DeviceState::FACTORY;
    } else if(deviceIsConfigured) {
        return DeviceState::CONFIGURED;
    } else {
        return DeviceState::SETUP;
    }
}

std::optional<std::string> getPowerProfile() {
    std::ifstream file("/run/agentconfd/power_profile");
    std::string name;
    if(!std::getline(file, name) || name.empty()) {
        return std::nullopt;
    }
    return name;
}

std::optional<uint32_t> getLedBrightnessCeiling(const std::string& ledName) {
    std::ifstream file("/sys/class/fsync-stm/fsync-stm:led:" + ledName + "/brightness_ceiling");
    uint32_t ceiling;
    if(!(file >> ceiling) || ceiling > 255) {
        return std::nullopt;
    }
    return ceiling;
}

std::optional<M8ConnectorState> getM8State() {
    std::ifstream stateFile("/run/agentconfd/m8_connector_state");
    std::string content;
    stateFile >> content;
    if(content == "0") {
        return M8ConnectorState::DISCONNECTED;
    } else if(content == "1") {
        return M8ConnectorState::CONNECTED;
    }
    return std::nullopt;
}

std::string getSerialNumber() {
    const char* serialPath = "/sys/bus/soc/devices/soc0/serial_number";

    std::string serial = "xlink-server";
    try {
        std::ifstream file(serialPath);
        if(!file) {
            throw std::runtime_error("File not found");
        }
        std::string line;
        if(std::getline(file, line)) {
            serial = line.substr(0, line.find('\0'));
        }
    } catch(const std::exception& ex) {
        std::cerr << "Couldn't retrieve device's serial number. Using default 'xlink-server'. Error: " << ex.what() << std::endl;
    }
    return serial;
}

dai::UsbSpeed getUsbSpeed() {
    std::ifstream speedFile("/sys/class/udc/a600000.dwc3/current_speed");
    std::string speed;
    if(!std::getline(speedFile, speed)) return dai::UsbSpeed::UNKNOWN;

    if(speed == "low-speed") return dai::UsbSpeed::LOW;
    if(speed == "full-speed") return dai::UsbSpeed::FULL;
    if(speed == "high-speed") return dai::UsbSpeed::HIGH;
    if(speed == "super-speed") return dai::UsbSpeed::SUPER;
    if(speed == "super-speed-plus") return dai::UsbSpeed::SUPER_PLUS;
    return dai::UsbSpeed::UNKNOWN;
}

int getEthernetLinkSpeed(const std::string& address) {
    ifaddrs* interfaces = nullptr;
    if(getifaddrs(&interfaces) != 0) return 0;
    const std::unique_ptr<ifaddrs, decltype(&freeifaddrs)> guard(interfaces, freeifaddrs);
    for(auto* iface = interfaces; iface != nullptr; iface = iface->ifa_next) {
        if(!iface->ifa_addr || iface->ifa_addr->sa_family != AF_INET || (iface->ifa_flags & IFF_LOOPBACK) != 0) continue;
        std::array<char, NI_MAXHOST> ip{};
        if(getnameinfo(iface->ifa_addr, sizeof(sockaddr_in), ip.data(), ip.size(), nullptr, 0, NI_NUMERICHOST) != 0 || address != ip.data()) continue;

        std::ifstream speedFile(std::string("/sys/class/net/") + iface->ifa_name + "/speed");
        int speedMbps = 0;
        if(speedFile >> speedMbps && speedMbps > 0) return speedMbps;
        return 0;
    }
    return 0;
}

std::optional<int64_t> getProcessMemoryUsage() {
    std::ifstream status("/proc/self/status");
    for(std::string line; std::getline(status, line);) {
        if(line.rfind("VmRSS:", 0) != 0) continue;
        std::istringstream rss(line.substr(6));
        int64_t memory;
        if(rss >> memory && memory >= 0) return memory;
        break;
    }
    return std::nullopt;
}

void enableCrashDump(bool enable) {
    const rlim_t limit = enable ? RLIM_INFINITY : 0;
    const rlimit coreLimit{limit, limit};
    if(setrlimit(RLIMIT_CORE, &coreLimit) == -1) {
        std::cerr << "Error setting core limit" << std::endl;
        return;
    }
    std::cout << "Core limit set" << std::endl;

    const char* path = "/proc/self/coredump_filter";
    const char* value = "0x33\n";
    std::ofstream file(path);
    if(!file) {
        std::cerr << "Failed to open " << path << " for writing.\n";
        return;
    }
    file << value;
    if(file.fail()) {
        std::cerr << "Failed to write to " << path << ".\n";
        return;
    }
    std::cout << "Successfully set coredump_filter to " << value;
}

}  // namespace gate
}  // namespace dai
