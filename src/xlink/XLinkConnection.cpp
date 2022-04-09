#include "xlink/XLinkConnection.hpp"

#include <array>
#include <cassert>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

// project
#include "depthai/utility/Initialization.hpp"
#include "utility/Environment.hpp"
#include "utility/spdlog-fmt.hpp"

// libraries
#include <XLink/XLink.h>
extern "C" {
#include <XLink/XLinkLog.h>
}

#include "spdlog/details/os.h"
#include "spdlog/fmt/chrono.h"
#include "spdlog/spdlog.h"

namespace dai {

static XLinkProtocol_t getDefaultProtocol() {
    XLinkProtocol_t defaultProtocol = X_LINK_ANY_PROTOCOL;

    auto protocolStr = utility::getEnv("DEPTHAI_PROTOCOL");

    std::transform(protocolStr.begin(), protocolStr.end(), protocolStr.begin(), ::tolower);
    if(protocolStr.empty() || protocolStr == "any") {
        defaultProtocol = X_LINK_ANY_PROTOCOL;
    } else if(protocolStr == "usb") {
        defaultProtocol = X_LINK_USB_VSC;
    } else if(protocolStr == "tcpip") {
        defaultProtocol = X_LINK_TCP_IP;
    } else {
        spdlog::warn("Unsupported protocol specified");
    }

    return defaultProtocol;
}

// DeviceInfo
DeviceInfo::DeviceInfo(std::string mxId) {
    // Add dash at the end of mxId ([mxId]-[xlinkDevName] format)
    mxId += "-";
    // Construct device info which will points to device with specific mxId
    std::strncpy(desc.name, mxId.c_str(), sizeof(desc.name) - 1);

    // set protocol to any
    desc.protocol = X_LINK_ANY_PROTOCOL;

    // set platform to any
    desc.platform = X_LINK_ANY_PLATFORM;

    // set state to any
    state = X_LINK_ANY_STATE;
}
DeviceInfo::DeviceInfo(const char* mxId) : DeviceInfo(std::string(mxId)) {}

std::string DeviceInfo::getMxId() const {
    std::string mxId = "";
    auto len = std::strlen(desc.name);
    for(std::size_t i = 0; i < len; i++) {
        if(desc.name[i] == '-') break;
        mxId += desc.name[i];
    }
    return mxId;
}

static DeviceInfo deviceInfoFix(const DeviceInfo& d, XLinkDeviceState_t state);

// STATIC
constexpr std::chrono::milliseconds XLinkConnection::WAIT_FOR_BOOTUP_TIMEOUT_TCPIP;
constexpr std::chrono::milliseconds XLinkConnection::WAIT_FOR_BOOTUP_TIMEOUT_USB;
constexpr std::chrono::milliseconds XLinkConnection::WAIT_FOR_CONNECT_TIMEOUT;

std::vector<DeviceInfo> XLinkConnection::getAllConnectedDevices(XLinkDeviceState_t state, bool skipInvalidDevices) {
    initialize();

    std::vector<DeviceInfo> devices;

    std::vector<XLinkDeviceState_t> states;
    if(state == X_LINK_ANY_STATE) {
        states = {X_LINK_UNBOOTED, X_LINK_BOOTLOADER, X_LINK_BOOTED, X_LINK_FLASH_BOOTED};
    } else {
        states = {state};
    }

    auto allowedDeviceIds = utility::getEnv("DEPTHAI_DEVICE_MXID_LIST");

    // Get all available devices (unbooted & booted)
    for(const auto& state : states) {
        unsigned int numdev = 0;
        std::array<deviceDesc_t, 32> deviceDescAll = {};
        deviceDesc_t suitableDevice = {};
        suitableDevice.protocol = getDefaultProtocol();
        suitableDevice.platform = X_LINK_ANY_PLATFORM;

        auto status = XLinkFindAllSuitableDevices(state, suitableDevice, deviceDescAll.data(), static_cast<unsigned int>(deviceDescAll.size()), &numdev);
        if(status != X_LINK_SUCCESS && status != X_LINK_DEVICE_NOT_FOUND) {
            throw std::runtime_error("Couldn't retrieve all connected devices");
        }

        for(unsigned i = 0; i < numdev; i++) {
            DeviceInfo info = {};
            if(skipInvalidDevices && std::strcmp("<error>", deviceDescAll.at(i).name) == 0) {
                spdlog::warn("skipping {} device having name \"{}\"", XLinkDeviceStateToStr(state), deviceDescAll.at(i).name);
                continue;
            }
            info.desc = deviceDescAll.at(i);
            info.state = state;
            bool allowedId = allowedDeviceIds.find(info.getMxId()) != std::string::npos || allowedDeviceIds.empty();
            if(allowedId) {
                devices.push_back(info);
            }
        }
    }

    return devices;
}

std::tuple<bool, DeviceInfo> XLinkConnection::getFirstDevice(XLinkDeviceState_t state, bool skipInvalidDevice) {
    auto devices = getAllConnectedDevices(state, skipInvalidDevice);
    for(const auto& d : devices) {
        if(d.state == state || state == X_LINK_ANY_STATE) return {true, d};
    }
    return {false, DeviceInfo()};
}

std::tuple<bool, DeviceInfo> XLinkConnection::getDeviceByMxId(std::string mxId, XLinkDeviceState_t state, bool skipInvalidDevice) {
    auto devices = getAllConnectedDevices(state, skipInvalidDevice);
    for(const auto& d : devices) {
        if(d.state == state || state == X_LINK_ANY_STATE) {
            if(d.getMxId() == mxId) {
                return {true, d};
            }
        }
    }
    return {false, DeviceInfo()};
}

DeviceInfo XLinkConnection::bootBootloader(const DeviceInfo& deviceInfo) {
    using namespace std::chrono;

    // Device is in flash booted state. Boot to bootloader first
    XLinkBootBootloader(&deviceInfo.desc);

    // Fix deviceInfo for BOOTLOADER state
    DeviceInfo deviceToWait = deviceInfoFix(deviceInfo, X_LINK_BOOTLOADER);

    // Device desc if found
    deviceDesc_t foundDeviceDesc = {};

    // Wait for device to get to bootloader state
    XLinkError_t rc;
    auto bootupTimeout = WAIT_FOR_BOOTUP_TIMEOUT_USB;
    if(deviceToWait.desc.protocol == X_LINK_TCP_IP) {
        bootupTimeout = WAIT_FOR_BOOTUP_TIMEOUT_TCPIP;
    }

    // Override with environment variables, if set
    const std::vector<std::pair<std::string, std::chrono::milliseconds*>> evars = {
        {"DEPTHAI_BOOTUP_TIMEOUT", &bootupTimeout},
    };

    for(auto ev : evars) {
        auto name = ev.first;
        auto valstr = utility::getEnv(name);
        if(!valstr.empty()) {
            try {
                std::chrono::milliseconds value{std::stoi(valstr)};
                *ev.second = value;
                // auto initial = *ev.second;
                // spdlog::warn("{} override: {} -> {}", name, initial, value);
            } catch(const std::invalid_argument& e) {
                spdlog::warn("{} value invalid: {}", name, e.what());
            }
        }
    }

    auto tstart = steady_clock::now();
    do {
        rc = XLinkFindFirstSuitableDevice(X_LINK_BOOTLOADER, deviceToWait.desc, &foundDeviceDesc);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if(rc == X_LINK_SUCCESS) break;
    } while(steady_clock::now() - tstart < bootupTimeout);

    // If device not found
    if(rc != X_LINK_SUCCESS) {
        throw std::runtime_error("Failed to find device (" + std::string(deviceToWait.desc.name) + "), error message: " + convertErrorCodeToString(rc));
    }

    deviceToWait.state = X_LINK_BOOTLOADER;
    deviceToWait.desc = foundDeviceDesc;
    return deviceToWait;
}

XLinkConnection::XLinkConnection(const DeviceInfo& deviceDesc, std::vector<std::uint8_t> mvcmdBinary, XLinkDeviceState_t expectedState) {
    bootDevice = true;
    bootWithPath = false;
    this->mvcmd = std::move(mvcmdBinary);
    initDevice(deviceDesc, expectedState);
}

XLinkConnection::XLinkConnection(const DeviceInfo& deviceDesc, dai::Path pathToMvcmd, XLinkDeviceState_t expectedState) {
    if(!pathToMvcmd.empty()) {
        std::ifstream f(pathToMvcmd);
        if(!f.good()) throw std::runtime_error("Error path doesn't exist. Note: Environment variables in path are not expanded. (E.g. '~', '$PATH').");
    }
    bootDevice = true;
    bootWithPath = true;
    this->pathToMvcmd = std::move(pathToMvcmd);
    initDevice(deviceDesc, expectedState);
}

// Skip boot
XLinkConnection::XLinkConnection(const DeviceInfo& deviceDesc, XLinkDeviceState_t expectedState) {
    bootDevice = false;
    initDevice(deviceDesc, expectedState);
}

bool XLinkConnection::isClosed() const {
    return closed;
}

void XLinkConnection::checkClosed() const {
    if(isClosed()) throw std::invalid_argument("XLinkConnection already closed or disconnected");
}

void XLinkConnection::close() {
    if(closed.exchange(true)) return;

    using namespace std::chrono;
    constexpr auto RESET_TIMEOUT = seconds(2);
    constexpr auto BOOTUP_SEARCH = seconds(5);

    if(deviceLinkId != -1 && rebootOnDestruction) {
        auto tmp = deviceLinkId;

        auto ret = XLinkResetRemoteTimeout(deviceLinkId, duration_cast<milliseconds>(RESET_TIMEOUT).count());
        if(ret != X_LINK_SUCCESS) {
            spdlog::debug("XLinkResetRemoteTimeout returned: {}", XLinkErrorToStr(ret));
        }

        deviceLinkId = -1;

        // TODO(themarpe) - revisit for TCPIP protocol

        // Wait till same device reappears (is rebooted).
        // Only in case if device was booted to begin with
        if(bootDevice) {
            auto t1 = steady_clock::now();
            bool found = false;
            do {
                DeviceInfo tmp;
                for(const auto& state : {X_LINK_UNBOOTED, X_LINK_BOOTLOADER}) {
                    std::tie(found, tmp) = XLinkConnection::getDeviceByMxId(deviceInfo.getMxId(), state, false);
                    if(found) break;
                }
            } while(!found && steady_clock::now() - t1 < BOOTUP_SEARCH);
        }

        spdlog::debug("XLinkResetRemote of linkId: ({})", tmp);
    }
}

XLinkConnection::~XLinkConnection() {
    close();
}

void XLinkConnection::setRebootOnDestruction(bool reboot) {
    rebootOnDestruction = reboot;
}

bool XLinkConnection::getRebootOnDestruction() const {
    return rebootOnDestruction;
}

bool XLinkConnection::bootAvailableDevice(const deviceDesc_t& deviceToBoot, const dai::Path& pathToMvcmd) {
    std::ifstream fwStream(pathToMvcmd, std::ios::binary);
    if(!fwStream.is_open()) throw std::runtime_error(fmt::format("Cannot boot firmware, binary at path: {} doesn't exist", pathToMvcmd));
    std::vector<uint8_t> package = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(fwStream), {});
    return bootAvailableDevice(deviceToBoot, package);
}

bool XLinkConnection::bootAvailableDevice(const deviceDesc_t& deviceToBoot, std::vector<std::uint8_t>& mvcmd) {
    auto status = XLinkBootMemory(&deviceToBoot, mvcmd.data(), static_cast<unsigned long>(mvcmd.size()));
    return status == X_LINK_SUCCESS;
}

void XLinkConnection::initDevice(const DeviceInfo& deviceToInit, XLinkDeviceState_t expectedState) {
    initialize();
    assert(deviceLinkId == -1);

    XLinkError_t rc = X_LINK_ERROR;
    deviceDesc_t deviceDesc = {};

    using namespace std::chrono;

    // if device is in UNBOOTED then boot
    bootDevice = deviceToInit.state == X_LINK_UNBOOTED;

    std::chrono::milliseconds connectTimeout = WAIT_FOR_CONNECT_TIMEOUT;
    std::chrono::milliseconds bootupTimeout = WAIT_FOR_BOOTUP_TIMEOUT_USB;
    if(deviceToInit.desc.protocol == X_LINK_TCP_IP) {
        bootupTimeout = WAIT_FOR_BOOTUP_TIMEOUT_TCPIP;
    }

    // Override with environment variables, if set
    const std::vector<std::pair<std::string, std::chrono::milliseconds*>> evars = {
        {"DEPTHAI_CONNECT_TIMEOUT", &connectTimeout},
        {"DEPTHAI_BOOTUP_TIMEOUT", &bootupTimeout},
    };

    for(auto ev : evars) {
        auto name = ev.first;
        auto valstr = utility::getEnv(name);
        if(!valstr.empty()) {
            try {
                std::chrono::milliseconds value{std::stoi(valstr)};
                *ev.second = value;
                // auto initial = *ev.second;
            } catch(const std::invalid_argument& e) {
                spdlog::warn("{} value invalid: {}", name, e.what());
            }
        }
    }

    // boot device
    if(bootDevice) {
        DeviceInfo deviceToBoot = deviceInfoFix(deviceToInit, X_LINK_UNBOOTED);
        deviceDesc_t foundDeviceDesc = {};

        // Wait for the device to be available
        auto tstart = steady_clock::now();
        do {
            rc = XLinkFindFirstSuitableDevice(X_LINK_UNBOOTED, deviceToBoot.desc, &foundDeviceDesc);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            if(rc == X_LINK_SUCCESS) break;
        } while(steady_clock::now() - tstart < bootupTimeout);

        // If device not found
        if(rc != X_LINK_SUCCESS) {
            throw std::runtime_error("Failed to find device (" + std::string(deviceToBoot.desc.name) + "), error message: " + convertErrorCodeToString(rc));
        }

        bool bootStatus;
        if(bootWithPath) {
            bootStatus = bootAvailableDevice(foundDeviceDesc, pathToMvcmd);
        } else {
            bootStatus = bootAvailableDevice(foundDeviceDesc, mvcmd);
        }
        if(!bootStatus) {
            throw std::runtime_error("Failed to boot device!");
        }
    }

    // Search for booted device
    {
        // Create description of device to look for
        DeviceInfo bootedDeviceInfo = deviceToInit;
        if(deviceToInit.desc.protocol != X_LINK_TCP_IP) {
            bootedDeviceInfo = deviceInfoFix(deviceToInit, expectedState);
        }

        // Find booted device
        auto tstart = steady_clock::now();
        do {
            rc = XLinkFindFirstSuitableDevice(expectedState, bootedDeviceInfo.desc, &deviceDesc);
            if(rc == X_LINK_SUCCESS) break;
        } while(steady_clock::now() - tstart < bootupTimeout);

        if(rc != X_LINK_SUCCESS) {
            throw std::runtime_error("Failed to find device after booting, error message: " + convertErrorCodeToString(rc));
        }
    }

    // Try to connect to device
    {
        XLinkHandler_t connectionHandler = {};
        connectionHandler.devicePath = deviceDesc.name;
        connectionHandler.protocol = deviceDesc.protocol;

        auto tstart = steady_clock::now();
        do {
            if((rc = XLinkConnect(&connectionHandler)) == X_LINK_SUCCESS) break;
        } while(steady_clock::now() - tstart < connectTimeout);

        if(rc != X_LINK_SUCCESS) throw std::runtime_error("Failed to connect to device, error message: " + convertErrorCodeToString(rc));

        deviceLinkId = connectionHandler.linkId;
        deviceInfo.desc = deviceDesc;
        deviceInfo.state = X_LINK_BOOTED;
    }
}

int XLinkConnection::getLinkId() const {
    return deviceLinkId;
}

std::string XLinkConnection::convertErrorCodeToString(XLinkError_t errorCode) {
    switch(errorCode) {
        case X_LINK_SUCCESS:
            return "X_LINK_SUCCESS";
        case X_LINK_ALREADY_OPEN:
            return "X_LINK_ALREADY_OPEN";
        case X_LINK_COMMUNICATION_NOT_OPEN:
            return "X_LINK_COMMUNICATION_NOT_OPEN";
        case X_LINK_COMMUNICATION_FAIL:
            return "X_LINK_COMMUNICATION_FAIL";
        case X_LINK_COMMUNICATION_UNKNOWN_ERROR:
            return "X_LINK_COMMUNICATION_UNKNOWN_ERROR";
        case X_LINK_DEVICE_NOT_FOUND:
            return "X_LINK_DEVICE_NOT_FOUND";
        case X_LINK_TIMEOUT:
            return "X_LINK_TIMEOUT";
        case X_LINK_ERROR:
            return "X_LINK_ERROR";
        case X_LINK_OUT_OF_MEMORY:
            return "X_LINK_OUT_OF_MEMORY";
        case X_LINK_NOT_IMPLEMENTED:
            return "X_LINK_NOT_IMPLEMENTED";
        default:
            return "<UNKNOWN ERROR>";
    }
}

DeviceInfo deviceInfoFix(const DeviceInfo& dev, XLinkDeviceState_t state) {
    if(dev.desc.protocol == X_LINK_TCP_IP) {
        // X_LINK_TCP_IP doesn't need a fix
        return dev;
    }

    DeviceInfo fixed(dev);

    // Remove everything after dash
    for(int i = sizeof(fixed.desc.name) - 1; i >= 0; i--) {
        if(fixed.desc.name[i] != '-')
            fixed.desc.name[i] = 0;
        else
            break;
    }

    // If bootloader state add "bootloader"
    if(state == X_LINK_BOOTLOADER) {
        std::strncat(fixed.desc.name, "bootloader", sizeof(fixed.desc.name) - std::strlen(fixed.desc.name));
        // set platform to any
        fixed.desc.platform = X_LINK_ANY_PLATFORM;
    } else if(state == X_LINK_UNBOOTED) {
        // if unbooted add ending ("ma2480" or "ma2450")
        if(fixed.desc.platform == X_LINK_MYRIAD_2) {
            std::strncat(fixed.desc.name, "ma2450", sizeof(fixed.desc.name) - std::strlen(fixed.desc.name));
        } else {
            std::strncat(fixed.desc.name, "ma2480", sizeof(fixed.desc.name) - std::strlen(fixed.desc.name));
        }
    } else if(state == X_LINK_BOOTED) {
        // set platform to any
        fixed.desc.platform = X_LINK_ANY_PLATFORM;
    }

    return fixed;
}

}  // namespace dai
