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

DeviceInfo::DeviceInfo(const deviceDesc_t& desc) {
    name = std::string(desc.name);
    mxid = std::string(desc.mxid);
    state = desc.state;
    protocol = desc.protocol;
    platform = desc.platform;
    status = desc.status;
}

DeviceInfo::DeviceInfo(std::string name, std::string mxid, XLinkDeviceState_t state, XLinkProtocol_t protocol, XLinkPlatform_t platform, XLinkError_t status)
    : name(std::move(name)), mxid(std::move(mxid)), state(state), protocol(protocol), platform(platform), status(status) {}

DeviceInfo::DeviceInfo(std::string mxidOrName) {
    // Parse parameter and set to ip if any dots found
    // mxid doesn't have a dot in the name
    if(mxidOrName.find(".") != std::string::npos) {
        // This is reasoned as an IP address or USB path (name). Set rest of info accordingly
        name = std::move(mxidOrName);
        mxid = "";
    } else {
        // This is reasoned as mxid
        name = "";
        mxid = std::move(mxidOrName);
    }
}

deviceDesc_t DeviceInfo::getXLinkDeviceDesc() const {
    // Create XLink deviceDesc_t, init all fields to zero
    deviceDesc_t desc = {};

    // c_str is guranteed to be nullterminated
    desc.mxid[sizeof(desc.mxid) - 1] = 0;
    strncpy(desc.mxid, mxid.c_str(), sizeof(desc.mxid) - 1);
    desc.name[sizeof(desc.name) - 1] = 0;
    strncpy(desc.name, name.c_str(), sizeof(desc.name) - 1);

    desc.platform = platform;
    desc.protocol = protocol;
    desc.state = state;
    desc.status = status;

    return desc;
}

// backward compatibility
std::string DeviceInfo::getMxId() const {
    return mxid;
}

std::string DeviceInfo::toString() const {
    return fmt::format("DeviceInfo(name={}, mxid={}, {}, {}, {}, {})",
                       name,
                       mxid,
                       XLinkDeviceStateToStr(state),
                       XLinkProtocolToStr(protocol),
                       XLinkPlatformToStr(platform),
                       XLinkErrorToStr(status));
}

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

// STATIC
constexpr std::chrono::milliseconds XLinkConnection::WAIT_FOR_BOOTUP_TIMEOUT;
constexpr std::chrono::milliseconds XLinkConnection::WAIT_FOR_CONNECT_TIMEOUT;
constexpr std::chrono::milliseconds XLinkConnection::POLLING_DELAY_TIME;

std::vector<DeviceInfo> XLinkConnection::getAllConnectedDevices(XLinkDeviceState_t state, bool skipInvalidDevices) {
    initialize();

    std::vector<DeviceInfo> devices;

    unsigned int numdev = 0;
    std::array<deviceDesc_t, 32> deviceDescAll = {};
    deviceDesc_t suitableDevice = {};
    suitableDevice.protocol = getDefaultProtocol();
    suitableDevice.platform = X_LINK_ANY_PLATFORM;
    suitableDevice.state = state;

    auto allowedDeviceIds = utility::getEnv("DEPTHAI_DEVICE_MXID_LIST");

    auto status = XLinkFindAllSuitableDevices(suitableDevice, deviceDescAll.data(), static_cast<unsigned int>(deviceDescAll.size()), &numdev);
    if(status != X_LINK_SUCCESS) throw std::runtime_error("Couldn't retrieve all connected devices");

    for(unsigned i = 0; i < numdev; i++) {
        DeviceInfo info(deviceDescAll.at(i));

        if(skipInvalidDevices) {
            if(info.status == X_LINK_SUCCESS) {
                // device is okay
            } else if(info.status == X_LINK_INSUFFICIENT_PERMISSIONS) {
                spdlog::warn("Insufficient permissions to communicate with {} device having name \"{}\". Make sure udev rules are set",
                             XLinkDeviceStateToStr(info.state),
                             info.name);
                continue;
            } else {
                spdlog::warn("skipping {} device having name \"{}\"", XLinkDeviceStateToStr(info.state), info.name);
                continue;
            }
        }

        bool allowedId = allowedDeviceIds.find(info.getMxId()) != std::string::npos || allowedDeviceIds.empty();
        if(allowedId) {
            devices.push_back(info);
        }
    }

    return devices;
}

std::tuple<bool, DeviceInfo> XLinkConnection::getFirstDevice(XLinkDeviceState_t state, bool skipInvalidDevice) {
    initialize();

    DeviceInfo devReq = {};
    devReq.protocol = X_LINK_ANY_PROTOCOL;
    devReq.platform = X_LINK_ANY_PLATFORM;
    devReq.name = "";
    devReq.mxid = "";
    devReq.state = state;

    deviceDesc_t desc = {};
    auto res = XLinkFindFirstSuitableDevice(devReq.getXLinkDeviceDesc(), &desc);
    if(res == X_LINK_SUCCESS) {
        if(skipInvalidDevice) {
            if(desc.status == X_LINK_SUCCESS) {
                // device is okay
            } else if(desc.status == X_LINK_INSUFFICIENT_PERMISSIONS) {
                spdlog::warn("Insufficient permissions to communicate with {} device having name \"{}\". Make sure udev rules are set",
                             XLinkDeviceStateToStr(desc.state),
                             desc.name);
                return {false, {}};
            } else {
                spdlog::warn("skipping {} device having name \"{}\"", XLinkDeviceStateToStr(desc.state), desc.name);
                return {false, {}};
            }
        }
        DeviceInfo info(desc);
        return {true, info};
    }
    return {false, {}};
}

std::tuple<bool, DeviceInfo> XLinkConnection::getDeviceByMxId(std::string mxId, XLinkDeviceState_t state, bool skipInvalidDevices) {
    initialize();

    DeviceInfo dev;
    dev.mxid = mxId;
    dev.state = state;

    deviceDesc_t desc = {};
    auto res = XLinkFindFirstSuitableDevice(dev.getXLinkDeviceDesc(), &desc);
    if(res == X_LINK_SUCCESS) {
        if(skipInvalidDevices) {
            if(desc.status == X_LINK_SUCCESS) {
                // device is okay
            } else if(desc.status == X_LINK_INSUFFICIENT_PERMISSIONS) {
                spdlog::warn("Insufficient permissions to communicate with {} device having name \"{}\". Make sure udev rules are set",
                             XLinkDeviceStateToStr(desc.state),
                             desc.name);
                return {false, {}};
            } else {
                spdlog::warn("skipping {} device having name \"{}\"", XLinkDeviceStateToStr(desc.state), desc.name);
                return {false, {}};
            }
        }
        return {true, DeviceInfo{desc}};
    }
    return {false, {}};
}

DeviceInfo XLinkConnection::bootBootloader(const DeviceInfo& deviceInfo) {
    initialize();

    using namespace std::chrono;

    // Device is in flash booted state. Boot to bootloader first
    auto deviceDesc = deviceInfo.getXLinkDeviceDesc();

    // Device is in flash booted state. Boot to bootloader first
    XLinkError_t bootBootloaderError = XLinkBootBootloader(&deviceDesc);
    if(bootBootloaderError != X_LINK_SUCCESS) {
        throw std::runtime_error(fmt::format("Couldn't boot device to bootloader - {}", XLinkErrorToStr(bootBootloaderError)));
    }

    // Wait for a bootloader device now
    DeviceInfo deviceToWait = deviceInfo;
    if(deviceInfo.protocol == X_LINK_USB_VSC) {
        deviceToWait.name = "";
    }
    deviceToWait.state = X_LINK_BOOTLOADER;

    // Device desc if found
    deviceDesc_t foundDeviceDesc = {};

    // Wait for device to get to bootloader state
    XLinkError_t rc;
    auto bootupTimeout = WAIT_FOR_BOOTUP_TIMEOUT;

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
        rc = XLinkFindFirstSuitableDevice(deviceToWait.getXLinkDeviceDesc(), &foundDeviceDesc);
        if(rc == X_LINK_SUCCESS) break;
        std::this_thread::sleep_for(POLLING_DELAY_TIME);
    } while(steady_clock::now() - tstart < bootupTimeout);

    // If device not found
    if(rc != X_LINK_SUCCESS) {
        throw std::runtime_error(fmt::format("Failed to find device ({}), error message: {}", deviceToWait.toString(), convertErrorCodeToString(rc)));
    }

    return DeviceInfo(foundDeviceDesc);
}

XLinkConnection::XLinkConnection(const DeviceInfo& deviceDesc, std::vector<std::uint8_t> mvcmdBinary, XLinkDeviceState_t expectedState) {
    initialize();

    bootDevice = true;
    bootWithPath = false;
    this->mvcmd = std::move(mvcmdBinary);
    initDevice(deviceDesc, expectedState);
}

XLinkConnection::XLinkConnection(const DeviceInfo& deviceDesc, dai::Path pathToMvcmd, XLinkDeviceState_t expectedState) {
    initialize();

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
    initialize();

    bootDevice = false;
    initDevice(deviceDesc, expectedState);
}

bool XLinkConnection::isClosed() const {
    std::unique_lock<std::mutex> lock(closedMtx);
    return closed;
}

void XLinkConnection::checkClosed() const {
    if(isClosed()) throw std::invalid_argument("XLinkConnection already closed or disconnected");
}

void XLinkConnection::close() {
    std::unique_lock<std::mutex> lock(closedMtx);
    if(closed) return;

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
                std::tie(found, tmp) = XLinkConnection::getDeviceByMxId(deviceInfo.getMxId(), X_LINK_ANY_STATE, false);
                if(found) {
                    if(tmp.state == X_LINK_UNBOOTED || tmp.state == X_LINK_BOOTLOADER) {
                        break;
                    }
                }
            } while(!found && steady_clock::now() - t1 < BOOTUP_SEARCH);
        }

        spdlog::debug("XLinkResetRemote of linkId: ({})", tmp);
    }

    closed = true;
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
    assert(deviceLinkId == -1);

    XLinkError_t rc = X_LINK_ERROR;

    using namespace std::chrono;

    // if device is in UNBOOTED then boot
    bootDevice = deviceToInit.state == X_LINK_UNBOOTED;

    DeviceInfo lastDeviceInfo = deviceToInit;

    std::chrono::milliseconds connectTimeout = WAIT_FOR_CONNECT_TIMEOUT;
    std::chrono::milliseconds bootupTimeout = WAIT_FOR_BOOTUP_TIMEOUT;

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
        DeviceInfo deviceToBoot = lastDeviceInfo;
        deviceToBoot.state = X_LINK_UNBOOTED;

        deviceDesc_t foundDeviceDesc = {};

        // Wait for the device to be available
        auto tstart = steady_clock::now();
        do {
            rc = XLinkFindFirstSuitableDevice(deviceToBoot.getXLinkDeviceDesc(), &foundDeviceDesc);
            if(rc == X_LINK_SUCCESS) break;
            std::this_thread::sleep_for(POLLING_DELAY_TIME);
        } while(steady_clock::now() - tstart < bootupTimeout);

        // If device not found
        if(rc != X_LINK_SUCCESS) {
            throw std::runtime_error("Failed to find device (" + deviceToBoot.name + "), error message: " + convertErrorCodeToString(rc));
        }

        lastDeviceInfo = DeviceInfo(foundDeviceDesc);

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
        DeviceInfo bootedDeviceInfo = lastDeviceInfo;
        // Reset "name" and search for MXID again after booting the device for all cases
        bootedDeviceInfo.name = "";

        // Has to match expected state
        bootedDeviceInfo.state = expectedState;

        spdlog::debug("Searching for device: {}", bootedDeviceInfo.toString());

        // Find booted device
        deviceDesc_t foundDeviceDesc = {};
        auto tstart = steady_clock::now();
        do {
            rc = XLinkFindFirstSuitableDevice(bootedDeviceInfo.getXLinkDeviceDesc(), &foundDeviceDesc);
            if(rc == X_LINK_SUCCESS) break;
            std::this_thread::sleep_for(POLLING_DELAY_TIME);
        } while(steady_clock::now() - tstart < bootupTimeout);

        if(rc != X_LINK_SUCCESS) {
            throw std::runtime_error("Failed to find device after booting, error message: " + convertErrorCodeToString(rc));
        }

        lastDeviceInfo = DeviceInfo(foundDeviceDesc);
    }

    // Try to connect to device
    {
        XLinkHandler_t connectionHandler = {};
        auto desc = lastDeviceInfo.getXLinkDeviceDesc();
        connectionHandler.devicePath = desc.name;
        connectionHandler.protocol = lastDeviceInfo.protocol;

        auto tstart = steady_clock::now();
        do {
            if((rc = XLinkConnect(&connectionHandler)) == X_LINK_SUCCESS) break;
            std::this_thread::sleep_for(POLLING_DELAY_TIME);
        } while(steady_clock::now() - tstart < connectTimeout);

        if(rc != X_LINK_SUCCESS) throw std::runtime_error("Failed to connect to device, error message: " + convertErrorCodeToString(rc));

        deviceLinkId = connectionHandler.linkId;
        deviceInfo = lastDeviceInfo;
        deviceInfo.state = X_LINK_BOOTED;
    }
}

int XLinkConnection::getLinkId() const {
    return deviceLinkId;
}

std::string XLinkConnection::convertErrorCodeToString(XLinkError_t errorCode) {
    return XLinkErrorToStr(errorCode);
}

}  // namespace dai
