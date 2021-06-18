#include "xlink/XLinkConnection.hpp"

#include <array>
#include <cassert>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>

// libraries
#include <XLink/XLink.h>
extern "C" {
#include <XLink/XLinkLog.h>
}

#include <spdlog/spdlog.h>

namespace dai {

// DeviceInfo
DeviceInfo::DeviceInfo(std::string mxId) {
    // Add dash at the end of mxId ([mxId]-[xlinkDevName] format)
    mxId += "-";
    // Construct device info which will points to device with specific mxId
    std::strncpy(desc.name, mxId.c_str(), sizeof(desc.name));

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
constexpr std::chrono::milliseconds XLinkConnection::WAIT_FOR_BOOTUP_TIMEOUT;
constexpr std::chrono::milliseconds XLinkConnection::WAIT_FOR_CONNECT_TIMEOUT;

void XLinkConnection::initXLinkGlobal() {
    if(xlinkGlobalInitialized.exchange(true)) return;

    xlinkGlobalHandler.protocol = X_LINK_USB_VSC;
    auto status = XLinkInitialize(&xlinkGlobalHandler);
    if(X_LINK_SUCCESS != status) {
        throw std::runtime_error("Couldn't initialize XLink");
    }

    // Suppress XLink related errors
    mvLogDefaultLevelSet(MVLOG_LAST);
}

std::atomic<bool> XLinkConnection::xlinkGlobalInitialized{false};
XLinkGlobalHandler_t XLinkConnection::xlinkGlobalHandler = {};

std::vector<DeviceInfo> XLinkConnection::getAllConnectedDevices(XLinkDeviceState_t state) {
    initXLinkGlobal();

    std::vector<DeviceInfo> devices;

    std::vector<XLinkDeviceState_t> states;
    if(state == X_LINK_ANY_STATE) {
        states = {X_LINK_UNBOOTED, X_LINK_BOOTLOADER, X_LINK_BOOTED};
    } else {
        states = {state};
    }

    // Get all available devices (unbooted & booted)
    for(const auto& state : states) {
        unsigned int numdev = 0;
        std::array<deviceDesc_t, 32> deviceDescAll = {};
        deviceDesc_t suitableDevice = {};
        suitableDevice.protocol = X_LINK_ANY_PROTOCOL;
        suitableDevice.platform = X_LINK_ANY_PLATFORM;

        auto status = XLinkFindAllSuitableDevices(state, suitableDevice, deviceDescAll.data(), static_cast<unsigned int>(deviceDescAll.size()), &numdev);
        if(status != X_LINK_SUCCESS) throw std::runtime_error("Couldn't retrieve all connected devices");

        for(unsigned i = 0; i < numdev; i++) {
            DeviceInfo info = {};
            info.desc = deviceDescAll.at(i);
            info.state = state;
            devices.push_back(info);
        }
    }

    return devices;
}

std::tuple<bool, DeviceInfo> XLinkConnection::getFirstDevice(XLinkDeviceState_t state) {
    auto devices = getAllConnectedDevices();
    for(const auto& d : devices) {
        if(d.state == state || state == X_LINK_ANY_STATE) return {true, d};
    }
    return {false, DeviceInfo()};
}

std::tuple<bool, DeviceInfo> XLinkConnection::getDeviceByMxId(std::string mxId, XLinkDeviceState_t state) {
    auto devices = getAllConnectedDevices();
    for(const auto& d : devices) {
        if(d.state == state || state == X_LINK_ANY_STATE) {
            if(d.getMxId() == mxId) {
                return {true, d};
            }
        }
    }
    return {false, DeviceInfo()};
}

XLinkConnection::XLinkConnection(const DeviceInfo& deviceDesc, std::vector<std::uint8_t> mvcmdBinary, XLinkDeviceState_t expectedState) {
    bootDevice = true;
    bootWithPath = false;
    this->mvcmd = std::move(mvcmdBinary);
    initDevice(deviceDesc, expectedState);
}

XLinkConnection::XLinkConnection(const DeviceInfo& deviceDesc, std::string pathToMvcmd, XLinkDeviceState_t expectedState) {
    if(!pathToMvcmd.empty()) {
        std::ifstream f(pathToMvcmd.c_str());
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

    if(deviceLinkId != -1 && rebootOnDestruction) {
        auto tmp = deviceLinkId;
        XLinkResetRemote(deviceLinkId);
        deviceLinkId = -1;

        // TODO(themarpe) - revisit for TCPIP protocol
        using namespace std::chrono;
        const auto BOOTUP_SEARCH = std::chrono::seconds(5);

        // Wait till same device reappears (is rebooted).
        // Only in case if device was booted to begin with
        if(bootDevice) {
            auto t1 = steady_clock::now();
            bool found = false;
            do {
                DeviceInfo tmp;
                for(const auto& state : {X_LINK_UNBOOTED, X_LINK_BOOTLOADER}) {
                    std::tie(found, tmp) = XLinkConnection::getDeviceByMxId(deviceInfo.getMxId(), state);
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

bool XLinkConnection::bootAvailableDevice(const deviceDesc_t& deviceToBoot, const std::string& pathToMvcmd) {
    auto status = XLinkBoot(&deviceToBoot, pathToMvcmd.c_str());
    return status == X_LINK_SUCCESS;
}

bool XLinkConnection::bootAvailableDevice(const deviceDesc_t& deviceToBoot, std::vector<std::uint8_t>& mvcmd) {
    auto status = XLinkBootMemory(&deviceToBoot, mvcmd.data(), static_cast<unsigned long>(mvcmd.size()));
    return status == X_LINK_SUCCESS;
}

void XLinkConnection::initDevice(const DeviceInfo& deviceToInit, XLinkDeviceState_t expectedState) {
    initXLinkGlobal();
    assert(deviceLinkId == -1);

    XLinkError_t rc = X_LINK_ERROR;
    deviceDesc_t deviceDesc = {};

    using namespace std::chrono;

    // if device is in UNBOOTED then boot
    bootDevice = deviceToInit.state == X_LINK_UNBOOTED;

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
        } while(steady_clock::now() - tstart < WAIT_FOR_BOOTUP_TIMEOUT);

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
        } while(steady_clock::now() - tstart < WAIT_FOR_BOOTUP_TIMEOUT);

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
        } while(steady_clock::now() - tstart < WAIT_FOR_CONNECT_TIMEOUT);

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
