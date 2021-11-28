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

// libraries
#include <XLink/XLink.h>
extern "C" {
#include <XLink/XLinkLog.h>
}

#include <spdlog/spdlog.h>

namespace dai {

DeviceInfo::DeviceInfo(const deviceDesc_t& desc){
    name = std::string(desc.name);
    mxid = std::string(desc.mxid);
    state = desc.state;
    protocol = desc.protocol;
    platform = desc.platform;
}

DeviceInfo::DeviceInfo(std::string mxidOrName){
    // Parse parameter and set to ip if any dots found
    // mxid doesn't have a dot in the name
    if(mxidOrName.find(".") != std::string::npos){
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
    strncpy(desc.mxid, mxid.c_str(), sizeof(desc.mxid));
    strncpy(desc.name, name.c_str(), sizeof(desc.name));

    desc.platform = platform;
    desc.protocol = protocol;
    desc.state = state;

    return desc;
}

// backward compatibility
std::string DeviceInfo::getMxId() const {
    return mxid;
}

std::string DeviceInfo::toString() const {
    return fmt::format("{}, {}, {}, {}, {}", name, mxid, XLinkDeviceStateToStr(state), XLinkProtocolToStr(protocol), XLinkPlatformToStr(platform));
}

// STATIC
constexpr std::chrono::milliseconds XLinkConnection::WAIT_FOR_BOOTUP_TIMEOUT;
constexpr std::chrono::milliseconds XLinkConnection::WAIT_FOR_CONNECT_TIMEOUT;
constexpr std::chrono::milliseconds XLinkConnection::POLLING_DELAY_TIME;

std::vector<DeviceInfo> XLinkConnection::getAllConnectedDevices(XLinkDeviceState_t state) {
    initialize();

    std::vector<DeviceInfo> devices;

    unsigned int numdev = 0;
    std::array<deviceDesc_t, 32> deviceDescAll = {};
    deviceDesc_t suitableDevice = {};
    suitableDevice.protocol = X_LINK_ANY_PROTOCOL;
    suitableDevice.platform = X_LINK_ANY_PLATFORM;
    suitableDevice.state = state;

    auto status = XLinkFindAllSuitableDevices(suitableDevice, deviceDescAll.data(), static_cast<unsigned int>(deviceDescAll.size()), &numdev);
    if(status != X_LINK_SUCCESS) throw std::runtime_error("Couldn't retrieve all connected devices");

    for(unsigned i = 0; i < numdev; i++) {
        devices.push_back(DeviceInfo(deviceDescAll.at(i)));
    }

    return devices;
}

std::tuple<bool, DeviceInfo> XLinkConnection::getFirstDevice(XLinkDeviceState_t state) {

    deviceDesc_t devReq = {};
    devReq.protocol = X_LINK_ANY_PROTOCOL;
    devReq.platform = X_LINK_ANY_PLATFORM;
    devReq.name[0] = 0;
    devReq.mxid[0] = 0;
    devReq.state = state;

    deviceDesc_t dev = {};
    auto res = XLinkFindFirstSuitableDevice(devReq, &dev);
    if(res == X_LINK_SUCCESS){
        DeviceInfo info(dev);
        return {true, info};
    }
    return {false, {}};
}

std::tuple<bool, DeviceInfo> XLinkConnection::getDeviceByMxId(std::string mxId, XLinkDeviceState_t state) {
    DeviceInfo dev;
    dev.mxid = mxId;
    dev.state = state;

    deviceDesc_t desc = {};
    auto res = XLinkFindFirstSuitableDevice(dev.getXLinkDeviceDesc(), &desc);
    if(res == X_LINK_SUCCESS){
        return {true, {desc}};
    }
    return {false, {}};
}

DeviceInfo XLinkConnection::bootBootloader(const DeviceInfo& deviceInfo) {
    using namespace std::chrono;

    // Device is in flash booted state. Boot to bootloader first
    auto deviceDesc = deviceInfo.getXLinkDeviceDesc();
    XLinkBootBootloader(&deviceDesc);

    // Wait for a bootloader device now
    DeviceInfo deviceToWait = deviceInfo;
    if(deviceInfo.protocol == X_LINK_USB_VSC){
        deviceToWait.name = "";
    }
    deviceToWait.state = X_LINK_BOOTLOADER;

    // Device desc if found
    deviceDesc_t foundDeviceDesc = {};

    // Wait for device to get to bootloader state
    XLinkError_t rc;
    auto tstart = steady_clock::now();
    do {
        rc = XLinkFindFirstSuitableDevice(deviceToWait.getXLinkDeviceDesc(), &foundDeviceDesc);
        if(rc == X_LINK_SUCCESS) break;
        std::this_thread::sleep_for(POLLING_DELAY_TIME);
    } while(steady_clock::now() - tstart < WAIT_FOR_BOOTUP_TIMEOUT);

    // If device not found
    if(rc != X_LINK_SUCCESS) {
        throw std::runtime_error(fmt::format("Failed to find device (mxid: {}), error message: {}", deviceToWait.mxid, convertErrorCodeToString(rc)));
    }

    return DeviceInfo(foundDeviceDesc);
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
                    std::tie(found, tmp) = XLinkConnection::getDeviceByMxId(deviceInfo.getMxId(), state);
                    if(found) break;
                    std::this_thread::sleep_for(POLLING_DELAY_TIME);
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
    initialize();
    assert(deviceLinkId == -1);

    XLinkError_t rc = X_LINK_ERROR;

    using namespace std::chrono;

    // if device is in UNBOOTED then boot
    bootDevice = deviceToInit.state == X_LINK_UNBOOTED;

    DeviceInfo lastDeviceInfo = deviceToInit;

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
        } while(steady_clock::now() - tstart < WAIT_FOR_BOOTUP_TIMEOUT);

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
        // If USB, search by mxid only as name usually changes
        if(deviceToInit.protocol == X_LINK_USB_VSC) {
            bootedDeviceInfo.name = "";
        }
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
        } while(steady_clock::now() - tstart < WAIT_FOR_BOOTUP_TIMEOUT);

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
        } while(steady_clock::now() - tstart < WAIT_FOR_CONNECT_TIMEOUT);

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
