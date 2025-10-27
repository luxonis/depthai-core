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
#include "utility/Logging.hpp"

namespace dai {

DeviceInfo::DeviceInfo(const deviceDesc_t& desc) {
    name = std::string(desc.name);
    deviceId = std::string(desc.mxid);
    state = desc.state;
    protocol = desc.protocol;
    platform = desc.platform;
    status = desc.status;
}

DeviceInfo::DeviceInfo(
    std::string name, std::string deviceId, XLinkDeviceState_t state, XLinkProtocol_t protocol, XLinkPlatform_t platform, XLinkError_t status)
    : name(std::move(name)), deviceId(std::move(deviceId)), state(state), protocol(protocol), platform(platform), status(status) {}

DeviceInfo::DeviceInfo(std::string deviceIdOrName) {
    // Parse parameter and set to ip if any dots found
    // deviceId doesn't have a dot in the name
    if(deviceIdOrName.find(".") != std::string::npos) {
        // This is reasoned as an IP address or USB path (name). Set rest of info accordingly
        name = std::move(deviceIdOrName);
        deviceId = "";
    } else {
        // This is reasoned as deviceId
        name = "";
        deviceId = std::move(deviceIdOrName);
    }
}

deviceDesc_t DeviceInfo::getXLinkDeviceDesc() const {
    // Create XLink deviceDesc_t, init all fields to zero
    deviceDesc_t desc = {};

    // c_str is guranteed to be nullterminated
    desc.mxid[sizeof(desc.mxid) - 1] = 0;
    strncpy(desc.mxid, deviceId.c_str(), sizeof(desc.mxid) - 1);
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
    return deviceId;
}

std::string DeviceInfo::getDeviceId() const {
    return deviceId;
}

std::string DeviceInfo::toString() const {
    return fmt::format("DeviceInfo(name={}, deviceId={}, {}, {}, {}, {})",
                       name,
                       deviceId,
                       XLinkDeviceStateToStr(state),
                       XLinkProtocolToStr(protocol),
                       XLinkPlatformToStr(platform),
                       XLinkErrorToStr(status));
}

static XLinkProtocol_t getDefaultProtocol() {
    XLinkProtocol_t defaultProtocol = X_LINK_ANY_PROTOCOL;

    auto protocolStr = utility::getEnvAs<std::string>("DEPTHAI_PROTOCOL", "");

    std::transform(protocolStr.begin(), protocolStr.end(), protocolStr.begin(), ::tolower);
    if(protocolStr.empty() || protocolStr == "any") {
        defaultProtocol = X_LINK_ANY_PROTOCOL;
    } else if(protocolStr == "usb") {
        defaultProtocol = X_LINK_USB_VSC;
    } else if(protocolStr == "tcpip") {
        defaultProtocol = X_LINK_TCP_IP;
    } else if(protocolStr == "tcpshd") {
        defaultProtocol = X_LINK_TCP_IP_OR_LOCAL_SHDMEM;
    } else if(protocolStr == "shdmem") {
        defaultProtocol = X_LINK_LOCAL_SHDMEM;
    } else {
        logger::warn("Unsupported protocol specified");
    }

    return defaultProtocol;
}

static XLinkPlatform_t getDefaultPlatform() {
    XLinkPlatform_t defaultPlatform = X_LINK_ANY_PLATFORM;

    auto protocolStr = utility::getEnvAs<std::string>("DEPTHAI_PLATFORM", "");

    std::transform(protocolStr.begin(), protocolStr.end(), protocolStr.begin(), ::tolower);
    if(protocolStr.empty() || protocolStr == "any") {
        defaultPlatform = X_LINK_ANY_PLATFORM;
    } else if(protocolStr == "rvc2" || protocolStr == "myriadx") {
        defaultPlatform = X_LINK_MYRIAD_X;
    } else if(protocolStr == "rvc3") {
        defaultPlatform = X_LINK_RVC3;
    } else if(protocolStr == "rvc4") {
        defaultPlatform = X_LINK_RVC4;
    } else {
        spdlog::warn("Unsupported platform specified");
    }

    return defaultPlatform;
}

// STATIC
constexpr std::chrono::milliseconds XLinkConnection::WAIT_FOR_BOOTUP_TIMEOUT;
constexpr std::chrono::milliseconds XLinkConnection::WAIT_FOR_CONNECT_TIMEOUT;
constexpr std::chrono::milliseconds XLinkConnection::POLLING_DELAY_TIME;

bool isInCommaSeparatedVar(std::string list, std::string value) {
    const std::string delimiter = ",";
    for(auto& val : utility::splitList(list, delimiter)) {
        if(val == value) {
            return true;
        }
    }
    return false;
}

std::vector<DeviceInfo> filterDevices(const std::vector<DeviceInfo>& deviceInfos, bool skipInvalidDevices = true) {
    auto allowedDeviceMxIds = utility::getEnvAs<std::string>("DEPTHAI_DEVICE_MXID_LIST", "");
    auto allowedDeviceIds = utility::getEnvAs<std::string>("DEPTHAI_DEVICE_ID_LIST", "");
    auto allowedDeviceNames = utility::getEnvAs<std::string>("DEPTHAI_DEVICE_NAME_LIST", "");
    std::vector<DeviceInfo> filteredEnvs;
    for(auto& info : deviceInfos) {
        bool allowedMxId = isInCommaSeparatedVar(allowedDeviceMxIds, info.getDeviceId()) || allowedDeviceMxIds.empty();
        bool allowedId = isInCommaSeparatedVar(allowedDeviceIds, info.getDeviceId()) || allowedDeviceIds.empty();
        bool allowedName = isInCommaSeparatedVar(allowedDeviceNames, info.name) || allowedDeviceNames.empty();
        if(allowedMxId && allowedId && allowedName) {
            filteredEnvs.push_back(info);
            logger::info("Adding device to the filtered list: {}", info.toString());
        } else {
            logger::info("Skipping device: {}", info.toString());
        }
    }
    std::vector<DeviceInfo> filtered;
    for(auto& info : filteredEnvs) {
        if(skipInvalidDevices) {
            if(info.status == X_LINK_SUCCESS) {
                // device is okay
            } else if(info.status == X_LINK_INSUFFICIENT_PERMISSIONS) {
                logger::warn("Insufficient permissions to communicate with {} device having name \"{}\". Make sure udev rules are set",
                             XLinkDeviceStateToStr(info.state),
                             info.name);
                continue;
            } else {
                logger::warn("skipping {} device having name \"{}\"", XLinkDeviceStateToStr(info.state), info.name);
                continue;
            }
        }
        filtered.push_back(info);
    }

    return filtered;
}

std::vector<DeviceInfo> XLinkConnection::getAllConnectedDevices(XLinkDeviceState_t state, bool skipInvalidDevices, int timeoutMs) {
    initialize();

    std::vector<DeviceInfo> devices;
    std::vector<DeviceInfo> devicesFiltered;
    unsigned int numdev = 0;
    std::array<deviceDesc_t, 64> deviceDescAll = {};
    deviceDesc_t suitableDevice = {};
    suitableDevice.protocol = getDefaultProtocol();
    suitableDevice.platform = getDefaultPlatform();
    suitableDevice.state = state;

    auto status = XLinkFindAllSuitableDevices(suitableDevice, deviceDescAll.data(), static_cast<unsigned int>(deviceDescAll.size()), &numdev, timeoutMs);
    if(status == X_LINK_DEVICE_NOT_FOUND) {
        return devices;
    }
    if(status != X_LINK_SUCCESS) {
        throw std::runtime_error(fmt::format("Couldn't retrieve all connected devices - status {}", static_cast<int>(status)));
    }

    for(unsigned i = 0; i < numdev; i++) {
        DeviceInfo info(deviceDescAll.at(i));
        devices.push_back(info);
    }

    devicesFiltered = filterDevices(devices, skipInvalidDevices);
    auto allowedDeviceNames = utility::getEnvAs<std::string>("DEPTHAI_DEVICE_NAME_LIST", "");
    auto splitList = utility::splitList(allowedDeviceNames, ",");
    bool allDevicesInEnvVarsFound = true;
    for(auto& name : splitList) {
        bool found = false;
        for(const auto& existingInfo : devices) {
            if(existingInfo.name == name) {
                found = true;
                break;
            }
        }
        if(!found) {
            allDevicesInEnvVarsFound = false;
            break;
        }
    }

    if(!devicesFiltered.empty() && allDevicesInEnvVarsFound) {  // If a device from the list is found, return it without further searching
        return devicesFiltered;
    }

    // Now also try to find all devices in the DEPTHAI_DEVICE_NAME_LIST (they were not found earlier if they were not in the same subnet)
    for(auto& name : splitList) {
        deviceDesc_t desc = suitableDevice;
        desc.name[sizeof(desc.name) - 1] = 0;
        strncpy(desc.name, name.c_str(), sizeof(desc.name) - 1);
        auto status = XLinkFindAllSuitableDevices(desc, deviceDescAll.data(), static_cast<unsigned int>(deviceDescAll.size()), &numdev, timeoutMs);
        if(status != X_LINK_SUCCESS) throw std::runtime_error("Couldn't retrieve all connected devices while searching by name");
        for(unsigned i = 0; i < numdev; i++) {
            DeviceInfo info(deviceDescAll.at(i));
            Logging::getInstance().logger.debug("Found device by name: {}", info.toString());
            // Check if device info was already found and is between the found devices
            bool alreadyExists = false;
            for(const auto& existingInfo : devices) {
                if(existingInfo.getDeviceId() == info.getDeviceId()) {
                    alreadyExists = true;
                    break;
                }
            }
            if(!alreadyExists) {
                devices.push_back(info);
            }
        }
    }

    devicesFiltered = filterDevices(devices, skipInvalidDevices);
    return devicesFiltered;
}

std::tuple<bool, DeviceInfo> XLinkConnection::getFirstDevice(XLinkDeviceState_t state, bool skipInvalidDevice) {
    initialize();

    DeviceInfo devReq = {};
    devReq.protocol = X_LINK_ANY_PROTOCOL;
    devReq.platform = X_LINK_ANY_PLATFORM;
    devReq.name = "";
    devReq.deviceId = "";
    devReq.state = state;

    deviceDesc_t desc = {};
    auto res = XLinkFindFirstSuitableDevice(devReq.getXLinkDeviceDesc(), &desc);
    if(res == X_LINK_SUCCESS) {
        if(skipInvalidDevice) {
            if(desc.status == X_LINK_SUCCESS) {
                // device is okay
            } else if(desc.status == X_LINK_INSUFFICIENT_PERMISSIONS) {
                logger::warn("Insufficient permissions to communicate with {} device having name \"{}\". Make sure udev rules are set",
                             XLinkDeviceStateToStr(desc.state),
                             desc.name);
                return {false, {}};
            } else {
                logger::warn("skipping {} device having name \"{}\"", XLinkDeviceStateToStr(desc.state), desc.name);
                return {false, {}};
            }
        }
        DeviceInfo info(desc);
        return {true, info};
    }
    return {false, {}};
}

std::tuple<bool, DeviceInfo> XLinkConnection::getDeviceById(std::string deviceId, XLinkDeviceState_t state, bool skipInvalidDevices) {
    initialize();

    DeviceInfo dev;
    dev.deviceId = deviceId;
    dev.state = state;

    deviceDesc_t desc = {};
    auto res = XLinkFindFirstSuitableDevice(dev.getXLinkDeviceDesc(), &desc);
    if(res == X_LINK_SUCCESS) {
        if(skipInvalidDevices) {
            if(desc.status == X_LINK_SUCCESS) {
                // device is okay
            } else if(desc.status == X_LINK_INSUFFICIENT_PERMISSIONS) {
                logger::warn("Insufficient permissions to communicate with {} device having name \"{}\". Make sure udev rules are set",
                             XLinkDeviceStateToStr(desc.state),
                             desc.name);
                return {false, {}};
            } else {
                logger::warn("skipping {} device having name \"{}\"", XLinkDeviceStateToStr(desc.state), desc.name);
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
    deviceToWait.state = X_LINK_BOOTLOADER;

    // Prepare descriptor to search for
    auto descToWait = deviceToWait.getXLinkDeviceDesc();
    // Use "name" as hint only, but might still change
    descToWait.nameHintOnly = true;

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
        auto valstr = utility::getEnvAs<std::string>(name, "");
        if(!valstr.empty()) {
            try {
                std::chrono::milliseconds value{std::stoi(valstr)};
                *ev.second = value;
                // auto initial = *ev.second;
                // logger::warn("{} override: {} -> {}", name, initial, value);
            } catch(const std::invalid_argument& e) {
                logger::warn("{} value invalid: {}", name, e.what());
            }
        }
    }

    auto tstart = steady_clock::now();
    do {
        rc = XLinkFindFirstSuitableDevice(descToWait, &foundDeviceDesc);
        if(rc == X_LINK_SUCCESS) break;
        std::this_thread::sleep_for(POLLING_DELAY_TIME);
    } while(steady_clock::now() - tstart < bootupTimeout);

    // If device not found
    if(rc != X_LINK_SUCCESS) {
        throw std::runtime_error(fmt::format("Failed to find device ({}), error message: {}", deviceToWait.toString(), convertErrorCodeToString(rc)));
    }

    return DeviceInfo(foundDeviceDesc);
}

XLinkConnection::XLinkConnection(const DeviceInfo& deviceDesc, std::vector<std::uint8_t> mvcmdBinary, XLinkDeviceState_t expectedState)
    : bootWithPath(false), mvcmd(std::move(mvcmdBinary)) {
    initialize();
    initDevice(deviceDesc, expectedState);
}

XLinkConnection::XLinkConnection(const DeviceInfo& deviceDesc, std::filesystem::path mvcmdPath, XLinkDeviceState_t expectedState)
    : pathToMvcmd(std::move(mvcmdPath)) {
    initialize();
    if(!pathToMvcmd.empty()) {
        std::ifstream testStream(pathToMvcmd);
        if(!testStream.good()) throw std::runtime_error("Error path doesn't exist. Note: Environment variables in path are not expanded. (E.g. '~', '$PATH').");
    }
    initDevice(deviceDesc, expectedState);
}

// Skip boot
XLinkConnection::XLinkConnection(const DeviceInfo& deviceDesc, XLinkDeviceState_t expectedState) : bootDevice(false) {
    initialize();
    initDevice(deviceDesc, expectedState);
}

// This function is thread-unsafe. The `closed` value is only known and valid
// within the context of the lock_guard. The value is immediately invalid and outdated
// when it is returned by value to the caller
bool XLinkConnection::isClosed() const {
    std::lock_guard<std::mutex> lock(closedMtx);
    return closed;
}

void XLinkConnection::close() {
    std::lock_guard<std::mutex> lock(closedMtx);
    if(closed) return;

    using namespace std::chrono;
    constexpr auto RESET_TIMEOUT = seconds(2);
    constexpr auto BOOTUP_SEARCH = seconds(5);

    if(deviceLinkId != -1 && rebootOnDestruction) {
        auto previousLinkId = deviceLinkId;

        auto ret = XLinkResetRemoteTimeout(deviceLinkId, duration_cast<milliseconds>(RESET_TIMEOUT).count());
        if(ret != X_LINK_SUCCESS) {
            logger::debug("XLinkResetRemoteTimeout returned: {}", XLinkErrorToStr(ret));
        }

        deviceLinkId = -1;

        // TODO(themarpe) - revisit for TCPIP protocol

        // Wait till same device reappears (is rebooted).
        // Only in case if device was booted to begin with
        if(bootDevice) {
            auto t1 = steady_clock::now();
            bool found = false;
            do {
                DeviceInfo rebootingDeviceInfo;
                std::tie(found, rebootingDeviceInfo) = XLinkConnection::getDeviceById(deviceInfo.getDeviceId(), X_LINK_ANY_STATE, false);
                if(found) {
                    if(rebootingDeviceInfo.state == X_LINK_UNBOOTED || rebootingDeviceInfo.state == X_LINK_BOOTLOADER) {
                        break;
                    }
                }
            } while(!found && steady_clock::now() - t1 < BOOTUP_SEARCH);
        }

        logger::debug("XLinkResetRemote of linkId: ({})", previousLinkId);
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

bool XLinkConnection::bootAvailableDevice(const deviceDesc_t& deviceToBoot, const std::filesystem::path& pathToMvcmd) {
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
        auto valstr = utility::getEnvAs<std::string>(name, "");
        if(!valstr.empty()) {
            try {
                std::chrono::milliseconds value{std::stoi(valstr)};
                *ev.second = value;
                // auto initial = *ev.second;
            } catch(const std::invalid_argument& e) {
                logger::warn("{} value invalid: {}", name, e.what());
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

    // Search for booted device (if expected state is specified)
    if(expectedState != X_LINK_ANY_STATE) {
        // Create description of device to look for
        DeviceInfo bootedDeviceInfo = lastDeviceInfo;
        // Has to match expected state
        bootedDeviceInfo.state = expectedState;

        // Prepare descriptor to search for
        auto bootedDescInfo = bootedDeviceInfo.getXLinkDeviceDesc();
        // Use "name" as hint only, but might still change
        bootedDescInfo.nameHintOnly = true;

        logger::debug("Searching for booted device: {}, name used as hint only", bootedDeviceInfo.toString());

        // Find booted device
        deviceDesc_t foundDeviceDesc = {};
        auto tstart = steady_clock::now();
        do {
            rc = XLinkFindFirstSuitableDevice(bootedDescInfo, &foundDeviceDesc);
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

        if(rc != X_LINK_SUCCESS) {
            if(rc == X_LINK_INSUFFICIENT_PERMISSIONS) {
                throw std::runtime_error(
                    fmt::format("Insufficient permissions to communicate with {} device with name \"{}\". Make sure udev rules are set. Error: {}",
                                XLinkDeviceStateToStr(desc.state),
                                desc.name,
                                convertErrorCodeToString(rc)));
            } else if(rc == X_LINK_DEVICE_ALREADY_IN_USE) {
                throw std::runtime_error(fmt::format(
                    "Cannot connect to device with name \"{}\", it is used by another process. Error: {}", desc.name, convertErrorCodeToString(rc)));
            } else {
                throw std::runtime_error(fmt::format("Failed to connect to device with name \"{}\". Error: {}", desc.name, convertErrorCodeToString(rc)));
            }
        }

        deviceLinkId = connectionHandler.linkId;
        deviceInfo = lastDeviceInfo;
        deviceInfo.state = X_LINK_BOOTED;
        deviceInfo.protocol = connectionHandler.protocol;
    }
}

int XLinkConnection::getLinkId() const {
    return deviceLinkId;
}

std::string XLinkConnection::convertErrorCodeToString(XLinkError_t errorCode) {
    return XLinkErrorToStr(errorCode);
}

ProfilingData XLinkConnection::getGlobalProfilingData() {
    ProfilingData data;
    XLinkProf_t prof;
    if(XLinkGetGlobalProfilingData(&prof) != X_LINK_SUCCESS) {
        throw std::runtime_error("Couldn't retrieve profiling data");
    }
    data.numBytesRead = prof.totalReadBytes;
    data.numBytesWritten = prof.totalWriteBytes;
    return data;
}

ProfilingData XLinkConnection::getProfilingData() {
    ProfilingData data;
    XLinkProf_t prof;
    if(XLinkGetProfilingData(deviceLinkId, &prof) != X_LINK_SUCCESS) {
        throw std::runtime_error("Couldn't retrieve profiling data");
    }
    data.numBytesRead = prof.totalReadBytes;
    data.numBytesWritten = prof.totalWriteBytes;
    return data;
}

}  // namespace dai
