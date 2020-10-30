#include "xlink/XLinkConnection.hpp"

#include <XLink.h>

#include <array>
#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

namespace dai {

// STATIC
constexpr std::chrono::milliseconds XLinkConnection::WAIT_FOR_BOOTUP_TIMEOUT;
constexpr std::chrono::milliseconds XLinkConnection::WAIT_FOR_CONNECT_TIMEOUT;
constexpr std::chrono::milliseconds XLinkConnection::WAIT_FOR_STREAM_RETRY;
constexpr int XLinkConnection::STREAM_OPEN_RETRIES;

void XLinkConnection::initXLinkGlobal() {
    if(xlinkGlobalInitialized) return;

    xlinkGlobalHandler.protocol = X_LINK_USB_VSC;
    auto status = XLinkInitialize(&xlinkGlobalHandler);
    if(X_LINK_SUCCESS != status) {
        throw std::runtime_error("Couldn't initialize XLink");
    }

    // Suppress XLink related errors
    // TODO(themarpe) - XLinkLog.h for Windows caueses issues
    // FUNCATTR_WEAK static, causes log level symbols to not be accessible
    // Closest alternative for weak linking in MSVC is: __declspec(selectany)

    // mvLogDefaultLevelSet(MVLOG_FATAL);
    // extern int mvLogLevel_default;
    // mvLogLevel_default = 4; // MVLOG_FATAL

    xlinkGlobalInitialized = true;
}

std::atomic<bool> XLinkConnection::xlinkGlobalInitialized{false};
XLinkGlobalHandler_t XLinkConnection::xlinkGlobalHandler = {};
std::mutex XLinkConnection::xlinkStreamOperationMutex;

std::vector<DeviceInfo> XLinkConnection::getAllConnectedDevices() {
    initXLinkGlobal();

    std::vector<DeviceInfo> devices;

    // Get all available devices (unbooted & booted)
    for(const auto& state : {X_LINK_UNBOOTED, X_LINK_BOOTED}) {
        unsigned int numdev = 0;
        std::array<deviceDesc_t, 32> deviceDescAll = {};
        deviceDesc_t suitableDevice = {};
        suitableDevice.protocol = X_LINK_ANY_PROTOCOL;
        suitableDevice.platform = X_LINK_ANY_PLATFORM;

        auto status = XLinkFindAllSuitableDevices(state, suitableDevice, deviceDescAll.data(), deviceDescAll.size(), &numdev);
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
        if(d.state == state) return {true, d};
    }
    return {false, DeviceInfo()};
}

XLinkConnection::XLinkConnection(const DeviceInfo& deviceDesc, std::vector<std::uint8_t> mvcmdBinary) {
    bootDevice = true;
    bootWithPath = false;
    this->mvcmd = std::move(mvcmdBinary);
    initDevice(deviceDesc);
}

XLinkConnection::XLinkConnection(const DeviceInfo& deviceDesc, std::string pathToMvcmd) {
    bootDevice = true;
    bootWithPath = true;
    this->pathToMvcmd = std::move(pathToMvcmd);
    initDevice(deviceDesc);
}

// Skip boot
XLinkConnection::XLinkConnection(const DeviceInfo& deviceDesc) {
    bootDevice = false;
    initDevice(deviceDesc);
}

XLinkConnection::~XLinkConnection() {
    if(deviceLinkId != -1) {
        if(rebootOnDestruction) XLinkResetRemote(deviceLinkId);
    }
}

void XLinkConnection::setRebootOnDestruction(bool reboot) {
    rebootOnDestruction = reboot;
}

bool XLinkConnection::getRebootOnDestruction() const {
    return rebootOnDestruction;
}

bool XLinkConnection::bootAvailableDevice(const deviceDesc_t& deviceToBoot, const std::string& pathToMvcmd) {
    deviceDesc_t toBoot(deviceToBoot);
    auto status = XLinkBoot(&toBoot, pathToMvcmd.c_str());
    return status == X_LINK_SUCCESS;
}

bool XLinkConnection::bootAvailableDevice(const deviceDesc_t& deviceToBoot, std::vector<std::uint8_t>& mvcmd) {
    deviceDesc_t toBoot(deviceToBoot);
    auto status = XLinkBootMemory(&toBoot, mvcmd.data(), mvcmd.size());
    return status == X_LINK_SUCCESS;
}

void XLinkConnection::initDevice(const DeviceInfo& deviceToInit) {
    initXLinkGlobal();
    assert(deviceLinkId == -1);

    XLinkError_t rc = X_LINK_ERROR;
    deviceDesc_t deviceDesc = {};

    using namespace std::chrono;

    // if device in booted state, then don't boot, just connect
    bootDevice = deviceToInit.state != X_LINK_BOOTED;

    // boot device
    if(bootDevice) {
        if(bootWithPath) {
            bootAvailableDevice(deviceToInit.desc, pathToMvcmd);
        } else {
            bootAvailableDevice(deviceToInit.desc, mvcmd);
        }
    } else {
        // TODO(themarpe) - Add logging library
        std::cout << "Device boot is skipped" << std::endl;
    }

    // Search for booted device
    {
        // Create description of device to look for
        deviceDesc_t bootedDeviceDesc = {};
        for(unsigned int i = 0; i < sizeof(deviceToInit.desc.name); i++) {
            bootedDeviceDesc.name[i] = deviceToInit.desc.name[i];
            if(deviceToInit.desc.name[i] == '-') break;
        }

        // Find booted device
        auto tstart = steady_clock::now();
        do {
            rc = XLinkFindFirstSuitableDevice(X_LINK_BOOTED, bootedDeviceDesc, &deviceDesc);
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
    }
}

void XLinkConnection::openStream(const std::string& streamName, std::size_t maxWriteSize) {
    if(streamName.empty()) throw std::invalid_argument("streamName is empty");

    std::unique_lock<std::mutex> lock(xlinkStreamOperationMutex);
    streamId_t streamId = INVALID_STREAM_ID;

    assert(deviceLinkId != -1);

    for(int retryCount = 0; retryCount < STREAM_OPEN_RETRIES; retryCount++) {
        streamId = XLinkOpenStream(deviceLinkId, streamName.c_str(), maxWriteSize);

        if(streamId != INVALID_STREAM_ID) {
            break;
        }

        // Give some time before retrying
        std::this_thread::sleep_for(WAIT_FOR_STREAM_RETRY);
    }

    if(streamId == INVALID_STREAM_ID) throw std::runtime_error("Couldn't open stream");

    streamIdMap[streamName] = streamId;
}

void XLinkConnection::closeStream(const std::string& streamName) {
    if(streamName.empty()) throw std::invalid_argument("streamName is empty");

    std::unique_lock<std::mutex> lock(xlinkStreamOperationMutex);
    if(streamIdMap.count(streamName) == 0) return;
    XLinkCloseStream(streamIdMap[streamName]);

    // remove from map
    streamIdMap.erase(streamName);
}

////////////////////
// BLOCKING VERSIONS
////////////////////

void XLinkConnection::writeToStream(const std::string& streamName, const std::uint8_t* data, std::size_t size) {
    if(streamName.empty()) throw std::invalid_argument("streamName is empty");
    if(streamIdMap.count(streamName) == 0) throw std::logic_error("Stream: " + streamName + " isn't opened.");

    auto status = XLinkWriteData(streamIdMap[streamName], data, size);
    if(status != X_LINK_SUCCESS) throw std::runtime_error("XLink write error, error message: " + convertErrorCodeToString(status));
}
void XLinkConnection::writeToStream(const std::string& streamName, const void* data, std::size_t size) {
    writeToStream(streamName, reinterpret_cast<const uint8_t*>(data), size);
}

void XLinkConnection::writeToStream(const std::string& streamName, const std::vector<std::uint8_t>& data) {
    writeToStream(streamName, data.data(), data.size());
}

void XLinkConnection::readFromStream(const std::string& streamName, std::vector<std::uint8_t>& data) {
    if(streamName.empty()) throw std::invalid_argument("streamName is empty");
    if(streamIdMap.count(streamName) == 0) throw std::logic_error("Stream: " + streamName + " isn't opened.");

    streamPacketDesc_t* pPacket = nullptr;
    auto status = XLinkReadData(streamIdMap[streamName], &pPacket);
    if(status != X_LINK_SUCCESS) throw std::runtime_error("Couldn't read data from stream: " + streamName);
    data = std::vector<std::uint8_t>(pPacket->data, pPacket->data + pPacket->length);
    XLinkReleaseData(streamIdMap[streamName]);
}

std::vector<std::uint8_t> XLinkConnection::readFromStream(const std::string& streamName) {
    std::vector<std::uint8_t> data;
    readFromStream(streamName, data);
    return data;
}

// USE ONLY WHEN COPYING DATA AT LATER STAGES
streamPacketDesc_t* XLinkConnection::readFromStreamRaw(const std::string& streamName) {
    if(streamName.empty()) throw std::invalid_argument("streamName is empty");
    if(streamIdMap.count(streamName) == 0) throw std::logic_error("Stream: " + streamName + " isn't opened.");
    streamPacketDesc_t* pPacket = nullptr;
    auto status = XLinkReadData(streamIdMap[streamName], &pPacket);
    if(status != X_LINK_SUCCESS) {
        throw std::runtime_error("Error while reading data from xlink channel: " + streamName + " (" + XLinkErrorToStr(status) + ")");
    }
    return pPacket;
}

// USE ONLY WHEN COPYING DATA AT LATER STAGES
void XLinkConnection::readFromStreamRawRelease(const std::string& streamName) {
    if(streamName.empty()) throw std::invalid_argument("streamName is empty");
    if(streamIdMap.count(streamName) == 0) throw std::logic_error("Stream: " + streamName + " isn't opened.");
    XLinkReleaseData(streamIdMap[streamName]);
}

///////////////////////
// Timeout versions //
//////////////////////

// bool XLinkConnection::writeToStream(const std::string& streamName, const std::uint8_t* data, std::size_t size, std::chrono::milliseconds timeout) {
//     if(streamName.empty()) throw std::invalid_argument("streamName is empty");
//     if(streamIdMap.count(streamName) == 0) throw std::logic_error("Stream: " + streamName + " isn't opened.");
//     auto status = XLinkWriteDataWithTimeout(streamIdMap[streamName], data, size, timeout.count());
//     if(status == X_LINK_SUCCESS) return true;
//     else if(status == X_LINK_TIMEOUT) return false;
//     else throw std::runtime_error("XLink write error, error message: " + convertErrorCodeToString(status));
// }
//
// bool XLinkConnection::writeToStream(const std::string& streamName, const void* data, std::size_t size, std::chrono::milliseconds timeout) {
//     return writeToStream(streamName, reinterpret_cast<const std::uint8_t*>(data), size, timeout);
// }
//
// bool XLinkConnection::writeToStream(const std::string& streamName, const std::vector<std::uint8_t>& data, std::chrono::milliseconds timeout) {
//     return writeToStream(streamName, data.data(), data.size(), timeout);
// }
//
// bool XLinkConnection::readFromStream(const std::string& streamName, std::vector<std::uint8_t>& data, std::chrono::milliseconds timeout) {
//     if(streamName.empty()) throw std::invalid_argument("streamName is empty");
//     if(streamIdMap.count(streamName) == 0) throw std::logic_error("Stream: " + streamName + " isn't opened.");
//
//     streamPacketDesc_t* pPacket = nullptr;
//     auto status = XLinkReadDataWithTimeout(streamIdMap[streamName], &pPacket, timeout.count());
//
//     if(status == X_LINK_SUCCESS){
//         data = std::vector<std::uint8_t>(pPacket->data, pPacket->data + pPacket->length);
//         XLinkReleaseData(streamIdMap[streamName]);
//         return true;
//     } else if(status == X_LINK_TIMEOUT){
//         return false;
//     } else {
//         throw std::runtime_error("XLink write error, error message: " + convertErrorCodeToString(status));
//     }
//
//     return false;
// }
//
// bool XLinkConnection::readFromStreamRaw(streamPacketDesc_t*& pPacket, const std::string& streamName, std::chrono::milliseconds timeout) {
//     if(streamName.empty()) throw std::invalid_argument("streamName is empty");
//     if(streamIdMap.count(streamName) == 0) throw std::logic_error("Stream: " + streamName + " isn't opened.");
//     auto status = XLinkReadDataWithTimeout(streamIdMap[streamName], &pPacket, timeout.count());
//     if(status == X_LINK_SUCCESS) return true;
//     else if(status == X_LINK_TIMEOUT) return false;
//     else throw std::runtime_error("XLink write error, error message: " + convertErrorCodeToString(status));
// }

streamId_t XLinkConnection::getStreamId(const std::string& name) const {
    return streamIdMap.at(name);
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
        default:
            return "<UNKNOWN ERROR>";
    }
}

}  // namespace dai
