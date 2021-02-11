#pragma once

// Std
#include <atomic>
#include <chrono>
#include <cstdint>
#include <list>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <vector>

// Libraries
#include <XLink/XLinkPublicDefines.h>

// Shared
//#include "depthai-shared/general/data_observer.hpp"
//#include "depthai-shared/general/data_subject.hpp"
//#include "depthai-shared/stream/stream_data.hpp"
//#include "depthai-shared/stream/stream_info.hpp"

namespace dai {

struct DeviceInfo {
    DeviceInfo() = default;
    DeviceInfo(const char*);
    DeviceInfo(std::string);
    deviceDesc_t desc = {};
    XLinkDeviceState_t state = X_LINK_ANY_STATE;
    std::string getMxId() const;
};

class XLinkConnection {
    static std::atomic<bool> xlinkGlobalInitialized;
    static XLinkGlobalHandler_t xlinkGlobalHandler;
    static void initXLinkGlobal();
    static std::mutex xlinkStreamOperationMutex;

   public:
    // static API
    static std::vector<DeviceInfo> getAllConnectedDevices(XLinkDeviceState_t state = X_LINK_ANY_STATE);
    static std::tuple<bool, DeviceInfo> getFirstDevice(XLinkDeviceState_t state = X_LINK_ANY_STATE);
    static std::tuple<bool, DeviceInfo> getDeviceByMxId(std::string, XLinkDeviceState_t state = X_LINK_ANY_STATE);

    XLinkConnection(const DeviceInfo& deviceDesc, std::vector<std::uint8_t> mvcmdBinary, XLinkDeviceState_t expectedState = X_LINK_BOOTED);
    XLinkConnection(const DeviceInfo& deviceDesc, std::string pathToMvcmd, XLinkDeviceState_t expectedState = X_LINK_BOOTED);
    explicit XLinkConnection(const DeviceInfo& deviceDesc, XLinkDeviceState_t expectedState = X_LINK_BOOTED);

    ~XLinkConnection();

    void setRebootOnDestruction(bool reboot);
    bool getRebootOnDestruction() const;

    void openStream(const std::string& streamName, std::size_t maxWriteSize);
    void closeStream(const std::string& streamName);
    void writeToStream(const std::string& streamName, const void* data, std::size_t size);
    void writeToStream(const std::string& streamName, const std::uint8_t* data, std::size_t size);
    void writeToStream(const std::string& streamName, const std::vector<std::uint8_t>& data);
    std::vector<std::uint8_t> readFromStream(const std::string& streamName);
    void readFromStream(const std::string& streamName, std::vector<std::uint8_t>& data);

    // split write helper
    void writeToStreamSplit(const std::string& streamName, const void* data, std::size_t size, std::size_t split);
    void writeToStreamSplit(const std::string& streamName, const std::vector<uint8_t>& data, std::size_t split);

    // timeout versions
    bool writeToStream(const std::string& streamName, const void* data, std::size_t size, std::chrono::milliseconds timeout);
    bool writeToStream(const std::string& streamName, const std::uint8_t* data, std::size_t size, std::chrono::milliseconds timeout);
    bool writeToStream(const std::string& streamName, const std::vector<std::uint8_t>& data, std::chrono::milliseconds timeout);
    bool readFromStream(const std::string& streamName, std::vector<std::uint8_t>& data, std::chrono::milliseconds timeout);
    bool readFromStreamRaw(streamPacketDesc_t*& pPacket, const std::string& streamName, std::chrono::milliseconds timeout);

    // USE ONLY WHEN COPYING DATA AT LATER STAGES
    streamPacketDesc_t* readFromStreamRaw(const std::string& streamName);

    // USE ONLY WHEN COPYING DATA AT LATER STAGES
    void readFromStreamRawRelease(const std::string& streamName);

    streamId_t getStreamId(const std::string& name) const;
    int getLinkId() const;

   private:
    // static
    static bool bootAvailableDevice(const deviceDesc_t& deviceToBoot, const std::string& pathToMvcmd);
    static bool bootAvailableDevice(const deviceDesc_t& deviceToBoot, std::vector<std::uint8_t>& mvcmd);
    static std::string convertErrorCodeToString(XLinkError_t errorCode);

    void initDevice(const DeviceInfo& deviceToInit, XLinkDeviceState_t expectedState = X_LINK_BOOTED);

    std::unordered_map<std::string, streamId_t> streamIdMap;

    bool bootDevice = true;
    bool bootWithPath = true;
    std::string pathToMvcmd;
    std::vector<std::uint8_t> mvcmd;

    bool rebootOnDestruction{true};

    int deviceLinkId = -1;

    constexpr static std::chrono::milliseconds WAIT_FOR_BOOTUP_TIMEOUT{5000};
    constexpr static std::chrono::milliseconds WAIT_FOR_CONNECT_TIMEOUT{5000};
    constexpr static int STREAM_OPEN_RETRIES = 5;
    constexpr static std::chrono::milliseconds WAIT_FOR_STREAM_RETRY{50};
};

}  // namespace dai
