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

// project
#include "depthai/utility/Path.hpp"

// Libraries
#include <XLink/XLinkPublicDefines.h>

namespace dai {

/**
 * Describes a connected device
 */
struct DeviceInfo {
    DeviceInfo() = default;
    DeviceInfo(std::string name, std::string mxid, XLinkDeviceState_t state, XLinkProtocol_t protocol, XLinkPlatform_t platform, XLinkError_t status);
    /**
     * Creates a DeviceInfo by checking whether supplied parameter is a MXID or IP/USB name
     * @param mxidOrName Either MXID, IP Address or USB port name
     */
    explicit DeviceInfo(std::string mxidOrName);
    explicit DeviceInfo(const deviceDesc_t& desc);
    deviceDesc_t getXLinkDeviceDesc() const;
    std::string getMxId() const;
    std::string toString() const;

    std::string name = "";
    std::string mxid = "";
    XLinkDeviceState_t state = X_LINK_ANY_STATE;
    XLinkProtocol_t protocol = X_LINK_ANY_PROTOCOL;
    XLinkPlatform_t platform = X_LINK_ANY_PLATFORM;
    XLinkError_t status = X_LINK_SUCCESS;
};

/**
 * Represents connection between host and device over XLink protocol
 */
class XLinkConnection {
   public:
    // static API
    static std::vector<DeviceInfo> getAllConnectedDevices(XLinkDeviceState_t state = X_LINK_ANY_STATE, bool skipInvalidDevices = true);
    static std::tuple<bool, DeviceInfo> getFirstDevice(XLinkDeviceState_t state = X_LINK_ANY_STATE, bool skipInvalidDevices = true);
    static std::tuple<bool, DeviceInfo> getDeviceByMxId(std::string, XLinkDeviceState_t state = X_LINK_ANY_STATE, bool skipInvalidDevice = true);
    static DeviceInfo bootBootloader(const DeviceInfo& devInfo);

    XLinkConnection(const DeviceInfo& deviceDesc, std::vector<std::uint8_t> mvcmdBinary, XLinkDeviceState_t expectedState = X_LINK_BOOTED);
    XLinkConnection(const DeviceInfo& deviceDesc, dai::Path pathToMvcmd, XLinkDeviceState_t expectedState = X_LINK_BOOTED);
    explicit XLinkConnection(const DeviceInfo& deviceDesc, XLinkDeviceState_t expectedState = X_LINK_BOOTED);

    ~XLinkConnection();

    void setRebootOnDestruction(bool reboot);
    bool getRebootOnDestruction() const;

    int getLinkId() const;

    /**
     * Explicitly closes xlink connection.
     * @note This function does not need to be explicitly called
     * as destructor closes the connection automatically
     */
    void close();

    /**
     * Is the connection already closed (or disconnected)
     */
    bool isClosed() const;

   private:
    friend struct XLinkReadError;
    friend struct XLinkWriteError;
    // static
    static bool bootAvailableDevice(const deviceDesc_t& deviceToBoot, const dai::Path& pathToMvcmd);
    static bool bootAvailableDevice(const deviceDesc_t& deviceToBoot, std::vector<std::uint8_t>& mvcmd);
    static std::string convertErrorCodeToString(XLinkError_t errorCode);

    void initDevice(const DeviceInfo& deviceToInit, XLinkDeviceState_t expectedState = X_LINK_BOOTED);
    void checkClosed() const;

    bool bootDevice = true;
    bool bootWithPath = true;
    dai::Path pathToMvcmd;
    std::vector<std::uint8_t> mvcmd;

    bool rebootOnDestruction{true};

    int deviceLinkId = -1;

    DeviceInfo deviceInfo;

    // closed
    mutable std::mutex closedMtx;
    bool closed{false};

    constexpr static std::chrono::milliseconds WAIT_FOR_BOOTUP_TIMEOUT{15000};
    constexpr static std::chrono::milliseconds WAIT_FOR_CONNECT_TIMEOUT{5000};
    constexpr static std::chrono::milliseconds POLLING_DELAY_TIME{10};
};

}  // namespace dai
