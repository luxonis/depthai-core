#pragma once

// std
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <vector>

// project
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/UsbSpeed.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/openvino/OpenVINO.hpp"
#include "depthai/utility/Pimpl.hpp"
#include "depthai/xlink/XLinkConnection.hpp"
#include "depthai/xlink/XLinkStream.hpp"

// shared
#include "depthai-shared/common/ChipTemperature.hpp"
#include "depthai-shared/common/CpuUsage.hpp"
#include "depthai-shared/common/MemoryInfo.hpp"
#include "depthai-shared/device/PrebootConfig.hpp"
#include "depthai-shared/log/LogLevel.hpp"
#include "depthai-shared/log/LogMessage.hpp"

namespace dai {

// Forward declare Pipeline
class Pipeline;

/**
 * The core of depthai device for RAII, connects to device and maintains watchdog, timesync, ...
 */
class DeviceBase {
   public:
    // constants

    /// Default search time for constructors which discover devices
    static constexpr std::chrono::seconds DEFAULT_SEARCH_TIME{3};
    /// Default rate at which system information is logged
    static constexpr float DEFAULT_SYSTEM_INFORMATION_LOGGING_RATE_HZ{1.0f};
    /// Default UsbSpeed for device connection
    static constexpr UsbSpeed DEFAULT_USB_SPEED{UsbSpeed::SUPER};

    // Structures

    /**
     * Device specific configuration
     */
    struct Config {
        OpenVINO::Version version;
        PrebootConfig preboot;
    };

    // static API

    /**
     * Waits for any available device with a timeout
     *
     * @param timeout duration of time to wait for the any device
     * @returns Tuple of bool and DeviceInfo. Bool specifies if device was found. DeviceInfo specifies the found device
     */
    template <typename Rep, typename Period>
    static std::tuple<bool, DeviceInfo> getAnyAvailableDevice(std::chrono::duration<Rep, Period> timeout);

    /**
     * Gets any available device
     *
     * @returns Tuple of bool and DeviceInfo. Bool specifies if device was found. DeviceInfo specifies the found device
     */
    static std::tuple<bool, DeviceInfo> getAnyAvailableDevice();

    /**
     * Gets first available device. Device can be either in XLINK_UNBOOTED or XLINK_BOOTLOADER state
     * @returns Tuple of bool and DeviceInfo. Bool specifies if device was found. DeviceInfo specifies the found device
     */
    static std::tuple<bool, DeviceInfo> getFirstAvailableDevice();

    /**
     * Finds a device by MX ID. Example: 14442C10D13EABCE00
     * @param mxId MyraidX ID which uniquely specifies a device
     * @returns Tuple of bool and DeviceInfo. Bool specifies if device was found. DeviceInfo specifies the found device
     */
    static std::tuple<bool, DeviceInfo> getDeviceByMxId(std::string mxId);

    /**
     * Returns all connected devices
     * @returns Vector of connected devices
     */
    static std::vector<DeviceInfo> getAllAvailableDevices();

    /**
     * Gets device firmware binary for a specific OpenVINO version
     * @param usb2Mode USB2 mode firmware
     * @param version Version of OpenVINO which firmware will support
     * @returns Firmware binary
     */
    static std::vector<std::uint8_t> getEmbeddedDeviceBinary(bool usb2Mode, OpenVINO::Version version = OpenVINO::DEFAULT_VERSION);

    /**
     * Gets device firmware binary for a specific configuration
     * @param config FW with applied configuration
     * @returns Firmware binary
     */
    static std::vector<std::uint8_t> getEmbeddedDeviceBinary(Config config);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param pipeline Pipeline to be executed on the device
     */
    explicit DeviceBase(const Pipeline& pipeline);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param pipeline Pipeline to be executed on the device
     * @param usb2Mode Boot device using USB2 mode firmware
     */
    DeviceBase(const Pipeline& pipeline, bool usb2Mode);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param pipeline Pipeline to be executed on the device
     * @param maxUsbSpeed Maximum allowed USB speed
     */
    DeviceBase(const Pipeline& pipeline, UsbSpeed maxUsbSpeed);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param pipeline Pipeline to be executed on the device
     * @param pathToCmd Path to custom device firmware
     */
    DeviceBase(const Pipeline& pipeline, const char* pathToCmd);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param pipeline Pipeline to be executed on the device
     * @param pathToCmd Path to custom device firmware
     */
    DeviceBase(const Pipeline& pipeline, const std::string& pathToCmd);

    /**
     * Connects to device specified by devInfo.
     * @param pipeline Pipeline to be executed on the device
     * @param devInfo DeviceInfo which specifies which device to connect to
     */
    DeviceBase(const Pipeline& pipeline, const DeviceInfo& devInfo);

    /**
     * Connects to device specified by devInfo.
     * @param pipeline Pipeline to be executed on the device
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param usb2Mode Boot device using USB2 mode firmware
     */
    DeviceBase(const Pipeline& pipeline, const DeviceInfo& devInfo, bool usb2Mode);

    /**
     * Connects to device specified by devInfo.
     * @param pipeline Pipeline to be executed on the device
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param maxUsbSpeed Maximum allowed USB speed
     */
    DeviceBase(const Pipeline& pipeline, const DeviceInfo& devInfo, UsbSpeed maxUsbSpeed);

    /**
     * Connects to device specified by devInfo.
     * @param pipeline Pipeline to be executed on the device
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param pathToCmd Path to custom device firmware
     */
    DeviceBase(const Pipeline& pipeline, const DeviceInfo& devInfo, const char* pathToCmd);

    /**
     * Connects to device specified by devInfo.
     * @param pipeline Pipeline to be executed on the device
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param pathToCmd Path to custom device firmware
     */
    DeviceBase(const Pipeline& pipeline, const DeviceInfo& devInfo, const std::string& pathToCmd);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * Uses OpenVINO version Pipeline::DEFAULT_OPENVINO_VERSION
     */
    DeviceBase();

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param version OpenVINO version which the device will be booted with.
     */
    explicit DeviceBase(OpenVINO::Version version);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param version OpenVINO version which the device will be booted with
     * @param usb2Mode Boot device using USB2 mode firmware
     */
    DeviceBase(OpenVINO::Version version, bool usb2Mode);

    /**
     * Connects to device specified by devInfo.
     * @param version OpenVINO version which the device will be booted with
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param maxUsbSpeed Maximum allowed USB speed
     */
    DeviceBase(OpenVINO::Version version, UsbSpeed maxUsbSpeed);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param version OpenVINO version which the device will be booted with
     * @param pathToCmd Path to custom device firmware
     */
    DeviceBase(OpenVINO::Version version, const char* pathToCmd);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param version OpenVINO version which the device will be booted with
     * @param pathToCmd Path to custom device firmware
     */
    DeviceBase(OpenVINO::Version version, const std::string& pathToCmd);

    /**
     * Connects to device specified by devInfo.
     * @param version OpenVINO version which the device will be booted with
     * @param devInfo DeviceInfo which specifies which device to connect to
     */
    DeviceBase(OpenVINO::Version version, const DeviceInfo& devInfo);

    /**
     * Connects to device specified by devInfo.
     * @param version OpenVINO version which the device will be booted with
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param usb2Mode Boot device using USB2 mode firmware
     */
    DeviceBase(OpenVINO::Version version, const DeviceInfo& devInfo, bool usb2Mode);

    /**
     * Connects to device specified by devInfo.
     * @param version OpenVINO version which the device will be booted with
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param maxUsbSpeed Maximum allowed USB speed
     */
    DeviceBase(OpenVINO::Version version, const DeviceInfo& devInfo, UsbSpeed maxUsbSpeed);

    /**
     * Connects to device specified by devInfo.
     * @param version OpenVINO version which the device will be booted with
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param pathToCmd Path to custom device firmware
     */
    DeviceBase(OpenVINO::Version version, const DeviceInfo& devInfo, const char* pathToCmd);

    /**
     * Connects to device specified by devInfo.
     * @param version OpenVINO version which the device will be booted with
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param usb2Mode Path to custom device firmware
     */
    DeviceBase(OpenVINO::Version version, const DeviceInfo& devInfo, const std::string& pathToCmd);

    /**
     * Connects to any available device with custom config.
     * @param config Device custom configuration to boot with
     */
    explicit DeviceBase(Config config);

    /**
     * Connects to device 'devInfo' with custom config.
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param config Device custom configuration to boot with
     */
    DeviceBase(Config config, const DeviceInfo& devInfo);

    /**
     * Device destructor
     * @note In the destructor of the derived class, remember to call close()
     */
    virtual ~DeviceBase();

    /**
     * Checks if devices pipeline is already running
     *
     * @returns True if running, false otherwise
     */
    bool isPipelineRunning();

    /**
     * Starts the execution of the devices pipeline
     *
     * @returns True if pipeline started, false otherwise
     */
    [[deprecated("Device(pipeline) starts the pipeline automatically. See Device() and startPipeline(pipeline) otherwise")]] bool startPipeline();

    /**
     * Starts the execution of a given pipeline
     * @param pipeline OpenVINO version of the pipeline must match the one which the device was booted with.
     *
     * @returns True if pipeline started, false otherwise
     */
    bool startPipeline(const Pipeline& pipeline);

    /**
     * Sets the devices logging severity level. This level affects which logs are transfered from device to host.
     *
     * @param level Logging severity
     */
    void setLogLevel(LogLevel level);

    /**
     * Gets current logging severity level of the device.
     *
     * @returns Logging severity level
     */
    LogLevel getLogLevel();

    /**
     * Sets the chunk size for splitting device-sent XLink packets. A larger value could
     * increase performance, and 0 disables chunking. A negative value is ignored.
     * Device defaults are configured per protocol, currently 64*1024 for both USB and Ethernet.
     *
     * @param sizeBytes XLink chunk size in bytes
     */
    void setXLinkChunkSize(int sizeBytes);

    /**
     * Gets current XLink chunk size.
     *
     * @returns XLink chunk size in bytes
     */
    int getXLinkChunkSize();

    /**
     * Get the Device Info object o the device which is currently running
     *
     * @return DeviceInfo of the current device in execution
     */
    DeviceInfo getDeviceInfo() const;

    /**
     * Get MxId of device
     *
     * @returns MxId of connected device
     */
    std::string getMxId();

    /**
     * Sets logging level which decides printing level to standard output.
     * If lower than setLogLevel, no messages will be printed
     *
     * @param level Standard output printing severity
     */
    void setLogOutputLevel(LogLevel level);

    /**
     * Gets logging level which decides printing level to standard output.
     *
     * @returns Standard output printing severity
     */
    LogLevel getLogOutputLevel();

    /**
     * Add a callback for device logging. The callback will be called from a separate thread with the LogMessage being passed.
     *
     * @param callback Callback to call whenever a log message arrives
     * @returns Id which can be used to later remove the callback
     */
    int addLogCallback(std::function<void(LogMessage)> callback);

    /**
     * Removes a callback
     *
     * @param callbackId Id of callback to be removed
     * @returns True if callback was removed, false otherwise
     */
    bool removeLogCallback(int callbackId);

    /**
     * Sets rate of system information logging ("info" severity). Default 1Hz
     * If parameter is less or equal to zero, then system information logging will be disabled
     *
     * @param rateHz Logging rate in Hz
     */
    void setSystemInformationLoggingRate(float rateHz);

    /**
     * Gets current rate of system information logging ("info" severity) in Hz.
     *
     * @returns Logging rate in Hz
     */
    float getSystemInformationLoggingRate();

    /**
     * Get cameras that are connected to the device
     *
     * @returns Vector of connected cameras
     */
    std::vector<CameraBoardSocket> getConnectedCameras();

    /**
     * Get sensor names for cameras that are connected to the device
     *
     * @returns Map/dictionary with camera sensor names, indexed by socket
     */
    std::unordered_map<CameraBoardSocket, std::string> getCameraSensorNames();

    /**
     * Retrieves current DDR memory information from device
     *
     * @returns Used, remaining and total ddr memory
     */
    MemoryInfo getDdrMemoryUsage();

    /**
     * Retrieves current CMX memory information from device
     *
     * @returns Used, remaining and total cmx memory
     */
    MemoryInfo getCmxMemoryUsage();

    /**
     * Retrieves current CSS Leon CPU heap information from device
     *
     * @returns Used, remaining and total heap memory
     */
    MemoryInfo getLeonCssHeapUsage();

    /**
     * Retrieves current MSS Leon CPU heap information from device
     *
     * @returns Used, remaining and total heap memory
     */
    MemoryInfo getLeonMssHeapUsage();

    /**
     * Retrieves current chip temperature as measured by device
     *
     * @returns Temperature of various onboard sensors
     */
    ChipTemperature getChipTemperature();

    /**
     * Retrieves average CSS Leon CPU usage
     *
     * @returns Average CPU usage and sampling duration
     */
    CpuUsage getLeonCssCpuUsage();

    /**
     * Retrieves average MSS Leon CPU usage
     *
     * @returns Average CPU usage and sampling duration
     */
    CpuUsage getLeonMssCpuUsage();

    /**
     * Stores the Calibration and Device information to the Device EEPROM
     *
     * @param calibrationObj CalibrationHandler object which is loaded with calibration information.
     *
     * @return true on successful flash, false on failure
     */
    bool flashCalibration(CalibrationHandler calibrationDataHandler);

    /**
     * Fetches the EEPROM data from the device and loads it into CalibrationHandler object
     *
     * @return The CalibrationHandler object containing the calibration currently flashed on device EEPROM
     */
    CalibrationHandler readCalibration();

    /**
     * Retrieves USB connection speed
     *
     * @returns USB connection speed of connected device if applicable. Unknown otherwise.
     */
    UsbSpeed getUsbSpeed();

    /**
     * Explicitly closes connection to device.
     * @note This function does not need to be explicitly called
     * as destructor closes the device automatically
     */
    void close();

    /**
     * Is the device already closed (or disconnected)
     */
    bool isClosed() const;

    /**
     * Returns underlying XLinkConnection
     */
    std::shared_ptr<XLinkConnection> getConnection() {
        return connection;
    }

    /**
     * Returns underlying XLinkConnection
     */
    std::shared_ptr<const XLinkConnection> getConnection() const {
        return connection;
    }

   protected:
    std::shared_ptr<XLinkConnection> connection;

    /**
     * @brief a safe way to start a pipeline, which is closed if any exception occurs
     */
    void tryStartPipeline(const Pipeline& pipeline);

    /**
     * throws an error if the device has been closed or the watchdog has died
     */
    void checkClosed() const;

    /**
     * Allows the derived classes to handle custom setup for starting the pipeline
     *
     * @param pipeline OpenVINO version of the pipeline must match the one which the device was booted with
     * @sa startPipeline
     * @note Remember to call this function in the overload to setup the comunication properly
     *
     * @returns True if pipeline started, false otherwise
     */
    virtual bool startPipelineImpl(const Pipeline& pipeline);

    /**
     * Allows the derived classes to handle custom setup for gracefully stopping the pipeline
     *
     * @note Remember to call this function in the overload to setup the comunication properly
     */
    virtual void closeImpl();

   private:
    // private functions
    void init(OpenVINO::Version version, bool usb2Mode, const std::string& pathToMvcmd);
    void init(const Pipeline& pipeline, bool usb2Mode, const std::string& pathToMvcmd);
    void init(OpenVINO::Version version, UsbSpeed maxUsbSpeed, const std::string& pathToMvcmd);
    void init(const Pipeline& pipeline, UsbSpeed maxUsbSpeed, const std::string& pathToMvcmd);
    void init2(Config cfg, const std::string& pathToMvcmd, tl::optional<const Pipeline&> pipeline);

    DeviceInfo deviceInfo = {};

    // Log callback
    int uniqueCallbackId = 0;
    std::mutex logCallbackMapMtx;
    std::unordered_map<int, std::function<void(LogMessage)>> logCallbackMap;

    // Watchdog thread
    std::thread watchdogThread;
    std::atomic<bool> watchdogRunning{true};

    // Timesync thread
    std::thread timesyncThread;
    std::atomic<bool> timesyncRunning{true};

    // Logging thread
    std::thread loggingThread;
    std::atomic<bool> loggingRunning{true};

    // RPC stream
    std::unique_ptr<XLinkStream> rpcStream;

    // closed
    std::atomic<bool> closed{false};

    // pimpl
    class Impl;
    Pimpl<Impl> pimpl;

    // Device config
    Config config;
};
}  // namespace dai
