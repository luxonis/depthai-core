#pragma once

// std
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <type_traits>

// project
#include "CallbackHandler.hpp"
#include "DataQueue.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/UsbSpeed.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/utility/Pimpl.hpp"
#include "depthai/xlink/XLinkConnection.hpp"
#include "depthai/xlink/XLinkStream.hpp"

// shared
#include "depthai-shared/common/ChipTemperature.hpp"
#include "depthai-shared/common/CpuUsage.hpp"
#include "depthai-shared/common/MemoryInfo.hpp"
#include "depthai-shared/log/LogLevel.hpp"
#include "depthai-shared/log/LogMessage.hpp"

namespace dai {

// Device (RAII), connects to device and maintains watchdog, timesync, ...

/**
 * Represents the DepthAI device with the methods to interact with it.
 */
class Device {
   public:
    // constants

    /// Default search time for constructors which discover devices
    static constexpr std::chrono::seconds DEFAULT_SEARCH_TIME{3};
    /// Maximum number of elements in event queue
    static constexpr std::size_t EVENT_QUEUE_MAXIMUM_SIZE{2048};
    /// Default rate at which system information is logged
    static constexpr float DEFAULT_SYSTEM_INFORMATION_LOGGING_RATE_HZ{1.0f};

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
    static std::vector<std::uint8_t> getEmbeddedDeviceBinary(bool usb2Mode, OpenVINO::Version version = Pipeline::DEFAULT_OPENVINO_VERSION);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param pipeline Pipeline to be executed on the device
     */
    explicit Device(const Pipeline& pipeline);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param pipeline Pipeline to be executed on the device
     * @param usb2Mode Boot device using USB2 mode firmware
     */
    Device(const Pipeline& pipeline, bool usb2Mode);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param pipeline Pipeline to be executed on the device
     * @param pathToCmd Path to custom device firmware
     */
    Device(const Pipeline& pipeline, const char* pathToCmd);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param pipeline Pipeline to be executed on the device
     * @param pathToCmd Path to custom device firmware
     */
    Device(const Pipeline& pipeline, const std::string& pathToCmd);

    /**
     * Connects to device specified by devInfo.
     * @param pipeline Pipeline to be executed on the device
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param usb2Mode Boot device using USB2 mode firmware
     */
    Device(const Pipeline& pipeline, const DeviceInfo& devInfo, bool usb2Mode = false);

    /**
     * Connects to device specified by devInfo.
     * @param pipeline Pipeline to be executed on the device
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param pathToCmd Path to custom device firmware
     */
    Device(const Pipeline& pipeline, const DeviceInfo& devInfo, const char* pathToCmd);

    /**
     * Connects to device specified by devInfo.
     * @param pipeline Pipeline to be executed on the device
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param usb2Mode Path to custom device firmware
     */
    Device(const Pipeline& pipeline, const DeviceInfo& devInfo, const std::string& pathToCmd);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param version OpenVINO version which the device will be booted with. Default is Pipeline::DEFAULT_OPENVINO_VERSION
     */
    explicit Device(OpenVINO::Version version = Pipeline::DEFAULT_OPENVINO_VERSION);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param version OpenVINO version which the device will be booted with
     * @param usb2Mode Boot device using USB2 mode firmware
     */
    Device(OpenVINO::Version version, bool usb2Mode);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param version OpenVINO version which the device will be booted with
     * @param pathToCmd Path to custom device firmware
     */
    Device(OpenVINO::Version version, const char* pathToCmd);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param version OpenVINO version which the device will be booted with
     * @param pathToCmd Path to custom device firmware
     */
    Device(OpenVINO::Version version, const std::string& pathToCmd);

    /**
     * Connects to device specified by devInfo.
     * @param version OpenVINO version which the device will be booted with
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param usb2Mode Boot device using USB2 mode firmware
     */
    Device(OpenVINO::Version version, const DeviceInfo& devInfo, bool usb2Mode = false);

    /**
     * Connects to device specified by devInfo.
     * @param version OpenVINO version which the device will be booted with
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param pathToCmd Path to custom device firmware
     */
    Device(OpenVINO::Version version, const DeviceInfo& devInfo, const char* pathToCmd);

    /**
     * Connects to device specified by devInfo.
     * @param version OpenVINO version which the device will be booted with
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param usb2Mode Path to custom device firmware
     */
    Device(OpenVINO::Version version, const DeviceInfo& devInfo, const std::string& pathToCmd);

    /**
     * Device destructor. Closes the connection and data queues.
     */
    ~Device();

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
     * Get the Device Info object o the device which is currently running
     *
     * @return DeviceInfo of the current device in execution
     */
    DeviceInfo getDeviceInfo();

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
     * Gets an output queue corresponding to stream name. If it doesn't exist it throws
     *
     * @param name Queue/stream name, created by XLinkOut node
     * @returns Smart pointer to DataOutputQueue
     */
    std::shared_ptr<DataOutputQueue> getOutputQueue(const std::string& name);

    /**
     * Gets a queue corresponding to stream name, if it exists, otherwise it throws. Also sets queue options
     *
     * @param name Queue/stream name, set in XLinkOut node
     * @param maxSize Maximum number of messages in queue
     * @param blocking Queue behavior once full. True specifies blocking and false overwriting of oldest messages. Default: true
     * @returns Smart pointer to DataOutputQueue
     */
    std::shared_ptr<DataOutputQueue> getOutputQueue(const std::string& name, unsigned int maxSize, bool blocking = true);

    /**
     * Get all available output queue names
     *
     * @returns Vector of output queue names
     */
    std::vector<std::string> getOutputQueueNames() const;

    /**
     * Gets an input queue corresponding to stream name. If it doesn't exist it throws
     *
     * @param name Queue/stream name, set in XLinkIn node
     * @returns Smart pointer to DataInputQueue
     */
    std::shared_ptr<DataInputQueue> getInputQueue(const std::string& name);

    /**
     * Gets an input queue corresponding to stream name. If it doesn't exist it throws. Also sets queue options
     *
     * @param name Queue/stream name, set in XLinkOut node
     * @param maxSize Maximum number of messages in queue
     * @param blocking Queue behavior once full. True: blocking, false: overwriting of oldest messages. Default: true
     * @returns Smart pointer to DataInputQueue
     */
    std::shared_ptr<DataInputQueue> getInputQueue(const std::string& name, unsigned int maxSize, bool blocking = true);

    /**
     * Get all available input queue names
     *
     * @returns Vector of input queue names
     */
    std::vector<std::string> getInputQueueNames() const;

    // void setCallback(const std::string& name, std::function<std::shared_ptr<RawBuffer>(std::shared_ptr<RawBuffer>)> cb);

    /**
     * Gets or waits until any of specified queues has received a message
     *
     * @param queueNames Names of queues for which to block
     * @param maxNumEvents Maximum number of events to remove from queue - Default is unlimited
     * @param timeout Timeout after which return regardless. If negative then wait is indefinite - Default is -1
     * @returns Names of queues which received messages first
     */
    std::vector<std::string> getQueueEvents(const std::vector<std::string>& queueNames,
                                            std::size_t maxNumEvents = std::numeric_limits<std::size_t>::max(),
                                            std::chrono::microseconds timeout = std::chrono::microseconds(-1));
    std::vector<std::string> getQueueEvents(const std::initializer_list<std::string>& queueNames,
                                            std::size_t maxNumEvents = std::numeric_limits<std::size_t>::max(),
                                            std::chrono::microseconds timeout = std::chrono::microseconds(-1));

    /**
     * Gets or waits until specified queue has received a message
     *
     * @param queueName Name of queues for which to wait for
     * @param maxNumEvents Maximum number of events to remove from queue. Default is unlimited
     * @param timeout Timeout after which return regardless. If negative then wait is indefinite. Default is -1
     * @returns Names of queues which received messages first
     */
    std::vector<std::string> getQueueEvents(std::string queueName,
                                            std::size_t maxNumEvents = std::numeric_limits<std::size_t>::max(),
                                            std::chrono::microseconds timeout = std::chrono::microseconds(-1));

    /**
     * Gets or waits until any any queue has received a message
     *
     * @param maxNumEvents Maximum number of events to remove from queue. Default is unlimited
     * @param timeout Timeout after which return regardless. If negative then wait is indefinite. Default is -1
     * @returns Names of queues which received messages first
     */
    std::vector<std::string> getQueueEvents(std::size_t maxNumEvents = std::numeric_limits<std::size_t>::max(),
                                            std::chrono::microseconds timeout = std::chrono::microseconds(-1));

    /**
     * Gets or waits until any of specified queues has received a message
     *
     * @param queueNames Names of queues for which to wait for
     * @param timeout Timeout after which return regardless. If negative then wait is indefinite. Default is -1
     * @returns Queue name which received a message first
     */
    std::string getQueueEvent(const std::vector<std::string>& queueNames, std::chrono::microseconds timeout = std::chrono::microseconds(-1));
    std::string getQueueEvent(const std::initializer_list<std::string>& queueNames, std::chrono::microseconds timeout = std::chrono::microseconds(-1));

    /**
     * Gets or waits until specified queue has received a message
     *
     * @param queueNames Name of queues for which to wait for
     * @param timeout Timeout after which return regardless. If negative then wait is indefinite. Default is -1
     * @returns Queue name which received a message
     */
    std::string getQueueEvent(std::string queueName, std::chrono::microseconds timeout = std::chrono::microseconds(-1));

    /**
     * Gets or waits until any queue has received a message
     *
     * @param timeout Timeout after which return regardless. If negative then wait is indefinite. Default is -1
     * @returns Queue name which received a message
     */
    std::string getQueueEvent(std::chrono::microseconds timeout = std::chrono::microseconds(-1));

    /**
     * Get MxId of device
     *
     * @returns MxId of connected device
     */
    std::string getMxId();

    /**
     * Get cameras that are connected to the device
     *
     * @returns Vector of connected cameras
     */
    std::vector<CameraBoardSocket> getConnectedCameras();

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

   private:
    // private static
    void init(OpenVINO::Version version, bool embeddedMvcmd, bool usb2Mode, const std::string& pathToMvcmd);
    void init(const Pipeline& pipeline, bool embeddedMvcmd, bool usb2Mode, const std::string& pathToMvcmd);
    void init2(bool embeddedMvcmd, bool usb2Mode, const std::string& pathToMvcmd, tl::optional<const Pipeline&> pipeline);
    void checkClosed() const;

    std::shared_ptr<XLinkConnection> connection;
    std::vector<uint8_t> patchedCmd;

    DeviceInfo deviceInfo = {};

    std::unordered_map<std::string, std::shared_ptr<DataOutputQueue>> outputQueueMap;
    std::unordered_map<std::string, std::shared_ptr<DataInputQueue>> inputQueueMap;
    std::unordered_map<std::string, DataOutputQueue::CallbackId> callbackIdMap;
    // std::unordered_map<std::string, CallbackHandler> callbackMap;

    // Log callback
    int uniqueCallbackId = 0;
    std::mutex logCallbackMapMtx;
    std::unordered_map<int, std::function<void(LogMessage)>> logCallbackMap;

    // Event queue
    std::mutex eventMtx;
    std::condition_variable eventCv;
    std::deque<std::string> eventQueue;

    // Watchdog thread
    std::thread watchdogThread;
    std::atomic<bool> watchdogRunning{true};

    // Timesync thread
    std::thread timesyncThread;
    std::atomic<bool> timesyncRunning{true};

    // Logging thread
    std::thread loggingThread;
    std::atomic<bool> loggingRunning{true};

    // closed
    std::atomic<bool> closed{false};

    // pimpl
    class Impl;
    Pimpl<Impl> pimpl;

    // OpenVINO version device was booted with
    OpenVINO::Version openvinoVersion;
};

}  // namespace dai
