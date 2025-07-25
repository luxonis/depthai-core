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
#include <type_traits>
#include <unordered_map>
#include <vector>

// project
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraFeatures.hpp"
#include "depthai/common/UsbSpeed.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/device/DeviceGate.hpp"
#include "depthai/device/Version.hpp"
#include "depthai/openvino/OpenVINO.hpp"
#include "depthai/pipeline/PipelineSchema.hpp"
#include "depthai/utility/Pimpl.hpp"
#include "depthai/utility/ProfilingData.hpp"
#include "depthai/xlink/XLinkConnection.hpp"
#include "depthai/xlink/XLinkStream.hpp"

// shared
#include "depthai/common/ChipTemperature.hpp"
#include "depthai/common/ConnectionInterface.hpp"
#include "depthai/common/CpuUsage.hpp"
#include "depthai/common/MemoryInfo.hpp"
#include "depthai/common/StereoPair.hpp"
#include "depthai/device/BoardConfig.hpp"
#include "depthai/device/CrashDump.hpp"
#include "depthai/log/LogLevel.hpp"
#include "depthai/log/LogMessage.hpp"

namespace dai {

// Forward declare Pipeline
class Pipeline;
class PipelineImpl;
/**
 * The core of depthai device for RAII, connects to device and maintains watchdog, timesync, ...
 */
class DeviceBase {
    friend class PipelineImpl;  // Needed for reconnections

   public:
    // constants

    /// Default search time for constructors which discover devices
    static constexpr std::chrono::seconds DEFAULT_SEARCH_TIME{10};
    /// Default rate at which system information is logged
    static constexpr float DEFAULT_SYSTEM_INFORMATION_LOGGING_RATE_HZ{1.0f};
    /// Default UsbSpeed for device connection
    static constexpr UsbSpeed DEFAULT_USB_SPEED{UsbSpeed::SUPER};
    /// Default Timesync period
    static constexpr std::chrono::milliseconds DEFAULT_TIMESYNC_PERIOD{5000};
    /// Default Timesync number of samples per sync
    static constexpr int DEFAULT_TIMESYNC_NUM_SAMPLES{10};
    /// Default Timesync packet interval randomness
    static constexpr bool DEFAULT_TIMESYNC_RANDOM{true};

    // Structures

    /**
     * Device specific configuration
     */
    struct Config {
        OpenVINO::Version version = OpenVINO::VERSION_UNIVERSAL;
        BoardConfig board;
        bool nonExclusiveMode = false;
        std::optional<LogLevel> outputLogLevel;
        std::optional<LogLevel> logLevel;
    };

    // static API

    /**
     * @brief Get the Default Search Time for finding devices
     *
     * @returns Default search time in milliseconds
     */
    static std::chrono::milliseconds getDefaultSearchTime();

    /**
     * Waits for any available device with a timeout
     *
     * @param timeout duration of time to wait for the any device
     * @returns Tuple of bool and DeviceInfo. Bool specifies if device was found. DeviceInfo specifies the found device
     */
    static std::tuple<bool, DeviceInfo> getAnyAvailableDevice(std::chrono::milliseconds timeout);

    /**
     * Gets any available device
     *
     * @returns Tuple of bool and DeviceInfo. Bool specifies if device was found. DeviceInfo specifies the found device
     */
    static std::tuple<bool, DeviceInfo> getAnyAvailableDevice();

    /**
     * Waits for any available device with a timeout
     *
     * @param timeout duration of time to wait for the any device
     * @param cb callback function called between pooling intervals
     * @returns Tuple of bool and DeviceInfo. Bool specifies if device was found. DeviceInfo specifies the found device
     */
    static std::tuple<bool, DeviceInfo> getAnyAvailableDevice(std::chrono::milliseconds timeout, std::function<void()> cb);

    /**
     * Gets first available device. Device can be either in XLINK_UNBOOTED or XLINK_BOOTLOADER state
     * @returns Tuple of bool and DeviceInfo. Bool specifies if device was found. DeviceInfo specifies the found device
     */
    static std::tuple<bool, DeviceInfo> getFirstAvailableDevice(bool skipInvalidDevice = true);

    /**
     * Finds a device by Device ID. Example: 14442C10D13EABCE00
     * @param deviceId Device ID which uniquely specifies a device
     * @returns Tuple of bool and DeviceInfo. Bool specifies if device was found. DeviceInfo specifies the found device
     */
    static std::tuple<bool, DeviceInfo> getDeviceById(std::string deviceId);

    /**
     * Returns all available devices
     * @returns Vector of available devices
     */
    static std::vector<DeviceInfo> getAllAvailableDevices();

    /**
     * Returns information of all connected devices.
     * The devices could be both connectable as well as already connected to devices.
     *
     * @returns Vector of connected device information
     */
    static std::vector<DeviceInfo> getAllConnectedDevices();

    /**
     * Gets device firmware binary for a specific OpenVINO version
     * @param usb2Mode USB2 mode firmware
     * @param version Version of OpenVINO which firmware will support
     * @returns Firmware binary
     */
    static std::vector<std::uint8_t> getEmbeddedDeviceBinary(bool usb2Mode, OpenVINO::Version version = OpenVINO::VERSION_UNIVERSAL);

    /**
     * Gets device firmware binary for a specific configuration
     * @param config FW with applied configuration
     * @returns Firmware binary
     */
    static std::vector<std::uint8_t> getEmbeddedDeviceBinary(Config config);

    /**
     * Get current global accumulated profiling data
     *
     * @returns ProfilingData from all devices
     */
    static ProfilingData getGlobalProfilingData();

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * Uses OpenVINO version OpenVINO::VERSION_UNIVERSAL
     */
    DeviceBase();

    /**
     * Connects to device
     * @param maxUsbSpeed Maximum allowed USB speed
     */
    DeviceBase(UsbSpeed maxUsbSpeed);

    /**
     * Connects to device specified by devInfo.
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param maxUsbSpeed Maximum allowed USB speed
     */
    DeviceBase(const DeviceInfo& devInfo, UsbSpeed maxUsbSpeed);

    /**
     * Connects to device specified by devInfo.
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param pathToCmd Path to custom device firmware
     */
    DeviceBase(const DeviceInfo& devInfo, const std::filesystem::path& pathToCmd);

    /**
     * Connects to any available device with custom config.
     * @param config Device custom configuration to boot with
     */
    explicit DeviceBase(Config config);

    /**
     * Connects to device 'devInfo' with custom config.
     * @param config Device custom configuration to boot with
     * @param devInfo DeviceInfo which specifies which device to connect to
     */
    DeviceBase(Config config, const DeviceInfo& devInfo);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * Uses OpenVINO version OpenVINO::VERSION_UNIVERSAL
     *
     * @param devInfo DeviceInfo which specifies which device to connect to
     */
    explicit DeviceBase(const DeviceInfo& devInfo);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * Uses OpenVINO version OpenVINO::VERSION_UNIVERSAL
     *
     * @param nameOrDeviceId Creates DeviceInfo with nameOrDeviceId to connect to
     */
    DeviceBase(std::string nameOrDeviceId);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * Uses OpenVINO version OpenVINO::VERSION_UNIVERSAL
     *
     * @param nameOrDeviceId Creates DeviceInfo with nameOrDeviceId to connect to
     * @param maxUsbSpeed Maximum allowed USB speed
     */
    DeviceBase(std::string nameOrDeviceId, UsbSpeed maxUsbSpeed);

    /**
     * Connects to device specified by devInfo.
     * @param config Config with which the device will be booted with
     * @param maxUsbSpeed Maximum allowed USB speed
     */
    DeviceBase(Config config, UsbSpeed maxUsbSpeed);

    /**
     * Connects to any available device with a DEFAULT_SEARCH_TIME timeout.
     * @param config Config with which the device will be booted with
     * @param pathToCmd Path to custom device firmware
     */
    DeviceBase(Config config, const std::filesystem::path& pathToCmd);

    /**
     * Connects to device specified by devInfo.
     * @param config Config with which the device will be booted with
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param maxUsbSpeed Maximum allowed USB speed
     */
    DeviceBase(Config config, const DeviceInfo& devInfo, UsbSpeed maxUsbSpeed);

    /**
     * Connects to device specified by devInfo.
     * @param config Config with which the device will be booted with
     * @param devInfo DeviceInfo which specifies which device to connect to
     * @param pathToCmd Path to custom device firmware
     * @param dumpOnly If true only the minimal connection is established to retrieve the crash dump
     */
    DeviceBase(Config config, const DeviceInfo& devInfo, const std::filesystem::path& pathToCmd, bool dumpOnly = false);

    /**
     * Device destructor
     * @note In the destructor of the derived class, remember to call close()
     */
    virtual ~DeviceBase();

    /**
     * Gets Bootloader version if it was booted through Bootloader
     *
     * @returns DeviceBootloader::Version if booted through Bootloader or none otherwise
     */
    std::optional<Version> getBootloaderVersion();

    /**
     * Checks if devices pipeline is already running
     *
     * @returns True if running, false otherwise
     */
    bool isPipelineRunning();

    /**
     * Starts the execution of a given pipeline
     * @param pipeline OpenVINO version of the pipeline must match the one which the device was booted with.
     *
     * @returns True if pipeline started, false otherwise
     */
    bool startPipeline(const Pipeline& pipeline);

    /**
     * Sets the devices logging severity level. This level affects which logs are transferred from device to host.
     *
     * @param level Logging severity
     */
    void setLogLevel(LogLevel level);

    /**
     * Sets the logging severity level for a specific node with a given ID.
     *
     * @param id Node ID
     * @param level Logging severity
     */
    void setNodeLogLevel(int64_t id, LogLevel level);

    /**
     * Gets current logging severity level of the device.
     *
     * @returns Logging severity level
     */
    LogLevel getLogLevel();

    /**
     * Gets the logging severity level for a specific node with a given ID.
     *
     * @param id Node ID
     * @returns Logging severity level
     */
    LogLevel getNodeLogLevel(int64_t id);

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
     * Get device name if available
     * @returns device name or empty string if not available
     */
    std::string getDeviceName();

    /**
     * Get product name if available
     * @returns product name or empty string if not available
     */
    std::string getProductName();

    /**
     * Get MxId of device
     *
     * @returns MxId of connected device
     */
    [[deprecated("Use getDeviceId() instead")]] std::string getMxId();

    /**
     * Get DeviceId of device
     *
     * @returns DeviceId of connected device
     */
    std::string getDeviceId();

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
     * Sets the intensity of the IR Laser Dot Projector. Limits: up to 765mA at 30% frame time duty cycle when exposure time is longer than 30% frame time.
     * Otherwise, duty cycle is 100% of exposure time, with current increased up to max 1200mA to make up for shorter duty cycle.
     * The duty cycle is controlled by `left` camera STROBE, aligned to start of exposure.
     * The emitter is turned off by default
     *
     * @param intensity Intensity on range 0 to 1, that will determine brightness. 0 or negative to turn off
     * @param mask Optional mask to modify only Left (0x1) or Right (0x2) sides on OAK-D-Pro-W-DEV
     * @returns True on success, false if not found or other failure
     */
    bool setIrLaserDotProjectorIntensity(float intensity, int mask = -1);

    /**
     * Sets the intensity of the IR Flood Light. Limits: Intensity is directly normalized to 0 - 1500mA current.
     * The duty cycle is 30% when exposure time is longer than 30% frame time. Otherwise, duty cycle is 100% of exposure time.
     * The duty cycle is controlled by the `left` camera STROBE, aligned to start of exposure.
     * The emitter is turned off by default
     *
     * @param intensity Intensity on range 0 to 1, that will determine brightness, 0 or negative to turn off
     * @param mask Optional mask to modify only Left (0x1) or Right (0x2) sides on OAK-D-Pro-W-DEV
     * @returns True on success, false if not found or other failure
     */
    bool setIrFloodLightIntensity(float intensity, int mask = -1);

    /**
     * Retrieves detected IR laser/LED drivers.
     *
     * @returns Vector of tuples containing: driver name, I2C bus, I2C address.
     * For OAK-D-Pro it should be `[{"LM3644", 2, 0x63}]`
     */
    std::vector<std::tuple<std::string, int, int>> getIrDrivers();

    /**
     * Retrieves crash dump for debugging.
     */
    dai::CrashDump getCrashDump(bool clearCrashDump = true);

    /**
     * Retrieves whether the is crash dump stored on device or not.
     */
    bool hasCrashDump();

    /**
     * Get current accumulated profiling data
     *
     * @returns ProfilingData from the specific device
     */
    ProfilingData getProfilingData();

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
     * Get connection interfaces for device
     *
     * @returns Vector of connection type
     */
    std::vector<ConnectionInterface> getConnectionInterfaces();

    /**
     * Get cameras that are connected to the device with their features/properties
     *
     * @returns Vector of connected camera features
     */
    std::vector<CameraFeatures> getConnectedCameraFeatures();

    /**
     * Get stereo pairs based on the device type.
     *
     * @returns Vector of stereo pairs
     */
    std::vector<StereoPair> getStereoPairs();

    /**
     * Get stereo pairs taking into account the calibration and connected cameras.
     *
     * @note This method will always return a subset of `getStereoPairs`.
     *
     * @returns Vector of stereo pairs
     */
    std::vector<StereoPair> getAvailableStereoPairs();

    /**
     * Get sensor names for cameras that are connected to the device
     *
     * @returns Map/dictionary with camera sensor names, indexed by socket
     */
    std::unordered_map<CameraBoardSocket, std::string> getCameraSensorNames();

    /**
     * Get connected IMU type
     *
     * @returns IMU type
     */
    std::string getConnectedIMU();

    /**
     * Get connected IMU firmware version
     *
     * @returns IMU firmware version
     */
    dai::Version getIMUFirmwareVersion();

    /**
     * Get embedded IMU firmware version to which IMU can be upgraded
     *
     * @returns Get embedded IMU firmware version to which IMU can be upgraded.
     */
    dai::Version getEmbeddedIMUFirmwareVersion();

    /**
     * Starts IMU firmware update asynchronously only if IMU node is not running.
     * If current firmware version is the same as embedded firmware version then it's no-op. Can be overridden by forceUpdate parameter.
     * State of firmware update can be monitored using getIMUFirmwareUpdateStatus API.
     *
     * @param forceUpdate Force firmware update or not. Will perform FW update regardless of current version and embedded firmware version.
     *
     * @returns Returns whether firmware update can be started. Returns false if IMU node is started.
     */
    bool startIMUFirmwareUpdate(bool forceUpdate = false);

    /**
     * Get IMU firmware update status
     *
     * @returns Whether IMU firmware update is done and last firmware update progress as percentage.
     * return value true and 100 means that the update was successful
     * return value true and other than 100 means that the update failed
     */
    std::tuple<bool, float> getIMUFirmwareUpdateStatus();

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
     * Check if EEPROM is available
     * @returns True if EEPROM is present on board, false otherwise
     */
    bool isEepromAvailable();

    /**
     * Stores the Calibration and Device information to the Device EEPROM
     *
     * @param calibrationObj CalibrationHandler object which is loaded with calibration information.
     *
     * @return true on successful flash, false on failure
     */
    bool tryFlashCalibration(CalibrationHandler calibrationDataHandler);

    /**
     * Stores the Calibration and Device information to the Device EEPROM
     *
     * @throws std::runtime_exception if failed to flash the calibration
     * @param calibrationObj CalibrationHandler object which is loaded with calibration information.
     */
    void flashCalibration(CalibrationHandler calibrationDataHandler);

    /**
     * Sets the Calibration at runtime. This is not persistent and will be lost after device reset.
     *
     * @throws std::runtime_error if failed to set the calibration
     * @param calibrationObj CalibrationHandler object which is loaded with calibration information.
     *
     */
    void setCalibration(CalibrationHandler calibrationDataHandler);

    /**
     * Retrieves the CalibrationHandler object containing the non-persistent calibration
     *
     * @throws std::runtime_exception if failed to get the calibration
     * @returns The CalibrationHandler object containing the non-persistent calibration
     */
    CalibrationHandler getCalibration();

    /**
     * Fetches the EEPROM data from the device and loads it into CalibrationHandler object
     * If no calibration is flashed, it returns default
     *
     * @return The CalibrationHandler object containing the calibration currently flashed on device EEPROM
     */
    CalibrationHandler readCalibration();

    /**
     * Fetches the EEPROM data from the device and loads it into CalibrationHandler object
     *
     * @throws std::runtime_exception if no calibration is flashed
     * @return The CalibrationHandler object containing the calibration currently flashed on device EEPROM
     */
    CalibrationHandler readCalibration2();

    /**
     * Fetches the EEPROM data from the device and loads it into CalibrationHandler object
     * If no calibration is flashed, it returns default
     *
     * @return The CalibrationHandler object containing the calibration currently flashed on device EEPROM
     */
    CalibrationHandler readCalibrationOrDefault();

    /**
     * Factory reset EEPROM data if factory backup is available.
     *
     * @throws std::runtime_exception If factory reset was unsuccessful
     */
    void factoryResetCalibration();

    /**
     * Stores the Calibration and Device information to the Device EEPROM in Factory area
     * To perform this action, correct env variable must be set
     *
     * @throws std::runtime_exception if failed to flash the calibration
     * @return True on successful flash, false on failure
     */
    void flashFactoryCalibration(CalibrationHandler calibrationHandler);

    /**
     * Destructive action, deletes User area EEPROM contents
     * Requires PROTECTED permissions
     *
     * @throws std::runtime_exception if failed to flash the calibration
     * @return True on successful flash, false on failure
     */
    void flashEepromClear();

    /**
     * Destructive action, deletes Factory area EEPROM contents
     * Requires FACTORY PROTECTED permissions
     *
     * @throws std::runtime_exception if failed to flash the calibration
     * @return True on successful flash, false on failure
     */
    void flashFactoryEepromClear();

    /**
     * Fetches the EEPROM data from Factory area and loads it into CalibrationHandler object
     *
     * @throws std::runtime_exception if no calibration is flashed
     * @return The CalibrationHandler object containing the calibration currently flashed on device EEPROM in Factory Area
     */
    CalibrationHandler readFactoryCalibration();

    /**
     * Fetches the EEPROM data from Factory area and loads it into CalibrationHandler object
     * If no calibration is flashed, it returns default
     *
     * @return The CalibrationHandler object containing the calibration currently flashed on device EEPROM in Factory Area
     */
    CalibrationHandler readFactoryCalibrationOrDefault();

    /**
     * Fetches the raw EEPROM data from User area
     *
     * @throws std::runtime_exception if any error occurred
     * @returns Binary dump of User area EEPROM data
     */
    std::vector<std::uint8_t> readCalibrationRaw();

    /**
     * Fetches the raw EEPROM data from Factory area
     *
     * @throws std::runtime_exception if any error occurred
     * @returns Binary dump of Factory area EEPROM data
     */
    std::vector<std::uint8_t> readFactoryCalibrationRaw();

    /**
     * Retrieves USB connection speed
     *
     * @returns USB connection speed of connected device if applicable. Unknown otherwise.
     */
    UsbSpeed getUsbSpeed();

    /**
     * Configures Timesync service on device. It keeps host and device clocks in sync
     * First time timesync is started it waits until the initial sync is completed
     * Afterwards the function changes the following parameters
     *
     * @param period Interval between timesync runs
     * @param numSamples Number of timesync samples per run which are used to compute a better value. Set to zero to disable timesync
     * @param random If true partial timesync requests will be performed at random intervals, otherwise at fixed intervals
     */
    void setTimesync(std::chrono::milliseconds period, int numSamples, bool random);

    /**
     * Enables or disables Timesync service on device. It keeps host and device clocks in sync.
     *
     * @param enable Enables or disables consistent timesyncing
     */
    void setTimesync(bool enable);

    /**
     * Explicitly closes connection to device.
     * @note This function does not need to be explicitly called
     * as destructor closes the device automatically
     */
    void close();

    /**
     * Is the device already closed (or disconnected)
     *
     * @warning This function is thread-unsafe and may return outdated incorrect values. It is
     * only meant for use in simple single-threaded code. Well written code should handle
     * exceptions when calling any DepthAI apis to handle hardware events and multithreaded use.
     */
    bool isClosed() const;

    /**
     * Crashes the device
     * @warning ONLY FOR TESTING PURPOSES, it causes an unrecoverable crash on the device
     */
    void crashDevice();

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
    enum class ReconnectionStatus { RECONNECTED, RECONNECTING, RECONNECT_FAILED };
    /**
     * Sets max number of automatic reconnection attempts
     * @param maxAttempts Maximum number of reconnection attempts, 0 to disable reconnection
     * @param callBack Callback to be called when reconnection is attempted
     */
    void setMaxReconnectionAttempts(int maxAttempts, std::function<void(ReconnectionStatus)> callBack = nullptr);

   protected:
    std::shared_ptr<XLinkConnection> connection;

    /**
     * @brief a safe way to start a pipeline, which is closed if any exception occurs
     */
    void tryStartPipeline(const Pipeline& pipeline);

    /**
     * Allows the derived classes to handle custom setup for starting the pipeline
     *
     * @param pipeline OpenVINO version of the pipeline must match the one which the device was booted with
     * @sa startPipeline
     * @note Remember to call this function in the overload to setup the communication properly
     *
     * @returns True if pipeline started, false otherwise
     */
    virtual bool startPipelineImpl(const Pipeline& pipeline);

    /**
     * Allows the derived classes to handle custom setup for gracefully stopping the pipeline
     *
     * @note Remember to call this function in the overload to setup the communication properly
     */
    virtual void closeImpl();

   protected:
    // protected functions
    void init();
    void init(const std::filesystem::path& pathToCmd);
    void init(UsbSpeed maxUsbSpeed);
    void init(UsbSpeed maxUsbSpeed, const std::filesystem::path& pathToMvcmd);
    void init(const Pipeline& pipeline);
    void init(const Pipeline& pipeline, UsbSpeed maxUsbSpeed);
    void init(const Pipeline& pipeline, const std::filesystem::path& pathToCmd);
    void init(const Pipeline& pipeline, const DeviceInfo& devInfo);
    void init(const Pipeline& pipeline, const DeviceInfo& devInfo, bool usb2Mode);
    void init(const Pipeline& pipeline, const DeviceInfo& devInfo, UsbSpeed maxUsbSpeed);
    void init(const Pipeline& pipeline, const DeviceInfo& devInfo, const std::filesystem::path& pathToCmd);
    void init(const Pipeline& pipeline, UsbSpeed maxUsbSpeed, const std::filesystem::path& pathToMvcmd);
    void init(Config config, UsbSpeed maxUsbSpeed, const std::filesystem::path& pathToMvcmd);
    void init(Config config, UsbSpeed maxUsbSpeed);
    void init(Config config, const std::filesystem::path& pathToCmd);
    void init(Config config, const DeviceInfo& devInfo, UsbSpeed maxUsbSpeed);
    void init(Config config, const DeviceInfo& devInfo, const std::filesystem::path& pathToCmd);

   private:
    // private functions
    void init2(Config cfg, const std::filesystem::path& pathToMvcmd, bool hasPipeline, bool reconnect = false);
    void tryGetDevice();
    struct PrevInfo {
        DeviceInfo deviceInfo;
        Config cfg;
        std::filesystem::path pathToMvcmd;
        bool hasPipeline;
    };
    void monitorCallback(std::chrono::milliseconds watchdogTimeout, PrevInfo prev);
    DeviceInfo deviceInfo = {};
    std::optional<Version> bootloaderVersion;

    // Log callback
    int uniqueCallbackId = 0;
    std::mutex logCallbackMapMtx;
    std::unordered_map<int, std::function<void(LogMessage)>> logCallbackMap;

    // Watchdog thread
    std::thread watchdogThread;
    std::atomic<bool> watchdogRunning{true};
    std::condition_variable watchdogCondVar;
    std::mutex watchdogMtx;

    // Timesync thread
    std::thread timesyncThread;
    std::atomic<bool> timesyncRunning{true};

    // Logging thread
    std::thread loggingThread;
    std::atomic<bool> loggingRunning{true};

    // Profiling thread
    std::thread profilingThread;
    std::atomic<bool> profilingRunning{true};

    // Monitor thread
    std::thread monitorThread;
    std::mutex lastWatchdogPingTimeMtx;
    std::chrono::steady_clock::time_point lastWatchdogPingTime;

    // RPC stream
    std::unique_ptr<XLinkStream> rpcStream;

    // DeviceGate connection
    std::unique_ptr<DeviceGate> gate;

    // closed
    mutable std::mutex closedMtx;
    bool closed{false};

    // pimpl
    class Impl;
    Pimpl<Impl> pimpl;

    // Device config
    Config config;

    std::filesystem::path firmwarePath;
    bool dumpOnly = false;

    // Started pipeline
    std::optional<PipelineSchema> pipelineSchema;

    // Reconnection attempts and pointer to reset connections
    int maxReconnectionAttempts = 1;
    std::weak_ptr<PipelineImpl> pipelinePtr;
    bool isClosing = false;  // if true, don't attempt to reconnect
    std::function<void(ReconnectionStatus)> reconnectionCallback = nullptr;
};
}  // namespace dai
