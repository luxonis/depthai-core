#include "depthai/device/DeviceBase.hpp"

// std
#include <XLink/XLinkPublicDefines.h>
#include <spdlog/fmt/ostr.h>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <optional>
#include <system_error>
#include <thread>

// shared
#include "depthai-bootloader-shared/Bootloader.hpp"
#include "depthai-bootloader-shared/XLinkConstants.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/device/CrashDump.hpp"
#include "depthai/log/LogConstants.hpp"
#include "depthai/log/LogLevel.hpp"
#include "depthai/log/LogMessage.hpp"
#include "depthai/pipeline/Assets.hpp"
#include "depthai/utility/Serialization.hpp"
#include "depthai/xlink/XLinkConstants.hpp"

// project
#include "DeviceLogger.hpp"
#include "depthai/device/EepromError.hpp"
#include "depthai/pipeline/node/internal/XLinkIn.hpp"
#include "depthai/pipeline/node/internal/XLinkOut.hpp"
#include "pipeline/Pipeline.hpp"
#include "utility/EepromDataParser.hpp"
#include "utility/Environment.hpp"
#include "utility/Files.hpp"
#include "utility/Initialization.hpp"
#include "utility/PimplImpl.hpp"
#include "utility/Resources.hpp"

// libraries
#include "XLink/XLink.h"
#include "XLink/XLinkTime.h"
#include "nanorpc/core/client.h"
#include "nanorpc/packer/nlohmann_msgpack.h"
#include "spdlog/details/os.h"
#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/chrono.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"
#include "utility/LogCollection.hpp"
#include "utility/Logging.hpp"
#include "utility/spdlog-fmt.hpp"

namespace {

struct ScopedRpcTimeout {
   public:
    static thread_local std::optional<std::chrono::milliseconds> tlRpcTimeout;

    explicit ScopedRpcTimeout(std::optional<std::chrono::milliseconds> timeout) : prev(tlRpcTimeout) {
        tlRpcTimeout = timeout;
    }
    ~ScopedRpcTimeout() {
        tlRpcTimeout = prev;
    }

    ScopedRpcTimeout(const ScopedRpcTimeout&) = delete;
    ScopedRpcTimeout& operator=(const ScopedRpcTimeout&) = delete;

    std::optional<std::chrono::milliseconds> prev;
};

thread_local std::optional<std::chrono::milliseconds> ScopedRpcTimeout::tlRpcTimeout;

std::optional<std::chrono::milliseconds> currentRpcTimeout() {
    return ScopedRpcTimeout::tlRpcTimeout;
}
}  // namespace
namespace dai {

const std::string MAGIC_PROTECTED_FLASHING_VALUE = "235539980";
const std::string MAGIC_FACTORY_FLASHING_VALUE = "413424129";
const std::string MAGIC_FACTORY_PROTECTED_FLASHING_VALUE = "868632271";
constexpr int DEVICE_SEARCH_FIRST_TIMEOUT_MS = 30;

const unsigned int DEFAULT_CRASHDUMP_TIMEOUT_MS = 9000;
const unsigned int RPC_READ_TIMEOUT = 10000;

// local static function
static void getFlashingPermissions(bool& factoryPermissions, bool& protectedPermissions) {
    auto permissionEnv = utility::getEnvAs<std::string>("DEPTHAI_ALLOW_FACTORY_FLASHING", "");
    if(permissionEnv == MAGIC_FACTORY_FLASHING_VALUE) {
        factoryPermissions = true;
        protectedPermissions = false;
    } else if(permissionEnv == MAGIC_PROTECTED_FLASHING_VALUE) {
        factoryPermissions = false;
        protectedPermissions = true;
    } else if(permissionEnv == MAGIC_FACTORY_PROTECTED_FLASHING_VALUE) {
        factoryPermissions = true;
        protectedPermissions = true;
    } else {
        factoryPermissions = false;
        protectedPermissions = false;
    }
}

constexpr std::chrono::seconds DeviceBase::DEFAULT_SEARCH_TIME;
constexpr float DeviceBase::DEFAULT_SYSTEM_INFORMATION_LOGGING_RATE_HZ;
constexpr UsbSpeed DeviceBase::DEFAULT_USB_SPEED;
constexpr std::chrono::milliseconds DeviceBase::DEFAULT_TIMESYNC_PERIOD;
constexpr bool DeviceBase::DEFAULT_TIMESYNC_RANDOM;
constexpr int DeviceBase::DEFAULT_TIMESYNC_NUM_SAMPLES;

std::chrono::milliseconds DeviceBase::getDefaultSearchTime() {
    std::chrono::milliseconds defaultSearchTime = DEFAULT_SEARCH_TIME;
    auto searchTimeStr = utility::getEnvAs<std::string>("DEPTHAI_SEARCH_TIMEOUT", "");

    if(!searchTimeStr.empty()) {
        // Try parsing the string as a number
        try {
            defaultSearchTime = std::chrono::milliseconds{std::stoi(searchTimeStr)};
        } catch(const std::invalid_argument& e) {
            logger::warn("DEPTHAI_SEARCH_TIMEOUT value invalid: {}", e.what());
        }
    }

    return defaultSearchTime;
}

std::tuple<bool, DeviceInfo> DeviceBase::getAnyAvailableDevice(std::chrono::milliseconds timeout) {
    return getAnyAvailableDevice(timeout, nullptr);
}

std::tuple<bool, DeviceInfo> DeviceBase::getAnyAvailableDevice(std::chrono::milliseconds timeout, std::function<void()> cb) {
    using namespace std::chrono;
    constexpr auto POOL_SLEEP_TIME = milliseconds(100);

    // First looks for UNBOOTED, then BOOTLOADER, for 'timeout' time
    auto searchStartTime = steady_clock::now();
    bool found = false;
    DeviceInfo deviceInfo;
    std::unordered_map<std::string, DeviceInfo> invalidDevices;
    bool first = true;
    do {
        int timeoutMs = XLINK_DEVICE_DEFAULT_SEARCH_TIMEOUT_MS;
        if(first) {
            timeoutMs = DEVICE_SEARCH_FIRST_TIMEOUT_MS;  // for the first iteraton, have a shorter timeout
            first = false;
        }
        auto devices = XLinkConnection::getAllConnectedDevices(X_LINK_ANY_STATE, false, timeoutMs);
        for(auto searchState : {X_LINK_UNBOOTED, X_LINK_BOOTLOADER, X_LINK_FLASH_BOOTED, X_LINK_GATE, X_LINK_GATE_SETUP}) {
            for(const auto& device : devices) {
                if(device.state == searchState) {
                    if(device.status == X_LINK_SUCCESS) {
                        found = true;
                        deviceInfo = device;
                        break;
                    } else {
                        found = false;
                        invalidDevices[device.name] = device;
                    }
                }
            }
            if(found) break;
        }
        if(found) break;

        // Call the callback
        if(cb) cb();

        // If 'timeout' < 'POOL_SLEEP_TIME', use 'timeout' as sleep time and then break
        if(timeout < POOL_SLEEP_TIME) {
            // sleep for 'timeout'
            std::this_thread::sleep_for(timeout);
            break;
        } else {
            std::this_thread::sleep_for(POOL_SLEEP_TIME);  // default pool rate
        }
    } while(steady_clock::now() - searchStartTime < timeout);

    // Check if its an invalid device
    for(const auto& invalidDevice : invalidDevices) {
        const auto& invalidDeviceInfo = invalidDevice.second;
        if(invalidDeviceInfo.status == X_LINK_INSUFFICIENT_PERMISSIONS) {
            logger::warn("Insufficient permissions to communicate with {} device with name \"{}\". Make sure udev rules are set",
                         XLinkDeviceStateToStr(invalidDeviceInfo.state),
                         invalidDeviceInfo.name);
        } else {
            // Warn
            logger::warn(
                "Skipping {} device with name \"{}\" ({})", XLinkDeviceStateToStr(invalidDeviceInfo.state), invalidDeviceInfo.name, invalidDeviceInfo.deviceId);
        }
    }

    // If none were found, try BOOTED
    if(!found) std::tie(found, deviceInfo) = XLinkConnection::getFirstDevice(X_LINK_BOOTED);

    return {found, deviceInfo};
}

// Default overload ('DEFAULT_SEARCH_TIME' timeout)
std::tuple<bool, DeviceInfo> DeviceBase::getAnyAvailableDevice() {
    return getAnyAvailableDevice(getDefaultSearchTime());
}

// static api

// First tries to find UNBOOTED device, then BOOTLOADER device
std::tuple<bool, DeviceInfo> DeviceBase::getFirstAvailableDevice(bool skipInvalidDevice) {
    // Get all connected devices
    auto devices = XLinkConnection::getAllConnectedDevices(X_LINK_ANY_STATE, skipInvalidDevice);
    // Search order - first unbooted, then bootloader and last flash booted
    for(auto searchState : {X_LINK_UNBOOTED, X_LINK_BOOTLOADER, X_LINK_FLASH_BOOTED, X_LINK_GATE, X_LINK_GATE_SETUP}) {
        for(const auto& device : devices) {
            if(device.state == searchState) {
                return {true, device};
            }
        }
    }
    return {false, {}};
}

// Returns all devices which aren't already booted
std::vector<DeviceInfo> DeviceBase::getAllAvailableDevices() {
    std::vector<DeviceInfo> availableDevices;
    auto connectedDevices = XLinkConnection::getAllConnectedDevices();
    for(const auto& d : connectedDevices) {
        if(d.state != X_LINK_BOOTED) availableDevices.push_back(d);
    }
    return availableDevices;
}

// Returns all devices, also the ones that are already booted
std::vector<DeviceInfo> DeviceBase::getAllConnectedDevices() {
    return XLinkConnection::getAllConnectedDevices();
}

// First tries to find UNBOOTED device with deviceId, then BOOTLOADER device with deviceId
std::tuple<bool, DeviceInfo> DeviceBase::getDeviceById(std::string deviceId) {
    std::vector<DeviceInfo> availableDevices;
    auto states = {X_LINK_UNBOOTED, X_LINK_BOOTLOADER, X_LINK_GATE, X_LINK_GATE_SETUP};
    bool found;
    DeviceInfo dev;
    for(const auto& state : states) {
        std::tie(found, dev) = XLinkConnection::getDeviceById(deviceId, state);
        if(found) return {true, dev};
    }
    return {false, DeviceInfo()};
}

std::vector<std::uint8_t> DeviceBase::getEmbeddedDeviceBinary(bool usb2Mode, OpenVINO::Version version) {
    return Resources::getInstance().getDeviceFirmware(usb2Mode, version);
}

std::vector<std::uint8_t> DeviceBase::getEmbeddedDeviceBinary(Config config) {
    return Resources::getInstance().getDeviceFirmware(config);
}

ProfilingData DeviceBase::getGlobalProfilingData() {
    return XLinkConnection::getGlobalProfilingData();
}

/*
std::vector<DeviceInfo> DeviceBase::getAllConnectedDevices(){
    return XLinkConnection::getAllConnectedDevices();
}


std::tuple<bool, DeviceInfo> DeviceBase::getFirstDevice(){
    return XLinkConnection::getFirstAvailableDevice();
}
*/

///////////////////////////////////////////////
// Impl section - use this to hide dependencies
///////////////////////////////////////////////
class DeviceBase::Impl {
   public:
    Impl() = default;

    // Default sink
    std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> stdoutColorSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    // Device Logger
    DeviceLogger logger{"host", stdoutColorSink};

    // RPC
    std::mutex rpcMutex;
    std::shared_ptr<XLinkStream> rpcStream;
    std::unique_ptr<nanorpc::core::client<nanorpc::packer::nlohmann_msgpack>> rpcClient;

    void setLogLevel(LogLevel level);
    LogLevel getLogLevel();
    void setPattern(const std::string& pattern);

    /*
     * RPC call with custom timeout. Set timeout to 0 to enable endless wait.
     */
    template <typename... Args>
    auto rpcCall(std::chrono::milliseconds timeout, std::string name, Args&&... args) -> decltype(rpcClient->call(std::string(name),
                                                                                                                  std::forward<Args>(args)...)) {
        ScopedRpcTimeout guard(timeout);
        return rpcClient->call(name, std::forward<Args>(args)...);
    }

    /*
     * Default RPC call with timeout set to RPC_READ_TIMEOUT.
     */
    template <typename... Args>
    auto rpcCall(std::string name, Args&&... args) -> decltype(rpcClient->call(std::string(name), std::forward<Args>(args)...)) {
        // ScopedRpcTimeout guard(std::nullopt);
        return rpcClient->call(name, std::forward<Args>(args)...);
    }
};

void DeviceBase::Impl::setPattern(const std::string& pattern) {
    logger.set_pattern(pattern);
}

void DeviceBase::Impl::setLogLevel(LogLevel level) {
    // Converts LogLevel to spdlog and reconfigures logger level
    auto spdlogLevel = logLevelToSpdlogLevel(level, spdlog::level::warn);
    // Set level for all configured sinks
    logger.set_level(spdlogLevel);
}

LogLevel DeviceBase::Impl::getLogLevel() {
    // Converts spdlog to LogLevel
    return spdlogLevelToLogLevel(logger.level(), LogLevel::WARN);
}

///////////////////////////////////////////////
// END OF Impl section
///////////////////////////////////////////////

void DeviceBase::tryGetDevice() {
    // Searches for any available device for 'default' timeout
    bool found = false;
    std::tie(found, deviceInfo) = getAnyAvailableDevice();

    // If no device found, throw
    if(!found) {
        auto numConnected = getAllAvailableDevices().size();
        if(numConnected > 0) {
            throw std::runtime_error(fmt::format("No available devices ({} connected, but in use)", numConnected));
        } else {
            throw std::runtime_error("No available devices");
        }
    }
}

DeviceBase::DeviceBase(const DeviceInfo& devInfo) : DeviceBase(devInfo, DeviceBase::DEFAULT_USB_SPEED) {}

DeviceBase::DeviceBase(const DeviceInfo& devInfo, UsbSpeed maxUsbSpeed) : deviceInfo(devInfo) {
    init(maxUsbSpeed, "");
}

DeviceBase::DeviceBase(const DeviceInfo& devInfo, const std::filesystem::path& pathToCmd) : deviceInfo(devInfo) {
    Config cfg;

    init2(cfg, pathToCmd, false);
}

DeviceBase::DeviceBase() {
    init();
}

DeviceBase::DeviceBase(std::string nameOrDeviceId) : DeviceBase(dai::DeviceInfo(std::move(nameOrDeviceId))) {}

DeviceBase::DeviceBase(std::string nameOrDeviceId, UsbSpeed maxUsbSpeed) : DeviceBase(dai::DeviceInfo(std::move(nameOrDeviceId)), maxUsbSpeed) {}

DeviceBase::DeviceBase(UsbSpeed maxUsbSpeed) {
    init(maxUsbSpeed);
}

DeviceBase::DeviceBase(Config config, const DeviceInfo& devInfo, UsbSpeed maxUsbSpeed) : deviceInfo(devInfo) {
    init(config, maxUsbSpeed, "");
}

DeviceBase::DeviceBase(Config config, const DeviceInfo& devInfo, const std::filesystem::path& pathToCmd, bool dumpOnly)
    : deviceInfo(devInfo), dumpOnly(dumpOnly) {
    init2(config, pathToCmd, false);
}

DeviceBase::DeviceBase(Config config, const std::filesystem::path& pathToCmd) {
    init(config, pathToCmd);
}

DeviceBase::DeviceBase(Config config, UsbSpeed maxUsbSpeed) {
    init(config, maxUsbSpeed);
}

void DeviceBase::init() {
    tryGetDevice();

    Config cfg;
    init2(cfg, "", false);
}

void DeviceBase::init(const std::filesystem::path& pathToCmd) {
    tryGetDevice();

    Config cfg;
    init2(cfg, pathToCmd, false);
}

void DeviceBase::init(UsbSpeed maxUsbSpeed) {
    tryGetDevice();
    init(maxUsbSpeed, "");
}

void DeviceBase::init(Config config, UsbSpeed maxUsbSpeed) {
    tryGetDevice();
    init(config, maxUsbSpeed, "");
}

void DeviceBase::init(Config config, const std::filesystem::path& pathToCmd) {
    tryGetDevice();
    init2(config, pathToCmd, false);
}

void DeviceBase::init(Config config, const DeviceInfo& devInfo, UsbSpeed maxUsbSpeed) {
    deviceInfo = devInfo;
    init(config, maxUsbSpeed, "");
}

void DeviceBase::init(Config config, const DeviceInfo& devInfo, const std::filesystem::path& pathToCmd) {
    deviceInfo = devInfo;
    init2(config, pathToCmd, false);
}

DeviceBase::DeviceBase(Config config) {
    tryGetDevice();
    init2(config, {}, false);
}

DeviceBase::DeviceBase(Config config, const DeviceInfo& devInfo) : deviceInfo(devInfo) {
    init2(config, {}, false);
}

void DeviceBase::close() {
    std::unique_lock<std::mutex> lock(closedMtx);
    if(!closed) {
        closeImpl();
        closed = true;
    }
}

unsigned int getCrashdumpTimeout(XLinkProtocol_t protocol) {
    std::chrono::milliseconds protocolTimeout = (protocol == X_LINK_TCP_IP ? device::XLINK_TCP_WATCHDOG_TIMEOUT : device::XLINK_USB_WATCHDOG_TIMEOUT);
    int timeoutMs = utility::getEnvAs<int>("DEPTHAI_CRASHDUMP_TIMEOUT", DEFAULT_CRASHDUMP_TIMEOUT_MS + protocolTimeout.count());
    return timeoutMs;
}

void DeviceBase::closeImpl() {
    using namespace std::chrono;
    isClosing = true;
    auto t1 = steady_clock::now();
    bool shouldGetCrashDump = false;
    // Check if the device is RVC3 - in case it is, crash dump retrieval is done differently
    bool isRvc2 = deviceInfo.platform == X_LINK_MYRIAD_X;
    if(!dumpOnly && isRvc2) {
        pimpl->logger.debug("Device about to be closed...");
        try {
            if(hasCrashDump()) {
                connection->setRebootOnDestruction(true);
                auto dump = getCrashDump();
                logCollection::logCrashDump(pipelineSchema, dump, deviceInfo);
            } else {
                bool isRunning = pimpl->rpcCall("isRunning").as<bool>();
                shouldGetCrashDump = !isRunning;
                connection->setRebootOnDestruction(connection->getRebootOnDestruction() || shouldGetCrashDump);
                pimpl->logger.debug("Shutdown {}", isRunning ? "OK" : "error");
            }
        } catch(const std::exception& ex) {
            pimpl->logger.debug("shutdown call error: {}", ex.what());
            shouldGetCrashDump = true;
        }
    }

    bool waitForGate = true;
    if(!isRvc2) {
        // Check if the device is still alive and well, if yes, don't wait for gate, crash dump not relevant
        try {
            waitForGate = !pimpl->rpcCall("isRunning").as<bool>();
            pimpl->logger.debug("Will wait for gate: {}", waitForGate);
        } catch(const std::exception& ex) {
            pimpl->logger.debug("isRunning call error: {}", ex.what());
        }
    }

    // Close connection first; causes Xlink internal calls to unblock semaphore waits and
    // return error codes, which then allows queues to unblock
    // always manage ownership because other threads (e.g. watchdog) are running and need to
    // keep the shared_ptr valid (even if closed). Otherwise leads to using null pointers,
    // invalid memory, etc. which hard crashes main app
    connection->close();

    if(gate && !waitForGate) {
        gate->destroySession();
    }
    {
        std::lock_guard<std::mutex> lock(watchdogMtx);
        watchdogRunning = false;
        watchdogCondVar.notify_all();
    }

    if(watchdogThread.joinable()) watchdogThread.join();

    // Stop various threads
    timesyncRunning = false;
    loggingRunning = false;
    profilingRunning = false;

    // Then stop timesync
    if(timesyncThread.joinable()) timesyncThread.join();
    // And at the end stop logging thread
    if(loggingThread.joinable()) loggingThread.join();
    // And at the end stop profiling thread
    if(profilingThread.joinable()) profilingThread.join();
    // At the end stop the monitor thread
    if(monitorThread.joinable()) monitorThread.join();

    // If the device was operated through gate, wait for the session to end
    if(gate && waitForGate) {
        auto crashDump = gate->waitForSessionEnd();
        if(crashDump) {
            logCollection::logCrashDump(pipelineSchema, crashDump.value(), deviceInfo);
        }
    }

    // Close rpcStream
    pimpl->rpcStream = nullptr;
    pimpl->rpcClient = nullptr;

    if(!dumpOnly) {
        auto timeout = getCrashdumpTimeout(deviceInfo.protocol);
        // Get crash dump if needed
        if(shouldGetCrashDump && timeout > 0) {
            pimpl->logger.debug("Getting crash dump...");
            auto t1 = steady_clock::now();
            bool gotDump = false;
            bool found = false;
            do {
                DeviceInfo rebootingDeviceInfo;
                std::tie(found, rebootingDeviceInfo) = XLinkConnection::getDeviceById(deviceInfo.getDeviceId(), X_LINK_ANY_STATE, false);
                if(found && (rebootingDeviceInfo.state == X_LINK_UNBOOTED || rebootingDeviceInfo.state == X_LINK_BOOTLOADER)) {
                    pimpl->logger.trace("Found rebooting device in {}ns", duration_cast<nanoseconds>(steady_clock::now() - t1).count());
                    DeviceBase rebootingDevice(config, rebootingDeviceInfo, firmwarePath, true);
                    if(rebootingDevice.hasCrashDump()) {
                        auto dump = rebootingDevice.getCrashDump();
                        logCollection::logCrashDump(pipelineSchema, dump, deviceInfo);
                    } else {
                        pimpl->logger.warn("Device crashed, but no crash dump could be extracted.");
                    }
                    gotDump = true;
                    break;
                }
            } while(!found && steady_clock::now() - t1 < std::chrono::milliseconds(timeout));
            if(!gotDump) {
                pimpl->logger.error("Device likely crashed but did not reboot in time to get the crash dump");
            }
        } else if(shouldGetCrashDump) {
            pimpl->logger.warn("Device crashed. Crash dump retrieval disabled.");
        }

        pimpl->logger.debug("Device closed, {}", duration_cast<milliseconds>(steady_clock::now() - t1).count());
    }
}

void DeviceBase::setMaxReconnectionAttempts(int maxAttempts, std::function<void(ReconnectionStatus)> callback) {
    maxReconnectionAttempts = maxAttempts;
    reconnectionCallback = std::move(callback);
}

// This function is thread-unsafe. The idea of "isClosed" is ephemerial and
// is invalidated during the return by value and continues to degrade in
// validity to the caller
bool DeviceBase::isClosed() const {
    std::unique_lock<std::mutex> lock(closedMtx);
    return closed || !watchdogRunning;
}

DeviceBase::~DeviceBase() {
    DeviceBase::close();
}

void DeviceBase::tryStartPipeline(const Pipeline& pipeline) {
    try {
        if(!startPipeline(pipeline)) {
            throw std::runtime_error("Couldn't start the pipeline");
        }
    } catch(const std::exception&) {
        // close device (cleanup)
        close();
        // Rethrow original exception
        throw;
    }
}

void DeviceBase::init(UsbSpeed maxUsbSpeed, const std::filesystem::path& pathToMvcmd) {
    Config cfg;
    // Specify usb speed
    cfg.board.usb.maxSpeed = maxUsbSpeed;
    init2(cfg, pathToMvcmd, false);
}
void DeviceBase::init(const Pipeline& pipeline, UsbSpeed maxUsbSpeed, const std::filesystem::path& pathToMvcmd) {
    Config cfg = pipeline.getDeviceConfig();
    // Modify usb speed
    cfg.board.usb.maxSpeed = maxUsbSpeed;
    init2(cfg, pathToMvcmd, true);
}
void DeviceBase::init(Config config, UsbSpeed maxUsbSpeed, const std::filesystem::path& pathToMvcmd) {
    Config cfg = config;
    // Modify usb speed
    cfg.board.usb.maxSpeed = maxUsbSpeed;
    init2(cfg, pathToMvcmd, {});
}

void DeviceBase::init2(Config cfg, const std::filesystem::path& pathToMvcmd, bool hasPipeline, bool reconnect) {
    // Initalize depthai library if not already
    if(!dumpOnly) initialize();

    // Save previous state in case of a reconnection attempt
    PrevInfo prev;
    prev.deviceInfo = deviceInfo;
    prev.cfg = cfg;
    prev.pathToMvcmd = std::filesystem::path(pathToMvcmd);
    prev.hasPipeline = hasPipeline;

    // Specify cfg
    config = cfg;
    firmwarePath = pathToMvcmd;

    // Apply nonExclusiveMode
    config.board.nonExclusiveMode = config.nonExclusiveMode;

    // Apply device specific logger level
    {
        auto deviceLogLevel = config.logLevel.value_or(spdlogLevelToLogLevel(logger::get_level()));
        setLogOutputLevel(config.outputLogLevel.value_or(deviceLogLevel));
    }

    // Specify expected running mode
    XLinkDeviceState_t expectedBootState = X_LINK_BOOTED;
    if(config.nonExclusiveMode) {
        expectedBootState = X_LINK_BOOTED_NON_EXCLUSIVE;
    }

    // If deviceInfo isn't fully specified (eg ANY_STATE, etc...), try finding it first
    if(deviceInfo.state == X_LINK_ANY_STATE || deviceInfo.protocol == X_LINK_ANY_PROTOCOL) {
        auto timeout = getDefaultSearchTime();
        auto startSearchTime = std::chrono::steady_clock::now();
        constexpr auto SEARCH_INTERVAL = std::chrono::milliseconds(100);

        deviceDesc_t foundDesc;
        XLinkError_t ret = X_LINK_DEVICE_NOT_FOUND;
        do {
            ret = XLinkFindFirstSuitableDevice(deviceInfo.getXLinkDeviceDesc(), &foundDesc);
            if(ret == X_LINK_SUCCESS) {
                pimpl->logger.trace("Found device by given DeviceInfo: {}", deviceInfo.toString());
                break;
            }
            if(timeout < SEARCH_INTERVAL) {
                std::this_thread::sleep_for(timeout);
                break;
            } else {
                std::this_thread::sleep_for(SEARCH_INTERVAL);
            }
        } while(std::chrono::steady_clock::now() - startSearchTime < timeout);

        if(ret == X_LINK_SUCCESS) {
            deviceInfo = DeviceInfo(foundDesc);
            pimpl->logger.debug("Found an actual device by given DeviceInfo: {}", deviceInfo.toString());
        } else {
            deviceInfo.state = X_LINK_ANY_STATE;
            pimpl->logger.error("Searched, but no actual device found by given DeviceInfo: {}", deviceInfo.toString());
        }
    }

    if(hasPipeline) {
        pimpl->logger.debug("Device - pipeline serialized, OpenVINO version: {}", OpenVINO::getVersionName(config.version));
    } else {
        pimpl->logger.debug("Device - OpenVINO version: {}", OpenVINO::getVersionName(config.version));
    }

    // Set logging pattern of device (device id + shared pattern)
    pimpl->setPattern(fmt::format("[{}] [{}] {}", deviceInfo.deviceId, deviceInfo.name, LOG_DEFAULT_PATTERN));

    // Check if WD env var is set
    std::chrono::milliseconds watchdogTimeout = device::XLINK_USB_WATCHDOG_TIMEOUT;
    if(deviceInfo.protocol == X_LINK_TCP_IP) {
        watchdogTimeout = device::XLINK_TCP_WATCHDOG_TIMEOUT;
    }
    auto watchdogMsStr = utility::getEnvAs<std::string>("DEPTHAI_WATCHDOG", "");
    if(!watchdogMsStr.empty()) {
        // Try parsing the string as a number
        try {
            std::chrono::milliseconds watchdog{std::stoi(watchdogMsStr)};
            config.board.watchdogTimeoutMs = static_cast<uint32_t>(watchdog.count());
            watchdogTimeout = watchdog;
            if(watchdogTimeout.count() == 0) {
                pimpl->logger.warn("Watchdog disabled! In case of unclean exit, the device needs reset or power-cycle for next run", watchdogTimeout);
            } else {
                pimpl->logger.warn("Using a custom watchdog value of {}", watchdogTimeout);
            }
        } catch(const std::invalid_argument& e) {
            pimpl->logger.warn("DEPTHAI_WATCHDOG value invalid: {}", e.what());
        }
    }

    auto watchdogInitMsStr = utility::getEnvAs<std::string>("DEPTHAI_WATCHDOG_INITIAL_DELAY", "");
    if(!watchdogInitMsStr.empty()) {
        // Try parsing the string as a number
        try {
            std::chrono::milliseconds watchdog{std::stoi(watchdogInitMsStr)};
            config.board.watchdogInitialDelayMs = static_cast<uint32_t>(watchdog.count());
            pimpl->logger.warn("Watchdog initial delay set to {}", watchdog);
        } catch(const std::invalid_argument& e) {
            pimpl->logger.warn("DEPTHAI_WATCHDOG_INITIAL_DELAY value invalid: {}", e.what());
        }
    }

    auto deviceDebugStr = utility::getEnvAs<std::string>("DEPTHAI_DEBUG", "");
    if(!deviceDebugStr.empty()) {
        // Try parsing the string as a number
        try {
            int deviceDebug{std::stoi(deviceDebugStr)};
            config.board.logDevicePrints = deviceDebug;
        } catch(const std::invalid_argument& e) {
            pimpl->logger.warn("DEPTHAI_DEBUG value invalid: {}, should be a number (non-zero to enable)", e.what());
        }
    }

    // Get embedded mvcmd or external with applied config
    if(getLogOutputLevel() <= LogLevel::DEBUG) {
        nlohmann::json jBoardConfig = config.board;
        pimpl->logger.debug("Device - BoardConfig: {} \nlibnop:{}", jBoardConfig.dump(), spdlog::to_hex(utility::serialize(config.board)));
    }

    // Init device (if bootloader, handle correctly - issue USB boot command)
    if(deviceInfo.state == X_LINK_UNBOOTED) {
        // Unbooted device found, boot and connect with XLinkConnection constructor
        std::vector<std::uint8_t> fwWithConfig = Resources::getInstance().getDeviceFirmware(config, pathToMvcmd);
        connection = std::make_shared<XLinkConnection>(deviceInfo, fwWithConfig);
    } else if(deviceInfo.state == X_LINK_BOOTLOADER || deviceInfo.state == X_LINK_FLASH_BOOTED) {
        std::vector<std::uint8_t> fwWithConfig = Resources::getInstance().getDeviceFirmware(config, pathToMvcmd);
        // Scope so DeviceBootloader is disconnected
        {
            DeviceBootloader bl(deviceInfo);
            auto version = bl.getVersion();
            // Save DeviceBootloader version, to be able to retrieve later optionally
            bootloaderVersion = version;

            // If version is >= 0.0.12 then boot directly, otherwise jump to USB ROM bootloader
            // Check if version is recent enough for this operation
            if(version >= DeviceBootloader::Version(0, 0, 12)) {
                using namespace std::chrono;
                // Boot the given FW
                auto t1 = steady_clock::now();
                bl.bootMemory(fwWithConfig);
                auto t2 = steady_clock::now();
                pimpl->logger.debug("Booting FW with Bootloader. Version {}, Time taken: {}", version.toString(), duration_cast<milliseconds>(t2 - t1));

                // After that the state will be expectedBootState
                deviceInfo.state = expectedBootState;
            } else {
                // Boot into USB ROM BOOTLOADER
                bl.bootUsbRomBootloader();
                pimpl->logger.debug("Booting FW by jumping to USB ROM Bootloader first. Bootloader Version {}", version.toString());

                // After that the state will be UNBOOTED
                deviceInfo.state = X_LINK_UNBOOTED;
            }
        }

        // Boot and connect with XLinkConnection constructor
        connection = std::make_shared<XLinkConnection>(deviceInfo, fwWithConfig, expectedBootState);

    } else if(deviceInfo.state == X_LINK_BOOTED) {
        // Connect without booting
        std::vector<std::uint8_t> fwWithConfig = Resources::getInstance().getDeviceFirmware(config, pathToMvcmd);
        connection = std::make_shared<XLinkConnection>(deviceInfo, fwWithConfig);
    } else if(deviceInfo.state == X_LINK_GATE || deviceInfo.state == X_LINK_GATE_BOOTED || deviceInfo.state == X_LINK_GATE_SETUP) {
        // Boot FW using DeviceGate then connect directly
        gate = std::make_unique<DeviceGate>(deviceInfo);

        // Get version for debug
        if(spdlog::get_level() <= spdlog::level::debug) {
            auto info = gate->getAllVersion();
            spdlog::debug("OS version: {}, Gate version: {}", info.os, info.gate);
        }

        // Create and start session
        // TODO Tie create and start session together. Split for now, since in some cases starting the session works, even if creating failed.
        if(!gate->createSession()) {
            spdlog::error("Could not create the session on gate!");
        }

        if(!gate->startSession()) {
            spdlog::error("Could not start the session on gate!");
        }
        // Connect with XLinkConnection (skip checking if booted)
        connection = std::make_shared<XLinkConnection>(deviceInfo, X_LINK_ANY_STATE);
    } else {
        throw std::runtime_error("Cannot find any device with given deviceInfo");
    }

    deviceInfo.state = expectedBootState;

    // prepare rpc for both attached and host controlled mode
    pimpl->rpcStream = std::make_shared<XLinkStream>(connection, device::XLINK_CHANNEL_MAIN_RPC, device::XLINK_USB_BUFFER_MAX_SIZE);
    auto rpcStream = pimpl->rpcStream;

    pimpl->rpcClient = std::make_unique<nanorpc::core::client<nanorpc::packer::nlohmann_msgpack>>([this, rpcStream](nanorpc::core::type::buffer request) {
        // Lock for time of the RPC call, to not mix the responses between calling threads.
        // Note: might cause issues on Windows on incorrect shutdown. To be investigated
        std::unique_lock<std::mutex> lock(pimpl->rpcMutex);

        // Log the request data
        if(getLogOutputLevel() == LogLevel::TRACE) {
            pimpl->logger.trace("RPC: {}", nlohmann::json::from_msgpack(request).dump());
        }

        try {
            // Send request to device
            rpcStream->write(std::move(request));

            // Receive response back
            // Send to nanorpc to parse
            std::chrono::milliseconds timeout =
                currentRpcTimeout().value_or(std::chrono::milliseconds{RPC_READ_TIMEOUT});  // if no timeout specified, defaults to RPC_READ_TIMEOUT
            if(timeout.count() > 0) {
                return rpcStream->read(timeout);
            }
            logger::trace("Endless wait for RPC client.");
            return rpcStream->read();  // endless wait
        } catch(const std::exception& e) {
            // If any exception is thrown, log it and rethrow
            pimpl->logger.debug("RPC error: {}", e.what());
            throw std::system_error(std::make_error_code(std::errc::io_error), "Device already closed or disconnected");
        }
    });

    // prepare watchdog thread, which will keep device alive
    // separate stream so it doesn't miss between potentially long RPC calls
    // Only create the thread if watchdog is enabled
    if(watchdogTimeout > std::chrono::milliseconds(0)) {
        // Specify "last" ping time (5s in the future, for some grace time)
        {
            std::unique_lock<std::mutex> lock(lastWatchdogPingTimeMtx);
            lastWatchdogPingTime = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        }

        // Start watchdog thread for device
        watchdogThread = std::thread([this, watchdogTimeout]() {
            try {
                XLinkStream stream(connection, device::XLINK_CHANNEL_WATCHDOG, 128);
                std::vector<uint8_t> watchdogKeepalive = {0, 0, 0, 0};
                while(watchdogRunning) {
                    stream.write(watchdogKeepalive);
                    {
                        std::unique_lock<std::mutex> lock(lastWatchdogPingTimeMtx);
                        lastWatchdogPingTime = std::chrono::steady_clock::now();
                    }
                    std::unique_lock<std::mutex> lock(watchdogMtx);
                    // Calculate the dynamic sleep time based on protocol
                    auto sleepDuration = (deviceInfo.protocol == X_LINK_TCP_IP) ? watchdogTimeout / 4 : watchdogTimeout / 2;
                    watchdogCondVar.wait_for(lock, sleepDuration, [this]() { return !watchdogRunning; });
                }
            } catch(const std::exception& ex) {
                // ignore
                pimpl->logger.debug("Watchdog thread exception caught: {}", ex.what());
            }

            // Watchdog ended. Useful for checking disconnects
            watchdogRunning = false;
        });

        // Start monitor thread for host - makes sure that device is responding to pings, otherwise it disconnects
        if(!reconnect) monitorThread = std::thread(&DeviceBase::monitorCallback, this, watchdogTimeout, prev);
    } else {
        // Still set watchdogRunning explictitly
        // as it indicates device not being closed
        watchdogRunning = true;
    }

    if(!dumpOnly) {
        // Below can throw - make sure to gracefully exit threads
        try {
            auto level = spdlogLevelToLogLevel(logger::get_level());
            setLogLevel(config.logLevel.value_or(level));

            // Sets system inforation logging rate. By default 1s
            setSystemInformationLoggingRate(DEFAULT_SYSTEM_INFORMATION_LOGGING_RATE_HZ);
        } catch(const std::exception&) {
            // close device (cleanup)
            close();
            // Rethrow original exception
            throw;
        }

        // prepare timesync thread, which will keep device synchronized
        timesyncThread = std::thread([this]() {
            using namespace std::chrono;

            try {
                XLinkStream stream(connection, device::XLINK_CHANNEL_TIMESYNC, 128);
                while(timesyncRunning) {
                    // Block
                    XLinkTimespec timestamp;
                    stream.read(timestamp);

                    // Write timestamp back
                    stream.write(&timestamp, sizeof(timestamp));
                }
            } catch(const std::exception& ex) {
                // ignore
                pimpl->logger.debug("Timesync thread exception caught: {}", ex.what());
            }

            timesyncRunning = false;
        });

        // prepare logging thread, which will log device messages
        loggingThread = std::thread([this]() {
            using namespace std::chrono;
            std::vector<LogMessage> messages;
            try {
                XLinkStream stream(connection, device::XLINK_CHANNEL_LOG, 128);
                while(loggingRunning) {
                    // Block
                    auto log = stream.read();

                    try {
                        // Deserialize incoming messages
                        utility::deserialize(log, messages);

                        pimpl->logger.trace("Log vector decoded, size: {}", messages.size());

                        // log the messages in incremental order (0 -> size-1)
                        for(const auto& msg : messages) {
                            pimpl->logger.logMessage(msg);
                        }

                        // Log to callbacks
                        {
                            // lock mtx to callback map (shared)
                            std::unique_lock<std::mutex> l(logCallbackMapMtx);
                            for(const auto& msg : messages) {
                                for(const auto& kv : logCallbackMap) {
                                    const auto& cb = kv.second;
                                    // If available, callback with msg
                                    if(cb) cb(msg);
                                }
                            }
                        }

                    } catch(const nlohmann::json::exception& ex) {
                        pimpl->logger.error("Exception while parsing or calling callbacks for log message from device: {}", ex.what());
                    }
                }
            } catch(const std::exception& ex) {
                // ignore exception from logging
                pimpl->logger.debug("Log thread exception caught: {}", ex.what());
            }

            loggingRunning = false;
        });

        if(utility::getEnvAs<std::string>("DEPTHAI_PROFILING", "") == "1") {
            // prepare profiling thread, which will log device messages
            profilingThread = std::thread([this]() {
                using namespace std::chrono;
                try {
                    ProfilingData lastData = {};
                    // TODO(themarpe) - expose
                    float rate = 1.0f;
                    while(profilingRunning) {
                        ProfilingData data = getProfilingData();
                        long long w = data.numBytesWritten - lastData.numBytesWritten;
                        long long r = data.numBytesRead - lastData.numBytesRead;
                        w = static_cast<long long>(w / rate);
                        r = static_cast<long long>(r / rate);

                        lastData = data;

                        pimpl->logger.debug("Profiling write speed: {:.2f} MiB/s, read speed: {:.2f} MiB/s, total written: {:.2f} MiB, read: {:.2f} MiB",
                                            w / 1024.0f / 1024.0f,
                                            r / 1024.0f / 1024.0f,
                                            data.numBytesWritten / 1024.0f / 1024.0f,
                                            data.numBytesRead / 1024.0f / 1024.0f);

                        std::this_thread::sleep_for(duration<float>(1) / rate);
                    }
                } catch(const std::exception& ex) {
                    // ignore exception from logging
                    pimpl->logger.debug("Profiling thread exception caught: {}", ex.what());
                }

                profilingRunning = false;
            });
        }
        auto crashdumpPathStr = utility::getEnvAs<std::string>("DEPTHAI_CRASHDUMP", "");
        if(crashdumpPathStr == "0") {
            pimpl->rpcCall("enableCrashDump", false);
        } else {
            pimpl->rpcCall("enableCrashDump", true);
        }

        // Below can throw - make sure to gracefully exit threads
        try {
            // Starts and waits for inital timesync
            setTimesync(DEFAULT_TIMESYNC_PERIOD, DEFAULT_TIMESYNC_NUM_SAMPLES, DEFAULT_TIMESYNC_RANDOM);
        } catch(const std::exception&) {
            // close device (cleanup)
            close();
            // Rethrow original exception
            throw;
        }
    }
}

void DeviceBase::monitorCallback(std::chrono::milliseconds watchdogTimeout, PrevInfo prev) {
    try {
        while(true) {
            while(watchdogRunning) {
                // Ping with a period half of that of the watchdog timeout
                std::unique_lock<std::mutex> lock(watchdogMtx);
                watchdogCondVar.wait_for(lock, watchdogTimeout, [this]() { return !watchdogRunning; });
                // Check if wd was pinged in the specified watchdogTimeout time.
                decltype(lastWatchdogPingTime) prevPingTime;
                {
                    std::unique_lock<std::mutex> lock(lastWatchdogPingTimeMtx);
                    prevPingTime = lastWatchdogPingTime;
                }
                // Recheck if watchdogRunning wasn't already closed and close if more than twice of WD passed
                if(watchdogRunning && std::chrono::steady_clock::now() - prevPingTime > watchdogTimeout * 2) {
                    pimpl->logger.warn(
                        "Monitor thread (device: {} [{}]) - ping was missed, closing the device connection", deviceInfo.deviceId, deviceInfo.name);
                    // ping was missed, reset the device
                    watchdogRunning = false;
                    // close the underlying connection
                    connection->close();
                }
            }
            if(isClosing) {
                auto shared = pipelinePtr.lock();
                if(shared) {
                    shared->disconnectXLinkHosts();
                }
                return;
            }
            if(maxReconnectionAttempts == 0) {
                if(reconnectionCallback) reconnectionCallback(ReconnectionStatus::RECONNECT_FAILED);
                break;
            }
            auto reconnectionTimeoutMs = 10000;
            auto timeoutStr = utility::getEnvAs<std::string>("DEPTHAI_RECONNECT_TIMEOUT", "");
            if(!timeoutStr.empty()) {
                try {
                    reconnectionTimeoutMs = std::stoi(timeoutStr);
                    if(reconnectionTimeoutMs <= 0) {
                        pimpl->logger.info("Reconnection timeout set to 0, reconnection disabled");
                    }
                } catch(const std::invalid_argument& e) {
                    pimpl->logger.warn("DEPTHAI_RECONNECT_TIMEOUT value invalid: {}, should be a number (non-zero to enable)", e.what());
                }
            }

            if(reconnectionTimeoutMs <= 0) {
                if(reconnectionCallback) reconnectionCallback(ReconnectionStatus::RECONNECT_FAILED);
                break;
            }
            // reconnection attempt
            // stop other threads
            if(watchdogThread.joinable()) watchdogThread.join();
            timesyncRunning = false;
            loggingRunning = false;
            profilingRunning = false;
            if(timesyncThread.joinable()) timesyncThread.join();
            if(loggingThread.joinable()) loggingThread.join();
            if(profilingThread.joinable()) profilingThread.join();
            if(gate) {
                auto crashDump = gate->waitForSessionEnd();
                if(crashDump) {
                    logCollection::logCrashDump(pipelineSchema, crashDump.value(), deviceInfo);
                }
            }

            // get timeout (in seconds)
            std::chrono::milliseconds reconnectTimeout(reconnectionTimeoutMs);

            pimpl->logger.warn("Closed connection\n");
            // Reconnect
            deviceInfo = prev.deviceInfo;
            watchdogRunning = true;
            timesyncRunning = true;
            loggingRunning = true;
            profilingRunning = true;
            int attempts = 0;
            pimpl->logger.warn("Attempting to reconnect. Timeout is {}\n", reconnectTimeout);
            auto reconnected = false;
            for(attempts = 0; attempts < maxReconnectionAttempts; attempts++) {
                if(reconnectionCallback) reconnectionCallback(ReconnectionStatus::RECONNECTING);
                if(std::get<0>(getAnyAvailableDevice(reconnectTimeout))) {
                    init2(prev.cfg, prev.pathToMvcmd, prev.hasPipeline, true);
                    if(hasCrashDump()) {
                        auto dump = getCrashDump();
                        logCollection::logCrashDump(pipelineSchema, dump, deviceInfo);
                    }
                    auto shared = pipelinePtr.lock();
                    if(!shared) throw std::runtime_error("Pipeline was destroyed");
                    shared->resetConnections();
                    reconnected = true;
                    break;
                }
                pimpl->logger.warn("Reconnection unsuccessful, trying again. Attempts left: {}\n", maxReconnectionAttempts - attempts - 1);
            }
            if(!reconnected) {
                if(reconnectionCallback) reconnectionCallback(ReconnectionStatus::RECONNECT_FAILED);
                pimpl->logger.warn("Reconnection unsuccessful, closing the device\n");
                break;
            }
            if(reconnectionCallback) reconnectionCallback(ReconnectionStatus::RECONNECTED);
            pimpl->logger.warn("Reconnection successful\n");
        }
    } catch(const std::exception& ex) {
        pimpl->logger.info("Monitor thread exception caught: {}", ex.what());
    }
    // Close the pipeline
    auto shared = pipelinePtr.lock();
    if(shared) {
        shared->disconnectXLinkHosts();
    }
}

std::string DeviceBase::getMxId() {
    return pimpl->rpcCall("getMxId").as<std::string>();
}

std::string DeviceBase::getDeviceId() {
    return pimpl->rpcCall("getMxId").as<std::string>();
}

std::vector<CameraBoardSocket> DeviceBase::getConnectedCameras() {
    return pimpl->rpcCall("getConnectedCameras").as<std::vector<CameraBoardSocket>>();
}

std::vector<StereoPair> DeviceBase::getAvailableStereoPairs() {
    std::vector<dai::StereoPair> stereoPairs;
    dai::CalibrationHandler calibHandler;
    try {
        calibHandler = readCalibration2();
        if(calibHandler.getEepromData().cameraData.empty()) {
            throw std::runtime_error("No camera data found.");
        }
    } catch(const std::exception&) {
        try {
            calibHandler = readFactoryCalibration();
        } catch(const std::exception&) {
            pimpl->logger.info("No calibration found.");
            return stereoPairs;
        }
    }
    // Find links between cameras.
    for(auto const& camIdAndInfo1 : calibHandler.getEepromData().cameraData) {
        auto camId1 = camIdAndInfo1.first;
        for(auto const& camIdAndInfo2 : calibHandler.getEepromData().cameraData) {
            auto camId2 = camIdAndInfo2.first;
            try {
                auto translationVector = calibHandler.getCameraTranslationVector(camId1, camId2, false);
                auto baseline = std::abs(translationVector[0]) > std::abs(translationVector[1]) ? translationVector[0] : translationVector[1];  // X or Y
                auto leftSocket = baseline < 0 ? camId1 : camId2;
                auto rightSocket = leftSocket == camId1 ? camId2 : camId1;
                int baselineDiff = std::abs(static_cast<int>(translationVector[0]) - static_cast<int>(translationVector[1]));
                if(baselineDiff == static_cast<int>(std::abs(baseline))) {
                    if(std::find_if(stereoPairs.begin(),
                                    stereoPairs.end(),
                                    [&leftSocket, &rightSocket](const dai::StereoPair& pair) { return pair.left == leftSocket && pair.right == rightSocket; })
                       == stereoPairs.end()) {
                        stereoPairs.push_back(dai::StereoPair{leftSocket, rightSocket, std::abs(baseline), static_cast<int>(translationVector[0]) == 0});
                    }
                } else {
                    pimpl->logger.debug("Skipping diagonal pair, left: {}, right: {}.", leftSocket, rightSocket);
                }
            } catch(const std::exception&) {
                continue;
            }
        }
    }
    // Filter out undetected cameras and socket pairs which are not present in getStereoPairs
    auto deviceStereoPairs = getStereoPairs();
    auto connectedCameras = getConnectedCameras();
    std::vector<dai::StereoPair> filteredStereoPairs;
    std::copy_if(
        stereoPairs.begin(), stereoPairs.end(), std::back_inserter(filteredStereoPairs), [this, connectedCameras, deviceStereoPairs](dai::StereoPair pair) {
            if(std::find(connectedCameras.begin(), connectedCameras.end(), pair.left) == connectedCameras.end()) {
                pimpl->logger.debug("Skipping calibrated stereo pair because, camera {} was not detected.", pair.left);
                return false;
            } else if(std::find(connectedCameras.begin(), connectedCameras.end(), pair.right) == connectedCameras.end()) {
                pimpl->logger.debug("Skipping calibrated stereo pair because, camera {} was not detected.", pair.right);
                return false;
            }
            return std::find_if(deviceStereoPairs.begin(),
                                deviceStereoPairs.end(),
                                [pair](dai::StereoPair devicePair) { return devicePair.left == pair.left && devicePair.right == pair.right; })
                   != deviceStereoPairs.end();
        });

    std::sort(filteredStereoPairs.begin(), filteredStereoPairs.end(), [](dai::StereoPair a, dai::StereoPair b) { return a.baseline < b.baseline; });
    return filteredStereoPairs;
}

std::vector<ConnectionInterface> DeviceBase::getConnectionInterfaces() {
    return pimpl->rpcCall("getConnectionInterfaces").as<std::vector<ConnectionInterface>>();
}

std::vector<CameraFeatures> DeviceBase::getConnectedCameraFeatures() {
    return pimpl->rpcCall("getConnectedCameraFeatures").as<std::vector<CameraFeatures>>();
}

std::vector<StereoPair> DeviceBase::getStereoPairs() {
    return pimpl->rpcCall("getStereoPairs").as<std::vector<StereoPair>>();
}

std::unordered_map<CameraBoardSocket, std::string> DeviceBase::getCameraSensorNames() {
    return pimpl->rpcCall("getCameraSensorNames").as<std::unordered_map<CameraBoardSocket, std::string>>();
}

std::string DeviceBase::getConnectedIMU() {
    isClosed();
    return pimpl->rpcCall("getConnectedIMU").as<std::string>();
}

void DeviceBase::crashDevice() {
    isClosed();
    // Check that the protective ENV variable is set
    if(utility::getEnvAs<std::string>("DEPTHAI_CRASH_DEVICE", "") != "1") {
        pimpl->logger.error("Crashing the device is disabled. Set DEPTHAI_CRASH_DEVICE=1 to enable.");
        return;
    }
    try {
        pimpl->rpcCall("crashDevice");
    } catch(const std::system_error& ex) {
        pimpl->logger.debug("Crash device threw an exception: {} (expected)", ex.what());
    }
}

std::tuple<bool, std::string>  DeviceBase::setM8FsyncRole(M8FsyncRole role) {
    return pimpl->rpcClient->call("setM8FsyncRole", role);
}

M8FsyncRole DeviceBase::getM8FsyncRole() {
    return pimpl->rpcClient->call("getM8FsyncRole");
}

std::tuple<bool, std::string>  DeviceBase::setM8StrobeLimits(float min, float max) {
    return pimpl->rpcClient->call("setM8StrobeLimits", min, max);
}

void DeviceBase::setM8StrobeEnable(bool enable) {
    pimpl->rpcCall("setM8StrobeEnable", enable);
}

dai::Version DeviceBase::getIMUFirmwareVersion() {
    isClosed();
    std::string versionStr = pimpl->rpcCall("getIMUFirmwareVersion").as<std::string>();
    try {
        dai::Version version = dai::Version(versionStr);
        return version;
    } catch(const std::exception& ex) {
        dai::Version version = dai::Version(0, 0, 0);
        return version;
    }
}

dai::Version DeviceBase::getEmbeddedIMUFirmwareVersion() {
    isClosed();
    std::string versionStr = pimpl->rpcCall("getEmbeddedIMUFirmwareVersion").as<std::string>();
    try {
        dai::Version version = dai::Version(versionStr);
        return version;
    } catch(const std::exception& ex) {
        dai::Version version = dai::Version(0, 0, 0);
        return version;
    }
}

bool DeviceBase::startIMUFirmwareUpdate(bool forceUpdate) {
    isClosed();
    return pimpl->rpcCall("startIMUFirmwareUpdate", forceUpdate).as<bool>();
}

std::tuple<bool, float> DeviceBase::getIMUFirmwareUpdateStatus() {
    isClosed();
    return pimpl->rpcCall("getIMUFirmwareUpdateStatus").as<std::tuple<bool, float>>();
}

// Convenience functions for querying current system information
MemoryInfo DeviceBase::getDdrMemoryUsage() {
    return pimpl->rpcCall("getDdrUsage").as<MemoryInfo>();
}

MemoryInfo DeviceBase::getCmxMemoryUsage() {
    return pimpl->rpcCall("getCmxUsage").as<MemoryInfo>();
}

MemoryInfo DeviceBase::getLeonCssHeapUsage() {
    return pimpl->rpcCall("getLeonCssHeapUsage").as<MemoryInfo>();
}

MemoryInfo DeviceBase::getLeonMssHeapUsage() {
    return pimpl->rpcCall("getLeonMssHeapUsage").as<MemoryInfo>();
}

ChipTemperature DeviceBase::getChipTemperature() {
    return pimpl->rpcCall("getChipTemperature").as<ChipTemperature>();
}

CpuUsage DeviceBase::getLeonCssCpuUsage() {
    return pimpl->rpcCall("getLeonCssCpuUsage").as<CpuUsage>();
}

CpuUsage DeviceBase::getLeonMssCpuUsage() {
    return pimpl->rpcCall("getLeonMssCpuUsage").as<CpuUsage>();
}

int64_t DeviceBase::getProcessMemoryUsage() {
    return pimpl->rpcClient->call("getProcessMemoryUsage").as<int64_t>();
}

UsbSpeed DeviceBase::getUsbSpeed() {
    return pimpl->rpcCall("getUsbSpeed").as<UsbSpeed>();
}

bool DeviceBase::isNeuralDepthSupported() {
    return pimpl->rpcCall("isNeuralDepthSupported").as<bool>();
}

std::optional<Version> DeviceBase::getBootloaderVersion() {
    return bootloaderVersion;
}

bool DeviceBase::isPipelineRunning() {
    return pimpl->rpcCall("isPipelineRunning").as<bool>();
}

void DeviceBase::setLogLevel(LogLevel level) {
    pimpl->rpcCall("setLogLevel", level);
}

void DeviceBase::setNodeLogLevel(int64_t id, LogLevel level) {
    pimpl->rpcCall("setNodeLogLevel", id, level);
}

LogLevel DeviceBase::getLogLevel() {
    return pimpl->rpcCall("getLogLevel").as<LogLevel>();
}

LogLevel DeviceBase::getNodeLogLevel(int64_t id) {
    return pimpl->rpcCall("getNodeLogLevel", id).as<LogLevel>();
}

void DeviceBase::setXLinkChunkSize(int sizeBytes) {
    pimpl->rpcCall("setXLinkChunkSize", sizeBytes);
}

int DeviceBase::getXLinkChunkSize() {
    return pimpl->rpcCall("getXLinkChunkSize").as<int>();
}

DeviceInfo DeviceBase::getDeviceInfo() const {
    return deviceInfo;
}

std::string DeviceBase::getProductName() {
    EepromData eepromFactory = readFactoryCalibrationOrDefault().getEepromData();
    EepromData eeprom = readCalibrationOrDefault().getEepromData();
    return utility::parseProductName(eeprom, eepromFactory);
}

std::string DeviceBase::getDeviceName() {
    EepromData eepromFactory = readFactoryCalibrationOrDefault().getEepromData();
    EepromData eeprom = readCalibrationOrDefault().getEepromData();
    return utility::parseDeviceName(eeprom, eepromFactory);
}

void DeviceBase::setLogOutputLevel(LogLevel level) {
    pimpl->setLogLevel(level);
}

LogLevel DeviceBase::getLogOutputLevel() {
    return pimpl->getLogLevel();
}

bool DeviceBase::setIrLaserDotProjectorIntensity(float intensity, int mask) {
    return pimpl->rpcCall("setIrLaserDotProjectorBrightness", intensity, mask, true);
}

bool DeviceBase::setIrFloodLightIntensity(float intensity, int mask) {
    return pimpl->rpcCall("setIrFloodLightBrightness", intensity, mask, true);
}

std::vector<std::tuple<std::string, int, int>> DeviceBase::getIrDrivers() {
    return pimpl->rpcCall("getIrDrivers");
}

dai::CrashDump DeviceBase::getCrashDump(bool clearCrashDump) {
    return pimpl->rpcCall("getCrashDump", clearCrashDump).as<dai::CrashDump>();
}

bool DeviceBase::hasCrashDump() {
    return pimpl->rpcCall("hasCrashDump").as<bool>();
}

ProfilingData DeviceBase::getProfilingData() {
    return connection->getProfilingData();
}

int DeviceBase::addLogCallback(std::function<void(LogMessage)> callback) {
    // Lock first
    std::unique_lock<std::mutex> l(logCallbackMapMtx);

    // Get unique id
    int id = uniqueCallbackId++;

    // assign callback
    logCallbackMap[id] = callback;

    // return id assigned to the callback
    return id;
}

bool DeviceBase::removeLogCallback(int callbackId) {
    // Lock first
    std::unique_lock<std::mutex> l(logCallbackMapMtx);

    // If callback with id 'callbackId' doesn't exists, return false
    if(logCallbackMap.count(callbackId) == 0) return false;

    // Otherwise erase and return true
    logCallbackMap.erase(callbackId);
    return true;
}

void DeviceBase::setTimesync(std::chrono::milliseconds period, int numSamples, bool random) {
    if(period < std::chrono::milliseconds(10)) {
        throw std::invalid_argument("Period must be greater or equal than 10ms");
    }

    using namespace std::chrono;
    pimpl->rpcCall("setTimesync", duration_cast<milliseconds>(period).count(), numSamples, random);
}

void DeviceBase::setTimesync(bool enable) {
    if(enable) {
        setTimesync(DEFAULT_TIMESYNC_PERIOD, DEFAULT_TIMESYNC_NUM_SAMPLES, DEFAULT_TIMESYNC_RANDOM);
    } else {
        setTimesync(std::chrono::milliseconds(1000), 0, false);
    }
}

void DeviceBase::setSystemInformationLoggingRate(float rateHz) {
    pimpl->rpcCall("setSystemInformationLoggingRate", rateHz);
}

float DeviceBase::getSystemInformationLoggingRate() {
    return pimpl->rpcCall("getSystemInformationLoggingRate").as<float>();
}

bool DeviceBase::isEepromAvailable() {
    return pimpl->rpcCall("isEepromAvailable").as<bool>();
}

bool DeviceBase::tryFlashCalibration(CalibrationHandler calibrationDataHandler) {
    try {
        flashCalibration(calibrationDataHandler);
    } catch(const EepromError& e) {
        pimpl->logger.error("Failed to flash calibration: {}", e.what());
        return false;
    }
    return true;
}

void DeviceBase::flashCalibration(CalibrationHandler calibrationDataHandler) {
    bool factoryPermissions = false;
    bool protectedPermissions = false;
    getFlashingPermissions(factoryPermissions, protectedPermissions);
    pimpl->logger.debug("Flashing calibration. Factory permissions {}, Protected permissions {}", factoryPermissions, protectedPermissions);

    /* if(!calibrationDataHandler.validateCameraArray()) {
        throw std::runtime_error("Failed to validate the extrinsics connection. Enable debug mode for more information.");
    } */

    bool success;
    std::string errorMsg;
    std::tie(success, errorMsg) =
        pimpl->rpcCall("storeToEeprom", calibrationDataHandler.getEepromData(), factoryPermissions, protectedPermissions).as<std::tuple<bool, std::string>>();

    if(!success) {
        throw EepromError(errorMsg);
    }
}

void DeviceBase::setCalibration(CalibrationHandler calibrationDataHandler) {
    bool success;
    std::string errorMsg;
    std::tie(success, errorMsg) = pimpl->rpcCall("setCalibration", calibrationDataHandler.getEepromData()).as<std::tuple<bool, std::string>>();
    if(!success) {
        throw std::runtime_error(errorMsg);
    }
}

CalibrationHandler DeviceBase::getCalibration() {
    bool success;
    std::string errorMsg;
    dai::EepromData eepromData;
    std::tie(success, errorMsg, eepromData) = pimpl->rpcCall("getCalibration").as<std::tuple<bool, std::string, dai::EepromData>>();
    if(!success) {
        throw EepromError(errorMsg);
    }
    return CalibrationHandler(eepromData);
}

CalibrationHandler DeviceBase::readCalibration() {
    dai::EepromData eepromData{};
    try {
        return readCalibration2();
    } catch(const EepromError&) {
        // ignore - use default
    }
    return CalibrationHandler(eepromData);
}
CalibrationHandler DeviceBase::readCalibration2() {
    bool success;
    std::string errorMsg;
    dai::EepromData eepromData;
    std::tie(success, errorMsg, eepromData) = pimpl->rpcCall("readFromEeprom").as<std::tuple<bool, std::string, dai::EepromData>>();
    if(!success) {
        throw EepromError(errorMsg);
    }
    return CalibrationHandler(eepromData);
}

CalibrationHandler DeviceBase::readCalibrationOrDefault() {
    return readCalibration();
}

void DeviceBase::flashFactoryCalibration(CalibrationHandler calibrationDataHandler) {
    bool factoryPermissions = false;
    bool protectedPermissions = false;
    getFlashingPermissions(factoryPermissions, protectedPermissions);
    pimpl->logger.debug("Flashing factory calibration. Factory permissions {}, Protected permissions {}", factoryPermissions, protectedPermissions);

    if(!factoryPermissions) {
        throw std::runtime_error("Calling factory API is not allowed in current configuration");
    }

    /* if(!calibrationDataHandler.validateCameraArray()) {
        throw std::runtime_error("Failed to validate the extrinsics connection. Enable debug mode for more information.");
    } */

    bool success;
    std::string errorMsg;
    std::tie(success, errorMsg) = pimpl->rpcCall("storeToEepromFactory", calibrationDataHandler.getEepromData(), factoryPermissions, protectedPermissions)
                                      .as<std::tuple<bool, std::string>>();
    if(!success) {
        throw EepromError(errorMsg);
    }
}

CalibrationHandler DeviceBase::readFactoryCalibration() {
    bool success;
    std::string errorMsg;
    dai::EepromData eepromData;
    std::tie(success, errorMsg, eepromData) = pimpl->rpcCall("readFromEepromFactory").as<std::tuple<bool, std::string, dai::EepromData>>();
    if(!success) {
        throw EepromError(errorMsg);
    }
    return CalibrationHandler(eepromData);
}
CalibrationHandler DeviceBase::readFactoryCalibrationOrDefault() {
    dai::EepromData eepromData{};
    try {
        return readFactoryCalibration();
    } catch(const EepromError&) {
        // ignore - use default
    }
    return CalibrationHandler(eepromData);
}

void DeviceBase::factoryResetCalibration() {
    bool success;
    std::string errorMsg;
    std::tie(success, errorMsg) = pimpl->rpcCall("eepromFactoryReset").as<std::tuple<bool, std::string>>();
    if(!success) {
        throw EepromError(errorMsg);
    }
}

std::vector<std::uint8_t> DeviceBase::readCalibrationRaw() {
    bool success;
    std::string errorMsg;
    std::vector<uint8_t> eepromDataRaw;
    std::tie(success, errorMsg, eepromDataRaw) = pimpl->rpcCall("readFromEepromRaw").as<std::tuple<bool, std::string, std::vector<uint8_t>>>();
    if(!success) {
        throw EepromError(errorMsg);
    }
    return eepromDataRaw;
}

std::vector<std::uint8_t> DeviceBase::readFactoryCalibrationRaw() {
    bool success;
    std::string errorMsg;
    std::vector<uint8_t> eepromDataRaw;
    std::tie(success, errorMsg, eepromDataRaw) = pimpl->rpcCall("readFromEepromFactoryRaw").as<std::tuple<bool, std::string, std::vector<uint8_t>>>();
    if(!success) {
        throw EepromError(errorMsg);
    }
    return eepromDataRaw;
}

void DeviceBase::flashEepromClear() {
    bool factoryPermissions = false;
    bool protectedPermissions = false;
    getFlashingPermissions(factoryPermissions, protectedPermissions);
    pimpl->logger.debug("Clearing User EEPROM contents. Factory permissions {}, Protected permissions {}", factoryPermissions, protectedPermissions);

    if(!protectedPermissions) {
        throw std::runtime_error("Calling EEPROM clear API is not allowed in current configuration");
    }

    bool success;
    std::string errorMsg;
    std::tie(success, errorMsg) = pimpl->rpcCall("eepromClear", protectedPermissions, factoryPermissions).as<std::tuple<bool, std::string>>();
    if(!success) {
        throw EepromError(errorMsg);
    }
}

void DeviceBase::flashFactoryEepromClear() {
    bool factoryPermissions = false;
    bool protectedPermissions = false;
    getFlashingPermissions(factoryPermissions, protectedPermissions);
    pimpl->logger.debug("Clearing User EEPROM contents. Factory permissions {}, Protected permissions {}", factoryPermissions, protectedPermissions);

    if(!protectedPermissions || !factoryPermissions) {
        throw std::runtime_error("Calling factory EEPROM clear API is not allowed in current configuration");
    }

    bool success;
    std::string errorMsg;
    std::tie(success, errorMsg) = pimpl->rpcCall("eepromFactoryClear", protectedPermissions, factoryPermissions).as<std::tuple<bool, std::string>>();
    if(!success) {
        throw EepromError(errorMsg);
    }
}

bool DeviceBase::startPipeline(const Pipeline& pipeline) {
    // first check if pipeline is not already running
    if(isPipelineRunning()) {
        throw std::runtime_error("Pipeline is already running");
    }

    return startPipelineImpl(pipeline);
}

bool DeviceBase::startPipelineImpl(const Pipeline& pipeline) {
    // Serialize the pipeline
    PipelineSchema schema;
    Assets assets;
    std::vector<std::uint8_t> assetStorage;
    pipeline.serialize(schema, assets, assetStorage);

    // if debug or lower
    if(getLogOutputLevel() <= LogLevel::DEBUG) {
        nlohmann::json jSchema = schema;
        pimpl->logger.debug("Schema dump: {}", jSchema.dump());
        nlohmann::json jAssets = assets;
        pimpl->logger.debug("Asset map dump: {}", jAssets.dump());
    }

    // Load pipelineDesc, assets, and asset storage
    logger::trace("Setting pipeline schema.");
    try {
        // pimpl->rpcCall("setPipelineSchema", schema);
        pimpl->rpcCall("setPipelineSchema", schema);
    } catch(const std::exception& e) {
        logger::error("Exception during setting pipeline schema: {}", e.what());
        throw;
        return false;
    }

    // Transfer storage != empty
    if(!assetStorage.empty()) {
        logger::trace("Setting assets.");
        try {
            pimpl->rpcCall("setAssets", assets);
        } catch(const std::exception& e) {
            logger::error("Exception during setting assets: {}", e.what());
            throw;
            return false;
        }

        // Transfer the whole assetStorage in a separate thread
        const std::string streamAssetStorage = "__stream_asset_storage";
        std::thread t1([this, &streamAssetStorage, &assetStorage]() {
            XLinkStream stream(connection, streamAssetStorage, device::XLINK_USB_BUFFER_MAX_SIZE);
            int64_t offset = 0;
            do {
                int64_t toTransfer = std::min(static_cast<int64_t>(device::XLINK_USB_BUFFER_MAX_SIZE), static_cast<int64_t>(assetStorage.size() - offset));
                stream.write(&assetStorage[offset], toTransfer);
                offset += toTransfer;
            } while(offset < static_cast<int64_t>(assetStorage.size()));
        });

        logger::trace("Transfering assets of size {} with no timeout.", assetStorage.size());
        try {
            pimpl->rpcCall(std::chrono::milliseconds(0), "readAssetStorageFromXLink", streamAssetStorage, assetStorage.size());
        } catch(const std::exception& e) {
            if(t1.joinable()) {
                t1.join();
            }
            logger::error("Exception during asset transfer: {}", e.what());
            throw;
            return false;
        }
        t1.join();
    }

    // // print assets on device side for test
    // pimpl->rpcCall("printAssets");

    // Log the pipeline
    logCollection::logPipeline(schema, deviceInfo);
    this->pipelineSchema = schema;  // Save the schema so it can be saved alongside the crashdump

    bool success = false;
    std::string errorMsg;

    // Initialize the device (M8 Fsync slaves need to lock onto the signal first)    
    std::tie(success, errorMsg) = pimpl->rpcCall(std::chrono::seconds(60), "waitForDeviceReady").as<std::tuple<bool, std::string>>();

    if (!success) {
        throw std::runtime_error("Device " + getDeviceId() + " not ready: " + errorMsg);
    }

    // Build and start the pipeline
    std::tie(success, errorMsg) = pimpl->rpcCall("buildPipeline").as<std::tuple<bool, std::string>>();
    if(success) {
        pimpl->rpcCall("startPipeline");
    } else {
        throw std::runtime_error("Device " + getDeviceId() + " error: " + errorMsg);
        return false;
    }

    return true;
}
}  // namespace dai
