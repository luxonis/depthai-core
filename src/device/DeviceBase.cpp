#include "depthai/device/DeviceBase.hpp"

// std
#include <iostream>

// shared
#include "depthai-bootloader-shared/Bootloader.hpp"
#include "depthai-bootloader-shared/XLinkConstants.hpp"
#include "depthai-shared/datatype/RawImgFrame.hpp"
#include "depthai-shared/log/LogConstants.hpp"
#include "depthai-shared/log/LogLevel.hpp"
#include "depthai-shared/log/LogMessage.hpp"
#include "depthai-shared/pipeline/Assets.hpp"
#include "depthai-shared/utility/Serialization.hpp"
#include "depthai-shared/xlink/XLinkConstants.hpp"

// project
#include "DeviceLogger.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "pipeline/Pipeline.hpp"
#include "utility/Environment.hpp"
#include "utility/Initialization.hpp"
#include "utility/PimplImpl.hpp"
#include "utility/Resources.hpp"

// libraries
#include "XLink/XLink.h"
#include "nanorpc/core/client.h"
#include "nanorpc/packer/nlohmann_msgpack.h"
#include "spdlog/details/os.h"
#include "spdlog/fmt/bin_to_hex.h"
#include "spdlog/fmt/chrono.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

namespace dai {

const std::string MAGIC_PROTECTED_FLASHING_VALUE = "235539980";
const std::string MAGIC_FACTORY_FLASHING_VALUE = "413424129";
const std::string MAGIC_FACTORY_PROTECTED_FLASHING_VALUE = "868632271";

// local static function
static void getFlashingPermissions(bool& factoryPermissions, bool& protectedPermissions) {
    auto permissionEnv = utility::getEnv("DEPTHAI_ALLOW_FACTORY_FLASHING");
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

static LogLevel spdlogLevelToLogLevel(spdlog::level::level_enum level, LogLevel defaultValue = LogLevel::OFF) {
    switch(level) {
        case spdlog::level::trace:
            return LogLevel::TRACE;
        case spdlog::level::debug:
            return LogLevel::DEBUG;
        case spdlog::level::info:
            return LogLevel::INFO;
        case spdlog::level::warn:
            return LogLevel::WARN;
        case spdlog::level::err:
            return LogLevel::ERR;
        case spdlog::level::critical:
            return LogLevel::CRITICAL;
        case spdlog::level::off:
            return LogLevel::OFF;
        // Default
        case spdlog::level::n_levels:
        default:
            return defaultValue;
            break;
    }
    // Default
    return defaultValue;
}
static spdlog::level::level_enum logLevelToSpdlogLevel(LogLevel level, spdlog::level::level_enum defaultValue = spdlog::level::off) {
    switch(level) {
        case LogLevel::TRACE:
            return spdlog::level::trace;
        case LogLevel::DEBUG:
            return spdlog::level::debug;
        case LogLevel::INFO:
            return spdlog::level::info;
        case LogLevel::WARN:
            return spdlog::level::warn;
        case LogLevel::ERR:
            return spdlog::level::err;
        case LogLevel::CRITICAL:
            return spdlog::level::critical;
        case LogLevel::OFF:
            return spdlog::level::off;
    }
    // Default
    return defaultValue;
}

constexpr std::chrono::seconds DeviceBase::DEFAULT_SEARCH_TIME;
constexpr float DeviceBase::DEFAULT_SYSTEM_INFORMATION_LOGGING_RATE_HZ;
constexpr UsbSpeed DeviceBase::DEFAULT_USB_SPEED;

std::chrono::milliseconds DeviceBase::getDefaultSearchTime() {
    std::chrono::milliseconds defaultSearchTime = DEFAULT_SEARCH_TIME;
    auto searchTimeStr = utility::getEnv("DEPTHAI_SEARCH_TIMEOUT");

    if(!searchTimeStr.empty()) {
        // Try parsing the string as a number
        try {
            defaultSearchTime = std::chrono::milliseconds{std::stoi(searchTimeStr)};
        } catch(const std::invalid_argument& e) {
            spdlog::warn("DEPTHAI_SEARCH_TIMEOUT value invalid: {}", e.what());
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
    do {
        auto devices = XLinkConnection::getAllConnectedDevices(X_LINK_ANY_STATE, false);
        for(auto searchState : {X_LINK_UNBOOTED, X_LINK_BOOTLOADER, X_LINK_FLASH_BOOTED}) {
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
            spdlog::warn("Insufficient permissions to communicate with {} device with name \"{}\". Make sure udev rules are set",
                         XLinkDeviceStateToStr(invalidDeviceInfo.state),
                         invalidDeviceInfo.name);
        } else {
            // Warn
            spdlog::warn(
                "Skipping {} device with name \"{}\" ({})", XLinkDeviceStateToStr(invalidDeviceInfo.state), invalidDeviceInfo.name, invalidDeviceInfo.mxid);
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
    for(auto searchState : {X_LINK_UNBOOTED, X_LINK_BOOTLOADER, X_LINK_FLASH_BOOTED}) {
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

// First tries to find UNBOOTED device with mxId, then BOOTLOADER device with mxId
std::tuple<bool, DeviceInfo> DeviceBase::getDeviceByMxId(std::string mxId) {
    std::vector<DeviceInfo> availableDevices;
    auto states = {X_LINK_UNBOOTED, X_LINK_BOOTLOADER};
    bool found;
    DeviceInfo dev;
    for(const auto& state : states) {
        std::tie(found, dev) = XLinkConnection::getDeviceByMxId(mxId, state);
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
    DeviceLogger logger{"", stdoutColorSink};

    // RPC
    std::mutex rpcMutex;
    std::shared_ptr<XLinkStream> rpcStream;
    std::unique_ptr<nanorpc::core::client<nanorpc::packer::nlohmann_msgpack>> rpcClient;

    void setLogLevel(LogLevel level);
    LogLevel getLogLevel();
    void setPattern(const std::string& pattern);
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
    if(!found) throw std::runtime_error("No available devices");
}

DeviceBase::DeviceBase(OpenVINO::Version version, const DeviceInfo& devInfo) : DeviceBase(version, devInfo, DeviceBase::DEFAULT_USB_SPEED) {}

DeviceBase::DeviceBase(OpenVINO::Version version, const DeviceInfo& devInfo, UsbSpeed maxUsbSpeed) : deviceInfo(devInfo) {
    init(version, maxUsbSpeed, "");
}

DeviceBase::DeviceBase(OpenVINO::Version version, const DeviceInfo& devInfo, const dai::Path& pathToCmd) : deviceInfo(devInfo) {
    init(version, false, pathToCmd);
}

DeviceBase::DeviceBase() : DeviceBase(OpenVINO::DEFAULT_VERSION) {}

DeviceBase::DeviceBase(OpenVINO::Version version) {
    tryGetDevice();
    init(version, false, "");
}

DeviceBase::DeviceBase(OpenVINO::Version version, const dai::Path& pathToCmd) {
    tryGetDevice();
    init(version, false, pathToCmd);
}

DeviceBase::DeviceBase(OpenVINO::Version version, UsbSpeed maxUsbSpeed) {
    tryGetDevice();
    init(version, maxUsbSpeed, "");
}

DeviceBase::DeviceBase(const Pipeline& pipeline) : DeviceBase(pipeline.getOpenVINOVersion()) {
    tryStartPipeline(pipeline);
}

DeviceBase::DeviceBase(const Pipeline& pipeline, UsbSpeed maxUsbSpeed) : DeviceBase(pipeline.getOpenVINOVersion(), maxUsbSpeed) {
    tryStartPipeline(pipeline);
}

DeviceBase::DeviceBase(const Pipeline& pipeline, const dai::Path& pathToCmd) : DeviceBase(pipeline.getOpenVINOVersion(), pathToCmd) {
    tryStartPipeline(pipeline);
}

DeviceBase::DeviceBase(const Pipeline& pipeline, const DeviceInfo& devInfo)
    : DeviceBase(pipeline.getOpenVINOVersion(), devInfo, DeviceBase::DEFAULT_USB_SPEED) {}

DeviceBase::DeviceBase(const Pipeline& pipeline, const DeviceInfo& devInfo, UsbSpeed maxUsbSpeed)
    : DeviceBase(pipeline.getOpenVINOVersion(), devInfo, maxUsbSpeed) {
    tryStartPipeline(pipeline);
}

DeviceBase::DeviceBase(const Pipeline& pipeline, const DeviceInfo& devInfo, const dai::Path& pathToCmd)
    : DeviceBase(pipeline.getOpenVINOVersion(), devInfo, pathToCmd) {
    tryStartPipeline(pipeline);
}

DeviceBase::DeviceBase(Config config) {
    tryGetDevice();
    init2(config, {}, {});
}

DeviceBase::DeviceBase(Config config, const DeviceInfo& devInfo) : deviceInfo(devInfo) {
    init2(config, {}, {});
}

void DeviceBase::close() {
    std::unique_lock<std::mutex> lock(closedMtx);
    if(!closed) {
        closeImpl();
        closed = true;
    }
}

void DeviceBase::closeImpl() {
    using namespace std::chrono;
    auto t1 = steady_clock::now();
    spdlog::debug("Device about to be closed...");

    // Close connection first; causes Xlink internal calls to unblock semaphore waits and
    // return error codes, which then allows queues to unblock
    // always manage ownership because other threads (e.g. watchdog) are running and need to
    // keep the shared_ptr valid (even if closed). Otherwise leads to using null pointers,
    // invalid memory, etc. which hard crashes main app
    connection->close();

    // Stop various threads
    watchdogRunning = false;
    timesyncRunning = false;
    loggingRunning = false;

    // Stop watchdog first (this resets and waits for link to fall down)
    if(watchdogThread.joinable()) watchdogThread.join();
    // Then stop timesync
    if(timesyncThread.joinable()) timesyncThread.join();
    // And at the end stop logging thread
    if(loggingThread.joinable()) loggingThread.join();
    // At the end stop the monitor thread
    if(monitorThread.joinable()) monitorThread.join();

    // Close rpcStream
    pimpl->rpcStream = nullptr;

    spdlog::debug("Device closed, {}", duration_cast<milliseconds>(steady_clock::now() - t1).count());
}

bool DeviceBase::isClosed() const {
    std::unique_lock<std::mutex> lock(closedMtx);
    return closed || !watchdogRunning;
}

void DeviceBase::checkClosed() const {
    if(isClosed()) throw std::invalid_argument("Device already closed or disconnected");
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

void DeviceBase::init(OpenVINO::Version version, bool usb2Mode, const dai::Path& pathToMvcmd) {
    Config cfg;
    // Specify usb speed
    cfg.board.usb.maxSpeed = usb2Mode ? UsbSpeed::HIGH : DeviceBase::DEFAULT_USB_SPEED;
    // Specify the OpenVINO version
    cfg.version = version;
    init2(cfg, pathToMvcmd, {});
}
void DeviceBase::init(const Pipeline& pipeline, bool usb2Mode, const dai::Path& pathToMvcmd) {
    Config cfg = pipeline.getDeviceConfig();
    // Modify usb speed
    cfg.board.usb.maxSpeed = usb2Mode ? UsbSpeed::HIGH : DeviceBase::DEFAULT_USB_SPEED;
    init2(cfg, pathToMvcmd, pipeline);
}
void DeviceBase::init(OpenVINO::Version version, UsbSpeed maxUsbSpeed, const dai::Path& pathToMvcmd) {
    Config cfg;
    // Specify usb speed
    cfg.board.usb.maxSpeed = maxUsbSpeed;
    // Specify the OpenVINO version
    cfg.version = version;
    init2(cfg, pathToMvcmd, {});
}
void DeviceBase::init(const Pipeline& pipeline, UsbSpeed maxUsbSpeed, const dai::Path& pathToMvcmd) {
    Config cfg = pipeline.getDeviceConfig();
    // Modify usb speed
    cfg.board.usb.maxSpeed = maxUsbSpeed;
    init2(cfg, pathToMvcmd, pipeline);
}

void DeviceBase::init2(Config cfg, const dai::Path& pathToMvcmd, tl::optional<const Pipeline&> pipeline) {
    // Initalize depthai library if not already
    initialize();

    // Specify cfg
    config = cfg;

    // If deviceInfo isn't fully specified (eg ANY_STATE, etc...), try finding it first
    if(deviceInfo.state == X_LINK_ANY_STATE || deviceInfo.protocol == X_LINK_ANY_PROTOCOL) {
        deviceDesc_t foundDesc;
        auto ret = XLinkFindFirstSuitableDevice(deviceInfo.getXLinkDeviceDesc(), &foundDesc);
        if(ret == X_LINK_SUCCESS) {
            deviceInfo = DeviceInfo(foundDesc);
            spdlog::debug("Found an actual device by given DeviceInfo: {}", deviceInfo.toString());
        } else {
            deviceInfo.state = X_LINK_ANY_STATE;
            spdlog::debug("Searched, but no actual device found by given DeviceInfo");
        }
    }

    if(pipeline) {
        spdlog::debug("Device - pipeline serialized, OpenVINO version: {}", OpenVINO::getVersionName(config.version));
    } else {
        spdlog::debug("Device - OpenVINO version: {}", OpenVINO::getVersionName(config.version));
    }

    // Set logging pattern of device (device id + shared pattern)
    pimpl->setPattern(fmt::format("[{}] [{}] {}", deviceInfo.mxid, deviceInfo.name, LOG_DEFAULT_PATTERN));

    // Check if WD env var is set
    std::chrono::milliseconds watchdogTimeout = device::XLINK_WATCHDOG_TIMEOUT;
    auto watchdogMsStr = utility::getEnv("DEPTHAI_WATCHDOG");
    if(!watchdogMsStr.empty()) {
        // Try parsing the string as a number
        try {
            std::chrono::milliseconds watchdog{std::stoi(watchdogMsStr)};
            config.board.watchdogTimeoutMs = static_cast<uint32_t>(watchdog.count());
            watchdogTimeout = watchdog;
            if(watchdogTimeout.count() == 0) {
                spdlog::warn("Watchdog disabled! In case of unclean exit, the device needs reset or power-cycle for next run", watchdogTimeout);
            } else {
                spdlog::warn("Using a custom watchdog value of {}", watchdogTimeout);
            }
        } catch(const std::invalid_argument& e) {
            spdlog::warn("DEPTHAI_WATCHDOG value invalid: {}", e.what());
        }
    }

    auto watchdogInitMsStr = utility::getEnv("DEPTHAI_WATCHDOG_INITIAL_DELAY");
    if(!watchdogInitMsStr.empty()) {
        // Try parsing the string as a number
        try {
            std::chrono::milliseconds watchdog{std::stoi(watchdogInitMsStr)};
            config.board.watchdogInitialDelayMs = static_cast<uint32_t>(watchdog.count());
            spdlog::warn("Watchdog initial delay set to {}", watchdog);
        } catch(const std::invalid_argument& e) {
            spdlog::warn("DEPTHAI_WATCHDOG_INITIAL_DELAY value invalid: {}", e.what());
        }
    }

    // Get embedded mvcmd or external with applied config
    if(spdlog::get_level() == spdlog::level::debug) {
        nlohmann::json jBoardConfig = config.board;
        spdlog::debug("Device - BoardConfig: {} \nlibnop:{}", jBoardConfig.dump(), spdlog::to_hex(utility::serialize(config.board)));
    }
    std::vector<std::uint8_t> fwWithConfig = Resources::getInstance().getDeviceFirmware(config, pathToMvcmd);

    // Init device (if bootloader, handle correctly - issue USB boot command)
    if(deviceInfo.state == X_LINK_UNBOOTED) {
        // Unbooted device found, boot and connect with XLinkConnection constructor
        connection = std::make_shared<XLinkConnection>(deviceInfo, fwWithConfig);
    } else if(deviceInfo.state == X_LINK_BOOTLOADER || deviceInfo.state == X_LINK_FLASH_BOOTED) {
        // Scope so DeviceBootloader is disconnected
        {
            DeviceBootloader bl(deviceInfo);
            auto version = bl.getVersion();

            // If version is >= 0.0.12 then boot directly, otherwise jump to USB ROM bootloader
            // Check if version is recent enough for this operation
            if(version >= DeviceBootloader::Version(0, 0, 12)) {
                using namespace std::chrono;
                // Boot the given FW
                auto t1 = steady_clock::now();
                bl.bootMemory(fwWithConfig);
                auto t2 = steady_clock::now();
                spdlog::debug("Booting FW with Bootloader. Version {}, Time taken: {}", version.toString(), duration_cast<milliseconds>(t2 - t1));

                // After that the state will be BOOTED
                deviceInfo.state = X_LINK_BOOTED;
            } else {
                // Boot into USB ROM BOOTLOADER
                bl.bootUsbRomBootloader();
                spdlog::debug("Booting FW by jumping to USB ROM Bootloader first. Bootloader Version {}", version.toString());

                // After that the state will be UNBOOTED
                deviceInfo.state = X_LINK_UNBOOTED;
            }
        }

        // Boot and connect with XLinkConnection constructor
        connection = std::make_shared<XLinkConnection>(deviceInfo, fwWithConfig);

    } else if(deviceInfo.state == X_LINK_BOOTED) {
        // Connect without booting
        connection = std::make_shared<XLinkConnection>(deviceInfo, fwWithConfig);
    } else {
        throw std::runtime_error("Cannot find any device with given deviceInfo");
    }

    deviceInfo.state = X_LINK_BOOTED;

    // prepare rpc for both attached and host controlled mode
    pimpl->rpcStream = std::make_shared<XLinkStream>(connection, device::XLINK_CHANNEL_MAIN_RPC, device::XLINK_USB_BUFFER_MAX_SIZE);
    auto rpcStream = pimpl->rpcStream;

    pimpl->rpcClient = std::make_unique<nanorpc::core::client<nanorpc::packer::nlohmann_msgpack>>([this, rpcStream](nanorpc::core::type::buffer request) {
        // Lock for time of the RPC call, to not mix the responses between calling threads.
        // Note: might cause issues on Windows on incorrect shutdown. To be investigated
        std::unique_lock<std::mutex> lock(pimpl->rpcMutex);

        // Log the request data
        if(spdlog::get_level() == spdlog::level::trace) {
            spdlog::trace("RPC: {}", nlohmann::json::from_msgpack(request).dump());
        }

        // Send request to device
        rpcStream->write(std::move(request));

        // Receive response back
        // Send to nanorpc to parse
        return rpcStream->read();
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
                    // Ping with a period half of that of the watchdog timeout
                    std::this_thread::sleep_for(watchdogTimeout / 2);
                }
            } catch(const std::exception& ex) {
                // ignore
                spdlog::debug("Watchdog thread exception caught: {}", ex.what());
            }

            // Watchdog ended. Useful for checking disconnects
            watchdogRunning = false;
        });

        // Start monitor thread for host - makes sure that device is responding to pings, otherwise it disconnects
        monitorThread = std::thread([this, watchdogTimeout]() {
            while(watchdogRunning) {
                // Ping with a period half of that of the watchdog timeout
                std::this_thread::sleep_for(watchdogTimeout);
                // Check if wd was pinged in the specified watchdogTimeout time.
                decltype(lastWatchdogPingTime) prevPingTime;
                {
                    std::unique_lock<std::mutex> lock(lastWatchdogPingTimeMtx);
                    prevPingTime = lastWatchdogPingTime;
                }
                // Recheck if watchdogRunning wasn't already closed
                if(watchdogRunning && std::chrono::steady_clock::now() - prevPingTime > watchdogTimeout) {
                    spdlog::warn("Monitor thread (device: {} [{}]) - ping was missed, closing the device connection", deviceInfo.mxid, deviceInfo.name);
                    // ping was missed, reset the device
                    watchdogRunning = false;
                    // close the underlying connection
                    connection->close();
                }
            }
        });

    } else {
        // Still set watchdogRunning explictitly
        // as it indicates device not being closed
        watchdogRunning = true;
    }

    // prepare timesync thread, which will keep device synchronized
    timesyncThread = std::thread([this]() {
        using namespace std::chrono;

        try {
            XLinkStream stream(connection, device::XLINK_CHANNEL_TIMESYNC, 128);
            Timestamp timestamp = {};
            while(timesyncRunning) {
                // Block
                stream.read();

                // Timestamp
                auto d = std::chrono::steady_clock::now().time_since_epoch();
                timestamp.sec = duration_cast<seconds>(d).count();
                timestamp.nsec = duration_cast<nanoseconds>(d).count() % 1000000000;

                // Write timestamp back
                stream.write(&timestamp, sizeof(timestamp));
            }
        } catch(const std::exception& ex) {
            // ignore
            spdlog::debug("Timesync thread exception caught: {}", ex.what());
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

                    spdlog::trace("Log vector decoded, size: {}", messages.size());

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
                    spdlog::error("Exception while parsing or calling callbacks for log message from device: {}", ex.what());
                }
            }
        } catch(const std::exception& ex) {
            // ignore exception from logging
            spdlog::debug("Log thread exception caught: {}", ex.what());
        }

        loggingRunning = false;
    });

    // Below can throw - make sure to gracefully exit threads
    try {
        // Set logging level (if DEPTHAI_LEVEL lower than warning, then device is configured accordingly as well)
        if(spdlog::get_level() < spdlog::level::warn) {
            auto level = spdlogLevelToLogLevel(spdlog::get_level());
            setLogLevel(level);
            setLogOutputLevel(level);
        } else {
            setLogLevel(LogLevel::WARN);
            setLogOutputLevel(LogLevel::WARN);
        }

        // Sets system inforation logging rate. By default 1s
        setSystemInformationLoggingRate(DEFAULT_SYSTEM_INFORMATION_LOGGING_RATE_HZ);
    } catch(const std::exception&) {
        // close device (cleanup)
        close();
        // Rethrow original exception
        throw;
    }
}

std::string DeviceBase::getMxId() {
    checkClosed();

    return pimpl->rpcClient->call("getMxId").as<std::string>();
}

std::vector<CameraBoardSocket> DeviceBase::getConnectedCameras() {
    checkClosed();

    return pimpl->rpcClient->call("getConnectedCameras").as<std::vector<CameraBoardSocket>>();
}

std::unordered_map<CameraBoardSocket, std::string> DeviceBase::getCameraSensorNames() {
    checkClosed();

    return pimpl->rpcClient->call("getCameraSensorNames").as<std::unordered_map<CameraBoardSocket, std::string>>();
}

// Convenience functions for querying current system information
MemoryInfo DeviceBase::getDdrMemoryUsage() {
    checkClosed();

    return pimpl->rpcClient->call("getDdrUsage").as<MemoryInfo>();
}

MemoryInfo DeviceBase::getCmxMemoryUsage() {
    checkClosed();

    return pimpl->rpcClient->call("getCmxUsage").as<MemoryInfo>();
}

MemoryInfo DeviceBase::getLeonCssHeapUsage() {
    checkClosed();

    return pimpl->rpcClient->call("getLeonCssHeapUsage").as<MemoryInfo>();
}

MemoryInfo DeviceBase::getLeonMssHeapUsage() {
    checkClosed();

    return pimpl->rpcClient->call("getLeonMssHeapUsage").as<MemoryInfo>();
}

ChipTemperature DeviceBase::getChipTemperature() {
    checkClosed();

    return pimpl->rpcClient->call("getChipTemperature").as<ChipTemperature>();
}

CpuUsage DeviceBase::getLeonCssCpuUsage() {
    checkClosed();

    return pimpl->rpcClient->call("getLeonCssCpuUsage").as<CpuUsage>();
}

CpuUsage DeviceBase::getLeonMssCpuUsage() {
    checkClosed();

    return pimpl->rpcClient->call("getLeonMssCpuUsage").as<CpuUsage>();
}

UsbSpeed DeviceBase::getUsbSpeed() {
    checkClosed();

    return pimpl->rpcClient->call("getUsbSpeed").as<UsbSpeed>();
}

bool DeviceBase::isPipelineRunning() {
    checkClosed();

    return pimpl->rpcClient->call("isPipelineRunning").as<bool>();
}

void DeviceBase::setLogLevel(LogLevel level) {
    checkClosed();

    pimpl->rpcClient->call("setLogLevel", level);
}

LogLevel DeviceBase::getLogLevel() {
    checkClosed();

    return pimpl->rpcClient->call("getLogLevel").as<LogLevel>();
}

void DeviceBase::setXLinkChunkSize(int sizeBytes) {
    checkClosed();

    pimpl->rpcClient->call("setXLinkChunkSize", sizeBytes);
}

int DeviceBase::getXLinkChunkSize() {
    checkClosed();

    return pimpl->rpcClient->call("getXLinkChunkSize").as<int>();
}

DeviceInfo DeviceBase::getDeviceInfo() const {
    return deviceInfo;
}

void DeviceBase::setLogOutputLevel(LogLevel level) {
    checkClosed();

    pimpl->setLogLevel(level);
}

LogLevel DeviceBase::getLogOutputLevel() {
    checkClosed();

    return pimpl->getLogLevel();
}

bool DeviceBase::setIrLaserDotProjectorBrightness(float mA, int mask) {
    checkClosed();

    return pimpl->rpcClient->call("setIrLaserDotProjectorBrightness", mA, mask);
}

bool DeviceBase::setIrFloodLightBrightness(float mA, int mask) {
    checkClosed();

    return pimpl->rpcClient->call("setIrFloodLightBrightness", mA, mask);
}

std::vector<std::tuple<std::string, int, int>> DeviceBase::getIrDrivers() {
    checkClosed();

    return pimpl->rpcClient->call("getIrDrivers");
}

int DeviceBase::addLogCallback(std::function<void(LogMessage)> callback) {
    checkClosed();

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
    checkClosed();

    // Lock first
    std::unique_lock<std::mutex> l(logCallbackMapMtx);

    // If callback with id 'callbackId' doesn't exists, return false
    if(logCallbackMap.count(callbackId) == 0) return false;

    // Otherwise erase and return true
    logCallbackMap.erase(callbackId);
    return true;
}

void DeviceBase::setSystemInformationLoggingRate(float rateHz) {
    checkClosed();

    pimpl->rpcClient->call("setSystemInformationLoggingRate", rateHz);
}

float DeviceBase::getSystemInformationLoggingRate() {
    checkClosed();

    return pimpl->rpcClient->call("getSystemInformationLoggingrate").as<float>();
}

bool DeviceBase::isEepromAvailable() {
    return pimpl->rpcClient->call("isEepromAvailable").as<bool>();
}

bool DeviceBase::flashCalibration(CalibrationHandler calibrationDataHandler) {
    try {
        flashCalibration2(calibrationDataHandler);
    } catch(const std::exception& ex) {
        return false;
    }
    return true;
}

void DeviceBase::flashCalibration2(CalibrationHandler calibrationDataHandler) {
    bool factoryPermissions = false;
    bool protectedPermissions = false;
    getFlashingPermissions(factoryPermissions, protectedPermissions);
    spdlog::debug("Flashing calibration. Factory permissions {}, Protected permissions {}", factoryPermissions, protectedPermissions);

    if(!calibrationDataHandler.validateCameraArray()) {
        throw std::runtime_error("Failed to validate the extrinsics connection. Enable debug mode for more information.");
    }

    bool success;
    std::string errorMsg;
    std::tie(success, errorMsg) = pimpl->rpcClient->call("storeToEeprom", calibrationDataHandler.getEepromData(), factoryPermissions, protectedPermissions)
                                      .as<std::tuple<bool, std::string>>();

    if(!success) {
        throw std::runtime_error(errorMsg);
    }
}

CalibrationHandler DeviceBase::readCalibration() {
    dai::EepromData eepromData{};
    try {
        return readCalibration2();
    } catch(const std::exception& ex) {
        // ignore - use default
    }
    return CalibrationHandler(eepromData);
}
CalibrationHandler DeviceBase::readCalibration2() {
    bool success;
    std::string errorMsg;
    dai::EepromData eepromData;
    std::tie(success, errorMsg, eepromData) = pimpl->rpcClient->call("readFromEeprom").as<std::tuple<bool, std::string, dai::EepromData>>();
    if(!success) {
        throw std::runtime_error(errorMsg);
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
    spdlog::debug("Flashing factory calibration. Factory permissions {}, Protected permissions {}", factoryPermissions, protectedPermissions);

    if(!factoryPermissions) {
        throw std::runtime_error("Calling factory API is not allowed in current configuration");
    }

    if(!calibrationDataHandler.validateCameraArray()) {
        throw std::runtime_error("Failed to validate the extrinsics connection. Enable debug mode for more information.");
    }

    bool success;
    std::string errorMsg;
    std::tie(success, errorMsg) =
        pimpl->rpcClient->call("storeToEepromFactory", calibrationDataHandler.getEepromData(), factoryPermissions, protectedPermissions)
            .as<std::tuple<bool, std::string>>();
    if(!success) {
        throw std::runtime_error(errorMsg);
    }
}

CalibrationHandler DeviceBase::readFactoryCalibration() {
    bool success;
    std::string errorMsg;
    dai::EepromData eepromData;
    std::tie(success, errorMsg, eepromData) = pimpl->rpcClient->call("readFromEepromFactory").as<std::tuple<bool, std::string, dai::EepromData>>();
    if(!success) {
        throw std::runtime_error(errorMsg);
    }
    return CalibrationHandler(eepromData);
}
CalibrationHandler DeviceBase::readFactoryCalibrationOrDefault() {
    dai::EepromData eepromData{};
    try {
        return readFactoryCalibration();
    } catch(const std::exception& ex) {
        // ignore - use default
    }
    return CalibrationHandler(eepromData);
}

void DeviceBase::factoryResetCalibration() {
    bool success;
    std::string errorMsg;
    std::tie(success, errorMsg) = pimpl->rpcClient->call("eepromFactoryReset").as<std::tuple<bool, std::string>>();
    if(!success) {
        throw std::runtime_error(errorMsg);
    }
}

std::vector<std::uint8_t> DeviceBase::readCalibrationRaw() {
    bool success;
    std::string errorMsg;
    std::vector<uint8_t> eepromDataRaw;
    std::tie(success, errorMsg, eepromDataRaw) = pimpl->rpcClient->call("readFromEepromRaw").as<std::tuple<bool, std::string, std::vector<uint8_t>>>();
    if(!success) {
        throw std::runtime_error(errorMsg);
    }
    return eepromDataRaw;
}

std::vector<std::uint8_t> DeviceBase::readFactoryCalibrationRaw() {
    bool success;
    std::string errorMsg;
    std::vector<uint8_t> eepromDataRaw;
    std::tie(success, errorMsg, eepromDataRaw) = pimpl->rpcClient->call("readFromEepromFactoryRaw").as<std::tuple<bool, std::string, std::vector<uint8_t>>>();
    if(!success) {
        throw std::runtime_error(errorMsg);
    }
    return eepromDataRaw;
}

bool DeviceBase::startPipeline() {
    // Deprecated
    return true;
}

bool DeviceBase::startPipeline(const Pipeline& pipeline) {
    // first check if pipeline is not already running
    if(isPipelineRunning()) {
        throw std::runtime_error("Pipeline is already running");
    }

    return startPipelineImpl(pipeline);
}

bool DeviceBase::startPipelineImpl(const Pipeline& pipeline) {
    // Check openvino version
    if(!pipeline.isOpenVINOVersionCompatible(config.version)) {
        throw std::runtime_error("Device booted with different OpenVINO version that pipeline requires");
    }

    // Serialize the pipeline
    PipelineSchema schema;
    Assets assets;
    std::vector<std::uint8_t> assetStorage;
    pipeline.serialize(schema, assets, assetStorage);

    // if debug
    if(spdlog::get_level() == spdlog::level::debug) {
        nlohmann::json jSchema = schema;
        spdlog::debug("Schema dump: {}", jSchema.dump());
        nlohmann::json jAssets = assets;
        spdlog::debug("Asset map dump: {}", jAssets.dump());
    }

    // Load pipelineDesc, assets, and asset storage
    pimpl->rpcClient->call("setPipelineSchema", schema);

    // Transfer storage != empty
    if(!assetStorage.empty()) {
        pimpl->rpcClient->call("setAssets", assets);

        // allocate, returns a pointer to memory on device side
        auto memHandle = pimpl->rpcClient->call("memAlloc", static_cast<std::uint32_t>(assetStorage.size())).as<uint32_t>();

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

        // Open a channel to transfer AssetStorage
        pimpl->rpcClient->call("readFromXLink", streamAssetStorage, memHandle, assetStorage.size());
        t1.join();

        // After asset storage is transfers, set the asset storage
        pimpl->rpcClient->call("setAssetStorage", memHandle, assetStorage.size());
    }

    // print assets on device side for test
    pimpl->rpcClient->call("printAssets");

    // Build and start the pipeline
    bool success = false;
    std::string errorMsg;
    std::tie(success, errorMsg) = pimpl->rpcClient->call("buildPipeline").as<std::tuple<bool, std::string>>();
    if(success) {
        pimpl->rpcClient->call("startPipeline");
    } else {
        throw std::runtime_error(errorMsg);
        return false;
    }

    return true;
}
}  // namespace dai
