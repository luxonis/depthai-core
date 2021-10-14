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
#include "depthai-shared/xlink/XLinkConstants.hpp"

// project
#include "DeviceLogger.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "pipeline/Pipeline.hpp"
#include "utility/Initialization.hpp"
#include "utility/PimplImpl.hpp"
#include "utility/Resources.hpp"

// libraries
#include "XLink/XLink.h"
#include "nanorpc/core/client.h"
#include "nanorpc/packer/nlohmann_msgpack.h"
#include "spdlog/details/os.h"
#include "spdlog/fmt/chrono.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

namespace dai {

// local static function
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

// Common explicit instantiation, to remove the need to define in header
template std::tuple<bool, DeviceInfo> DeviceBase::getAnyAvailableDevice(std::chrono::nanoseconds);
template std::tuple<bool, DeviceInfo> DeviceBase::getAnyAvailableDevice(std::chrono::microseconds);
template std::tuple<bool, DeviceInfo> DeviceBase::getAnyAvailableDevice(std::chrono::milliseconds);
template std::tuple<bool, DeviceInfo> DeviceBase::getAnyAvailableDevice(std::chrono::seconds);
constexpr std::chrono::seconds DeviceBase::DEFAULT_SEARCH_TIME;
constexpr float DeviceBase::DEFAULT_SYSTEM_INFORMATION_LOGGING_RATE_HZ;
constexpr UsbSpeed DeviceBase::DEFAULT_USB_SPEED;

template <typename Rep, typename Period>
std::tuple<bool, DeviceInfo> DeviceBase::getAnyAvailableDevice(std::chrono::duration<Rep, Period> timeout) {
    using namespace std::chrono;
    constexpr auto POOL_SLEEP_TIME = milliseconds(100);

    // First looks for UNBOOTED, then BOOTLOADER, for 'timeout' time
    auto searchStartTime = steady_clock::now();
    bool found = false;
    DeviceInfo deviceInfo;
    do {
        for(auto searchState : {X_LINK_UNBOOTED, X_LINK_BOOTLOADER, X_LINK_FLASH_BOOTED}) {
            std::tie(found, deviceInfo) = XLinkConnection::getFirstDevice(searchState);
            if(found) break;
        }
        if(found) break;

        // If 'timeout' < 'POOL_SLEEP_TIME', use 'timeout' as sleep time and then break
        if(timeout < POOL_SLEEP_TIME) {
            // sleep for 'timeout'
            std::this_thread::sleep_for(timeout);
            break;
        } else {
            std::this_thread::sleep_for(POOL_SLEEP_TIME);  // default pool rate
        }
    } while(steady_clock::now() - searchStartTime < timeout);

    // If none were found, try BOOTED
    if(!found) std::tie(found, deviceInfo) = XLinkConnection::getFirstDevice(X_LINK_BOOTED);

    return {found, deviceInfo};
}

// Default overload ('DEFAULT_SEARCH_TIME' timeout)
std::tuple<bool, DeviceInfo> DeviceBase::getAnyAvailableDevice() {
    return getAnyAvailableDevice(DEFAULT_SEARCH_TIME);
}

// static api

// First tries to find UNBOOTED device, then BOOTLOADER device
std::tuple<bool, DeviceInfo> DeviceBase::getFirstAvailableDevice() {
    bool found;
    DeviceInfo dev;
    std::tie(found, dev) = XLinkConnection::getFirstDevice(X_LINK_UNBOOTED);
    if(!found) {
        std::tie(found, dev) = XLinkConnection::getFirstDevice(X_LINK_BOOTLOADER);
    }
    if(!found) {
        std::tie(found, dev) = XLinkConnection::getFirstDevice(X_LINK_FLASH_BOOTED);
    }
    return {found, dev};
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
    std::unique_ptr<XLinkStream> rpcStream;
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

DeviceBase::DeviceBase(OpenVINO::Version version, const DeviceInfo& devInfo) : DeviceBase(version, devInfo, false) {}

DeviceBase::DeviceBase(OpenVINO::Version version, const DeviceInfo& devInfo, bool usb2Mode) : deviceInfo(devInfo) {
    init(version, usb2Mode, "");
}

DeviceBase::DeviceBase(OpenVINO::Version version, const DeviceInfo& devInfo, UsbSpeed maxUsbSpeed) : deviceInfo(devInfo) {
    init(version, maxUsbSpeed, "");
}

DeviceBase::DeviceBase(OpenVINO::Version version, const DeviceInfo& devInfo, const char* pathToCmd) : deviceInfo(devInfo) {
    init(version, false, std::string(pathToCmd));
}

DeviceBase::DeviceBase(OpenVINO::Version version, const DeviceInfo& devInfo, const std::string& pathToCmd) : deviceInfo(devInfo) {
    init(version, false, pathToCmd);
}

DeviceBase::DeviceBase() : DeviceBase(OpenVINO::DEFAULT_VERSION) {}

DeviceBase::DeviceBase(OpenVINO::Version version) {
    // Searches for any available device for 'default' timeout
    bool found = false;
    std::tie(found, deviceInfo) = getAnyAvailableDevice();

    // If no device found, throw
    if(!found) throw std::runtime_error("No available devices");
    init(version, false, "");
}

DeviceBase::DeviceBase(OpenVINO::Version version, const char* pathToCmd) {
    // Searches for any available device for 'default' timeout

    bool found = false;
    std::tie(found, deviceInfo) = getAnyAvailableDevice();

    // If no device found, throw
    if(!found) throw std::runtime_error("No available devices");
    init(version, false, std::string(pathToCmd));
}

DeviceBase::DeviceBase(OpenVINO::Version version, const std::string& pathToCmd) {
    // Searches for any available device for 'default' timeout
    bool found = false;
    std::tie(found, deviceInfo) = getAnyAvailableDevice();

    // If no device found, throw
    if(!found) throw std::runtime_error("No available devices");
    init(version, false, pathToCmd);
}

DeviceBase::DeviceBase(OpenVINO::Version version, bool usb2Mode) {
    // Searches for any available device for 'default' timeout
    bool found = false;
    std::tie(found, deviceInfo) = getAnyAvailableDevice();

    // If no device found, throw
    if(!found) throw std::runtime_error("No available devices");
    init(version, usb2Mode, "");
}

DeviceBase::DeviceBase(OpenVINO::Version version, UsbSpeed maxUsbSpeed) {
    // Searches for any available device for 'default' timeout
    bool found = false;
    std::tie(found, deviceInfo) = getAnyAvailableDevice();

    // If no device found, throw
    if(!found) throw std::runtime_error("No available devices");
    init(version, maxUsbSpeed, "");
}

DeviceBase::DeviceBase(const Pipeline& pipeline) : DeviceBase(pipeline.getOpenVINOVersion()) {
    tryStartPipeline(pipeline);
}

DeviceBase::DeviceBase(const Pipeline& pipeline, bool usb2Mode) : DeviceBase(pipeline.getOpenVINOVersion(), usb2Mode) {
    tryStartPipeline(pipeline);
}

DeviceBase::DeviceBase(const Pipeline& pipeline, UsbSpeed maxUsbSpeed) : DeviceBase(pipeline.getOpenVINOVersion(), maxUsbSpeed) {
    tryStartPipeline(pipeline);
}

DeviceBase::DeviceBase(const Pipeline& pipeline, const char* pathToCmd) : DeviceBase(pipeline.getOpenVINOVersion(), pathToCmd) {
    tryStartPipeline(pipeline);
}

DeviceBase::DeviceBase(const Pipeline& pipeline, const std::string& pathToCmd) : DeviceBase(pipeline.getOpenVINOVersion(), pathToCmd) {
    tryStartPipeline(pipeline);
}

DeviceBase::DeviceBase(const Pipeline& pipeline, const DeviceInfo& devInfo) : DeviceBase(pipeline.getOpenVINOVersion(), devInfo, false) {}

DeviceBase::DeviceBase(const Pipeline& pipeline, const DeviceInfo& devInfo, bool usb2Mode) : DeviceBase(pipeline.getOpenVINOVersion(), devInfo, usb2Mode) {
    tryStartPipeline(pipeline);
}

DeviceBase::DeviceBase(const Pipeline& pipeline, const DeviceInfo& devInfo, UsbSpeed maxUsbSpeed)
    : DeviceBase(pipeline.getOpenVINOVersion(), devInfo, maxUsbSpeed) {
    tryStartPipeline(pipeline);
}

DeviceBase::DeviceBase(const Pipeline& pipeline, const DeviceInfo& devInfo, const char* pathToCmd)
    : DeviceBase(pipeline.getOpenVINOVersion(), devInfo, pathToCmd) {
    tryStartPipeline(pipeline);
}

DeviceBase::DeviceBase(const Pipeline& pipeline, const DeviceInfo& devInfo, const std::string& pathToCmd)
    : DeviceBase(pipeline.getOpenVINOVersion(), devInfo, pathToCmd) {
    tryStartPipeline(pipeline);
}

DeviceBase::DeviceBase(Config config) {
    // Searches for any available device for 'default' timeout
    bool found = false;
    std::tie(found, deviceInfo) = getAnyAvailableDevice();

    // If no device found, throw
    if(!found) throw std::runtime_error("No available devices");
    init2(config, {}, {});
}

DeviceBase::DeviceBase(Config config, const DeviceInfo& devInfo) : deviceInfo(devInfo) {
    init2(config, {}, {});
}

void DeviceBase::close() {
    // Only allow to close once
    if(closed.exchange(true)) return;

    closeImpl();
}

void DeviceBase::closeImpl() {
    using namespace std::chrono;
    auto t1 = steady_clock::now();
    spdlog::debug("Device about to be closed...");

    // Close connection first
    // Resets device as well and queues unblock
    connection->close();
    connection = nullptr;

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

    // Close rpcStream
    pimpl->rpcStream = nullptr;

    spdlog::debug("Device closed, {}", duration_cast<milliseconds>(steady_clock::now() - t1).count());
}

bool DeviceBase::isClosed() const {
    return closed || !watchdogRunning;
}

void DeviceBase::checkClosed() const {
    if(isClosed()) throw std::invalid_argument("Device already closed or disconnected");
}

DeviceBase::~DeviceBase() {
    // Only allow to close once
    if(closed.exchange(true)) return;

    DeviceBase::closeImpl();
}

void DeviceBase::tryStartPipeline(const Pipeline& pipeline) {
    try {
        if(!startPipeline(pipeline)) {
            throw std::runtime_error("Couldn't start the pipeline");
        }
    } catch(const std::exception& e) {
        // close device (cleanup)
        close();
        // Rethrow original exception
        throw;
    }
}

void DeviceBase::init(OpenVINO::Version version, bool usb2Mode, const std::string& pathToMvcmd) {
    Config cfg;
    // Specify usb speed
    cfg.preboot.usb.maxSpeed = usb2Mode ? UsbSpeed::HIGH : DeviceBase::DEFAULT_USB_SPEED;
    // Specify the OpenVINO version
    cfg.version = version;
    init2(cfg, pathToMvcmd, {});
}
void DeviceBase::init(const Pipeline& pipeline, bool usb2Mode, const std::string& pathToMvcmd) {
    Config cfg = pipeline.getDeviceConfig();
    // Modify usb speed
    cfg.preboot.usb.maxSpeed = usb2Mode ? UsbSpeed::HIGH : DeviceBase::DEFAULT_USB_SPEED;
    init2(cfg, pathToMvcmd, pipeline);
}
void DeviceBase::init(OpenVINO::Version version, UsbSpeed maxUsbSpeed, const std::string& pathToMvcmd) {
    Config cfg;
    // Specify usb speed
    cfg.preboot.usb.maxSpeed = maxUsbSpeed;
    // Specify the OpenVINO version
    cfg.version = version;
    init2(cfg, pathToMvcmd, {});
}
void DeviceBase::init(const Pipeline& pipeline, UsbSpeed maxUsbSpeed, const std::string& pathToMvcmd) {
    Config cfg = pipeline.getDeviceConfig();
    // Modify usb speed
    cfg.preboot.usb.maxSpeed = maxUsbSpeed;
    init2(cfg, pathToMvcmd, pipeline);
}

void DeviceBase::init2(Config cfg, const std::string& pathToMvcmd, tl::optional<const Pipeline&> pipeline) {
    // Initalize depthai library if not already
    initialize();

    // Specify cfg
    config = cfg;

    if(pipeline) {
        spdlog::debug("Device - pipeline serialized, OpenVINO version: {}", OpenVINO::getVersionName(config.version));
    } else {
        spdlog::debug("Device - OpenVINO version: {}", OpenVINO::getVersionName(config.version));
    }

    // Set logging pattern of device (device id + shared pattern)
    pimpl->setPattern(fmt::format("[{}] {}", deviceInfo.getMxId(), LOG_DEFAULT_PATTERN));

    // Check if WD env var is set
    std::chrono::milliseconds watchdogTimeout = device::XLINK_WATCHDOG_TIMEOUT;
    auto watchdogMsStr = spdlog::details::os::getenv("DEPTHAI_WATCHDOG");
    if(!watchdogMsStr.empty()) {
        // Try parsing the string as a number
        try {
            std::chrono::milliseconds watchdog{std::stoi(watchdogMsStr)};
            config.preboot.watchdogTimeoutMs = watchdog.count();
            watchdogTimeout = watchdog;
            spdlog::debug("Using a custom watchdog value of {}", watchdogTimeout);
        } catch(const std::invalid_argument& e) {
            spdlog::warn("DEPTHAI_WATCHDOG value invalid: {}", e.what());
        }
    }

    // Get embedded mvcmd or external with applied config
    std::vector<std::uint8_t> fwWithConfig = Resources::getInstance().getDeviceFirmware(config, pathToMvcmd);

    // Init device (if bootloader, handle correctly - issue USB boot command)
    if(deviceInfo.state == X_LINK_UNBOOTED) {
        // Unbooted device found, boot and connect with XLinkConnection constructor
        connection = std::make_shared<XLinkConnection>(deviceInfo, fwWithConfig);
    } else if(deviceInfo.state == X_LINK_BOOTLOADER || deviceInfo.state == X_LINK_FLASH_BOOTED) {
        // If device is in flash booted state, reset to bootloader and then continue by booting appropriate FW
        if(deviceInfo.state == X_LINK_FLASH_BOOTED) {
            // Boot bootloader and set current deviceInfo to new device state
            deviceInfo = XLinkConnection::bootBootloader(deviceInfo);
        }

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
    pimpl->rpcStream = std::make_unique<XLinkStream>(*connection, device::XLINK_CHANNEL_MAIN_RPC, device::XLINK_USB_BUFFER_MAX_SIZE);

    pimpl->rpcClient = std::make_unique<nanorpc::core::client<nanorpc::packer::nlohmann_msgpack>>([this](nanorpc::core::type::buffer request) {
        // Lock for time of the RPC call, to not mix the responses between calling threads.
        // Note: might cause issues on Windows on incorrect shutdown. To be investigated
        std::unique_lock<std::mutex> lock(pimpl->rpcMutex);

        // Log the request data
        if(spdlog::get_level() == spdlog::level::trace) {
            spdlog::trace("RPC: {}", nlohmann::json::from_msgpack(request).dump());
        }

        // Send request to device
        pimpl->rpcStream->write(std::move(request));

        // Receive response back
        // Send to nanorpc to parse
        return pimpl->rpcStream->read();
    });

    // prepare watchdog thread, which will keep device alive
    // separate stream so it doesn't miss between potentially long RPC calls
    watchdogThread = std::thread([this, watchdogTimeout]() {
        try {
            XLinkStream stream(*this->connection, device::XLINK_CHANNEL_WATCHDOG, 128);
            std::vector<uint8_t> watchdogKeepalive = {0, 0, 0, 0};
            while(watchdogRunning) {
                stream.write(watchdogKeepalive);
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

    // prepare timesync thread, which will keep device synchronized
    timesyncThread = std::thread([this]() {
        using namespace std::chrono;

        try {
            XLinkStream stream(*this->connection, device::XLINK_CHANNEL_TIMESYNC, 128);
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
            XLinkStream stream(*this->connection, device::XLINK_CHANNEL_LOG, 128);
            while(loggingRunning) {
                // Block
                auto log = stream.read();

                // parse packet as msgpack
                try {
                    auto j = nlohmann::json::from_msgpack(log);
                    // create pipeline schema from retrieved data
                    nlohmann::from_json(j, messages);

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

// Convinience functions for querying current system information
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

bool DeviceBase::flashCalibration(CalibrationHandler calibrationDataHandler) {
    if(!calibrationDataHandler.validateCameraArray()) {
        throw std::runtime_error("Failed to validate the extrinsics connection. Enable debug mode for more information.");
    }
    return pimpl->rpcClient->call("storeToEeprom", calibrationDataHandler.getEepromData()).as<bool>();
}

CalibrationHandler DeviceBase::readCalibration() {
    dai::EepromData eepromData = pimpl->rpcClient->call("readFromEeprom");
    return CalibrationHandler(eepromData);
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
            XLinkStream stream(*connection, streamAssetStorage, device::XLINK_USB_BUFFER_MAX_SIZE);
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
