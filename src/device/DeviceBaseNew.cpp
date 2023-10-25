#include "depthai/device/DeviceBaseNew.hpp"

#include <dlfcn.h>

#include <iostream>

#include "depthai-shared/log/LogConstants.hpp"
#include "depthai-shared/log/LogLevel.hpp"
#include "depthai-shared/log/LogMessage.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/device/DeviceBaseImpl.hpp"
#include "depthai/device/DeviceBase.hpp"

namespace dai {

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



// Implementation of the implementation class
std::vector<CameraBoardSocket> DeviceBaseImpl::getConnectedCameras() {
    // Return some mock data for demonstration purposes
    return {CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B};
}

DeviceBaseImpl::~DeviceBaseImpl() {
    // Destructor
    std::cout << "DeviceBaseImpl destructor called" << std::endl;
}

void DeviceBaseImpl::setPattern(const std::string& pattern) {
    logger.set_pattern(pattern);
}

void DeviceBaseImpl::setLogLevel(LogLevel level) {
    // Converts LogLevel to spdlog and reconfigures logger level
    auto spdlogLevel = logLevelToSpdlogLevel(level, spdlog::level::warn);
    // Set level for all configured sinks
    logger.set_level(spdlogLevel);
}

LogLevel DeviceBaseImpl::getLogLevel() {
    // Converts spdlog to LogLevel
    return spdlogLevelToLogLevel(logger.level(), LogLevel::WARN);
}

// END OF IMPLEMENTATION OF THE IMPLEMENTATION CLASS
// Constructor
dai::DeviceBaseNew::DeviceBaseNew() : pimpl{static_cast<DeviceBaseImpl*>(new DeviceBaseImpl())} {
    shared_library_handle = dlopen("libdepthai-device-kb_shared.so", RTLD_LAZY);
    if (!shared_library_handle) {
        std::cerr << "Cannot open library: " << dlerror() << '\n';
        return;
    }

    // load the symbol
    typedef DeviceBaseImpl* (*create_t)();

    // reset errors
    dlerror();

    create_t createDeviceBaseImpl = (create_t) dlsym(shared_library_handle, "createDeviceImpl");
    const char *dlsym_error = dlerror();
    if (dlsym_error) {
        std::cerr << "Cannot load symbol 'create': " << dlsym_error << '\n';
        dlclose(shared_library_handle);
        return;
    }
    DeviceBaseImpl* impl = createDeviceBaseImpl();

    if(1){
        pimpl = std::unique_ptr<DeviceBaseImpl>(impl);
    }
}

dai::DeviceBaseNew::~DeviceBaseNew() {
    // Destructor
    std::cout << "DeviceBaseNew destructor called" << std::endl;
}

std::vector<CameraBoardSocket> dai::DeviceBaseNew::getConnectedCameras() {
    return pimpl->getConnectedCameras();
}
}  // namespace dai