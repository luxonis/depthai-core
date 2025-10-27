#include "depthai/device/Device.hpp"

// std
#include <fstream>
#include <iostream>

// shared
#include <XLink/XLinkPublicDefines.h>

#include "depthai-bootloader-shared/Bootloader.hpp"
#include "depthai-bootloader-shared/XLinkConstants.hpp"
#include "depthai/xlink/XLinkConstants.hpp"

// project
#include "DeviceLogger.hpp"
#include "depthai/device/DeviceBootloader.hpp"
#include "depthai/pipeline/node/internal/XLinkIn.hpp"
#include "depthai/pipeline/node/internal/XLinkOut.hpp"
#include "pipeline/Pipeline.hpp"
#include "utility/Environment.hpp"
#include "utility/Initialization.hpp"
#include "utility/Resources.hpp"

namespace dai {

std::string platform2string(Platform platform) {
    switch(platform) {
        case Platform::RVC2:
            return "RVC2";
        case Platform::RVC3:
            return "RVC3";
        case Platform::RVC4:
            return "RVC4";
    }
    throw std::runtime_error("Unknown platform");
}

Platform string2platform(const std::string& platform) {
    // Convert to lower case
    std::string platformLower = platform;
    std::for_each(platformLower.begin(), platformLower.end(), [](char& c) { c = ::tolower(c); });

    // Check which platform it is
    if(platformLower == "rvc2") {
        return Platform::RVC2;
    } else if(platformLower == "rvc3") {
        return Platform::RVC3;
    } else if(platformLower == "rvc4") {
        return Platform::RVC4;
    }

    std::string errorMessage = "Unknown platform: " + platform;
    throw std::runtime_error(errorMessage);
}

Device::Device() : DeviceBase() {}

Device::~Device() {
    DeviceBase::close();
}

Platform Device::getPlatform() const {
    auto platform = getDeviceInfo().platform;
    switch(platform) {
        case X_LINK_MYRIAD_X:
            return Platform::RVC2;
            break;
        case X_LINK_RVC3:
            return Platform::RVC3;
            break;
        case X_LINK_RVC4:
            return Platform::RVC4;
            break;
        case X_LINK_ANY_PLATFORM:
        case X_LINK_MYRIAD_2:
        default:
            throw std::runtime_error("Unknown platform");
            break;
    }
}

std::string Device::getPlatformAsString() const {
    return platform2string(getPlatform());
}

void Device::closeImpl() {
    DeviceBase::closeImpl();
}

}  // namespace dai
