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

Device::Device() : DeviceBase() {}

Device::~Device() {
    DeviceBase::close();
}

void Device::closeImpl() {
    DeviceBase::closeImpl();
}

}  // namespace dai
