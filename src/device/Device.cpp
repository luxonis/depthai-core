#include "depthai/device/Device.hpp"

#include <XLink/XLinkPublicDefines.h>

#include <fstream>
#include <iostream>

#include "DeviceLogger.hpp"
#include "depthai-bootloader-shared/Bootloader.hpp"
#include "depthai-bootloader-shared/XLinkConstants.hpp"
#include "depthai/device/DeviceBootloader.hpp"
#include "depthai/pipeline/node/internal/XLinkIn.hpp"
#include "depthai/pipeline/node/internal/XLinkOut.hpp"
#include "depthai/xlink/XLinkConstants.hpp"
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
