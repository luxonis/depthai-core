#include "depthai/pipeline/node/OpticalFlow.hpp"

#include <stdexcept>

#include "depthai/device/Device.hpp"
namespace dai {
namespace node {
void OpticalFlow::buildInternal() {
    if(device && device->getPlatform() != Platform::RVC4) {
        throw std::runtime_error("OpticalFlow is supported only on RVC4 devices");
    }
}
}  // namespace node
}  // namespace dai
