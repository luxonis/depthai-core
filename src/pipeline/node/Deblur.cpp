#include "depthai/pipeline/node/Deblur.hpp"

#include "depthai/device/Device.hpp"
#include "utility/DeblurValidation.hpp"
namespace dai {
namespace node {
Deblur& Deblur::setConfig(const DeblurProperties& config) {
    utility::validateDeblurConfig(config);
    properties = config;
    return *this;
}
DeblurProperties Deblur::getConfig() const {
    return properties;
}
void Deblur::buildInternal() {
    utility::validateDeblurConfig(properties);
    if(device && device->getPlatform() != Platform::RVC4) throw std::runtime_error("Deblur is supported only on RVC4 devices");
}
}  // namespace node
}  // namespace dai
