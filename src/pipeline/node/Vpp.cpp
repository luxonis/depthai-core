#include "depthai/pipeline/node/Vpp.hpp"

#include "depthai/pipeline/datatype/VppConfig.hpp"
#include "utility/Logging.hpp"

namespace dai {
namespace node {

Vpp::~Vpp() = default;

std::shared_ptr<Vpp> Vpp::build(Output& leftInput, Output& rightInput, Output& disparityInput, Output& confidenceInput) {
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    leftInput.link(left);
    rightInput.link(right);
    disparityInput.link(disparity);
    confidenceInput.link(confidence);
#endif
    return std::static_pointer_cast<Vpp>(shared_from_this());
}

Vpp::Vpp(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, Vpp, VppProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

Vpp::Properties& Vpp::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

void Vpp::buildInternal() {
    if(device) {
        auto platform = device->getPlatform();
        if(platform != Platform::RVC4) {
            throw std::runtime_error("Vpp node is supported only on RVC4 devices.");
        }
    }
    sync->out.link(syncedInputs);
}

}  // namespace node
}  // namespace dai
