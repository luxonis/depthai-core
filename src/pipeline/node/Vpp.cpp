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

void Vpp::postBuildStage() {
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    auto isConnected = [&](const std::string& name) { return sync->inputs.has(name) && sync->inputs[name].isConnected(); };
    if(isConnected(disparityName) && isConnected(depthName)) {
        throw std::invalid_argument("VPP expects either depth or disparity, not both");
    }

    // Sync waits for every declared input, so drop the optional ones that are not linked.
    for(const auto& name : {disparityName, depthName, confidenceName}) {
        auto it = sync->inputs.find({sync->inputs.name, name});
        if(it != sync->inputs.end() && !it->second.isConnected()) {
            sync->inputs.erase(it);
        }
    }
#endif
}

}  // namespace node
}  // namespace dai
