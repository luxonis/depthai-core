#include "depthai/pipeline/node/Vpp.hpp"

#include "depthai/pipeline/datatype/VppConfig.hpp"
#include "utility/Logging.hpp"

namespace dai {
namespace node {

Vpp::~Vpp() = default;

std::shared_ptr<Vpp> Vpp::build(Output& leftInput, Output& rightInput, Output* depthInput, Output* disparityInput, Output& confidenceInput) {
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    leftInput.link(left);
    rightInput.link(right);
    if(depthInput) depthInput->link(depth);
    if(disparityInput) disparityInput->link(disparity);
    confidenceInput.link(confidence);
    validateAndPruneSync();
#endif
    return std::static_pointer_cast<Vpp>(shared_from_this());
}

std::shared_ptr<Vpp> Vpp::build(Output& leftInput, Output& rightInput, Output& disparityInput, Output& confidenceInput) {
    return build(leftInput, rightInput, nullptr, &disparityInput, confidenceInput);
}

Vpp::Vpp(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, Vpp, VppProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

Vpp::Properties& Vpp::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
void Vpp::validateAndPruneSync() {
    if(depth.isConnected() && disparity.isConnected()) {
        throw std::runtime_error("Vpp: cannot connect both 'depth' and 'disparity' inputs simultaneously.");
    }
    if(!depth.isConnected() && !disparity.isConnected()) {
        throw std::runtime_error("Vpp: either 'depth' or 'disparity' input must be connected.");
    }
    for(auto it = sync->inputs.begin(); it != sync->inputs.end();) {
        const auto& name = it->first.second;
        if(name != leftInputName && name != rightInputName && !it->second.isConnected()) {
            it = sync->inputs.erase(it);
        } else {
            ++it;
        }
    }
}
#endif

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
