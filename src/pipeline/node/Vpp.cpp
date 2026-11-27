#include "depthai/pipeline/node/Vpp.hpp"

#include "depthai/device/Device.hpp"
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
#else
    (void)leftInput;
    (void)rightInput;
    (void)disparityInput;
    (void)confidenceInput;
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
    sync->inputs.merge(unusedSyncInputs);
    auto isConnected = [&](const std::string& name) { return sync->inputs.has(name) && sync->inputs[name].isConnected(); };
    const bool hasDisparity = isConnected(disparityName);
    const bool hasDepth = isConnected(depthName);
    if(hasDirectSyncedInput && !hasDisparity && !hasDepth) {  // Legacy direct MessageGroup input: the device validates its contents
        return;
    }
    if(hasDisparity == hasDepth) {
        throw std::invalid_argument("VPP expects exactly one of depth or disparity to be linked");
    }

    for(const auto& name : {disparityName, depthName, confidenceName}) {  // Sync waits for every input: move unlinked ones out, keep Vpp references valid
        auto it = sync->inputs.find({sync->inputs.name, name});
        if(it != sync->inputs.end() && !it->second.isConnected()) {
            unusedSyncInputs.insert(sync->inputs.extract(it));
        }
    }
#endif
}

void Vpp::buildStage1() {
#ifndef DEPTHAI_INTERNAL_DEVICE_BUILD_RVC4
    for(const auto& connection : sync->out.getConnections()) {  // Must run before host/device bridges replace the original connections
        if(connection.in == &syncedInputs) {
            sync->out.unlink(syncedInputs);
            hasDirectSyncedInput = syncedInputs.isConnected();
            sync->out.link(syncedInputs);
            return;
        }
    }
    hasDirectSyncedInput = syncedInputs.isConnected();
#endif
}

}  // namespace node
}  // namespace dai
