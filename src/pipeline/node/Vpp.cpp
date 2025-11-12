#include "depthai/pipeline/node/Vpp.hpp"

#include "depthai/pipeline/datatype/VppConfig.hpp"
#include "utility/Logging.hpp"

namespace dai {
namespace node {

Vpp::~Vpp() = default;

Vpp::Vpp(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, Vpp, VppProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

Vpp::Vpp()
    : DeviceNodeCRTP<DeviceNode, Vpp, VppProperties>(std::make_unique<Properties>()),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

Vpp::Properties& Vpp::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

void Vpp::buildInternal() {
    left = &sync->inputs[leftInputName];
    right = &sync->inputs[rightInputName];
    disparity = &sync->inputs[disparityName];
    confidence = &sync->inputs[confidenceName];
    sync->out.link(syncedInputs);
}

}  // namespace node
}  // namespace dai
