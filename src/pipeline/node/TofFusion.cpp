#include "depthai/pipeline/node/TofFusion.hpp"

namespace dai {
namespace node {

TofFusion::Properties& TofFusion::getProperties() {
    properties.initialConfig = *initialConfig;
    return properties;
}

TofFusion::TofFusion(std::unique_ptr<Properties> props)
    : DeviceNodeCRTP<DeviceNode, TofFusion, TofFusionProperties>(std::move(props)),
      initialConfig(std::make_shared<decltype(properties.initialConfig)>(properties.initialConfig)) {}

}  // namespace node
}  // namespace dai
