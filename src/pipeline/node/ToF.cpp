#include "depthai/pipeline/node/ToF.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

ToF::ToF()
    : NodeCRTP<DeviceNode, ToF, ToFProperties>(), rawConfig(std::make_shared<RawToFConfig>()), initialConfig(rawConfig) {}

ToF::ToF(std::unique_ptr<Properties> props)
    : NodeCRTP<DeviceNode, ToF, ToFProperties>(std::move(props)),
      rawConfig(std::make_shared<RawToFConfig>(properties.initialConfig)),
      initialConfig(rawConfig) {}

ToF::Properties& ToF::getProperties() {
    properties.initialConfig = *rawConfig;
    return properties;
}

}  // namespace node
}  // namespace dai
