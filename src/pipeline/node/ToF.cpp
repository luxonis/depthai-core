#include "depthai/pipeline/node/ToF.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

ToF::ToF(std::unique_ptr<Properties> props)
    : NodeCRTP<DeviceNode, ToF, ToFProperties>(std::move(props)) {}

ToF::Properties& ToF::getProperties() {
    properties.initialConfig = initialConfig;
    return properties;
}

}  // namespace node
}  // namespace dai
