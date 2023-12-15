#pragma once
#include "depthai/pipeline/Node.hpp"

// shared
#include <depthai-shared/properties/ThermalProperties.hpp>

#include "depthai/pipeline/node/Thermal.hpp"

namespace dai {
namespace node {

Thermal::Thermal(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Thermal(par, nodeId, std::make_unique<ThermalProperties>()) {}
Thermal::Thermal(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, Thermal, ThermalProperties>(par, nodeId, std::move(props)) {
        properties.width = 292;
        properties.baudrate = 22000000;
    }

Thermal::Properties& Thermal::getProperties() {
    return properties;
}
}  // namespace node
}  // namespace dai
