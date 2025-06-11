#pragma once

#include "NodeConnectionSchema.hpp"
#include "NodeObjInfo.hpp"
#include "depthai/properties/GlobalProperties.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * Specifies whole pipeline, nodes, properties and connections between nodes IOs
 */
struct PipelineSchema {
    std::vector<NodeConnectionSchema> connections;
    GlobalProperties globalProperties;
    std::unordered_map<int64_t, NodeObjInfo> nodes;
};

DEPTHAI_SERIALIZE_EXT(PipelineSchema, connections, globalProperties, nodes);

}  // namespace dai
