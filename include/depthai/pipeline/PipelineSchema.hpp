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
    std::vector<std::pair<int64_t, int64_t>> bridges;
};

DEPTHAI_SERIALIZE_EXT(PipelineSchema, connections, globalProperties, nodes, bridges);

}  // namespace dai
