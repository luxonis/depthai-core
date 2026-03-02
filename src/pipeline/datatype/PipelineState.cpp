#include "depthai/pipeline/datatype/PipelineState.hpp"

namespace dai {

PipelineState::~PipelineState() = default;

nlohmann::json PipelineState::toJson() const {
    nlohmann::json j;
    j["nodeStates"] = nodeStates;
    return j;
}
}  // namespace dai
