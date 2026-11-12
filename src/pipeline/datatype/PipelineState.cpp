#include "depthai/pipeline/datatype/PipelineState.hpp"

namespace dai {

PipelineState::~PipelineState() = default;

void PipelineState::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::PipelineState;
}

nlohmann::json PipelineState::toJson() const {
    nlohmann::json j;
    j["nodeStates"] = nodeStates;
    return j;
}
}  // namespace dai
