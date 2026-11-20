#include "depthai/pipeline/datatype/PipelineEvent.hpp"

namespace dai {
PipelineEvent::~PipelineEvent() = default;

void PipelineEvent::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::PipelineEvent;
}
}  // namespace dai
