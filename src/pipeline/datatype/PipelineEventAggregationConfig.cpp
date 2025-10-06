#include "depthai/pipeline/datatype/PipelineEventAggregationConfig.hpp"

namespace dai {

PipelineEventAggregationConfig::~PipelineEventAggregationConfig() = default;

void PipelineEventAggregationConfig::serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const {
    metadata = utility::serialize(*this);
    datatype = DatatypeEnum::PipelineEventAggregationConfig;
}

}  // namespace dai
