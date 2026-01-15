#pragma once
#include <cstdint>
#include <vector>

#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/DatatypeEnum.hpp"

namespace dai {

class NodeEventAggregationConfig {
   public:
    int64_t nodeId = -1;
    std::optional<std::vector<std::string>> inputs;
    std::optional<std::vector<std::string>> outputs;
    std::optional<std::vector<std::string>> others;
    bool events = false;

    DEPTHAI_SERIALIZE(NodeEventAggregationConfig, nodeId, inputs, outputs, others, events);
};

/// PipelineEventAggregationConfig configuration structure
class PipelineEventAggregationConfig : public Buffer {
   public:
    std::vector<NodeEventAggregationConfig> nodes;
    std::optional<uint32_t> repeatIntervalSeconds = std::nullopt;  // Keep sending the aggregated state without waiting for new config

    PipelineEventAggregationConfig() = default;
    virtual ~PipelineEventAggregationConfig();

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::PipelineEventAggregationConfig;
    }

    DEPTHAI_SERIALIZE(PipelineEventAggregationConfig, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum, nodes, repeatIntervalSeconds);
};

}  // namespace dai
