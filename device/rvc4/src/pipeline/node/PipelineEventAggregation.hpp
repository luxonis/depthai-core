#pragma once

#include "depthai/pipeline/node/internal/PipelineEventAggregation.hpp"

namespace dai {
namespace gate {

class PipelineEventAggregation : public node::internal::PipelineEventAggregation {
   public:
    constexpr static const char* NAME = node::internal::PipelineEventAggregation::NAME;
    PipelineEventAggregation() : PipelineEventAggregation(std::make_unique<dai::node::internal::PipelineEventAggregation::Properties>()) {}
    PipelineEventAggregation(std::unique_ptr<Properties> props) : dai::node::internal::PipelineEventAggregation(std::move(props)) {}
    bool runOnHost() const override {
        return true;
    }
};

}  // namespace gate
}  // namespace dai
