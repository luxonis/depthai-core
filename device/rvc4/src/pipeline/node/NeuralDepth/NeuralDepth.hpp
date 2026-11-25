#pragma once

// libraries
#include <depthai/pipeline/InputQueue.hpp>
#include <depthai/pipeline/Node.hpp>
#include <depthai/pipeline/Subnode.hpp>

#include "depthai/pipeline/node/NeuralDepth.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/node/StereoDepth/TemporalFilter/TemporalFilter.hpp"
#include "pipeline/node/StereoDepth/ThresholdFilter/ThresholdFilter.hpp"

namespace dai {
namespace gate {
class NeuralDepth : public ::dai::node::NeuralDepth {
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;

   public:
    constexpr static const char* NAME = node::NeuralDepth::NAME;
    NeuralDepth() : NeuralDepth(std::make_unique<dai::node::NeuralDepth::Properties>()) {}
    NeuralDepth(std::unique_ptr<Properties> props) : dai::node::NeuralDepth(std::move(props)) {
        // override that this is a host node
    }
    bool runOnHost() const override {
        return true;
    }
    void run() override;

    /* Post processing pipeline */
    TemporalFilter<float> temporalFilter;
    ThresholdFilter thresholdFilter;
};

}  // namespace gate
}  // namespace dai
