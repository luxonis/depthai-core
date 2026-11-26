#pragma once

#include <spdlog/async_logger.h>

#include "depthai/pipeline/node/SpatialLocationCalculator.hpp"
#include "depthai/properties/SpatialLocationCalculatorProperties.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class SpatialLocationCalculator : public node::SpatialLocationCalculator {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;

    constexpr static const char* NAME = node::SpatialLocationCalculator::NAME;
    SpatialLocationCalculator() : SpatialLocationCalculator(std::make_unique<SpatialLocationCalculator::Properties>()) {}
    SpatialLocationCalculator(std::unique_ptr<Properties> props)
        : node::SpatialLocationCalculator(std::move(props)), roiConfig(std::make_shared<SpatialLocationCalculatorConfig>(properties.roiConfig)) {}
    bool runOnHost() const override {
        return true;
    }
    std::shared_ptr<SpatialLocationCalculatorConfig> roiConfig;
};
}  // namespace gate
}  // namespace dai