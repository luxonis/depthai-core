#pragma once

#include <cstdint>
#include <map>
#include <memory>

#include "FeatureTrackerAccel.hpp"
#include "depthai/pipeline/node/FeatureTracker.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

namespace dai {
namespace gate {
class FeatureTracker : public ::dai::node::FeatureTracker {
   public:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;

    static constexpr const uint8_t DESCRIPTOR_SIZE = DescriptorCalculator::DESCRIPTOR_SIZE;
    constexpr static const char* NAME = node::FeatureTracker::NAME;
    FeatureTracker() : FeatureTracker(std::make_unique<dai::node::FeatureTracker::Properties>()) {}
    FeatureTracker(std::unique_ptr<Properties> props)
        : dai::node::FeatureTracker(std::move(props)), featureTrackerConfig(std::make_shared<FeatureTrackerConfig>(properties.initialConfig)) {}
    bool runOnHost() const override {
        return true;
    }
    void run() override;

    std::shared_ptr<FeatureTrackerConfig> featureTrackerConfig;

    bool addMoreFeatures = true;

    uint32_t featureId = 0;
    uint32_t minimumNumberOfFeatures = 800;
    uint32_t minumumHarrisScore = 0;

    std::map<TrackedPoint, uint32_t> motionTrackedFeaturesInPreviousFrame;

    bool enableDescriptorCalculation = false;
};

}  // namespace gate
}  // namespace dai
