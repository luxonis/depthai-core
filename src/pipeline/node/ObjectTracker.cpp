#include "depthai/pipeline/node/ObjectTracker.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

ObjectTracker::ObjectTracker(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {}

std::string ObjectTracker::getName() const {
    return "ObjectTracker";
}

std::vector<Node::Output> ObjectTracker::getOutputs() {
    return {out, passthroughTrackerFrame, passthroughDetectionFrame, passthroughDetections};
}

std::vector<Node::Input> ObjectTracker::getInputs() {
    return {inputTrackerFrame, inputDetectionFrame, inputDetections};
}

nlohmann::json ObjectTracker::getProperties() {
    nlohmann::json j;
    nlohmann::to_json(j, properties);
    return j;
}

std::shared_ptr<Node> ObjectTracker::clone() {
    return std::make_shared<std::decay<decltype(*this)>::type>(*this);
}

void ObjectTracker::setTrackerThreshold(float threshold) {
    properties.trackerThreshold = threshold;
}

void ObjectTracker::setMaxObjectsToTrack(std::int32_t maxObjectsToTrack) {
    properties.maxObjectsToTrack = maxObjectsToTrack;
}

void ObjectTracker::setDetectionLabelsToTrack(std::vector<std::uint32_t> labels) {
    properties.detectionLabelsToTrack = labels;
}

void ObjectTracker::setTrackerType(TrackerType type) {
    properties.trackerType = type;
}

void ObjectTracker::setTrackerIdAssigmentPolicy(TrackerIdAssigmentPolicy type) {
    properties.trackerIdAssigmentPolicy = type;
}

}  // namespace node
}  // namespace dai
