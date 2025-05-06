#include "depthai/pipeline/node/ObjectTracker.hpp"

#include "spdlog/fmt/fmt.h"

namespace dai {
namespace node {

ObjectTracker::ObjectTracker(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId)
    : ObjectTracker(par, nodeId, std::make_unique<ObjectTracker::Properties>()) {}
ObjectTracker::ObjectTracker(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props)
    : NodeCRTP<Node, ObjectTracker, ObjectTrackerProperties>(par, nodeId, std::move(props)) {
    setInputRefs({&inputTrackerFrame, &inputDetectionFrame, &inputDetections, &inputConfig});
    setOutputRefs({&out, &passthroughTrackerFrame, &passthroughDetectionFrame, &passthroughDetections});
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

void ObjectTracker::setTrackerIdAssignmentPolicy(TrackerIdAssignmentPolicy type) {
    properties.trackerIdAssignmentPolicy = type;
}
void ObjectTracker::setTrackingPerClass(bool trackingPerClass) {
    properties.trackingPerClass = trackingPerClass;
}

void ObjectTracker::setOcclusionRatioThreshold(float occlusionRatioThreshold) {
    properties.occlusionRatioThreshold = occlusionRatioThreshold;

    if(occlusionRatioThreshold < 0.0f || occlusionRatioThreshold > 1.0f) {
        throw std::runtime_error(fmt::format("Occlusion ratio threshold must be between 0 and 1, got {}", occlusionRatioThreshold));
    }
}

void ObjectTracker::setTrackletMaxLifespan(uint32_t lifespan) {
    properties.trackletMaxLifespan = lifespan;
    if(lifespan < 1) {
        throw std::runtime_error(fmt::format("Tracklet lifespan must be greater than 0, got {}", lifespan));
    }
}

void ObjectTracker::setTrackletBirthThreshold(uint32_t birthThreshold) {
    properties.trackletBirthThreshold = birthThreshold;
}

}  // namespace node
}  // namespace dai
