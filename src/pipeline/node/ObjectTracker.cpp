#include "depthai/pipeline/node/ObjectTracker.hpp"

#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <stdexcept>
#include <utility>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Rect.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai/pipeline/datatype/Tracklets.hpp"
#include "depthai/properties/ObjectTrackerProperties.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/ObjectTrackerImpl.hpp"

namespace dai {
namespace node {

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
void ObjectTracker::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

/**
 * Check if the node is set to run on host
 */
bool ObjectTracker::runOnHost() const {
    return runOnHostVar;
}

cv::Rect toCvRect(Rect r) {
    if(r.isNormalized()) throw std::runtime_error("dai::Rect must not be normalized in conversion to cv::Rect");
    return {(int)r.x, (int)r.y, (int)r.width, (int)r.height};
}
Rect fromCvRect(cv::Rect r) {
    return Rect(r.x, r.y, r.width, r.height);
}
Rect detToRect(const ImgDetection& det) {
    return Rect(Point2f(det.xmin, det.ymin), Point2f(det.xmax, det.ymax));
}

template <typename T>
bool contains(const std::vector<T>& vec, const T& el) {
    for(const auto& e : vec) {
        if(e == el) return true;
    }
    return false;
}

void ObjectTracker::run() {
    float trackerThreshold = properties.trackerThreshold;
    std::vector<std::uint32_t> detectionLabelsToTrack = properties.detectionLabelsToTrack;
    TrackerType trackerType = properties.trackerType;  // TODO

    auto& logger = pimpl->logger;

    impl::OCSTracker tracker(properties);
    bool first = true;

    uint32_t seqNum = 0;

    while(isRunning()) {
        std::shared_ptr<ImgFrame> inputTrackerImg;
        std::shared_ptr<ImgFrame> inputDetectionImg;
        std::shared_ptr<ImgDetections> inputImgDetections;
        std::shared_ptr<SpatialImgDetections> inputSpatialImgDetections;

        bool gotDetections = false;

        inputTrackerImg = inputTrackerFrame.get<ImgFrame>();
        if(inputDetections.has()) {
            auto detectionsBuffer = inputDetections.get<Buffer>();
            inputImgDetections = std::dynamic_pointer_cast<ImgDetections>(detectionsBuffer);
            inputSpatialImgDetections = std::dynamic_pointer_cast<SpatialImgDetections>(detectionsBuffer);
            if(inputImgDetections && inputImgDetections->detections.size() > 0) {
                logger->warn("Input detections received, tracking will be performed on the detections");
                gotDetections = true;
                if(!inputImgDetections->transformation.has_value()) {
                    logger->debug("Transformation is not set for input detections, inputDetectionFrame is required");
                    inputDetectionImg = inputDetectionFrame.get<ImgFrame>();
                }
            } else if(inputSpatialImgDetections && inputSpatialImgDetections->detections.size() > 0) {
                gotDetections = true;
                if(!inputSpatialImgDetections->transformation.has_value()) {
                    logger->debug("Transformation is not set for input detections, inputDetectionFrame is required");
                    inputDetectionImg = inputDetectionFrame.get<ImgFrame>();
                }
            } else if(!inputImgDetections && !inputSpatialImgDetections) {
                logger->error("Input detections is not of type ImgDetections or SpatialImgDetections, skipping tracking");
            }
        }
        // Either update or track, not both
        if(gotDetections) {
            // TODO: sync messages !!!
            ImgTransformation detectionsTransformation = (inputImgDetections ? inputImgDetections->transformation : inputSpatialImgDetections->transformation)
                                                             .value_or(inputDetectionImg->transformation);
            std::vector<ImgDetection> detections;
            detections.reserve(inputImgDetections ? inputImgDetections->detections.size() : inputSpatialImgDetections->detections.size());
            for(size_t i = 0; i < (inputImgDetections ? inputImgDetections->detections.size() : inputSpatialImgDetections->detections.size()); ++i) {
                const auto& detection = inputImgDetections ? inputImgDetections->detections[i] : (ImgDetection)inputSpatialImgDetections->detections[i];
                if(detection.confidence >= trackerThreshold && (detectionLabelsToTrack.empty() || contains(detectionLabelsToTrack, detection.label))) {
                    // Denormalize and remap to inputTrackerImg
                    uint32_t width = detectionsTransformation.getSize().first;
                    uint32_t height = detectionsTransformation.getSize().second;

                    RotatedRect detRRect(detToRect(detection));
                    if(detRRect.isNormalized()) detRRect = detRRect.denormalize(width, height);

                    auto remapped = detectionsTransformation.remapRectTo(inputTrackerImg->transformation, detRRect);

                    const auto [minx, miny, maxx, maxy] = remapped.getOuterRect();

                    ImgDetection det(detection);
                    det.xmin = std::max(0.f, minx);
                    det.ymin = std::max(0.f, miny);
                    det.xmax = std::min((float)width, maxx);
                    det.ymax = std::min((float)height, maxy);

                    if(det.xmin < det.xmax && det.ymin < det.ymax) detections.push_back(det);
                }
            }
            if(!detections.empty()) {
                if(first) {
                    first = false;
                    tracker.init(*inputTrackerImg, detections);
                } else {
                    tracker.update(*inputTrackerImg, detections);
                }
            }
        } else if(!first) {
            tracker.track(*inputTrackerImg);
        }
        if(!first) {
            auto trackletsMsg = std::make_shared<Tracklets>();
            trackletsMsg->tracklets = tracker.getTracklets();
            trackletsMsg->ts = inputTrackerImg->ts;
            trackletsMsg->tsDevice = inputTrackerImg->tsDevice;
            trackletsMsg->sequenceNum = seqNum++;

            out.send(trackletsMsg);
            passthroughTrackerFrame.send(inputTrackerImg);
            if(gotDetections) {
                passthroughDetections.send(inputImgDetections ? std::dynamic_pointer_cast<Buffer>(inputImgDetections)
                                                              : std::dynamic_pointer_cast<Buffer>(inputSpatialImgDetections));
                if(inputDetectionImg) {
                    passthroughDetectionFrame.send(inputDetectionImg);
                }
            }
        }
    }
}

}  // namespace node
}  // namespace dai
