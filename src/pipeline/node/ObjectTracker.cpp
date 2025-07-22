#include "depthai/pipeline/node/ObjectTracker.hpp"

#include <cmath>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Rect.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai/pipeline/datatype/Tracklets.hpp"
#include "depthai/properties/ObjectTrackerProperties.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

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

// Compute Intersection-over-Union between two rectangles
float computeIoU(const cv::Rect& a, const cv::Rect& b) {
    int x1 = std::max(a.x, b.x);
    int y1 = std::max(a.y, b.y);
    int x2 = std::min(a.x + a.width, b.x + b.width);
    int y2 = std::min(a.y + a.height, b.y + b.height);
    int interArea = std::max(0, x2 - x1) * std::max(0, y2 - y1);
    int unionArea = a.area() + b.area() - interArea;
    return unionArea > 0 ? static_cast<float>(interArea) / unionArea : 0.0f;
}
float computeIoU(const Rect& a, const Rect& b) {
    int x1 = std::max(a.x, b.x);
    int y1 = std::max(a.y, b.y);
    int x2 = std::min(a.x + a.width, b.x + b.width);
    int y2 = std::min(a.y + a.height, b.y + b.height);
    int interArea = std::max(0, x2 - x1) * std::max(0, y2 - y1);
    int unionArea = a.area() + b.area() - interArea;
    return unionArea > 0 ? static_cast<float>(interArea) / unionArea : 0.0f;
}
float computeDistance(const Rect& a, const Rect& b) {
    Point2f centerA(a.x + a.width / 2.f, a.y + a.height / 2.f);
    Point2f centerB(b.x + b.width / 2.f, b.y + b.height / 2.f);
    return std::sqrt(std::pow(centerA.x - centerB.x, 2) + std::pow(centerA.y - centerB.y, 2));
}

// Basic NMS
std::vector<bool> nonMaxSuppression(const std::vector<ImgDetection>& detections, const std::vector<bool>& keep, float iouThreshold = 0.5f) {
    std::vector<bool> suppressed(detections.size(), false);

    bool useKeep = keep.size() == detections.size();

    if(useKeep) {
        // Filter detections overlapping with kept
        for(size_t i = 0; i < detections.size() - 1; ++i) {
            if(keep[i]) {
                auto rect1 = detToRect(detections[i]);
                for(size_t j = i + 1; j < detections.size(); ++j) {
                    if(!keep[j] && !suppressed[j]) {
                        auto rect2 = detToRect(detections[j]);
                        if(computeIoU(rect1, rect2) > iouThreshold) suppressed[j] = true;
                    }
                }
            }
        }
    }
    // Filter among non-kept detections
    for(size_t i = 0; i < detections.size() - 1; ++i) {
        if(!suppressed[i] && (!useKeep || !keep[i])) {
            auto rect1 = detToRect(detections[i]);
            for(size_t j = i + 1; j < detections.size(); ++j) {
                if(!suppressed[j] && (!useKeep || !keep[j])) {
                    auto rect2 = detToRect(detections[j]);
                    if(computeIoU(rect1, rect2) > iouThreshold) suppressed[j] = true;
                }
            }
        }
    }
    std::vector<bool> remaining(suppressed.size());
    for(size_t i = 0; i < suppressed.size(); ++i) {
        remaining[i] = !suppressed[i];
    }
    return remaining;
}

class TrackletExt : public Tracklet {
   public:
    uint32_t ageSinceStatusUpdate = 1;
    uint32_t detectedCount = 1;
    int closestDetection = -1;

    void update(const Rect& rect) {
        this->roi = rect;
        ++this->age;
        ++this->ageSinceStatusUpdate;

        // TODO: spatial stuff (if even possible with selected impls)
        spatialCoordinates.x = rect.x + rect.width / 2;
        spatialCoordinates.y = rect.y + rect.height / 2;
    }

    void match(const Rect& rect) {
        ++detectedCount;
        update(rect);
    }

    void updateStatus(Tracklet::TrackingStatus status) {
        this->status = status;
        ageSinceStatusUpdate = 1;
    }
};

class MultiKCFTracker {
    std::vector<std::shared_ptr<cv::Tracker>> trackers;
    std::vector<TrackletExt> objects;
    uint32_t maxId = 0;
    uint32_t numTracked = 0;

    float occlusionRatioThreshold;
    int32_t maxObjectsToTrack;
    std::vector<std::uint32_t> detectionLabelsToTrack;
    TrackerIdAssignmentPolicy trackerIdAssignmentPolicy;
    bool trackingPerClass;
    uint32_t trackletMaxLifespan;
    uint32_t trackletBirthThreshold;

    std::pair<uint32_t, uint32_t> getNextId() {
        uint32_t id = maxId + 1;
        switch(trackerIdAssignmentPolicy) {
            case TrackerIdAssignmentPolicy::UNIQUE_ID:
                for(uint32_t k = 0; k < trackers.size(); ++k)
                    if(trackers[k] != nullptr && k < id) id = k;
                if(id == maxId + 1) ++maxId;
                return {maxId++, id};
            case TrackerIdAssignmentPolicy::SMALLEST_ID: {
                for(uint32_t k = 0; k < trackers.size(); ++k)
                    if(trackers[k] != nullptr && k < id) id = k;
                if(id == maxId + 1) ++maxId;
                return {id, id};
            }
        }
        throw std::runtime_error("Unknown TrackerIdAssignmentPolicy");
    }

   public:
    MultiKCFTracker(const ObjectTrackerProperties& properties) {
        this->maxObjectsToTrack = properties.maxObjectsToTrack;
        this->trackerIdAssignmentPolicy = properties.trackerIdAssignmentPolicy;
        this->trackingPerClass = properties.trackingPerClass;
        this->occlusionRatioThreshold = properties.occlusionRatioThreshold;
        this->trackletMaxLifespan = properties.trackletMaxLifespan;
        // this->trackletBirthThreshold = properties.trackletBirthThreshold; // TODO uncomment
        this->trackletBirthThreshold = 1;
        objects.resize(maxObjectsToTrack);
        trackers.resize(maxObjectsToTrack, nullptr);
    }

    // Mapped detections must be mapped to frame and denormalized
    void init(const cv::Mat& frame, const std::vector<dai::ImgDetection>& mappedDetections) {
        auto remaining = nonMaxSuppression(mappedDetections, std::vector<bool>(), this->occlusionRatioThreshold);
        trackers.clear();
        trackers.resize(maxObjectsToTrack, nullptr);
        for(size_t i = 0; i < mappedDetections.size(); ++i) {
            if(!remaining[i] || (int)numTracked >= maxObjectsToTrack) continue;
            const auto& det = mappedDetections[i];
            auto tracker = cv::TrackerKCF::create();

            tracker->init(frame, toCvRect(detToRect(det)));
            const auto id = maxId++;
            ++numTracked;
            trackers[id] = std::move(tracker);
            objects[id] = TrackletExt{Tracklet{detToRect(det), (int)id, (int)det.label, 1, Tracklet::TrackingStatus::NEW, det, Point3f()}};
        }
    }

    void track(const cv::Mat& frame) {
        std::vector<Rect> updatedObjects;
        for(size_t i = 0; i < trackers.size(); ++i) {
            if(trackers[i]) {
                cv::Rect newBox;
                if(trackers[i]->update(frame, newBox)) {
                    objects[i].update(fromCvRect(newBox));
                }
            }
        }
    }

    void update(const cv::Mat& frame, const std::vector<dai::ImgDetection>& mappedDetections) {
        // First update location based on tracker
        track(frame);

        // Then map trackers to closest detection and re-initialize

        // Find closest (most overlapping) detection for each tracker
        std::vector<bool> matched(mappedDetections.size());
        for(uint32_t k = 0; k < trackers.size(); ++k) {
            auto& v = objects[k];
            if(v.status == Tracklet::TrackingStatus::REMOVED) continue;
            float maxIoU = 0;
            float minDistance = 1e9;
            for(auto i = 0u; i < mappedDetections.size(); ++i) {
                if(matched[i]) continue;
                const auto& det = mappedDetections[i];
                auto rect = detToRect(det);
                auto iou = computeIoU(v.roi, rect);
                auto dist = computeDistance(v.roi, rect);
                if((!trackingPerClass || (int)det.label == v.label) && iou < occlusionRatioThreshold
                   && (iou > maxIoU || (iou == maxIoU && dist < minDistance))) {
                    maxIoU = iou;
                    minDistance = dist;
                    v.closestDetection = i;
                }
            }
            if(v.closestDetection >= 0 && v.closestDetection < (int)matched.size()) matched[v.closestDetection] = true;
        }

        // Remove overlapping detections, keeping the matched
        auto remaining = nonMaxSuppression(mappedDetections, matched, occlusionRatioThreshold);

        // Reinitialize matched trackers & update status of unmatched
        for(uint32_t k = 0; k < trackers.size(); ++k) {
            auto& v = objects[k];
            v.closestDetection = -1;  // Reset closest detection
            if(v.closestDetection >= 0) {
                // Matched
                auto rect = detToRect(mappedDetections[v.closestDetection]);
                v.match(rect);
                if(v.status == Tracklet::TrackingStatus::NEW && v.detectedCount >= this->trackletBirthThreshold)
                    v.updateStatus(Tracklet::TrackingStatus::TRACKED);
                if(v.status == Tracklet::TrackingStatus::LOST) {
                    v.updateStatus(Tracklet::TrackingStatus::TRACKED);
                }
                trackers[k]->init(frame, toCvRect(rect));
            } else {
                // Not matched
                if(v.status == Tracklet::TrackingStatus::NEW) {
                    // Drop
                    --numTracked;
                    trackers[k] = nullptr;
                    objects[k].updateStatus(Tracklet::TrackingStatus::REMOVED);
                } else if(v.status == Tracklet::TrackingStatus::LOST) {
                    if(v.ageSinceStatusUpdate > trackletMaxLifespan) {
                        // Drop
                        --numTracked;
                        trackers[k] = nullptr;
                        objects[k].updateStatus(Tracklet::TrackingStatus::REMOVED);
                    }
                } else if(v.status == Tracklet::TrackingStatus::TRACKED) {
                    objects[k].updateStatus(Tracklet::TrackingStatus::LOST);
                }
            }
        }

        // Add unmatched detections as new trackers
        for(size_t i = 0; i < mappedDetections.size(); ++i) {
            if(remaining[i] && !matched[i] && (int)numTracked < maxObjectsToTrack - 1) {
                auto tracker = cv::TrackerKCF::create();
                const auto [id, index] = getNextId();
                if(index < trackers.size()) {
                    auto rect = detToRect(mappedDetections[i]);
                    tracker->init(frame, toCvRect(rect));
                    ++numTracked;
                    trackers[index] = std::move(tracker);
                    objects[index] =
                        TrackletExt{Tracklet{rect, (int)id, (int)mappedDetections[i].label, 1, Tracklet::TrackingStatus::NEW, mappedDetections[i], Point3f()}};
                }
            }
        }
    }

    std::vector<Tracklet> getTracklets() const {
        std::vector<Tracklet> ret;
        ret.reserve(numTracked);
        for(uint32_t k = 0; k < trackers.size(); ++k) {
            if(trackers[k] != nullptr) {
                ret.push_back((Tracklet)objects[k]);
            }
        }
        return ret;
    }
};

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

    MultiKCFTracker tracker(properties);
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
            } else {
                logger->error("Input detections is not of type ImgDetections or SpatialImgDetections, skipping tracking");
            }
        }
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
                    tracker.init(inputTrackerImg->getCvFrame(), detections);
                } else {
                    tracker.update(inputTrackerImg->getCvFrame(), detections);
                }
            }
        } else if(!first) {
            tracker.track(inputTrackerImg->getCvFrame());
        }
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

}  // namespace node
}  // namespace dai
