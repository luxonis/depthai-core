#include "depthai/pipeline/node/ObjectTracker.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Rect.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ObjectTrackerConfig.hpp"
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai/pipeline/datatype/Tracklets.hpp"
#include "depthai/properties/ObjectTrackerProperties.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/ObjectTrackerImpl.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/opencv.hpp>
#endif

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
void ObjectTracker::setOcclusionRatioThreshold(float theshold) {
    properties.occlusionRatioThreshold = theshold;
}
void ObjectTracker::setTrackletMaxLifespan(uint32_t trackletMaxLifespan) {
    properties.trackletMaxLifespan = trackletMaxLifespan;
}
void ObjectTracker::setTrackletBirthThreshold(uint32_t trackletBirthThreshold) {
    properties.trackletBirthThreshold = trackletBirthThreshold;
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

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
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
#endif

template <typename T>
bool contains(const std::vector<T>& vec, const T& el) {
    for(const auto& e : vec) {
        if(e == el) return true;
    }
    return false;
}

void ObjectTracker::run() {
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    auto& logger = pimpl->logger;

    float trackerThreshold = properties.trackerThreshold;
    std::vector<std::uint32_t> detectionLabelsToTrack = properties.detectionLabelsToTrack;
    TrackerType trackerType = properties.trackerType;

    if(trackerType != TrackerType::SHORT_TERM_IMAGELESS) {
        logger->warn("Selected tracker type is not supported on RVC4, using SHORT_TERM_IMAGELESS instead");
    }

    impl::OCSTracker tracker(properties);

    while(isRunning()) {
        std::shared_ptr<ImgFrame> inputTrackerImg;
        std::shared_ptr<ImgFrame> inputDetectionImg;
        std::shared_ptr<ImgDetections> inputImgDetections;
        std::shared_ptr<SpatialImgDetections> inputSpatialImgDetections;
        std::shared_ptr<ObjectTrackerConfig> inputCfg;

        inputTrackerImg = inputTrackerFrame.get<ImgFrame>();

        bool gotDetections = false;
        auto detectionsBuffer = inputDetections.get<Buffer>();
        inputImgDetections = std::dynamic_pointer_cast<ImgDetections>(detectionsBuffer);
        inputSpatialImgDetections = std::dynamic_pointer_cast<SpatialImgDetections>(detectionsBuffer);
        if(inputImgDetections) {
            gotDetections = true;
            if(!inputImgDetections->transformation.has_value()) {
                logger->debug("Transformation is not set for input detections, inputDetectionFrame is required");
                inputDetectionImg = inputDetectionFrame.get<ImgFrame>();
            }
        } else if(inputSpatialImgDetections) {
            gotDetections = true;
            if(!inputSpatialImgDetections->transformation.has_value()) {
                logger->debug("Transformation is not set for input detections, inputDetectionFrame is required");
                inputDetectionImg = inputDetectionFrame.get<ImgFrame>();
            }
        } else {
            logger->error("Input detections is not of type ImgDetections or SpatialImgDetections, skipping tracking");
        }
        if(inputConfig.getWaitForMessage()) {
            inputCfg = inputConfig.get<ObjectTrackerConfig>();
        } else {
            inputCfg = inputConfig.tryGet<ObjectTrackerConfig>();
        }

        if(inputCfg) {
            tracker.configure(*inputCfg);
        }

        // Either update or track, not both
        if(gotDetections) {
            // TODO: sync messages (currently OK since only imageless tracker is supported)
            std::vector<ImgDetection> detections;
            std::vector<Point3f> spatialData;
            detections.reserve(inputImgDetections ? inputImgDetections->detections.size() : inputSpatialImgDetections->detections.size());
            spatialData.reserve(detections.size());

            ImgTransformation detectionsTransformation = (inputImgDetections ? inputImgDetections->transformation : inputSpatialImgDetections->transformation)
                                                             .value_or(inputDetectionImg->transformation);

            if(!detectionsTransformation.isValid()) {
                throw std::runtime_error("ImgTransformation must be set on either inputDetections or inputDetectionFrame");
            }

            for(size_t i = 0; i < (inputImgDetections ? inputImgDetections->detections.size() : inputSpatialImgDetections->detections.size()); ++i) {
                const auto& detection = inputImgDetections ? inputImgDetections->detections[i] : inputSpatialImgDetections->detections[i].getImgDetection();
                if(detection.confidence >= trackerThreshold && (detectionLabelsToTrack.empty() || contains(detectionLabelsToTrack, detection.label))) {
                    // Denormalize and remap to inputTrackerImg
                    uint32_t width = detectionsTransformation.getSize().first;
                    uint32_t height = detectionsTransformation.getSize().second;

                    RotatedRect detRRect(detToRect(detection));
                    if(detRRect.size.width <= 1.5f && detRRect.size.height <= 1.5f) detRRect = detRRect.denormalize(width, height, true);

                    auto remapped = detectionsTransformation.remapRectTo(inputTrackerImg->transformation, detRRect);

                    const auto [minx, miny, maxx, maxy] = remapped.getOuterRect();

                    ImgDetection det(detection);
                    det.xmin = std::max(0.f, minx);
                    det.ymin = std::max(0.f, miny);
                    det.xmax = std::min((float)inputTrackerImg->getWidth(), maxx);
                    det.ymax = std::min((float)inputTrackerImg->getHeight(), maxy);

                    if(det.xmin < det.xmax && det.ymin < det.ymax) {
                        detections.push_back(det);
                        if(inputSpatialImgDetections) {
                            spatialData.push_back(inputSpatialImgDetections->detections[i].spatialCoordinates);
                        } else {
                            spatialData.push_back(Point3f(0, 0, 0));  // No spatial data available
                        }
                    }
                }
            }
            if(!detections.empty() && !tracker.isInitialized()) {
                tracker.init(*inputTrackerImg, detections, spatialData);
            } else if(tracker.isInitialized()) {
                tracker.update(*inputTrackerImg, detections, spatialData);
            }
        } else if(tracker.isInitialized()) {
            tracker.track(*inputTrackerImg);
        }
        auto trackletsMsg = std::make_shared<Tracklets>();
        trackletsMsg->ts = inputTrackerImg->ts;
        trackletsMsg->tsDevice = inputTrackerImg->tsDevice;
        trackletsMsg->sequenceNum = inputTrackerImg->sequenceNum;
        trackletsMsg->tracklets = tracker.isInitialized() ? tracker.getTracklets() : std::vector<Tracklet>();
        // Normalize the tracklets
        for(auto& tracklet : trackletsMsg->tracklets) {
            tracklet.roi = tracklet.roi.normalize(inputTrackerImg->getWidth(), inputTrackerImg->getHeight());
        }
        trackletsMsg->transformation = inputTrackerImg->transformation;

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
#else
    throw std::runtime_error("ObjectTracker::run() requires OpenCV support. Please compile with OpenCV.");
#endif
}

}  // namespace node
}  // namespace dai
