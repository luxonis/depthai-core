#include "beta/utilities/Detection/DetectionUtils.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <utility>

#include "depthai/common/Point2f.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "depthai/common/Size2f.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace DetectionUtils {

namespace {

/**
 * Overlap of two [x, y, width, height] boxes as intersection area over union area, matching
 * OpenCV's rectOverlap (1.f - (float)jaccardDistance) on cv::Rect2d, including the float cast of
 * the double-precision jaccard distance. An intersection with non-positive width or height has
 * zero area. Two boxes whose areas sum to at most the double-precision epsilon overlap fully
 * (jaccard distance 0, overlap 1).
 */
float rectOverlap(const std::array<double, 4>& a, const std::array<double, 4>& b) {
    const double areaA = a[2] * a[3];
    const double areaB = b[2] * b[3];
    double jaccardDistance = 0.0;
    if((areaA + areaB) > std::numeric_limits<double>::epsilon()) {
        double intersectionArea = 0.0;
        // An empty (non-positive width or height) box has an empty intersection, matching
        // cv::Rect_ operator&.
        if(a[2] > 0 && a[3] > 0 && b[2] > 0 && b[3] > 0) {
            const double xMin = std::max(a[0], b[0]);
            const double yMin = std::max(a[1], b[1]);
            const double xMax = std::min(a[0] + a[2], b[0] + b[2]);
            const double yMax = std::min(a[1] + a[3], b[1] + b[3]);
            if(xMax > xMin && yMax > yMin) {
                intersectionArea = (xMax - xMin) * (yMax - yMin);
            }
        }
        jaccardDistance = 1.0 - intersectionArea / (areaA + areaB - intersectionArea);
    }
    return 1.0f - static_cast<float>(jaccardDistance);
}

}  // namespace

std::vector<int> nmsBoxes(
    const std::vector<std::array<double, 4>>& bboxes, const std::vector<float>& scores, float scoreThreshold, float nmsThreshold, int topK, float eta) {
    DAI_CHECK_V(bboxes.size() == scores.size(), "NMS boxes and scores must have the same length, got {} boxes and {} scores.", bboxes.size(), scores.size());
    DAI_CHECK(scoreThreshold >= 0.0f, "NMS score threshold must be non-negative.");
    DAI_CHECK(nmsThreshold >= 0.0f, "NMS threshold must be non-negative.");
    DAI_CHECK(eta > 0.0f, "NMS eta must be positive.");

    // Get the topK highest scores (with corresponding indices), matching OpenCV's
    // GetMaxScoreIndex: filter with a strict greater-than, stable-sort descending by score and
    // truncate to topK when topK > 0.
    std::vector<std::pair<float, int>> scoreIndexPairs;
    for(std::size_t i = 0; i < scores.size(); i++) {
        if(scores[i] > scoreThreshold) {
            scoreIndexPairs.emplace_back(scores[i], static_cast<int>(i));
        }
    }
    std::stable_sort(
        scoreIndexPairs.begin(), scoreIndexPairs.end(), [](const std::pair<float, int>& a, const std::pair<float, int>& b) { return a.first > b.first; });
    if(topK > 0 && static_cast<std::size_t>(topK) < scoreIndexPairs.size()) {
        scoreIndexPairs.resize(static_cast<std::size_t>(topK));
    }

    // Greedy suppression, matching OpenCV's NMSFast_.
    float adaptiveThreshold = nmsThreshold;
    std::vector<int> indices;
    for(const auto& scoreIndexPair : scoreIndexPairs) {
        const int idx = scoreIndexPair.second;
        bool keep = true;
        for(std::size_t k = 0; k < indices.size() && keep; k++) {
            const int keptIdx = indices[k];
            const float overlap = rectOverlap(bboxes[idx], bboxes[keptIdx]);
            keep = overlap <= adaptiveThreshold;
        }
        if(keep) {
            indices.push_back(idx);
        }
        if(keep && eta < 1.0f && adaptiveThreshold > 0.5f) {
            adaptiveThreshold *= eta;
        }
    }
    return indices;
}

std::shared_ptr<dai::ImgDetections> createDetectionMessage(const std::vector<std::array<float, 4>>& bboxes,
                                                           const std::vector<float>& scores,
                                                           const std::vector<float>& angles,
                                                           const std::vector<std::uint32_t>& labels,
                                                           const std::vector<std::string>& labelNames,
                                                           const std::vector<std::vector<Keypoint>>& keypoints,
                                                           const std::vector<Edge>& keypointEdges) {
    auto message = std::make_shared<dai::ImgDetections>();
    if(bboxes.empty()) {
        return message;
    }

    const std::size_t numBboxes = bboxes.size();
    DAI_CHECK_V(scores.size() == numBboxes, "Scores should have same length as bboxes, got {} scores and {} bounding boxes.", scores.size(), numBboxes);
    DAI_CHECK_V(labels.empty() || labels.size() == numBboxes,
                "Labels should have same length as bboxes, got {} labels and {} bounding boxes.",
                labels.size(),
                numBboxes);
    DAI_CHECK_V(labelNames.empty() || labelNames.size() == numBboxes,
                "Label names should have same length as bboxes, got {} label names and {} bounding boxes.",
                labelNames.size(),
                numBboxes);
    DAI_CHECK_V(angles.empty() || angles.size() == numBboxes,
                "Angles should have same length as bboxes, got {} angles and {} bounding boxes.",
                angles.size(),
                numBboxes);
    for(const float angle : angles) {
        DAI_CHECK_V(angle >= -360.0f && angle <= 360.0f, "Angles should be between -360 and 360, got {}.", angle);
    }
    DAI_CHECK_V(keypoints.empty() || keypoints.size() == numBboxes,
                "Keypoints should have same length as bboxes, got {} keypoints and {} bounding boxes.",
                keypoints.size(),
                numBboxes);

    message->detections.reserve(numBboxes);
    for(std::size_t detectionIdx = 0; detectionIdx < numBboxes; detectionIdx++) {
        dai::ImgDetection detection;

        const float angle = angles.empty() ? 0.0f : angles[detectionIdx];
        const dai::RotatedRect boundingBox(
            dai::Point2f(bboxes[detectionIdx][0], bboxes[detectionIdx][1], true), dai::Size2f(bboxes[detectionIdx][2], bboxes[detectionIdx][3], true), angle);
        detection.setBoundingBox(boundingBox);
        detection.confidence = scores[detectionIdx];

        if(!labels.empty()) {
            detection.label = labels[detectionIdx];
            if(!labelNames.empty()) {
                detection.labelName = labelNames[detectionIdx];
            }
        }
        if(!keypoints.empty()) {
            detection.setKeypoints(keypoints[detectionIdx]);
            if(!keypointEdges.empty()) {
                detection.setEdges(keypointEdges);
            }
        }
        message->detections.push_back(std::move(detection));
    }

    return message;
}

}  // namespace DetectionUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
