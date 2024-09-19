#include "depthai/pipeline/datatype/Keypoints.hpp"

namespace dai {

const std::vector<Point3f>& Keypoints::getKeypoints() const {
    return keypoints;
}

// 3D keypoints
Keypoints& Keypoints::setKeypoints(const std::vector<Point3f>& keypoints) {
    this->keypoints = keypoints;

    return *this;
}
Keypoints& Keypoints::setKeypoints(const std::vector<Point3f>& keypoints, const std::vector<float>& scores, float confidenceThreshold) {
    if (keypoints.size() != scores.size()) {
        throw std::invalid_argument("Keypoints and scores should have the same length. Got "
                                    + std::to_string(keypoints.size())
                                    + " keypoints and "
                                    + std::to_string(scores.size())
                                    + " scores.");
    }

    for (const auto& score: scores) {
        if (0 > score || score > 1) {
            throw std::invalid_argument("Scores should only contain values between 0 and 1.");
        }
    }

    if (0 > confidenceThreshold || confidenceThreshold > 1) {
        throw std::invalid_argument("Confidence threshold should be between 0 and 1. Got " + std::to_string(confidenceThreshold) + ".");
    }

   std::vector<dai::Point3f> filteredPoints = std::vector<dai::Point3f>();

    for (size_t i = 0; i < keypoints.size(); i++) {
        if (scores[i] >= confidenceThreshold) {
            filteredPoints.push_back(keypoints[i]);
        }
    }

    return this->setKeypoints(filteredPoints);
}

// 2D keypoints
Keypoints& Keypoints::setKeypoints(const std::vector<Point2f>& keypoints) {
    std::vector<dai::Point3f> points3d = std::vector<dai::Point3f>(keypoints.size());
    for (size_t i = 0; i < keypoints.size(); i++) {
        points3d[i].x = keypoints[i].x;
        points3d[i].y = keypoints[i].y;
        points3d[i].z = 0;
    }

    return this->setKeypoints(points3d);
}
Keypoints& Keypoints::setKeypoints(const std::vector<Point2f>& keypoints, const std::vector<float>& scores, float confidenceThreshold) {
    std::vector<dai::Point3f> points3d = std::vector<dai::Point3f>(keypoints.size());
    for (size_t i = 0; i < keypoints.size(); i++) {
        points3d[i].x = keypoints[i].x;
        points3d[i].y = keypoints[i].y;
        points3d[i].z = 0;
    }

    return this->setKeypoints(points3d, scores, confidenceThreshold);
}
 
}  // namespace dai
