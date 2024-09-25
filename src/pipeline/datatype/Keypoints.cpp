#include "depthai/pipeline/datatype/Keypoints.hpp"

namespace dai {

const std::vector<Keypoint>& Keypoints::getKeypoints() const {
    return keypoints;
}

Keypoints& Keypoints::setKeypoints(const std::vector<Keypoint>& keypoints) {
    this->keypoints = keypoints;

    return *this;
}

// 3D keypoints
Keypoints& Keypoints::setKeypoints(const std::vector<Point3f>& points) {
    std::vector<Keypoint> keypoints = std::vector<Keypoint>(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        keypoints[i].x = points[i].x;
        keypoints[i].y = points[i].y;
        keypoints[i].z = points[i].z;
    };

    return this->setKeypoints(keypoints);
}
Keypoints& Keypoints::setKeypoints(const std::vector<Point3f>& points, const std::vector<float>& scores) {
    if (points.size() != scores.size()) {
        throw std::invalid_argument("Keypoints and scores should have the same length. Got "
                                    + std::to_string(points.size())
                                    + " keypoints and "
                                    + std::to_string(scores.size())
                                    + " scores.");
    }

    for (const auto& score: scores) {
        if (0 > score || score > 1) {
            throw std::invalid_argument("Scores should only contain values between 0 and 1.");
        }
    }

    std::vector<Keypoint> keypoints = std::vector<Keypoint>(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        keypoints[i].x = points[i].x;
        keypoints[i].y = points[i].y;
        keypoints[i].z = points[i].z;
        keypoints[i].confidence = scores[i];
    };

    return this->setKeypoints(keypoints);
}
Keypoints& Keypoints::setKeypoints(const std::vector<Point3f>& points, const std::vector<float>& scores, float confidenceThreshold) {
    if (points.size() != scores.size()) {
        throw std::invalid_argument("Keypoints and scores should have the same length. Got "
                                    + std::to_string(points.size())
                                    + " keypoints and "
                                    + std::to_string(scores.size())
                                    + " scores.");
    }

    if (0 > confidenceThreshold || confidenceThreshold > 1) {
        throw std::invalid_argument("Confidence threshold should be between 0 and 1. Got " + std::to_string(confidenceThreshold) + ".");
    }

   std::vector<dai::Point3f> filteredPoints = std::vector<dai::Point3f>();
   std::vector<float> filteredScores = std::vector<float>();

    for (size_t i = 0; i < points.size(); i++) {
        if (scores[i] >= confidenceThreshold) {
            filteredPoints.push_back(points[i]);
            filteredScores.push_back(scores[i]);
        }
    }

    return this->setKeypoints(filteredPoints, filteredScores);
}

// 2D keypoints
Keypoints& Keypoints::setKeypoints(const std::vector<Point2f>& points) {
    std::vector<Keypoint> keypoints = std::vector<Keypoint>(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        keypoints[i].x = points[i].x;
        keypoints[i].y = points[i].y;
    };

    return this->setKeypoints(keypoints);
}
Keypoints& Keypoints::setKeypoints(const std::vector<Point2f>& points, const std::vector<float>& scores) {
    if (points.size() != scores.size()) {
        throw std::invalid_argument("Keypoints and scores should have the same length. Got "
                                    + std::to_string(points.size())
                                    + " keypoints and "
                                    + std::to_string(scores.size())
                                    + " scores.");
    }

    for (const auto& score: scores) {
        if (0 > score || score > 1) {
            throw std::invalid_argument("Scores should only contain values between 0 and 1.");
        }
    }

    std::vector<Keypoint> keypoints = std::vector<Keypoint>(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        keypoints[i].x = points[i].x;
        keypoints[i].y = points[i].y;
        keypoints[i].confidence = scores[i];
    };

    return this->setKeypoints(keypoints);
}
Keypoints& Keypoints::setKeypoints(const std::vector<Point2f>& points, const std::vector<float>& scores, float confidenceThreshold) {
    if (points.size() != scores.size()) {
        throw std::invalid_argument("Keypoints and scores should have the same length. Got "
                                    + std::to_string(points.size())
                                    + " keypoints and "
                                    + std::to_string(scores.size())
                                    + " scores.");
    }

    if (0 > confidenceThreshold || confidenceThreshold > 1) {
        throw std::invalid_argument("Confidence threshold should be between 0 and 1. Got " + std::to_string(confidenceThreshold) + ".");
    }

   std::vector<dai::Point2f> filteredPoints = std::vector<dai::Point2f>();
   std::vector<float> filteredScores = std::vector<float>();

    for (size_t i = 0; i < points.size(); i++) {
        if (scores[i] >= confidenceThreshold) {
            filteredPoints.push_back(points[i]);
            filteredScores.push_back(scores[i]);
        }
    }

    return this->setKeypoints(filteredPoints, filteredScores);
}
 
}  // namespace dai
