#include "depthai/pipeline/datatype/creators/KeypointsCreator.hpp"

namespace dai {

// 3D keypoints
std::shared_ptr<Keypoints> createKeypointsMessage(const std::vector<Point3f> keypoints) {
    std::shared_ptr<Keypoints> keypointsMsg = std::make_shared<Keypoints>();
    keypointsMsg->setKeypoints(keypoints);

    return keypointsMsg;
}

std::shared_ptr<Keypoints> createKeypointsMessage(const std::vector<Point3f> keypoints, const std::vector<float> scores, float confidence_threshold) {
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

    if (0 > confidence_threshold || confidence_threshold > 1) {
        throw std::invalid_argument("Confidence threshold should be between 0 and 1. Got " + std::to_string(confidence_threshold) + ".");
    }

   std::vector<dai::Point3f> filteredPoints = std::vector<dai::Point3f>();

    for (size_t i = 0; i < keypoints.size(); i++) {
        if (scores[i] >= confidence_threshold) {
            filteredPoints.push_back(keypoints[i]);
        }
    }

    return createKeypointsMessage(filteredPoints);
}


// 2D keypoints
std::shared_ptr<Keypoints> createKeypointsMessage(const std::vector<Point2f> keypoints) {
    std::vector<dai::Point3f> points3d = std::vector<dai::Point3f>(keypoints.size());
    for (size_t i = 0; i < keypoints.size(); i++) {
        points3d[i].x = keypoints[i].x;
        points3d[i].y = keypoints[i].y;
        points3d[i].z = 0;
    }

    return createKeypointsMessage(points3d);
}

std::shared_ptr<Keypoints> createKeypointsMessage(const std::vector<Point2f> keypoints, const std::vector<float> scores, float confidence_threshold) {
    std::vector<dai::Point3f> points3d = std::vector<dai::Point3f>(keypoints.size());
    for (size_t i = 0; i < keypoints.size(); i++) {
        points3d[i].x = keypoints[i].x;
        points3d[i].y = keypoints[i].y;
        points3d[i].z = 0;
    }

    return createKeypointsMessage(points3d, scores, confidence_threshold);
}

}  // namespace dai
