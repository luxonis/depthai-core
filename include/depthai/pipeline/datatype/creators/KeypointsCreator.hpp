#pragma once

#include <vector>

// project
#include "depthai/pipeline/datatype/Keypoints.hpp"

// shared
#include "depthai/common/Point2f.hpp"
#include "depthai/common/Point3f.hpp"

namespace dai {

/**
 * Keypoints message creator
 */

/**
 * Create a DepthAI message for 3D keypoints
 *
 * @param keypoints detected 3D keypoints
 * @param scores confidence scores for each keypoint
 * @param confidence_threshold confidence threshold
 *  
 * @returns keypoints message
 */
std::shared_ptr<Keypoints> createKeypointsMessage(const std::vector<Point3f> keypoints);
std::shared_ptr<Keypoints> createKeypointsMessage(const std::vector<Point3f> keypoints, const std::vector<float> scores, float confidence_threshold);

/**
 * Create a DepthAI message for 2D keypoints
 *
 * @param keypoints detected 2D keypoints
 * @param scores confidence scores for each keypoint
 * @param confidence_threshold confidence threshold
 *
 * @returns keypoints message
 */
std::shared_ptr<Keypoints> createKeypointsMessage(const std::vector<Point2f> keypoints);
std::shared_ptr<Keypoints> createKeypointsMessage(const std::vector<Point2f> keypoints, const std::vector<float> scores, float confidence_threshold);

}  // namespace dai
