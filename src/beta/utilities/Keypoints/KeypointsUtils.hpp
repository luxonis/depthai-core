#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "depthai/beta/datatype/Keypoints.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace KeypointsUtils {

/**
 * @brief Keypoint coordinates of shape (numKeypoints, numCoords), stored row-major.
 */
struct KeypointCoordinates {
    /// Coordinate values in row-major order, values.size() == numKeypoints * numCoords.
    std::vector<float> values;
    /// Number of coordinates per keypoint: 2 for (x, y) or 3 for (x, y, z); 0 when empty.
    std::size_t numCoords = 0;

    /// Number of keypoints (rows).
    std::size_t getNumKeypoints() const {
        return numCoords == 0 ? 0 : values.size() / numCoords;
    }
};

/**
 * @brief Reshape and normalize a flattened keypoint tensor.
 *
 * The number of coordinates per keypoint is derived from the tensor size as
 * values.size() / nKeypoints and must be 2 or 3. Values are divided by the scale factor and
 * clipped to [0, 1].
 *
 * @param values Flattened keypoint tensor values.
 * @param nKeypoints Number of keypoints, must be greater than 0.
 * @param scaleFactor Scale factor to divide the keypoint coordinates by.
 * @return Keypoint coordinates of shape (nKeypoints, numCoords).
 */
KeypointCoordinates computeKeypoints(std::vector<float> values, std::int64_t nKeypoints, float scaleFactor);

/**
 * @brief Create a Keypoints message from keypoint coordinates.
 *
 * Validates that the coordinates have 2 or 3 coordinates per keypoint, that scores (when
 * provided) match the number of keypoints and lie in [0, 1], and that the confidence threshold
 * (when provided) lies in [0, 1]. When both scores and a confidence threshold are provided,
 * keypoints with score strictly below the threshold are dropped and the edges are filtered to
 * the kept keypoints and remapped to their new indices; edges referencing dropped or
 * out-of-range keypoint indices are dropped silently.
 *
 * Each kept keypoint carries its image coordinates (z is 0 for 2D keypoints), its score as
 * confidence (-1.0 when no scores are provided) and its label name selected from labelNames by
 * the keypoint's original index.
 *
 * @param coordinates Keypoint coordinates of shape (numKeypoints, numCoords).
 * @param scores Optional per-keypoint confidence scores in [0, 1], index-aligned with the
 *               keypoints.
 * @param confidenceThreshold Optional confidence threshold in [0, 1]; keypoints with score below
 *                            the threshold are dropped. Only applied when scores are provided.
 * @param labelNames Optional label names, indexed by the keypoint's original index. Empty when
 *                   not provided.
 * @param edges Optional skeleton edges as pairs of keypoint indices. Empty when not provided.
 * @return Keypoints message with the kept keypoints and the filtered, remapped edges.
 */
std::shared_ptr<Keypoints> createKeypointsMessage(const KeypointCoordinates& coordinates,
                                                  const std::optional<std::vector<float>>& scores = std::nullopt,
                                                  std::optional<float> confidenceThreshold = std::nullopt,
                                                  const std::vector<std::string>& labelNames = {},
                                                  const std::vector<Edge>& edges = {});

}  // namespace KeypointsUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
