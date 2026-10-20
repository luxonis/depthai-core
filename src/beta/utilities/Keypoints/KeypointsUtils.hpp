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
 * @brief Normalized keypoints and their confidence scores extracted from HRNet heatmaps.
 */
struct HrnetKeypoints {
    /// Keypoint coordinates of shape (numKeypoints, 2), normalized by the heatmap size.
    KeypointCoordinates coordinates;
    /// Per-keypoint confidence scores, clipped to [0, 1], scores.size() == numKeypoints.
    std::vector<float> scores;
};

/**
 * @brief Normalized keypoints and their confidence scores extracted from SuperAnimal heatmaps.
 */
struct SuperAnimalKeypoints {
    /// Keypoint coordinates of shape (numKeypoints, 2), normalized by the scale factor.
    KeypointCoordinates coordinates;
    /// Per-keypoint confidence scores taken from the heatmap maxima, not clipped, scores.size() == numKeypoints.
    std::vector<float> scores;
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
 * @brief Extract normalized keypoints and confidence scores from HRNet heatmaps.
 *
 * The heatmap tensor must be provided in (numKeypoints, height, width) orientation, stored
 * row-major. A leading dimension of 1 (typically the batch dimension) is dropped exactly once;
 * the remaining tensor must be 3D, so a batch of more than 1 is rejected. The number of
 * keypoints and the heatmap size are derived from the tensor shape.
 *
 * Per heatmap, the score is the maximum value clipped to [0, 1] and the keypoint is the (x, y)
 * position of the first maximum in row-major scan order (matching np.argmax tie behavior),
 * normalized by the heatmap (width, height).
 *
 * @param values Heatmap tensor values, stored row-major over dims.
 * @param dims Heatmap tensor shape.
 * @return Normalized keypoints of shape (numKeypoints, 2) and their scores.
 */
HrnetKeypoints computeHrnetKeypoints(const std::vector<float>& values, const std::vector<std::size_t>& dims);

/**
 * @brief Extract normalized keypoints and confidence scores from SuperAnimal heatmaps.
 *
 * The heatmap tensor must be a 4D tensor in (batch, height, width, numKeypoints) orientation,
 * stored row-major, with a batch size of exactly 1 and a non-empty heatmap. The number of
 * keypoints is derived from the tensor's last dimension.
 *
 * Per keypoint, the score is the heatmap value at the position of the first maximum in
 * row-major scan order (matching np.argmax tie behavior), taken as-is without clipping; scores
 * outside [0, 1] are rejected later by createKeypointsMessage(), matching the source parser.
 * The keypoint is the (x, y) position of that maximum mapped to input-image pixels with a
 * 0.5-pixel center offset, x = (col + 0.5) * scaleFactor / width and
 * y = (row + 0.5) * scaleFactor / height, then normalized by the scale factor. The coordinates
 * are not clipped.
 *
 * @param values Heatmap tensor values, stored row-major over dims.
 * @param dims Heatmap tensor shape, (batch, height, width, numKeypoints).
 * @param scaleFactor Scale factor the keypoint coordinates are scaled and normalized by, must be
 *                    greater than 0.
 * @return Normalized keypoints of shape (numKeypoints, 2) and their scores.
 */
SuperAnimalKeypoints computeSuperAnimalKeypoints(const std::vector<float>& values, const std::vector<std::size_t>& dims, float scaleFactor);

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
