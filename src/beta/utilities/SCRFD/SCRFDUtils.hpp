#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace dai {
namespace beta {
namespace utilities {
namespace SCRFDUtils {

/// Number of keypoints per SCRFD detection. SCRFD models predict 5 keypoints (10 coordinate
/// values) per anchor; this is part of the SCRFD output contract, not a per-model value.
constexpr std::size_t NUM_KEYPOINTS = 5;

/// Number of keypoint coordinate values per SCRFD anchor (NUM_KEYPOINTS (x, y) pairs).
constexpr std::size_t KPS_VALUES_PER_ANCHOR = 2 * NUM_KEYPOINTS;

/// Number of bounding box distance values per SCRFD anchor (left, top, right, bottom).
constexpr std::size_t BBOX_VALUES_PER_ANCHOR = 4;

/**
 * @brief Decoded SCRFD detections.
 *
 * All coordinates are normalized to [0, 1] against the model input size and clipped.
 * Detections are ordered by descending score after non-maximum suppression.
 */
struct ScrfdDetections {
    /// Bounding boxes as (xCenter, yCenter, width, height), index-aligned with scores.
    std::vector<std::array<float, 4>> bboxes;
    /// Confidence scores of the detections.
    std::vector<float> scores;
    /// Keypoints per detection as NUM_KEYPOINTS (x, y) pairs, index-aligned with scores.
    std::vector<std::vector<std::array<float, 2>>> keypoints;
};

/**
 * @brief Compute the anchor centers for one stride, mirroring the source compute_anchor_centers.
 *
 * The anchor grid is (inputHeight / stride) rows by (inputWidth / stride) columns (integer
 * division). Centers are (x, y) = (column * stride, row * stride) in row-major order over
 * (row, column); when numAnchors is greater than 1, every center is duplicated numAnchors
 * times consecutively. A numAnchors of 1 or less yields the unduplicated grid, mirroring the
 * source's num_anchors > 1 duplication condition.
 *
 * @param stride Feature stride, must be positive.
 * @param inputWidth Model input image width.
 * @param inputHeight Model input image height.
 * @param numAnchors Number of anchors per grid position.
 * @return Anchor centers as (x, y) pairs.
 */
std::vector<std::array<float, 2>> computeAnchorCenters(std::int64_t stride, std::uint32_t inputWidth, std::uint32_t inputHeight, std::int64_t numAnchors);

/**
 * @brief Decode distance predictions to bounding boxes, mirroring the source distance2bbox.
 *
 * Each row of distance holds the (left, top, right, bottom) distances from the anchor center;
 * the decoded box is (x1, y1, x2, y2) = (cx - left, cy - top, cx + right, cy + bottom),
 * computed in single precision.
 *
 * @param points Anchor centers as (x, y) pairs.
 * @param distance Distances, BBOX_VALUES_PER_ANCHOR per point, row-major, index-aligned with
 *                 points.
 * @return Decoded boxes as (x1, y1, x2, y2).
 */
std::vector<std::array<float, 4>> distance2Bbox(const std::vector<std::array<float, 2>>& points, const std::vector<float>& distance);

/**
 * @brief Decode distance predictions to keypoints, mirroring the source distance2kps.
 *
 * Each row of distance holds interleaved (dx, dy) offsets from the anchor center; the decoded
 * keypoint pair i is (cx + distance[2 * i], cy + distance[2 * i + 1]), computed in single
 * precision. The source indexes the anchor center with points[:, i % 2] where i is the even
 * coordinate index, which always selects (cx, cy); that behavior is preserved.
 *
 * @param points Anchor centers as (x, y) pairs.
 * @param distance Distances, valuesPerPoint per point, row-major, index-aligned with points.
 * @param valuesPerPoint Number of coordinate values per point, must be a positive multiple of 2.
 * @return Decoded keypoint coordinates, valuesPerPoint per point, row-major.
 */
std::vector<float> distance2Kps(const std::vector<std::array<float, 2>>& points, const std::vector<float>& distance, std::size_t valuesPerPoint);

/**
 * @brief Greedy non-maximum suppression, an exact port of the source SCRFD numpy nms
 * (depthai_nodes utils/nms.py), which differs from cv2.dnn.NMSBoxes.
 *
 * Rows of dets are [x1, y1, x2, y2, score]. Box areas and intersections use the +1 offset
 * semantics of the original SCRFD implementation: area = (x2 - x1 + 1) * (y2 - y1 + 1) and
 * intersection sides are max(0, right - left + 1). Candidates are ordered by descending score
 * with numpy's values.argsort()[::-1] tie ordering (generic portable scalar introsort,
 * reversed); a candidate survives suppression against a kept box when the intersection over
 * union is less than or equal to the threshold. All arithmetic is single precision.
 *
 * @param dets Detections as rows of [x1, y1, x2, y2, score].
 * @param nmsThreshold Non-maximum suppression (IoU) threshold, inclusive for keeping.
 * @return Indices of the kept rows in descending score order.
 */
std::vector<std::int64_t> nms(const std::vector<std::array<float, 5>>& dets, float nmsThreshold);

/**
 * @brief Decode SCRFD outputs into final detections, mirroring the source decode_scrfd plus
 * compute_scrfd_detections.
 *
 * Per stride, the bounding box and keypoint distances are multiplied by the stride and decoded
 * against the stride's anchor centers with distance2Bbox and distance2Kps; positions whose
 * score is greater than or equal to scoreThreshold (inclusive) are kept. The kept candidates of
 * all strides are concatenated in stride order, sorted by descending score with numpy's
 * values.argsort()[::-1] tie ordering, and suppressed with the SCRFD nms. The kept boxes and
 * keypoints are normalized in double precision, x coordinates by inputWidth and y coordinates
 * by inputHeight, mirroring the source's float32/int64 -> float64 promotion. Keypoints are
 * clipped to [0, 1]; boxes are clipped to [0, 1], validated to satisfy x1 <= x2 and y1 <= y2,
 * converted from (x1, y1, x2, y2) to center (x, y, width, height) and clipped to [0, 1] again.
 *
 * Validates that per-stride vectors are index-aligned with featStrideFpn, that every stride's
 * bbox and kps value counts match the score count times BBOX_VALUES_PER_ANCHOR and
 * KPS_VALUES_PER_ANCHOR, and that every stride's score count matches its anchor center count
 * (the source numpy broadcast requirement).
 *
 * @param bboxesPerStride Raw bounding box distances per stride, BBOX_VALUES_PER_ANCHOR values
 *                        per anchor, row-major.
 * @param scoresPerStride Raw scores per stride, one value per anchor.
 * @param kpsPerStride Raw keypoint distances per stride, KPS_VALUES_PER_ANCHOR values per
 *                     anchor, row-major.
 * @param featStrideFpn Feature strides, index-aligned with the per-stride vectors, must not be
 *                      empty.
 * @param inputWidth Model input image width the x coordinates are normalized by.
 * @param inputHeight Model input image height the y coordinates are normalized by.
 * @param scoreThreshold Confidence score threshold (inclusive).
 * @param nmsThreshold Non-maximum suppression (IoU) threshold.
 * @param anchors Anchor centers per stride, index-aligned with featStrideFpn.
 * @return Decoded detections ordered by descending score.
 */
ScrfdDetections computeScrfdDetections(const std::vector<std::vector<float>>& bboxesPerStride,
                                       const std::vector<std::vector<float>>& scoresPerStride,
                                       const std::vector<std::vector<float>>& kpsPerStride,
                                       const std::vector<std::int64_t>& featStrideFpn,
                                       std::uint32_t inputWidth,
                                       std::uint32_t inputHeight,
                                       float scoreThreshold,
                                       float nmsThreshold,
                                       const std::vector<std::vector<std::array<float, 2>>>& anchors);

}  // namespace SCRFDUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
