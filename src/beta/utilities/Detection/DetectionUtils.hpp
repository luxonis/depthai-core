#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "depthai/common/Keypoint.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace DetectionUtils {

/**
 * @brief Greedy non-maximum suppression on axis-aligned boxes, compatible with cv2.dnn.NMSBoxes.
 *
 * Boxes are given as [x, y, width, height] with (x, y) the top-left corner. Candidates with a
 * score strictly greater than the score threshold are sorted by descending score (stable with
 * respect to the input order on ties) and, when topK is greater than 0, truncated to the topK
 * highest-scoring candidates before suppression. A candidate is kept when its overlap with every
 * previously kept box is less than or equal to the NMS threshold. The overlap is the intersection
 * area over the union area, rounded to single precision as OpenCV's rectOverlap does; two boxes
 * whose areas sum to at most the double-precision epsilon overlap fully (overlap 1). When eta is
 * less than 1, the adaptive NMS threshold is multiplied by eta after every kept candidate while
 * the threshold is greater than 0.5.
 *
 * @param bboxes Boxes as [x, y, width, height], index-aligned with scores.
 * @param scores Confidence scores, one per box.
 * @param scoreThreshold Score threshold, boxes with a score strictly greater than it are
 *                       considered; must be non-negative.
 * @param nmsThreshold NMS (IoU) threshold used to suppress overlapping boxes; must be
 *                     non-negative.
 * @param topK Maximum number of highest-scoring candidates entering suppression; 0 or negative
 *             means no limit. Defaults to 0.
 * @param eta Adaptive threshold decay factor; must be positive. Defaults to 1 (no decay).
 * @return Indices of the kept boxes in descending score order.
 */
std::vector<int> nmsBoxes(const std::vector<std::array<double, 4>>& bboxes,
                          const std::vector<float>& scores,
                          float scoreThreshold,
                          float nmsThreshold,
                          int topK = 0,
                          float eta = 1.0f);

/**
 * @brief Create an ImgDetections message, mirroring the source create_detection_message creator.
 *
 * Bounding boxes are given as [xCenter, yCenter, width, height] in normalized coordinates. Each
 * detection carries a normalized rotated-rectangle bounding box with the detection's angle in
 * degrees (0 when no angles are provided), its confidence score, its label (0 when no labels are
 * provided) and, when both labels and label names are provided, the label name selected by the
 * detection index. Empty bboxes yield an empty message.
 *
 * Validates that scores, and, when provided, angles, labels, label names and keypoints have the
 * same length as bboxes and that every angle lies in [-360, 360].
 *
 * A non-empty segmentation mask is stored on the message even when bboxes are empty, mirroring
 * the source creator's masks path. The mask is a row-major (height, width) uint8 array where a
 * pixel holds the index of the detection it belongs to and 255 marks background.
 *
 * @param bboxes Bounding boxes as [xCenter, yCenter, width, height], normalized.
 * @param scores Confidence scores, index-aligned with bboxes.
 * @param angles Optional angles of the bounding boxes in degrees, each in [-360, 360],
 *               index-aligned with bboxes. Empty when not provided.
 * @param labels Optional labels, index-aligned with bboxes. Empty when not provided.
 * @param labelNames Optional label names, indexed by the detection index. Only applied when
 *                   labels are provided as well. Empty when not provided.
 * @param keypoints Optional per-detection keypoints, index-aligned with bboxes. Empty when not
 *                  provided.
 * @param keypointEdges Optional keypoint skeleton edges as pairs of keypoint indices, applied to
 *                      every detection with keypoints. Empty when not provided.
 * @param segmentationMask Optional row-major instance segmentation mask of size
 *                         segmentationMaskWidth * segmentationMaskHeight. Empty when not
 *                         provided.
 * @param segmentationMaskWidth Segmentation mask width; only used when the mask is provided.
 * @param segmentationMaskHeight Segmentation mask height; only used when the mask is provided.
 * @return ImgDetections message with the detections.
 */
std::shared_ptr<dai::ImgDetections> createDetectionMessage(const std::vector<std::array<float, 4>>& bboxes,
                                                           const std::vector<float>& scores,
                                                           const std::vector<float>& angles = {},
                                                           const std::vector<std::uint32_t>& labels = {},
                                                           const std::vector<std::string>& labelNames = {},
                                                           const std::vector<std::vector<Keypoint>>& keypoints = {},
                                                           const std::vector<Edge>& keypointEdges = {},
                                                           const std::vector<std::uint8_t>& segmentationMask = {},
                                                           std::size_t segmentationMaskWidth = 0,
                                                           std::size_t segmentationMaskHeight = 0);

}  // namespace DetectionUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
