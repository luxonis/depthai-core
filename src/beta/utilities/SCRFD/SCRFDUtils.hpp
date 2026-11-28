#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace dai {
namespace beta {
namespace utilities {
namespace SCRFDUtils {

constexpr std::size_t NUM_KEYPOINTS = 5;  // Fixed by the SCRFD output contract, not a per-model value

constexpr std::size_t KPS_VALUES_PER_ANCHOR = 2 * NUM_KEYPOINTS;

constexpr std::size_t BBOX_VALUES_PER_ANCHOR = 4;  // (left, top, right, bottom) distances

struct ScrfdDetections {                       // Coordinates normalized to [0, 1] of the model input size; descending score order after NMS
    std::vector<std::array<float, 4>> bboxes;  // (xCenter, yCenter, width, height), index-aligned with scores
    std::vector<float> scores;
    std::vector<std::vector<std::array<float, 2>>> keypoints;
};

std::vector<std::array<float, 2>> computeAnchorCenters(std::int64_t stride, std::uint32_t inputWidth, std::uint32_t inputHeight, std::int64_t numAnchors);

std::vector<std::array<float, 4>> distance2Bbox(const std::vector<std::array<float, 2>>& points, const std::vector<float>& distance);

std::vector<float> distance2Kps(const std::vector<std::array<float, 2>>& points, const std::vector<float>& distance, std::size_t valuesPerPoint);

std::vector<std::int64_t> nms(const std::vector<std::array<float, 5>>& dets, float nmsThreshold);  // Port of depthai_nodes utils/nms.py, not cv2.dnn.NMSBoxes

ScrfdDetections computeScrfdDetections(const std::vector<std::vector<float>>& bboxesPerStride,  // Mirrors the source decode_scrfd + compute_scrfd_detections
                                       const std::vector<std::vector<float>>& scoresPerStride,
                                       const std::vector<std::vector<float>>& kpsPerStride,
                                       const std::vector<std::int64_t>& featStrideFpn,
                                       std::uint32_t inputWidth,
                                       std::uint32_t inputHeight,
                                       float scoreThreshold,
                                       float nmsThreshold,
                                       int maxDetections,
                                       const std::vector<std::vector<std::array<float, 2>>>& anchors);

}  // namespace SCRFDUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
