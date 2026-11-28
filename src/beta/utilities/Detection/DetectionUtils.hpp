#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "depthai/common/Keypoint.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace DetectionUtils {

template <typename T>
T clip01(T value) {  // Like np.clip(value, 0, 1) in the precision of T; NaN propagates
    static_assert(std::is_floating_point<T>::value, "clip01 is only for floating-point values.");
    return std::min(std::max(value, static_cast<T>(0)), static_cast<T>(1));
}

std::vector<int> nmsBoxes(const std::vector<std::array<double, 4>>& bboxes,  // Like cv2.dnn.NMSBoxes; topK <= 0 = no limit; returns indices by descending score
                          const std::vector<float>& scores,
                          float scoreThreshold,
                          float nmsThreshold,
                          int topK = 0,
                          float eta = 1.0f);

std::shared_ptr<dai::ImgDetections> createDetectionMessage(const std::vector<std::array<float, 4>>& bboxes,  // bboxes: normalized [xCenter, yCenter, w, h]
                                                           const std::vector<float>& scores,
                                                           const std::vector<float>& angles = {},
                                                           const std::vector<std::uint32_t>& labels = {},
                                                           const std::vector<std::string>& labelNames = {},
                                                           const std::vector<std::vector<Keypoint>>& keypoints = {},
                                                           const std::vector<Edge>& keypointEdges = {},
                                                           const std::vector<std::uint8_t>& segmentationMask = {},
                                                           std::size_t segmentationMaskWidth = 0,
                                                           std::size_t segmentationMaskHeight = 0);

void setTransformationAndBounds(dai::ImgDetections& message, const dai::ImgTransformation& transformation);  // Bounds of rotated boxes need the frame size

}  // namespace DetectionUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
