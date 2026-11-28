#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "beta/utilities/Classification/ClassificationUtils.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace RFDETRUtils {

struct RfDetrDetections {
    std::vector<std::array<float, 4>> bboxes;  // (xCenter, yCenter, width, height), normalized to [0, 1], descending score order
    std::vector<float> scores;
    std::vector<std::uint32_t> labels;
    std::vector<std::string> labelNames;         // Index-aligned with bboxes; empty when no label names are configured
    std::vector<std::uint8_t> segmentationMask;  // Row-major (maskHeight, maskWidth); pixel = detection index, 255 = background; empty in detection mode
    std::size_t maskWidth = 0;
    std::size_t maskHeight = 0;
    std::size_t ignoredInstances = 0;  // Confident instances beyond the 255 the mask can encode; > 0: log the parser warning
};

RfDetrDetections computeRfDetrDetections(const ClassificationUtils::ShapedTensorData& boxesTensor,  // Mirrors the source compute_rfdetr_detections()
                                         const ClassificationUtils::ShapedTensorData& logitsTensor,
                                         const std::optional<ClassificationUtils::ShapedTensorData>& masksTensor,
                                         float confThreshold,
                                         int maxDetections,
                                         const std::vector<std::string>& labelNames,
                                         float maskConf,
                                         const std::optional<std::pair<std::uint32_t, std::uint32_t>>& inputSize);

}  // namespace RFDETRUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
