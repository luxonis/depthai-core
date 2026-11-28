#pragma once  // Ported from depthai-nodes utils/fastsam.py, yolo.py, nms.py, masks_utils.py; parts from Ultralytics FastSAM

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
namespace FastSAMUtils {

using DetectionRow = std::array<double, 10>;  // [x1, y1, x2, y2, conf, cls, head, anchor, x, y], box in model-input pixels

struct PromptConfig {
    std::string prompt = "everything";
    std::optional<std::pair<std::int32_t, std::int32_t>> points;
    std::optional<std::int32_t> pointLabel;           // 1 adds the masks containing the point, 0 subtracts them
    std::optional<std::array<std::int32_t, 4>> bbox;  // (x1, y1, x2, y2) in model-input pixels
};

struct FastsamResult {
    std::vector<std::uint8_t> mergedMask;  // Pixel = instance index, 255 = background
    std::size_t width = 0;
    std::size_t height = 0;
    std::size_t maskCount = 0;
};

std::vector<std::size_t> nms(const std::vector<std::array<float, 4>>& boxes, const std::vector<float>& scores, float nmsThreshold);

void adjustBboxesToImageBorder(std::vector<DetectionRow>& rows, std::int64_t imgHeight, std::int64_t imgWidth, std::int64_t threshold = 20);

std::vector<std::size_t> bboxIou(  // Source (rewritten to numpy): https://github.com/ultralytics/ultralytics/blob/main/ultralytics/models/fastsam/utils.py#L30
    const std::array<double, 4>& box1,
    std::vector<DetectionRow>& rows,
    std::int64_t imgHeight,
    std::int64_t imgWidth,
    double iouThres = 0.9);

std::vector<DetectionRow> decodeFastsamOutput(const std::vector<ClassificationUtils::ShapedTensorData>& yoloOutputs,  // yoloOutputs: NCHW, sorted by layer name
                                              std::int64_t imgHeight,
                                              std::int64_t imgWidth,
                                              float confThres,
                                              float iouThres,
                                              std::int32_t numClasses);

std::vector<float> buildMaskCoeffs(const std::vector<DetectionRow>& rows,  // maskOutputs: sorted by layer name, index-aligned with yoloOutputs
                                   const std::vector<ClassificationUtils::ShapedTensorData>& maskOutputs,
                                   std::size_t protosLen);

std::vector<std::uint8_t> processMasks(const std::vector<DetectionRow>& rows,
                                       const std::vector<float>& maskCoeffs,
                                       const std::vector<float>& protos,
                                       std::size_t protosLen,
                                       std::size_t protoHeight,
                                       std::size_t protoWidth,
                                       std::size_t outHeight,
                                       std::size_t outWidth,
                                       float maskConf);

std::vector<std::uint8_t> boxPrompt(  // Numpy port of https://github.com/ultralytics/ultralytics/blob/main/ultralytics/models/fastsam/prompt.py#L286
    const std::vector<std::uint8_t>& masks,
    std::size_t maskCount,
    std::size_t height,
    std::size_t width,
    const std::array<std::int32_t, 4>& bbox);

std::vector<std::uint8_t> pointPrompt(const std::vector<std::uint8_t>& masks,  // Numpy port of Ultralytics fastsam/prompt.py point_prompt() + format_results()
                                      std::size_t maskCount,
                                      std::size_t height,
                                      std::size_t width,
                                      const std::pair<std::int32_t, std::int32_t>& point,
                                      std::int32_t pointLabel);

std::vector<std::uint8_t> mergeMasks(const std::vector<std::uint8_t>& masks, std::size_t maskCount, std::size_t height, std::size_t width);

FastsamResult computeFastsamMask(const std::vector<ClassificationUtils::ShapedTensorData>& yoloOutputs,
                                 const std::vector<ClassificationUtils::ShapedTensorData>& maskOutputs,
                                 const ClassificationUtils::ShapedTensorData& protos,
                                 std::size_t protosLen,
                                 float confThreshold,
                                 std::int32_t numClasses,
                                 float iouThreshold,
                                 float maskConf,
                                 const PromptConfig& promptConfig);

}  // namespace FastSAMUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
