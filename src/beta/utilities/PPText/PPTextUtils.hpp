#pragma once  // Ported from depthai-nodes utils/ppdet.py and utils/bbox_format_converters.py

#include <array>
#include <cstddef>
#include <vector>

namespace dai {
namespace beta {
namespace utilities {
namespace PPTextUtils {

struct PPTextDetections {
    std::vector<std::array<float, 4>> bboxes;  // [xCenter, yCenter, width, height], normalized to the model input, clipped to [0, 1]
    std::vector<float> angles;                 // Degrees, rounded half to even
    std::vector<float> scores;
};

PPTextDetections parsePaddleDetectionOutputs(  // Throws a runtime error when built without OpenCV
    const std::vector<float>& predictions,
    const std::vector<std::size_t>& dims,
    float maskThreshold,
    float bboxThreshold,
    int maxDetections);

}  // namespace PPTextUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
