#pragma once

#include "depthai/common/EepromData.hpp"
#include "depthai/common/Rect.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
// #include <spdlog/spdlog.h>
// #include "Node.hpp"
// #include "PipelineBuilder.hpp"

namespace dai {
namespace gate {
namespace RectangleRemapUtils {

void clampRect(dai::Rect& toClamp, float min = 0.f, float max = 1.f);

double radians(double degrees);

// Scale the rectangle on both dimesions by scaleFactor
dai::Rect scaleRect(dai::Rect toScale, float scaleFactor);

}  // namespace RectangleRemapUtils
}  // namespace gate
}  // namespace dai
