#pragma once

#include "depthai/common/EepromData.hpp"
#include "depthai/common/Rect.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai {
namespace gate {
namespace RectangleRemapUtils {

void clampRect(dai::Rect& toClamp, float min = 0.f, float max = 1.f);

double radians(double degrees);

dai::Rect scaleRect(dai::Rect toScale, float scaleFactor);

}  // namespace RectangleRemapUtils
}  // namespace gate
}  // namespace dai
