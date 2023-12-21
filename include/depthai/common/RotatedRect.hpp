#pragma once

// std
#include <cstdint>

// shared
#include "depthai/common/Point2f.hpp"
#include "depthai/common/Size2f.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/// RotatedRect structure
struct RotatedRect {
    Point2f center;
    Size2f size;
    /// degrees, increasing clockwise
    float angle = 0.f;
};

DEPTHAI_SERIALIZE_EXT(RotatedRect, center, size, angle);

}  // namespace dai
