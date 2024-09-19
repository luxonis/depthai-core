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

    std::array<dai::Point2f, 4> getPoints() const {
        float angleRad = angle * 3.14159265358979323846f / 180.0f;
        float cosA = std::cos(angleRad);
        float sinA = std::sin(angleRad);

        float w = size.width / 2;
        float h = size.height / 2;

        std::array<dai::Point2f, 4> points = {
            dai::Point2f(center.x - w * cosA + h * sinA, center.y - w * sinA + h * cosA),
            dai::Point2f(center.x + w * cosA + h * sinA, center.y + w * sinA + h * cosA),
            dai::Point2f(center.x + w * cosA - h * sinA, center.y + w * sinA - h * cosA),
            dai::Point2f(center.x - w * cosA - h * sinA, center.y - w * sinA - h * cosA),
        };

        return points;
    }
    std::array<float, 4> getOuterRect() const {
        auto points = getPoints();
        float minx = std::min({points[0].x, points[1].x, points[2].x, points[3].x});
        float maxx = std::max({points[0].x, points[1].x, points[2].x, points[3].x});
        float miny = std::min({points[0].y, points[1].y, points[2].y, points[3].y});
        float maxy = std::max({points[0].y, points[1].y, points[2].y, points[3].y});
        return {minx, miny, maxx, maxy};
    }
};

DEPTHAI_SERIALIZE_EXT(RotatedRect, center, size, angle);

}  // namespace dai
