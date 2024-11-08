#pragma once

// std
#include <cstdint>

// shared
#include "depthai/common/Point2f.hpp"
#include "depthai/common/Rect.hpp"
#include "depthai/common/Size2f.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/// RotatedRect structure
struct RotatedRect {
    Point2f center;
    Size2f size;
    /// degrees, increasing clockwise
    float angle = 0.f;

    RotatedRect() = default;
    RotatedRect(const Point2f& center, const Size2f& size, float angle) : center(center), size(size), angle(angle) {}
    RotatedRect(const Rect& rect, float angle) : center(rect.x + rect.width / 2.0f, rect.y + rect.height / 2.0f), size(rect.width, rect.height), angle(angle) {}

    /**
     * Normalize the rotated rectangle. The normalized rectangle will have center and size coordinates in range [0,1]
     * @return Normalized rotated rectangle
     */
    RotatedRect normalize(unsigned int width, unsigned int height) const {
        RotatedRect normalized = *this;
        normalized.center.x /= width;
        normalized.center.y /= height;
        normalized.size.width /= width;
        normalized.size.height /= height;
        return normalized;
    }

    /**
     * Denormalize the rotated rectangle. The denormalized rectangle will have center and size coordinates in range [0, width] and [0, height]
     * @return Denormalized rotated rectangle
     */
    RotatedRect denormalize(unsigned int width, unsigned int height) const {
        RotatedRect denormalized = *this;
        denormalized.center.x *= width;
        denormalized.center.y *= height;
        denormalized.size.width *= width;
        denormalized.size.height *= height;
        return denormalized;
    }

    /**
     * Get the 4 corner points of the rotated rectangle
     * @return 4 corner points
     */
    std::array<dai::Point2f, 4> getPoints() const {
        float angleRad = angle * 3.14159265358979323846f / 180.0f;

        float halfWidth = size.width / 2.0f;
        float halfHeight = size.height / 2.0f;

        // Precompute sin and cos of the angle
        float cosAngle = std::cos(angleRad);
        float sinAngle = std::sin(angleRad);

        // Corners relative to the center before rotation
        std::array<dai::Point2f, 4> corners;
        corners[0] = {-halfWidth, -halfHeight};
        corners[1] = {halfWidth, -halfHeight};
        corners[2] = {halfWidth, halfHeight};
        corners[3] = {-halfWidth, halfHeight};

        // Rotate each corner and translate it back to the center position
        for(auto& corner : corners) {
            float x = corner.x;
            float y = corner.y;
            corner.x = x * cosAngle - y * sinAngle + center.x;
            corner.y = x * sinAngle + y * cosAngle + center.y;
        }

        return corners;
    }
    /**
     * Returns the outer non-rotated rectangle
     * @return [minx, miny, maxx, maxy]
     */
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
