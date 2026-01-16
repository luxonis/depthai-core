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
    /**
     * A rotated rectangle is specified by the center point, size, and rotation angle in degrees.
     */
    RotatedRect(const Point2f& center, const Size2f& size, float angle) : center(center), size(size), angle(angle) {
        if(size.isNormalized() != center.isNormalized()) {
            throw std::runtime_error("Cannot create RotatedRect with mixed normalization");
        }
    }
    /**
     * Construct RotatedRect from a non-rotated rectangle and an angle
     */
    RotatedRect(const Rect& rect, float angle = 0.f)
        : center(rect.x + rect.width / 2.0f, rect.y + rect.height / 2.0f, rect.isNormalized()),
          size(rect.width, rect.height, rect.isNormalized()),
          angle(angle) {}

    operator Rect() const {
        const auto [minx, miny, maxx, maxy] = getOuterRect();
        return Rect(minx, miny, maxx - minx, maxy - miny);
    }

    bool isNormalized() const {
        if(size.isNormalized() != center.isNormalized()) {
            throw std::runtime_error("Cannot determine normalization as size and center have mixed normalization.");
        }
        return size.isNormalized();
    }

    /**
     * Normalize the rotated rectangle. The normalized rectangle will have center and size coordinates in range [0,1]
     * @return Normalized rotated rectangle
     */
    RotatedRect normalize(unsigned int width, unsigned int height) const {
        if(width == 0 || height == 0) {
            throw std::runtime_error("Width and height must be non-zero to denormalize RotatedRect");
        }
        if(isNormalized()) return *this;
        RotatedRect normalized = *this;
        normalized.center = dai::Point2f(center.x / width, center.y / height, true);
        normalized.size = dai::Size2f(size.width / width, size.height / height, true);
        return normalized;
    }

    /**
     * Denormalize the rotated rectangle. The denormalized rectangle will have center and size coordinates in range [0, width] and [0, height]
     * @return Denormalized rotated rectangle
     */
    RotatedRect denormalize(unsigned int width, unsigned int height, bool force = false) const {
        if(width == 0 || height == 0) {
            throw std::runtime_error("Width and height must be non-zero to denormalize RotatedRect");
        }
        if(!force && !isNormalized()) return *this;
        RotatedRect denormalized = *this;
        denormalized.center = dai::Point2f(center.x * width, center.y * height, false);
        denormalized.size = dai::Size2f(size.width * width, size.height * height, false);
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

    /**
     * Returns the outer non-rotated rectangle in the COCO (xmin, ymin, width, height) format.
     * @return (top-left point, size)
     */
    std::tuple<dai::Point2f, dai::Size2f> getOuterXYWH() const {
        auto [minx, miny, maxx, maxy] = getOuterRect();
        return {dai::Point2f{minx, miny, isNormalized()}, dai::Size2f{maxx - minx, maxy - miny, isNormalized()}};
    }

    /**
     * Returns the outer non-rotated rectangle in the YOLO (xcenter, ycenter, width, height) format.
     * @return (center point, size)
     */
    std::tuple<dai::Point2f, dai::Size2f> getOuterCXCYWH() const {
        auto [minx, miny, maxx, maxy] = getOuterRect();
        return {dai::Point2f{(minx + maxx) / 2.0f, (miny + maxy) / 2.0f, isNormalized()}, dai::Size2f{maxx - minx, maxy - miny, isNormalized()}};
    }
};

DEPTHAI_SERIALIZE_EXT(RotatedRect, center, size, angle);

}  // namespace dai
