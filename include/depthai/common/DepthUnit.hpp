#pragma once

#include <cstdint>

namespace dai {

/**
 * Measurement unit for depth data
 */
enum class DepthUnit : int32_t { METER, CENTIMETER, MILLIMETER, INCH, FOOT, CUSTOM };

constexpr float getDepthUnitMultiplier(DepthUnit unit) {
    switch(unit) {
        case DepthUnit::METER:
            return 1.0f;
        case DepthUnit::CENTIMETER:
            return 100.0f;
        case DepthUnit::MILLIMETER:
            return 1000.0f;
        case DepthUnit::INCH:
            return 39.3701f;
        case DepthUnit::FOOT:
            return 3.28084f;
        case DepthUnit::CUSTOM:
            return 1.0f;  // CUSTOM multiplier should be handled separately
        default:
            return 1.0f;
    }
}

}  // namespace dai
