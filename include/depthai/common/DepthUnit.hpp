#pragma once

#include <cstdint>

namespace dai {

enum class SIPrefix : int32_t { GIGA, MEGA, KILO, DEKA, DEFAULT, CENTI, MILLI };
/**
 * Measurement unit for depth and calibration data.
 */
enum class LengthUnit : int32_t { METER, CENTIMETER, MILLIMETER, INCH, FOOT, CUSTOM };
// Backward-compatible aliases.
using DepthUnit = LengthUnit;

constexpr float getSIPrefixMultiplier(SIPrefix unit);

constexpr float getLengthUnitMultiplier(LengthUnit unit) {
    switch(unit) {
        case LengthUnit::METER:
            return getSIPrefixMultiplier(SIPrefix::DEFAULT);
        case LengthUnit::CENTIMETER:
            return getSIPrefixMultiplier(SIPrefix::CENTI);
        case LengthUnit::MILLIMETER:
            return getSIPrefixMultiplier(SIPrefix::MILLI);
        case LengthUnit::INCH:
            return 39.3701f;
        case LengthUnit::FOOT:
            return 3.28084f;
        case LengthUnit::CUSTOM:
            return 1.0f;  // CUSTOM multiplier should be handled separately
        default:
            return 1.0f;
    }
}

constexpr float getSIPrefixMultiplier(SIPrefix unit) {
    switch(unit) {
        case SIPrefix::GIGA:
            return 1 / 1000000000.0f;
        case SIPrefix::MEGA:
            return 1 / 1000000.0f;
        case SIPrefix::KILO:
            return 1 / 1000.0f;
        case SIPrefix::DEKA:
            return 1 / 10.0f;
        case SIPrefix::DEFAULT:
            return 1.0f;
        case SIPrefix::CENTI:
            return 100.0f;
        case SIPrefix::MILLI:
            return 1000.0f;
        default:
            return 1.0f;
    }
}

}  // namespace dai
