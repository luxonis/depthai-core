#pragma once

#include <cstdint>
#include <ostream>
#include <string>

namespace dai {

/// IPP processing preset for RVC4 ToF (replaces ImageFiltersPresetMode on host).
/// OFF is internal (raw IPP, all filters bypassed) and not part of the BETA public surface.
enum class ToFPreset : uint32_t {
    OFF,
    LOW_RANGE,
    MID_RANGE,
    HIGH_RANGE,
    FAST_OBJECTS,
};

inline std::string toString(ToFPreset preset) {
    switch(preset) {
        case ToFPreset::OFF:
            return "OFF";
        case ToFPreset::LOW_RANGE:
            return "LOW_RANGE";
        case ToFPreset::MID_RANGE:
            return "MID_RANGE";
        case ToFPreset::HIGH_RANGE:
            return "HIGH_RANGE";
        case ToFPreset::FAST_OBJECTS:
            return "FAST_OBJECTS";
    }
    return "UNKNOWN";
}

}  // namespace dai

inline std::ostream& operator<<(std::ostream& out, dai::ToFPreset preset) {
    return out << dai::toString(preset);
}
