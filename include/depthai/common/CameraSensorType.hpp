#pragma once

#include <nlohmann/json.hpp>
#include <string>

namespace dai {

/// Camera sensor type
enum class CameraSensorType : int32_t { AUTO = -1, COLOR = 0, MONO = 1, TOF = 2, THERMAL = 3 };

inline std::string toString(CameraSensorType type) {
    switch(type) {
        case dai::CameraSensorType::AUTO:
            return "AUTO";
        case dai::CameraSensorType::COLOR:
            return "COLOR";
        case dai::CameraSensorType::MONO:
            return "MONO";
        case dai::CameraSensorType::TOF:
            return "TOF";
        case dai::CameraSensorType::THERMAL:
            return "THERMAL";
        default:
            return "UNKNOWN";
    }
}

}  // namespace dai

// Global namespace
inline std::ostream& operator<<(std::ostream& out, const dai::CameraSensorType& type) {
    return out << dai::toString(type);
}
