#pragma once

#include <nlohmann/json.hpp>

namespace dai {

/// Camera sensor type
enum class CameraSensorType : int32_t { AUTO = -1, COLOR = 0, MONO = 1, TOF = 2, THERMAL = 3 };

}  // namespace dai

// Global namespace
inline std::ostream& operator<<(std::ostream& out, const dai::CameraSensorType& type) {
    switch(type) {
        case dai::CameraSensorType::AUTO:
            out << "AUTO";
            break;
        case dai::CameraSensorType::COLOR:
            out << "COLOR";
            break;
        case dai::CameraSensorType::MONO:
            out << "MONO";
            break;
        case dai::CameraSensorType::TOF:
            out << "TOF";
            break;
        case dai::CameraSensorType::THERMAL:
            out << "THERMAL";
            break;
    }
    return out;
}
