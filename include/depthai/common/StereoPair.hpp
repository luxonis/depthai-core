#pragma once

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {
/**
 * Describes which camera sockets can be used for stereo and their baseline.
 *
 */
struct StereoPair {
    CameraBoardSocket left;
    CameraBoardSocket right;
    /**
     * Baseline in centimeters.
     */
    float baseline = -1;
    bool isVertical = false;
    DEPTHAI_SERIALIZE(StereoPair, left, right, baseline, isVertical);
};
}  // namespace dai

// Global namespace
inline std::ostream& operator<<(std::ostream& out, const dai::StereoPair& pair) {
    out << "{left: " << pair.left << ", ";
    out << "right: " << pair.right << ", ";
    out << "baseline: " << pair.baseline << ", ";
    out << "isVertical: " << pair.isVertical << "}";
    return out;
}

inline std::ostream& operator<<(std::ostream& out, const std::vector<dai::StereoPair>& pairs) {
    out << "[";
    for(size_t i = 0; i < pairs.size(); i++) {
        if(i != 0) {
            out << ", ";
        }
        out << pairs.at(i);
    }
    out << "]";

    return out;
}
