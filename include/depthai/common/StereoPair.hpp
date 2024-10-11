#pragma once

#include "depthai-shared/common/StereoPair.hpp"

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
