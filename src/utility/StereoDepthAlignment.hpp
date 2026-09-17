#pragma once

#include <stdexcept>

#include "depthai/common/StereoPair.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"

namespace dai {
namespace utility {

using StereoDepthAlign = StereoDepthConfig::AlgorithmControl::DepthAlign;

// Shared policy for initial/runtime stereo configuration and calibration anchoring.
constexpr StereoDepthAlign resolveStereoDepthAlignment(StereoDepthAlign alignment) {
    return alignment == StereoDepthAlign::AUTO ? StereoDepthAlign::RECTIFIED_LEFT : alignment;
}

inline CameraBoardSocket stereoDepthReferenceCamera(const StereoPair& pair, StereoDepthAlign alignment = StereoDepthAlign::AUTO) {
    switch(resolveStereoDepthAlignment(alignment)) {
        case StereoDepthAlign::LEFT:
        case StereoDepthAlign::RECTIFIED_LEFT:
            return pair.left;
        case StereoDepthAlign::RIGHT:
        case StereoDepthAlign::RECTIFIED_RIGHT:
            return pair.right;
        case StereoDepthAlign::AUTO:
        case StereoDepthAlign::CENTER:
            throw std::invalid_argument("Stereo depth alignment has no single reference camera.");
    }
    throw std::invalid_argument("Invalid stereo depth alignment.");
}

}  // namespace utility
}  // namespace dai
