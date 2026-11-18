#pragma once

#include <cstdint>
#include <optional>
#include <utility>

#include "depthai/common/CameraFeatures.hpp"

namespace dai::utility {

// Match Camera::requestFullResolutionOutput(..., useHighestResolution=false).
inline std::optional<std::pair<uint32_t, uint32_t>> findDefaultFullResolutionSize(const CameraFeatures& cameraFeatures) {
    int32_t maxWidth = 0;
    int32_t maxHeight = 0;
    // Loop over all the configs and find the highest resolution that is not higher than 5000x4000
    for(const auto& config : cameraFeatures.configs) {
        // Only consider the full FOV configurations
        auto denormalizedFov = config.fov.denormalize(cameraFeatures.width, cameraFeatures.height);
        if((static_cast<int>(denormalizedFov.width) != cameraFeatures.width) || (static_cast<int>(denormalizedFov.height) != cameraFeatures.height)
           || denormalizedFov.x != 0 || denormalizedFov.y != 0) {
            continue;
        }
        if(config.width <= 5000 && config.height <= 4000) {
            if(config.width > maxWidth || config.height > maxHeight) {
                maxWidth = config.width;
                maxHeight = config.height;
            }
        }
    }
    if(maxWidth == 0 || maxHeight == 0) {
        return std::nullopt;
    }
    return std::make_pair(static_cast<uint32_t>(maxWidth), static_cast<uint32_t>(maxHeight));
}

}  // namespace dai::utility
