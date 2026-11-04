#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

#include "depthai/properties/FramePoolProperties.hpp"

namespace dai {
namespace utility {

constexpr std::uint64_t DEFAULT_FRAME_POOL_BUDGET = 192ULL * 1024 * 1024;

inline void validateFramePoolCount(std::uint32_t count) {
    if(count == 1) throw std::invalid_argument("Frame pool numFramesPool must be 0 (automatic) or at least 2");
}


/** Resolve one pool without modifying the requested configuration.
 * Automatic count targets 100 ms of buffering (2..30 frames), limited to a soft
 * platform budget (192 MiB by default), expanded to fit at least two buffers.
 * Explicit counts override that soft budget. A supplied maxPoolSize
 * is a hard limit, and can reduce an automatic count down to max(2, minimumFrames).
 * Explicit values are never silently increased or decreased.
 */
inline FramePoolProperties resolveFramePool(const FramePoolProperties& requested,
                                            std::uint64_t requiredFrameSize,
                                            double expectedFps = 30.0,
                                            std::uint32_t minimumFrames = 2,
                                            std::uint64_t automaticBudget = DEFAULT_FRAME_POOL_BUDGET) {
    if(requiredFrameSize == 0 || minimumFrames == 0) throw std::invalid_argument("Frame pool requires nonzero frame size and minimum count");
    minimumFrames = std::max(minimumFrames, std::uint32_t{2});
    validateFramePoolCount(requested.numFramesPool);
    FramePoolProperties resolved = requested;
    if(resolved.outputFrameSize == 0) resolved.outputFrameSize = requiredFrameSize;
    if(resolved.outputFrameSize < requiredFrameSize) throw std::invalid_argument("Frame pool outputFrameSize is smaller than the required buffer");
    if(resolved.outputFrameSize > std::numeric_limits<std::size_t>::max()) throw std::overflow_error("Frame pool frame size exceeds addressable memory");

    if(resolved.numFramesPool == 0) {
        if(!std::isfinite(expectedFps) || expectedFps <= 0) expectedFps = 30.0;
        auto target = static_cast<std::uint32_t>(std::clamp(std::ceil(expectedFps * 0.1), 2.0, 30.0));
        const auto automaticMinimum = minimumFrames;
        if(!requested.maxPoolSize && resolved.outputFrameSize > std::numeric_limits<std::uint64_t>::max() / automaticMinimum)
            throw std::overflow_error("Frame pool automatic budget multiplication overflow");
        const auto budget = requested.maxPoolSize ? requested.maxPoolSize
                                                 : std::max(automaticBudget, resolved.outputFrameSize * automaticMinimum);
        const auto affordable = budget / resolved.outputFrameSize;
        resolved.numFramesPool = std::max(minimumFrames, static_cast<std::uint32_t>(std::min<std::uint64_t>(target, affordable)));
    }
    if(resolved.numFramesPool < minimumFrames) throw std::invalid_argument("Frame pool numFramesPool is below the algorithm's minimum count");
    if(resolved.outputFrameSize > std::numeric_limits<std::uint64_t>::max() / resolved.numFramesPool)
        throw std::overflow_error("Frame pool size multiplication overflow");
    const auto requiredPoolSize = resolved.outputFrameSize * resolved.numFramesPool;
    if(resolved.maxPoolSize == 0) resolved.maxPoolSize = requiredPoolSize;
    if(resolved.maxPoolSize < requiredPoolSize) throw std::invalid_argument("Frame pool requires maxPoolSize >= outputFrameSize * numFramesPool");
    if(requiredPoolSize > std::numeric_limits<std::size_t>::max()) throw std::overflow_error("Frame pool exceeds addressable memory");
    return resolved;
}

}  // namespace utility
}  // namespace dai
