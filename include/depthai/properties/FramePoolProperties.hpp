#pragma once

#include <cstdint>

#include "depthai/utility/Serialization.hpp"

namespace dai {

/** Configuration of an individual frame pool. Zero means automatic for each field.
 * Resolved values satisfy maxPoolSize >= outputFrameSize * numFramesPool.
 * Nodes with multiple pools apply this configuration to each pool independently;
 * maxPoolSize is not a limit on the node's aggregate memory usage.
 */
struct FramePoolProperties {
    /// Capacity of one frame in bytes. Automatic uses the required buffer size, including backend padding.
    std::uint64_t outputFrameSize = 0;
    /// Number of frames. Automatic targets 100 ms at the expected FPS (4..30 frames), reduced for large frames by a soft 64 MiB budget.
    std::uint32_t numFramesPool = 0;
    /// Maximum bytes per pool. Automatic uses the resolved frame size times the resolved count.
    std::uint64_t maxPoolSize = 0;

    DEPTHAI_SERIALIZE(FramePoolProperties, outputFrameSize, numFramesPool, maxPoolSize);
};

}  // namespace dai
