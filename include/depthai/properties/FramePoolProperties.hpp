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
    /// Number of frames: 0 selects automatic, otherwise at least 2. Automatic initial counts use platform defaults and may use a 200 ms FPS target.
    /// Platforms supporting growth may add frames within the byte budget when downstream consumers retain the initial allocation.
    std::uint32_t numFramesPool = 0;
    /// Maximum payload bytes per pool, including any growth. Automatic budgets may expand to fit the platform's initial count and algorithm minimum.
    /// An explicit limit must fit at least two frames and the algorithm minimum.
    std::uint64_t maxPoolSize = 0;

    DEPTHAI_SERIALIZE(FramePoolProperties, outputFrameSize, numFramesPool, maxPoolSize);
};

}  // namespace dai
