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
    /// Number of frames: 0 selects automatic, otherwise at least 2. Automatic targets 100 ms (2..30 frames) within a platform budget.
    std::uint32_t numFramesPool = 0;
    /// Maximum bytes per pool. Automatic budgets expand to fit two buffers; an explicit limit must fit at least two.
    std::uint64_t maxPoolSize = 0;

    DEPTHAI_SERIALIZE(FramePoolProperties, outputFrameSize, numFramesPool, maxPoolSize);
};

}  // namespace dai
