#pragma once

#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/**
 * CpuUsage structure
 *
 * Average usage in percent and time span of the average (since last query)
 */
struct CpuUsage {
    /**
     *  Average CPU usage, expressed with a normalized value (0-1)
     */
    float average = 0.f;
    /**
     *  Time span in which the average was calculated in milliseconds
     */
    int32_t msTime = 0;
};

DEPTHAI_SERIALIZE_EXT(CpuUsage, average, msTime);

}  // namespace dai