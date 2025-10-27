#pragma once

#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * Chip temperature information.
 *
 * Multiple temperature measurement points and their average
 */
struct ChipTemperature {
    /**
     *  CPU Subsystem
     */
    float css;
    /**
     *  Media Subsystem
     */
    float mss;
    /**
     *  Shave Array
     */
    float upa;
    /**
     *  DRAM Subsystem
     */
    float dss;
    /**
     *  Average of measurements
     */
    float average;
};

DEPTHAI_SERIALIZE_EXT(ChipTemperature, css, mss, upa, dss, average);

}  // namespace dai