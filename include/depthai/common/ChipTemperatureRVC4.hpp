#pragma once

#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * Chip temperature information.
 *
 * Multiple temperature measurement points and their average
 */
struct ChipTemperatureRVC4 {
    /**
     *  CPU Subsystem
     */
    float css;
    /**
     *  Media Subsystem
     */
    float mss;
    /**
     *  TODO: What does nce stand for?
     */
    float nce;
    /**
     *  SoC
     */
    float soc;
    /**
     *  Average of measurements
     */
    float average;
};

DEPTHAI_SERIALIZE_EXT(ChipTemperatureRVC4, css, mss, nce, soc, average);

}  // namespace dai