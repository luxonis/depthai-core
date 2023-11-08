#pragma once

#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/**
 * Chip temperature information.
 *
 * Multiple temperature measurement points and their average
 */
struct ChipTemperatureS3 {
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

DEPTHAI_SERIALIZE_EXT(ChipTemperatureS3, css, mss, nce, soc, average);

}  // namespace dai