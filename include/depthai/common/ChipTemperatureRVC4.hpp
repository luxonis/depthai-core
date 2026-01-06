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
    float cpuss;
    /**
     *  GPU Subsystem
     */
    float gpuss;
    /**
     *  Modem Subsystem
     */
    float mdmss;
    /**
     *  Video
     */
    float video;
    /**
     *  DDR Memory
     */
    float ddr;
    /**
     *  Camera
     */
    float camera;
    /**
     *  Average of measurements
     */
    float average;
};

DEPTHAI_SERIALIZE_EXT(ChipTemperatureRVC4, cpuss, gpuss, mdmss, video, ddr, camera, average);

}  // namespace dai