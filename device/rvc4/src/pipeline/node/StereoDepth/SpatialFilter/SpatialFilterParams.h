#pragma once
#include <cstdint>
#include "depthai/pipeline/datatype/ImgFrame.hpp"

struct SpatialFilterParams {
    std::shared_ptr<dai::ImgFrame> currentFrame = {};

    float alpha = 0.5f;
    uint8_t delta = 20;
    int iterationNr = 2;
    int holesFillingRadius = 0;
};
