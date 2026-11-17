#pragma once
#include <cstdint>
#include "depthai/pipeline/datatype/ImgFrame.hpp"

enum DecimationMode {
    PIXEL_SKIPPING = 0,
    NON_ZERO_MEDIAN = 1,
    NON_ZERO_MEAN = 2,
};

struct DecimationFilterParams {
    std::shared_ptr<dai::ImgFrame> currentFrame;
    std::shared_ptr<dai::ImgFrame> destFrame;

    int decimationFactor = 2;
    enum DecimationMode decimationMode = PIXEL_SKIPPING;
};
