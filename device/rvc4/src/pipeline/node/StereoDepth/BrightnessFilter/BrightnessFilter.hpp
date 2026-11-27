#pragma once
#include <cstdint>

#include <cstdlib>
#include <vector>
#include <array>
#include <cmath>
#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai
{

    class BrightnessFilter
    {
    public:
        BrightnessFilter();
        ~BrightnessFilter();
        int Init();

        void process(const std::shared_ptr<const dai::ImgFrame>& referenceFrame, std::shared_ptr<dai::ImgFrame>& disparityFrame, int minBrightness, int maxBrightness);

    };

}