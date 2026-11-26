#pragma once
#include <cstdint>

#include <cstdlib>
#include <vector>
#include <array>
#include <cmath>
#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai2
{

    class MedianFilter
    {
    public:
        MedianFilter();
        ~MedianFilter();
        int Init();

        void process(std::shared_ptr<dai::ImgFrame>& disparityFrame, int medianSize);

    };

}