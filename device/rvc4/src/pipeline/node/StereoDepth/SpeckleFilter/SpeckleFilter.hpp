#pragma once
#include <cstdint>

#include <cstdlib>
#include <vector>
#include <array>
#include <cmath>
#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai
{

    class SpeckleFilter
    {
    public:
        SpeckleFilter();
        ~SpeckleFilter();
        int Init();

        void process(std::shared_ptr<dai::ImgFrame>& disparityFrame, int speckleRange, int maxDiff);

    };

}