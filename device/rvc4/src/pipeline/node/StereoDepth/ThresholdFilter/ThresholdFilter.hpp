#pragma once
#include <cstdint>

#include <cstdlib>
#include <vector>
#include <array>
#include <cmath>
#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai
{

    class ThresholdFilter
    {
    public:
        ThresholdFilter();
        ~ThresholdFilter();
        int Init();

        void process(std::shared_ptr<dai::ImgFrame>& depthFrame, int minDepth, int maxDepth);

    };

}