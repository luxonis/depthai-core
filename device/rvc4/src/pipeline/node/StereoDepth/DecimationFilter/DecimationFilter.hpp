#pragma once
#include <cstdint>

#include <cstdint>
#include <cstdlib>
#include <vector>
#include <array>
#include <cmath>
#include "DecimationFilterParams.h"

namespace dai
{

    class DecimationFilter
    {
    public:
        DecimationFilter();
        ~DecimationFilter();
        int Init(int decimationFactor, DecimationMode mode);

        void process(std::shared_ptr<dai::ImgFrame>& fb, std::shared_ptr<dai::ImgFrame>& destFb);

    private:

        DecimationFilterParams params = {};
    };

}