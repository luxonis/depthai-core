#pragma once
#include <cstdint>
#include <cstdint>
#include <cstdlib>
#include <vector>
#include <array>
#include <cmath>

#include "SpatialFilterParams.h"

namespace dai
{

    class SpatialFilter
    {
    public:
        SpatialFilter();
        ~SpatialFilter();
        int Init(float alpha, int delta, int iterationNr, int holesFillingRadius);

        void process(std::shared_ptr<dai::ImgFrame>& disparityFrame);

    private:

        SpatialFilterParams params = {};
    };

}