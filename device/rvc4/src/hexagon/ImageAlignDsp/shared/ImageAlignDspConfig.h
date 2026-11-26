#pragma once

#include <array>

struct ImageAlignDspConfig {
    std::array<std::array<float, 3>, 3> depthSourceIntrinsics;
    std::array<std::array<float, 3>, 3> alignSourceIntrinsics;
    std::array<std::array<float, 4>, 4> depthToAlignExtrinsics;

    int width, height, stride, bpp;
};
