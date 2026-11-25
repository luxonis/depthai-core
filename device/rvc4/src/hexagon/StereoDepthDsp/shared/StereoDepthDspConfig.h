#pragma once

#include <array>

struct StereoDepthDspConfig {

    int width, height;

    int leftRightCheckThreshold;
    int primarySearchR2L = 0;
    int confidenceThreshold;

    float fixedNumerator;
    int scaleFactor;

    int numDisparities = 64;

    int downscaleFactor = 2;

    uint16_t disp2depthLUT[65536];
};
