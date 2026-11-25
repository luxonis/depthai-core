#pragma once

namespace gpu_stereo {

struct Config {
    // Constructor for runtime parameters
    Config(int w = 0, int h = 0, int conf = 10)
        : width(w), height(h), confidence_threshold(conf) {}

    int width = 0;
    int height = 0;
    int max_disparity = 128;
    int num_pyramid_levels = 3;
    int block_match_radius = 3;
    int refinement_radius = 6;
    int refinement_radius_full = 3;
    int subpixel_bits = 4;
    int confidence_threshold = 10;
    // When true, the algorithm stores internal matching cost and exposes it as an 8-bit confidence map.
    bool output_confidence = false;
    // Depth unit multiplier applied when converting disparity -> depth (default: millimeters).
    double depth_unit_multiplier = 1000.0;
};

}
